"""The PLC simulator node: Omron (Matching_Code_10) + Siemens + parts, in real time.

``PLCSim`` scans the Omron port every 4 ms in a thread and serves the JSON-lines
protocol that ``comm.omron.MockPLC`` and the mock ``SiemensGateway`` already speak
when the PLC address is ``127.0.0.1`` — so ``main.py`` and the scheduler run
unmodified against it. ``SimCamera`` feeds the vision interface of the production
loop from the simulated belt. See ``doc/plc/simulator.md``.
"""
from __future__ import annotations

import bisect
import json
import socket
import socketserver
import struct
import threading
import time
from collections import deque
from typing import Any

from modules.core.frames import ConveyorFrame
from modules.core.kinematics import calc_inverse_kinematics
from modules.settings import Settings, load_settings
from modules.plc_sim.omron_core import SCAN_S, OmronPLC, Patches, Quirks
from modules.plc_sim.siemens_core import SiemensPLC
from modules.plc_sim.world import Feeder, World

# Program_UDP on the real Omron: 3 x REAL little-endian (X, Y, Z mm), measured every 12 ms.
POSE_PACKET = struct.Struct("<3f")
POSE_PERIOD_S = 0.012


class PLCSim:
    def __init__(
        self,
        *,
        settings: Settings | None = None,
        quirks: Quirks | None = None,
        patches: Patches | None = None,
        servo_tau_s: float = 0.0,
        tag_latency_s: float = 0.0,
        feed_interval_s: float | None = None,
        feed_mode: str = "periodic",
        seed: int = 1,
    ) -> None:
        self.settings = settings or load_settings()
        home = self.settings.robot.home_position
        _, start_angles, _ = calc_inverse_kinematics(*home)
        self.omron = OmronPLC(quirks, patches=patches, start_angles_deg=start_angles,
                              servo_tau_s=servo_tau_s)
        self.siemens = SiemensPLC(belt_accel_mm_s2=self.settings.conveyor.accel_mm_s2)
        self.frame = ConveyorFrame.from_settings(self.settings.conveyor)
        self.world = World.from_settings(self.settings, self.frame)
        self.feeder = None
        if feed_interval_s:
            self.feeder = Feeder(
                interval_s=float(feed_interval_s),
                types=list(self.settings.plc_sim.feed_types),
                lanes=list(self.settings.plc_sim.feed_lanes),
                mode=feed_mode, seed=seed,
            )
        self.tag_latency_s = max(float(tag_latency_s), 0.0)
        self.lock = threading.RLock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._server: socketserver.ThreadingTCPServer | None = None
        self._server_thread: threading.Thread | None = None
        self._hist_t: deque[float] = deque(maxlen=5000)
        self._hist_p: deque[float] = deque(maxlen=5000)
        self.overruns = 0
        self.address: tuple[str, int] = ("127.0.0.1", 0)
        self._pose_thread: threading.Thread | None = None
        self.pose_packets_sent = 0

    # ---- lifecycle ------------------------------------------------------------------
    def start(self) -> None:
        self._thread = threading.Thread(target=self._loop, name="plc-sim", daemon=True)
        self._thread.start()

    def serve(self, host: str = "127.0.0.1", port: int = 0) -> tuple[str, int]:
        server = _Server((host, port), _Handler)
        server.sim = self  # type: ignore[attr-defined]
        self._server = server
        self._server_thread = threading.Thread(
            target=server.serve_forever, kwargs={"poll_interval": 0.1}, name="plc-sim-server", daemon=True)
        self._server_thread.start()
        self.address = server.server_address[:2]
        return self.address

    def start_pose_stream(self, port: int, host: str = "127.0.0.1",
                          period_s: float = POSE_PERIOD_S) -> None:
        """Push the end-effector position over UDP like Program_UDP on the real Omron."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        def loop() -> None:
            next_t = time.monotonic()
            while not self._stop.is_set():
                with self.lock:
                    x, y, z = self.omron.pos_ee
                try:
                    sock.sendto(POSE_PACKET.pack(x, y, z), (host, port))
                    self.pose_packets_sent += 1
                except OSError:
                    pass
                next_t += period_s
                time.sleep(max(0.0, next_t - time.monotonic()))
            sock.close()

        self._pose_thread = threading.Thread(target=loop, name="plc-sim-pose-udp", daemon=True)
        self._pose_thread.start()

    def shutdown(self) -> None:
        self._stop.set()
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _loop(self) -> None:
        next_t = time.monotonic()
        while not self._stop.is_set():
            with self.lock:
                self.step(time.monotonic())
            next_t += SCAN_S
            delay = next_t - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
            elif delay < -0.1:
                self.overruns += 1
                next_t = time.monotonic()

    def step(self, now: float) -> None:
        """One 4 ms scan of both PLCs and the plant (caller holds ``lock``)."""
        self.omron.scan()
        self.siemens.step(SCAN_S)
        self.world.update(now, self.siemens.position_mm, self.omron._fk_xyz, self.omron.pump_out)
        if self.feeder is not None and self.siemens.speed_current > 1.0:
            self.feeder.tick(now, self.world, self.siemens.position_mm)
        self._hist_t.append(now)
        self._hist_p.append(self.siemens.position_mm)

    # ---- queries ----------------------------------------------------------------------
    def belt_position_at(self, t: float) -> float:
        with self.lock:
            times, pos = list(self._hist_t), list(self._hist_p)
        if not times:
            return self.siemens.position_mm
        i = bisect.bisect_left(times, t)
        if i <= 0:
            return pos[0]
        if i >= len(times):
            return pos[-1]
        t0, t1 = times[i - 1], times[i]
        return pos[i - 1] + (pos[i] - pos[i - 1]) * ((t - t0) / (t1 - t0) if t1 > t0 else 0.0)

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            return {
                "pos_EE": [round(c, 2) for c in self.omron.pos_ee],
                "pump": self.omron.pump_out,
                "chain_states": self.omron.chain_states,
                "task_state": self.omron.plc_package["task_state"],
                "belt_speed_mm_s": round(self.siemens.speed_current, 1),
                "belt_position_mm": round(self.siemens.position_mm, 1),
                "rotate_deg": round(self.siemens.rotate_current, 1),
                "world": self.world.counts(),
                "plc_events": dict(self.omron.event_counts),
                "overruns": self.overruns,
            }

    def summary(self) -> str:
        snap = self.snapshot()
        contacts = [c for c in self.world.contacts if c.error_uv is not None]
        along = sorted(c.error_uv[0] for c in contacts)
        med = along[len(along) // 2] if along else None
        return json.dumps({"world": snap["world"], "plc_events": snap["plc_events"],
                           "along_belt_error_median_mm": None if med is None else round(med, 2),
                           "overruns": snap["overruns"]}, ensure_ascii=True)

    def camera(self, **kwargs: Any) -> "SimCamera":
        return SimCamera(self, **kwargs)

    # ---- protocol ---------------------------------------------------------------------
    def handle(self, message: dict[str, Any]) -> dict[str, Any]:
        action = message.get("action")
        if action == "write":
            if self.tag_latency_s:
                time.sleep(self.tag_latency_s)
            tag = str(message.get("tag", ""))
            with self.lock:
                ok = self.omron.write_tag(tag, message.get("value"))
            return {"ok": True} if ok else {"ok": False, "error": f"tag not writable: {tag}"}
        if action == "read":
            if self.tag_latency_s:
                time.sleep(self.tag_latency_s)
            with self.lock:
                values = {t: self.omron.read_tag(t)[1] for t in message.get("tags", [])}
            return {"ok": True, "values": values}
        if action == "siemens_status":
            with self.lock:
                return self.siemens.status()
        if "CommandID" in message or "commandID" in message:
            command = int(message.get("CommandID", message.get("commandID", 0)))
            with self.lock:
                return self.siemens.accept(command, float(message.get("rotate", 0.0)),
                                           float(message.get("speed", 0.0)))
        return {"ok": False, "error": "unknown message"}


class _Handler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        sim: PLCSim = self.server.sim  # type: ignore[attr-defined]
        for raw in self.rfile:
            line = raw.decode("utf-8").strip()
            if not line:
                continue
            try:
                response = sim.handle(json.loads(line))
            except Exception as exc:  # malformed request: report, keep the link
                response = {"ok": False, "error": str(exc)}
            self.wfile.write((json.dumps(response, ensure_ascii=True) + "\n").encode("utf-8"))
            self.wfile.flush()


class _Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


class SimCamera:
    """Vision stand-in for the production loop: ``poll(now)`` returns detections of
    the boards inside ``camera_window_uv`` as they were ``latency_s`` ago, stamped
    with that capture time (so the loop's backdated anchoring is exercised)."""

    def __init__(self, sim: PLCSim, *, latency_s: float = 0.09, period_s: float = 1.0 / 30.0,
                 noise_mm: float = 0.0) -> None:
        from modules.core.tracking import ObjectDetection

        self._detection = ObjectDetection
        self.sim = sim
        self.latency_s = latency_s
        self.period_s = period_s
        self.noise_mm = noise_mm
        self.camera_window_uv = sim.settings.conveyor.camera_window_uv
        self._next = 0.0
        self._rng = __import__("random").Random(7)

    def poll(self, now: float) -> list[Any]:
        if now < self._next:
            return []
        self._next = now + self.period_s
        capture_t = now - self.latency_s
        belt = self.sim.belt_position_at(capture_t)
        u_min, u_max, v_min, v_max = self.camera_window_uv
        with self.sim.lock:
            boards = [b for b in self.sim.world.boards if b.state == "belt"]
        out = []
        for b in boards:
            u = b.u_at(belt)
            if not (u_min <= u <= u_max and v_min <= b.v <= v_max):
                continue
            n = self.noise_mm
            out.append(self._detection(
                object_id=b.board_id,
                x=u + (self._rng.gauss(0.0, n) if n else 0.0),
                y=b.v + (self._rng.gauss(0.0, n) if n else 0.0),
                object_type=b.board_type,
                timestamp=capture_t,
                confidence=0.95,
                angle_deg=b.heading_deg,
            ))
        return out

    def stop(self) -> None:
        pass

    def close_window(self) -> None:
        pass

    def jpeg_frame(self) -> bytes | None:
        """Top view of the belt for the web dashboard's MJPEG slot."""
        try:
            import cv2
            import numpy as np
        except ImportError:
            return None
        sim = self.sim
        with sim.lock:
            belt = sim.siemens.position_mm
            boards = [(b.u_at(belt), b.v, b.board_type, b.state, b.xy) for b in sim.world.boards
                      if b.state in ("belt", "held")]
            tcp = sim.omron.pos_ee
            pump = sim.omron.pump_out
            speed = sim.siemens.speed_current
            counts = sim.world.counts()
        width, height, u0, u1, v0, v1 = 800, 330, -20.0, 420.0, -150.0, 170.0
        img = np.full((height, width, 3), 24, np.uint8)

        def px(u: float, v: float) -> tuple[int, int]:
            return (int((u - u0) / (u1 - u0) * width), int((v1 - v) / (v1 - v0) * height))

        cw = self.camera_window_uv
        ws = sim.world.workspace_window_uv
        cv2.rectangle(img, px(u0, cw[3] + 5), px(u1, cw[2] - 5), (55, 55, 60), -1)
        cv2.rectangle(img, px(cw[0], cw[3]), px(cw[1], cw[2]), (120, 90, 40), 1)
        cv2.rectangle(img, px(ws[0], ws[3]), px(ws[1], ws[2]), (60, 140, 60), 1)
        for board_type, (bx, by) in sim.world.bins.items():
            bu, bv = sim.frame.to_conveyor(bx, by)
            cv2.rectangle(img, px(bu - 12, bv + 12), px(bu + 12, bv - 12), (160, 160, 160), 1)
            cv2.putText(img, board_type, px(bu - 12, bv - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                        (160, 160, 160), 1)
        for u, v, board_type, state, xy in boards:
            if state == "held" and xy is not None:
                u, v = sim.frame.to_conveyor(*xy)
            color = (80, 180, 255) if board_type == "TQFP" else (255, 160, 80)
            cv2.rectangle(img, px(u - 10, v + 10), px(u + 10, v - 10), color, -1)
        cu, cv_ = sim.frame.to_conveyor(tcp[0], tcp[1])
        cv2.circle(img, px(cu, cv_), 9, (60, 60, 230) if pump else (230, 230, 230), 2)
        text = (f"belt {speed:5.1f} mm/s  z {tcp[2]:6.1f}  placed {counts['placed']}  "
                f"lost {counts['lost']}  dropped {counts['dropped']}  [SIMULATOR]")
        cv2.putText(img, text, (8, height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)
        ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
        return buf.tobytes() if ok else None
