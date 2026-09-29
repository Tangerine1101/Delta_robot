"""The PLC link: a worker process that owns both PLC connections, and its handle in the main
process.

The worker serialises every request through one command queue, converts the rotation angle
verbatim at the wire (modules/core/angles.py) and scales the real belt feedback to true mm. The
main process talks to it only through `PlcLink.dispatch` / `PlcLink.request_status`; the UDP
pose stream and the run log hang off the same handle.
"""
from __future__ import annotations

import itertools
import math
import multiprocessing as mp
import time
from pathlib import Path
from queue import Empty
from typing import Any

from modules.comm.omron import PLCGateway
from modules.comm.siemens import SiemensGateway
from modules.core.angles import robot_rad_to_wire_deg, wire_deg_to_robot_rad
from modules.settings import Settings

# A pose stream that has delivered nothing this long after start is reported once.
NO_STREAM_WARN_S = 2.0

# How often (seconds) the worker probes the PLC connection when idle, to prevent
# EtherNet/IP and snap7 sessions from being dropped by firmware keep-alive timers.
_KEEPALIVE_S = 25.0

# Status fields filled from the Siemens PLC; everything else in a status dict is Omron's.
_SIEMENS_STATUS_KEYS = frozenset({
    "rotate_current", "speed_current", "siemens_task_doing", "siemens_task_state",
    "conveyor_position", "conveyor_position_raw", "speed_current_raw",
})


def _worker(
    command_queue: mp.Queue,
    response_queue: mp.Queue,
    settings: Settings,
    ip: str,
    port: int,
    log_dir: str | None = None,
) -> None:
    import signal

    # Ctrl-C reaches the whole process group; the parent shuts this worker down itself
    # (a "shutdown" message) after stopping the belt, so the worker must not die first.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    if log_dir is not None:
        from modules.runlog import tee_console

        tee_console(Path(log_dir) / "worker.log")
    if ip in ("127.0.0.1", "localhost"):
        siemens_ip = ip
        siemens_port = port
        # The simulator reports the belt in true mm; the scale corrects the real encoder only.
        belt_scale = 1.0
    else:
        siemens_ip = settings.plc.siemens.ip
        siemens_port = settings.plc.siemens.port
        belt_scale = settings.conveyor.position_scale_mm

    gateway = PLCGateway(ip, port, settings.plc.interpolar_points, settings.robot.limits)
    siemens_gateway = SiemensGateway(siemens_ip, siemens_port)

    try:
        gateway.connect()
        siemens_gateway.connect()
    except Exception as exc:
        response_queue.put({"ok": False, "type": "connect_failed", "req_id": None, "error": str(exc)})
        return

    response_queue.put({"ok": True, "type": "connected", "req_id": None, "ip": ip, "port": port})

    try:
        while True:
            try:
                message = command_queue.get(timeout=_KEEPALIVE_S)
            except Empty:
                # Idle keepalive: probe both connections to prevent firmware session timeouts.
                try:
                    gateway._probe_connection()
                except Exception:
                    pass
                try:
                    siemens_gateway.get_status()
                except Exception:
                    pass
                continue

            req_id = message.get("req_id")
            message_type = message.get("type")

            if message_type == "shutdown":
                response_queue.put({"ok": True, "type": "shutdown", "req_id": req_id})
                break

            if message_type == "status":
                try:
                    # omron=False: the robot pose comes from the UDP stream, so only the
                    # Siemens DB is read; the caller merges its cached Omron fields.
                    status = gateway.get_package() if message.get("omron", True) else {}
                    if status is not None:
                        try:
                            s_status = siemens_gateway.get_status()
                            if s_status is not None:
                                rotate_wire = s_status.get("rotate_current")
                                status.update({
                                    # Wire degrees [-359,359] feedback -> R-frame
                                    # DEGREES, verbatim (identity zero, no wrap so
                                    # the true PLC angle shows). Human-readable for
                                    # logs/dashboard; radians live only inside the
                                    # scheduler algorithm.
                                    "rotate_current": (
                                        math.degrees(wire_deg_to_robot_rad(rotate_wire))
                                        if rotate_wire is not None else None
                                    ),
                                    # Belt feedback in true mm and mm/s from here on;
                                    # the PLC's raw values are kept for the run log.
                                    "speed_current": _scaled(s_status.get("speed_current"), belt_scale),
                                    "speed_current_raw": s_status.get("speed_current"),
                                    "siemens_task_doing": s_status.get("task_doing"),
                                    "siemens_task_state": s_status.get("task_state"),
                                    "conveyor_position": _scaled(s_status.get("conveyor_position"), belt_scale),
                                    "conveyor_position_raw": s_status.get("conveyor_position"),
                                })
                        except Exception as s_exc:
                            print(f"[WARN] Failed to query Siemens status: {s_exc}")
                    response_queue.put({"ok": True, "type": "status", "req_id": req_id, "data": status})
                except Exception as exc:
                    response_queue.put({"ok": False, "type": "error", "req_id": req_id, "error": str(exc)})
                continue

            if message_type == "send":
                try:
                    pkg = message["package"]
                    cmd_id = pkg.get("commandID")
                    if cmd_id in (7, 8, 9):
                        # Siemens command. rotate_absolute (7) carries an R-frame
                        # angle in RADIANS: convert VERBATIM to wire degrees
                        # [-359,359] (identity zero, no wrap) on the wire only
                        # (echo the original radian pkg back to the caller).
                        if cmd_id == 7:
                            wire_pkg = dict(pkg)
                            wire_pkg["rotate"] = robot_rad_to_wire_deg(
                                pkg.get("rotate", 0.0)
                            )
                        else:
                            wire_pkg = pkg
                        s_status = siemens_gateway.send_package(wire_pkg)
                        response_queue.put(
                            {
                                "ok": True,
                                "type": "sent",
                                "req_id": req_id,
                                "commandID": cmd_id,
                                "package": pkg,
                                "status": s_status,
                            }
                        )
                    else:
                        # Omron command
                        package = gateway.send_package(pkg)
                        status = gateway.get_package()
                        response_queue.put(
                            {
                                "ok": True,
                                "type": "sent",
                                "req_id": req_id,
                                "commandID": package.get("commandID"),
                                "package": package,
                                "status": status,
                            }
                        )
                except Exception as exc:
                    response_queue.put({"ok": False, "type": "error", "req_id": req_id, "error": str(exc)})
                continue

            response_queue.put(
                {
                    "ok": False,
                    "type": "error",
                    "req_id": req_id,
                    "error": f"Unknown message type: {message_type}",
                }
            )
    finally:
        gateway.disconnect()
        siemens_gateway.disconnect()


def _scaled(value: Any, scale: float) -> float | None:
    return None if value is None else float(value) * scale


def _wait_for_response(
    response_queue: mp.Queue,
    expected_id: int | None,
    timeout: float = 5.0,
) -> dict[str, Any] | None:
    """Drain queue until we get the response with req_id == expected_id.

    Responses with a different (older) req_id are discarded with a warning.
    Returns None on timeout.
    """
    import time
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        try:
            resp = response_queue.get(timeout=remaining)
        except Empty:
            return None
        if resp.get("req_id") == expected_id:
            return resp
        # Stale response from a previous timed-out request.
        print(f"[WARN] discarding stale IPC response (req_id={resp.get('req_id')}, expected={expected_id})")


def _start_worker(
    ctx: Any,
    command_queue: mp.Queue,
    response_queue: mp.Queue,
    settings: Settings,
    ip: str,
    port: int,
    log_dir: str | None,
) -> "mp.Process | None":
    """Start the PLC worker and wait for connection confirmation.

    Returns the Process on success, None if connection failed.
    """
    worker = ctx.Process(
        target=_worker,
        args=(command_queue, response_queue, settings, ip, port, log_dir),
        daemon=True,
    )
    worker.start()
    startup = _wait_for_response(response_queue, expected_id=None, timeout=10.0)
    if startup is None:
        print("[ERROR] PLC worker did not report readiness in time — aborting.")
        worker.terminate()
        worker.join(timeout=2.0)
        return None
    if not startup.get("ok"):
        print(f"[ERROR] Worker failed to connect: {startup.get('error')}")
        worker.join(timeout=2.0)
        if worker.is_alive():
            worker.terminate()
            worker.join(timeout=2.0)
        return None
    print(f"[INFO] Worker connected to {startup.get('ip')}:{startup.get('port')}")
    return worker


def _stop_worker(worker: "mp.Process", command_queue: mp.Queue, response_queue: mp.Queue, req_counter: Any) -> None:
    req_id = next(req_counter)
    command_queue.put({"type": "shutdown", "req_id": req_id})
    _wait_for_response(response_queue, expected_id=req_id, timeout=5.0)
    worker.join(timeout=5.0)
    if worker.is_alive():
        worker.terminate()
        worker.join(timeout=5.0)


class PlcLink:
    """The main process's handle on the PLC worker: `dispatch`, `request_status`, the UDP
    pose stream and the run log.

    `request_status` takes the robot pose from the UDP stream while it is fresh; the Omron
    status block (task_state, bit_doing, end_effector) is then re-read only every
    `omron_status_period_s` and merged from cache, and each poll reads just the Siemens DB.
    When the stream is missing or stale every poll reads both PLCs, as before.
    """

    def __init__(
        self,
        settings: Settings,
        ip: str,
        port: int,
        *,
        runlog: Any = None,
        raise_errors: bool = True,
    ) -> None:
        ctx = mp.get_context("spawn")
        self.settings = settings
        self.ip = ip
        self.raise_errors = raise_errors
        self.runlog = runlog
        self.command_queue: mp.Queue = ctx.Queue()
        self.response_queue: mp.Queue = ctx.Queue()
        self.req_counter = itertools.count(1)
        log_dir = str(runlog.dir) if runlog is not None else None
        self.worker = _start_worker(ctx, self.command_queue, self.response_queue, settings,
                                    ip, port, log_dir)
        self.pose_stream = None
        self.omron_status_period_s = 0.0
        self._omron_cache: dict[str, Any] | None = None
        self._omron_read_at = 0.0
        self._pose_source: str | None = None
        self._stream_started_t = 0.0
        self._no_stream_warned = False
        if self.worker is not None:
            self._start_pose_stream()

    @property
    def ok(self) -> bool:
        return self.worker is not None

    def _start_pose_stream(self) -> None:
        cfg = self.settings.pose_stream
        if not cfg.enabled:
            return
        from modules.comm.pose_stream import PoseStream

        runlog = self.runlog
        stream = PoseStream(
            cfg.port,
            source_ip=self.ip,
            stale_s=cfg.stale_s,
            on_sample=(lambda s: runlog.pose(s.t_mono, s.t_wall, s.position)) if runlog else None,
        )
        if stream.start():
            self.pose_stream = stream
            self.omron_status_period_s = cfg.omron_status_period_s
            self._stream_started_t = time.monotonic()

    def _request(self, message: dict[str, Any]) -> dict[str, Any] | None:
        req_id = next(self.req_counter)
        message["req_id"] = req_id
        self.command_queue.put(message)
        return _wait_for_response(self.response_queue, expected_id=req_id, timeout=10.0)

    def _fail(self, text: str) -> None:
        if self.raise_errors:
            raise RuntimeError(text)
        print(f"[ERROR] {text}")

    def dispatch(self, package: dict[str, Any]) -> dict[str, Any] | None:
        t_send = time.monotonic()
        response = self._request({"type": "send", "package": package})
        t_done = time.monotonic()
        error = None
        if response is None:
            error = "no response from PLC worker"
        elif not response.get("ok", False):
            error = str(response.get("error"))
        if self.runlog is not None:
            self.runlog.command(package, t_send, t_done, ok=error is None, error=error,
                                reply=response.get("status") if response else None)
        if error is not None:
            if self.raise_errors and response is None:
                raise TimeoutError(error)
            self._fail(error)
            return None
        return response.get("status")

    def request_status(self) -> dict[str, Any] | None:
        t_send = time.monotonic()
        pose = self.pose_stream.fresh(t_send) if self.pose_stream is not None else None
        read_omron = (pose is None or self._omron_cache is None
                      or t_send - self._omron_read_at >= self.omron_status_period_s)
        response = self._request({"type": "status", "omron": read_omron})
        t_done = time.monotonic()
        if response is None or not response.get("ok", False):
            error = "no response from PLC worker" if response is None else str(response.get("error"))
            if self.runlog is not None:
                self.runlog.status(t_send, t_done, None, omron_read=read_omron)
            if self.raise_errors and response is None:
                raise TimeoutError(error + " while polling status")
            self._fail(error)
            return None

        data = response.get("data")
        if data is not None:
            if read_omron:
                self._omron_cache = {k: v for k, v in data.items() if k not in _SIEMENS_STATUS_KEYS}
                self._omron_read_at = t_done
            else:
                merged = dict(self._omron_cache or {})
                merged.update(data)
                data = merged
            pose = self.pose_stream.fresh(t_done) if self.pose_stream is not None else None
            if pose is not None:
                data["pos_EE"] = list(pose.position)
                data["pos_EE_t"] = pose.t_mono
                data["pose_age_s"] = round(t_done - pose.t_mono, 4)
                data["pose_source"] = "udp"
            else:
                data["pose_source"] = "omron"
            self._note_pose_source(data["pose_source"])
        if self.runlog is not None:
            self.runlog.status(t_send, t_done, data, omron_read=read_omron)
        return data

    def _note_pose_source(self, source: str) -> None:
        if self.pose_stream is None:
            return
        if (not self._no_stream_warned and self.pose_stream.latest() is None
                and time.monotonic() - self._stream_started_t > NO_STREAM_WARN_S):
            self._no_stream_warned = True
            print(f"[WARN] no UDP pose datagram in {NO_STREAM_WARN_S:.0f} s: is this PC the "
                  "destination Program_UDP sends to (open-issues L13)?")
        if source == self._pose_source:
            return
        if source == "udp":
            print("[INFO] robot pose from the UDP stream")
        elif self._pose_source is None and self.pose_stream.latest() is None:
            # The first status read can come before the first datagram: not a fault.
            print("[INFO] robot pose from EtherNet/IP polling until the UDP stream starts")
        else:
            print(f"[WARN] UDP pose stream silent for > {self.pose_stream.stale_s:.2f} s; "
                  "robot pose from EtherNet/IP polling")
        self._pose_source = source

    def close(self) -> None:
        if self.pose_stream is not None:
            self.pose_stream.stop()
            print(f"[INFO] pose stream: {self.pose_stream.received} samples, "
                  f"{self.pose_stream.rejected} rejected")
        if self.worker is not None:
            _stop_worker(self.worker, self.command_queue, self.response_queue, self.req_counter)
