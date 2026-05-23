from __future__ import annotations

import argparse
import json
import socket
import socketserver
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any
from datetime import datetime

from modules.EthernetCom import ARRAY_FIELDS, COMMAND_ID, COMMAND_NAME, load_config


Position3D = tuple[float, float, float]


@dataclass
class MotionTarget:
    position: Position3D
    duration_s: float
    end_effector: int


@dataclass
class FakePLCState:
    interpolar_points: int
    log_path: Path
    sample_period_s: float
    home_position: Position3D = (0.0, 0.0, -300.0)
    started_at: float = field(default_factory=time.monotonic)
    position: Position3D = field(init=False)
    end_effector: int = 0
    task_doing: int = COMMAND_ID["stop"]
    task_state: int = 0
    last_pc_package: dict[str, Any] | None = None
    motion_queue: list[MotionTarget] = field(default_factory=list)
    lock: threading.RLock = field(default_factory=threading.RLock)

    def __post_init__(self) -> None:
        self.position = self.home_position
        self.accumulated_pc_package = {
            "commandID": COMMAND_ID["stop"],
            "argument_number": 0,
            "argument_x": [0.0] * self.interpolar_points,
            "argument_y": [0.0] * self.interpolar_points,
            "argument_z": [0.0] * self.interpolar_points,
            "argument_e": [0] * self.interpolar_points,
            "argument_time": [0.0] * self.interpolar_points,
            "bit_doing": 0,
        }

    def handle_tag_write(self, tag: str, value: Any) -> bool:
        parts = tag.split(".", 1)
        if len(parts) < 2:
            return False
        field_part = parts[1]

        with self.lock:
            if "[" in field_part and field_part.endswith("]"):
                name, index_str = field_part[:-1].split("[", 1)
                try:
                    index = int(index_str)
                    if name in self.accumulated_pc_package and 0 <= index < self.interpolar_points:
                        self.accumulated_pc_package[name][index] = value
                except ValueError:
                    return False
            else:
                if field_part in self.accumulated_pc_package:
                    self.accumulated_pc_package[field_part] = value

            if field_part in ("bit_doing", "doing_bit") and int(value) == 1:
                self.accumulated_pc_package[field_part] = 0
                self.accept_pc_package(self.accumulated_pc_package)
        return True

    def handle_tag_read(self, tags: list[str]) -> dict[str, Any]:
        plc_pkg = self.build_plc_package()
        values = {}
        for tag in tags:
            parts = tag.split(".", 1)
            if len(parts) < 2:
                values[tag] = None
                continue
            field_part = parts[1]

            with self.lock:
                if "[" in field_part and field_part.endswith("]"):
                    name, index_str = field_part[:-1].split("[", 1)
                    try:
                        index = int(index_str)
                        if name in plc_pkg and 0 <= index < len(plc_pkg[name]):
                            values[tag] = plc_pkg[name][index]
                        else:
                            values[tag] = None
                    except ValueError:
                        values[tag] = None
                else:
                    if field_part in plc_pkg:
                        values[tag] = plc_pkg[field_part]
                    else:
                        values[tag] = None
        return values

    def normalize_pc_package(self, package: dict[str, Any]) -> dict[str, Any]:
        normalized = {
            "commandID": int(package.get("commandID", COMMAND_ID["stop"])),
            "argument_number": int(package.get("argument_number", 0)),
        }
        for field_name in ARRAY_FIELDS:
            values = list(package.get(field_name, []))[: self.interpolar_points]
            if len(values) < self.interpolar_points:
                fill_value = 0 if field_name == "argument_e" else 0.0
                values.extend([fill_value] * (self.interpolar_points - len(values)))
            if field_name == "argument_e":
                normalized[field_name] = [1 if bool(value) else 0 for value in values]
            else:
                normalized[field_name] = [float(value) for value in values]
        normalized["argument_number"] = max(
            0,
            min(normalized["argument_number"], self.interpolar_points),
        )
        normalized["bit_doing"] = 1 if bool(package.get("bit_doing", package.get("doing_bit", 0))) else 0
        return normalized

    def accept_pc_package(self, package: dict[str, Any]) -> dict[str, Any]:
        normalized = self.normalize_pc_package(package)
        command_id = normalized["commandID"]
        command_name = COMMAND_NAME.get(command_id, f"unknown_{command_id}")

        with self.lock:
            self.last_pc_package = normalized
            self.task_doing = command_id
            self.task_state = 1

            if command_id == COMMAND_ID["stop"]:
                self.motion_queue.clear()
                self.task_state = 0
            elif command_id == COMMAND_ID["goto_absolute"]:
                self.motion_queue = [
                    MotionTarget(
                        (
                            normalized["argument_x"][0],
                            normalized["argument_y"][0],
                            normalized["argument_z"][0],
                        ),
                        1.0,
                        self.end_effector,
                    )
                ]
            elif command_id == COMMAND_ID["go_trajectory"]:
                self.motion_queue = self._trajectory_targets(normalized)
            elif command_id == COMMAND_ID["pick"]:
                self.end_effector = 1
                self.task_state = 0
            elif command_id == COMMAND_ID["release"]:
                self.end_effector = 0
                self.task_state = 0
            elif command_id == COMMAND_ID["calibrate"]:
                self.position = self.home_position
                self.end_effector = 0
                self.motion_queue.clear()
                self.task_state = 0

            response = {
                "ok": True,
                "accepted_command": command_name,
                "plc_package": self.build_plc_package(),
            }

        self.log_event("accept", {"pc_package": normalized, **response})
        return response

    def _trajectory_targets(self, package: dict[str, Any]) -> list[MotionTarget]:
        targets: list[MotionTarget] = []
        for index in range(package["argument_number"]):
            targets.append(
                MotionTarget(
                    (
                        package["argument_x"][index],
                        package["argument_y"][index],
                        package["argument_z"][index],
                    ),
                    1.0,
                    package["argument_e"][index],
                )
            )
        return targets

    def build_plc_package(self) -> dict[str, Any]:
        x, y, z = self.position
        return {
            "pos_angular": [round(x * 0.01, 4), round(y * 0.01, 4), round(z * 0.01, 4)],
            "pos_EE": [round(x, 4), round(y, 4), round(z, 4)],
            "task_doing": self.task_doing,
            "task_state": self.task_state,
            "end_effector": self.end_effector,
        }

    def get_status_str(self) -> str:
        pkg = self.build_plc_package()
        parts = []
        for key, value in pkg.items():
            if isinstance(value, list):
                rendered = "[" + ", ".join(str(item) for item in value) + "]"
            else:
                rendered = str(value)
            parts.append(f"{key}={rendered}")
        return "[INFO] PLC status: " + ", ".join(parts)

    def log_event(self, event: str, payload: dict[str, Any] | None = None) -> None:
        now = datetime.now()
        timestamp = now.strftime("%H:%M:%S.%f")[:-3]
        status_str = self.get_status_str()
        line = f"[{timestamp}] {status_str}"
        print(line, flush=True)
        with self.log_path.open("a", encoding="utf-8") as handle:
            handle.write(line + "\n")

    def run_motion_loop(self, stop_event: threading.Event) -> None:
        while not stop_event.is_set():
            target = self._pop_target()
            if target is None:
                time.sleep(self.sample_period_s)
                continue
            self._move_linearly(target, stop_event)

    def _pop_target(self) -> MotionTarget | None:
        with self.lock:
            if not self.motion_queue:
                if self.task_state != 0:
                    self.task_state = 0
                    self.log_event("idle")
                return None
            self.task_state = 1
            return self.motion_queue.pop(0)

    def _move_linearly(self, target: MotionTarget, stop_event: threading.Event) -> None:
        with self.lock:
            start = self.position
            self.task_state = 1

        steps = max(1, int(target.duration_s / self.sample_period_s))
        for step in range(1, steps + 1):
            if stop_event.is_set():
                return
            fraction = step / steps
            with self.lock:
                self.position = (
                    start[0] + (target.position[0] - start[0]) * fraction,
                    start[1] + (target.position[1] - start[1]) * fraction,
                    start[2] + (target.position[2] - start[2]) * fraction,
                )
                self.end_effector = target.end_effector
            self.log_event("motion_sample")
            time.sleep(self.sample_period_s)


class FakePLCRequestHandler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        state: FakePLCState = self.server.state  # type: ignore[attr-defined]
        state.log_event("client_connected", {"client": self.client_address[0]})

        for raw_line in self.rfile:
            line = raw_line.decode("utf-8").strip()
            if not line:
                continue
            try:
                message = json.loads(line)
                action = message.get("action")
                if action == "write":
                    tag = message.get("tag", "")
                    value = message.get("value")
                    ok = state.handle_tag_write(tag, value)
                    response = {"ok": ok}
                elif action == "read":
                    tags = message.get("tags", [])
                    values = state.handle_tag_read(tags)
                    response = {"ok": True, "values": values}
                else:
                    response = state.accept_pc_package(message)
            except Exception as exc:
                response = {"ok": False, "error": str(exc)}
                state.log_event("error", response)

            self.wfile.write((json.dumps(response, ensure_ascii=True) + "\n").encode("utf-8"))
            self.wfile.flush()

        state.log_event("client_disconnected", {"client": self.client_address[0]})


class ThreadedFakePLCServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True

    def __init__(self, server_address: tuple[str, int], state: FakePLCState) -> None:
        super().__init__(server_address, FakePLCRequestHandler)
        self.state = state


def _demo_package(interpolar_points: int) -> dict[str, Any]:
    x_values = [0.0, 40.0, 80.0, 120.0]
    y_values = [0.0, -20.0, -40.0, -60.0]
    z_values = [-180.0, -190.0, -210.0, -230.0]
    e_values = [0, 0, 1, 1]
    t_values = [0.15, 0.15, 0.15, 0.15]
    return {
        "commandID": COMMAND_ID["go_trajectory"],
        "argument_number": min(4, interpolar_points),
        "argument_x": x_values[:interpolar_points],
        "argument_y": y_values[:interpolar_points],
        "argument_z": z_values[:interpolar_points],
        "argument_e": e_values[:interpolar_points],
        "argument_time": t_values[:interpolar_points],
        "bit_doing": 1,
    }


def _send_self_test(host: str, port: int, interpolar_points: int) -> None:
    time.sleep(0.2)
    with socket.create_connection((host, port), timeout=5.0) as sock:
        package = _demo_package(interpolar_points)
        sock.sendall((json.dumps(package, ensure_ascii=True) + "\n").encode("utf-8"))
        response = sock.recv(4096).decode("utf-8").strip()
        print(f"[SELF_TEST] {response}", flush=True)


def run_fake_plc(
    *,
    host: str,
    port: int,
    interpolar_points: int,
    log_path: Path,
    sample_period_s: float,
    self_test: bool,
    duration_s: float | None,
    home_position: Position3D,
) -> None:
    state = FakePLCState(
        interpolar_points=interpolar_points,
        log_path=log_path,
        sample_period_s=sample_period_s,
        home_position=home_position,
    )
    stop_event = threading.Event()
    motion_thread = threading.Thread(
        target=state.run_motion_loop,
        args=(stop_event,),
        daemon=True,
    )
    motion_thread.start()

    with ThreadedFakePLCServer((host, port), state) as server:
        state.log_event(
            "server_started",
            {
                "host": host,
                "port": port,
                "interpolar_points": interpolar_points,
                "protocol": "json-lines",
            },
        )
        if self_test:
            threading.Thread(
                target=_send_self_test,
                args=(host, port, interpolar_points),
                daemon=True,
            ).start()
        if duration_s is not None:
            threading.Thread(
                target=_shutdown_later,
                args=(server, duration_s),
                daemon=True,
            ).start()

        try:
            server.serve_forever(poll_interval=0.1)
        except KeyboardInterrupt:
            print("\n[INFO] fake PLC interrupted", flush=True)
        finally:
            stop_event.set()
            state.log_event("server_stopped")


def _shutdown_later(server: ThreadedFakePLCServer, duration_s: float) -> None:
    time.sleep(max(duration_s, 0.0))
    server.shutdown()


def main() -> None:
    config = load_config()
    parser = argparse.ArgumentParser(description="Standalone fake PLC test module")
    parser.add_argument("--host", default="127.0.0.1", help="Host/IP to bind")
    parser.add_argument(
        "--port",
        type=int,
        default=int(getattr(config, "port", 502)),
        help="TCP port to bind. Defaults to modules/config.json port.",
    )
    parser.add_argument(
        "--interpolar-points",
        type=int,
        default=int(getattr(config, "interpolar_points", 4)),
        help="Fixed package array length used by the fake PLC.",
    )
    parser.add_argument(
        "--log-path",
        default="test_module.log",
        help="JSON-lines log file written by the fake PLC.",
    )
    parser.add_argument(
        "--sample-period",
        type=float,
        default=0.05,
        help="Linear robot simulation sample period in seconds.",
    )
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="Send one demo trajectory package to the fake PLC after startup.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Optional runtime in seconds before the fake PLC stops itself.",
    )
    args = parser.parse_args()

    scheduler_raw = getattr(config, "scheduler", {}) or {}
    raw_home = scheduler_raw.get("home_position", [0.0, 0.0, -300.0])
    home_position = (float(raw_home[0]), float(raw_home[1]), float(raw_home[2]))

    run_fake_plc(
        host=args.host,
        port=args.port,
        interpolar_points=args.interpolar_points,
        log_path=Path(args.log_path),
        sample_period_s=max(args.sample_period, 0.01),
        self_test=args.self_test,
        duration_s=args.duration,
        home_position=home_position,
    )


if __name__ == "__main__":
    main()
