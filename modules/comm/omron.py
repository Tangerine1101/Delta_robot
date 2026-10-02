"""Omron NX1P2 gateway (EtherNet/IP through pylogix), and the JSON-lines mock the PLC
simulator speaks when the address is localhost."""
from __future__ import annotations

import json
import socket
import time
from typing import Any

from modules.comm.packets import (
    ARRAY_FIELDS,
    COMMAND_ID,
    RobotPacket,
    coerce_flag_byte,
    coerce_list,
    zero_package,
)
from modules.settings import RobotLimits

try:
    from pylogix import PLC
except ImportError:  # pragma: no cover - handled at runtime
    PLC = None

# Commands whose argument_x/argument_y carry ABSOLUTE robot-frame coordinates and
# must therefore be checked against the physical reach limit. goto_relative (1)
# holds relative/joint deltas, not an absolute position, so it is excluded.
_ABSOLUTE_POSITION_COMMANDS = (COMMAND_ID["goto_absolute"], COMMAND_ID["go_trajectory"])

# Belt feedback members of plc_package (Section4 telemetry of MC_Conveyor): velocity in
# mm/s and position in mm, both positive along the belt; conveyor_state 0 stopped,
# 1 ramping, 2 at speed, 3 error, 4 servo off.
CONVEYOR_STATUS_FIELDS = ("conveyor_velocity", "conveyor_position", "conveyor_state")

# How long send_package waits for the PLC to consume a belt command (bit_doing back to 0)
# before the next command may overwrite pc_package.commandID.
_CONVEYOR_ACK_TIMEOUT_S = 0.1
_CONVEYOR_ACK_POLL_S = 0.002


class WorkspaceLimitError(ValueError):
    """Raised when a command would move the robot outside its physical reach."""


class MockPLC:
    """Mock PLC speaking the JSON-lines TCP protocol of the PLC simulator (modules.plc_sim)."""

    def __init__(self) -> None:
        self.IPAddress = "127.0.0.1"
        self.Port = 502
        self._socket: socket.socket | None = None

    def Connect(self) -> bool:
        if self._socket is not None:
            return True
        try:
            self._socket = socket.create_connection((self.IPAddress, self.Port), timeout=2.0)
            return True
        except Exception:
            self._socket = None
            return False

    def Close(self) -> None:
        if self._socket is not None:
            try:
                self._socket.close()
            except Exception:
                pass
            self._socket = None

    def Write(self, tag_name: str | list[tuple[str, Any]], value: Any = None) -> Any:
        class MockResponse:
            def __init__(self, t: Any = None, v: Any = None, s: str = "Success") -> None:
                self.TagName = t
                self.Value = v
                self.Status = s

        if self._socket is None:
            if not self.Connect():
                if isinstance(tag_name, list):
                    return [MockResponse(t, v, "Connection Error") for t, v in tag_name]
                return MockResponse(tag_name, value, "Connection Error")
        try:
            if isinstance(tag_name, list):
                results = []
                for t, v in tag_name:
                    req = {"action": "write", "tag": t, "value": v}
                    self._socket.sendall((json.dumps(req, ensure_ascii=True) + "\n").encode("utf-8"))
                    resp_bytes = self._socket.recv(4096)
                    if not resp_bytes:
                        raise ConnectionError("Connection closed by peer")
                    resp = json.loads(resp_bytes.decode("utf-8").strip())
                    status = "Success" if resp.get("ok") else resp.get("error", "Error")
                    results.append(MockResponse(t, v, status))
                return results
            else:
                req = {"action": "write", "tag": tag_name, "value": value}
                self._socket.sendall((json.dumps(req, ensure_ascii=True) + "\n").encode("utf-8"))
                resp_bytes = self._socket.recv(4096)
                if not resp_bytes:
                    raise ConnectionError("Connection closed by peer")
                resp = json.loads(resp_bytes.decode("utf-8").strip())
                status = "Success" if resp.get("ok") else resp.get("error", "Error")
                return MockResponse(tag_name, value, status)
        except Exception as exc:
            self.Close()
            if isinstance(tag_name, list):
                return [MockResponse(t, v, str(exc)) for t, v in tag_name]
            return MockResponse(tag_name, value, str(exc))

    def Read(self, tags: list[str]) -> list[Any] | None:
        if self._socket is None:
            if not self.Connect():
                return None
        try:
            req = {"action": "read", "tags": tags}
            self._socket.sendall((json.dumps(req, ensure_ascii=True) + "\n").encode("utf-8"))
            resp_bytes = self._socket.recv(4096)
            if not resp_bytes:
                raise ConnectionError("Connection closed by peer")
            resp = json.loads(resp_bytes.decode("utf-8").strip())
            if not resp.get("ok"):
                return None
            values = resp.get("values", {})
            class MockResponseItem:
                def __init__(self, tag_name: str, val: Any) -> None:
                    self.TagName = tag_name
                    self.Value = val
                    self.Status = "Success"
            return [MockResponseItem(t, values.get(t)) for t in tags]
        except Exception:
            self.Close()
            return None


class PLCGateway:
    """Simple PLC gateway built on top of pylogix."""

    def __init__(
        self,
        ip: str,
        port: int,
        interpolar_points: int,
        limits: RobotLimits,
        tag_write: str = "pc_package",
        tag_read: str = "plc_package",
    ) -> None:
        self.ip = ip
        self.port = port
        self.interpolar_points = interpolar_points
        self.tag_write = tag_write
        self.tag_read = tag_read
        self.connected = False
        # Motion envelope: points outside robot.limits are rejected, never clamped.
        self.limits = limits

        if PLC is None and self.ip not in ("127.0.0.1", "localhost"):
            # MockPLC speaks JSON-lines, not EtherNet/IP: against a real PLC every
            # read returns None. Stop here instead of retrying forever.
            raise ImportError(
                f"pylogix is not importable in this interpreter, so the real PLC at "
                f"{self.ip} cannot be reached. Run with the project venv "
                f"(.venv/bin/python) or `pip install pylogix`."
            )
        if PLC is not None and self.ip not in ("127.0.0.1", "localhost"):
            self.plc = PLC()
            self.plc.IPAddress = self.ip
            if hasattr(self.plc, "Port"):
                self.plc.Port = self.port
        else:
            self.plc = MockPLC()
            self.plc.IPAddress = self.ip
            self.plc.Port = self.port

    def _status_tags(self) -> list[str]:
        return [
            f"{self.tag_read}.pos_angular[0]",
            f"{self.tag_read}.pos_angular[1]",
            f"{self.tag_read}.pos_angular[2]",
            f"{self.tag_read}.pos_EE[0]",
            f"{self.tag_read}.pos_EE[1]",
            f"{self.tag_read}.pos_EE[2]",
            f"{self.tag_read}.task_doing",
            f"{self.tag_read}.task_state",
            f"{self.tag_read}.end_effector",
        ] + self._conveyor_tags()

    def _conveyor_tags(self) -> list[str]:
        return [f"{self.tag_read}.{name}" for name in CONVEYOR_STATUS_FIELDS]

    def _probe_tags(self) -> list[str]:
        return [
            f"{self.tag_read}.task_doing",
            f"{self.tag_read}.task_state",
        ]

    def _response_has_success(self, response: list[Any]) -> bool:
        if not response:
            return False

        for item in response:
            status = getattr(item, "Status", None)
            if status is None or str(status).lower() == "success":
                return True
        return False

    def _probe_connection(self) -> bool:
        try:
            response = self._read_tags(self._probe_tags(), retry=False)
        except Exception:
            return False
        return self._response_has_success(response)

    def connect(self) -> bool:
        if self.plc is None:
            self.connected = False
            raise ImportError(
                "pylogix is not installed. Install it or replace PLCGateway with "
                "the communication backend you are using."
            )

        if not self._probe_connection():
            self.connected = False
            raise ConnectionError(f"Unable to reach PLC at {self.ip}:{self.port}")

        self.connected = True
        print(f"[INFO] PLC gateway connected to {self.ip}:{self.port}")
        return True

    def disconnect(self) -> None:
        if self.plc is None:
            self.connected = False
            return

        try:
            self.plc.Close()
        except Exception as exc:  # pragma: no cover - defensive cleanup
            self.connected = self._probe_connection()
            if self.connected:
                raise RuntimeError(f"PLC connection is still active after Close(): {exc}") from exc
            print(f"[WARN] PLC close raised an error but connection is no longer active: {exc}")
            return

        self.connected = False
        print("[INFO] PLC connection closed")

    def _normalize_package(self, package: dict[str, Any] | RobotPacket) -> dict[str, Any]:
        if isinstance(package, RobotPacket):
            package = package.to_dict(self.interpolar_points)
        else:
            normalized = zero_package(self.interpolar_points)
            normalized["commandID"] = int(package.get("commandID", COMMAND_ID["stop"]))
            normalized["argument_number"] = int(package.get("argument_number", 0))
            for field_name in ARRAY_FIELDS:
                fill_value = 0 if field_name == "argument_e" else 0.0
                normalized[field_name] = coerce_list(
                    package.get(field_name, []), self.interpolar_points, fill_value
                )
            normalized["argument_e"] = [
                coerce_flag_byte(value) for value in normalized["argument_e"]
            ]
            normalized["bit_doing"] = coerce_flag_byte(
                package.get("bit_doing", package.get("doing_bit", 1))
            )
            package = normalized

        if package["commandID"] == COMMAND_ID["goto_absolute"]:
            package["argument_e"] = [0] * self.interpolar_points

        return package

    def _check_workspace_limit(self, package: dict[str, Any]) -> None:
        """Reject any absolute-position command with a point outside self.limits.

        Only goto_absolute / go_trajectory carry absolute robot XYZ; for those we
        check the first `argument_number` points (the meaningful ones — the rest is
        padding). Raises WorkspaceLimitError on the first violation so no
        out-of-envelope point is ever written to the PLC.
        """
        if package.get("commandID") not in _ABSOLUTE_POSITION_COMMANDS:
            return

        xs = package.get("argument_x", [])
        ys = package.get("argument_y", [])
        zs = package.get("argument_z", [])
        n_points = max(int(package.get("argument_number", 0) or 0), 1)
        for i in range(min(n_points, len(xs), len(ys), len(zs))):
            x, y, z = float(xs[i]), float(ys[i]), float(zs[i])
            problem = self.limits.violation(x, y, z)
            if problem is not None:
                raise WorkspaceLimitError(f"point {i} ({x:.1f}, {y:.1f}, {z:.1f}): {problem}")

    @staticmethod
    def _write_result_ok(result: Any) -> bool:
        if result is None:
            return True
        status = getattr(result, "Status", None)
        if status is None:
            return True
        return str(status).lower() == "success"

    def _write_tag(self, tag_name: str, value: Any) -> None:
        try:
            result = self.plc.Write(tag_name, value)
        except Exception:
            self.connected = False
            raise
        if not self._write_result_ok(result):
            self.connected = False
            raise RuntimeError(f"Write failed for {tag_name}: {getattr(result, 'Status', result)}")

    def _write_tags(self, tags_and_values: list[tuple[str, Any]]) -> None:
        for attempt in range(2):
            try:
                results = self.plc.Write(tags_and_values)
            except Exception as exc:
                self.connected = False
                if attempt == 0:
                    print(f"[WARN] PLCGateway write error (attempt 1), reconnecting: {exc}")
                    self.connect()
                    continue
                raise
            ok = True
            if isinstance(results, list):
                for r in results:
                    if not self._write_result_ok(r):
                        ok = False
                        err_msg = f"Write failed for {getattr(r, 'TagName', 'unknown')}: {getattr(r, 'Status', r)}"
                        break
            else:
                if not self._write_result_ok(results):
                    ok = False
                    err_msg = f"Write failed: {getattr(results, 'Status', results)}"
            if ok:
                return
            self.connected = False
            if attempt == 0:
                print(f"[WARN] PLCGateway write result error (attempt 1), reconnecting: {err_msg}")
                self.connect()
                continue
            raise RuntimeError(err_msg)

    def _read_tags(self, tags: list[str], *, retry: bool = True) -> list[Any]:
        # retry=False from the connection probe: connect() -> probe -> _read_tags
        # must not call connect() again, or a dead link recurses without bound.
        for attempt in range(2 if retry else 1):
            try:
                result = self.plc.Read(tags)
            except Exception as exc:
                self.connected = False
                if retry and attempt == 0:
                    print(f"[WARN] PLCGateway read error (attempt 1), reconnecting: {exc}")
                    self.connect()
                    continue
                raise
            if result is None:
                self.connected = False
                if retry and attempt == 0:
                    print("[WARN] PLCGateway read returned None (attempt 1), reconnecting")
                    self.connect()
                    continue
                return []
            return list(result)
        return []

    def send_package(self, package: dict[str, Any] | RobotPacket) -> dict[str, Any]:
        if not self.connected:
            self.connect()

        if not isinstance(package, RobotPacket) and package.get("commandID") == COMMAND_ID["change_speed"]:
            return self._send_conveyor_speed(float(package.get("speed", 0.0)))

        normalized = self._normalize_package(package)
        self._check_workspace_limit(normalized)
        normalized["bit_doing"] = 1

        tags_and_values: list[tuple[str, Any]] = []
        for key, value in normalized.items():
            if key == "bit_doing":
                continue
            if key in ARRAY_FIELDS:
                for index, element in enumerate(value):
                    tags_and_values.append((f"{self.tag_write}.{key}[{index}]", element))
            else:
                tags_and_values.append((f"{self.tag_write}.{key}", value))
        tags_and_values.append((f"{self.tag_write}.bit_doing", normalized["bit_doing"]))

        self._write_tags(tags_and_values)
        return normalized

    def _send_conveyor_speed(self, speed_mm_s: float) -> dict[str, Any]:
        """Command 8: belt speed in mm/s. Only conveyor_speed, commandID and bit_doing are
        written, so the trajectory arguments of a running command 3 are left alone.

        The PLC dispatches one command per scan through a single commandID, so this
        waits until it has consumed the belt command; otherwise a command sent right
        after (a goto at arm_free) could overwrite commandID before the PLC read it.
        """
        if speed_mm_s < 0.0:
            raise ValueError(f"conveyor speed must be >= 0 mm/s (the belt runs one way), got {speed_mm_s}")
        self._write_tags([
            (f"{self.tag_write}.conveyor_speed", float(speed_mm_s)),
            (f"{self.tag_write}.commandID", COMMAND_ID["change_speed"]),
            (f"{self.tag_write}.bit_doing", 1),
        ])
        deadline = time.monotonic() + _CONVEYOR_ACK_TIMEOUT_S
        while True:
            response = self._read_tags([f"{self.tag_write}.bit_doing"])
            value = getattr(response[0], "Value", None) if response else None
            if value is not None and int(value) == 0:
                break
            if time.monotonic() >= deadline:
                print(f"[WARN] PLC did not acknowledge conveyor command within "
                      f"{_CONVEYOR_ACK_TIMEOUT_S * 1000:.0f} ms (bit_doing={value})")
                break
            time.sleep(_CONVEYOR_ACK_POLL_S)
        return {"commandID": COMMAND_ID["change_speed"], "conveyor_speed": float(speed_mm_s)}

    def get_conveyor(self) -> dict[str, Any] | None:
        """The belt feedback alone: conveyor_velocity, conveyor_position, conveyor_state."""
        if not self.connected:
            self.connect()
        try:
            response = self._read_tags(self._conveyor_tags())
        except Exception as exc:
            self.connected = False
            print(f"[ERROR] Unable to read PLC conveyor status: {exc}")
            return None
        values = {}
        for item in response:
            status = getattr(item, "Status", None)
            if status is not None and str(status).lower() != "success":
                continue
            values[getattr(item, "TagName", "").split(".")[-1]] = getattr(item, "Value", None)
        return values or None

    def get_package(self) -> dict[str, Any] | None:
        if not self.connected:
            self.connect()

        tags = self._status_tags()
        try:
            response = self._read_tags(tags)
        except Exception as exc:
            self.connected = False
            print(f"[ERROR] Unable to read PLC status: {exc}")
            return None

        if not self._response_has_success(response):
            self.connected = False
            return None

        status_dict: dict[str, Any] = {}
        list_values: dict[str, dict[int, Any]] = {}
        for item in response:
            status = getattr(item, "Status", None)
            if status is not None and str(status).lower() != "success":
                continue
            tag_name = getattr(item, "TagName", "")
            key_name = tag_name.split(".")[-1]
            value = getattr(item, "Value", None)
            if "[" in key_name and key_name.endswith("]"):
                base_name, raw_index = key_name[:-1].split("[", 1)
                try:
                    index = int(raw_index)
                except ValueError:
                    status_dict[key_name] = value
                    continue
                list_values.setdefault(base_name, {})[index] = value
                continue
            status_dict[key_name] = value

        for base_name, indexed_values in list_values.items():
            status_dict[base_name] = [
                indexed_values[index] for index in sorted(indexed_values)
            ]

        self.connected = True
        return status_dict or None

    def build_command(
        self,
        command_name: str,
        *,
        x=None,
        y=None,
        z=None,
        e=None,
        t=None,
        argument_number: int = 0,
    ) -> dict[str, Any]:
        if command_name not in COMMAND_ID:
            raise KeyError(f"Unknown command: {command_name}")

        return RobotPacket(
            commandID=COMMAND_ID[command_name],
            argument_number=argument_number,
            argument_x=list(x or []),
            argument_y=list(y or []),
            argument_z=list(z or []),
            argument_e=list(e or []),
            argument_time=list(t or []),
        ).to_dict(self.interpolar_points)

    def build_zero_command(self, command_name: str) -> dict[str, Any]:
        if command_name not in COMMAND_ID:
            raise KeyError(f"Unknown command: {command_name}")
        package = zero_package(self.interpolar_points)
        package["commandID"] = COMMAND_ID[command_name]
        return package
