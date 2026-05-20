from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Iterable

try:
    from pylogix import PLC
except ImportError:  # pragma: no cover - handled at runtime
    PLC = None


PARENT_DIR = Path(__file__).parent
CONFIG_FILE = PARENT_DIR / "config.json"

DEFAULT_CONFIG = {
    "ip_address": "192.168.250.1",
    "port": 502,
    "period_s": 0.1,
    "interpolar_points": 4,
}

COMMAND_ID = {
    "stop": 0,
    "goto_relative": 1,
    "goto_absolute": 2,
    "go_trajectory": 3,
    "calibrate": 4,
    "pick": 5,
    "release": 6,
}

COMMAND_NAME = {value: key for key, value in COMMAND_ID.items()}
ARRAY_FIELDS = ("argument_x", "argument_y", "argument_z", "argument_e", "argument_time")


def load_config() -> SimpleNamespace:
    """Load module config and fall back to sane defaults."""

    data = dict(DEFAULT_CONFIG)
    try:
        with CONFIG_FILE.open("r", encoding="utf-8") as handle:
            raw = json.load(handle)
    except FileNotFoundError:
        raw = {}
    except json.JSONDecodeError as exc:
        print(f"[WARN] Invalid config file {CONFIG_FILE}: {exc}. Using defaults.")
        raw = {}

    if isinstance(raw, dict):
        data.update(raw)
    return SimpleNamespace(**data)


def _coerce_list(values: Iterable[Any], size: int, fill_value: Any = 0.0) -> list[Any]:
    result = list(values)[:size]
    if len(result) < size:
        result.extend([fill_value] * (size - len(result)))
    return result


def _zero_package(slots: int) -> dict[str, Any]:
    return {
        "commandID": COMMAND_ID["stop"],
        "argument_number": 0,
        "argument_x": [0.0] * slots,
        "argument_y": [0.0] * slots,
        "argument_z": [0.0] * slots,
        "argument_e": [0.0] * slots,
        "argument_time": [0.0] * slots,
    }


@dataclass
class RobotPacket:
    """Convenience wrapper for building a PLC package."""

    commandID: int
    argument_number: int = 0
    argument_x: list[float] = field(default_factory=list)
    argument_y: list[float] = field(default_factory=list)
    argument_z: list[float] = field(default_factory=list)
    argument_e: list[float] = field(default_factory=list)
    argument_time: list[float] = field(default_factory=list)

    def to_dict(self, slots: int) -> dict[str, Any]:
        package = _zero_package(slots)
        package["commandID"] = self.commandID
        package["argument_number"] = int(self.argument_number)
        package["argument_x"] = _coerce_list(self.argument_x, slots, 0.0)
        package["argument_y"] = _coerce_list(self.argument_y, slots, 0.0)
        package["argument_z"] = _coerce_list(self.argument_z, slots, 0.0)
        package["argument_e"] = _coerce_list(self.argument_e, slots, 0.0)
        package["argument_time"] = _coerce_list(self.argument_time, slots, 0.0)
        return package


class PLCGateway:
    """Simple PLC gateway built on top of pylogix."""

    def __init__(
        self,
        ip: str | None = None,
        port: int | None = None,
        interpolar_points: int | None = None,
        tag_write: str = "pc_package",
        tag_read: str = "plc_package",
    ) -> None:
        self.config = load_config()
        self.ip = ip or self.config.ip_address
        self.port = port or getattr(self.config, "port", DEFAULT_CONFIG["port"])
        self.interpolar_points = interpolar_points or getattr(
            self.config, "interpolar_points", DEFAULT_CONFIG["interpolar_points"]
        )
        self.tag_write = tag_write
        self.tag_read = tag_read
        self.connected = False

        self.plc = PLC() if PLC is not None else None
        if self.plc is not None:
            self.plc.IPAddress = self.ip
            if hasattr(self.plc, "Port"):
                self.plc.Port = self.port

    def _status_tags(self) -> list[str]:
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
            response = self._read_tags(self._status_tags())
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
            normalized = _zero_package(self.interpolar_points)
            normalized["commandID"] = int(package.get("commandID", COMMAND_ID["stop"]))
            normalized["argument_number"] = int(package.get("argument_number", 0))
            for field_name in ARRAY_FIELDS:
                normalized[field_name] = _coerce_list(
                    package.get(field_name, []), self.interpolar_points, 0.0
                )
            package = normalized

        return package

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

    def _read_tags(self, tags: list[str]) -> list[Any]:
        try:
            result = self.plc.Read(tags)
        except Exception:
            self.connected = False
            raise
        if result is None:
            self.connected = False
            return []
        return list(result)

    def send_package(self, package: dict[str, Any] | RobotPacket) -> dict[str, Any]:
        if not self.connected:
            self.connect()

        normalized = self._normalize_package(package)
        for key, value in normalized.items():
            if isinstance(value, list):
                for index, item in enumerate(value):
                    self._write_tag(f"{self.tag_write}.{key}[{index}]", item)
            else:
                self._write_tag(f"{self.tag_write}.{key}", value)
        return normalized

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
        for item in response:
            status = getattr(item, "Status", None)
            if status is not None and str(status).lower() != "success":
                continue
            tag_name = getattr(item, "TagName", "")
            key_name = tag_name.split(".")[-1]
            status_dict[key_name] = getattr(item, "Value", None)

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
        package = _zero_package(self.interpolar_points)
        package["commandID"] = COMMAND_ID[command_name]
        return package


if __name__ == "__main__":
    # Lightweight smoke test so the module can still be run directly.
    config = load_config()
    gateway = PLCGateway(config.ip_address, config.port, config.interpolar_points)
    try:
        gateway.connect()
        test_package = gateway.build_command(
            "go_trajectory",
            x=[100.0, 150.0, 200.0, 250.0],
            y=[0.0, 50.0, 100.0, 150.0],
            z=[-150.0, -150.0, -150.0, -150.0],
            e=[0.0, 0.0, 1.0, 1.0],
            t=[0.5, 0.5, 0.5, 0.5],
            argument_number=config.interpolar_points,
        )
        gateway.send_package(test_package)
        print("[INFO] test package sent")
        print(f"[INFO] status: {gateway.get_package()}")
    finally:
        gateway.disconnect()
