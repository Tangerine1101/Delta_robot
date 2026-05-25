from __future__ import annotations

import ctypes
import json
import socket
from dataclasses import dataclass, field
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Iterable

try:
    from pylogix import PLC
except ImportError:  # pragma: no cover - handled at runtime
    PLC = None

try:
    from snap7.client import Client
except ImportError:
    Client = None

# ================= Siemens PLC Memory Config =================
SIEMENS_DB_WRITE = 1          # DB để ghi lệnh xuống PLC (ví dụ DB1)
SIEMENS_DB_WRITE_OFFSET = 0   # Offset byte bắt đầu ghi lệnh

SIEMENS_DB_READ = 2           # DB để đọc trạng thái phản hồi từ PLC (ví dụ DB2)
SIEMENS_DB_READ_OFFSET = 0    # Offset byte bắt đầu đọc trạng thái

class SiemensSendPacket(ctypes.Structure):
    """Cấu trúc gói tin gửi từ PC xuống Siemens PLC (PC -> PLC)"""
    _pack_ = 1
    _fields_ = [
        ("CommandID", ctypes.c_int32),  # Kiểu int (4 bytes)
        ("rotate", ctypes.c_float),     # Kiểu float (4 bytes)
        ("speed", ctypes.c_float),      # Kiểu float (4 bytes)
    ]

class SiemensReceivePacket(ctypes.Structure):
    """Cấu trúc gói tin đọc từ Siemens PLC lên PC (PLC -> PC)"""
    _pack_ = 1
    _fields_ = [
        ("rotate_current", ctypes.c_float), # Kiểu float (4 bytes)
        ("speed_current", ctypes.c_float),  # Kiểu float (4 bytes)
        ("task_doing", ctypes.c_int32),     # Kiểu int (4 bytes)
        ("task_state", ctypes.c_int32),     # Kiểu int (4 bytes)
    ]
# =============================================================


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
    "rotate_absolute": 7,
    "change_speed": 8,
    "plan_siemen": 9,
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


def _coerce_flag_byte(value: Any) -> int:
    return 1 if bool(value) else 0


def _zero_package(slots: int) -> dict[str, Any]:
    return {
        "commandID": COMMAND_ID["stop"],
        "argument_number": 0,
        "argument_x": [0.0] * slots,
        "argument_y": [0.0] * slots,
        "argument_z": [0.0] * slots,
        "argument_e": [0] * slots,
        "argument_time": [0.0] * slots,
        "bit_doing": 0,
    }


@dataclass
class RobotPacket:
    """Convenience wrapper for building a PLC package."""

    commandID: int
    argument_number: int = 0
    argument_x: list[float] = field(default_factory=list)
    argument_y: list[float] = field(default_factory=list)
    argument_z: list[float] = field(default_factory=list)
    argument_e: list[int] = field(default_factory=list)
    argument_time: list[float] = field(default_factory=list)
    bit_doing: int = 1

    def to_dict(self, slots: int) -> dict[str, Any]:
        package = _zero_package(slots)
        package["commandID"] = self.commandID
        package["argument_number"] = int(self.argument_number)
        package["argument_x"] = _coerce_list(self.argument_x, slots, 0.0)
        package["argument_y"] = _coerce_list(self.argument_y, slots, 0.0)
        package["argument_z"] = _coerce_list(self.argument_z, slots, 0.0)
        package["argument_e"] = [_coerce_flag_byte(value) for value in _coerce_list(self.argument_e, slots, 0)]
        package["argument_time"] = _coerce_list(self.argument_time, slots, 0.0)
        package["bit_doing"] = _coerce_flag_byte(self.bit_doing)
        return package


class MockPLC:
    """Mock PLC that uses JSON-lines TCP protocol to communicate with test_module."""

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

    def Write(self, tag_name: str, value: Any) -> Any:
        if self._socket is None:
            if not self.Connect():
                class MockResponse:
                    Status = "Connection Error"
                return MockResponse()
        try:
            req = {"action": "write", "tag": tag_name, "value": value}
            self._socket.sendall((json.dumps(req, ensure_ascii=True) + "\n").encode("utf-8"))
            resp_bytes = self._socket.recv(4096)
            if not resp_bytes:
                raise ConnectionError("Connection closed by peer")
            resp = json.loads(resp_bytes.decode("utf-8").strip())
            class MockResponse:
                Status = "Success" if resp.get("ok") else resp.get("error", "Error")
            return MockResponse()
        except Exception as exc:
            self.Close()
            class MockResponse:
                Status = str(exc)
            return MockResponse()

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


class SiemensGateway:
    """Gateway for Siemens S7-1200 supporting snap7 (Real Mode) and TCP Socket (Mock Mode)."""

    def __init__(self, ip: str | None = None, port: int | None = None) -> None:
        self.config = load_config()
        self.ip = ip or getattr(self.config, "siemens_ip", "192.168.250.2")
        self.port = port or getattr(self.config, "siemens_port", 1502)
        # Các thông số rack và slot dùng cho snap7 (mặc định S7-1200 là rack=0, slot=1)
        self.rack = int(getattr(self.config, "siemens_rack", 0))
        self.slot = int(getattr(self.config, "siemens_slot", 1))
        
        self.connected = False
        
        # Xác định chế độ hoạt động
        self.is_mock = self.ip in ("127.0.0.1", "localhost")
        
        # Biến quản lý kết nối
        self._socket: socket.socket | None = None
        self._snap7_client: Client | None = None

    def connect(self) -> bool:
        if self.connected:
            return True
            
        if self.is_mock:
            # Chế độ giả lập (Mock Mode) - Dùng TCP socket JSON-lines
            try:
                self._socket = socket.create_connection((self.ip, self.port), timeout=2.0)
                self.connected = True
                print(f"[INFO] Siemens gateway (Mock Mode) connected to {self.ip}:{self.port}")
                return True
            except Exception as exc:
                self._socket = None
                self.connected = False
                print(f"[ERROR] Siemens gateway (Mock Mode) failed to connect to {self.ip}:{self.port}: {exc}")
                return False
        else:
            # Chế độ thực tế (Real Mode) - Dùng python-snap7
            if Client is None:
                self.connected = False
                print("[ERROR] Siemens gateway (Real Mode) failed: python-snap7 is not installed.")
                return False
            try:
                self._snap7_client = Client()
                self._snap7_client.connect(self.ip, self.rack, self.slot)
                self.connected = self._snap7_client.get_connected()
                if self.connected:
                    print(f"[INFO] Siemens gateway (Real Mode) connected to {self.ip} (rack={self.rack}, slot={self.slot})")
                return self.connected
            except Exception as exc:
                self._snap7_client = None
                self.connected = False
                print(f"[ERROR] Siemens gateway (Real Mode) failed to connect to {self.ip}: {exc}")
                return False

    def disconnect(self) -> None:
        if self.is_mock:
            if self._socket is not None:
                try:
                    self._socket.close()
                except Exception:
                    pass
                self._socket = None
        else:
            if self._snap7_client is not None:
                try:
                    self._snap7_client.disconnect()
                except Exception:
                    pass
                self._snap7_client = None
        self.connected = False
        print("[INFO] Siemens gateway disconnected")

    def send_package(self, package: dict[str, Any]) -> dict[str, Any] | None:
        """Send Siemens command package to the PLC and read its status back."""
        if not self.connected:
            if not self.connect():
                return None
                
        if self.is_mock:
            # Gửi nhận qua socket (JSON-lines) ở chế độ Mock
            try:
                payload = {
                    "CommandID": int(package.get("CommandID", package.get("commandID", 0))),
                    "rotate": float(package.get("rotate", 0.0)),
                    "speed": float(package.get("speed", 0.0)),
                }
                self._socket.sendall((json.dumps(payload, ensure_ascii=True) + "\n").encode("utf-8"))
                resp_bytes = self._socket.recv(4096)
                if not resp_bytes:
                    raise ConnectionError("Siemens connection closed by peer")
                resp = json.loads(resp_bytes.decode("utf-8").strip())
                return resp
            except Exception as exc:
                print(f"[ERROR] Siemens gateway communication error (Mock): {exc}")
                self.disconnect()
                return None
        else:
            # Gửi nhận qua snap7 (DB Read/Write) ở chế độ Real
            try:
                # 1. Ghi gói tin điều khiển xuống PLC
                send_data = SiemensSendPacket()
                send_data.CommandID = int(package.get("CommandID", package.get("commandID", 0)))
                send_data.rotate = float(package.get("rotate", 0.0))
                send_data.speed = float(package.get("speed", 0.0))
                self._snap7_client.db_write(SIEMENS_DB_WRITE, SIEMENS_DB_WRITE_OFFSET, bytes(send_data))

                # 2. Đọc gói tin phản hồi từ PLC
                read_size = ctypes.sizeof(SiemensReceivePacket)
                raw_bytes = self._snap7_client.db_read(SIEMENS_DB_READ, SIEMENS_DB_READ_OFFSET, read_size)
                recv_data = SiemensReceivePacket.from_buffer_copy(raw_bytes)
                
                return {
                    "rotate_current": recv_data.rotate_current,
                    "speed_current": recv_data.speed_current,
                    "task_doing": recv_data.task_doing,
                    "task_state": recv_data.task_state,
                }
            except Exception as exc:
                print(f"[ERROR] Siemens gateway communication error (Real): {exc}")
                self.disconnect()
                return None

    def get_status(self) -> dict[str, Any] | None:
        """Query state from Siemens PLC."""
        if self.is_mock:
            return self.send_package({"CommandID": 0, "rotate": 0.0, "speed": 0.0})
        else:
            if not self.connected:
                if not self.connect():
                    return None
            try:
                read_size = ctypes.sizeof(SiemensReceivePacket)
                raw_bytes = self._snap7_client.db_read(SIEMENS_DB_READ, SIEMENS_DB_READ_OFFSET, read_size)
                recv_data = SiemensReceivePacket.from_buffer_copy(raw_bytes)
                return {
                    "rotate_current": recv_data.rotate_current,
                    "speed_current": recv_data.speed_current,
                    "task_doing": recv_data.task_doing,
                    "task_state": recv_data.task_state,
                }
            except Exception as exc:
                print(f"[ERROR] Siemens gateway communication error (Real): {exc}")
                self.disconnect()
                return None


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
        ]

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
            response = self._read_tags(self._probe_tags())
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
                fill_value = 0 if field_name == "argument_e" else 0.0
                normalized[field_name] = _coerce_list(
                    package.get(field_name, []), self.interpolar_points, fill_value
                )
            normalized["argument_e"] = [
                _coerce_flag_byte(value) for value in normalized["argument_e"]
            ]
            normalized["bit_doing"] = _coerce_flag_byte(
                package.get("bit_doing", package.get("doing_bit", 1))
            )
            package = normalized

        if package["commandID"] == COMMAND_ID["goto_absolute"]:
            package["argument_e"] = [0] * self.interpolar_points

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
        normalized["bit_doing"] = 1
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
            e=[0, 0, 1, 1],
            t=[0.5, 0.5, 0.5, 0.5],
            argument_number=config.interpolar_points,
        )
        gateway.send_package(test_package)
        print("[INFO] test package sent")
        print(f"[INFO] status: {gateway.get_package()}")
    finally:
        gateway.disconnect()
