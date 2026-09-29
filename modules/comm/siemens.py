"""Siemens S7-1200 gateway: snap7 against the real PLC, JSON-lines against the simulator
(localhost). Carries belt speed commands, 4th-DOF rotation commands and their feedback."""
from __future__ import annotations

import ctypes
import json
import socket
from typing import Any

from modules.comm.packets import (
    SIEMENS_DB_READ,
    SIEMENS_DB_READ_OFFSET,
    SIEMENS_DB_WRITE,
    SIEMENS_DB_WRITE_OFFSET,
    SiemensReceivePacket,
    SiemensSendPacket,
)

try:
    from snap7.client import Client
except ImportError:
    Client = None

# snap7 rack/slot of an S7-1200.
SIEMENS_RACK = 0
SIEMENS_SLOT = 1

class SiemensGateway:
    """Gateway for Siemens S7-1200 supporting snap7 (Real Mode) and TCP Socket (Mock Mode)."""

    def __init__(self, ip: str, port: int) -> None:
        self.ip = ip
        self.port = port
        self.rack = SIEMENS_RACK
        self.slot = SIEMENS_SLOT

        self.connected = False

        # Determine operating mode
        self.is_mock = self.ip in ("127.0.0.1", "localhost")

        # Connection management variables
        self._socket: socket.socket | None = None
        self._snap7_client: Client | None = None

    def connect(self) -> bool:
        if self.connected:
            return True
            
        if self.is_mock:
            # Mock Mode — TCP socket JSON-lines
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
            # Real Mode — python-snap7
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
            # Send/receive via socket (JSON-lines) in Mock mode
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
            # Send/receive via snap7 (DB Read/Write) in Real mode
            try:
                # 1. Write command packet to PLC
                send_data = SiemensSendPacket()
                send_data.CommandID = int(package.get("CommandID", package.get("commandID", 0)))
                send_data.rotate = float(package.get("rotate", 0.0))
                send_data.speed = float(package.get("speed", 0.0))
                self._snap7_client.db_write(SIEMENS_DB_WRITE, SIEMENS_DB_WRITE_OFFSET, bytes(send_data))

                # 2. Read response packet from PLC
                read_size = ctypes.sizeof(SiemensReceivePacket)
                raw_bytes = self._snap7_client.db_read(SIEMENS_DB_READ, SIEMENS_DB_READ_OFFSET, read_size)
                recv_data = SiemensReceivePacket.from_buffer_copy(raw_bytes)
                
                return {
                    "rotate_current": recv_data.rotate_current,
                    "speed_current": recv_data.speed_current,
                    "task_doing": recv_data.task_doing,
                    "task_state": recv_data.task_state,
                    "conveyor_position": recv_data.conveyor_position,
                }
            except Exception as exc:
                print(f"[ERROR] Siemens gateway communication error (Real): {exc}")
                self.disconnect()
                return None

    def get_status(self) -> dict[str, Any] | None:
        """Query state from Siemens PLC (read-only; no command side-effects)."""
        if self.is_mock:
            if not self.connected:
                if not self.connect():
                    return None
            try:
                req = {"action": "siemens_status"}
                self._socket.sendall((json.dumps(req, ensure_ascii=True) + "\n").encode("utf-8"))
                resp_bytes = self._socket.recv(4096)
                if not resp_bytes:
                    raise ConnectionError("Siemens connection closed by peer")
                return json.loads(resp_bytes.decode("utf-8").strip())
            except Exception as exc:
                print(f"[ERROR] Siemens gateway communication error (Mock): {exc}")
                self.disconnect()
                return None
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
                    "conveyor_position": recv_data.conveyor_position,
                }
            except Exception as exc:
                print(f"[ERROR] Siemens gateway communication error (Real): {exc}")
                self.disconnect()
                return None
