"""The PC <-> PLC data contract: command IDs and packet layouts.

The byte layouts below must match the PLC DB offsets exactly (basis-programming.md §3):
never remove or reorder a field of `SiemensSendPacket` / `SiemensReceivePacket`, and every Omron
packet array is padded to `plc.interpolar_points` elements.
"""
from __future__ import annotations

import ctypes
from dataclasses import dataclass, field
from typing import Any, Iterable

# ================= Siemens PLC Memory Config =================
SIEMENS_DB_WRITE = 1          # DB for writing commands to the PLC (e.g. DB1)
SIEMENS_DB_WRITE_OFFSET = 0   # Byte offset for command write start

SIEMENS_DB_READ = 2           # DB for reading status feedback from the PLC (e.g. DB2)
SIEMENS_DB_READ_OFFSET = 0    # Byte offset for status read start

class SiemensSendPacket(ctypes.BigEndianStructure):
    """Command packet sent from PC to Siemens PLC (PC -> PLC)."""
    _fields_ = [
        ("CommandID", ctypes.c_int32),  # int (4 bytes)
        ("rotate", ctypes.c_float),     # float (4 bytes)
        ("speed", ctypes.c_float),      # float (4 bytes)
    ]

class SiemensReceivePacket(ctypes.BigEndianStructure):
    """Status packet read from Siemens PLC by the PC (PLC -> PC)."""
    _fields_ = [
        ("rotate_current", ctypes.c_float), # float (4 bytes)
        ("speed_current", ctypes.c_float),  # float (4 bytes)
        ("task_doing", ctypes.c_int32),     # int (4 bytes)
        ("task_state", ctypes.c_int32),     # int (4 bytes)
        ("conveyor_position", ctypes.c_float),  # float (4 bytes) — belt position in mm (REAL), pre-decoded by PLC (scale = 1.0)
    ]
# =============================================================


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
    "enable": 10,
}

COMMAND_NAME = {value: key for key, value in COMMAND_ID.items()}
ARRAY_FIELDS = ("argument_x", "argument_y", "argument_z", "argument_e", "argument_time")


def coerce_list(values: Iterable[Any], size: int, fill_value: Any = 0.0) -> list[Any]:
    result = list(values)[:size]
    if len(result) < size:
        result.extend([fill_value] * (size - len(result)))
    return result


def coerce_flag_byte(value: Any) -> int:
    return 1 if bool(value) else 0


def zero_package(slots: int) -> dict[str, Any]:
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
        package = zero_package(slots)
        package["commandID"] = self.commandID
        package["argument_number"] = int(self.argument_number)
        package["argument_x"] = coerce_list(self.argument_x, slots, 0.0)
        package["argument_y"] = coerce_list(self.argument_y, slots, 0.0)
        package["argument_z"] = coerce_list(self.argument_z, slots, 0.0)
        package["argument_e"] = [coerce_flag_byte(value) for value in coerce_list(self.argument_e, slots, 0)]
        package["argument_time"] = coerce_list(self.argument_time, slots, 0.0)
        package["bit_doing"] = coerce_flag_byte(self.bit_doing)
        return package
