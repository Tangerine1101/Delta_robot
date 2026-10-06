"""Siemens S7-1200 stand-in: the 4th-DOF cup rotation.

Contract: ``doc/basis-programming.md`` §3.2 (DB1 command, DB2 status). The real
program is not in the repository; only the PC-visible effects are modelled:
commands 7 and 9 rotate. The belt is driven by the Omron (command 8,
``omron_core.ConveyorAxis``), so DB2 ``speed_current`` / ``conveyor_position`` stay
0 here. ``task_state`` has no reliable meaning on the real PLC (open issue L3);
here it is 1 while the cup turns, else 0.
"""
from __future__ import annotations

import math
from typing import Any

from modules.core.kinematics import f32

ROTATE_LIMIT_DEG = 359.0


class SiemensPLC:
    def __init__(self, rotate_rate_deg_s: float = 180.0) -> None:
        self.rotate_rate_deg_s = float(rotate_rate_deg_s)
        self.rotate_target = 0.0
        self.rotate_current = 0.0
        self.task_doing = 0
        self.task_state = 0

    def accept(self, command_id: int, rotate: float, speed: float) -> dict[str, Any]:
        self.task_doing = int(command_id)
        if command_id in (7, 9):
            self.rotate_target = max(-ROTATE_LIMIT_DEG, min(ROTATE_LIMIT_DEG, float(rotate)))
        return self.status()

    def step(self, dt: float) -> None:
        delta = self.rotate_target - self.rotate_current
        step = self.rotate_rate_deg_s * dt
        self.rotate_current = self.rotate_target if abs(delta) <= step else (
            self.rotate_current + math.copysign(step, delta))
        self.task_state = 0 if self.rotate_current == self.rotate_target else 1

    def status(self) -> dict[str, Any]:
        return {
            "rotate_current": f32(self.rotate_current),
            "speed_current": 0.0,
            "task_doing": self.task_doing,
            "task_state": self.task_state,
            "conveyor_position": 0.0,
        }
