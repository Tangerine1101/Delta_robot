"""Siemens S7-1200 stand-in: belt odometer and the 4th-DOF cup rotation.

Contract: ``doc/basis-programming.md`` §3.2 (DB1 command, DB2 status). The real
program is not in the repository; only the PC-visible effects are modelled:
command 7 rotates, 8 sets the belt speed, 9 does both; DB2 reports the belt
position in **mm** as a REAL. ``task_state`` has no reliable meaning on the real PLC
(open issue L3); here it is 1 while the belt ramps or the cup turns, else 0.
"""
from __future__ import annotations

import math
from typing import Any

from modules.core.kinematics import f32

ROTATE_LIMIT_DEG = 359.0


class SiemensPLC:
    def __init__(self, belt_accel_mm_s2: float = 22.31, rotate_rate_deg_s: float = 180.0) -> None:
        self.belt_accel_mm_s2 = max(float(belt_accel_mm_s2), 0.0)
        self.rotate_rate_deg_s = float(rotate_rate_deg_s)
        self.speed_setpoint = 0.0
        self.speed_current = 0.0
        self.position_mm = 0.0
        self.rotate_target = 0.0
        self.rotate_current = 0.0
        self.task_doing = 0
        self.task_state = 0

    def accept(self, command_id: int, rotate: float, speed: float) -> dict[str, Any]:
        self.task_doing = int(command_id)
        if command_id in (7, 9):
            self.rotate_target = max(-ROTATE_LIMIT_DEG, min(ROTATE_LIMIT_DEG, float(rotate)))
        if command_id in (8, 9):
            self.speed_setpoint = max(0.0, float(speed))
        return self.status()

    def step(self, dt: float) -> None:
        error = self.speed_setpoint - self.speed_current
        dv = self.belt_accel_mm_s2 * dt
        if self.belt_accel_mm_s2 <= 0.0 or abs(error) <= dv:
            self.speed_current = self.speed_setpoint
        else:
            self.speed_current += math.copysign(dv, error)
        self.position_mm += self.speed_current * dt
        delta = self.rotate_target - self.rotate_current
        step = self.rotate_rate_deg_s * dt
        self.rotate_current = self.rotate_target if abs(delta) <= step else (
            self.rotate_current + math.copysign(step, delta))
        settled = self.speed_current == self.speed_setpoint and self.rotate_current == self.rotate_target
        self.task_state = 0 if settled else 1

    def status(self) -> dict[str, Any]:
        return {
            "rotate_current": f32(self.rotate_current),
            "speed_current": f32(self.speed_current),
            "task_doing": self.task_doing,
            "task_state": self.task_state,
            "conveyor_position": f32(self.position_mm),
        }
