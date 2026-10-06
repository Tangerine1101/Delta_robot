"""Servo plant between the PLC setpoint and the encoder (``Act.Pos``).

The following behaviour of the real MADLN05BE drives has not been measured
(open issue C5); ``tau_s`` is a first-order lag parameter, 0 = ideal tracking.
"""
from __future__ import annotations

import math
from typing import Any


class ServoAxis:
    def __init__(self, angle_deg: float = 0.0, tau_s: float = 0.0) -> None:
        self.setpoint = float(angle_deg)
        self.actual = float(angle_deg)
        self.tau_s = max(float(tau_s), 0.0)
        self.owner: Any = None  # the motion instruction currently driving the axis

    def command(self, angle_deg: float) -> None:
        self.setpoint = float(angle_deg)

    def step(self, dt: float) -> None:
        if self.tau_s <= 0.0:
            self.actual = self.setpoint
        else:
            self.actual += (self.setpoint - self.actual) * (1.0 - math.exp(-dt / self.tau_s))

    @property
    def following_error(self) -> float:
        return self.setpoint - self.actual
