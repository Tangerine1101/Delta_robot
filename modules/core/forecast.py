"""Belt position forecast under a held setpoint.

A speed law has to ask "if I commanded V, where would each object be at time t?" about
speeds the belt is not running at, so the prediction is a value rather than a query on the
live belt. The assumption — the setpoint then holds — is made true by the speed controller,
which never moves the setpoint between goto dispatch and cup contact (doc/basis-theory.md §7.4,
open-issues L9).

The drive ramps linearly at `accel_mm_s2` (the Omron belt servo and the PLC simulator both use a
constant acceleration). With jerk enabled the S-curve is replaced by its chord: exact across a
whole ramp, approximate inside one.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class BeltForecast:
    v0: float                    # belt speed at the forecast's origin (mm/s)
    target: float                # setpoint the drive is heading for (mm/s)
    accel_mm_s2: float = 0.0     # 0 = the drive jumps to target instantly
    jerk_mm_s3: float = 0.0

    def settling_time_s(self, delta_v: float) -> float:
        """Time for the drive to change speed by `delta_v`."""
        if self.accel_mm_s2 <= 0.0:
            return 0.0
        dv = abs(delta_v)
        if self.jerk_mm_s3 <= 0.0:
            return dv / self.accel_mm_s2
        if dv >= self.accel_mm_s2 ** 2 / self.jerk_mm_s3:
            return dv / self.accel_mm_s2 + self.accel_mm_s2 / self.jerk_mm_s3
        return 2.0 * math.sqrt(dv / self.jerk_mm_s3)

    @property
    def ramp_s(self) -> float:
        return self.settling_time_s(self.target - self.v0)

    def speed_at(self, dt: float) -> float:
        ramp = self.ramp_s
        if dt >= ramp:
            return self.target
        return self.v0 + (self.target - self.v0) * (dt / ramp)

    def at(self, dt: float) -> "BeltForecast":
        """The same forecast, re-originated `dt` seconds later."""
        return BeltForecast(self.speed_at(dt), self.target, self.accel_mm_s2, self.jerk_mm_s3)

    def reachable_in(self, horizon_s: float) -> bool:
        return self.ramp_s <= horizon_s

    def travel_in(self, dt: float) -> float:
        """Belt travel (mm) over the next `dt` seconds."""
        if dt <= 0.0:
            return 0.0
        ramp = self.ramp_s
        if dt >= ramp:
            return 0.5 * (self.v0 + self.target) * ramp + self.target * (dt - ramp)
        v_end = self.v0 + (self.target - self.v0) * (dt / ramp)
        return 0.5 * (self.v0 + v_end) * dt

    def time_to_travel(self, distance_mm: float) -> float:
        """Seconds until the belt has moved `distance_mm`; inf if it never does."""
        if distance_mm <= 0.0:
            return 0.0
        ramp = self.ramp_s
        during_ramp = 0.5 * (self.v0 + self.target) * ramp

        if distance_mm > during_ramp:
            if self.target <= 0.0:
                return float("inf")
            return ramp + (distance_mm - during_ramp) / self.target

        if ramp <= 0.0:
            return distance_mm / self.v0 if self.v0 > 0.0 else float("inf")
        k = (self.target - self.v0) / ramp
        if abs(k) < 1e-12:
            return distance_mm / self.v0 if self.v0 > 0.0 else float("inf")
        disc = self.v0 * self.v0 + 2.0 * k * distance_mm
        if disc < 0.0:
            return float("inf")
        return (math.sqrt(disc) - self.v0) / k


def steady(speed_mm_s: float) -> BeltForecast:
    """A belt that keeps its current speed."""
    return BeltForecast(speed_mm_s, speed_mm_s)


def live_forecast(
    measured_mm_s: float,
    setpoint_mm_s: float,
    accel_mm_s2: float,
    settled_band_mm_s: float,
) -> BeltForecast:
    """Forecast from the live belt: steady at the measured speed when it already sits within
    `settled_band_mm_s` of the setpoint (encoder noise, drive scale error), otherwise a ramp
    from the measured speed to the setpoint."""
    if abs(setpoint_mm_s - measured_mm_s) <= settled_band_mm_s:
        return steady(measured_mm_s)
    return BeltForecast(measured_mm_s, max(0.0, setpoint_mm_s), accel_mm_s2)
