"""An abstract arm: a tool point with a speed and an acceleration limit, nothing more.

The abstraction of the research repo's model family: planar point-to-point moves timed by the
same S-curve segment model the delta uses, and fixed dwell times standing in for the descent
onto the part (`h_pick_s`, contact at `descend_fraction` of it) and the release over the bin
(`h_place_s`). Reach is the workspace window. Use it to ask whether a result depends on the
delta's kinematics or only on its cycle times.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from modules.core.arm_model import Intercept
from modules.core.forecast import BeltForecast
from modules.core.frames import ConveyorFrame
from modules.core.motion import segment_profile_time
from modules.settings import Point, Settings
from sandbox.models import arm_model


@dataclass(frozen=True)
class PointMoverConfig:
    v_max: float = 300.0            # mm/s
    a_max: float = 1000.0           # mm/s^2
    shape_factor: float = 1.5       # S-curve stretch of a stop-and-go move
    dispatch_delay_s: float = 0.186 # command -> first motion
    h_pick_s: float = 0.4           # descent + grab + lift dwell
    descend_fraction: float = 0.5   # contact happens this far into h_pick
    h_place_s: float = 0.3          # descent + release dwell over the bin
    intercept_lead_time_s: float = 0.8


class PointMoverArm:
    def __init__(self, settings: Settings, cfg: PointMoverConfig) -> None:
        self.cfg = cfg
        self.frame = ConveyorFrame.from_settings(settings.conveyor)
        self.window = settings.conveyor.workspace_window_uv
        self.z = settings.robot.heights.pickup
        self.home: Point = settings.robot.home_position
        self.contact_s = cfg.h_pick_s * cfg.descend_fraction
        self.gate_lead_s = cfg.dispatch_delay_s + self.contact_s

    def _move_s(self, a: Point, b: Point) -> float:
        cfg = self.cfg
        return segment_profile_time(math.hypot(b[0] - a[0], b[1] - a[1]), 0.0, 0.0,
                                    cfg.v_max, cfg.a_max, cfg.a_max, cfg.shape_factor)

    def _point(self, u: float, v: float) -> Point:
        x, y = self.frame.to_robot(u, v)
        return (x, y, self.z)

    # ---- model -------------------------------------------------------------------------
    def predict(self, u_at_anchor: float, v: float, start: Point, start_time: float,
                forecast: BeltForecast, anchor_time: float) -> Intercept | None:
        u_min, u_max, v_min, v_max = self.window
        if v < v_min or v > v_max:
            return None
        elapsed = max(0.0, start_time - anchor_time)
        u_start = u_at_anchor + forecast.travel_in(elapsed)
        belt = forecast.at(elapsed)
        if u_start > u_max:
            return None
        t_enter = start_time + (belt.time_to_travel(u_min - u_start) if u_start < u_min else 0.0)
        # Contact no earlier than the minimum lead or the workspace entry; then push it
        # downstream until the arm can be parked a full gate lead before it.
        contact = max(start_time + self.cfg.intercept_lead_time_s, t_enter)
        for _ in range(8):
            u_pick = u_start + belt.travel_in(contact - start_time)
            if u_pick > u_max:
                return None
            park = self._point(u_pick, v)
            required = start_time + self.cfg.dispatch_delay_s + self._move_s(start, park) + self.gate_lead_s
            if required <= contact + 1e-3:
                return Intercept(contact, contact - self.gate_lead_s, park, u_pick, self._move_s(start, park))
            contact = required
        return None

    def costs(self, start: Point, start_time: float, intercept: Intercept, bin_position: Point,
              belt_speed_at_pick: float, setup_s: float) -> tuple[float, float]:
        occupancy = (intercept.gate_fire_time - start_time) + self.pick_time_s(
            intercept.pick_position, bin_position, belt_speed_at_pick) + setup_s
        return intercept.pick_time - start_time, occupancy

    def place_position(self, bin_position: Point) -> Point:
        return (bin_position[0], bin_position[1], self.z)

    # ---- plant -------------------------------------------------------------------------
    def goto_time_s(self, start: Point, park: Point) -> float:
        return self.cfg.dispatch_delay_s + self._move_s(start, park)

    def contact_delay_s(self, belt_speed: float) -> float:
        return self.cfg.dispatch_delay_s + self.contact_s

    def pick_time_s(self, park: Point, bin_position: Point, belt_speed: float) -> float:
        cfg = self.cfg
        return cfg.dispatch_delay_s + cfg.h_pick_s + self._move_s(park, bin_position) + cfg.h_place_s


@arm_model("point_mover", config=PointMoverConfig)
def point_mover(settings: Settings, cfg: PointMoverConfig) -> PointMoverArm:
    """A tool point with speed/acceleration limits and fixed pick/place dwells."""
    return PointMoverArm(settings, cfg)
