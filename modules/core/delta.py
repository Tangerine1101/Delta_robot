"""The cell's delta arm as an `ArmModel`: 7-point templates, the PLC time model, the gate lead.

This is the one delta model in the repository: the realtime loop plans with it and the sandbox
simulates with it, so an algorithm is judged on the motion the robot actually runs.

Timing chain of one pick (doc/basis-theory.md §4.4):

    goto dispatched ──(command delay + goto flight)──► parked above the intercept
    gate fires at  u_pick − v·gate_lead ──(command delay [+ descent])──► cup contact

`predict` is the realtime intercept solver generalised to an arbitrary start position, start
time and belt forecast: every `speed * dt` is `forecast.travel_in(dt)` and every
`distance / speed` is `forecast.time_to_travel(distance)`.
"""

from __future__ import annotations

import math

from modules.core.arm_model import Intercept
from modules.core.forecast import BeltForecast
from modules.core.frames import ConveyorFrame, UVWindow
from modules.core.kinematics import ik_reachable
from modules.core.motion import segment_profile_time, trajectory_time
from modules.core.trajectory import goto_waypoints, pick_waypoints
from modules.settings import PickGateSettings, Point, RobotSettings, Settings

# Period of the realtime perception thread (s). Half of it is part of the gate's sampling
# staleness, so it lives with the model that budgets for it.
PERCEPTION_PERIOD_S = 0.025

# Iteration budgets and tolerances of the intercept solver.
_EARLIEST_ITERATIONS = 6
_EARLIEST_TOL_S = 0.01
_PARK_ITERATIONS = 4
_PARK_TOL_S = 1e-3


def gate_sampling_latency_s(poll_interval_s: float, perception_period_s: float = PERCEPTION_PERIOD_S) -> float:
    """Average staleness of a part's u as the gate sees it: half the gate poll plus half the
    perception tick."""
    return poll_interval_s / 2.0 + perception_period_s / 2.0


class DeltaArm:
    def __init__(
        self,
        robot: RobotSettings,
        pick_gate: PickGateSettings,
        frame: ConveyorFrame,
        workspace_window_uv: UVWindow,
        sampling_latency_s: float,
    ) -> None:
        self.robot = robot
        self.gate = pick_gate
        self.frame = frame
        self.workspace_window_uv = workspace_window_uv
        self.sampling_latency_s = sampling_latency_s
        self.home: Point = robot.home_position
        self.command_delay_s = pick_gate.command_delay_s
        self.gate_lead_s = (self.command_delay_s + sampling_latency_s + self.gate_descent_lead_s())

    @classmethod
    def from_settings(cls, settings: Settings, frame: ConveyorFrame | None = None) -> "DeltaArm":
        return cls(
            settings.robot,
            settings.pick_gate,
            frame or ConveyorFrame.from_settings(settings.conveyor),
            settings.conveyor.workspace_window_uv,
            gate_sampling_latency_s(settings.runtime.poll_interval_s),
        )

    # ---- geometry and time ------------------------------------------------------------
    def trajectory_time(self, points: list[Point]) -> float:
        return trajectory_time(points, self.robot.interpolator)

    def goto_points(self, start: Point, park: Point) -> list[Point]:
        return goto_waypoints(start, park, self.robot)

    def pick_points(self, contact: Point, bin_position: Point) -> list[Point]:
        return pick_waypoints(contact, bin_position, self.robot)

    def pick_point(self, u: float, v: float) -> Point:
        x, y = self.frame.to_robot(u, v)
        return (x, y, self.robot.heights.pickup)

    def descent_time_s(self, belt_speed_mm_s: float) -> float:
        """Modelled park -> contact descent (pre_pick -> pickup), including the pick command's
        one-off soft start. An oblique descent slants along the belt by v·t_d while dropping
        |Δz|, so one fixed-point pass folds the slant into the segment length."""
        interp = self.robot.interpolator
        dz = abs(self.robot.heights.pre_pick - self.robot.heights.pickup)

        def seg(length: float) -> float:
            return interp.soft_start_s + segment_profile_time(
                length, 0.0, 0.0, interp.v_max, interp.a_max, interp.d_max,
                interp.scurve_shape_factor)

        t0 = seg(dz)
        slant = max(0.0, belt_speed_mm_s) * t0
        return seg(math.hypot(dz, slant))

    def vertical_descent_time_s(self) -> float:
        """t_d of the straight-down descent: the [GATE] log and the rotate fallback use it."""
        return self.descent_time_s(0.0)

    def gate_descent_lead_s(self) -> float:
        """Descent share of the gate lead beyond robot_movement_delay_s: the configured
        `pick_descent_time_s` for a vertical descent (0 by default: the descent is inside the
        measured delay). Oblique descent: the contact is already shifted downstream by v·t_d,
        so nothing is added."""
        if self.gate.oblique_descent_enabled:
            return 0.0
        return max(0.0, self.gate.pick_descent_time_s)

    def contact_position(self, pick_position: Point, belt_speed_mm_s: float) -> tuple[Point, float]:
        """Contact point of the pick phase and the modelled descent time t_d.

        The arm parks above `pick_position` (unchanged by the descent mode). With the oblique
        descent the contact is shifted DOWNSTREAM along +u by v·t_d so the short drop tracks
        the moving part; otherwise (or at belt speed 0) it is `pick_position` itself."""
        pickup = self.robot.heights.pickup
        if not self.gate.oblique_descent_enabled:
            return (pick_position[0], pick_position[1], pickup), self.vertical_descent_time_s()
        t_d = self.descent_time_s(belt_speed_mm_s)
        offset = max(0.0, belt_speed_mm_s) * t_d
        u_x, u_y = self.frame.u_hat
        return (pick_position[0] + u_x * offset, pick_position[1] + u_y * offset, pickup), t_d

    def first_unreachable(self, points: list[Point]) -> Point | None:
        """First waypoint the PLC IK would reject, or None. The deployed program reports a
        joint-limit trip as "no error" and commands unwritten joint outputs (open-issues O1),
        so an out-of-envelope waypoint must never be sent."""
        for point in points:
            if not ik_reachable(point[0], point[1], point[2]):
                return point
        return None

    # ---- ArmModel: the scheduler's belief ---------------------------------------------
    def predict(self, u_at_anchor: float, v: float, start: Point, start_time: float,
                forecast: BeltForecast, anchor_time: float) -> Intercept | None:
        elapsed = max(0.0, start_time - anchor_time)
        u_start = u_at_anchor + forecast.travel_in(elapsed)
        belt = forecast.at(elapsed)
        window = self.workspace_window_uv
        u_min, u_max, v_min, v_max = window
        command_delay_s = self.command_delay_s

        if v < v_min or v > v_max:
            return None

        def goto_time(target: Point) -> float:
            return self.trajectory_time(self.goto_points(start, target))

        # Stage (a): earliest reachable intercept, no lead.
        guess = start_time + max(0.0, command_delay_s)
        t_enter = start_time
        if u_start < u_min:
            wait = belt.time_to_travel(u_min - u_start)
            if math.isfinite(wait):
                t_enter = start_time + wait
                guess = max(guess, t_enter)
        soft_start_s = self.robot.interpolator.soft_start_s
        for _ in range(_EARLIEST_ITERATIONS):
            u_pick = u_start + belt.travel_in(max(0.0, guess - start_time))
            if u_pick > u_max:
                return None
            new_guess = start_time + command_delay_s + goto_time(self.pick_point(u_pick, v)) + soft_start_s
            new_guess = max(new_guess, t_enter)
            converged = abs(new_guess - guess) < _EARLIEST_TOL_S
            guess = new_guess
            if converged:
                break
        u_pick = u_start + belt.travel_in(max(0.0, guess - start_time))
        if not ConveyorFrame.is_in_window_uv(u_pick, v, window):
            return None

        # Stage (b): the arm must be parked a full gate lead before contact.
        final = max(guess, start_time + self.gate.intercept_lead_time_s)
        if u_start > u_max:
            return None
        gate_lead_s = self.gate_lead_s
        for _ in range(_PARK_ITERATIONS):
            u_pick = u_start + belt.travel_in(max(0.0, final - start_time))
            clamped = False
            if u_pick > u_max:
                to_edge = belt.time_to_travel(u_max - u_start)
                if not math.isfinite(to_edge):
                    return None
                u_pick = u_max
                final = start_time + max(0.0, to_edge)
                clamped = True
            if not ConveyorFrame.is_in_window_uv(u_pick, v, window):
                return None
            position = self.pick_point(u_pick, v)
            goto_s = goto_time(position)
            required = start_time + command_delay_s + goto_s + gate_lead_s
            if required <= final + _PARK_TOL_S:
                return Intercept(final, final - gate_lead_s, position, u_pick, goto_s)
            if clamped:
                return None
            final = required
        return None

    def costs(self, start: Point, start_time: float, intercept: Intercept, bin_position: Point,
              belt_speed_at_pick: float, setup_s: float) -> tuple[float, float]:
        """The descent is counted both in the gate lead and in the pick trajectory, so
        occupancy is slightly conservative."""
        contact, _ = self.contact_position(intercept.pick_position, belt_speed_at_pick)
        pick_points = self.pick_points(contact, bin_position)
        grab = intercept.pick_time - start_time
        occupancy = ((intercept.gate_fire_time - start_time) + self.command_delay_s
                     + self.trajectory_time(pick_points) + setup_s)
        return grab, occupancy

    def place_position(self, bin_position: Point) -> Point:
        return (bin_position[0], bin_position[1], self.robot.heights.place)

    # ---- ArmModel: plant ---------------------------------------------------------------
    def goto_time_s(self, start: Point, park: Point) -> float:
        return self.command_delay_s + self.trajectory_time(self.goto_points(start, park))

    def contact_delay_s(self, belt_speed: float) -> float:
        return self.command_delay_s + self.gate_descent_lead_s()

    def pick_time_s(self, park: Point, bin_position: Point, belt_speed: float) -> float:
        contact, _ = self.contact_position(park, belt_speed)
        return self.command_delay_s + self.trajectory_time(self.pick_points(contact, bin_position))
