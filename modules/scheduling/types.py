"""The values a scheduling plugin reads and returns. Plugins import from here and nowhere else
in the repository (see README.md)."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Callable

from modules.core.arm_model import ArmModel, Intercept
from modules.core.forecast import BeltForecast
from modules.settings import ArmCycleSettings, Point

__all__ = [
    "ArmCycleSettings", "BeltForecast", "Intercept", "Job", "ObjectSnapshot", "PlanContext",
    "Point", "ScheduledPick", "SpeedDecision", "SpeedView",
]


# ---------------------------------------------------------------------------
# Sequencing: which part next
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ObjectSnapshot:
    """A part on the belt, copied out of the live tracker at decision time."""

    object_id: str
    object_type: str
    u: float                        # belt coordinate at snapshot time (mm)
    v: float
    bin: Point                      # drop position of its class
    detected_at: float              # when the camera first saw it (release time r_j)


@dataclass
class Job:
    """One part as a scheduling job (doc/basis-theory.md §7.1).

    `processing`, `intercept` and `path_mm` are costed from the arm's current position;
    `deadline` is when the part leaves the workspace under the belt forecast."""

    obj: ObjectSnapshot
    now: float
    release: float                  # r_j — when the camera saw it
    deadline: float                 # d_j — when it passes u_max
    processing: float               # p_j from the current arm position (inf if unreachable)
    feasible: bool                  # can be picked at all from here
    intercept: Intercept | None
    reachable_at: float             # the r_j a planner uses: dispatchable now
    path_mm: float = math.inf       # arm -> contact -> bin path length
    safety_margin_s: float = 0.0
    arrival: float = 0.0            # a_j — when it enters the workspace (now if already inside)
    # Parts lying on this one's pick point. The vision has no stacking information, so this
    # is always empty; kept so stacking-aware planners port unchanged.
    blockers: frozenset[str] = frozenset()

    @property
    def deadline_safe(self) -> float:
        return self.deadline - self.safety_margin_s

    @property
    def slack(self) -> float:
        """Time to spare if the part were picked first: deadline minus predicted contact."""
        if self.intercept is None:
            return -math.inf
        return self.deadline_safe - self.intercept.pick_time


@dataclass
class ScheduledPick:
    job: Job
    start_s: float                  # when the arm is dispatched for it
    grab_s: float                   # predicted cup contact
    finish_s: float                 # when the arm is free again
    processing: float               # p_j against the retained predecessor
    intercept: Intercept | None = None

    @property
    def late(self) -> bool:
        return self.grab_s > self.job.deadline_safe


@dataclass(frozen=True)
class PlanContext:
    """What a planner may ask besides the jobs themselves."""

    now: float
    arm_position: Point
    forecast: BeltForecast          # belt at `now`
    arm: ArmModel
    workspace_window_uv: tuple[float, float, float, float]
    safety_margin_s: float = 0.0
    setup_s: float = 0.0

    def predict(self, obj: ObjectSnapshot, position: Point, start: float) -> Intercept | None:
        """Intercept of `obj` if the arm leaves `position` at `start`."""
        return self.arm.predict(obj.u, obj.v, position, start, self.forecast, self.now)

    def costs(self, obj: ObjectSnapshot, position: Point, start: float,
              intercept: Intercept) -> tuple[float, float]:
        """(grab_s, occupancy_s) of that pick."""
        speed = self.forecast.speed_at(max(0.0, intercept.pick_time - self.now))
        return self.arm.costs(position, start, intercept, obj.bin, speed, self.setup_s)

    def next_position(self, obj: ObjectSnapshot) -> Point:
        """Where the arm is after placing `obj`."""
        return self.arm.place_position(obj.bin)


# ---------------------------------------------------------------------------
# Belt speed
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class SpeedView:
    """Read-only snapshot a speed law decides from."""

    now: float
    setpoint_mm_s: float            # last commanded belt speed
    measured_mm_s: float            # PLC feedback
    band: tuple[float, float]       # (v_min, v_max) every commit is clamped to
    max_step_mm_s: float            # largest change one commit may make (0 = none)
    static_mm_s: float              # speed.static_mm_s
    workspace_window_uv: tuple[float, float, float, float]
    objects_u: tuple[float, ...]    # u of every unclaimed part with 0 <= u <= u_max
    arm_cycle: ArmCycleSettings
    event: str                      # what opened the gate: arm_free | idle | contact | busy
    detection_times: tuple[float, ...] = ()   # first sighting of each recent part (same clock)
    # Model-based laws: the forecast of a candidate setpoint, and the schedule a planner
    # would make under a forecast (planner=None: the executing planner).
    forecast_for: Callable[[float], BeltForecast] = field(repr=False, default=None)
    schedule_for: Callable[..., list[ScheduledPick]] = field(repr=False, default=None)

    @property
    def n_in_window(self) -> int:
        """Parts between the camera origin and the downstream workspace edge."""
        return len(self.objects_u)

    def within_step(self, v: float) -> bool:
        """True when one commit can reach `v` from the setpoint."""
        return self.max_step_mm_s <= 0.0 or abs(v - self.setpoint_mm_s) <= self.max_step_mm_s + 1e-9


@dataclass(frozen=True)
class SpeedDecision:
    target_mm_s: float
    info: dict[str, Any] = field(default_factory=dict)      # logged with the decision
    # The schedule the law scored its winner with, and which planner made it. The loop
    # executes it instead of re-planning when that planner is the executing one.
    schedule: list[ScheduledPick] | None = None
    schedule_planner: str | None = None
