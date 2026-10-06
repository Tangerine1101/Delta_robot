"""One simulated run: belt, feeder, a model arm the scheduler plans with and a plant arm that
executes, driven through the same decision cycle as the robot (modules/runtime/loop.py).

    arm free  -> speed law at its gate -> commit -> planner -> first valid entry -> pick
    pick      -> goto (plant time) -> gate fires at u_pick - v*lead (model lead)
              -> contact after the plant's delay: gripped if the part is within tolerance
              -> "contact" decision point -> carry to the bin -> arm free again

The setpoint is frozen from goto dispatch to cup contact, exactly as on the robot (L9). A
difference between model and plant (delays, motion limits) shows up as contact error and
missed grips — the cost of a wrong prediction.
"""

from __future__ import annotations

import math
import random
import statistics
import time
from dataclasses import dataclass, field
from typing import Any

from modules.core.arm_model import ArmModel, Intercept
from modules.core.forecast import BeltForecast, live_forecast, steady
from modules.scheduling.commit import CommitPolicy, commit_step
from modules.scheduling.jobs import build_jobs
from modules.scheduling.registry import BoundPlanner, get_planner, get_speed_law
from modules.scheduling.types import ObjectSnapshot, PlanContext, ScheduledPick, SpeedDecision, SpeedView
from modules.settings import Point, Settings, with_overrides
from sandbox.config import ModelFile, RunConfig, cell_settings, plant_of
from modules.core.feeders import SpawnEvent, build_events
from sandbox.models import build_arm

PICKED, MISSED, LOST, ON_BELT = "picked", "missed", "lost", "on_belt"


@dataclass
class Part:
    pid: str
    part_type: str
    v: float
    belt_at_spawn: float            # belt position when it landed at u = 0
    spawned_at: float
    state: str = ON_BELT
    claimed: bool = False
    attempted: bool = False
    error_mm: float | None = None

    def u(self, belt_position: float) -> float:
        return belt_position - self.belt_at_spawn


@dataclass
class Belt:
    speed: float
    setpoint: float
    accel: float
    position: float = 0.0

    def step(self, dt: float) -> None:
        v0 = self.speed
        dv = self.setpoint - v0
        limit = self.accel * dt if self.accel > 0.0 else abs(dv)
        self.speed = v0 + max(-limit, min(limit, dv))
        self.position += 0.5 * (v0 + self.speed) * dt


@dataclass
class Pick:
    part: Part
    intercept: Intercept
    contact_u: float                # belt coordinate the cup comes down on (model)
    bin: Point
    phase: str                      # goto | gate | descend | carry
    arrive_t: float
    gate_start_t: float = 0.0
    contact_t: float = 0.0
    free_t: float = 0.0


@dataclass
class Metrics:
    spawned: int = 0
    picked: int = 0
    missed: int = 0                 # attempted, cup off the part
    lost: int = 0                   # left the workspace never attempted
    aborted: int = 0                # gate late / stalled: re-queued, not attempted
    remaining: int = 0              # still on the belt at the end
    commits: int = 0
    decisions: int = 0
    plans: int = 0
    errors_mm: list[float] = field(default_factory=list)
    speed_integral: float = 0.0
    duration_s: float = 0.0
    wall_s: float = 0.0

    def summary(self) -> dict[str, Any]:
        finished = self.picked + self.missed + self.lost
        errors = [abs(e) for e in self.errors_mm]
        return {
            "spawned": self.spawned, "picked": self.picked, "missed": self.missed, "lost": self.lost,
            "aborted": self.aborted, "remaining": self.remaining,
            "pick_rate": round(self.picked / finished, 4) if finished else None,
            "picks_per_min": round(60.0 * self.picked / self.duration_s, 2) if self.duration_s else None,
            "mean_belt_mm_s": round(self.speed_integral / self.duration_s, 2) if self.duration_s else None,
            "median_abs_error_mm": round(statistics.median(errors), 2) if errors else None,
            "commits": self.commits, "decisions": self.decisions, "plans": self.plans,
            "wall_s": round(self.wall_s, 2),
        }


def _within_step(max_step: float, setpoint: float, v: float) -> bool:
    return max_step <= 0.0 or abs(v - setpoint) <= max_step + 1e-9


class Simulation:
    def __init__(self, run: RunConfig, model_file: ModelFile, *, verbose: bool = False) -> None:
        self.run = run
        self.verbose = verbose
        self.settings: Settings = cell_settings(model_file, run)
        cell_over, params_over = plant_of(model_file, run)
        self.model: ArmModel = build_arm(model_file.builder, self.settings, model_file.params, "params")
        plant_settings = with_overrides(self.settings, cell_over) if cell_over else self.settings
        self.plant: ArmModel = build_arm(model_file.builder, plant_settings,
                                         {**model_file.params, **params_over}, "plant")
        s = self.settings
        self.window = s.conveyor.workspace_window_uv
        self.planner: BoundPlanner = get_planner(s.scheduling.planner, s.scheduling.planners)
        self.law = get_speed_law(s.speed.law, s.speed.laws, s.speed.setpoint_gate)
        self.policy = CommitPolicy((s.speed.band.min_mm_s, s.speed_ceiling_mm_s()),
                                   s.speed.commit.deadband_mm_s, s.speed.commit.max_step_mm_s)
        self._planners: dict[str | None, BoundPlanner] = {None: self.planner, self.planner.name: self.planner}
        rng = random.Random(run.sim.seed)
        feeder = dict(run.feeder)
        kind = str(feeder.pop("kind", "poisson"))
        # Only the selected feeder's section is read, so an experiment can switch `kind`
        # without deleting the other sections.
        self.events: list[SpawnEvent] = build_events(kind, feeder.get(kind) or {}, rng, run.sim.duration_s)
        self.metrics = Metrics(spawned=len(self.events))

    # ---- speed control ----------------------------------------------------------------
    def _view(self, now: float, event: str, parts: list[Part]) -> SpeedView:
        s = self.settings
        belt = self.belt
        u_max = self.window[1]
        objects_u = tuple(u for u in (p.u(belt.position) for p in parts if not p.claimed and not p.attempted)
                          if 0.0 <= u <= u_max)
        snapshot = self._snapshot(parts)

        def schedule_for(forecast: BeltForecast, planner: str | None = None) -> list[ScheduledPick]:
            if planner not in self._planners:
                self._planners[planner] = get_planner(planner, s.scheduling.planners)
            return self._schedule(snapshot, forecast, self._planners[planner], now)

        return SpeedView(
            now=now, setpoint_mm_s=belt.setpoint, measured_mm_s=belt.speed, band=self.policy.band,
            max_step_mm_s=s.speed.commit.max_step_mm_s, static_mm_s=s.speed.static_mm_s,
            workspace_window_uv=self.window, objects_u=objects_u, arm_cycle=s.scheduling.arm_cycle,
            event=event, detection_times=tuple(p.spawned_at for p in parts),
            forecast_for=lambda v: BeltForecast(belt.speed, v, s.conveyor.accel_mm_s2),
            schedule_for=schedule_for,
        )

    def _speed_tick(self, now: float, event: str, parts: list[Part]) -> SpeedDecision | None:
        if not self.law.adaptive or self.frozen:
            return None
        if not self.law.gate_open(event, now - self.last_decision_t, self.settings.speed.control_period_s):
            return None
        decision = self.law(self._view(now, event, parts))
        self.last_decision_t = now
        self.metrics.decisions += 1
        step = commit_step(decision.target_mm_s, self.belt.setpoint, self.belt.speed, now - self.last_commit_t,
                           self.policy, use_deadband=self.law.spec.deadband)
        if step.send is not None:
            self.belt.setpoint = step.send
            self.last_commit_t = now
            self.metrics.commits += 1
        return decision

    # ---- planning -----------------------------------------------------------------------
    def _snapshot(self, parts: list[Part]) -> tuple[ObjectSnapshot, ...]:
        belt = self.belt
        return tuple(ObjectSnapshot(p.pid, p.part_type, p.u(belt.position), p.v,
                                    self.settings.object_types[p.part_type].bin, p.spawned_at)
                     for p in parts if not p.claimed and not p.attempted)

    def _context(self, now: float, forecast: BeltForecast) -> PlanContext:
        s = self.settings.scheduling
        return PlanContext(now, self.arm_position, forecast, self.model, self.window,
                           s.safety_margin_s, s.setup_time_s)

    def _schedule(self, snapshot, forecast, planner: BoundPlanner, now: float) -> list[ScheduledPick]:
        ctx = self._context(now, forecast)
        return planner(build_jobs(snapshot, ctx), ctx)

    def _forecast(self) -> BeltForecast:
        if not self.law.adaptive:
            return steady(self.belt.speed)
        s = self.settings
        return live_forecast(self.belt.speed, self.belt.setpoint, s.conveyor.accel_mm_s2,
                             s.speed.commit.deadband_mm_s)

    def _plan(self, now: float, parts: list[Part], reused: list[ScheduledPick] | None) -> Pick | None:
        forecast = self._forecast()
        schedule = reused if reused is not None else self._schedule(self._snapshot(parts), forecast,
                                                                     self.planner, now)
        by_id = {p.pid: p for p in parts}
        u_max = self.window[1]
        margin = self.settings.scheduling.safety_margin_s
        for entry in schedule:
            part = by_id.get(entry.job.obj.object_id)
            if part is None or part.claimed or part.attempted or part.state != ON_BELT:
                continue
            u_now = part.u(self.belt.position)
            if u_now > u_max:
                continue
            intercept = self.model.predict(u_now, part.v, self.arm_position, now, forecast, now)
            if intercept is None:
                continue
            if intercept.pick_time > now + forecast.time_to_travel(u_max - u_now) - margin:
                continue
            bin_position = self.settings.object_types[part.part_type].bin
            contact_u = intercept.u_pick
            contact_of = getattr(self.model, "contact_position", None)
            if contact_of is not None:
                contact, _ = contact_of(intercept.pick_position, self.belt.speed)
                contact_u = self.model.frame.to_conveyor(contact[0], contact[1])[0]
            part.claimed = True
            self.metrics.plans += 1
            return Pick(part, intercept, contact_u, bin_position, "goto",
                        now + self.plant.goto_time_s(self.arm_position, intercept.pick_position))
        return None

    # ---- the clock ------------------------------------------------------------------------
    def run_sim(self) -> Metrics:
        started = time.perf_counter()
        s = self.settings
        sim = self.run.sim
        self.belt = Belt(s.speed.static_mm_s, s.speed.static_mm_s, s.conveyor.accel_mm_s2)
        self.arm_position: Point = self.model.home
        self.frozen = False
        self.last_decision_t = -math.inf
        self.last_commit_t = 0.0
        parts: list[Part] = []
        pick: Pick | None = None
        next_event = 0
        next_plan_t = 0.0
        arm_was_free = False
        u_max = self.window[1]
        late_abort_mm = s.pick_gate.late_abort_mm
        dt = sim.dt_s
        steps = int(round(sim.duration_s / dt))

        for i in range(steps + 1):
            now = i * dt
            while next_event < len(self.events) and self.events[next_event].time_s <= now:
                e = self.events[next_event]
                parts.append(Part(f"p{next_event:05d}", e.part_type, e.v_mm, self.belt.position, now))
                next_event += 1

            if pick is not None:
                part = pick.part
                if pick.phase == "goto" and now >= pick.arrive_t:
                    pick.phase, pick.gate_start_t = "gate", now
                if pick.phase == "gate":
                    u_now = part.u(self.belt.position)
                    threshold = pick.intercept.u_pick - self.belt.speed * self.model.gate_lead_s
                    if u_now >= threshold:
                        if late_abort_mm > 0.0 and u_now - threshold > late_abort_mm:
                            pick = self._abort(pick)
                        else:
                            pick.phase = "descend"
                            pick.contact_t = now + self.plant.contact_delay_s(self.belt.speed)
                            pick.free_t = now + self.plant.pick_time_s(pick.intercept.pick_position,
                                                                        pick.bin, self.belt.speed)
                            part.attempted = True
                    elif now - pick.gate_start_t > sim.stall_timeout_s:
                        pick = self._abort(pick)
                if pick is not None and pick.phase == "descend" and now >= pick.contact_t:
                    error = part.u(self.belt.position) - pick.contact_u
                    part.error_mm = error
                    self.metrics.errors_mm.append(error)
                    # Picked or not, the part is done: suction is never verified, so an
                    # attempted grip is not retried (exactly-once), as on the robot.
                    part.state = PICKED if abs(error) <= sim.capture_tolerance_mm else MISSED
                    self._count(part)
                    parts.remove(part)
                    self.frozen = False
                    pick.phase = "carry"
                    self._speed_tick(now, "contact", parts)
                elif pick is not None and pick.phase == "carry":
                    if now >= pick.free_t:
                        self.arm_position = self.model.place_position(pick.bin)
                        pick = None
                        arm_was_free = False
                    elif now >= next_plan_t:
                        self._speed_tick(now, "busy", parts)
                        next_plan_t = now + sim.replan_period_s

            if pick is None and now >= next_plan_t:
                decision = self._speed_tick(now, "idle" if arm_was_free else "arm_free", parts)
                arm_was_free = True
                reused = None
                if decision is not None and decision.schedule is not None and \
                        decision.schedule_planner in (None, self.planner.name):
                    reused = decision.schedule
                pick = self._plan(now, parts, reused)
                if pick is not None:
                    self.frozen = True
                next_plan_t = now + sim.replan_period_s

            self.belt.step(dt)
            self.metrics.speed_integral += self.belt.speed * dt
            off_belt = [p for p in parts if not p.claimed and p.u(self.belt.position) > u_max + sim.lost_margin_mm]
            for p in off_belt:
                p.state = LOST
                self._count(p)
                parts.remove(p)

        self.metrics.remaining = len(parts)
        self.metrics.duration_s = sim.duration_s
        self.metrics.wall_s = time.perf_counter() - started
        return self.metrics

    def _abort(self, pick: Pick) -> None:
        """A late or stalled gate: the part stays on the belt and is planned again, the arm
        stays parked where it is."""
        pick.part.claimed = False
        self.arm_position = pick.intercept.pick_position
        self.frozen = False
        self.metrics.aborted += 1
        return None

    def _count(self, part: Part) -> None:
        if part.state == PICKED:
            self.metrics.picked += 1
        elif part.state == MISSED:
            self.metrics.missed += 1
        elif part.state == LOST:
            self.metrics.lost += 1
