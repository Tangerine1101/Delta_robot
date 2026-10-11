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
from modules.core.footprint import Footprint, board_footprint
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
    heading_deg: float = 0.0
    state: str = ON_BELT
    claimed: bool = False
    attempted: bool = False
    error_mm: float | None = None
    covered_by: set[str] = field(default_factory=set)   # parts that landed on this one
    reason: str = ""
    seen_u: float = 0.0
    seen_v: float = 0.0
    dispatched_at: float | None = None
    contact_at: float | None = None
    closed_at: float | None = None
    bypassed: bool = False          # an arm commit with a long wait chose another part over it

    def footprint(self, belt_position: float, size: tuple[float, float]) -> Footprint:
        return board_footprint(self.u(belt_position), self.v, size[0], size[1], self.heading_deg)

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


def _timing_stats(series: list[float]) -> dict[str, float | None]:
    """Mean, 95th percentile and maximum of a list of wall times (ms); None when it is empty."""
    if not series:
        return {"mean": None, "p95": None, "max": None, "n": 0}
    ordered = sorted(series)
    return {"mean": round(statistics.fmean(ordered), 3), "p95": round(ordered[int(0.95 * (len(ordered) - 1))], 3),
            "max": round(ordered[-1], 3), "n": len(ordered)}


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
    covered_misses: int = 0         # contact on a part another part still lay on
    long_waits: int = 0             # commits whose contact was > sim.long_wait_s away ...
    long_wait_s: float = 0.0        # ... and the total time the arm spent waiting in them
    bypassed: int = 0               # ... while another part could have been picked sooner
    bypassed_lost: int = 0          # parts so passed over that left the workspace unpicked
    idle_s: float = 0.0             # arm free with nothing committed
    dips: int = 0                   # stretches where the 60 s throughput left the input (sandbox/flow.py)
    dip_s: float = 0.0              # their total length
    errors_mm: list[float] = field(default_factory=list)
    speed_integral: float = 0.0
    duration_s: float = 0.0
    wall_s: float = 0.0
    law_ms: list[float] = field(default_factory=list)    # wall time of each speed-law decision (ms)
    plan_jobs: list[int] = field(default_factory=list)   # parts in the queue at each of those plannings
    plan_ms: list[float] = field(default_factory=list)   # ... and of each commit planning, the law's own planner calls excluded (ms)

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
            "covered_misses": self.covered_misses, "long_waits": self.long_waits,
            "long_wait_s": round(self.long_wait_s, 1), "bypassed": self.bypassed,
            "bypassed_lost": self.bypassed_lost, "idle_s": round(self.idle_s, 1),
            "dips": self.dips, "dip_s": round(self.dip_s, 1),
            "commits": self.commits, "decisions": self.decisions, "plans": self.plans,
            "wall_s": round(self.wall_s, 2),
            "plan_jobs_mean": round(statistics.fmean(self.plan_jobs), 2) if self.plan_jobs else None,
            "plan_jobs_max": max(self.plan_jobs, default=None),
            **{f"{name}_ms_{stat}": value for name, series in (("law", self.law_ms), ("plan", self.plan_ms))
               for stat, value in _timing_stats(series).items()},
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
        max_parts = int(feeder.get("max_parts") or 0)
        if max_parts > 0:            # as the robot's `feeder.max_parts`: the first parts only
            self.events = self.events[:max_parts]
        self.metrics = Metrics(spawned=len(self.events))
        self.sizes = {name: (spec.w, spec.h) for name, spec in self.settings.object_types.items()}
        self.by_id: dict[str, Part] = {}
        self.wait_log: list[dict[str, Any]] = []
        self.belt_log: list[tuple[float, float]] = []   # (t, belt speed) once a second

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
        started = time.perf_counter()
        decision = self.law(self._view(now, event, parts))
        self.metrics.law_ms.append(1000.0 * (time.perf_counter() - started))
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
                                    self.settings.object_types[p.part_type].bin, p.spawned_at,
                                    frozenset(p.covered_by))
                     for p in parts if not p.claimed and not p.attempted)

    def _lying_on(self, part: Part) -> bool:
        """Is any part that landed on this one still on the belt? Mirrors the part record
        (runtime/outcomes.py): only a *picked* part is gone; one that was missed or left the
        workspace still lies there."""
        return any(self.by_id[pid].state != PICKED for pid in part.covered_by)

    def _cover_below(self, part: Part) -> None:
        """A part landing now lies on every part in the camera view it overlaps (the later drop
        is the upper one) — `VirtualFeed._cover_below` of the robot."""
        shape = part.footprint(self.belt.position, self.sizes[part.part_type])
        u_cam = self.camera_u_max
        for below in self.by_id.values():
            if below is part or below.state != ON_BELT or below.u(self.belt.position) > u_cam:
                continue
            if shape.overlaps(below.footprint(self.belt.position, self.sizes[below.part_type])):
                below.covered_by.add(part.pid)

    def _context(self, now: float, forecast: BeltForecast, snapshot=()) -> PlanContext:
        s = self.settings.scheduling
        on_belt = frozenset()
        if s.stack_aware:
            on_belt = frozenset(pid for o in snapshot for pid in o.covered_by
                                if self.by_id[pid].state != PICKED)
        return PlanContext(now, self.arm_position, forecast, self.model, self.window,
                           s.safety_margin_s, s.setup_time_s, s.stack_aware, on_belt)

    def _schedule(self, snapshot, forecast, planner: BoundPlanner, now: float) -> list[ScheduledPick]:
        ctx = self._context(now, forecast, snapshot)
        return planner(build_jobs(snapshot, ctx), ctx)

    def _forecast(self) -> BeltForecast:
        if not self.law.adaptive:
            return steady(self.belt.speed)
        s = self.settings
        return live_forecast(self.belt.speed, self.belt.setpoint, s.conveyor.accel_mm_s2,
                             s.speed.commit.deadband_mm_s)

    def _plan(self, now: float, parts: list[Part], reused: list[ScheduledPick] | None) -> Pick | None:
        forecast = self._forecast()
        if reused is not None:
            schedule = reused
        else:
            snapshot = self._snapshot(parts)
            started = time.perf_counter()
            schedule = self._schedule(snapshot, forecast, self.planner, now)
            if snapshot:             # an empty queue costs nothing and would dilute the statistics
                self.metrics.plan_ms.append(1000.0 * (time.perf_counter() - started))
                self.metrics.plan_jobs.append(len(snapshot))
        by_id = {p.pid: p for p in parts}
        u_max = self.window[1]
        margin = self.settings.scheduling.safety_margin_s
        for entry in schedule:
            part = by_id.get(entry.job.obj.object_id)
            if part is None or part.claimed or part.attempted or part.state != ON_BELT:
                continue
            if self.settings.scheduling.stack_aware and self._lying_on(part):
                continue            # the part on top has not been picked (yet)
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
            part.dispatched_at = now
            self._note_wait(now, part, intercept, parts, forecast)
            return Pick(part, intercept, contact_u, bin_position, "goto",
                        now + self._leg_s(self.plant.goto_time_s(self.arm_position, intercept.pick_position),
                                          self.run.sim.goto_extra_s))
        return None

    def _leg_s(self, plant_s: float, extra_s: float | None) -> float:
        """A plant leg time with its fixed overhead replaced: the plant adds the model's command
        delay to the trajectory time; `extra_s` (when set) is added instead."""
        if extra_s is None:
            return plant_s
        return plant_s - getattr(self.plant, "command_delay_s", 0.0) + extra_s

    def _note_wait(self, now: float, part: Part, intercept: Intercept, parts: list[Part],
                   forecast: BeltForecast) -> None:
        """Book a commit that makes the arm wait long for its part, and mark every other part
        the arm could have picked sooner from where it stands (the starvation signature)."""
        wait = intercept.pick_time - now
        if wait <= self.run.sim.long_wait_s:
            return
        metrics = self.metrics
        metrics.long_waits += 1
        metrics.long_wait_s += wait
        u_max = self.window[1]
        margin = self.settings.scheduling.safety_margin_s
        stack_aware = self.settings.scheduling.stack_aware
        sooner = []
        for other in parts:
            if other is part or other.claimed or other.attempted or other.state != ON_BELT:
                continue
            if stack_aware and self._lying_on(other):
                continue
            u_now = other.u(self.belt.position)
            if u_now > u_max:
                continue
            alt = self.model.predict(u_now, other.v, self.arm_position, now, forecast, now)
            if alt is None or alt.pick_time > now + forecast.time_to_travel(u_max - u_now) - margin:
                continue
            if alt.pick_time < intercept.pick_time - 0.5:
                sooner.append(other.pid)
                if not other.bypassed:
                    other.bypassed = True
                    metrics.bypassed += 1
        self.wait_log.append({"t": round(now, 2), "part": part.pid, "wait_s": round(wait, 2),
                              "u": round(part.u(self.belt.position), 1), "sooner": sooner})

    # ---- the clock ------------------------------------------------------------------------
    def run_sim(self) -> Metrics:
        started = time.perf_counter()
        s = self.settings
        sim = self.run.sim
        self.belt = Belt(s.speed.static_mm_s, s.speed.static_mm_s, s.conveyor.accel_mm_s2)
        self.camera_u_max = s.conveyor.camera_window_uv[1]
        stacking = sim.stacking
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
        per_second = max(1, int(round(1.0 / dt)))

        for i in range(steps + 1):
            now = i * dt
            while next_event < len(self.events) and self.events[next_event].time_s <= now:
                e = self.events[next_event]
                part = Part(f"p{next_event:05d}", e.part_type, e.v_mm, self.belt.position, now,
                            heading_deg=e.heading_deg, seen_u=0.0, seen_v=e.v_mm)
                if stacking:
                    self._cover_below(part)
                parts.append(part)
                self.by_id[part.pid] = part
                next_event += 1

            if pick is not None:
                part = pick.part
                if pick.phase == "goto" and now >= pick.arrive_t:
                    pick.phase, pick.gate_start_t = "gate", now
                if pick.phase == "gate" and sim.gate_cover_abort and stacking and self._lying_on(part):
                    pick = self._abort(pick)        # a part landed on the target while the arm waited
                if pick is not None and pick.phase == "gate":
                    u_now = part.u(self.belt.position)
                    threshold = pick.intercept.u_pick - self.belt.speed * self.model.gate_lead_s
                    if u_now >= threshold:
                        if late_abort_mm > 0.0 and u_now - threshold > late_abort_mm:
                            pick = self._abort(pick)
                        else:
                            pick.phase = "descend"
                            pick.contact_t = now + self.plant.contact_delay_s(self.belt.speed)
                            pick.free_t = now + sim.carry_scale * self._leg_s(self.plant.pick_time_s(
                                pick.intercept.pick_position, pick.bin, self.belt.speed), sim.pick_extra_s)
                            part.attempted = True
                    elif now - pick.gate_start_t > sim.stall_timeout_s:
                        pick = self._abort(pick)
                if pick is not None and pick.phase == "descend" and now >= pick.contact_t:
                    error = part.u(self.belt.position) - pick.contact_u
                    part.error_mm = error
                    self.metrics.errors_mm.append(error)
                    part.contact_at = now
                    # Picked or not, the part is done: suction is never verified, so an
                    # attempted grip is not retried (exactly-once), as on the robot.
                    if stacking and self._lying_on(part):
                        part.state, part.reason = MISSED, "covered"
                        self.metrics.covered_misses += 1
                    elif abs(error) <= sim.capture_tolerance_mm:
                        part.state, part.reason = PICKED, "gripped"
                    else:
                        part.state, part.reason = MISSED, "cup_off_part"
                    part.closed_at = now
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

            if pick is None:
                self.metrics.idle_s += dt
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
            if i % per_second == 0:
                self.belt_log.append((round(now, 3), round(self.belt.speed, 2)))
            off_belt = [p for p in parts if not p.claimed and p.u(self.belt.position) > u_max + sim.lost_margin_mm]
            for p in off_belt:
                p.state, p.reason, p.closed_at = LOST, "never_planned", now
                if p.bypassed:
                    self.metrics.bypassed_lost += 1
                self._count(p)
                parts.remove(p)

        self.metrics.remaining = len(parts)
        for p in parts:
            p.state, p.reason = ON_BELT, "run_ended"
        self.metrics.duration_s = sim.duration_s
        from sandbox.flow import find_dips

        dips = find_dips([{k: str(v) for k, v in row.items()} for row in self.records()])
        self.metrics.dips, self.metrics.dip_s = len(dips), sum(d.length_s for d in dips)
        self.metrics.wall_s = time.perf_counter() - started
        return self.metrics

    # ---- records ------------------------------------------------------------------------
    def records(self) -> list[dict[str, Any]]:
        """One row per part, in the columns of the robot's `parts.csv` that matter for a flow
        analysis (so `python3 -m sandbox dips` reads both)."""
        rows = []
        for p in sorted(self.by_id.values(), key=lambda q: q.spawned_at):
            outcome = {PICKED: "picked", MISSED: "miss_grip", LOST: "miss_late", ON_BELT: "on_belt"}[p.state]
            rows.append({"part_id": p.pid, "part_type": p.part_type, "t_seen": round(p.spawned_at, 3),
                         "u_seen": p.seen_u, "v_seen": round(p.seen_v, 2),
                         "t_dispatch": "" if p.dispatched_at is None else round(p.dispatched_at, 3),
                         "t_contact": "" if p.contact_at is None else round(p.contact_at, 3),
                         "outcome": outcome, "reason": p.reason,
                         "t_closed": "" if p.closed_at is None else round(p.closed_at, 3),
                         "covered_by": ";".join(sorted(p.covered_by)), "bypassed": int(p.bypassed)})
        return rows

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
