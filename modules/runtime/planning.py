"""From the live tracker to a committed PickPlan, through the configured planner.

Thread contract: snapshot under `state_lock` -> plan without any lock -> re-validate the chosen
entry against the live tracker under `state_lock`. A solve can take long enough for parts to
move, so nothing computed without the lock is dispatched unchecked (basis-programming §2.2).
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from typing import Callable

from modules.core.delta import DeltaArm
from modules.core.forecast import BeltForecast, live_forecast, steady
from modules.core.frames import ConveyorFrame
from modules.core.tracking import BeltTracker, ObjectDetection
from modules.runtime.outcomes import PartLedger
from modules.runtime.pick_gate import find_tracked_object
from modules.runtime.plan import PickPlan, build_pick_plan
from modules.runtime.state import RealtimeState, SchedulerMetrics
from modules.scheduling.jobs import build_jobs
from modules.scheduling.registry import BoundPlanner
from modules.scheduling.types import ObjectSnapshot, PlanContext, ScheduledPick
from modules.settings import Point, Settings

# Decisions slower than this are reported: the arm idles while they run.
SLOW_DECISION_WARN_S = 0.5
# First sightings are kept this long for load-scheduled speed laws (rate_schedule).
DETECTION_HISTORY_S = 120.0


@dataclass(frozen=True)
class PlanningSnapshot:
    taken_at: float                 # monotonic time the snapshot represents
    belt_position_mm: float
    measured_speed_mm_s: float
    setpoint_mm_s: float
    arm_position: Point
    objects: tuple[ObjectSnapshot, ...]     # unclaimed parts of a known class


class PickPlanner:
    """Owns the tracker's bookkeeping (what was seen, what was attempted, the run's part
    record) and turns the planner's schedule into the next PickPlan."""

    def __init__(self, settings: Settings, arm: DeltaArm, frame: ConveyorFrame,
                 tracker: BeltTracker, ledger: PartLedger | None = None) -> None:
        self.settings = settings
        self.arm = arm
        self.frame = frame
        self.tracker = tracker
        self.interpolar_points = settings.plc.interpolar_points
        self.object_dimensions = {name: (spec.w, spec.h) for name, spec in settings.object_types.items()}
        # First sighting of each part, for load-scheduled speed laws.
        self.first_seen: dict[str, float] = {}
        # Parts already attempted: vision re-emits the same id every frame while a part is
        # visible, so without this guard an attempted part would be re-created and re-picked.
        self.planned_object_ids: dict[str, float] = {}
        self.metrics = SchedulerMetrics()
        self.ledger = ledger or PartLedger(time.monotonic(),
                                           grip_tolerance_mm=settings.runtime.grip_tolerance_mm)
        self.current_position: Point = settings.robot.home_position
        self.plan_counter = 0

    # ---- perception side (called under state_lock) ------------------------------------
    def ingest_detections(
        self,
        detections: list[ObjectDetection],
        p_now: float,
        position_at: Callable[[float], float | None] | None = None,
    ) -> None:
        for detection in detections:
            if detection.object_id in self.planned_object_ids:
                continue
            self.metrics.total_detections += 1
            self.ledger.seen(detection.object_id, detection.object_type, detection.timestamp,
                             detection.x, detection.y)
            # Camera-latency compensation: anchor the part to the belt position AT the
            # frame's capture time, not at ingest; fall back to p_now without history.
            p_anchor = p_now
            if position_at is not None:
                past_p = position_at(detection.timestamp)
                if past_p is not None:
                    p_anchor = past_p
            self.tracker.ingest_detection(detection, p_anchor, object_dimensions=self.object_dimensions)
            self.first_seen.setdefault(detection.object_id, detection.timestamp)
        self.metrics.queue_peak = max(self.metrics.queue_peak, len(list(self.tracker.objects())))

    def prune(self, claimed_object_ids: set[str], p_now: float, now: float) -> None:
        """Drop parts that left the tracked belt span (BeltTracker.should_prune) and forget
        ids older than the stale timeout (track ids are monotonic, never reused sooner)."""
        for obj in list(self.tracker.objects()):
            if obj.object_id in claimed_object_ids:
                continue
            if self.tracker.should_prune(obj, p_now, now):
                u_now, _ = obj.current_uv(p_now)
                self.tracker.remove(obj.object_id)
                self.metrics.stale_drops += 1
                self.ledger.left(obj.object_id, now,
                                 past_workspace=u_now > self.settings.conveyor.workspace_window_uv[1])
        limit = now - self.settings.runtime.stale_timeout_s
        self.planned_object_ids = {k: t for k, t in self.planned_object_ids.items() if t >= limit}
        history = now - DETECTION_HISTORY_S
        self.first_seen = {k: t for k, t in self.first_seen.items() if t >= history}

    # ---- decision side -----------------------------------------------------------------
    def forecast(self, measured_mm_s: float, setpoint_mm_s: float, adaptive: bool) -> BeltForecast:
        """The belt the planner plans against: steady at the measured speed under a constant
        law, otherwise a ramp toward the setpoint unless it has already settled."""
        if not adaptive:
            return steady(measured_mm_s)
        return live_forecast(measured_mm_s, setpoint_mm_s, self.settings.conveyor.accel_mm_s2,
                             self.settings.speed.commit.deadband_mm_s)

    def snapshot(self, state: RealtimeState, now: float, setpoint_mm_s: float) -> PlanningSnapshot | None:
        """Value copy of everything a planner reads, or None when the belt sample is missing
        or stale."""
        with state.state_lock:
            sample = state.latest_speed
            if sample is None or now - sample.timestamp > self.settings.runtime.speed_timeout_s:
                return None
            objects = []
            for obj in self.tracker.objects():
                if obj.object_id in state.claimed_object_ids or obj.object_id in self.planned_object_ids:
                    continue
                bin_position = self.settings.bin_of(obj.object_type)
                if bin_position is None:
                    self.metrics.skipped_unknown_type += 1
                    continue
                u_now, v_now = obj.current_uv(sample.position_mm)
                objects.append(ObjectSnapshot(obj.object_id, obj.object_type, u_now, v_now,
                                              bin_position, obj.last_seen_at))
            return PlanningSnapshot(
                taken_at=now,
                belt_position_mm=sample.position_mm,
                measured_speed_mm_s=sample.speed_uv,
                setpoint_mm_s=setpoint_mm_s,
                arm_position=self.current_position,
                objects=tuple(objects),
            )

    def context(self, snapshot: PlanningSnapshot, forecast: BeltForecast) -> PlanContext:
        scheduling = self.settings.scheduling
        return PlanContext(snapshot.taken_at, snapshot.arm_position, forecast, self.arm,
                           self.settings.conveyor.workspace_window_uv,
                           scheduling.safety_margin_s, scheduling.setup_time_s)

    def schedule(self, snapshot: PlanningSnapshot, forecast: BeltForecast,
                 planner: BoundPlanner) -> tuple[list[ScheduledPick], int]:
        """Run a planner on a snapshot. Holds no lock. Returns (schedule, number of jobs)."""
        ctx = self.context(snapshot, forecast)
        jobs = build_jobs(snapshot.objects, ctx)
        return planner(jobs, ctx), len(jobs)

    def plan_next(self, state: RealtimeState, planner: BoundPlanner, setpoint_mm_s: float,
                  adaptive: bool, reused: list[ScheduledPick] | None) -> PickPlan | None:
        """Snapshot, plan (or reuse the schedule a speed law just scored) and commit."""
        now = time.monotonic()
        started = time.perf_counter()
        snapshot = self.snapshot(state, now, setpoint_mm_s)
        if snapshot is None:
            return None
        if reused is not None:
            schedule, n_jobs = reused, len(snapshot.objects)
        else:
            forecast = self.forecast(snapshot.measured_speed_mm_s, snapshot.setpoint_mm_s, adaptive)
            schedule, n_jobs = self.schedule(snapshot, forecast, planner)
        decision_s = time.perf_counter() - started
        plan = self.commit(state, schedule, planner.name, snapshot.taken_at, setpoint_mm_s, adaptive)
        if plan is not None:
            record = {
                "planner": planner.name,
                "jobs": n_jobs,
                "scheduled": [pick.job.obj.object_id for pick in schedule],
                "committed": plan.object_id,
                "reused": reused is not None,
                "decision_s": round(decision_s, 4),
            }
            record.update(plan.debug_info.get("schedule", {}))
            print("[SCHEDULE]", json.dumps(record, ensure_ascii=True))
        if decision_s > SLOW_DECISION_WARN_S:
            print(f"[WARN] {planner.name} planning took {decision_s:.2f} s")
        return plan

    def commit(self, state: RealtimeState, schedule: list[ScheduledPick], planner_name: str,
               snapshot_taken_at: float, setpoint_mm_s: float, adaptive: bool) -> PickPlan | None:
        """Claim the first schedule entry that is still pickable now: part still tracked and
        unclaimed, a live intercept from the arm's real position that meets the deadline, and
        every waypoint IK-reachable (open-issues O1)."""
        settings = self.settings
        now = time.monotonic()
        u_max = settings.conveyor.workspace_window_uv[1]
        with state.state_lock:
            sample = state.latest_speed
            if sample is None or now - sample.timestamp > settings.runtime.speed_timeout_s:
                return None
            live = self.forecast(sample.speed_uv, setpoint_mm_s, adaptive)
            for rank, entry in enumerate(schedule):
                object_id = entry.job.obj.object_id
                if object_id in state.claimed_object_ids or object_id in self.planned_object_ids:
                    continue
                obj = find_tracked_object(self.tracker, object_id)
                if obj is None:
                    continue
                u_now, v_now = obj.current_uv(sample.position_mm)
                if u_now > u_max:
                    self.metrics.skipped_outside_workspace += 1
                    self.tracker.remove(object_id)
                    self.ledger.left(object_id, now, past_workspace=True)
                    continue
                intercept = self.arm.predict(u_now, v_now, self.current_position, now, live, now)
                if intercept is None:
                    continue
                deadline = now + live.time_to_travel(u_max - u_now) - settings.scheduling.safety_margin_s
                if intercept.pick_time > deadline:
                    continue
                self.plan_counter += 1
                plan = build_pick_plan(
                    self.arm, f"plan-{self.plan_counter:06d}", obj, entry.job.obj.bin, intercept,
                    self.current_position, sample.position_mm, sample.speed_uv,
                    (sample.vx, sample.vy), self.interpolar_points)
                points = [(p.x, p.y, p.z) for p in (*plan.trajectory_goto, *plan.trajectory_pick)]
                unreachable = self.arm.first_unreachable(points)
                if unreachable is not None:
                    print("[WARN]", json.dumps({"event": "plan_ik_unreachable", "object_id": object_id,
                                                "waypoint": [round(c, 2) for c in unreachable]},
                                               ensure_ascii=True))
                    continue
                self.metrics.planned_picks += 1
                self.metrics.total_planning_latency += max(now - obj.last_seen_at, 0.0)
                self.metrics.planning_events += 1
                state.claimed_object_ids.add(object_id)
                self.ledger.planned(object_id, now)
                plan.debug_info["schedule"] = {
                    "planner": planner_name,
                    "rank": rank,
                    "drift_s": round(intercept.pick_time - entry.grab_s, 4),
                    "snapshot_age_s": round(now - snapshot_taken_at, 4),
                }
                return plan
        return None

    def mark_attempted(self, plan: PickPlan, success: bool, robot_pose: Point | None) -> None:
        """Book-keeping after the executor returns (called under state_lock). Exactly-once
        applies to DISPATCHED picks only: suction is never verified, so an attempted grip is
        not retried; a pre-grip abort leaves the part on the belt to be re-planned."""
        if plan.debug_info.get("pick_dispatched"):
            self.ledger.pick_done(plan.object_id, time.monotonic(), success)
        else:
            self.ledger.aborted(plan.object_id, plan.debug_info.get("abort_reason", "aborted"))
        if success or plan.debug_info.get("pick_dispatched"):
            self.planned_object_ids[plan.object_id] = time.monotonic()
            self.tracker.remove(plan.object_id)
        if success:
            self.current_position = self.arm.place_position(plan.sorting_position)
            self.metrics.completed_picks += 1
        elif robot_pose is not None:
            # Plan the next pick from where the arm really is, not from a park point the
            # failed goto may never have reached (T9).
            self.current_position = robot_pose
        else:
            goto_end = plan.trajectory_goto[-1]
            self.current_position = (goto_end.x, goto_end.y, goto_end.z)

    def recent_detection_times(self) -> tuple[float, ...]:
        return tuple(self.first_seen.values())
