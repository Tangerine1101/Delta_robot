"""The belt speed controller of a realtime run: the gate says when, the law says what, the
commit policy says how much.

Called on the main thread only — by the decision loop (`arm_free` / `idle`) and by the pick
executor after cup contact (`contact` / `busy`). Nothing is decided while a pick is committed
(goto dispatch -> cup contact), whatever the gate says (open-issues L9).
"""

from __future__ import annotations

import json
import math
import time
from typing import Any, Callable

from modules.comm.packets import COMMAND_ID
from modules.core.forecast import BeltForecast
from modules.runtime.planning import SLOW_DECISION_WARN_S, PickPlanner
from modules.runtime.state import RealtimeState
from modules.scheduling.commit import CommitPolicy, commit_step
from modules.scheduling.registry import BoundPlanner, get_planner, get_speed_law
from modules.scheduling.types import ScheduledPick, SpeedDecision, SpeedView
from modules.settings import Settings


def change_speed_packet(speed_mm_s: float) -> dict[str, Any]:
    return {"commandID": COMMAND_ID["change_speed"], "CommandID": COMMAND_ID["change_speed"],
            "rotate": 0.0, "speed": float(speed_mm_s)}


class SpeedController:
    def __init__(
        self,
        settings: Settings,
        state: RealtimeState,
        planner: PickPlanner,
        executing: BoundPlanner,
        dispatch: Callable[[dict[str, Any]], Any],
    ) -> None:
        self.settings = settings
        self.state = state
        self.planner = planner
        self.executing = executing
        self.dispatch = dispatch
        speed = settings.speed
        self.law = get_speed_law(speed.law, speed.laws, speed.setpoint_gate)
        self.policy = CommitPolicy(
            band=(speed.band.min_mm_s, settings.speed_ceiling_mm_s()),
            deadband_mm_s=speed.commit.deadband_mm_s,
            max_step_mm_s=speed.commit.max_step_mm_s,
        )
        self.setpoint_mm_s = speed.static_mm_s
        self.last_commit_t = 0.0
        self.last_decision_t = -math.inf
        self._planners: dict[str | None, BoundPlanner] = {None: executing, executing.name: executing}

    @property
    def adaptive(self) -> bool:
        return self.law.adaptive

    def seed(self) -> None:
        """Command the startup setpoint: the constant law's speed, or where an adaptive law
        starts walking from."""
        try:
            self.dispatch(change_speed_packet(self.setpoint_mm_s))
            print(f"[INFO] Conveyor speed set to {self.setpoint_mm_s} mm/s")
            self.last_commit_t = time.monotonic()
        except Exception as exc:
            print(f"[WARN] Could not set conveyor speed: {exc}")

    def tick(self, event: str) -> SpeedDecision | None:
        """A decision point. Returns the law's decision when the gate opened."""
        if not self.law.adaptive:
            return None
        now = time.monotonic()
        with self.state.state_lock:
            if self.state.pick_committed:
                return None
            measured = self.state.belt_speed_measured_mm_s
        if not self.law.gate_open(event, now - self.last_decision_t, self.settings.speed.control_period_s):
            return None
        view = self._view(now, event)
        if view is None:
            return None
        setpoint_before = self.setpoint_mm_s
        decision = self.law(view)
        self.last_decision_t = now
        record = {"law": self.law.name, "event": event, "target_mm_s": round(decision.target_mm_s, 2),
                  "setpoint_before_mm_s": round(setpoint_before, 2), "objects": len(view.objects_u)}
        record.update(decision.info)
        print("[SPEED-LAW]", json.dumps(record, ensure_ascii=True))
        if decision.info.get("decision_s", 0.0) > SLOW_DECISION_WARN_S:
            print(f"[WARN] {self.law.name} decision took {decision.info['decision_s']:.2f} s")
        self._commit(decision.target_mm_s, measured, now)
        return decision

    def reusable_schedule(self, decision: SpeedDecision | None) -> list[ScheduledPick] | None:
        """The schedule a law scored its winner with, when it was made by the executing
        planner — "the executed schedule is the scored schedule"."""
        if decision is None or decision.schedule is None:
            return None
        if decision.schedule_planner not in (None, self.executing.name):
            return None
        return decision.schedule

    # ---- internals ------------------------------------------------------------------------
    def _commit(self, target: float, measured: float | None, now: float) -> None:
        step = commit_step(target, self.setpoint_mm_s, measured, now - self.last_commit_t,
                           self.policy, use_deadband=self.law.spec.deadband)
        if step.send is None:
            return
        if step.resync:
            print(f"[WARN] belt speed diverged (setpoint {self.setpoint_mm_s:.1f}, "
                  f"measured {measured:.1f} mm/s) — re-sending setpoint")
        try:
            self.dispatch(change_speed_packet(step.send))
            print(f"[SPEED] belt -> {step.send:.1f} mm/s (target {target:.1f})")
        except Exception as exc:
            print(f"[WARN] change_speed failed: {exc}")
            return
        self.setpoint_mm_s = step.send
        self.last_commit_t = now

    def _planner(self, name: str | None) -> BoundPlanner:
        if name not in self._planners:
            self._planners[name] = get_planner(name, self.settings.scheduling.planners,
                                               key="speed.laws.<law>.planner")
        return self._planners[name]

    def _view(self, now: float, event: str) -> SpeedView | None:
        snapshot = self.planner.snapshot(self.state, now, self.setpoint_mm_s)
        if snapshot is None:
            return None
        u_max = self.settings.conveyor.workspace_window_uv[1]
        with self.state.state_lock:
            p_now = self.state.belt_position_mm
            objects_u = tuple(
                u for u in (obj.current_uv(p_now)[0] for obj in self.planner.tracker.objects()
                            if obj.object_id not in self.state.claimed_object_ids)
                if 0.0 <= u <= u_max)
            detection_times = self.planner.recent_detection_times()
        accel = self.settings.conveyor.accel_mm_s2

        def forecast_for(target: float) -> BeltForecast:
            return BeltForecast(snapshot.measured_speed_mm_s, target, accel)

        def schedule_for(forecast: BeltForecast, planner: str | None = None) -> list[ScheduledPick]:
            return self.planner.schedule(snapshot, forecast, self._planner(planner))[0]

        speed = self.settings.speed
        return SpeedView(
            now=snapshot.taken_at,
            setpoint_mm_s=self.setpoint_mm_s,
            measured_mm_s=snapshot.measured_speed_mm_s,
            band=self.policy.band,
            max_step_mm_s=speed.commit.max_step_mm_s,
            static_mm_s=speed.static_mm_s,
            workspace_window_uv=self.settings.conveyor.workspace_window_uv,
            objects_u=objects_u,
            arm_cycle=self.settings.scheduling.arm_cycle,
            event=event,
            detection_times=detection_times,
            forecast_for=forecast_for,
            schedule_for=schedule_for,
        )
