"""The decision cycle — the only place perception, the speed controller, the planner and the
pick executor meet (doc/basis-theory.md §7, basis-programming.md §2.2).

    arm becomes free ─► speed.tick("arm_free")        gate -> law -> commit
                        schedule = the law's, if scored with the executing planner,
                                   else planner(jobs)
                        plan = first entry that still validates
                        executor.execute(plan) ── at cup contact ─► speed.tick("contact")
    arm stays free   ─► speed.tick("idle"), re-plan every poll
"""

from __future__ import annotations

import json
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

from modules.runtime.perception import EventSink, Perception
from modules.runtime.pick_executor import RealtimePickExecutor
from modules.runtime.planning import PickPlanner
from modules.runtime.speed import SpeedController
from modules.runtime.state import RealtimeState
from modules.scheduling.registry import get_planner
from modules.settings import Settings


@dataclass
class RunContext:
    """Everything a scenario run needs, built by `modules.runtime.scenarios`."""

    scenario: str
    settings: Settings
    state: RealtimeState
    planner: PickPlanner
    speed_source: Any
    image_source: Any
    start_time: float
    duration_s: float | None
    event_sink: EventSink | None = None
    stop_event: threading.Event | None = None
    round_trip_s: Callable[[], float] = lambda: 0.0
    record_dir: Path | None = None           # where the part record is written at the end
    record_meta: dict[str, Any] = field(default_factory=dict)

    def perception(self) -> Perception:
        return Perception(self.settings, self.state, self.planner, self.speed_source,
                          self.image_source, scenario_name=self.scenario,
                          start_time=self.start_time, event_sink=self.event_sink,
                          round_trip_s=self.round_trip_s)

    def should_stop(self, now: float) -> bool:
        if self.duration_s is not None and now >= self.start_time + self.duration_s:
            return True
        if self.feed_done():
            print("[INFO] Virtual feed finished: every scheduled part has been dealt with")
            return True
        return self.stop_event is not None and self.stop_event.is_set()

    def feed_done(self) -> bool:
        """A virtual feed whose parts have all landed and left the tracker."""
        if not getattr(self.image_source, "exhausted", False):
            return False
        with self.state.state_lock:
            return not list(self.planner.tracker.objects()) and not self.state.claimed_object_ids

    def pump_window(self) -> bool:
        """Pump the live camera window on the main thread (Qt requirement)."""
        if hasattr(self.image_source, "render_window") and not self.image_source.render_window():
            print("\n[INFO] Vision window closed by user (q)")
            return False
        return True

    def close(self, perception: Perception) -> None:
        self.state.stop_event.set()
        perception.join(timeout=2.0)
        if hasattr(self.image_source, "stop"):
            self.image_source.stop()
        if hasattr(self.image_source, "close_window"):
            self.image_source.close_window()
        self.finish_record()

    def finish_record(self) -> None:
        """Book the parts still on the belt, then write and announce the run's record."""
        ledger = self.planner.ledger
        ledger.finish(time.monotonic())
        if "feeder" in self.record_meta and hasattr(self.image_source, "describe"):
            self.record_meta["feeder"].update(self.image_source.describe())
        summary = None
        if self.record_dir is not None:
            try:
                summary = ledger.write(self.record_dir, self.record_meta, self.settings.runtime.flow_bin_s)
                if hasattr(self.image_source, "write_arrivals"):
                    self.image_source.write_arrivals(self.record_dir / "arrivals.csv")
                print(f"[INFO] part record written to {self.record_dir}")
            except OSError as exc:
                print(f"[WARN] could not write the part record: {exc}")
        if summary is None:
            summary = {**self.record_meta, **ledger.summary()}
        print("[RUN-SUMMARY]", json.dumps(summary, ensure_ascii=True))
        if self.event_sink is not None:
            self.event_sink("flow", ledger.flow(time.monotonic()))
            self.event_sink("run_summary", summary)


def run_pick_loop(run: RunContext, executor: RealtimePickExecutor) -> None:
    settings = run.settings
    state = run.state
    planner_bound = get_planner(settings.scheduling.planner, settings.scheduling.planners)
    speed = SpeedController(settings, state, run.planner, planner_bound, executor.dispatch)
    executor.on_speed_event = speed.tick
    executor.ledger = run.planner.ledger
    print(f"[INFO] planner={planner_bound.name} speed_law={speed.law.name} gate={speed.law.gate.name}")

    perception = run.perception()
    perception.start()
    speed.seed()

    cycle_time_samples: deque[float] = deque(maxlen=20)
    arm_was_free = False
    try:
        while not state.stop_event.is_set():
            if run.should_stop(time.monotonic()):
                break
            decision = speed.tick("idle" if arm_was_free else "arm_free")
            arm_was_free = True
            plan = run.planner.plan_next(state, planner_bound, speed.setpoint_mm_s, speed.adaptive,
                                         speed.reusable_schedule(decision))
            if plan is not None:
                arm_was_free = False
                plan_summary = plan.to_summary()
                print("[PLAN]", json.dumps(plan_summary, ensure_ascii=True))
                if run.event_sink is not None:
                    run.event_sink("plan", plan_summary)
                cycle_t0 = time.monotonic()
                success = executor.execute(plan, state)
                cycle_time_samples.append(time.monotonic() - cycle_t0)
                with state.state_lock:
                    state.claimed_object_ids.discard(plan.object_id)
                    state.recent_pick_cycle_s = sum(cycle_time_samples) / len(cycle_time_samples)
                    run.planner.mark_attempted(plan, success, state.robot_pose)
            if not run.pump_window():
                break
            time.sleep(settings.runtime.poll_interval_s)
    except KeyboardInterrupt:
        print("\n[INFO] Scheduler scenario interrupted by user")
    finally:
        run.close(perception)
    print("[INFO] Scheduler metrics:", json.dumps(run.planner.metrics.as_dict(), ensure_ascii=True))


def run_observe_loop(run: RunContext) -> None:
    """Perception only: the arm stays idle and no belt command is sent. Every part stays
    tracked, so the dashboard lists it over the whole ROI -> workspace journey."""
    perception = run.perception()
    perception.start()
    try:
        while not run.state.stop_event.is_set():
            if run.should_stop(time.monotonic()):
                break
            if not run.pump_window():
                break
            time.sleep(run.settings.runtime.poll_interval_s)
    except KeyboardInterrupt:
        print("\n[INFO] Scenario interrupted by user")
    finally:
        run.close(perception)
    print("[INFO] Scheduler metrics:", json.dumps(run.planner.metrics.as_dict(), ensure_ascii=True))
