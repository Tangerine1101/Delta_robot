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
    dispatch: Callable[[dict[str, Any]], Any] | None = None   # PLC commands (belt-driving scenarios)

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


# test_camera_latency: belt on for BELT_STEP_ON_S, off for BELT_STEP_OFF_S, repeated. Both are
# the shortest the fits of core/latency allow (1.0 s before a change, up to 1.4 s after it).
BELT_STEP_ON_S = 1.6
BELT_STEP_OFF_S = 1.6
# Sampling period of the scenario's own recorder (s): finer than the perception tick, so no
# perception sample is missed.
LATENCY_SAMPLE_S = 0.005
# Longest wait for a part in the camera window once the model is ready (s); the cycle then
# starts anyway (a simulated feeder only brings parts while the belt moves).
LATENCY_PART_WAIT_S = 20.0


def run_belt_step_loop(run: RunContext) -> None:
    """Measure the camera latency the detection stamps leave out (core/latency.py).

    The arm stays idle; the belt runs at `speed.static_mm_s` for BELT_STEP_ON_S and stops for
    BELT_STEP_OFF_S, repeatedly, so every part in the camera window sees several starts and
    stops. Each perception sample's belt position and each visible part's offset u - p are
    recorded; at the end the jump of the offset at every start / stop gives the latency.

    The belt only starts when the most downstream part in view, moved by one on-phase, still
    lies wholly inside the camera window (its centre at least half the largest board size
    from the edge). Otherwise the run ends with the belt stopped, the parts still in view."""
    from modules.comm.packets import change_speed_packet
    from modules.core.frames import ConveyorFrame
    from modules.core.latency import change_lags, median_lag_s

    settings = run.settings
    state = run.state
    if run.dispatch is None:
        raise RuntimeError("test_camera_latency drives the belt and needs a live PLC link")
    speed = settings.speed.static_mm_s
    if not 5.0 <= speed <= 40.0:
        print(f"[WARN] speed.static_mm_s = {speed} mm/s: 15-30 mm/s keeps a part in the camera "
              "window across several starts and stops")
    camera_window = settings.conveyor.camera_window_uv
    step_mm = speed * BELT_STEP_ON_S
    edge_margin_mm = max((max(t.w, t.h) for t in settings.object_types.values()), default=0.0) / 2.0
    u_limit = camera_window[1] - edge_margin_mm
    room = u_limit - camera_window[0] - edge_margin_mm
    if step_mm > room:
        print(f"[WARN] one on-phase moves the belt {step_mm:.0f} mm but the camera window only has "
              f"{room:.0f} mm of room: lower speed.static_mm_s to <= {room / BELT_STEP_ON_S:.0f} mm/s")
    else:
        print(f"[LATENCY] {step_mm:.0f} mm per on-phase: a part placed at the upstream edge sees "
              f"{int(room // step_mm)} start/stop pairs")
    belt: list[tuple[float, float, float]] = []
    offsets: dict[str, list[tuple[float, float]]] = {}
    last_sample_t: float | None = None

    perception = run.perception()
    perception.start()
    belt_on = False
    cycle_started: float | None = None
    next_toggle = 0.0
    next_hint = 0.0
    model_ready_at: float | None = None
    print(f"[LATENCY] belt {speed:.0f} mm/s, {BELT_STEP_ON_S:.1f} s on / {BELT_STEP_OFF_S:.1f} s off; "
          "waiting for the vision model and for a part in the camera window")
    try:
        while not state.stop_event.is_set():
            now = time.monotonic()
            if run.stop_event is not None and run.stop_event.is_set():
                break
            if not run.pump_window():
                break
            if cycle_started is None:
                # The belt stays still until the model detects and a part sits in the camera
                # window; --duration counts from then, so loading the model costs nothing.
                if model_ready_at is None and _vision_ready(run.image_source):
                    model_ready_at = now
                if model_ready_at is not None and _part_in_window(state, camera_window):
                    cycle_started = next_toggle = now
                    print("[LATENCY] part in view: belt cycle starts")
                elif model_ready_at is not None and now - model_ready_at >= LATENCY_PART_WAIT_S:
                    cycle_started = next_toggle = now
                    print(f"[WARN] no part in the camera window after {LATENCY_PART_WAIT_S:.0f} s: "
                          "belt cycle starts anyway; only parts seen across a start/stop count")
                elif now >= next_hint:
                    waiting = "the vision model" if not _vision_ready(run.image_source) else "a part in the camera window"
                    print(f"[LATENCY] waiting for {waiting}")
                    next_hint = now + 5.0
                time.sleep(LATENCY_SAMPLE_S)
                continue
            if run.duration_s is not None and now >= cycle_started + run.duration_s:
                break
            if now >= next_toggle:
                if not belt_on:
                    lead = _lead_part_u(state, camera_window)
                    if lead is not None and lead + step_mm > u_limit:
                        print(f"[LATENCY] the next step would carry the lead part (u = {lead:.0f} mm) "
                              f"out of the camera window: ending with the belt stopped")
                        break
                belt_on = not belt_on
                run.dispatch(change_speed_packet(speed if belt_on else 0.0))
                next_toggle = now + (BELT_STEP_ON_S if belt_on else BELT_STEP_OFF_S)
            with state.state_lock:
                sample = state.latest_speed
                if sample is not None and sample.timestamp != last_sample_t:
                    last_sample_t = sample.timestamp
                    belt.append((sample.timestamp, sample.position_mm, sample.speed_uv))
                    for obj in state.tracker.objects():
                        u, v = obj.current_uv(sample.position_mm)
                        if ConveyorFrame.is_in_window_uv(u, v, camera_window):
                            offsets.setdefault(obj.object_id, []).append(
                                (sample.timestamp, u - sample.position_mm))
            time.sleep(LATENCY_SAMPLE_S)
    except KeyboardInterrupt:
        print("\n[INFO] Scenario interrupted by user")
    finally:
        try:
            run.dispatch(change_speed_packet(0.0))
        except Exception as exc:
            print(f"[WARN] belt stop failed: {exc}")
        run.close(perception)

    lags = change_lags(belt, offsets)
    for lag in lags:
        print("[LATENCY]", json.dumps({
            "t": round(float(lag["t"]) - run.start_time, 2), "part": lag["part"],
            "v_before": round(float(lag["v_before"]), 1), "v_after": round(float(lag["v_after"]), 1),
            "jump_mm": round(float(lag["jump_mm"]), 2), "lag_ms": round(float(lag["lag_s"]) * 1000, 1),
        }, ensure_ascii=True))
    lag_s = median_lag_s(lags)
    if lag_s is None:
        print("[LATENCY] no start/stop with a part in the camera window on both sides; "
              "place parts upstream in the window and run longer")
        return
    current = settings.vision.latency_offset_s
    spread = (max(float(x["lag_s"]) for x in lags) - min(float(x["lag_s"]) for x in lags)) * 1000
    print("[LATENCY-RESULT]", json.dumps({
        "samples": len(lags), "unmodelled_lag_ms": round(lag_s * 1000, 1), "range_ms": round(spread, 1),
        "latency_offset_s_now": current, "latency_offset_s_suggested": round(current + lag_s, 3),
    }, ensure_ascii=True))
    print(f"[LATENCY] set vision.latency_offset_s: {current + lag_s:.3f} "
          f"(> 0 = detections were stamped {lag_s * 1000:.0f} ms after the true capture)")


def _vision_ready(image_source: Any) -> bool:
    """The camera pipeline has its model loaded (sources without a model are always ready)."""
    return bool(getattr(image_source, "ready", True))


def _lead_part_u(state: RealtimeState, window: Any) -> float | None:
    """u of the most downstream part inside the camera window, or None."""
    from modules.core.frames import ConveyorFrame

    with state.state_lock:
        position = state.belt_position_mm
        us = [u for u, v in (obj.current_uv(position) for obj in state.tracker.objects())
              if ConveyorFrame.is_in_window_uv(u, v, window)]
    return max(us) if us else None


def _part_in_window(state: RealtimeState, window: Any) -> bool:
    from modules.core.frames import ConveyorFrame

    with state.state_lock:
        position = state.belt_position_mm
        return any(ConveyorFrame.is_in_window_uv(*obj.current_uv(position), window)
                   for obj in state.tracker.objects())
