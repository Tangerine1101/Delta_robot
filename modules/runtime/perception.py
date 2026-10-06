"""The perception thread: belt sample, detections, tracker upkeep and dashboard telemetry.

It only observes — it never decides a pick or a belt speed. It runs every
`PERCEPTION_PERIOD_S` so the pick gate always reads a fresh part position.
"""

from __future__ import annotations

import json
import math
import threading
import time
from typing import Any, Callable

from modules.core.delta import PERCEPTION_PERIOD_S
from modules.core.frames import belt_zone
from modules.runtime.cell_view import arm_geometry
from modules.runtime.planning import PickPlanner
from modules.runtime.state import RealtimeState, SpeedSample
from modules.settings import Settings

EventSink = Callable[[str, dict[str, Any]], None]


def _float_or_none(status: Any, key: str) -> float | None:
    if isinstance(status, dict) and status.get(key) is not None:
        try:
            return float(status[key])
        except (TypeError, ValueError):
            return None
    return None


def extract_robot_pose(status: dict[str, Any] | None) -> tuple[tuple[float, float, float] | None, int | None]:
    if not isinstance(status, dict):
        return None, None
    pose = status.get("pos_EE")
    if not isinstance(pose, (list, tuple)) or len(pose) < 3:
        return None, None
    try:
        position = (float(pose[0]), float(pose[1]), float(pose[2]))
    except (TypeError, ValueError):
        return None, None
    end_effector = status.get("end_effector")
    try:
        e_value = int(end_effector) if end_effector is not None else None
    except (TypeError, ValueError):
        e_value = None
    return position, e_value


class Perception:
    def __init__(
        self,
        settings: Settings,
        state: RealtimeState,
        planner: PickPlanner,
        speed_source: Any,
        image_source: Any,
        *,
        scenario_name: str,
        start_time: float,
        event_sink: EventSink | None = None,
        round_trip_s: Callable[[], float] = lambda: 0.0,
    ) -> None:
        self.settings = settings
        self.state = state
        self.planner = planner
        self.speed_source = speed_source
        self.image_source = image_source
        self.scenario_name = scenario_name
        self.start_time = start_time
        self.event_sink = event_sink
        self.round_trip_s = round_trip_s
        self._last_speed_log_t = 0.0
        self._last_flow_t = 0.0
        self._had_objects = False
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._loop, name="realtime-perception", daemon=True)
        self._thread.start()

    def join(self, timeout: float = 2.0) -> None:
        if self._thread is not None:
            self._thread.join(timeout=timeout)

    def _loop(self) -> None:
        while not self.state.stop_event.is_set():
            try:
                self.tick(time.monotonic())
            except Exception as exc:
                print(f"[WARN] realtime perception tick failed: {exc}", flush=True)
            time.sleep(PERCEPTION_PERIOD_S)

    def tick(self, now: float) -> SpeedSample:
        settings = self.settings
        state = self.state
        planner = self.planner
        sample = self.speed_source.sample(now)
        detections = self.image_source.poll(now)
        last_status = getattr(self.speed_source, "last_status", None)
        pose, end_effector = extract_robot_pose(last_status)
        measured_speed = _float_or_none(last_status, "speed_current")
        rotate_current = _float_or_none(last_status, "rotate_current")
        camera_window = settings.conveyor.camera_window_uv
        workspace_window = settings.conveyor.workspace_window_uv
        with state.state_lock:
            state.latest_speed = sample
            state.belt_position_mm = sample.position_mm
            state.belt_speed_mm_s = sample.speed_uv
            state.robot_pose = pose
            state.end_effector = end_effector
            state.belt_speed_measured_mm_s = measured_speed
            if rotate_current is not None:
                state.rotate_current_deg = rotate_current
            planner.ingest_detections(detections, sample.position_mm,
                                      position_at=getattr(self.speed_source, "position_at", None))
            # Every part still on the belt — from the camera ROI, through the transit gap, to
            # the downstream workspace edge — with its live u and zone, for the dashboard.
            snapshot = []
            belt_theta_deg = settings.conveyor.frame.theta_deg
            for obj in planner.tracker.objects():
                u_now, v_now = obj.current_uv(sample.position_mm)
                x_r, y_r = planner.tracker.current_position_R(obj, sample.position_mm)
                if planner.ledger.is_held(obj.object_id):
                    part_state = "held"
                elif obj.object_id in state.claimed_object_ids:
                    part_state = "claimed"
                else:
                    part_state = "free"
                snapshot.append({
                    "id": obj.object_id,
                    "type": obj.object_type,
                    "x": round(x_r, 2),
                    "y": round(y_r, 2),
                    "u": round(u_now, 1),
                    "v": round(v_now, 1),
                    "zone": belt_zone(u_now, camera_window, workspace_window),
                    "state": part_state,
                    # Board heading in the belt frame, for the dashboard's cell view.
                    "heading_uv_deg": round(math.degrees(obj.rotation_rad) - belt_theta_deg, 1),
                    "vision_angle_deg": round(obj.vision_angle_deg, 2),
                })
            planner.prune(state.claimed_object_ids, sample.position_mm, now)
            # Live density (unclaimed parts between the camera origin and u_max), for the
            # dashboard's density chart.
            u_max = workspace_window[1]
            n_density = sum(
                1 for obj in planner.tracker.objects()
                if obj.object_id not in state.claimed_object_ids
                and 0.0 <= obj.current_uv(sample.position_mm)[0] <= u_max)
            recent_pick_cycle_s = state.recent_pick_cycle_s
            robot_pose = state.robot_pose
        # Outside state_lock: stdout can stall (slow terminal/SSH) and must never block the
        # gate poll or plan build, which contend on the same lock.
        self._log_speed(sample)
        if self.event_sink is not None:
            status_payload = {
                "scenario": self.scenario_name,
                "vx": round(sample.vx, 3),
                "vy": round(sample.vy, 3),
                "speed_mm_s": round(math.hypot(sample.vx, sample.vy), 3),
                "position_mm": round(sample.position_mm, 2),
                "object_density": n_density,
                "round_trip_latency_s": round(self.round_trip_s(), 4),
                "pick_cycle_s": round(recent_pick_cycle_s, 3),
            }
            if robot_pose is not None:
                status_payload["x"] = round(robot_pose[0], 2)
                status_payload["y"] = round(robot_pose[1], 2)
                status_payload["z"] = round(robot_pose[2], 2)
                status_payload["arm"] = arm_geometry(state.frame, robot_pose)
            if end_effector is not None:
                status_payload["e"] = end_effector
            self.event_sink("status", status_payload)
        if now - self._last_flow_t >= 1.0:
            # Input / throughput record, 1 Hz: belt sample for flow.csv, counts for the charts.
            self._last_flow_t = now
            planner.ledger.belt_sample(now, sample.speed_uv, n_density)
            if self.event_sink is not None:
                self.event_sink("flow", planner.ledger.flow(now))
        if snapshot or self._had_objects:
            # One empty payload after the last part leaves, so the dashboard clears it.
            detect_payload = {
                "t": round(now - self.start_time, 3),
                "z": round(settings.robot.heights.pickup, 2),
                "objects": snapshot,
            }
            if snapshot:
                print("[DETECT]", json.dumps(detect_payload, ensure_ascii=True), flush=True)
            if self.event_sink is not None:
                self.event_sink("detect", detect_payload)
        self._had_objects = bool(snapshot)
        return sample

    def _log_speed(self, sample: SpeedSample) -> None:
        """Rate-limited (~1 Hz) belt trace, called outside state_lock."""
        now = time.monotonic()
        if now - self._last_speed_log_t < 1.0:
            return
        self._last_speed_log_t = now
        print(f"[BELT] vx={sample.vx:.4f} vy={sample.vy:.4f} "
              f"p={sample.position_mm:.3f} t={sample.timestamp:.4f}", flush=True)
