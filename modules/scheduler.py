from __future__ import annotations

import json
import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from modules.EthernetCom import COMMAND_ID, RobotPacket, load_config
from modules.image_processing import ObjectDetection, SimulatedImageProcessing


SCENARIO_NAMES = {"test_accuracy", "test_throughput", "evaluate"}

Position3D = tuple[float, float, float]


@dataclass(frozen=True)
class SpeedSample:
    vx: float
    vy: float
    timestamp: float


@dataclass(frozen=True)
class TrajectoryPoint:
    x: float
    y: float
    z: float
    e: int
    time_s: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "e": self.e,
            "time": self.time_s,
        }


@dataclass
class PickPlan:
    plan_id: str
    object_id: str
    object_type: str
    detected_at: float
    source_position_2d: tuple[float, float]
    cycle_start_position: Position3D
    assumed_speed: tuple[float, float]
    predicted_pick_time: float
    pick_dispatch_time: float
    predicted_pick_position_2d: tuple[float, float, float]
    sorting_position: Position3D
    trajectory_goto: list[TrajectoryPoint]
    trajectory_pick: list[TrajectoryPoint]
    status: str = "planned"
    debug_info: dict[str, Any] = field(default_factory=dict)

    def total_duration(self) -> float:
        return sum(point.time_s for point in self.trajectory_goto + self.trajectory_pick)

    def to_robot_packets(self, interpolar_points: int) -> list[dict[str, Any]]:
        return [
            _trajectory_packet(self.trajectory_goto, interpolar_points),
            _trajectory_packet(self.trajectory_pick, interpolar_points),
        ]

    def to_summary(self) -> dict[str, Any]:
        return {
            "plan_id": self.plan_id,
            "object_id": self.object_id,
            "object_type": self.object_type,
            "predicted_pick_time": round(self.predicted_pick_time, 3),
            "pick_dispatch_time": round(self.pick_dispatch_time, 3),
            "predicted_pick_position_2d": [
                round(self.predicted_pick_position_2d[0], 3),
                round(self.predicted_pick_position_2d[1], 3),
                round(self.predicted_pick_position_2d[2], 3),
            ],
            "sorting_position": [round(value, 3) for value in self.sorting_position],
            "duration_s": round(self.total_duration(), 3),
            "status": self.status,
        }


@dataclass
class SchedulerMetrics:
    total_detections: int = 0
    planned_picks: int = 0
    completed_picks: int = 0
    stale_drops: int = 0
    skipped_unknown_type: int = 0
    skipped_outside_workspace: int = 0
    total_planning_latency: float = 0.0
    planning_events: int = 0
    queue_peak: int = 0

    def as_dict(self) -> dict[str, Any]:
        average_latency = (
            self.total_planning_latency / self.planning_events if self.planning_events else 0.0
        )
        return {
            "total_detections": self.total_detections,
            "planned_picks": self.planned_picks,
            "completed_picks": self.completed_picks,
            "stale_drops": self.stale_drops,
            "skipped_unknown_type": self.skipped_unknown_type,
            "skipped_outside_workspace": self.skipped_outside_workspace,
            "average_planning_latency_s": round(average_latency, 4),
            "queue_peak": self.queue_peak,
        }


@dataclass
class EvaluateMetrics:
    cycles_completed: int = 0
    picks_completed: int = 0
    total_phase_wall_time_s: float = 0.0
    phase_wall_times: list[float] = field(default_factory=list)
    phase_distances: list[float] = field(default_factory=list)
    position_wait_timeouts: int = 0
    position_stability_accepts: int = 0

    def as_dict(self) -> dict[str, Any]:
        total_t = self.total_phase_wall_time_s
        total_d = sum(self.phase_distances)
        speeds = [d / t for d, t in zip(self.phase_distances, self.phase_wall_times) if t > 0.0]
        pick_per_min = (self.picks_completed / (total_t / 60.0)) if total_t > 0.0 else 0.0
        avg_speed = (total_d / total_t) if total_t > 0.0 else 0.0
        peak_speed = max(speeds) if speeds else 0.0
        n = len(self.phase_wall_times)
        avg_phase = (total_t / n) if n else 0.0
        sorted_t = sorted(self.phase_wall_times)
        if sorted_t:
            p95_index = min(len(sorted_t) - 1, int(round(len(sorted_t) * 0.95)) - 1)
            p95_phase = sorted_t[max(p95_index, 0)]
            min_phase = sorted_t[0]
            max_phase = sorted_t[-1]
        else:
            p95_phase = 0.0
            min_phase = 0.0
            max_phase = 0.0
        return {
            "cycles_completed": self.cycles_completed,
            "picks_completed": self.picks_completed,
            "throughput_pick_per_min": round(pick_per_min, 2),
            "avg_speed_mm_s": round(avg_speed, 2),
            "peak_speed_mm_s": round(peak_speed, 2),
            "total_path_mm": round(total_d, 2),
            "total_phase_wall_time_s": round(total_t, 4),
            "avg_phase_s": round(avg_phase, 4),
            "min_phase_s": round(min_phase, 4),
            "max_phase_s": round(max_phase, 4),
            "p95_phase_s": round(p95_phase, 4),
            "position_wait_timeouts": self.position_wait_timeouts,
            "position_stability_accepts": self.position_stability_accepts,
        }


@dataclass(frozen=True)
class SchedulerSettings:
    home_position: Position3D
    clearance_height: float
    slope_transition_height: float
    pickup_height: float
    pre_pick_height: float
    place_height: float
    corner_blend_xy: float
    intercept_lead_time_s: float
    release_descent_time_s: float
    nominal_xy_speed: float
    nominal_z_speed: float
    stale_timeout_s: float
    speed_timeout_s: float
    poll_interval_s: float
    default_speed: tuple[float, float]
    robot_movement_delay_s: float
    ethernet_delay_s: float
    pickup_window_x: tuple[float, float]
    pickup_window_y: tuple[float, float]
    accuracy_points: list[Position3D]
    log_path: str
    object_type_map: dict[str, str]
    object_thickness_mm: dict[str, float]
    sorting_positions: dict[str, Position3D]
    throughput_object_types: list[str]
    throughput_lanes: list[float]
    throughput_spawn_x: float
    throughput_spawn_y: float
    throughput_emit_interval_s: float
    accuracy_emit_interval_s: float
    execution_margin_s: float
    evaluate_position_tolerance_mm: float = 0.01
    evaluate_wait_timeout_s: float = 10.0
    evaluate_stability_window_s: float = 0.4
    evaluate_stability_mm: float = 0.3

    def validate(self) -> None:
        # In physical delta coordinates (negative Z), values closer to 0 are higher (closer to base).
        # Therefore, clearance_height must be higher (less negative) than the
        # slope transition plane, which must stay above pre-pick and pickup.
        # Format: clearance_height > slope_transition_height > pre_pick_height > pickup_height
        if self.clearance_height <= self.slope_transition_height:
            raise ValueError(
                f"Configuration Error: clearance_height ({self.clearance_height}) must be higher "
                f"than slope_transition_height ({self.slope_transition_height}) in physical space."
            )
        if self.slope_transition_height <= self.pre_pick_height:
            raise ValueError(
                f"Configuration Error: slope_transition_height ({self.slope_transition_height}) "
                f"must be higher than pre_pick_height ({self.pre_pick_height}) in physical space."
            )
        if self.pre_pick_height <= self.pickup_height:
            raise ValueError(
                f"Configuration Error: pre_pick_height ({self.pre_pick_height}) must be higher "
                f"than pickup_height ({self.pickup_height}) in physical space (less negative)."
            )
        if self.clearance_height < self.place_height:
            raise ValueError(
                f"Configuration Error: clearance_height ({self.clearance_height}) must be higher "
                f"than or equal to place_height ({self.place_height}) in physical space."
            )

    @classmethod
    def from_config(cls, config: Any) -> "SchedulerSettings":
        scheduler_raw = getattr(config, "scheduler", {}) or {}
        raw_object_types = dict(getattr(config, "object_types", {}) or {})
        object_type_map: dict[str, str] = {}
        object_thickness_mm: dict[str, float] = {}
        sorting_positions: dict[str, Position3D] = {}
        for object_type, type_info in raw_object_types.items():
            if isinstance(type_info, dict):
                destination_name = str(type_info.get("destination", object_type))
                object_thickness_mm[object_type] = float(type_info.get("thickness_mm", 0.0))
            else:
                destination_name = str(type_info)
            object_type_map[object_type] = destination_name
            raw_position = getattr(config, destination_name, None)
            if raw_position is None:
                continue
            sorting_positions[destination_name] = _coerce_position3d(raw_position, (0.0, 0.0, -210.0))

        accuracy_points = [
            _coerce_position3d(point, (0.0, 0.0, -220.0))
            for point in scheduler_raw.get(
                "accuracy_points",
                [
                    [40.0, -60.0, -300.0],
                    [0.0, 0.0, -300.0],
                    [-40.0, 60.0, -300.0],
                ],
            )
        ]

        clearance_height = float(scheduler_raw.get("clearance_height", -165.0))
        pre_pick_height = float(scheduler_raw.get("pre_pick_height", -210.0))
        slope_transition_height = float(
            scheduler_raw.get(
                "slope_transition_height",
                (clearance_height + pre_pick_height) / 2.0,
            )
        )

        settings = cls(
            home_position=_coerce_position3d(
                scheduler_raw.get("home_position", [0.0, 0.0, -180.0]),
                (0.0, 0.0, -180.0),
            ),
            clearance_height=clearance_height,
            slope_transition_height=slope_transition_height,
            pickup_height=float(scheduler_raw.get("pickup_height", -230.0)),
            pre_pick_height=pre_pick_height,
            place_height=float(scheduler_raw.get("place_height", -205.0)),
            corner_blend_xy=float(scheduler_raw.get("corner_blend_xy", 35.0)),
            intercept_lead_time_s=float(scheduler_raw.get("intercept_lead_time_s", 0.14)),
            release_descent_time_s=float(scheduler_raw.get("release_descent_time_s", 0.14)),
            nominal_xy_speed=float(scheduler_raw.get("nominal_xy_speed", 220.0)),
            nominal_z_speed=float(scheduler_raw.get("nominal_z_speed", 180.0)),
            stale_timeout_s=float(scheduler_raw.get("stale_timeout_s", 5.0)),
            speed_timeout_s=float(scheduler_raw.get("speed_timeout_s", 1.0)),
            poll_interval_s=float(scheduler_raw.get("poll_interval_s", 0.05)),
            default_speed=_coerce_vector2d(
                scheduler_raw.get("default_speed", [0.0, 80.0]),
                (0.0, 80.0),
            ),
            robot_movement_delay_s=float(scheduler_raw.get("robot_movement_delay_s", 0.05)),
            ethernet_delay_s=float(scheduler_raw.get("ethernet_delay_s", 0.002)),
            pickup_window_x=_coerce_range(
                scheduler_raw.get("pickup_window_x", [-120.0, 120.0]),
                (-120.0, 120.0),
            ),
            pickup_window_y=_coerce_range(
                scheduler_raw.get("pickup_window_y", [-120.0, 120.0]),
                (-120.0, 120.0),
            ),
            accuracy_points=accuracy_points,
            log_path=str(scheduler_raw.get("log_path", "data.log")),
            object_type_map=object_type_map,
            object_thickness_mm=object_thickness_mm,
            sorting_positions=sorting_positions,
            throughput_object_types=list(scheduler_raw.get("throughput_object_types", ["object_A"])),
            throughput_lanes=[float(value) for value in scheduler_raw.get("throughput_lanes", [-60.0, 0.0, 60.0])],
            throughput_spawn_x=float(scheduler_raw.get("throughput_spawn_x", -180.0)),
            throughput_spawn_y=float(scheduler_raw.get("throughput_spawn_y", -180.0)),
            throughput_emit_interval_s=float(scheduler_raw.get("throughput_emit_interval_s", 0.35)),
            accuracy_emit_interval_s=float(scheduler_raw.get("accuracy_emit_interval_s", 0.8)),
            execution_margin_s=float(scheduler_raw.get("execution_margin_s", 0.3)),
            evaluate_position_tolerance_mm=float(
                scheduler_raw.get("evaluate_position_tolerance_mm", 0.01)
            ),
            evaluate_wait_timeout_s=float(
                scheduler_raw.get("evaluate_wait_timeout_s", 10.0)
            ),
            evaluate_stability_window_s=float(
                scheduler_raw.get("evaluate_stability_window_s", 0.4)
            ),
            evaluate_stability_mm=float(
                scheduler_raw.get("evaluate_stability_mm", 0.3)
            ),
        )
        settings.validate()
        return settings


class SimulatedSpeedSource:
    def __init__(self, scenario_name: str, settings: SchedulerSettings, start_time: float) -> None:
        self.scenario_name = scenario_name
        self.settings = settings
        self.start_time = start_time

    def sample(self, now: float) -> SpeedSample:
        if self.scenario_name == "test_accuracy":
            return SpeedSample(vx=0.0, vy=0.0, timestamp=now)

        elapsed = now - self.start_time
        band = int(elapsed // 4.0) % 3
        scale = [0.8, 1.0, 1.2][band]
        vx = self.settings.default_speed[0] * scale
        vy = self.settings.default_speed[1] * scale
        return SpeedSample(vx=vx, vy=vy, timestamp=now)


class RealSpeedSource:
    """Read conveyor speed from the status callback."""

    def __init__(self, request_status, scenario_name: str = "") -> None:
        self.request_status = request_status
        self.scenario_name = scenario_name

    def sample(self, now: float) -> SpeedSample:
        if self.scenario_name == "test_accuracy":
            return SpeedSample(vx=0.0, vy=0.0, timestamp=now)
        try:
            status = self.request_status()
            if status is not None:
                speed = float(status.get("speed_current", 80.0) or 80.0)
                return SpeedSample(vx=0.0, vy=speed, timestamp=now)
        except Exception as exc:
            print(f"[WARN] RealSpeedSource failed to read speed: {exc}")
        return SpeedSample(vx=0.0, vy=80.0, timestamp=now)


class SimulatedExecutor:
    def __init__(self, log_path: str, sample_period_s: float) -> None:
        self.log_path = Path(log_path)
        self.sample_period_s = max(sample_period_s, 0.02)
        if self.log_path.parent != Path("."):
            self.log_path.parent.mkdir(parents=True, exist_ok=True)

    def execute(
        self,
        plan: PickPlan,
        *,
        log_samples: bool = False,
        real_time: bool = False,
        scenario_name: str,
    ) -> None:
        if log_samples:
            self._log_plan_trace(plan, real_time=real_time, scenario_name=scenario_name)
        plan.status = "completed"

    def _log_plan_trace(self, plan: PickPlan, *, real_time: bool, scenario_name: str) -> None:
        cycle_start = time.time()
        current_position = plan.cycle_start_position
        current_time = cycle_start
        trace_entries: list[dict[str, Any]] = []

        for phase_name, trajectory in (
            ("goto", plan.trajectory_goto),
            ("pick", plan.trajectory_pick),
        ):
            previous = current_position
            for point in trajectory:
                segment_samples = max(1, int(math.ceil(point.time_s / self.sample_period_s)))
                for sample_index in range(1, segment_samples + 1):
                    fraction = sample_index / segment_samples
                    x = previous[0] + (point.x - previous[0]) * fraction
                    y = previous[1] + (point.y - previous[1]) * fraction
                    z = previous[2] + (point.z - previous[2]) * fraction
                    entry = {
                        "logged_at": round(current_time, 6),
                        "scenario": scenario_name,
                        "plan_id": plan.plan_id,
                        "object_id": plan.object_id,
                        "phase": phase_name,
                        "x": round(x, 4),
                        "y": round(y, 4),
                        "z": round(z, 4),
                        "e": point.e,
                    }
                    trace_entries.append(entry)
                    current_time += point.time_s / segment_samples
                    if real_time:
                        time.sleep(point.time_s / segment_samples)
                previous = (point.x, point.y, point.z)
            current_position = previous

        with self.log_path.open("a", encoding="utf-8") as handle:
            for entry in trace_entries:
                handle.write(json.dumps(entry, ensure_ascii=True) + "\n")


class RealRobotExecutor:
    """Execute a PickPlan by sending real trajectory packages to the PLC."""

    def __init__(
        self,
        dispatch,
        request_status,
        *,
        interpolar_points: int,
        wait_margin_s: float,
        status_poll_interval_s: float,
    ) -> None:
        self.dispatch = dispatch
        self.request_status = request_status
        self.interpolar_points = interpolar_points
        self.wait_margin_s = wait_margin_s
        self.status_poll_interval_s = max(status_poll_interval_s, 0.02)

    def execute(
        self,
        plan: PickPlan,
        *,
        log_samples: bool = False,
        real_time: bool = False,
        scenario_name: str,
    ) -> None:
        del log_samples, real_time, scenario_name
        packets = plan.to_robot_packets(self.interpolar_points)
        for phase_name, packet in zip(("goto", "pick"), packets):
            if phase_name == "pick":
                self._wait_until_pick_dispatch(plan)
                try:
                    rotate_pkg = {
                        "commandID": COMMAND_ID["rotate_absolute"],
                        "CommandID": COMMAND_ID["rotate_absolute"],
                        "rotate": 90.0,
                        "speed": 0.0,
                    }
                    self.dispatch(rotate_pkg)
                except Exception as s_exc:
                    print(f"[WARN] Failed to dispatch Siemens rotation: {s_exc}")
            print(
                "[EXEC]",
                json.dumps(
                    {
                        "plan_id": plan.plan_id,
                        "phase": phase_name,
                        "commandID": packet.get("commandID"),
                        "argument_number": packet.get("argument_number"),
                    },
                    ensure_ascii=True,
                ),
            )
            status = self.dispatch(packet)
            if status is not None:
                print("[PLC]", json.dumps(status, ensure_ascii=True))
            self._wait_for_phase_completion(packet)
        plan.status = "completed"

    def _wait_until_pick_dispatch(self, plan: PickPlan) -> None:
        remaining_s = plan.pick_dispatch_time - time.monotonic()
        if remaining_s <= 0.0:
            print(
                "[WARN]",
                json.dumps(
                    {
                        "plan_id": plan.plan_id,
                        "event": "late_pick_dispatch",
                        "late_by_s": round(abs(remaining_s), 4),
                    },
                    ensure_ascii=True,
                ),
            )
            return
        time.sleep(remaining_s)

    def _wait_for_phase_completion(self, packet: dict[str, Any]) -> None:
        argument_number = int(packet.get("argument_number", 0))
        durations = list(packet.get("argument_time", []))[:argument_number]
        expected_duration_s = max(sum(float(value) for value in durations), 0.0)
        minimum_deadline = time.monotonic() + expected_duration_s
        hard_deadline = minimum_deadline + self.wait_margin_s
        last_status: dict[str, Any] | None = None

        while True:
            now = time.monotonic()
            if now >= minimum_deadline:
                last_status = self.request_status()
                task_state = None if last_status is None else last_status.get("task_state")
                if task_state is not None and int(task_state) == 0:
                    return
                if now >= hard_deadline:
                    return
            time.sleep(self.status_poll_interval_s)


class EvaluateExecutor:
    """Dispatch evaluate plans and gate phase progression on pos_EE feedback.

    The Omron PLC firmware drives motors at a fixed maximum speed and ignores
    argument_time, so wall-time measured here reflects the true mechanism speed.

    A background thread polls request_status() every `status_poll_interval_s`
    (default 10 ms) and caches the result. A shared mutex guarantees that
    polling and packet dispatch never share the communication queue simultaneously.
    """

    def __init__(
        self,
        dispatch,
        request_status,
        *,
        interpolar_points: int,
        position_tolerance_mm: float = 0.01,
        status_poll_interval_s: float = 0.01,
        wait_timeout_s: float = 10.0,
        stability_window_s: float = 0.4,
        stability_mm: float = 0.3,
    ) -> None:
        self._dispatch_fn = dispatch
        self._request_status_fn = request_status
        self.interpolar_points = interpolar_points
        self.position_tolerance_mm = float(position_tolerance_mm)
        self.status_poll_interval_s = max(float(status_poll_interval_s), 0.005)
        self.wait_timeout_s = float(wait_timeout_s)
        self.stability_window_s = max(float(stability_window_s), 0.0)
        self.stability_mm = max(float(stability_mm), 0.0)

        # Mutex shared between the background poller and dispatch calls.
        # Ensures only one message is in the communication queue at any time.
        self._comm_lock = threading.Lock()
        self._cached_status: dict[str, Any] | None = None
        self._cache_lock = threading.Lock()  # guards _cached_status reads/writes

        self._stop_event = threading.Event()
        self._poller_thread = threading.Thread(
            target=self._poll_loop, name="status-poller", daemon=True
        )
        self._poller_thread.start()

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def close(self) -> None:
        """Stop the background status-poller thread."""
        self._stop_event.set()
        self._poller_thread.join(timeout=2.0)

    def execute_evaluate(self, plan: "PickPlan", metrics: EvaluateMetrics) -> "Position3D | None":
        packets = plan.to_robot_packets(self.interpolar_points)
        goto_end = (
            plan.trajectory_goto[-1].x,
            plan.trajectory_goto[-1].y,
            plan.trajectory_goto[-1].z,
        )
        phase_starts: list[Position3D] = [plan.cycle_start_position, goto_end]
        trajectories = [plan.trajectory_goto, plan.trajectory_pick]
        last_actual_pos: Position3D | None = None

        for phase_name, packet, trajectory, phase_start in zip(
            ("goto", "pick"), packets, trajectories, phase_starts
        ):
            if phase_name == "pick":
                try:
                    self._locked_dispatch({
                        "commandID": COMMAND_ID["rotate_absolute"],
                        "CommandID": COMMAND_ID["rotate_absolute"],
                        "rotate": 90.0,
                        "speed": 0.0,
                    })
                except Exception as exc:
                    print(f"[WARN] evaluate: Siemens rotate failed: {exc}")

            target = (trajectory[-1].x, trajectory[-1].y, trajectory[-1].z)
            distance = _path_distance(phase_start, trajectory)

            print(
                "[EXEC]",
                json.dumps(
                    {"plan_id": plan.plan_id, "phase": phase_name, "target": list(target)},
                    ensure_ascii=True,
                ),
                flush=True,
            )
            t0 = time.monotonic()
            try:
                self._locked_dispatch(packet)
            except Exception as exc:
                print(f"[ERROR] evaluate: dispatch failed: {exc}")
                raise
            last_actual_pos = self._wait_until_position_reached(target, metrics)
            wall = time.monotonic() - t0
            metrics.phase_wall_times.append(wall)
            metrics.phase_distances.append(distance)
            metrics.total_phase_wall_time_s += wall

        metrics.picks_completed += 1
        plan.status = "completed"
        return last_actual_pos

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _poll_loop(self) -> None:
        """Background thread: read status every status_poll_interval_s and cache it."""
        while not self._stop_event.is_set():
            with self._comm_lock:
                try:
                    status = self._request_status_fn()
                    with self._cache_lock:
                        self._cached_status = status
                except Exception as exc:
                    print(f"[WARN] evaluate poller: {exc}", flush=True)
            time.sleep(self.status_poll_interval_s)

    def _locked_dispatch(self, packet: dict[str, Any]) -> Any:
        """Send a packet while holding the comm lock so the poller cannot interfere."""
        with self._comm_lock:
            return self._dispatch_fn(packet)

    def _read_cache(self) -> dict[str, Any] | None:
        with self._cache_lock:
            return self._cached_status

    def _wait_until_position_reached(
        self, target: Position3D, metrics: EvaluateMetrics
    ) -> "Position3D | None":
        deadline = time.monotonic() + self.wait_timeout_s
        checks = 0
        last_pos: Position3D | None = None
        last_distance: float | None = None
        last_task_state: int | None = None
        min_distance: float | None = None
        idle_seen_at: float | None = None
        # Require task_state==0 to persist for at least two poll cycles before accepting.
        idle_settle_s = self.status_poll_interval_s * 2.0
        # Rolling window of recent pos_EE samples for mechanical-stability detection.
        pos_window: deque[tuple[float, Position3D]] = deque()
        first_sample_time: float | None = None

        while True:
            checks += 1
            status = self._read_cache()

            if status is not None:
                pos = status.get("pos_EE")
                if isinstance(pos, (list, tuple)) and len(pos) >= 3:
                    try:
                        px, py, pz = float(pos[0]), float(pos[1]), float(pos[2])
                    except (TypeError, ValueError):
                        px = py = pz = float("nan")
                    last_pos = (px, py, pz)
                    dx = px - target[0]
                    dy = py - target[1]
                    dz = pz - target[2]
                    last_distance = math.sqrt(dx * dx + dy * dy + dz * dz)
                    if min_distance is None or last_distance < min_distance:
                        min_distance = last_distance
                    if last_distance < self.position_tolerance_mm:
                        return last_pos

                ts = status.get("task_state")
                if ts is not None:
                    try:
                        last_task_state = int(ts)
                    except (TypeError, ValueError):
                        last_task_state = None

                # PLC idle (task_state==0): accept after settle window and log offset.
                if last_task_state == 0 and last_distance is not None:
                    now = time.monotonic()
                    if idle_seen_at is None:
                        idle_seen_at = now
                    elif now - idle_seen_at >= idle_settle_s:
                        if last_distance >= self.position_tolerance_mm:
                            print(
                                "[INFO]",
                                json.dumps(
                                    {
                                        "event": "evaluate_idle_with_offset",
                                        "target": [round(v, 4) for v in target],
                                        "pos_EE": [round(v, 4) for v in last_pos] if last_pos else None,
                                        "distance_mm": round(last_distance, 4),
                                        "tolerance_mm": self.position_tolerance_mm,
                                    },
                                    ensure_ascii=True,
                                ),
                                flush=True,
                            )
                        return last_pos
                else:
                    idle_seen_at = None

                # Mechanical-stability fallback: robot is physically stopped even
                # though PLC firmware hasn't transitioned task_state to 0.
                if (
                    self.stability_mm > 0.0
                    and self.stability_window_s > 0.0
                    and last_pos is not None
                ):
                    now = time.monotonic()
                    if first_sample_time is None:
                        first_sample_time = now
                    pos_window.append((now, last_pos))
                    cutoff = now - self.stability_window_s
                    while pos_window and pos_window[0][0] < cutoff:
                        pos_window.popleft()
                    if (
                        len(pos_window) >= 2
                        and (now - first_sample_time) >= self.stability_window_s
                    ):
                        xs = [p[1][0] for p in pos_window]
                        ys = [p[1][1] for p in pos_window]
                        zs = [p[1][2] for p in pos_window]
                        spread = math.sqrt(
                            (max(xs) - min(xs)) ** 2
                            + (max(ys) - min(ys)) ** 2
                            + (max(zs) - min(zs)) ** 2
                        )
                        if spread <= self.stability_mm:
                            metrics.position_stability_accepts += 1
                            print(
                                "[INFO]",
                                json.dumps(
                                    {
                                        "event": "evaluate_stability_accept",
                                        "target": [round(v, 4) for v in target],
                                        "pos_EE": [round(v, 4) for v in last_pos],
                                        "distance_mm": round(last_distance, 4) if last_distance is not None else None,
                                        "spread_mm": round(spread, 4),
                                        "stability_mm": self.stability_mm,
                                        "stability_window_s": self.stability_window_s,
                                        "samples": len(pos_window),
                                        "last_task_state": last_task_state,
                                        "tolerance_mm": self.position_tolerance_mm,
                                    },
                                    ensure_ascii=True,
                                ),
                                flush=True,
                            )
                            return last_pos

            if time.monotonic() >= deadline:
                metrics.position_wait_timeouts += 1
                print(
                    "[WARN]",
                    json.dumps(
                        {
                            "event": "evaluate_position_timeout",
                            "target": [round(v, 4) for v in target],
                            "last_pos_EE": [round(v, 4) for v in last_pos] if last_pos else None,
                            "last_distance_mm": round(last_distance, 4) if last_distance is not None else None,
                            "min_distance_mm": round(min_distance, 4) if min_distance is not None else None,
                            "tolerance_mm": self.position_tolerance_mm,
                            "last_task_state": last_task_state,
                            "cache_checks": checks,
                            "wait_timeout_s": self.wait_timeout_s,
                        },
                        ensure_ascii=True,
                    ),
                    flush=True,
                )
                return last_pos

            # Small yield so the CPU isn't fully busy; actual data rate is driven
            # by the background poller (status_poll_interval_s = 10 ms).
            time.sleep(0.002)


class PickScheduler:
    def __init__(self, settings: SchedulerSettings, interpolar_points: int) -> None:
        self.settings = settings
        self.interpolar_points = interpolar_points
        self.pending_objects: list[ObjectDetection] = []
        self.seen_object_ids: dict[str, float] = {}
        self.metrics = SchedulerMetrics()
        self.current_position: Position3D = settings.home_position
        self.latest_speed: SpeedSample | None = None
        self.plan_counter = 0

    def ingest_detections(self, detections: list[ObjectDetection]) -> None:
        for detection in detections:
            self.metrics.total_detections += 1
            if detection.object_id in self.seen_object_ids:
                continue
            self.seen_object_ids[detection.object_id] = detection.timestamp
            self.pending_objects.append(detection)
        self.metrics.queue_peak = max(self.metrics.queue_peak, len(self.pending_objects))

    def update_speed(self, sample: SpeedSample) -> None:
        self.latest_speed = sample
        print(f"[SPEED] vx={sample.vx:.4f} vy={sample.vy:.4f} t={sample.timestamp:.4f}", flush=True)

    def prune_stale(self, now: float) -> None:
        kept: list[ObjectDetection] = []
        for detection in self.pending_objects:
            if now - detection.timestamp > self.settings.stale_timeout_s:
                self.metrics.stale_drops += 1
                continue
            kept.append(detection)
        self.pending_objects = kept

        # Prune seen_object_ids to prevent memory leaks
        limit = now - self.settings.stale_timeout_s
        self.seen_object_ids = {
            obj_id: ts for obj_id, ts in self.seen_object_ids.items() if ts >= limit
        }

    def plan_next(self, now: float) -> PickPlan | None:
        if self.latest_speed is None:
            return None
        if now - self.latest_speed.timestamp > self.settings.speed_timeout_s:
            return None

        candidates: list[tuple[float, ObjectDetection, Position3D]] = []
        kept_pending: list[ObjectDetection] = []
        for detection in self.pending_objects:
            sorting_position = self._resolve_sorting_position(detection.object_type)
            if sorting_position is None:
                self.metrics.skipped_unknown_type += 1
                continue

            prediction = self._predict_pick_position(detection, self.latest_speed, now)
            if prediction is None:
                # Check if it has passed downstream boundary
                dt = max(0.0, now - detection.timestamp)
                current_y = detection.y + self.latest_speed.vy * dt
                if current_y > self.settings.pickup_window_y[1]:
                    self.metrics.skipped_outside_workspace += 1
                else:
                    kept_pending.append(detection)
                continue
            predicted_pick_time, _, pick_position = prediction
            candidates.append((predicted_pick_time, detection, sorting_position))
            kept_pending.append(detection)

        self.pending_objects = kept_pending

        if not candidates:
            return None

        candidates.sort(key=lambda item: item[0])
        _, detection, sorting_position = candidates[0]
        self.pending_objects = [
            item for item in self.pending_objects if item.object_id != detection.object_id
        ]
        return self._build_pick_plan(detection, sorting_position, now)

    def mark_completed(self, plan: PickPlan) -> None:
        self.current_position = (
            plan.sorting_position[0],
            plan.sorting_position[1],
            self.settings.place_height,
        )
        self.metrics.completed_picks += 1

    def _resolve_sorting_position(self, object_type: str) -> Position3D | None:
        destination_name = self.settings.object_type_map.get(object_type, object_type)
        return self.settings.sorting_positions.get(destination_name)

    def _build_pick_plan(
        self,
        detection: ObjectDetection,
        sorting_position: Position3D,
        now: float,
    ) -> PickPlan:
        prediction = self._predict_pick_position(detection, self.latest_speed, now)
        if prediction is None:
            raise RuntimeError("Unable to build pick plan for an unreachable detection.")

        predicted_pick_time, pick_dispatch_time, pick_position = prediction
        goto_points = _build_goto_geometry(
            self.current_position,
            pick_position,
            self.settings,
        )
        goto_times = _build_goto_timing(
            self.current_position,
            goto_points,
            self.settings,
        )
        trajectory_goto = [
            TrajectoryPoint(point[0], point[1], point[2], e_value, duration)
            for point, e_value, duration in zip(
                goto_points,
                [0, 0, 0, 0, 0, 0, 0],
                goto_times,
            )
        ]

        pick_points = _build_pick_geometry(
            pick_position,
            sorting_position,
            self.settings,
            goto_points,
        )
        pick_times = _build_pick_timing(
            pick_position,
            pick_points,
            self.settings,
            goto_points,
        )
        trajectory_pick = [
            TrajectoryPoint(point[0], point[1], point[2], e_value, duration)
            for point, e_value, duration in zip(
                pick_points,
                [1, 1, 1, 1, 1, 1, 0],
                pick_times,
            )
        ]

        self.plan_counter += 1
        self.metrics.planned_picks += 1
        self.metrics.total_planning_latency += max(now - detection.timestamp, 0.0)
        self.metrics.planning_events += 1

        return PickPlan(
            plan_id=f"plan-{self.plan_counter:06d}",
            object_id=detection.object_id,
            object_type=detection.object_type,
            detected_at=detection.timestamp,
            source_position_2d=(detection.x, detection.y),
            cycle_start_position=self.current_position,
            assumed_speed=(self.latest_speed.vx, self.latest_speed.vy),
            predicted_pick_time=predicted_pick_time,
            pick_dispatch_time=pick_dispatch_time,
            predicted_pick_position_2d=(pick_position[0], pick_position[1], pick_position[2]),
            sorting_position=sorting_position,
            trajectory_goto=trajectory_goto,
            trajectory_pick=trajectory_pick,
            debug_info={
                "pick_position_3d": pick_position,
                "timing_formula": {
                    "t_p_real": pick_dispatch_time,
                    "t_p_theory": predicted_pick_time,
                    "robot_movement_delay_s": self.settings.robot_movement_delay_s,
                    "ethernet_delay_s": self.settings.ethernet_delay_s,
                },
                "robot_packets": [
                    _trajectory_packet(trajectory_goto, self.interpolar_points),
                    _trajectory_packet(trajectory_pick, self.interpolar_points),
                ],
            },
        )

    def _predict_pick_position(
        self,
        detection: ObjectDetection,
        speed_sample: SpeedSample,
        now: float,
    ) -> tuple[float, float, Position3D] | None:
        command_delay_s = self.settings.robot_movement_delay_s + self.settings.ethernet_delay_s
        guess_pick_time = now + max(self.settings.intercept_lead_time_s, command_delay_s)

        t_enter = detection.timestamp
        if speed_sample.vy > 0.001 and detection.y < self.settings.pickup_window_y[0]:
            t_enter = detection.timestamp + (self.settings.pickup_window_y[0] - detection.y) / speed_sample.vy
            guess_pick_time = max(guess_pick_time, t_enter)

        predicted_x = detection.x
        predicted_y = detection.y
        for _ in range(6):
            dt = max(0.0, guess_pick_time - detection.timestamp)
            predicted_x = detection.x + speed_sample.vx * dt
            predicted_y = detection.y + speed_sample.vy * dt
            pick_position = (predicted_x, predicted_y, self.settings.pickup_height)
            # Only return None if the object has already passed the downstream boundary
            # of the workspace, or if it is out of bounds horizontally (X).
            if (
                predicted_y > self.settings.pickup_window_y[1]
                or predicted_x < self.settings.pickup_window_x[0]
                or predicted_x > self.settings.pickup_window_x[1]
            ):
                return None
            goto_points = _build_goto_geometry(
                self.current_position,
                pick_position,
                self.settings,
            )
            goto_times = _build_goto_timing(
                self.current_position,
                goto_points,
                self.settings,
            )
            new_guess = now + sum(goto_times) + command_delay_s
            new_guess = max(new_guess, t_enter)
            if abs(new_guess - guess_pick_time) < 0.01:
                guess_pick_time = new_guess
                break
            guess_pick_time = new_guess
        dt = max(0.0, guess_pick_time - detection.timestamp)
        predicted_x = detection.x + speed_sample.vx * dt
        predicted_y = detection.y + speed_sample.vy * dt
        pick_position = (predicted_x, predicted_y, self.settings.pickup_height)
        if not self._within_workspace(pick_position):
            return None
        pick_dispatch_time = guess_pick_time - command_delay_s
        return guess_pick_time, pick_dispatch_time, pick_position

    def _within_workspace(self, position: Position3D) -> bool:
        return (
            self.settings.pickup_window_x[0] <= position[0] <= self.settings.pickup_window_x[1]
            and self.settings.pickup_window_y[0] <= position[1] <= self.settings.pickup_window_y[1]
        )


def _coerce_position3d(raw_value: Any, fallback: Position3D) -> Position3D:
    if not isinstance(raw_value, (list, tuple)) or len(raw_value) != 3:
        return fallback
    return float(raw_value[0]), float(raw_value[1]), float(raw_value[2])


def _coerce_vector2d(raw_value: Any, fallback: tuple[float, float]) -> tuple[float, float]:
    if not isinstance(raw_value, (list, tuple)) or len(raw_value) != 2:
        return fallback
    return float(raw_value[0]), float(raw_value[1])


def _coerce_range(raw_value: Any, fallback: tuple[float, float]) -> tuple[float, float]:
    if not isinstance(raw_value, (list, tuple)) or len(raw_value) != 2:
        return fallback
    start, end = float(raw_value[0]), float(raw_value[1])
    return min(start, end), max(start, end)


def _sign(value: float) -> float:
    if value < 0.0:
        return -1.0
    return 1.0


def _segment_duration(start: Position3D, end: Position3D, settings: SchedulerSettings) -> float:
    horizontal = math.hypot(end[0] - start[0], end[1] - start[1])
    vertical = abs(end[2] - start[2])
    return max(
        0.08,
        horizontal / settings.nominal_xy_speed if settings.nominal_xy_speed > 0.0 else 0.0,
        vertical / settings.nominal_z_speed if settings.nominal_z_speed > 0.0 else 0.0,
    )


def _build_goto_geometry(
    start_position: Position3D,
    pick_position: Position3D,
    settings: SchedulerSettings,
    pre_pick_z: float | None = None,
) -> list[Position3D]:
    dx = pick_position[0] - start_position[0]
    dy = pick_position[1] - start_position[1]
    d = math.hypot(dx, dy)
    if d > 0.001:
        u_x = dx / d
        u_y = dy / d
    else:
        u_x = 0.0
        u_y = 0.0
    blend = min(d * 0.2, settings.corner_blend_xy)
    _pre_pick_z = pre_pick_z if pre_pick_z is not None else settings.pre_pick_height
    return [
        # P1: Vertical lift from start to slope_transition
        (start_position[0], start_position[1], settings.slope_transition_height),
        # P2: Diagonal slope up to clearance (XY moves blend toward pick)
        (start_position[0] + u_x * blend, start_position[1] + u_y * blend, settings.clearance_height),
        # P3: Flat at clearance, blend zone after slope
        (start_position[0] + u_x * 2 * blend, start_position[1] + u_y * 2 * blend, settings.clearance_height),
        # P4: Flat at clearance, approaching pick zone
        (pick_position[0] - u_x * 2 * blend, pick_position[1] - u_y * 2 * blend, settings.clearance_height),
        # P5: Flat at clearance, blend zone before descent
        (pick_position[0] - u_x * blend, pick_position[1] - u_y * blend, settings.clearance_height),
        # P6: Diagonal slope down to slope_transition (XY reaches pick)
        (pick_position[0], pick_position[1], settings.slope_transition_height),
        # P7: Vertical descent to pre-pick
        (pick_position[0], pick_position[1], _pre_pick_z),
    ]


def _build_goto_timing(
    start_position: Position3D,
    points: list[Position3D],
    settings: SchedulerSettings,
) -> list[float]:
    times: list[float] = []
    previous = start_position
    for point in points:
        times.append(_segment_duration(previous, point, settings))
        previous = point
    return times


def _build_pick_geometry(
    pick_position: Position3D,
    sorting_position: Position3D,
    settings: SchedulerSettings,
    goto_points: list[Position3D],
    place_z: float | None = None,
) -> list[Position3D]:
    dx = sorting_position[0] - pick_position[0]
    dy = sorting_position[1] - pick_position[1]
    d = math.hypot(dx, dy)
    if d > 0.001:
        u_x = dx / d
        u_y = dy / d
    else:
        u_x = 0.0
        u_y = 0.0
    blend = min(d * 0.2, settings.corner_blend_xy)
    _place_z = place_z if place_z is not None else settings.place_height
    return [
        # P1: Descend to pickup height (suction ON)
        pick_position,
        # P2: Vertical lift to slope_transition
        (pick_position[0], pick_position[1], settings.slope_transition_height),
        # P3: Diagonal slope up to clearance (XY moves blend toward sort)
        (pick_position[0] + u_x * blend, pick_position[1] + u_y * blend, settings.clearance_height),
        # P4: Flat at clearance, approaching sort zone
        (sorting_position[0] - u_x * 2 * blend, sorting_position[1] - u_y * 2 * blend, settings.clearance_height),
        # P5: Flat at clearance, blend zone before descent into sort
        (sorting_position[0] - u_x * blend, sorting_position[1] - u_y * blend, settings.clearance_height),
        # P6: Diagonal slope down to slope_transition at sort
        (sorting_position[0], sorting_position[1], settings.slope_transition_height),
        # P7: Vertical descent to place_height (precise placement, suction OFF)
        (sorting_position[0], sorting_position[1], _place_z),
    ]


def _build_pick_timing(
    pick_position: Position3D,
    points: list[Position3D],
    settings: SchedulerSettings,
    goto_points: list[Position3D],
) -> list[float]:
    times: list[float] = []
    previous = pick_position
    for index, point in enumerate(points):
        if index == 0:
            d_goto = goto_points[-1]
            times.append(_segment_duration(d_goto, point, settings))
        elif index == len(points) - 1:
            travel_time = _segment_duration(previous, point, settings)
            times.append(max(travel_time, settings.release_descent_time_s))
        else:
            times.append(_segment_duration(previous, point, settings))
        previous = point
    return times


def _trajectory_packet(points: list[TrajectoryPoint], interpolar_points: int) -> dict[str, Any]:
    return RobotPacket(
        commandID=COMMAND_ID["go_trajectory"],
        argument_number=len(points),
        argument_x=[point.x for point in points],
        argument_y=[point.y for point in points],
        argument_z=[point.z for point in points],
        argument_e=[point.e for point in points],
        argument_time=[point.time_s for point in points],
    ).to_dict(interpolar_points)


def _path_distance(start: Position3D, trajectory: list[TrajectoryPoint]) -> float:
    total = 0.0
    prev_x, prev_y, prev_z = start
    for point in trajectory:
        total += math.sqrt(
            (point.x - prev_x) ** 2 + (point.y - prev_y) ** 2 + (point.z - prev_z) ** 2
        )
        prev_x, prev_y, prev_z = point.x, point.y, point.z
    return total


def _build_evaluate_plan(
    pick_xy: tuple[float, float],
    place_xy: tuple[float, float],
    pick_z: float,
    place_z: float,
    settings: SchedulerSettings,
    plan_id: str,
    current_position: Position3D,
) -> PickPlan:
    pick_position: Position3D = (pick_xy[0], pick_xy[1], pick_z)
    sorting_position: Position3D = (place_xy[0], place_xy[1], place_z)

    # Maintain the same relative pre-pick offset as the nominal conveyor setup.
    pre_pick_z = pick_z + (settings.pre_pick_height - settings.pickup_height)

    goto_points = _build_goto_geometry(
        current_position, pick_position, settings, pre_pick_z=pre_pick_z
    )
    goto_times = _build_goto_timing(current_position, goto_points, settings)
    trajectory_goto = [
        TrajectoryPoint(point[0], point[1], point[2], e_value, duration)
        for point, e_value, duration in zip(
            goto_points, [0, 0, 0, 0, 0, 0, 0], goto_times
        )
    ]

    pick_points = _build_pick_geometry(
        pick_position, sorting_position, settings, goto_points, place_z=place_z
    )
    pick_times = _build_pick_timing(pick_position, pick_points, settings, goto_points)
    trajectory_pick = [
        TrajectoryPoint(point[0], point[1], point[2], e_value, duration)
        for point, e_value, duration in zip(
            pick_points, [1, 1, 1, 1, 1, 1, 0], pick_times
        )
    ]

    return PickPlan(
        plan_id=plan_id,
        object_id=plan_id,
        object_type="pcb1",
        detected_at=time.monotonic(),
        source_position_2d=(pick_xy[0], pick_xy[1]),
        cycle_start_position=current_position,
        assumed_speed=(0.0, 0.0),
        predicted_pick_time=0.0,
        pick_dispatch_time=0.0,
        predicted_pick_position_2d=(pick_position[0], pick_position[1], pick_position[2]),
        sorting_position=sorting_position,
        trajectory_goto=trajectory_goto,
        trajectory_pick=trajectory_pick,
    )


def _dispatch_evaluate_plan(
    plan: PickPlan,
    executor: EvaluateExecutor | None,
    metrics: EvaluateMetrics,
) -> "Position3D | None":
    # Print planned waypoints before dispatch — parsed by run_test.py for trajectory viz.
    for phase_name, trajectory in (("goto", plan.trajectory_goto), ("pick", plan.trajectory_pick)):
        print(
            "[TRAJ]",
            json.dumps(
                {
                    "plan_id": plan.plan_id,
                    "phase": phase_name,
                    "waypoints": [
                        {
                            "x": round(pt.x, 4),
                            "y": round(pt.y, 4),
                            "z": round(pt.z, 4),
                            "e": pt.e,
                            "t": round(pt.time_s, 4),
                        }
                        for pt in trajectory
                    ],
                },
                ensure_ascii=True,
            ),
            flush=True,
        )

    if executor is not None:
        return executor.execute_evaluate(plan, metrics)

    # Simulated path (no PLC): wall_time falls back to sum(argument_time).
    goto_end = (
        plan.trajectory_goto[-1].x,
        plan.trajectory_goto[-1].y,
        plan.trajectory_goto[-1].z,
    )
    phase_starts: list[Position3D] = [plan.cycle_start_position, goto_end]
    trajectories = (plan.trajectory_goto, plan.trajectory_pick)
    for phase_name, trajectory, phase_start in zip(
        ("goto", "pick"), trajectories, phase_starts
    ):
        wall = max(sum(point.time_s for point in trajectory), 0.001)
        distance = _path_distance(phase_start, trajectory)
        metrics.phase_wall_times.append(wall)
        metrics.phase_distances.append(distance)
        metrics.total_phase_wall_time_s += wall
        print(
            "[SIM]",
            json.dumps(
                {
                    "plan_id": plan.plan_id,
                    "phase": phase_name,
                    "wall_s": round(wall, 4),
                    "path_mm": round(distance, 2),
                },
                ensure_ascii=True,
            ),
            flush=True,
        )
    metrics.picks_completed += 1
    plan.status = "completed"
    return None  # no real pos_EE in simulated mode


def _run_evaluate_loop(
    settings: SchedulerSettings,
    interpolar_points: int,
    executor: Any,
    duration_s: float | None,
) -> None:
    box = settings.sorting_positions.get("pcb1")
    if box is None:
        raise RuntimeError(
            "evaluate scenario requires a 'pcb1' destination in config (top-level 'pcb1' key)."
        )
    if len(settings.accuracy_points) < 3:
        raise RuntimeError(
            f"evaluate scenario requires >= 3 accuracy_points, got {len(settings.accuracy_points)}."
        )

    box_xy = (float(box[0]), float(box[1]))
    box_pick_z = float(box[2])
    targets_3d: list[Position3D] = [
        (float(p[0]), float(p[1]), float(p[2])) for p in settings.accuracy_points[:3]
    ]

    eval_executor: EvaluateExecutor | None = None
    if executor is not None and hasattr(executor, "dispatch") and hasattr(executor, "request_status"):
        eval_executor = EvaluateExecutor(
            executor.dispatch,
            executor.request_status,
            interpolar_points=interpolar_points,
            position_tolerance_mm=settings.evaluate_position_tolerance_mm,
            status_poll_interval_s=settings.poll_interval_s,
            wait_timeout_s=settings.evaluate_wait_timeout_s,
            stability_window_s=settings.evaluate_stability_window_s,
            stability_mm=settings.evaluate_stability_mm,
        )

    metrics = EvaluateMetrics()
    current_position: Position3D = settings.home_position
    plan_counter = 0
    start_time = time.monotonic()

    print("[INFO] Running scheduler scenario: evaluate")
    print(f"[INFO] Box pickup XY: {box_xy}, Z: {box_pick_z}")
    print(f"[INFO] Accuracy targets: {targets_3d}")
    print(
        f"[INFO] Position tolerance: {settings.evaluate_position_tolerance_mm} mm, "
        f"wait timeout: {settings.evaluate_wait_timeout_s} s"
    )
    if duration_s is None:
        print("[INFO] Continuous loop — press Ctrl-C to stop and print metrics.")
    else:
        print(f"[INFO] Will stop after {duration_s:.2f}s (or on Ctrl-C).")

    try:
        while True:
            if duration_s is not None and time.monotonic() - start_time >= duration_s:
                break

            # Phase A: box -> 3 accuracy targets (conveyor)
            for target in targets_3d:
                plan_counter += 1
                plan = _build_evaluate_plan(
                    box_xy,
                    (target[0], target[1]),
                    pick_z=box_pick_z,
                    place_z=target[2],
                    settings=settings,
                    plan_id=f"eval-A-{plan_counter:06d}",
                    current_position=current_position,
                )
                actual_pos = _dispatch_evaluate_plan(plan, eval_executor, metrics)
                # Use actual pos_EE reported by PLC as next start; fall back to
                # planned sorting_position only in simulated mode (actual_pos is None).
                current_position = actual_pos if actual_pos is not None else plan.sorting_position

            # Phase B: 3 accuracy targets (conveyor) -> box
            for target in targets_3d:
                plan_counter += 1
                plan = _build_evaluate_plan(
                    (target[0], target[1]),
                    box_xy,
                    pick_z=target[2],
                    place_z=box_pick_z,
                    settings=settings,
                    plan_id=f"eval-B-{plan_counter:06d}",
                    current_position=current_position,
                )
                actual_pos = _dispatch_evaluate_plan(plan, eval_executor, metrics)
                current_position = actual_pos if actual_pos is not None else plan.sorting_position

            metrics.cycles_completed += 1
            print(
                "[CYCLE]",
                json.dumps(metrics.as_dict(), ensure_ascii=True),
                flush=True,
            )
    except KeyboardInterrupt:
        print("\n[INFO] Evaluate scenario interrupted by operator.")
    finally:
        if eval_executor is not None:
            eval_executor.close()

    print("[INFO] Evaluate metrics:", json.dumps(metrics.as_dict(), ensure_ascii=True))


def run_scheduler_scenario(
    scenario_name: str,
    *,
    duration_s: float | None,
    interpolar_points: int,
    executor: SimulatedExecutor | RealRobotExecutor | None = None,
) -> None:
    if scenario_name not in SCENARIO_NAMES:
        known = ", ".join(sorted(SCENARIO_NAMES))
        raise ValueError(f"Unknown scenario '{scenario_name}'. Available: {known}")

    config = load_config()
    settings = SchedulerSettings.from_config(config)

    if scenario_name == "evaluate":
        _run_evaluate_loop(settings, interpolar_points, executor, duration_s)
        return

    start_time = time.monotonic()
    image_processing = SimulatedImageProcessing(
        scenario_name,
        {
            "throughput_object_types": settings.throughput_object_types,
            "throughput_lanes": settings.throughput_lanes,
            "throughput_spawn_x": settings.throughput_spawn_x,
            "throughput_spawn_y": settings.throughput_spawn_y,
            "throughput_emit_interval_s": settings.throughput_emit_interval_s,
            "accuracy_emit_interval_s": settings.accuracy_emit_interval_s,
            "accuracy_points": settings.accuracy_points,
        },
        start_time,
    )
    if executor is None:
        executor = SimulatedExecutor(settings.log_path, settings.poll_interval_s)
        speed_source = SimulatedSpeedSource(scenario_name, settings, start_time)
    else:
        if hasattr(executor, "request_status"):
            speed_source = RealSpeedSource(executor.request_status, scenario_name)
        else:
            speed_source = SimulatedSpeedSource(scenario_name, settings, start_time)
    scheduler = PickScheduler(settings, interpolar_points)

    print(f"[INFO] Running scheduler scenario: {scenario_name}")
    print(f"[INFO] Fixed PLC slot count: {interpolar_points}")
    if duration_s is None:
        print("[INFO] Scenario will run until interrupted")
    else:
        print(f"[INFO] Scenario duration: {duration_s:.2f}s")

    deadline = None if duration_s is None else start_time + duration_s
    try:
        while True:
            now = time.monotonic()
            if deadline is not None and now >= deadline:
                break

            detections = image_processing.poll(now)
            scheduler.ingest_detections(detections)
            scheduler.prune_stale(now)
            scheduler.update_speed(speed_source.sample(now))

            plan = scheduler.plan_next(now)
            if plan is not None:
                print("[PLAN]", json.dumps(plan.to_summary(), ensure_ascii=True))
                try:
                    executor.execute(
                        plan,
                        log_samples=scenario_name == "test_accuracy",
                        real_time=False,
                        scenario_name=scenario_name,
                    )
                except Exception as exc:
                    plan.status = "failed"
                    print(f"[ERROR] scheduler execution failed: {exc}")
                    break
                scheduler.mark_completed(plan)

            time.sleep(settings.poll_interval_s)
    except KeyboardInterrupt:
        print("\n[INFO] Scheduler scenario interrupted by user")

    print("[INFO] Scheduler metrics:", json.dumps(scheduler.metrics.as_dict(), ensure_ascii=True))
