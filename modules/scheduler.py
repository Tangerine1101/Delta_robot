from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from modules.EthernetCom import COMMAND_ID, RobotPacket, load_config
from modules.image_processing import ObjectDetection, SimulatedImageProcessing


SCENARIO_NAMES = {"test_accuracy", "test_throughput"}

Position3D = tuple[float, float, float]


@dataclass(frozen=True)
class SpeedSample:
    speed: float
    timestamp: float


@dataclass(frozen=True)
class TrajectoryPoint:
    x: float
    y: float
    z: float
    e: float
    time_s: float

    def to_dict(self) -> dict[str, float]:
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
    assumed_speed: float
    predicted_pick_time: float
    predicted_pick_position_2d: tuple[float, float]
    sorting_position: Position3D
    trajectory_outbound: list[TrajectoryPoint]
    trajectory_inbound: list[TrajectoryPoint]
    status: str = "planned"
    debug_info: dict[str, Any] = field(default_factory=dict)

    def total_duration(self) -> float:
        return sum(point.time_s for point in self.trajectory_outbound + self.trajectory_inbound)

    def to_robot_packets(self, interpolar_points: int) -> list[dict[str, Any]]:
        return [
            _trajectory_packet(self.trajectory_outbound, interpolar_points),
            _trajectory_packet(self.trajectory_inbound, interpolar_points),
        ]

    def to_summary(self) -> dict[str, Any]:
        return {
            "plan_id": self.plan_id,
            "object_id": self.object_id,
            "object_type": self.object_type,
            "predicted_pick_time": round(self.predicted_pick_time, 3),
            "predicted_pick_position_2d": [
                round(self.predicted_pick_position_2d[0], 3),
                round(self.predicted_pick_position_2d[1], 3),
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


@dataclass(frozen=True)
class SchedulerSettings:
    home_position: Position3D
    clearance_height: float
    pickup_height: float
    place_height: float
    corner_blend_xy: float
    intercept_lead_time_s: float
    pickup_descent_time_s: float
    release_descent_time_s: float
    nominal_xy_speed: float
    nominal_z_speed: float
    stale_timeout_s: float
    speed_timeout_s: float
    poll_interval_s: float
    default_speed: float
    pickup_window_x: tuple[float, float]
    pickup_window_y: tuple[float, float]
    accuracy_points: list[Position3D]
    log_path: str
    object_type_map: dict[str, str]
    sorting_positions: dict[str, Position3D]
    throughput_object_types: list[str]
    throughput_lanes: list[float]
    throughput_spawn_x: float
    throughput_emit_interval_s: float
    accuracy_emit_interval_s: float

    @classmethod
    def from_config(cls, config: Any) -> "SchedulerSettings":
        scheduler_raw = getattr(config, "scheduler", {}) or {}
        object_type_map = dict(getattr(config, "object_types", {}) or {})
        sorting_positions: dict[str, Position3D] = {}
        for object_type, destination_name in object_type_map.items():
            raw_position = getattr(config, destination_name, None)
            if raw_position is None:
                continue
            sorting_positions[destination_name] = _coerce_position3d(raw_position, (0.0, 0.0, -210.0))

        accuracy_points = [
            _coerce_position3d(point, (0.0, 0.0, -220.0))
            for point in scheduler_raw.get(
                "accuracy_points",
                [
                    [40.0, -60.0, -220.0],
                    [0.0, 0.0, -220.0],
                    [-40.0, 60.0, -220.0],
                ],
            )
        ]

        return cls(
            home_position=_coerce_position3d(
                scheduler_raw.get("home_position", [0.0, 0.0, -180.0]),
                (0.0, 0.0, -180.0),
            ),
            clearance_height=float(scheduler_raw.get("clearance_height", -165.0)),
            pickup_height=float(scheduler_raw.get("pickup_height", -230.0)),
            place_height=float(scheduler_raw.get("place_height", -205.0)),
            corner_blend_xy=float(scheduler_raw.get("corner_blend_xy", 35.0)),
            intercept_lead_time_s=float(scheduler_raw.get("intercept_lead_time_s", 0.14)),
            pickup_descent_time_s=float(scheduler_raw.get("pickup_descent_time_s", 0.14)),
            release_descent_time_s=float(scheduler_raw.get("release_descent_time_s", 0.14)),
            nominal_xy_speed=float(scheduler_raw.get("nominal_xy_speed", 220.0)),
            nominal_z_speed=float(scheduler_raw.get("nominal_z_speed", 180.0)),
            stale_timeout_s=float(scheduler_raw.get("stale_timeout_s", 5.0)),
            speed_timeout_s=float(scheduler_raw.get("speed_timeout_s", 1.0)),
            poll_interval_s=float(scheduler_raw.get("poll_interval_s", 0.05)),
            default_speed=float(scheduler_raw.get("default_speed", 80.0)),
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
            sorting_positions=sorting_positions,
            throughput_object_types=list(scheduler_raw.get("throughput_object_types", ["object_A"])),
            throughput_lanes=[float(value) for value in scheduler_raw.get("throughput_lanes", [-60.0, 0.0, 60.0])],
            throughput_spawn_x=float(scheduler_raw.get("throughput_spawn_x", -180.0)),
            throughput_emit_interval_s=float(scheduler_raw.get("throughput_emit_interval_s", 0.35)),
            accuracy_emit_interval_s=float(scheduler_raw.get("accuracy_emit_interval_s", 0.8)),
        )


class SimulatedSpeedSource:
    def __init__(self, scenario_name: str, settings: SchedulerSettings, start_time: float) -> None:
        self.scenario_name = scenario_name
        self.settings = settings
        self.start_time = start_time

    def sample(self, now: float) -> SpeedSample:
        if self.scenario_name == "test_accuracy":
            return SpeedSample(speed=0.0, timestamp=now)

        elapsed = now - self.start_time
        band = int(elapsed // 4.0) % 3
        scale = [0.8, 1.0, 1.2][band]
        return SpeedSample(speed=self.settings.default_speed * scale, timestamp=now)


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
            ("outbound", plan.trajectory_outbound),
            ("inbound", plan.trajectory_inbound),
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


class PickScheduler:
    def __init__(self, settings: SchedulerSettings, interpolar_points: int) -> None:
        self.settings = settings
        self.interpolar_points = interpolar_points
        self.pending_objects: list[ObjectDetection] = []
        self.seen_object_ids: set[str] = set()
        self.metrics = SchedulerMetrics()
        self.current_position: Position3D = settings.home_position
        self.latest_speed: SpeedSample | None = None
        self.plan_counter = 0

    def ingest_detections(self, detections: list[ObjectDetection]) -> None:
        for detection in detections:
            self.metrics.total_detections += 1
            if detection.object_id in self.seen_object_ids:
                continue
            self.seen_object_ids.add(detection.object_id)
            self.pending_objects.append(detection)
        self.metrics.queue_peak = max(self.metrics.queue_peak, len(self.pending_objects))

    def update_speed(self, sample: SpeedSample) -> None:
        self.latest_speed = sample

    def prune_stale(self, now: float) -> None:
        kept: list[ObjectDetection] = []
        for detection in self.pending_objects:
            if now - detection.timestamp > self.settings.stale_timeout_s:
                self.metrics.stale_drops += 1
                continue
            kept.append(detection)
        self.pending_objects = kept

    def plan_next(self, now: float) -> PickPlan | None:
        if self.latest_speed is None:
            return None
        if now - self.latest_speed.timestamp > self.settings.speed_timeout_s:
            return None

        candidates: list[tuple[float, ObjectDetection, Position3D]] = []
        for detection in self.pending_objects:
            sorting_position = self._resolve_sorting_position(detection.object_type)
            if sorting_position is None:
                self.metrics.skipped_unknown_type += 1
                continue

            prediction = self._predict_pick_position(detection, self.latest_speed, now)
            if prediction is None:
                continue
            predicted_pick_time, pick_position = prediction
            candidates.append((predicted_pick_time, detection, sorting_position))

        if not candidates:
            return None

        candidates.sort(key=lambda item: item[0])
        _, detection, sorting_position = candidates[0]
        self.pending_objects = [
            item for item in self.pending_objects if item.object_id != detection.object_id
        ]
        return self._build_pick_plan(detection, sorting_position, now)

    def mark_completed(self, plan: PickPlan) -> None:
        self.current_position = plan.sorting_position
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

        predicted_pick_time, pick_position = prediction
        outbound_points = _build_outbound_geometry(
            self.current_position,
            pick_position,
            self.settings,
        )
        outbound_times = _build_outbound_timing(
            self.current_position,
            outbound_points,
            self.settings,
        )
        outbound = [
            TrajectoryPoint(point[0], point[1], point[2], e_value, duration)
            for point, e_value, duration in zip(
                outbound_points,
                [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                outbound_times,
            )
        ]

        inbound_points = _build_inbound_geometry(
            pick_position,
            sorting_position,
            self.settings,
        )
        inbound_times = _build_inbound_timing(
            pick_position,
            inbound_points,
            self.settings,
        )
        inbound = [
            TrajectoryPoint(point[0], point[1], point[2], e_value, duration)
            for point, e_value, duration in zip(
                inbound_points,
                [1.0, 1.0, 1.0, 1.0, 1.0, 0.0],
                inbound_times,
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
            assumed_speed=self.latest_speed.speed,
            predicted_pick_time=predicted_pick_time,
            predicted_pick_position_2d=(pick_position[0], pick_position[1]),
            sorting_position=sorting_position,
            trajectory_outbound=outbound,
            trajectory_inbound=inbound,
            debug_info={
                "pick_position_3d": pick_position,
                "robot_packets": [
                    _trajectory_packet(outbound, self.interpolar_points),
                    _trajectory_packet(inbound, self.interpolar_points),
                ],
            },
        )

    def _predict_pick_position(
        self,
        detection: ObjectDetection,
        speed_sample: SpeedSample,
        now: float,
    ) -> tuple[float, Position3D] | None:
        guess_pick_time = now + self.settings.intercept_lead_time_s
        predicted_x = detection.x
        for _ in range(6):
            predicted_x = detection.x + speed_sample.speed * max(0.0, guess_pick_time - detection.timestamp)
            pick_position = (predicted_x, detection.y, self.settings.pickup_height)
            if not self._within_workspace(pick_position):
                return None
            outbound_points = _build_outbound_geometry(
                self.current_position,
                pick_position,
                self.settings,
            )
            outbound_times = _build_outbound_timing(
                self.current_position,
                outbound_points,
                self.settings,
            )
            new_guess = now + sum(outbound_times)
            if abs(new_guess - guess_pick_time) < 0.01:
                guess_pick_time = new_guess
                break
            guess_pick_time = new_guess
        return guess_pick_time, (predicted_x, detection.y, self.settings.pickup_height)

    def _within_workspace(self, position: Position3D) -> bool:
        return (
            self.settings.pickup_window_x[0] <= position[0] <= self.settings.pickup_window_x[1]
            and self.settings.pickup_window_y[0] <= position[1] <= self.settings.pickup_window_y[1]
        )


def _coerce_position3d(raw_value: Any, fallback: Position3D) -> Position3D:
    if not isinstance(raw_value, (list, tuple)) or len(raw_value) != 3:
        return fallback
    return float(raw_value[0]), float(raw_value[1]), float(raw_value[2])


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
    return max(0.08, horizontal / settings.nominal_xy_speed + vertical / settings.nominal_z_speed)


def _build_outbound_geometry(
    start_position: Position3D,
    pick_position: Position3D,
    settings: SchedulerSettings,
) -> list[Position3D]:
    dx = pick_position[0] - start_position[0]
    dy = pick_position[1] - start_position[1]
    blend_x = min(abs(dx) * 0.5, settings.corner_blend_xy)
    blend_y = min(abs(dy) * 0.5, settings.corner_blend_xy)
    return [
        (start_position[0], start_position[1], settings.clearance_height),
        (start_position[0] + _sign(dx) * blend_x, start_position[1], settings.clearance_height),
        (
            start_position[0] + dx * 0.6,
            start_position[1] + _sign(dy) * blend_y,
            settings.clearance_height,
        ),
        (pick_position[0] - _sign(dx) * blend_x, pick_position[1], settings.clearance_height),
        (pick_position[0], pick_position[1], settings.clearance_height),
        pick_position,
    ]


def _build_outbound_timing(
    start_position: Position3D,
    points: list[Position3D],
    settings: SchedulerSettings,
) -> list[float]:
    times: list[float] = []
    previous = start_position
    for index, point in enumerate(points):
        if index == len(points) - 1:
            times.append(max(settings.pickup_descent_time_s, settings.intercept_lead_time_s))
        else:
            times.append(_segment_duration(previous, point, settings))
        previous = point
    return times


def _build_inbound_geometry(
    pick_position: Position3D,
    sorting_position: Position3D,
    settings: SchedulerSettings,
) -> list[Position3D]:
    dx = sorting_position[0] - pick_position[0]
    dy = sorting_position[1] - pick_position[1]
    blend_x = min(abs(dx) * 0.5, settings.corner_blend_xy)
    blend_y = min(abs(dy) * 0.5, settings.corner_blend_xy)
    return [
        (pick_position[0], pick_position[1], settings.clearance_height),
        (pick_position[0] + _sign(dx) * blend_x, pick_position[1], settings.clearance_height),
        (
            pick_position[0] + dx * 0.6,
            pick_position[1] + _sign(dy) * blend_y,
            settings.clearance_height,
        ),
        (sorting_position[0] - _sign(dx) * blend_x, sorting_position[1], settings.clearance_height),
        (sorting_position[0], sorting_position[1], settings.clearance_height),
        (sorting_position[0], sorting_position[1], settings.place_height),
    ]


def _build_inbound_timing(
    pick_position: Position3D,
    points: list[Position3D],
    settings: SchedulerSettings,
) -> list[float]:
    times: list[float] = []
    previous = pick_position
    for index, point in enumerate(points):
        if index == len(points) - 1:
            times.append(settings.release_descent_time_s)
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


def run_scheduler_scenario(
    scenario_name: str,
    *,
    duration_s: float | None,
    interpolar_points: int,
) -> None:
    if scenario_name not in SCENARIO_NAMES:
        known = ", ".join(sorted(SCENARIO_NAMES))
        raise ValueError(f"Unknown scenario '{scenario_name}'. Available: {known}")

    config = load_config()
    settings = SchedulerSettings.from_config(config)
    start_time = time.monotonic()
    image_processing = SimulatedImageProcessing(
        scenario_name,
        {
            "throughput_object_types": settings.throughput_object_types,
            "throughput_lanes": settings.throughput_lanes,
            "throughput_spawn_x": settings.throughput_spawn_x,
            "throughput_emit_interval_s": settings.throughput_emit_interval_s,
            "accuracy_emit_interval_s": settings.accuracy_emit_interval_s,
            "accuracy_points": settings.accuracy_points,
        },
        start_time,
    )
    speed_source = SimulatedSpeedSource(scenario_name, settings, start_time)
    scheduler = PickScheduler(settings, interpolar_points)
    executor = SimulatedExecutor(settings.log_path, settings.poll_interval_s)

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
                executor.execute(
                    plan,
                    log_samples=scenario_name == "test_accuracy",
                    real_time=scenario_name == "test_accuracy",
                    scenario_name=scenario_name,
                )
                scheduler.mark_completed(plan)

            time.sleep(settings.poll_interval_s)
    except KeyboardInterrupt:
        print("\n[INFO] Scheduler scenario interrupted by user")

    print("[INFO] Scheduler metrics:", json.dumps(scheduler.metrics.as_dict(), ensure_ascii=True))
