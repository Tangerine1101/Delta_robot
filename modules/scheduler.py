from __future__ import annotations

import json
import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

from modules.EthernetCom import COMMAND_ID, RobotPacket, load_config
from modules.conveyor import (
    BeltPositionTracker,
    BeltTracker,
    ConveyorFrame,
    TrackedObject,
    UVWindow,
)
from modules.image_processing import ObjectDetection, SimulatedImageProcessing, VisionImageProcessing


SCENARIO_NAMES = {"test_accuracy", "test_throughput", "evaluate", "production",
                  "test_conveyor", "test_vision_only"}

Position3D = tuple[float, float, float]


@dataclass(frozen=True)
class SpeedSample:
    vx: float
    vy: float
    timestamp: float
    # Belt position along +u in C-frame (mm). Used as the anchor reference for
    # belt-position dead reckoning. May be 0.0 if no belt-position source is wired up.
    position_mm: float = 0.0
    # Scalar belt speed along +u in C-frame (mm/s). Phase 1 keeps both the
    # scalar and the projected (vx, vy) so callers can pick whichever they need.
    speed_uv: float = 0.0


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
    # C-frame anchor used for late-dispatch re-prediction (set at plan build time)
    object_uv_anchor: tuple[float, float] = (0.0, 0.0)
    belt_pos_anchor: float = 0.0
    # Rotation angle for the 4th-DOF Siemens suction cup (degrees).
    rotate_deg: float = 0.0

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
            "rotate_deg": round(self.rotate_deg, 2),
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
    workspace_window_uv: UVWindow            # (u_min, u_max, v_min, v_max) on belt
    camera_window_uv: UVWindow
    conveyor_length_mm: float
    conveyor_position_scale_mm: float   # multiply incoming conveyor_position (cm) by this to get mm
    object_dimensions: dict[str, tuple[float, float]]   # type -> (w_mm, h_mm)
    accuracy_points: list[Position3D]
    # Optional C-frame (u, v, z) test points inside workspace_window_uv. When
    # provided, the evaluate scenario transforms these into R-frame XYZ via F so
    # the trajectory tracks the physical workspace regardless of belt calibration.
    accuracy_points_uv: list[Position3D]
    # (u, v) spawn positions used only by test_accuracy SimulatedImageProcessing.
    # Must lie inside workspace_window_uv so objects are pickable at belt speed=0.
    accuracy_spawn_uv: list[tuple[float, float]]
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
    # Minimum displacement from the phase start before the mechanical-stability
    # fallback is armed.  Prevents accepting "stable" at the starting position
    # if the PLC servo start latency exceeds the stability window.
    evaluate_stability_arm_mm: float = 3.0
    # Offset added to the vision angle_deg before sending rotate_absolute.
    rotate_offset_deg: float = 0.0
    # Fixed belt speed (mm/s) for test_conveyor scenario.
    test_conveyor_belt_speed_mm_s: float = 50.0
    # PLC MC_Inter_Curve_Vel interpolator limits (mm/s, mm/s^2) and the State-10
    # soft-start duration (s). Used to compute EXACT trajectory times that mirror
    # the PLC, instead of the crude distance/nominal_speed estimate. Defaults from
    # doc/PLC_Program_description/MC_inter_curve_vel.md (V_max=300, A=D=1000) and
    # main_logic.md (20-cycle / 80 ms soft start).
    interp_v_max: float = 300.0
    interp_a_max: float = 1000.0
    interp_d_max: float = 1000.0
    interp_soft_start_s: float = 0.08
    # S-curve shape-compensation factor (MC §3.2.2): t_acc = factor * V/A. 1.5 for
    # the PLC's 4th-order polynomial (bell-shaped accel needs 50% more time than a
    # constant-accel ramp to hit A_max). Exposed so the model can be matched to the
    # real mechanism via the speed_tuning validation.
    interp_scurve_shape_factor: float = 1.5

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
        conveyor_raw = getattr(config, "conveyor", {}) or {}
        interpolator_raw = scheduler_raw.get("interpolator", {}) or {}
        raw_object_types = dict(getattr(config, "object_types", {}) or {})
        object_type_map: dict[str, str] = {}
        object_thickness_mm: dict[str, float] = {}
        object_dimensions: dict[str, tuple[float, float]] = {}
        sorting_positions: dict[str, Position3D] = {}
        for object_type, type_info in raw_object_types.items():
            if isinstance(type_info, dict):
                destination_name = str(type_info.get("destination", object_type))
                object_thickness_mm[object_type] = float(type_info.get("thickness_mm", 0.0))
                object_dimensions[object_type] = (
                    float(type_info.get("w", 0.0)),
                    float(type_info.get("h", 0.0)),
                )
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

        # Default accuracy_spawn_uv places objects inside workspace_window_uv
        # so test_accuracy can plan picks at belt speed = 0.
        default_spawn_uv = [[470.0, 40.0], [520.0, 10.0], [560.0, -30.0]]
        accuracy_spawn_uv: list[tuple[float, float]] = [
            (float(pt[0]), float(pt[1]))
            for pt in scheduler_raw.get("accuracy_spawn_uv", default_spawn_uv)
            if isinstance(pt, (list, tuple)) and len(pt) >= 2
        ]
        if not accuracy_spawn_uv:
            accuracy_spawn_uv = [(float(p[0]), float(p[1])) for p in default_spawn_uv]

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
            workspace_window_uv=_coerce_uv_window(
                conveyor_raw.get("workspace_window_uv", [0.0, 200.0, -60.0, 60.0]),
                (0.0, 200.0, -60.0, 60.0),
            ),
            camera_window_uv=_coerce_uv_window(
                conveyor_raw.get("camera_window_uv", [0.0, 200.0, -75.0, 75.0]),
                (0.0, 200.0, -75.0, 75.0),
            ),
            conveyor_length_mm=float(conveyor_raw.get("length_mm", 800.0)),
            conveyor_position_scale_mm=float(
                conveyor_raw.get("conveyor_position_scale_mm", 10.0)
            ),
            object_dimensions=object_dimensions,
            accuracy_points=accuracy_points,
            accuracy_points_uv=[
                _coerce_position3d(point, (500.0, 0.0, -310.0))
                for point in conveyor_raw.get("accuracy_points_uv", [])
            ],
            accuracy_spawn_uv=accuracy_spawn_uv,
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
            evaluate_stability_arm_mm=float(
                scheduler_raw.get("evaluate_stability_arm_mm", 3.0)
            ),
            rotate_offset_deg=float(scheduler_raw.get("rotate_offset_deg", 0.0)),
            test_conveyor_belt_speed_mm_s=float(
                scheduler_raw.get("test_conveyor_belt_speed_mm_s", 50.0)
            ),
            interp_v_max=float(interpolator_raw.get("v_max", 300.0)),
            interp_a_max=float(interpolator_raw.get("a_max", 1000.0)),
            interp_d_max=float(interpolator_raw.get("d_max", 1000.0)),
            interp_soft_start_s=float(interpolator_raw.get("soft_start_s", 0.08)),
            interp_scurve_shape_factor=float(
                interpolator_raw.get("scurve_shape_factor", 1.5)
            ),
        )
        settings.validate()
        return settings


def _default_belt_scalar_speed(default_xy: tuple[float, float]) -> float:
    """Pre-conveyor-frame migration the config stored an R-frame default speed.
    Treat its magnitude as the scalar belt speed along +u for backwards
    compatibility until a dedicated config key is added."""
    return math.hypot(default_xy[0], default_xy[1])


class SimulatedSpeedSource:
    """Produce a synthetic belt speed + synthetic encoder position in C-frame."""

    def __init__(
        self,
        scenario_name: str,
        settings: SchedulerSettings,
        start_time: float,
        frame: ConveyorFrame,
    ) -> None:
        self.scenario_name = scenario_name
        self.settings = settings
        self.start_time = start_time
        self.frame = frame
        self._last_sample_time: float | None = None
        self._integrated_position_mm: float = 0.0

    def sample(self, now: float) -> SpeedSample:
        # test_vision_only runs camera-only with a STATIC belt: the camera is the
        # authoritative position source, so report zero belt speed / position and
        # let detections stay anchored where they are seen (no dead-reckoning drift).
        if self.scenario_name in ("test_accuracy", "test_vision_only"):
            self._last_sample_time = now
            return SpeedSample(
                vx=0.0, vy=0.0, timestamp=now,
                position_mm=self._integrated_position_mm, speed_uv=0.0,
            )

        if self.scenario_name == "test_conveyor":
            scalar = self.settings.test_conveyor_belt_speed_mm_s
            if self._last_sample_time is not None:
                dt = max(0.0, now - self._last_sample_time)
                self._integrated_position_mm += scalar * dt
            self._last_sample_time = now
            vx, vy = self.frame.velocity_to_robot(scalar)
            return SpeedSample(
                vx=vx, vy=vy, timestamp=now,
                position_mm=self._integrated_position_mm, speed_uv=scalar,
            )

        elapsed = now - self.start_time
        band = int(elapsed // 4.0) % 3
        scale = [0.8, 1.0, 1.2][band]
        scalar = _default_belt_scalar_speed(self.settings.default_speed) * scale

        if self._last_sample_time is not None:
            dt = max(0.0, now - self._last_sample_time)
            self._integrated_position_mm += scalar * dt
        self._last_sample_time = now

        vx, vy = self.frame.velocity_to_robot(scalar)
        return SpeedSample(
            vx=vx, vy=vy, timestamp=now,
            position_mm=self._integrated_position_mm, speed_uv=scalar,
        )


class ConveyorSpeedSource:
    """Derive belt speed and position from the Siemens `conveyor_position` field (cm)."""

    def __init__(
        self,
        request_status,
        frame: ConveyorFrame,
        decoder: BeltPositionTracker,
        scenario_name: str = "",
        position_scale_mm: float = 10.0,
    ) -> None:
        self.request_status = request_status
        self.frame = frame
        self.decoder = decoder
        self.scenario_name = scenario_name
        self.position_scale_mm = float(position_scale_mm)
        # Latest raw PLC status dict (carries pos_EE / end_effector). Cached here
        # so the scheduler loop can read the robot pose for the web dashboard
        # without issuing a second PLC round-trip per loop.
        self.last_status: dict[str, Any] | None = None

    def sample(self, now: float) -> SpeedSample:
        if self.scenario_name == "test_accuracy":
            return SpeedSample(
                vx=0.0, vy=0.0, timestamp=now,
                position_mm=self.decoder.position_mm, speed_uv=0.0,
            )

        try:
            status = self.request_status()
        except Exception as exc:
            print(f"[WARN] ConveyorSpeedSource failed to read status: {exc}")
            status = None
        self.last_status = status

        if status is not None:
            conveyor_position = status.get("conveyor_position")
            if conveyor_position is not None:
                self.decoder.update(float(conveyor_position) * self.position_scale_mm, now)

        scalar = self.decoder.velocity_mm_per_s
        vx, vy = self.frame.velocity_to_robot(scalar)
        return SpeedSample(
            vx=vx, vy=vy, timestamp=now,
            position_mm=self.decoder.position_mm, speed_uv=scalar,
        )


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
        # A3: wait until the pick dispatch window to mirror RealRobotExecutor timing.
        remaining_s = plan.pick_dispatch_time - time.monotonic()
        if remaining_s > 0.0:
            time.sleep(remaining_s)

        # Sleep the remaining trajectory time (pick phase) so throughput reflects
        # real robot cycle time rather than completing instantly.
        pick_duration = sum(pt.time_s for pt in plan.trajectory_pick)
        if pick_duration > 0.0:
            time.sleep(pick_duration)

        if log_samples:
            self._log_plan_trace(plan, real_time=False, scenario_name=scenario_name)
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


class NullExecutor:
    """For test_vision_only: marks plans completed immediately without sending any
    robot trajectory commands.

    It still carries optional `dispatch` / `request_status` callables wired to a
    live PLC worker so the scheduler can (a) read the real Siemens
    `conveyor_position` via ConveyorSpeedSource and (b) send the belt speed
    command — all without moving the Omron robot. The presence of a
    `request_status` attribute is what makes `run_scheduler_scenario` pick
    ConveyorSpeedSource (real feedback) instead of SimulatedSpeedSource.
    """

    def __init__(self, dispatch=None, request_status=None) -> None:
        self.dispatch = dispatch
        self.request_status = request_status

    def execute(self, plan: PickPlan, *, log_samples=False, real_time=False, scenario_name="") -> None:
        plan.status = "completed"


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
        position_tolerance_mm: float = 5.0,
    ) -> None:
        self.dispatch = dispatch
        self.request_status = request_status
        self.interpolar_points = interpolar_points
        self.wait_margin_s = wait_margin_s
        self.status_poll_interval_s = max(status_poll_interval_s, 0.02)
        # pos_EE arrival tolerance for phase-completion gating (mm). Loose enough
        # to absorb encoder/pos_EE quantization noise; tight enough to confirm the
        # arm actually reached the commanded waypoint before the next phase.
        self.position_tolerance_mm = max(float(position_tolerance_mm), 0.0)
        # Optional: set by run_scheduler_scenario for late-dispatch re-prediction (B2).
        self.speed_source: Any = None
        self.frame: Any = None
        self.settings: "SchedulerSettings | None" = None

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
        pick_packet = packets[1]

        # --- goto phase ---
        goto_packet = packets[0]
        print(
            "[EXEC]",
            json.dumps(
                {"plan_id": plan.plan_id, "phase": "goto",
                 "commandID": goto_packet.get("commandID"),
                 "argument_number": goto_packet.get("argument_number")},
                ensure_ascii=True,
            ),
        )
        status = self.dispatch(goto_packet)
        if status is not None:
            print("[PLC]", json.dumps(status, ensure_ascii=True))
        self._wait_for_phase_completion(goto_packet)

        # --- pre-pick: wait for dispatch window then optionally re-predict ---
        self._wait_until_pick_dispatch(plan)
        try:
            rotate_pkg = {
                "commandID": COMMAND_ID["rotate_absolute"],
                "CommandID": COMMAND_ID["rotate_absolute"],
                "rotate": plan.rotate_deg,
                "speed": 0.0,
            }
            self.dispatch(rotate_pkg)
        except Exception as s_exc:
            print(f"[WARN] Failed to dispatch Siemens rotation: {s_exc}")

        # B2: re-predict pick position using latest encoder reading at dispatch time.
        pick_packet = self._repredicted_pick_packet(plan, pick_packet)
        if pick_packet is None:
            plan.status = "aborted"
            print(
                "[WARN]",
                json.dumps(
                    {"plan_id": plan.plan_id, "event": "pick_aborted_outside_workspace"},
                    ensure_ascii=True,
                ),
            )
            return

        print(
            "[EXEC]",
            json.dumps(
                {"plan_id": plan.plan_id, "phase": "pick",
                 "commandID": pick_packet.get("commandID"),
                 "argument_number": pick_packet.get("argument_number")},
                ensure_ascii=True,
            ),
        )
        status = self.dispatch(pick_packet)
        if status is not None:
            print("[PLC]", json.dumps(status, ensure_ascii=True))
        self._wait_for_phase_completion(pick_packet)
        plan.status = "completed"

    def _repredicted_pick_packet(
        self, plan: PickPlan, original_packet: dict[str, Any]
    ) -> dict[str, Any] | None:
        """Re-compute the pick waypoints using the latest encoder position.

        Returns the updated packet, or None if the updated pick position is
        outside the workspace window (plan must be aborted in that case).
        If speed_source / frame / settings are not wired up, returns the
        original packet unchanged.
        """
        if self.speed_source is None or self.frame is None or self.settings is None:
            return original_packet

        try:
            sample = self.speed_source.sample(time.monotonic())
        except Exception as exc:
            print(f"[WARN] late re-prediction: speed_source.sample failed: {exc}")
            return original_packet

        u_anchor, v_anchor = plan.object_uv_anchor
        u_now = u_anchor + (sample.position_mm - plan.belt_pos_anchor)
        command_delay_s = (
            self.settings.robot_movement_delay_s + self.settings.ethernet_delay_s
        )
        pick_z = self.settings.pickup_height
        # Starting point of the pick phase: the arm is parked at the last goto
        # waypoint (hover above the *old* predicted point). _build_pick_geometry /
        # _build_pick_timing expect Position3D tuples for goto_points (they index
        # goto_points[-1][0]); plan.trajectory_goto holds TrajectoryPoint objects,
        # so convert the last waypoint to a tuple.
        last_goto_pt = plan.trajectory_goto[-1]
        last_goto_pos: Position3D = (last_goto_pt.x, last_goto_pt.y, last_goto_pt.z)

        # Contact lead: time from issuing the pick command until the gripper
        # touches the part at Pos[0]=pickup. This is NOT just the 80 ms soft-start
        # — when the part has drifted, the arm must traverse horizontally from the
        # parked hover XY (last_goto_pos) to the new descent XY before dropping. We
        # budget that full pick-phase approach flight via the PLC interpolator model
        # (_trajectory_total_time over [hover -> descent], = soft_start + rest->rest
        # segment time), iterating because the descent point depends on the lead.
        # Mirrors the planning-time predictor (_predict_pick_position), which adds
        # the full goto flight time. Degrades to the old soft-start-only lead when
        # the traverse is ~0 (single-object case, arm already above the part).
        v_belt = sample.speed_uv
        approach_s = self.settings.interp_soft_start_s
        u_contact = u_now + v_belt * (command_delay_s + approach_s)
        for _ in range(6):
            u_contact = u_now + v_belt * (command_delay_s + approach_s)
            pick_xy = self.frame.to_robot(u_contact, v_anchor)
            candidate: Position3D = (pick_xy[0], pick_xy[1], pick_z)
            new_approach = _trajectory_total_time([last_goto_pos, candidate], self.settings)
            if abs(new_approach - approach_s) < 0.005:
                approach_s = new_approach
                break
            approach_s = new_approach
        u_contact = u_now + v_belt * (command_delay_s + approach_s)

        if not self.frame.is_in_window_uv(u_contact, v_anchor, self.settings.workspace_window_uv):
            return None

        pick_xy = self.frame.to_robot(u_contact, v_anchor)
        new_pick_position: Position3D = (pick_xy[0], pick_xy[1], pick_z)

        # Rebuild pick phase geometry from the parked goto waypoint as start point.
        new_pick_points = _build_pick_geometry(
            new_pick_position, plan.sorting_position, self.settings, [last_goto_pos]
        )
        new_pick_times = _build_pick_timing(
            new_pick_position, new_pick_points, self.settings, [last_goto_pos]
        )
        new_trajectory_pick = [
            TrajectoryPoint(pt[0], pt[1], pt[2], e_val, dur)
            for pt, e_val, dur in zip(
                new_pick_points, [1, 1, 1, 1, 1, 1, 0], new_pick_times
            )
        ]
        print(
            "[REPREDICT]",
            json.dumps(
                {
                    "plan_id": plan.plan_id,
                    "original_xy": [round(plan.predicted_pick_position_2d[0], 3),
                                    round(plan.predicted_pick_position_2d[1], 3)],
                    "updated_xy": [round(new_pick_position[0], 3), round(new_pick_position[1], 3)],
                    "delta_u_mm": round(u_contact - (u_anchor + (sample.position_mm - plan.belt_pos_anchor)), 3),
                },
                ensure_ascii=True,
            ),
        )
        return _trajectory_packet(new_trajectory_pick, self.interpolar_points)

    def _wait_until_pick_dispatch(self, plan: PickPlan) -> None:
        """Hold until the object actually reaches the workspace, then return.

        The precomputed `pick_dispatch_time` assumes the belt holds the speed
        seen at planning. Over the multi-second intercept wait the belt drifts
        (noisy encoder + imprecise speed command), so a fixed sleep dispatches
        early/late and the B2 re-prediction aborts as `outside_workspace`.
        Instead, poll the live belt position and return as soon as the object's
        contact point enters the window (or a safety deadline elapses) — the
        caller's re-prediction then builds the pick from the true position.

        Falls back to the legacy timed sleep when the live-feedback hooks
        (speed_source / settings) are not wired (e.g. simulated executors).
        """
        if self.speed_source is None or self.settings is None:
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
            return

        u_min, u_max, _v_min, _v_max = self.settings.workspace_window_uv
        command_delay_s = (
            self.settings.robot_movement_delay_s + self.settings.ethernet_delay_s
        )
        # Contact lead consistent with _repredicted_pick_packet: command delay plus
        # the full pick-phase approach flight (parked hover -> descent point), not
        # just the soft-start. Computed per-sample in the loop below because the
        # descent point (and therefore the traverse distance) moves with the belt.
        u_anchor, v_anchor = plan.object_uv_anchor
        pick_z = self.settings.pickup_height
        last_goto_pt = plan.trajectory_goto[-1]
        last_goto_pos: Position3D = (last_goto_pt.x, last_goto_pt.y, last_goto_pt.z)
        # Dispatch threshold = the planner's predicted pick u, so the object
        # arrives where the goto already parked the arm (minimal re-prediction
        # correction). Fall back to the window entry if the frame isn't wired.
        u_target = u_min
        if self.frame is not None:
            try:
                u_target, _ = self.frame.to_conveyor(
                    plan.predicted_pick_position_2d[0],
                    plan.predicted_pick_position_2d[1],
                )
            except Exception:
                u_target = u_min
        u_target = min(max(u_target, u_min), u_max)
        # Safety ceiling so a stalled/under-speed belt can never hang the loop.
        deadline = max(plan.pick_dispatch_time, time.monotonic()) + self.wait_margin_s
        while True:
            now = time.monotonic()
            try:
                sample = self.speed_source.sample(now)
            except Exception as exc:
                print(f"[WARN] pick-dispatch belt poll failed: {exc}")
                return
            u_now = u_anchor + (sample.position_mm - plan.belt_pos_anchor)
            # Project the contact point with the same approach-inclusive lead the
            # re-prediction will use, so the dispatch threshold matches the motion
            # model (and the timeout diagnostics report the true contact point).
            if self.frame is not None:
                approach_s = self.settings.interp_soft_start_s
                for _ in range(3):
                    u_c = u_now + sample.speed_uv * (command_delay_s + approach_s)
                    xy = self.frame.to_robot(u_c, v_anchor)
                    next_approach = _trajectory_total_time(
                        [last_goto_pos, (xy[0], xy[1], pick_z)], self.settings
                    )
                    if abs(next_approach - approach_s) < 0.005:
                        approach_s = next_approach
                        break
                    approach_s = next_approach
            else:
                approach_s = self.settings.interp_soft_start_s
            u_contact = u_now + sample.speed_uv * (command_delay_s + approach_s)
            if u_contact >= u_target:
                # Object has reached the planned pick point — dispatch now; the
                # re-prediction enforces the downstream bound and aborts on
                # genuine overshoot.
                return
            if now >= deadline:
                print(
                    "[WARN]",
                    json.dumps(
                        {
                            "plan_id": plan.plan_id,
                            "event": "pick_dispatch_timeout",
                            "u_contact_mm": round(u_contact, 2),
                            "u_min_mm": round(u_min, 2),
                        },
                        ensure_ascii=True,
                    ),
                )
                return
            time.sleep(self.status_poll_interval_s)

    def _phase_target(self, packet: dict[str, Any]) -> "Position3D | None":
        """Final commanded waypoint (R-frame XYZ) of a trajectory packet."""
        n = int(packet.get("argument_number", 0))
        xs = packet.get("argument_x") or []
        ys = packet.get("argument_y") or []
        zs = packet.get("argument_z") or []
        if n >= 1 and len(xs) >= n and len(ys) >= n and len(zs) >= n:
            try:
                return (float(xs[n - 1]), float(ys[n - 1]), float(zs[n - 1]))
            except (TypeError, ValueError):
                return None
        return None

    def _wait_for_phase_completion(self, packet: dict[str, Any]) -> None:
        """Block until the commanded motion of `packet` actually finishes.

        Primary signal: pos_EE convergence on the final commanded waypoint. The
        Omron firmware ignores argument_time (fixed-max-speed) and reports a
        stale task_state, so the old fixed timer burned the whole margin every
        phase and pushed the pick past its window. Polling pos_EE ends the phase
        the moment the arm arrives. A timer ceiling (expected_duration +
        wait_margin_s) caps the wait so a locked arm or missing pos_EE can never
        hang.
        """
        argument_number = int(packet.get("argument_number", 0))
        durations = list(packet.get("argument_time", []))[:argument_number]
        expected_duration_s = max(sum(float(value) for value in durations), 0.0)
        hard_deadline = time.monotonic() + expected_duration_s + self.wait_margin_s
        target = self._phase_target(packet)
        # Guard against a stale pos_EE (arm still at the previous waypoint, which
        # for back-to-back picks can sit within tolerance of the new target):
        # only accept "arrived" once the arm has first departed past tolerance.
        departed = False

        while True:
            now = time.monotonic()
            status = self.request_status()
            # Primary: arm has reached the final waypoint (after departing).
            if target is not None and isinstance(status, dict):
                pos = status.get("pos_EE")
                if isinstance(pos, (list, tuple)) and len(pos) >= 3:
                    try:
                        dx = float(pos[0]) - target[0]
                        dy = float(pos[1]) - target[1]
                        dz = float(pos[2]) - target[2]
                        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
                    except (TypeError, ValueError):
                        distance = None
                    if distance is not None:
                        if distance > self.position_tolerance_mm:
                            departed = True
                        elif departed:
                            return
            # Secondary (kept per CLAUDE.md §4.3): PLC reports idle.
            task_state = status.get("task_state") if isinstance(status, dict) else None
            if task_state is not None:
                try:
                    if int(task_state) == 0:
                        return
                except (TypeError, ValueError):
                    pass
            if now >= hard_deadline:
                return
            time.sleep(self.status_poll_interval_s)


class EvaluateExecutor:
    """Dispatch evaluate plans and gate phase progression on pos_EE feedback.

    The Omron PLC firmware drives motors at a fixed maximum speed and ignores
    argument_time, so wall-time measured here reflects the true mechanism speed.

    A background thread polls request_status() every `status_poll_interval_s`
    (wired to poll_interval_s from config, default 50 ms) and caches the
    result. A shared mutex guarantees that polling and packet dispatch never
    share the communication queue simultaneously.

    Stability-fallback arming: the mechanical-stability fallback is only armed
    after pos_EE has moved at least `stability_arm_mm` from the phase start.
    This prevents falsely accepting "stable" while the robot is still at the
    starting position before servo start latency has elapsed.
    """

    def __init__(
        self,
        dispatch,
        request_status,
        *,
        interpolar_points: int,
        position_tolerance_mm: float = 0.01,
        status_poll_interval_s: float = 0.05,
        wait_timeout_s: float = 10.0,
        stability_window_s: float = 0.4,
        stability_mm: float = 0.3,
        stability_arm_mm: float = 3.0,
    ) -> None:
        self._dispatch_fn = dispatch
        self._request_status_fn = request_status
        self.interpolar_points = interpolar_points
        self.position_tolerance_mm = float(position_tolerance_mm)
        self.status_poll_interval_s = max(float(status_poll_interval_s), 0.005)
        self.wait_timeout_s = float(wait_timeout_s)
        self.stability_window_s = max(float(stability_window_s), 0.0)
        self.stability_mm = max(float(stability_mm), 0.0)
        self.stability_arm_mm = max(float(stability_arm_mm), 0.0)

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
        initial_pos: Position3D | None = None   # pos_EE captured on first valid sample
        last_distance: float | None = None
        last_task_state: int | None = None
        min_distance: float | None = None
        idle_seen_at: float | None = None
        stability_armed: bool = False
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
                    if initial_pos is None:
                        initial_pos = last_pos
                    dx = px - target[0]
                    dy = py - target[1]
                    dz = pz - target[2]
                    last_distance = math.sqrt(dx * dx + dy * dy + dz * dz)
                    if min_distance is None or last_distance < min_distance:
                        min_distance = last_distance
                    if last_distance < self.position_tolerance_mm:
                        return last_pos

                    # Arm stability fallback once the robot has moved away from
                    # the start position by at least stability_arm_mm.
                    if not stability_armed and initial_pos is not None and self.stability_arm_mm > 0.0:
                        ax = px - initial_pos[0]
                        ay = py - initial_pos[1]
                        az = pz - initial_pos[2]
                        if math.sqrt(ax * ax + ay * ay + az * az) >= self.stability_arm_mm:
                            stability_armed = True

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
                # Only active after stability_armed (robot must have left start pos first).
                if (
                    stability_armed
                    and self.stability_mm > 0.0
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
                                        "stability_arm_mm": self.stability_arm_mm,
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
    def __init__(
        self,
        settings: SchedulerSettings,
        interpolar_points: int,
        frame: ConveyorFrame,
        tracker: BeltTracker,
    ) -> None:
        self.settings = settings
        self.interpolar_points = interpolar_points
        self.frame = frame
        self.tracker = tracker
        self.seen_object_ids: dict[str, float] = {}
        # Object ids already committed to a pick plan. Vision now re-emits the
        # same id every frame while the object is visible, so without this guard a
        # still-visible (already-planned) object would be re-created and re-picked.
        self.planned_object_ids: dict[str, float] = {}
        self.metrics = SchedulerMetrics()
        self.current_position: Position3D = settings.home_position
        self.latest_speed: SpeedSample | None = None
        self.plan_counter = 0

    def ingest_detections(self, detections: list[ObjectDetection], p_now: float) -> None:
        for detection in detections:
            # Skip detections for objects already committed to a pick plan — the
            # vision pipeline re-emits the same id every frame while it is visible.
            if detection.object_id in self.planned_object_ids:
                continue
            self.metrics.total_detections += 1
            self.tracker.ingest_detection(
                detection, p_now, object_dimensions=self.settings.object_dimensions
            )
            self.seen_object_ids[detection.object_id] = detection.timestamp
        self.metrics.queue_peak = max(
            self.metrics.queue_peak, len(list(self.tracker.objects()))
        )

    def update_speed(self, sample: SpeedSample) -> None:
        self.latest_speed = sample
        print(
            f"[SPEED] vx={sample.vx:.4f} vy={sample.vy:.4f} "
            f"p={sample.position_mm:.3f} t={sample.timestamp:.4f}",
            flush=True,
        )

    def prune_stale(self, now: float) -> None:
        if self.latest_speed is None:
            return
        removed = self.tracker.prune(self.latest_speed.position_mm, now)
        self.metrics.stale_drops += removed

        # Prune seen_object_ids to prevent memory leaks
        limit = now - self.settings.stale_timeout_s
        self.seen_object_ids = {
            obj_id: ts for obj_id, ts in self.seen_object_ids.items() if ts >= limit
        }
        # Prune the planned-pick guard the same way. Vision track ids are
        # monotonic, so an id will not be reused within the timeout window.
        self.planned_object_ids = {
            obj_id: ts for obj_id, ts in self.planned_object_ids.items() if ts >= limit
        }

    def plan_next(self, now: float) -> PickPlan | None:
        if self.latest_speed is None:
            return None
        if now - self.latest_speed.timestamp > self.settings.speed_timeout_s:
            return None

        sample = self.latest_speed
        candidates: list[tuple[float, TrackedObject, Position3D]] = []
        for obj in self.tracker.objects():
            sorting_position = self._resolve_sorting_position(obj.object_type)
            if sorting_position is None:
                self.metrics.skipped_unknown_type += 1
                continue

            prediction = self._predict_pick_position(obj, sample, now)
            if prediction is None:
                # If the object is already downstream of the workspace, drop it.
                u_now, _ = obj.current_uv(sample.position_mm)
                if u_now > self.settings.workspace_window_uv[1]:
                    self.metrics.skipped_outside_workspace += 1
                    self.tracker.remove(obj.object_id)
                continue
            predicted_pick_time, _, pick_position = prediction
            candidates.append((predicted_pick_time, obj, sorting_position))

        if not candidates:
            return None

        candidates.sort(key=lambda item: item[0])
        _, obj, sorting_position = candidates[0]
        plan = self._build_pick_plan(obj, sorting_position, now)
        self.tracker.remove(obj.object_id)
        self.planned_object_ids[obj.object_id] = now
        return plan

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
        obj: TrackedObject,
        sorting_position: Position3D,
        now: float,
    ) -> PickPlan:
        if self.latest_speed is None:
            raise RuntimeError("Cannot build pick plan without a current speed sample.")
        prediction = self._predict_pick_position(obj, self.latest_speed, now)
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
        self.metrics.total_planning_latency += max(now - obj.last_seen_at, 0.0)
        self.metrics.planning_events += 1

        rotate_deg = math.degrees(obj.rotation_rad) + self.settings.rotate_offset_deg

        return PickPlan(
            plan_id=f"plan-{self.plan_counter:06d}",
            object_id=obj.object_id,
            object_type=obj.object_type,
            detected_at=obj.last_seen_at,
            source_position_2d=obj.conveyor_uv,
            cycle_start_position=self.current_position,
            assumed_speed=(self.latest_speed.vx, self.latest_speed.vy),
            predicted_pick_time=predicted_pick_time,
            pick_dispatch_time=pick_dispatch_time,
            predicted_pick_position_2d=(pick_position[0], pick_position[1], pick_position[2]),
            sorting_position=sorting_position,
            trajectory_goto=trajectory_goto,
            trajectory_pick=trajectory_pick,
            object_uv_anchor=obj.conveyor_uv,
            belt_pos_anchor=self.latest_speed.position_mm,
            rotate_deg=rotate_deg,
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
        obj: TrackedObject,
        speed_sample: SpeedSample,
        now: float,
    ) -> tuple[float, float, Position3D] | None:
        """Iteratively solve for t_pick in the C-frame and project to R-frame.

        The object's anchor is fixed in C-frame; its u-coordinate at time t is
        `u_anchor + (p_now - p_anchor) + v_belt * (t - now)`. We pick t such
        that the resulting workspace u is inside the workspace window AND the
        robot can fly to (u, v) within (t - now) seconds.
        """
        command_delay_s = (
            self.settings.robot_movement_delay_s + self.settings.ethernet_delay_s
        )
        guess_pick_time = now + max(
            self.settings.intercept_lead_time_s, command_delay_s
        )

        u_anchor, v_anchor = obj.conveyor_uv
        p_now = speed_sample.position_mm
        u_now = u_anchor + (p_now - obj.belt_pos_anchor)
        v_now = v_anchor
        belt_speed = speed_sample.speed_uv

        u_min, u_max, v_min, v_max = self.settings.workspace_window_uv

        # If we're upstream of the workspace, require the object to first reach
        # u_min before any pick attempt can succeed.
        t_enter = now
        if belt_speed > 0.001 and u_now < u_min:
            t_enter = now + (u_min - u_now) / belt_speed
            guess_pick_time = max(guess_pick_time, t_enter)

        # Outside lateral bounds — no chance, the belt's u motion won't fix v.
        if v_now < v_min or v_now > v_max:
            return None

        for _ in range(6):
            dt_future = max(0.0, guess_pick_time - now)
            u_pick = u_now + belt_speed * dt_future
            v_pick = v_now
            if u_pick > u_max:
                return None
            pick_xy = self.frame.to_robot(u_pick, v_pick)
            pick_position = (pick_xy[0], pick_xy[1], self.settings.pickup_height)
            goto_points = _build_goto_geometry(
                self.current_position,
                pick_position,
                self.settings,
            )
            # Exact gototime via the PLC interpolator model (the Omron ignores
            # argument_time and runs at fixed V_max/A/D). The subsequent pick
            # command's own 80 ms soft-start descends Pos[0]=pre_pick -> pickup to
            # gripper contact, so add interp_soft_start_s once here.
            goto_total = _trajectory_total_time(goto_points, self.settings)
            new_guess = (
                now + command_delay_s + goto_total + self.settings.interp_soft_start_s
            )
            new_guess = max(new_guess, t_enter)
            if abs(new_guess - guess_pick_time) < 0.01:
                guess_pick_time = new_guess
                break
            guess_pick_time = new_guess

        dt_future = max(0.0, guess_pick_time - now)
        u_pick = u_now + belt_speed * dt_future
        v_pick = v_now
        if not self.frame.is_in_window_uv(
            u_pick, v_pick, self.settings.workspace_window_uv
        ):
            return None
        pick_xy = self.frame.to_robot(u_pick, v_pick)
        pick_position = (pick_xy[0], pick_xy[1], self.settings.pickup_height)
        pick_dispatch_time = guess_pick_time - command_delay_s
        return guess_pick_time, pick_dispatch_time, pick_position


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


def _coerce_uv_window(
    raw_value: Any,
    fallback: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    if not isinstance(raw_value, (list, tuple)) or len(raw_value) != 4:
        return fallback
    u_a, u_b, v_a, v_b = (float(v) for v in raw_value)
    return (min(u_a, u_b), max(u_a, u_b), min(v_a, v_b), max(v_a, v_b))


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


# ---------------------------------------------------------------------------
# Exact trajectory timing — a Python port of the PLC MC_Inter_Curve_Vel
# function block. See doc/PLC_Program_description/MC_inter_curve_vel.md (the
# section numbers below cite that file) and main_logic.md (Rungs 13-18 chain +
# Rung 21 t_total_estimate telemetry).
#
# These replace the crude distance/nominal_speed estimate as the basis for
# pick-time prediction: the Omron ignores argument_time and runs the interpolator
# at fixed V_max=300, A=D=1000, so this model matches the real motion far better.
# ---------------------------------------------------------------------------


def _segment_profile_time(
    length: float,
    v_start: float,
    v_end: float,
    v_max: float,
    a_max: float,
    d_max: float,
    shape_factor: float = 1.5,
) -> float:
    """Execution time (s) of ONE linear segment under the PLC velocity model.

    - both boundary velocities zero -> jerk-bounded S-curve (MC §3.2),
    - otherwise -> trapezoidal with triangular fallback (MC §3.3).

    `shape_factor` is the S-curve accel-shape compensation (1.5 for the PLC's
    4th-order polynomial, MC §3.2.2); it only affects the stop-and-go branch.
    """
    if length <= 1e-9:
        return 0.0
    if v_max <= 0.0 or a_max <= 0.0 or d_max <= 0.0:
        return 0.0

    # Stop-and-go S-curve: both ends at rest.
    if v_start <= 1e-9 and v_end <= 1e-9:
        # min-distance coefficient = 0.5 * shape_factor (MC Eq 16: S_acc = 0.5*V*t_acc
        # with t_acc = shape_factor*V/A).
        coef = 0.5 * shape_factor
        inv_sum = 1.0 / a_max + 1.0 / d_max
        l_min = coef * v_max * v_max * inv_sum  # MC Eq 16
        if length < l_min:
            v_peak = math.sqrt(length / (coef * inv_sum))  # MC Eq 17
        else:
            v_peak = v_max
        t_acc = shape_factor * v_peak / a_max  # MC Eq 8
        t_dec = shape_factor * v_peak / d_max  # MC Eq 14
        s_acc = 0.5 * v_peak * t_acc  # MC Eq 9
        s_dec = 0.5 * v_peak * t_dec
        s_run = length - s_acc - s_dec
        t_run = s_run / v_peak if (s_run > 0.0 and v_peak > 0.0) else 0.0
        return t_acc + max(0.0, t_run) + t_dec

    # Trapezoidal (non-zero boundary velocity). V_peak from MC Eq 28, with a
    # triangular fallback when the segment is too short to reach v_max.
    s_limit = (
        abs(v_max * v_max - v_start * v_start) / (2.0 * a_max)
        + abs(v_max * v_max - v_end * v_end) / (2.0 * d_max)
    )  # MC Eq 27
    if s_limit > length:
        v_peak = math.sqrt(
            (2.0 * a_max * d_max * length + d_max * v_start * v_start + a_max * v_end * v_end)
            / (a_max + d_max)
        )  # MC Eq 28
    else:
        v_peak = v_max
    # Safety clamps (MC §3.4.4): V_peak must not dip below the boundary velocities.
    v_peak = max(v_peak, v_start, v_end)
    t_acc = abs(v_peak - v_start) / a_max  # MC Eq 22
    t_dec = abs(v_peak - v_end) / d_max    # MC Eq 23
    s_acc = 0.5 * (v_start + v_peak) * t_acc  # MC Eq 24
    s_dec = 0.5 * (v_end + v_peak) * t_dec    # MC Eq 25
    s_run = length - s_acc - s_dec
    t_run = s_run / v_peak if (s_run > 0.0 and v_peak > 0.0) else 0.0
    return t_acc + max(0.0, t_run) + t_dec


def _corner_v_end(
    seg_start: Position3D,
    seg_mid: Position3D,
    seg_next: Position3D,
    v_start: float,
    v_max: float,
    a_max: float,
) -> float:
    """Blend exit velocity at seg_mid (MC §3.4).

    V_corner = V_max*cos(theta/2) via the half-angle identity, clamped by the
    reachability limit V_reach = sqrt(v_start^2 + 2*A*L1) and V_max.
    """
    v1 = (seg_mid[0] - seg_start[0], seg_mid[1] - seg_start[1], seg_mid[2] - seg_start[2])
    v2 = (seg_next[0] - seg_mid[0], seg_next[1] - seg_mid[1], seg_next[2] - seg_mid[2])
    l1 = math.sqrt(v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2])
    l2 = math.sqrt(v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2])
    if l1 <= 1e-9 or l2 <= 1e-9:
        cos_theta = 1.0
    else:
        cos_theta = (v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]) / (l1 * l2)
        cos_theta = max(-1.0, min(1.0, cos_theta))  # MC line 175 clamp
    v_corner = v_max * math.sqrt(max(0.0, (cos_theta + 1.0) / 2.0))  # MC Eq 30
    v_reach = math.sqrt(max(0.0, v_start * v_start + 2.0 * a_max * l1))  # MC Eq 33
    return min(v_corner, v_reach, v_max)  # MC Eq 34


def _trajectory_total_time(
    points: list[Position3D],
    settings: SchedulerSettings,
) -> float:
    """Exact execution time (s) of a go_trajectory packet under the PLC model.

    `points` are the packet waypoints Pos[0..N-1] (e.g. the 7 goto/pick points).
    The PLC daisy-chains MC_Inter_Curve_Vel instances over the N-1 segments:
    segments 0..N-2 blend (each exits at the look-ahead corner velocity, which
    becomes the next segment's entry velocity) and the final segment decelerates
    to a full stop. A one-time 80 ms soft-start (State 10) is added because the
    chain begins from rest at Pos[0]. The arm's pre-trajectory position is bridged
    to Pos[0] by that same soft-start, so it is not a separate timed segment.
    """
    v_max = settings.interp_v_max
    a_max = settings.interp_a_max
    d_max = settings.interp_d_max
    shape = settings.interp_scurve_shape_factor

    n_seg = len(points) - 1
    if n_seg <= 0:
        return 0.0

    total = settings.interp_soft_start_s  # State 10 soft-start, once (v_start = 0)
    v_start = 0.0
    for i in range(n_seg):
        a = points[i]
        b = points[i + 1]
        length = math.dist(a, b)
        if i == n_seg - 1:
            v_end = 0.0  # final segment: stop-and-go
        else:
            v_end = _corner_v_end(a, b, points[i + 2], v_start, v_max, a_max)
        total += _segment_profile_time(length, v_start, v_end, v_max, a_max, d_max, shape)
        v_start = v_end
    return total


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
        object_type="QFP",
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
    # Print planned waypoints before dispatch (evaluate trajectory viz / debugging).
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
        sim_duration = max(sum(point.time_s for point in trajectory), 0.001)
        distance = _path_distance(phase_start, trajectory)
        t0 = time.monotonic()
        # A4: sleep to reflect real robot cycle time rather than busy-looping.
        time.sleep(sim_duration)
        wall = time.monotonic() - t0
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
    box = settings.sorting_positions.get("QFP")
    if box is None:
        raise RuntimeError(
            "evaluate scenario requires a 'QFP' destination in config (top-level 'QFP' key)."
        )

    box_xy = (float(box[0]), float(box[1]))
    box_pick_z = float(box[2])

    # Prefer C-frame test points so the trajectory always lands inside the
    # configured workspace_window_uv regardless of how F's translation is
    # calibrated. Fall back to legacy R-frame accuracy_points only if no
    # accuracy_points_uv is provided.
    if settings.accuracy_points_uv and len(settings.accuracy_points_uv) >= 3:
        frame = ConveyorFrame()
        targets_3d: list[Position3D] = []
        for u, v, z in settings.accuracy_points_uv[:3]:
            x_r, y_r = frame.to_robot(u, v)
            targets_3d.append((x_r, y_r, z))
    elif len(settings.accuracy_points) >= 3:
        targets_3d = [
            (float(p[0]), float(p[1]), float(p[2]))
            for p in settings.accuracy_points[:3]
        ]
    else:
        raise RuntimeError(
            "evaluate scenario requires >= 3 accuracy_points_uv (preferred) "
            "or accuracy_points (legacy R-frame fallback)."
        )

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
            stability_arm_mm=settings.evaluate_stability_arm_mm,
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
    executor: SimulatedExecutor | RealRobotExecutor | NullExecutor | None = None,
    event_sink: "Callable[[str, dict[str, Any]], None] | None" = None,
    frame_register: "Callable[[Any], None] | None" = None,
    disable_native_window: bool = False,
) -> None:
    """Run a scheduler scenario.

    ``event_sink`` (optional): called as ``event_sink(type, payload)`` for each
    structured event ('status' / 'detect' / 'predict' / 'plan') so an in-process
    consumer (e.g. ``modules.interface.DashboardServer``) can render it live. The
    existing ``print(...)`` debug lines are always kept.
    ``frame_register`` (optional): called once with the live vision pipeline so
    the consumer can pull annotated frames for an MJPEG stream.
    """
    if scenario_name not in SCENARIO_NAMES:
        known = ", ".join(sorted(SCENARIO_NAMES))
        raise ValueError(f"Unknown scenario '{scenario_name}'. Available: {known}")

    config = load_config()
    settings = SchedulerSettings.from_config(config)

    if scenario_name == "evaluate":
        _run_evaluate_loop(settings, interpolar_points, executor, duration_s)
        return

    start_time = time.monotonic()
    vision_config = dict(getattr(config, "vision", {}) or {})
    # When the web dashboard owns the display, suppress the native cv2 window so
    # the two GUIs don't compete (the annotated frame still streams over MJPEG
    # because frame_register enables the web overlay).
    if disable_native_window:
        vision_config["show_window"] = False
    _vision_scenarios = ("production", "test_conveyor", "test_vision_only")

    # Scenarios that drive the real robot/belt and therefore require live PLC
    # feedback. test_vision_only is camera-only (robot idle, belt static) and may
    # run without a PLC, so it is intentionally excluded here.
    _plc_required_scenarios = ("production", "test_conveyor")

    # Fail fast (before opening the camera) if a belt scenario was started without
    # a live PLC executor — these scenarios must use real belt feedback.
    if scenario_name in _plc_required_scenarios and executor is None:
        raise RuntimeError(
            f"Scenario '{scenario_name}' requires live PLC conveyor_position feedback; "
            "do not use --simulate-executor for real scenarios."
        )

    if scenario_name in _vision_scenarios:
        image_processing: SimulatedImageProcessing | VisionImageProcessing = VisionImageProcessing(
            vision_config, start_time
        )
    else:
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
                "accuracy_spawn_uv": settings.accuracy_spawn_uv,
            },
            start_time,
        )
    # Let an in-process consumer (web dashboard) pull annotated frames for MJPEG.
    if frame_register is not None:
        frame_register(image_processing)

    frame = ConveyorFrame()
    tracker = BeltTracker(
        frame,
        workspace_window_uv=settings.workspace_window_uv,
        stale_timeout_s=settings.stale_timeout_s,
    )
    decoder = BeltPositionTracker()
    if executor is None:
        if scenario_name == "test_vision_only":
            executor: Any = NullExecutor()
        else:
            executor = SimulatedExecutor(settings.log_path, settings.poll_interval_s)
        speed_source: Any = SimulatedSpeedSource(
            scenario_name, settings, start_time, frame
        )
    else:
        if hasattr(executor, "request_status"):
            speed_source = ConveyorSpeedSource(
                executor.request_status, frame, decoder, scenario_name,
                position_scale_mm=settings.conveyor_position_scale_mm,
            )
        else:
            speed_source = SimulatedSpeedSource(
                scenario_name, settings, start_time, frame
            )
        # Wire speed_source, frame, settings into RealRobotExecutor for B2 re-prediction.
        if isinstance(executor, RealRobotExecutor):
            executor.speed_source = speed_source
            executor.frame = frame
            executor.settings = settings

    # Belt scenarios must use live PLC conveyor_position feedback — never a
    # fabricated belt speed. Fail loudly if the wiring fell back to SimulatedSpeedSource.
    # test_vision_only is camera-only (static belt) so it is allowed to use it.
    if scenario_name in _plc_required_scenarios and isinstance(speed_source, SimulatedSpeedSource):
        raise RuntimeError(
            f"Scenario '{scenario_name}' requires live PLC conveyor_position feedback; "
            "do not use --simulate-executor for real scenarios."
        )

    scheduler = PickScheduler(settings, interpolar_points, frame, tracker)

    print(f"[INFO] Running scheduler scenario: {scenario_name}")
    print(f"[INFO] Fixed PLC slot count: {interpolar_points}")
    if duration_s is None:
        print("[INFO] Scenario will run until interrupted")
    else:
        print(f"[INFO] Scenario duration: {duration_s:.2f}s")

    deadline = None if duration_s is None else start_time + duration_s
    _conveyor_speed_sent = False
    try:
        while True:
            now = time.monotonic()
            if deadline is not None and now >= deadline:
                break

            # Send conveyor speed command once at the start of the belt scenario.
            # test_vision_only keeps the robot idle and must NOT command the belt
            # to run — it only reads conveyor_position passively from the PLC.
            if scenario_name == "test_conveyor" and not _conveyor_speed_sent:
                _conveyor_speed_sent = True
                if hasattr(executor, "dispatch"):
                    try:
                        executor.dispatch({
                            "commandID": COMMAND_ID["change_speed"],
                            "CommandID": COMMAND_ID["change_speed"],
                            "rotate": 0.0,
                            "speed": settings.test_conveyor_belt_speed_mm_s,
                        })
                        print(f"[INFO] Conveyor speed set to {settings.test_conveyor_belt_speed_mm_s} mm/s")
                    except Exception as exc:
                        print(f"[WARN] Could not set conveyor speed: {exc}")

            # Sample speed FIRST so that ingest_detections can anchor detections
            # to the latest encoder reading.
            sample = speed_source.sample(now)
            scheduler.update_speed(sample)
            if event_sink is not None:
                status_payload = {
                    "scenario": scenario_name,
                    "vx": round(sample.vx, 3),
                    "vy": round(sample.vy, 3),
                    "speed_mm_s": round(math.hypot(sample.vx, sample.vy), 3),
                    "position_mm": round(sample.position_mm, 2),
                }
                # Robot end-effector pose for the dashboard charts. Available from
                # the live PLC status (real / test_vision_only / test_conveyor);
                # absent in --simulate-executor runs (no real pos_EE).
                last_status = getattr(speed_source, "last_status", None)
                pose = last_status.get("pos_EE") if isinstance(last_status, dict) else None
                if isinstance(pose, (list, tuple)) and len(pose) >= 3:
                    status_payload["x"] = round(float(pose[0]), 2)
                    status_payload["y"] = round(float(pose[1]), 2)
                    status_payload["z"] = round(float(pose[2]), 2)
                    if isinstance(last_status, dict) and "end_effector" in last_status:
                        status_payload["e"] = int(last_status.get("end_effector") or 0)
                event_sink("status", status_payload)
            detections = image_processing.poll(now)
            scheduler.ingest_detections(detections, sample.position_mm)

            # Emit a snapshot of every tracked object's real R-frame position so
            # the web dashboard (--interface) can show objects moving live on the belt.
            # Snapshot BEFORE prune so a freshly ingested object is reported at
            # least once even if prune is about to drop it this loop (e.g. it
            # already sits past workspace u_max).
            tracked_objs = []
            for obj in scheduler.tracker.objects():
                x_r, y_r = scheduler.tracker.current_position_R(obj, sample.position_mm)
                tracked_objs.append({
                    "id": obj.object_id,
                    "type": obj.object_type,
                    "x": round(x_r, 2),
                    "y": round(y_r, 2),
                })
            if tracked_objs:
                detect_payload = {
                    "t": round(now - start_time, 3),
                    "z": round(settings.pickup_height, 2),
                    "objects": tracked_objs,
                }
                print("[DETECT]", json.dumps(detect_payload, ensure_ascii=True), flush=True)
                if event_sink is not None:
                    event_sink("detect", detect_payload)

            scheduler.prune_stale(now)

            plan = scheduler.plan_next(now)
            if plan is not None:
                plan_summary = plan.to_summary()
                print("[PLAN]", json.dumps(plan_summary, ensure_ascii=True))
                if event_sink is not None:
                    event_sink("plan", plan_summary)
                predict_payload = {
                    "t": round(now - start_time, 3),
                    "x": round(plan.predicted_pick_position_2d[0], 2),
                    "y": round(plan.predicted_pick_position_2d[1], 2),
                    "z": round(plan.predicted_pick_position_2d[2], 2),
                }
                if scenario_name in _vision_scenarios:
                    print("[PREDICT]", json.dumps(predict_payload, ensure_ascii=True))
                if event_sink is not None:
                    event_sink("predict", predict_payload)
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

            # Pump the live camera window on the MAIN thread (Qt requirement).
            if hasattr(image_processing, "render_window"):
                if not image_processing.render_window():
                    print("\n[INFO] Vision window closed by user (q)")
                    break

            time.sleep(settings.poll_interval_s)
    except KeyboardInterrupt:
        print("\n[INFO] Scheduler scenario interrupted by user")
    finally:
        if hasattr(image_processing, "stop"):
            image_processing.stop()
        if hasattr(image_processing, "close_window"):
            image_processing.close_window()

    print("[INFO] Scheduler metrics:", json.dumps(scheduler.metrics.as_dict(), ensure_ascii=True))
