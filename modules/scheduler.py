from __future__ import annotations

import json
import math
import statistics
import threading
import time
from collections import deque
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, Callable

from modules.EthernetCom import COMMAND_ID, RobotPacket, load_config, wrap_rad
from modules.conveyor import (
    BeltPositionTracker,
    BeltTracker,
    ConveyorFrame,
    TrackedObject,
    UVWindow,
)
from modules.image_processing import ObjectDetection, SimulatedImageProcessing, VisionImageProcessing


SCENARIO_NAMES = {"test_accuracy", "test_acceptance", "test_throughput", "evaluate",
                  "production", "test_vision_only"}

# test_accuracy / test_acceptance: static fake objects at fixed accuracy points, no real
# board to grip. Both scenarios share suction-off + wave-gated spawning + the
# EvaluateExecutor real-hardware backend (see PickScheduler, SimulatedImageProcessing,
# main.py's executor selection).
_ACCURACY_SCENARIOS = ("test_accuracy", "test_acceptance")

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
    # C-frame anchor used by the live pick-position gate.
    object_uv_anchor: tuple[float, float] = (0.0, 0.0)
    belt_pos_anchor: float = 0.0
    # Post-grip target for the 4th-DOF Siemens suction cup: R-frame RADIANS,
    # [-pi, pi) (the wrap itself gives the shortest-way rotation). Converted to
    # wire degrees only at the IPC boundary (main.py _worker).
    rotate_rad: float = 0.0
    # Modeled park->contact descent time (s). The pick gate must fire this much
    # earlier (on top of the dispatch delay) so the oblique descent lands on the
    # object; also logged for T_delay calibration.
    descend_time_s: float = 0.0

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
            "rotate_deg": round(math.degrees(self.rotate_rad), 2),
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
class RealtimeState:
    tracker: BeltTracker
    frame: ConveyorFrame
    ipc_lock: threading.Lock = field(default_factory=threading.Lock)
    state_lock: threading.Lock = field(default_factory=threading.Lock)
    stop_event: threading.Event = field(default_factory=threading.Event)
    latest_speed: SpeedSample | None = None
    belt_position_mm: float = 0.0
    belt_speed_mm_s: float = 0.0
    robot_pose: Position3D | None = None
    end_effector: int | None = None
    claimed_object_ids: set[str] = field(default_factory=set)
    # T_delay = robot_movement_delay_s + ethernet_delay_s; sizes the pick-gate lead
    # offset (v * T_delay). Constant config, set once at construction.
    command_delay_s: float = 0.0
    # Last belt speed setpoint committed by the adaptive controller (deadband state).
    belt_speed_setpoint_mm_s: float = 0.0
    # Adaptive belt speed (doc/basis-theory.md §6.5): continuously refreshed by the
    # perception thread every tick from live density; committed opportunistically by
    # _commit_adaptive_speed whenever the pick gate is not imminent. Constant config
    # below, set once.
    belt_speed_target_mm_s: float = 0.0
    belt_speed_deadband_mm_s: float = 0.0
    adaptive_speed_enabled: bool = False
    # Max |Δv| per change_speed commit (mm/s); each ramp then settles within
    # max_step / belt_accel_mm_s2 seconds. 0 disables the rate limit.
    belt_speed_max_step_mm_s: float = 0.0
    # Live PLC speed_current feedback (mm/s), refreshed by the perception thread.
    # Used to close the loop: if the commanded setpoint and the measured speed
    # diverge long after the ramp should have settled, the setpoint is re-sent.
    belt_speed_measured_mm_s: float | None = None
    # monotonic time of the last dispatched change_speed (throttle + resync timing).
    last_speed_commit_monotonic: float = 0.0
    # True while the pick gate is imminent (object within the critical distance of
    # the fire threshold): speed commits are suppressed so the belt is steady when
    # the gate fires. Set/cleared by RealtimePickExecutor.
    gate_critical: bool = False
    # Static compensation for gate sampling latency (gate poll /2 + perception
    # tick /2), added to command_delay_s when sizing the pick-gate lead offset.
    gate_sampling_latency_s: float = 0.0
    # Rolling average pick-cycle wall time (s), for the web dashboard's Performance
    # card. Written by the main thread after each executor.execute() call, read by
    # the perception thread when building the "status" event — hence state_lock.
    recent_pick_cycle_s: float = 0.0
    # Live Siemens suction-cup angle feedback (R-frame DEGREES, [-180,180)),
    # refreshed by the perception thread from the status packet. None until the
    # first Siemens status arrives (and always None in simulation). Read by the
    # [ROTATE] calibration log and the rotate_home_tolerance_deg warning.
    rotate_current_deg: float | None = None


@dataclass(frozen=True)
class RealtimePickCandidate:
    obj: TrackedObject
    sorting_position: Position3D
    predicted_pick_time: float
    pick_dispatch_time: float
    pick_position: Position3D
    u_now: float
    u_pick: float
    is_danger: bool
    cycle_distance_mm: float


@dataclass
class EvaluateMetrics:
    cycles_completed: int = 0
    picks_completed: int = 0
    total_phase_wall_time_s: float = 0.0
    phase_wall_times: list[float] = field(default_factory=list)
    phase_distances: list[float] = field(default_factory=list)
    # Interpolator-modeled duration of each phase (parallel to phase_wall_times),
    # so the CONFIG-SUGGEST tool can back out the fixed dispatch/servo overhead.
    phase_modeled_times: list[float] = field(default_factory=list)
    position_wait_timeouts: int = 0
    position_stability_accepts: int = 0

    def config_suggestions(
        self,
        round_trip_avg_s: float,
        nominal_xy_speed: float,
        simulated: bool,
    ) -> dict[str, Any]:
        """Config values computed from this run's runtime measurements, so the
        oversized config can be re-tuned from data instead of by hand. On real
        hardware prefer the latency-probe Omron figure for ethernet_delay_s."""
        overheads = [
            w - m
            for w, m in zip(self.phase_wall_times, self.phase_modeled_times)
        ]
        overhead_mean = statistics.fmean(overheads) if overheads else 0.0
        speeds = [
            d / t for d, t in zip(self.phase_distances, self.phase_wall_times) if t > 0.0
        ]
        eff_speed = statistics.fmean(speeds) if speeds else 0.0
        return {
            "simulated": simulated,
            "samples": len(self.phase_wall_times),
            "scheduler.robot_movement_delay_s": round(max(0.0, overhead_mean), 4),
            "scheduler.ethernet_delay_s": round(round_trip_avg_s, 4),
            "measured_effective_xy_speed_mm_s": round(eff_speed, 1),
            "configured_nominal_xy_speed": round(nominal_xy_speed, 1),
            "note": (
                "SIMULATED run — do NOT paste into a hardware config"
                if simulated else
                "prefer modules.latency_probe Omron mean for ethernet_delay_s"
            ),
        }

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
    conveyor_position_scale_mm: float   # multiply incoming conveyor_position by this to get mm (1.0: PLC reports mm)
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
    # Bin orientation for the post-grip normalisation (R-frame RADIANS; the
    # config key `rotate_offset_deg` stays in degrees for readability and is
    # converted once in from_config).
    rotate_offset_rad: float = 0.0
    # Direction of the Siemens suction axis relative to the R-frame CCW
    # convention: +1.0 = matches, -1.0 = inverted. Calibrate with
    # `python3 -m modules.test_rotate` (config key `rotate_sign`).
    rotate_sign: float = 1.0
    # Warn when the cup is not back at 0 (within this many degrees) by the time
    # the pick fires. 0 disables the check. Warn-only: the positional gate must
    # never be delayed by the rotation axis.
    rotate_home_tolerance_deg: float = 0.0
    # test_acceptance stops after exactly this many completed picks (ignores --duration).
    test_acceptance_cycles: int = 9
    # Initial/static belt speed (mm/s): used as production's startup setpoint before
    # the adaptive controller takes over (and as the static belt speed if adaptive
    # control is disabled).
    belt_speed_static_mm_s: float = 50.0
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
    # --- Adaptive belt speed (doc/basis-theory.md §6). Opt-in; when disabled the
    # belt keeps the static belt_speed_static_mm_s. The controller holds the
    # presentation rate lambda_nom = headroom / pick_cycle_s by setting belt speed
    # INVERSELY to product density: v = clamp(lambda_nom * L_meas / N, v_min, v_cap).
    adaptive_speed_enabled: bool = False
    pick_cycle_s: float = 2.0              # t_pick -> mu_max = 1 / pick_cycle_s
    pick_transit_min_s: float = 2.0        # t_transit -> v_cap = L / pick_transit_min_s
    belt_speed_headroom: float = 0.75      # k -> lambda_nom = k * mu_max
    belt_speed_min_mm_s: float = 30.0      # v_min hardware floor
    belt_speed_max_mm_s: float = 0.0       # operational cap (<=0 = use v_cap from geometry)
    belt_speed_hw_max_mm_s: float = 200.0  # absolute hardware safety limit
    belt_speed_deadband_mm_s: float = 8.0  # Delta_min anti-thrash
    belt_density_length_mm: float = 0.0    # L_meas override; <=0 -> derive = u_max
    belt_accel_mm_s2: float = 22.31        # a_nom (informational ramp-settle log only)
    belt_ramp_s: float = 0.25              # T_ramp (informational only)
    # Max |Δv| per change_speed commit: each ramp then settles within
    # max_step / a_nom (≈0.9 s at 20 mm/s) instead of the multi-second ramps the
    # raw hyperbolic law produces at small N (54 mm/s N=1↔2 jump). 0 disables.
    belt_speed_max_step_mm_s: float = 20.0
    # BeltPositionTracker velocity EMA alpha (conveyor.velocity_ema_alpha) —
    # lower = smoother belt-speed estimate against encoder quantisation noise.
    belt_velocity_ema_alpha: float = 0.4
    # Physical XY reach radius (mm) around the robot origin, mirrored from the
    # top-level limit_radius_xy. Used to reject an oblique-descent park point
    # that would fall outside the mechanism's reach before it is dispatched.
    limit_radius_xy: float = 180.0
    # Oblique (belt-tracking) descent. When OFF (default) the pick descends
    # straight down to the predicted point (known-good). When ON, only the pick-
    # phase contact point is slanted DOWNSTREAM by v*t_d so the cup tracks the
    # object during the short pre_pick->pickup drop; the goto/park is unchanged.
    oblique_descent_enabled: bool = False

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
            intercept_lead_time_s=float(scheduler_raw.get("intercept_lead_time_s", 1.6)),
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
            rotate_offset_rad=math.radians(
                float(scheduler_raw.get("rotate_offset_deg", 0.0))
            ),
            rotate_sign=(
                1.0 if float(scheduler_raw.get("rotate_sign", 1.0)) >= 0.0 else -1.0
            ),
            rotate_home_tolerance_deg=float(
                scheduler_raw.get("rotate_home_tolerance_deg", 0.0)
            ),
            belt_speed_static_mm_s=float(
                scheduler_raw.get("belt_speed_static_mm_s", 50.0)
            ),
            test_acceptance_cycles=int(scheduler_raw.get("test_acceptance_cycles", 9)),
            interp_v_max=float(interpolator_raw.get("v_max", 300.0)),
            interp_a_max=float(interpolator_raw.get("a_max", 1000.0)),
            interp_d_max=float(interpolator_raw.get("d_max", 1000.0)),
            interp_soft_start_s=float(interpolator_raw.get("soft_start_s", 0.08)),
            interp_scurve_shape_factor=float(
                interpolator_raw.get("scurve_shape_factor", 1.5)
            ),
            adaptive_speed_enabled=bool(
                scheduler_raw.get("adaptive_speed_enabled", False)
            ),
            pick_cycle_s=float(scheduler_raw.get("pick_cycle_s", 2.0)),
            pick_transit_min_s=float(scheduler_raw.get("pick_transit_min_s", 2.0)),
            belt_speed_headroom=float(scheduler_raw.get("belt_speed_headroom", 0.75)),
            belt_speed_min_mm_s=float(scheduler_raw.get("belt_speed_min_mm_s", 30.0)),
            belt_speed_max_mm_s=float(scheduler_raw.get("belt_speed_max_mm_s", 0.0)),
            belt_speed_hw_max_mm_s=float(
                scheduler_raw.get("belt_speed_hw_max_mm_s", 200.0)
            ),
            belt_speed_deadband_mm_s=float(
                scheduler_raw.get("belt_speed_deadband_mm_s", 8.0)
            ),
            belt_density_length_mm=float(
                scheduler_raw.get("belt_density_length_mm", 0.0)
            ),
            belt_accel_mm_s2=float(scheduler_raw.get("belt_accel_mm_s2", 22.31)),
            belt_ramp_s=float(scheduler_raw.get("belt_ramp_s", 0.25)),
            belt_speed_max_step_mm_s=float(
                scheduler_raw.get("belt_speed_max_step_mm_s", 20.0)
            ),
            belt_velocity_ema_alpha=float(
                conveyor_raw.get("velocity_ema_alpha", 0.4)
            ),
            limit_radius_xy=float(getattr(config, "limit_radius_xy", 180.0)),
            oblique_descent_enabled=bool(
                scheduler_raw.get("oblique_descent_enabled", False)
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
        if self.scenario_name in _ACCURACY_SCENARIOS or self.scenario_name == "test_vision_only":
            self._last_sample_time = now
            return SpeedSample(
                vx=0.0, vy=0.0, timestamp=now,
                position_mm=self._integrated_position_mm, speed_uv=0.0,
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

    def position_at(self, t: float) -> float | None:
        """No belt-position history in the synthetic source — the scheduler falls
        back to the current position (camera-latency compensation is a no-op for
        the static/simulated belt)."""
        return None


class ConveyorSpeedSource:
    """Derive belt speed and position from the Siemens `conveyor_position` field
    (mm as of June 2026, `conveyor_position_scale_mm = 1.0`).

    Only wired for scenarios with live PLC feedback (production /
    test_vision_only); the accuracy scenarios use SimulatedSpeedSource."""

    def __init__(
        self,
        request_status,
        frame: ConveyorFrame,
        decoder: BeltPositionTracker,
        scenario_name: str = "",
        position_scale_mm: float = 1.0,
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

    def position_at(self, t: float) -> float | None:
        """Belt position at a past capture time via the decoder's history buffer
        (camera-latency compensation). Returns None if too stale, so the caller
        falls back to the current position."""
        return self.decoder.position_at(t)


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
        # A3: wait until the pick dispatch window to mirror real robot timing.
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
    live PLC worker so the scheduler can read the real Siemens
    `conveyor_position` via ConveyorSpeedSource without moving the Omron robot.
    The presence of a `request_status` attribute is what makes
    `run_scheduler_scenario` pick ConveyorSpeedSource (real feedback) instead of
    SimulatedSpeedSource. No belt speed command is sent in test_vision_only.
    """

    def __init__(self, dispatch=None, request_status=None) -> None:
        self.dispatch = dispatch
        self.request_status = request_status

    def execute(self, plan: PickPlan, *, log_samples=False, real_time=False, scenario_name="") -> None:
        plan.status = "completed"


class _RoundTripTracker:
    """Rolling average of PLC dispatch round-trip durations (seconds), for the
    web dashboard's Performance card. Cheap and lock-protected by the caller."""

    def __init__(self, maxlen: int = 20) -> None:
        self._samples: deque[float] = deque(maxlen=maxlen)

    def record(self, duration_s: float) -> None:
        self._samples.append(duration_s)

    @property
    def average_s(self) -> float:
        if not self._samples:
            return 0.0
        return sum(self._samples) / len(self._samples)


class RealtimePickExecutor:
    """Dispatch real pick packets while all waits consume shared realtime state."""

    def __init__(
        self,
        dispatch,
        request_status,
        *,
        interpolar_points: int,
        wait_margin_s: float,
        status_poll_interval_s: float,
        position_tolerance_mm: float = 5.0,
        position_tolerance_max_mm: float | None = None,
        tolerance_speed_min_mm_s: float = 0.0,
        tolerance_speed_max_mm_s: float = 0.0,
        ipc_lock: threading.Lock | None = None,
        rotate_home_tolerance_deg: float = 0.0,
        rotate_offset_rad: float = 0.0,
        rotate_sign: float = 1.0,
        rotate_refresh_max_delta_deg: float = 15.0,
    ) -> None:
        self._dispatch_fn = dispatch
        self._request_status_fn = request_status
        self.interpolar_points = interpolar_points
        self.wait_margin_s = float(wait_margin_s)
        self.status_poll_interval_s = max(status_poll_interval_s, 0.02)
        self.position_tolerance_mm = max(float(position_tolerance_mm), 0.0)
        # Speed-mapped arrival tolerance: linear ramp from position_tolerance_mm
        # (at/below tolerance_speed_min_mm_s) to position_tolerance_max_mm
        # (at/above tolerance_speed_max_mm_s). Ceiling <= floor or a degenerate
        # speed range collapses to the static floor (old behavior).
        ceiling = (
            self.position_tolerance_mm
            if position_tolerance_max_mm is None
            else max(float(position_tolerance_max_mm), 0.0)
        )
        self.position_tolerance_max_mm = max(ceiling, self.position_tolerance_mm)
        self.tolerance_speed_min_mm_s = max(float(tolerance_speed_min_mm_s), 0.0)
        self.tolerance_speed_max_mm_s = max(float(tolerance_speed_max_mm_s), 0.0)
        self.ipc_lock = ipc_lock or threading.Lock()
        self.round_trip = _RoundTripTracker()
        # 0 disables the pick-time "cup back at 0 yet?" warning (see execute()).
        self.rotate_home_tolerance_deg = max(float(rotate_home_tolerance_deg), 0.0)
        # Mirrors SchedulerSettings.rotate_offset_rad/rotate_sign (main.py wires
        # these from the same `scheduler.rotate_offset_deg`/`rotate_sign` config
        # keys) so the post-grip rotate can be refreshed from the object's latest
        # tracked heading at the gate, not just the plan-build-time snapshot.
        self.rotate_offset_rad = float(rotate_offset_rad)
        self.rotate_sign = 1.0 if float(rotate_sign) >= 0.0 else -1.0
        # Reject the gate-time refresh if it swings more than this many degrees
        # from the plan-build heading (likely a vision glitch, not a real
        # refinement). 0 disables the refresh entirely.
        self.rotate_refresh_max_delta_deg = max(float(rotate_refresh_max_delta_deg), 0.0)

    def _arrival_tolerance_mm(self, belt_speed_mm_s: float | None) -> float:
        """Arm-arrival tolerance for the current belt speed: linear between the
        (speed_min -> tolerance floor) and (speed_max -> tolerance ceiling)
        anchors, clamped outside. Falls back to the static floor when the
        ceiling/speed range is degenerate or the speed is unknown."""
        span = self.tolerance_speed_max_mm_s - self.tolerance_speed_min_mm_s
        if (
            belt_speed_mm_s is None
            or span <= 0.0
            or self.position_tolerance_max_mm <= self.position_tolerance_mm
        ):
            return self.position_tolerance_mm
        fraction = (belt_speed_mm_s - self.tolerance_speed_min_mm_s) / span
        fraction = min(max(fraction, 0.0), 1.0)
        return self.position_tolerance_mm + fraction * (
            self.position_tolerance_max_mm - self.position_tolerance_mm
        )

    def dispatch(self, packet: dict[str, Any]) -> dict[str, Any] | None:
        with self.ipc_lock:
            t0 = time.monotonic()
            try:
                return self._dispatch_fn(packet)
            finally:
                self.round_trip.record(time.monotonic() - t0)

    def request_status(self) -> dict[str, Any] | None:
        with self.ipc_lock:
            return self._request_status_fn()

    def execute(
        self,
        plan: PickPlan,
        *,
        state: RealtimeState | None = None,
        log_samples: bool = False,
        real_time: bool = False,
        scenario_name: str,
    ) -> bool:
        del log_samples, real_time, scenario_name
        if state is None:
            raise RuntimeError("RealtimePickExecutor requires RealtimeState for execution.")

        packets = plan.to_robot_packets(self.interpolar_points)
        goto_packet = packets[0]
        pick_packet = packets[1]

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
        # Home the suction axis to 0 rad while the arm flies to the park
        # point: the cup grips at 0 and the board is normalised to the bin
        # orientation only AFTER grip (see the post-grip rotate below). Off the
        # critical path — the target angle is known since plan build and the
        # board does not rotate on the belt.
        try:
            self.dispatch({
                "commandID": COMMAND_ID["rotate_absolute"],
                "CommandID": COMMAND_ID["rotate_absolute"],
                "rotate": 0.0,
                "speed": 0.0,
            })
        except Exception as s_exc:
            print(f"[WARN] Failed to home suction rotation: {s_exc}")
        if not self._wait_for_arm_arrival(plan, "goto", goto_packet, state):
            plan.status = "failed"
            return False

        if not self._wait_for_object_arrival(plan, state):
            plan.status = "aborted"
            return False
        gate_fired_at = time.monotonic()
        # Refresh the post-grip rotate target from the object's latest tracked
        # heading: the plan was built from the first sighting, but vision keeps
        # refining the marker-vector angle for as long as the object stays in
        # the camera ROI (observed drift up to a few degrees in practice).
        # Reject the refresh if it swings further than rotate_refresh_max_delta_deg
        # from the plan-build heading — that is a vision glitch (e.g. a dropped
        # marker forcing the OBB symmetry-fold fallback), not a refinement.
        if self.rotate_refresh_max_delta_deg > 0.0:
            with state.state_lock:
                tracked_obj = _find_tracked_object(state.tracker, plan.object_id)
                rotation_rad_now = tracked_obj.rotation_rad if tracked_obj is not None else None
            plan_heading_deg = plan.debug_info.get("board_heading_deg")
            if rotation_rad_now is not None and plan_heading_deg is not None:
                heading_now_deg = math.degrees(rotation_rad_now)
                delta_deg = abs(((heading_now_deg - plan_heading_deg + 180.0) % 360.0) - 180.0)
                if delta_deg <= self.rotate_refresh_max_delta_deg:
                    plan.rotate_rad = wrap_rad(
                        self.rotate_sign * (self.rotate_offset_rad - rotation_rad_now)
                    )
                    plan.debug_info["board_heading_at_gate_deg"] = round(heading_now_deg, 2)
                else:
                    print(
                        "[WARN]",
                        json.dumps(
                            {
                                "plan_id": plan.plan_id,
                                "event": "rotate_refresh_outlier",
                                "plan_board_heading_deg": plan_heading_deg,
                                "latest_board_heading_deg": round(heading_now_deg, 2),
                                "delta_deg": round(delta_deg, 2),
                            },
                            ensure_ascii=True,
                        ),
                        flush=True,
                    )
        # Cup-angle snapshot at the gate: shows the home-to-0 residual (grip
        # while the axis is still travelling => random orientation error).
        with state.state_lock:
            rotate_at_gate = state.rotate_current_deg
        home_tol = self.rotate_home_tolerance_deg
        if (
            home_tol > 0.0
            and rotate_at_gate is not None
            and abs(rotate_at_gate) > home_tol
        ):
            # Warn-only by design: delaying the positional gate would miss the
            # object. If this fires often the axis is too slow for the cycle.
            print(
                "[WARN]",
                json.dumps(
                    {
                        "plan_id": plan.plan_id,
                        "event": "rotate_home_incomplete",
                        "rotate_at_gate_deg": round(rotate_at_gate, 2),
                        "tolerance_deg": home_tol,
                    },
                    ensure_ascii=True,
                ),
                flush=True,
            )

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
        dispatched_at = time.monotonic()
        # The pick is in flight: the object's fate is decided; pre-grip abort
        # handling no longer applies (main loop keys exactly-once off this flag).
        plan.debug_info["pick_dispatched"] = True
        with state.state_lock:
            state.gate_critical = False
            belt_speed_at_grip = state.belt_speed_mm_s
        if status is not None:
            print("[PLC]", json.dumps(status, ensure_ascii=True))
        # Adaptive belt speed (doc/basis-theory.md §6.5): commit the live target at
        # the grip instant — the object is already past the gate, so the density
        # drop from this pick is reflected immediately rather than waiting for the
        # next opportunistic commit.
        _commit_adaptive_speed(self.dispatch, state)
        if not self._wait_for_arm_arrival(
            plan,
            "pick",
            pick_packet,
            state,
            contact_z=plan.trajectory_pick[0].z,
            gate_fired_at=gate_fired_at,
            dispatched_at=dispatched_at,
            belt_speed_mm_s=belt_speed_at_grip,
            post_grip_rotate_rad=plan.rotate_rad,
            pre_pick_z=plan.trajectory_goto[-1].z,
        ):
            plan.status = "failed"
            return False
        # Rotation-calibration datum (one line per pick): commanded angles from
        # the plan vs measured cup angle at the gate (home-to-0 residual) and at
        # the trajectory end (mid-rotation release check). All degrees.
        with state.state_lock:
            rotate_at_end = state.rotate_current_deg
        print(
            "[ROTATE]",
            json.dumps(
                {
                    "plan_id": plan.plan_id,
                    "vision_angle_deg": plan.debug_info.get("vision_angle_deg"),
                    "board_heading_deg": plan.debug_info.get("board_heading_deg"),
                    "board_heading_at_gate_deg": plan.debug_info.get("board_heading_at_gate_deg"),
                    "rotate_cmd_deg": round(math.degrees(plan.rotate_rad), 2),
                    "rotate_at_gate_deg": (
                        round(rotate_at_gate, 2) if rotate_at_gate is not None else None
                    ),
                    "rotate_at_end_deg": (
                        round(rotate_at_end, 2) if rotate_at_end is not None else None
                    ),
                },
                ensure_ascii=True,
            ),
            flush=True,
        )
        plan.status = "completed"
        return True

    def _wait_for_arm_arrival(
        self,
        plan: PickPlan,
        phase_name: str,
        packet: dict[str, Any],
        state: RealtimeState,
        contact_z: float | None = None,
        gate_fired_at: float | None = None,
        dispatched_at: float | None = None,
        belt_speed_mm_s: float | None = None,
        post_grip_rotate_rad: float | None = None,
        pre_pick_z: float | None = None,
    ) -> bool:
        target = _packet_final_target(packet)
        if target is None:
            return True
        expected_duration_s = _packet_duration_s(packet)
        started_at = time.monotonic()
        deadline = started_at + expected_duration_s + self.wait_margin_s
        departed = False
        static_accept_allowed: bool | None = None
        contact_logged = contact_z is None
        # Post-grip suction rotation: dispatched once the arm has gripped
        # (descended to contact) AND lifted back up to the pre-pick height, so
        # the board is clear of the belt before it is turned to the bin
        # orientation. None => nothing to rotate (e.g. goto phase).
        rotate_dispatched = post_grip_rotate_rad is None or pre_pick_z is None
        # Wider (not the +2mm contact_z band _contact_logged_ uses for [GATE]
        # calibration) descent marker: the whole pick-height dip is only ~13mm
        # and the interpolator doesn't dwell at the bottom, so the 50ms pose
        # poll was missing the narrow contact_z+2mm band on roughly half of
        # real picks (confirmed against a production log: those picks' [GATE]
        # line never printed AND the post-grip rotate never fired, leaving the
        # cup at its home angle — not a PLC/ST retrigger issue). Using the
        # midpoint between pre_pick_z and contact_z gives the poll a much wider
        # window to catch, and the time-based fallback below is the backstop
        # that makes a miss impossible regardless of sampling luck.
        descent_seen = rotate_dispatched
        descent_mid_z: float | None = None
        rotate_fallback_deadline: float | None = None
        if not rotate_dispatched:
            descent_mid_z = (
                (pre_pick_z + contact_z) / 2.0 if contact_z is not None else pre_pick_z
            )
            if dispatched_at is not None:
                modeled_s = plan.descend_time_s
                if len(plan.trajectory_pick) >= 2:
                    modeled_s += plan.trajectory_pick[1].time_s
                rotate_fallback_deadline = dispatched_at + modeled_s + _ROTATE_FALLBACK_MARGIN_S

        while True:
            now = time.monotonic()
            # Opportunistic adaptive-speed commit: the gate is not imminent while
            # the arm is flying (goto) or already past the grip (pick return).
            _commit_adaptive_speed(
                self.dispatch, state, min_interval_s=_OPPORTUNISTIC_COMMIT_INTERVAL_S
            )
            with state.state_lock:
                pose = state.robot_pose
                live_belt_speed = state.belt_speed_mm_s
            # Re-evaluated each iteration: adaptive speed can change mid-wait
            # and the tolerance follows the live belt speed.
            tolerance_mm = self._arrival_tolerance_mm(live_belt_speed)

            def _log_gate(contact_observed: bool) -> None:
                # T_delay calibration datum (doc/basis-theory.md §4.4): true
                # dispatch->contact latency vs the configured robot_movement_delay_s.
                # contact_observed=False means the narrow contact_z+2mm band was
                # missed by the poll — the timing below is a degraded estimate
                # (later than the true contact instant), kept rather than lost.
                print(
                    "[GATE]",
                    json.dumps(
                        {
                            "plan_id": plan.plan_id,
                            "contact_observed": contact_observed,
                            "gate_to_dispatch_s": round(
                                (dispatched_at or now) - (gate_fired_at or now), 4
                            ),
                            "dispatch_to_contact_s": round(
                                now - (dispatched_at or now), 4
                            ),
                            # Modeled descent time — subtract from dispatch_to_contact_s
                            # to calibrate robot_movement_delay_s (the descent is no
                            # longer lumped into the dispatch->grip delay).
                            "t_d_model_s": round(plan.descend_time_s, 4),
                            "belt_speed_mm_s": round(belt_speed_mm_s or 0.0, 2),
                        },
                        ensure_ascii=True,
                    ),
                    flush=True,
                )

            if not contact_logged and pose is not None and pose[2] <= contact_z + 2.0:
                contact_logged = True
                _log_gate(contact_observed=True)
            if not descent_seen and pose is not None and pose[2] <= descent_mid_z:
                descent_seen = True
            if not rotate_dispatched and (
                (descent_seen and pose is not None and pose[2] >= pre_pick_z)
                or (rotate_fallback_deadline is not None and now >= rotate_fallback_deadline)
            ):
                # Board gripped and lifted clear — turn it to the bin orientation.
                # (Or the modeled-time fallback fired: the pose poll never caught
                # a sample confirming it, but the trajectory has certainly moved
                # past this point by now — dispatch anyway rather than never.)
                if not contact_logged:
                    # The narrow +2mm contact band was missed entirely; this is
                    # the best evidence we have that contact happened, so log a
                    # degraded [GATE] datum instead of losing the calibration
                    # sample outright.
                    contact_logged = True
                    _log_gate(contact_observed=False)
                if not descent_seen:
                    print(
                        "[WARN]",
                        json.dumps(
                            {"plan_id": plan.plan_id, "event": "rotate_dispatch_fallback"},
                            ensure_ascii=True,
                        ),
                        flush=True,
                    )
                rotate_dispatched = True
                try:
                    self.dispatch({
                        "commandID": COMMAND_ID["rotate_absolute"],
                        "CommandID": COMMAND_ID["rotate_absolute"],
                        "rotate": post_grip_rotate_rad,
                        "speed": 0.0,
                    })
                except Exception as s_exc:
                    print(f"[WARN] Failed to dispatch post-grip rotation: {s_exc}")
            if pose is not None:
                distance = _distance_3d(pose, target)
                if static_accept_allowed is None:
                    static_accept_allowed = (
                        distance <= tolerance_mm and expected_duration_s <= 0.25
                    )
                if distance > tolerance_mm:
                    departed = True
                elif departed or (
                    bool(static_accept_allowed)
                    and (now - started_at) >= min(0.2, expected_duration_s)
                ):
                    return True
            if now >= deadline:
                print(
                    "[WARN]",
                    json.dumps(
                        {
                            "plan_id": plan.plan_id,
                            "event": "arm_arrival_timeout",
                            "phase": phase_name,
                            "target": [round(value, 3) for value in target],
                        },
                        ensure_ascii=True,
                    ),
                )
                return False
            time.sleep(self.status_poll_interval_s)

    def _wait_for_object_arrival(self, plan: PickPlan, state: RealtimeState) -> bool:
        """Positional pick gate with a progress-based (not wall-clock) timeout.

        The old fixed deadline (`predicted_pick_time + margin`) aborted picks —
        permanently dropping still-pickable objects — whenever the belt slowed
        after plan-build (e.g. mid adaptive ramp). The object's u is encoder-
        anchored, so the only genuine failure modes are the track disappearing
        or the belt stalling: abort only when the object has made no forward
        progress for `stall_timeout_s`.
        """
        stall_timeout_s = max(3.0, 3.0 * self.wait_margin_s)
        last_progress_u: float | None = None
        last_progress_t = time.monotonic()
        gate_reached = False
        try:
            while True:
                now = time.monotonic()
                gate = _object_pick_gate_status(state, plan)
                if gate is None:
                    print(
                        "[WARN]",
                        json.dumps(
                            {"plan_id": plan.plan_id, "event": "pick_object_missing"},
                            ensure_ascii=True,
                        ),
                    )
                    return False
                if gate["reached"]:
                    gate_reached = True
                    return True
                if last_progress_u is None or gate["object_u"] > last_progress_u + 0.5:
                    last_progress_u = gate["object_u"]
                    last_progress_t = now
                elif now - last_progress_t > stall_timeout_s:
                    print(
                        "[WARN]",
                        json.dumps(
                            {
                                "plan_id": plan.plan_id,
                                "event": "pick_object_stalled",
                                "object_u_mm": round(gate["object_u"], 2),
                                "pick_u_mm": round(gate["pick_u"], 2),
                                "stall_timeout_s": stall_timeout_s,
                            },
                            ensure_ascii=True,
                        ),
                    )
                    return False
                # Suppress speed commits once the object is inside the critical
                # window (belt displacement over ~_GATE_CRITICAL_LEAD_S) so any
                # ramp has settled by gate-fire time; commit freely before that.
                remaining_mm = gate["threshold_u"] - gate["object_u"]
                with state.state_lock:
                    speed = state.belt_speed_mm_s
                    critical = remaining_mm <= max(speed, 0.0) * _GATE_CRITICAL_LEAD_S
                    state.gate_critical = critical
                if not critical:
                    _commit_adaptive_speed(
                        self.dispatch,
                        state,
                        min_interval_s=_OPPORTUNISTIC_COMMIT_INTERVAL_S,
                    )
                time.sleep(self.status_poll_interval_s)
        finally:
            # On the success path the flag stays set through the rotate + pick
            # dispatch (execute() clears it right after); on abort clear it here
            # so a failed gate never leaves speed commits blocked.
            if not gate_reached:
                with state.state_lock:
                    state.gate_critical = False


def _packet_final_target(packet: dict[str, Any]) -> Position3D | None:
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


def _packet_duration_s(packet: dict[str, Any]) -> float:
    argument_number = int(packet.get("argument_number", 0))
    durations = list(packet.get("argument_time", []))[:argument_number]
    total = 0.0
    for value in durations:
        try:
            total += float(value)
        except (TypeError, ValueError):
            pass
    return max(total, 0.0)


def _distance_3d(a: Position3D, b: Position3D) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def _find_tracked_object(tracker: BeltTracker, object_id: str) -> TrackedObject | None:
    for obj in tracker.objects():
        if obj.object_id == object_id:
            return obj
    return None


def _belt_lead_offset_mm(belt_speed_mm_s: float, command_delay_s: float) -> float:
    """Lead distance the pick gate must fire early to absorb the dispatch->grip
    latency T_delay (doc/basis-theory.md §4.4). The object's u is anchored to
    the belt encoder, so during T_delay it advances by exactly the belt's
    displacement. Steady-belt form (a=0): offset = v * T_delay. The §6.5 timing
    strategy guarantees the belt is steady at gate-fire time, so no acceleration
    term is needed; any residual ramp error is corrected by the live gate itself."""
    return max(0.0, belt_speed_mm_s * command_delay_s)


def _descent_time_s(settings: SchedulerSettings, belt_speed_mm_s: float) -> float:
    """Modeled time for the oblique park->contact descent (pre_pick -> pickup),
    including the pick command's one-off soft start. The move slants along the
    belt by v*t_d while dropping |Δz|, so one fixed-point pass folds the belt
    slant into the segment length. Mirrors the PLC interpolator via
    _segment_profile_time (doc/PLC_Program_description/MC_inter_curve_vel.md)."""
    dz = abs(settings.pre_pick_height - settings.pickup_height)

    def _seg(length: float) -> float:
        return settings.interp_soft_start_s + _segment_profile_time(
            length, 0.0, 0.0,
            settings.interp_v_max, settings.interp_a_max, settings.interp_d_max,
            settings.interp_scurve_shape_factor,
        )

    t0 = _seg(dz)
    slant = max(0.0, belt_speed_mm_s) * t0
    return _seg(math.hypot(dz, slant))


def _contact_position(
    frame: ConveyorFrame,
    settings: SchedulerSettings,
    pick_position: Position3D,
    belt_speed_mm_s: float,
) -> tuple[Position3D, float]:
    """Downstream contact point for the oblique (belt-tracking) descent, and the
    modeled descent time t_d.

    The arm parks ABOVE ``pick_position`` (the predicted object-arrival point, at
    pre_pick height — unchanged from the straight-down logic). During the short
    pre_pick->pickup drop the object keeps moving, so the pick-phase contact is
    shifted DOWNSTREAM along the belt (+u_hat) by v*t_d, letting the descent slant
    to track it. When the oblique descent is disabled (or belt speed 0) the offset
    is 0 and the contact coincides with ``pick_position`` (vertical descent)."""
    t_d = _descent_time_s(settings, belt_speed_mm_s)
    if not settings.oblique_descent_enabled:
        return (pick_position[0], pick_position[1], settings.pickup_height), t_d
    offset = max(0.0, belt_speed_mm_s) * t_d
    u_x, u_y = frame.u_hat
    contact: Position3D = (
        pick_position[0] + u_x * offset,
        pick_position[1] + u_y * offset,
        settings.pickup_height,
    )
    return contact, t_d


def _adaptive_belt_speed(n_objects: int, settings: SchedulerSettings) -> float:
    """Rate-regulation belt speed (doc/basis-theory.md §6.2/§6.3).

    Holds the presentation rate lambda_nom = headroom * mu_max by setting belt
    speed INVERSELY to product density: v = clamp(lambda_nom / rho, v_min, v_cap)
    with rho = N / L_meas, i.e. v = lambda_nom * L_meas / N. Three regimes emerge
    from the clamps: sparse -> v_cap (fetch the few objects fast), regulated ->
    interior, dense -> v_min (overload drains emergently at the floor)."""
    u_min, u_max, _v_min_uv, _v_max_uv = settings.workspace_window_uv
    window_len = max(1e-6, u_max - u_min)
    v_min = settings.belt_speed_min_mm_s
    # v_cap = pickability ceiling L / t_transit, clamped first to the soft operational
    # max (belt_speed_max_mm_s, if set) then to the absolute hardware safety limit.
    v_cap = window_len / max(1e-6, settings.pick_transit_min_s)
    if settings.belt_speed_max_mm_s > 0.0:
        v_cap = min(v_cap, settings.belt_speed_max_mm_s)
    v_cap = min(v_cap, settings.belt_speed_hw_max_mm_s)
    v_cap = max(v_min, v_cap)
    # L_meas spans O_conveyor (C-frame origin = ROI origin) to u_max unless overridden.
    l_meas = settings.belt_density_length_mm if settings.belt_density_length_mm > 0.0 else u_max
    if n_objects <= 0:
        return v_cap  # sparse feeder: run fast to fetch the few available objects
    mu_max = 1.0 / max(1e-6, settings.pick_cycle_s)
    lambda_nom = settings.belt_speed_headroom * mu_max
    v = lambda_nom * l_meas / float(n_objects)
    return max(v_min, min(v_cap, v))


# Only the leading (closest-to-pick) objects constrain the belt via spacing: a
# tight cluster still far upstream should not force a premature slow-down.
_SPACING_LEAD_OBJECTS = 4


def _spacing_speed_cap(object_u_mm: list[float], settings: SchedulerSettings) -> float:
    """Belt-speed ceiling from inter-object spacing (queueing constraint).

    The density law (_adaptive_belt_speed) regulates the *average* presentation
    rate but ignores how objects are *spaced*: a tight pair (N=2, 40 mm apart)
    is serviced at the same speed as a spread pair (N=2, 300 mm apart), yet the
    robot needs pick_cycle_s between consecutive picks. Enforce
    v <= gap / pick_cycle_s on the tightest adjacent gap among the leading
    objects so a cluster slows the belt even when N is small. Returns +inf when
    fewer than two objects lead (no spacing constraint). A caller clamps the
    result to v_min, so a cluster tighter than v_min·pick_cycle_s just pins the
    belt to its floor (best-effort; the trailing object may still be missed)."""
    if len(object_u_mm) < 2:
        return float("inf")
    lead = sorted(object_u_mm, reverse=True)[:_SPACING_LEAD_OBJECTS]
    gap_min = min(a - b for a, b in zip(lead, lead[1:]))
    if gap_min <= 0.0:
        return float("inf")  # overlapping detections — leave it to density/floor
    return gap_min / max(1e-6, settings.pick_cycle_s)


# Re-send the setpoint when the measured belt speed still diverges this long
# after the last commit (any commanded ramp has settled by then).
_SPEED_RESYNC_AFTER_S = 3.0

# Suppress speed commits once the tracked object is within this much belt travel
# time of the gate threshold, so the commit's ramp (max_step / belt_accel_mm_s2,
# ≈0.9 s at the default 20 mm/s step) has settled before the gate fires.
_GATE_CRITICAL_LEAD_S = 2.0

# Throttle for the opportunistic commits issued from the executor wait loops and
# the idle main loop (each commit costs one Siemens round-trip on ipc_lock).
_OPPORTUNISTIC_COMMIT_INTERVAL_S = 0.75

# Safety margin added on top of the modeled dispatch->contact->lift-past-pre_pick
# time before the post-grip rotate is force-dispatched even if the 50 ms pose
# poll never sampled a point inside the (narrow, fast-transited) descent/lift
# window (see _wait_for_arm_arrival's rotate_fallback_deadline).
_ROTATE_FALLBACK_MARGIN_S = 0.3


def _commit_adaptive_speed(
    dispatch_fn: Callable[[dict[str, Any]], dict[str, Any] | None],
    state: RealtimeState,
    *,
    min_interval_s: float = 0.0,
) -> None:
    """Walk the belt setpoint toward the live adaptive target (doc/basis-theory.md
    §6.5, revised). Called opportunistically from the executor wait loops, at the
    grip instant, and from the idle main loop — commits are suppressed only while
    `gate_critical` is set (pick gate imminent). Each commit is rate-limited to
    `belt_speed_max_step_mm_s` so its ramp settles quickly, and throttled to at
    most one dispatch per `min_interval_s`. When the target is already reached
    but the PLC-measured speed diverges, the setpoint is re-sent (closed loop).
    `dispatch_fn` takes `ipc_lock`; this function never holds `state_lock` while
    calling it, so the two locks are never nested."""
    now = time.monotonic()
    with state.state_lock:
        if not state.adaptive_speed_enabled or state.gate_critical:
            return
        target = state.belt_speed_target_mm_s
        setpoint = state.belt_speed_setpoint_mm_s
        deadband = state.belt_speed_deadband_mm_s
        max_step = state.belt_speed_max_step_mm_s
        measured = state.belt_speed_measured_mm_s
        last_commit = state.last_speed_commit_monotonic
    if target <= 0.0:
        return
    if min_interval_s > 0.0 and (now - last_commit) < min_interval_s:
        return
    step_target = target
    if max_step > 0.0:
        delta = max(-max_step, min(max_step, target - setpoint))
        step_target = setpoint + delta
    if abs(step_target - setpoint) <= deadband:
        # Setpoint already at (the stepped) target. Closed-loop resync: if the
        # measured speed still diverges well past the deadband long after the
        # ramp should have settled, the PLC missed/clamped the command — re-send.
        if (
            measured is None
            or abs(setpoint - measured) <= 2.0 * deadband
            or (now - last_commit) < _SPEED_RESYNC_AFTER_S
        ):
            return
        step_target = setpoint
        print(
            f"[WARN] belt speed diverged (setpoint {setpoint:.1f}, "
            f"measured {measured:.1f} mm/s) — re-sending setpoint"
        )
    try:
        dispatch_fn(
            {
                "commandID": COMMAND_ID["change_speed"],
                "CommandID": COMMAND_ID["change_speed"],
                "rotate": 0.0,
                "speed": float(step_target),
            }
        )
        print(f"[SPEED] belt -> {step_target:.1f} mm/s (target {target:.1f})")
    except Exception as exc:
        print(f"[WARN] adaptive change_speed failed: {exc}")
        return
    with state.state_lock:
        state.belt_speed_setpoint_mm_s = step_target
        state.last_speed_commit_monotonic = now


def _object_pick_gate_status(state: RealtimeState, plan: PickPlan) -> dict[str, float | bool] | None:
    with state.state_lock:
        obj = _find_tracked_object(state.tracker, plan.object_id)
        if obj is None:
            return None
        p_now = state.belt_position_mm
        speed = state.belt_speed_mm_s
        u_now, _ = obj.current_uv(p_now)
        u_pick, _ = state.frame.to_conveyor(
            plan.predicted_pick_position_2d[0],
            plan.predicted_pick_position_2d[1],
        )
        # Lead budget = dispatch->grip delay + gate sampling staleness (poll /2 +
        # perception tick /2). The gate fires when the object reaches the park
        # (pick_position); the oblique descent (if enabled) then SLANTS to follow
        # the object during t_d, so the descent time must NOT be added here (doing
        # so double-counted the object's travel and fired the gate far too early).
        command_delay_s = state.command_delay_s + state.gate_sampling_latency_s
    threshold = u_pick - _belt_lead_offset_mm(speed, command_delay_s)
    return {
        "reached": u_now >= threshold,
        "object_u": u_now,
        "pick_u": u_pick,
        "threshold_u": threshold,
    }


def _prune_unclaimed_tracker(
    tracker: BeltTracker,
    claimed_object_ids: set[str],
    p_now: float,
    now: float,
) -> int:
    removed = 0
    for obj in list(tracker.objects()):
        if obj.object_id in claimed_object_ids:
            continue
        # Keep objects that have only left the camera FOV (still on the belt
        # within the workspace) so the dashboard lists them across the whole
        # ROI-to-workspace span; drop only those truly off the belt or lost
        # while still under the camera. See BeltTracker.should_prune.
        if tracker.should_prune(obj, p_now, now):
            tracker.remove(obj.object_id)
            removed += 1
    return removed


def _belt_zone_label(u: float, settings: SchedulerSettings) -> str:
    """Classify a belt position u into a human-readable zone for the dashboard:
    ROI (under camera), transit (between camera and workspace), or workspace.
    """
    cam_min, cam_max = settings.camera_window_uv[0], settings.camera_window_uv[1]
    ws_min, ws_max = settings.workspace_window_uv[0], settings.workspace_window_uv[1]
    if u < cam_min:
        return "upstream"
    if u <= cam_max:
        return "ROI"
    if u < ws_min:
        return "transit"
    if u <= ws_max:
        return "workspace"
    return "past"


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

        # Owned metrics for callers using the execute() compat wrapper below
        # (test_accuracy/test_acceptance) — _run_evaluate_loop builds and owns
        # its own EvaluateMetrics instead and never reads this one.
        self.metrics = EvaluateMetrics()
        self.round_trip = _RoundTripTracker()

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def close(self) -> None:
        """Stop the background status-poller thread."""
        self._stop_event.set()
        self._poller_thread.join(timeout=2.0)

    def execute(
        self,
        plan: "PickPlan",
        *,
        log_samples: bool = False,
        real_time: bool = False,
        scenario_name: str = "",
    ) -> bool:
        """Polymorphic-executor compat shim (matches SimulatedExecutor/NullExecutor/
        RealtimePickExecutor's call signature) so test_accuracy/test_acceptance's
        single-thread loop can use this class as a drop-in real-hardware backend
        without a belt-gate (their objects are static, not tracked on a moving belt)."""
        del log_samples, real_time, scenario_name
        self.execute_evaluate(plan, self.metrics, rotate_rad=plan.rotate_rad)
        return plan.status == "completed"

    def execute_evaluate(
        self,
        plan: "PickPlan",
        metrics: EvaluateMetrics,
        *,
        rotate_rad: float | None = None,
    ) -> "Position3D | None":
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
                        "rotate": 0.0 if rotate_rad is None else rotate_rad,
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
            metrics.phase_modeled_times.append(
                float(plan.debug_info.get(
                    "modeled_goto_s" if phase_name == "goto" else "modeled_pick_s", 0.0
                ))
            )
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
            t0 = time.monotonic()
            try:
                return self._dispatch_fn(packet)
            finally:
                self.round_trip.record(time.monotonic() - t0)

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
        scenario_name: str = "",
    ) -> None:
        self.settings = settings
        self.interpolar_points = interpolar_points
        self.frame = frame
        self.tracker = tracker
        self.scenario_name = scenario_name
        self.seen_object_ids: dict[str, float] = {}
        # Object ids already committed to a pick plan. Vision now re-emits the
        # same id every frame while the object is visible, so without this guard a
        # still-visible (already-planned) object would be re-created and re-picked.
        self.planned_object_ids: dict[str, float] = {}
        self.metrics = SchedulerMetrics()
        self.current_position: Position3D = settings.home_position
        self.latest_speed: SpeedSample | None = None
        self.plan_counter = 0
        self._last_speed_log_t = 0.0

    def ingest_detections(
        self,
        detections: list[ObjectDetection],
        p_now: float,
        position_at: "Callable[[float], float | None] | None" = None,
    ) -> None:
        for detection in detections:
            # Skip detections for objects already committed to a pick plan — the
            # vision pipeline re-emits the same id every frame while it is visible.
            if detection.object_id in self.planned_object_ids:
                continue
            self.metrics.total_detections += 1
            # Camera-latency compensation: anchor the object to the belt position
            # AT the frame's capture time (detection.timestamp), not the current
            # ingest position. Falls back to p_now when history is unavailable
            # (simulated/static belt, or the sample is too stale to interpolate).
            p_anchor = p_now
            if position_at is not None:
                past_p = position_at(detection.timestamp)
                if past_p is not None:
                    p_anchor = past_p
            self.tracker.ingest_detection(
                detection, p_anchor, object_dimensions=self.settings.object_dimensions
            )
            self.seen_object_ids[detection.object_id] = detection.timestamp
        self.metrics.queue_peak = max(
            self.metrics.queue_peak, len(list(self.tracker.objects()))
        )

    def update_speed(self, sample: SpeedSample) -> None:
        self.latest_speed = sample
        self.log_speed(sample)

    def log_speed(self, sample: SpeedSample) -> None:
        """Rate-limited [SPEED] console trace (~1 Hz). The realtime perception
        thread calls this OUTSIDE state_lock — at the old per-tick (40 Hz) rate
        the print's flush could stall the lock and delay the pick gate."""
        now = time.monotonic()
        if now - self._last_speed_log_t < 1.0:
            return
        self._last_speed_log_t = now
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
        prediction: tuple[float, float, Position3D] | None = None,
    ) -> PickPlan:
        if self.latest_speed is None and prediction is None:
            raise RuntimeError("Cannot build pick plan without a current speed sample.")
        if prediction is None:
            prediction = self._predict_pick_position(obj, self.latest_speed, now)
        if prediction is None:
            raise RuntimeError("Unable to build pick plan for an unreachable detection.")

        predicted_pick_time, pick_dispatch_time, pick_position = prediction
        # The arm parks ABOVE pick_position (the predicted object-arrival point,
        # pre_pick height) exactly as the straight-down logic — the goto is
        # unchanged and stays inside the workspace. Only the pick-phase CONTACT is
        # shifted downstream (+u_hat by v*t_d) when the oblique descent is enabled,
        # so the short pre_pick->pickup drop slants to track the moving object. At
        # belt speed 0 or with the flag off, contact == pick_position (vertical).
        belt_speed_uv = self.latest_speed.speed_uv if self.latest_speed is not None else 0.0
        contact_position, descend_time_s = _contact_position(
            self.frame, self.settings, pick_position, belt_speed_uv
        )
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
            contact_position,
            sorting_position,
            self.settings,
            goto_points,
        )
        pick_times = _build_pick_timing(
            contact_position,
            pick_points,
            self.settings,
            goto_points,
        )
        # test_accuracy/test_acceptance grip static fake objects at fixed points with no
        # real board present — never command suction so the runs are unattended.
        pick_e_values = (
            [0, 0, 0, 0, 0, 0, 0]
            if self.scenario_name in _ACCURACY_SCENARIOS
            else [1, 1, 1, 1, 1, 1, 0]
        )
        trajectory_pick = [
            TrajectoryPoint(point[0], point[1], point[2], e_value, duration)
            for point, e_value, duration in zip(
                pick_points,
                pick_e_values,
                pick_times,
            )
        ]

        self.plan_counter += 1
        self.metrics.planned_picks += 1
        self.metrics.total_planning_latency += max(now - obj.last_seen_at, 0.0)
        self.metrics.planning_events += 1

        # Normalise-to-zero suction angle. The rotation happens AFTER grip
        # (board attached): drive the axis to cancel the board's R-frame heading
        # (obj.rotation_rad — already converted from the vision angle at ingest)
        # so every board lands in the bin at the `rotate_offset_deg` orientation.
        # rotate_sign flips the whole command if the physical axis turns opposite
        # to the R-frame CCW convention (calibrate with modules/test_rotate).
        # The wrap to [-pi, pi) is by itself the shortest-way rotation.
        rotate_rad = wrap_rad(
            self.settings.rotate_sign
            * (self.settings.rotate_offset_rad - obj.rotation_rad)
        )

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
            rotate_rad=rotate_rad,
            descend_time_s=descend_time_s,
            debug_info={
                "pick_position_3d": pick_position,
                "contact_position_3d": contact_position,
                "descend_time_s": descend_time_s,
                # [ROTATE] calibration log inputs (degrees for readability).
                "vision_angle_deg": round(obj.vision_angle_deg, 2),
                "board_heading_deg": round(math.degrees(obj.rotation_rad), 2),
                "rotate_cmd_deg": round(math.degrees(rotate_rad), 2),
                "timing_formula": {
                    "t_p_real": pick_dispatch_time,
                    "t_p_theory": predicted_pick_time,
                    "robot_movement_delay_s": self.settings.robot_movement_delay_s,
                    "ethernet_delay_s": self.settings.ethernet_delay_s,
                },
                # Interpolator-model phase times (mirror the PLC, unlike the crude
                # nominal-speed argument_time). Used by the CONFIG-SUGGEST tool to
                # estimate robot_movement_delay_s = measured_wall - modeled_motion.
                "modeled_goto_s": _trajectory_total_time(goto_points, self.settings),
                "modeled_pick_s": _trajectory_total_time(pick_points, self.settings),
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


def _predict_realtime_pick_position(
    scheduler: PickScheduler,
    obj: TrackedObject,
    speed_sample: SpeedSample,
    now: float,
) -> tuple[float, float, Position3D] | None:
    """Predict the realtime pick using the kept solver and a caller-side lead.

    The kept solver is probed with a zero lead so it still supplies the earliest
    reachable intercept. The realtime caller then applies the configured minimum
    lead and performs the final reachability / workspace check.
    """
    original_settings = scheduler.settings
    try:
        scheduler.settings = replace(original_settings, intercept_lead_time_s=0.0)
        earliest = scheduler._predict_pick_position(obj, speed_sample, now)
    finally:
        scheduler.settings = original_settings
    if earliest is None:
        return None

    earliest_pick_time, _, _ = earliest
    settings = scheduler.settings
    command_delay_s = settings.robot_movement_delay_s + settings.ethernet_delay_s
    final_pick_time = max(earliest_pick_time, now + settings.intercept_lead_time_s)

    u_anchor, v_anchor = obj.conveyor_uv
    p_now = speed_sample.position_mm
    u_now = u_anchor + (p_now - obj.belt_pos_anchor)
    v_now = v_anchor
    belt_speed = speed_sample.speed_uv
    _u_min, u_max, v_min, v_max = settings.workspace_window_uv

    if v_now < v_min or v_now > v_max:
        return None
    if u_now > u_max:
        return None

    dt_future = max(0.0, final_pick_time - now)
    u_pick = u_now + belt_speed * dt_future
    if u_pick > u_max:
        if belt_speed <= 0.0:
            return None
        u_pick = u_max
        final_pick_time = now + max(0.0, (u_pick - u_now) / belt_speed)

    if not scheduler.frame.is_in_window_uv(u_pick, v_now, settings.workspace_window_uv):
        return None

    pick_xy = scheduler.frame.to_robot(u_pick, v_now)
    pick_position: Position3D = (pick_xy[0], pick_xy[1], settings.pickup_height)
    # The arm parks above pick_position (in-workspace); the oblique descent only
    # slants the pick-phase contact, so the goto reachability is on pick_position
    # itself, exactly as the straight-down logic.
    goto_points = _build_goto_geometry(scheduler.current_position, pick_position, settings)
    goto_total = _trajectory_total_time(goto_points, settings)
    arm_arrival_time = now + command_delay_s + goto_total
    if arm_arrival_time > final_pick_time:
        return None

    return final_pick_time, final_pick_time - command_delay_s, pick_position


def _cycle_distance_mm(
    start_position: Position3D,
    pick_position: Position3D,
    sorting_position: Position3D,
) -> float:
    return math.dist(start_position, pick_position) + math.dist(pick_position, sorting_position)


def _build_realtime_pick_plan(
    scheduler: PickScheduler,
    state: RealtimeState,
    now: float,
) -> PickPlan | None:
    with state.state_lock:
        sample = state.latest_speed
        if sample is None:
            return None
        if now - sample.timestamp > scheduler.settings.speed_timeout_s:
            return None

        danger_u = scheduler.settings.workspace_window_uv[0] + (
            (scheduler.settings.workspace_window_uv[1] - scheduler.settings.workspace_window_uv[0])
            * (2.0 / 3.0)
        )
        current_position = scheduler.current_position
        candidates: list[tuple[tuple[int, float], RealtimePickCandidate]] = []

        for obj in scheduler.tracker.objects():
            if obj.object_id in state.claimed_object_ids:
                continue
            # Defensive guard against re-targeting an already-attempted object
            # (phantom re-pick): the main loop removes completed/failed objects
            # from the tracker, so this should normally never trigger.
            if obj.object_id in scheduler.planned_object_ids:
                continue

            sorting_position = scheduler._resolve_sorting_position(obj.object_type)
            if sorting_position is None:
                scheduler.metrics.skipped_unknown_type += 1
                continue

            prediction = _predict_realtime_pick_position(scheduler, obj, sample, now)
            if prediction is None:
                u_now, _ = obj.current_uv(sample.position_mm)
                if u_now > scheduler.settings.workspace_window_uv[1]:
                    scheduler.metrics.skipped_outside_workspace += 1
                    scheduler.tracker.remove(obj.object_id)
                continue

            predicted_pick_time, pick_dispatch_time, pick_position = prediction
            u_now, _ = obj.current_uv(sample.position_mm)
            is_danger = u_now >= danger_u
            cycle_distance = _cycle_distance_mm(current_position, pick_position, sorting_position)
            candidate = RealtimePickCandidate(
                obj=obj,
                sorting_position=sorting_position,
                predicted_pick_time=predicted_pick_time,
                pick_dispatch_time=pick_dispatch_time,
                pick_position=pick_position,
                u_now=u_now,
                u_pick=scheduler.frame.to_conveyor(pick_position[0], pick_position[1])[0],
                is_danger=is_danger,
                cycle_distance_mm=cycle_distance,
            )
            key = (0 if is_danger else 1, -u_now if is_danger else cycle_distance)
            candidates.append((key, candidate))

        if not candidates:
            return None

        candidates.sort(key=lambda item: item[0])
        _, chosen = candidates[0]
        plan = scheduler._build_pick_plan(
            chosen.obj,
            chosen.sorting_position,
            now,
            prediction=(chosen.predicted_pick_time, chosen.pick_dispatch_time, chosen.pick_position),
        )
        state.claimed_object_ids.add(chosen.obj.object_id)
        return plan


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


def _extract_robot_pose(status: dict[str, Any] | None) -> tuple[Position3D | None, int | None]:
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


def _run_realtime_pick_loop(
    scenario_name: str,
    settings: SchedulerSettings,
    interpolar_points: int,
    executor: RealtimePickExecutor,
    image_processing: SimulatedImageProcessing | VisionImageProcessing,
    frame: ConveyorFrame,
    scheduler: PickScheduler,
    speed_source: Any,
    start_time: float,
    duration_s: float | None,
    event_sink: "Callable[[str, dict[str, Any]], None] | None",
) -> None:
    state = RealtimeState(
        tracker=scheduler.tracker,
        frame=frame,
        ipc_lock=executor.ipc_lock,
        command_delay_s=settings.robot_movement_delay_s + settings.ethernet_delay_s,
        belt_speed_setpoint_mm_s=settings.belt_speed_static_mm_s,
        belt_speed_deadband_mm_s=settings.belt_speed_deadband_mm_s,
        adaptive_speed_enabled=settings.adaptive_speed_enabled,
        belt_speed_max_step_mm_s=settings.belt_speed_max_step_mm_s,
        # Average staleness of the object's u as seen by the gate: half the gate
        # poll interval + half the perception tick (25 ms).
        gate_sampling_latency_s=settings.poll_interval_s / 2.0 + 0.0125,
    )

    def perceive_tick(now: float | None = None) -> SpeedSample:
        if now is None:
            now = time.monotonic()
        sample = speed_source.sample(now)
        detections = image_processing.poll(now)
        last_status = getattr(speed_source, "last_status", None)
        pose, end_effector = _extract_robot_pose(last_status)
        measured_speed: float | None = None
        if isinstance(last_status, dict) and last_status.get("speed_current") is not None:
            try:
                measured_speed = float(last_status["speed_current"])
            except (TypeError, ValueError):
                measured_speed = None
        rotate_current: float | None = None
        if isinstance(last_status, dict) and last_status.get("rotate_current") is not None:
            try:
                rotate_current = float(last_status["rotate_current"])
            except (TypeError, ValueError):
                rotate_current = None
        with state.state_lock:
            state.latest_speed = sample
            state.belt_position_mm = sample.position_mm
            state.belt_speed_mm_s = sample.speed_uv
            state.robot_pose = pose
            state.end_effector = end_effector
            state.belt_speed_measured_mm_s = measured_speed
            if rotate_current is not None:
                state.rotate_current_deg = rotate_current
            scheduler.latest_speed = sample
            scheduler.ingest_detections(
                detections, sample.position_mm,
                position_at=getattr(speed_source, "position_at", None),
            )
            # Every object still on the belt — from the camera ROI, through the
            # transit gap, all the way to the downstream edge of the workspace —
            # with its live belt position u (mm) and zone so the dashboard can
            # show the full ROI-to-workspace journey, not just what is under the
            # camera right now.
            snapshot = []
            for obj in scheduler.tracker.objects():
                u_now, _ = obj.current_uv(sample.position_mm)
                x_r, y_r = scheduler.tracker.current_position_R(obj, sample.position_mm)
                snapshot.append({
                    "id": obj.object_id,
                    "type": obj.object_type,
                    "x": round(x_r, 2),
                    "y": round(y_r, 2),
                    "u": round(u_now, 1),
                    "zone": _belt_zone_label(u_now, settings),
                    "vision_angle_deg": round(obj.vision_angle_deg, 2),
                })
            removed = _prune_unclaimed_tracker(
                scheduler.tracker,
                state.claimed_object_ids,
                sample.position_mm,
                now,
            )
            scheduler.metrics.stale_drops += removed
            limit = now - settings.stale_timeout_s
            scheduler.seen_object_ids = {
                obj_id: ts for obj_id, ts in scheduler.seen_object_ids.items() if ts >= limit
            }
            scheduler.planned_object_ids = {
                obj_id: ts for obj_id, ts in scheduler.planned_object_ids.items() if ts >= limit
            }
            # Live object density (count within the workspace window) — sensed every
            # tick (25 ms) regardless of whether adaptive control is enabled, so the
            # dashboard density chart is always meaningful. Adaptive belt speed
            # (doc/basis-theory.md §6.5) reuses the same count when it's on; the
            # executor commits the target at the grip instant via _commit_adaptive_speed.
            u_max = settings.workspace_window_uv[1]
            density_u = []
            for obj in scheduler.tracker.objects():
                if obj.object_id in state.claimed_object_ids:
                    continue
                u_obj = obj.current_uv(sample.position_mm)[0]
                if 0.0 <= u_obj <= u_max:
                    density_u.append(u_obj)
            n_density = len(density_u)
            if state.adaptive_speed_enabled:
                # Average-rate term (density) and cluster term (spacing): a tight
                # group forces a slow-down even at small N so consecutive picks
                # stay >= pick_cycle_s apart. Clamp to the floor so a cluster too
                # tight for v_min just pins the belt low (best-effort).
                v_density = _adaptive_belt_speed(n_density, settings)
                v_spacing = _spacing_speed_cap(density_u, settings)
                state.belt_speed_target_mm_s = max(
                    settings.belt_speed_min_mm_s, min(v_density, v_spacing)
                )
            recent_pick_cycle_s = state.recent_pick_cycle_s
        # Outside state_lock: stdout can stall (slow terminal/SSH) and must never
        # block the gate poll or plan build, which contend on the same lock.
        scheduler.log_speed(sample)
        if event_sink is not None:
            status_payload = {
                "scenario": scenario_name,
                "vx": round(sample.vx, 3),
                "vy": round(sample.vy, 3),
                "speed_mm_s": round(math.hypot(sample.vx, sample.vy), 3),
                "position_mm": round(sample.position_mm, 2),
                "object_density": n_density,
                "round_trip_latency_s": round(executor.round_trip.average_s, 4),
                "pick_cycle_s": round(recent_pick_cycle_s, 3),
            }
            if state.robot_pose is not None:
                status_payload["x"] = round(state.robot_pose[0], 2)
                status_payload["y"] = round(state.robot_pose[1], 2)
                status_payload["z"] = round(state.robot_pose[2], 2)
            if state.end_effector is not None:
                status_payload["e"] = state.end_effector
            event_sink("status", status_payload)
        if snapshot:
            detect_payload = {
                "t": round(now - start_time, 3),
                "z": round(settings.pickup_height, 2),
                "objects": snapshot,
            }
            print("[DETECT]", json.dumps(detect_payload, ensure_ascii=True), flush=True)
            if event_sink is not None:
                event_sink("detect", detect_payload)
        return sample

    perception_thread = threading.Thread(
        target=lambda: _realtime_perception_loop(state, perceive_tick),
        name="realtime-perception",
        daemon=True,
    )
    perception_thread.start()

    # Seed the initial belt speed unconditionally: it is the static operating
    # speed when adaptive control is off, and the starting setpoint the adaptive
    # controller walks from when it is on. (Previously only sent when adaptive
    # was enabled, leaving the belt uncommanded in static production runs.)
    try:
        executor.dispatch(
            {
                "commandID": COMMAND_ID["change_speed"],
                "CommandID": COMMAND_ID["change_speed"],
                "rotate": 0.0,
                "speed": settings.belt_speed_static_mm_s,
            }
        )
        print(f"[INFO] Conveyor speed set to {settings.belt_speed_static_mm_s} mm/s")
        with state.state_lock:
            state.last_speed_commit_monotonic = time.monotonic()
    except Exception as exc:
        print(f"[WARN] Could not set conveyor speed: {exc}")

    deadline = None if duration_s is None else start_time + duration_s
    cycle_time_samples: deque[float] = deque(maxlen=20)
    try:
        while not state.stop_event.is_set():
            now = time.monotonic()
            if deadline is not None and now >= deadline:
                break

            plan = _build_realtime_pick_plan(scheduler, state, now)
            if plan is not None:
                plan_summary = plan.to_summary()
                print("[PLAN]", json.dumps(plan_summary, ensure_ascii=True))
                if event_sink is not None:
                    event_sink("plan", plan_summary)
                cycle_t0 = time.monotonic()
                success = executor.execute(plan, state=state, scenario_name=scenario_name)
                cycle_time_samples.append(time.monotonic() - cycle_t0)
                # Exactly-once applies to DISPATCHED picks only (suction is never
                # verified, so an attempted grip must not be retried — issue-2 /
                # theory §6.8). A pre-grip abort (goto failure, gate stall, track
                # lost) leaves the object physically on the belt: keep it in the
                # tracker and only unclaim it, so the next plan build can re-target
                # it instead of dropping a still-pickable object forever.
                attempted = success or bool(plan.debug_info.get("pick_dispatched"))
                with state.state_lock:
                    state.claimed_object_ids.discard(plan.object_id)
                    state.recent_pick_cycle_s = sum(cycle_time_samples) / len(cycle_time_samples)
                    if attempted:
                        scheduler.planned_object_ids[plan.object_id] = time.monotonic()
                        scheduler.tracker.remove(plan.object_id)
                    if success:
                        scheduler.mark_completed(plan)
                    else:
                        goto_end = plan.trajectory_goto[-1]
                        scheduler.current_position = (
                            goto_end.x,
                            goto_end.y,
                            goto_end.z,
                        )
            elif settings.adaptive_speed_enabled:
                # No pick in flight: let the belt ramp back toward v_cap (sparse
                # density) so the next objects are fetched fast. Off the gate path.
                _commit_adaptive_speed(
                    executor.dispatch, state,
                    min_interval_s=_OPPORTUNISTIC_COMMIT_INTERVAL_S,
                )

            if hasattr(image_processing, "render_window"):
                if not image_processing.render_window():
                    print("\n[INFO] Vision window closed by user (q)")
                    break

            time.sleep(settings.poll_interval_s)
    except KeyboardInterrupt:
        print("\n[INFO] Scheduler scenario interrupted by user")
    finally:
        state.stop_event.set()
        perception_thread.join(timeout=2.0)
        if hasattr(image_processing, "stop"):
            image_processing.stop()
        if hasattr(image_processing, "close_window"):
            image_processing.close_window()

    print("[INFO] Scheduler metrics:", json.dumps(scheduler.metrics.as_dict(), ensure_ascii=True))


def _realtime_perception_loop(
    state: RealtimeState,
    perceive_tick: Callable[[float | None], SpeedSample],
) -> None:
    while not state.stop_event.is_set():
        try:
            perceive_tick(time.monotonic())
        except Exception as exc:
            print(f"[WARN] realtime perception tick failed: {exc}", flush=True)
        time.sleep(0.025)


def run_scheduler_scenario(
    scenario_name: str,
    *,
    duration_s: float | None,
    interpolar_points: int,
    executor: SimulatedExecutor | RealtimePickExecutor | NullExecutor | None = None,
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
    _vision_scenarios = ("production", "test_vision_only")

    # Scenarios that drive the real robot/belt and therefore require live PLC
    # feedback. test_vision_only is camera-only (robot idle, belt static) and may
    # run without a PLC, so it is intentionally excluded here.
    _plc_required_scenarios = ("production",)

    # Fail fast (before opening the camera) if a belt scenario was started without
    # a live PLC executor — these scenarios must use real belt feedback.
    if scenario_name in _plc_required_scenarios and executor is None:
        raise RuntimeError(
            f"Scenario '{scenario_name}' requires live PLC conveyor_position feedback; "
            "do not use --simulate-executor for real scenarios."
        )
    if scenario_name in _plc_required_scenarios and not isinstance(executor, RealtimePickExecutor):
        raise RuntimeError(
            f"Scenario '{scenario_name}' requires RealtimePickExecutor for the rebuilt real path."
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
                "throughput_spawn_y": settings.throughput_spawn_y,
                "throughput_emit_interval_s": settings.throughput_emit_interval_s,
                "accuracy_emit_interval_s": settings.accuracy_emit_interval_s,
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
        camera_window_uv=settings.camera_window_uv,
    )
    decoder = BeltPositionTracker(velocity_ema_alpha=settings.belt_velocity_ema_alpha)
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

    # Belt scenarios must use live PLC conveyor_position feedback — never a
    # fabricated belt speed. Fail loudly if the wiring fell back to SimulatedSpeedSource.
    # test_vision_only is camera-only (static belt) so it is allowed to use it.
    if scenario_name in _plc_required_scenarios and isinstance(speed_source, SimulatedSpeedSource):
        raise RuntimeError(
            f"Scenario '{scenario_name}' requires live PLC conveyor_position feedback; "
            "do not use --simulate-executor for real scenarios."
        )

    scheduler = PickScheduler(settings, interpolar_points, frame, tracker, scenario_name)

    print(f"[INFO] Running scheduler scenario: {scenario_name}")
    print(f"[INFO] Fixed PLC slot count: {interpolar_points}")
    if duration_s is None:
        print("[INFO] Scenario will run until interrupted")
    else:
        print(f"[INFO] Scenario duration: {duration_s:.2f}s")

    if scenario_name in _plc_required_scenarios:
        _run_realtime_pick_loop(
            scenario_name,
            settings,
            interpolar_points,
            executor,
            image_processing,
            frame,
            scheduler,
            speed_source,
            start_time,
            duration_s,
            event_sink,
        )
        return

    # Single-thread perception tick for simulated and camera-only scenarios. The
    # rebuilt real pick path returns above and uses its dedicated perception thread.
    def perceive_tick(now: float | None = None) -> SpeedSample:
        if now is None:
            now = time.monotonic()
        sample = speed_source.sample(now)
        scheduler.update_speed(sample)
        if event_sink is not None:
            u_max = settings.workspace_window_uv[1]
            n_density = sum(
                1
                for obj in scheduler.tracker.objects()
                if 0.0 <= obj.current_uv(sample.position_mm)[0] <= u_max
            )
            round_trip = getattr(executor, "round_trip", None)
            pick_cycle_s = sum(cycle_time_samples) / len(cycle_time_samples) if cycle_time_samples else 0.0
            status_payload = {
                "scenario": scenario_name,
                "vx": round(sample.vx, 3),
                "vy": round(sample.vy, 3),
                "speed_mm_s": round(math.hypot(sample.vx, sample.vy), 3),
                "position_mm": round(sample.position_mm, 2),
                "object_density": n_density,
                "round_trip_latency_s": round(round_trip.average_s, 4) if round_trip is not None else 0.0,
                "pick_cycle_s": round(pick_cycle_s, 3),
            }
            # Robot end-effector pose for the dashboard charts. Available from
            # the live PLC status (test_vision_only / test_accuracy / test_acceptance
            # / production); absent in --simulate-executor runs (no real pos_EE).
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
        scheduler.ingest_detections(
            detections, sample.position_mm,
            position_at=getattr(speed_source, "position_at", None),
        )

        # Emit a snapshot of every tracked object's real R-frame position so the
        # web dashboard (--interface) can show objects moving live on the belt.
        # Snapshot BEFORE prune so a freshly ingested object is reported at least
        # once even if prune is about to drop it this loop (e.g. it already sits
        # past workspace u_max).
        tracked_objs = []
        for obj in scheduler.tracker.objects():
            u_now, _ = obj.current_uv(sample.position_mm)
            x_r, y_r = scheduler.tracker.current_position_R(obj, sample.position_mm)
            tracked_objs.append({
                "id": obj.object_id,
                "type": obj.object_type,
                "x": round(x_r, 2),
                "y": round(y_r, 2),
                "u": round(u_now, 1),
                "zone": _belt_zone_label(u_now, settings),
                "vision_angle_deg": round(obj.vision_angle_deg, 2),
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
        return sample

    deadline = None if duration_s is None else start_time + duration_s
    acceptance_cycles: list[dict[str, Any]] = []  # test_acceptance only
    cycle_time_samples: deque[float] = deque(maxlen=20)
    try:
        while True:
            now = time.monotonic()
            if deadline is not None and now >= deadline:
                break

            # Pump one perception tick (sample belt, poll vision, ingest/re-anchor,
            # snapshot for the dashboard, prune). Same path the executor pumps via
            # realtime path uses a dedicated perception thread, so only the
            # single-thread simulated/camera-only loop reaches this call.
            perceive_tick(now)

            # test_vision_only is pure camera observation (no arm). plan_next()
            # selects a pickable object and immediately removes it from the
            # tracker; with the NullExecutor "picking" instantly, that drained the
            # tracker the moment an object became pickable in the workspace, so the
            # dashboard only ever listed the not-yet-pickable objects upstream in
            # the ROI. Skip planning entirely so every object stays on the belt and
            # the list spans the whole ROI → workspace journey.
            plan = None if scenario_name == "test_vision_only" else scheduler.plan_next(now)
            if plan is not None:
                plan_summary = plan.to_summary()
                print("[PLAN]", json.dumps(plan_summary, ensure_ascii=True))
                if event_sink is not None:
                    event_sink("plan", plan_summary)
                # EvaluateExecutor (test_accuracy/test_acceptance real-hardware backend)
                # owns a metrics object that accumulates phase_wall_times/phase_distances
                # across calls — capture the length before so we can slice out exactly
                # this cycle's (goto, pick) pair below. None on every other executor.
                metrics_before = (
                    len(executor.metrics.phase_wall_times) if hasattr(executor, "metrics") else None
                )
                cycle_t0 = time.monotonic()
                try:
                    executor.execute(
                        plan,
                        log_samples=scenario_name in _ACCURACY_SCENARIOS,
                        real_time=False,
                        scenario_name=scenario_name,
                    )
                except Exception as exc:
                    plan.status = "failed"
                    print(f"[ERROR] scheduler execution failed: {exc}")
                    if hasattr(image_processing, "notify_pick_finished"):
                        image_processing.notify_pick_finished(plan.object_id)
                    break
                cycle_time_samples.append(time.monotonic() - cycle_t0)
                scheduler.mark_completed(plan)
                if hasattr(image_processing, "notify_pick_finished"):
                    image_processing.notify_pick_finished(plan.object_id)

                if scenario_name == "test_acceptance":
                    cycle_no = len(acceptance_cycles) + 1
                    cycle_record: dict[str, Any] = {"cycle": cycle_no, "object_id": plan.object_id}
                    if metrics_before is not None:
                        new_phases = zip(
                            ("goto", "pick"),
                            executor.metrics.phase_wall_times[metrics_before:],
                            executor.metrics.phase_distances[metrics_before:],
                        )
                        for phase_name, wall_s, distance_mm in new_phases:
                            accept_payload = {
                                "cycle": cycle_no,
                                "object_id": plan.object_id,
                                "phase": phase_name,
                                "wall_s": round(wall_s, 4),
                                "distance_mm": round(distance_mm, 2),
                            }
                            print("[ACCEPT]", json.dumps(accept_payload, ensure_ascii=True))
                            if event_sink is not None:
                                event_sink("accept_phase", accept_payload)
                            cycle_record[f"{phase_name}_s"] = round(wall_s, 4)
                            cycle_record[f"{phase_name}_distance_mm"] = round(distance_mm, 2)
                    acceptance_cycles.append(cycle_record)

                    if len(acceptance_cycles) >= settings.test_acceptance_cycles:
                        summary_payload: dict[str, Any] = {"per_cycle": acceptance_cycles}
                        if hasattr(executor, "metrics"):
                            summary_payload.update(executor.metrics.as_dict())
                        else:
                            summary_payload["cycles_completed"] = len(acceptance_cycles)
                        print("[ACCEPT-SUMMARY]", json.dumps(summary_payload, ensure_ascii=True))
                        if event_sink is not None:
                            event_sink("accept_summary", summary_payload)
                        break

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

    # Computed-config suggestion: fires for the EvaluateExecutor-backed accuracy/
    # acceptance runs (real hardware), turning this run's per-phase timing into
    # calibrated config values so the oversized config can be re-tuned from data.
    eval_metrics = getattr(executor, "metrics", None)
    if isinstance(eval_metrics, EvaluateMetrics) and eval_metrics.phase_wall_times:
        round_trip = getattr(executor, "round_trip", None)
        suggest = eval_metrics.config_suggestions(
            round_trip.average_s if round_trip is not None else 0.0,
            settings.nominal_xy_speed,
            simulated=False,
        )
        print("[CONFIG-SUGGEST]", json.dumps(suggest, ensure_ascii=True))
        if event_sink is not None:
            event_sink("config_suggest", suggest)

    print("[INFO] Scheduler metrics:", json.dumps(scheduler.metrics.as_dict(), ensure_ascii=True))
