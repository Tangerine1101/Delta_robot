"""Typed configuration: every key of `modules/config.yaml`, its type and its one default.

The YAML tree maps one-to-one onto the nested frozen dataclasses below; a field name is the
YAML key. `load_settings()` builds them with one generic loader that coerces types and
refuses anything it does not know:

* an unknown key is an error (a typo must not silently fall back to a default),
* a key that moved or was removed is an error naming its new place (`MOVED_KEYS`),
* a field without a default is required: cell geometry and calibration values have no
  sensible default, so a config that omits them does not run.

A default here is not a claim that the value was measured; `doc/open-issues.md` §A–§B says
which ones are pending calibration.

Per-plugin sections (`scheduling.planners.<name>`, `speed.laws.<name>`) stay plain dicts
here: the plugin registry owns their schema (`modules/scheduling/registry.py`), so adding a
plugin never touches this file.
"""

from __future__ import annotations

import dataclasses
import math
import types
import typing
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from modules.config_io import CONFIG_PATH, ConfigError, read_config

Point = tuple[float, float, float]
UVWindow = tuple[float, float, float, float]      # (u_min, u_max, v_min, v_max)


class SettingsError(ValueError):
    """The configuration cannot be used as it is."""


# ---------------------------------------------------------------------------
# Sections
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Endpoint:
    ip: str
    port: int


@dataclass(frozen=True)
class PlcSettings:
    omron: Endpoint                       # EtherNet/IP (arm + suction)
    siemens: Endpoint                     # snap7 (belt speed + 4th-DOF rotation)
    # Length of every Omron packet waypoint array; equals the PLC DB array size.
    interpolar_points: int = 7


@dataclass(frozen=True)
class PoseStreamSettings:
    enabled: bool = False
    port: int = 9001
    stale_s: float = 0.1
    omron_status_period_s: float = 0.2


@dataclass(frozen=True)
class LoggingSettings:
    enabled: bool = False
    dir: str = "log"


@dataclass(frozen=True)
class RobotLimits:
    """PC-side motion envelope: a vertical cylinder around the robot origin. Every
    absolute-position command is checked against it before it reaches the PLC; the PLC's own
    IK/joint-limit handling is not trusted to reject a bad point (open-issues O1)."""

    radius_xy_mm: float = 180.0
    z_min_mm: float = -305.0
    z_max_mm: float = -220.0

    def violation(self, x: float, y: float, z: float) -> str | None:
        radius = math.hypot(x, y)
        if radius > self.radius_xy_mm:
            return f"XY radius {radius:.1f} mm > robot.limits.radius_xy_mm {self.radius_xy_mm:.1f}"
        if not self.z_min_mm <= z <= self.z_max_mm:
            return f"z={z:.1f} mm outside robot.limits [{self.z_min_mm:.1f}, {self.z_max_mm:.1f}]"
        return None


@dataclass(frozen=True)
class Heights:
    """Heights of the 7-point templates, robot frame (mm, negative = down)."""

    clearance: float                      # travel height between picks
    slope_transition: float               # where the slanted approach turns vertical
    pre_pick: float                       # park height above the part
    pickup: float                         # suction-cup contact height
    place: float                          # release height over the bin


@dataclass(frozen=True)
class Interpolator:
    """The PLC's MC_Inter_Curve_Vel limits: the scheduler's model of every motion time.
    Defaults are the Matching_Code_10 program's constants (open-issues C5)."""

    v_max: float = 300.0                  # mm/s
    a_max: float = 1000.0                 # mm/s^2
    d_max: float = 1000.0                 # mm/s^2
    soft_start_s: float = 0.08            # State-10 soft start
    scurve_shape_factor: float = 1.5      # S-curve time stretch over the trapezoid


@dataclass(frozen=True)
class PacketTime:
    """Values written into each packet's `argument_time`. The Omron firmware ignores that
    field, but the pick executor sums it for its arrival timeout, so it must stay sane."""

    nominal_xy_speed: float = 220.0       # mm/s
    nominal_z_speed: float = 180.0        # mm/s
    release_descent_time_s: float = 0.14  # floor for the place-phase descent


@dataclass(frozen=True)
class Rotation:
    """Post-grip rotation chain of the Siemens suction axis (basis-theory §5)."""

    sign: float = 1.0                     # +1 / -1 axis direction (open-issues C1)
    offset_deg: float = 0.0               # bin orientation (open-issues C4)
    home_tolerance_deg: float = 0.0       # warn when the cup is not home at the gate; 0 = off
    refresh_max_delta_deg: float = 15.0   # largest late heading refresh accepted; 0 = off


@dataclass(frozen=True)
class RobotSettings:
    home_position: Point
    heights: Heights
    corner_blend_xy: float                # XY corner blend of the 7-point templates (mm)
    limits: RobotLimits = RobotLimits()
    interpolator: Interpolator = Interpolator()
    packet_time: PacketTime = PacketTime()
    rotation: Rotation = Rotation()


@dataclass(frozen=True)
class FrameSettings:
    theta_deg: float                      # rotation of the belt axis in the robot frame
    robot_origin_uv: tuple[float, float]  # robot origin in belt coordinates (mm)


@dataclass(frozen=True)
class ConveyorSettings:
    frame: FrameSettings
    camera_window_uv: UVWindow            # region the camera sees (belt frame, mm)
    workspace_window_uv: UVWindow         # region the arm may pick in (belt frame, mm)
    # Real Siemens belt feedback -> true mm and mm/s (open-issues C8). Not applied to the
    # simulator.
    position_scale_mm: float = 1.0
    velocity_ema_alpha: float = 0.4       # EMA of the measured belt velocity
    accel_mm_s2: float = 22.31            # drive ramp; forecasts and the simulator use it
    hw_max_mm_s: float = 200.0            # drive hardware maximum


@dataclass(frozen=True)
class ObjectType:
    """Everything about one part class, in one place."""

    bin: Point                            # drop position, robot frame (mm)
    w: float = 0.0                        # footprint (mm)
    h: float = 0.0
    model_class: str = ""                 # YOLO class name; "" = the type's own name
    marker_class: str | None = None       # YOLO class of its orientation marker
    symmetry_deg: float = 180.0           # rotational symmetry of the board
    heading_offset_deg: float = 0.0       # marker placement offset (open-issues C4)


@dataclass(frozen=True)
class CaptureSettings:
    camera_usb_id: str | None = None      # USB vendor:product used to locate the camera
    device: str | None = None             # explicit /dev/videoN; None = from camera_usb_id
    width: int = 1920
    height: int = 1080
    pixelformat: str = "mjpeg"
    fps: int = 30


@dataclass(frozen=True)
class RoiSettings:
    enabled: bool = False
    polygon: tuple[tuple[int, int], ...] = ()


@dataclass(frozen=True)
class TriggerLineSettings:
    y_px: int = 540
    direction: str = "down"
    min_conf: float | None = None         # None = vision.conf


@dataclass(frozen=True)
class OrientationSettings:
    enabled: bool = False
    cross_check: bool = False             # take the board type from its marker
    marker_max_dist_mm: float = 30.0      # a farther marker is not this board's


@dataclass(frozen=True)
class VisionTrackerSettings:
    max_match_dist_px: float = 80.0
    max_missing_frames: int = 15


@dataclass(frozen=True)
class BeltEstimatorSettings:
    """Belt speed from track motion — informational; the scheduler uses the PLC encoder."""

    enabled: bool = True
    ema_alpha: float = 0.3
    min_track_frames: int = 3
    axis: str = "y"


@dataclass(frozen=True)
class VisionSettings:
    model_weights: str = "models/nano@1280/weights/best.pt"
    imgsz: int = 1280
    conf: float = 0.6
    conf_marker: float | None = None      # None = conf
    iou: float = 0.7
    device: str = "0"
    half: bool = True
    show_window: bool = True
    mjpeg_jpeg_quality: int = 80
    pixels_per_mm: float = 4.0
    capture: CaptureSettings = CaptureSettings()
    v4l2_controls: dict[str, int] | None = None
    roi: RoiSettings = RoiSettings()
    trigger_line: TriggerLineSettings = TriggerLineSettings()
    orientation: OrientationSettings = OrientationSettings()
    tracker: VisionTrackerSettings = VisionTrackerSettings()
    belt_estimator: BeltEstimatorSettings = BeltEstimatorSettings()


@dataclass(frozen=True)
class PickGateSettings:
    """Everything the pick gate and the pick executor read (basis-theory §4.4)."""

    # Dispatch -> motion latency; their sum is the gate's lead offset (open-issues C2).
    robot_movement_delay_s: float
    ethernet_delay_s: float
    pick_descent_time_s: float = 0.0      # extra lead for the vertical descent (T1)
    late_abort_mm: float = 12.0           # abort and re-queue a gate this late; <= 0 disables
    # Arm-arrival tolerance, linear in belt speed between speed.band.min_mm_s and
    # speed.band.max_mm_s (open-issues G6). None = the floor everywhere.
    arrival_tolerance_mm: float = 5.0
    arrival_tolerance_max_mm: float | None = None
    arrival_timeout_margin_s: float = 0.3  # slack on the modelled flight time before giving up
    oblique_descent_enabled: bool = False  # belt-tracking slanted descent (open-issues C3)
    intercept_lead_time_s: float = 1.6    # minimum time from planning to predicted contact

    @property
    def command_delay_s(self) -> float:
        return self.robot_movement_delay_s + self.ethernet_delay_s


@dataclass(frozen=True)
class RuntimeSettings:
    poll_interval_s: float = 0.05         # main-loop / status poll period
    stale_timeout_s: float = 5.0          # drop a track not re-seen this long in camera view
    speed_timeout_s: float = 1.0          # an older belt speed sample blocks planning
    # Part outcome record (runtime/outcomes.py): a pick counts as gripped when the part's
    # tracked centre is this close to the cup at contact (open-issues L17).
    grip_tolerance_mm: float = 12.7
    flow_bin_s: float = 10.0              # time bin of flow.csv (input / throughput per bin)


@dataclass(frozen=True)
class ArmCycleSettings:
    """What the speed laws assume about one pick cycle (open-issues G2, C7)."""

    cycle_s: float = 2.0                  # t_pick
    occupancy_worst_s: float = 3.0        # worst arm occupancy per pick
    grab_worst_s: float = 1.5             # worst dispatch -> contact


@dataclass(frozen=True)
class SchedulingSettings:
    planner: str = "spt"                  # any registered dispatch rule or planner
    safety_margin_s: float = 0.1          # margin on every deadline
    setup_time_s: float = 0.0             # executor overhead per pick beyond the motion model
    arm_cycle: ArmCycleSettings = ArmCycleSettings()
    planners: dict[str, dict[str, Any]] = field(default_factory=dict)


@dataclass(frozen=True)
class SpeedBand:
    min_mm_s: float = 30.0                # operational floor, every law
    max_mm_s: float = 0.0                 # operational ceiling; <= 0 = none below hw_max


@dataclass(frozen=True)
class SpeedCommit:
    deadband_mm_s: float = 8.0            # smaller target changes are not sent
    max_step_mm_s: float = 20.0           # largest setpoint change per commit; 0 = no limit


@dataclass(frozen=True)
class SpeedSettings:
    law: str = "constant"                 # any registered speed law
    setpoint_gate: str | None = None      # None = the law's own default gate
    static_mm_s: float = 50.0             # `constant` speed; startup setpoint of the others
    control_period_s: float = 1.0         # re-decision period of the `periodic` gate
    band: SpeedBand = SpeedBand()
    commit: SpeedCommit = SpeedCommit()
    laws: dict[str, dict[str, Any]] = field(default_factory=dict)


@dataclass(frozen=True)
class PlcSimSettings:
    """Feeder of the PLC simulator (`main.py --sim`)."""

    feed_types: tuple[str, ...] = ("QFP", "TQFP")
    feed_lanes: tuple[float, ...] = (20.0, 60.0, 100.0)   # v of each lane (mm)


@dataclass(frozen=True)
class FeederSettings:
    """Virtual parts of the `simulate_feeder` scenario (runtime/virtual_feed.py): arrival
    times, lanes and types drawn from a seed by a feeder of modules/core/feeders.py."""

    kind: str = "poisson"                 # any registered feeder
    seed: int = 1
    max_parts: int = 0                    # stop feeding after this many parts; 0 = no limit
    schedule_s: float = 3600.0            # arrivals drawn for this long when the run has no duration
    detection_period_s: float = 1.0 / 30.0  # the virtual camera's frame period
    detection_latency_s: float = 0.0      # capture-to-poll delay the virtual camera reports
    position_noise_mm: float = 0.0        # Gaussian noise on each virtual detection (u, v)
    kinds: dict[str, dict[str, Any]] = field(default_factory=dict)   # feeder.kinds.<kind>


@dataclass(frozen=True)
class InterfaceSettings:
    port: int = 8000
    mjpeg_fps: float = 15.0


@dataclass(frozen=True)
class Settings:
    plc: PlcSettings
    robot: RobotSettings
    conveyor: ConveyorSettings
    object_types: dict[str, ObjectType]
    pick_gate: PickGateSettings
    pose_stream: PoseStreamSettings = PoseStreamSettings()
    logging: LoggingSettings = LoggingSettings()
    vision: VisionSettings = VisionSettings()
    runtime: RuntimeSettings = RuntimeSettings()
    scheduling: SchedulingSettings = SchedulingSettings()
    speed: SpeedSettings = SpeedSettings()
    plc_sim: PlcSimSettings = PlcSimSettings()
    feeder: FeederSettings = FeederSettings()
    interface: InterfaceSettings = InterfaceSettings()

    # ---- derived views ---------------------------------------------------------------
    def bin_of(self, object_type: str) -> Point | None:
        spec = self.object_types.get(object_type)
        return spec.bin if spec is not None else None

    def model_class(self, object_type: str) -> str:
        return self.object_types[object_type].model_class or object_type

    def speed_ceiling_mm_s(self) -> float:
        """Operational belt ceiling shared by every law: band max, then hardware max."""
        ceiling = self.conveyor.hw_max_mm_s
        if self.speed.band.max_mm_s > 0.0:
            ceiling = min(ceiling, self.speed.band.max_mm_s)
        return ceiling

    def validate(self) -> None:
        """Cross-field checks. Plugin names are checked by the registry at start-up."""
        h = self.robot.heights
        order = [("clearance", h.clearance), ("slope_transition", h.slope_transition),
                 ("pre_pick", h.pre_pick), ("pickup", h.pickup)]
        for (upper, z_up), (lower, z_low) in zip(order, order[1:]):
            if z_up <= z_low:
                raise SettingsError(
                    f"robot.heights.{upper} ({z_up}) must be above robot.heights.{lower} ({z_low})")
        if h.clearance < h.place:
            raise SettingsError(
                f"robot.heights.clearance ({h.clearance}) must be at or above "
                f"robot.heights.place ({h.place})")
        limits = self.robot.limits
        if limits.z_min_mm >= limits.z_max_mm or limits.radius_xy_mm <= 0.0:
            raise SettingsError(f"invalid robot.limits {limits}")
        heights = {
            **{f"robot.heights.{name}": z for name, z in dataclasses.asdict(h).items()},
            "robot.home_position.z": self.robot.home_position[2],
            **{f"object_types.{name}.bin.z": spec.bin[2] for name, spec in self.object_types.items()},
        }
        for name, z in heights.items():
            if not limits.z_min_mm <= z <= limits.z_max_mm:
                raise SettingsError(
                    f"{name} ({z}) is outside robot.limits z [{limits.z_min_mm}, "
                    f"{limits.z_max_mm}]; every goto to it would be rejected")
        for name in ("camera_window_uv", "workspace_window_uv"):
            u_min, u_max, v_min, v_max = getattr(self.conveyor, name)
            if not (u_min < u_max and v_min < v_max):
                raise SettingsError(f"conveyor.{name} must be [u_min, u_max, v_min, v_max], "
                                    f"ascending; got {getattr(self.conveyor, name)}")
        if not self.object_types:
            raise SettingsError("object_types is empty: the cell has no part class to pick")


# ---------------------------------------------------------------------------
# Keys that moved or were removed. The loader refuses them with this message.
# ---------------------------------------------------------------------------

MOVED_KEYS: dict[str, str] = {
    "ip_address": "plc.omron.ip",
    "port": "plc.omron.port",
    "siemens_ip": "plc.siemens.ip",
    "siemens_port": "plc.siemens.port",
    "interpolar_points": "plc.interpolar_points",
    "robot_limits": "robot.limits",
    "limit_radius_xy": "robot.limits.radius_xy_mm",
    "conveyor.conveyor_position_scale_mm": "conveyor.position_scale_mm",
    "conveyor.accuracy_points_uv": "removed with the evaluate scenario",
    "object_types.*.destination": "object_types.<type>.bin (the drop position itself)",
    "vision.class_map": "object_types.<type>.model_class",
    "vision.orientation.pcb_classes": "object_types (every type is a board class)",
    "vision.orientation.marker_map": "object_types.<type>.marker_class",
    "vision.orientation.offset_by_class": "object_types.<type>.heading_offset_deg",
    "vision.orientation.symmetry_by_class": "object_types.<type>.symmetry_deg",
    "vision.orientation.offset_deg": "object_types.<type>.heading_offset_deg",
    "vision.orientation.symmetry_deg": "object_types.<type>.symmetry_deg",
    "scheduler": "split into robot, pick_gate, runtime, scheduling and speed",
    "scheduler.planner": "scheduling.planner",
    "scheduler.speed_law": "speed.law",
    "scheduler.adaptive_speed_enabled": "speed.law",
    "scheduler.belt_speed_static_mm_s": "speed.static_mm_s",
    "scheduler.home_position": "robot.home_position",
    "scheduler.clearance_height": "robot.heights.clearance",
    "scheduler.slope_transition_height": "robot.heights.slope_transition",
    "scheduler.pickup_height": "robot.heights.pickup",
    "scheduler.pre_pick_height": "robot.heights.pre_pick",
    "scheduler.place_height": "robot.heights.place",
    "scheduler.corner_blend_xy": "robot.corner_blend_xy",
    "scheduler.interpolator": "robot.interpolator",
    "scheduler.nominal_xy_speed": "robot.packet_time.nominal_xy_speed",
    "scheduler.nominal_z_speed": "robot.packet_time.nominal_z_speed",
    "scheduler.release_descent_time_s": "robot.packet_time.release_descent_time_s",
    "scheduler.rotate_sign": "robot.rotation.sign",
    "scheduler.rotate_offset_deg": "robot.rotation.offset_deg",
    "scheduler.rotate_home_tolerance_deg": "robot.rotation.home_tolerance_deg",
    "scheduler.rotate_refresh_max_delta_deg": "robot.rotation.refresh_max_delta_deg",
    "scheduler.intercept_lead_time_s": "pick_gate.intercept_lead_time_s",
    "scheduler.robot_movement_delay_s": "pick_gate.robot_movement_delay_s",
    "scheduler.ethernet_delay_s": "pick_gate.ethernet_delay_s",
    "scheduler.pick_descent_time_s": "pick_gate.pick_descent_time_s",
    "scheduler.gate_late_abort_mm": "pick_gate.late_abort_mm",
    "scheduler.pick_arrival_tolerance_mm": "pick_gate.arrival_tolerance_mm",
    "scheduler.pick_arrival_tolerance_max_mm": "pick_gate.arrival_tolerance_max_mm",
    "scheduler.execution_margin_s": "pick_gate.arrival_timeout_margin_s",
    "scheduler.oblique_descent_enabled": "pick_gate.oblique_descent_enabled",
    "scheduler.stale_timeout_s": "runtime.stale_timeout_s",
    "scheduler.speed_timeout_s": "runtime.speed_timeout_s",
    "scheduler.poll_interval_s": "runtime.poll_interval_s",
    "scheduler.pick_cycle_s": "scheduling.arm_cycle.cycle_s",
    "scheduler.pick_transit_min_s": "speed.laws.inverse_density.transit_min_s",
    "scheduler.belt_speed_headroom": "speed.laws.inverse_density.headroom",
    "scheduler.belt_density_length_mm": "speed.laws.inverse_density.density_length_mm",
    "scheduler.belt_speed_min_mm_s": "speed.band.min_mm_s",
    "scheduler.belt_speed_max_mm_s": "speed.band.max_mm_s",
    "scheduler.belt_speed_hw_max_mm_s": "conveyor.hw_max_mm_s",
    "scheduler.belt_speed_deadband_mm_s": "speed.commit.deadband_mm_s",
    "scheduler.belt_speed_max_step_mm_s": "speed.commit.max_step_mm_s",
    "scheduler.belt_accel_mm_s2": "conveyor.accel_mm_s2",
    "scheduler.planning": "scheduling (safety_margin_s, setup_time_s)",
    "scheduler.predictive_rank": "speed.laws.predictive_rank and scheduling.arm_cycle",
    "scheduler.default_speed": "removed with the single-thread harness",
    "scheduler.log_path": "removed; run logs live under logging.dir",
    "scheduler.test_acceptance_cycles": "removed with the test_acceptance scenario",
    "scheduler.accuracy_spawn_uv": "removed with the test_accuracy scenario",
    "scheduler.accuracy_object_types": "removed with the test_accuracy scenario",
    "scheduler.accuracy_emit_interval_s": "removed with the test_accuracy scenario",
    "scheduler.throughput_object_types": "plc_sim.feed_types",
    "scheduler.throughput_lanes": "plc_sim.feed_lanes",
    "scheduler.throughput_spawn_y": "removed with the test_throughput scenario",
    "scheduler.throughput_emit_interval_s": "removed with the test_throughput scenario",
    "scheduler.evaluate_position_tolerance_mm": "removed with the evaluate scenario",
    "scheduler.evaluate_wait_timeout_s": "removed with the evaluate scenario",
}


# ---------------------------------------------------------------------------
# Generic loader
# ---------------------------------------------------------------------------


def _moved_message(path: str) -> str | None:
    if path in MOVED_KEYS:
        return MOVED_KEYS[path]
    parts = path.split(".")
    for i in range(len(parts)):
        wildcard = ".".join(parts[:i] + ["*"] + parts[i + 1:])
        if wildcard in MOVED_KEYS:
            return MOVED_KEYS[wildcard]
    return None


def _type_name(tp: Any) -> str:
    return getattr(tp, "__name__", None) or str(tp).replace("typing.", "")


def _coerce(value: Any, tp: Any, path: str) -> Any:
    origin = typing.get_origin(tp)
    args = typing.get_args(tp)

    if tp is Any:
        return value
    if isinstance(tp, type) and dataclasses.is_dataclass(tp) and isinstance(value, tp):
        return value                # already built (with_overrides of a whole section)
    if origin in (typing.Union, types.UnionType):
        if value is None and type(None) in args:
            return None
        errors = []
        for option in args:
            if option is type(None):
                continue
            try:
                return _coerce(value, option, path)
            except SettingsError as exc:
                errors.append(str(exc))
        raise SettingsError(errors[0] if len(errors) == 1 else f"{path}: {value!r} fits none of "
                            f"{', '.join(_type_name(a) for a in args)}")
    if dataclasses.is_dataclass(tp):
        return _build(tp, value, path)
    if value is None:
        raise SettingsError(f"{path}: a value is required, got null")
    if origin is dict:
        if not isinstance(value, dict):
            raise SettingsError(f"{path}: expected a mapping, got {value!r}")
        key_tp, item_tp = args or (str, Any)
        return {_coerce(k, key_tp, path): _coerce(v, item_tp, f"{path}.{k}") for k, v in value.items()}
    if origin in (tuple, list):
        if not isinstance(value, (list, tuple)):
            raise SettingsError(f"{path}: expected a list, got {value!r}")
        if origin is tuple and args and args[-1] is not Ellipsis:
            if len(value) != len(args):
                raise SettingsError(f"{path}: expected {len(args)} values, got {len(value)}")
            return tuple(_coerce(v, a, f"{path}[{i}]") for i, (v, a) in enumerate(zip(value, args)))
        item_tp = args[0] if args else Any
        items = [_coerce(v, item_tp, f"{path}[{i}]") for i, v in enumerate(value)]
        return tuple(items) if origin is tuple else items
    if tp is bool:
        if isinstance(value, bool):
            return value
        raise SettingsError(f"{path}: expected true/false, got {value!r}")
    if tp in (int, float):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise SettingsError(f"{path}: expected a number, got {value!r}")
        if tp is int and float(value) != int(value):
            raise SettingsError(f"{path}: expected an integer, got {value!r}")
        return tp(value)
    if tp is str:
        if isinstance(value, (dict, list)):
            raise SettingsError(f"{path}: expected text, got {value!r}")
        return str(value)
    raise SettingsError(f"{path}: unsupported setting type {tp!r}")


def _build(cls: type, data: Any, path: str) -> Any:
    label = path or "config"
    if data is None:
        data = {}
    if not isinstance(data, dict):
        raise SettingsError(f"{label}: expected a mapping, got {data!r}")
    hints = typing.get_type_hints(cls)
    fields = {f.name: f for f in dataclasses.fields(cls)}

    for key in data:
        if key not in fields:
            dotted = f"{path}.{key}" if path else str(key)
            moved = _moved_message(dotted)
            if moved is not None:
                raise SettingsError(f"{dotted}: moved or removed -> {moved}")
            raise SettingsError(f"{dotted}: unknown key. Known keys of {label}: "
                                f"{', '.join(sorted(fields))}")

    kwargs = {}
    for name, f in fields.items():
        dotted = f"{path}.{name}" if path else name
        if name in data:
            kwargs[name] = _coerce(data[name], hints[name], dotted)
        elif f.default is dataclasses.MISSING and f.default_factory is dataclasses.MISSING:
            if dataclasses.is_dataclass(hints[name]):
                kwargs[name] = _build(hints[name], {}, dotted)
            else:
                raise SettingsError(f"{dotted}: required key is missing")
    return cls(**kwargs)


def build_section(cls: type, data: Any, path: str) -> Any:
    """Build one config dataclass from a plain mapping with the same rules as the whole file
    (plugin registries use it for `scheduling.planners.<name>` / `speed.laws.<name>`)."""
    return _build(cls, data, path)


def _check_object_types(raw: dict[str, Any]) -> None:
    for name, spec in (raw.get("object_types") or {}).items():
        if isinstance(spec, dict) and "destination" in spec:
            raise SettingsError(f"object_types.{name}.destination: moved or removed -> "
                                f"{MOVED_KEYS['object_types.*.destination']}")


def settings_from_dict(raw: dict[str, Any]) -> Settings:
    """Build and validate Settings from a plain config tree."""
    _check_object_types(raw)
    settings = _build(Settings, raw, "")
    settings.validate()
    return settings


def load_settings(path: str | Path | None = None) -> Settings:
    """Read `modules/config.yaml` (or `path`) into Settings. Any problem is fatal."""
    config_path = Path(path) if path is not None else CONFIG_PATH
    try:
        raw = read_config(config_path)
    except FileNotFoundError as exc:
        raise SystemExit(f"[ERROR] config file not found: {config_path}") from exc
    except ConfigError as exc:
        raise SystemExit(f"[ERROR] invalid config file {config_path}: {exc}") from exc
    try:
        return settings_from_dict(raw if isinstance(raw, dict) else {})
    except SettingsError as exc:
        raise SystemExit(f"[ERROR] config {config_path}: {exc}") from exc


def with_overrides(settings: Settings, overrides: dict[str, Any]) -> Settings:
    """A copy with dotted-path values replaced, e.g. {"speed.law": "predictive_rank"}.
    Used by tests, the sandbox and one-off runs; the config file is never written."""
    for dotted, value in overrides.items():
        settings = _replace_path(settings, dotted.split("."), value, dotted)
    settings.validate()
    return settings


def _replace_path(obj: Any, parts: list[str], value: Any, dotted: str) -> Any:
    head, rest = parts[0], parts[1:]
    if isinstance(obj, dict):
        current = obj.get(head, {})
        new = dict(obj)
        new[head] = _replace_path(current, rest, value, dotted) if rest else value
        return new
    if not dataclasses.is_dataclass(obj) or head not in {f.name for f in dataclasses.fields(obj)}:
        raise SettingsError(f"{dotted}: no such setting")
    if rest:
        return dataclasses.replace(obj, **{head: _replace_path(getattr(obj, head), rest, value, dotted)})
    hint = typing.get_type_hints(type(obj))[head]
    return dataclasses.replace(obj, **{head: _coerce(value, hint, dotted)})
