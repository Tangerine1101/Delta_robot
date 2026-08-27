"""A virtual pick-and-place cell that the *real* production loop can drive.

`modules.scheduler._run_realtime_pick_loop` is the code path that runs on the
physical rig, and `open-issues.md` L7/L8 record that it has no offline dry-run:
every experiment needs the hardware.  This module supplies the missing other
half -- a plant -- so that exact function, the exact `RealtimePickExecutor`, the
exact perception thread and the exact pick gate can be run end to end on a
laptop and *measured*.

Nothing here is imported by the repository.  It only provides the four objects
the production loop takes as arguments:

    dispatch(packet)      -> a fake Omron + Siemens accepting the real packets
    request_status()      -> a fake DB2 / pos_EE status word
    image_processing.poll -> a fake camera emitting `ObjectDetection`s
    a ground-truth log    -> where the cup actually landed, in belt coordinates

Fidelity notes, in the order they matter for pick accuracy:

* **State 10.**  `doc/PLC_Program_description/MC_inter_curve_vel.md` says the
  interpolator does *not* start segment 0 from where the arm is: on a
  stationary start it linearly ramps the joint setpoints from the measured
  servo position to the IK solution of `Pos[0]` over exactly 20 scans (80 ms),
  then begins `Pos[0] -> Pos[1]`.  For the pick phase `Pos[0]` *is* the contact
  point, so this 80 ms bridge is the entire park->board descent.  The plant
  models it that way, with an optional extra `servo_lag_s` because an 80 ms
  linear ramp over 16 mm is a velocity step the drive cannot track exactly.

* **Two-pass segment scheduling.**  The plant caps each corner at the speed from
  which it can still stop by the following waypoint, then at the speed it can
  actually accelerate to.  That is what a machine must do; the repo's own
  `_trajectory_total_time` skips the first of those passes, which is exactly the
  kind of model/plant gap this harness exists to expose.

* **Model vs plant.**  `PlantProfile` is independent of
  `config.json > scheduler.interpolator`, so the planner's beliefs and the
  arm's behaviour can be detuned apart (C5).

* **Belt.**  Trapezoidal ramp toward the commanded setpoint at
  `belt_accel_mm_s2`, so `change_speed` is not instantaneous.

* **Wire.**  Every dispatch and every status read costs a round trip.
"""

from __future__ import annotations

import math
import random
import threading
import time
from dataclasses import dataclass, field
from typing import Any

from modules.EthernetCom import COMMAND_ID
from modules.image_processing import ObjectDetection

Point = tuple[float, float, float]


# Physically correct multi-segment scheduling

def two_pass_schedule(points: list[Point], v_max: float, a_max: float,
                      d_max: float, shape: float) -> list[float]:
    """Per-segment durations with both the stop-distance and the reach clamp.

    Backward pass first: a corner is worth nothing if the arm cannot bleed the
    speed off before the next waypoint.  Forward pass second: cap again at what
    it can accelerate to.  Only the last waypoint is a full stop.
    """
    n_seg = len(points) - 1
    if n_seg < 1:
        return []
    lengths = [math.dist(points[i], points[i + 1]) for i in range(n_seg)]

    v_corner: list[float] = []
    for i in range(n_seg):
        if i == n_seg - 1:
            v_corner.append(0.0)
            continue
        a, b, c = points[i], points[i + 1], points[i + 2]
        v1 = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
        v2 = (c[0] - b[0], c[1] - b[1], c[2] - b[2])
        l1 = math.sqrt(sum(t * t for t in v1))
        l2 = math.sqrt(sum(t * t for t in v2))
        if l1 <= 1e-9 or l2 <= 1e-9:
            cos_theta = 1.0
        else:
            cos_theta = sum(p * q for p, q in zip(v1, v2)) / (l1 * l2)
            cos_theta = max(-1.0, min(1.0, cos_theta))
        v_corner.append(v_max * math.sqrt(max(0.0, (cos_theta + 1.0) / 2.0)))

    for i in range(n_seg - 2, -1, -1):
        can_stop = math.sqrt(v_corner[i + 1] ** 2 + 2.0 * d_max * lengths[i + 1])
        v_corner[i] = min(v_corner[i], can_stop)

    durations: list[float] = []
    v_in = 0.0
    for i in range(n_seg):
        v_out = min(v_corner[i], math.sqrt(v_in ** 2 + 2.0 * a_max * lengths[i]))
        durations.append(_segment_time(lengths[i], v_in, v_out, v_max, a_max, d_max, shape))
        v_in = v_out
    return durations


def _segment_time(length: float, v_start: float, v_end: float, v_max: float,
                  a_max: float, d_max: float, shape: float) -> float:
    if length <= 1e-9 or v_max <= 0 or a_max <= 0 or d_max <= 0:
        return 0.0
    if v_start <= 1e-9 and v_end <= 1e-9:
        coef = 0.5 * shape
        inv_sum = 1.0 / a_max + 1.0 / d_max
        min_length = coef * v_max ** 2 * inv_sum
        v_peak = v_max if length >= min_length else math.sqrt(length / (coef * inv_sum))
        t_acc = shape * v_peak / a_max
        t_dec = shape * v_peak / d_max
        s_cruise = length - 0.5 * v_peak * t_acc - 0.5 * v_peak * t_dec
        return t_acc + max(0.0, s_cruise / v_peak if s_cruise > 0 else 0.0) + t_dec

    s_limit = (abs(v_max ** 2 - v_start ** 2) / (2.0 * a_max)
               + abs(v_max ** 2 - v_end ** 2) / (2.0 * d_max))
    if s_limit > length:
        v_peak = math.sqrt((2.0 * a_max * d_max * length + d_max * v_start ** 2
                            + a_max * v_end ** 2) / (a_max + d_max))
    else:
        v_peak = v_max
    v_peak = max(v_peak, v_start, v_end)
    t_acc = abs(v_peak - v_start) / a_max
    t_dec = abs(v_peak - v_end) / d_max
    s_cruise = length - 0.5 * (v_start + v_peak) * t_acc - 0.5 * (v_end + v_peak) * t_dec
    return t_acc + max(0.0, s_cruise / v_peak if s_cruise > 0 else 0.0) + t_dec


# Plant description

@dataclass
class PlantProfile:
    """What the arm really does, as opposed to what the planner believes.

    `command_delay_s` is the wire hop plus the servo waking up -- everything
    between `dispatch()` returning and the first commanded motion.  It is the
    physical counterpart of `robot_movement_delay_s + ethernet_delay_s`, and
    keeping them separable is the whole point: the config value is a guess
    (`open-issues.md` C2).
    """

    v_max: float = 300.0
    a_max: float = 1000.0
    d_max: float = 1000.0
    scurve_shape_factor: float = 1.5
    # State 10: 20 PLC scans of linear setpoint ramp from the measured servo
    # position to the IK solution of Pos[0].
    state10_s: float = 0.08
    # Extra time the drive needs on top of State 10 before it is physically at
    # Pos[0].  An 80 ms linear ramp across a 16 mm drop is a 200 mm/s velocity
    # step at both ends; a real drive lags it.
    servo_lag_s: float = 0.0
    command_delay_s: float = 0.10


@dataclass
class BeltPlant:
    accel_mm_s2: float = 22.31
    position_mm: float = 0.0
    speed_mm_s: float = 0.0
    setpoint_mm_s: float = 0.0

    def step(self, dt: float) -> None:
        if self.accel_mm_s2 > 0.0:
            error = self.setpoint_mm_s - self.speed_mm_s
            change = math.copysign(self.accel_mm_s2 * dt, error)
            if abs(change) >= abs(error):
                self.speed_mm_s = self.setpoint_mm_s
            else:
                self.speed_mm_s = max(0.0, self.speed_mm_s + change)
        else:
            self.speed_mm_s = self.setpoint_mm_s
        self.position_mm += self.speed_mm_s * dt


@dataclass
class Board:
    board_id: str
    board_type: str
    u_anchor: float
    v: float
    belt_anchor: float
    heading_deg: float = 0.0
    picked: bool = False
    lost: bool = False
    spawned_at: float = 0.0

    def u_at(self, belt_position: float) -> float:
        return self.u_anchor + (belt_position - self.belt_anchor)


@dataclass
class ContactRecord:
    """One measured grab: where the cup went down and where the board was."""

    t: float
    plan_object_id: str | None
    target_board_id: str | None
    belt_speed_mm_s: float
    cup_u: float
    cup_v: float
    board_u: float | None
    board_v: float | None
    hit: bool

    @property
    def along_belt_error_mm(self) -> float | None:
        """Signed: positive means the board was DOWNSTREAM of the cup, i.e. the
        grab fired too late."""
        if self.board_u is None:
            return None
        return self.board_u - self.cup_u

    @property
    def cross_belt_error_mm(self) -> float | None:
        if self.board_v is None:
            return None
        return self.board_v - self.cup_v

    @property
    def error_mm(self) -> float | None:
        if self.board_u is None or self.board_v is None:
            return None
        return math.hypot(self.board_u - self.cup_u, self.board_v - self.cup_v)


class VirtualArm:
    """A delta arm executing `go_trajectory` packets the way the PLC does."""

    def __init__(self, profile: PlantProfile, home: Point) -> None:
        self.profile = profile
        self.position: Point = home
        self.end_effector = 0
        self._pending_start: float | None = None     # command delay expiry
        self._points: list[Point] | None = None
        self._e_values: list[int] | None = None
        self._bridge_from: Point | None = None
        self._bridge_end: float = 0.0                # State 10 finishes here
        self._arrivals: list[float] = []
        self._finish: float = 0.0
        self._queued: tuple[list[Point], list[int], float] | None = None
        self.contact_callback = None                 # called at Pos[0] arrival
        self._contact_fired = True

    @property
    def busy(self) -> bool:
        return self._points is not None or self._pending_start is not None

    def dispatch(self, points: list[Point], e_values: list[int], now: float) -> None:
        """Accept a new trajectory.

        A command that arrives while the arm is still moving *preempts* the
        current one -- the Omron dispatcher is a single-scan copy into the
        interpolator's waypoint array with no busy interlock
        (`main_logic.md` Rung 4), so the pipeline restarts from wherever the
        mechanism happens to be.
        """
        self._queued = (list(points), list(e_values), now + self.profile.command_delay_s)
        self._pending_start = now + self.profile.command_delay_s
        self._points = None
        self._contact_fired = False

    def _begin(self, now: float) -> None:
        assert self._queued is not None
        points, e_values, _ = self._queued
        self._queued = None
        self._pending_start = None
        self._points = points
        self._e_values = e_values
        self._bridge_from = self.position
        # State 10: linear ramp from the actual pose to Pos[0], 20 scans, plus
        # whatever the drive needs to actually converge on it.
        self._bridge_end = now + self.profile.state10_s + self.profile.servo_lag_s
        durations = two_pass_schedule(points, self.profile.v_max, self.profile.a_max,
                                      self.profile.d_max, self.profile.scurve_shape_factor)
        t = self._bridge_end
        self._arrivals = []
        for duration in durations:
            t += duration
            self._arrivals.append(t)
        self._finish = t

    def step(self, now: float) -> None:
        if self._pending_start is not None and now >= self._pending_start:
            self._begin(now)
        if self._points is None:
            return
        points = self._points

        if now < self._bridge_end:
            span = self._bridge_end - (self._bridge_end - self.profile.state10_s
                                       - self.profile.servo_lag_s)
            frac = 1.0 - (self._bridge_end - now) / span if span > 0 else 1.0
            a, b = self._bridge_from, points[0]
            self.position = (a[0] + (b[0] - a[0]) * frac,
                             a[1] + (b[1] - a[1]) * frac,
                             a[2] + (b[2] - a[2]) * frac)
            return

        if not self._contact_fired:
            self._contact_fired = True
            self.position = points[0]
            if self._e_values:
                self.end_effector = int(self._e_values[0])
            if self.contact_callback is not None:
                self.contact_callback(self._bridge_end, points[0], self._e_values)

        if now >= self._finish:
            self.position = points[-1]
            if self._e_values:
                self.end_effector = int(self._e_values[-1])
            self._points = None
            return

        prev_point = points[0]
        prev_time = self._bridge_end
        for index, (point, arrival) in enumerate(zip(points[1:], self._arrivals)):
            if now <= arrival:
                span = arrival - prev_time
                frac = 1.0 if span <= 1e-9 else (now - prev_time) / span
                self.position = (prev_point[0] + (point[0] - prev_point[0]) * frac,
                                 prev_point[1] + (point[1] - prev_point[1]) * frac,
                                 prev_point[2] + (point[2] - prev_point[2]) * frac)
                if self._e_values and index + 1 < len(self._e_values):
                    self.end_effector = int(self._e_values[index + 1])
                return
            prev_point, prev_time = point, arrival


class VirtualCell:
    """Belt + arm + boards + wire, exposing the PLC gateway interface."""

    def __init__(
        self,
        settings,
        frame,
        *,
        plant: PlantProfile | None = None,
        wire_latency_s: float = 0.008,
        camera_latency_s: float = 0.090,
        camera_period_s: float = 1.0 / 30.0,
        camera_noise_mm: float = 0.0,
        physics_dt: float = 0.002,
        grip_tolerance_mm: float = 12.7,
        seed: int = 1,
    ) -> None:
        self.settings = settings
        self.frame = frame
        self.plant = plant or PlantProfile()
        self.wire_latency_s = wire_latency_s
        self.camera_latency_s = camera_latency_s
        self.camera_period_s = camera_period_s
        self.camera_noise_mm = camera_noise_mm
        self.physics_dt = physics_dt
        self.grip_tolerance_mm = grip_tolerance_mm
        self.rng = random.Random(seed)

        self.belt = BeltPlant(accel_mm_s2=settings.belt_accel_mm_s2)
        self.arm = VirtualArm(self.plant, settings.home_position)
        self.arm.contact_callback = self._on_contact
        self.rotate_current_deg = 0.0
        self._rotate_target_deg = 0.0

        self.boards: list[Board] = []
        self._board_counter = 0
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._history: list[tuple[float, float]] = []   # (t, belt position) ground truth

        # Which object the scheduler most recently committed to (fed by the
        # production loop's event sink), so a contact can be scored against the
        # board it was aimed at rather than the nearest one.
        self.current_plan_object_id: str | None = None

        self.contacts: list[ContactRecord] = []
        # One entry per grab command, sampled the instant it lands at the PLC:
        # (park offset still to be covered, how far the board is already past
        # the cup).  The first says whether the arm was actually parked when the
        # gate fired; the second is the lateness the gate could not see.
        self.grab_dispatches: list[tuple[float, float]] = []
        self.dispatch_log: list[tuple[float, int]] = []
        self.speed_commands: list[tuple[float, float]] = []
        self.picked = 0
        self.missed = 0
        self.spawned = 0

    # lifecycle

    def start(self) -> None:
        self._thread = threading.Thread(target=self._physics_loop, name="virtual-cell",
                                        daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _physics_loop(self) -> None:
        last = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            dt = now - last
            last = now
            with self._lock:
                self.belt.step(dt)
                self.arm.step(now)
                self._settle_rotation(dt)
                self._expire_boards()
                self._history.append((now, self.belt.position_mm))
                if len(self._history) > 20000:
                    del self._history[:10000]
            time.sleep(self.physics_dt)

    def _settle_rotation(self, dt: float) -> None:
        rate = 180.0        # deg/s, generous; rotation is off the accuracy path
        delta = self._rotate_target_deg - self.rotate_current_deg
        step = math.copysign(min(abs(delta), rate * dt), delta) if delta else 0.0
        self.rotate_current_deg += step

    def _expire_boards(self) -> None:
        u_max = self.settings.workspace_window_uv[1]
        for board in self.boards:
            if board.picked or board.lost:
                continue
            if board.u_at(self.belt.position_mm) > u_max + 30.0:
                board.lost = True
                self.missed += 1

    # boards

    def spawn(self, board_type: str, lane_v: float, now: float, heading_deg: float = 0.0) -> Board:
        with self._lock:
            self._board_counter += 1
            board = Board(
                board_id=f"vb{self._board_counter:04d}",
                board_type=board_type,
                u_anchor=0.0,
                v=lane_v,
                belt_anchor=self.belt.position_mm,
                heading_deg=heading_deg,
                spawned_at=now,
            )
            self.boards.append(board)
            self.spawned += 1
            return board

    def _board(self, board_id: str | None) -> Board | None:
        if board_id is None:
            return None
        for board in self.boards:
            if board.board_id == board_id:
                return board
        return None

    # contact scoring

    def _on_contact(self, t: float, contact_point: Point, e_values) -> None:
        """Called the instant the cup reaches Pos[0] of a trajectory.

        Only a suction-on Pos[0] is a grab: the goto phase's Pos[0] is a lift
        with `argument_e = 0`.
        """
        if not e_values or int(e_values[0]) != 1:
            return
        belt_pos = self.belt.position_mm
        cup_u, cup_v = self.frame.to_conveyor(contact_point[0], contact_point[1])
        target = self._board(self.current_plan_object_id)
        board_u = board_v = None
        hit = False
        if target is not None and not target.picked and not target.lost:
            board_u = target.u_at(belt_pos)
            board_v = target.v
            if math.hypot(board_u - cup_u, board_v - cup_v) <= self.grip_tolerance_mm:
                hit = True
                target.picked = True
                self.picked += 1
        self.contacts.append(ContactRecord(
            t=t,
            plan_object_id=self.current_plan_object_id,
            target_board_id=target.board_id if target else None,
            belt_speed_mm_s=self.belt.speed_mm_s,
            cup_u=cup_u, cup_v=cup_v,
            board_u=board_u, board_v=board_v,
            hit=hit,
        ))

    # the PLC gateway the production loop talks to

    def dispatch(self, packet: dict[str, Any]) -> dict[str, Any] | None:
        time.sleep(self.wire_latency_s)
        now = time.monotonic()
        command = int(packet.get("commandID", packet.get("CommandID", -1)))
        with self._lock:
            self.dispatch_log.append((now, command))
            if command == COMMAND_ID["go_trajectory"]:
                n = int(packet.get("argument_number", 0))
                xs = packet.get("argument_x") or []
                ys = packet.get("argument_y") or []
                zs = packet.get("argument_z") or []
                es = packet.get("argument_e") or []
                points = [(float(xs[i]), float(ys[i]), float(zs[i])) for i in range(n)]
                e_values = [int(es[i]) if i < len(es) else 0 for i in range(n)]
                if points and e_values and e_values[0] == 1:
                    self._record_grab_dispatch(points[0])
                self.arm.dispatch(points, e_values, now)
            elif command == COMMAND_ID["change_speed"]:
                speed = float(packet.get("speed", 0.0))
                self.belt.setpoint_mm_s = max(0.0, speed)
                self.speed_commands.append((now, speed))
            elif command == COMMAND_ID["rotate_absolute"]:
                self._rotate_target_deg = float(packet.get("rotate", 0.0))
        return self.status_payload()

    def _record_grab_dispatch(self, contact_point: Point) -> None:
        """Sample the two things the gate cannot check, at the grab instant."""
        pose = self.arm.position
        park_offset = math.hypot(pose[0] - contact_point[0], pose[1] - contact_point[1])
        cup_u, _ = self.frame.to_conveyor(contact_point[0], contact_point[1])
        target = self._board(self.current_plan_object_id)
        overshoot = 0.0
        if target is not None and not target.picked and not target.lost:
            overshoot = target.u_at(self.belt.position_mm) - cup_u
        self.grab_dispatches.append((park_offset, overshoot))

    def request_status(self) -> dict[str, Any] | None:
        time.sleep(self.wire_latency_s)
        with self._lock:
            return self.status_payload()

    def status_payload(self) -> dict[str, Any]:
        pose = self.arm.position
        return {
            "pos_EE": [pose[0], pose[1], pose[2]],
            "end_effector": self.arm.end_effector,
            "conveyor_position": self.belt.position_mm,
            "speed_current": self.belt.speed_mm_s,
            "rotate_current": self.rotate_current_deg,
            "task_doing": 0,
            "task_state": 1,
        }

    # the camera

    def belt_position_at(self, t: float) -> float:
        with self._lock:
            if not self._history:
                return self.belt.position_mm
            if t >= self._history[-1][0]:
                return self._history[-1][1]
            prev_t, prev_p = self._history[0]
            for sample_t, sample_p in self._history:
                if sample_t >= t:
                    span = sample_t - prev_t
                    if span <= 0:
                        return sample_p
                    frac = (t - prev_t) / span
                    return prev_p + (sample_p - prev_p) * frac
                prev_t, prev_p = sample_t, sample_p
            return self._history[-1][1]


class VirtualCamera:
    """`poll(now)` compatible with the vision pipeline the scheduler consumes.

    Detections are stamped with their *capture* time and carry the board's
    position at that instant, so the repo's backdated-anchoring path
    (`BeltPositionTracker.position_at`) is genuinely exercised: get the
    backdating wrong and the anchor lands `v * latency` off.
    """

    def __init__(self, cell: VirtualCell, settings) -> None:
        self.cell = cell
        self.settings = settings
        self._next_frame = 0.0
        self.emitted = 0

    def poll(self, now: float) -> list[ObjectDetection]:
        if now < self._next_frame:
            return []
        self._next_frame = now + self.cell.camera_period_s
        capture_t = now - self.cell.camera_latency_s
        belt_at_capture = self.cell.belt_position_at(capture_t)
        u_min, u_max, v_min, v_max = self.settings.camera_window_uv
        detections: list[ObjectDetection] = []
        with self.cell._lock:
            boards = list(self.cell.boards)
        for board in boards:
            if board.picked or board.lost:
                continue
            u = board.u_at(belt_at_capture)
            if not (u_min <= u <= u_max and v_min <= board.v <= v_max):
                continue
            noise = self.cell.camera_noise_mm
            detections.append(ObjectDetection(
                object_id=board.board_id,
                x=u + (self.cell.rng.gauss(0.0, noise) if noise else 0.0),
                y=board.v + (self.cell.rng.gauss(0.0, noise) if noise else 0.0),
                object_type=board.board_type,
                timestamp=capture_t,
                confidence=0.95,
                angle_deg=board.heading_deg,
            ))
            self.emitted += 1
        return detections

    # the production loop probes for these
    def stop(self) -> None:
        pass

    def close_window(self) -> None:
        pass


@dataclass
class Spawner:
    """Feeder: puts boards on the belt at the camera line."""

    cell: VirtualCell
    types: list[str] = field(default_factory=lambda: ["TQFP", "QFP"])
    lanes: list[float] = field(default_factory=lambda: [20.0, 60.0, 100.0])
    interval_s: float = 2.0
    mode: str = "periodic"
    _next_t: float | None = None
    _index: int = 0

    def tick(self, now: float) -> None:
        if self._next_t is None:
            self._next_t = now
        if now < self._next_t:
            return
        board_type = self.types[self._index % len(self.types)]
        lane = self.lanes[self._index % len(self.lanes)]
        heading = self.cell.rng.uniform(-90.0, 90.0)
        self.cell.spawn(board_type, lane, now, heading_deg=heading)
        self._index += 1
        if self.mode == "poisson":
            self._next_t = now + self.cell.rng.expovariate(1.0 / self.interval_s)
        else:
            self._next_t = now + self.interval_s
