"""Execute one committed pick on the hardware: goto -> pick gate -> pick -> place -> rotate.

The executor never chooses a part or a belt speed. It freezes the belt setpoint from goto
dispatch until cup contact (`RealtimeState.pick_committed`, open-issues L9) and reports the
decision points after contact to the speed controller through `on_speed_event`. When the
decision loop hands it the run's `PartLedger`, it books the dispatch and the contact (with the
part-to-cup error at contact) there; a pre-grip abort leaves its reason in
`plan.debug_info["abort_reason"]`.
"""
from __future__ import annotations

import json
import math
import threading
import time
from collections import deque
from typing import Any, Callable

from modules.comm.packets import COMMAND_ID
from modules.core.delta import GATE_POLL_S, DeltaArm
from modules.runtime.pick_gate import (
    find_tracked_object,
    in_final_segment,
    object_gate_status,
    packet_duration_s,
    packet_final_segment_start,
    packet_final_target,
)
from modules.runtime.outcomes import PartLedger
from modules.runtime.plan import PickPlan, post_grip_rotation_rad
from modules.runtime.state import RealtimeState
from modules.settings import Settings

# Safety margin on top of the modelled dispatch -> contact -> lift-past-pre_pick time before
# the post-grip rotate is force-dispatched even if the pose poll never sampled a point inside
# the (narrow, fast-transited) descent/lift window.
ROTATE_FALLBACK_MARGIN_S = 0.3

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
        dispatch: Callable[[dict[str, Any]], dict[str, Any] | None],
        request_status: Callable[[], dict[str, Any] | None],
        settings: Settings,
        arm: DeltaArm,
        *,
        ipc_lock: threading.Lock | None = None,
    ) -> None:
        gate = settings.pick_gate
        self._dispatch_fn = dispatch
        self._request_status_fn = request_status
        self.arm = arm
        self.interpolar_points = settings.plc.interpolar_points
        self.wait_margin_s = gate.arrival_timeout_margin_s
        self.status_poll_interval_s = max(settings.runtime.poll_interval_s, 0.02)
        # Arrival tolerance: linear in belt speed from the floor (at speed.band.min_mm_s) to
        # the ceiling (at speed.band.max_mm_s). A missing ceiling or a degenerate speed range
        # collapses to the floor.
        self.position_tolerance_mm = max(gate.arrival_tolerance_mm, 0.0)
        ceiling = (self.position_tolerance_mm if gate.arrival_tolerance_max_mm is None
                   else max(gate.arrival_tolerance_max_mm, 0.0))
        self.position_tolerance_max_mm = max(ceiling, self.position_tolerance_mm)
        self.tolerance_speed_min_mm_s = max(settings.speed.band.min_mm_s, 0.0)
        self.tolerance_speed_max_mm_s = max(settings.speed.band.max_mm_s, 0.0)
        self.rotation = settings.robot.rotation
        # 0 disables the pick-time "cup back at 0 yet?" warning (see _execute()).
        self.rotate_home_tolerance_deg = max(self.rotation.home_tolerance_deg, 0.0)
        # Reject a gate-time heading refresh that swings more than this from the plan-build
        # heading (likely a vision glitch, not a refinement). 0 disables the refresh.
        self.rotate_refresh_max_delta_deg = max(self.rotation.refresh_max_delta_deg, 0.0)
        self.ipc_lock = ipc_lock or threading.Lock()
        self.round_trip = _RoundTripTracker()
        # Decision points after cup contact, for the speed controller: "contact" once, then
        # "busy" while the part is carried to the bin. Set by the decision loop.
        self.on_speed_event: Callable[[str], None] | None = None
        # The run's part record (runtime/outcomes.py); set by the decision loop.
        self.ledger: PartLedger | None = None

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

    def execute(self, plan: PickPlan, state: RealtimeState) -> bool:
        # The belt is frozen from goto dispatch until cup contact: the plan, the
        # gate lead and the grip all assume a steady belt (T6/T7, L9).
        with state.state_lock:
            state.pick_committed = True
        try:
            return self._execute(plan, state)
        finally:
            with state.state_lock:
                state.pick_committed = False

    def _execute(self, plan: PickPlan, state: RealtimeState) -> bool:
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
            plan.debug_info["abort_reason"] = "goto_failed"
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
                tracked_obj = find_tracked_object(state.tracker, plan.object_id)
                rotation_rad_now = tracked_obj.rotation_rad if tracked_obj is not None else None
            plan_heading_deg = plan.debug_info.get("board_heading_deg")
            if rotation_rad_now is not None and plan_heading_deg is not None:
                heading_now_deg = math.degrees(rotation_rad_now)
                delta_deg = abs(((heading_now_deg - plan_heading_deg + 180.0) % 360.0) - 180.0)
                if delta_deg <= self.rotate_refresh_max_delta_deg:
                    plan.rotate_rad = post_grip_rotation_rad(self.rotation, rotation_rad_now)
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
        if self.ledger is not None:
            self.ledger.dispatched(plan.object_id, dispatched_at)
        with state.state_lock:
            belt_speed_at_grip = state.belt_speed_mm_s
        if status is not None:
            print("[PLC]", json.dumps(status, ensure_ascii=True))
        # Speed commits resume inside the pick-phase wait once contact is seen
        # (pick_committed cleared there), never during the dead-time chain.
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
        target = packet_final_target(packet)
        if target is None:
            return True
        expected_duration_s = packet_duration_s(packet)
        started_at = time.monotonic()
        deadline = started_at + expected_duration_s + self.wait_margin_s
        departed = False
        static_accept_allowed: bool | None = None
        final_segment_start = packet_final_segment_start(packet)
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
        contact_reported = contact_z is None
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
                rotate_fallback_deadline = dispatched_at + modeled_s + ROTATE_FALLBACK_MARGIN_S

        while True:
            now = time.monotonic()
            # Decision point for the speed controller: ignored while the pick is committed
            # (goto flight, gate, descent); "contact" once contact has cleared
            # pick_committed, then "busy" while the part is carried to the bin.
            if self.on_speed_event is not None:
                self.on_speed_event("busy" if contact_reported else "contact")
                with state.state_lock:
                    contact_reported = contact_reported or not state.pick_committed
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
                error_uv = self._contact_error_uv(plan, state, pose)
                if self.ledger is not None:
                    self.ledger.contact(plan.object_id, now, error_uv, contact_observed,
                                        live_belt_speed or 0.0)
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
                            # Part centre − cup at this pose sample, belt frame (u, v).
                            "contact_error_uv_mm": (
                                None if error_uv is None
                                else [round(error_uv[0], 2), round(error_uv[1], 2)]
                            ),
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
            if contact_z is not None and contact_logged:
                with state.state_lock:
                    state.pick_committed = False
            if pose is not None:
                distance = math.dist(pose, target)
                if static_accept_allowed is None:
                    static_accept_allowed = (
                        distance <= tolerance_mm and expected_duration_s <= 0.25
                    )
                if distance > tolerance_mm or not in_final_segment(
                    pose, final_segment_start, target
                ):
                    # O2: the deployed PLC accepts a command 3 on top of a running
                    # chain and a still-running instance k<=4 then hands over on
                    # the NEW geometry. Arrival therefore also requires the arm to
                    # be inside the final (vertical) segment, not merely near it.
                    departed = departed or distance > tolerance_mm
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

    @staticmethod
    def _contact_error_uv(plan: PickPlan, state: RealtimeState,
                          pose: tuple[float, float, float] | None) -> tuple[float, float] | None:
        """Tracked part centre minus cup position (belt frame, mm) at a pose sample."""
        if pose is None:
            return None
        with state.state_lock:
            obj = find_tracked_object(state.tracker, plan.object_id)
            if obj is None:
                return None
            u, v = obj.current_uv(state.belt_position_mm)
        cup_u, cup_v = state.frame.to_conveyor(pose[0], pose[1])
        return u - cup_u, v - cup_v

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
        late_abort_mm = self.arm.gate.late_abort_mm
        while True:
            now = time.monotonic()
            gate = object_gate_status(state, plan, self.arm.gate_lead_s, now,
                                      self.arm.gate.gate_offset_mm)
            if gate is None:
                print(
                    "[WARN]",
                    json.dumps(
                        {"plan_id": plan.plan_id, "event": "pick_object_missing"},
                        ensure_ascii=True,
                    ),
                )
                plan.debug_info["abort_reason"] = "object_missing"
                return False
            if gate["reached"]:
                late_mm = gate["object_u"] - gate["threshold_u"]
                if late_abort_mm > 0.0 and late_mm > late_abort_mm:
                    # Two-sided gate (T4): the arm parked after the object had
                    # already crossed the threshold, so firing now would land
                    # the cup behind the board by late_mm. The object is still
                    # un-dispatched, so the main loop re-queues it.
                    print(
                        "[WARN]",
                        json.dumps(
                            {
                                "plan_id": plan.plan_id,
                                "event": "pick_gate_late",
                                "late_mm": round(late_mm, 2),
                                "late_abort_mm": late_abort_mm,
                            },
                            ensure_ascii=True,
                        ),
                    )
                    plan.debug_info["abort_reason"] = "gate_late"
                    return False
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
                plan.debug_info["abort_reason"] = "object_stalled"
                return False
            time.sleep(GATE_POLL_S)
