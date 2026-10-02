"""The pick gate: when to dispatch the pick phase, and when the arm counts as arrived.

The gate fires once the part reaches u_pick − v·lead (doc/basis-theory.md §4.4). The part's u
is anchored to the belt encoder, so during the dispatch → contact delay it advances by exactly
the belt's displacement; the steady-belt form holds because the setpoint is frozen from goto
dispatch to cup contact (open-issues L9).
"""

from __future__ import annotations

import math
from typing import Any

from modules.core.tracking import BeltTracker, TrackedObject
from modules.runtime.plan import PickPlan
from modules.runtime.state import RealtimeState
from modules.settings import Point

# The pose must be this far into a vertical final segment to count as arrived.
FINAL_SEGMENT_ENTRY_MM = 0.5
# Longest a belt sample is extrapolated forward; an older sample is used as it is.
MAX_EXTRAPOLATION_S = 0.2


def belt_lead_offset_mm(belt_speed_mm_s: float, lead_s: float) -> float:
    """Distance the gate fires early to absorb the dispatch -> contact latency."""
    return max(0.0, belt_speed_mm_s * lead_s)


def find_tracked_object(tracker: BeltTracker, object_id: str) -> TrackedObject | None:
    for obj in tracker.objects():
        if obj.object_id == object_id:
            return obj
    return None


def object_gate_status(state: RealtimeState, plan: PickPlan, lead_s: float,
                       now: float | None = None, offset_mm: float = 0.0) -> dict[str, Any] | None:
    """Where the part is relative to the gate threshold, or None when its track is gone.

    With `now`, the belt position of the latest perception sample (up to one tick old) is
    extrapolated to `now` at the sampled speed, so the gate sees where the part is rather
    than where it was. `offset_mm` (`pick_gate.gate_offset_mm`) moves the threshold upstream
    (> 0, earlier) or downstream (< 0, later) by a fixed distance."""
    with state.state_lock:
        obj = find_tracked_object(state.tracker, plan.object_id)
        if obj is None:
            return None
        p_now = state.belt_position_mm
        speed = state.belt_speed_mm_s
        sample = state.latest_speed
        if now is not None and sample is not None:
            age = now - sample.timestamp
            if 0.0 < age <= MAX_EXTRAPOLATION_S:
                p_now = sample.position_mm + sample.speed_uv * age
        u_now, _ = obj.current_uv(p_now)
        u_pick, _ = state.frame.to_conveyor(plan.predicted_pick_position_2d[0],
                                            plan.predicted_pick_position_2d[1])
    threshold = u_pick - belt_lead_offset_mm(speed, lead_s) - offset_mm
    return {"reached": u_now >= threshold, "object_u": u_now, "pick_u": u_pick,
            "threshold_u": threshold}


# ---------------------------------------------------------------------------
# Packet geometry, for the arrival checks
# ---------------------------------------------------------------------------


def packet_final_target(packet: dict[str, Any]) -> Point | None:
    return _packet_point(packet, 1)


def packet_final_segment_start(packet: dict[str, Any]) -> Point | None:
    """Waypoint Pos[n-2] where the packet's final segment begins, if any."""
    return _packet_point(packet, 2)


def _packet_point(packet: dict[str, Any], from_end: int) -> Point | None:
    n = int(packet.get("argument_number", 0))
    xs = packet.get("argument_x") or []
    ys = packet.get("argument_y") or []
    zs = packet.get("argument_z") or []
    if n >= from_end and len(xs) >= n and len(ys) >= n and len(zs) >= n:
        try:
            i = n - from_end
            return (float(xs[i]), float(ys[i]), float(zs[i]))
        except (TypeError, ValueError):
            return None
    return None


def packet_duration_s(packet: dict[str, Any]) -> float:
    """Sum of the packet's argument_time (the PLC ignores it; it only bounds the waits)."""
    argument_number = int(packet.get("argument_number", 0))
    total = 0.0
    for value in list(packet.get("argument_time", []))[:argument_number]:
        try:
            total += float(value)
        except (TypeError, ValueError):
            pass
    return max(total, 0.0)


def in_final_segment(pose: Point, segment_start: Point | None, target: Point) -> bool:
    """True once the arm is past the start of the final segment (open-issues O2).

    Only vertical final segments are checked (both 7-point templates end with a vertical
    drop); any other geometry falls back to the distance test alone."""
    if segment_start is None:
        return True
    if math.hypot(target[0] - segment_start[0], target[1] - segment_start[1]) > 0.5:
        return True
    dz = target[2] - segment_start[2]
    if abs(dz) <= FINAL_SEGMENT_ENTRY_MM:
        return True
    return (pose[2] - segment_start[2]) * math.copysign(1.0, dz) >= FINAL_SEGMENT_ENTRY_MM
