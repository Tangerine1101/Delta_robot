"""How a speed law's target becomes a belt setpoint — the same rule for every law.

Pure logic; the realtime controller (modules/runtime/speed.py) and the sandbox belt call it and
do the sending themselves. The freeze from goto dispatch to cup contact (open-issues L9) is
checked by the caller before a law even runs, so a law cannot break it.
"""

from __future__ import annotations

from dataclasses import dataclass

# Re-send the setpoint when the measured speed still diverges this long after the last
# commit (any commanded ramp has settled by then).
RESYNC_AFTER_S = 3.0


@dataclass(frozen=True)
class CommitPolicy:
    band: tuple[float, float]       # (v_min, v_max) every setpoint is clamped to
    deadband_mm_s: float            # a smaller change is not sent
    max_step_mm_s: float            # largest change per commit; 0 = none


@dataclass(frozen=True)
class CommitStep:
    send: float | None              # setpoint to send now, or None
    resync: bool = False            # True when `send` re-sends an unchanged setpoint


def commit_step(
    target_mm_s: float,
    setpoint_mm_s: float,
    measured_mm_s: float | None,
    since_last_commit_s: float,
    policy: CommitPolicy,
    use_deadband: bool = True,
) -> CommitStep:
    """Walk the setpoint toward `target_mm_s`: clamp to the band, limit the step, hold inside
    the deadband. When the setpoint already sits at the target but the measured belt still
    diverges well past the deadband long after the last commit, the PLC missed or clamped the
    command: re-send the setpoint (closed loop)."""
    if target_mm_s <= 0.0:
        return CommitStep(None)
    v_min, v_max = policy.band
    target = min(max(target_mm_s, v_min), max(v_min, v_max))
    deadband = policy.deadband_mm_s if use_deadband else 0.0
    step_target = target
    if policy.max_step_mm_s > 0.0:
        delta = max(-policy.max_step_mm_s, min(policy.max_step_mm_s, target - setpoint_mm_s))
        step_target = setpoint_mm_s + delta
    if abs(step_target - setpoint_mm_s) > deadband:
        return CommitStep(step_target)
    if (measured_mm_s is None or abs(setpoint_mm_s - measured_mm_s) <= 2.0 * deadband
            or since_last_commit_s < RESYNC_AFTER_S):
        return CommitStep(None)
    return CommitStep(setpoint_mm_s, resync=True)
