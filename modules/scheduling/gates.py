"""Setpoint gates: *when* the belt setpoint may move.

A gate is asked at every decision point of the cell with the event that created it:

* `arm_free` — the arm has just become free (after a pick, or at start-up);
* `idle`     — the arm is still free (every loop pass until the next pick is committed);
* `contact`  — the cup has just touched the part;
* `busy`     — the arm is returning a picked part to its bin.

Whatever a gate answers, the controller never moves the setpoint between goto dispatch and
cup contact (open-issues L9): no event is raised in that interval.
"""

from __future__ import annotations

from modules.scheduling.registry import setpoint_gate


@setpoint_gate("never")
def never(event: str, since_last_s: float, period_s: float) -> bool:
    """The setpoint never moves (the `constant` law)."""
    return False


@setpoint_gate("arm_free")
def arm_free(event: str, since_last_s: float, period_s: float) -> bool:
    """When the arm becomes free, then every control_period_s while it stays free."""
    return event == "arm_free" or (event == "idle" and since_last_s >= period_s)


@setpoint_gate("at_contact")
def at_contact(event: str, since_last_s: float, period_s: float) -> bool:
    """As arm_free, plus the instant the cup touches the part."""
    return event == "contact" or arm_free(event, since_last_s, period_s)


@setpoint_gate("periodic")
def periodic(event: str, since_last_s: float, period_s: float) -> bool:
    """Every control_period_s, whether the arm is free or returning a part."""
    return since_last_s >= period_s


@setpoint_gate("always")
def always(event: str, since_last_s: float, period_s: float) -> bool:
    """At every decision point."""
    return True
