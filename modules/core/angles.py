"""Suction rotation angle convention.

Three layers, one unit each, converted only at the boundaries:

1. vision measures a marker heading on image pixels (degrees) and converts it ONCE into an
   R-frame heading in RADIANS (`ConveyorFrame.vision_heading_to_robot_rad`) — 0 = robot +X
   axis, positive = CCW seen from above, wrapped to [-pi, pi).
2. the scheduler computes everything in R-frame radians; the wrap to [-pi, pi) is by itself
   the shortest-way rotation (<= 180 deg). The "rotate" field of a rotate_absolute (7) packet
   carries this radian value.
3. the Siemens rotation axis accepts signed degrees in [-360, 360] and uses the SAME zero as
   the R-frame (0 deg = 0 rad). The PLC command encodes both the axis position AND the
   direction of travel in the numeric value, so the IPC boundary (the PLC worker) must be
   VERBATIM: plain radians->degrees with NO wrap, only clamped to [-359, 359] (the PLC's ST
   misbehaves at exactly +/-360 — it can flip direction). Wrapping there to [-180, 180) would
   turn 180 -> -180 and 270 -> -90 on the wire; because the sign selects the spin direction,
   a 179->180 step would then drive the axis the long way round. The minimal-turn decision
   belongs upstream (the plan wraps rotate_rad once, relative to the homed 0). Convert ONLY
   for rotate_absolute (command 7); change_speed (8) also carries a rotate field but the PLC
   ignores it, and touching it would spin the cup.
"""

from __future__ import annotations

import math

# Siemens rotation-axis limit (deg). Kept off the exact +/-360 edge on purpose.
ROTATE_WIRE_LIMIT_DEG = 359.0


def wrap_angle_180(angle_deg: float) -> float:
    """Wrap an angle (degrees) into the half-open interval [-180, 180)."""
    return (float(angle_deg) + 180.0) % 360.0 - 180.0


def wrap_rad(angle_rad: float) -> float:
    """Wrap an angle (radians) into the half-open interval [-pi, pi)."""
    return (float(angle_rad) + math.pi) % (2.0 * math.pi) - math.pi


def robot_rad_to_wire_deg(angle_rad: float) -> float:
    """R-frame suction angle (radians) -> Siemens wire degrees, VERBATIM.

    Identity zero (0 rad = 0 deg), NO wrap: the caller's exact angle is sent so the PLC —
    whose command value encodes spin direction as well as position — turns the intended way.
    Only clamped to [-359, 359] to stay off the +/-360 edge.
    """
    deg = math.degrees(angle_rad)
    return max(-ROTATE_WIRE_LIMIT_DEG, min(ROTATE_WIRE_LIMIT_DEG, deg))


def wire_deg_to_robot_rad(wire_deg: float) -> float:
    """Siemens wire degrees feedback -> R-frame radians, VERBATIM (identity zero).

    No wrap, so status feedback shows the PLC's true angle (270 stays 270, not pulled back
    to -90).
    """
    return math.radians(float(wire_deg))
