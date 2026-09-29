"""Delta kinematics exactly as the deployed PLC computes them.

Port of the ``Calc_Angles_YZ`` / ``Calc_Inverse_Kinematics`` / ``Calc_Forward_Kinematic``
functions of ``Matching_Code_10`` (``doc/plc/program-old.md``), including their defects: the
IK's silent joint-limit return (open-issues O1) and the forward kinematics' unreported error
(O9). The scheduler rejects any waypoint this IK would refuse; the PLC simulator runs the same
functions, so both agree with the real controller.
"""
from __future__ import annotations

import math
import struct

# Geometry constants as declared in Calc_Angles_YZ / Calc_Forward_Kinematic.
BASE = 346.4
END_EFFECTOR = 86.6
BICEP = 140.0
FOREARM = 315.0
PI = 3.141592653589793
TAN30 = 0.5773502691896257
COS120, SIN120 = -0.5, 0.8660254037844386
COS30, SIN30 = 0.8660254037844386, 0.5
JOINT_LIMIT_DEG = -20.0


def f32(value: float) -> float:
    """Round to IEC REAL (float32), as REAL_TO_LREAL / LREAL_TO_REAL do."""
    return struct.unpack("<f", struct.pack("<f", float(value)))[0]


def calc_angles_yz(x0: float, y0: float, z0: float) -> tuple[bool, float | None]:
    if z0 == 0.0:
        return True, None
    y1 = -0.5 * TAN30 * BASE
    tmp_y0 = y0 - 0.5 * TAN30 * END_EFFECTOR
    a = (x0 * x0 + tmp_y0 * tmp_y0 + z0 * z0 + BICEP * BICEP - FOREARM * FOREARM - y1 * y1) / (2.0 * z0)
    b = (y1 - tmp_y0) / z0
    d = -(a + b * y1) * (a + b * y1) + BICEP * (b * b * BICEP + BICEP)
    if d < 0.0:
        return True, None
    yj = (y1 - a * b - math.sqrt(d)) / (b * b + 1.0)
    zj = a + b * yj
    add = 180.0 if yj > y1 else 0.0
    if (y1 - yj) == 0.0:
        theta = (90.0 if -zj >= 0.0 else -90.0) + add
    else:
        theta = 180.0 * math.atan(-zj / (y1 - yj)) / PI + add
    return False, theta


def calc_inverse_kinematics(
    x: float, y: float, z: float, unassigned: float = 0.0, limit_is_error: bool = False
) -> tuple[bool, tuple[float, float, float], bool]:
    """Returns (return_value, (θ1, θ2, θ3), joint_limit_tripped).

    Faithful to the ST: when only the −20° joint limit trips, the function RETURNs
    with its return value still FALSE (O1) and the remaining outputs unwritten.
    """
    out = [unassigned, unassigned, unassigned]
    arms = (
        (x, y, 1.0),
        (x * COS120 + y * SIN120, y * COS120 - x * SIN120, 1.0),
        (x * COS120 - y * SIN120, y * COS120 + x * SIN120, -1.0),
    )
    for i, (xa, ya, sign) in enumerate(arms):
        ret, th = calc_angles_yz(xa, ya, z)
        # The ST tests the raw arm angle for all three arms (`tmp_Theta3*-1 > 20` is
        # `tmp_Theta3 < -20`); only arm 3's output is negated.
        if ret or th < JOINT_LIMIT_DEG:
            return (ret or limit_is_error), (out[0], out[1], out[2]), True
        out[i] = th * sign
    return False, (out[0], out[1], out[2]), False


def ik_reachable(x: float, y: float, z: float) -> bool:
    """True when the PLC IK accepts the point without error or joint-limit trip."""
    ret, _, limit = calc_inverse_kinematics(x, y, z)
    return not ret and not limit


def calc_forward_kinematic(
    th1: float, th2: float, th3: float, unassigned: float = 0.0
) -> tuple[bool, tuple[float, float, float]]:
    """Returns (error, (X, Y, Z) as REAL). The PLC never reports the error (O9)."""
    t1 = th1 * PI / 180.0
    t2 = th2 * PI / 180.0
    t3 = th3 * PI / -180.0
    w = 0.5 * TAN30 * (BASE - END_EFFECTOR)
    y_j1 = -(w + BICEP * math.cos(t1))
    z_j1 = -BICEP * math.sin(t1)
    x_j2 = (w + BICEP * math.cos(t2)) * COS30
    y_j2 = (w + BICEP * math.cos(t2)) * SIN30
    z_j2 = -BICEP * math.sin(t2)
    x_j3 = -(w + BICEP * math.cos(t3)) * COS30
    y_j3 = (w + BICEP * math.cos(t3)) * SIN30
    z_j3 = -BICEP * math.sin(t3)
    r1 = y_j1 * y_j1 + z_j1 * z_j1
    r2 = x_j2 * x_j2 + y_j2 * y_j2 + z_j2 * z_j2
    r3 = x_j3 * x_j3 + y_j3 * y_j3 + z_j3 * z_j3
    a1c, b1c, c1c, d1c = x_j2, y_j2 - y_j1, z_j2 - z_j1, 0.5 * (r2 - r1)
    a2c, b2c, c2c, d2c = x_j3, y_j3 - y_j1, z_j3 - z_j1, 0.5 * (r3 - r1)
    det = a1c * b2c - a2c * b1c
    bad = (f32(unassigned),) * 3
    if det == 0.0:
        return True, bad
    a1 = (d1c * b2c - d2c * b1c) / det
    b1 = -(c1c * b2c - c2c * b1c) / det
    a2 = (a1c * d2c - a2c * d1c) / det
    b2 = -(a1c * c2c - a2c * c1c) / det
    aq = b1 * b1 + b2 * b2 + 1.0
    bq = 2.0 * (a1 * b1 + a2 * b2 - b2 * y_j1 - z_j1)
    cq = a1 * a1 + a2 * a2 - 2.0 * a2 * y_j1 + r1 - FOREARM * FOREARM
    d = bq * bq - 4.0 * aq * cq
    if d < 0.0:
        return True, bad
    z = (-bq - math.sqrt(d)) / (2.0 * aq)
    return False, (f32(a1 + b1 * z), f32(a2 + b2 * z), f32(z))
