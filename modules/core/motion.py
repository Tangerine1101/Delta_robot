"""Trajectory-time model: a Python port of the PLC's MC_Inter_Curve_Vel chain.

As it exists in the deployed Matching_Code_10 program (6-instance chain, cosine corner
look-ahead; doc/plc/motion-fbs.md). The Omron ignores `argument_time` and runs the
interpolator at fixed V_max / A / D, so this model — not a distance/speed estimate — is the
basis of every processing time the scheduler predicts.

The chain is forward-only: no backward pass, exactly like the deployed PLC (open-issues T5,
P7). Do not add one until the PLC is patched.
"""

from __future__ import annotations

import math

from modules.settings import Interpolator, Point


def segment_profile_time(
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

    `shape_factor` is the S-curve accel-shape compensation (1.5 for the PLC's 4th-order
    polynomial, MC §3.2.2); it only affects the stop-and-go branch.
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

    # Trapezoidal (non-zero boundary velocity). V_peak from MC Eq 28, with a triangular
    # fallback when the segment is too short to reach v_max.
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


def corner_v_end(
    seg_start: Point,
    seg_mid: Point,
    seg_next: Point,
    v_start: float,
    v_max: float,
    a_max: float,
) -> float:
    """Blend exit velocity at seg_mid (MC §3.4).

    V_corner = V_max*cos(theta/2) via the half-angle identity, clamped by the reachability
    limit V_reach = sqrt(v_start^2 + 2*A*L1) and V_max.
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


def trajectory_time(points: list[Point], interp: Interpolator) -> float:
    """Execution time (s) of a go_trajectory packet under the PLC model.

    `points` are the packet waypoints Pos[0..N-1] (e.g. the 7 goto/pick points). The PLC
    daisy-chains MC_Inter_Curve_Vel instances over the N-1 segments: segments 0..N-2 blend
    (each exits at the look-ahead corner velocity, which becomes the next segment's entry
    velocity) and the final segment decelerates to a full stop. A one-time soft start
    (State 10) is added because the chain begins from rest at Pos[0]. The arm's
    pre-trajectory position is bridged to Pos[0] by that same soft start, so it is not a
    separate timed segment.
    """
    n_seg = len(points) - 1
    if n_seg <= 0:
        return 0.0

    total = interp.soft_start_s  # State 10 soft start, once (v_start = 0)
    v_start = 0.0
    for i in range(n_seg):
        a = points[i]
        b = points[i + 1]
        length = math.dist(a, b)
        if i == n_seg - 1:
            v_end = 0.0  # final segment: stop-and-go
        else:
            v_end = corner_v_end(a, b, points[i + 2], v_start, interp.v_max, interp.a_max)
        total += segment_profile_time(length, v_start, v_end, interp.v_max, interp.a_max,
                                      interp.d_max, interp.scurve_shape_factor)
        v_start = v_end
    return total
