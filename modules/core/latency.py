"""Unmodelled camera latency from belt speed steps.

While the camera sees a part, its tracked position is the camera position plus the belt travel
since the frame's capture stamp, so the offset `u - p` (tracked u minus belt position) is
continuous across a change of belt speed only if the stamp is right. A stamp late by Δ makes
the offset jump by -Δ·(v_after - v_before) at the change. Fitting a line to the offset on each
side of a start or stop, both extrapolated to the change, gives Δ.

A camera scale error makes the offset drift while the belt moves (the camera sees the part
travel s·Δp instead of Δp); the line fits absorb it, so Δ is measured independently of it.
"""

from __future__ import annotations

import statistics
from typing import Hashable, Iterable

# Belt sample: (t, position_mm, speed_mm_s). Track sample: (t, u - p) in mm.
BeltSample = tuple[float, float, float]
OffsetSample = tuple[float, float]

# A start / stop changes the settled speed by at least this (mm/s).
MIN_SPEED_STEP_MM_S = 10.0
# Fit windows around a change (s): the camera needs its latency to catch up after it.
BEFORE_S = (-1.0, -0.15)
AFTER_S = (0.6, 1.4)
# Fewest offset samples per side for a fit.
MIN_SAMPLES = 3


def speed_changes(belt: list[BeltSample], min_step_mm_s: float = MIN_SPEED_STEP_MM_S
                  ) -> list[tuple[float, float, float]]:
    """(t, v_before, v_after) of every belt start and stop: the speed crosses 0.5 mm/s and
    the settled speeds on either side differ by at least `min_step_mm_s`."""
    out = []
    for i in range(1, len(belt)):
        before, after = belt[i - 1][2], belt[i][2]
        if (before < 0.5 <= after) or (after < 0.5 <= before):
            t = belt[i][0]
            v0 = _median_speed(belt, t + BEFORE_S[0], t + BEFORE_S[1])
            v1 = _median_speed(belt, t + AFTER_S[0], t + AFTER_S[1])
            if v0 is not None and v1 is not None and abs(v1 - v0) >= min_step_mm_s:
                out.append((t, v0, v1))
    return out


def change_lags(belt: list[BeltSample], tracks: dict[Hashable, list[OffsetSample]]
                ) -> list[dict[str, object]]:
    """One latency estimate per (speed change, part seen on both sides of it)."""
    out: list[dict[str, object]] = []
    for t_change, v0, v1 in speed_changes(belt):
        for key, samples in tracks.items():
            before = [s for s in samples if t_change + BEFORE_S[0] <= s[0] <= t_change + BEFORE_S[1]]
            after = [s for s in samples if t_change + AFTER_S[0] <= s[0] <= t_change + AFTER_S[1]]
            k0, k1 = line_at(before, t_change), line_at(after, t_change)
            if k0 is None or k1 is None:
                continue
            jump = k1 - k0
            out.append({"t": t_change, "part": key, "v_before": v0, "v_after": v1,
                        "jump_mm": jump, "lag_s": -jump / (v1 - v0)})
    return out


def median_lag_s(lags: Iterable[dict[str, object]]) -> float | None:
    values = [float(x["lag_s"]) for x in lags]  # type: ignore[arg-type]
    return statistics.median(values) if values else None


def line_at(samples: list[OffsetSample], t: float) -> float | None:
    """Least-squares line through (time, value) samples, evaluated at t."""
    if len(samples) < MIN_SAMPLES:
        return None
    mt = statistics.fmean(s[0] for s in samples)
    mv = statistics.fmean(s[1] for s in samples)
    sxx = sum((s[0] - mt) ** 2 for s in samples)
    slope = sum((s[0] - mt) * (s[1] - mv) for s in samples) / sxx if sxx > 0 else 0.0
    return mv + slope * (t - mt)


def _median_speed(belt: list[BeltSample], t0: float, t1: float) -> float | None:
    values = [s[2] for s in belt if t0 <= s[0] <= t1]
    return statistics.median(values) if values else None
