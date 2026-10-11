"""Input vs output over time, and the stretches where the output falls away from the input.

The symptom of a planner that waits for a part it should not wait for is a trailing-window
throughput line that leaves the input line while parts sit in the workspace unplanned (the
"dips" of the dashboard's *Input and throughput* chart). `find_dips` finds those stretches in
any part record with the columns `t_seen`, `t_contact`, `outcome` — the `parts.csv` of a robot
run (`simulate_feeder`) or of a sandbox run (`python3 -m sandbox run --out DIR`):

    python3 -m sandbox dips log/<run>/ [--threshold 8] [--min-len 15]
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

WINDOW_S = 60.0


@dataclass(frozen=True)
class Dip:
    t_start: float
    t_end: float
    depth_per_min: float            # largest input - output over the stretch
    victims: tuple[str, ...]        # parts that left the workspace never planned during it

    @property
    def length_s(self) -> float:
        return self.t_end - self.t_start


def read_parts(path: str | Path) -> list[dict[str, str]]:
    path = Path(path)
    if path.is_dir():
        path = path / "parts.csv"
    with open(path, newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def _times(rows: list[dict[str, str]]) -> tuple[list[float], list[float]]:
    seen = sorted(float(r["t_seen"]) for r in rows)
    out = sorted(float(r["t_contact"]) for r in rows if r["outcome"] == "picked" and r["t_contact"])
    return seen, out


def _rate(times: list[float], lo: int, t: float, window: float) -> tuple[int, float]:
    """(first index after the window start, parts per minute in (t - window, t]) — the times
    are sorted and `t` only grows, so the window start is advanced, never searched."""
    while lo < len(times) and times[lo] <= t - window:
        lo += 1
    hi = lo
    while hi < len(times) and times[hi] <= t:
        hi += 1
    return lo, 60.0 * (hi - lo) / min(window, max(t, 1e-9))


def flow_series(rows: list[dict[str, str]], step_s: float = 1.0, window_s: float = WINDOW_S):
    """[(t, input per min, output per min)] over trailing windows — the dashboard's curves."""
    seen, out = _times(rows)
    end = max([*(float(r["t_closed"]) for r in rows if r.get("t_closed")), *seen, *out, 0.0])
    series, lo_in, lo_out, t = [], 0, 0, step_s
    while t <= end:
        lo_in, rate_in = _rate(seen, lo_in, t, window_s)
        lo_out, rate_out = _rate(out, lo_out, t, window_s)
        series.append((t, rate_in, rate_out))
        t += step_s
    return series


def find_dips(rows: list[dict[str, str]], threshold_per_min: float = 8.0, min_len_s: float = 15.0,
              step_s: float = 1.0, window_s: float = WINDOW_S, warmup_s: float = WINDOW_S) -> list[Dip]:
    """Stretches of at least `min_len_s` where input - output exceeds `threshold_per_min`
    (the first `warmup_s` are skipped: the trailing window is not full yet)."""
    dips: list[Dip] = []
    start = depth = None
    for t, rate_in, rate_out in flow_series(rows, step_s, window_s) + [(float("inf"), 0.0, 0.0)]:
        gap = rate_in - rate_out
        if t >= warmup_s and gap > threshold_per_min:
            start = t if start is None else start
            depth = gap if depth is None else max(depth, gap)
            last = t
        elif start is not None:
            if last - start >= min_len_s:
                lo = start - window_s
                victims = tuple(r["part_id"] for r in rows
                                if r["outcome"] == "miss_late" and r.get("t_closed")
                                and lo <= float(r["t_closed"]) <= last)
                dips.append(Dip(start, last, depth, victims))
            start = depth = None
    return dips


def report(rows: list[dict[str, str]], **kwargs) -> str:
    dips = find_dips(rows, **kwargs)
    if not dips:
        return "no dip: the output follows the input"
    lines = [f"{len(dips)} dip(s) where the output left the input:"]
    for d in dips:
        lines.append(f"  {d.t_start:7.0f}-{d.t_end:7.0f} s  ({d.length_s:4.0f} s)  depth {d.depth_per_min:4.1f}/min"
                     f"  {len(d.victims)} part(s) left the workspace unplanned")
    return "\n".join(lines)
