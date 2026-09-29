"""What happened to every part: the run's input, throughput and miss record.

One `PartRecord` per part, opened at its first camera sighting and closed exactly once with an
outcome and a reason:

| outcome      | meaning                                       | reasons |
|--------------|-----------------------------------------------|---------|
| `picked`     | the cup came down on the part                 | `gripped`; `unverified` (contact not seen by the pose poll) |
| `miss_grip`  | a pick was dispatched but the part was not taken | `cup_off_part` (contact error > `runtime.grip_tolerance_mm`); `pick_motion_failed` (no contact) |
| `miss_late`  | the part left the workspace never dispatched  | `never_planned`; the last pre-grip abort: `goto_failed`, `gate_late`, `object_stalled`, `object_missing` |
| `lost_track` | the track died inside the camera view         | `stale_in_camera` |
| `on_belt`    | still on the belt when the run ended          | `run_ended` |

"Gripped" is judged from geometry, not from suction (the cell has no vacuum sensor, open-issues
L6): at the first pose sample inside the contact band, the part's tracked centre is compared
with the cup. For a virtual part (`simulate_feeder`) the tracked centre is the part's true
position, so the judgement is exact up to the pose and belt sampling; for a real part it also
carries the tracking error (open-issues L17).

Input is counted at the first sighting, throughput at contact. `write()` produces the run's
`parts.csv`, `flow.csv` and `summary.json` (doc/basis-programming.md §2.5).
"""

from __future__ import annotations

import csv
import json
import math
import statistics
import threading
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Callable

PICKED, MISS_GRIP, MISS_LATE, LOST_TRACK, ON_BELT = "picked", "miss_grip", "miss_late", "lost_track", "on_belt"
OUTCOMES = (PICKED, MISS_GRIP, MISS_LATE, LOST_TRACK, ON_BELT)
RATE_WINDOW_S = 60.0


@dataclass
class PartRecord:
    part_id: str
    part_type: str
    source: str                     # camera | virtual
    t_seen: float                   # run time (s) of the first sighting
    u_seen: float
    v_seen: float
    plans: int = 0
    t_first_plan: float | None = None
    last_abort: str | None = None
    t_dispatch: float | None = None
    t_contact: float | None = None
    contact_observed: bool | None = None
    error_u_mm: float | None = None  # part − cup along the belt at contact
    error_v_mm: float | None = None
    belt_mm_s_at_contact: float | None = None
    outcome: str | None = None
    reason: str | None = None
    t_closed: float | None = None

    @property
    def error_mm(self) -> float | None:
        if self.error_u_mm is None or self.error_v_mm is None:
            return None
        return math.hypot(self.error_u_mm, self.error_v_mm)

    @property
    def held(self) -> bool:
        """On the cup: contact seen, outcome not yet booked or booked as picked."""
        return self.t_contact is not None and self.outcome in (None, PICKED)


class PartLedger:
    """Thread-safe: perception opens and prunes records, the decision loop and the pick
    executor book plans, aborts, contacts and pick ends. Times are monotonic; records store
    them relative to `start_time`."""

    def __init__(self, start_time: float, *, grip_tolerance_mm: float, source: str = "camera",
                 on_close: Callable[[dict[str, Any]], None] | None = None) -> None:
        self.start_time = start_time
        self.grip_tolerance_mm = grip_tolerance_mm
        self.source = source
        self.on_close = on_close
        self._lock = threading.Lock()
        self._records: dict[str, PartRecord] = {}
        self._belt: list[tuple[float, float, int]] = []    # (run time, belt mm/s, density)
        self.t_end: float | None = None

    def _t(self, now: float) -> float:
        return round(now - self.start_time, 4)

    # ---- bookings ---------------------------------------------------------------------
    def seen(self, part_id: str, part_type: str, now: float, u: float, v: float) -> None:
        """Any sighting; opens the record on the first. A track that went stale in camera
        view and is seen again is reopened (the part never left)."""
        with self._lock:
            record = self._records.get(part_id)
            if record is None:
                self._records[part_id] = PartRecord(part_id, part_type, self.source, self._t(now),
                                                    round(u, 2), round(v, 2))
            elif record.outcome == LOST_TRACK:
                record.outcome = record.reason = record.t_closed = None

    def planned(self, part_id: str, now: float) -> None:
        with self._lock:
            record = self._records.get(part_id)
            if record is not None and record.outcome is None:
                record.plans += 1
                if record.t_first_plan is None:
                    record.t_first_plan = self._t(now)

    def aborted(self, part_id: str, reason: str) -> None:
        """A pre-grip abort: the part stays on the belt and may be planned again."""
        with self._lock:
            record = self._records.get(part_id)
            if record is not None and record.outcome is None:
                record.last_abort = reason

    def dispatched(self, part_id: str, now: float) -> None:
        with self._lock:
            record = self._records.get(part_id)
            if record is not None and record.outcome is None:
                record.t_dispatch = self._t(now)

    def contact(self, part_id: str, now: float, error_uv: tuple[float, float] | None,
                observed: bool, belt_mm_s: float) -> None:
        """The cup reached the part's height. A known error decides the grip now, so a
        picked part counts toward throughput at contact, not when the arm is back."""
        with self._lock:
            record = self._records.get(part_id)
            if record is None or record.outcome is not None:
                return
            record.t_contact = self._t(now)
            record.contact_observed = observed
            record.belt_mm_s_at_contact = round(belt_mm_s, 2)
            if error_uv is not None:
                record.error_u_mm, record.error_v_mm = round(error_uv[0], 2), round(error_uv[1], 2)
            if observed and error_uv is not None:
                gripped = record.error_mm <= self.grip_tolerance_mm
                closed = self._close(record, now, PICKED if gripped else MISS_GRIP,
                                     "gripped" if gripped else "cup_off_part")
            else:
                closed = None
        self._notify(closed)

    def pick_done(self, part_id: str, now: float, completed: bool) -> None:
        """The pick trajectory ended (completed or not). Closes a part the contact did not."""
        with self._lock:
            record = self._records.get(part_id)
            if record is None or record.outcome is not None:
                return
            if record.t_contact is not None:
                closed = self._close(record, now, PICKED, "unverified")
            else:
                closed = self._close(record, now, MISS_GRIP, "pick_motion_failed")
        self._notify(closed)

    def left(self, part_id: str, now: float, *, past_workspace: bool) -> None:
        """The tracker dropped the part: past the workspace edge, or stale in camera view."""
        with self._lock:
            record = self._records.get(part_id)
            if record is None or record.outcome is not None or record.t_dispatch is not None:
                return
            if past_workspace:
                closed = self._close(record, now, MISS_LATE, record.last_abort or "never_planned")
            else:
                closed = self._close(record, now, LOST_TRACK, "stale_in_camera")
        self._notify(closed)

    def belt_sample(self, now: float, speed_mm_s: float, density: int) -> None:
        with self._lock:
            self._belt.append((self._t(now), round(speed_mm_s, 2), density))

    def finish(self, now: float) -> None:
        """End of run: every part still open is booked `on_belt`."""
        with self._lock:
            self.t_end = self._t(now)
            open_records = [r for r in self._records.values() if r.outcome is None]
            closed = [self._close(record, now, ON_BELT, "run_ended") for record in open_records]
        for payload in closed:
            self._notify(payload)

    def _close(self, record: PartRecord, now: float, outcome: str, reason: str) -> dict[str, Any]:
        record.outcome, record.reason, record.t_closed = outcome, reason, self._t(now)
        error = record.error_mm
        return {"id": record.part_id, "type": record.part_type, "outcome": outcome, "reason": reason,
                "t": record.t_closed, "error_mm": None if error is None else round(error, 2)}

    def _notify(self, closed: dict[str, Any] | None) -> None:
        if closed is not None and self.on_close is not None:
            self.on_close(closed)

    # ---- views ------------------------------------------------------------------------
    def is_held(self, part_id: str) -> bool:
        with self._lock:
            record = self._records.get(part_id)
            return record is not None and record.held

    def records(self) -> list[PartRecord]:
        with self._lock:
            return sorted(self._records.values(), key=lambda r: r.t_seen)

    def counts(self) -> dict[str, int]:
        with self._lock:
            out = {"input": len(self._records), **{name: 0 for name in OUTCOMES}, "open": 0}
            for record in self._records.values():
                out[record.outcome or "open"] += 1
            return out

    def flow(self, now: float) -> dict[str, Any]:
        """Cumulative counts and rates over the last RATE_WINDOW_S, for the dashboard."""
        t = self._t(now)
        window = min(RATE_WINDOW_S, max(t, 1e-9))
        counts = self.counts()
        with self._lock:
            inputs = sum(1 for r in self._records.values() if r.t_seen >= t - window)
            picks = sum(1 for r in self._records.values()
                        if r.outcome == PICKED and _pick_time(r) >= t - window)
        return {"t": round(t, 2), **counts,
                "input_per_min": round(60.0 * inputs / window, 2),
                "throughput_per_min": round(60.0 * picks / window, 2),
                "rate_window_s": round(window, 1)}

    def summary(self) -> dict[str, Any]:
        with self._lock:
            belt = list(self._belt)
        return summarize(self.records(), belt, self.t_end or 0.0, self.grip_tolerance_mm)

    # ---- files ------------------------------------------------------------------------
    def write(self, directory: Path, meta: dict[str, Any], bin_s: float) -> dict[str, Any]:
        """Write parts.csv, flow.csv and summary.json; returns the summary."""
        directory.mkdir(parents=True, exist_ok=True)
        records = self.records()
        with self._lock:
            belt = list(self._belt)
        duration = self.t_end if self.t_end is not None else max((r.t_closed or 0.0 for r in records), default=0.0)

        with open(directory / "parts.csv", "w", newline="", encoding="utf-8") as handle:
            fields = [*PartRecord.__dataclass_fields__, "error_mm"]
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            for record in records:
                row = asdict(record)
                error = record.error_mm
                row["error_mm"] = None if error is None else round(error, 2)
                writer.writerow(row)

        bins = flow_bins(records, belt, duration, bin_s)
        with open(directory / "flow.csv", "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(FLOW_FIELDS))
            writer.writeheader()
            writer.writerows(bins)

        summary = summarize(records, belt, duration, self.grip_tolerance_mm)
        summary = {**meta, **summary}
        (directory / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False))
        return summary


FLOW_FIELDS = ("t_start", "t_end", "input", PICKED, MISS_GRIP, MISS_LATE, LOST_TRACK,
               "input_per_min", "throughput_per_min", "mean_belt_mm_s", "mean_density")


def _pick_time(record: PartRecord) -> float:
    return record.t_contact if record.t_contact is not None else (record.t_closed or 0.0)


def flow_bins(records: list[PartRecord], belt: list[tuple[float, float, int]], duration: float,
              bin_s: float) -> list[dict[str, Any]]:
    """Per time bin: parts seen, outcomes booked (picks at contact, misses when closed),
    and the belt's mean speed and density."""
    bin_s = max(bin_s, 1e-3)
    n = max(1, math.ceil(duration / bin_s)) if duration > 0 else 0
    rows = []
    for i in range(n):
        lo, hi = i * bin_s, min((i + 1) * bin_s, duration)
        span = hi - lo
        if span <= 0:
            continue
        row: dict[str, Any] = {"t_start": round(lo, 2), "t_end": round(hi, 2)}
        row["input"] = sum(1 for r in records if lo <= r.t_seen < hi)
        for outcome in (PICKED, MISS_GRIP, MISS_LATE, LOST_TRACK):
            row[outcome] = sum(1 for r in records if r.outcome == outcome
                               and lo <= (_pick_time(r) if outcome == PICKED else (r.t_closed or 0.0)) < hi)
        row["input_per_min"] = round(60.0 * row["input"] / span, 2)
        row["throughput_per_min"] = round(60.0 * row[PICKED] / span, 2)
        samples = [(s, d) for t, s, d in belt if lo <= t < hi]
        row["mean_belt_mm_s"] = round(statistics.fmean(s for s, _ in samples), 2) if samples else None
        row["mean_density"] = round(statistics.fmean(d for _, d in samples), 2) if samples else None
        rows.append(row)
    return rows


def summarize(records: list[PartRecord], belt: list[tuple[float, float, int]], duration: float,
              grip_tolerance_mm: float) -> dict[str, Any]:
    counts = {name: sum(1 for r in records if r.outcome == name) for name in OUTCOMES}
    reasons: dict[str, int] = {}
    for record in records:
        if record.outcome in (MISS_GRIP, MISS_LATE, LOST_TRACK):
            key = f"{record.outcome}/{record.reason}"
            reasons[key] = reasons.get(key, 0) + 1
    finished = counts[PICKED] + counts[MISS_GRIP] + counts[MISS_LATE]
    errors = [r.error_mm for r in records if r.error_mm is not None and r.contact_observed]
    minutes = duration / 60.0 if duration > 0 else None
    return {
        "duration_s": round(duration, 2),
        "input": len(records),
        **counts,
        "miss_reasons": dict(sorted(reasons.items())),
        "pick_rate": round(counts[PICKED] / finished, 4) if finished else None,
        "input_per_min": round(len(records) / minutes, 2) if minutes else None,
        "throughput_per_min": round(counts[PICKED] / minutes, 2) if minutes else None,
        "mean_belt_mm_s": round(statistics.fmean(s for _, s, _ in belt), 2) if belt else None,
        "median_error_mm": round(statistics.median(errors), 2) if errors else None,
        "grip_tolerance_mm": grip_tolerance_mm,
    }
