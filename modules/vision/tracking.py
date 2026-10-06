"""Centroid tracking of boards across frames, and the informational belt-speed estimate."""
from __future__ import annotations

import math
import statistics
from dataclasses import dataclass


@dataclass
class Track:
    """A tracked centroid. Carries trigger state and per-frame velocity."""
    id: int
    cx: float
    cy: float
    t: float
    missing: int = 0
    prev_side: int = 0           # set by TriggerLine: -1 upstream, +1 downstream
    triggered: bool = False
    frames: int = 1              # consecutive frames matched
    vx: float = 0.0              # px/s, last instantaneous velocity
    vy: float = 0.0


class CentroidTracker:
    """Lightweight nearest-neighbour centroid tracker.

    The belt is slow, single-lane, unidirectional and non-occluding, so heavy
    trackers (ByteTrack/BoT-SORT) are unnecessary — we just need a stable id per
    board so the trigger fires exactly once, plus a per-track velocity for the
    belt-speed estimator.
    """

    def __init__(self, max_match_dist: float = 80.0, max_missing: int = 15) -> None:
        self.max_match_dist = float(max_match_dist)
        self.max_missing = int(max_missing)
        self._next_id = 1
        self.tracks: dict[int, Track] = {}

    def update(self, detections: list[tuple[float, float]], now: float) -> dict[int, Track]:
        """Match (cx, cy) centroids to nearest unclaimed track within
        max_match_dist, else start a new track. Returns id -> Track for tracks
        seen this frame, and updates per-track instantaneous velocity (px/s).

        The match distance is measured against each track's *velocity-predicted*
        position `(cx + vx*dt, cy + vy*dt)`, not its last position. On a fast
        belt at low inference fps a board can jump far between processed frames;
        matching on the last position would exceed `max_match_dist` and spawn a
        new id every frame. Predicting forward keeps the same id while moving."""
        unmatched_ids = set(self.tracks.keys())
        matched_ids: set[int] = set()

        for cx, cy in detections:
            best_id, best_d = None, self.max_match_dist
            for tid in unmatched_ids:
                t = self.tracks[tid]
                dt = max(0.0, now - t.t)
                pred_cx = t.cx + t.vx * dt
                pred_cy = t.cy + t.vy * dt
                d = math.hypot(pred_cx - cx, pred_cy - cy)
                if d < best_d:
                    best_id, best_d = tid, d
            if best_id is None:
                tid = self._next_id
                self.tracks[tid] = Track(id=tid, cx=cx, cy=cy, t=now)
                matched_ids.add(tid)
                self._next_id += 1
            else:
                t = self.tracks[best_id]
                dt = now - t.t
                if dt > 0.0:
                    t.vx = (cx - t.cx) / dt
                    t.vy = (cy - t.cy) / dt
                t.cx, t.cy, t.t, t.missing = cx, cy, now, 0
                t.frames += 1
                unmatched_ids.discard(best_id)
                matched_ids.add(best_id)

        # Age / retire tracks not matched this frame.
        for tid in list(unmatched_ids):
            t = self.tracks[tid]
            t.missing += 1
            if t.missing > self.max_missing:
                del self.tracks[tid]

        return {tid: self.tracks[tid] for tid in matched_ids}


class BeltVelocityEstimator:
    """Estimate belt speed (mm/s) from tracked-object displacement.

    For every track seen for at least `min_track_frames`, the centroid velocity
    (px/s) is known. We take the median of the belt-axis component across tracks
    (robust to a single mis-tracked box) and EMA-smooth it, then convert px/s →
    mm/s with `pixels_per_mm`.

    NOTE: this is NOT used for operation. The scheduler's belt position/velocity
    still comes from the Siemens `conveyor_position` field. This estimate is for
    cross-checking / future calibration only.
    """

    def __init__(self, pixels_per_mm: float, axis: str = "y",
                 ema_alpha: float = 0.3, min_track_frames: int = 3) -> None:
        self.pixels_per_mm = float(pixels_per_mm)
        self.axis = axis  # 'y' = vertical belt motion, 'x' = horizontal, 'mag' = magnitude
        self.ema_alpha = float(ema_alpha)
        self.min_track_frames = int(min_track_frames)
        self._velocity_mm_per_s = 0.0
        self._n_tracks = 0
        self._initialised = False

    def _component(self, trk: Track) -> float:
        if self.axis == "x":
            return trk.vx
        if self.axis == "mag":
            return math.hypot(trk.vx, trk.vy)
        return trk.vy

    def update(self, active_tracks: dict[int, Track]) -> float:
        comps = [self._component(t) for t in active_tracks.values()
                 if t.frames >= self.min_track_frames]
        self._n_tracks = len(comps)
        if comps:
            inst_px = statistics.median(comps)
            inst_mm = inst_px / self.pixels_per_mm if self.pixels_per_mm > 0 else 0.0
            if not self._initialised:
                self._velocity_mm_per_s = inst_mm
                self._initialised = True
            else:
                self._velocity_mm_per_s = (
                    self.ema_alpha * inst_mm + (1.0 - self.ema_alpha) * self._velocity_mm_per_s
                )
        return self._velocity_mm_per_s

    @property
    def velocity_mm_per_s(self) -> float:
        return self._velocity_mm_per_s

    @property
    def n_tracks(self) -> int:
        return self._n_tracks
