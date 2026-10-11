"""Which board lies on which, from what the camera sees (open-issues S1).

The YOLO-OBB detector gives every board a rotated box. Two boards whose boxes overlap, frame
after frame, are a stack. The detector cannot tell which is on top, so the rule of the virtual
feed is used: a board dropped later lies on the earlier one, i.e. the track that appeared
later is on top. Tracks that appear within `min_birth_gap_frames` of each other (a stack that
entered the view together) have no usable order and assert nothing.

The result is sticky: once a track is seen under another it stays covered, and the belt
tracker keeps that after the board leaves the camera view — so the check is done while the
board is in view, and the planner and the pick gate read it afterwards. Pure geometry on the
boxes; OpenCV is handed in by the pipeline (it is only used for polygon areas).
"""

from __future__ import annotations

from collections import defaultdict
from typing import Any

from modules.vision.heading import Obb, _obb_corners

_FORGET_AFTER_FRAMES = 600     # state of a track not seen for this long is dropped


class StackDetector:
    def __init__(self, cv2_mod: Any, np_mod: Any, *, min_overlap_frac: float = 0.15,
                 confirm_frames: int = 3, min_birth_gap_frames: int = 2) -> None:
        self._cv2, self._np = cv2_mod, np_mod
        self.min_overlap_frac = float(min_overlap_frac)
        self.confirm_frames = max(1, int(confirm_frames))
        self.min_birth_gap_frames = max(1, int(min_birth_gap_frames))
        self.frame = 0
        self._birth: dict[int, int] = {}
        self._last_seen: dict[int, int] = {}
        self._run: dict[tuple[int, int], int] = {}          # consecutive overlapping frames per pair
        self._covered: dict[int, set[int]] = defaultdict(set)
        self.new_covers: list[tuple[int, int]] = []         # (top, bottom) asserted by the last update
        self.ambiguous: set[tuple[int, int]] = set()        # overlapping pairs with no usable order

    def overlap_fraction(self, a: Obb, b: Obb) -> float:
        """Shared area over the area of the smaller box (1 = one box inside the other)."""
        cv2, np = self._cv2, self._np
        pa, pb = _obb_corners(a, cv2, np), _obb_corners(b, cv2, np)
        smaller = min(float(cv2.contourArea(pa)), float(cv2.contourArea(pb)))
        if smaller <= 0.0:
            return 0.0
        area, _ = cv2.intersectConvexConvex(pa, pb)
        return max(0.0, float(area)) / smaller

    def update(self, boards: dict[int, Obb]) -> dict[int, frozenset[int]]:
        """One frame: {track id: its box}. Returns {track id: the tracks lying on it} for the
        tracks of this frame that are covered."""
        self.frame += 1
        self.new_covers = []
        for tid in boards:
            self._birth.setdefault(tid, self.frame)
            self._last_seen[tid] = self.frame
        ids = sorted(boards)
        seen_pairs = set()
        for i, a in enumerate(ids):
            for b in ids[i + 1:]:
                if self.overlap_fraction(boards[a], boards[b]) < self.min_overlap_frac:
                    continue
                pair = (a, b)
                seen_pairs.add(pair)
                self._run[pair] = self._run.get(pair, 0) + 1
                if self._run[pair] != self.confirm_frames:
                    continue
                gap = self._birth[b] - self._birth[a]
                if abs(gap) < self.min_birth_gap_frames:
                    self.ambiguous.add(pair)
                    continue
                top, bottom = (b, a) if gap > 0 else (a, b)
                if top not in self._covered[bottom]:
                    self._covered[bottom].add(top)
                    self.new_covers.append((top, bottom))
        for pair in [p for p in self._run if p not in seen_pairs]:
            del self._run[pair]
        self._forget()
        return {tid: frozenset(self._covered[tid]) for tid in boards if self._covered.get(tid)}

    def _forget(self) -> None:
        stale = [t for t, seen in self._last_seen.items() if self.frame - seen > _FORGET_AFTER_FRAMES]
        for tid in stale:
            for table in (self._birth, self._last_seen, self._covered):
                table.pop(tid, None)
