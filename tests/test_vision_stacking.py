"""Stacks read from the camera's boxes (modules/vision/stacking.py, open-issues S1): the
overlap test, the confirmation, the "later track is on top" rule, the ambiguous start, and the
detection that carries the result. No camera or model: boxes are synthetic.

Run:  python3 -m unittest tests.test_vision_stacking
"""

from __future__ import annotations

import math
import unittest

try:
    import cv2
    import numpy as np
except ImportError:                       # pragma: no cover
    cv2 = np = None

from tests.helpers import settings as load_settings
from modules.vision.stacking import StackDetector


def box(cx, cy, w=100.0, h=60.0, deg=0.0):
    return (cx, cy, w, h, math.radians(deg), 0, 0.9)


@unittest.skipIf(cv2 is None, "OpenCV not installed")
class Detector(unittest.TestCase):
    def detector(self, **kw):
        return StackDetector(cv2, np, **{"confirm_frames": 2, "min_birth_gap_frames": 2, **kw})

    def test_overlap_fraction_is_over_the_smaller_box(self):
        d = self.detector()
        self.assertAlmostEqual(d.overlap_fraction(box(0, 0), box(0, 0)), 1.0, places=3)
        self.assertAlmostEqual(d.overlap_fraction(box(0, 0), box(50, 0)), 0.5, places=3)
        self.assertEqual(d.overlap_fraction(box(0, 0), box(200, 0)), 0.0)
        self.assertAlmostEqual(d.overlap_fraction(box(0, 0, 100, 60), box(0, 0, 20, 20, 30)), 1.0, places=2)

    def test_a_rotated_box_overlaps_by_its_real_shape(self):
        d = self.detector()
        # Two thin boards crossing at right angles overlap; side by side they do not.
        self.assertGreater(d.overlap_fraction(box(0, 0, 100, 10), box(0, 0, 100, 10, 90)), 0.0)
        self.assertEqual(d.overlap_fraction(box(0, 0, 100, 10), box(0, 40, 100, 10)), 0.0)

    def test_the_later_track_is_on_top_once_confirmed(self):
        d = self.detector()
        self.assertEqual(d.update({1: box(0, 0)}), {})
        self.assertEqual(d.update({1: box(10, 0)}), {})
        self.assertEqual(d.update({1: box(20, 0)}), {})
        self.assertEqual(d.update({1: box(30, 0), 2: box(40, 5)}), {})        # first overlapping frame
        self.assertEqual(d.update({1: box(40, 0), 2: box(50, 5)}), {1: frozenset({2})})
        self.assertEqual(d.new_covers, [(2, 1)])

    def test_the_result_is_sticky_and_survives_a_missed_frame(self):
        d = self.detector()
        for i in range(3):
            d.update({1: box(10 * i, 0)})
        d.update({1: box(30, 0), 2: box(40, 0)})
        d.update({1: box(40, 0), 2: box(50, 0)})
        self.assertEqual(d.update({1: box(50, 0)}), {1: frozenset({2})})       # the top one is missed
        self.assertEqual(d.update({1: box(60, 0), 2: box(200, 200)}), {1: frozenset({2})})
        self.assertEqual(d.new_covers, [])

    def test_a_flicker_of_one_frame_is_not_a_stack(self):
        d = self.detector(confirm_frames=3)
        for i in range(3):
            d.update({1: box(10 * i, 0)})
        d.update({1: box(30, 0), 2: box(40, 0)})
        self.assertEqual(d.update({1: box(40, 0), 2: box(300, 0)}), {})
        self.assertEqual(d.update({1: box(50, 0), 2: box(300, 0)}), {})

    def test_tracks_born_together_have_no_order(self):
        d = self.detector()
        d.update({1: box(0, 0), 2: box(30, 0)})
        d.update({1: box(10, 0), 2: box(40, 0)})
        self.assertEqual(d.update({1: box(20, 0), 2: box(50, 0)}), {})
        self.assertEqual(d.ambiguous, {(1, 2)})

    def test_separate_boards_never_cover_each_other(self):
        d = self.detector()
        for i in range(6):
            self.assertEqual(d.update({1: box(10 * i, 0), 2: box(10 * i, 200)}), {})

    def test_two_boards_lying_on_one(self):
        d = self.detector()
        for i in range(3):
            d.update({1: box(0, 0)})
        for i in range(3):
            d.update({1: box(0, 0), 2: box(20, 0)})
        covered = {}
        for i in range(3):
            covered = d.update({1: box(0, 0), 2: box(20, 0), 3: box(10, 10)})
        self.assertEqual(covered[1], frozenset({2, 3}))
        self.assertEqual(covered[2], frozenset({3}))


class Settings(unittest.TestCase):
    def test_off_by_default_and_the_keys_exist(self):
        s = load_settings().vision.stacking
        self.assertFalse(s.enabled)
        self.assertGreater(s.min_overlap_frac, 0.0)
        self.assertEqual(load_settings(**{"vision.stacking.enabled": True}).vision.stacking.enabled, True)


if __name__ == "__main__":
    unittest.main()
