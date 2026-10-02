"""Camera latency from belt speed steps (modules/core/latency.py).

Run:  python3 -m unittest tests.test_latency
"""

from __future__ import annotations

import unittest

from modules.core.latency import change_lags, median_lag_s

TICK_S = 0.025          # perception period
FRAME_S = 1.0 / 30.0    # camera frame period


def _belt_speed(t: float, steps: list[tuple[float, float]]) -> float:
    speed = 0.0
    for t_step, v in steps:
        if t >= t_step:
            speed = v
    return speed


def _simulate(stamp_late_s: float, scale: float, steps: list[tuple[float, float]], duration_s: float):
    """A part at belt offset 10 mm seen by a camera whose stamps are `stamp_late_s` late and
    whose distances are `scale` x true; returns the recorder's belt and offset samples."""
    dt = 0.001
    times, positions = [0.0], [0.0]
    t = 0.0
    while t < duration_s:
        t += dt
        positions.append(positions[-1] + _belt_speed(t, steps) * dt)
        times.append(t)

    def p(at: float) -> float:
        i = min(max(int(round(at / dt)), 0), len(positions) - 1)
        return positions[i]

    belt, offsets = [], []
    t = 0.5
    while t < duration_s - 0.5:
        frame_true = (t // FRAME_S) * FRAME_S - 0.04          # newest frame, 40 ms in the pipe
        u_cam = 10.0 + scale * p(frame_true)
        stamp = frame_true + stamp_late_s
        u_tracked = u_cam + p(t) - p(stamp)
        belt.append((t, p(t), _belt_speed(t, steps)))
        offsets.append((t, u_tracked - p(t)))
        t += TICK_S
    return belt, {"part": offsets}


class CameraLag(unittest.TestCase):
    STEPS = [(2.0, 30.0), (5.0, 0.0), (8.0, 30.0), (11.0, 0.0)]

    def test_recovers_a_late_stamp(self):
        belt, offsets = _simulate(0.06, 1.0, self.STEPS, 13.0)
        lags = change_lags(belt, offsets)
        self.assertEqual(len(lags), 4)
        self.assertAlmostEqual(median_lag_s(lags), 0.06, delta=0.008)

    def test_unaffected_by_a_scale_error(self):
        belt, offsets = _simulate(0.06, 1.07, self.STEPS, 13.0)
        self.assertAlmostEqual(median_lag_s(change_lags(belt, offsets)), 0.06, delta=0.01)

    def test_correct_stamp_reads_zero(self):
        belt, offsets = _simulate(0.0, 1.0, self.STEPS, 13.0)
        self.assertAlmostEqual(median_lag_s(change_lags(belt, offsets)), 0.0, delta=0.008)

    def test_no_change_no_estimate(self):
        belt, offsets = _simulate(0.06, 1.0, [(0.0, 30.0)], 6.0)
        self.assertEqual(change_lags(belt, offsets), [])
        self.assertIsNone(median_lag_s([]))


if __name__ == "__main__":
    unittest.main(verbosity=2)
