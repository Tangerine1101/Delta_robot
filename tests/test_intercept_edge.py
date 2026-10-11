"""The intercept of a part upstream of the workspace lands exactly on the window edge; float
rounding there must not make it unreachable, and dp's `max_jobs` comes from the config.

Run:  python3 -m unittest tests.test_intercept_edge
"""

from __future__ import annotations

import random
import unittest

from modules.core.delta import DeltaArm
from modules.core.forecast import steady
from modules.core.frames import ConveyorFrame
from modules.scheduling.registry import get_planner
from tests.helpers import settings as load_settings


class UpstreamEdge(unittest.TestCase):
    def test_upstream_parts_stay_reachable_whatever_the_clock(self):
        cfg = load_settings()
        arm = DeltaArm.from_settings(cfg, ConveyorFrame.from_settings(cfg.conveyor))
        park = arm.place_position(cfg.bin_of("TQFP"))
        u_min = cfg.conveyor.workspace_window_uv[0]
        rng = random.Random(2)
        for speed in (5.0, 15.0, 30.0):
            for _ in range(300):
                start = 9000.0 + rng.random() * 400.0
                u, v = rng.uniform(0.0, u_min - 1.0), rng.uniform(5.0, 120.0)
                intercept = arm.predict(u, v, park, start, steady(speed), start)
                self.assertIsNotNone(intercept, (speed, u, v, start))

    def test_the_recorded_failure(self):
        """vf0834 of run03_164219: u = 174.7 at now = 9252.6 sat 2.6e-12 mm below u_min."""
        cfg = load_settings()
        arm = DeltaArm.from_settings(cfg, ConveyorFrame.from_settings(cfg.conveyor))
        park = arm.place_position(cfg.bin_of("TQFP"))
        self.assertIsNotNone(arm.predict(174.7, 96.8, park, 9252.6, steady(5.0), 9252.6))

    def test_a_part_past_the_edge_is_still_unreachable(self):
        cfg = load_settings()
        arm = DeltaArm.from_settings(cfg, ConveyorFrame.from_settings(cfg.conveyor))
        park = arm.place_position(cfg.bin_of("TQFP"))
        u_max = cfg.conveyor.workspace_window_uv[1]
        self.assertIsNone(arm.predict(u_max + 1.0, 60.0, park, 100.0, steady(5.0), 100.0))


class DpMaxJobs(unittest.TestCase):
    def test_config_reaches_the_planner(self):
        cfg = load_settings(**{"scheduling.planners": {"dp": {"max_jobs": 3}}})
        self.assertEqual(get_planner("dp", cfg.scheduling.planners).cfg.max_jobs, 3)
        configured = load_settings().scheduling.planners.get("dp", {}).get("max_jobs", 8)
        self.assertEqual(get_planner("dp", load_settings().scheduling.planners).cfg.max_jobs, configured)


if __name__ == "__main__":
    unittest.main()
