"""The 2026-10 starvation findings: the feeder's hidden horizon, SPT parking the arm for a part
far upstream, and the sandbox that reproduces them (stacking, bypass metrics, dip finder).

Run:  python3 -m unittest tests.test_starvation
"""

from __future__ import annotations

import csv
import math
import random
import tempfile
import unittest
from pathlib import Path

from modules.core.delta import DeltaArm
from modules.core.feeders import build_events
from modules.core.forecast import steady
from modules.core.frames import ConveyorFrame
from modules.runtime.virtual_feed import arrival_schedule
from modules.scheduling import jobs as jobs_module
from modules.scheduling.jobs import build_jobs, released
from modules.scheduling.registry import get_planner
from modules.scheduling.types import ObjectSnapshot, PlanContext
from sandbox.flow import find_dips, flow_series
from sandbox.sweep import run_one
from tests.helpers import settings as load_settings


class FeederHorizon(unittest.TestCase):
    """Seed 110 at 24 parts/min draws exactly 1391 parts in 3600 s: that was the "stop at 1391"."""

    def test_max_parts_is_a_count_not_a_cap_on_schedule_s(self):
        base = {"feeder.seed": 110, "feeder.kinds.poisson.rate_per_min": 24.0,
                "feeder.kinds.poisson.lane_band_v": [15.0, 110.0]}
        default = arrival_schedule(load_settings(**base), 3600.0)
        self.assertEqual(len(default), 1391)
        more = arrival_schedule(load_settings(**base, **{"feeder.max_parts": 2000}), 3600.0)
        self.assertEqual(len(more), 2000)
        self.assertEqual(more[:1391], default)          # the longer horizon only appends

    def test_a_fixed_duration_is_never_extended(self):
        cfg = load_settings(**{"feeder.seed": 110, "feeder.max_parts": 100000})
        events = arrival_schedule(cfg, 600.0, fixed_horizon=True)
        self.assertTrue(events and events[-1].time_s < 600.0)

    def test_a_finite_script_stops_growing(self):
        cfg = load_settings(**{"feeder.kind": "script", "feeder.max_parts": 50,
                               "feeder.kinds.script": {"events": [[1.0, "QFP", 20.0], [2.0, "QFP", 30.0]]}})
        self.assertEqual(len(arrival_schedule(cfg, 60.0)), 2)


def _context(cfg, now: float = 1000.0, belt: float = 12.0) -> tuple[DeltaArm, PlanContext]:
    arm = DeltaArm.from_settings(cfg, ConveyorFrame.from_settings(cfg.conveyor))
    ctx = PlanContext(now, arm.place_position(cfg.bin_of("QFP")), steady(belt), arm,
                      cfg.conveyor.workspace_window_uv, cfg.scheduling.safety_margin_s, cfg.scheduling.setup_time_s)
    return arm, ctx


class ReleasedJobs(unittest.TestCase):
    """vf0080 / vf0082 of run03_200933: the part in reach (u = 164) was passed over for one that
    had just landed (u = 4) and took 19 s to arrive, because SPT ranks by path, not by wait."""

    def setUp(self):
        self.cfg = load_settings()
        self.arm, self.ctx = _context(self.cfg)
        self.bin = self.cfg.bin_of("QFP")

    def _snap(self, name: str, u: float, v: float) -> ObjectSnapshot:
        return ObjectSnapshot(name, "QFP", u, v, self.bin, self.ctx.now)

    def _pair(self):
        """A near part (in reach within seconds) and a far one whose PATH is the shorter."""
        near = self._snap("near", 164.0, 40.0)
        near_path = build_jobs([near], self.ctx)[0].path_mm
        for v in range(0, 125, 5):
            far = self._snap("far", 4.0, float(v))
            job = build_jobs([far], self.ctx)[0]
            if job.intercept is not None and job.path_mm < near_path:
                return near, far
        self.fail("no lane where the far part has the shorter path: the fixture no longer shows the defect")

    def test_wait_separates_the_two_while_path_does_not(self):
        near, far = self._pair()
        near_job, far_job = build_jobs([near, far], self.ctx)
        self.assertLess(far_job.path_mm, near_job.path_mm)
        self.assertGreater(far_job.wait_s, near_job.wait_s + 5.0)
        self.assertEqual([job.obj.object_id for job in released([near_job, far_job])], ["near"])

    def test_spt_takes_the_part_in_reach(self):
        near, far = self._pair()
        jobs = build_jobs([far, near], self.ctx)
        for name in ("spt", "spt_rollout", "edd_rollout"):
            schedule = get_planner(name)(jobs, self.ctx)
            self.assertEqual(schedule[0].job.obj.object_id, "near", name)

    def test_when_everything_is_upstream_the_earliest_arrivals_stay(self):
        jobs = build_jobs([self._snap("a", 20.0, 40.0), self._snap("b", 60.0, 40.0),
                           self._snap("c", 5.0, 40.0)], self.ctx)
        self.assertEqual([j.obj.object_id for j in released(jobs)], ["b"])

    def test_unreachable_jobs_do_not_break_it(self):
        jobs = build_jobs([self._snap("gone", 500.0, 40.0)], self.ctx)
        self.assertEqual(released(jobs), jobs)
        self.assertTrue(math.isinf(jobs[0].wait_s))


class Sandbox(unittest.TestCase):
    BASE = {"sim.seed": 110, "sim.stacking": True, "scheduling.stack_aware": True,
            "speed.law": "predictive_rank", "speed.static_mm_s": 20, "speed.band.min_mm_s": 5,
            "speed.laws.predictive_rank.planner": "drop_longest",
            "feeder.poisson.lane_band_v": [15.0, 110.0], "sim.duration_s": 600,
            "scheduling.planner": "spt_rollout"}

    def test_spt_does_not_park_the_arm_for_an_upstream_part(self):
        """Without `released` the same run bypasses 44 parts and loses 36 of them."""
        result = run_one(self.BASE)
        self.assertEqual(result["bypassed_lost"], 0)
        self.assertGreater(result["picked"], 150)
        saved = jobs_module.RELEASE_SLACK_S
        jobs_module.RELEASE_SLACK_S = math.inf
        try:
            broken = run_one(self.BASE)
        finally:
            jobs_module.RELEASE_SLACK_S = saved
        self.assertGreater(broken["bypassed_lost"], 10)
        self.assertLess(broken["picked"], result["picked"])

    def test_stacking_covers_parts_and_a_cup_on_a_covered_part_misses(self):
        covered = run_one({**self.BASE, "sim.duration_s": 300, "scheduling.stack_aware": False,
                           "speed.law": "constant", "speed.static_mm_s": 10, "scheduling.planner": "edd"})
        self.assertGreater(covered["covered_misses"], 0)
        aware = run_one({**self.BASE, "sim.duration_s": 300, "scheduling.stack_aware": True,
                         "speed.law": "constant", "speed.static_mm_s": 10, "scheduling.planner": "edd"})
        self.assertLess(aware["covered_misses"], covered["covered_misses"])

    def test_no_stacking_no_covered_misses(self):
        result = run_one({**self.BASE, "sim.stacking": False, "sim.duration_s": 120,
                          "scheduling.stack_aware": False})
        self.assertEqual(result["covered_misses"], 0)

    def test_replaying_a_logged_arrival_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "arrivals.csv"
            with open(path, "w", newline="") as handle:
                writer = csv.writer(handle)
                writer.writerow(("part_id", "t_s", "type", "v_mm", "heading_deg", "regime"))
                writer.writerow(("vf0001", 1.0, "QFP", 40.0, 33.0, "poisson"))
                writer.writerow(("vf0002", 4.0, "TQFP", 60.0, 120.0, "poisson"))
            events = build_events("script", {"arrivals_csv": str(path)}, random.Random(1), 60.0)
        self.assertEqual([(e.time_s, e.part_type, e.v_mm, e.heading_deg) for e in events],
                         [(1.0, "QFP", 40.0, 33.0), (4.0, "TQFP", 60.0, 120.0)])


def _rows(arrivals: list[float], picked) -> list[dict[str, str]]:
    """One part per arrival; `picked(t)` says whether its cup came down (a second later)."""
    return [{"part_id": f"p{n}", "t_seen": str(t), "t_contact": str(t + 1.0) if picked(t) else "",
             "outcome": "picked" if picked(t) else "miss_late", "t_closed": str(t + 5.0)}
            for n, t in enumerate(arrivals)]


class DipFinder(unittest.TestCase):
    ARRIVALS = [float(t) for t in range(0, 400, 3)]               # 20 per min throughout

    def test_a_stretch_without_picks_is_found(self):
        dips = find_dips(_rows(self.ARRIVALS, lambda t: not 150 <= t <= 250))
        self.assertEqual(len(dips), 1)
        self.assertTrue(150 <= dips[0].t_start <= 215 and 250 <= dips[0].t_end <= 330, dips)
        self.assertGreater(dips[0].depth_per_min, 15.0)
        self.assertGreater(len(dips[0].victims), 20)

    def test_output_that_follows_the_input_has_no_dip(self):
        self.assertEqual(find_dips(_rows(self.ARRIVALS, lambda t: True)), [])

    def test_series_is_a_trailing_rate(self):
        series = flow_series(_rows(self.ARRIVALS, lambda t: False), step_s=10.0)
        self.assertAlmostEqual(dict((t, i) for t, i, _ in series)[100.0], 20.0, delta=1.0)


if __name__ == "__main__":
    unittest.main()
