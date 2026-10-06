"""Stack awareness: which part lies on which, what the planners do with it, and how a pick on a
covered part is booked.

Run:  python3 -m unittest tests.test_stacking
"""

from __future__ import annotations

import unittest
from types import SimpleNamespace

from modules.core.footprint import Footprint, board_footprint
from modules.core.forecast import steady
from modules.core.tracking import BeltTracker, ObjectDetection
from modules.runtime.outcomes import PartLedger
from modules.runtime.virtual_feed import VirtualFeed
from modules.scheduling.jobs import build_jobs
from modules.scheduling.registry import get_planner
from modules.scheduling.types import ObjectSnapshot, PlanContext
from modules.ui.supervisor import Supervisor
from tests.helpers import WINDOW, ToyArm, settings as load_settings

T0 = 1000.0
PLANNERS = ("kim", "kim_release", "cardinality_arrival", "cardinality_release", "drop_longest", "dp",
            "spt_rollout", "edd_rollout")


def stack_jobs(*, aware: bool, on_belt=("top",)):
    """`low` lies downstream (earlier in belt order) with `top` lying on its centre; `free`
    is an unrelated part further upstream. Equal processing times, no deadline pressure."""
    objects = (
        ObjectSnapshot("low", "QFP", 350.0, 50.0, (0.0, 0.0, -280.0), 0.0, frozenset({"top"})),
        ObjectSnapshot("top", "QFP", 340.0, 50.0, (0.0, 0.0, -280.0), 1.0),
        ObjectSnapshot("free", "QFP", 330.0, 90.0, (0.0, 0.0, -280.0), 2.0),
    )
    arm = ToyArm({"low": 1.0, "top": 1.0, "free": 1.0})
    arm._by_u = {round(o.u, 6): o.object_id for o in objects}
    ctx = PlanContext(0.0, arm.home, steady(0.0), arm, WINDOW, stack_aware=aware,
                      on_belt=frozenset(on_belt))
    jobs = build_jobs(objects, ctx)
    for job in jobs:
        job.deadline = 100.0
        job.arrival = {"low": 0.0, "top": 1.0, "free": 2.0}[job.obj.object_id]
    return jobs, ctx


class Jobs(unittest.TestCase):
    def test_aware_context_makes_the_covered_part_blocked(self):
        jobs, _ = stack_jobs(aware=True)
        by_id = {job.obj.object_id: job for job in jobs}
        self.assertEqual(by_id["low"].blockers, {"top"})
        self.assertFalse(by_id["low"].feasible)
        self.assertTrue(by_id["top"].feasible and by_id["free"].feasible)

    def test_blind_context_sees_no_blockers(self):
        jobs, _ = stack_jobs(aware=False)
        self.assertTrue(all(not job.blockers and job.feasible for job in jobs))

    def test_a_blocker_that_left_the_belt_is_no_blocker(self):
        jobs, _ = stack_jobs(aware=True, on_belt=())
        self.assertTrue(all(not job.blockers and job.feasible for job in jobs))


class Planners(unittest.TestCase):
    def schedule(self, name: str, aware: bool) -> list[str]:
        jobs, ctx = stack_jobs(aware=aware)
        return [pick.job.obj.object_id for pick in get_planner(name)(jobs, ctx)]

    def test_the_part_on_top_goes_before_the_part_under_it(self):
        for name in PLANNERS:
            order = self.schedule(name, aware=True)
            self.assertIn("low", order, name)
            self.assertLess(order.index("top"), order.index("low"), name)

    def test_blind_sequence_planners_walk_belt_order(self):
        for name in ("kim", "cardinality_arrival", "drop_longest"):
            order = self.schedule(name, aware=False)
            self.assertLess(order.index("low"), order.index("top"), name)

    def test_blocker_that_is_not_a_job_keeps_the_part_out(self):
        jobs, ctx = stack_jobs(aware=True)
        jobs = [job for job in jobs if job.obj.object_id != "top"]
        for name in PLANNERS:
            order = [p.job.obj.object_id for p in get_planner(name)(jobs, ctx)]
            self.assertNotIn("low", order, name)
            self.assertIn("free", order, name)


class Ledger(unittest.TestCase):
    def ledger(self) -> PartLedger:
        led = PartLedger(T0, grip_tolerance_mm=12.7)
        led.seen("low", "QFP", T0, 0.0, 50.0, ("top",))
        led.seen("top", "QFP", T0, 5.0, 50.0)
        return led

    def test_cup_on_a_covered_part_is_a_miss(self):
        led = self.ledger()
        led.contact("low", T0 + 3, (0.5, 0.0), True, 40.0)
        record = next(r for r in led.records() if r.part_id == "low")
        self.assertEqual((record.outcome, record.reason, record.covered_by), ("miss_grip", "covered", "top"))

    def test_picking_the_top_part_first_frees_the_other(self):
        led = self.ledger()
        led.contact("top", T0 + 2, (0.5, 0.0), True, 40.0)
        self.assertFalse(led.is_covered("low"))
        led.contact("low", T0 + 5, (0.5, 0.0), True, 40.0)
        self.assertEqual({r.part_id: r.outcome for r in led.records()}, {"low": "picked", "top": "picked"})

    def test_a_failed_blocker_stays_on_the_belt(self):
        led = self.ledger()
        led.contact("top", T0 + 2, (40.0, 0.0), True, 40.0)           # cup_off_part: left on the belt
        self.assertTrue(led.is_covered("low"))
        self.assertEqual(led.blockers_on_belt({"top"}), {"top"})

    def test_an_unrecorded_coverer_counts_as_on_the_belt(self):
        led = PartLedger(T0, grip_tolerance_mm=12.7)
        led.seen("low", "QFP", T0, 0.0, 50.0, ("hidden",))
        self.assertTrue(led.is_covered("low"))


class Tracking(unittest.TestCase):
    def test_covered_by_only_grows(self):
        cfg = load_settings()
        tracker = BeltTracker(SimpleNamespace(vision_heading_to_robot_rad=lambda deg: 0.0),
                              cfg.conveyor.workspace_window_uv)
        base = dict(object_id="p", x=10.0, y=50.0, object_type="QFP", timestamp=T0)
        tracker.ingest_detection(ObjectDetection(**base), 0.0)
        obj = tracker.ingest_detection(ObjectDetection(**base, covered_by=("a",)), 0.0)
        obj = tracker.ingest_detection(ObjectDetection(**base, covered_by=("b",)), 0.0)
        obj = tracker.ingest_detection(ObjectDetection(**base), 0.0)
        self.assertEqual(obj.covered_by, {"a", "b"})


class Belt:
    def __init__(self, speed: float) -> None:
        self.speed = speed

    def __call__(self, t: float) -> float:
        return self.speed * (t - T0)


class VirtualFeeding(unittest.TestCase):
    def test_footprint_geometry(self):
        square = Footprint(0.0, 0.0, 30.0, 30.0)
        self.assertTrue(square.contains(14.0, -14.0))
        self.assertFalse(square.contains(16.0, 0.0))
        along = board_footprint(0.0, 0.0, 40.0, 10.0, 0.0)             # 180° from +u: width along the belt
        self.assertTrue(along.contains(19.0, 0.0))
        self.assertFalse(along.contains(0.0, 19.0))
        across = board_footprint(0.0, 0.0, 40.0, 10.0, 90.0)           # width across the belt
        self.assertTrue(across.contains(0.0, 19.0))
        self.assertFalse(across.contains(19.0, 0.0))

    def feed(self, **overrides) -> VirtualFeed:
        cfg = load_settings(**{"feeder.kind": "script", "feeder.detection_latency_s": 0.0,
                               "feeder.kinds.script.events": [[0.0, "QFP", 50.0], [0.2, "QFP", 52.0],
                                                             [0.4, "QFP", 110.0]],
                               **overrides})
        return VirtualFeed(cfg, T0, Belt(40.0), duration_s=10.0)

    def test_later_part_covers_the_earlier_one_it_lands_on(self):
        feed = self.feed()
        detections = {d.object_id: d for d in feed.poll(T0 + 0.5)}
        self.assertEqual(detections["vf0001"].covered_by, ("vf0002",))
        self.assertEqual(detections["vf0002"].covered_by, ())
        self.assertEqual(detections["vf0003"].covered_by, ())          # lane 110: far away

    def test_stacking_can_be_switched_off(self):
        feed = self.feed(**{"feeder.stacking": False})
        self.assertTrue(all(not d.covered_by for d in feed.poll(T0 + 0.5)))


class Console(unittest.TestCase):
    def overrides(self, body):
        return Supervisor.run_overrides(SimpleNamespace(settings=load_settings()), body)

    def test_stack_aware_tick_becomes_an_override(self):
        self.assertEqual(self.overrides({"stack_aware": True}), {"scheduling.stack_aware": True})
        self.assertEqual(self.overrides({"stack_aware": False}), {"scheduling.stack_aware": False})
        self.assertEqual(self.overrides({}), {})

    def test_stack_aware_must_be_a_real_bool(self):
        with self.assertRaises(ValueError):
            self.overrides({"stack_aware": "false"})


class LogTools(unittest.TestCase):
    def test_timeline_and_compare_write_figures(self):
        import json
        import sys
        import tempfile
        from pathlib import Path
        from unittest import mock

        from modules.tools import log_compare, log_timeline

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for name, aware in (("blind", False), ("aware", True)):
                led = PartLedger(T0, grip_tolerance_mm=12.7)
                for i in range(4):
                    led.seen(f"p{i}", "QFP", T0 + 5 * i, 0.0, 50.0)
                    led.contact(f"p{i}", T0 + 5 * i + 3, (1.0, 0.0), True, 40.0)
                led.finish(T0 + 60)
                led.write(root / name, {"scenario": "simulate_feeder", "planner": "kim", "stack_aware": aware,
                                        "speed_law": "constant"}, 10.0)
            with mock.patch("builtins.print"):
                with mock.patch.object(sys, "argv", ["log_timeline", str(root)]):
                    self.assertEqual(log_timeline.main(), 0)
                with mock.patch.object(sys, "argv", ["log_compare", str(root), "--out", str(root / "cmp")]):
                    self.assertEqual(log_compare.main(), 0)
            self.assertEqual(len(list((root / "timelines").glob("*_timeline.png"))), 2)
            self.assertTrue((root / "cmp" / "compare.png").exists())
            self.assertEqual(len(json.loads(json.dumps(list(log_compare.collect([root], None, 0, 1e9))))), 2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
