"""The run's part record (runtime/outcomes.py), the virtual feeder (runtime/virtual_feed.py),
the cell-view geometry and the cross-run report (tools/flow_report.py).

Run:  python3 -m unittest tests.test_outcomes
"""

from __future__ import annotations

import csv
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from modules.core.frames import ConveyorFrame
from modules.runtime.cell_view import arm_geometry
from modules.runtime.outcomes import PartLedger
from modules.runtime.virtual_feed import VirtualFeed, arrival_schedule
from modules.settings import SettingsError
from tests.helpers import settings as load_settings

T0 = 1000.0


def ledger(closed=None) -> PartLedger:
    return PartLedger(T0, grip_tolerance_mm=10.0, on_close=(closed.append if closed is not None else None))


class Classification(unittest.TestCase):
    def test_every_outcome_and_reason(self):
        closed = []
        led = ledger(closed)
        for i in range(7):
            led.seen(f"p{i}", "QFP", T0 + i, 1.0, 50.0)
        # p0: gripped at contact
        led.planned("p0", T0 + 10)
        led.dispatched("p0", T0 + 11)
        led.contact("p0", T0 + 12, (3.0, 4.0), True, 40.0)
        led.pick_done("p0", T0 + 13, True)
        # p1: cup 13 mm off the part
        led.planned("p1", T0 + 10)
        led.dispatched("p1", T0 + 11)
        led.contact("p1", T0 + 12, (12.0, 5.0), True, 40.0)
        led.pick_done("p1", T0 + 13, True)
        # p2: dispatched, the pick trajectory never reached contact
        led.dispatched("p2", T0 + 11)
        led.pick_done("p2", T0 + 14, False)
        # p3: late gate abort, then left the workspace
        led.planned("p3", T0 + 10)
        led.aborted("p3", "gate_late")
        led.left("p3", T0 + 20, past_workspace=True)
        # p4: never planned
        led.left("p4", T0 + 21, past_workspace=True)
        # p5: contact not seen by the pose poll, pick completed
        led.dispatched("p5", T0 + 11)
        led.contact("p5", T0 + 12, (30.0, 0.0), False, 40.0)
        led.pick_done("p5", T0 + 13, True)
        # p6: track died in the camera view, came back, still on the belt at the end
        led.left("p6", T0 + 22, past_workspace=False)
        self.assertEqual(led.counts()["lost_track"], 1)
        led.seen("p6", "QFP", T0 + 23, 5.0, 50.0)
        led.finish(T0 + 30)

        by_id = {r.part_id: (r.outcome, r.reason) for r in led.records()}
        self.assertEqual(by_id, {
            "p0": ("picked", "gripped"),
            "p1": ("miss_grip", "cup_off_part"),
            "p2": ("miss_grip", "pick_motion_failed"),
            "p3": ("miss_late", "gate_late"),
            "p4": ("miss_late", "never_planned"),
            "p5": ("picked", "unverified"),
            "p6": ("on_belt", "run_ended"),
        })
        # Picked at contact, not at pick end; a dispatched part is never closed by `left`.
        p0 = next(r for r in led.records() if r.part_id == "p0")
        self.assertEqual(p0.t_closed, 12.0)
        self.assertAlmostEqual(p0.error_mm, 5.0)
        self.assertEqual(len(closed), 8)          # p6 closed twice (lost, then on_belt)

    def test_flow_bins_and_summary(self):
        led = ledger()
        for i in range(6):
            led.seen(f"p{i}", "QFP", T0 + 2.0 * i, 0.0, 50.0)
        for i in range(4):
            led.dispatched(f"p{i}", T0 + 15 + i)
            led.contact(f"p{i}", T0 + 15 + i, (1.0, 0.0), True, 40.0)
        led.left("p4", T0 + 21, past_workspace=True)
        led.belt_sample(T0 + 5, 40.0, 2)
        led.belt_sample(T0 + 25, 20.0, 1)
        led.finish(T0 + 30)
        with tempfile.TemporaryDirectory() as tmp:
            summary = led.write(Path(tmp), {"planner": "kim"}, bin_s=10.0)
            with open(Path(tmp) / "flow.csv", newline="") as handle:
                bins = list(csv.DictReader(handle))
            with open(Path(tmp) / "parts.csv", newline="") as handle:
                self.assertEqual(len(list(csv.DictReader(handle))), 6)
        self.assertEqual([int(b["input"]) for b in bins], [5, 1, 0])
        self.assertEqual([int(b["picked"]) for b in bins], [0, 4, 0])
        self.assertEqual([int(b["miss_late"]) for b in bins], [0, 0, 1])
        self.assertEqual(float(bins[0]["mean_belt_mm_s"]), 40.0)
        self.assertEqual(summary["planner"], "kim")
        self.assertEqual((summary["input"], summary["picked"], summary["miss_late"], summary["on_belt"]), (6, 4, 1, 1))
        self.assertEqual(summary["pick_rate"], 0.8)
        self.assertEqual(summary["throughput_per_min"], 8.0)
        flow = led.flow(T0 + 30)
        self.assertEqual(flow["throughput_per_min"], 8.0)   # 4 picks in the last 30 s of run


class _Belt:
    """A belt at constant speed, as a position_at callable."""

    def __init__(self, speed: float) -> None:
        self.speed = speed

    def __call__(self, t: float) -> float:
        return self.speed * (t - T0)


class VirtualFeeder(unittest.TestCase):
    def setUp(self):
        self.settings = load_settings(**{"feeder.kind": "poisson", "feeder.seed": 11,
                                         "feeder.kinds.poisson.rate_per_min": 30.0})

    def test_same_seed_same_arrivals(self):
        a = arrival_schedule(self.settings, 120.0)
        b = arrival_schedule(self.settings, 120.0)
        c = arrival_schedule(load_settings(**{"feeder.seed": 12}), 120.0)
        self.assertEqual(a, b)
        self.assertNotEqual(a, c)
        self.assertTrue(40 <= len(a) <= 80, len(a))

    def test_max_parts_and_unknown_type(self):
        capped = load_settings(**{"feeder.max_parts": 5})
        self.assertEqual(len(arrival_schedule(capped, 600.0)), 5)
        bad = load_settings(**{"feeder.kinds.poisson.types": ["QFP", "SOIC"]})
        with self.assertRaises(SettingsError):
            arrival_schedule(bad, 60.0)

    def test_parts_ride_the_belt_through_the_camera_window(self):
        feed = VirtualFeed(self.settings, T0, _Belt(40.0), duration_s=60.0)
        first = feed.events[0]
        t = T0 + first.time_s + 1.0
        detections = feed.poll(t)
        self.assertTrue(detections)
        d = next(x for x in detections if x.object_id == "vf0001")
        u_min = self.settings.conveyor.camera_window_uv[0]
        self.assertAlmostEqual(d.x, u_min + 40.0 * 1.0, places=6)
        self.assertEqual(d.y, first.v_mm)
        self.assertEqual(d.timestamp, t)
        # Past the camera window the part is no longer reported (the tracker dead-reckons it).
        u_max = self.settings.conveyor.camera_window_uv[1]
        later = T0 + first.time_s + (u_max - u_min) / 40.0 + 1.0
        self.assertNotIn("vf0001", {x.object_id for x in feed.poll(later)})

    def test_exhausted_after_the_last_part_leaves(self):
        feed = VirtualFeed(load_settings(**{"feeder.max_parts": 2}), T0, _Belt(100.0), duration_s=60.0)
        self.assertFalse(feed.exhausted)
        t = T0 + feed.events[-1].time_s + 10.0
        feed.poll(t)
        feed.poll(t + 1.0)
        self.assertTrue(feed.exhausted)
        self.assertEqual(feed.describe(), {"scheduled": 2, "landed": 2})


class CellGeometry(unittest.TestCase):
    def test_forearms_keep_their_length(self):
        frame = ConveyorFrame(28.0, (287.0, 88.0))
        for pose in ((0.0, 0.0, -300.0), (80.0, -40.0, -320.0), (-60.0, 90.0, -280.0)):
            g = arm_geometry(frame, pose)
            self.assertEqual(len(g["elbows"]), 3)
            # Distances are frame-independent: elbow (u, v, z) to wrist (u, v, pose z).
            for elbow, wrist in zip(g["elbows"], g["wrists"]):
                self.assertAlmostEqual(math.dist(elbow, (*wrist, pose[2])), 315.0, delta=0.3)


class FlowReport(unittest.TestCase):
    def test_collects_runs(self):
        from modules.tools import flow_report

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for seed in (1, 2):
                led = ledger()
                for i in range(4):
                    led.seen(f"p{i}", "QFP", T0 + 5 * i, 0.0, 50.0)
                    led.contact(f"p{i}", T0 + 5 * i + 3, (1.0, 0.0), True, 40.0)
                led.finish(T0 + 60)
                led.write(root / f"run{seed}", {"scenario": "simulate_feeder", "planner": "kim",
                                                "speed_law": "constant",
                                                "feeder": {"kind": "poisson", "seed": seed,
                                                           "params": {"rate_per_min": 24.0}}}, 10.0)
            out = root / "report"
            argv = ["flow_report", str(root), "--out", str(out)]
            with mock.patch.object(sys, "argv", argv), mock.patch("builtins.print"):
                self.assertEqual(flow_report.main(), 0)
            with open(out / "runs.csv", newline="") as handle:
                runs = list(csv.DictReader(handle))
            self.assertEqual(sorted(r["feeder_seed"] for r in runs), ["1", "2"])
            self.assertEqual({r["feeder_rate"] for r in runs}, {"24.0"})
            self.assertEqual({r["picked"] for r in runs}, {"4"})
            self.assertTrue((out / "windows.csv").exists())
            self.assertEqual(json.loads(runs[0]["miss_reasons"]), {})


if __name__ == "__main__":
    unittest.main(verbosity=2)
