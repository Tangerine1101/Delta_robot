"""`conveyor.frame.robot_origin_offset` and `pick_gate.gate_out_n`."""

from __future__ import annotations

import time
import unittest
from types import SimpleNamespace

from modules.core.delta import DeltaArm
from modules.core.frames import ConveyorFrame
from modules.core.tracking import BeltTracker, TrackedObject
from modules.runtime.planning import PickPlanner
from tests.helpers import settings as load_settings


class OriginOffset(unittest.TestCase):
    def test_default_offset_changes_nothing(self):
        cfg = load_settings(**{"conveyor.frame.robot_origin_offset": [0.0, 0.0]})
        self.assertEqual(cfg.conveyor.frame.effective_origin_uv, tuple(cfg.conveyor.frame.robot_origin_uv))

    def test_negative_u_offset_lands_the_cup_downstream(self):
        base = load_settings(**{"conveyor.frame.robot_origin_offset": [0.0, 0.0]})
        shifted = load_settings(**{"conveyor.frame.robot_origin_offset": [-11.0, 0.0]})
        true_frame = ConveyorFrame.from_settings(base.conveyor)
        cfg_frame = ConveyorFrame.from_settings(shifted.conveyor)
        x, y = cfg_frame.to_robot(250.0, 60.0)       # where the cup is sent for belt point (250, 60)
        u, v = true_frame.to_conveyor(x, y)           # belt point it actually reaches
        self.assertAlmostEqual(u, 261.0, places=6)
        self.assertAlmostEqual(v, 60.0, places=6)


class GateOut(unittest.TestCase):
    def _planner(self, n: int):
        cfg = load_settings(**{"pick_gate.gate_out_n": n})
        frame = ConveyorFrame.from_settings(cfg.conveyor)
        arm = DeltaArm.from_settings(cfg, frame)
        tracker = BeltTracker(frame, cfg.conveyor.workspace_window_uv,
                              camera_window_uv=cfg.conveyor.camera_window_uv)
        tracker._objects["p1"] = TrackedObject("p1", "TQFP", (200.0, 60.0), belt_pos_anchor=0.0,
                                               last_seen_at=time.monotonic())
        planner = PickPlanner(cfg, arm, frame, tracker)
        planner.ledger.seen("p1", "TQFP", time.monotonic(), 200.0, 60.0)
        return planner, tracker

    @staticmethod
    def _plan(reason: str):
        return SimpleNamespace(object_id="p1", debug_info={"abort_reason": reason},
                               trajectory_goto=[SimpleNamespace(x=0.0, y=0.0, z=-265.0)])

    def test_dropped_after_n_consecutive_gate_late(self):
        planner, tracker = self._planner(3)
        for _ in range(2):
            planner.mark_attempted(self._plan("gate_late"), False, None)
        self.assertIn("p1", [o.object_id for o in tracker.objects()])
        planner.mark_attempted(self._plan("gate_late"), False, None)
        self.assertNotIn("p1", [o.object_id for o in tracker.objects()])
        self.assertIn("p1", planner.planned_object_ids)
        record = planner.ledger.records()[0]
        self.assertEqual((record.outcome, record.reason), ("miss_late", "gate_out"))

    def test_other_abort_resets_the_count(self):
        planner, tracker = self._planner(2)
        planner.mark_attempted(self._plan("gate_late"), False, None)
        planner.mark_attempted(self._plan("goto_failed"), False, None)
        planner.mark_attempted(self._plan("gate_late"), False, None)
        self.assertIn("p1", [o.object_id for o in tracker.objects()])

    def test_zero_disables(self):
        planner, tracker = self._planner(0)
        for _ in range(10):
            planner.mark_attempted(self._plan("gate_late"), False, None)
        self.assertIn("p1", [o.object_id for o in tracker.objects()])


if __name__ == "__main__":
    unittest.main()
