"""The pick gate and the intercept solver at the boundaries the realtime loop reaches.

Run:  python3 -m unittest tests.test_pick_gate
"""

from __future__ import annotations

import math
import unittest

from modules.core.delta import GATE_POLL_S, DeltaArm
from modules.core.forecast import steady
from modules.core.frames import ConveyorFrame
from modules.core.tracking import BeltTracker, TrackedObject
from modules.runtime.pick_gate import in_final_segment, object_gate_status
from modules.runtime.plan import PickPlan
from modules.runtime.state import RealtimeState, SpeedSample
from tests.helpers import settings


class GateThreshold(unittest.TestCase):
    def setUp(self) -> None:
        self.cfg = settings()
        self.frame = ConveyorFrame.from_settings(self.cfg.conveyor)
        self.arm = DeltaArm.from_settings(self.cfg, self.frame)
        self.tracker = BeltTracker(self.frame, workspace_window_uv=self.cfg.conveyor.workspace_window_uv,
                                   camera_window_uv=self.cfg.conveyor.camera_window_uv)
        self.u_pick, self.v_pick = 300.0, 60.0
        x, y = self.frame.to_robot(self.u_pick, self.v_pick)
        self.plan = PickPlan("plan-test", "obj-1", "TQFP", 0.0, (self.u_pick, self.v_pick),
                             self.cfg.robot.home_position, (0.0, 0.0), 0.0, 0.0,
                             (x, y, self.cfg.robot.heights.pickup), (149.65, -13.07, -280.0), [], [])

    def _gate(self, u_object: float, belt_speed: float):
        self.tracker._objects.clear()
        obj = TrackedObject("obj-1", "TQFP", (u_object, self.v_pick), belt_pos_anchor=0.0)
        self.tracker._objects[obj.object_id] = obj
        state = RealtimeState(tracker=self.tracker, frame=self.frame, belt_position_mm=0.0,
                              belt_speed_mm_s=belt_speed)
        return object_gate_status(state, self.plan, self.arm.gate_lead_s)

    def test_lead_is_the_delay_budget_times_belt_speed(self):
        gate = self.cfg.pick_gate
        lead_s = gate.robot_movement_delay_s + gate.ethernet_delay_s + GATE_POLL_S / 2.0
        self.assertAlmostEqual(self.arm.gate_lead_s, lead_s, places=12)
        for belt in (0.0, 60.0, 120.0, 200.0):
            status = self._gate(0.0, belt)
            self.assertAlmostEqual(status["pick_u"] - status["threshold_u"], belt * lead_s, places=6)

    def test_offset_moves_the_threshold(self):
        """gate_offset_mm > 0 fires earlier (threshold upstream), < 0 later."""
        self.tracker._objects.clear()
        obj = TrackedObject("obj-1", "TQFP", (0.0, self.v_pick), belt_pos_anchor=0.0)
        self.tracker._objects[obj.object_id] = obj
        state = RealtimeState(tracker=self.tracker, frame=self.frame, belt_speed_mm_s=40.0)
        lead = self.arm.gate_lead_s
        base = object_gate_status(state, self.plan, lead)["threshold_u"]
        self.assertAlmostEqual(object_gate_status(state, self.plan, lead, None, 10.0)["threshold_u"], base - 10.0)
        self.assertAlmostEqual(object_gate_status(state, self.plan, lead, None, -5.0)["threshold_u"], base + 5.0)

    def test_belt_is_extrapolated_to_now(self):
        """A sample one tick old is advanced by speed x age; a stale one is used as it is."""
        self.tracker._objects.clear()
        obj = TrackedObject("obj-1", "TQFP", (100.0, self.v_pick), belt_pos_anchor=0.0)
        self.tracker._objects[obj.object_id] = obj
        state = RealtimeState(tracker=self.tracker, frame=self.frame, belt_position_mm=10.0,
                              belt_speed_mm_s=40.0)
        state.latest_speed = SpeedSample(vx=0.0, vy=0.0, timestamp=5.0, position_mm=10.0, speed_uv=40.0)
        lead = self.arm.gate_lead_s
        self.assertAlmostEqual(object_gate_status(state, self.plan, lead, 5.025)["object_u"], 111.0)
        self.assertAlmostEqual(object_gate_status(state, self.plan, lead)["object_u"], 110.0)
        self.assertAlmostEqual(object_gate_status(state, self.plan, lead, 6.0)["object_u"], 110.0)

    def test_status_is_one_sided(self):
        """The raw status only asks "has it got here yet?"; the late bound is the executor's."""
        for overshoot in (0.0, 20.0, 200.0):
            self.assertTrue(self._gate(self.u_pick + overshoot, 120.0)["reached"])

    def test_missing_track(self):
        self.tracker._objects.clear()
        state = RealtimeState(tracker=self.tracker, frame=self.frame)
        self.assertIsNone(object_gate_status(state, self.plan, self.arm.gate_lead_s))


class FinalSegment(unittest.TestCase):
    def test_vertical_final_segment(self):
        start, target = (10.0, 20.0, -275.0), (10.0, 20.0, -300.0)
        self.assertFalse(in_final_segment((10.0, 20.0, -275.2), start, target))
        self.assertTrue(in_final_segment((10.0, 20.0, -276.0), start, target))
        self.assertTrue(in_final_segment((0.0, 0.0, 0.0), None, target))
        self.assertTrue(in_final_segment((0.0, 0.0, 0.0), (0.0, 0.0, -275.0), target))


class InterceptFeasibility(unittest.TestCase):
    def setUp(self) -> None:
        self.cfg = settings()
        self.arm = DeltaArm.from_settings(self.cfg)
        self.start = self.arm.place_position((149.65, -13.07, -280.0))

    def test_accepted_plans_park_a_full_gate_lead_early(self):
        margins = []
        for belt in (60.0, 90.0, 120.0, 150.0):
            for u_now in (150.0, 200.0, 260.0, 300.0):
                got = self.arm.predict(u_now, 60.0, self.start, 1000.0, steady(belt), 1000.0)
                if got is None:
                    continue
                arrival = 1000.0 + self.arm.command_delay_s + self.arm.trajectory_time(
                    self.arm.goto_points(self.start, got.pick_position))
                margins.append(got.pick_time - arrival)
        self.assertTrue(margins)
        self.assertGreaterEqual(min(margins), self.arm.gate_lead_s - 1e-3)

    def test_plannable_window_marches_upstream_with_belt_speed(self):
        u_max = self.cfg.conveyor.workspace_window_uv[1]
        cutoffs = []
        for belt in (60.0, 90.0, 120.0, 150.0, 180.0):
            plannable = [u for u in range(0, int(u_max) + 1, 2)
                         if self.arm.predict(float(u), 60.0, self.start, 1000.0, steady(belt), 1000.0)]
            if plannable:
                cutoffs.append(max(plannable))
        self.assertTrue(cutoffs)
        self.assertEqual(cutoffs, sorted(cutoffs, reverse=True))

    def test_pick_packet_starts_at_the_contact_point(self):
        contact = (60.0, -120.0, self.cfg.robot.heights.pickup)
        points = self.arm.pick_points(contact, (149.65, -13.07, -280.0))
        self.assertAlmostEqual(math.dist(points[0], contact), 0.0, places=9)

    def test_oblique_descent_shifts_contact_downstream(self):
        from modules.settings import with_overrides

        oblique = DeltaArm.from_settings(with_overrides(self.cfg, {"pick_gate.oblique_descent_enabled": True}))
        park = (40.0, -100.0, self.cfg.robot.heights.pickup)
        contact, t_d = oblique.contact_position(park, 100.0)
        shift = math.dist(park[:2], contact[:2])
        self.assertAlmostEqual(shift, 100.0 * t_d, places=9)
        vertical, _ = self.arm.contact_position(park, 100.0)
        self.assertEqual(vertical, park)


if __name__ == "__main__":
    unittest.main(verbosity=2)
