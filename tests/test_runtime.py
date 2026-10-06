"""The realtime pieces driven end to end with scripted plants: executor dispatch order and the
setpoint freeze (L9), the plan's descent model, belt history stamping, and the decision loop's
bookkeeping after a failed pick. No hardware, no network.

Run:  python3 -m unittest tests.test_runtime
"""

from __future__ import annotations

import threading
import time
import unittest
from types import SimpleNamespace

from modules.comm.packets import COMMAND_ID
from modules.core.arm_model import Intercept
from modules.core.delta import DeltaArm
from modules.core.forecast import BeltForecast
from modules.core.frames import ConveyorFrame
from modules.core.tracking import BeltPositionTracker, BeltTracker, TrackedObject
from modules.runtime.loop import RunContext, run_pick_loop
from modules.runtime.pick_executor import RealtimePickExecutor
from modules.runtime.pick_gate import belt_lead_offset_mm, find_tracked_object
from modules.runtime.plan import build_pick_plan
from modules.runtime.planning import PickPlanner
from modules.runtime.speed import SpeedController
from modules.runtime.speed_source import ConveyorSpeedSource
from modules.runtime.state import RealtimeState, SpeedSample
from modules.scheduling.registry import get_planner
from tests.helpers import settings as load

GO_TRAJECTORY = COMMAND_ID["go_trajectory"]
CHANGE_SPEED = COMMAND_ID["change_speed"]
ROTATE_ABSOLUTE = COMMAND_ID["rotate_absolute"]

BELT_MM_S = 120.0
U_PICK = 300.0
V_LANE = 60.0
QFP_BIN = (149.65, -13.07, -280.0)


def _fixture(now: float, **overrides):
    cfg = load(**{"speed.law": "inverse_density", "speed.static_mm_s": BELT_MM_S,
                  "runtime.poll_interval_s": 0.02, "pick_gate.arrival_timeout_margin_s": 3.0,
                  **overrides})
    frame = ConveyorFrame.from_settings(cfg.conveyor)
    arm = DeltaArm.from_settings(cfg, frame)
    tracker = BeltTracker(frame, cfg.conveyor.workspace_window_uv,
                          camera_window_uv=cfg.conveyor.camera_window_uv)
    obj = TrackedObject("seq-obj-1", "TQFP", (230.0, V_LANE), belt_pos_anchor=0.0,
                        rotation_rad=0.4, vision_angle_deg=10.0, last_seen_at=now)
    tracker._objects[obj.object_id] = obj
    x, y = frame.to_robot(U_PICK, V_LANE)
    intercept = Intercept(now + 1.0, now + 1.0 - arm.gate_lead_s, (x, y, cfg.robot.heights.pickup), U_PICK, 0.5)
    vx, vy = frame.velocity_to_robot(BELT_MM_S)
    plan = build_pick_plan(arm, "plan-test", obj, cfg.object_types["TQFP"].bin, intercept, QFP_BIN,
                           0.0, BELT_MM_S, (vx, vy), cfg.plc.interpolar_points)
    return cfg, frame, arm, tracker, plan


class _RecordingDispatcher:
    def __init__(self):
        self.lock = threading.Lock()
        self.events: list[tuple[float, int, dict]] = []

    def __call__(self, packet):
        with self.lock:
            self.events.append((time.monotonic(), int(packet.get("commandID", -1)), dict(packet)))
        return None

    def of_type(self, command_id):
        with self.lock:
            return [(t, p) for (t, c, p) in self.events if c == command_id]


class _BeltDriver(threading.Thread):
    """Advances the belt in real time and keeps a fresh belt sample in the state."""

    def __init__(self, state):
        super().__init__(daemon=True)
        self.state = state
        self.stop_event = threading.Event()
        self.t0 = time.monotonic()

    def run(self):
        while not self.stop_event.is_set():
            now = time.monotonic()
            p = BELT_MM_S * (now - self.t0)
            with self.state.state_lock:
                self.state.belt_position_mm = p
                self.state.belt_speed_mm_s = BELT_MM_S
                self.state.belt_speed_measured_mm_s = BELT_MM_S
                self.state.latest_speed = SpeedSample(0.0, BELT_MM_S, now, p, BELT_MM_S)
            time.sleep(0.005)


class _PoseDriver(threading.Thread):
    """Scripts the pose: depart, park, then descend through contact, lift, land on the bin."""

    def __init__(self, state, dispatcher, plan):
        super().__init__(daemon=True)
        self.state, self.dispatcher, self.plan = state, dispatcher, plan
        self.t_contact_pose: float | None = None
        self.failure: str | None = None

    def _set(self, pose):
        with self.state.state_lock:
            self.state.robot_pose = pose

    def _wait_for(self, n, timeout=6.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if len(self.dispatcher.of_type(GO_TRAJECTORY)) >= n:
                return True
            time.sleep(0.005)
        return False

    def run(self):
        end = self.plan.trajectory_goto[-1]
        park = (end.x, end.y, end.z)
        contact_z = self.plan.trajectory_pick[0].z
        last = self.plan.trajectory_pick[-1]
        if not self._wait_for(1):
            self.failure = "goto never dispatched"
            return
        self._set(QFP_BIN)
        time.sleep(0.08)
        self._set(park)
        if not self._wait_for(2):
            self.failure = "pick never dispatched"
            return
        time.sleep(0.06)
        self._set((park[0], park[1], (park[2] + contact_z) / 2.0 - 1.0))
        time.sleep(0.06)
        self.t_contact_pose = time.monotonic()
        self._set((park[0], park[1], contact_z))
        time.sleep(0.06)
        self._set(park)
        time.sleep(0.3)
        self._set((last.x, last.y, last.z))


class ExecutorSequence(unittest.TestCase):
    def test_dispatch_order_and_setpoint_frozen_until_contact(self):
        now = time.monotonic()
        cfg, frame, arm, tracker, plan = _fixture(now)
        dispatcher = _RecordingDispatcher()
        executor = RealtimePickExecutor(dispatcher, lambda: None, cfg, arm)
        state = RealtimeState(tracker=tracker, frame=frame, ipc_lock=executor.ipc_lock)
        planner = PickPlanner(cfg, arm, frame, tracker)
        speed = SpeedController(cfg, state, planner, get_planner("spt"), executor.dispatch)
        executor.on_speed_event = speed.tick
        belt, pose = _BeltDriver(state), _PoseDriver(state, dispatcher, plan)
        belt.start()
        pose.start()
        time.sleep(0.05)
        try:
            ok = executor.execute(plan, state)
        finally:
            belt.stop_event.set()
            pose.join(timeout=8.0)
            belt.join(timeout=2.0)

        self.assertIsNone(pose.failure, pose.failure)
        self.assertTrue(ok)
        trajectories = dispatcher.of_type(GO_TRAJECTORY)
        rotates = dispatcher.of_type(ROTATE_ABSOLUTE)
        commits = dispatcher.of_type(CHANGE_SPEED)
        self.assertEqual(len(trajectories), 2)
        t_goto, t_pick = trajectories[0][0], trajectories[1][0]
        # rotate #1 homes the cup during the goto; #2 turns the gripped board after the pick.
        self.assertEqual(len(rotates), 2)
        self.assertAlmostEqual(rotates[0][1]["rotate"], 0.0)
        self.assertLess(rotates[0][0], t_pick)
        self.assertGreater(rotates[1][0], t_pick)
        self.assertAlmostEqual(rotates[1][1]["rotate"], plan.rotate_rad, places=6)
        # L9: nothing moves the setpoint from goto dispatch to cup contact...
        self.assertIsNotNone(pose.t_contact_pose)
        frozen = [p["speed"] for t, p in commits if t_goto <= t <= pose.t_contact_pose]
        self.assertEqual(frozen, [])
        # ...and the at_contact gate lets the law commit once contact cleared the freeze.
        self.assertGreater(len([t for t, _ in commits if t > pose.t_contact_pose]), 0)
        self.assertFalse(state.pick_committed)


class PlanDescent(unittest.TestCase):
    def test_descend_time_is_the_vertical_model(self):
        cfg, _, arm, _, plan = _fixture(time.monotonic())
        self.assertFalse(cfg.pick_gate.oblique_descent_enabled)
        h, pt = cfg.robot.heights, cfg.robot.packet_time
        self.assertAlmostEqual(plan.trajectory_pick[0].time_s,
                               max(0.08, (h.pre_pick - h.pickup) / pt.nominal_z_speed), places=6)
        self.assertAlmostEqual(plan.descend_time_s, arm.vertical_descent_time_s(), places=12)
        self.assertEqual(plan.debug_info["contact_position_3d"], plan.debug_info["pick_position_3d"])


class BeltHistoryStamp(unittest.TestCase):
    """The belt sample is stamped at the middle of the blocking PLC round trip, so camera
    backdating reads the belt position with at most ~v*RTT of bias."""

    RTT_S = 0.05
    V = 120.0

    def test_position_at_bias(self):
        t0 = time.monotonic()

        def fake_request_status():
            time.sleep(self.RTT_S)
            return {"conveyor_position": self.V * (time.monotonic() - t0)}

        decoder = BeltPositionTracker(velocity_ema_alpha=0.4)
        cfg = load()
        source = ConveyorSpeedSource(fake_request_status, ConveyorFrame.from_settings(cfg.conveyor), decoder)
        for _ in range(12):
            source.sample(time.monotonic())
        t_query = time.monotonic() - 0.2
        bias_mm = decoder.position_at(t_query) - self.V * (t_query - t0)
        self.assertGreater(bias_mm, self.V * self.RTT_S * 0.25)
        self.assertLess(bias_mm, self.V * self.RTT_S * 1.6)


class _AlwaysFailExecutor:
    def __init__(self):
        self.ipc_lock = threading.Lock()
        self.round_trip = SimpleNamespace(average_s=0.0)
        self.on_speed_event = None
        self.plans: list = []

    def dispatch(self, packet):
        return None

    def execute(self, plan, state):
        self.plans.append(plan)
        plan.status = "failed"
        return False


class _StubSpeedSource:
    def __init__(self, cfg, v_mm_s, pose=None):
        self.v = v_mm_s
        self.t0 = time.monotonic()
        self.frame = ConveyorFrame.from_settings(cfg.conveyor)
        self.last_status = {"pos_EE": list(pose), "end_effector": 0} if pose else None

    def sample(self, now):
        vx, vy = self.frame.velocity_to_robot(self.v)
        return SpeedSample(vx, vy, now, self.v * (now - self.t0), self.v)

    def position_at(self, t):
        return None


class _NoCamera:
    def poll(self, now):
        return []


class FailedPickBookkeeping(unittest.TestCase):
    def _run(self, pose):
        cfg = load(**{"speed.law": "constant", "scheduling.planner": "spt"})
        frame = ConveyorFrame.from_settings(cfg.conveyor)
        arm = DeltaArm.from_settings(cfg, frame)
        tracker = BeltTracker(frame, cfg.conveyor.workspace_window_uv,
                              camera_window_uv=cfg.conveyor.camera_window_uv)
        now = time.monotonic()
        tracker._objects["fail-obj"] = TrackedObject("fail-obj", "TQFP", (200.0, 60.0), belt_pos_anchor=0.0,
                                                     last_seen_at=now)
        planner = PickPlanner(cfg, arm, frame, tracker)
        executor = _AlwaysFailExecutor()
        state = RealtimeState(tracker=tracker, frame=frame, ipc_lock=executor.ipc_lock)
        run = RunContext("production", cfg, state, planner, _StubSpeedSource(cfg, 60.0, pose), _NoCamera(),
                         now, 1.0)
        run_pick_loop(run, executor)
        return planner, executor, tracker

    def test_falls_back_to_the_park_point_without_a_pose(self):
        planner, executor, tracker = self._run(None)
        self.assertGreaterEqual(len(executor.plans), 1)
        park = executor.plans[-1].trajectory_goto[-1]
        self.assertEqual(planner.current_position, (park.x, park.y, park.z))
        # A pre-grip failure leaves the part on the belt to be planned again.
        self.assertIsNotNone(find_tracked_object(tracker, "fail-obj"))

    def test_uses_the_last_reported_pose(self):
        reported = (5.0, -40.0, -295.0)
        planner, executor, _ = self._run(reported)
        self.assertGreaterEqual(len(executor.plans), 1)
        self.assertEqual(planner.current_position, reported)


class RampLeadResidual(unittest.TestCase):
    """The gate lead is v*T; mid-ramp the integrated belt travel differs by ~a*T^2/2 (L9)."""

    def test_residual_is_small_for_one_settled_step(self):
        lead_s = 0.2235
        forecast = BeltForecast(120.0, 100.0, 22.31).at(0.2)
        residual = belt_lead_offset_mm(forecast.v0, lead_s) - forecast.travel_in(lead_s)
        self.assertGreater(abs(residual), 0.2)
        self.assertLess(abs(residual), 1.5)


if __name__ == "__main__":
    unittest.main(verbosity=2)
