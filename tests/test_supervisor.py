"""The operator console back-end (``modules.ui.supervisor``) against the PLC simulator,
through the unmodified gateways of ``modules/comm``.

Run:  python3 -m unittest tests.test_supervisor
"""

from __future__ import annotations

import math
import threading
import time
import unittest

from modules.comm.omron import PLCGateway
from modules.comm.siemens import SiemensGateway
from modules.core.angles import robot_rad_to_wire_deg
from modules.plc_sim.sim import PLCSim
from modules.ui.supervisor import Supervisor
from tests.helpers import settings as load_settings


def _wait(pred, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(0.05)
    return False


class ConsoleAgainstSimulator(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        settings = load_settings()
        cls.sim = PLCSim(settings=settings, tag_latency_s=0.0, feed_interval_s=2.5)
        cls.sim.start()
        host, port = cls.sim.serve()
        omron = PLCGateway(host, port, settings.plc.interpolar_points, settings.robot.limits)
        siemens = SiemensGateway(host, port)
        omron.connect()
        siemens.connect()
        lock = threading.Lock()

        def dispatch(pkg):
            with lock:
                cmd = pkg.get("commandID")
                if cmd in (7, 8, 9):
                    wire = dict(pkg)
                    if cmd == 7:
                        wire["rotate"] = robot_rad_to_wire_deg(pkg.get("rotate", 0.0))
                    return siemens.send_package(wire)
                omron.send_package(pkg)
                return omron.get_package()

        def request_status():
            with lock:
                status = omron.get_package()
                s = siemens.get_status() or {}
                status.update({"speed_current": s.get("speed_current"),
                               "conveyor_position": s.get("conveyor_position"),
                               "rotate_current": s.get("rotate_current")})
                return status

        cls.events: list[tuple[str, dict]] = []
        cls.sup = Supervisor(
            dispatch, request_status, settings,
            emit=lambda kind, payload: cls.events.append((kind, payload)),
            sim=cls.sim, sim_camera=cls.sim.camera(),
        )
        cls.sup.start()
        assert _wait(lambda: cls.sup.arm_idle()[0], 5.0)

    @classmethod
    def tearDownClass(cls):
        cls.sup.shutdown()
        cls.sim.shutdown()

    def api(self, path, body=None, method="POST"):
        return self.sup.handle_api(method, path, body or {})

    def _idle(self):
        self.assertTrue(_wait(lambda: self.sup.arm_idle()[0], 15.0), self.sup.arm_idle())

    def test_1_goto_and_jog(self):
        self._idle()
        code, r = self.api("/api/manual/goto", {"x": 30.0, "y": -20.0, "z": -290.0})
        self.assertEqual(code, 200, r)
        code, r = self.api("/api/manual/goto", {"x": 0.0, "y": 0.0, "z": -290.0})
        self.assertEqual(code, 409, r)                       # previous goto still running
        self.assertTrue(_wait(lambda: math.dist(self.sim.omron.pos_ee, (30.0, -20.0, -290.0)) < 0.5, 15.0))
        self._idle()
        code, r = self.api("/api/manual/jog", {"axis": "z", "step": 5.0})
        self.assertEqual(code, 200, r)
        self.assertTrue(_wait(lambda: abs(self.sim.omron.pos_ee[2] + 285.0) < 0.5, 10.0))

    def test_2_rejects_unsafe_targets(self):
        code, r = self.api("/api/manual/goto", {"x": 0.0, "y": 0.0, "z": -210.0})
        self.assertEqual(code, 400)                          # z above robot_limits.z_max_mm
        code, r = self.api("/api/manual/goto", {"x": 0.0, "y": 0.0, "z": -310.0})
        self.assertEqual(code, 400)                          # z below robot_limits.z_min_mm
        code, r = self.api("/api/manual/goto", {"x": 150.0, "y": -150.0, "z": -290.0})
        self.assertEqual(code, 400)                          # outside robot_limits.radius_xy_mm
        code, r = self.api("/api/manual/jog", {"axis": "z", "step": 50.0})
        self.assertEqual(code, 400)

    def test_2b_jog_near_calibration_pose(self):
        # After homing (command 4) the arm sits at z = -230.2: gotos and jogs there must pass.
        self._idle()
        code, r = self.api("/api/manual/goto", {"x": 0.0, "y": 0.0, "z": -230.0})
        self.assertEqual(code, 200, r)
        self.assertTrue(_wait(lambda: abs(self.sim.omron.pos_ee[2] + 230.0) < 0.5, 15.0))
        self._idle()
        code, r = self.api("/api/manual/jog", {"axis": "z", "step": -5.0})
        self.assertEqual(code, 200, r)
        self.assertTrue(_wait(lambda: abs(self.sim.omron.pos_ee[2] + 235.0) < 0.5, 10.0))
        self._idle()

    def test_3_pump_in_place(self):
        self._idle()
        before = self.sim.omron.pos_ee
        code, r = self.api("/api/manual/pump", {"on": True})
        self.assertEqual(code, 200, r)
        self.assertTrue(_wait(lambda: self.sim.omron.pump_out, 2.0))
        time.sleep(0.3)
        self.assertLess(math.dist(self.sim.omron.pos_ee, before), 0.05)
        self._idle()
        self.assertEqual(self.api("/api/manual/pump", {"on": False})[0], 200)
        self.assertTrue(_wait(lambda: not self.sim.omron.pump_out, 2.0))

    def test_4_belt_and_rotation(self):
        self.assertEqual(self.api("/api/belt", {"speed": 50.0})[0], 200)
        self.assertTrue(_wait(lambda: self.sim.siemens.speed_current > 10.0, 3.0))
        self.assertEqual(self.api("/api/belt", {"speed": 0.0})[0], 200)
        self.assertEqual(self.api("/api/rotate", {"deg": 45.0})[0], 200)
        self.assertTrue(_wait(lambda: abs(self.sim.siemens.rotate_current - 45.0) < 0.5, 3.0))

    def test_5_scenario_start_stop_locks_manual(self):
        self._idle()
        code, r = self.api("/api/scenario/start", {"name": "production", "duration": None})
        self.assertEqual(code, 200, r)
        self.assertEqual(self.sup.mode, "running")
        code, r = self.api("/api/manual/goto", {"x": 0.0, "y": 0.0, "z": -290.0})
        self.assertEqual(code, 409)
        code, r = self.api("/api/belt", {"speed": 30.0})
        self.assertEqual(code, 409)
        time.sleep(2.0)
        self.assertEqual(self.api("/api/scenario/stop")[0], 200)
        self.assertTrue(_wait(lambda: self.sup.mode == "idle", 40.0), self.sup.mode)
        self.assertTrue(any(k == "log" and "ended" in p["text"] for k, p in self.events))

    def test_6_simulate_feeder_with_run_settings_writes_a_record(self):
        import json
        import tempfile
        from pathlib import Path

        self._idle()
        with tempfile.TemporaryDirectory() as tmp:
            self.sup.record_root = Path(tmp)
            try:
                code, r = self.api("/api/scenario/start", {"name": "simulate_feeder", "duration": 60, "settings": {
                    "planner": "kim", "speed_law": "constant", "static_mm_s": 40,
                    "feeder_kind": "poisson", "feeder_seed": 2, "feeder_rate": 60, "feeder_max_parts": 3}})
                self.assertEqual(code, 200, r)
                self.assertTrue(_wait(lambda: self.sup.mode == "idle", 60.0), self.sup.mode)
            finally:
                self.sup.record_root = None
            [run_dir] = list(Path(tmp).iterdir())
            summary = json.loads((run_dir / "summary.json").read_text())
            for name in ("parts.csv", "flow.csv", "arrivals.csv"):
                self.assertTrue((run_dir / name).exists(), name)
        self.assertEqual(summary["planner"], "kim")
        self.assertEqual(summary["feeder"]["seed"], 2)
        self.assertEqual(summary["feeder"]["params"]["rate_per_min"], 60.0)
        self.assertEqual(summary["input"], 3)
        self.assertEqual(summary["picked"] + summary["miss_grip"] + summary["miss_late"], 3)
        self.assertGreaterEqual(summary["picked"], 1)
        kinds = {k for k, _ in self.events}
        self.assertTrue({"layout", "flow", "run_summary"} <= kinds, kinds)
        # The simulator's own boards were paused for the virtual run and are back.
        self.assertIsNotNone(self.sim.feeder)

    def test_7_rejects_a_rate_for_a_feeder_without_one(self):
        code, r = self.api("/api/scenario/start", {"name": "simulate_feeder", "settings": {
            "feeder_kind": "periodic", "feeder_rate": 30}})
        self.assertEqual(code, 400, r)
        self.assertEqual(self.sup.mode, "idle")


if __name__ == "__main__":
    unittest.main(verbosity=2)
