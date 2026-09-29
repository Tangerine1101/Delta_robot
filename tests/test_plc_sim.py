"""Pins ``modules.plc_sim`` to the ST of ``Matching_Code_10`` and to the defects
documented in ``doc/plc/version-diff-and-defects.md``.

Run:  python3 -m unittest tests.test_plc_sim
"""

from __future__ import annotations

import math
import time
import unittest

from modules.comm.omron import PLCGateway, WorkspaceLimitError
from modules.comm.packets import COMMAND_ID, RobotPacket
from modules.comm.siemens import SiemensGateway
from modules.core.kinematics import calc_forward_kinematic, calc_inverse_kinematics
from modules.plc_sim.omron_core import OmronPLC, Patches
from modules.plc_sim.sim import PLCSim
from modules.plc_sim.world import World
from tests.helpers import settings as load_settings

# Seven points, 20 mm apart on a straight line — every corner has cos θ = 1.
LINE = [(0.0, -60.0 + 20.0 * i, -290.0) for i in range(7)]
WIDE = [(0.0, -90.0 + 30.0 * i, -290.0) for i in range(7)]


def _send(plc: OmronPLC, cmd: int, pts, e=None) -> None:
    e = e or [0] * 7
    for i in range(7):
        x, y, z = pts[i] if i < len(pts) else (0.0, 0.0, 0.0)
        plc.write_tag(f"pc_package.argument_x[{i}]", x)
        plc.write_tag(f"pc_package.argument_y[{i}]", y)
        plc.write_tag(f"pc_package.argument_z[{i}]", z)
        plc.write_tag(f"pc_package.argument_e[{i}]", e[i] if i < len(e) else 0)
    plc.write_tag("pc_package.commandID", cmd)
    plc.write_tag("pc_package.bit_doing", 1)


def _plc_at(p, **kw) -> OmronPLC:
    _, th, _ = calc_inverse_kinematics(*p)
    return OmronPLC(start_angles_deg=th, **kw)


def _run_until_idle(plc: OmronPLC, max_s: float = 5.0) -> float:
    t0 = plc.time_s
    plc.scan()
    while plc.chain_busy and plc.time_s - t0 < max_s:
        plc.scan()
    return plc.time_s - t0


class Kinematics(unittest.TestCase):
    def test_round_trip(self):
        for p in [(0, 0, -290), (80, -40, -300), (-100, 30, -280), (20, 120, -270)]:
            ret, th, limit = calc_inverse_kinematics(*p)
            self.assertFalse(ret or limit, p)
            _, q = calc_forward_kinematic(*th)
            self.assertLess(math.dist(p, q), 1e-3)

    def test_o1_joint_limit_is_silent(self):
        ret, th, limit = calc_inverse_kinematics(0.0, -150.0, -255.0)
        self.assertTrue(limit)
        self.assertFalse(ret)                       # deployed program: "success"
        ret, _, _ = calc_inverse_kinematics(0.0, -150.0, -255.0, limit_is_error=True)
        self.assertTrue(ret)                        # proposed patch


class Profile(unittest.TestCase):
    def test_segment_times_match_the_st_formulas(self):
        plc = _plc_at(LINE[0])
        _send(plc, 3, LINE)
        _run_until_idle(plc)
        # Seg 0: V0=0, L=20, cos=1 -> V_end = min(300, sqrt(2*1000*20)=200, 300) = 200;
        # S_limit = 45 + 25 = 70 > 20 -> V = sqrt((4e7 + 4e7)/2000) = 200 -> t_acc = 0.2,
        # t_dec = 0; estimate adds the 0.08 s soft start.
        self.assertAlmostEqual(plc.icv_t[0], 0.28, places=6)
        # Seg 1: V0=200 -> V_end = sqrt(200² + 40000) = 282.84; V = 282.84; t_acc = 0.08284.
        self.assertAlmostEqual(plc.icv_t[1], (math.sqrt(80000.0) - 200.0) / 1000.0, places=6)
        self.assertLess(math.dist(plc.pos_ee, LINE[-1]), 0.01)

    def test_junction_dwells_exist(self):
        plc = _plc_at(LINE[0])
        _send(plc, 3, LINE)
        last, zero_runs, moving = None, 0, False
        plc.scan()
        while plc.chain_busy:
            plc.scan()
            sp = tuple(ax.setpoint for ax in plc.axes)
            if last is not None:
                still = max(abs(a - b) for a, b in zip(sp, last)) < 1e-9
                if moving and still:
                    zero_runs += 1
                moving = not still
            last = sp
        self.assertGreaterEqual(zero_runs, 5)       # P12: at least one per junction


class Dispatcher(unittest.TestCase):
    def test_task_state_rules(self):
        plc = _plc_at((0.0, 0.0, -290.0))
        _send(plc, 2, [(0.0, 0.0, -280.0)])
        plc.scan()
        self.assertEqual(plc.plc_package["task_state"], 2)
        plc.run_for(3.0)
        self.assertEqual(plc.plc_package["task_state"], 1)
        _send(plc, 3, LINE)
        _run_until_idle(plc)
        plc.run_for(0.5)
        self.assertEqual(plc.plc_package["task_state"], 2)   # P5: never completes
        _send(plc, 5, [])
        plc.scan()
        self.assertEqual(plc.plc_package["task_state"], 1)
        self.assertFalse(plc.pump_out)                       # command 5 has no effect

    def test_pump_releases_half_way_into_segment_five(self):
        plc = _plc_at(LINE[0])
        plc.mem_icv_t5 = 0.1                                 # previous trajectory (O5)
        _send(plc, 3, LINE, [1, 1, 1, 1, 1, 1, 0])
        t_step5 = t_off = None
        plc.scan()
        self.assertTrue(plc.pump_out)
        while plc.chain_busy:
            plc.scan()
            if t_step5 is None and plc.current_step == 5:
                t_step5 = plc.time_s
            if t_step5 is not None and t_off is None and not plc.pump_out:
                t_off = plc.time_s
        self.assertIsNotNone(t_off)
        self.assertAlmostEqual(t_off - t_step5, 0.05, delta=0.009)


class Defects(unittest.TestCase):
    def test_o3_zero_length_segment_stalls(self):
        pts = [LINE[0], LINE[1], LINE[1], LINE[2], LINE[3], LINE[4], LINE[5]]
        plc = _plc_at(LINE[0])
        _send(plc, 3, pts)
        _run_until_idle(plc)
        self.assertLess(math.dist(plc.pos_ee, LINE[1]), 0.01)
        self.assertEqual(plc.event_counts["zero_length_segment"], 1)

    def _overlap(self, patches):
        plc = _plc_at(WIDE[0], patches=patches)
        _send(plc, 3, WIDE)
        plc.scan()
        while not any(plc.icv[k].state == 2 for k in (2, 3)):
            plc.scan()
        _send(plc, 3, list(reversed(WIDE)))
        _run_until_idle(plc)
        _run_until_idle(plc)
        return plc

    def test_o2_command_on_running_chain_jumps(self):
        plc = self._overlap(Patches())
        self.assertGreater(plc.event_counts["cmd3_while_busy"], 0)
        self.assertGreater(plc.event_counts["setpoint_overspeed"], 0)

    def test_o2_patch_rejects_instead(self):
        plc = self._overlap(Patches(o2_reject_busy=True))
        self.assertEqual(plc.event_counts["setpoint_overspeed"], 0)
        self.assertEqual(plc.event_counts["cmd3_rejected"], 1)
        self.assertEqual(plc.plc_package["task_state"], 4)

    def test_o4_padded_single_point_locks_trajectories(self):
        here = (0.0, 0.0, -290.0)
        plc = _plc_at(here)
        _send(plc, 3, [here], [1])
        _run_until_idle(plc, 3.0)
        self.assertEqual(plc.icv[0].state, 99)               # P2: sticky
        _send(plc, 3, LINE)
        plc.run_for(1.0)
        self.assertEqual(plc.icv[0].state, 99)

    def test_o6_aborted_goto_blocks_later_gotos(self):
        plc = _plc_at((0.0, 0.0, -290.0))
        _send(plc, 2, [(60.0, 0.0, -290.0)])
        plc.run_for(0.3)                                     # goto under way
        here = plc.pos_ee
        _send(plc, 3, [here] * 7)                            # any trajectory takes the axes
        plc.run_for(0.5)
        self.assertTrue(plc.mc_goto_abs)                     # Done never came
        _send(plc, 2, [(0.0, 60.0, -290.0)])
        plc.run_for(5.0)
        self.assertGreater(math.dist(plc.pos_ee, (0.0, 60.0, -290.0)), 20.0)   # ignored

    def test_p2_patch_recovers(self):
        here = (0.0, 0.0, -290.0)
        plc = _plc_at(here, patches=Patches(o1_limit_is_error=True, p2_recover=True))
        _send(plc, 3, [here], [1])
        _run_until_idle(plc, 3.0)
        plc.run_for(0.1)
        _send(plc, 3, [plc.pos_ee] + LINE[1:])
        _run_until_idle(plc)
        self.assertLess(math.dist(plc.pos_ee, LINE[-1]), 0.01)


class WorkspaceEnvelope(unittest.TestCase):
    """PLCGateway.send_package rejects any goto/trajectory point outside robot_limits."""

    def _packet(self, command: str, points) -> dict:
        n = len(points)
        return RobotPacket(commandID=COMMAND_ID[command], argument_number=n,
                           argument_x=[p[0] for p in points], argument_y=[p[1] for p in points],
                           argument_z=[p[2] for p in points], argument_e=[0] * n,
                           argument_time=[0.0] * n).to_dict(7)

    def test_z_band_and_radius(self):
        gw = PLCGateway("127.0.0.1", 1, 7, load_settings().robot.limits)
        lim = gw.limits
        gw._check_workspace_limit(self._packet("goto_absolute", [(0.0, 0.0, -230.0)]))
        for bad in [(0.0, 0.0, lim.z_max_mm + 10.0), (0.0, 0.0, lim.z_min_mm - 5.0),
                    (lim.radius_xy_mm, 10.0, -290.0)]:
            with self.assertRaises(WorkspaceLimitError):
                gw._check_workspace_limit(self._packet("goto_absolute", [bad]))
        trajectory = [(0.0, 0.0, -290.0)] * 6 + [(0.0, 0.0, lim.z_min_mm - 1.0)]
        with self.assertRaises(WorkspaceLimitError):
            gw._check_workspace_limit(self._packet("go_trajectory", trajectory))
        # Padding beyond argument_number is not checked (it is zeros).
        gw._check_workspace_limit(self._packet("goto_absolute", [(0.0, 0.0, -290.0)]))


class WorldScoring(unittest.TestCase):
    def test_grip_and_place(self):
        world = World.from_settings(load_settings())
        board = world.spawn("TQFP", 60.0, 0.0, belt_position=0.0, u=250.0)
        x, y = world.frame.to_robot(250.0, 60.0)
        world.update(0.0, 0.0, (x, y, -290.0), True)
        world.update(0.1, 0.0, (x, y, world.pickup_height), True)
        self.assertEqual(board.state, "held")
        bx, by = world.bins["TQFP"]
        world.update(0.5, 0.0, (bx, by, -280.0), False)
        self.assertEqual(board.state, "placed")


class Protocol(unittest.TestCase):
    """The unmodified gateways of modules/comm against the simulator."""

    def test_gateways_drive_the_simulator(self):
        sim = PLCSim(tag_latency_s=0.0)
        sim.start()
        host, port = sim.serve()
        try:
            omron = PLCGateway(host, port, 7, load_settings().robot.limits)
            omron.connect()
            start = tuple(sim.omron.pos_ee)
            pts = [start] + [(start[0], start[1] + 5.0 * i, start[2]) for i in range(1, 7)]
            omron.send_package(RobotPacket(
                commandID=COMMAND_ID["go_trajectory"], argument_number=7,
                argument_x=[p[0] for p in pts], argument_y=[p[1] for p in pts],
                argument_z=[p[2] for p in pts], argument_e=[0] * 7, argument_time=[0.1] * 7,
            ))
            deadline = time.monotonic() + 3.0
            status = omron.get_package()
            while time.monotonic() < deadline and math.dist(status["pos_EE"][:3], pts[-1]) > 0.5:
                time.sleep(0.05)
                status = omron.get_package()
            self.assertLess(math.dist(status["pos_EE"][:3], pts[-1]), 0.5)
            siemens = SiemensGateway(host, port)
            siemens.connect()
            siemens.send_package({"CommandID": COMMAND_ID["change_speed"], "speed": 100.0})
            time.sleep(1.0)
            s = siemens.get_status()
            self.assertGreater(s["speed_current"], 15.0)
            self.assertGreater(s["conveyor_position"], 5.0)      # mm, not cm
        finally:
            sim.shutdown()


if __name__ == "__main__":
    unittest.main(verbosity=2)
