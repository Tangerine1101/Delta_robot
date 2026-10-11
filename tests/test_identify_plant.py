"""System identification from a console session (``modules.tools.identify_plant``) on a
synthetic session whose timings are known.

Run:  python3 -m unittest tests.test_identify_plant
"""

from __future__ import annotations

import csv
import json
import shutil
import tempfile
import unittest
from pathlib import Path

import numpy as np

from modules.config_io import CONFIG_PATH
from modules.tools import identify_plant

GOTO_S, WAIT_S, PICK_S, SETUP_S = 0.90, 0.20, 0.95, 0.07
LAG_S = 0.10          # send -> first motion


def _trajectory(command_id, sent, points, rtt=0.065):
    n = len(points)
    return {"t_wall": 0.0, "t_mono_send": sent, "t_mono_done": sent + rtt, "rtt_s": rtt, "commandID": command_id,
            "ok": True, "error": None, "package": {
                "commandID": command_id, "argument_number": n, "argument_x": [p[0] for p in points],
                "argument_y": [p[1] for p in points], "argument_z": [p[2] for p in points]}, "reply": {}}


class IdentifyPlant(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.session = self.tmp / "session"
        self.session.mkdir()
        shutil.copy(CONFIG_PATH, self.session / "config.yaml")
        bin_xyz, part_xyz = (150.0, 0.0, -280.0), (30.0, -60.0, -287.0)
        t0 = 1000.0
        goto1, pick1, goto2 = t0, t0 + GOTO_S + WAIT_S, t0 + GOTO_S + WAIT_S + PICK_S + SETUP_S
        wp = lambda a, b: [a, a, b, b]
        commands = [_trajectory(3, goto1, wp(bin_xyz, part_xyz)),
                    _trajectory(3, pick1, wp((30.0, -60.0, -300.0), bin_xyz)),
                    _trajectory(3, goto2, wp(bin_xyz, part_xyz))]
        # belt: a 20 mm/s step at t0 + 5, the servo follows 0.08 s later
        commands += [{"t_mono_send": t0 + 2, "t_mono_done": t0 + 2.03, "rtt_s": 0.03, "commandID": 8, "ok": True,
                      "package": {"commandID": 8, "speed": 10.0}},
                     {"t_mono_send": t0 + 5, "t_mono_done": t0 + 5.03, "rtt_s": 0.03, "commandID": 8, "ok": True,
                      "package": {"commandID": 8, "speed": 30.0}},
                     {"t_mono_send": t0 + 9, "t_mono_done": t0 + 9.03, "rtt_s": 0.03, "commandID": 8, "ok": True,
                      "package": {"commandID": 8, "speed": 30.0}}]
        (self.session / "commands.jsonl").write_text("\n".join(json.dumps(c) for c in commands) + "\n")

        # pose: sits at each leg's start, moves linearly from send + LAG_S to its end
        legs = [(goto1, bin_xyz, part_xyz, GOTO_S), (pick1, (30.0, -60.0, -300.0), bin_xyz, PICK_S),
                (goto2, bin_xyz, part_xyz, GOTO_S)]
        times = np.arange(t0 - 0.5, t0 + 5.0, 0.012)
        pose = np.tile(np.array(bin_xyz), (len(times), 1))
        for sent, start, end, duration in legs:
            frac = np.clip((times - sent - LAG_S) / (duration - LAG_S), 0.0, 1.0)[:, None]
            moving = times >= sent
            pose[moving] = (np.array(start) + frac * (np.array(end) - np.array(start)))[moving]
        with open(self.session / "pose.csv", "w") as handle:
            handle.write("t_mono,t_wall,x,y,z\n")
            for t, (x, y, z) in zip(times, pose):
                handle.write(f"{t:.6f},0,{x:.4f},{y:.4f},{z:.4f}\n")
        status_t = np.arange(t0, t0 + 11.0, 0.04)
        speed = np.where(status_t < t0 + 2.08, 0.0, np.where(status_t < t0 + 5.08, 10.0, 30.0))
        with open(self.session / "status.csv", "w") as handle:
            handle.write("t_mono,t_wall,rtt_s,omron_read,pose_source,pose_age_s,x,y,z,conveyor_position,speed_current\n")
            for t, v in zip(status_t, speed):
                handle.write(f"{t:.6f},0,0.03,1,udp,0.01,0,0,-240,0,{v}\n")

        end = t0 + 20.0
        for name, picked in (("run01_a_simulate_feeder", 150), ("run02_b_simulate_feeder", 3)):
            run = self.session / name
            run.mkdir()
            (run / "summary.json").write_text(json.dumps({"duration_s": 20.0, "picked": picked, "run_name": name[:5]}))
            with open(run / "parts.csv", "w", newline="") as handle:
                writer = csv.writer(handle)
                writer.writerow(["part_id", "outcome", "error_u_mm", "error_v_mm", "belt_mm_s_at_contact"])
                for i in range(20):
                    speed_at = 10.0 + i
                    writer.writerow([f"p{i}", "picked", 0.02 * speed_at + 0.4, 0.0, speed_at])
        (self.session / "console.log").write_text(
            "\n".join(f"{1.79e9:.6f} {end:.6f} [CONSOLE] INFO scenario 'simulate_feeder' ended (9 plans); record in "
                      f"/x/{name}" for name in ("run01_a_simulate_feeder", "run02_b_simulate_feeder")) + "\n")

    def test_windows_come_from_the_console_log_and_the_run_duration(self):
        windows = identify_plant.run_windows(self.session)
        self.assertAlmostEqual(windows["run01_a_simulate_feeder"][0], 1000.0)
        self.assertAlmostEqual(windows["run01_a_simulate_feeder"][1], 1020.0)

    def test_report_measures_the_known_timings_and_skips_short_runs(self):
        text, rows, extra = identify_plant.build_report(self.session, min_picked=100)
        self.assertIn("Skipped: run02_b_simulate_feeder (3 picked)", text)
        self.assertEqual({r["run"] for r in rows}, {"run01"})
        goto = [r for r in rows if r["kind"] == "goto"]
        pick = [r for r in rows if r["kind"] == "pick"]
        self.assertEqual((len(goto), len(pick)), (2, 1))
        self.assertAlmostEqual(goto[0]["arrived"] - goto[0]["sent"], GOTO_S, delta=0.05)
        self.assertAlmostEqual(pick[0]["arrived"] - pick[0]["sent"], PICK_S, delta=0.05)
        [cycle] = extra["cycles"]
        self.assertAlmostEqual(cycle["wait_s"], WAIT_S, delta=0.06)
        self.assertAlmostEqual(cycle["setup_s"], SETUP_S, delta=0.06)

    def test_belt_step_and_pick_error_fit(self):
        _, _, extra = identify_plant.build_report(self.session, min_picked=100)
        [step] = extra["steps"]
        self.assertAlmostEqual(step["step"], 20.0)
        self.assertAlmostEqual(step["t50"], 0.08, delta=0.05)
        errors = extra["errors"]
        slope, offset = np.linalg.lstsq(np.c_[errors[:, 2], np.ones(len(errors))], errors[:, 0], rcond=None)[0]
        self.assertAlmostEqual(slope, 0.02, places=4)
        self.assertAlmostEqual(offset, 0.4, places=3)

    def test_main_writes_the_report_files(self):
        out = self.tmp / "out"
        self.assertEqual(identify_plant.main([str(self.session), "--out", str(out)]), 0)
        for name in ("identification.md", "legs.csv"):
            self.assertTrue((out / name).is_file(), name)

    def test_a_folder_that_is_not_a_session_is_refused(self):
        with self.assertRaises(SystemExit):
            identify_plant.main([str(self.tmp)])


if __name__ == "__main__":
    unittest.main(verbosity=2)
