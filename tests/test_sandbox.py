"""The offline bench: runs are deterministic per seed, every plugin runs, model files agree
with the operational config, and a model/plant mismatch costs what physics says it should.

Run:  python3 -m unittest tests.test_sandbox
"""

from __future__ import annotations

import unittest

from modules.scheduling.registry import PLANNERS, SPEED_LAWS
from sandbox.__main__ import check
from sandbox.sweep import expand, run_one

SHORT = {"sim.duration_s": 120.0}


class Runs(unittest.TestCase):
    def test_deterministic_per_seed(self):
        a = run_one({**SHORT, "sim.seed": 3})
        b = run_one({**SHORT, "sim.seed": 3})
        a.pop("wall_s")
        b.pop("wall_s")
        self.assertEqual(a, b)
        c = run_one({**SHORT, "sim.seed": 4})
        c.pop("wall_s")
        self.assertNotEqual(a, c)

    def test_every_planner_and_law_runs(self):
        for name in PLANNERS:
            result = run_one({**SHORT, "scheduling.planner": name})
            self.assertGreater(result["picked"], 0, name)
            self.assertEqual(result["spawned"], result["picked"] + result["missed"] + result["lost"]
                             + result["remaining"], name)
        for name in SPEED_LAWS:
            result = run_one({**SHORT, "speed.law": name, "speed.laws.predictive_rank.queue_depth_k": 0})
            self.assertGreater(result["picked"], 0, name)

    def test_point_mover_model(self):
        self.assertGreater(run_one({**SHORT, "arm.model": "point_mover"})["picked"], 0)

    def test_every_feeder_runs(self):
        for kind in ("periodic", "poisson", "regime", "bursty"):
            self.assertGreater(run_one({**SHORT, "feeder.kind": kind})["spawned"], 0, kind)

    def test_wrong_delay_estimate_costs_contact_error(self):
        exact = run_one({**SHORT, "speed.static_mm_s": 120})
        late = run_one({**SHORT, "speed.static_mm_s": 120,
                        "arm.plant": {"pick_gate.robot_movement_delay_s": 0.27}})
        self.assertLess(exact["median_abs_error_mm"], 2.0)
        # 100 ms more delay than modelled at 120 mm/s lands the cup ~12 mm behind the part.
        self.assertGreater(late["median_abs_error_mm"], 11.0)


class Specs(unittest.TestCase):
    def test_grid_and_cases(self):
        cases = expand({"base": {"a": 1}, "grid": {"x.y": [1, 2], "z": ["p", "q"]},
                        "cases": [{"name": "extra", "set": {"a": 2}}]})
        self.assertEqual(len(cases), 5)
        self.assertEqual(cases[-1], ("extra", {"a": 2}))
        self.assertEqual(cases[0][1], {"a": 1, "x.y": 1, "z": "p"})


class ModelFiles(unittest.TestCase):
    def test_model_cells_match_the_operational_config(self):
        self.assertEqual(check(), 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
