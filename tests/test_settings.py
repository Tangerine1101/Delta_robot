"""Pins the config loader: one default per key, unknown and moved keys refused, overrides.

Run:  python3 -m unittest tests.test_settings
"""

from __future__ import annotations

import copy
import unittest

from modules.config_io import read_config
from modules.settings import SettingsError, load_settings, settings_from_dict, with_overrides


class Loader(unittest.TestCase):
    def setUp(self) -> None:
        self.raw = read_config()

    def test_repository_config_loads(self):
        settings = load_settings()
        self.assertEqual(settings.plc.interpolar_points, 7)
        self.assertIn("TQFP", settings.object_types)
        self.assertAlmostEqual(settings.pick_gate.command_delay_s,
                               settings.pick_gate.robot_movement_delay_s + settings.pick_gate.ethernet_delay_s)

    def test_unknown_key_is_an_error(self):
        raw = copy.deepcopy(self.raw)
        raw["robot"]["heights"]["pickup_heigth"] = -300.0
        with self.assertRaises(SettingsError) as ctx:
            settings_from_dict(raw)
        self.assertIn("robot.heights.pickup_heigth", str(ctx.exception))

    def test_moved_key_names_its_new_place(self):
        raw = copy.deepcopy(self.raw)
        raw["scheduler"] = {"pickup_height": -300.0}
        with self.assertRaises(SettingsError) as ctx:
            settings_from_dict(raw)
        self.assertIn("robot", str(ctx.exception))

    def test_missing_required_key(self):
        raw = copy.deepcopy(self.raw)
        del raw["pick_gate"]["robot_movement_delay_s"]
        with self.assertRaises(SettingsError) as ctx:
            settings_from_dict(raw)
        self.assertIn("pick_gate.robot_movement_delay_s", str(ctx.exception))

    def test_type_errors(self):
        raw = copy.deepcopy(self.raw)
        raw["runtime"]["poll_interval_s"] = "fast"
        with self.assertRaises(SettingsError):
            settings_from_dict(raw)
        raw = copy.deepcopy(self.raw)
        raw["conveyor"]["workspace_window_uv"] = [188.0, 363.0, 0.0]
        with self.assertRaises(SettingsError):
            settings_from_dict(raw)

    def test_height_hierarchy(self):
        raw = copy.deepcopy(self.raw)
        raw["robot"]["heights"]["pre_pick"] = -305.0
        with self.assertRaises(SettingsError):
            settings_from_dict(raw)

    def test_overrides(self):
        settings = load_settings()
        changed = with_overrides(settings, {"speed.law": "predictive_rank", "robot.heights.place": -282.0,
                                            "speed.laws.predictive_rank.horizon_s": 30})
        self.assertEqual(changed.speed.law, "predictive_rank")
        self.assertEqual(changed.robot.heights.place, -282.0)
        self.assertEqual(changed.speed.laws["predictive_rank"]["horizon_s"], 30)
        self.assertEqual(settings.speed.law, load_settings().speed.law)     # the original is untouched
        with self.assertRaises(SettingsError):
            with_overrides(settings, {"robot.heights.nope": 1.0})

    def test_derived_views(self):
        settings = load_settings()
        self.assertEqual(settings.model_class("QFP"), "QFP")
        self.assertEqual(settings.bin_of("TQFP"), settings.object_types["TQFP"].bin)
        self.assertIsNone(settings.bin_of("nothing"))
        self.assertLessEqual(settings.speed_ceiling_mm_s(), settings.conveyor.hw_max_mm_s)


if __name__ == "__main__":
    unittest.main(verbosity=2)
