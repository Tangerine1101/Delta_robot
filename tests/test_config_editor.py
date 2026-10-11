"""The console's configuration editor: reading the file as flat keys, checked writes that keep
the comments, applying to the running console, and the refusals. No hardware: the supervisor
gets dummy PLC callables and a temporary copy of the config.

Run:  python3 -m unittest tests.test_config_editor
"""

from __future__ import annotations

import shutil
import tempfile
import unittest
from pathlib import Path

from modules.config_io import CONFIG_PATH, read_comments, read_config
from modules.settings import load_settings
from modules.ui import config_editor
from modules.ui.supervisor import RUNNING, Supervisor


class Flat(unittest.TestCase):
    def test_flatten_makes_dotted_keys_and_lists_are_leaves(self):
        flat = config_editor.flatten({"a": {"b": 1, "c": [1, 2]}, "d": {}, "e": "x"})
        self.assertEqual(flat, {"a.b": 1, "a.c": [1, 2], "d": {}, "e": "x"})

    def test_teach_points_are_not_editable_here(self):
        self.assertNotIn("interface.teach_points", config_editor.flatten({"interface": {"teach_points": {"p": [1, 2, 3]}, "port": 1}}))
        with self.assertRaises(ValueError):
            config_editor.set_path({}, "interface.teach_points.p", [1, 2, 3])

    def test_apply_changes_copies_and_creates_sections(self):
        raw = {"a": {"b": 1}}
        out = config_editor.apply_changes(raw, {"a.b": 2, "x.y.z": 3})
        self.assertEqual((raw, out), ({"a": {"b": 1}}, {"a": {"b": 2}, "x": {"y": {"z": 3}}}))
        with self.assertRaises(ValueError):
            config_editor.apply_changes({"a": 1}, {"a.b": 2})

    def test_comments_are_read_per_key(self):
        comments = read_comments(CONFIG_PATH)
        self.assertIn("scoring horizon", comments["speed.laws.predictive_rank.horizon_s"])

    def test_restart_keys(self):
        self.assertTrue(config_editor.needs_restart("plc.omron.ip"))
        self.assertFalse(config_editor.needs_restart("speed.law"))


class Console(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name) / "config.yaml"
        shutil.copy(CONFIG_PATH, self.path)
        self.events: list[tuple[str, dict]] = []
        self.sup = Supervisor(lambda p: None, lambda: None, load_settings(self.path),
                              emit=lambda kind, data: self.events.append((kind, data)), config_path=self.path)

    def tearDown(self):
        self.tmp.cleanup()

    def call(self, method, path, body=None):
        return self.sup.handle_api(method, path, body or {})

    def test_get_lists_values_comments_and_nothing_pending(self):
        status, body = self.call("GET", "/api/config")
        self.assertEqual(status, 200)
        self.assertEqual(body["values"]["speed.law"], self.sup.settings.speed.law)
        self.assertEqual((body["pending"], body["restart_needed"], body["stale"], body["file_error"]), ([], [], False, None))
        self.assertIn("speed.band.min_mm_s", body["comments"])
        self.assertNotIn("interface.teach_points", body["values"])

    def test_save_and_apply_writes_the_file_keeps_comments_and_updates_the_console(self):
        status, body = self.call("POST", "/api/config/save", {
            "changes": {"speed.laws.predictive_rank.horizon_s": 20.0, "runtime.duration_s": 120}})
        self.assertEqual(status, 200, body)
        self.assertTrue(body["applied"])
        self.assertEqual(self.sup.settings.speed.laws["predictive_rank"]["horizon_s"], 20.0)
        self.assertEqual(self.sup.settings.runtime.duration_s, 120.0)
        text = self.path.read_text()
        self.assertIn("scoring horizon W (s)", text)                       # comments survive
        self.assertTrue(self.path.with_name("config.yaml.bak").is_file())
        self.assertEqual(read_config(self.path)["runtime"]["duration_s"], 120)
        self.assertEqual(self.call("GET", "/api/config")[1]["pending"], [])
        self.assertIn("sup", [kind for kind, _ in self.events])

    def test_save_without_apply_leaves_the_running_settings_pending(self):
        status, body = self.call("POST", "/api/config/save", {"changes": {"speed.static_mm_s": 33.0}, "apply": False})
        self.assertEqual(status, 200, body)
        self.assertNotEqual(self.sup.settings.speed.static_mm_s, 33.0)
        snap = self.call("GET", "/api/config")[1]
        self.assertEqual(snap["pending"], ["speed.static_mm_s"])
        status, body = self.call("POST", "/api/config/reload")
        self.assertEqual((status, body["changed"]), (200, ["speed.static_mm_s"]))
        self.assertEqual(self.sup.settings.speed.static_mm_s, 33.0)

    def test_a_bad_value_is_refused_and_the_file_is_untouched(self):
        before = self.path.read_text()
        for changes in ({"speed.band.min_mm_s": "fast"}, {"speed.nonsense": 1}, {"scheduling.planner": "no_such_planner"}):
            status, body = self.call("POST", "/api/config/save", {"changes": changes})
            self.assertEqual(status, 400, (changes, body))
            self.assertFalse(body["ok"])
        self.assertEqual(self.path.read_text(), before)

    def test_a_run_blocks_apply_and_reload_but_not_a_plain_save(self):
        self.sup.mode = RUNNING
        self.assertEqual(self.call("POST", "/api/config/save", {"changes": {"speed.static_mm_s": 31.0}})[0], 409)
        self.assertEqual(self.call("POST", "/api/config/reload")[0], 409)
        self.assertEqual(self.call("POST", "/api/config/save", {"changes": {"speed.static_mm_s": 31.0}, "apply": False})[0], 200)

    def test_edit_on_disk_is_seen_as_stale_and_a_broken_file_is_survived(self):
        self.assertFalse(self.sup.state()["config_stale"])
        self.path.write_text(self.path.read_text().replace("seed: 1", "seed: 7", 1))
        self.assertTrue(self.sup.state()["config_stale"])
        self.path.write_text("speed: [not a mapping")
        snap = self.call("GET", "/api/config")[1]
        self.assertIsNotNone(snap["file_error"])
        self.assertEqual(self.call("POST", "/api/config/reload")[0], 400)
        self.assertEqual(self.sup.settings.speed.law, load_settings().speed.law)   # still running the old one

    def test_command_line_overrides_stay_on_top_of_a_reload(self):
        sup = Supervisor(lambda p: None, lambda: None, load_settings(self.path),
                         emit=lambda *_: None, config_path=self.path, overrides={"speed.static_mm_s": 77.0})
        sup.settings = load_settings(self.path)          # as main.py does: file + --set
        status, body = sup.handle_api("POST", "/api/config/save", {"changes": {"speed.static_mm_s": 12.0}})
        self.assertEqual(status, 200, body)
        self.assertEqual(sup.settings.speed.static_mm_s, 77.0)
        self.assertEqual(body["masked"], ["speed.static_mm_s"])

    def test_teach_point_saves_do_not_make_the_file_look_changed(self):
        self.call("POST", "/api/teach/save", {"name": "t", "x": 1, "y": 2, "z": -250})
        self.assertFalse(self.sup.state()["config_stale"])
        self.assertIn("t", self.call("GET", "/api/state")[1]["teach_points"])


if __name__ == "__main__":
    unittest.main()
