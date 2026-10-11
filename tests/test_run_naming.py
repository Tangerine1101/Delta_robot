"""Names of run logs: `main.py --log-name` and the console's run name.

Run:  python3 -m unittest tests.test_run_naming
"""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from unittest import mock

from modules import runlog
from modules.runlog import RunLog, safe_name


class SafeName(unittest.TestCase):
    def test_keeps_letters_digits_and_a_few_marks(self):
        self.assertEqual(safe_name("W15_dp-seed110.v2"), "W15_dp-seed110.v2")

    def test_everything_else_becomes_a_dash_and_the_ends_are_trimmed(self):
        self.assertEqual(safe_name("  W15 dp / seed 2 "), "W15-dp-seed-2")
        self.assertEqual(safe_name("../../etc"), "etc")
        self.assertEqual(safe_name("Tên chạy đầu"), "Ten-chay-dau")

    def test_nothing_usable_is_empty_and_none_is_empty(self):
        self.assertEqual((safe_name("///"), safe_name(""), safe_name(None)), ("", "", ""))

    def test_is_cut_to_the_limit(self):
        self.assertEqual(len(safe_name("a" * 200)), 60)


class LogDirectory(unittest.TestCase):
    def open(self, name):
        with tempfile.TemporaryDirectory() as tmp, mock.patch.object(runlog, "tee_console"):
            run = RunLog.open(tmp, "console", {}, name=name)
            try:
                made = run.dir.name
                meta = (run.dir / "meta.json").read_text()
            finally:
                run.close()
        return made, meta

    def test_the_name_goes_between_the_time_and_the_mode(self):
        made, meta = self.open("W15 dp")
        self.assertRegex(made, r"^\d{8}-\d{6}_W15-dp_console$")
        self.assertIn('"log_name": "W15-dp"', meta)

    def test_without_a_name_the_directory_is_as_before(self):
        made, meta = self.open(None)
        self.assertRegex(made, r"^\d{8}-\d{6}_console$")
        self.assertIn('"log_name": null', meta)


if __name__ == "__main__":
    unittest.main()
