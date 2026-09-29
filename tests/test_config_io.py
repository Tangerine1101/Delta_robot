"""Pins ``modules.config_io``: a calibration-tool write keeps the hand-written comments.

Run:  python3 -m unittest tests.test_config_io
"""

from __future__ import annotations

import shutil
import tempfile
import unittest
from pathlib import Path

from modules.config_io import CONFIG_PATH, read_config, write_config


class RoundTrip(unittest.TestCase):
    def setUp(self):
        self.dir = Path(tempfile.mkdtemp())
        self.path = self.dir / "config.yaml"
        shutil.copy(CONFIG_PATH, self.path)

    def tearDown(self):
        shutil.rmtree(self.dir)

    def test_unchanged_write_is_byte_identical(self):
        write_config(read_config(self.path), self.path)
        self.assertEqual(self.path.read_text(), CONFIG_PATH.read_text())

    def test_value_change_keeps_comments_and_types(self):
        data = read_config(self.path)
        data["vision"]["pixels_per_mm"] = 6.5
        data["vision"]["roi"]["polygon"] = [[1, 2], [3, 4], [5, 6]]
        data["plc_sim"]["new_key"] = 1.5
        write_config(data, self.path)

        self.assertEqual(read_config(self.path), data)
        text = self.path.read_text()
        self.assertIn("# image scale (written by camera_calibrate.py)", text)
        self.assertIn("polygon: [[1, 2], [3, 4], [5, 6]]", text)
        self.assertIn('device: "0"', text)
        self.assertIsInstance(read_config(self.path)["vision"]["device"], str)
        # The appended key stays inside `plc_sim`, above the next section's header.
        self.assertLess(text.index("new_key"), text.index("# Operator console"))

    def test_removed_key_is_deleted(self):
        data = read_config(self.path)
        del data["interface"]["mjpeg_fps"]
        write_config(data, self.path)
        self.assertNotIn("mjpeg_fps", read_config(self.path)["interface"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
