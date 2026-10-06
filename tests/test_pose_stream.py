"""Pins ``modules.comm.pose_stream`` and ``modules.runlog``.

Run:  python3 -m unittest tests.test_pose_stream
"""

from __future__ import annotations

import csv
import json
import shutil
import socket
import tempfile
import time
import unittest
from pathlib import Path

from modules.comm.pose_stream import PACKET, PoseStream
from modules.runlog import RunLog


def _free_udp_port() -> int:
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    return port


class Stream(unittest.TestCase):
    def setUp(self):
        self.port = _free_udp_port()
        self.samples = []
        self.stream = PoseStream(self.port, source_ip="127.0.0.1", bind="127.0.0.1", stale_s=0.05,
                                 on_sample=self.samples.append)
        self.assertTrue(self.stream.start())
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def tearDown(self):
        self.stream.stop()
        self.tx.close()

    def _send(self, payload: bytes) -> None:
        self.tx.sendto(payload, ("127.0.0.1", self.port))

    def _wait(self, predicate, timeout=1.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate():
                return True
            time.sleep(0.005)
        return False

    def test_decodes_and_timestamps(self):
        before = time.monotonic()
        self._send(PACKET.pack(1.5, -2.25, -230.0))
        self.assertTrue(self._wait(lambda: self.stream.received == 1))
        sample = self.stream.latest()
        self.assertEqual(sample.position, (1.5, -2.25, -230.0))
        self.assertGreaterEqual(sample.t_mono, before - 0.01)
        self.assertLessEqual(sample.t_mono, time.monotonic())
        self.assertAlmostEqual(sample.t_wall - sample.t_mono, time.time() - time.monotonic(), delta=0.05)
        self.assertEqual(len(self.samples), 1)

    def test_wrong_size_rejected_and_staleness(self):
        self._send(b"\x00" * 8)
        self.assertTrue(self._wait(lambda: self.stream.rejected == 1))
        self.assertIsNone(self.stream.latest())
        self._send(PACKET.pack(0.0, 0.0, -290.0))
        self.assertTrue(self._wait(lambda: self.stream.fresh() is not None))
        time.sleep(0.08)
        self.assertIsNone(self.stream.fresh())

    def test_busy_port_reports_failure(self):
        other = PoseStream(self.port, bind="127.0.0.1")
        self.assertFalse(other.start())


class Log(unittest.TestCase):
    def setUp(self):
        self.dir = Path(tempfile.mkdtemp())
        self.log = RunLog(self.dir)

    def tearDown(self):
        self.log.close()
        shutil.rmtree(self.dir)

    def test_records(self):
        t0 = time.monotonic()
        package = {"commandID": 3, "argument_number": 2, "argument_x": [1.0, 2.0, 0.0, 0.0],
                   "argument_y": [0.0] * 4, "argument_z": [-290.0] * 4,
                   "argument_e": [0] * 4, "argument_time": [0.0] * 4}
        self.log.command(package, t0, t0 + 0.01, ok=True)
        self.log.status(t0, t0 + 0.002, {"pos_EE": [1.0, 2.0, -290.0], "pose_source": "udp",
                                         "conveyor_position": 12.5, "task_state": 0}, omron_read=False)
        self.log.pose(t0, time.time(), (1.0, 2.0, -290.0))
        self.log.close()

        command = json.loads((self.dir / "commands.jsonl").read_text().splitlines()[0])
        self.assertEqual(command["package"]["argument_x"], [1.0, 2.0])
        self.assertAlmostEqual(command["rtt_s"], 0.01, places=6)
        status = list(csv.DictReader(open(self.dir / "status.csv")))[0]
        self.assertEqual((status["pose_source"], status["conveyor_position"], status["omron_read"]),
                         ("udp", "12.5", "0"))
        pose = list(csv.DictReader(open(self.dir / "pose.csv")))[0]
        self.assertEqual(pose["z"], "-290.0000")


if __name__ == "__main__":
    unittest.main(verbosity=2)
