"""Per-run debug logs under `log/<YYYYmmdd-HHMMSS>_<mode>/`.

Every record carries both clocks so commands, status and motion can be lined up:

* `t_mono` — `time.monotonic()`, the clock the scheduler plans with (`[PLAN]` times,
  gate times) and the pose stream is stamped on;
* `t_wall` — `time.time()`, for matching against the PLC trace or a video.

Files (doc/basis-programming.md §2.3):

| File | Content |
|---|---|
| `meta.json` | command line, start time on both clocks, git revision, pose-stream settings |
| `config.yaml` | copy of the config the run started with |
| `console.log` | everything printed by the main process, each line prefixed `t_wall t_mono` |
| `worker.log` | the same for the PLC worker process |
| `commands.jsonl` | every command sent to a PLC: send/done times, round trip, packet, result |
| `status.csv` | every status poll: belt position/speed, rotation, Omron handshake, pose source |
| `pose.csv` | every UDP pose sample |
"""

from __future__ import annotations

import csv
import io
import json
import os
import shutil
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, TextIO

from modules.config_io import CONFIG_PATH

REPO_ROOT = Path(__file__).resolve().parent.parent
_FLUSH_PERIOD_S = 1.0
_ARRAY_KEYS = ("argument_x", "argument_y", "argument_z", "argument_e", "argument_time")

STATUS_FIELDS = (
    "t_mono", "t_wall", "rtt_s", "omron_read", "pose_source", "pose_age_s",
    "x", "y", "z", "conveyor_position", "speed_current", "rotate_current",
    "task_doing", "task_state", "bit_doing", "end_effector", "siemens_task_doing",
    "siemens_task_state", "conveyor_position_raw", "speed_current_raw", "conveyor_state",
)


def _stamp() -> str:
    return f"{time.time():.6f} {time.monotonic():.6f}"


class _TimestampTee(io.TextIOBase):
    """Writes through to the original stream and to a file, prefixing each file line with
    both clocks. Partial lines are buffered until their newline arrives."""

    def __init__(self, original: TextIO, sink: TextIO, lock: threading.Lock) -> None:
        self._original = original
        self._sink = sink
        self._lock = lock
        self._pending = ""

    def write(self, text: str) -> int:
        self._original.write(text)
        with self._lock:
            self._pending += text
            while "\n" in self._pending:
                line, self._pending = self._pending.split("\n", 1)
                self._sink.write(f"{_stamp()} {line}\n")
        return len(text)

    def flush(self) -> None:
        self._original.flush()
        with self._lock:
            self._sink.flush()

    def isatty(self) -> bool:
        return self._original.isatty()

    @property
    def encoding(self) -> str:
        return getattr(self._original, "encoding", "utf-8")


def tee_console(path: Path) -> None:
    """Mirror this process's stdout/stderr into `path` with timestamps."""
    sink = open(path, "a", encoding="utf-8", buffering=1)
    lock = threading.Lock()
    sys.stdout = _TimestampTee(sys.stdout, sink, lock)
    sys.stderr = _TimestampTee(sys.stderr, sink, lock)


def _git_revision() -> str | None:
    try:
        out = subprocess.run(["git", "-C", str(REPO_ROOT), "rev-parse", "--short", "HEAD"],
                             capture_output=True, text=True, timeout=2.0)
        dirty = subprocess.run(["git", "-C", str(REPO_ROOT), "status", "--porcelain"],
                               capture_output=True, text=True, timeout=2.0)
        return out.stdout.strip() + ("+dirty" if dirty.stdout.strip() else "")
    except (OSError, subprocess.SubprocessError):
        return None


def _trim_packet(package: dict[str, Any]) -> dict[str, Any]:
    """Drop the padding of the waypoint arrays so a log line shows only the used points."""
    trimmed = dict(package)
    n = trimmed.get("argument_number")
    if isinstance(n, int) and n >= 0:
        for key in _ARRAY_KEYS:
            if isinstance(trimmed.get(key), list):
                trimmed[key] = trimmed[key][:max(n, 1)]
    return trimmed


class RunLog:
    """Thread-safe writers for one run directory. Create with `RunLog.open`."""

    def __init__(self, directory: Path) -> None:
        self.dir = directory
        self._lock = threading.Lock()
        self._commands = open(directory / "commands.jsonl", "a", encoding="utf-8")
        self._status_file = open(directory / "status.csv", "a", newline="", encoding="utf-8")
        self._status = csv.writer(self._status_file)
        self._status.writerow(STATUS_FIELDS)
        self._pose_file = open(directory / "pose.csv", "a", newline="", encoding="utf-8")
        self._pose = csv.writer(self._pose_file)
        self._pose.writerow(("t_mono", "t_wall", "x", "y", "z"))
        self._last_flush = time.monotonic()
        self._closed = False

    @classmethod
    def open(cls, root: str | Path, mode: str, meta: dict[str, Any]) -> "RunLog":
        root = Path(root)
        if not root.is_absolute():
            root = REPO_ROOT / root
        name = f"{datetime.now().strftime('%Y%m%d-%H%M%S')}_{mode}"
        directory = root / name
        suffix = 1
        while directory.exists():
            suffix += 1
            directory = root / f"{name}-{suffix}"
        directory.mkdir(parents=True)
        run = cls(directory)
        info = {
            "mode": mode,
            "argv": sys.argv,
            "start_wall": time.time(),
            "start_mono": time.monotonic(),
            "start_iso": datetime.now().isoformat(timespec="milliseconds"),
            "git": _git_revision(),
            "pid": os.getpid(),
            **meta,
        }
        (directory / "meta.json").write_text(json.dumps(info, indent=2, ensure_ascii=False))
        if CONFIG_PATH.exists():
            shutil.copy(CONFIG_PATH, directory / "config.yaml")
        tee_console(directory / "console.log")
        print(f"[INFO] run log: {directory}")
        return run

    def _maybe_flush(self) -> None:
        now = time.monotonic()
        if now - self._last_flush >= _FLUSH_PERIOD_S:
            self._last_flush = now
            self._commands.flush()
            self._status_file.flush()
            self._pose_file.flush()

    def command(self, package: dict[str, Any], t_send: float, t_done: float, *,
                ok: bool, error: str | None = None, reply: Any = None) -> None:
        record = {
            "t_wall": round(time.time() - (time.monotonic() - t_send), 6),
            "t_mono_send": round(t_send, 6),
            "t_mono_done": round(t_done, 6),
            "rtt_s": round(t_done - t_send, 6),
            "commandID": package.get("commandID", package.get("CommandID")),
            "ok": ok,
            "error": error,
            "package": _trim_packet(package),
        }
        if isinstance(reply, dict):
            record["reply"] = {k: v for k, v in reply.items() if k not in _ARRAY_KEYS}
        line = json.dumps(record, ensure_ascii=True, default=str)
        with self._lock:
            if self._closed:
                return
            self._commands.write(line + "\n")
            self._commands.flush()   # commands are rare and the most valuable record

    def status(self, t_send: float, t_done: float, status: dict[str, Any] | None, *,
               omron_read: bool) -> None:
        s = status or {}
        pose = s.get("pos_EE")
        xyz = (pose[:3] if isinstance(pose, (list, tuple)) and len(pose) >= 3 else (None, None, None))
        row = {
            "t_mono": f"{t_done:.6f}",
            "t_wall": f"{time.time() - (time.monotonic() - t_done):.6f}",
            "rtt_s": f"{t_done - t_send:.6f}",
            "omron_read": int(omron_read),
            "pose_source": s.get("pose_source"),
            "pose_age_s": s.get("pose_age_s"),
            "x": xyz[0], "y": xyz[1], "z": xyz[2],
        }
        for key in STATUS_FIELDS[9:]:
            row[key] = s.get(key)
        with self._lock:
            if self._closed:
                return
            self._status.writerow([row[k] for k in STATUS_FIELDS])
            self._maybe_flush()

    def pose(self, t_mono: float, t_wall: float, xyz: tuple[float, float, float]) -> None:
        with self._lock:
            if self._closed:
                return
            self._pose.writerow((f"{t_mono:.6f}", f"{t_wall:.6f}",
                                 f"{xyz[0]:.4f}", f"{xyz[1]:.4f}", f"{xyz[2]:.4f}"))
            self._maybe_flush()

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            for handle in (self._commands, self._status_file, self._pose_file):
                handle.close()
        sys.stdout.flush()
