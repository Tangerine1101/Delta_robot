"""Realtime robot pose from the Omron UDP stream.

The Omron program (`Program_UDP`) pushes the end-effector position as a 12-byte datagram —
3 x REAL (IEEE-754 float32, little-endian): X, Y, Z in mm — to the PC every ~12 ms. Reading
the pose this way replaces polling `pos_EE` over EtherNet/IP: it is fresher, cheaper and
timestamped at the moment the packet reached the network stack.

Timestamps: the kernel receive time (SO_TIMESTAMPNS, CLOCK_REALTIME) is converted onto
`time.monotonic()`, the clock the scheduler plans with, so a GIL stall in this thread does
not shift the pose in time. Without kernel timestamps the receive call's return time is used.

The PLC's destination IP is set in the PLC program; the PC only has to own that IP and let
UDP to `port` through its firewall (doc/basis-programming.md §3.5).
"""

from __future__ import annotations

import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import Callable

PACKET = struct.Struct("<3f")
_TIMESPEC = struct.Struct("@qq")
_SO_TIMESTAMPNS = getattr(socket, "SO_TIMESTAMPNS", 35)
_CLOCK_OFFSET_REFRESH_S = 1.0

Position3D = tuple[float, float, float]


@dataclass(frozen=True)
class PoseSample:
    t_mono: float           # receive time on time.monotonic()
    t_wall: float           # receive time on time.time()
    position: Position3D    # end effector (mm)


class PoseStream:
    """Background UDP receiver keeping the latest pose. Thread-safe."""

    def __init__(
        self,
        port: int,
        *,
        source_ip: str | None = None,
        bind: str = "0.0.0.0",
        stale_s: float = 0.1,
        on_sample: Callable[[PoseSample], None] | None = None,
    ) -> None:
        self.port = int(port)
        self.source_ip = source_ip
        self.bind = bind
        self.stale_s = float(stale_s)
        self.on_sample = on_sample
        self._lock = threading.Lock()
        self._latest: PoseSample | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._sock: socket.socket | None = None
        self._kernel_timestamps = False
        self._wall_minus_mono = time.time() - time.monotonic()
        self._offset_refreshed = time.monotonic()
        self.received = 0
        self.rejected = 0

    def start(self) -> bool:
        """Bind and start receiving. Returns False (and stays inert) if the port is taken."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        try:
            sock.setsockopt(socket.SOL_SOCKET, _SO_TIMESTAMPNS, 1)
            self._kernel_timestamps = True
        except OSError:
            self._kernel_timestamps = False
        try:
            sock.bind((self.bind, self.port))
        except OSError as exc:
            print(f"[WARN] pose stream: cannot bind UDP {self.bind}:{self.port} ({exc}); "
                  "robot pose falls back to EtherNet/IP polling")
            sock.close()
            return False
        sock.settimeout(0.5)
        self._sock = sock
        self._thread = threading.Thread(target=self._loop, name="pose-stream", daemon=True)
        self._thread.start()
        print(f"[INFO] pose stream listening on UDP {self.bind}:{self.port}"
              + (f" (source {self.source_ip})" if self.source_ip else ""))
        return True

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._sock is not None:
            self._sock.close()

    def latest(self) -> PoseSample | None:
        with self._lock:
            return self._latest

    def fresh(self, now: float | None = None) -> PoseSample | None:
        """The latest sample if it is younger than `stale_s`, else None."""
        sample = self.latest()
        if sample is None:
            return None
        now = time.monotonic() if now is None else now
        return sample if now - sample.t_mono <= self.stale_s else None

    def _receive_time(self, ancdata: list) -> tuple[float, float]:
        mono_now = time.monotonic()
        if mono_now - self._offset_refreshed >= _CLOCK_OFFSET_REFRESH_S:
            self._wall_minus_mono = time.time() - time.monotonic()
            self._offset_refreshed = mono_now
        for level, kind, data in ancdata:
            if level == socket.SOL_SOCKET and kind == _SO_TIMESTAMPNS and len(data) >= _TIMESPEC.size:
                sec, nsec = _TIMESPEC.unpack_from(data)
                wall = sec + nsec * 1e-9
                return min(wall - self._wall_minus_mono, mono_now), wall
        return mono_now, mono_now + self._wall_minus_mono

    def _loop(self) -> None:
        assert self._sock is not None
        ancbufsize = socket.CMSG_SPACE(_TIMESPEC.size) if self._kernel_timestamps else 0
        while not self._stop.is_set():
            try:
                data, ancdata, _flags, addr = self._sock.recvmsg(64, ancbufsize)
            except socket.timeout:
                continue
            except OSError:
                if self._stop.is_set():
                    return
                continue
            if len(data) != PACKET.size or (self.source_ip and addr[0] != self.source_ip):
                self.rejected += 1
                continue
            t_mono, t_wall = self._receive_time(ancdata)
            sample = PoseSample(t_mono, t_wall, tuple(float(v) for v in PACKET.unpack(data)))
            with self._lock:
                self._latest = sample
            self.received += 1
            if self.on_sample is not None:
                self.on_sample(sample)
