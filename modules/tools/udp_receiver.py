"""Receive X/Y/Z telemetry sent by Program_UDP on the Omron NX1P2.

Packet format: 12 bytes = 3 x REAL (IEEE-754 float32, little-endian) -> X, Y, Z in mm.
Prints a stats line every second (rate, period, jitter, gaps) and can log to CSV.

    python3 -m modules.tools.udp_receiver                        # listen on 0.0.0.0:9001
    python3 -m modules.tools.udp_receiver --csv log.csv --duration 60
"""

from __future__ import annotations

import argparse
import csv
import socket
import statistics
import struct
import time
from pathlib import Path

PACKET = struct.Struct("<3f")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="UDP receiver for PLC X/Y/Z telemetry")
    parser.add_argument("--bind", default="0.0.0.0", help="local IP to listen on")
    parser.add_argument("--port", type=int, default=9001, help="local UDP port (PC_Port on PLC)")
    parser.add_argument("--plc-ip", default=None, help="only accept packets from this IP")
    parser.add_argument("--expect-ms", type=float, default=8.0,
                        help="expected send period; intervals > 1.75x this count as gaps")
    parser.add_argument("--csv", type=Path, default=None, help="write every packet to this CSV")
    parser.add_argument("--duration", type=float, default=0.0, help="stop after N seconds (0 = forever)")
    parser.add_argument("--quiet", action="store_true", help="do not print last X/Y/Z")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    sock.bind((args.bind, args.port))
    sock.settimeout(1.0)
    print(f"[INFO] Listening UDP {args.bind}:{args.port} (Ctrl+C to stop)")

    csv_file = args.csv.open("w", newline="", encoding="utf-8") if args.csv else None
    writer = csv.writer(csv_file) if csv_file else None
    if writer:
        writer.writerow(["t_s", "dt_ms", "x_mm", "y_mm", "z_mm"])

    # One lost packet doubles the interval, so anything above 1.75x is a gap
    gap_limit_s = 1.75 * args.expect_ms / 1000.0
    t_start = time.perf_counter()
    t_prev: float | None = None
    t_first = t_start
    t_report = t_start
    intervals: list[float] = []
    total = bad_size = gaps = window_packets = 0
    last_xyz = (0.0, 0.0, 0.0)
    source = None

    try:
        while True:
            now = time.perf_counter()
            if args.duration and now - t_start >= args.duration:
                break

            try:
                data, addr = sock.recvfrom(1024)
            except socket.timeout:
                if total:
                    print("[WARN] No packet in the last 1 s")
                continue

            t_rx = time.perf_counter()
            if args.plc_ip and addr[0] != args.plc_ip:
                continue
            if len(data) != PACKET.size:
                bad_size += 1
                print(f"[WARN] {addr[0]}:{addr[1]} sent {len(data)} bytes (expected {PACKET.size})")
                continue

            if source != addr:
                source = addr
                print(f"[INFO] Receiving from {addr[0]}:{addr[1]}")
            if total == 0:
                t_report = t_first = t_rx

            last_xyz = PACKET.unpack(data)
            total += 1
            window_packets += 1
            dt = 0.0 if t_prev is None else t_rx - t_prev
            if t_prev is not None:
                intervals.append(dt)
                if dt > gap_limit_s:
                    gaps += 1
            t_prev = t_rx

            if writer:
                writer.writerow([f"{t_rx - t_start:.6f}", f"{dt * 1000:.3f}", *(f"{v:.4f}" for v in last_xyz)])

            if t_rx - t_report >= 1.0:
                elapsed = t_rx - t_report
                line = f"rate {window_packets / elapsed:6.1f} pkt/s"
                if intervals:
                    ms = [v * 1000 for v in intervals]
                    jitter = statistics.pstdev(ms) if len(ms) > 1 else 0.0
                    line += (f" | period avg {statistics.fmean(ms):5.2f} min {min(ms):5.2f}"
                             f" max {max(ms):6.2f} jitter {jitter:5.2f} ms")
                line += f" | gaps {gaps} | total {total}"
                if not args.quiet:
                    line += " | XYZ " + " ".join(f"{v:8.2f}" for v in last_xyz)
                print(line)
                intervals.clear()
                window_packets = 0
                t_report = t_rx
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        if csv_file:
            csv_file.close()

    active_s = (t_prev - t_first) if total > 1 else 0.0
    rate = (total - 1) / active_s if active_s else 0.0
    print(f"\n[SUMMARY] {total} packets over {active_s:.1f} s "
          f"(avg {rate:.1f} pkt/s), gaps {gaps}, wrong-size {bad_size}")


if __name__ == "__main__":
    main()
