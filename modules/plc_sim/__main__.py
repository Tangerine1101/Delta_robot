"""Standalone fake PLC: runs the simulator as a JSON-lines TCP server.

The simulator impersonates both PLCs (scan-accurate Omron ``Matching_Code_10`` +
Siemens belt/rotation) on one port; point the CLI or the scheduler at it with
``--ip 127.0.0.1 --port <port>``. ``main.py --cli --dummy`` and
``main.py --scheduler --sim`` start the same simulator in-process instead.

    python3 -m modules.plc_sim [--port 44818] [--feed 2.5] [--duration 60]
"""
from __future__ import annotations

import argparse
import time

from modules.plc_sim.sim import PLCSim
from modules.settings import load_settings


def main() -> None:
    settings = load_settings()
    parser = argparse.ArgumentParser(description="Standalone PLC simulator (modules.plc_sim)")
    parser.add_argument("--host", default="127.0.0.1", help="Host/IP to bind")
    parser.add_argument("--port", type=int, default=settings.plc.omron.port,
                        help="TCP port to bind (default: plc.omron.port)")
    parser.add_argument("--feed", type=float, default=None,
                        help="Seconds between boards fed onto the belt (default: no feeder)")
    parser.add_argument("--servo-tau", type=float, default=0.0,
                        help="First-order servo lag per axis in seconds (0 = ideal)")
    parser.add_argument("--tag-latency", type=float, default=0.002,
                        help="Simulated round trip per Omron tag request in seconds")
    parser.add_argument("--duration", type=float, default=None,
                        help="Stop after this many seconds (default: until Ctrl-C)")
    args = parser.parse_args()

    sim = PLCSim(settings=settings, servo_tau_s=args.servo_tau, tag_latency_s=args.tag_latency,
                 feed_interval_s=args.feed)
    sim.start()
    host, port = sim.serve(args.host, args.port)
    print(f"[SIM] PLC simulator on {host}:{port}  (Ctrl-C to stop)", flush=True)
    started = time.monotonic()
    try:
        while args.duration is None or time.monotonic() - started < args.duration:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print()
    finally:
        print("[SIM]", sim.summary(), flush=True)
        sim.shutdown()


if __name__ == "__main__":
    main()
