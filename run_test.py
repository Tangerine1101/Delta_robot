#!/usr/bin/env python3
import argparse
import sys
import subprocess
import threading
import time

def stream_output(process, prefix):
    last_print_time = 0.0
    for line in iter(process.stdout.readline, ''):
        cleaned = line.strip()
        if cleaned:
            if "PLC status:" in cleaned:
                now = time.monotonic()
                if now - last_print_time >= 0.5:
                    print(f"{prefix} {cleaned}")
                    last_print_time = now
            else:
                print(f"{prefix} {cleaned}")
    process.stdout.close()

def main():
    parser = argparse.ArgumentParser(description="Run Delta Robot simulation integration test.")
    parser.add_argument(
        "--scenario",
        default="test_throughput",
        choices=["test_throughput", "test_accuracy"],
        help="Test scenario to run (default: test_throughput)",
    )
    parser.add_argument(
        "--duration",
        type=int,
        default=10,
        help="Duration of the test in seconds (default: 10)",
    )
    args = parser.parse_args()

    port = 1502
    # Start the PLC fake module with duration slightly longer to allow clean shutdown of client first
    plc_cmd = [
        sys.executable,
        "-m",
        "modules.test_module",
        "--port",
        str(port),
        "--duration",
        str(args.duration + 5),
    ]

    # Start main.py scheduler simulation
    main_cmd = [
        sys.executable,
        "main.py",
        "--scheduler",
        "--ip",
        "127.0.0.1",
        "--port",
        str(port),
        "--scenario",
        args.scenario,
        "--duration",
        str(args.duration),
    ]

    print(f"[*] Starting simulated PLC on port {port}...")
    plc_proc = subprocess.Popen(
        plc_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    # Let the PLC start and bind to the port
    time.sleep(1.0)

    print(f"[*] Starting scheduler for scenario '{args.scenario}' (duration: {args.duration}s)...")
    main_proc = subprocess.Popen(
        main_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    # Stream outputs in separate threads
    t_plc = threading.Thread(target=stream_output, args=(plc_proc, "\033[94m[PLC]\033[0m"), daemon=True)
    t_main = threading.Thread(target=stream_output, args=(main_proc, "\033[92m[MAIN]\033[0m"), daemon=True)

    t_plc.start()
    t_main.start()

    try:
        # Wait for the main scheduler to finish
        main_proc.wait()
    except KeyboardInterrupt:
        print("\n[*] Interrupted by user. Cleaning up...")
    finally:
        # Clean up processes
        if main_proc.poll() is None:
            main_proc.terminate()
            main_proc.wait()
        if plc_proc.poll() is None:
            plc_proc.terminate()
            plc_proc.wait()
        print("[*] Stopped all processes.")

if __name__ == "__main__":
    main()
