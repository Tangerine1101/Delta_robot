#!/usr/bin/env python3
import argparse
import sys
import subprocess
import threading
import time
import os
import re
import math
from pathlib import Path

# Global data collectors for plotting
data_lock = threading.Lock()
robot_positions = []  # tuples of (t_rel, x, y, z)
conveyor_speeds = []   # tuples of (t_rel, vx, vy)
start_time = 0.0

def stream_output(process, prefix):
    is_plc = "[PLC]" in prefix
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
            
            # Parse trajectory / speed data
            t_rel = time.monotonic() - start_time
            if is_plc:
                match = re.search(r"pos_EE=\[([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+)\]", cleaned)
                if match:
                    x, y, z = map(float, match.groups())
                    with data_lock:
                        robot_positions.append((t_rel, x, y, z))
            else:
                match = re.search(r"\[SPEED\] vx=([-\d\.]+) vy=([-\d\.]+)", cleaned)
                if match:
                    vx, vy = map(float, match.groups())
                    with data_lock:
                        conveyor_speeds.append((t_rel, vx, vy))
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

    global start_time
    start_time = time.monotonic()

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

    # Plot generation at the end of the test
    generate_plots()

def generate_plots():
    try:
        import matplotlib
        matplotlib.use('Agg')  # Headless backend
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n[WARN] matplotlib is not installed. Skipping plot generation.")
        print("[*] Install it with: pip install matplotlib")
        return

    with data_lock:
        positions = list(robot_positions)
        speeds = list(conveyor_speeds)

    if not positions:
        print("\n[WARN] No robot positions captured. Skipping plot generation.")
        return

    # Sort positions by relative time
    positions.sort(key=lambda x: x[0])
    speeds.sort(key=lambda x: x[0])

    t_p = [p[0] for p in positions]
    x_p = [p[1] for p in positions]
    y_p = [p[2] for p in positions]
    z_p = [p[3] for p in positions]

    # Calculate robot velocity
    robot_vel = []
    t_v = []
    for i in range(1, len(positions)):
        dt = t_p[i] - t_p[i-1]
        if dt > 0.005:  # avoid division by zero or noisy small dt
            dx = x_p[i] - x_p[i-1]
            dy = y_p[i] - y_p[i-1]
            dz = z_p[i] - z_p[i-1]
            v = math.hypot(math.hypot(dx, dy), dz) / dt
            robot_vel.append(v)
            t_v.append(t_p[i])

    # Plotting
    try:
        fig, axs = plt.subplots(2, 2, figsize=(12, 10))
        fig.suptitle("Delta Robot Simulation Analysis (Experimental Plots)", fontsize=16, color="#00F0FF", weight="bold")

        # 1. XY Plane Plot (Top View)
        axs[0, 0].plot(x_p, y_p, color="#00F0FF", label="EE Path", linewidth=1.5)
        axs[0, 0].set_title("XY Plane Projection (Top View)", color="white")
        axs[0, 0].set_xlabel("X (mm)")
        axs[0, 0].set_ylabel("Y (mm)")
        axs[0, 0].grid(True, color="#444444", linestyle="--")
        axs[0, 0].set_facecolor("#111111")
        axs[0, 0].axis('equal')

        # 2. XZ Plane Plot (Front View)
        axs[0, 1].plot(x_p, z_p, color="#FF007F", label="EE Path", linewidth=1.5)
        axs[0, 1].set_title("XZ Plane Projection (Front View)", color="white")
        axs[0, 1].set_xlabel("X (mm)")
        axs[0, 1].set_ylabel("Z (mm)")
        axs[0, 1].grid(True, color="#444444", linestyle="--")
        axs[0, 1].set_facecolor("#111111")

        # 3. YZ Plane Plot (Side View)
        axs[1, 0].plot(y_p, z_p, color="#39FF14", label="EE Path", linewidth=1.5)
        axs[1, 0].set_title("YZ Plane Projection (Side View)", color="white")
        axs[1, 0].set_xlabel("Y (mm)")
        axs[1, 0].set_ylabel("Z (mm)")
        axs[1, 0].grid(True, color="#444444", linestyle="--")
        axs[1, 0].set_facecolor("#111111")

        # 4. Velocities over time
        if t_v:
            axs[1, 1].plot(t_v, robot_vel, color="#00F0FF", label="Mechanism (EE)", linewidth=1.5)
        if speeds:
            t_s = [s[0] for s in speeds]
            v_s = [math.hypot(s[1], s[2]) for s in speeds]
            axs[1, 1].step(t_s, v_s, where='post', color="#FF007F", label="Conveyor", linewidth=1.5)
        axs[1, 1].set_title("Velocity Profile over Time", color="white")
        axs[1, 1].set_xlabel("Time (s)")
        axs[1, 1].set_ylabel("Velocity (mm/s)")
        axs[1, 1].grid(True, color="#444444", linestyle="--")
        axs[1, 1].set_facecolor("#111111")
        axs[1, 1].legend(loc="upper right", facecolor="#222222", edgecolor="#444444")

        # Color customizations
        fig.patch.set_facecolor("#1e1e1e")
        for ax in axs.flat:
            ax.tick_params(colors="white")
            ax.xaxis.label.set_color("white")
            ax.yaxis.label.set_color("white")

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        
        # Save plots
        brain_dir = Path("/home/tangerine/.gemini/antigravity/brain/0101fdf5-a10b-4f14-8826-f283eca685d6")
        brain_dir.mkdir(parents=True, exist_ok=True)
        brain_plots_path = brain_dir / "simulation_plots.png"
        
        plt.savefig(brain_plots_path, dpi=150)
        plt.savefig("./simulation_plots.png", dpi=150)
        plt.close()
        print(f"\n[+] Experimental plots saved to: {brain_plots_path}")
        print("[+] Also saved locally to: ./simulation_plots.png")
    except Exception as exc:
        print(f"\n[ERROR] Failed to generate plots: {exc}")


if __name__ == "__main__":
    main()
