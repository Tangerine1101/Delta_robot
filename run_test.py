#!/usr/bin/env python3
import argparse
import json
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
robot_positions = []   # tuples of (t_rel, x, y, z, e)  — from pos_EE (real/fake PLC)
conveyor_speeds = []   # tuples of (t_rel, vx, vy)
planned_waypoints = [] # tuples of (t_rel, x, y, z, e)  — from [TRAJ] lines (evaluate)
cycle_metrics = []     # tuples of (t_rel, dict)          — from [CYCLE] lines (evaluate)
start_time = 0.0

def stream_output(process, prefix):
    is_plc = "[PLC]" in prefix
    last_print_time = 0.0
    for line in iter(process.stdout.readline, ''):
        cleaned = line.strip()
        if cleaned:
            if "PLC status:" in cleaned:
                now = time.monotonic()
                if now - last_print_time >= 0.1:
                    print(f"{prefix} {cleaned}")
                    last_print_time = now
            else:
                print(f"{prefix} {cleaned}")

            # Parse trajectory / speed data
            t_rel = time.monotonic() - start_time
            if is_plc:
                match = re.search(r"pos_EE=\[([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+)\].*?end_effector=(\d+)", cleaned)
                if match:
                    x, y, z, e = map(float, match.groups())
                    with data_lock:
                        robot_positions.append((t_rel, x, y, z, int(e)))
            else:
                # [SPEED] lines (throughput / accuracy scenarios)
                match = re.search(r"\[SPEED\] vx=([-\d\.]+) vy=([-\d\.]+)", cleaned)
                if match:
                    vx, vy = map(float, match.groups())
                    with data_lock:
                        conveyor_speeds.append((t_rel, vx, vy))

                # [TRAJ] lines (evaluate scenario) — planned waypoints for trajectory viz
                if "[TRAJ]" in cleaned:
                    traj_match = re.search(r"\[TRAJ\] (\{.*\})", cleaned)
                    if traj_match:
                        try:
                            traj_data = json.loads(traj_match.group(1))
                            e_val = 1 if traj_data.get("phase") == "pick" else 0
                            for pt in traj_data.get("waypoints", []):
                                with data_lock:
                                    planned_waypoints.append((
                                        t_rel,
                                        float(pt["x"]),
                                        float(pt["y"]),
                                        float(pt["z"]),
                                        int(pt.get("e", e_val)),
                                    ))
                        except Exception:
                            pass

                # [CYCLE] lines (evaluate scenario) — throughput metrics over time
                if "[CYCLE]" in cleaned:
                    cycle_match = re.search(r"\[CYCLE\] (\{.*\})", cleaned)
                    if cycle_match:
                        try:
                            with data_lock:
                                cycle_metrics.append((t_rel, json.loads(cycle_match.group(1))))
                        except Exception:
                            pass
    process.stdout.close()

def main():
    parser = argparse.ArgumentParser(description="Run Delta Robot simulation integration test with real-time visualization.")
    parser.add_argument(
        "--scenario",
        default="test_throughput",
        choices=["test_throughput", "test_accuracy", "evaluate"],
        help="Test scenario to run (default: test_throughput)",
    )
    parser.add_argument(
        "--duration",
        type=int,
        default=None,
        help="Duration of the test in seconds (default: 30 for throughput/accuracy, 60 for evaluate)",
    )
    args = parser.parse_args()
    if args.duration is None:
        args.duration = 60 if args.scenario == "evaluate" else 30

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

    # Real-time Plotting setup
    fig = None
    ax_traj = None
    ax_time = None
    ax_vel = None
    try:
        import matplotlib
        matplotlib.use('TkAgg') # Use interactive GUI backend
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        
        plt.ion() # Turn interactive mode on
        fig = plt.figure(figsize=(18, 6))
        ax_traj = fig.add_subplot(1, 3, 1, projection='3d')
        ax_time = fig.add_subplot(1, 3, 2)
        ax_vel = fig.add_subplot(1, 3, 3)
        fig.patch.set_facecolor("#1e1e1e")
        plt.show(block=False)
        print("[*] Real-time visualization window initialized successfully.")
    except Exception as exc:
        print(f"\n[WARN] Failed to initialize real-time plot window: {exc}")
        print("[*] Make sure Tkinter is installed: sudo apt-get install python3-tk")
        fig = None

    try:
        # Loop and update plots in real-time until main process exits
        while main_proc.poll() is None:
            if fig is not None:
                try:
                    with data_lock:
                        positions = list(robot_positions)
                        c_speeds = list(conveyor_speeds)
                    
                    with data_lock:
                        waypoints = list(planned_waypoints)
                        c_metrics = list(cycle_metrics)

                    # Choose data source for 3D trajectory:
                    # evaluate: use planned waypoints; others: use pos_EE from PLC.
                    traj_source = waypoints if args.scenario == "evaluate" else positions
                    has_data = len(traj_source) > 0 or len(positions) > 0

                    if has_data:
                        ax_traj.clear()
                        ax_time.clear()
                        ax_vel.clear()

                        # --- Chart 1: 3D Trajectory ---
                        traj_title = "3D Planned Trajectory (evaluate)" if args.scenario == "evaluate" else "3D Trajectory (Last 3 Cycles)"
                        if traj_source:
                            phases = []
                            current_phase = []
                            for p in traj_source:
                                e = p[4] if len(p) > 4 else 0
                                if not current_phase:
                                    current_phase.append(p)
                                elif e == (current_phase[-1][4] if len(current_phase[-1]) > 4 else 0):
                                    current_phase.append(p)
                                else:
                                    phases.append(current_phase)
                                    current_phase = [p]
                            if current_phase:
                                phases.append(current_phase)

                            last_6_phases = phases[-6:]
                            for i, phase in enumerate(last_6_phases):
                                xs = [pt[1] for pt in phase]
                                ys = [pt[2] for pt in phase]
                                zs = [pt[3] for pt in phase]
                                e = phase[0][4] if len(phase[0]) > 4 else 0
                                color = '#FF007F' if e == 1 else '#00F0FF'
                                label = 'Pick (Suction ON)' if e == 1 else 'Goto (Suction OFF)'
                                alpha = 1.0 if i >= len(last_6_phases) - 2 else 0.4
                                lw = 2.0 if alpha == 1.0 else 1.5
                                show_label = i >= len(last_6_phases) - 2
                                ax_traj.plot(xs, ys, zs, color=color, linewidth=lw, alpha=alpha,
                                             label=label if show_label else None)
                                # Mark waypoints as dots
                                ax_traj.scatter(xs, ys, zs, color=color, s=18, alpha=alpha, zorder=5)

                        ax_traj.set_title(traj_title, color="white", weight="bold")
                        ax_traj.set_xlabel("X (mm)", color="white")
                        ax_traj.set_ylabel("Y (mm)", color="white")
                        ax_traj.set_zlabel("Z (mm)", color="white")
                        ax_traj.grid(True, color="#444444", linestyle="--")
                        ax_traj.set_facecolor("#111111")
                        ax_traj.tick_params(colors="white")
                        ax_traj.xaxis.label.set_color("white")
                        ax_traj.yaxis.label.set_color("white")
                        ax_traj.zaxis.label.set_color("white")
                        ax_traj.legend(loc="upper right", facecolor="#222222", edgecolor="#444444", labelcolor="white")

                        # --- Chart 2: Coordinates vs Time ---
                        coord_source = waypoints if (args.scenario == "evaluate" and waypoints) else positions
                        if coord_source:
                            t_p = [pt[0] for pt in coord_source]
                            x_p = [pt[1] for pt in coord_source]
                            y_p = [pt[2] for pt in coord_source]
                            z_p = [pt[3] for pt in coord_source]
                            ax_time.plot(t_p, x_p, color="#00F0FF", label="X", linewidth=1.5)
                            ax_time.plot(t_p, y_p, color="#39FF14", label="Y", linewidth=1.5)
                            ax_time.plot(t_p, z_p, color="#FF007F", label="Z", linewidth=1.5)

                        coord_label = "Planned Waypoints" if args.scenario == "evaluate" else "pos_EE"
                        ax_time.set_title(f"Coordinates vs Time ({coord_label})", color="white", weight="bold")
                        ax_time.set_xlabel("Time (s)", color="white")
                        ax_time.set_ylabel("Position (mm)", color="white")
                        ax_time.grid(True, color="#444444", linestyle="--")
                        ax_time.set_facecolor("#111111")
                        ax_time.tick_params(colors="white")
                        ax_time.xaxis.label.set_color("white")
                        ax_time.yaxis.label.set_color("white")
                        ax_time.legend(loc="upper right", facecolor="#222222", edgecolor="#444444", labelcolor="white")

                        # --- Chart 3: Velocity / Metrics ---
                        if args.scenario == "evaluate" and c_metrics:
                            # Plot throughput and avg_speed over time from [CYCLE] logs
                            ct = [cm[0] for cm in c_metrics]
                            throughput = [cm[1].get("throughput_pick_per_min", 0) for cm in c_metrics]
                            avg_speed = [cm[1].get("avg_speed_mm_s", 0) for cm in c_metrics]
                            ax_vel.plot(ct, throughput, color="#FF007F", label="Throughput (pick/min)", linewidth=2.0)
                            ax2 = ax_vel.twinx()
                            ax2.plot(ct, avg_speed, color="#39FF14", label="Avg speed (mm/s)", linewidth=1.5, linestyle="--")
                            ax2.set_ylabel("Avg speed (mm/s)", color="#39FF14")
                            ax2.tick_params(colors="#39FF14")
                            ax_vel.set_title("Evaluate Metrics vs Time", color="white", weight="bold")
                            ax_vel.set_xlabel("Time (s)", color="white")
                            ax_vel.set_ylabel("Throughput (pick/min)", color="#FF007F")
                            lines1, labels1 = ax_vel.get_legend_handles_labels()
                            lines2, labels2 = ax2.get_legend_handles_labels()
                            ax_vel.legend(lines1 + lines2, labels1 + labels2, loc="upper left",
                                          facecolor="#222222", edgecolor="#444444", labelcolor="white")
                        else:
                            # Standard velocity from pos_EE diffs
                            ee_times = []
                            ee_speeds = []
                            for i in range(1, len(positions)):
                                dt = positions[i][0] - positions[i-1][0]
                                if dt > 0.001:
                                    dx = positions[i][1] - positions[i-1][1]
                                    dy = positions[i][2] - positions[i-1][2]
                                    dz = positions[i][3] - positions[i-1][3]
                                    speed = math.sqrt(dx*dx + dy*dy + dz*dz) / dt
                                    ee_times.append(positions[i][0])
                                    ee_speeds.append(speed)
                            conv_times = [cs[0] for cs in c_speeds]
                            conv_speeds_list = [math.sqrt(cs[1]**2 + cs[2]**2) for cs in c_speeds]
                            if ee_times:
                                ax_vel.plot(ee_times, ee_speeds, color="#FF007F", label="End-Effector (3D)", linewidth=1.5)
                            if conv_times:
                                ax_vel.plot(conv_times, conv_speeds_list, color="#39FF14", label="Conveyor", linewidth=1.5)
                            ax_vel.legend(loc="upper right", facecolor="#222222", edgecolor="#444444", labelcolor="white")

                        ax_vel.grid(True, color="#444444", linestyle="--")
                        ax_vel.set_facecolor("#111111")
                        ax_vel.tick_params(colors="white")
                        ax_vel.xaxis.label.set_color("white")
                        ax_vel.yaxis.label.set_color("white")
                        if args.scenario != "evaluate":
                            ax_vel.set_title("Velocity vs Time", color="white", weight="bold")
                            ax_vel.set_xlabel("Time (s)", color="white")
                            ax_vel.set_ylabel("Speed (mm/s)", color="white")

                        plt.draw()
                except Exception as draw_exc:
                    print(f"[DEBUG] Redraw issue: {draw_exc}")
            time.sleep(0.1)
            try:
                plt.pause(0.01)
            except Exception:
                pass
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

    # Save final plots at the end of the test
    if fig is not None:
        try:
            plt.ioff()
            # Save plots to correct paths
            brain_dir = Path("/home/tangerine/.gemini/antigravity-ide/brain/f7d0b057-6c2b-48d6-a58c-f117a7708c07")
            brain_dir.mkdir(parents=True, exist_ok=True)
            brain_plots_path = brain_dir / "simulation_plots.png"
            fig.savefig(brain_plots_path, dpi=150, facecolor=fig.get_facecolor(), edgecolor='none')
            fig.savefig("./simulation_plots.png", dpi=150, facecolor=fig.get_facecolor(), edgecolor='none')
            print(f"\n[+] Final plots saved to: {brain_plots_path}")
            print("[+] Also saved locally to: ./simulation_plots.png")
            plt.close(fig)
        except Exception as exc:
            print(f"[ERROR] Failed to save final plots: {exc}")

if __name__ == "__main__":
    main()
