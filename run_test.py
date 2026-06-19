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

# Make sibling modules importable so we can reuse F, windows, length.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from modules.conveyor import ConveyorFrame  # noqa: E402
from modules.EthernetCom import load_config  # noqa: E402

# Global data collectors for plotting
data_lock = threading.Lock()
robot_positions = []     # tuples of (t_rel, x, y, z, e)  — from pos_EE (real/fake PLC)
conveyor_speeds = []     # tuples of (t_rel, vx, vy)
planned_waypoints = []   # tuples of (t_rel, x, y, z, e)  — from [TRAJ] lines (evaluate)
cycle_metrics = []       # tuples of (t_rel, dict)          — from [CYCLE] lines (evaluate)
predicted_positions = [] # tuples of (t_rel, x, y, z)      — from [PREDICT] lines (real scenarios)
detected_objects = {}    # object_id -> (t_rel, x, y, z)    — from [DETECT] lines (live tracked objects)
start_time = 0.0


def _uv_window_to_corners(frame: ConveyorFrame, window, z):
    """Project a C-frame (u_min, u_max, v_min, v_max) rectangle onto R-frame
    corners at constant Z, closed polygon (5 points)."""
    u_min, u_max, v_min, v_max = window
    uv = [(u_min, v_min), (u_max, v_min), (u_max, v_max), (u_min, v_max), (u_min, v_min)]
    xs, ys = [], []
    for u, v in uv:
        x, y = frame.to_robot(u, v)
        xs.append(x)
        ys.append(y)
    zs = [z] * len(xs)
    return xs, ys, zs

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

                # [PREDICT] lines (real scenarios) — predicted pick positions
                predict_match = re.search(r'\[PREDICT\]\s+(\{.*\})', cleaned)
                if predict_match:
                    try:
                        d = json.loads(predict_match.group(1))
                        with data_lock:
                            predicted_positions.append((d["t"], d["x"], d["y"], d["z"]))
                    except Exception:
                        pass

                # [DETECT] lines — live R-frame positions of every tracked object.
                detect_match = re.search(r'\[DETECT\]\s+(\{.*\})', cleaned)
                if detect_match:
                    try:
                        d = json.loads(detect_match.group(1))
                        z = d.get("z", 0.0)
                        with data_lock:
                            for obj in d.get("objects", []):
                                detected_objects[obj["id"]] = (d["t"], obj["x"], obj["y"], z)
                    except Exception:
                        pass
    process.stdout.close()

def main():
    parser = argparse.ArgumentParser(description="Run Delta Robot simulation integration test with real-time visualization.")
    parser.add_argument(
        "--scenario",
        default="test_throughput",
        choices=["test_throughput", "test_accuracy", "evaluate", "test_conveyor", "test_vision_only"],
        help="Test scenario to run (default: test_throughput)",
    )
    parser.add_argument(
        "--duration",
        type=int,
        default=None,
        help="Duration of the test in seconds (default: 99999 — effectively unlimited; "
             "stop with Ctrl-C). Pass an explicit value to cap the run.",
    )
    args = parser.parse_args()
    if args.duration is None:
        # Default to ~unlimited so a test runs until the operator stops it (Ctrl-C).
        args.duration = 99999

    port = 1502

    # Load config early so we have the real PLC IP for test_conveyor.
    cfg_early = load_config()

    # Determine subprocess commands based on scenario.
    _real_scenarios = {"test_conveyor", "test_vision_only"}
    if args.scenario in _real_scenarios:
        # No fake PLC — connect directly to the real hardware. test_vision_only
        # keeps the robot idle (NullExecutor in main.py) but still reads the real
        # Siemens conveyor_position, so it needs the live PLC IP just like
        # test_conveyor — never --simulate-executor (that fabricates belt speed).
        plc_cmd = None
        main_cmd = [
            sys.executable,
            "main.py",
            "--scheduler",
            "--scenario", args.scenario,
            "--ip", str(getattr(cfg_early, "ip_address", "192.168.250.1")),
            "--port", str(getattr(cfg_early, "port", 44818)),
            "--duration", str(args.duration),
        ]
    else:
        # Simulated scenarios — start fake PLC first.
        plc_cmd = [
            sys.executable,
            "-m", "modules.test_module",
            "--port", str(port),
            "--duration", str(args.duration + 5),
        ]
        main_cmd = [
            sys.executable,
            "main.py",
            "--scheduler",
            "--ip", "127.0.0.1",
            "--port", str(port),
            "--scenario", args.scenario,
            "--duration", str(args.duration),
        ]

    # --- Geometry: belt frame and windows (loaded once for plot overlays) ---
    cfg = cfg_early  # reuse already-loaded config
    conveyor_cfg = getattr(cfg, "conveyor", {}) or {}
    scheduler_cfg = getattr(cfg, "scheduler", {}) or {}
    frame = ConveyorFrame()
    workspace_window = tuple(conveyor_cfg.get("workspace_window_uv", [450.0, 620.0, -65.0, 65.0]))
    camera_window = tuple(conveyor_cfg.get("camera_window_uv", [50.0, 250.0, -75.0, 75.0]))
    belt_length_mm = float(conveyor_cfg.get("length_mm", 800.0))
    pickup_height = float(scheduler_cfg.get("pickup_height", -310.0))
    clearance_height = float(scheduler_cfg.get("clearance_height", -240.0))
    # Sorting bin XY locations (just to anchor the X/Y view bounds). QFP/TQFP with pcb1/pcb2 fallback.
    bin1 = list(getattr(cfg, "QFP",  getattr(cfg, "pcb1", [0.0, 0.0, -300.0])))[:2]
    bin2 = list(getattr(cfg, "TQFP", getattr(cfg, "pcb2", [0.0, 0.0, -300.0])))[:2]
    # Compute belt vector endpoints in robot frame, then derive fixed axes bounds.
    belt_start_R = frame.to_robot(0.0, 0.0)
    belt_end_R = frame.to_robot(belt_length_mm, 0.0)
    all_x = [belt_start_R[0], belt_end_R[0], bin1[0], bin2[0]]
    all_y = [belt_start_R[1], belt_end_R[1], bin1[1], bin2[1]]
    margin = 60.0
    x_lim = (min(all_x) - margin, max(all_x) + margin)
    y_lim = (min(all_y) - margin, max(all_y) + margin)
    z_lim = (pickup_height - 20.0, clearance_height + 20.0)

    global start_time
    start_time = time.monotonic()

    plc_proc = None
    if plc_cmd is not None:
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
    if plc_proc is not None:
        t_plc = threading.Thread(target=stream_output, args=(plc_proc, "\033[94m[PLC]\033[0m"), daemon=True)
        t_plc.start()
    t_main = threading.Thread(target=stream_output, args=(main_proc, "\033[92m[MAIN]\033[0m"), daemon=True)
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
                        detected = dict(detected_objects)
                        pred_snapshot = list(predicted_positions)

                    # Choose data source for 3D trajectory:
                    # evaluate: use planned waypoints; others: use pos_EE from PLC.
                    traj_source = waypoints if args.scenario == "evaluate" else positions
                    # Also render when only vision data is present (test_vision_only
                    # has no robot pos_EE), so detected objects still show up.
                    has_data = (
                        len(traj_source) > 0
                        or len(positions) > 0
                        or len(detected) > 0
                        or len(pred_snapshot) > 0
                        or len(c_speeds) > 0          # belt speed flows from t≈0 in vision scenarios
                    )

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

                        # --- Belt vector at pickup height ---
                        dx_belt = belt_end_R[0] - belt_start_R[0]
                        dy_belt = belt_end_R[1] - belt_start_R[1]
                        ax_traj.quiver(
                            belt_start_R[0], belt_start_R[1], pickup_height,
                            dx_belt, dy_belt, 0.0,
                            color="#00FFFF", linewidth=2.0,
                            arrow_length_ratio=0.05, label="Belt direction (+u)",
                        )
                        # --- Camera window (magenta dashed) ---
                        cx, cy, cz = _uv_window_to_corners(frame, camera_window, pickup_height)
                        ax_traj.plot(cx, cy, cz, color="#FF00FF", linestyle="--",
                                     linewidth=1.5, label="Camera window")
                        # --- Workspace window (green solid) ---
                        wx, wy, wz = _uv_window_to_corners(frame, workspace_window, pickup_height)
                        ax_traj.plot(wx, wy, wz, color="#39FF14", linestyle="-",
                                     linewidth=2.0, label="Workspace (pick zone)")
                        # --- Sort bin markers ---
                        ax_traj.scatter([bin1[0]], [bin1[1]], [pickup_height],
                                        color="#FFB000", s=60, marker="s", label="QFP bin")
                        ax_traj.scatter([bin2[0]], [bin2[1]], [pickup_height],
                                        color="#FF6F00", s=60, marker="s", label="TQFP bin")

                        # --- Detected objects (live R-frame positions) ---
                        # Keep only objects seen in the last ~1.5s (scheduler clock)
                        # so stale markers fade as objects leave the belt.
                        if detected:
                            latest_t = max(v[0] for v in detected.values())
                            fresh = [v for v in detected.values() if v[0] >= latest_t - 1.5]
                            if fresh:
                                dx = [v[1] for v in fresh]
                                dy = [v[2] for v in fresh]
                                dz = [v[3] for v in fresh]
                                ax_traj.scatter(dx, dy, dz, color="#FFFFFF", s=45, marker="o",
                                                edgecolors="#00F0FF", zorder=7,
                                                label="Detected objects")

                        # --- Predicted pick positions (real scenarios) ---
                        pred_pts = pred_snapshot[-50:]
                        if pred_pts:
                            px = [p[1] for p in pred_pts]
                            py = [p[2] for p in pred_pts]
                            pz = [p[3] for p in pred_pts]
                            ax_traj.scatter(px, py, pz, color="#FF8C00", s=40, marker="^",
                                            zorder=6, label="Predicted pick")

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
                        # Fixed axes — prevents the camera frame from drifting between redraws.
                        ax_traj.set_xlim(*x_lim)
                        ax_traj.set_ylim(*y_lim)
                        ax_traj.set_zlim(*z_lim)
                        try:
                            ax_traj.set_box_aspect((1.0, 1.0, 0.5))
                        except Exception:
                            pass
                        ax_traj.legend(loc="upper right", facecolor="#222222", edgecolor="#444444", labelcolor="white", fontsize=8)

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
        if plc_proc is not None and plc_proc.poll() is None:
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
