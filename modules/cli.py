from __future__ import annotations

import math
import os
import shlex
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Any, Callable, Iterable

from modules.EthernetCom import COMMAND_ID, RobotPacket


INTERPOLAR_POINTS = 4


@dataclass
class CommandPlan:
    packages: list[dict[str, Any]]
    show_status: bool = False
    quit_requested: bool = False


PRESET_TRAJECTORIES: dict[str, list[dict[str, Any]]] = {
    "demo": [
        {"x": 0.0, "y": 0.0, "z": -300.0, "e": 0, "time": 0.4},
        {"x": -50.0, "y": 2.0, "z": -300.0, "e": 0, "time": 0.4},
        {"x": -60.0, "y": 4.0, "z": -315.0, "e": 0, "time": 0.4},
        {"x": -60.0, "y": 6.0, "z": -325.0, "e": 0, "time": 0.4},
    ],
    "square": [
        {"x": 0.0, "y": -80.0, "z": -300.0, "e": 0, "time": 0.35},
        {"x": 0.0, "y": -10.0, "z": -300.0, "e": 0, "time": 0.35},
        {"x": 0.0, "y": 0.0,  "z": -305.0, "e": 0, "time": 0.35},
        {"x": 0.0, "y": 0.0, "z": -325.0, "e": 0, "time": 0.35},
    ],
    "home": [
        {"x": 0.0, "y": 0.0, "z": -260.0, "e": 0, "time": 0.6},
    ],
}


def _pad(
    values: Iterable[Any],
    interpolar_points: int = INTERPOLAR_POINTS,
    fill_value: Any = 0.0,
) -> list[Any]:
    items = list(values)[:interpolar_points]
    if len(items) < interpolar_points:
        items.extend([fill_value] * (interpolar_points - len(items)))
    return items


def _zero_command(command_name: str, interpolar_points: int = INTERPOLAR_POINTS) -> dict[str, Any]:
    return RobotPacket(
        commandID=COMMAND_ID[command_name],
        argument_number=0,
        argument_x=[0.0] * interpolar_points,
        argument_y=[0.0] * interpolar_points,
        argument_z=[0.0] * interpolar_points,
        argument_e=[0] * interpolar_points,
        argument_time=[0.0] * interpolar_points,
    ).to_dict(interpolar_points)


def _trajectory_command(
    name: str,
    points: list[dict[str, Any]],
    interpolar_points: int = INTERPOLAR_POINTS,
) -> dict[str, Any]:
    del name
    if len(points) > interpolar_points:
        raise ValueError(
            f"Trajectory has {len(points)} points but PLC package only allows {interpolar_points} points."
        )
    return RobotPacket(
        commandID=COMMAND_ID["go_trajectory"],
        argument_number=len(points),
        argument_x=_pad((point["x"] for point in points), interpolar_points),
        argument_y=_pad((point["y"] for point in points), interpolar_points),
        argument_z=_pad((point["z"] for point in points), interpolar_points),
        argument_e=_pad((1 if point.get("e", 0) else 0 for point in points), interpolar_points, 0),
        argument_time=_pad((point["time"] for point in points), interpolar_points, 0.0),
    ).to_dict(interpolar_points)


def _joint_command(
    command_name: str,
    theta1: float,
    theta2: float,
    theta3: float,
    interpolar_points: int = INTERPOLAR_POINTS,
) -> dict[str, Any]:
    return RobotPacket(
        commandID=COMMAND_ID[command_name],
        argument_number=1,
        argument_x=_pad([theta1], interpolar_points, 0.0),
        argument_y=_pad([theta2], interpolar_points, 0.0),
        argument_z=_pad([theta3], interpolar_points, 0.0),
        argument_e=[0] * interpolar_points,
        argument_time=[0.0] * interpolar_points,
    ).to_dict(interpolar_points)


def _cartesian_command(
    command_name: str,
    x: float,
    y: float,
    z: float,
    interpolar_points: int = INTERPOLAR_POINTS,
) -> dict[str, Any]:
    return RobotPacket(
        commandID=COMMAND_ID[command_name],
        argument_number=1,
        argument_x=_pad([x], interpolar_points, 0.0),
        argument_y=_pad([y], interpolar_points, 0.0),
        argument_z=_pad([z], interpolar_points, 0.0),
        argument_e=[0] * interpolar_points,
        argument_time=[0.0] * interpolar_points,
    ).to_dict(interpolar_points)


def _parse_plan(
    line: str,
    interpolar_points: int = INTERPOLAR_POINTS,
    request_status: Callable[[], dict[str, Any] | None] | None = None,
) -> CommandPlan:
    tokens = shlex.split(line)
    if not tokens:
        return CommandPlan(packages=[])

    command = tokens[0].lower()

    if command in {"exit", "quit"}:
        return CommandPlan(packages=[], quit_requested=True)
    if command in {"?", "help"}:
        return CommandPlan(packages=[])
    if command == "status":
        return CommandPlan(packages=[], show_status=True)
    if command == "stop":
        return CommandPlan(packages=[_zero_command("stop", interpolar_points)])
    if command == "go":
        if len(tokens) != 4:
            raise ValueError("go expects 3 numbers: go <theta1> <theta2> <theta3>")
        return CommandPlan(
            packages=[
                _joint_command(
                    "goto_relative",
                    float(tokens[1]),
                    float(tokens[2]),
                    float(tokens[3]),
                    interpolar_points,
                )
            ]
        )
    if command == "goto":
        if len(tokens) != 4:
            raise ValueError("goto expects 3 values: goto <x> <y> <z>")
        x = float(tokens[1])
        y = float(tokens[2])
        z = float(tokens[3])
        return CommandPlan(
            packages=[_cartesian_command("goto_absolute", x, y, z, interpolar_points)]
        )
    if command == "go_trajectory":
        if len(tokens) != 2:
            raise ValueError("go_trajectory expects a preset name: go_trajectory <name>")
        preset_name = tokens[1].lower()
        if preset_name not in PRESET_TRAJECTORIES:
            known = ", ".join(sorted(PRESET_TRAJECTORIES))
            raise ValueError(f"Unknown trajectory preset '{preset_name}'. Available: {known}")
        return CommandPlan(
            packages=[
                _trajectory_command(
                    preset_name,
                    PRESET_TRAJECTORIES[preset_name],
                    interpolar_points,
                )
            ]
        )
    if command == "calib":
        return CommandPlan(packages=[_zero_command("calibrate", interpolar_points)])
    if command in {"pick", "release"}:
        # Suction is controlled via argument_e inside a go_trajectory packet
        # (the dedicated pick/release command IDs are no-ops on the real PLC).
        # Send a 1-point trajectory at the current pos_EE with the gripper bit
        # set/cleared accordingly.
        if request_status is None:
            raise RuntimeError(
                f"{command} requires robot status to read current pos_EE"
            )
        status = request_status()
        if not status:
            raise RuntimeError(f"Cannot retrieve robot status to execute {command}")
        pos = status.get("pos_EE")
        if not pos or len(pos) != 3:
            raise RuntimeError("Invalid pos_EE in robot status")
        x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
        e_value = 1 if command == "pick" else 0
        points = [{"x": x, "y": y, "z": z, "e": e_value, "time": 0.1}]
        return CommandPlan(
            packages=[_trajectory_command(command, points, interpolar_points)]
        )
    if command == "rotate":
        if len(tokens) != 2:
            raise ValueError("rotate expects 1 angle value: rotate <angle>")
        # Human types R-frame DEGREES; the packet carries radians (sent verbatim
        # to the PLC as wire degrees [-359,359] at the IPC boundary in
        # main._worker — no wrap, so 270 stays 270).
        return CommandPlan(
            packages=[{"commandID": COMMAND_ID["rotate_absolute"], "CommandID": COMMAND_ID["rotate_absolute"], "rotate": math.radians(float(tokens[1])), "speed": 0.0}]
        )
    if command == "setspeed":
        if len(tokens) != 2:
            raise ValueError("setspeed expects 1 speed value: setspeed <speed>")
        return CommandPlan(
            packages=[{"commandID": COMMAND_ID["change_speed"], "CommandID": COMMAND_ID["change_speed"], "rotate": 0.0, "speed": float(tokens[1])}]
        )
    if command == "plan_siemen":
        if len(tokens) != 3:
            raise ValueError("plan_siemen expects 2 values: plan_siemen <rotate> <speed>")
        return CommandPlan(
            packages=[{"commandID": COMMAND_ID["plan_siemen"], "CommandID": COMMAND_ID["plan_siemen"], "rotate": float(tokens[1]), "speed": float(tokens[2])}]
        )
    if command == "grab":
        print("[WARN] grab/place do not actuate suction on the real PLC — known limitation")
        if len(tokens) not in (5, 6):
            raise ValueError("grab expects: grab <object> <x> <y> <z> [rotate]")
        obj_name = tokens[1]
        x = float(tokens[2])
        y = float(tokens[3])
        z = float(tokens[4])
        rotate_angle = float(tokens[5]) if len(tokens) == 6 else None

        from modules.EthernetCom import load_config
        config = load_config()
        scheduler_raw = getattr(config, "scheduler", {}) or {}
        clearance = float(scheduler_raw.get("clearance_height", -290.0))

        packages = []
        if rotate_angle is not None:
            packages.append({
                "commandID": COMMAND_ID["rotate_absolute"],
                "CommandID": COMMAND_ID["rotate_absolute"],
                # human degrees -> packet radians (wire conversion in main._worker)
                "rotate": math.radians(rotate_angle),
                "speed": 0.0
            })
        packages.append(_cartesian_command("goto_absolute", x, y, clearance, interpolar_points))
        packages.append(_cartesian_command("goto_absolute", x, y, z, interpolar_points))
        packages.append(_zero_command("pick", interpolar_points))
        packages.append(_cartesian_command("goto_absolute", x, y, clearance, interpolar_points))
        return CommandPlan(packages=packages)
    if command == "place":
        print("[WARN] grab/place do not actuate suction on the real PLC — known limitation")
        if len(tokens) not in (5, 6):
            raise ValueError("place expects: place <object> <x> <y> <z> [rotate]")
        obj_name = tokens[1]
        x = float(tokens[2])
        y = float(tokens[3])
        z = float(tokens[4])
        rotate_angle = float(tokens[5]) if len(tokens) == 6 else None

        from modules.EthernetCom import load_config
        config = load_config()
        scheduler_raw = getattr(config, "scheduler", {}) or {}
        clearance = float(scheduler_raw.get("clearance_height", -290.0))

        packages = []
        if rotate_angle is not None:
            packages.append({
                "commandID": COMMAND_ID["rotate_absolute"],
                "CommandID": COMMAND_ID["rotate_absolute"],
                # human degrees -> packet radians (wire conversion in main._worker)
                "rotate": math.radians(rotate_angle),
                "speed": 0.0
            })
        packages.append(_cartesian_command("goto_absolute", x, y, clearance, interpolar_points))
        packages.append(_cartesian_command("goto_absolute", x, y, z, interpolar_points))
        packages.append(_zero_command("release", interpolar_points))
        packages.append(_cartesian_command("goto_absolute", x, y, clearance, interpolar_points))
        return CommandPlan(packages=packages)
    if command == "jog":
        if len(tokens) != 3:
            raise ValueError("jog expects: jog <x|y|z> <distance>")
        axis = tokens[1].lower()
        if axis not in ("x", "y", "z"):
            raise ValueError("jog axis must be x, y, or z")
        val = float(tokens[2])

        if request_status is None:
            dx = val if axis == "x" else 0.0
            dy = val if axis == "y" else 0.0
            dz = val if axis == "z" else 0.0
            return CommandPlan(packages=[
                _cartesian_command("goto_relative", dx, dy, dz, interpolar_points)
            ])

        status = request_status()
        if not status:
            raise RuntimeError("Cannot retrieve robot status to execute jog command")
        pos = status.get("pos_EE")
        if not pos or len(pos) != 3:
            raise RuntimeError("Invalid pos_EE in robot status")
        x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
        if axis == "x":
            x += val
        elif axis == "y":
            y += val
        elif axis == "z":
            z += val

        return CommandPlan(packages=[
            _cartesian_command("goto_absolute", x, y, z, interpolar_points)
        ])
    raise ValueError(f"Unknown command: {command}")


_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _run_validate() -> None:
    """Validate the whole config (delegates to calibrate_everything --check)."""
    try:
        import calibrate_everything  # repo-root script; no cv2/ultralytics imported
    except Exception as exc:  # noqa: BLE001
        print(f"[ERROR] could not import calibrate_everything: {exc}")
        return
    print("[INFO] Validating config (calibrate_everything --check)...")
    try:
        rc = calibrate_everything.main(["--check"])
    except SystemExit as exc:
        rc = int(exc.code) if exc.code is not None else 0
    except Exception as exc:  # noqa: BLE001
        print(f"[ERROR] validation crashed: {exc}")
        return
    print(f"[INFO] validate result: {'OK' if rc == 0 else f'FAILED ({rc})'}")


def _run_camera_tuning(extra_args: list[str]) -> None:
    """Run the interactive camera calibration tool (camera_calibrate.py).

    Delegated as a subprocess so QT_QPA_PLATFORM/cv2 stay isolated from the CLI
    process. Extra tokens are passed through (e.g. `camera_tuning --roi`).
    """
    script = os.path.join(_REPO_ROOT, "camera_calibrate.py")
    cmd = [sys.executable, script, *extra_args]
    print(f"[INFO] delegating to camera_calibrate.py: {' '.join(cmd)}")
    try:
        rc = subprocess.run(cmd, cwd=_REPO_ROOT, check=False).returncode
    except Exception as exc:  # noqa: BLE001
        print(f"[ERROR] camera_tuning failed: {exc}")
        return
    print(f"[INFO] camera_tuning exit code: {rc}")


def _wait_for_arrival(
    request_status: Callable[[], dict[str, Any] | None],
    target: tuple[float, float, float],
    *,
    tol_mm: float = 5.0,
    timeout_s: float = 30.0,
    poll_s: float = 0.05,
    depart_from: tuple[float, float, float] | None = None,
) -> bool:
    """Poll pos_EE until it is within `tol_mm` of `target` (or timeout).

    When `depart_from` is given, the arm must first move more than `tol_mm` away
    from it before an arrival is accepted. This guards against a stale pos_EE
    reading "already arrived" before the commanded motion has actually started —
    the main cause of the flaky single-shot measurement.
    """
    deadline = time.monotonic() + timeout_s
    departed = depart_from is None
    while time.monotonic() < deadline:
        status = request_status()
        if isinstance(status, dict):
            pos = status.get("pos_EE")
            if isinstance(pos, (list, tuple)) and len(pos) >= 3:
                try:
                    p = (float(pos[0]), float(pos[1]), float(pos[2]))
                except (TypeError, ValueError):
                    p = None
                if p is not None:
                    if not departed and depart_from is not None:
                        if math.dist(p, depart_from) > tol_mm:
                            departed = True
                    if departed and math.dist(p, target) <= tol_mm:
                        return True
        time.sleep(poll_s)
    return False


def _run_speed_tuning(
    dispatch: Callable[[dict[str, Any]], dict[str, Any] | None],
    request_status: Callable[[], dict[str, Any] | None] | None,
    interpolar_points: int,
) -> None:
    """Validate the MC_Inter_Curve_Vel timing model against the real mechanism.

    Builds an `interpolar_points`-vertex polygon (heptagon at the default 7) on a
    circle of radius `limit_radius_xy / 2`, tilted in Z between `clearance_height`
    (highest) and `slope_transition_height` (lowest). The arm is parked at vertex
    0, then the polygon is sent as one go_trajectory and the real execution time is
    measured against pos_EE feedback and compared with `_trajectory_total_time`
    (the PLC model used for pick prediction). Reports the measured/model ratio and
    a first-order interpolator suggestion — it never writes config.
    """
    if request_status is None:
        print("[ERROR] speed_tuning needs PLC status feedback (run with a live/fake PLC)")
        return

    from modules.EthernetCom import load_config
    from modules.conveyor import is_within_xy_limit
    from modules.scheduler import SchedulerSettings, _trajectory_total_time

    try:
        settings = SchedulerSettings.from_config(load_config())
    except Exception as exc:  # noqa: BLE001
        print(f"[ERROR] could not load scheduler settings: {exc}")
        return
    limit_radius_xy = float(getattr(load_config(), "limit_radius_xy", 180.0))

    n = max(3, interpolar_points)
    radius = limit_radius_xy / 2.0
    z_mid = (settings.clearance_height + settings.slope_transition_height) / 2.0
    amp = (settings.clearance_height - settings.slope_transition_height) / 2.0

    points: list[dict[str, Any]] = []
    waypoints: list[tuple[float, float, float]] = []
    for i in range(n):
        ang = 2.0 * math.pi * i / n
        x = radius * math.cos(ang)
        y = radius * math.sin(ang)
        z = z_mid + amp * math.cos(ang)  # highest at clearance, lowest at slope
        if not is_within_xy_limit(x, y, limit_radius_xy):
            print(f"[ERROR] vertex ({x:.1f},{y:.1f}) outside reach circle; abort")
            return
        points.append({"x": x, "y": y, "z": z, "e": 0, "time": 0.4})
        waypoints.append((x, y, z))

    # Model prediction for the SAME packet the PLC will run (Pos[0..n-1]).
    model_time = _trajectory_total_time(waypoints, settings)
    path_len = sum(math.dist(waypoints[i], waypoints[i + 1]) for i in range(n - 1))
    first, last = waypoints[0], waypoints[-1]

    print(f"[INFO] speed_tuning (model validation): {n}-gon r={radius:.1f}mm, "
          f"z {settings.slope_transition_height:.1f}..{settings.clearance_height:.1f}, "
          f"path={path_len:.1f}mm")
    print(f"[INFO] model predicts {model_time:.3f}s "
          f"(v_max={settings.interp_v_max:.0f}, a={settings.interp_a_max:.0f}, "
          f"d={settings.interp_d_max:.0f}, soft_start={settings.interp_soft_start_s:.3f}s)")

    # 1) Park at vertex 0.
    print("[INFO] moving to start vertex...")
    dispatch(_cartesian_command("goto_absolute", first[0], first[1], first[2], interpolar_points))
    if not _wait_for_arrival(request_status, first):
        print("[WARN] did not confirm arrival at start vertex; aborting measurement")
        return

    # 2) Send the polygon and time it. depart_from guards against a stale pos_EE
    #    reading "arrived" before the motion actually starts.
    print("[INFO] running trajectory and measuring...")
    dispatch(_trajectory_command("speed_tuning", points, interpolar_points))
    t_start = time.monotonic()
    reached = _wait_for_arrival(request_status, last, timeout_s=60.0, depart_from=first)
    elapsed = time.monotonic() - t_start

    if not reached:
        print(f"[WARN] arm did not reach the final vertex within timeout "
              f"(elapsed {elapsed:.2f}s) — measurement unreliable, retry")
        return
    if elapsed <= 1e-3 or model_time <= 1e-6:
        print("[WARN] elapsed/model time too small to compare")
        return

    ratio = elapsed / model_time
    measured_speed = path_len / elapsed
    print(f"[RESULT] measured {elapsed:.3f}s vs model {model_time:.3f}s  "
          f"-> ratio {ratio:.3f} (avg speed {measured_speed:.1f} mm/s)")
    if ratio > 1.05:
        print(f"[RESULT] real arm is SLOWER than the model. First-order fix: lower "
              f"interpolator.v_max to ~{settings.interp_v_max / ratio:.0f} mm/s "
              f"(also check a_max/d_max).")
    elif ratio < 0.95:
        print(f"[RESULT] real arm is FASTER than the model. First-order fix: raise "
              f"interpolator.v_max to ~{settings.interp_v_max / ratio:.0f} mm/s.")
    else:
        print("[RESULT] model matches the mechanism within 5% — interpolator config OK.")
    print("[RESULT] report only — config NOT modified.")


def _print_help() -> None:
    print(
        "\nCommands:\n"
        "  stop\n"
        "  go <theta1> <theta2> <theta3>        # relative joint move\n"
        "  goto <x> <y> <z>                     # absolute Cartesian move\n"
        "  go_trajectory <demo|square|home>\n"
        "  rotate <angle>                       # Siemens EE suction rotation (R-frame deg, verbatim [-359,359])\n"
        "  setspeed <speed>                     # Siemens conveyor speed\n"
        "  plan_siemen <rotate> <speed>         # Siemens plan\n"
        "  grab <object> <x> <y> <z> [rotate]   # manual grab sequence\n"
        "  place <object> <x> <y> <z> [rotate]  # manual place sequence\n"
        "  jog <x|y|z> <distance>               # jog axis\n"
        "  calib\n"
        "  pick\n"
        "  release\n"
        "  validate                             # validate the whole config\n"
        "  camera_tuning [args]                 # run camera calibration tool\n"
        "  speed_tuning                         # validate PLC timing model (tilted heptagon)\n"
        "  status\n"
        "  help\n"
        "  quit / exit\n"
    )


def format_status(status: dict[str, Any] | None) -> str:
    if not status:
        return "[INFO] no PLC status available"
    parts: list[str] = []
    for key, value in status.items():
        if isinstance(value, list):
            rendered = "[" + ", ".join(str(item) for item in value) + "]"
        else:
            rendered = str(value)
        parts.append(f"{key}={rendered}")
    return "[INFO] PLC status: " + ", ".join(parts)


def run_interactive(
    dispatch: Callable[[dict[str, Any]], dict[str, Any] | None],
    request_status: Callable[[], dict[str, Any] | None] | None = None,
    *,
    interpolar_points: int = INTERPOLAR_POINTS,
    prompt: str = "robot> ",
) -> None:
    print("Delta Robot CLI")
    print("Type 'help' for available commands.")

    while True:
        try:
            line = input(prompt).strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return

        if not line:
            continue

        # Calibration meta-commands need dispatch/request_status and don't map to
        # a single PLC package, so intercept them before _parse_plan.
        try:
            meta_tokens = shlex.split(line)
        except ValueError:
            meta_tokens = line.split()
        meta_cmd = meta_tokens[0].lower() if meta_tokens else ""
        if meta_cmd == "validate":
            _run_validate()
            continue
        if meta_cmd == "camera_tuning":
            _run_camera_tuning(meta_tokens[1:])
            continue
        if meta_cmd == "speed_tuning":
            _run_speed_tuning(dispatch, request_status, interpolar_points)
            continue

        try:
            plan = _parse_plan(line, interpolar_points, request_status)
        except Exception as exc:
            print(f"[ERROR] {exc}")
            continue

        if plan.quit_requested:
            return

        if line.lower() in {"?", "help"}:
            _print_help()
            continue

        if plan.show_status:
            if request_status is None:
                print("[WARN] status request is not available in this mode")
            else:
                print(format_status(request_status()))
            continue

        for package in plan.packages:
            response = dispatch(package)
            if response is not None:
                print(format_status(response))
