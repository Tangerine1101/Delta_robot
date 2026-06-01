from __future__ import annotations

import shlex
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
    if command == "pick":
        return CommandPlan(packages=[_zero_command("pick", interpolar_points)])
    if command == "release":
        return CommandPlan(packages=[_zero_command("release", interpolar_points)])
    if command == "rotate":
        if len(tokens) != 2:
            raise ValueError("rotate expects 1 angle value: rotate <angle>")
        return CommandPlan(
            packages=[{"commandID": COMMAND_ID["rotate_absolute"], "CommandID": COMMAND_ID["rotate_absolute"], "rotate": float(tokens[1]), "speed": 0.0}]
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
                "rotate": rotate_angle,
                "speed": 0.0
            })
        packages.append(_cartesian_command("goto_absolute", x, y, clearance, interpolar_points))
        packages.append(_cartesian_command("goto_absolute", x, y, z, interpolar_points))
        packages.append(_zero_command("pick", interpolar_points))
        packages.append(_cartesian_command("goto_absolute", x, y, clearance, interpolar_points))
        return CommandPlan(packages=packages)
    if command == "place":
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
                "rotate": rotate_angle,
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
    if command == "snap":
        print("[INFO] snap command is a placeholder (empty)")
        return CommandPlan(packages=[])
    if command == "autocollect":
        print("[INFO] autocollect command is a placeholder (empty)")
        return CommandPlan(packages=[])

    raise ValueError(f"Unknown command: {command}")


def _print_help() -> None:
    print(
        "\nCommands:\n"
        "  stop\n"
        "  go <theta1> <theta2> <theta3>        # relative joint move\n"
        "  goto <x> <y> <z>                     # absolute Cartesian move\n"
        "  go_trajectory <demo|square|home>\n"
        "  rotate <angle>                       # Siemens EE suction cup rotation\n"
        "  setspeed <speed>                     # Siemens conveyor speed\n"
        "  plan_siemen <rotate> <speed>         # Siemens plan\n"
        "  grab <object> <x> <y> <z> [rotate]   # manual grab sequence\n"
        "  place <object> <x> <y> <z> [rotate]  # manual place sequence\n"
        "  jog <x|y|z> <distance>               # jog axis\n"
        "  snap [filename]                      # snap photo (placeholder)\n"
        "  autocollect                          # auto data collection (placeholder)\n"
        "  calib\n"
        "  pick\n"
        "  release\n"
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
