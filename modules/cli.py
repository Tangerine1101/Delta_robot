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


PRESET_TRAJECTORIES: dict[str, list[dict[str, float]]] = {
    "demo": [
        {"x": 100.0, "y": 0.0, "z": -220.0, "e": 0.0, "time": 0.5},
        {"x": 120.0, "y": 20.0, "z": -220.0, "e": 0.0, "time": 0.5},
        {"x": 140.0, "y": 0.0, "z": -220.0, "e": 0.0, "time": 0.5},
        {"x": 100.0, "y": 0.0, "z": -220.0, "e": 0.0, "time": 0.5},
    ],
    "square": [
        {"x": 80.0, "y": 80.0, "z": -230.0, "e": 0.0, "time": 0.4},
        {"x": 120.0, "y": 80.0, "z": -230.0, "e": 0.0, "time": 0.4},
        {"x": 120.0, "y": 120.0, "z": -230.0, "e": 0.0, "time": 0.4},
        {"x": 80.0, "y": 120.0, "z": -230.0, "e": 0.0, "time": 0.4},
    ],
    "home": [
        {"x": 0.0, "y": 0.0, "z": -200.0, "e": 0.0, "time": 0.6},
    ],
}


def _pad(values: Iterable[float], interpolar_points: int = INTERPOLAR_POINTS) -> list[float]:
    items = list(values)[:interpolar_points]
    if len(items) < interpolar_points:
        items.extend([0.0] * (interpolar_points - len(items)))
    return items


def _zero_command(command_name: str, interpolar_points: int = INTERPOLAR_POINTS) -> dict[str, Any]:
    return RobotPacket(
        commandID=COMMAND_ID[command_name],
        argument_number=0,
        argument_x=[0.0] * interpolar_points,
        argument_y=[0.0] * interpolar_points,
        argument_z=[0.0] * interpolar_points,
        argument_e=[0.0] * interpolar_points,
        argument_time=[0.0] * interpolar_points,
    ).to_dict(interpolar_points)


def _trajectory_command(
    name: str,
    points: list[dict[str, float]],
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
        argument_e=_pad((point.get("e", 0.0) for point in points), interpolar_points),
        argument_time=_pad((point["time"] for point in points), interpolar_points),
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
        argument_x=_pad([theta1], interpolar_points),
        argument_y=_pad([theta2], interpolar_points),
        argument_z=_pad([theta3], interpolar_points),
        argument_e=[0.0] * interpolar_points,
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
        argument_x=_pad([x], interpolar_points),
        argument_y=_pad([y], interpolar_points),
        argument_z=_pad([z], interpolar_points),
        argument_e=[0.0] * interpolar_points,
        argument_time=[0.0] * interpolar_points,
    ).to_dict(interpolar_points)


def _parse_plan(line: str, interpolar_points: int = INTERPOLAR_POINTS) -> CommandPlan:
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

    raise ValueError(f"Unknown command: {command}")


def _print_help() -> None:
    print(
        "\nCommands:\n"
        "  stop\n"
        "  go <theta1> <theta2> <theta3>        # relative joint move\n"
        "  goto <x> <y> <z>                     # absolute Cartesian move\n"
        "  go_trajectory <demo|square|home>\n"
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
    return "[INFO] PLC status: " + ", ".join(f"{key}={value}" for key, value in status.items())


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
            plan = _parse_plan(line, interpolar_points)
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
