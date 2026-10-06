"""Geometry for the dashboard's animated cell view, in belt coordinates (u along the belt, v
across it, mm).

`layout()` is sent once per run (the fixed scene: windows, bins, part sizes, the arm's base);
`arm_geometry()` goes with every status (the three arm chains, from the PLC's own inverse
kinematics, so the drawing bends the way the real arm does).
"""

from __future__ import annotations

import math
from typing import Any

from modules.core import kinematics as k
from modules.core.frames import ConveyorFrame
from modules.settings import Settings

# Arm i works in a vertical plane turned by ARM_TURN_DEG[i] about the base axis (the ST's
# Calc_Inverse_Kinematics rotates the target by -120° / +120° for arms 2 / 3).
ARM_TURN_DEG = (0.0, 120.0, -120.0)
BASE_JOINT_R = 0.5 * k.TAN30 * k.BASE          # base centre -> shoulder joint (mm)
EFFECTOR_JOINT_R = 0.5 * k.TAN30 * k.END_EFFECTOR


def _turn(x: float, y: float, deg: float) -> tuple[float, float]:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return x * c - y * s, x * s + y * c


def _uv(frame: ConveyorFrame, x: float, y: float) -> list[float]:
    u, v = frame.to_conveyor(x, y)
    return [round(u, 1), round(v, 1)]


def layout(settings: Settings, frame: ConveyorFrame) -> dict[str, Any]:
    conveyor = settings.conveyor
    cam, ws = conveyor.camera_window_uv, conveyor.workspace_window_uv
    heights = settings.robot.heights
    return {
        "camera_window_uv": list(cam),
        "workspace_window_uv": list(ws),
        "belt_v": [min(cam[2], ws[2]), max(cam[3], ws[3])],
        "bins": {name: _uv(frame, spec.bin[0], spec.bin[1]) for name, spec in settings.object_types.items()},
        "sizes": {name: [spec.w, spec.h] for name, spec in settings.object_types.items()},
        "base": _uv(frame, 0.0, 0.0),
        "shoulders": [_uv(frame, *_turn(0.0, -BASE_JOINT_R, turn)) for turn in ARM_TURN_DEG],
        "z": {"pickup": heights.pickup, "clearance": heights.clearance,
              "min": settings.robot.limits.z_min_mm, "max": settings.robot.limits.z_max_mm},
        "belt_theta_deg": conveyor.frame.theta_deg,
    }


def arm_geometry(frame: ConveyorFrame, pose: tuple[float, float, float]) -> dict[str, Any]:
    """Cup centre, elbows (with z) and effector joints for one pose; elbows are omitted when
    the pose has no IK solution."""
    x, y, z = pose
    elbows, wrists = [], []
    for turn in ARM_TURN_DEG:
        lx, ly = _turn(x, y, -turn)
        failed, theta = k.calc_angles_yz(lx, ly, z)
        wx, wy = _turn(0.0, -EFFECTOR_JOINT_R, turn)
        wrists.append(_uv(frame, x + wx, y + wy))
        if failed or theta is None:
            continue
        rad = math.radians(theta)
        ex, ey = _turn(0.0, -BASE_JOINT_R - k.BICEP * math.cos(rad), turn)
        elbows.append([*_uv(frame, ex, ey), round(-k.BICEP * math.sin(rad), 1)])
    return {"c": [*_uv(frame, x, y), round(z, 1)], "elbows": elbows, "wrists": wrists}
