"""A committed pick: the two packets the executor sends, and everything it logs about them."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

from modules.comm.packets import COMMAND_ID, RobotPacket
from modules.core.angles import wrap_rad
from modules.core.arm_model import Intercept
from modules.core.delta import DeltaArm
from modules.core.tracking import TrackedObject
from modules.core.trajectory import (
    GOTO_SUCTION,
    PICK_SUCTION,
    TrajectoryPoint,
    as_trajectory,
    goto_packet_times,
    pick_packet_times,
)
from modules.settings import Point, Rotation


@dataclass
class PickPlan:
    plan_id: str
    object_id: str
    object_type: str
    detected_at: float
    source_position_2d: tuple[float, float]
    cycle_start_position: Point
    assumed_speed: tuple[float, float]
    predicted_pick_time: float
    pick_dispatch_time: float
    predicted_pick_position_2d: tuple[float, float, float]
    sorting_position: Point
    trajectory_goto: list[TrajectoryPoint]
    trajectory_pick: list[TrajectoryPoint]
    status: str = "planned"
    debug_info: dict[str, Any] = field(default_factory=dict)
    # C-frame anchor used by the live pick-position gate.
    object_uv_anchor: tuple[float, float] = (0.0, 0.0)
    belt_pos_anchor: float = 0.0
    # Post-grip target for the 4th-DOF suction cup: R-frame RADIANS, [-pi, pi) (the wrap is
    # itself the shortest-way rotation). Converted to wire degrees only in the PLC worker.
    rotate_rad: float = 0.0
    # Modelled park -> contact descent time (s), logged for the T_delay calibration.
    descend_time_s: float = 0.0

    def total_duration(self) -> float:
        return sum(point.time_s for point in self.trajectory_goto + self.trajectory_pick)

    def to_robot_packets(self, interpolar_points: int) -> list[dict[str, Any]]:
        return [
            trajectory_packet(self.trajectory_goto, interpolar_points),
            trajectory_packet(self.trajectory_pick, interpolar_points),
        ]

    def to_summary(self) -> dict[str, Any]:
        return {
            "plan_id": self.plan_id,
            "object_id": self.object_id,
            "object_type": self.object_type,
            "predicted_pick_time": round(self.predicted_pick_time, 3),
            "pick_dispatch_time": round(self.pick_dispatch_time, 3),
            "predicted_pick_position_2d": [
                round(self.predicted_pick_position_2d[0], 3),
                round(self.predicted_pick_position_2d[1], 3),
                round(self.predicted_pick_position_2d[2], 3),
            ],
            "sorting_position": [round(value, 3) for value in self.sorting_position],
            "duration_s": round(self.total_duration(), 3),
            "rotate_deg": round(math.degrees(self.rotate_rad), 2),
            "status": self.status,
        }


def trajectory_packet(points: list[TrajectoryPoint], interpolar_points: int) -> dict[str, Any]:
    return RobotPacket(
        commandID=COMMAND_ID["go_trajectory"],
        argument_number=len(points),
        argument_x=[point.x for point in points],
        argument_y=[point.y for point in points],
        argument_z=[point.z for point in points],
        argument_e=[point.e for point in points],
        argument_time=[point.time_s for point in points],
    ).to_dict(interpolar_points)


def post_grip_rotation_rad(rotation: Rotation, heading_rad: float) -> float:
    """Suction angle that turns a board with R-frame heading `heading_rad` to the bin
    orientation after grip. `rotation.sign` flips the command if the axis turns opposite to
    the R-frame CCW convention (open-issues C1); the wrap to [-pi, pi) is the shortest way."""
    sign = 1.0 if rotation.sign >= 0.0 else -1.0
    return wrap_rad(sign * (math.radians(rotation.offset_deg) - heading_rad))


def build_pick_plan(
    arm: DeltaArm,
    plan_id: str,
    obj: TrackedObject,
    bin_position: Point,
    intercept: Intercept,
    start_position: Point,
    belt_position_mm: float,
    belt_speed_mm_s: float,
    assumed_speed: tuple[float, float],
    interpolar_points: int,
) -> PickPlan:
    """Turn an intercept into the goto and pick packets.

    The arm parks above the intercept at pre-pick height; only the pick-phase contact is
    shifted downstream when the oblique descent is on (DeltaArm.contact_position)."""
    pick_position = intercept.pick_position
    contact_position, descend_time_s = arm.contact_position(pick_position, belt_speed_mm_s)
    packet_time = arm.robot.packet_time
    goto_points = arm.goto_points(start_position, pick_position)
    trajectory_goto = as_trajectory(goto_points, goto_packet_times(start_position, goto_points, packet_time),
                                    GOTO_SUCTION)
    pick_points = arm.pick_points(contact_position, bin_position)
    trajectory_pick = as_trajectory(pick_points, pick_packet_times(pick_points, goto_points, packet_time),
                                    PICK_SUCTION)
    rotate_rad = post_grip_rotation_rad(arm.robot.rotation, obj.rotation_rad)

    return PickPlan(
        plan_id=plan_id,
        object_id=obj.object_id,
        object_type=obj.object_type,
        detected_at=obj.last_seen_at,
        source_position_2d=obj.conveyor_uv,
        cycle_start_position=start_position,
        assumed_speed=assumed_speed,
        predicted_pick_time=intercept.pick_time,
        pick_dispatch_time=intercept.gate_fire_time,
        predicted_pick_position_2d=(pick_position[0], pick_position[1], pick_position[2]),
        sorting_position=bin_position,
        trajectory_goto=trajectory_goto,
        trajectory_pick=trajectory_pick,
        object_uv_anchor=obj.conveyor_uv,
        belt_pos_anchor=belt_position_mm,
        rotate_rad=rotate_rad,
        descend_time_s=descend_time_s,
        debug_info={
            "pick_position_3d": pick_position,
            "contact_position_3d": contact_position,
            "descend_time_s": descend_time_s,
            # [ROTATE] calibration log inputs (degrees for readability).
            "vision_angle_deg": round(obj.vision_angle_deg, 2),
            "board_heading_deg": round(math.degrees(obj.rotation_rad), 2),
            "rotate_cmd_deg": round(math.degrees(rotate_rad), 2),
            "timing_formula": {
                "t_p_real": intercept.gate_fire_time,
                "t_p_theory": intercept.pick_time,
                "robot_movement_delay_s": arm.gate.robot_movement_delay_s,
                "ethernet_delay_s": arm.gate.ethernet_delay_s,
            },
            # Interpolator-model phase times (mirror the PLC, unlike argument_time).
            "modeled_goto_s": arm.trajectory_time(goto_points),
            "modeled_pick_s": arm.trajectory_time(pick_points),
            "robot_packets": [
                trajectory_packet(trajectory_goto, interpolar_points),
                trajectory_packet(trajectory_pick, interpolar_points),
            ],
        },
    )
