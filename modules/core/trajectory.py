"""The two 7-point templates every pick is flown with (basis-programming.md §4).

`goto`: lift from where the arm is, travel at clearance height, descend to the park point
above the part (pre-pick height). `pick`: drop to the part (suction on), lift, travel, descend
over the bin and release. Both end with a vertical segment, which the pick executor's
final-segment arrival check relies on (open-issues O2).

`argument_time` values are a rough constant-velocity ETA (`robot.packet_time`): the Omron
ignores them, but the pick executor sums them for its arrival timeout.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from modules.settings import PacketTime, Point, RobotSettings

GOTO_SUCTION = (0, 0, 0, 0, 0, 0, 0)
PICK_SUCTION = (1, 1, 1, 1, 1, 1, 0)   # on from contact, off on the final descent over the bin


@dataclass(frozen=True)
class TrajectoryPoint:
    x: float
    y: float
    z: float
    e: int          # suction on/off during the segment ending here
    time_s: float   # argument_time of the segment

    def to_dict(self) -> dict[str, Any]:
        return {"x": self.x, "y": self.y, "z": self.z, "e": self.e, "time": self.time_s}


def _unit_and_blend(start: Point, end: Point, corner_blend_xy: float) -> tuple[float, float, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    d = math.hypot(dx, dy)
    if d > 0.001:
        u_x, u_y = dx / d, dy / d
    else:
        u_x = u_y = 0.0
    return u_x, u_y, min(d * 0.2, corner_blend_xy)


def goto_waypoints(start: Point, park: Point, robot: RobotSettings,
                   pre_pick_z: float | None = None) -> list[Point]:
    """Goto template from `start` to the park point above `park` (its XY is used)."""
    h = robot.heights
    u_x, u_y, blend = _unit_and_blend(start, park, robot.corner_blend_xy)
    park_z = pre_pick_z if pre_pick_z is not None else h.pre_pick
    return [
        # P1: vertical lift from start to slope_transition
        (start[0], start[1], h.slope_transition),
        # P2: diagonal slope up to clearance (XY moves blend toward the pick)
        (start[0] + u_x * blend, start[1] + u_y * blend, h.clearance),
        # P3: flat at clearance, blend zone after the slope
        (start[0] + u_x * 2 * blend, start[1] + u_y * 2 * blend, h.clearance),
        # P4: flat at clearance, approaching the pick zone
        (park[0] - u_x * 2 * blend, park[1] - u_y * 2 * blend, h.clearance),
        # P5: flat at clearance, blend zone before the descent
        (park[0] - u_x * blend, park[1] - u_y * blend, h.clearance),
        # P6: diagonal slope down to slope_transition (XY reaches the pick)
        (park[0], park[1], h.slope_transition),
        # P7: vertical descent to pre-pick
        (park[0], park[1], park_z),
    ]


def pick_waypoints(contact: Point, bin_position: Point, robot: RobotSettings,
                   place_z: float | None = None) -> list[Point]:
    """Pick template from the contact point to the release point over the bin."""
    h = robot.heights
    u_x, u_y, blend = _unit_and_blend(contact, bin_position, robot.corner_blend_xy)
    release_z = place_z if place_z is not None else h.place
    return [
        # P1: descend to pickup height (suction ON)
        contact,
        # P2: vertical lift to slope_transition
        (contact[0], contact[1], h.slope_transition),
        # P3: diagonal slope up to clearance (XY moves blend toward the bin)
        (contact[0] + u_x * blend, contact[1] + u_y * blend, h.clearance),
        # P4: flat at clearance, approaching the bin
        (bin_position[0] - u_x * 2 * blend, bin_position[1] - u_y * 2 * blend, h.clearance),
        # P5: flat at clearance, blend zone before the descent into the bin
        (bin_position[0] - u_x * blend, bin_position[1] - u_y * blend, h.clearance),
        # P6: diagonal slope down to slope_transition over the bin
        (bin_position[0], bin_position[1], h.slope_transition),
        # P7: vertical descent to the place height (suction OFF)
        (bin_position[0], bin_position[1], release_z),
    ]


def segment_eta_s(start: Point, end: Point, packet_time: PacketTime) -> float:
    """Rough constant-velocity ETA of one segment, for `argument_time` only."""
    horizontal = math.hypot(end[0] - start[0], end[1] - start[1])
    vertical = abs(end[2] - start[2])
    xy = packet_time.nominal_xy_speed
    z = packet_time.nominal_z_speed
    return max(0.08, horizontal / xy if xy > 0.0 else 0.0, vertical / z if z > 0.0 else 0.0)


def goto_packet_times(start: Point, points: list[Point], packet_time: PacketTime) -> list[float]:
    times: list[float] = []
    previous = start
    for point in points:
        times.append(segment_eta_s(previous, point, packet_time))
        previous = point
    return times


def pick_packet_times(points: list[Point], goto_points: list[Point],
                      packet_time: PacketTime) -> list[float]:
    """argument_time of the pick packet; its first segment starts at the park point (the
    last goto waypoint) and its last one lasts at least the release descent time."""
    times: list[float] = []
    previous = points[0]
    for index, point in enumerate(points):
        if index == 0:
            times.append(segment_eta_s(goto_points[-1], point, packet_time))
        elif index == len(points) - 1:
            times.append(max(segment_eta_s(previous, point, packet_time),
                             packet_time.release_descent_time_s))
        else:
            times.append(segment_eta_s(previous, point, packet_time))
        previous = point
    return times


def as_trajectory(points: list[Point], times: list[float], suction: tuple[int, ...]) -> list[TrajectoryPoint]:
    return [TrajectoryPoint(p[0], p[1], p[2], e, t) for p, e, t in zip(points, suction, times)]

