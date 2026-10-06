"""Top-view footprint of a board lying on the belt: which parts overlap.

A part is a flat rectangle at some orientation, in belt coordinates (u along travel, v across)
in millimetres. Used by the virtual feed to say which parts lie on top of which
(doc/basis-theory.md §7.7).
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Footprint:
    """The rectangle a part covers on the belt. `width` lies along the part's own x axis,
    turned `angle_deg` counter-clockwise from the belt's +u direction."""

    u: float
    v: float
    width: float
    height: float
    angle_deg: float = 0.0

    def axes(self) -> tuple[tuple[float, float], tuple[float, float]]:
        c, s = math.cos(math.radians(self.angle_deg)), math.sin(math.radians(self.angle_deg))
        return (c, s), (-s, c)

    def contains(self, u: float, v: float) -> bool:
        """Is the belt point (u, v) on this part?"""
        (cx, cy), (sx, sy) = self.axes()
        du, dv = u - self.u, v - self.v
        return abs(cx * du + cy * dv) <= 0.5 * self.width and abs(sx * du + sy * dv) <= 0.5 * self.height

    def overlaps(self, other: "Footprint") -> bool:
        """Do the two rectangles share area? Touching along an edge does not count. Separating
        axis test: rectangles overlap unless some edge direction of either separates them."""
        gap_u, gap_v = other.u - self.u, other.v - self.v
        for shape in (self, other):
            for ax, ay in shape.axes():
                reach = sum(abs(ax * bx + ay * by) * half
                            for box in (self, other)
                            for (bx, by), half in zip(box.axes(), (0.5 * box.width, 0.5 * box.height)))
                if abs(ax * gap_u + ay * gap_v) >= reach - 1e-9:
                    return False
        return True


def board_footprint(u: float, v: float, width: float, height: float,
                    vision_heading_deg: float) -> Footprint:
    """The footprint of a board whose width axis lies along its vision heading.

    The heading is the image-pixel angle `ConveyorFrame.vision_heading_to_robot_rad` converts;
    the belt frame (u, v) is a reflection of the robot frame, so the board's width axis points
    at 180° − heading from +u. Only the axis matters (a rectangle is symmetric under 180°).
    Whether the marker heading really is the width axis is open-issues C4."""
    return Footprint(u, v, width, height, 180.0 - vision_heading_deg)
