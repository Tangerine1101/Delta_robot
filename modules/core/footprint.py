"""Top-view footprint of a board lying on the belt: which part's centre lies under which.

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

    def contains(self, u: float, v: float) -> bool:
        """Is the belt point (u, v) on this part? Used for "is this part's centre covered?"."""
        c, s = math.cos(math.radians(self.angle_deg)), math.sin(math.radians(self.angle_deg))
        du, dv = u - self.u, v - self.v
        return abs(c * du + s * dv) <= 0.5 * self.width and abs(-s * du + c * dv) <= 0.5 * self.height


def board_footprint(u: float, v: float, width: float, height: float,
                    vision_heading_deg: float) -> Footprint:
    """The footprint of a board whose width axis lies along its vision heading.

    The heading is the image-pixel angle `ConveyorFrame.vision_heading_to_robot_rad` converts;
    the belt frame (u, v) is a reflection of the robot frame, so the board's width axis points
    at 180° − heading from +u. Only the axis matters (a rectangle is symmetric under 180°).
    Whether the marker heading really is the width axis is open-issues C4."""
    return Footprint(u, v, width, height, 180.0 - vision_heading_deg)
