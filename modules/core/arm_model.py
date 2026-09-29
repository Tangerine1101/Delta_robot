"""What the scheduler needs to know about an arm, as one small interface.

A scheduling plugin never sees the arm: it sees jobs, whose release times, deadlines and
processing times an `ArmModel` computed. Two uses:

* the **model** — what the scheduler believes (`predict`, `costs`, `place_position`);
* the **plant** — how a simulated arm actually executes a committed pick (`goto_time_s`,
  `contact_delay_s`, `pick_time_s`), used by the sandbox. Running the plant with different
  parameters from the model measures the cost of a wrong prediction.

`modules/core/delta.py` is the real cell (used by the robot and the sandbox);
`sandbox/models/` holds abstract arms for research.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from modules.core.forecast import BeltForecast
from modules.settings import Point


@dataclass(frozen=True)
class Intercept:
    """Where and when the arm meets a part, if it is dispatched now."""

    pick_time: float                # predicted cup contact (s, same clock as the caller)
    gate_fire_time: float           # when the pick phase is dispatched
    pick_position: Point            # park/contact XY at pickup height
    u_pick: float                   # belt coordinate of the contact
    goto_s: float                   # modelled flight to the park point


class ArmModel(Protocol):
    home: Point
    gate_lead_s: float              # the pick command is dispatched this long before contact

    # ---- model: what the scheduler believes -------------------------------------------
    def predict(self, u_at_anchor: float, v: float, start: Point, start_time: float,
                forecast: BeltForecast, anchor_time: float) -> Intercept | None:
        """Earliest feasible pick of a part at belt position (u, v) at `anchor_time`, if the
        arm leaves `start` at `start_time`. None when it cannot be picked in the workspace."""

    def costs(self, start: Point, start_time: float, intercept: Intercept, bin_position: Point,
              belt_speed_at_pick: float, setup_s: float) -> tuple[float, float]:
        """(grab_s, occupancy_s): time to cup contact, and until the arm is free over the bin."""

    def place_position(self, bin_position: Point) -> Point:
        """Where the arm is once it has released a part into `bin_position`."""

    # ---- plant: how a simulated arm executes a committed pick -------------------------
    def goto_time_s(self, start: Point, park: Point) -> float:
        """Flight from `start` to the park point above `park`."""

    def contact_delay_s(self, belt_speed: float) -> float:
        """Pick dispatch -> cup contact."""

    def pick_time_s(self, park: Point, bin_position: Point, belt_speed: float) -> float:
        """Pick dispatch -> arm free over the bin."""
