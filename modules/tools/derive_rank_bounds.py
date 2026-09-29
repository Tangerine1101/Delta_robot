"""Worst-case grab and occupancy times of the trajectory model, for the queue bound.

`scheduling.arm_cycle.{occupancy_worst_s, grab_worst_s}` set the admissible ceiling
v_max = L / (k * p + g) of the queue-bounded speed laws. This sweeps the delta model over a grid
of pick points in the workspace, each bin as the destination, starting from home and from every
bin, on a stationary belt, and prints the maxima. The result inherits every uncertainty of the
model (open-issues C2, C5, C7).

    python3 -m modules.tools.derive_rank_bounds [--k 1]
"""

from __future__ import annotations

import argparse

from modules.core.delta import DeltaArm
from modules.core.forecast import steady
from modules.settings import load_settings

GRID_STEPS = 8


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--k", type=float, default=1.0, help="queue depth k of the bound")
    args = parser.parse_args()

    settings = load_settings()
    arm = DeltaArm.from_settings(settings)
    u_min, u_max, v_min, v_max = settings.conveyor.workspace_window_uv
    bins = {name: spec.bin for name, spec in settings.object_types.items()}
    starts = [("home", settings.robot.home_position)] + [
        (f"bin {name}", arm.place_position(position)) for name, position in bins.items()]
    grab_worst = (0.0, "")
    occupancy_worst = (0.0, "")
    for i in range(GRID_STEPS + 1):
        u = u_min + (u_max - u_min) * i / GRID_STEPS
        for j in range(GRID_STEPS + 1):
            v = v_min + (v_max - v_min) * j / GRID_STEPS
            for start_name, start in starts:
                intercept = arm.predict(u, v, start, 0.0, steady(0.0), 0.0)
                if intercept is None:
                    continue
                for bin_name, position in bins.items():
                    grab, occupancy = arm.costs(start, 0.0, intercept, position, 0.0,
                                                settings.scheduling.setup_time_s)
                    where = f"u={u:.0f} v={v:.0f} from {start_name} to bin {bin_name}"
                    grab_worst = max(grab_worst, (grab, where))
                    occupancy_worst = max(occupancy_worst, (occupancy, where))

    band = u_max - u_min
    v_bound = band / (args.k * occupancy_worst[0] + grab_worst[0])
    print(f"grab_worst_s      = {grab_worst[0]:.3f}  ({grab_worst[1]})")
    print(f"occupancy_worst_s = {occupancy_worst[0]:.3f}  ({occupancy_worst[1]})")
    print(f"v_max = {band:.0f} / ({args.k:g} * p + g) = {v_bound:.1f} mm/s "
          f"(then capped by the speed band's ceiling {settings.speed_ceiling_mm_s():g})")


if __name__ == "__main__":
    main()
