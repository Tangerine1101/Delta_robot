# `config.yaml` Checked Against the Deployed PLC

> `modules/config.yaml` was tuned by hand for a demo run. This file re-derives the values
> that the Omron program (`Matching_Code_10`) constrains, and proposes corrections.
> **Nothing here has been applied**: the user edits the config.
>
> **Method.** IK/FK and trajectory timing come from the scan-accurate port
> [`modules/plc_sim/omron_core.py`](../../modules/plc_sim/omron_core.py), with an ideal servo.
> The Python templates are `core/trajectory.goto_waypoints` and `pick_waypoints` with the current
> config. The worked example is:
> - start at the `TQFP` bin;
> - pick at the workspace centre `uv (276, 62)` → robot `(−17.1, −22.1)`.
>
> The open issues these results feed are in [`../open-issues.md`](../open-issues.md).

## 1. Geometry and reach

The PLC geometry constants are:

| Constant | Value |
|---|---|
| `Base` | 346.4 mm (inradius 100 mm) |
| `EndEffector` | 86.6 mm (inradius 25 mm) |
| `Bicep` | 140 mm |
| `Forearm` | 315 mm |

IK→FK round trip error is < 2·10⁻⁵ mm.

**Reach per Z.** This is the largest XY radius reachable in every heading, with the −20°
joint limit included:

| Height used by the config | z (mm) | Worst-heading reach | Binding limit |
|---|---|---|---|
| `clearance_height` | −260 | **140 mm** (headings 30°, 150°, 270°) | joint −20° |
| (for comparison) | −265 | 170 mm | joint −20° |
| (for comparison) | −268 | 277 mm | none below 270 mm |
| `slope_transition_height` | −275 | 273 mm | no IK solution |
| `place_height` | −280 | 270 mm | no IK solution |
| `pre_pick_height` | −287 | 266 mm | no IK solution |
| `home_position` z | −290 | 264 mm | no IK solution |
| `pickup_height` | −303 | 255 mm | no IK solution |

The centre column (X = Y = 0) is reachable for z ∈ [−350, −190.5].

**Named points.** All of them are reachable today:
- both bins: `QFP` at r = 150 mm, `TQFP` at r = 127 mm;
- `home_position`;
- the three `accuracy_points_uv`;
- the four corners and the centre of `workspace_window_uv`, at both pickup and clearance
  height.

The tightest is the window corner `uv (188, 0)` at clearance height, with θ1 = −18.0°, 2°
from the joint limit.

## 2. Parameter table

| Key | Current | Derived from the PLC | Proposed | Reason |
|---|---|---|---|---|
| `robot_limits.radius_xy_mm` | 180 | Reach is ≥ 255 mm at pickup height, but only **140 mm** at `clearance_height = −260` | Keep 180 **and** lower `clearance_height`, **or** add a PC-side IK check of every waypoint | A clearance waypoint between 140 and 180 mm in the bad headings trips the joint limit, which the PLC does not report (O1) |
| `scheduler.clearance_height` | −260 | Joint limit binds for z ≥ −265 | **−265** (worst-heading reach 170 mm, ≥ every current point) | Keeps 38 mm of clearance over the pickup height; the slope point −275 stays between clearance and place |
| `scheduler.interpolator.v_max / a_max / d_max` | 300 / 1000 / 1000 | Equal to `ICV_Vmax/Amax/Dmax` initial values (Cartesian mm/s, mm/s²) | unchanged | Matches the PLC. Physical validity is still open (C5) |
| `scheduler.interpolator.soft_start_s` | 0.08 | State 10 = 20 scans × 4 ms | unchanged | Matches |
| `scheduler.interpolator.scurve_shape_factor` | 1.5 | Used by the PLC only for a rest-to-rest last segment, which a 7-point chain never runs | unchanged (inert) | Harmless |
| `pick_arrival_tolerance_mm / _max_mm` | 15 / 50 | After the executor accepts arrival, instances 0–4 are still busy for 12–36 ms (15 mm) and 160–172 ms (50 mm): a dispatch in that window triggers O2 | Accept arrival only once the pose is **inside the final vertical segment** (z below `slope_transition_height` at the target XY), and keep the tolerance for XY only | Safety (O2), independent of the value of the band (G6) |
| `pick_cycle_s` | 3.0 | Simulated arm motion per pick at the workspace centre: 0.88 s goto + 0.94 s pick = **1.8 s**, before gate wait and dispatch latency | Measure; expect 2.0–2.4 s | Feeds μ_max (G2). 3.0 s is conservative |
| `robot_movement_delay_s` | 0.17 | Dispatch → contact on the PLC: 1 scan dispatch + 20-scan ramp = 84 ms commanded, plus servo lag and write time (not measured) | Calibrate from `[GATE]` (C2) | Plausible if the write takes ~40 ms and the servo lags ~40 ms |
| `nominal_xy_speed`, `nominal_z_speed` | 220 / 220 | Only set `argument_time`, which the PLC does not read; they also set the arrival deadline `Σ argument_time + execution_margin_s` | Derive the deadline from the PLC model instead | No physical meaning on this PLC (L1) |
| `release_descent_time_s` | 0.14 | The PLC releases the vacuum 0.5 × (previous last-segment time) ≈ 0.13 s after the last segment starts | unchanged (informational) | Only feeds `argument_time` |
| `home_position` | (0, 0, −290) | θ = (21.4, 21.4, −21.4)° | unchanged | Reachable |

## 3. Trajectory-time model vs the PLC

| Phase | Python `core/motion.trajectory_time` | PLC motion ends | PLC chain idle |
|---|---|---|---|
| goto | 1.027 s | 0.880 s | 1.100 s |
| pick | 1.120 s | 0.944 s | 1.196 s |

**Why motion ends early.** The Python model books the full planned time of the last segment
(0.26 s for a 5 mm or 12 mm descent). The PLC plans that time but covers the distance in
about 20 ms because of the braking-sign fault (P7). So the arm arrives 0.15–0.18 s earlier
than the model predicts.

**Why the chain stays busy late.** It ends about 70 ms after the model, because of the
junction stalls (P12) and the idle tail of the last segment.

The processing-time estimate of the new scheduler should come from the simulator core, not
from this model (open issue G8).
