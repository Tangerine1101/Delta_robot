# Basis Theory — Delta Robot Pick-and-Place

> **Scope**: The theoretical framework behind every algorithm in this repository — coordinate
> transforms, kinematics, trajectory profiles, tracking/interception, orientation resolution,
> and the adaptive conveyor speed law.
> **Companions**: [`basis-programming.md`](basis-programming.md) (how the program implements
> this), [`context.md`](context.md) (AI onboarding), [`open-issues.md`](open-issues.md)
> (what is still unresolved), [`decision-log.md`](decision-log.md) (superseded designs and
> why they were replaced).
> **Numeric policy**: formulas here are symbolic; each symbol maps to a `modules/config.yaml`
> key (tables below). Live values belong to the config, not this document.
> **Status policy**: this document describes the system **as it currently behaves**. It
> contains no history and no roadmap — those live in `decision-log.md` and
> `open-issues.md` respectively. A parameter appearing here is *not* evidence that it has
> been calibrated; check `open-issues.md` §A before trusting a number.

---

## 1. Coordinate Frames & Spatial Transformations

Three Cartesian frames map camera pixels to physical robot targets:

```mermaid
graph TD
    V[Vision frame: p_x, p_y pixels] -->|Homography H| C[Conveyor C-frame: u, v]
    C -->|Homogeneous transform F| R[Robot R-frame: X_R, Y_R, Z_R]
```

1. **Robot frame (R-frame)** — centered at the upper fixed base plate, $+Z$ up, right-hand
   rule. The parallel linkage constrains the end-effector to $Z < 0$ ($Z = 0$: arms fully
   retracted; $Z \approx -305$ mm: end-effector at the belt surface).
2. **Conveyor frame (C-frame)** — 2D planar frame on the belt surface: $+u$ downstream
   (belt flow), $+v$ transverse. Origin $O_C$ is a fixed physical point chosen at calibration.
3. **Vision frame (V-frame)** — 2D pixel space $(p_x, p_y)$ of the camera.

### 1.1. Homogeneous transform F (C → R)

The belt surface is planar and parallel to the robot $XY$ plane at constant pickup height
$Z_{\text{pickup}}$, so a 2D homogeneous transform suffices:

$$
\begin{bmatrix} X_R \\ Y_R \\ 1 \end{bmatrix}
= \mathbf{F}
\begin{bmatrix} u \\ v \\ 1 \end{bmatrix}
=
\begin{bmatrix}
-\sin\theta & \cos\theta & T_X \\
\cos\theta & \sin\theta & T_Y \\
0 & 0 & 1
\end{bmatrix}
\begin{bmatrix} u \\ v \\ 1 \end{bmatrix}
$$

* $\theta$: angle between the belt flow axis ($+u$) and the robot axes
  (`conveyor.frame.theta_deg`).
* $(T_X, T_Y)$: translation of the conveyor origin (`conveyor.frame.robot_origin_uv`).

Implementation: `ConveyorFrame` in `modules/core/frames.py`; calibration procedure via the
`test_vision_only` scenario (see `basis-programming.md`).

### 1.2. Planar homography H (V → C)

For the flat belt surface, pixels map to C-frame via a homography:

$$
\lambda \begin{bmatrix} u \\ v \\ 1 \end{bmatrix}
= \mathbf{H} \begin{bmatrix} p_x \\ p_y \\ 1 \end{bmatrix}
$$

In the deployed system the camera is mounted square to the belt, so $\mathbf{H}$ degenerates
into an axis swap + uniform scale (`vision.pixels_per_mm`, ROI offsets) — see
`M_VISION_TO_CONVEYOR` in `modules/core/frames.py`.

---

## 2. Kinematics of the Delta Mechanism

The kinematics run **on the Omron PLC**, not the PC — this section is the reference model.
What the PLC code does (limits, sign conventions, error cases) is in
[`plc/kinematics.md`](plc/kinematics.md).

Geometric parameters: $s_b$ base triangle side (346.4 mm), $s_p$ platform side (86.6 mm),
$L$ bicep length (140.0 mm), $l$ forearm length (315.0 mm).

### 2.1. Inverse kinematics (IK)

Given $(X_0, Y_0, Z_0)$, find joint angles $(\theta_1, \theta_2, \theta_3)$. By the 120°
rotational symmetry, the 3D problem reduces to three identical 2D single-arm solvers in a
local $YZ$-plane.

**Single-arm 2D solver** (`Calc_Angles_YZ`): the base joint sits at
$y_1 = -\frac{s_b}{2\sqrt{3}}$ and the platform connection at
$y_{\text{tmp}} = Y_0 - \frac{s_p}{2\sqrt{3}}$. The elbow $(0, y_j, z_j)$ satisfies

1. Bicep circle: $(y_j - y_1)^2 + z_j^2 = L^2$
2. Forearm sphere: $X_0^2 + (y_{\text{tmp}} - y_j)^2 + (Z_0 - z_j)^2 = l^2$

Subtracting gives the line $z_j = a + b\,y_j$ with

$$a = \frac{X_0^2 + y_{\text{tmp}}^2 + Z_0^2 + L^2 - l^2 - y_1^2}{2Z_0},
\qquad b = \frac{y_1 - y_{\text{tmp}}}{Z_0}$$

Substituting back yields $A y_j^2 + B y_j + C = 0$ with $A = 1 + b^2$, $B = 2(ab - y_1)$,
$C = y_1^2 + a^2 - L^2$, whose half-discriminant is

$$d = -(a + b\,y_1)^2 + L^2(b^2 + 1)$$

$d < 0$ ⇒ unreachable. The physical "elbow-down" root is

$$y_j = \frac{y_1 - ab - \sqrt{d}}{b^2 + 1}, \qquad z_j = a + b\,y_j$$

$$\theta = \frac{180}{\pi}\arctan\!\left(\frac{-z_j}{y_1 - y_j}\right) + \theta_{\text{offset}},
\qquad \theta_{\text{offset}} = \begin{cases}180^\circ & y_j > y_1\\ 0 & \text{else}\end{cases}$$

**Top-level coordinator**: rotate the target $(X, Y)$ by ±120° for arms 2 and 3
($X_2 = X\cos 120^\circ + Y\sin 120^\circ$, $Y_2 = -X\sin 120^\circ + Y\cos 120^\circ$;
arm 3 mirrored), with $\theta_3$ negated for the inverted motor direction.

### 2.2. Forward kinematics (FK)

Given $(\theta_1, \theta_2, \theta_3)$, find $(X_0, Y_0, Z_0)$ as the intersection of three
spheres of radius $l$ centered at the elbows. With $w = \frac{s_b - s_p}{2\sqrt{3}}$:

$$\mathbf{J}_1 = \begin{pmatrix} 0 \\ -(w + L\cos\theta_1) \\ -L\sin\theta_1 \end{pmatrix},\;
\mathbf{J}_2 = \begin{pmatrix} (w + L\cos\theta_2)\cos 30^\circ \\ (w + L\cos\theta_2)\sin 30^\circ \\ -L\sin\theta_2 \end{pmatrix},\;
\mathbf{J}_3 = \begin{pmatrix} -(w + L\cos\theta_3)\cos 30^\circ \\ (w + L\cos\theta_3)\sin 30^\circ \\ -L\sin\theta_3 \end{pmatrix}$$

Subtracting sphere 1 from spheres 2 and 3 eliminates the quadratic terms, giving a linear
system solved by Cramer's rule for $X_0 = a_1 + b_1 Z_0$, $Y_0 = a_2 + b_2 Z_0$; substituting
back into sphere 1 yields the quadratic $A_q Z_0^2 + B_q Z_0 + C_q = 0$ with

$$A_q = b_1^2 + b_2^2 + 1,\quad
B_q = 2(a_1 b_1 + a_2 b_2 - y_{j1} b_2 - z_{j1}),\quad
C_q = a_1^2 + a_2^2 - 2 a_2 y_{j1} + r_1 - l^2$$

The physical (lower) root: $Z_0 = \frac{-B_q - \sqrt{B_q^2 - 4 A_q C_q}}{2 A_q}$.

---

## 3. Trajectory Profiles

The PLC executes the trajectory; the PC's time model (`modules/core/motion.py`,
`robot.interpolator.*` config) mirrors the same model to *predict segment timing* for
gate leads and ETAs. The PLC function blocks themselves are described in
[`plc/motion-fbs.md`](plc/motion-fbs.md); the target PLC program's profile differs from the
model below (no corner look-ahead) — see `open-issues.md` §C.1.

### 3.1. Polynomial S-curve profile (jerk-bounded)

For stationary endpoints ($V_{\text{start}} = V_{\text{end}} = 0$), a 4th-order smoothstep
position profile limits jerk. With $\tau = t / t_{\text{acc}}$:

$$S(\tau) = V_{\max}\, t_{\text{acc}} \left(\tau^3 - \tfrac{1}{2}\tau^4\right), \qquad
v(\tau) = V_{\max}(3\tau^2 - 2\tau^3), \qquad
a(\tau) = \frac{6 V_{\max}}{t_{\text{acc}}}\,\tau(1 - \tau)$$

**Shape compensation factor**: the parabolic acceleration peaks at $\tau = 0.5$ with
$a_{\text{peak}} = 1.5 V_{\max} / t_{\text{acc}}$. Requiring $a_{\text{peak}} = A_{\max}$
gives $t_{\text{acc}} = 1.5\, V_{\max} / A_{\max}$ — the **1.5× factor** relative to a linear
ramp (`interpolator.scurve_shape_factor`).

### 3.2. Blended corner velocity

Traversing waypoint $\mathbf{B}$ (from $\mathbf{A}$, toward $\mathbf{C}$) without stopping,
the corner velocity is bounded by the direction-change angle $\Theta$:

$$\cos\Theta = \frac{(\mathbf{B}-\mathbf{A}) \cdot (\mathbf{C}-\mathbf{B})}
{\|\mathbf{B}-\mathbf{A}\|\,\|\mathbf{C}-\mathbf{B}\|},
\qquad
V_{\text{corner}} = V_{\max} \cos\frac{\Theta}{2} = V_{\max}\sqrt{\frac{\cos\Theta + 1}{2}}$$

This bounds centripetal acceleration and mechanical shock at corner transitions
(`corner_blend_xy`).

| Symbol | Config key (`robot.`) |
|---|---|
| $V_{\max}$, $A_{\max}$, $D_{\max}$ | `interpolator.v_max` / `.a_max` / `.d_max` |
| soft-start dwell | `interpolator.soft_start_s` |
| 1.5× shape factor | `interpolator.scurve_shape_factor` |
| coarse ETA speeds written into `argument_time` (NOT the timing model) | `packet_time.nominal_xy_speed`, `.nominal_z_speed` |

---

## 4. Object Tracking & Interception

### 4.1. Encoder-anchored dead-reckoning

Integrating velocity over time drifts and fails under speed changes. Instead, each detected
object is anchored to the absolute belt encoder position $p(t)$ (mm, from the Siemens PLC)
at its detection instant:

$$u(t) = u_{\text{anchor}} + \big(p(t) - p_{\text{anchor}}\big)$$

Position is a direct function of physical belt displacement — **drift-free** regardless of
speed variation.

### 4.2. Backdated anchoring (camera-latency compensation)

The anchor is only drift-free if $p_{\text{anchor}}$ is the belt position **at frame capture**,
not at ingest. Exposure + decode + YOLO + poll ≈ 80–150 ms, during which the belt advances
$v_{\text{belt}} \cdot \Delta t_{\text{lat}}$ — anchoring stale detections to the *current*
belt position injects that as a fixed upstream error. Solution: stamp each frame with its
decode time backdated by half the exposure (photons integrate over the exposure window), keep
a $(t, p)$ ring buffer, and anchor at the interpolated capture-time position:

$$p_{\text{anchor}} = p(t_{\text{cap}}), \qquad
t_{\text{cap}} = t_{\text{decode}} - \tfrac{1}{2} t_{\text{exposure}}$$

Falls back to the current position when history is unavailable (static/simulated belt).
Implementation: `BeltPositionTracker.position_at` (`modules/core/tracking.py`), `_capture_loop`
in `modules/vision/pipeline.py`.

### 4.3. Fixed-point pick-position prediction

The predictor chooses a stable **park position**, not a firing time — the pick itself fires on
a live positional gate (§4.4), which makes it immune to belt-speed estimate noise.

Iterate to the earliest goto-feasible pick:

1. Guess $t_{\text{pick}}^{(0)} = t_{\text{now}} + \text{lead}$.
2. Project the object: $u(t^{(k)}) = u_{\text{now}} + v_{\text{belt}}(t^{(k)} - t_{\text{now}})$.
3. Map to R-frame; compute robot travel time $\Delta t_{\text{goto}}$.
4. $t^{(k+1)} = t_{\text{now}} + \Delta t_{\text{goto}} + t_{\text{delay}}$; repeat to
   convergence.
5. Apply the minimum lead (`pick_gate.intercept_lead_time_s`) so the arm parks *downstream* of the
   object; clamp $u_{\text{pick}}$ to the workspace edge. If the arm
   cannot arrive before the object passes $u_{\text{pick}}$, skip (genuinely unreachable).

This solver is `DeltaArm.predict` (`modules/core/delta.py`); it costs every job a planner sees
(§7). The `spt` rule picks the job with the shortest start → pick → bin path: path length
stands in for processing time, since on this arm a shorter path is a shorter pick cycle.

### 4.4. Positional pick gate

After parking, **no time math**: the main thread watches the claimed object's live
encoder-anchored position and dispatches the pick the moment

$$u_{\text{now}} \ge u_{\text{pick}} - \text{offset}(v_{\text{belt}})$$

**Lead offset.** Between gate-true and physical suction contact lies
$T_{\text{delay}} =$ `pick_gate.robot_movement_delay_s` + `ethernet_delay_s` + `pick_descent_time_s`
(+ sampling latency ≈ gate poll/2 + perception tick/2). `robot_movement_delay_s` is the
empirical dispatch→contact delay and already contains the deployed PLC's State-10 descent
(≈ 0.08 s ramp to `Pos[0]`); `pick_descent_time_s` (default 0) is an explicit extra term for
a vertical descent, used only if a stationary-belt measurement shows contact later than that.
With the oblique descent on it is not added (the slanted contact absorbs the travel, §4.5).
The object moves $v_{\text{belt}} \cdot T_{\text{delay}}$ downstream in that window, so the
gate fires early by exactly that displacement:

$$\text{offset}(v_{\text{belt}}) = v_{\text{belt}} \cdot T_{\text{delay}}$$

The general accelerating-belt forms (belt mid-ramp at gate time) are derived by splitting the
window at the ramp end $T_{\text{accel}} = |v_{sp} - v_c| / a_{\text{nom}}$:

* ramp does not finish within the window:
  $\text{offset} = v_c T_{\text{delay}} + \tfrac{1}{2} a T_{\text{delay}}^2$
* ramp finishes: $\text{offset} = \tfrac{v_c + v_{sp}}{2} T_{\text{accel}}
  + v_{sp}(T_{\text{delay}} - T_{\text{accel}})$

These are **not implemented**: the speed controller (§6.5) freezes the belt setpoint for the
whole committed pick, so the precision path stays single-term and the live gate absorbs
residual drift. Any redesign of the speed controller that relaxes that guarantee must revisit
this — see `open-issues.md` **L9**.

**Park before the gate.** The gate fires $T_{\text{lead}}$ (the full lead above, in seconds)
before the object reaches $u_{\text{pick}}$, so a plan is feasible only if
$t_{\text{now}} + T_{\text{cmd}} + T_{\text{goto}} + T_{\text{lead}} \le t_{\text{pick}}$;
the predictor pushes the intercept downstream (fixed point, ≤ 4 passes) until that holds, and
rejects the object if the push reaches $u_{\max}$.

**Two-sided gate.** If the arm parks late and the object is already more than
`pick_gate.late_abort_mm` past the threshold when the gate is evaluated, the pick is aborted
before dispatch (the object is re-queued) instead of landing the cup behind the board.

**Arrival acceptance.** The arm counts as arrived only when it is within the speed-mapped
tolerance of the target **and** inside the final vertical segment of the packet: the deployed
PLC accepts a new command 3 on top of a running chain (`open-issues.md` **O2**), which is
harmless only once the last chain instance is running.

**IK pre-check.** Every waypoint of a plan is checked against the PLC-faithful IK port
(`modules/core/kinematics.ik_reachable`, joint limit included) before the object is
claimed; the deployed IK reports a joint-limit trip as success (**O1**).

$T_{\text{delay}}$ is calibrated from the per-pick `[GATE]` log (`dispatch_to_contact_s`,
with `t_d_model_s` reporting the vertical descent model) and `modules/tools/latency_probe.py`; the
terms are **currently uncalibrated estimates** (`open-issues.md` **C2**, **T3**).

### 4.5. Oblique intercept (belt-tracking descent, opt-in)

A vertical descent at a fixed R-frame point contacts the board with horizontal *relative*
velocity equal to the belt speed, dragging it during suction settling. With
`pick_gate.oblique_descent_enabled`: the arm still **parks above the predicted point**
$\mathbf{p}_{\text{pick}}$ (goto unchanged, stays in-workspace), but the pick-phase **contact
point shifts downstream** by the board's travel during the descent:

$$\mathbf{p}_{\text{contact}} = \mathbf{p}_{\text{pick}} + \hat{u}\,(v_{\text{belt}} \cdot t_d)$$

where $\hat{u}$ is the belt-flow unit vector in the R-frame and $t_d$ the modeled descent time
(one fixed-point pass folding the slant into the segment length). The cup follows the board
and meets it at near-zero relative velocity. The gate lead is **unchanged** (the slant itself
absorbs the $t_d$ travel — $t_d$ is *not* added to the lead).

> **Invariant**: only the *contact* point shifts downstream — never the park/goto target,
> which would leave the workspace at operating belt speed.
>
> **Currently disabled** (`oblique_descent_enabled = false` ⇒ vertical descent) because
> `robot.interpolator.a_max` is uncalibrated and $t_d$ is consequently over-estimated: at high belt
> speed the computed contact can approach $u_{\max}$, which is handled as a safe
> dispatch-reject rather than a clamp. See `open-issues.md` **C3**/**C5**/**T2**.

### 4.6. Tracked-object lifecycle

```mermaid
stateDiagram-v2
    [*] --> NEW : first detection
    NEW --> TRACKED : centroid matched >= 3 frames
    TRACKED --> DEAD_RECKONED : exits camera FOV (encoder anchor carries on)
    DEAD_RECKONED --> DONE : picked, or passed u_max
    DONE --> [*] : pruned from tracker
```

### 4.7. Pick-attempt exclusivity (no re-pick)

There is **no slip-detection/retry**: `execute()` reports success when the arm's *motion*
completes; suction is never verified (`open-issues.md` **L6** — the software success count is
therefore an optimistic estimate of the true one). Exactly-once bookkeeping keys off whether
the grip was actually **dispatched**:

* **Pick dispatched** (success, or failure after the grip command): remove the object from the
  tracker — a possible suction miss is never retried.
* **Aborted pre-grip** (goto timeout, gate stall, track lost): only unclaim; the object stays
  tracked and re-plannable, and continues to count toward the density $N$ of §6.

The gate abort itself is a **progress-based stall check**: abort when the object's $u$
advances < 0.5 mm for several seconds, or when the track is lost. It is deliberately not a
wall-clock deadline, which cannot survive a belt slow-down after plan-build.

---

## 5. Yaw Orientation Resolution (360°)

QFP/TQFP parts have 180° rotational symmetry; YOLO-OBB reports tilt only in $[-90°, 90°)$.
The absolute 360° heading comes from the round locator dot (pin-1 marker): the board→marker
vector $\vec{v} = (x_m - x_b,\, y_m - y_b)$ gives the heading via `atan2`. Because the marker
sits diagonally on the board, a **per-class offset** (`vision.orientation.offset_by_class`)
is added at measurement time. A track that loses the marker in later frames reuses its last
marker-resolved heading rather than falling back to the ±90°-ambiguous OBB fold.

### 5.1. Rotation timeline

The suction rotation is applied **after** the grip, not before: home the axis to 0 during the
goto flight → gate fires → pick descends → once the arm lifts back to
$z \ge z_{\text{pre\_pick}}$, rotate the attached board to the bin orientation. The post-grip
angle is refreshed from the object's live tracked heading at the gate (plan-build values can
be several degrees stale), guarded by `rotate_refresh_max_delta_deg` against outliers.

### 5.2. Angle convention — exactly three layers

One unit per layer, converted only at the boundaries:

**Layer 1 — measurement (pixels → R-frame radians).** The vision heading $h$ (degrees,
measured against image +y **down**) converts once, in
`ConveyorFrame.vision_heading_to_robot_rad`:

$$\varphi_{\text{board}} = \text{wrap}_{\pi}\!\big(\text{rad}(h - 90°) + \theta_{\text{frame}}\big)$$

($0$ = robot $+X$, positive = CCW from above. The $-90°$ is a fixed constant that rebases the
angle from the image axis convention to the R-frame $+X$ axis.)

**Layer 2 — algorithm (R-frame radians only).** `TrackedObject.rotation_rad` stores
$\varphi_{\text{board}}$; the post-grip command is

$$\theta_{\text{cmd}} = \text{wrap}_{\pi}\!\big(s\,(\theta_{\text{offset}} - \varphi_{\text{board}})\big)$$

with $\theta_{\text{offset}}$ = `robot.rotation.offset_deg` (bin orientation) and $s$ =
`robot.rotation.sign` $\in \{+1, -1\}$ (physical axis direction vs. R-frame CCW; calibrate with
`python3 -m modules.tools.test_rotate`); `runtime/plan.post_grip_rotation_rad`. The wrap to $[-\pi, \pi)$ **is** the minimal-turn decision,
made exactly once, relative to the homed 0.

**Layer 3 — wire (IPC boundary, verbatim).** The Siemens axis accepts signed degrees in
$[-360, 360]$, shares the R-frame zero, and its command value encodes **both position and spin
direction**. The boundary is therefore a plain radians→degrees identity clamped to
$[-359, 359]$ — **no wrap**:

$$\theta_{\text{wire}} = \text{clamp}_{\pm 359}\big(\deg(\theta_{\text{cmd}})\big)$$

Wrapping here ($180° \to -180°$, $270° \to -90°$) would flip the commanded spin direction and
drive the axis nearly a full turn the wrong way on a $179° \to 180°$ step. Manual/CLI absolute
angles pass through untouched.
(`robot_rad_to_wire_deg` / `wire_deg_to_robot_rad` in `modules/core/angles.py`.)

**Calibration**: (1) `test_rotate` probe — remap/settle, implied axis speed, visual direction
check for `robot.rotation.sign`, cmd-7→cmd-7 retrigger test; (2) a hardware run reading the per-pick
`[ROTATE]` log (`vision_angle / board_heading / rotate_cmd / rotate_at_gate / rotate_at_end`)
to set `robot.rotation.offset_deg` + `object_types.<type>.heading_offset_deg`.
`robot.rotation.home_tolerance_deg` > 0 turns "axis not yet home at grip" into a warn-only
check. Neither the sign nor the offsets
have been confirmed on hardware — `open-issues.md` **C1**/**C4**.

---

## 6. Adaptive Conveyor Speed (Rate Regulation)

> **Baseline notice.** §6 documents the `inverse_density` speed law, the cell's first
> adaptive law. It is a baseline, not the target design — in particular it is open-loop with
> respect to missed picks (`open-issues.md` **L10**). The other laws are in §7.

**Goal: preserve the serial arm's throughput under an unstable feeder.** The belt is a **rate
regulator**, not a transport to maximize: its job is to hold the *presentation rate* of
pickable objects near a nominal target. Belt speed is therefore set **inversely** to object
density: belt speed does not set throughput — the serial arm's pick cycle does.

### 6.1. Symbols ↔ config

| Symbol | Meaning | Config key |
|---|---|---|
| $v_{\min}$ | operational floor (hardware control imprecise at very low speed) | `speed.band.min_mm_s` |
| $v_{\text{cap}}$ | pickability ceiling, derived $= \min(L/t_{\text{transit}},\, v_{\text{soft}},\, v_{\text{hw}})$ | via `speed.laws.inverse_density.transit_min_s`, `speed.band.max_mm_s`, `conveyor.hw_max_mm_s` |
| $a_{\text{nom}}$ | belt accel magnitude (forecasts, simulator ramp) | `conveyor.accel_mm_s2` |
| $t_{\text{pick}}$, $\mu_{\max} = 1/t_{\text{pick}}$ | calibrated pick cycle / arm ceiling | `scheduling.arm_cycle.cycle_s` |
| $k$, $\lambda_{\text{nom}} = k\,\mu_{\max}$ | headroom factor / presentation-rate target | `speed.laws.inverse_density.headroom` |
| $L$ | workspace window length $= u_{\max} - u_{\min}$ | `conveyor.workspace_window_uv` |
| $L_{\text{meas}}$ | density region length (0 ⇒ derive $= u_{\max}$) | `speed.laws.inverse_density.density_length_mm` |
| $\Delta_{\min}$, $\Delta v_{\max}$ | commit deadband / per-commit step limit | `speed.commit.deadband_mm_s`, `speed.commit.max_step_mm_s` |

### 6.2. The inverse density law

Presentation rate $\lambda = \rho v$ (linear density × speed). Holding
$\lambda = \lambda_{\text{nom}}$:

$$v = \text{clamp}\!\left(\frac{\lambda_{\text{nom}}}{\rho},\; v_{\min},\; v_{\text{cap}}\right),
\qquad \rho = \frac{N}{L_{\text{meas}}}$$

with $N$ = count of **unclaimed, not-yet-passed** objects ($0 \le u \le u_{\max}$).
Equivalently $v = \lambda_{\text{nom}} L_{\text{meas}} / N$ — **hyperbolic** in $N$.
More density ⇒ *slower* belt: a sparse feeder is sped up so objects reach the arm before it
starves; a dense feeder is slowed so the serial arm gets transit time.

**Pickability ceiling**: the object must stay in the $L$-length window at least
$t_{\text{transit}}$ (`pick_transit_min_s`) for the arm to intercept:
$v_{\text{cap}} = \min(L / t_{\text{transit}},\, v_{\text{soft}},\, v_{\text{hw}})$.

**Rate target**: $\lambda_{\text{nom}} = k\,\mu_{\max}$ with $k < 1$ deliberately below the
arm's ceiling — the headroom absorbs feeder bursts and timing jitter.

### 6.3. Three regimes (one law, two clamps)

1. **Feeder-limited (sparse)**: law wants $v > v_{\text{cap}}$, clamps to $v_{\text{cap}}$.
   The arm is under-fed and the belt is already doing its best.
2. **Regulated (normal)**: $v \in (v_{\min}, v_{\text{cap}})$, arrival held at
   $\lambda_{\text{nom}}$ — sub-unity utilization *by design*.
3. **Overload (dense)**: law pins $v = v_{\min}$; the window stays full and utilization rises
   to ~100 % **emergently** — no separate brake/exception state. The backlog count (in-window
   unpicked objects predicted to pass $u_{\max}$) is **telemetry only — it never feeds back
   into the speed decision** (`open-issues.md` **L10**). If the feeder exceeds
   $\mu_{\max}$ even here, loss is unavoidable — no speed policy recovers a genuinely
   over-saturated cell.

### 6.4. Spacing cap (cluster ceiling)

The count-only law regulates the *average* rate but is blind to clustering: a tight pair and a
spread pair both read $N = 2$ and get the same speed, so a tight trailing object can pass
$u_{\max}$ unpicked. The binding constraint for a burst is the inter-arrival time of adjacent
objects: $s_i / v \ge t_{\text{pick}}$. A **spacing ceiling** is min-ed onto the density law:

$$v_{\text{target}} = \max\!\Big(v_{\min},\; \min\big(v_{\rho},\; g_{\min} / t_{\text{pick}}\big)\Big),
\qquad g_{\min} = \min_i (u_i - u_{i+1})$$

over the leading few objects only (`speed.laws.inverse_density.spacing_lead_objects`, default 4) — an upstream cluster
does not force a premature slow-down, but is also invisible to the law until it reaches the
front (`open-issues.md` **L11**). A cluster tighter than $v_{\min} t_{\text{pick}}$ pins the
floor (best-effort; the cell is locally over-dense).

### 6.5. When the setpoint moves, and how far

Density is sensed **continuously** (~25 ms perception tick), but the law runs only when its
**setpoint gate** opens (`modules/scheduling/gates.py`). `inverse_density` defaults to
`at_contact`: when the arm becomes free, every `speed.control_period_s` while it stays free, and
once at cup contact — the belt then ramps during the carry to the bin. Nothing is decided from
goto dispatch to cup contact (`RealtimeState.pick_committed`), so the plan, the gate lead and
the grip all see the setpoint that was live when the plan was built. The same commit policy
applies to every law (`modules/scheduling/commit.py`):

* **Band**: the target is clamped to $[v_{\min},\, \min(v_{\text{soft}}, v_{\text{hw}})]$.
* **Deadband**: commit only if the step exceeds $\Delta_{\min}$.
* **Step limit**: each commit moves at most $\Delta v_{\max}$ toward the target, so each ramp
  settles in $\Delta v_{\max} / a_{\text{nom}}$ (≈ 0.9 s at 20 mm/s). (The raw hyperbolic law
  is near bang-bang at small $N$; the $N = 1 \leftrightarrow 2$ jump alone would ramp for
  seconds.)
* **Closed loop**: if the PLC's measured `speed_current` still diverges from the setpoint by
  more than $2\Delta_{\min}$ 3 s after the last commit, the setpoint is re-sent. This catches
  commands lost on the wire — the controller never assumes a commit was applied.

**Jerk avoidance**: modeling the belt's S-curve ramp explicitly in the gate offset would add a
piecewise-cubic bookkeeping for a sub-millimetre correction that only applies mid-ramp.
Instead, the freeze above keeps the belt **steady from dispatch to contact**, so the precision
path keeps the single-term offset of §4.4, and the live gate absorbs residuals. Startup seeds
the belt with `speed.static_mm_s` unconditionally (the `constant` law then never moves it).

### 6.6. Known gaps in this controller

Recorded here so the baseline is not mistaken for a finished design. Details and status in
[`open-issues.md`](open-issues.md):

* **L10** — no feedback from missed/at-risk picks into the speed decision; the law sees only
  instantaneous density and spacing. The `predictive_rank` law (§7.3) closes this loop when
  selected.
* **L11** — the spacing cap has a 4-object horizon.
* **L9** — the freeze of §6.5 is what licenses the single-term gate offset of §4.4. A commit
  issued while the arm is free can still be ramping when the next plan's gate fires.
* **G1/G2/G3** — the static seed sits outside the band, the calibrated pick cycle disagrees
  with the cell's stated nominal rate, and the density measurement region is longer than the
  workspace it regulates.

---

## 7. Pluggable Scheduling: Planners and Speed Laws

The pick order (§4) and the belt speed (§6) are each chosen by a named plugin
(`scheduling.planner`, `speed.law`; code and contracts in `modules/scheduling/`). This section
is the theory of the planners and laws beyond the `spt` rule and the density law; they were
ported from the scheduling research repository onto the robot's own timing model. The
configured planner is `drop_longest` (§7.2) and the configured law `constant`
(`open-issues.md` **G10**, **L10**).

### 7.1. The job model

Every unclaimed tracked object $j$ becomes a job at planning time $t_0$:

| Symbol | Meaning | Source |
|---|---|---|
| $r_j$ | release — dispatchable now | $t_0$ |
| $d_j$ | deadline — the object passes $u_\max$ | $t_0 + T_V(u_\max - u_j)$ |
| $g_j$ | grab — predicted cup contact | intercept of §4.3 from the arm's start pose |
| $p_j$ | processing — until the arm is free over the bin | gate-fire time $+ T_\text{delay} + T(\text{pick trajectory}) + s$ |

$T_V(\Delta u)$ is the time the belt needs to travel $\Delta u$ under the forecast of §7.4,
$T(\cdot)$ the trajectory-time model of §3, and $s$ = `scheduling.setup_time_s`. The intercept is
the two-stage fixed point of §4.3–§4.4 (earliest reachable contact, then pushed downstream until
the arm is parked a full gate lead before it), generalised to an arbitrary start pose and start
time and to a ramping belt: $v\,\Delta t$ becomes the forecast travel and $\Delta u / v$
becomes $T_V$. On a steady belt it is identical to the realtime predictor.

Because the arm starts each pick from the previous bin, $p_j$ and $g_j$ depend on the
predecessor: they are re-predicted for every candidate sequence, never read from the job. A
sequence is on time when $g_j \le d_j - \delta$ for every job ($\delta$ =
`scheduling.safety_margin_s`).

### 7.2. Sequence planners

The belt fixes the order of arrival: objects cannot overtake, so release order and deadline
order agree. For $1 \mid r_j \mid \sum U_j$ with agreeable release dates and deadlines:

* **`kim`** — Kise–Ibaraki–Mine (1978): add the jobs one at a time in arrival order; if the
  retained set plus the new job is on time, keep it; otherwise remove **exactly one** job — the
  one whose removal leaves an on-time set that frees the arm earliest (ties remove the job
  latest in the order, the new job first). Removing the new job restores the previous on-time
  set, so one removal always suffices. **`kim_release`** walks the jobs in estimated release
  order $r_j = a_j - g_j$ instead, which need not be agreeable.
* **`cardinality_arrival` / `_release`** — Lawler's cardinality recurrence (1983): for every
  $k$ keep the on-time schedule of $k$ jobs that frees the arm earliest; each job extends each of
  them; the largest $k$ wins.
* **`drop_longest`** — add jobs in belt order; while the sequence is late, drop the job with
  the longest processing time (several drops per insertion can happen). Moore–Hodgson style;
  the cell's planner before 2026-09.
* **`dp`** — exact search over the `max_jobs` most urgent parts: layer $k$ keeps, for every
  (set of $k$ parts, last part), the earliest instant the arm is free; $O(2^n n^2)$. The
  reference the heuristics are measured against.
* **`<rule>_rollout`** — a dispatch rule run forward against a simulated clock, so a rule can be
  scored by a speed law like a planner.

All are exact on the textbook problem (fixed $p_j$, agreeable order; `tests/test_scheduling.py`
checks them against brute force). With sequence-dependent $p_j$ and the deadline tested at
contact they are heuristics; the research repository measured `kim` within about one percentage
point of the exact search's pick rate (paper, tag `paper-submitted`). The schedule is a plan: the realtime
loop commits only its first entry that still validates against the live tracker (§7.5),
executes it, and plans again. Objects a planner leaves out are abandoned on purpose.

### 7.3. Predictive-rank speed law

Choose the belt speed $V$ that maximises the number of picks the planner predicts within a
horizon $W$ (`speed.laws.predictive_rank.horizon_s`):

$$K(V) = \left|\{\, j \in \text{schedule}(V) : g_j \le t_0 + W \,\}\right|$$

Candidates are enumerated on a grid (`candidate_step_mm_s`), not searched: $K$ is integer-valued
and not unimodal. A candidate is admissible when

* $v_\min \le V \le v_\max$, with $v_\min$ = `speed.band.min_mm_s` and
  $$v_\max = \min\left(\frac{L}{k\,p_\text{worst} + g_\text{worst}},\; v_\text{max,op},\; v_\text{hw}\right)$$
  where $L = u_\max - u_\min$: an object entering the band must still be pickable after the
  arm finishes $k$ queued picks (`speed.laws.predictive_rank.queue_depth_k`,
  `scheduling.arm_cycle.occupancy_worst_s`, `grab_worst_s`);
* the drive reaches $V$ within $W$;
* $|V - V_\text{set}| \le$ `speed.commit.max_step_mm_s`, so the speed scored is the speed the
  step-limited commit (§6.5) actually sends (the law is exempt from the deadband for the same
  reason).

The current setpoint, if admissible, is scored first and replaced only by a strictly higher
$K$ or an equal $K$ at a faster speed (`predictive_rank_hysteresis` keeps it unless a candidate
scores strictly higher). An inadmissible setpoint is abandoned. The winner's schedule is
executed as planned when it was made by the executing planner.

### 7.4. Belt forecast and when the setpoint may move

Scoring $V$ needs object positions under a speed the belt is not running at. The forecast
assumes the drive ramps linearly from the measured speed $v_0$ to the setpoint at
$a$ = `conveyor.accel_mm_s2` and then holds it:

$$x(t) = \begin{cases} v_0 t + \tfrac{1}{2}\,\sigma a t^2 & t < t_r \\ \tfrac{1}{2}(v_0 + V)\,t_r + V (t - t_r) & t \ge t_r \end{cases}
\qquad t_r = \frac{|V - v_0|}{a},\; \sigma = \operatorname{sign}(V - v_0)$$

"Then holds it" is made true by the freeze of §6.5: whatever the gate (predictive-rank's
default is `arm_free` — when the arm becomes free, then every `speed.control_period_s`), the
setpoint never moves between goto dispatch and cup contact, which also keeps the single-term
gate lead of §4.4 valid (L9). A commit made just before a plan can still be ramping at that
plan's gate; the forecast used to plan it includes the ramp.

### 7.5. Plan, then re-validate

A KIM or predictive-rank solve can take a noticeable fraction of a second, during which objects
move. Planning therefore runs on a value snapshot of the tracker, and the chosen entry is
re-validated under the state lock before it is claimed: the object is still tracked and
unclaimed, a fresh intercept from the arm's real pose on the live forecast meets
$d_j - \delta$, and every waypoint passes the PLC-faithful IK. The first entry that passes is
committed; the difference between its fresh and planned contact times is logged as `drift_s`.

### 7.6. Other speed laws

With $L = u_\max - u_\min$, $[v_\min, v_\max]$ the queue bound of §7.3 (own $k$ per law) and
$N$ the unclaimed parts in view:

* **`bound_only`** — sit at $v_\max$ and never look at the belt (open loop). If it scores close
  to `predictive_rank`, the ranking is not earning its cost and the benefit was the bound.
* **`backlog_patience`** — $v = L / (N p + g)$ with the measured backlog $N$ in the workspace and
  mean occupancy $p$ and grab $g$, clamped to the bound.
* **`min_slack`** — the $i$-th part in line must survive $i$ cycles and a grab:
  $v = \min_i\, (u_\max - u_i) / (i\,p + g)$, clamped to the bound.
* **`rate_schedule`** — gain scheduling: the detection rate over a window maps to a speed through
  a declared table (linear between points).
* **`periodic_two_speed`** — alternate two speeds on a fixed clock, blind to the belt: the control
  that separates feedback from mere speed variation.

---

## 8. Concurrency Model (summary)

Theory-level view; the full runtime layout is in `basis-programming.md` §2.

* **Background communication process** — sole owner of PLC I/O (snap7 Modbus/TCP to Siemens,
  pylogix EtherNet/IP to Omron); IPC queues isolate network jitter from control loops.
* **Perception/state daemon thread** (~25 ms) — sensor fusion: belt encoder, vision poll,
  tracker refresh; sole regular PLC status reader. It decides nothing.
* **Decision/execution main thread** — speed law at its gate, plan build, claim, dispatch,
  gate wait; reads shared state, issues no status I/O of its own.
* Two locks: `ipc_lock` (exactly one dispatch/status round-trip in flight) and `state_lock`
  (guards `RealtimeState` between perception and decision threads).

---

## 9. What this document deliberately omits

* **Open problems** — every unresolved calibration, config inconsistency and algorithmic gap
  is registered in [`open-issues.md`](open-issues.md), including which of them constrain the
  scheduler redesign.
* **Superseded designs** — the alternatives that were tried and replaced, with the reason
  each was rejected, are in [`decision-log.md`](decision-log.md).
* **Speculative extensions** — parked ideas (dashboard v2, SQL pick history, queueing-theory
  belt control, suction verification) are in `decision-log.md` §4.
