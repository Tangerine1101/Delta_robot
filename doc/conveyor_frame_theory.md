# Conveyor Frame & Dataset Collection — Theoretical Foundations

> **Status**: Pre-implementation brainstorm. Decisions in Section 7 must be confirmed before coding begins.

---

## 1. Homogeneous Transform F: Conveyor Frame → Robot Frame

### 1.1 Problem Statement

The current codebase assumes the conveyor belt runs parallel to the robot Y-axis
(hardcoded in `scheduler.py:990`, `RealSpeedSource:355-365`, `image_processing.py:78-86`).
In reality the belt lies at an angle θ in the robot XY plane. We need a dedicated
conveyor coordinate frame (C-frame) and a homogeneous transform **F: C → R**.

### 1.2 Frame Definitions

| Frame | Origin | Axes |
|-------|--------|------|
| **R-frame** (robot) | Under base plate | Standard X, Y, Z (Z negative-down) |
| **C-frame** (conveyor) | Fixed point O_C on belt body, chosen at calibration time | u along belt flow direction (toward downstream), v perpendicular on belt surface |

### 1.3 Matrix Form (2D — belt surface assumed planar and horizontal)

Because the belt surface is parallel to the robot XY plane (Z is constant at
`pickup_height`), a 2D homogeneous transform suffices:

```
        [ cos θ   -sin θ   t_x ]   [ u ]   [ X_R ]
F  =    [ sin θ    cos θ   t_y ] , [ v ] → [ Y_R ]
        [   0        0      1  ]   [ 1 ]   [  1  ]
```

- `(t_x, t_y)` — origin O_C expressed in the robot frame.
- `θ` — angle from robot X-axis to conveyor u-axis (positive = counter-clockwise).
- The Z coordinate is constant: `Z = pickup_height` (or adjusted per object thickness).

**Velocity transform** (no translation for vectors):

```
[ v_Rx ]   [ cos θ   -sin θ ]   [ v_Cu ]
[ v_Ry ] = [ sin θ    cos θ ] × [ v_Cv ]
```

If the encoder returns a scalar speed `s` (mm/s) along the u-axis and `v_Cv = 0`:

```
v_Rx = s · cos θ
v_Ry = s · sin θ
```

### 1.4 Determining F (Calibration)

Place reference objects at **≥ 3 known conveyor positions** `(u_i, v_i)`, then measure
their robot-frame positions `(X_Ri, Y_Ri)` using the robot end-effector (touch-probe
method). With 3 point-pairs:

```
[ X_R1  X_R2  X_R3 ]   [ cos θ   -sin θ   t_x ]   [ u_1  u_2  u_3 ]
[ Y_R1  Y_R2  Y_R3 ] = [ sin θ    cos θ   t_y ] × [ v_1  v_2  v_3 ]
[  1     1     1   ]   [   0        0      1  ]   [  1    1    1  ]
```

Solve for `(θ, t_x, t_y)` via least-squares with 4+ pairs for robustness.
Store in `config.json` under `conveyor.theta_deg` and `conveyor.origin_xy`;
pre-compute the 3×3 matrix at load time.

---

## 2. Encoder-Based Dead-Reckoning (Superior to Velocity Integration)

### 2.1 Key Insight

A picked object is **fixed to the belt surface**. Therefore its coordinates in the
C-frame `(u_i, v_i)` are **constant** regardless of belt speed changes.

Let `p(t)` be the encoder reading (mm along u-axis, positive = downstream direction).
The belt origin O_C in robot frame shifts as the belt moves:

```
O_C(t) = O_C(0) + (p(t) - p(0)) · û
```

where `û = (cos θ, sin θ)` is the unit vector along the belt flow direction.

### 2.2 Object Position in Robot Frame

For an object anchored at conveyor coordinates `(u_i, v_i)`, detected when encoder
read `p_anchor`:

```
P_R(t) = F_t · (u_i, v_i, 1)^T

       where:
         F_t = [ R(θ) | O_C(0) + (p(t) - p(0)) · û ]
             = [ R(θ) | (t_x + (p(t)-p(0))·cosθ,  t_y + (p(t)-p(0))·sinθ) ]
```

Or equivalently, define the "relative encoder displacement" from anchor:

```
Δp(t) = p(t) - p_anchor

X_R(t) = t_x  +  (u_i + Δp(t)) · cos θ  -  v_i · sin θ
Y_R(t) = t_y  +  (u_i + Δp(t)) · sin θ  +  v_i · cos θ
```

### 2.3 Advantages Over the Current `P_detect + v · Δt` Model

| Property | Current (velocity integration) | Proposed (encoder position) |
|----------|--------------------------------|-----------------------------|
| Accumulates drift? | Yes — speed noise integrates | No — absolute register |
| Handles acceleration/deceleration? | No — assumes constant v | Yes — p(t) is always correct |
| Requires fresh speed sample? | Yes (stale_timeout enforced) | No — only needs latest p |
| Works when belt is stationary? | Implicitly (Δt model) | Trivially (Δp = 0) |

The iterative loop in `_predict_pick_position` still uses velocity to predict the
**future** position `p(t_pick)`. But updating the **current** position of an already-
detected object no longer requires `latest_speed`.

---

## 3. Camera → Conveyor → Robot Pipeline

### 3.1 Homography H: Image Pixels → C-frame

The camera looks straight down at the belt (top-down mount).
The belt surface is planar. Therefore a **planar homography** H (3×3) maps pixel
coordinates to conveyor coordinates:

```
           [ u ]       [ p_x ]
λ ·        [ v ]  =  H · [ p_y ]
           [ 1 ]       [  1  ]
```

H is calibrated once by placing a checkerboard or known-size marker pattern
**on the belt surface** and solving the 4-point (or more) DLT correspondence.

Requires camera lens-distortion correction (undistort) to be applied **before** H.

### 3.2 Full Pipeline: Pixel → Robot

```
pixel (p_x, p_y)
    │   apply H
    ▼
conveyor coords (u, v)
    │   apply F
    ▼
robot coords (X_R, Y_R)
```

Pre-multiply to get composite matrix **M = F · H** (3×3):

```
[ X_R ]       [ p_x ]
[ Y_R ]  =  M · [ p_y ]
[  1  ]       [  1  ]
```

Cache M at startup. Its inverse M^(-1) maps robot → pixel (needed for dataset labeling).

### 3.3 Calibration Plan

| Step | Tool | Produces |
|------|------|----------|
| Camera intrinsic + distortion coefficients | OpenCV `calibrateCamera` with checkerboard | `camera_matrix`, `dist_coeffs` |
| Homography H | 4+ belt-surface point correspondences (pixel ↔ (u,v)) | H |
| Conveyor frame F | 3+ robot touch-probe measurements at known (u,v) points | θ, t_x, t_y |

All three artifacts stored in `config.json` under `conveyor.camera.*` and
`conveyor.theta_deg`, `conveyor.origin_xy`. The dataset script and scheduler both
load and use them; no runtime recalibration needed.

---

## 4. Object Tracking Lifecycle in `image_processing`

### 4.1 TrackedObject Data Model

```python
@dataclass
class TrackedObject:
    object_id:        str
    object_type:      str
    conveyor_xy:      tuple[float, float]   # (u_i, v_i) — constant while tracked
    rotation_rad:     float                 # yaw angle (from YOLO or 0.0)
    belt_pos_anchor:  float                 # encoder p value at first detection
    state:            str                   # NEW | TRACKED | DEAD_RECKONED | DONE
    last_seen_at:     float                 # monotonic timestamp
    confidence:       float
```

### 4.2 State Machine

```
      ┌──────┐   detection matches     ┌─────────┐
      │  NEW │ ──────────────────────► │ TRACKED │
      └──────┘                         └────┬────┘
                                           │  exits camera FOV (no detection match)
                                           ▼
                                    ┌─────────────┐
                                    │DEAD_RECKONED│  position from encoder only
                                    └──────┬──────┘
                                           │  picked successfully OR past pickup window
                                           ▼
                                        ┌──────┐
                                        │ DONE │  pruned from list
                                        └──────┘
```

### 4.3 Transition Rules

- **NEW → TRACKED**: first frame where detection bounding-box centroid projects to
  conveyor coords `(u_i, v_i)` within the camera FOV.
- **TRACKED → TRACKED**: each subsequent frame with a matching detection (IOU or
  centroid proximity in C-frame < threshold). Update `conveyor_xy` with exponential
  moving average of measured `(u, v)` to reduce noise:

  ```
  conveyor_xy = α · (u_measured, v_measured) + (1 - α) · conveyor_xy
  ```

  Because the object is fixed to the belt, `conveyor_xy` should be nearly constant;
  drift indicates slip or measurement noise.

- **TRACKED → DEAD_RECKONED**: no detection match for > 1 frame AND
  object's predicted `u + Δp` has exited the downstream FOV boundary.
  `conveyor_xy` is frozen; position is computed purely from encoder.

- **DEAD_RECKONED → DONE**: `X_R(t)` exits `pickup_window_y[1]` boundary (or the
  corresponding downstream boundary projected along `û`), or `stale_timeout_s` elapses.

### 4.4 Delivery to Scheduler

`image_processing.poll(now, p_now)` returns a list of `ObjectDetection` (current API)
constructed on-the-fly from `TrackedObject.position_R(p_now)`. The scheduler
`PickScheduler` sees the same `ObjectDetection` interface — no change needed in
`scheduler.py` for basic integration.

---

## 5. YOLO Dataset Collection Script

### 5.1 Goal

Produce a dataset of labeled images for YOLO training. Each sample is a camera frame
with bounding-box annotations for 1–2 PCBs that were placed on the belt by the robot
at **known positions and orientations**. Ground-truth labels are computed geometrically —
no manual annotation needed.

### 5.2 Prerequisites

- F and H calibrated and stored in config.
- Belt encoder zeroed at a reference position `p_home`.
- Robot homed and calibrated (`calib` command).
- Two source boxes (`box_pcb1`, `box_pcb2`) at fixed known robot positions.
- A **drop zone** polygon defined in C-frame: set of (u, v) reachable by the robot arm
  that also lies within the camera FOV at position `p = p_camera_start`.

### 5.3 Collection Flow

```
Initialize:
    belt.set_position(p_home)          # absolute, waits for idle
    robot.calibrate()

For each sample (repeat N_samples times):

  [Phase A — Place objects on belt]
    for each source_box in [box_pcb1, box_pcb2]:
        robot.pick(source_box.xyz)
        θ_rand = uniform(0°, 360°)
        (u_drop, v_drop) = random_point_in_polygon(drop_zone_uv)
        P_drop_R = F @ (u_drop, v_drop, 1)
        robot.rotate(θ_rand)
        robot.place(P_drop_R)
        record: { type, u_drop, v_drop, θ_rand, belt_pos_anchor = p_home }

  [Phase B — Scroll belt backward; capture frames]
    for frame_idx in range(N_frames):
        p_target = p_home - frame_idx * Δp_per_frame
        belt.set_position(p_target)    # commandID = set_position (10)
        wait_until_idle()
        p_actual = read_position_current()
        frame = camera.capture()
        labels = []
        for obj in placed_objects:
            Δp = p_actual - obj.belt_pos_anchor
            u_now = obj.u_drop + Δp          # object moves with belt
            v_now = obj.v_drop
            P_R = F @ (u_now, v_now, 1)
            pixel_center = M_inv @ (P_R.x, P_R.y, 1)   # M = F·H
            bbox = compute_bbox(pixel_center, obj.type, obj.θ_rand)
            labels.append(YOLO_label(obj.type, bbox))
        save_image_and_label(frame, labels)

  [Phase C — Return objects to boxes]
    belt.set_position(p_home)
    for obj in placed_objects (reverse order):
        pick from P_drop_R
        place back to source_box.xyz
```

### 5.4 Label Format

YOLO-style (normalized xywh + class + optional rotation for OBB):

```
<class_id>  <x_center>  <y_center>  <width>  <height>  [<angle_rad>]
```

All values normalized to [0, 1] by image dimensions. Angle `θ_rand` (from the dataset
script) provides the ground-truth rotation for oriented bounding-box (OBB) models.

### 5.5 Key Design Decisions (to confirm)

| # | Question | Impact |
|---|----------|--------|
| 1 | Positive encoder direction: downstream or upstream? | Sign of Δp in Phase B |
| 2 | Is `set_position` absolute or relative? | Idempotency, retry safety |
| 3 | Drop zone polygon (u,v) boundaries? | Ensures robot reach ∩ camera FOV |
| 4 | N_samples, N_frames, Δp_per_frame values? | Dataset size and diversity |
| 5 | 2 objects per sample only, or variable? | Occlusion handling complexity |
| 6 | Lens distortion already corrected? | Accuracy of H at image borders |

---

## 6. Siemens Packet Extension

### 6.1 DB1 — PC → PLC (grows from 12 → 16 bytes)

| Offset | Field | PLC Type | Python | Notes |
|--------|-------|----------|--------|-------|
| 0 | `CommandID` | DINT | `c_int32` | Unchanged |
| 4 | `rotate` | REAL | `c_float` | Unchanged |
| 8 | `speed` | REAL | `c_float` | Unchanged |
| 12 | `position` | REAL | `c_float` | **New** — target belt position (mm) |

### 6.2 DB2 — PLC → PC (grows from 16 → 24 bytes, Phase 1 layout)

| Offset | Field | PLC Type | Python | Notes |
|--------|-------|----------|--------|-------|
| 0 | `rotate_current` | REAL | `c_float` | Unchanged |
| 4 | `speed_current` | REAL | `c_float` | Unchanged |
| 8 | `task_doing` | DINT | `c_int32` | Unchanged |
| 12 | `task_state` | DINT | `c_int32` | Unchanged |
| 16 | `encoderA` | DINT | `c_int32` | **Phase 1** — raw quadrature count A |
| 20 | `encoderB` | DINT | `c_int32` | **Phase 1** — raw quadrature count B |

> Phase 1 ships raw quadrature counts. The originally-proposed `position_current`
> REAL is deferred to Phase 4 alongside the `set_position` write command.
> Belt position in mm is derived on the PC side by `EncoderDecoder` in
> `modules/conveyor.py`.

### 6.3 Mutual Exclusion via CommandID Semantics

`position` and `speed` are never valid simultaneously. The CommandID determines which
field the PLC reads; the other must be `0.0` (enforced by the PC-side builder):

| CommandID | Name | PLC reads | PLC ignores |
|-----------|------|-----------|-------------|
| 8 | `change_speed` | `speed` | `position` |
| **10** | **`set_position`** (new) | `position` | `speed` |
| 7 | `rotate_absolute` | `rotate` | both |
| 9 | `plan_siemen` | `rotate`, `speed` | `position` |

The `set_position` command is only issued by the `dataset_collector` scenario.
No production scheduler code path issues commandID 10.

### 6.4 Required Code Changes

| File | Change |
|------|--------|
| `EthernetCom.py` | Add `position` to `SiemensSendPacket`; add `position_current` to `SiemensReceivePacket` |
| `EthernetCom.py` | Add `COMMAND_ID["set_position"] = 10`; update `send_package` to zero the unused field |
| `test_module.py` | Track `position_current` state; simulate belt motion when `set_position` received |
| `modules/cli.py` | Add `setpos <mm>` command for debug use |
| `doc/plc_programing.md` | Update DB1/DB2 layout tables (12→16 bytes, 16→20 bytes) |
| TIA Portal DB1 | Add `position` REAL at offset 12 |
| TIA Portal DB2 | Add `position_current` REAL at offset 16 |

### 6.5 TIA Portal Note

Both DBs must retain **"Optimized block access" = OFF**. The BigEndianStructure rule
and no-`_pack_` rule in `EthernetCom.py` remain unchanged (all fields are 4-byte
aligned; no padding is needed or inserted).

---

## 7. Open Questions (Must Be Answered Before Implementation)

1. **Encoder sign convention**: does `position_current` increase as the belt moves
   downstream (toward the robot) or upstream (toward the camera)?
   Determines the sign of all Δp calculations.

2. **`set_position` absolute vs relative**: absolute is strongly preferred (idempotent,
   safe to retry). Requires that the encoder can be zeroed at a reference mark
   (`p_home`). Is there a physical home/zero sensor on the belt?

3. **Dataset-collection isolation**: confirm that the scheduler loop is completely
   stopped (not running in parallel) when `dataset_collector` issues `set_position`
   commands. This removes the need for interlocking logic.

4. **PLC-side position block**: has the Siemens program already been updated to read
   the encoder into a REAL register and expose it in DB2? If not, PLC code must be
   written before the PC side can use `position_current`.

5. **Camera lens distortion**: are undistorted images already available (e.g., the
   camera SDK provides undistorted frames), or does the Python side need to apply
   `cv2.undistort` before computing H?

6. **Belt surface Z**: is the belt surface reliably at `pickup_height` (flat and
   horizontal), or does it vary enough across (u, v) to require a Z offset map?
   If flat, the 2D homogeneous F described here is sufficient.

---

## 7.1 Frames Convention (per `doc/frames.png`)

- Camera frame and conveyor frame share the paper-aligned axes: `x_cam = x_con` horizontal-right, `y_cam = y_con` vertical-up.
- Robot frame is rotated about Z by angle θ relative to the conveyor frame. `F_CONVEYOR_TO_ROBOT` in [modules/conveyor.py](../modules/conveyor.py) has first column `(sin θ, cos θ)` and second column `(-cos θ, sin θ)` to reflect this; the translation column `(t_x, t_y)` is a placeholder until physical calibration.
- Consequence: any "drop point" used by simulations or scenarios should be expressed in C-frame `(u, v)` and projected through F at runtime. Hardcoding R-frame coordinates couples the test to the placeholder `(t_x, t_y)` and produces visually-misleading trajectories.
- `config.json.conveyor.accuracy_points_uv` (C-frame) is now the preferred source of evaluate-scenario targets; the legacy R-frame `scheduler.accuracy_points` remains only as a fallback.

## 8. Implementation Order

| Phase | Scope | Status |
|-------|-------|--------|
| **P1** | Encoder fields (encoderA/B) + ConveyorFrame F + EncoderDecoder + BeltTracker + C-frame scheduler + run_test visualization | **DONE** |
| **P2** | Implement calibration routines (F from touch-probe, H from checkerboard) in `calibration.py`; replace `F_CONVEYOR_TO_ROBOT` placeholder | Pending |
| **P3** | Rewrite `image_processing.py` with full TrackedObject lifecycle + real YOLO + CameraFrame pipeline | Pending |
| **P4** | Add `position` write field + `set_position` command (CommandID=10); implement `dataset_collector.py` | Pending |

---

*Document owner: AI/Robot team. Last updated: 2026-06-08.*
