# Delta Robot Pick-and-Place: Theory Basis & Future Research
> **Target Audience**: Human Developers, Thesis Students, and Researchers.
> **Note**: This file is maintained by the human team. It focuses on clarity, software flowcharts, and high-level concepts rather than hard physical realities. Some calibration parameters or temporary coordinate offsets might differ in implementation.

---

## 1. Software Architecture & Multitasking Model

The Delta Robot real-time control system utilizes a multitasking model combining **Multiprocessing** and **Multithreading**. This isolates network communication latency, ensuring the decision loop and perception loop run at high frequencies without blocking.

```mermaid
graph TD
    subgraph PC_Software [Control PC - Python]
        MainThread[Main Thread: Decision / Scheduler]
        
        subgraph Perception_Process [Perception Process]
            CapThread[Capture Thread: PyAV Cam Ingest 30 FPS]
            InferThread[Inference Thread: YOLO-OBB & Tracker]
        end
        
        subgraph Comm_Process [Background Comm Process]
            PLC_Worker[Communication Worker: snap7 + pylogix]
        end
    end
    
    subgraph Hardware [Hardware Layer]
        Cam[Physical Camera] -->|MJPEG Stream| CapThread
        PLC_Worker -->|EtherNet/IP| Omron[Omron NX1P2 PLC]
        PLC_Worker -->|Modbus TCP| Siemens[Siemens S7-1200 PLC]
    end
    
    CapThread -->|Queue Frame| InferThread
    InferThread -->|Detections| MainThread
    MainThread -->|Command Packets| PLC_Worker
```

### 1.1. Thread Isolation
The software separates execution into four distinct resource areas:
1.  **Background Communication Process** (`multiprocessing.Process`): Handles connections over socket, EtherNet/IP (`pylogix` to Omron), and Modbus TCP (`snap7` to Siemens). It exchanges packets with the main thread via IPC queues (`multiprocessing.Queue`), isolating network jitter.
2.  **Main Decision Thread**: Executes the core `Scheduler Loop`, calculating trajectory points, executing safety radius checks, and planning picks.
3.  **Perception Process Threads**:
    *   **Capture Thread**: Uses **PyAV** to decode camera frames at 30 FPS at 1080p.
    *   **Inference Thread**: Runs YOLO-OBB on the latest frame, tracks objects using the Centroid Tracker, and returns coordinates.
4.  **UI Web Server Thread**: Runs a lightweight in-process server (`modules/interface.py`) streaming telemetry and video to a web browser.

---

## 2. Coordinate Systems & Spatial Transformations

The sorting cell utilizes three coordinate frames to map camera pixel coordinates to physical robot arm targets.

```mermaid
graph TD
    V[Vision Frame: u_px, v_px] -->|Homography H| C[Conveyor C-frame: u, v]
    C -->|Homogeneous Transform F| R[Robot R-frame: X_R, Y_R, Z_R]
```

### 2.1. Robot Frame (R-frame)
* **Origin**: Centered under the top base plate of the delta structure.
* **Orientation**: Standard Cartesian right-hand rule.
* **Kinematics constraint**: Due to the parallel linkage design, the end-effector operates in the negative Z region.
  * $Z = 0$ mm: Arms fully retracted.
  * $Z = -305$ mm: End-effector lowered to the conveyor belt.

### 2.2. Conveyor Frame (C-frame)
* **Origin**: An arbitrary fixed physical point $O_C$ on the conveyor frame body chosen at calibration time.
* **Axes**: 
  * $u$: Along the flow direction of the belt (downstream).
  * $v$: Transverse coordinate across the belt width.

### 2.3. Homogeneous Transform matrix F ($C \to R$)
Since the conveyor belt surface is planar and parallel to the robot $XY$ plane, a 2D homogeneous transformation relates C-frame coordinates $(u, v)$ to R-frame coordinates $(X_R, Y_R)$ at the constant pickup height $Z_{\text{pickup}}$:

$$
\begin{bmatrix} 
X_R \\ 
Y_R \\ 
1 
\end{bmatrix} 
= \mathbf{F} 
\begin{bmatrix} 
u \\ 
v \\ 
1 
\end{bmatrix} 
= 
\begin{bmatrix} 
-\sin\theta & \cos\theta & T_X \\ 
\cos\theta & \sin\theta & T_Y \\ 
0 & 0 & 1 
\end{bmatrix}
\begin{bmatrix} 
u \\ 
v \\ 
1 
\end{bmatrix}
$$

* $\theta$: The angle between the conveyor flow axis ($+u$) and the robot axes.
* $(T_X, T_Y)$ represents the translation vector of the conveyor origin.

---

## 3. Object Tracking & Interception Prediction

To pick a moving component from the conveyor belt, the controller must predict where the item will be when the arm descends.

### 3.1. Fixed-Point Pick-Position Prediction (with a live position gate)
The iterative solver (`_predict_pick_position`) is **kept**, but its output is used to choose
a stable **pick position**, not to fire the pick on a converged *time*. This is the core of
the real-time rewrite: it makes the pick immune to the ±60% belt-speed estimate noise that
previously caused the "arrive → wait → miss" lag at object density.

**Step 1 — solve for the park position.** Iterate to the earliest goto-feasible pick:
1. Guess a pick time: $t_{\text{pick}}^{(0)} = t_{\text{now}} + \text{lead time}$.
2. Project the object forward at belt velocity $v_{\text{belt}}$:
   $$u(t_{\text{pick}}^{(k)}) = u_{\text{now}} + v_{\text{belt}} \cdot (t_{\text{pick}}^{(k)} - t_{\text{now}})$$
3. Map to R-frame and compute the robot travel duration $\Delta t_{\text{goto}}$.
4. Update: $t_{\text{pick}}^{(k+1)} = t_{\text{now}} + \Delta t_{\text{goto}} + t_{\text{delay}}$.
5. Repeat until convergence; apply the **1.6 s minimum lead** (`intercept_lead_time_s`) so the
   arm parks downstream of the object, and clamp $u_{\text{pick}}$ to the workspace edge for
   danger-zone objects. The result is the **parked pick position** $u_{\text{pick}}$ — a fixed
   straight-down point. If the arm cannot arrive before the object passes $u_{\text{pick}}$,
   the object is skipped (genuinely unreachable).

**Step 2 — fire on a live positional gate (no time math).** After the arm has parked at
$u_{\text{pick}}$, the pick is **not** scheduled by the predicted time. The main thread waits
on the claimed object's live position (refreshed by the perception thread) and dispatches the
pick the moment:
$$u_{\text{now}} \ge u_{\text{pick}} - \text{offset}(v_{\text{belt}})$$
Closing the loop on the live object — rather than trusting a frozen, noisy speed sample — is
what removed the mistimed wait.

**The lead `offset`.** There is a latency $T_{\text{delay}}$ between the instant the gate
condition is evaluated true and the instant the suction cup actually contacts the object
(command dispatch + PLC pickup + soft-start descent; in code
$T_{\text{delay}} \approx \texttt{command\_delay\_s} = \texttt{robot\_movement\_delay\_s} +
\texttt{ethernet\_delay\_s} \approx 50\text{ ms}$). During $T_{\text{delay}}$ the object keeps
moving downstream, so the gate must fire **early** by exactly that displacement. Because the
object's $u$ is anchored to the belt encoder (§3.2), this displacement *is* the belt's
displacement over $T_{\text{delay}}$. In the steady-belt case it is simply
$$\text{offset}(v_{\text{belt}}) = v_{\text{belt}} \cdot T_{\text{delay}}$$
which the system engineers to be the normal regime via the speed-change timing strategy (§6.6).
The full belt-acceleration form (the robustness fallback, derived in **§6.4**) is *not*
implemented: the §6.6 timing strategy keeps the belt steady at gate-fire time and the live gate
corrects any residual, so the precision path stays single-term. `_belt_lead_offset_mm` now
returns $v_{\text{belt}} \cdot T_{\text{delay}}$ — about 2.5 mm at 50 mm/s, growing with speed —
correcting the small systematic *late* pick the old `0.0` stub left behind.

### 3.2. Why Encoder-Based Dead-Reckoning is Superior
Integrating velocity over time ($\Delta t$) to track position accumulates drift and fails when the conveyor speed varies or stops. 

**Solution**: Anchor the object to the absolute encoder counter ($p(t)$, in mm) from the Siemens PLC at the moment of detection ($p_{\text{anchor}}$):

$$\Delta p(t) = p(t) - p_{\text{anchor}}$$
$$u(t) = u_{\text{anchor}} + \Delta p(t)$$

This method is **drift-free** because position is determined directly by physical belt movement rather than time integration.

### 3.3. Backdated Anchoring (Camera-Latency Compensation)

The anchor above is only drift-free if $p_{\text{anchor}}$ is the belt position **at the instant the frame was captured**, not when the detection is ingested. A frame travels through exposure, decode, YOLO inference and the perception poll before it reaches the scheduler — on the order of 80–150 ms — during which the belt advances $v_{\text{belt}}\cdot\Delta t_{\text{lat}}$ (≈4 mm at 50 mm/s, up to ≈24 mm at the adaptive cap). Anchoring a stale $u$ to the *current* belt position injects that as a fixed upstream error that freezes into the last camera fix once the object leaves the field of view.

**Solution**: stamp each frame with its decode time $t_{\text{cap}}$ (backdated by half the exposure, since photons integrate over the exposure window), keep a short history of $(t, p(t))$ samples from the perception thread, and anchor with the interpolated position at capture time:

$$p_{\text{anchor}} = p(t_{\text{cap}}), \qquad t_{\text{cap}} = t_{\text{decode}} - \tfrac{1}{2}t_{\text{exposure}}$$

The rest of the latency chain needs no separate term — the encoder already recorded where the belt was, so we simply look it up. Falls back to the current position when history is unavailable (static/simulated belt).

### 3.4. Oblique Intercept (Belt-Tracking Descent)

A vertical descent at a fixed R-frame point makes contact with the cup and board at a horizontal *relative* velocity equal to the belt speed, dragging the board during suction settling. The arm still **parks above the predicted arrival point** $\mathbf{p}_{\text{pick}}$ (unchanged from the straight-down logic, so the goto stays inside the workspace), but the pick-phase **contact point is shifted downstream** by the distance the board travels during the short descent:

$$\mathbf{p}_{\text{contact}} = \mathbf{p}_{\text{pick}} + \hat{u}\,\big(v_{\text{belt}}\cdot t_d\big), \qquad z_{\text{park}} = z_{\text{pre\_pick}},\; z_{\text{contact}} = z_{\text{pickup}}$$

where $\hat{u}$ is the belt-flow unit vector in the R-frame ($+\hat{u}$ = downstream) and $t_d$ is the modeled descent time (interpolator model, one fixed-point pass folding the belt slant into the segment length). The pre\_pick→pickup drop therefore slants downstream, so the cup follows the board and meets it with near-zero relative velocity. At $v_{\text{belt}}=0$ (or when disabled) the offset vanishes and the descent is vertical. The gate is **unchanged**: it fires a lead of $v_{\text{belt}}\cdot(T_{\text{delay}} + t_{\text{sampling}})$ ahead of $\mathbf{p}_{\text{pick}}$ — the descent's own slant absorbs the $t_d$ travel, so $t_d$ is *not* added to the gate lead.

> A superseded revision instead moved the *park/goto* upstream by $v_{\text{belt}}\cdot t_d$ and added $t_d$ to the gate lead; at operating belt speed that parked the arm upstream of $u_{\min}$ (outside the workspace, and past the reach circle at the adaptive cap). Only the **contact** is shifted, never the park. The feature is opt-in (`scheduler.oblique_descent_enabled`, default off); $t_d$ is over-estimated while `interp.a_max` (≈0.1 g) is uncalibrated, so at high belt speed the downstream contact can approach $u_{\max}$ — a calibration item, handled as a safe dispatch-reject rather than a clamp.

---

## 4. Yaw Orientation Resolution (360° Angle)

QFP/TQFP components have 180° rotational symmetry. YOLO-OBB only detects the tilt angle within $[-90^\circ, 90^\circ)$. To drive the suction rotation mechanism to an absolute 360° angle:

1. **Marker Detection**: YOLO recognizes the round locator dot marking pin/corner #1 on the component.
2. **Heading Vector**: Compute the vector from the component center $\mathbf{P}_{\text{board}}$ to the marker center $\mathbf{P}_{\text{marker}}$:
   $$\vec{\mathbf{v}} = (x_m - x_b, y_m - y_b)$$
3. **Heading Angle**:
   $$\theta_{\text{heading}} = \left(\text{atan2}(v_y, v_x) \cdot \frac{180}{\pi} + \theta_{\text{offset}}\right) \pmod{360}$$

### 4.1. Rotation Timeline & Angle Convention (three layers, revised 2026-07-06)

The suction rotation is **not** applied before the grip to pre-orient the cup; it is applied **after** the grip to normalise the *attached* board to a uniform bin orientation. Timeline: home the axis to 0 while the arm flies to the park point → gate fires → pick descends → once the arm lifts back to $z \ge z_{\text{pre\_pick}}$, rotate the board.

Angles live in exactly **three layers**, one unit each, converted only at the boundaries (the earlier "logical $[-180,180)$ degrees" middle layer was retired as confusing):

**Layer 1 — measurement (image pixels → R-frame radians).** The vision heading $h$ (degrees) is the board→marker vector measured with `atan2(dx, dy)` on raw pixels, i.e. referenced to the image **+y (down)** axis. The ROI→C→R position chain (`M_VISION_TO_CONVEYOR` axis swap composed with $F$'s reflection block) is a pure $+\theta_{\text{frame}}$ rotation of angles measured CCW from the ROI x axis; the two references differ by exactly $-90°$. The single conversion point (`ConveyorFrame.vision_heading_to_robot_rad`) is therefore

$$\varphi_{\text{board}} = \text{wrap}_{\pi}\!\big(\text{rad}(h - 90°) + \theta_{\text{frame}}\big)$$

with $\text{wrap}_{\pi}$ mapping to $[-\pi, \pi)$; $0$ = robot $+X$ axis, positive = CCW seen from above. Because the marker sits diagonally on the board (a corner dot on a square QFP is $\approx 45°$ off the edges), a **per-class constant** `vision.orientation.offset_by_class` is added to $h$ at measurement time (calibrated; falls back to the global `offset_deg`).

**Layer 2 — algorithm (R-frame radians only).** `TrackedObject.rotation_rad` stores $\varphi_{\text{board}}$; the plan's post-grip command is

$$\theta_{\text{cmd}} = \text{wrap}_{\pi}\!\big(s \cdot (\theta_{\text{offset}} - \varphi_{\text{board}})\big)$$

where $\theta_{\text{offset}}$ = `rotate_offset_deg` (config in degrees, converted once in `from_config`) is the bin orientation, and $s$ = `rotate_sign` $\in \{+1, -1\}$ flips the whole command if the physical axis turns opposite to the R-frame CCW convention (calibrated with `python3 -m modules.test_rotate`). The wrap to $[-\pi,\pi)$ is by itself the shortest-way rotation ($\le 180°$).

**Layer 3 — wire (IPC boundary only).** The Siemens rotation axis accepts **signed degrees in $[-360, 360]$, shares the R-frame zero** ($0° = 0\,\text{rad}$), and its command value encodes **both the axis position and the direction of travel**. So `main.py`'s worker sends the angle **verbatim** — a plain radians→degrees identity with **no wrap**, only clamped to $[-359, 359]$:

$$\theta_{\text{wire}} = \text{clamp}_{\pm 359}\!\big(\deg(\theta_{\text{cmd}})\big), \qquad \theta_{\text{cmd,fb}} = \text{rad}(\theta_{\text{wire}})$$

(`robot_rad_to_wire_deg` / `wire_deg_to_robot_rad` in `EthernetCom.py`; the clamp keeps the axis off the $\pm 360$ edge, where the teammate's ST program can flip direction; status feedback is exposed verbatim in R-frame **degrees** for logs/dashboard). **The minimal-turn decision is made ONCE upstream** — the scheduler wraps $\theta_{\text{cmd}}$ to $[-\pi,\pi)$ relative to the homed $0$ (Layer 2) — **not at this boundary.** Wrapping here would map $180° \to -180°$ and $270° \to -90°$; because the sign selects the spin direction, a $179°\to180°$ step would then drive the axis nearly a full turn the wrong way instead of $+1°$ (the "random"/over-rotation symptom). **Superseded (2026-07-06):** two earlier revisions were wrong — first a $[0,360)$ premise with a $+180$ shift (drove "home to 0" to physical $180°$), then a $[-360,360]$ identity that still *wrapped* to $[-180,180)$ at the boundary (the $180\to-180$ flip above). Both removed.

**Calibration & diagnostics.** `rotate_sign`, `rotate_offset_deg` and `offset_by_class` must be calibrated on hardware: (1) `python3 -m modules.test_rotate` — Siemens-only probe: remap/settle check, implied axis speed (deg/s), **visual** direction check (mark the cup; +90° must turn CCW from above, else `rotate_sign: -1`), and a cmd-7→cmd-7 retrigger test (DB1 has no handshake; if the ST edge-triggers on CommandID change, consecutive rotates are dropped). (2) One production/test run reading the per-pick `[ROTATE]` log (`vision_angle`, `board_heading`, `rotate_cmd`, `rotate_at_gate`, `rotate_at_end`): `rotate_at_gate` far from 0 ⇒ the home-to-0 did not finish before the grip (axis too slow for the cycle); `rotate_at_end` far from `rotate_cmd` ⇒ the board was released mid-rotation. `scheduler.rotate_home_tolerance_deg` (> 0 to enable) turns the first condition into a `[WARN]` at pick time — warn-only, the positional gate is never delayed.

---

## 5. Flowcharts & State Machines

### 5.1. CLI Interactive Mode Flowchart
CLI mode allows engineers to type commands directly to test motion configurations.

```mermaid
flowchart TD
    Start([Start CLI Mode]) --> ReadInput[/Read Keyboard Input/]
    ReadInput --> Parse{Parse Command Syntax?}
    
    Parse -- Valid Command --> BuildPacket[Pack Command ID & Payload]
    Parse -- Syntax Error --> PrintError[Show Syntax Error] --> ReadInput
    
    BuildPacket --> CheckSafety{Target within mechanical reach radius?}
    
    CheckSafety -- No --> AlertOut[Raise WorkspaceLimitError & Abort] --> ReadInput
    CheckSafety -- Yes --> SendQueue[Push to Communication Queue]
    
    SendQueue --> WorkerSend[Background Comm Worker writes to PLC]
    WorkerSend --> WaitDone{Wait for PLC to reset bit_doing to 0?}
    
    WaitDone -- Yes --> Success[Print Success Message] --> ReadInput
    WaitDone -- Timeout --> TimeoutAlert[Warn PLC Connection Lost] --> ReadInput
```

---

### 5.2. Production Scenario Execution Flowchart
The production scenario coordinates camera, conveyor encoder, and robot arm.

```mermaid
flowchart TD
    Start([Start Production Mode]) --> InitSystems[Init Camera, PLC & Load config.json]
    InitSystems --> ReadPLC[Read conveyor position p from Siemens]
    
    ReadPLC --> Capture[Capture camera frame]
    Capture --> YOLO{Run YOLO-OBB detection?}
    
    YOLO -- Yes --> ResolveYaw[Resolve centroid and 360° absolute heading from marker]
    ResolveYaw --> TrackUpdate[Update Centroid Tracker and maintain object ID]
    TrackUpdate --> Ingest[Ingest new item with encoder anchor p_anchor]
    YOLO -- No --> Skip1[Skip perception step]
    
    Ingest & Skip1 --> Prune[Prune stale/passed items from list]
    
    Prune --> EmitEvents[Emit SPEED/DETECT + dashboard events]
    EmitEvents --> ReadPLC
```

> **Note:** the boxes above run on the **perception/state thread** (`_realtime_perception_loop`,
> ~25 ms). It owns the single PLC status read and keeps shared `RealtimeState` (belt
> position/speed, `pos_EE`, the tracker, claimed ids) fresh. The **main decision/execution
> thread** below runs concurrently and reads that shared state — its wait loops issue no PLC
> I/O of their own. The two share an `ipc_lock` (one round-trip in flight) and a `state_lock`.

```mermaid
flowchart TD
    Loop([Main loop]) --> Snapshot[Snapshot tracked objects + belt under state_lock]
    Snapshot --> Select{Unclaimed catchable object?}
    Select -- No --> Loop
    Select -- Yes --> Priority[Select highest priority: danger-zone tier first, else shortest pick to-bin cycle]
    Priority --> Predict[Solve P_pick via fixed-point iteration + 1.6s lead + edge clamp]
    Predict --> Reach{Arm can reach P_pick in time?}
    Reach -- No --> Loop
    Reach -- Yes --> Build[Build 7-point Goto & Pick trajectories]
    Build --> Claim[Mark object CLAIMED so perception thread will not prune it]
    Claim --> SendGoto[Dispatch Goto to Omron]
    SendGoto --> WaitGoto{Shared pos_EE reached parked pick point?}
    WaitGoto -- Reached --> GateObject{Live object u >= parked pick u - offset?}
    GateObject -- Reached --> SendRotate[Dispatch suction rotation to Siemens]
    SendRotate --> SendPick[Dispatch Pick: straight-down descent then transfer to bin]
    SendPick --> WaitPick{Shared pos_EE reached release/place point?}
    WaitPick -- Released --> MarkComplete[Unclaim + remove object, advance arm position to bin]
    MarkComplete --> Loop
```

> The **positional pick gate** (`GateObject`) replaces the old "re-predict object arrival
> time" step: once the arm is parked, the pick fires purely on the live object reaching
> $u_{\text{pick}}$ — no time recomputation, immune to belt-speed estimate noise.

---

### 5.3. TrackedObject Lifecycle State Machine
Each object on the belt is managed through a state machine to optimize memory and CPU.

```mermaid
stateDiagram-v2
    [*] --> NEW : Detected for the first time
    NEW --> TRACKED : Centroid matched for >= 3 frames
    TRACKED --> DEAD_RECKONED : Object exits camera FOV
    DEAD_RECKONED --> DONE : Successfully picked
    DEAD_RECKONED --> DONE : Passed workspace u_max limit
    DONE --> [*] : Pruned from tracker memory
```

---

## 6. Adaptive Conveyor Speed Design

**Goal: preserve the delta arm's throughput in an *unstable feeder* environment.** Upstream
supply is bursty and irregular; the serial arm is the valuable, rate-limited resource. The belt
is therefore treated as a **rate regulator** — its job is to keep the arm fed at a steady
*presentation rate*, **not** to maximise its own speed. Belt speed is set **inversely** to
product density so the rate of pickable objects arriving at the workspace stays near a nominal
target (§6.1, §6.5). The speed-change *timing* is chosen so the belt's acceleration ramp never
enters a precision-critical computation (§6.6).

> **Rejected (the old "run fast when busy" model).** Earlier drafts made belt speed rise with
> density (`v = A·N + v_min`) and patched a "backlog brake" on top. This is backwards in
> nature: belt speed does **not** set robot throughput — the serial arm's pick cycle does
> (`μ_max`, §6.2). Speeding the belt up under high density only shortens each object's transit
> time and pushes objects past `u_max` unpicked, *increasing* misses. The corrected model below
> regulates a *rate*, with a single law and no contradictory terms.

### 6.0. Belt kinematics & bounds

| Symbol | Meaning | Value |
|---|---|---|
| $v_{\min}$ | operational floor — hardware control is imprecise below 20 mm/s, so a margin is kept (`belt_speed_min_mm_s`) | 30 mm/s |
| $v_{\text{cap}}$ | pickability ceiling — *derived* $= L/t_{\text{transit}}$, the arm must still intercept (§6.2) | ≈ 87.5 mm/s |
| $v_{\text{hw,max}}$ | hardware effective maximum, hard clamp on $v_{\text{cap}}$ (`belt_speed_hw_max_mm_s`) | 200 mm/s |
| $a_{\text{nom}}$ | nominal belt acceleration magnitude (`belt_accel_mm_s2`) | 22.31 mm/s² |
| $T_{\text{ramp}}$ | acceleration **build** interval (jerk phase: accel ramps $0 \to a_{\text{nom}}$ and back; `belt_ramp_s`) | ≈ 0.25 s |
| $T_{\text{delay}}$ | dispatch→grip latency at the pick gate (§3.1) = `robot_movement_delay_s` + `ethernet_delay_s` | ≈ 50 ms |
| $\mu_{\max}$ | robot maximum pick rate $= 1/t_{\text{pick}}$ (`pick_cycle_s`, a **calibrated** value) | ≈ 0.50 obj/s |
| $\lambda_{\text{nom}}$ | nominal presentation-rate target $= k\,\mu_{\max}$ (`belt_speed_headroom` $k$) | ≈ 0.38 obj/s |

The adaptive controller operates inside $[v_{\min}, v_{\text{cap}}]$ — well below the 200 mm/s
hardware limit, because pickability (§6.2), not the motor, is the binding constraint. The
*setpoint policy* (which speed inside that band, §6.1/§6.5) is driven by holding the arm's
presentation rate at $\lambda_{\text{nom}}$, deliberately below the arm's ceiling $\mu_{\max}$.

> **Implementation note.** As of the adaptive-speed implementation these symbols are config keys
> under `scheduler` (opt-in via `adaptive_speed_enabled`). $t_{\text{pick}}$ is **calibrated** —
> the real arm runs faster than the original 2.5 s assumption — so `pick_cycle_s` defaults to
> 2.0 s; set it from the measured `throughput_pick_per_min` in `data.log`. The controller is
> `_adaptive_belt_speed()` and the gate offset is `_belt_lead_offset_mm()` in `modules/scheduler.py`.

### 6.1. Rate-Regulation Control Model (inverse density law)
The arm is fed at a **presentation rate** $\lambda = \rho \cdot v$ (linear density × belt speed:
objects/mm × mm/s = objects/s). To hold $\lambda$ at the nominal target $\lambda_{\text{nom}}$,
solve for $v$ and clamp to the physical band:
$$v = \text{clamp}\!\left(\frac{\lambda_{\text{nom}}}{\rho},\; v_{\min},\; v_{\text{cap}}\right)$$
* $\rho$: **linear density** of **unclaimed, not-yet-passed** products — count per unit belt
  length over the density region, $\rho = N / L_{\text{meas}}$.
* $N$: that same object count (objects with $0 \le u \le u_{\max}$); $L_{\text{meas}}$: the region
  length. The law is thus equivalently $v = \lambda_{\text{nom}} \cdot L_{\text{meas}} / N$ —
  **hyperbolic** (inverse) in $N$, *not* linear.
* $L_{\text{meas}}$ spans the C-frame origin $O_C$ to $u_{\max}$, i.e. $L_{\text{meas}} = u_{\max}$
  (= `workspace_window_uv[1]` = 363 mm). This works because $O_C$ coincides with the ROI origin,
  so no separate camera window is needed (`belt_density_length_mm = 0` ⇒ derive $= u_{\max}$).
* $v_{\min}, v_{\text{cap}}$: the §6.0 floor and *derived* pickability ceiling (30 / ≈87.5 mm/s).

**Direction is the whole point: more density ⇒ *slower* belt.** A sparse feeder is sped up so
the few available objects reach the arm before it starves; a dense feeder is slowed so the
serial arm gets transit time and objects are not shoved past $u_{\max}$. (Contrast the rejected
`v = A·N + v_{\min}` of the §6 note, which had the sign backwards because it regulated belt
speed as if it were robot throughput.)

### 6.2. Parameter Calculations
**Pickability ceiling (derived).** With the workspace window length $L = u_{\max} - u_{\min} =
363 - 188 = 175$ mm, the arm needs the object in-window long enough to intercept it:
* Transit time: $t_{\text{transit}} \geq 2.0$ s (`pick_transit_min_s`).
* $v_{\text{cap}} = \min(L / t_{\text{transit}},\, v_{\text{hw,max}}) = \min(175/2.0,\, 200)
  \approx 87.5$ mm/s.

**Rate target.** The arm's ceiling is $\mu_{\max} = 1/t_{\text{pick}} = 1/2.0 = 0.50$ obj/s
(calibrated; the arm beats the old 2.5 s assumption). We hold the presentation rate at
$\lambda_{\text{nom}} = k\,\mu_{\max} = 0.75 \times 0.50 = 0.375$ obj/s, leaving 25 % headroom
(§6.5). With $L_{\text{meas}} = u_{\max} = 363$ mm, the inverse law
$v = \lambda_{\text{nom}} \cdot L_{\text{meas}} / N = 136.1 / N$ gives:
* $N = 1$ ⇒ $v = 136$ mm/s, **clamped to $v_{\text{cap}} = 87.5$** (sparse / feeder-limited).
* $N = 2$ ⇒ $v = 68$ mm/s (regulated band).
* $N = 3$ ⇒ $v = 45$ mm/s (regulated band).
* $N \geq 5$ ⇒ $v \leq 27$ mm/s, **clamped to $v_{\min} = 30$** (overload; floor reached).

So a single constant — $\lambda_{\text{nom}}$ — plus the band $[v_{\min}, v_{\text{cap}}]$
replaces the old per-product slope $A$.

### 6.3. Belt acceleration & ramp model
$a_{\text{nom}}$ and $T_{\text{ramp}}$ describe **different** things and must not be conflated.
The physical speed transition is an S-curve: the acceleration itself ramps $0 \to a_{\text{nom}}$
over $T_{\text{ramp}} \approx 0.25$ s (the *jerk phase*), holds at $a_{\text{nom}}$, then ramps
back to $0$. The **constant-acceleration approximation** treats the whole transition as a single
constant $a = \pm a_{\text{nom}}$ (sign $+$ for speed-up, $-$ for slow-down). Under it, the time to
move from a current speed $v_c$ to a setpoint $v_{sp}$ is **not** a fixed 0.25 s but
$$T_{\text{accel}} = \frac{|v_{sp} - v_c|}{a_{\text{nom}}}.$$
$T_{\text{ramp}} \approx 0.25$ s is then just the jerk-phase error band at the two edges of this
ramp — the term we deliberately neglect (§6.6).

### 6.4. Lead-offset coupling
This generalises the pick-gate `offset` of §3.1. Because the object's $u$ is anchored to the belt
encoder, the lead offset equals the belt's displacement over the latency window $T_{\text{delay}}$:

* **Steady belt** ($a = 0$, the engineered normal regime):
$$\text{offset} = v_{\text{belt}} \cdot T_{\text{delay}}$$
* **Mid-ramp, ramp does *not* finish within $T_{\text{delay}}$** ($T_{\text{accel}} \ge T_{\text{delay}}$):
$$\text{offset} = v_c \cdot T_{\text{delay}} + \tfrac{1}{2}\,a\,T_{\text{delay}}^{2}$$
* **Mid-ramp, ramp finishes within $T_{\text{delay}}$** ($T_{\text{accel}} < T_{\text{delay}}$) — split into two phases:
$$\text{offset}_1 = v_c \cdot T_{\text{accel}} + \tfrac{1}{2}\,a\,T_{\text{accel}}^{2} = \tfrac{v_c + v_{sp}}{2}\,T_{\text{accel}}$$
$$\text{offset}_2 = v_{sp} \cdot (T_{\text{delay}} - T_{\text{accel}})$$
$$\text{offset} = \text{offset}_1 + \text{offset}_2$$

These forms are continuous: at $T_{\text{accel}} = T_{\text{delay}}$ the two-phase result equals
the single-phase one (with $\text{offset}_2 = 0$), and at $a = 0$ all forms reduce to
$v_{\text{belt}} \cdot T_{\text{delay}}$. The timing strategy of §6.6 guarantees the belt is
steady at gate-fire time, so the gate uses the simple $v_{\text{belt}} \cdot T_{\text{delay}}$;
the acceleration forms are the **robustness fallback** for the rare case where a gate fires while
a ramp is still settling.

### 6.5. Control law: rate regulation with three regimes
There is **one** law — the inverse rate regulator of §6.1,
$v_{\text{target}} = \text{clamp}(\lambda_{\text{nom}}/\rho,\, v_{\min},\, v_{\text{cap}})$ — and
its clamps produce three operating regimes. The arm is **physically serial — no pipelining**, so
the band edges, not a separate brake, do the protective work.

1. **Feeder-limited (sparse).** $\rho$ small ⇒ the law wants $v > v_{\text{cap}}$, so it clamps
   to $v_{\text{cap}}$. Arrival $\rho \cdot v_{\text{cap}} < \lambda_{\text{nom}}$: the arm is
   under-fed and the belt is already doing its best. Throughput is feeder-limited; nothing the
   belt can do raises it.
2. **Regulated (normal).** $v_{\text{target}} = \lambda_{\text{nom}}/\rho \in (v_{\min}, v_{\text{cap}})$.
   Arrival is held at $\lambda_{\text{nom}}$, i.e. **70–80 % arm utilisation**. The sub-unity
   target is deliberate **headroom**: it absorbs feeder bursts and timing jitter. This is *why*
   the nominal operating point is below 100 % — running flat-out would leave no slack and any
   burst would overflow.
3. **Overload (sustained dense).** $\rho$ so high the law pins $v$ at $v_{\min}$. The window
   stays full, the arm never idles, and **utilisation rises to ~100 % automatically** — the
   §6.2 headroom is intrinsically *spent* draining the burst. This is an **emergent** outcome of
   the $v_{\min}$ clamp + sub-unity $\lambda_{\text{nom}}$, **not** a separate control action: at
   $v_{\min}$ there is no slower setpoint to fall to, so the arm simply works at capacity. The
   backlog count $B$ — in-window unpicked objects predicted to pass $u_{\max}$ before the serial
   arm clears them (proxy: in-window unpicked count $-\,1$) — is the **detector** that overload
   has been entered, used only for telemetry/alerting. If the feeder rate exceeds $\mu_{\max}$
   even here, loss is unavoidable: the cell is genuinely over-saturated and no speed policy can
   recover it.

This resolves the dense-feeder counter-argument directly: utilisation climbs from the nominal
70–80 % to 100 % on its own, with no second control term fighting the first.

**Spacing cap (cluster constraint), added 2026-07-02.** The three-regime law above regulates
the *average* presentation rate $\lambda = \rho v$ but is blind to how objects are *spaced*: a
count $N$ over the fixed length $L_{\text{meas}}$ treats a tight pair (40 mm apart) and a spread
pair (300 mm apart) identically — same $\rho = N/L$, same commanded $v$. Yet the serial arm needs
$t_{\text{pick}}$ between consecutive picks, so the binding constraint for a *burst* is the
inter-arrival time of adjacent objects, $s_i / v \ge t_{\text{pick}}$, i.e.
$v \le s_i / t_{\text{pick}}$. The averaged law under-slows here: at $N{=}2$ it commands
$\lambda_{\text{nom}} L / 2$ regardless of whether those two objects are 40 mm or 300 mm apart,
and by the time the backlog grows $N$ enough to drop $v$ to the floor, the tight trailing object
has already passed $u_{\max}$ unpicked (a *dropped* pick, not just lost throughput). We therefore
layer a **spacing ceiling** on top of the density law:
$$v_{\text{target}} = \max\!\big(v_{\min},\; \min(\,v_{\rho},\; g_{\min}/t_{\text{pick}}\,)\big),\qquad
g_{\min} = \min_i (u_i - u_{i+1})$$
over the leading `_SPACING_LEAD_OBJECTS` (default 4) objects sorted toward the pick point — only
the *imminent* cluster constrains the belt, so a tight group still far upstream does not force a
premature slow-down. A cluster tighter than $v_{\min}\!\cdot t_{\text{pick}}$ pins the belt to its
floor (best-effort; the trailing object may still be missed — the cell is locally over-dense and
no belt speed catches both). This changes the *average*-rate regulator into a **rate-and-spacing**
regulator: throughput regulation when objects are spread, burst protection when they cluster —
the failure mode the count-only law could not see.

**Anti-thrash** (each avoided speed change is one fewer acceleration ramp):
* **Deadband** — only issue `change_speed` if $|v_{\text{target}} - v_{\text{setpoint}}| > \Delta_{\min}$ (≈ 5–10 mm/s).
* **Step limit** — each commit moves the setpoint at most `belt_speed_max_step_mm_s`
  (default 20 mm/s) toward the target, so every individual ramp settles within
  $\Delta v_{\max} / a_{\text{nom}} \approx 0.9$ s. This tames the raw hyperbolic law,
  which at small integer $N$ is near bang-bang (the $N{=}1 \leftrightarrow 2$ jump is
  ≈ 54 mm/s ⇒ a 2.4 s ramp at $a_{\text{nom}} = 22.31$ mm/s² — longer than a pick).
* **Throttle** — commits are dispatched at most every ≈ 0.75 s (each costs a Siemens
  round-trip), except the immediate grip-instant commit.

### 6.6. Jerk evaluation & timing avoidance
**Cost of modeling jerk explicitly:** the $\tfrac{1}{2}a t^2$ term would be replaced by the
integral of a jerk-limited S-curve velocity profile (the same family as the PLC's
`MC_inter_curve_vel` trajectory): a piecewise *cubic* with 3–7 segments of bookkeeping, and it
needs a belt jerk limit the hardware spec does not give directly (only $a_{\text{nom}}$ and
$T_{\text{ramp}}$ are known).

**Benefit:** at $T_{\text{delay}} \approx 50$ ms the jerk term shifts the displacement by
**sub-millimetre**, and it applies *only* during a ramp — which the strategy below pushes entirely
into the goto/park phase, where the **live positional gate corrects it anyway**. High cost,
negligible benefit ⇒ **explicit jerk modeling is rejected.**

**Timing avoidance (chosen, revised):** density $N$ is sensed **continuously** by the perception
thread (every ~25 ms tick, under `state_lock`, pure arithmetic — see §5.2), refreshing
$v_{\text{target}}$ live instead of once per multi-second pick cycle. The speed change itself is
committed **at the grip instant** — immediately after the pick packet is dispatched (the arm is
already descending/gripping; the *next* object's positional gate has not yet been evaluated) — and
again when the loop is idle (no candidate) so a cleared belt ramps back toward $v_{\text{cap}}$ to
fetch the next objects fast. Consequences:
* a density change ("an object was just lifted") is reflected immediately at the pick that caused it, not deferred to the start of the next ~multi-second cycle — this was the responsiveness fix (the controller previously looked mistimed/laggy);
* the 0.25 s accel-build plus the $T_{\text{accel}} = \Delta v / a_{\text{nom}}$ ramp complete during the *current* pick's descend/grip/lift/return and the *next* object's goto/park, so by the time the **next** positional gate matters the belt is already steady at $v_{\text{target}}$;
* the gate-time offset still reduces to $v_{\text{target}} \cdot T_{\text{delay}}$ ($a = 0$) — no accel/jerk term in the precision-critical computation;
* any in-flight position drift during the ramp is absorbed by the live gate (the core property of the §3 redesign);
* the deadband still caps commits to one per pick (plus the idle case), never during descent/grip of the pick that triggers it — the commit happens *after* that pick's descent is already dispatched.

**Predictor coupling (removed):** an earlier revision projected the in-flight object at the
post-ramp $v_{\text{target}}$ inside the same plan-build call that committed the speed change. Now
that the commit is decoupled from plan-build (it happens at the *previous* pick's grip instant),
the live encoder sample (`state.belt_speed_mm_s`) already reflects the settled post-ramp speed by
the time the next plan is built — no explicit coupling term is needed.

**Revision 2026-07-02 — opportunistic commits (grip-instant-only was too narrow):** the
grip-instant + idle policy above produced at most **one commit per pick cycle (2–10 s)** —
`execute()` blocks the main loop through goto/park/gate-wait, so density changes sensed by the
25 ms perception tick sat uncommitted for seconds (belt felt laggy and uneven on hardware). It
also failed quantitatively: the $N{=}1\leftrightarrow2$ jump of ≈ 54 mm/s needs
$T_{\text{accel}} = \Delta v / a_{\text{nom}} \approx 2.4$ s, longer than the
descend/grip/lift/return window the old text claimed would absorb the ramp. The policy is now
**inverted**: commits are allowed *any time* — from the executor wait loops (goto flight, far
gate wait, post-grip return) and the idle loop, throttled to ≥ 0.75 s apart — **except** inside
the *gate-critical window*, i.e. once the tracked object is within ≈ 2 s of belt travel of the
fire threshold (`RealtimeState.gate_critical`). Combined with the §6.5 step limit (ramps ≤ 0.9 s)
the belt is genuinely steady whenever a gate fires, restoring the §6.4 steady-belt assumption by
construction rather than by hope. Two supporting changes: (a) **closed loop** — the perception
thread stores the PLC's `speed_current`; if the measured speed still diverges from the setpoint
> 3 s after the last commit, the setpoint is re-sent (catches lost/clamped commands; the old
controller compared against a phantom setpoint it had only assumed was applied); (b) the pick-gate
lead offset now also covers the **sampling latency** (gate poll /2 + perception tick /2 ≈ 37 ms)
on top of $T_{\text{delay}}$, and each pick logs a `[GATE]` line
(`gate_to_dispatch_s` / `dispatch_to_contact_s`) so `robot_movement_delay_s` can be calibrated
from measured data instead of assumed (the configured 10 ms is far below the physical
dispatch→contact chain of soft-start + 13 mm descent ≈ 0.3–0.5 s; at 50–110 mm/s belt that is a
15–50 mm downstream contact offset if left uncalibrated).

### 6.7. Operational logic flow
Two threads (§5.2): the **perception thread** ticks every ~25 ms, refreshing the live belt
sample, the tracker, and — when adaptive speed is enabled — re-deriving $v_{\text{target}}$ from
the current density $N$ (pure arithmetic under `state_lock`). The **main decision/execution
thread** builds and runs one pick plan at a time and commits the speed change:

```mermaid
flowchart TD
    subgraph Perception["Perception thread (~25 ms tick)"]
        PTick([Tick]) --> PSample[Sample belt + poll vision; update tracker under state_lock]
        PSample --> PDensity[N = unclaimed objects with 0<=u<=u_max]
        PDensity --> PTarget[state.belt_speed_target_mm_s = adaptive_belt_speed(N)]
        PTarget --> PTick
    end

    subgraph Main["Main decision/execution thread"]
        Loop([Main loop]) --> Build[Build pick plan from live sample; select unclaimed catchable object]
        Build --> Select{Plan found?}
        Select -- No --> Idle[Commit adaptive speed if due belt relaxes toward v_cap]
        Idle --> Loop
        Select -- Yes --> DispatchGoto[Dispatch Goto]
        DispatchGoto --> WaitPark[Wait for arm to park]
        WaitPark --> Gate{Live object u reached u_pick minus offset?  offset is v times T_delay}
        Gate -- Reached --> DispatchPick[Dispatch rotate + Pick: grip at u_pick]
        DispatchPick --> Commit[Commit adaptive speed if beyond deadband fire-and-forget]
        Commit --> WaitLift[Wait for arm to lift and return to park]
        WaitLift --> Done[Unclaim + remove object from tracker; advance arm to bin]
        Done --> Loop
    end

    PTarget -.->|state.belt_speed_target_mm_s, under state_lock| Commit
```

> The speed commit fires **right after the pick dispatch** (the grip instant), not at the goto —
> so a density drop from "this object was just lifted" is applied immediately rather than waiting
> for the next ~multi-second cycle. Its 0.25 s ramp settles during the *current* pick's
> descend/grip/lift/return and the *next* object's goto/park, so by the time the **next** gate is
> evaluated the belt is steady — the commit is never on the gate's own critical path (the gate for
> the pick that triggers the commit has already fired). The picked object is removed from the
> tracker in `Done`, never left to be re-selected as a candidate (§6.8).

### 6.8. Pick-attempt exclusivity (no re-pick)
There is **no intentional slip-detection/retry logic** anywhere in the scheduler — `execute()`
returns success once the arm's *motion* completes; suction is never verified. An earlier
implementation gap meant a completed pick's object stayed in the tracker (only excluded from
candidate selection by `claimed_object_ids`, which is cleared right after `execute()` returns),
so it could be re-selected on the very next plan build — a "pick plan for a board that no longer
exists." Fixed by removing the object from the tracker as part of the same post-`execute()`
bookkeeping (`Done` in §6.7), plus a defensive `planned_object_ids` skip in the candidate loop.

**Scope (revised 2026-07-02): exactly-once applies to *dispatched* picks only.** The original fix
removed the object for *every* failure, including pre-grip aborts (goto timeout, gate stall,
track lost) — permanently dropping an object that was still on the belt and pickable, and, worse,
deleting it from the density count $N$ so the belt was commanded *faster* right when a pick had
just failed. Now the post-`execute()` bookkeeping keys off `plan.debug_info["pick_dispatched"]`:
* **Pick dispatched** (success, or failure after the grip was commanded): remove from tracker +
  `planned_object_ids` — a possible suction miss is never retried (suction is not verified).
* **Aborted before the grip**: only unclaim; the object stays tracked and is eligible for
  re-planning on the next build.

Relatedly, the pick gate's abort condition itself changed from a fixed wall-clock deadline
(`predicted_pick_time + margin` — which fired whenever the belt slowed after plan-build, e.g.
mid adaptive ramp) to a **progress-based stall check**: abort only if the object's
encoder-anchored $u$ advances < 0.5 mm for several seconds, or its track disappears.

---

## 7. Future Ideas & Research Proposals

### 7.1. Web GUI Dashboard
FastAPI backend with a Vue.js/React frontend visualizing:
* Live 3D Delta arm end-effector trajectory.
* Telemetry graphs plotting positional error from `data.log`.
* Active conveyor queues and database records of sorted items.

### 7.2. SQL Sorting Database Schema
```sql
CREATE TABLE product_types (
    type_id VARCHAR(50) PRIMARY KEY,
    description VARCHAR(255),
    sorting_destination_x REAL NOT NULL,
    sorting_destination_y REAL NOT NULL,
    sorting_destination_z REAL NOT NULL
);

CREATE TABLE pick_history (
    pick_id VARCHAR(50) PRIMARY KEY,
    object_id VARCHAR(50) NOT NULL,
    product_type VARCHAR(50) REFERENCES product_types(type_id),
    detected_timestamp REAL NOT NULL,
    picked_timestamp REAL,
    actual_speed_x REAL,
    actual_speed_y REAL,
    status VARCHAR(20) DEFAULT 'planned' -- 'completed', 'failed', 'stale'
);
```

### 7.3. Closed-Loop Conveyor Control
Implement closed-loop speed scaling using Little's Law to dynamically adjust speed setpoints written to the S7-1200 based on incoming queue pressure, avoiding overflow at the downstream limits.

> The **rate-regulation law of §6.5** (presentation rate held at $\lambda_{\text{nom}}$ via the
> inverse density law; overload drains emergently at the $v_{\min}$ clamp) is the near-term
> realization of this idea; a full Little's-Law queueing model is the longer-term evolution.
