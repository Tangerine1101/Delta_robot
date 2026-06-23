# Custom Function Block: `MC_Inter_Curve_Vel`

This document provides a rigorous mathematical analysis and detailed algorithm walkthrough of the custom Sysmac Studio Function Block **`MC_Inter_Curve_Vel`**, based on the Structured Text implementation in [MC_Inter_Curve_Vel.txt](file:///d:/1_Uni_ute/Graduation_Project/Delta_All/1.sysmac_code/Function_Blocks/MC_Inter_Curve_Vel.txt).

---

## 1. Functional Role in the Delta Robot System

The **`MC_Inter_Curve_Vel`** function block is the core **Real-Time Trajectory Generator (Interpolator)** of the Delta Robot.

Standard PLC motion instructions (e.g., `MC_MoveAbsolute`) command individual axes independently. For a Delta Robot — a parallel kinematic mechanism — independent axis commands produce severe path deviations because the three arms are mechanically coupled to a single end-effector platform. A commanded straight line in Cartesian space requires **coordinated, simultaneous** changes in all three joint angles at every control cycle.

This function block solves the problem by executing a closed-loop interpolation pipeline every **4ms** (the EtherCAT PDO cycle):

1.  **Input:** Receives three Cartesian waypoints: start $\mathbf{A}$, end $\mathbf{B}$, and look-ahead $\mathbf{C}$.
2.  **Path Parameterization:** Computes displacement $S(t)$ along the line $\overline{AB}$ using either a **polynomial S-Curve** or a **trapezoidal** velocity profile.
3.  **Coordinate Mapping:** Maps $S(t)$ to Cartesian coordinates $(X_t, Y_t, Z_t)$ via linear interpolation.
4.  **Inverse Kinematics:** Converts Cartesian coordinates to joint angles $(\theta_1, \theta_2, \theta_3)$ using [Calc_Inverse_Kinematics](file:///d:/1_Uni_ute/Graduation_Project/Delta_All/1.sysmac_code/Functions/inverse_kinematics.md).
5.  **Servo Command:** Streams angles to the three servo drives via `MC_SyncMoveAbsolute` for synchronized execution.

---

## 2. Variables Interface

### Inputs
| Variable | Type | Description |
| :--- | :--- | :--- |
| `X_A, Y_A, Z_A` | `LREAL` | Cartesian coordinates of segment start point $\mathbf{A}$ (mm) |
| `X_B, Y_B, Z_B` | `LREAL` | Cartesian coordinates of segment end point $\mathbf{B}$ (mm) |
| `X_C, Y_C, Z_C` | `LREAL` | Cartesian coordinates of next waypoint $\mathbf{C}$ (for corner blend look-ahead) |
| `V_max` | `LREAL` | Maximum linear velocity (mm/s) |
| `A_max` | `LREAL` | Maximum linear acceleration (mm/s²) |
| `D_max` | `LREAL` | Maximum linear deceleration (mm/s²) |
| `V_Start_Req` | `LREAL` | Entry velocity at point $\mathbf{A}$ ($> 0$ when blending from a previous segment) |
| `Blend_mode` | `BOOL` | `TRUE` = continuous path blending; `FALSE` = stop-and-go |
| `Execute` | `BOOL` | Rising-edge trigger to start interpolation |

### Outputs
| Variable | Type | Description |
| :--- | :--- | :--- |
| `Theta1, Theta2, Theta3` | `LREAL` | Joint angle setpoints streamed to servo drives (degrees) |
| `Out_Error_IK_OWS` | `BOOL` | Inverse kinematics workspace violation flag |
| `Done, Busy` | `BOOL` | General completion and activity flags |
| `Out_Done_Inter` | `BOOL` | Segment completed and servo settled flag |
| `Out_Done_Blend` | `BOOL` | Blend transition pulse (signals main program to load next waypoint) |
| `Out_V_End` | `LREAL` | Exit velocity at point $\mathbf{B}$ (feeds into next segment's `V_Start_Req`) |
| `t_total_estimate` | `LREAL` | Total estimated execution time for the segment (seconds) |

---

## 3. Mathematical Framework

### 3.1 Path Parameterization (Linear Interpolation in 3D)

The function block interpolates along a **straight line** from $\mathbf{A}$ to $\mathbf{B}$ in Cartesian space. At any instant, the cumulative arc-length $S(t)$ determines the position along the line.

**Segment length** (code line 122):
$$L = \|\mathbf{B} - \mathbf{A}\| = \sqrt{(X_B - X_A)^2 + (Y_B - Y_A)^2 + (Z_B - Z_A)^2} \tag{1}$$

**Position at displacement** $S(t)$ (code lines 259–261):
$$\mathbf{P}(t) = \mathbf{A} + \frac{S(t)}{L} \cdot (\mathbf{B} - \mathbf{A}) \tag{2}$$

The ratio $\lambda(t) = S(t)/L$ is a normalized progress parameter in $[0,\, 1]$. At $\lambda = 0$ the end-effector is at $\mathbf{A}$; at $\lambda = 1$ it is at $\mathbf{B}$.

---

### 3.2 Polynomial S-Curve Velocity Profile (Jerk-Bounded)

When the robot starts from rest ($V_{start} = 0$) and must stop at the destination ($V_{end} = 0$), the code uses a **4th-order polynomial** position profile. This is the `Use_Standard_SCurve = TRUE` branch (code lines 234–244).

#### 3.2.1 Acceleration Phase Derivation

Define the normalized time parameter during acceleration:
$$\tau = \frac{t}{t_{acc}}, \quad \tau \in [0,\, 1] \tag{3}$$

The **position** polynomial chosen is (code line 237):
$$S(\tau) = V_{max} \cdot t_{acc} \cdot \left(\tau^3 - \frac{1}{2}\,\tau^4\right) \tag{4}$$

To understand why this polynomial was selected, derive the full kinematic chain by successive differentiation:

**Velocity** ($v = dS/dt = \frac{dS}{d\tau} \cdot \frac{d\tau}{dt}$, where $\frac{d\tau}{dt} = \frac{1}{t_{acc}}$):
$$v(\tau) = V_{max} \cdot \left(3\tau^2 - 2\tau^3\right) \tag{5}$$

Boundary check:
- At $\tau = 0$: $v(0) = 0$ ✓ (starts from rest)
- At $\tau = 1$: $v(1) = V_{max}(3 - 2) = V_{max}$ ✓ (reaches target velocity)

> The polynomial $3\tau^2 - 2\tau^3$ is known in computer graphics and robotics as the **Hermite smoothstep** function — a $C^1$-continuous blending polynomial that guarantees zero-velocity boundaries.

**Acceleration** ($a = dv/dt = \frac{dv}{d\tau} \cdot \frac{1}{t_{acc}}$):
$$a(\tau) = \frac{V_{max}}{t_{acc}} \cdot \left(6\tau - 6\tau^2\right) = \frac{6\,V_{max}}{t_{acc}} \cdot \tau(1 - \tau) \tag{6}$$

Boundary check:
- At $\tau = 0$: $a(0) = 0$ ✓ (acceleration starts smoothly from zero)
- At $\tau = 1$: $a(1) = 0$ ✓ (acceleration returns to zero before cruise phase)
- Peak at $\tau = 0.5$: $a_{peak} = \frac{6\,V_{max}}{t_{acc}} \cdot 0.25 = \frac{1.5\,V_{max}}{t_{acc}}$

**Jerk** ($j = da/dt$):
$$j(\tau) = \frac{V_{max}}{t_{acc}^2} \cdot (6 - 12\tau) \tag{7}$$

The jerk is finite and bounded throughout, but **not zero** at the phase boundaries. This is a key distinction: this is a **3rd-order (cubic velocity)** S-curve, not a 7-segment jerk-limited profile. The acceleration has a smooth bell-shape (parabolic), which is sufficient to suppress mechanical vibrations for the system's inertia class.

#### 3.2.2 The 1.5× Time-Scaling Factor

The code computes (line 136):
```
t_acc := 1.5 * (Actual_Vmax / A_max);
```

**Why 1.5?** For a linear ramp (trapezoidal profile), reaching $V_{max}$ with constant acceleration $A_{max}$ takes $t_{lin} = V_{max}/A_{max}$. But the polynomial S-curve has a parabolic acceleration shape with a peak at $\tau = 0.5$. Setting the peak equal to the user-specified $A_{max}$:

$$a_{peak} = \frac{1.5\,V_{max}}{t_{acc}} \stackrel{!}{=} A_{max} \implies t_{acc} = \frac{1.5\,V_{max}}{A_{max}} \tag{8}$$

The factor $\mathbf{1.5}$ is the **shape compensation ratio**: because the polynomial acceleration curve is bell-shaped (zero at both ends, peak in the middle), it needs 50% more time than a constant-acceleration ramp to reach the same peak value while respecting the $A_{max}$ constraint.

#### 3.2.3 Displacement Budget Verification

Total displacement during acceleration (code line 138):
$$S_{acc} = S(\tau=1) = V_{max} \cdot t_{acc} \cdot (1 - 0.5) = 0.5 \cdot V_{max} \cdot t_{acc} \tag{9}$$

This equals the displacement of a linear ramp at average velocity $V_{max}/2$, which is expected since the smoothstep polynomial has the same integral as a linear ramp over $[0, 1]$.

#### 3.2.4 Deceleration Phase Derivation

Define the deceleration normalized time (code line 242):
$$\tau_d = \frac{t - t_{acc} - t_{run}}{t_{dec}}, \quad \tau_d \in [0,\, 1] \tag{10}$$

The **position** polynomial (code line 243):
$$S(t) = S_{acc} + S_{run} + V_{max} \cdot t_{dec} \cdot \left(\tau_d - \tau_d^3 + \frac{1}{2}\,\tau_d^4\right) \tag{11}$$

**Velocity:**
$$v(\tau_d) = V_{max} \cdot \left(1 - 3\tau_d^2 + 2\tau_d^3\right) \tag{12}$$

Boundary check:
- At $\tau_d = 0$: $v = V_{max}$ ✓ (enters decel at cruise speed)
- At $\tau_d = 1$: $v = V_{max}(1 - 3 + 2) = 0$ ✓ (comes to rest)

**Acceleration:**
$$a(\tau_d) = \frac{V_{max}}{t_{dec}} \cdot (-6\tau_d + 6\tau_d^2) = -\frac{6\,V_{max}}{t_{dec}} \cdot \tau_d(1 - \tau_d) \tag{13}$$

This is the mirror image of Eq. (6) — a smooth bell-shaped deceleration curve. Peak deceleration at $\tau_d = 0.5$:
$$|a_{peak}| = \frac{1.5\,V_{max}}{t_{dec}} = D_{max} \tag{14}$$

confirming that $t_{dec} = 1.5 \cdot V_{max} / D_{max}$ (code line 137).

#### 3.2.5 Short-Segment Velocity Capping

If the segment length $L$ is too short to allow reaching $V_{max}$, the profile degenerates. The code checks (line 132):

$$L < 0.75 \cdot V_{max}^2 \cdot \left(\frac{1}{A_{max}} + \frac{1}{D_{max}}\right) \tag{15}$$

**Derivation:** The minimum distance required for a full accel-decel profile (no cruise phase, $S_{run} = 0$) is:

$$L_{min} = S_{acc} + S_{dec} = 0.5 \cdot V_{max} \cdot t_{acc} + 0.5 \cdot V_{max} \cdot t_{dec}$$

Substituting $t_{acc} = 1.5V_{max}/A_{max}$ and $t_{dec} = 1.5V_{max}/D_{max}$:

$$L_{min} = 0.5 \cdot V_{max} \cdot \frac{1.5\,V_{max}}{A_{max}} + 0.5 \cdot V_{max} \cdot \frac{1.5\,V_{max}}{D_{max}} = 0.75 \cdot V_{max}^2 \cdot \left(\frac{1}{A_{max}} + \frac{1}{D_{max}}\right) \tag{16}$$

When $L < L_{min}$, the code reduces the peak velocity (code line 133):

$$V_{peak} = \sqrt{\frac{L}{0.75 \cdot \left(\frac{1}{A_{max}} + \frac{1}{D_{max}}\right)}} \tag{17}$$

This is obtained by substituting $V_{peak}$ for $V_{max}$ in Eq. (16), setting $L_{min} = L$, and solving for $V_{peak}$.

#### 3.2.6 Summary: Complete S-Curve Profile

| Phase | Time Interval | $S(t)$ Formula | Velocity |
| :--- | :--- | :--- | :--- |
| Acceleration | $0 \le t \le t_{acc}$ | $V_{max} \cdot t_{acc} \cdot (\tau^3 - 0.5\,\tau^4)$ | $0 \to V_{max}$ |
| Cruise | $t_{acc} < t \le t_{acc} + t_{run}$ | $S_{acc} + V_{max} \cdot (t - t_{acc})$ | $V_{max}$ |
| Deceleration | $t_{acc}+t_{run} < t \le T$ | $S_{acc}+S_{run}+V_{max} \cdot t_{dec} \cdot (\tau_d - \tau_d^3 + 0.5\,\tau_d^4)$ | $V_{max} \to 0$ |

**Total time:**
$$T_{total} = t_{acc} + t_{run} + t_{dec}, \quad \text{where}\; t_{run} = \frac{L - S_{acc} - S_{dec}}{V_{max}} \tag{18}$$

---

### 3.3 Trapezoidal Velocity Profile (Non-Zero Boundary Velocities)

When the robot enters a segment with nonzero starting velocity ($V_{start} > 0$) or must exit at nonzero velocity ($V_{end} > 0$), the S-curve polynomial cannot be used because the polynomials in Section 3.2 are hardcoded to boundary values $v=0$. The code falls back to a **linear acceleration / constant-deceleration (trapezoidal)** profile (`Use_Standard_SCurve = FALSE`, code lines 246–253).

#### 3.3.1 Phase Equations

**Acceleration phase** ($0 \le t \le t_{acc}$), code line 247:
$$S(t) = V_{start} \cdot t + \frac{1}{2} A_{max} \cdot t^2 \tag{19}$$
$$v(t) = V_{start} + A_{max} \cdot t$$

**Cruise phase** ($t_{acc} < t \le t_{acc} + t_{run}$), code line 249:
$$S(t) = S_{acc} + V_{peak} \cdot (t - t_{acc}) \tag{20}$$

**Deceleration phase** ($t_{acc}+t_{run} < t \le T_{total}$), code line 252:
$$S(t) = S_{acc} + S_{run} + V_{peak} \cdot t_d - \frac{1}{2} D_{max} \cdot t_d^2 \tag{21}$$
where $t_d = t - t_{acc} - t_{run}$.

#### 3.3.2 Phase Duration Calculations

The phase durations (code lines 149–152, 203–207):

$$t_{acc} = \frac{|V_{peak} - V_{start}|}{A_{max}} \tag{22}$$

$$t_{dec} = \frac{|V_{peak} - V_{end}|}{D_{max}} \tag{23}$$

$$S_{acc} = \frac{V_{start} + V_{peak}}{2} \cdot t_{acc} \tag{24}$$

$$S_{dec} = \frac{V_{end} + V_{peak}}{2} \cdot t_{dec} \tag{25}$$

$$S_{run} = L - S_{acc} - S_{dec}, \quad t_{run} = \frac{S_{run}}{V_{peak}} \tag{26}$$

#### 3.3.3 Profile Feasibility Check (Triangular Fallback)

The total distance consumed by acceleration and deceleration without any cruise phase is (code line 144/192):

$$S_{limit} = \frac{|V_{peak}^2 - V_{start}^2|}{2\,A_{max}} + \frac{|V_{peak}^2 - V_{end}^2|}{2\,D_{max}} \tag{27}$$

> **Derivation of Eq. (27):** From basic kinematics $v^2 = v_0^2 + 2a \cdot s$, the distance to accelerate from $V_{start}$ to $V_{peak}$ is $s_1 = (V_{peak}^2 - V_{start}^2)/(2A_{max})$, and to decelerate from $V_{peak}$ to $V_{end}$ is $s_2 = (V_{peak}^2 - V_{end}^2)/(2D_{max})$.

If $S_{limit} > L$, the segment is too short to reach $V_{max}$. The code computes the maximum achievable velocity by solving $S_{limit} = L$ for $V_{peak}$ (code line 146/196):

From $S_{limit} = L$:
$$\frac{V_{peak}^2 - V_{start}^2}{2\,A_{max}} + \frac{V_{peak}^2 - V_{end}^2}{2\,D_{max}} = L$$

Rearranging:
$$V_{peak}^2 \left(\frac{1}{2A_{max}} + \frac{1}{2D_{max}}\right) = L + \frac{V_{start}^2}{2A_{max}} + \frac{V_{end}^2}{2D_{max}}$$

$$V_{peak}^2 \cdot \frac{A_{max} + D_{max}}{2\,A_{max}\,D_{max}} = L + \frac{V_{start}^2}{2A_{max}} + \frac{V_{end}^2}{2D_{max}}$$

$$\boxed{V_{peak} = \sqrt{\frac{2\,A_{max}\,D_{max}\,L + D_{max}\,V_{start}^2 + A_{max}\,V_{end}^2}{A_{max} + D_{max}}}} \tag{28}$$

This matches the code exactly:
- **Stop-and-go** ($V_{end} = 0$), code line 146: `SQRT((2.0*A_max*D_max*L + D_max*(V_Start_Req**2)) / (A_max + D_max))`
- **Blend mode** ($V_{end} > 0$), code line 196: `SQRT((2.0*A_max*D_max*L + D_max*(V_Start_Req**2) + A_max*(Out_V_End**2)) / (A_max + D_max))`

---

### 3.4 Look-Ahead Corner Velocity Calculation (Blend Mode)

When `Blend_mode = TRUE`, the robot does not stop at point $\mathbf{B}$ but passes through it into the next segment $\overline{BC}$. The exit velocity $V_{end}$ must be limited based on the **turning angle** at $\mathbf{B}$ to prevent excessive centripetal acceleration and mechanical shock. This is the **look-ahead** algorithm (code lines 161–212).

#### 3.4.1 Direction Change Angle

Define the direction vectors of the two consecutive segments (code lines 165–166):

$$\vec{v}_1 = \mathbf{B} - \mathbf{A}, \quad \vec{v}_2 = \mathbf{C} - \mathbf{B}$$

The cosine of the angle $\Theta$ between these vectors (code line 173):

$$\cos(\Theta) = \frac{\vec{v}_1 \cdot \vec{v}_2}{\|\vec{v}_1\| \cdot \|\vec{v}_2\|} = \frac{V_{1x}V_{2x} + V_{1y}V_{2y} + V_{1z}V_{2z}}{L_1 \cdot L_2} \tag{29}$$

The code clamps the result to $[-1, +1]$ (line 175) to prevent `NaN` from floating-point rounding errors in the PLC's `ACOS` domain.

#### 3.4.2 Corner Velocity Limit — The Half-Angle Identity

The maximum safe velocity at the corner is (code line 181):

$$V_{corner} = V_{max} \cdot \sqrt{\frac{\cos(\Theta) + 1}{2}} \tag{30}$$

**Key mathematical insight:** This uses the **trigonometric half-angle identity**:

$$\cos\left(\frac{\Theta}{2}\right) = \sqrt{\frac{\cos(\Theta) + 1}{2}} \tag{31}$$

Therefore:
$$\boxed{V_{corner} = V_{max} \cdot \cos\!\left(\frac{\Theta}{2}\right)} \tag{32}$$

**Physical interpretation:** When the robot traverses a corner, the velocity vector must change direction by angle $\Theta$. The magnitude of the velocity change vector is:

$$\|\Delta \vec{v}\| = 2\,V \cdot \sin\!\left(\frac{\Theta}{2}\right)$$

The corner velocity formula keeps $\|\Delta \vec{v}\|$ proportional to $V_{max} \cdot \sin(\Theta/2) \cdot \cos(\Theta/2)$, which is bounded and smooth. Specifically:

| Corner Angle $\Theta$ | $\cos(\Theta/2)$ | $V_{corner}$ | Physical Meaning |
| :---: | :---: | :---: | :--- |
| $0°$ (straight) | $1.0$ | $V_{max}$ | No direction change → full speed |
| $60°$ (gentle) | $0.866$ | $0.866\,V_{max}$ | Mild slowdown |
| $90°$ (right angle) | $0.707$ | $0.707\,V_{max}$ | Moderate deceleration |
| $120°$ (sharp) | $0.5$ | $0.5\,V_{max}$ | Significant slowdown |
| $180°$ (reversal) | $0$ | $0$ | Full stop required |

#### 3.4.3 Exit Velocity Clamping

The corner velocity is further constrained by two physical limits (code lines 184–188):

1.  **Reachability constraint** — the robot cannot exceed the velocity achievable by accelerating from $V_{start}$ over distance $L$ (code line 184):
    $$V_{reach} = \sqrt{V_{start}^2 + 2\,A_{max}\,L} \tag{33}$$
    
    This is derived from the kinematic equation $v^2 = v_0^2 + 2as$.

2.  **Global velocity cap** — $V_{end} \le V_{max}$ (code line 188).

The final exit velocity (code lines 186–188):
$$V_{end} = \min\!\left(V_{corner},\; V_{reach},\; V_{max}\right) \tag{34}$$

This value is stored in `Out_V_End` and passed to the next segment's `V_Start_Req` input to form a continuous velocity chain across multiple segments.

#### 3.4.4 Safety Clamping on Peak Velocity

After computing the trapezoidal profile for the blend segment, two safety clamps ensure $V_{peak}$ is physically consistent (code lines 200–201):

```
IF Actual_Vmax < V_Start_Req THEN Actual_Vmax := V_Start_Req; END_IF;
IF Actual_Vmax < Out_V_End   THEN Actual_Vmax := Out_V_End;   END_IF;
```

These prevent floating-point rounding in Eq. (28) from producing a $V_{peak}$ that is slightly below the boundary velocities, which would result in negative phase durations and numerical instability.

---

### 3.5 Profile Selection Logic Summary

The profile selection depends on two boolean conditions:

| `V_Start_Req` | `Blend_mode` | Profile Type | $V_{start}$ | $V_{end}$ |
| :---: | :---: | :--- | :---: | :---: |
| $= 0$ | `FALSE` | **S-Curve polynomial** (Sec. 3.2) | $0$ | $0$ |
| $> 0$ | `FALSE` | **Trapezoidal** (Sec. 3.3) | $V_{start}$ | $0$ |
| $= 0$ | `TRUE` | **Trapezoidal + look-ahead** (Sec. 3.4) | $0$ | $V_{corner}$ |
| $> 0$ | `TRUE` | **Trapezoidal + look-ahead** (Sec. 3.4) | $V_{start}$ | $V_{corner}$ |

The S-curve polynomial is only used when **both** boundary velocities are zero (complete stop-and-go). This is because the polynomial in Eq. (4) has hardcoded boundary conditions $v(0) = 0$ and $v(1) = V_{max}$, making it unsuitable for arbitrary start/end velocities.

---

## 4. State Machine Execution Sequence

### Architecture Diagram

```
           ┌──────────────────────────────────────────────────────┐
           │                    Execute = TRUE                    │
           └────────────────────────┬─────────────────────────────┘
                                    ▼
                    ┌───────────────────────────────┐
                    │   State 0: Idle & IK Check    │
                    │  • IK validation of point A   │
                    │  • Check workspace boundary   │
                    └───────────┬───────────────────┘
                                │
               ┌────────────────┼────────────────┐
               │ V_start = 0   │                 │ V_start > 0
               ▼                                 ▼
  ┌─────────────────────┐              ┌──────────────────────┐
  │    State 10: Soft    │              │  Skip smoothing,     │
  │    Startup (20×4ms)  │              │  use IK target as    │
  │  Linear ramp from   │              │  starting position   │
  │  Act.Pos → Target   │              └──────────┬───────────┘
  └──────────┬──────────┘                         │
             └────────────────┬───────────────────┘
                              ▼
              ┌───────────────────────────────┐
              │   State 1: Profile Planning   │
              │  • Compute L, S_limit         │
              │  • Select S-Curve or Trapez.  │
              │  • Compute t_acc, t_dec, etc. │
              │  • Compute T_total            │
              └───────────────┬───────────────┘
                              ▼
              ┌───────────────────────────────┐
              │  State 2: Real-Time Interp.   │◄─── Executes every 4ms
              │  • Compute S(t) from profile  │     t := t + 0.004
              │  • Map S(t) → (X_t, Y_t, Z_t)│
              │  • IK → (θ₁, θ₂, θ₃)        │
              │  • Stream to servos           │
              └───────────┬───────────────────┘
                          │ t ≥ T_total
             ┌────────────┼────────────────┐
             │ Stop-and-go                 │ Blend mode
             ▼                             ▼
  ┌─────────────────────┐     ┌─────────────────────────┐
  │  State 3: Settle    │     │  Out_Done_Blend = TRUE  │
  │  • Check |Δθ| < 5e-4│     │  Execute := FALSE       │
  │  • Wait MC.Busy=F   │     │  Return to State 0      │
  │  • Out_Done_Inter   │     │  (await next waypoint)   │
  └──────────┬──────────┘     └─────────────────────────┘
             ▼
     Execute := FALSE
     Return to State 0

  ┌─────────────────────┐
  │  State 99: Fault    │ ◄── Any IK workspace error
  │  • Halt all motion  │
  │  • Report error     │
  └─────────────────────┘
```

### State 0: Idle & Pre-flight IK Validation (code lines 63–107)

- Waits for `Execute = TRUE`.
- Calls `calc_inverse_kinematics` on point $\mathbf{A}$ to verify it lies within the reachable workspace. If the IK solver returns an error (point outside workspace dome), the block immediately transitions to **State 99** and sets the error flag.
- **Branching logic based on entry velocity:**
  - **$V_{start} = 0$:** The robot is stationary. The code reads the current physical servo positions (`MC_Axis1.Act.Pos`, etc.) and transitions to **State 10** for soft startup. This is necessary because the theoretical IK-computed angles for point $\mathbf{A}$ may differ from the actual servo positions by a small offset due to servo lag, gravity sag, or previous motion residuals.
  - **$V_{start} > 0$:** The robot is already moving (blend from previous segment). The code directly adopts the IK-computed target angles as the current setpoint and jumps to **State 1**. Using the actual servo position here would introduce a discontinuity because the servos are still tracking the previous trajectory.

### State 10: Soft Startup Smoothing (code lines 109–119)

Executes for exactly **20 cycles** (80 ms at 4 ms cycle time). Linearly interpolates the angle setpoints from the actual servo position to the IK target:

$$\theta_i(k) = \theta_{i,actual} + \frac{k}{20} \cdot (\theta_{i,target} - \theta_{i,actual}), \quad k = 1, 2, \ldots, 20 \tag{35}$$

**Purpose:** If the IK-computed starting angles differ from the physical positions by even a fraction of a degree, commanding the full target instantly would create a step input — causing the servo current loop to spike (potentially triggering an overcurrent alarm) and generating a mechanical jolt in the Delta arms. The 80 ms ramp smoothly absorbs this error.

### State 1: Profile Planning (code lines 121–229)

Computes $L$, selects the velocity profile type, calculates all phase durations and displacements, and outputs the total time estimate:

$$t_{total\_estimate} = \begin{cases} T_{total} + 0.08\,\text{s} & \text{if } V_{start} = 0 \text{ (includes State 10 duration)} \\ T_{total} & \text{if } V_{start} > 0 \end{cases} \tag{36}$$

The 0.08 s correction (code line 219) accounts for the 20-cycle soft startup in State 10.

### State 2: Real-Time Interpolation Loop (code lines 231–293)

This is the core execution loop. On every PLC scan (4 ms):

1.  Computes $S(t)$ using the selected profile (S-Curve or Trapezoidal).
2.  Maps $S(t)$ to $(X_t, Y_t, Z_t)$ via Eq. (2).
3.  Calls `calc_inverse_kinematics` to convert to joint angles.
4.  If IK returns error → **State 99** (workspace violation during motion).
5.  Updates `Theta1`, `Theta2`, `Theta3` for the servo command section.
6.  Increments $t := t + 0.004$.

**Segment termination** when $t > T_{total}$:
- **Blend mode with $V_{end} > 0$:** Sets `Out_Done_Blend := TRUE` as a pulse signal, resets `Execute := FALSE`, and returns to State 0. The main program detects this pulse and loads the next waypoint.
- **Stop-and-go:** Transitions to State 3 for settling verification.

### State 3: Servo Settling Verification (code lines 295–315)

After the trajectory is complete, the servos may still be tracking residual position commands. This state:

1.  Disables the motion command (`Enable_Motion := FALSE`).
2.  Checks if each axis has settled within a **0.0005°** tolerance:
    $$|\theta_{i,cmd} - \theta_{i,actual}| \le 0.0005°, \quad \forall\, i \in \{1, 2, 3\} \tag{37}$$
3.  Waits until all `MC_SyncMoveAbsolute` instances report `Busy = FALSE`.
4.  Sets `Out_Done_Inter := TRUE` and returns to State 0.

### State 99: Kinematic Fault (code lines 317–323)

Emergency stop state. Disables all motion and holds until the error flag is externally cleared. This protects the mechanical structure from commands that would drive the arms into impossible configurations.

---

## 5. Servo Command Execution (code lines 325–331)

The servo synchronization calls execute **unconditionally on every PLC scan**, outside the state machine:

```
MC_Sync_Axis1(Axis := MC_Axis1, Execute := Enable_Motion, Position := Theta1);
MC_Sync_Axis2(Axis := MC_Axis2, Execute := Enable_Motion, Position := Theta2);
MC_Sync_Axis3(Axis := MC_Axis3, Execute := Enable_Motion, Position := Theta3);
```

This architecture is critical:
- The `MC_SyncMoveAbsolute` instruction operates in **CSP (Cyclic Synchronous Position)** mode — it does not plan its own trajectory but simply passes the commanded position value to the EtherCAT PDO output (`607Ah`) at each cycle.
- When `Enable_Motion = FALSE`, the servo drive holds its last position via its internal position loop.
- When `Enable_Motion = TRUE`, the servo continuously tracks the `Theta` variables, which are updated by State 2 every 4 ms.

This separation between trajectory computation (state machine) and servo command (always-running) ensures that the servo drives receive a position command on **every single EtherCAT cycle** without gaps, preventing position following errors or drive faults.

---

## 6. Velocity Profile Visualization

### 6.1 S-Curve Profile (Stop-and-Go)
```
 Velocity
    ▲
    │            ┌───────────┐
    │           ╱             ╲
 Vmax ─ ─ ─ ─╱───────────────╲─ ─ ─
    │        ╱  │    Cruise    │╲
    │       ╱   │              │ ╲
    │      ╱    │              │  ╲
    │    ╱      │              │    ╲
    │  ╱        │              │      ╲
    │╱          │              │        ╲
    └───────────┼──────────────┼──────────► t
    0     t_acc   t_acc+t_run      T_total

    Acceleration (bell-shaped):
    ▲
    │      ╱╲
 Amax ─ ╱────╲─ ─ ─
    │  ╱      ╲                  ╱╲
    │╱          ╲  ─ ─ ─ ─ ─  ╱    ╲
    ├─────────────────────────────────► t
    │              0          ╱      ╲
    │                        ╱────────╲
-Dmax                      ╱           ╲
```

### 6.2 Trapezoidal Profile (Blend Mode, Non-Zero Boundaries)
```
 Velocity
    ▲
    │         ┌───────────┐
    │        /             \
 Vpeak ─ ─ / ─ ─ ─ ─ ─ ─ ─\ ─ ─
    │      /  │    Cruise   │ \
    │     /   │             │  \
 Vend─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─\─ ─
    │   /     │             │    \
 Vstart       │             │
    │ /       │             │
    └─────────┼─────────────┼─────► t
    0    t_acc  t_acc+t_run    T_total
```

---

## 7. Algorithm Correctness Summary

| Property | S-Curve | Trapezoidal | Verification |
| :--- | :---: | :---: | :--- |
| Position continuity ($C^0$) | ✓ | ✓ | $S(t)$ matches at phase boundaries |
| Velocity continuity ($C^1$) | ✓ | ✓ | $v=V_{max}$ at accel/cruise join; $v=0$ at endpoints |
| Acceleration continuity ($C^2$) | ✓ | ✗ | S-curve: $a=0$ at boundaries; Trapez: step change |
| Bounded jerk | ✓ (finite) | ✗ (infinite at corners) | S-curve jerk $=6V_{max}/t_{acc}^2$ at max |
| Peak accel ≤ $A_{max}$ | ✓ | ✓ | 1.5× factor ensures $a_{peak} = A_{max}$ |
| Arbitrary $V_{start}$/$V_{end}$ | ✗ | ✓ | S-curve only supports $0 \to V_{max} \to 0$ |
| Short segment handling | ✓ | ✓ | Triangular fallback via Eq. (17)/(28) |
