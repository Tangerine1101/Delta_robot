# Delta Robot Inverse Kinematics: Algorithm & Mathematical Derivation

Source: [Calc_Angles_YZ.txt](file:///d:/1_Uni_ute/Graduation_Project/Delta_All/1.sysmac_code/Functions/Calc_Angles_YZ.txt) and [Calc_Inverse_Kinematics.txt](file:///d:/1_Uni_ute/Graduation_Project/Delta_All/1.sysmac_code/Functions/Calc_Inverse_Kinematics.txt)

---

## 1. Problem Statement

**Given:** Desired end-effector Cartesian position $(X, Y, Z)$.
**Find:** Three active joint angles $(\theta_1, \theta_2, \theta_3)$ to command the servo motors.

The algorithm exploits the **120° rotational symmetry** of the Delta robot by decomposing the 3D problem into three identical 2D problems, each solved in a single arm's vertical plane.

### Two-Tier Architecture

```
       [TCP Cartesian Coordinate (X, Y, Z)]
                       │
        ┌──────────────┼──────────────┐
        ▼              ▼              ▼
     [Arm 1]        [Arm 2]        [Arm 3]
  (No rotation)   (Rotate −120°)  (Rotate +120°)
        │              │              │
        ▼              ▼              ▼
  Calc_Angles_YZ  Calc_Angles_YZ  Calc_Angles_YZ
        │              │              │
        ▼              ▼              ▼
      θ₁             θ₂            θ₃ = −θ_tmp3

```
---

## 2. Geometric Parameters

| Symbol | Code Variable | Value | Description |
| :---: | :--- | :---: | :--- |
| $s_b$ | `Base` | 320.0 mm | Base equilateral triangle side length |
| $s_p$ | `EndEffector` | 94.0 mm | Platform equilateral triangle side length |
| $L$ | `Bicep` | 140.0 mm | Active arm (bicep) length |
| $l$ | `Forearm` | 315.0 mm | Passive arm (forearm) length |

---

## 3. Single-Arm 2D Solver: `Calc_Angles_YZ`

This function solves the IK for **one arm** operating in the vertical $YZ$-plane. The key idea is to find the elbow position where the bicep and forearm constraints intersect, then compute the motor angle from that elbow position.

### 3.1 Reference Points

**Base joint** — the motor axis for Arm 1 lies on the base triangle's inscribed circle, along the $-Y$ axis (Calc_Angles_YZ line 46):

$$y_1 = -\frac{1}{2}\tan(30°) \cdot s_b = -\frac{s_b}{2\sqrt{3}} \tag{1}$$

**Platform hinge** — the corresponding connection point on the moving platform, offset by the platform's inscribed radius (line 47):

$$y_{tmp} = Y_0 - \frac{1}{2}\tan(30°) \cdot s_p = Y_0 - \frac{s_p}{2\sqrt{3}} \tag{2}$$

The variable $y_{tmp}$ represents the $Y$-coordinate of the platform hinge point, shifted from the TCP position $Y_0$ by the platform's geometric offset.

### 3.2 Constraint Equations

The elbow joint $(0, y_j, z_j)$ must satisfy two simultaneous geometric constraints:

**Constraint 1 — Bicep circle:** The elbow lies on a circle of radius $L$ centered at the base joint $(0, y_1, 0)$ (line 50):

$$(y_j - y_1)^2 + z_j^2 = L^2 \tag{3}$$

**Constraint 2 — Forearm sphere:** The distance from the elbow to the platform hinge $(X_0, y_{tmp}, Z_0)$ equals the forearm length $l$ (line 52):

$$X_0^2 + (y_{tmp} - y_j)^2 + (Z_0 - z_j)^2 = l^2 \tag{4}$$

> **Why $X_0$ appears in Eq. (4):** Even though the arm operates in the $YZ$-plane, the forearm is a 3D link. If the target has a nonzero $X$-component (in the arm's local frame), the forearm stretches at an angle, consuming part of its length in the $X$-direction and leaving less reach in the $YZ$-plane.

### 3.3 Linearization: Deriving $z_j = a + b \cdot y_j$

Expand Eq. (3):

$$y_j^2 - 2y_1 y_j + y_1^2 + z_j^2 = L^2 \tag{3'}$$

Expand Eq. (4):

$$X_0^2 + y_{tmp}^2 - 2y_{tmp}y_j + y_j^2 + Z_0^2 - 2Z_0 z_j + z_j^2 = l^2 \tag{4'}$$

**Subtract (3') from (4')** — the quadratic terms $y_j^2$ and $z_j^2$ cancel:

$$X_0^2 + y_{tmp}^2 - 2y_{tmp}y_j + Z_0^2 - 2Z_0 z_j + 2y_1 y_j - y_1^2 = l^2 - L^2$$

Isolate $z_j$:

$$-2Z_0 z_j = (l^2 - L^2 - X_0^2 - y_{tmp}^2 - Z_0^2 + y_1^2) + 2(y_{tmp} - y_1)y_j$$

Multiply both sides by $-1$ and divide by $2Z_0$:

$$z_j = \underbrace{\frac{X_0^2 + y_{tmp}^2 + Z_0^2 + L^2 - l^2 - y_1^2}{2Z_0}}_{a} + \underbrace{\frac{y_1 - y_{tmp}}{Z_0}}_{b} \cdot y_j \tag{5}$$

This matches code lines 55–56 exactly:
```
a := (X0*X0 + tmp_y0*tmp_y0 + Z0*Z0 + Bicep*Bicep - Forearm*Forearm - y1*y1) / (2.0 * Z0);
b := (y1 - tmp_y0) / Z0;
```

> **Divide-by-zero guard** (line 40): If $Z_0 = 0$, the end-effector is at the same height as the base — the coefficients $a$ and $b$ are undefined. The function returns `TRUE` (error) immediately.

### 3.4 Quadratic Equation for $y_j$

Substitute $z_j = a + b \cdot y_j$ (Eq. 5) back into the bicep constraint (Eq. 3):

$$(y_j - y_1)^2 + (a + b \cdot y_j)^2 = L^2$$

Expand term by term:

$$\underbrace{y_j^2 - 2y_1 y_j + y_1^2}_{(y_j - y_1)^2} + \underbrace{a^2 + 2ab\,y_j + b^2 y_j^2}_{(a + b\,y_j)^2} = L^2$$

Collect by powers of $y_j$:

$$\underbrace{(1 + b^2)}_{A}\,y_j^2 + \underbrace{2(ab - y_1)}_{B}\,y_j + \underbrace{(y_1^2 + a^2 - L^2)}_{C} = 0 \tag{6}$$

### 3.5 Discriminant (Optimized Form)

The standard quadratic discriminant is $\Delta = B^2 - 4AC$. Expanding:

$$\Delta = 4(ab - y_1)^2 - 4(1 + b^2)(y_1^2 + a^2 - L^2)$$

Factor out 4 and expand the products:

$$\frac{\Delta}{4} = a^2 b^2 - 2ab y_1 + y_1^2 - y_1^2 - a^2 + L^2 - b^2 y_1^2 - a^2 b^2 + L^2 b^2$$

Simplify — the $a^2 b^2$ and $y_1^2$ terms cancel:

$$\frac{\Delta}{4} = -2ab y_1 - a^2 - b^2 y_1^2 + L^2 + L^2 b^2$$

Recognize the perfect square $-(a + by_1)^2 = -(a^2 + 2aby_1 + b^2y_1^2)$:

$$\boxed{d = \frac{\Delta}{4} = -(a + b\,y_1)^2 + L^2(b^2 + 1)} \tag{7}$$

This matches code line 59:
```
d := -(a + b * y1) * (a + b * y1) + Bicep * (b * b * Bicep + Bicep);
```

> The code computes $d = \Delta/4$ rather than $\Delta$ itself. This is a deliberate optimization: using $d$ instead of $\Delta$ in the quadratic formula eliminates a factor of 2, simplifying the root computation (see next section).

### 3.6 Solving for the Elbow Position

Using the half-discriminant form, the quadratic formula becomes:

$$y_j = \frac{-B \pm \sqrt{\Delta}}{2A} = \frac{-2(ab - y_1) \pm 2\sqrt{d}}{2(1 + b^2)} = \frac{y_1 - ab \pm \sqrt{d}}{b^2 + 1} \tag{8}$$

**Root selection:** The two solutions correspond to two possible elbow positions — one above the base plane ("elbow-up") and one below ("elbow-down"). The Delta robot's physical assembly requires the **elbow-down** configuration, which corresponds to the **negative branch** $(-\sqrt{d})$:

$$y_j = \frac{y_1 - ab - \sqrt{d}}{b^2 + 1} \tag{9}$$

This gives the elbow that is further from the center and lower — the only mechanically feasible configuration.

Code line 67: `yj := (y1 - a * b - SQRT(d)) / (b * b + 1.0);`

**Back-substitution for $z_j$** (code line 68):
$$z_j = a + b \cdot y_j \tag{10}$$

### 3.7 Motor Angle Calculation

With the elbow position $(y_j, z_j)$ known, the motor angle $\theta$ is the angle of the bicep arm measured from the horizontal.

The arm vector from the base joint $(y_1, 0)$ to the elbow $(y_j, z_j)$ has:
- **Horizontal (radial outward) component:** $y_1 - y_j$ (positive when elbow extends outward from center)
- **Vertical (downward) component:** $-z_j$ (positive when elbow is below the base plane)

$$\theta = \arctan\left(\frac{-z_j}{y_1 - y_j}\right) \tag{11}$$

**Conversion to degrees** (code line 85):

$$\theta_{out} = \frac{180}{\pi} \cdot \arctan\!\left(\frac{-z_j}{y_1 - y_j}\right) + \theta_{offset} \tag{12}$$

**Quadrant correction** ($\theta_{offset}$): The `ATAN` function returns values only in $(-90°, +90°)$. When $y_j > y_1$ (the elbow has swung past horizontal to the opposite side), the arm vector's horizontal component reverses sign. A $180°$ offset corrects the quadrant (code lines 71–73):

$$\theta_{offset} = \begin{cases} 180° & \text{if } y_j > y_1 \\ 0° & \text{otherwise} \end{cases}$$

**Division-by-zero guard** (code lines 77–82): When $y_j = y_1$, the arm is perfectly vertical. The angle is directly assigned:

$$\theta_{out} = \begin{cases} +90° + \theta_{offset} & \text{if } -z_j \geq 0 \text{ (elbow below base)} \\ -90° + \theta_{offset} & \text{if } -z_j < 0 \text{ (elbow above base)} \end{cases}$$

---

## 4. Top-Level Coordinator: `Calc_Inverse_Kinematics`

The three arms of the Delta robot are identical but physically mounted at $0°$, $+120°$, and $-120°$ around the $Z$-axis. Rather than deriving separate equations for each arm, the algorithm **rotates the target coordinate** into each arm's local frame and reuses the single 2D solver.

### 4.1 2D Rotation Matrix

A counterclockwise rotation by angle $\phi$ around the $Z$-axis transforms $(X, Y)$ as:

$$\begin{bmatrix} X' \\ Y' \end{bmatrix} = \mathbf{R}(\phi) \begin{bmatrix} X \\ Y \end{bmatrix} = \begin{bmatrix} \cos\phi & -\sin\phi \\ \sin\phi & \cos\phi \end{bmatrix} \begin{bmatrix} X \\ Y \end{bmatrix} \tag{13}$$

The $Z$-coordinate is unaffected by rotations around the $Z$-axis.

### 4.2 Arm 1 — No Rotation (code lines 42–57)

Arm 1 is the reference arm, already aligned with the $-Y$ direction. The target coordinates are passed directly:

$$\theta_1 = \text{Calc\_Angles\_YZ}(X,\; Y,\; Z) \tag{14}$$

### 4.3 Arm 2 — Rotate by $-120°$ (code lines 63–78)

Arm 2 is mounted at $+120°$ from Arm 1. To transform the target into Arm 2's local frame, apply $\mathbf{R}(-120°)$:

$$\begin{bmatrix} X_2 \\ Y_2 \end{bmatrix} = \begin{bmatrix} \cos(-120°) & -\sin(-120°) \\ \sin(-120°) & \cos(-120°) \end{bmatrix} \begin{bmatrix} X \\ Y \end{bmatrix} = \begin{bmatrix} \cos 120° & \sin 120° \\ -\sin 120° & \cos 120° \end{bmatrix} \begin{bmatrix} X \\ Y \end{bmatrix}$$

Using $\cos 120° = -0.5$ and $\sin 120° = \frac{\sqrt{3}}{2} \approx 0.866$:

$$X_2 = X \cos 120° + Y \sin 120° \tag{15}$$
$$Y_2 = Y \cos 120° - X \sin 120° \tag{16}$$

Code lines 63–64:
```
rotate_X2 := X * Cos120 + Y * Sin120;
rotate_Y2 := Y * Cos120 - X * Sin120;
```

$$\theta_2 = \text{Calc\_Angles\_YZ}(X_2,\; Y_2,\; Z) \tag{17}$$

### 4.4 Arm 3 — Rotate by $+120°$ (code lines 84–99)

Arm 3 is mounted at $-120°$ from Arm 1. Apply $\mathbf{R}(+120°)$:

$$\begin{bmatrix} X_3 \\ Y_3 \end{bmatrix} = \begin{bmatrix} \cos 120° & -\sin 120° \\ \sin 120° & \cos 120° \end{bmatrix} \begin{bmatrix} X \\ Y \end{bmatrix}$$

$$X_3 = X \cos 120° - Y \sin 120° \tag{18}$$
$$Y_3 = X \sin 120° + Y \cos 120° \tag{19}$$

Code lines 84–85:
```
rotate_X3 := X * Cos120 - Y * Sin120;
rotate_Y3 := Y * Cos120 + X * Sin120;
```

The 2D solver computes a temporary angle:
$$\theta_{tmp3} = \text{Calc\_Angles\_YZ}(X_3,\; Y_3,\; Z)$$

**Direction inversion** (code line 99): Axis 3's servo drive has an inverted direction parameter compared to Axes 1 and 2 (the positive encoder direction corresponds to downward arm movement rather than upward). The output angle is negated to compensate:

$$\theta_3 = -\theta_{tmp3} \tag{20}$$

### 4.5 Rotation Verification Table

| Arm | Physical Azimuth | Rotation Applied | Result |
| :---: | :---: | :---: | :--- |
| 1 | $0°$ | None | Target passes through directly |
| 2 | $+120°$ | $\mathbf{R}(-120°)$ | Target rotated $-120°$ to align with Arm 2's plane |
| 3 | $-120°$ | $\mathbf{R}(+120°)$ | Target rotated $+120°$ to align with Arm 3's plane |

---

## 5. Error Handling & Safety

The function implements three layers of protection:

| Check | Location | Condition | Meaning |
| :--- | :--- | :--- | :--- |
| Z-plane divide-by-zero | `Calc_Angles_YZ` line 40 | $Z_0 = 0$ | Platform at base height — coefficients $a, b$ undefined |
| Workspace boundary | `Calc_Angles_YZ` line 62 | $d < 0$ | Target unreachable — forearm cannot connect the elbow to the platform |
| Joint angle limit | `Calc_IK` lines 51, 73, 94 | $\theta < -20°$ | Arm would swing above horizontal — risk of mechanical collision with the base frame |
| ATAN divide-by-zero | `Calc_Angles_YZ` line 77 | $y_1 = y_j$ | Arm is vertical — direct angle assignment bypasses division |

When any check fails, the function returns `TRUE` (error) and aborts immediately. This propagates up to the motion interpolator ([MC_Inter_Curve_Vel](file:///d:/1_Uni_ute/Graduation_Project/Delta_All/1.sysmac_code/Function_Blocks/MC_inter_curve_vel.md)), which transitions to **State 99** (emergency halt).

> **Note on Axis 3 limit check** (code line 94): Because $\theta_3$ is inverted (Eq. 20), the raw check is `tmp_Theta3 * -1 > 20` — equivalent to $\theta_3 > 20°$ after inversion, ensuring the same $-20°$ physical limit applies to all three arms.

---

## 6. Algorithm Summary

```
  Input: (X, Y, Z)
      │
      ├─── Arm 1: (X, Y, Z)  ──── Calc_Angles_YZ ──→ θ₁
      │
      ├─── Arm 2: R(−120°)·(X,Y) ─ Calc_Angles_YZ ──→ θ₂
      │
      └─── Arm 3: R(+120°)·(X,Y) ─ Calc_Angles_YZ ──→ −θ_tmp₃ = θ₃


  Calc_Angles_YZ internals:
  ┌──────────────────────────────────────────────────┐
  │ 1. Compute reference points y₁, y_tmp            │
  │ 2. Linearize: z_j = a + b·y_j    (Eq. 5)        │
  │ 3. Substitute into bicep constraint               │
  │    → quadratic in y_j             (Eq. 6)        │
  │ 4. Discriminant d = Δ/4           (Eq. 7)        │
  │ 5. d < 0 → error (unreachable)                   │
  │ 6. y_j = (y₁ − ab − √d)/(b²+1)  (Eq. 9)        │
  │ 7. z_j = a + b·y_j               (Eq. 10)       │
  │ 8. θ = atan(−z_j / (y₁ − y_j))   (Eq. 11)       │
  └──────────────────────────────────────────────────┘
```

### Computational Cost

| Operation | Count |
| :--- | :---: |
| Trigonometric (`SIN`, `COS`, `ATAN`) | 1 per arm = 3 total |
| Multiplications | ~15 per arm = ~45 total |
| Divisions | 3 per arm = 9 total |
| Square roots | 1 per arm = 3 total |

All operations are $O(1)$ — no iteration, no convergence. Total execution time: **< 10 µs** on the NX1P2 PLC, fully compatible with the 4 ms EtherCAT cycle.
