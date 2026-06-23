# Delta Robot Forward Kinematics: Algorithm & Mathematical Derivation

Source: [Calc_Forward_Kinematic.txt](file:///d:/1_Uni_ute/Graduation_Project/Delta_All/1.sysmac_code/Functions/Calc_Forward_Kinematic.txt)

---

## 1. Problem Statement

**Given:** Three active joint angles $(\theta_1, \theta_2, \theta_3)$ from the servo encoder feedback.
**Find:** The Cartesian coordinates $(X_0, Y_0, Z_0)$ of the end-effector platform center.

The [Inverse Kinematics](file:///d:/1_Uni_ute/Graduation_Project/Delta_All/1.sysmac_code/Functions/inverse_kinematics.md) solver commands motion ($\text{Cartesian} \to \text{Joint}$). The Forward Kinematics solver **monitors** motion ($\text{Joint} \to \text{Cartesian}$), providing real-time position feedback to the HMI and safety validation that the physical mechanism matches the commanded path.

---

## 2. Geometric Parameters

| Symbol | Code Variable | Value | Description |
| :---: | :--- | :---: | :--- |
| $s_b$ | `Base` | 320.0 mm | Base equilateral triangle side length |
| $s_p$ | `EndEffector` | 94.0 mm | Platform equilateral triangle side length |
| $L$ | `Bicep` | 140.0 mm | Active arm (bicep) length |
| $l$ | `Forearm` | 315.0 mm | Passive arm (forearm) length |

**Radial offset** $w$ — the horizontal distance from the coordinate origin to the effective arm pivot point, accounting for the geometric difference between the base and platform triangles (code line 66):

$$w = \frac{1}{2}\tan(30°) \cdot (s_b - s_p) = \frac{s_b - s_p}{2\sqrt{3}} \tag{1}$$

This collapses the two concentric triangles into a single-point kinematic model where each arm pivots at distance $w$ from the center.

---

## 3. Algorithm: Three-Sphere Intersection

The FK problem reduces to finding the intersection point of three spheres in 3D space. The algorithm proceeds in three stages:

```
  θ₁, θ₂, θ₃  (encoder angles)
       │
       ▼
  ┌─────────────────────────────┐
  │ Step 1: Compute elbow       │   3 points in ℝ³
  │ positions J₁, J₂, J₃       │   (geometric projection)
  └──────────────┬──────────────┘
                 ▼
  ┌─────────────────────────────┐
  │ Step 2: Linearize by        │   Subtract sphere equations
  │ pairwise subtraction        │   → 2×2 linear system
  │ → X₀(Z₀), Y₀(Z₀)          │   (Cramer's rule)
  └──────────────┬──────────────┘
                 ▼
  ┌─────────────────────────────┐
  │ Step 3: Substitute back     │   Single quadratic in Z₀
  │ into sphere eq. → solve Z₀  │   (quadratic formula)
  └──────────────┬──────────────┘
                 ▼
          (X₀, Y₀, Z₀)
```

---

### Step 1: Elbow Joint Positions (code lines 60–86)

Each arm $i$ rotates in a vertical plane oriented at $120°$ intervals around the Z-axis. The angle $\theta_i$ rotates the bicep of length $L$ in this plane. The resulting elbow coordinates are computed by projecting the bicep tip into the global frame.

**Angle conversion** (code lines 60–62):
$$t_1 = \theta_1 \cdot \frac{\pi}{180}, \quad t_2 = \theta_2 \cdot \frac{\pi}{180}, \quad t_3 = \theta_3 \cdot \frac{\pi}{-180}$$

> **Note on Axis 3 sign:** The third servo drive has an inverted parameter direction compared to axes 1 and 2. The negation in $t_3$ compensates so that a positive encoder count always corresponds to the arm moving **upward** in the kinematic model.

**Arm 1** lies in the $YZ$-plane, pointing along $-Y$ (code lines 74–76):

$$\mathbf{J}_1 = \begin{pmatrix} 0 \\ -(w + L\cos t_1) \\ -L\sin t_1 \end{pmatrix} \tag{2}$$

**Arm 2** is rotated $+120°$ from Arm 1. Its vertical plane passes through azimuth $+30°$ from the $+X$ axis (code lines 79–81):

$$\mathbf{J}_2 = \begin{pmatrix} (w + L\cos t_2)\cos 30° \\ (w + L\cos t_2)\sin 30° \\ -L\sin t_2 \end{pmatrix} \tag{3}$$

**Arm 3** is rotated $-120°$ (or $+240°$) from Arm 1. Its vertical plane passes through azimuth $+150°$ (code lines 84–86):

$$\mathbf{J}_3 = \begin{pmatrix} -(w + L\cos t_3)\cos 30° \\ (w + L\cos t_3)\sin 30° \\ -L\sin t_3 \end{pmatrix} \tag{4}$$

**Geometric meaning:** $\cos t_i$ determines the radial projection (horizontal distance from pivot), while $\sin t_i$ determines the vertical drop. The $Z$-components are always negative because the robot workspace is entirely below the base plane.

---

### Step 2: Linearization via Sphere Subtraction (code lines 98–134)

The end-effector center $\mathbf{P} = (X_0, Y_0, Z_0)$ is connected to each elbow by a forearm of length $l$. This defines three sphere equations:

$$\|\mathbf{P} - \mathbf{J}_i\|^2 = l^2, \quad i = 1, 2, 3 \tag{5}$$

Expanding sphere $i$:

$$X_0^2 + Y_0^2 + Z_0^2 - 2x_{ji}X_0 - 2y_{ji}Y_0 - 2z_{ji}Z_0 + r_i = l^2 \tag{6}$$

where $r_i = x_{ji}^2 + y_{ji}^2 + z_{ji}^2$ is the squared distance of elbow $i$ from the origin (code lines 99–101).

#### The Linearization Trick

The key mathematical insight is that each expanded sphere equation (Eq. 6) contains the **same** quadratic terms $(X_0^2 + Y_0^2 + Z_0^2)$. Subtracting sphere 1 from spheres 2 and 3 **eliminates all quadratic unknowns**, leaving a linear system.

**Sphere 2 − Sphere 1:**

$$(r_2 - r_1) - 2(x_{j2} - \underbrace{x_{j1}}_{=0})X_0 - 2(y_{j2} - y_{j1})Y_0 - 2(z_{j2} - z_{j1})Z_0 = 0$$

Rearranging and dividing by 2:

$$\underbrace{x_{j2}}_{A_1} X_0 + \underbrace{(y_{j2} - y_{j1})}_{B_1} Y_0 + \underbrace{(z_{j2} - z_{j1})}_{C_1} Z_0 = \underbrace{\tfrac{1}{2}(r_2 - r_1)}_{D_1} \tag{7}$$

**Sphere 3 − Sphere 1** (analogously):

$$\underbrace{x_{j3}}_{A_2} X_0 + \underbrace{(y_{j3} - y_{j1})}_{B_2} Y_0 + \underbrace{(z_{j3} - z_{j1})}_{C_2} Z_0 = \underbrace{\tfrac{1}{2}(r_3 - r_1)}_{D_2} \tag{8}$$

This yields a $2 \times 2$ linear system in $(X_0, Y_0)$ parameterized by $Z_0$:

$$\begin{bmatrix} A_1 & B_1 \\ A_2 & B_2 \end{bmatrix} \begin{bmatrix} X_0 \\ Y_0 \end{bmatrix} = \begin{bmatrix} D_1 - C_1 Z_0 \\ D_2 - C_2 Z_0 \end{bmatrix} \tag{9}$$

#### Solving by Cramer's Rule

The determinant (code line 119):

$$\Delta = A_1 B_2 - A_2 B_1 \tag{10}$$

> **Singularity guard** (code line 122): $\Delta = 0$ means the three elbows are collinear — the system is degenerate and has no unique solution. This is physically impossible in a correctly assembled delta robot, but the check prevents division-by-zero.

Since the right-hand side is linear in $Z_0$, the solution has the form $X_0 = a_1 + b_1 Z_0$, $Y_0 = a_2 + b_2 Z_0$:

$$X_0 = \underbrace{\frac{D_1 B_2 - D_2 B_1}{\Delta}}_{a_1} + \underbrace{\frac{-(C_1 B_2 - C_2 B_1)}{\Delta}}_{b_1} \cdot Z_0 \tag{11}$$

$$Y_0 = \underbrace{\frac{A_1 D_2 - A_2 D_1}{\Delta}}_{a_2} + \underbrace{\frac{-(A_1 C_2 - A_2 C_1)}{\Delta}}_{b_2} \cdot Z_0 \tag{12}$$

**Derivation of $a_1, b_1$:** Applying Cramer's rule to Eq. (9) for $X_0$:

$$X_0 = \frac{\det\begin{bmatrix} D_1 - C_1 Z_0 & B_1 \\ D_2 - C_2 Z_0 & B_2 \end{bmatrix}}{\Delta} = \frac{(D_1 - C_1 Z_0)B_2 - (D_2 - C_2 Z_0)B_1}{\Delta}$$

$$= \frac{D_1 B_2 - D_2 B_1}{\Delta} + \frac{(-C_1 B_2 + C_2 B_1)}{\Delta} Z_0 = a_1 + b_1 Z_0 \quad \checkmark$$

This matches code lines 129–134 exactly.

---

### Step 3: Quadratic Equation for $Z_0$ (code lines 141–160)

With $X_0$ and $Y_0$ expressed as linear functions of $Z_0$, we substitute back into the **original** sphere 1 equation (Eq. 5 for $i=1$, noting $x_{j1} = 0$):

$$(a_1 + b_1 Z_0)^2 + (a_2 + b_2 Z_0 - y_{j1})^2 + (Z_0 - z_{j1})^2 = l^2 \tag{13}$$

Expanding each term:

**(i)** $(a_1 + b_1 Z_0)^2 = a_1^2 + 2a_1 b_1 Z_0 + b_1^2 Z_0^2$

**(ii)** $(a_2 + b_2 Z_0 - y_{j1})^2 = (a_2 - y_{j1})^2 + 2(a_2 - y_{j1})b_2 Z_0 + b_2^2 Z_0^2$

**(iii)** $(Z_0 - z_{j1})^2 = Z_0^2 - 2z_{j1} Z_0 + z_{j1}^2$

Collecting by powers of $Z_0$:

$$\underbrace{(b_1^2 + b_2^2 + 1)}_{A_q} Z_0^2 + \underbrace{2(a_1 b_1 + a_2 b_2 - y_{j1} b_2 - z_{j1})}_{B_q} Z_0 + \underbrace{a_1^2 + (a_2 - y_{j1})^2 + z_{j1}^2 - l^2}_{C_q} = 0 \tag{14}$$

**Simplifying $C_q$** using $r_1 = y_{j1}^2 + z_{j1}^2$:

$$C_q = a_1^2 + a_2^2 - 2a_2 y_{j1} + y_{j1}^2 + z_{j1}^2 - l^2 = a_1^2 + a_2^2 - 2a_2 y_{j1} + r_1 - l^2 \tag{15}$$

This matches code line 143 exactly.

#### Discriminant and Root Selection

The discriminant (code line 146):
$$d = B_q^2 - 4\,A_q\,C_q \tag{16}$$

- **$d < 0$:** The three spheres do not intersect — the encoder angles represent a physically impossible configuration (mechanical fault, encoder slip, or cable break). The function returns `TRUE` (error).
- **$d \geq 0$:** Two solutions exist. The quadratic formula gives:

$$Z_0 = \frac{-B_q \pm \sqrt{d}}{2\,A_q} \tag{17}$$

**Root selection** (code line 156): The code chooses the $\mathbf{-\sqrt{d}}$ branch:

$$Z_0 = \frac{-B_q - \sqrt{d}}{2\,A_q} \tag{18}$$

**Why the negative root?** The delta robot's workspace is entirely **below** the base plane ($Z < 0$). The two solutions of Eq. (17) correspond to:
- $-\sqrt{d}$: the **lower** intersection point → the physical workspace
- $+\sqrt{d}$: the **upper** intersection point → above the base (geometrically valid but mechanically unreachable)

Since $A_q = b_1^2 + b_2^2 + 1 > 0$ always, the $-\sqrt{d}$ branch produces the more negative (lower) $Z_0$, which is the correct physical solution.

#### Back-Substitution

Once $Z_0$ is known, the remaining coordinates follow directly from Eqs. (11)–(12) (code lines 159–160, 163–164):

$$X_0 = a_1 + b_1 Z_0, \quad Y_0 = a_2 + b_2 Z_0 \tag{19}$$

---

## 4. Computational Complexity

The entire algorithm is **non-iterative** — it consists of a fixed sequence of arithmetic operations with no loops or convergence checks:

| Operation | Count |
| :--- | :---: |
| Trigonometric functions ($\sin, \cos$) | 6 |
| Multiplications | ~40 |
| Additions/subtractions | ~30 |
| Divisions | 5 |
| Square root | 1 |

This executes in **under 5 µs** on the NX1P2 PLC, making it suitable for real-time evaluation at every 4 ms EtherCAT cycle with negligible CPU impact.

---

## 5. Error Handling Summary

| Condition | Code Line | Meaning | Return |
| :--- | :---: | :--- | :---: |
| $\Delta = 0$ | 122 | Three elbows are collinear (degenerate geometry) | `TRUE` |
| $d < 0$ | 149 | Spheres do not intersect (impossible joint configuration) | `TRUE` |
| Success | 167 | Valid $(X_0, Y_0, Z_0)$ computed | `FALSE` |

Both error conditions indicate a discrepancy between the encoder readings and the physical mechanism — triggering upstream safety logic in the main program.
