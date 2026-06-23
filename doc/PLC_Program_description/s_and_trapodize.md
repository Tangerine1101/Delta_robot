# Mathematical Analysis: Why Trapezoidal Profile Replaces S-Curve for Non-Zero Velocity Boundaries

> **Context:** This document explains the engineering decision made in `MC_Inter_Curve_Vel` to use a **Polynomial S-Curve** only when both boundary velocities are zero, and to fall back to a **Trapezoidal (linear acceleration)** profile for all other cases — specifically when $V_{start} \neq 0$ or $V_{end} \neq 0$.

---

## 1. The Core Trade-off

In motion control, the choice between a Trapezoidal profile and an S-Curve profile is a direct trade-off between **mechanical motion quality** and **CPU computational load**.

| Property | Trapezoidal | S-Curve |
| :--- | :--- | :--- |
| Position function degree | Quadratic ($2^{nd}$) | Quartic ($4^{th}$) |
| Velocity function degree | Linear ($1^{st}$) | Cubic ($3^{rd}$) |
| Acceleration continuity | Discontinuous (step change) | Continuous (smooth bell) |
| Jerk (rate of accel. change) | Infinite at phase corners | Bounded and finite |
| Handles $V_{start} \neq 0$? | **Yes — trivially** | **No — requires re-derivation** |
| Computational complexity | Very low | Very high |

The critical insight is that **the S-Curve polynomial is mathematically brittle**: it is derived under the assumption that boundary velocities and accelerations are zero. As soon as $V_{start} \neq 0$, the entire derivation collapses and must be rebuilt from scratch — at a computational cost that a real-time PLC cannot sustain.

---

## 2. Trapezoidal Profile — A Low-Order Linear Problem

The trapezoidal profile is built on one principle: **constant acceleration** ($a = \text{const}$). This makes the velocity function first-order and the position function second-order.

### 2.1 Equations for Arbitrary Boundary Velocities

Given any entry velocity $V_{start}$ and a required acceleration to reach $V_{peak}$:

**Phase duration (computed once at segment start):**
$$t_{acc} = \frac{|V_{peak} - V_{start}|}{A_{max}} \tag{1}$$

**Displacement during acceleration phase** (evaluated every 4 ms):
$$S(t) = V_{start} \cdot t + \frac{1}{2} A_{max} \cdot t^2 \tag{2}$$

**Velocity at any time** $t$:
$$v(t) = V_{start} + A_{max} \cdot t \tag{3}$$

### 2.2 Why This Is Computationally Trivial

The entire profile setup requires **one division** (Eq. 1), computed once at the start of a segment. During each subsequent 4 ms scan cycle, the PLC evaluates only Eq. (2) — two multiplications and one addition. A modern PLC CPU completes this in **under 1 microsecond**.

Crucially, **$V_{start}$ is just a number plugged directly into Eq. (2)**. There is no structural change to the formula regardless of what $V_{start}$ is. This is the key property that makes the trapezoidal profile suitable for real-time continuous motion.

---

## 3. S-Curve Profile — A Nonlinear Boundary-Value Problem

The S-Curve was designed to eliminate jerk discontinuities by making **jerk** ($j = d^3S/dt^3$) constant rather than infinite. This promotes the velocity function to a cubic polynomial and the position function to a quartic polynomial.

### 3.1 The General Velocity Polynomial

A general jerk-limited velocity profile has the form:
$$v(t) = c_3 t^3 + c_2 t^2 + c_1 t + c_0 \tag{4}$$

with derivatives:
$$a(t) = \frac{dv}{dt} = 3c_3 t^2 + 2c_2 t + c_1 \tag{5}$$
$$j(t) = \frac{da}{dt} = 6c_3 t + 2c_2 \tag{6}$$

The four unknown coefficients $c_0, c_1, c_2, c_3$ must be determined by imposing **four boundary conditions** on the phase.

### 3.2 The Zero-Velocity Special Case (Works Perfectly)

When both the start and end of a phase have zero velocity **and** zero acceleration:

$$v(0) = 0 \implies c_0 = 0$$
$$a(0) = 0 \implies c_1 = 0$$

The system reduces to only two unknowns ($c_2, c_3$) with two remaining boundary conditions at $t = t_{acc}$. This is a simple $2 \times 2$ linear system — solvable analytically in microseconds.

The result is the **Hermite smoothstep** polynomial (as used in `MC_Inter_Curve_Vel` State 2, S-Curve branch):
$$v(\tau) = V_{max}\left(3\tau^2 - 2\tau^3\right), \quad \tau = \frac{t}{t_{acc}} \tag{7}$$

This is elegant, cheap to compute, and baked in as a fixed formula.

### 3.3 The Non-Zero Velocity Case — Mathematical Collapse

Now suppose the segment begins with arbitrary $V_{start}$ and $A_{start}$ (inherited from the previous segment's exit state), and must end at $V_{end}$ and $A_{end}$. The four boundary conditions are:

$$v(0) = c_0 = V_{start} \tag{8}$$
$$a(0) = c_1 = A_{start} \tag{9}$$
$$v(T) = c_3 T^3 + c_2 T^2 + A_{start} T + V_{start} = V_{end} \tag{10}$$
$$a(T) = 3c_3 T^2 + 2c_2 T + A_{start} = A_{end} \tag{11}$$

From Eqs. (10) and (11), $c_2$ and $c_3$ are found by solving the **matrix system**:

$$\begin{bmatrix} T^2 & T^3 \\ 2T & 3T^2 \end{bmatrix} \begin{bmatrix} c_2 \\ c_3 \end{bmatrix} = \begin{bmatrix} V_{end} - V_{start} - A_{start}T \\ A_{end} - A_{start} \end{bmatrix} \tag{12}$$

The matrix determinant is:
$$\det = T^2 \cdot 3T^2 - T^3 \cdot 2T = 3T^4 - 2T^4 = T^4$$

Solving by Cramer's rule:
$$c_2 = \frac{3(V_{end} - V_{start} - A_{start}T) - T(A_{end} - A_{start})}{T^2} \cdot \frac{1}{...} \tag{13}$$

The formula involves **multiple divisions by powers of $T$**. But here is the fundamental problem: **$T$ itself is unknown**.

---

## 4. Why $T$ Cannot Be Known in Advance — The Root of the Problem

To solve Eq. (12), the total phase duration $T$ must be known first. But $T$ is not a free parameter — it is constrained by the physical limits of the actuator:

$$\max_{t \in [0,T]}|a(t)| \leq A_{max} \quad \text{and} \quad \max_{t \in [0,T]}|j(t)| \leq J_{max} \tag{14}$$

### 4.1 Finding the Acceleration Peak

The acceleration maximum occurs where jerk equals zero (setting Eq. (6) to zero):
$$j(t_{crit}) = 6c_3 \cdot t_{crit} + 2c_2 = 0 \implies t_{crit} = -\frac{c_2}{3c_3} \tag{15}$$

Substituting $t_{crit}$ back into Eq. (5) gives $a_{max}$. But $c_2$ and $c_3$ are themselves functions of $T$ from Eq. (12). Therefore, the constraint $a_{max} \leq A_{max}$ becomes:

$$f(T) = a\!\left(-\frac{c_2(T)}{3c_3(T)}\right) - A_{max} = 0 \tag{16}$$

This is a **high-degree nonlinear equation in $T$**. No closed-form solution exists. The CPU must solve it numerically.

### 4.2 The Iterative Solver Problem

To find $T$, a numerical root-finding algorithm must be used — such as Newton-Raphson or bisection. Each iteration of Newton-Raphson requires:

$$T_{k+1} = T_k - \frac{f(T_k)}{f'(T_k)} \tag{17}$$

where computing $f'(T_k)$ requires differentiating Eq. (16) with respect to $T$ — which means differentiating the coefficients $c_2(T)$ and $c_3(T)$ from Eq. (12) as functions of $T$, then propagating through Eq. (15) and Eq. (5). This is a full chain-rule differentiation computed numerically at each iteration.

**Typical convergence requires 5–15 iterations**, and **each iteration requires ~10–20 floating-point divisions and multiplications**.

---

## 5. Why a 4 ms Cycle Cannot Sustain This

The Omron NX1P2 PLC in this system runs its primary periodic task at **4 ms**. This 4 ms budget is shared across all real-time activities:

| Task | Typical Budget |
| :--- | :--- |
| EtherCAT PDO read/write (3 axes) | ~0.3 ms |
| Safety logic, alarm handling | ~0.5 ms |
| PLC I/O scan (digital in/out) | ~0.2 ms |
| Main program ladder/ST execution | ~1.5 ms |
| **Available for trajectory math** | **~0.5 ms** |

A single Newton-Raphson iteration on Eq. (16) with full coefficient chain-rule differentiation takes approximately **50–200 µs** on a soft-core PLC CPU (which lacks a dedicated FPU pipeline). With 10 iterations to converge, that is **0.5–2.0 ms** — already consuming or exceeding the entire trajectory computation budget.

### 5.1 The Watchdog Timer Risk

PLC hardware implements a **Watchdog Timer** as a safety mechanism. If any scan cycle exceeds the configured maximum execution time (typically set equal to the task period: 4 ms), the watchdog fires and executes an **emergency stop** of the entire system — cutting all servo commands simultaneously.

The danger of running an iterative solver with **unbounded iteration count** in a real-time task:
- The number of Newton-Raphson iterations needed to converge is not known in advance — it depends on the initial guess quality, which varies with $V_{start}$ magnitude.
- In edge cases (poorly conditioned $T$ estimates, $V_{start}$ close to $V_{max}$), convergence may require 20+ iterations.
- This produces **unpredictable CPU spikes** that randomly exceed the 4 ms boundary.
- The resulting Watchdog trip causes an uncontrolled emergency stop mid-motion — far more dangerous than the smooth motion quality the S-Curve was trying to achieve.

---

## 6. The Engineering Decision

The correct engineering choice is a **hybrid approach**:

| Condition | Profile | Justification |
| :--- | :--- | :--- |
| $V_{start} = 0$, $V_{end} = 0$ | **S-Curve polynomial** | Zero boundaries collapse the math to a fixed formula. No iteration needed. Motion quality is maximized at critical pick/place positions where the end-effector is stationary — vibration here directly damages positioning accuracy. |
| $V_{start} > 0$ or $V_{end} > 0$ | **Trapezoidal** | Arbitrary boundary velocities require $T$ to be found iteratively — computationally unsafe in a 4 ms cycle. The trapezoidal profile solves $T$ explicitly in $O(1)$ time via Eq. (1). The brief jerk spike at phase transitions is mechanically acceptable because the platform is moving and inertia absorbs transient impulses. |

The **fixed polynomial** S-Curve (Eq. 7) avoids all these problems because it is not a general solver — it is a **lookup formula** derived once for the specific boundary conditions $v(0) = 0,\ v(t_{acc}) = V_{max},\ a(0) = 0,\ a(t_{acc}) = 0$. Its coefficients ($c_0 = 0, c_1 = 0, c_2 = 3V_{max}/t_{acc}^2, c_3 = -2V_{max}/t_{acc}^3$) are computed analytically in advance, not solved iteratively at runtime.

---

## 7. Summary Comparison

| Criterion | Trapezoidal | S-Curve (General) |
| :--- | :--- | :--- |
| **Position function order** | $2^{nd}$ degree | $4^{th}$ degree |
| **Handles $V_{start} \neq 0$** | Plug directly into Eq. (2) | Must resolve all 4 polynomial coefficients |
| **$T$ determination** | Explicit formula — $O(1)$, one division | Nonlinear equation — requires iterative solver |
| **Algorithm complexity** | Low: $+, -, \times, \div$ only | High: matrix inversion + numerical root-finding |
| **CPU time** | Fixed, $< 1\,\mu\text{s}$ | Variable, $0.5$–$2.0\,\text{ms}$ per segment start |
| **Watchdog safety** | Deterministic — never spikes | Non-deterministic — spike risk on each segment |
| **Mechanical quality** | Finite jerk at phase transitions | Bounded jerk throughout |
| **Suited for 4 ms cycle** | ✓ Perfect for standard PLC | ✗ Requires dedicated DSP or offline pre-computation |

> **Conclusion:** The use of S-Curve is limited to the **zero-start, zero-stop** case because only in that case does the mathematical problem degenerate into a closed-form formula safe for real-time execution. For all blending segments where $V_{start} \neq 0$, the trapezoidal profile is the only computationally feasible choice on an embedded PLC running a 4 ms hard real-time cycle.