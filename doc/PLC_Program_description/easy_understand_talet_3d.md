# Mathematical Proof: Why LERP Ensures Absolute 3-Axis Synchronization

In multi-axis motion control (such as CNC machines, 3D printers, or robotic arms), keeping the $X$, $Y$, and $Z$ axes perfectly synchronized is critical. The Linear Interpolation (LERP) algorithm achieves this flawlessly through a single concept: **Parametric Synchronization via a Master Conductor, $\lambda(t)$**.

Instead of letting each axis calculate its own independent timeline, all three axes are mathematically locked to a single shared progress variable, $\lambda(t)$, at every execution cycle.

---

## 1. Absolute Boundary Constraints (Start & End Points)

Let's look at the system of parametric equations for each axis:

$$\begin{cases} 
X_P(t) = X_A + \lambda(t) \cdot (X_B - X_A) \\ 
Y_P(t) = Y_A + \lambda(t) \cdot (Y_B - Y_A) \\ 
Z_P(t) = Z_A + \lambda(t) \cdot (Z_B - Z_A) 
\end{cases}$$

The normalized progress parameter $\lambda(t)$ acts as a universal percentage gauge bound strictly within $[0, 1]$.

* **At the Start ($t = 0$):** The total distance traveled $S(0) = 0$, meaning $\lambda(0) = 0$. Substituting $\lambda = 0$ into the equations completely eliminates the displacement terms:
  $$X_P = X_A, \quad Y_P = Y_A, \quad Z_P = Z_A$$
  *Result:* All three axes are mathematically forced to be at the starting coordinate $\mathbf{A}$ at the exact same moment.

* **At the End ($t = T_{end}$):** When the robot completes the path, the accumulated distance equals the total length ($S(T_{end}) = L$), meaning $\lambda(T_{end}) = 1$. Substituting $\lambda = 1$ into the system yields:
  $$X_P = X_A + 1 \cdot (X_B - X_A) = X_B$$
  $$Y_P = Y_A + 1 \cdot (Y_B - Y_A) = Y_B$$
  $$Z_P = Z_A + 1 \cdot (Z_B - Z_A) = Z_B$$
  *Result:* All three axes reach their respective target coordinates simultaneously the exact millisecond $\lambda$ reaches $1$. No axis can arrive early or late.

---

## 2. Proportional Velocity Scaling

To see how the axes maintain synchronization *during* the transition between points $\mathbf{A}$ and $\mathbf{B}$, we take the **first derivative (velocity)** of the position equations with respect to time $t$:

$$\begin{cases}
v_x(t) = \frac{dX_P}{dt} = \frac{d\lambda}{dt} \cdot (X_B - X_A) \\
v_y(t) = \frac{dY_P}{dt} = \frac{d\lambda}{dt} \cdot (Y_B - Y_A) \\
v_z(t) = \frac{dZ_P}{dt} = \frac{d\lambda}{dt} \cdot (Z_B - Z_A)
\end{cases}$$

In this system, $\frac{d\lambda}{dt}$ (the rate of change of the progress parameter) is a **universal scalar multiplier** shared by all three equations at any given instant $t$.

> 📌 **Core Insight:**
> The velocity of each individual axis automatically scales based on the net distance it needs to cover ($\Delta X = X_B - X_A$, $\Delta Y = Y_B - Y_A$, $\Delta Z = Z_B - Z_A$).
> * An axis with a **longer distance** to travel is mathematically assigned a **higher velocity**.
> * An axis with a **shorter distance** to travel is automatically throttled to a **slower velocity**.

Because their velocities are perfectly proportioned to their structural distances, they run out of travel budget at the exact same time.

---

## 3. Comparison: Independent Control vs. Parametric LERP Control

| Feature | Independent Axis Control (Unsynchronized) | Parametric Control (Synchronized LERP) |
| :--- | :--- | :--- |
| **Mechanism** | Each axis drives at its own speed profile to reach its target coordinate. | All axes modulate their velocities bound to the master progress metric $\lambda$. |
| **Behavior** | Axes with shorter travel distances finish early and stall; longer axes finish late. | All axes start, accelerate, cruise, and stop in perfect unison. |
| **Toolpath Accuracy** | The resulting trajectory warps and distorts heavily in 3D space. | The tool tip maintains a **perfectly straight line** across 3D space. |