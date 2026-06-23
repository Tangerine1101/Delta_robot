# Delta Robot Pick-and-Place: Formal Academic & Mathematical Report
> **Target Audience**: Academic Reviewers, Kinematics Researchers, and Thesis Evaluators.
> **Note**: This document acts as an academic archive of the mathematical model, kinematics, and trajectory generation algorithms. It is not intended as a developer guide for daily codebase modifications.

---

## 1. Coordinate Frames & Spatial Transformations

To define the kinematics and vision-tracking loop, three Cartesian coordinate frames are established:

1.  **Robot Frame (R-frame)**: Centered at the upper fixed base plate, with $+Z$ pointing upward (the workspace lies entirely in $Z < 0$).
2.  **Conveyor Frame (C-frame)**: A 2D planar coordinate system aligned with the belt surface, where $+u$ is downstream and $+v$ is transverse.
3.  **Vision Frame (V-frame)**: 2D pixel space $(p_x, p_y)$ of the camera.

### 1.1. Homogeneous Transform F ($C \to R$)
Assuming the conveyor belt lies parallel to the robot $XY$ plane at a constant pickup height $Z = Z_{\text{pickup}}$, the 2D homogeneous transformation matrix $\mathbf{F}$ is defined as:

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

where:
*   $\theta$ is the angular rotation of the conveyor relative to the robot.
*   $(T_X, T_Y)$ represents the translation vector of the conveyor origin.

### 1.2. Planar Homography H ($V \to C$)
For a flat conveyor surface, camera pixel coordinates $(p_x, p_y)$ map to C-frame $(u, v)$ via homography matrix $\mathbf{H}$:

$$
\lambda 
\begin{bmatrix} 
u \\ 
v \\ 
1 
\end{bmatrix} 
= \mathbf{H} 
\begin{bmatrix} 
p_x \\ 
p_y \\ 
1 
\end{bmatrix}
$$

---

## 2. Kinematics of the Delta Robot

The geometric parameters of the delta mechanism are:
*   $s_b$: Base equilateral triangle side length (320.0 mm)
*   $s_p$: End-effector platform side length (94.0 mm)
*   $L$: Bicep arm length (140.0 mm)
*   $l$: Forearm link length (315.0 mm)

### 2.1. Inverse Kinematics (IK)
Given Cartesian coordinates $(X_0, Y_0, Z_0)$, find the joint angles $(\theta_1, \theta_2, \theta_3)$. Due to $120^\circ$ rotational symmetry, the 3D problem is rotated into three identical 2D single-arm solvers in the local $YZ$-plane.

#### 2.1.1. Single-Arm 2D Solver (`Calc_Angles_YZ`)
The base joint is at:
$$y_1 = -\frac{s_b}{2\sqrt{3}}$$
The platform connection point is at:
$$y_{\text{tmp}} = Y_0 - \frac{s_p}{2\sqrt{3}}$$

The elbow joint $(0, y_j, z_j)$ must satisfy:
1.  **Bicep constraint**: $(y_j - y_1)^2 + z_j^2 = L^2$
2.  **Forearm constraint**: $X_0^2 + (y_{\text{tmp}} - y_j)^2 + (Z_0 - z_j)^2 = l^2$

Subtracting these equations yields a linear relationship:
$$z_j = a + b \cdot y_j$$
where:
$$a = \frac{X_0^2 + y_{\text{tmp}}^2 + Z_0^2 + L^2 - l^2 - y_1^2}{2Z_0}$$
$$b = \frac{y_1 - y_{\text{tmp}}}{Z_0}$$

Substituting $z_j$ back into the bicep circle equation yields a quadratic:
$$A y_j^2 + B y_j + C = 0$$
where:
*   $A = 1 + b^2$
*   $B = 2(ab - y_1)$
*   $C = y_1^2 + a^2 - L^2$

The half-discriminant $d$ is:
$$d = \frac{\Delta}{4} = -(a + b\,y_1)^2 + L^2(b^2 + 1)$$
If $d < 0$, the point is unreachable. The physical "elbow-down" solution is selected:
$$y_j = \frac{y_1 - ab - \sqrt{d}}{b^2 + 1}$$
$$z_j = a + b \cdot y_j$$

The joint angle is computed as:
$$\theta = \frac{180}{\pi} \cdot \arctan\!\left(\frac{-z_j}{y_1 - y_j}\right) + \theta_{\text{offset}}$$
where $\theta_{\text{offset}} = 180^\circ$ if $y_j > y_1$, else $0^\circ$.

#### 2.1.2. Top-Level Coordinator
The target coordinate $(X, Y)$ is rotated for each arm:
*   **Arm 1**: $\theta_1 = \text{IK\_Solver}(X, Y, Z)$
*   **Arm 2**: $\theta_2 = \text{IK\_Solver}(X_2, Y_2, Z)$, where:
    $$X_2 = X\cos(120^\circ) + Y\sin(120^\circ)$$
    $$Y_2 = -X\sin(120^\circ) + Y\cos(120^\circ)$$
*   **Arm 3**: $\theta_3 = -\text{IK\_Solver}(X_3, Y_3, Z)$ (negated due to inverted motor direction), where:
    $$X_3 = X\cos(120^\circ) - Y\sin(120^\circ)$$
    $$Y_3 = X\sin(120^\circ) + Y\cos(120^\circ)$$

---

### 2.2. Forward Kinematics (FK)
Given joint angles $(\theta_1, \theta_2, \theta_3)$, calculate $(X_0, Y_0, Z_0)$. This is solved using the intersection of three spheres.

#### 2.2.1. Elbow Coordinates
Let $w = \frac{s_b - s_p}{2\sqrt{3}}$. Project the elbows in the rotated vertical planes:
$$\mathbf{J}_1 = \begin{pmatrix} 0 \\ -(w + L\cos \theta_1) \\ -L\sin \theta_1 \end{pmatrix}$$
$$\mathbf{J}_2 = \begin{pmatrix} (w + L\cos \theta_2)\cos 30^\circ \\ (w + L\cos \theta_2)\sin 30^\circ \\ -L\sin \theta_2 \end{pmatrix}$$
$$\mathbf{J}_3 = \begin{pmatrix} -(w + L\cos \theta_3)\cos 30^\circ \\ (w + L\cos \theta_3)\sin 30^\circ \\ -L\sin \theta_3 \end{pmatrix}$$

#### 2.2.2. Linearization and Cramer's Rule
The three spheres of radius $l$ are centered at $\mathbf{J}_i$:
$$\|\mathbf{P} - \mathbf{J}_i\|^2 = l^2, \quad i = 1, 2, 3$$
Subtracting sphere 1 from spheres 2 and 3 eliminates quadratic terms, giving a linear system:
$$\begin{bmatrix} A_1 & B_1 \\ A_2 & B_2 \end{bmatrix} \begin{bmatrix} X_0 \\ Y_0 \end{bmatrix} = \begin{bmatrix} D_1 - C_1 Z_0 \\ D_2 - C_2 Z_0 \end{bmatrix}$$
where:
*   $A_1 = x_{j2}$, $B_1 = y_{j2} - y_{j1}$, $C_1 = z_{j2} - z_{j1}$, $D_1 = 0.5(r_2 - r_1)$
*   $A_2 = x_{j3}$, $B_2 = y_{j3} - y_{j1}$, $C_2 = z_{j3} - z_{j1}$, $D_2 = 0.5(r_3 - r_1)$
*   $r_i = x_{ji}^2 + y_{ji}^2 + z_{ji}^2$

Using Cramer's Rule:
$$X_0 = a_1 + b_1 Z_0, \quad Y_0 = a_2 + b_2 Z_0$$
where $a_1, b_1, a_2, b_2$ are derived constants.

#### 2.2.3. Quadratic in $Z_0$
Substituting $X_0(Z_0)$ and $Y_0(Z_0)$ back into the sphere 1 equation yields:
$$A_q Z_0^2 + B_q Z_0 + C_q = 0$$
where:
*   $A_q = b_1^2 + b_2^2 + 1$
*   $B_q = 2(a_1 b_1 + a_2 b_2 - y_{j1}b_2 - z_{j1})$
*   $C_q = a_1^2 + a_2^2 - 2a_2 y_{j1} + r_1 - l^2$

The physical lower intersection root is selected:
$$Z_0 = \frac{-B_q - \sqrt{B_q^2 - 4A_q C_q}}{2A_q}$$

---

## 3. Path Planning & Real-Time Trajectory Profiles

### 3.1. Polynomial S-Curve Profile (Jerk-Bounded)
For stationary endpoints ($V_{start} = 0, V_{end} = 0$), a 4th-order position profile (smoothstep) limits the jerk:

$$\tau = \frac{t}{t_{\text{acc}}}$$
$$S(\tau) = V_{\text{max}} \cdot t_{\text{acc}} \cdot \left(\tau^3 - \frac{1}{2}\tau^4\right)$$
$$v(\tau) = V_{\text{max}} \cdot \left(3\tau^2 - 2\tau^3\right)$$
$$a(\tau) = \frac{6V_{\text{max}}}{t_{\text{acc}}} \cdot \tau(1 - \tau)$$

#### 3.1.1. Shape Compensation Factor
A linear ramp takes $t_{\text{lin}} = V_{\text{max}}/A_{\text{max}}$ to accelerate. The parabolic profile peaks at $\tau = 0.5$, which requires:
$$a_{\text{peak}} = \frac{1.5V_{\text{max}}}{t_{\text{acc}}} \stackrel{!}{=} A_{\text{max}} \implies t_{\text{acc}} = \frac{1.5 V_{\text{max}}}{A_{\text{max}}}$$
The **1.5× factor** compensates for the smooth acceleration shape.

---

### 3.2. Blended Corner Velocity
When traversing waypoint $\mathbf{B}$ to $\mathbf{C}$ without stopping, the corner velocity is bounded based on the direction change angle $\Theta$:
$$\cos(\Theta) = \frac{(\mathbf{B}-\mathbf{A}) \cdot (\mathbf{C}-\mathbf{B})}{\|\mathbf{B}-\mathbf{A}\|\|\mathbf{C}-\mathbf{B}\|}$$

Applying the half-angle identity, the corner velocity limit is:
$$V_{\text{corner}} = V_{\text{max}} \cdot \cos\left(\frac{\Theta}{2}\right) = V_{\text{max}} \cdot \sqrt{\frac{\cos(\Theta) + 1}{2}}$$

This reduces centripetal acceleration and mechanical shock during corner transitions.
