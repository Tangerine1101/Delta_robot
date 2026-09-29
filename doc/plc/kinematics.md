# On-PLC Kinematics

> **Source.** The declarations and ST bodies of the three functions in both `.smc2` projects,
> which are byte-identical between the two program versions. A port is in
> [`modules/plc_sim/omron_core.py`](../../modules/plc_sim/omron_core.py).
>
> The kinematic model itself (geometry, derivation) is in
> [`../basis-theory.md`](../basis-theory.md) §2. This file states what the PLC code does,
> including its limits and sign conventions.

## 1. Geometry constants

| Code name | Symbol | Value |
|---|---|---|
| `Base` | base triangle side $s_b$ | 346.4 mm (inradius $\tfrac12\tan30°\,s_b$ = 100 mm) |
| `EndEffector` | platform triangle side $s_p$ | 86.6 mm (inradius 25 mm) |
| `Bicep` | active arm $L$ | 140.0 mm |
| `Forearm` | passive arm $l$ | 315.0 mm |

Robot frame: origin at the base centre, Z negative downwards (work plane ≈ −260 … −303 mm);
arm 1 lies along −Y.

## 2. Inverse kinematics — `Calc_Inverse_Kinematics(X, Y, Z) → θ1, θ2, θ3`

Each arm is solved in its own vertical plane by `Calc_Angles_YZ`, after rotating the target
into that arm's frame:

| Arm | Input to `Calc_Angles_YZ` | Output |
|---|---|---|
| 1 | (X, Y, Z) | θ1 = θ |
| 2 | X cos120 + Y sin120, Y cos120 − X sin120, Z | θ2 = θ |
| 3 | X cos120 − Y sin120, Y cos120 + X sin120, Z | θ3 = **−θ** |

`Calc_Angles_YZ(X0, Y0, Z0)`:

1. `Z0 = 0` → error (division guard).
2. $y_1 = -\tfrac12\tan30°\,s_b$, $y_0' = Y_0 - \tfrac12\tan30°\,s_p$.
3. Elbow on the line $z_j = a + b\,y_j$, with
   $a = (X_0^2 + y_0'^2 + Z_0^2 + L^2 - l^2 - y_1^2)/(2Z_0)$, $b = (y_1 - y_0')/Z_0$.
4. Discriminant $d = -(a + b y_1)^2 + L^2(b^2 + 1)$; $d < 0$ → unreachable → error.
5. $y_j = (y_1 - ab - \sqrt d)/(b^2+1)$ (the outward elbow), $z_j = a + b y_j$.
6. $\theta = \operatorname{atan}(-z_j/(y_1 - y_j))\cdot180/\pi$ (+180° if $y_j > y_1$), with a
   ±90° branch when $y_1 = y_j$.

**Joint limit inside IK.** The function tests θ1 < −20°, θ2 < −20° and the negated
θ3 > 20°, i.e. every raw arm angle must be ≥ −20°.

When only this limit trips, the function sets a *local* flag and `RETURN`s. Its return
value is still the FALSE of the last successful `Calc_Angles_YZ`, and the outputs of the
failing arm and of every arm after it are left unwritten. The caller therefore sees
"success" (defect **O1**).

Only a real no-solution case (Z = 0 or d < 0) is reported as an error. Inside the
interpolator, that error stops the trajectory and is sticky (defect **P2**).

The −20° limit binds only near the top of the working volume: for z ≥ −265 mm the
worst-heading XY reach falls to 170 mm, and at −260 mm it falls to 140 mm. Below that, the
no-solution boundary (≈ 255–275 mm) is the limit. Reach per height is tabulated in
[`config-review.md`](config-review.md) §1.

## 3. Forward kinematics — `Calc_Forward_Kinematic(θ1, θ2, θ3) → X, Y, Z`

1. Radians: t1 = θ1, t2 = θ2, **t3 = −θ3** (undoes the IK sign flip).
2. $w = \tfrac12\tan30°(s_b - s_p)$. Elbows (already shifted by the platform offset):
   $J_1 = (0, -(w + L\cos t_1), -L\sin t_1)$,
   $J_2 = ((w + L\cos t_2)\cos30, (w + L\cos t_2)\sin30, -L\sin t_2)$,
   $J_3 = (-(w + L\cos t_3)\cos30, (w + L\cos t_3)\sin30, -L\sin t_3)$.
3. Intersect the three spheres of radius $l$: subtracting sphere 1 from spheres 2 and 3 gives
   two linear equations; Cramer's rule gives $X = a_1 + b_1 Z$, $Y = a_2 + b_2 Z$
   (`det = 0` → error).
4. Substituting into sphere 1 gives $A Z^2 + B Z + C = 0$; $d < 0$ → error; the lower root
   $Z = (-B - \sqrt d)/2A$ is taken.

FK runs every scan on the encoder angles (rung 9) and is the source of `pos_EE`.

**The error result is never returned.** It is assigned to a local variable named
`Calc_Forward_Kinematics`, while the function is `Calc_Forward_Kinematic`. On a degenerate
pose the X/Y/Z outputs are therefore unwritten, and `pos_EE` reads their default value with
no error flag (defect **O9**).

## 4. Consistency with the PC

The gateway does not run IK. It only checks the motion envelope (`robot_limits`: XY radius
and z band, `comm.omron.PLCGateway._check_workspace_limit`) before sending; it does **not** check the
−20° joint limit (the realtime planner and the web console run `ik_reachable` themselves). A point that passes the PC check can still fail on the PLC.
