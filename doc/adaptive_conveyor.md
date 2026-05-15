pandoc doc/adaptive_conveyor.md -o adaptive_conveyor.pdf --pdf-engine=xelatex -V geometry:margin=2.5cm -V fontsize=12pt -V monofont="DejaVu Sans Mono" 2>&1

# Adaptive Conveyor

## 1. Overview

Adaptive Conveyor is a mechanism that **automatically adjusts conveyor belt speed** based on the number of products currently awaiting sorting. The goal is to optimize the Delta robot's pick rate by keeping the product density on the belt within the robot's effective processing range.

**Operating principle:**

- When products are **scarce** → the conveyor runs **faster** to bring new products into the workspace sooner.
- When products are **abundant** → the conveyor **slows down** so the robot has enough time to process each product.

---

## 2. Mathematical Foundation

### 2.1. Conveyor Speed Equation

$$
v(t) = \frac{L_{wk} \cdot \eta}{u(t)}
$$

| Symbol | Description | Unit |
|--------|-------------|------|
| $v(t)$ | Conveyor speed at time $t$ | m/s |
| $L_{wk}$ | Length of the robot's workspace along the conveyor | m |
| $\eta$ | Throughput — pick rate | picks/s |
| $u(t)$ | Number of products on the conveyor at time $t$ | — |

**Physical interpretation:** Conveyor speed is inversely proportional to the number of products. As $u(t)$ increases, $v(t)$ decreases — giving the robot more time to handle each product within the workspace $L_{wk}$.

### 2.2. Position and Time Prediction System

To pick a product moving on the conveyor, the robot must **predict** the pick position $X_p$ and the travel time $t_p$ required to reach that position.

#### Object Position Equation

$$
X_p = x(t) + v(t) \cdot t_p
$$

| Symbol | Description |
|--------|-------------|
| $X_p$ | Predicted pick position |
| $x(t)$ | Current position of the product on the conveyor |
| $v(t)$ | Current conveyor speed |
| $t_p$ | Time required for the robot to reach $X_p$ |

#### Robot Constraint Equation

Assuming the algorithm operates in **actuator space** and the actuator trajectory approximates a **trapezoidal velocity profile**:

$$
t_p = \frac{S}{V_{max}} + \frac{V_{max}}{A_{max}}
$$

where:

$$
S = X_p - x(t)
$$

| Symbol | Description | Unit |
|--------|-------------|------|
| $S$ | Distance the robot must travel (in actuator space) | m |
| $V_{max}$ | Maximum actuator velocity | m/s |
| $A_{max}$ | Maximum actuator acceleration | m/s² |

> **Note:** The two equations above form an implicit system — $X_p$ depends on $t_p$ and vice versa. They must be solved simultaneously (via substitution or iteration) to find a feasible pair $(X_p, t_p)$.

---

## 3. Open Questions

### 3.1. Sampling Period of $v(t)$

Should the conveyor speed be updated in **real time** (continuous update) or only **after the robot completes its current pick cycle** (discrete, per-cycle update)?

| Approach | Pros | Cons |
|----------|------|------|
| **Real-time** | Fast response to product density changes | Speed change mid-pick → position prediction error |
| **Per pick cycle** | Stable during pick, easier to control | Slow response to sudden density changes |

### 3.2. Algorithm Workspace

Should the prediction algorithm operate in **joint space** or **actuator space**?

| Space | Characteristics |
|-------|-----------------|
| **Joint space** | Direct encoder feedback, no conversion needed. Clear velocity/acceleration limits per joint |
| **Actuator space** | Closer to Cartesian space, more intuitive for conveyor tracking. However, requires handling singularities and inter-axis coupling |

---

## 4. Logic Diagram

```
┌─────────────┐      ┌──────────────┐      ┌─────────────────┐
│  Camera /   │────▶│  Count u(t)  │────▶│  Compute v(t)   │
│  Sensor     │      │  products    │      │  = L_wk·η/u(t)  │
└─────────────┘      └──────────────┘      └────────┬────────┘
                                                    │
                                                    ▼
                                           ┌─────────────────┐
                                           │ Update conveyor │
                                           │ speed           │
                                           └────────┬────────┘
                                                    │
                    ┌──────────────┐                │
                    │ Solve system │◀──────────────┘
                    │  (X_p, t_p)  │
                    └──────┬───────┘
                           │
                           ▼
                    ┌──────────────┐
                    │  Robot       │
                    │  executes    │
                    │  pick        │
                    └──────────────┘
```
