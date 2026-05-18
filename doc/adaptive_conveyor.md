# Adaptive Conveyor

## 1. Overview

Adaptive Conveyor is the future mechanism that provides conveyor-speed state to the pick scheduler and may later also receive control requests back from the system.

Current repository status:

- scheduler already assumes a conveyor-speed input exists
- real speed input from `EthernetCom` is **not integrated yet**
- the current scheduler uses a simulated speed source
- current software treats speed primarily as an **input to prediction**

The immediate purpose of conveyor-speed data is:

- predict where the object will be at pickup time
- decide whether the object is still reachable
- build the outbound leg so the robot arrives slightly early above the future pickup point

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

This is still a good high-level model for future closed-loop conveyor behavior.

### 2.2. Position and Time Prediction

To pick a moving product, the scheduler must estimate a future pickup point:

$$
X_p = x(t) + v(t) \cdot t_p
$$

| Symbol | Description |
|--------|-------------|
| $X_p$ | Predicted pickup position |
| $x(t)$ | Current product position |
| $v(t)$ | Conveyor speed sample used by the scheduler |
| $t_p$ | Time until robot reaches the pickup point |

Robot timing can still be approximated by a travel model:

$$
t_p = \frac{S}{V_{max}} + \frac{V_{max}}{A_{max}}
$$

where:

$$
S = X_p - x(t)
$$

In the current repository, this is simplified in code into a segment-timing model based on:

- nominal XY speed
- nominal Z speed
- a fixed intercept lead time
- a fixed 6-point path template

---

## 3. Current Software Assumptions

The current scheduler implementation uses these practical assumptions:

### 3.1. Speed is frozen while planning one pick

The scheduler uses the latest available speed sample to build one `PickPlan`. That sample is treated as fixed during planning of that cycle.

This is currently the safest software assumption because:

- it avoids continuous mid-pick replanning
- it is easier to benchmark
- it matches the current simulator structure

### 3.2. Speed source shape

The future real input is expected to look like:

- `speed`
- `timestamp`

This matches the current internal `SpeedSample` model.

### 3.3. Current integration path

Planned long-term path:

```
Conveyor-speed PLC
        │
        ▼
EthernetCom
        │
        ▼
Scheduler
        │
        ▼
PickPlan
```

Current temporary path:

```
SimulatedSpeedSource
        │
        ▼
Scheduler
        │
        ▼
PickPlan
```

---

## 4. Relation to the Current Scheduler

The current scheduler does not only choose an object. It also uses conveyor speed to:

1. estimate whether the object is still reachable
2. predict where the object will be at pickup time
3. generate the outbound 6-point leg
4. place point 5 slightly early above the predicted pickup point
5. descend at point 6 and switch suction on

After pickup, the inbound leg is independent of conveyor speed and goes to the sorting position configured for the object type.

---

## 5. Benchmark Relevance

Conveyor-speed logic directly affects the current benchmark scenarios:

### 5.1. `test_throughput`

- simulated speed changes the rate at which objects enter the reachable pickup window
- affects queue pressure and number of successful plans

### 5.2. `test_accuracy`

- current test uses fixed points and simulated trace logging
- useful for validating motion-template timing and logging behavior
- less focused on conveyor dynamics than `test_throughput`

---

## 6. Current Open Questions

| Question | Current repository choice | Future work |
|----------|---------------------------|-------------|
| Should speed update continuously during one pick? | No, current plan is effectively frozen per cycle | Evaluate if late replanning is needed |
| Where does speed come from? | Simulated speed source | Replace with `EthernetCom` input |
| Is conveyor speed only an input, or also a control output? | Input only for now | Later may support closed-loop conveyor control |
| Which space should the prediction live in? | Practical Cartesian-like scheduler approximation | Revisit if actuator/joint constraints become dominant |

---

## 7. Logic Diagram

```
┌────────────────────┐      ┌─────────────────────┐
│ Object detections  │────▶│      Scheduler      │
└────────────────────┘      │                     │
                            │  predict pickup     │
┌────────────────────┐────▶│  using speed input  │
│ Conveyor speed     │      │  build PickPlan     │
└────────────────────┘      └─────────┬───────────┘
                                      │
                                      ▼
                             ┌──────────────────┐
                             │ Outbound 6-point │
                             │ pickup leg       │
                             └─────────┬────────┘
                                       │
                                       ▼
                             ┌──────────────────┐
                             │ Inbound 6-point  │
                             │ sorting leg      │
                             └──────────────────┘
```

---

*This document now reflects how conveyor-speed assumptions relate to the scheduler that already exists in the repository, while keeping the original mathematical intuition for future adaptive control.*
