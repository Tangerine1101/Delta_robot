# Delta Robot Pick-and-Place: Human Research & Future Ideas
> **Target Audience**: Human Developers & Researchers (Graduation Thesis Team)
> **AI Instruction**: AI agents should AVOID modifying this file unless explicitly requested by a human.

This document serves as a collaborative notebook for brainstorming, thesis research topics, and future architectural expansion proposals.

---

## 1. Thesis Research & Academic Topics

### 1.1. Advanced Closed-Loop Conveyor Control
* **Objective**: Move from passive conveyor tracking to active, bidirectional closed-loop speed regulation.
* **Problem**: Currently, the conveyor speed is an input constraint $\mathbf{v}(t)$ used to predict pickup time. If the queue of detected items spikes (queue pressure increases), the robot might miss objects.
* **Proposed Solution**: Implement an adaptive control loop using Little's Law / conveyor throughput equations:
  $$v(t) = \frac{L_{wk} \cdot \eta}{u(t)}$$
  Where the PC calculates the optimal conveyor speed $v(t)$ to match the robot's peak physical sorting rate $\eta$ and writes it back to the Siemens S7-1200 controller to dynamically slow down or speed up the conveyor.
* **Investigation Points**:
  - Stability criteria for the conveyor speed actuator under varying item density.
  - Acceleration/deceleration limits of conveyor steppers to avoid item slippage.

### 1.2 Object future location predict
Since camera lose track of objects location after they leave the camera zone, therefore we need to use integration of real time velocity vector to predict future location:
$$\vec{P}_{predicted} = \vec{P}_{detected} + \int_{t_{detected}}^{t_{pick}} \vec{v}_{conveyor} dt$$

And since the speed of conveyor apply to all object, then we only need to use 1 value of integral of speed and add it to detected objects locations to predict future location.
$$\vec{V} = \vec{V}_{x} + \vec{V}_{y}$$

To simplify the calculation and logic, we assume the conveyor placed along the Y axis. objects move from low y value to higher y value. and x value is fixed along the path (-50 - 50).
---

## 2. System Integration Proposals

### 2.1. Web GUI and Monitoring Dashboard (Thread 4)
* **Objective**: Build an intuitive, real-time web interface for diagnostic logging and telemetry monitoring.
* **Features**:
  - Live 3D visualization of the delta arm's end-effector position.
  - Graphing real-time coordinate tracking (parsing `data.log` dynamically).
  - Status panel showing PLC tags, active commands, error codes, and speed vector $\mathbf{v}$.
  - Interactive queue visualizer showing conveyor items inside the camera zone and active pickup window.
* **Tech Stack Ideas**:
  - Backend: Python FastAPI (asynchronous, high performance) or Flask + WebSockets.
  - Frontend: React or Vue.js with Three.js (for 3D web rendering of the robot).
  - Telemetry Database: SQLite or PostgreSQL to log throughput metrics, plan latencies, and sorting totals.

### 2.2. Object Sorting Database Schema
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
    dispatch_timestamp REAL,
    actual_speed_x REAL,
    actual_speed_y REAL,
    status VARCHAR(20) DEFAULT 'planned', -- 'completed', 'failed', 'stale'
    latency_s REAL
);
```

### 2.3. Dual-PLC Master Coordination
* **Objective**: Design clean interface logic between Omron NX1P2 and Siemens S7-1200.
* **Questions to Solve**:
  - Handshaking protocol: If the S7-1200 adjusts the conveyor speed, how does it notify the PC and Omron PLC synchronously without timing lag?
  - Network overhead: Does Modbus TCP or raw TCP socket communication between S7-1200 and Python introduce significant jitter to the pick prediction loop?
