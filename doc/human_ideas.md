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

## 3. packages
package:
1. pc package
 to omron plc package:
{
    "commandID": 0,
    "argument_number": 0,
    "argument_x": [0.0] * argument_number,
    "argument_y": [0.0] * argument_number,
    "argument_z": [0.0] * argument_number,
    "argument_e": [0] * argument_number, # end effector state along trajectory: 0 open, 1 pick
    "argument_time": [0.0] * argument_number,
    "doing_bit": 1,
}
 to siemen plc package:
 {
    "CommandID": 0,
    "rotate": 0.0,
    "speed": 0.0
 }
2. plc package
 from omron plc:
{
    "pos_angular": [theta1, theta2, theta3]
    "pos_EE": [x, y, z] # catersian coordinate of end effector
    "task_doing": 0,
    "task_state": 0
}
 from siemen plc:
 {
    "rotate_current": 0.0,
    "speed_current": 0.0,
    "task_doing": 0,
    "task_state": 0
 }
3. commandID (also task_doing)
COMMAND_ID = {
    "stop": 0,
    "goto_relative": 1,
    "goto_absolute": 2,
    "go_trajectory": 3,
    "calibrate": 4,
    "pick": 5,
    "release": 6,
    "rotate_absolute": 7,
    "change_speed": 8,
    "plan_siemen": 9
}

4. task state value
- 0 = done
- 1 = doing
- 2 = error

# Trajectory planning: 
(*new) Quỹ đạo 7 điểm:
            ----------------
          /                 \
         /                   \
        |                     |
        |                     |
      

# 28-5
## tình trạng hiện tại
- mô phỏng hoạt động tạm ổn, thể hiện ra một vài lỗi logic trong lập quỹ đạo. Cần có mô đun xử lý ảnh hoàn chỉnh để test sâu hơn -> tạm thời bỏ qua.
- về kết nối:
 - chưa test kết nối với plc siemen. tạm bỏ qua do chưa có chương trình trên plc siemen.
 - plc omron hoạt động rất tốt. đang có kết hoạch tăng số lượng điểm nội suy và điều chỉnh quỹ đạo.

## task
- xây dựng chương trình để hỗ trợ lấy mẫu train model yolo. cần xác định rõ logic trước.
- tăng số điểm nội suy và điều chỉnh quỹ đạo lên tối đa 11 điểm - cần kiểm chứng tính khả thi sau. điểm nội suy và cơ chế hoạt động hiện tại không phù hợp với ứng dụng bên dưới.
- xác định rõ ứng dụng xử lý ảnh thực tế, yêu cầu phải thực dụng: phân loại và gắp mạch pcb vào khay tương ứng. hiện có 2 loại hình vuông với 2 size khác nhau (25x25mm và 40x40mm)