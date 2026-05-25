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
optimize speed by reduce movement into 4 points and 2 phases: goto and pick phase where goto is when the robot move from current position to the conveyor and pick phase is when robot pick the object and move it to above of sort zone and release the end effector - a suction cup
C_pick/B_goto->   --------------------  D_pick/A_goto
                 / <- a slope to maintain speed on trajectory.
                /
                | <- C_goto/B_pick
                |
                | <- D_goto (a bit higher than A_pick so when the robot press down, the cup will suck and pick up object perpectly)
        A_pick->|

A_goto and D_pick not neccessary to equal, because multiple objects type require different bins location. Similarly for C_pick and B_goto, differents bins require different trajectory.

Chu kỳ gồm 2 phase: GOTO và PICK. Mỗi phase có 4 điểm.
Mục tiêu quỹ đạo:
Robot phải tránh các đoạn "đi ngang rồi hạ thẳng đứng" quá gắt. Hai đoạn chính:
- B_goto -> C_goto
- B_pick -> C_pick

bắt buộc là đoạn nghiêng 3D, tức là vừa di chuyển XY vừa thay đổi Z, để duy trì tốc độ động cơ tốt hơn và tăng throughput.

PHASE GOTO: đi từ vị trí hiện tại đến gần vật
A_goto = điểm bắt đầu/điểm thả trước đó, ở cao độ an toàn.
B_goto = điểm cao, bắt đầu đoạn nghiêng đi xuống về phía vật.
C_goto = điểm kết thúc đoạn nghiêng, đã gần phía trên vật nhưng vẫn cao hơn D_goto.
D_goto = điểm pre-pick, cùng X/Y với vật, cao hơn A_pick một chút.

Bắt buộc:
B_goto -> C_goto là đoạn nghiêng 3D:
- XY tiến dần về P_pick.
- Z giảm dần từ clearance_z xuống approach_z/pre_pick_z.
- Không được tách thành "đi ngang ở clearance rồi hạ thẳng".

PHASE PICK: hút vật và đưa đến bin
A_pick = điểm chạm/hút vật.
B_pick = điểm nâng vật lên khỏi mặt băng tải, nhưng chưa lên hẳn clearance.
C_pick = điểm kết thúc đoạn nghiêng, đã lên cao và tiến về phía bin.
D_pick = điểm thả vật tại bin.

Bắt buộc:
B_pick -> C_pick là đoạn nghiêng 3D:
- XY tiến dần từ P_pick sang P_place.
- Z tăng dần từ pick/approach height lên clearance/place-transfer height.
- Không được tách thành "nâng thẳng đứng rồi đi ngang".

D_goto và A_pick:
- Cùng X/Y tại vị trí vật.
- D_goto cao hơn A_pick.
- D_goto là chuẩn bị, A_pick là nhấn xuống hút.

C_goto và B_pick:
- Không nhất thiết phải hoàn toàn cùng Z nếu muốn tối ưu động học.
- Chúng nằm gần vùng pick.
- C_goto là kết thúc đoạn nghiêng khi đi vào.
- B_pick là bắt đầu đoạn nghiêng khi đi ra.

B_goto và C_pick:
- Có thể nằm ở vùng cao/clearance.
- Đây là đầu/cuối của các đoạn nghiêng.
- B_goto thuộc đường vào vật.
- C_pick thuộc đường ra bin.

A_goto và D_pick:
- Không bắt buộc bằng nhau.
- D_pick phụ thuộc bin của object hiện tại.
- A_goto của chu kỳ sau chính là vị trí robot đang đứng sau chu kỳ trước.