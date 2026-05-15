# Workflow — Development Phase 1: Trajectory Execution

> **Last updated:** 2026-04-10
> **Phase:** Phase 1 — PC sends trajectory, PLC controls motors
> **Status:** Workflow design
> **References:** `system_configuration.md`, `doc(removed)/workflow_algo.md`

---

## 1. Phase 1 Objectives

### In Scope

- **PC** computes the delta robot's movement trajectory (Inverse Kinematics, trajectory planning)
- **PC** packages the trajectory into a data packet and sends it to the PLC over the network
- **PLC** receives the packet, decodes it, and **drives 3 servo motors** along the computed trajectory

### Out of Scope (temporarily deferred)

| Component | Reason |
|-----------|--------|
| Image processing (YOLO, camera) | Later phase |
| Conveyor + encoder | Not yet decided whether to use adaptive conveyor |
| User interface (HMI/GUI) | Not a priority |
| Physical buttons, I/O | Not a priority |
| Pick scheduling, object tracking | Depends on image processing |

### Expected Outcome

> **The delta robot moves accurately along the trajectory computed by the PC.**
> Verified by comparing encoder feedback positions against target positions.

---

## 2. Phase 1 System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    PHASE 1 — ARCHITECTURE                        │
│                                                                  │
│  ┌────────────────────────┐                                      │
│  │        PC (Python)     │                                      │
│  │                        │                                      │
│  │  1. Inverse Kinematics │                                      │
│  │     (x,y,z) → (θ1,θ2,θ3)                                    │
│  │                        │                                      │
│  │  2. Trajectory Planner │                                      │
│  │     Generate point     │                                      │
│  │     sequence           │                                      │
│  │     (θ1,θ2,θ3)(t)     │                                      │
│  │                        │                                      │
│  │  3. Packet Builder     │                                      │
│  │     Package trajectory │                                      │
│  │     into data packet   │                                      │
│  │                        │                                      │
│  │  4. Communication      │                                      │
│  │     Send packet → PLC  │                                      │
│  └───────────┬────────────┘                                      │
│              │                                                    │
│              │  Ethernet (EtherNet/IP or TCP Socket)              │
│              │  Packet: trajectory data                           │
│              │                                                    │
│              ▼                                                    │
│  ┌────────────────────────┐                                      │
│  │   PLC — Omron NX1P2    │                                      │
│  │                        │                                      │
│  │  1. Packet Parser      │                                      │
│  │     Decode packet      │                                      │
│  │     from PC            │                                      │
│  │                        │                                      │
│  │  2. Trajectory Buffer  │                                      │
│  │     Store trajectory   │                                      │
│  │     in buffer          │                                      │
│  │                        │                                      │
│  │  3. Motion Executor    │                                      │
│  │     Each cycle, read   │                                      │
│  │     next point and     │                                      │
│  │     send to driver     │                                      │
│  │                        │                                      │
│  └───────────┬────────────┘                                      │
│              │                                                    │
│              │  EtherCAT (CSP mode, every cycle)                  │
│              │  Target Position for 3 axes                        │
│              │                                                    │
│              ▼                                                    │
│  ┌─────────────────────────────────────────────────┐             │
│  │  3x MADLN05BE + 3x MSMF012L1T2                 │             │
│  │  Driver receives position → controls motor      │             │
│  │  Motor returns encoder feedback → PLC           │             │
│  └─────────────────────────────────────────────────┘             │
│              │                                                    │
│              ▼                                                    │
│  ┌─────────────────────────────────────────────────┐             │
│  │              DELTA ROBOT (3-DOF)                 │             │
│  │      End-effector moves along trajectory         │             │
│  └─────────────────────────────────────────────────┘             │
└─────────────────────────────────────────────────────────────────┘
```

---

## 3. Detailed Data Flow

### 3.1. Step 1 — PC Computes Trajectory

```
Input:  End-effector target coordinates (x, y, z) over time
        e.g., move from (0, 0, -300) to (100, 50, -250)

                        │
                        ▼
        ┌───────────────────────────────┐
        │     INVERSE KINEMATICS        │
        │                               │
        │  (x, y, z) → (θ1, θ2, θ3)   │
        │                               │
        │  Algorithm: Delta geometry    │
        │  (see Section 5 — IK ref)    │
        └───────────────┬───────────────┘
                        │
                        ▼
        ┌───────────────────────────────┐
        │     TRAJECTORY PLANNER        │
        │                               │
        │  Interpolate between points:  │
        │  - Linear interpolation       │
        │  - Or Trapezoidal velocity    │
        │  - Or S-curve                │
        │                               │
        │  Output: point sequence       │
        │  [(θ1,θ2,θ3, t), ...]        │
        │  with dt = PLC T_cycle        │
        └───────────────┬───────────────┘
                        │
                        ▼
Output: Trajectory point array:
        [
          (θ1_0, θ2_0, θ3_0, t=0),
          (θ1_1, θ2_1, θ3_1, t=T),
          (θ1_2, θ2_2, θ3_2, t=2T),
          ...
          (θ1_N, θ2_N, θ3_N, t=N*T)
        ]
        where T = PLC cycle period (0.5 – 4 ms)
```

### 3.2. Step 2 — PC Packages and Sends Packet

```
Trajectory array
        │
        ▼
┌───────────────────────────────────────┐
│          PACKET BUILDER               │
│                                       │
│  Header:                              │
│    - Packet ID (uint16)               │
│    - Num points N (uint16)            │
│    - Cycle time T_ms (uint16)         │
│    - Additional params (see Sec. 4)   │
│                                       │
│  Payload:                             │
│    - N x (θ1, θ2, θ3) — each angle   │
│      as int32 (unit: encoder pulse)   │
│                                       │
│  Checksum (optional)                  │
└──────────────────┬────────────────────┘
                   │
                   │  Ethernet
                   │  (TCP Socket or EtherNet/IP)
                   ▼
             PLC NX1P2
```

### 3.3. Step 3 — PLC Receives, Buffers, and Executes

```
PLC receives packet from PC
        │
        ▼
┌───────────────────────────────────────┐
│          PACKET PARSER                │
│                                       │
│  Read header → determine num points N │
│  Read payload → store in BUFFER[]     │
│  Confirm → send ACK to PC            │
└──────────────────┬────────────────────┘
                   │
                   ▼
┌───────────────────────────────────────┐
│        TRAJECTORY BUFFER              │
│                                       │
│  BUFFER[0..N-1]                       │
│  Each element: (θ1, θ2, θ3)         │
│                                       │
│  index := 0                           │
│  state := READY                       │
└──────────────────┬────────────────────┘
                   │  Trigger: PC sends START command
                   │  or auto-start after full reception
                   ▼
┌───────────────────────────────────────┐
│        MOTION EXECUTOR                │
│        (runs in Primary Task)         │
│                                       │
│  Every T_cycle:                       │
│    if index < N:                      │
│      MC_MoveAbsolute(Axis1, θ1[i])   │
│      MC_MoveAbsolute(Axis2, θ2[i])   │
│      MC_MoveAbsolute(Axis3, θ3[i])   │
│      index++                          │
│    else:                              │
│      state := DONE                    │
│      Notify PC                        │
│                                       │
│  Or use CSP mode directly:            │
│    Write Target Position to PDO       │
│    every cycle → driver executes      │
└───────────────────────────────────────┘
```

---

## 4. Packet Format (Draft)

> **Note:** This is a preliminary design. Must be revised once the specific communication method is determined.

### 4.1. PC → PLC Packet (Trajectory Command)

```
┌────────────────────────────────────────────────────┐
│                   HEADER (12 bytes)                 │
├──────────┬──────────┬───────────┬─────────────────  │
│ Byte 0-1 │ Byte 2-3 │ Byte 4-5  │ Byte 6-11       │
│ CMD_ID   │ NUM_PTS  │ CYCLE_MS  │ RESERVED         │
│ (uint16) │ (uint16) │ (uint16)  │ (6 bytes)        │
├──────────┴──────────┴───────────┴──────────────────┤
│                   PAYLOAD (N x 12 bytes)           │
├────────────┬────────────┬────────────┐             │
│ θ1 (int32) │ θ2 (int32) │ θ3 (int32) │  Point 0    │
├────────────┼────────────┼────────────┤             │
│ θ1 (int32) │ θ2 (int32) │ θ3 (int32) │  Point 1    │
├────────────┼────────────┼────────────┤             │
│ ...        │ ...        │ ...        │  ...        │
├────────────┼────────────┼────────────┤             │
│ θ1 (int32) │ θ2 (int32) │ θ3 (int32) │  Point N-1  │
├────────────┴────────────┴────────────┘             │
│                   FOOTER (4 bytes)                  │
├─────────────────────────────────────────────────────│
│ CRC32 or Checksum (uint32)                         │
└────────────────────────────────────────────────────┘

Total size = 12 + (N x 12) + 4 bytes
e.g., 1000 points → 12 + 12000 + 4 = 12,016 bytes ≈ 12 KB
```

### 4.2. PLC → PC Packet (Status Response)

```
┌────────────────────────────────────────┐
│           STATUS RESPONSE (16 bytes)    │
├──────────┬──────────┬──────────────────┤
│ Byte 0-1 │ Byte 2   │ Byte 3-14       │
│ CMD_ID   │ STATUS   │ ACTUAL_POS      │
│ (uint16) │ (uint8)  │ θ1,θ2,θ3(int32)│
├──────────┼──────────┼──────────────────┤
│ Byte 15  │          │                  │
│ RESERVED │          │                  │
└──────────┴──────────┴──────────────────┘

STATUS values:
  0x00 = ACK (received successfully)
  0x01 = BUSY (currently executing)
  0x02 = DONE (trajectory completed)
  0x03 = ERROR
```

### 4.3. Additional Parameters (not yet clarified)

| Parameter | Description | Status |
|-----------|-------------|--------|
| Angle unit θ | Encoder pulse? Radian? Degree? | **Needs standardization** |
| Max velocity per axis | Safety limit | Requires delta mechanical analysis |
| Max acceleration per axis | Safety limit | Requires delta mechanical analysis |
| Home position (angle = 0) | Initial reference angle | Define after homing procedure |
| Interpolation type in PLC | PLC interpolates further or only plays each point | **TBD** |

---

## 5. Timing Sequence Diagram

```
  PC (Python)                    PLC (NX1P2)               Drives + Motors
      │                              │                          │
      │  ===== PHASE: INIT ====      │                          │
      │                              │                          │
      │  1. Establish Ethernet conn  │                          │
      │ ────────────────────────►    │                          │
      │         ACK                  │                          │
      │ ◄────────────────────────    │                          │
      │                              │  Servo ON, Homing        │
      │                              │ ────────────────────►    │
      │                              │         Done             │
      │                              │ ◄────────────────────    │
      │                              │                          │
      │  ===== PHASE: SEND =====     │                          │
      │                              │                          │
      │  2. Compute IK + Trajectory  │                          │
      │     (offline on PC)          │                          │
      │                              │                          │
      │  3. Send trajectory packet   │                          │
      │ ────────────────────────►    │                          │
      │                              │  4. Parse + Buffer       │
      │         ACK                  │                          │
      │ ◄────────────────────────    │                          │
      │                              │                          │
      │  ===== PHASE: EXEC =====     │                          │
      │                              │                          │
      │  5. Send START command       │                          │
      │ ────────────────────────►    │                          │
      │                              │  6. Every cycle T:       │
      │                              │     Read BUFFER[i]       │
      │                              │     Write Target Pos     │
      │                              │ ────────────────────►    │
      │                              │                          │  Motor rotates
      │                              │     Actual Pos feedback  │
      │                              │ ◄────────────────────    │
      │                              │     i++                  │
      │                              │                          │
      │                              │  ... (repeat N times)    │
      │                              │                          │
      │         STATUS: DONE         │                          │
      │ ◄────────────────────────    │                          │
      │                              │                          │
      │  ===== PHASE: VERIFY ====    │                          │
      │                              │                          │
      │  7. Read actual position log │                          │
      │ ────────────────────────►    │                          │
      │         Position data        │                          │
      │ ◄────────────────────────    │                          │
      │                              │                          │
      │  8. Compare target vs actual │                          │
      │     Validate error margins   │                          │
      ▼                              ▼                          ▼
```

---

## 6. PLC State Machine

```
┌───────────────────────────────────────────────────────────┐
│                  PLC STATE MACHINE                         │
└───────────────────────────────────────────────────────────┘

         ┌──────────────────────┐
         │   STATE: POWER_ON    │
         │   - Init EtherCAT    │
         │   - Wait for PC conn │
         └──────────┬───────────┘
                    │  PC connected successfully
                    ▼
         ┌──────────────────────┐
         │   STATE: IDLE        │
         │   - Servo OFF        │
         │   - Waiting for cmd  │
         └──────────┬───────────┘
                    │  Receive SERVO_ON + HOMING cmd
                    ▼
         ┌──────────────────────┐
         │   STATE: HOMING      │
         │   - Servo ON         │
         │   - Execute homing   │
         │     for 3 axes       │
         └──────────┬───────────┘
                    │  Homing completed
                    ▼
         ┌──────────────────────┐
         │   STATE: READY       │◄────────────────────┐
         │   - Waiting for traj │                      │
         │   - Servo ON, idle   │                      │
         └──────────┬───────────┘                      │
                    │  Trajectory packet received       │
                    ▼                                   │
         ┌──────────────────────┐                      │
         │   STATE: BUFFERED    │                      │
         │   - Trajectory stored│                      │
         │   - Waiting for START│                      │
         └──────────┬───────────┘                      │
                    │  START command received            │
                    ▼                                   │
         ┌──────────────────────┐                      │
         │   STATE: EXECUTING   │                      │
         │   - Read BUFFER[i]   │                      │
         │   - Write Target Pos │                      │
         │   - i++ every cycle  │                      │
         └──────────┬───────────┘                      │
                    │  i == N (all points consumed)     │
                    ▼                                   │
         ┌──────────────────────┐                      │
         │   STATE: DONE        │──────────────────────┘
         │   - Notify PC: DONE  │   PC sends new trajectory
         │   - Hold final pos   │   or RESET
         └──────────────────────┘

  FROM ANY STATE:
    E-STOP received → STATE: FAULT → Servo OFF → Requires RESET to return to IDLE
```

---

## 7. Open Issues to Resolve

### 7.1. High Priority (decide before coding)

| # | Issue | Options | Notes |
|---|-------|---------|-------|
| 1 | **PC ↔ PLC communication** | EtherNet/IP, TCP Socket, FINS | Affects entire packet design |
| 2 | **Angle unit** | Encoder pulse (int32) vs Radian (float) | Encoder pulse is simpler, but IK typically returns radians |
| 3 | **Where to interpolate** | PC generates full resolution (one point per cycle) vs PLC interpolates between sparse points | Affects packet size and trajectory smoothness |
| 4 | **Where to run IK** | PC (Python — easy to debug) vs PLC (ST — realtime) | Phase 1: PC. Later phases: may move to PLC |

### 7.2. Low Priority (resolve later)

| # | Issue | Notes |
|---|-------|-------|
| 5 | Acceleration / jerk limits | Requires actual robot mechanical parameters |
| 6 | Communication error handling | Retry, timeout, watchdog |
| 7 | Encoder feedback logging | For verification and debugging |

---

## 8. Next Steps (Action Items)

- [ ] Determine PC ↔ PLC communication method (investigate EtherNet/IP and TCP Socket)
- [ ] Design detailed packet structure (after communication method is decided)
- [ ] Write IK module for the delta robot on PC (Python)
- [ ] Write trajectory planner on PC (linear / trapezoidal / S-curve)
- [ ] Configure EtherCAT on NX1P2 with 3 Panasonic drivers (Sysmac Studio)
- [ ] Write PLC program: parser + buffer + motion executortime
- [ ] Test homing procedure for 3 axes
- [ ] Test with simple trajectory (e.g., single-axis linear move)
- [ ] Test with simultaneous 3-axis trajectory
- [ ] Compare target vs actual position, evaluate error margins

---

*This file describes the workflow for Phase 1 — update as progress is made. Any infomation could be changed in no time*
