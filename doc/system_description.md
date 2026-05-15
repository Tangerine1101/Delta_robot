# System Description — Delta Robot Pick-and-Place Project

> **Last updated:** 2026-04-10
> **Project type:** Graduation thesis
> **Topic:** Delta robot control for product sorting on a variable-speed conveyor, using YOLO image processing

> **⚠ Disclaimer:** This document was not written by a professional engineer. Its sole purpose is to provide structured context for AI-assisted development. Technical details may be incomplete or imprecise.

---

## 1. Project Overview

### 1.1. Problem Statement

In industrial pick-and-place applications, a delta robot must pick products from a moving conveyor and sort them into designated bins. The core challenge is that the **conveyor speed is not fixed** — it varies adaptively based on system load. This means the robot cannot rely on pre-programmed static trajectories; it must synchronize its motion with a continuously changing target.

### 1.2. Final System Vision (Full Scope)

```
┌─────────────────────────────────────────────────────────────────────┐
│                        FULL SYSTEM VISION                           │
│                                                                     │
│   Camera ──► PC (YOLO + Scheduler + Trajectory) ──► PLC (NX1P2)     │
│                                                         │           │
│                                          ┌──────────────┼────────┐  │
│                                          │              │        │  │
│                                          ▼              ▼        ▼  │
│                                     Servo Drives   Conveyor   Bins  │
│                                          │                          │
│                                          ▼                          │
│                                     Delta Robot                     │
│                                    (pick & place)                   │
└─────────────────────────────────────────────────────────────────────┘
```

The complete system would include:

1. **Camera + YOLO** — detect and classify products on the conveyor
2. **Tracking + Scheduler** — track product positions, decide pick order and timing
3. **Trajectory Planning + IK** — compute robot joint trajectories to reach each target
4. **PLC + Servo Drives** — execute motion in real-time
5. **Adaptive Conveyor** — adjust conveyor speed based on system load

### 1.3. Current Development Phase

> **Phase 1 goal: Prove that the PC can command the PLC to move the delta robot along a pre-computed trajectory.**

This phase focuses exclusively on the **motion execution pipeline** — everything from trajectory computation on the PC down to physical motor movement. Vision, scheduling, conveyor control, and UI are deferred.

---

## 2. System Architecture

### 2.1. Hardware

| # | Component | Model | Qty | Role |
|---|-----------|-------|-----|------|
| 1 | Machine Controller | Omron NX1P2-1140DT | 1 | Motion master — receives trajectory from PC, drives servos via EtherCAT |
| 2 | Servo Driver | Panasonic MADLN05BE (MINAS A6BE) | 3 | EtherCAT slave — executes position commands, returns encoder feedback |
| 3 | Servo Motor | Panasonic MSMF012L1T2 (MINAS A6) | 3 | 100W AC servo with 23-bit absolute encoder — one per robot arm |
| 4 | Delta Robot | 3-DOF parallel mechanism | 1 | Mechanical structure driven by the 3 motors |
| 5 | PC | Laptop / Desktop (Python) | 1 | Computes IK and trajectory, sends packets to PLC |

> Full hardware specifications: see `system_configuration.md`

### 2.2. Communication Stack

```
┌────────────┐      Ethernet (TBD)      ┌────────────┐      EtherCAT       ┌────────────┐
│     PC     │ ───────────────────────► │   NX1P2    │ ──────────────────► │  3x Driver │
│  (Python)  │ ◄─────────────────────── │   (PLC)    │ ◄────────────────── │  + 3x Motor│
└────────────┘   Trajectory packets     └────────────┘   CSP mode (cyclic  └────────────┘
                 Status responses                        position every
                                                         0.5–4 ms)
```

| Link | Protocol | Status | Purpose |
|------|----------|--------|---------|
| PC → PLC | EtherNet/IP or TCP Socket | **TBD** | Send trajectory data and commands |
| PLC → Drivers | EtherCAT (CoE, CSP mode) | **Confirmed** | Real-time servo control, < 1 us jitter |

### 2.3. Software Components

| Component | Runs on | Language | Description |
|-----------|---------|----------|-------------|
| Inverse Kinematics | PC | Python | Converts end-effector (x,y,z) to joint angles (θ1,θ2,θ3) |
| Trajectory Planner | PC | Python | Interpolates between waypoints (linear / trapezoidal / S-curve) |
| Packet Builder | PC | Python | Serializes trajectory into binary packet for transmission |
| Communication Client | PC | Python | Sends/receives packets over Ethernet |
| Packet Parser | PLC | Structured Text | Deserializes received packets, stores in buffer |
| Motion Executor | PLC | Structured Text | Reads buffer each cycle, writes target position to EtherCAT PDO |

---

## 3. How It Works (Phase 1)

### Step-by-step

```
 ① PC computes trajectory offline
    (x,y,z) waypoints → IK → (θ1,θ2,θ3) → interpolation → point array

 ② PC sends trajectory packet to PLC
    [Header: ID, N points, cycle time] + [N × (θ1,θ2,θ3)] + [CRC]

 ③ PLC buffers the trajectory
    Stores all N points in internal memory

 ④ PLC executes on START command
    Every 0.5–4 ms: reads next point → writes to 3 servo drives via EtherCAT

 ⑤ Servo drives move the motors
    CSP mode: driver receives position target each cycle, closes position loop

 ⑥ Robot moves
    3 motors rotate → delta mechanism translates to end-effector (x,y,z) motion

 ⑦ PC verifies
    Reads actual encoder positions from PLC → compares with target → evaluates error
```

### What success looks like

- End-effector follows the commanded trajectory with acceptable position error
- All 3 axes stay synchronized throughout the motion
- No communication drops or buffer underruns during execution

---

## 4. What Changed from the Original Plan

The project started with a different hardware and scope assumption (documented in `doc(removed)/`). Key changes:

| Aspect | Original (brainstorm) | Current | Why |
|--------|----------------------|---------|-----|
| PLC | Siemens S7-1200 | Omron NX1P2-1140DT | Built-in EtherCAT master, native motion control |
| PLC-Drive link | Modbus TCP / Analog signals | EtherCAT | Real-time, synchronized, industry standard for servo |
| Motors | Unspecified | Panasonic MSMF012L1T2 (100W) | Available hardware with 23-bit encoder |
| Drivers | Unspecified | Panasonic MADLN05BE (EtherCAT) | Matched to motors and PLC |
| Adaptive conveyor | Included (DC Motor + PID) | **Deferred** | Not yet decided if adaptive conveyor will be used |
| Vision (YOLO) | Included | **Deferred to later phase** | Focus on motion pipeline first |
| HMI / GUI | Included | **Deferred** | Not a priority |

---

## 5. Project Scope by Phase

### Phase 1 — Trajectory Execution (current)

| In scope | Out of scope |
|----------|-------------|
| Inverse Kinematics (PC) | Image processing (YOLO) |
| Trajectory planning (PC) | Object tracking / scheduling |
| PC → PLC communication | Conveyor control |
| PLC motion execution (EtherCAT + CSP) | HMI / GUI |
| Encoder feedback verification | Physical buttons / safety I/O |

> Detailed workflow: see `workflow.md`

### Phase 2+ — Future (not yet planned in detail)

- Integrate camera + YOLO for real-time product detection
- Implement pick scheduler with priority queue
- Add conveyor control (if adaptive conveyor is adopted)
- Closed-loop: vision → schedule → trajectory → execute → feedback
- Performance evaluation: throughput, miss rate, position accuracy

---

## 6. Open Questions

| # | Question | Impact | Status |
|---|----------|--------|--------|
| 1 | PC ↔ PLC communication method? | Determines packet design and library choice | **TBD** — investigating EtherNet/IP vs TCP Socket |
| 2 | Driver 50W vs Motor 100W — compatible? | May limit torque/speed if mismatched | **Needs verification** |
| 3 | Where does interpolation happen? | PC sends every point vs PLC interpolates sparse points | **TBD** |
| 4 | Will adaptive conveyor be used? | Affects overall system complexity and Phase 2 scope | **TBD** |
| 5 | IK on PC or PLC long-term? | PC is easier to debug; PLC is real-time | Phase 1: PC. Revisit later |

---

## 7. Document Index

| File | Location | Description |
|------|----------|-------------|
| `system_configuration.md` | `doc/` | Hardware specs, communication options, compatibility notes |
| `workflow.md` | `doc/` | Phase 1 data flow, packet format, PLC state machine, sequence diagram |
| `system_description.md` | `doc/` | This file — project overview and big picture |
| `brainstorm_review.md` | `doc(removed)/` | Original brainstorm (reference only — many ideas changed) |
| `workflow_algo.md` | `doc(removed)/` | Original algorithm design (reference only) |

---

*This file provides the high-level view of the project. For implementation details, see the linked documents above.*
