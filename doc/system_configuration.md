# System Configuration — Delta Robot Project

> **Last updated:** 2026-04-10
> **Phase:** Development Phase 1 — Trajectory Execution
> **Status:** Hardware confirmed, communication method under investigation

---

## 1. Overall Hardware Diagram

```
┌──────────────────────────────────────────────────────────────────┐
│                       HARDWARE SYSTEM                            │
│                                                                  │
│  ┌──────────┐    Ethernet / EtherCAT (TBD)                      │
│  │  PC      │ ──────────────────────────┐                        │
│  │ (Python) │                           │                        │
│  └──────────┘                           ▼                        │
│                              ┌─────────────────────┐             │
│                              │  Omron NX1P2        │             │
│                              │  CPU: 1140DT        │             │
│                              │                     │             │
│                              │  EtherCAT Master    │             │
│                              └────────┬────────────┘             │
│                                       │ EtherCAT                 │
│                      ┌────────────────┼────────────────┐         │
│                      │                │                │         │
│                      ▼                ▼                ▼         │
│               ┌────────────┐   ┌────────────┐   ┌────────────┐  │
│               │ Driver #1  │   │ Driver #2  │   │ Driver #3  │  │
│               │ MADLN05BE  │   │ MADLN05BE  │   │ MADLN05BE  │  │
│               └─────┬──────┘   └─────┬──────┘   └─────┬──────┘  │
│                     │                │                │         │
│                     ▼                ▼                ▼         │
│               ┌────────────┐   ┌────────────┐   ┌────────────┐  │
│               │ Motor #1   │   │ Motor #2   │   │ Motor #3   │  │
│               │ MSMF012L1T2│   │ MSMF012L1T2│   │ MSMF012L1T2│  │
│               └────────────┘   └────────────┘   └────────────┘  │
│                     │                │                │         │
│                     └────────────────┼────────────────┘         │
│                                      │                          │
│                                      ▼                          │
│                              ┌───────────────┐                  │
│                              │  DELTA ROBOT  │                  │
│                              │  (3-DOF)      │                  │
│                              └───────────────┘                  │
└──────────────────────────────────────────────────────────────────┘
```

---

## 2. PLC — Omron NX1P2-1140DT

### Key Specifications

| Parameter | Value |
|-----------|-------|
| **Model** | NX1P2-1140DT |
| **CPU Series** | NX1P2 (Machine Automation Controller) |
| **Motion control** | Built-in EtherCAT Master |
| **Max motion axes** | 8 axes (EtherCAT servo) |
| **Task period (fastest)** | 0.5 ms (Primary Periodic Task) |
| **Built-in EtherNet/IP** | Yes — for PC communication |
| **Built-in EtherCAT** | Yes — for servo drive control |
| **Programming languages** | Structured Text (ST), Ladder, Function Block |
| **Software** | Sysmac Studio |

### Role in the System

- Receive trajectory packets from PC via **Ethernet** (EtherNet/IP or TCP/UDP socket)
- Perform **trajectory interpolation** based on received data
- Control **3 servo drives** via EtherCAT (Position / Velocity / Torque mode)
- Ensure **3-axis synchronization** every control cycle

### Notes

- NX1P2 is a **Machine Controller**, not a traditional PLC — significantly more capable of motion control than the S7-1200
- EtherCAT is a **real-time fieldbus** protocol, ensuring jitter < 1 us between PLC and servo drives
- **PC ↔ PLC** communication method not yet determined:
  - **Option 1:** EtherNet/IP (built-in on NX1P2)
  - **Option 2:** TCP/UDP socket via Ethernet port
  - **Option 3:** EtherCAT (PC as EtherCAT master — more complex, needs further investigation)

---

## 3. Servo Driver — Panasonic MADLN05BE

### Key Specifications

| Parameter | Value |
|-----------|-------|
| **Model** | MADLN05BE |
| **Product line** | MINAS A6BE (A6B EtherCAT variant) |
| **Rated power** | 50 W |
| **Communication protocol** | **EtherCAT (CoE — CANopen over EtherCAT)** |
| **Control modes** | Position / Velocity / Torque (selected via EtherCAT) |
| **Encoder feedback** | 23-bit absolute encoder (from motor) |
| **Encoder resolution** | 8,388,608 pulses/rev (2^23) |
| **Quantity** | **3 units** (one driver per delta robot axis) |

### Role in the System

- Receive position/velocity commands from PLC via **EtherCAT**
- Execute **motor control loops** (current loop, velocity loop, position loop)
- Return **encoder feedback** (actual position, actual velocity) to PLC
- Support **motion profiles** per CiA 402 (Homing, Profile Position, Cyclic Synchronous Position, etc.)

### Relevant EtherCAT Modes

| Mode (CiA 402) | Description | Suitability |
|-----------------|-------------|-------------|
| **CSP (Cyclic Synchronous Position)** | PLC sends new position every cycle | **Preferred — best fit for trajectory control** |
| CSV (Cyclic Synchronous Velocity) | PLC sends new velocity every cycle | Backup option |
| PP (Profile Position) | Driver generates its own profile | Not suitable — driver self-interpolates |
| HM (Homing) | Return to home position | Required for startup procedure |

> **Recommendation:** Use **CSP (Cyclic Synchronous Position)** mode — PLC computes the target position each cycle and sends it to the driver. This is the most common approach for delta robot control.

---

## 4. Servo Motor — Panasonic MSMF012L1T2

### Key Specifications

| Parameter | Value |
|-----------|-------|
| **Model** | MSMF012L1T2 |
| **Product line** | MINAS A6 Family |
| **Rated power** | 100 W |
| **Rated torque** | 0.32 N·m |
| **Peak torque** | 0.96 N·m (300% of rated) |
| **Rated speed** | 3000 rpm |
| **Maximum speed** | 5000 rpm |
| **Encoder** | 23-bit absolute (built-in) |
| **Voltage** | 200 V (single-phase / three-phase) |
| **Quantity** | **3 units** (one motor per delta robot arm) |

### Role in the System

- Directly drive the **3 arms (links)** of the delta robot
- Provide precise position feedback via built-in 23-bit encoder
- 100W power rating suitable for small-to-medium delta robots (workspace ~200–400mm)

### Warning

- Driver rated power (50W) < Motor rated power (100W) — **compatibility must be verified**
  - If MADLN05BE only supports up to 50W, torque/speed may be limited
  - Check driver datasheet to confirm rated output current >= motor rated current
- If incompatible, upgrade driver to a 100W model (e.g., MADLN15BE or equivalent)

---

## 5. Connectivity and Communication

### 5.1. EtherCAT Bus (PLC ↔ Drivers)

```
NX1P2 (Master)
    │
    │  EtherCAT (daisy-chain)
    │
    ├── Driver #1 (MADLN05BE) ── Motor #1 (J1)
    │
    ├── Driver #2 (MADLN05BE) ── Motor #2 (J2)
    │
    └── Driver #3 (MADLN05BE) ── Motor #3 (J3)
```

- **Topology:** Daisy-chain (series connection)
- **Cycle time:** 0.5 – 4 ms (depends on NX1P2 task configuration)
- **Data exchanged per cycle:**
  - PLC → Driver: **Target Position** (CSP mode), Control Word
  - Driver → PLC: **Actual Position**, **Actual Velocity**, Status Word

### 5.2. PC ↔ PLC (TBD)

| Option | Protocol | Advantages | Disadvantages |
|--------|----------|-----------|---------------|
| **A** | EtherNet/IP (CIP) | Native on NX1P2, Python library available (cpppo) | CIP overhead, requires understanding of CIP structures |
| **B** | TCP/UDP Socket | Simple, easy to debug, custom protocol | Must handle packet framing/parsing manually |
| **C** | EtherCAT (PC as master) | High throughput, deterministic | Complex, conflicts with NX1P2 being master |
| **D** | FINS/TCP (Omron proprietary) | Native Omron, direct memory read/write | Speed limitations, best suited for single read/write |

> **Assessment:** Options **A** or **B** are the most feasible for the current phase. Option C requires careful investigation since NX1P2 is already the EtherCAT master.

---

## 6. Hardware Summary

| # | Device | Model | Quantity | Interface |
|---|--------|-------|----------|-----------|
| 1 | Machine Controller | Omron NX1P2-1140DT | 1 | EtherNet/IP (PC), EtherCAT (Drives) |
| 2 | Servo Driver | Panasonic MADLN05BE | 3 | EtherCAT slave |
| 3 | Servo Motor | Panasonic MSMF012L1T2 | 3 | Direct connection to driver |

### Changes from the Brainstorm Version (doc(removed))

| Item | Previous | Current | Reason |
|------|----------|---------|--------|
| PLC | Siemens S7-1200 | **Omron NX1P2-1140DT** | NX1P2 has built-in EtherCAT master, far superior motion control |
| PLC-Drive communication | Modbus TCP / Analog | **EtherCAT** | Synchronous, real-time, low jitter |
| Driver | Not yet determined | **Panasonic MADLN05BE** | Native EtherCAT, compatible with NX1P2 |
| Motor | Not yet determined | **Panasonic MSMF012L1T2** | AC servo, 23-bit encoder, power suitable for small delta |
| Adaptive conveyor | Yes (DC Motor + PID) | **TBD** | Not yet decided whether to use adaptive conveyor |

---

## 7. Open Issues

| # | Issue | Status | Notes |
|---|-------|--------|-------|
| 1 | Driver 50W vs Motor 100W compatibility | **Needs verification** | Check rated current |
| 2 | PC ↔ PLC communication method | **TBD** | EtherNet/IP or TCP Socket |
| 3 | Packet structure PC → PLC | **Not yet designed** | Need to define trajectory format |
| 4 | Adaptive conveyor mechanism | **TBD** | Not yet decided |
| 5 | Where to run Inverse Kinematics | **TBD** | PC (Python) or PLC (ST) |

---

*This file describes the confirmed hardware configuration — will be updated when changes occur.*
