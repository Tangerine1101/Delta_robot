# System Configuration — Delta Robot Project

> **Last updated:** 2026-05-18
> **Phase:** Phase 1 motion execution + offline scheduler simulation
> **Status:** Hardware confirmed, CLI mode implemented, scheduler simulator implemented

---

## 1. Overall Hardware Diagram

```
┌──────────────────────────────────────────────────────────────────┐
│                       HARDWARE SYSTEM                            │
│                                                                  │
│  ┌──────────┐    Ethernet / EtherNet/IP / TCP (TBD)             │
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

### Current software-facing role

- Receive robot command data from the PC through a fixed PLC struct
- Expose status back to the PC through a second PLC struct
- Execute motion commands through EtherCAT
- Later provide conveyor-speed data to the scheduler through `EthernetCom`

### Current PC-side PLC tags used by the repository

- PC writes: `pc_package`
- PC reads: `plc_package`

Current status probe fields:

- `plc_package.task_doing`
- `plc_package.task_state`

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
| **Quantity** | **3 units** |

### Relevant EtherCAT mode

| Mode (CiA 402) | Description | Suitability |
|-----------------|-------------|-------------|
| **CSP (Cyclic Synchronous Position)** | PLC sends new position every cycle | **Preferred** |
| CSV (Cyclic Synchronous Velocity) | PLC sends new velocity every cycle | Backup option |
| PP (Profile Position) | Driver generates its own profile | Not suitable for this project |
| HM (Homing) | Return to home position | Required for startup |

---

## 4. Servo Motor — Panasonic MSMF012L1T2

### Key Specifications

| Parameter | Value |
|-----------|-------|
| **Model** | MSMF012L1T2 |
| **Product line** | MINAS A6 Family |
| **Rated power** | 100 W |
| **Rated torque** | 0.32 N·m |
| **Peak torque** | 0.96 N·m |
| **Rated speed** | 3000 rpm |
| **Maximum speed** | 5000 rpm |
| **Encoder** | 23-bit absolute |
| **Voltage** | 200 V |
| **Quantity** | **3 units** |

### Warning

- Driver rated power (50 W) < motor rated power (100 W)
- Compatibility still needs verification on current/torque limits

---

## 5. Connectivity and Communication

### 5.1. EtherCAT Bus (PLC ↔ Drivers)

```
NX1P2 (Master)
    │
    │  EtherCAT (daisy-chain)
    │
    ├── Driver #1 (MADLN05BE) ── Motor #1
    ├── Driver #2 (MADLN05BE) ── Motor #2
    └── Driver #3 (MADLN05BE) ── Motor #3
```

- **Topology:** daisy-chain
- **Cycle time:** 0.5 – 4 ms depending on NX1P2 task configuration
- **Data exchanged per cycle:**
  - PLC → Driver: target position, control word
  - Driver → PLC: actual position, actual velocity, status word

### 5.2. PC ↔ PLC

| Link | Current code assumption | Status |
|------|-------------------------|--------|
| PC → PLC robot command | Python writes members of `pc_package` field-by-field | Implemented in code |
| PLC → PC robot status | Python reads `plc_package.task_doing` and `plc_package.task_state` | Implemented in code |
| PLC → PC conveyor speed | Future extension through `EthernetCom` | Planned |

### 5.3. Current PC command struct

The Python code currently normalizes and writes this fixed package:

```python
{
    "commandID": int,
    "argument_number": int,
    "argument_x": [float] * 6,
    "argument_y": [float] * 6,
    "argument_z": [float] * 6,
    "argument_e": [float] * 6,
    "argument_time": [float] * 6,
}
```

Important rules:

- fixed array length is now `6`
- `argument_number` is the number of active points in the current command
- unused slots must still be sent as `0.0`
- `argument_e` is used to control end-effector state along the trajectory

### 5.4. Current command IDs

| Command name | ID |
|--------------|----|
| `stop` | 0 |
| `goto_relative` | 1 |
| `goto_absolute` | 2 |
| `go_trajectory` | 3 |
| `calibrate` | 4 |
| `pick` | 5 |
| `release` | 6 |

---

## 6. Runtime Software Configuration

Source of truth:

- `modules/config.json`

### 6.1. Top-level config variables

| Key | Current value | Meaning |
|-----|---------------|---------|
| `ip_address` | `192.168.250.1` | Default PLC IP address |
| `port` | `502` | Default PLC port |
| `period_s` | `0.1` | Reserved timing/config period |
| `interpolar_points` | `6` | Fixed PLC trajectory slot count |

### 6.2. Object-type routing config

| Key | Current value | Meaning |
|-----|---------------|---------|
| `object_types` | `{"object_A": "object_A"}` | Maps detected object type to destination config key |
| `object_A` | `[140.0, -120.0, -205.0]` | Sorting destination for `object_A` |

### 6.3. Scheduler config variables

| Key | Current value | Meaning |
|-----|---------------|---------|
| `scheduler.home_position` | `[0.0, 0.0, -180.0]` | Robot rest/start position |
| `scheduler.clearance_height` | `-165.0` | Safe travel Z during horizontal motion |
| `scheduler.pickup_height` | `-230.0` | Pickup Z at conveyor |
| `scheduler.place_height` | `-205.0` | Placement Z at sorting area |
| `scheduler.corner_blend_xy` | `35.0` | XY blend amount for the smoothed-square path |
| `scheduler.intercept_lead_time_s` | `0.14` | Early-arrival margin above predicted pickup point |
| `scheduler.pickup_descent_time_s` | `0.14` | Final descent time during pickup |
| `scheduler.release_descent_time_s` | `0.14` | Final descent time during release |
| `scheduler.nominal_xy_speed` | `220.0` | XY timing model for segment durations |
| `scheduler.nominal_z_speed` | `180.0` | Z timing model for segment durations |
| `scheduler.stale_timeout_s` | `5.0` | Drop detections older than this timeout |
| `scheduler.speed_timeout_s` | `1.0` | Reject planning if speed sample is too old |
| `scheduler.poll_interval_s` | `0.05` | Scheduler loop polling interval |
| `scheduler.default_speed` | `80.0` | Default simulated conveyor speed |
| `scheduler.pickup_window_x` | `[-120.0, 120.0]` | Allowed pickup X workspace |
| `scheduler.pickup_window_y` | `[-120.0, 120.0]` | Allowed pickup Y workspace |
| `scheduler.throughput_object_types` | `["object_A"]` | Types emitted in throughput simulation |
| `scheduler.throughput_lanes` | `[-60.0, 0.0, 60.0]` | Simulated conveyor lanes |
| `scheduler.throughput_spawn_x` | `-180.0` | Spawn X for throughput objects |
| `scheduler.throughput_emit_interval_s` | `0.35` | Spawn interval in throughput scenario |
| `scheduler.accuracy_emit_interval_s` | `0.8` | Spawn interval in accuracy scenario |
| `scheduler.accuracy_points` | `[[40.0, -60.0, -220.0], [0.0, 0.0, -220.0], [-40.0, 60.0, -220.0]]` | Fixed target points for accuracy scenario |
| `scheduler.log_path` | `data.log` | Output log file for accuracy traces |

### 6.4. Current software modes

| Mode | Entry | Purpose |
|------|-------|---------|
| CLI mode | `python3 main.py --cli` | Manual PLC command testing |
| Scheduler mode | `python3 main.py --scheduler --scenario ...` | Offline planning/benchmark simulation |

Current scheduler scenarios:

- `test_throughput`
- `test_accuracy`

---

## 7. Hardware Summary

| # | Device | Model | Quantity | Interface |
|---|--------|-------|----------|-----------|
| 1 | Machine Controller | Omron NX1P2-1140DT | 1 | PC link + EtherCAT master |
| 2 | Servo Driver | Panasonic MADLN05BE | 3 | EtherCAT slave |
| 3 | Servo Motor | Panasonic MSMF012L1T2 | 3 | Direct connection to driver |

---

## 8. Open Issues

| # | Issue | Status | Notes |
|---|-------|--------|-------|
| 1 | Driver 50W vs Motor 100W compatibility | **Needs verification** | Check rated current and torque limits |
| 2 | Real PLC transport details | **TBD** | Current code assumes PLC tag reads/writes through `pylogix` |
| 3 | Real conveyor-speed source integration | **Planned** | Scheduler still uses simulated speed source |
| 4 | Real image-processing integration | **Planned** | Scheduler still uses simulated detections |
| 5 | PLC-side struct synchronization for 6 points | **Must be kept aligned** | Python now assumes fixed arrays of length 6 |

---

*This file now describes both the confirmed hardware and the current software/runtime configuration used by the repository.*
