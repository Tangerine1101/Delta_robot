# PLC Simulator Node — `modules/plc_sim`

One in-process simulator impersonates **both** PLCs at the protocol level the real gateways
use. The two-thread `production` loop, `RealtimePickExecutor`, the IPC worker,
`PLCGateway`/`MockPLC` and `SiemensGateway` all run **unmodified** against it. The simulator
models the deployed Omron program `Matching_Code_10` ([`program-old.md`](program-old.md)).

## 1. Running it

| Command | What runs |
|---|---|
| `python3 main.py --scheduler --scenario production --sim --duration 60` | The real production loop, the simulator, simulated boards and camera; prints a `[SIM]` summary at the end |
| `python3 main.py --scheduler --scenario production --sim --interface` | Same, with the web dashboard. The MJPEG slot shows a top view of the simulated belt |
| `python3 main.py --cli --dummy` | Interactive CLI against the simulator |
| `python3 -m modules.plc_sim --feed 2.5` | Standalone simulator server on the config port; connect with `--ip 127.0.0.1` |
| `python3 -m unittest tests.test_plc_sim` | Unit tests pinning the port to the ST and to the documented defects |

**`--sim` options.**

| Option | Meaning | Default |
|---|---|---|
| `--sim-feed` | Seconds between boards | 2.5 |
| `--sim-servo-tau` | First-order servo lag per axis, in seconds | 0, ideal tracking; the real value is not measured (C5) |
| `--sim-tag-latency` | Round trip per Omron tag request, in seconds | 0.002; not measured (C2) |

The last two options also apply to `--dummy`.

## 2. Structure

```
main.py ── IPC worker ── MockPLC / SiemensGateway (mock) ──JSON-lines TCP──► PLCSim.handle
                                                                              │ 4 ms scan thread
  production loop ◄── SimCamera.poll ◄── World (boards, feeder, scoring) ◄───┤
                                                                              ├─ OmronPLC   (omron_core.py)
                                                                              ├─ ServoAxis ×3 (plant.py)
                                                                              └─ SiemensPLC (siemens_core.py)
```

| File | Content |
|---|---|
| `omron_core.py` | Scan-by-scan port of `Program0` rungs 1–21, the six chained `MC_Inter_Curve_Vel` instances, IK/FK, `Goto_Absolute`, homing and the pump state machine. Also the per-instance `MC_SyncMoveAbsolute` hand-over by multi-execution. `REAL` fields are rounded to float32 as on the PLC. |
| `plant.py` | Setpoint → `Act.Pos` servo model (first-order lag). |
| `siemens_core.py` | Commands 7/8/9. Belt ramps at `belt_accel_mm_s2`. `conveyor_position` in **mm**. Rotation slews at 180 °/s. |
| `world.py` | Boards on the belt, the feeder, grip/place scoring. |
| `sim.py` | `PLCSim`: the real-time scan loop, the protocol server, `SimCamera` (detections stamped with the capture time) and a JPEG top view. |

## 3. Fidelity

| Aspect | How it is modelled |
|---|---|
| Omron command dispatch, `task_state`, pump, per-segment profile, State-10 ramp, junction stalls | **Exact** port of the ST. Segment times are pinned by tests to hand-computed values. |
| Program defects O1–O5, O9, P2, P5, P7, P11, P12 | **Present**, as on the PLC. |
| Proposed PLC fixes | Switchable through `Patches`: O1 limit-as-error, O2 reject-while-busy, P2 recovery. |
| Two run-time behaviours the project file cannot settle | Parameters in `Quirks`: the value of an unwritten function output, and whether `TP` reads `PT` live. |
| Servo following | First-order lag, default ideal. |
| `MC_Stop` / home switches | Axes frozen while a switch is active; no deceleration profile. |
| Homing | Compressed to its timing (search, calibration move, 5 s window). |
| Siemens program | Only its PC-visible effects. Command-drop behaviour (L5) and `task_state` (L3) are unknown. |
| Vision | Perfect detections: 90 ms latency, 30 fps, no noise, no misses. |
| Suction | A board is gripped if the cup comes down with the pump on within 12.7 mm of its centre. It is placed if released within 30 mm of its bin. |
| Clock | Wall-clock real time; the scan thread counts overruns. |

## 4. First production run

`production --sim --duration 60`, with the demo config as committed (adaptive belt,
45–80 mm/s) and a board every 2.5 s:

| Measure | Value |
|---|---|
| Boards fed / gripped / placed / lost | 25 / 17 / 16 / 5 |
| Along-belt error at contact (median) | −0.22 mm |
| `[GATE]` gate → dispatch | ≈ 0.10 s (37 tag writes × 2 ms plus the poll) |
| `[GATE]` dispatch → contact | ≈ 0.10 s |
| `[GATE]` `t_d_model_s` (the scheduler's descent-time model) | 0.39–0.48 s, against the ≈ 0.08 s the PLC commands (P11, C3) |
| PLC defect events | none: the belt stayed below 100 mm/s, so the arrival band stayed narrow (O2) |
| Scan overruns | 0 |

The simulator is therefore the offline A/B testbed for:
- the new scheduler and speed governor;
- the O2 mitigation on the PC;
- a run at ≥ 100 mm/s to observe O2 in the loop itself.
