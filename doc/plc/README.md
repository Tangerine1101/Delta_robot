# PLC Programs — Index

> **Scope.** What runs on the two PLCs of the cell, how the PC talks to them, and what is
> known to be wrong in the PLC code.
>
> **Which file describes which Omron program.**
> - [`program-old.md`](program-old.md) describes the **deployed** program.
> - `program-main.md` and `motion-fbs.md` describe the **target** program.
>
> **Defects.** Defects and proposed patches live in
> [`version-diff-and-defects.md`](version-diff-and-defects.md). Every defect that is still
> open is also registered in [`../open-issues.md`](../open-issues.md) §C.1 (IDs `P*`, `O*`).

## 1. Which program is which

| PLC | Program | Source in repo | Status |
|---|---|---|---|
| Omron NX1P2 (arm, suction, belt servo) | `Matching_Code_10` ("matching code") + `Section_Conveyor` | `OMRON/matching code/` (`.smc2` + ST/ladder exports; the belt section is not in the exported project) | **Deployed**: runs on the cell with the current Python ([`program-old.md`](program-old.md); belt contract in [`data-contract.md`](data-contract.md) §1) |
| Omron NX1P2 | `delta_paper_0_2` | `OMRON/delta_paper_0_2/` | **Target** (phase MID): not yet validated on hardware; needs the patches in `version-diff-and-defects.md` §4 |
| Siemens S7-1200 (4th-DOF rotation) | unchanged | not in repo (TIA Portal) | DB contract in [`data-contract.md`](data-contract.md) §3 |

The authoritative source is each `.smc2` project, which is a zip of XML: variable
declarations, ST bodies, and ladder rungs as JSON. The `.md` files and screenshots under
`OMRON/*/` are partial raw exports with comments in Vietnamese. The files in this folder are
the English reading, cross-checked against the project XML.

## 2. Files in this folder

| File | Contents |
|---|---|
| [`program-old.md`](program-old.md) | `Program0` of the deployed `Matching_Code_10`, rung by rung, and its `MC_Inter_Curve_Vel` differences |
| [`program-main.md`](program-main.md) | `Program0` of `delta_paper_0_2`, rung by rung |
| [`config-review.md`](config-review.md) | `config.yaml` re-derived from the PLC: reach per height, parameter table, time model vs PLC |
| [`motion-fbs.md`](motion-fbs.md) | `FB_ICV_Sequencer`, `MC_Inter_Curve_Vel` (profile maths + state machine), `Goto_Absolute`, `MC_Home_Delta`, `FB_TeachingMode` |
| [`kinematics.md`](kinematics.md) | On-PLC inverse / forward kinematics |
| [`data-contract.md`](data-contract.md) | PC ↔ Omron tags, PC ↔ Siemens DBs, command IDs, handshake, what Python must change for the target program |
| [`version-diff-and-defects.md`](version-diff-and-defects.md) | Old → new differences, full defect review of `delta_paper_0_2`, proposed ST patches |
| [`simulator.md`](simulator.md) | The PLC simulator node `modules/plc_sim`: what it models, how to run it, what it has shown |

## 3. Omron project at a glance

**Sysmac project tree** (`delta_paper_0_2`):

```
Programming
├── Programs
│   └── Program0 / Section0          ← ladder + inline ST (program-main.md)
├── Functions
│   ├── Calc_Inverse_Kinematics, Calc_Angles_YZ, Calc_Forward_Kinematic   (kinematics.md)
└── Function Blocks
    ├── Goto_Absolute, MC_Home_Delta
    ├── MC_Inter_Curve_Vel           ← one-segment interpolator
    ├── FB_ICV_Sequencer             ← N-point trajectory runner (new)
    └── FB_TeachingMode              ← hand-guided teaching (new)
Tasks: PrimaryTask → Program0
```

**Task and cycle.** `Program0` runs in the Primary periodic task. The interpolator advances
its time variable by a hard-coded `0.004` s per scan, so the task period **must** be 4 ms,
equal to the EtherCAT PDO cycle. Any other task period silently rescales every trajectory
in time.

**EtherCAT.** Three Panasonic MINAS A6 drives `MADLN05BE`: Node 72 (E001), Node 47 (E002),
Node 73 (E003). DC sync enabled, CSP mode (only Controlword `6040h` / Target position `607Ah`
out, Statusword `6041h` / Position actual `6064h` in).

**Axis settings** (`MC_Axis1..3`, MC1 = Primary task). Unit = degree; 8 388 608 command
pulses per motor revolution; 36° of work travel per motor revolution (i.e. the 10:1 reducer is
folded into the unit scale, no gearbox setting). No limit/home switch is mapped at axis
level — the three home switches are plain built-in inputs `Home_Axis1..3` used by ladder.

**Axis angle convention.** `pos_angular` and every `Theta` are output-shaft degrees. Axis 3's
angle is negated by both IK and FK (see [`kinematics.md`](kinematics.md) §3), so its sign is
opposite to axes 1 and 2.

## 4. How to keep this folder correct

* If the PLC program changes, update the file describing the changed POU **and** re-check
  [`data-contract.md`](data-contract.md) against `modules/comm/packets.py` and the gateways in `modules/comm/`.
* A defect fixed on the PLC: delete its row from `version-diff-and-defects.md` §3 and from
  `open-issues.md`, and record the fix in `decision-log.md`.
