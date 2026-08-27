# Open Issues — Delta Robot

> **This file is the single register of everything still unresolved in this repository.**
> If a problem is real and not yet fixed, it is here — not scattered through prose in other
> documents. If it is not here, it is either done or it never existed.
>
> **AI assistants may read *and* update this file.** When you close an item, delete the row
> and record the resolution in [`decision-log.md`](decision-log.md); do not leave `[FIXED]`
> entries behind — that is exactly the changelog-rot this file replaces.
> Historical/superseded material belongs in `decision-log.md`, never here.

**Roadmap tags** (the project's declared phases):

| Tag | Phase |
|---|---|
| **NEAR** | Redesign of the pick scheduler and the conveyor speed controller |
| **MID** | Rework of the packet layout and the PLC ↔ IPC communication method |
| **FAR** | Vision-model fine-tuning, configuration slimming |
| **DOC** | Documentation / repository hygiene |

Line references point at the file as of the last audit (2026-08-10); treat them as hints,
not addresses.

---

## A. Pending hardware calibration

| ID | Issue | Where | Action to close | Tag |
|---|---|---|---|---|
| **C1** | `rotate_sign = 1` has never been verified against the physical axis. If wrong, every rotation turns the wrong way. | `config.json` › `scheduler.rotate_sign` | `python3 -m modules.test_rotate` — a commanded +90° must turn the cup CCW seen from above; otherwise set `-1`. | — |
| **C2** | `robot_movement_delay_s = 0.17` and `ethernet_delay_s = 0.016` are estimates. Their sum *is* the pick gate's lead offset (`basis-theory.md` §4.4), so an error here lands every pick off-centre by `v_belt · Δ`. | `config.json` › `scheduler.*` | Read `dispatch_to_contact_s` from the per-pick `[GATE]` log; measure the wire latency with `python3 -m modules.latency_probe --target siemens`. | NEAR |
| **C3** | `oblique_descent_enabled = false`. The belt-tracking slanted descent is implemented but disabled because the descent-time model over-estimates $t_d$ while `interpolator.a_max` is uncalibrated (see C5). | `config.json` › `scheduler.oblique_descent_enabled` | Close C5 first, then re-test on hardware. | NEAR |
| **C4** | `rotate_offset_deg = -62.0` and `vision.orientation.offset_by_class` (both `0.0`) are unconfirmed; they must be re-checked after any change to the suction-cup marker or mounting. | `config.json` | Hardware run reading the per-pick `[ROTATE]` log (`vision_angle / board_heading / rotate_cmd / rotate_at_gate / rotate_at_end`). | — |
| **C5** | **`interpolator.{v_max, a_max, d_max}` have never been validated against the real arm.** This block is the trajectory-time model — i.e. the scheduler's estimate of processing time per job. Every pick-time prediction, gate lead, descent-time model and feasibility test is built on it. | `config.json` › `scheduler.interpolator`, `basis-theory.md` §3 | `speed_tuning` in the CLI runs a tilted heptagon and reports measured-vs-modelled ratio plus a first-order `v_max` suggestion (report-only, never writes config). Then apply by hand. | **NEAR — blocking** |
| **C6** | The physical-calibration stage of the config checker is a documented stub: `F_CONVEYOR_TO_ROBOT` (θ, T), `robot_movement_delay_s`, and `conveyor_position_scale_mm` have no automated procedure. | `calibrate_everything.py` › `_todo_physical_calibration()` | Implement, driven by the existing `test_vision_only` / `evaluate` scenarios. | FAR |

---

## B. Configuration inconsistencies

| ID | Issue | Where | Tag |
|---|---|---|---|
| **G1** | `belt_speed_static_mm_s = 120` is **above** the adaptive ceiling `belt_speed_max_mm_s = 100`. Startup seeds the belt outside the band the controller then regulates within, so the first adaptive commit always steps the belt down. Decide whether the static seed is meant to be exempt from the soft cap or whether one of the two values is wrong. | `config.json` › `scheduler` | NEAR |
| **G2** | `pick_cycle_s = 3.0` implies ~20 picks/min, while `doc/proposal/abstract-en.tex` claims a nominal **30–60 picks/min**. One of the two is wrong, and the discrepancy will be visible to reviewers. Note `pick_cycle_s` is the *calibrated arm cycle* $t_\text{pick}$ feeding $\mu_\max = 1/t_\text{pick}$, so this also shifts the whole rate target $\lambda_\text{nom}$. | `config.json` vs the abstract | **NEAR — publication-blocking** |
| **G3** | `belt_density_length_mm = 0.0` makes the density region derive to $L_\text{meas} = u_\max = 363$ mm, which starts upstream of the workspace ($u_\min = 188$). Density is therefore measured over a longer region than the one being regulated. This may be intentional (the camera previews parts before they arrive) but it is undocumented and it changes the meaning of $\rho$. | `config.json`, `basis-theory.md` §6.1–6.2 | NEAR |
| **G4** | `rotate_home_tolerance_deg = 0.0` keeps the "axis not yet home at grip" check strict, whereas the documented intent of a positive value is a warn-only degradation. Confirm which behaviour production wants. | `config.json`, `basis-theory.md` §5.2 | — |
| **G5** | `default_speed = [30.0, 0.0]` is a leftover 2-D velocity vector from an older frame convention: only its magnitude is used, and only in simulation. | `config.json` › `scheduler.default_speed` | FAR |
| **G6** | `pick_arrival_tolerance_mm / _max_mm` were widened to 15/50 mm to make picks land; that is a symptom treatment. The suspected root causes (C2 latency calibration, C3 vertical-vs-oblique descent) are still open, so the band cannot be tightened back yet. | `config.json`, `basis-theory.md` §4.4 | NEAR |

---

## C. Architectural and algorithmic limitations

These are real properties of the system as built. They are **not** bugs to be quietly
deleted from the docs — several of them constrain what the new scheduling algorithm can
assume.

### C.1 PLC / firmware constraints

| ID | Limitation | Consequence | Tag |
|---|---|---|---|
| **L1** | The Omron firmware **ignores `argument_time`**; the arm always runs at the interpolator's fixed limits. | The PC cannot modulate how fast a pick executes. Processing time $p_j$ is an *observable*, not a decision variable — any scheduling formulation that assumes controllable $p_j$ is invalid on this hardware. | MID |
| **L2** | `goto` and `pick` must be dispatched as two separate phases, because the PLC exposes no trajectory-time planner. | Pick timing is controlled by *when* the second phase is sent, not by the trajectory itself. This is what makes the positional gate necessary. | MID |
| **L3** | `task_state` (DB2 offset 12) reports inconsistent/legacy values; the real handshake is Omron's `bit_doing`. | Half of a status word is unusable. Candidate for removal in the packet rework. | MID |
| **L4** | `goto_relative` (command ID 1) is not implemented on the Omron program. | The ID is reserved but dead. | MID |
| **L5** | **Siemens DB1 carries no handshake bit.** It is unverified whether the ST program edge-triggers on a `CommandID` change; if it does, a `rotate_absolute` sent back-to-back with a `change_speed` can be silently dropped. | Silent command loss between the two most timing-sensitive Siemens commands — rotation and belt speed. Verify in TIA Portal and record the finding in `doc/PLC_Program_description/`. | **MID — highest risk** |

### C.2 Control / measurement gaps

| ID | Limitation | Consequence | Tag |
|---|---|---|---|
| **L6** | **No suction verification.** A pick is booked as successful when the arm's *motion* completes; the vacuum is never checked, and a dispatched grip is never retried. | The controller's own success count over-states reality. Any $\sum U_j$ reported from software is a lower bound on true losses unless picks are also counted by an independent means (bin count, downstream camera). | NEAR |
| **L9** | The accelerating-belt forms of the gate offset are derived but deliberately **not implemented**; the single-term $v \cdot T_\text{delay}$ is only correct while the belt is steady, which the current commit policy guarantees by suppressing commits in the gate-critical window. | A redesigned speed controller that commits more freely (or continuously) **invalidates this guarantee**. Either keep an equivalent suppression rule or implement the ramp-aware offset. | NEAR |
| **L10** | **The speed law is open-loop with respect to misses.** Belt speed is a function of instantaneous object density (and a spacing cap) only; the backlog / predicted-pass count is telemetry and never feeds back. | The outer loop described in `doc/proposal/abstract-en.tex` — predicting how many parts will miss their deadline and adjusting belt speed from that prediction — **does not exist in the code yet**. This is the central gap between the current implementation and the proposed research. | **NEAR — core** |
| **L11** | The spacing cap inspects only the leading `_SPACING_LEAD_OBJECTS = 4` objects. | A cluster further upstream is invisible to the speed law until it reaches the front. | NEAR |

### C.3 Experiment / reproducibility gaps

| ID | Limitation | Consequence | Tag |
|---|---|---|---|
| **L7** | **Two divergent execution paths.** `production` runs the two-thread realtime loop; `test_throughput`, `test_accuracy`, `test_acceptance` and `evaluate` all run the older single-threaded harness. | Simulated results do not exercise the code path being published. Any benchmark produced by the simulated scenarios measures a different program. | **NEAR — blocking for experiments** |
| **L8** | `production` cannot be dry-run: `--simulate-executor` is unsupported because the loop requires live `conveyor_position` feedback from the Siemens PLC. | There is no offline, repeatable way to A/B two scheduling policies. Every comparison currently requires the physical cell, with the feeder as an uncontrolled variable. A simulated belt-position source would unblock the paper's experiment section. | **NEAR — blocking for experiments** |

---

## D. Repository hygiene

| ID | Issue | Status |
|---|---|---|
| **D8** | `doc/proposal/` is untracked. LaTeX build artefacts are already ignored, so committing `abstract-en.tex` is safe — but it names the authors, and author names were deliberately purged from git history once already (see `decision-log.md`). Decide before committing. | **Open — user's call** |
| **D9** | The purged git history has still not been pushed to GitHub (no credentials available in the agent environment). Until it is, the remote — if any — carries the pre-purge history. | **Open — user action** |

---

## How to use this file

* **Before proposing work**, read §A–§C: several plausible-looking improvements are already
  blocked on a calibration or a hardware fact listed here.
* **Before trusting a number** in `modules/config.json`, check §A and §B — a value being
  present in the config does not mean it was measured.
* **Before designing the new scheduler**, read L1, L6, L7, L8, L9, L10 together: they define
  what the hardware permits, what can be measured, and what cannot currently be reproduced
  offline.
