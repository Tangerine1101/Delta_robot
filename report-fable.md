# Pick inaccuracy investigation — trajectory generation and Cartesian control, repo vs `modules/`

**Scope.** Compare the trajectory-generation and Cartesian kinetic-control layer of the
simulation sandbox (`kinetic_control.py`, `scheduler.py`) against the version running on
the physical cell (`modules/scheduler.py`, PLC function block documented in
`modules/doc/PLC_Program_description/MC_inter_curve_vel.md`), to explain why the real
cell picks off-center, especially at belt speeds above 120 mm/s.

**Context caveats.** `modules/config.json` was deliberately tuned to make the demo run
at 20–100 mm/s, so several of its numbers are physically wrong on purpose. Findings
below separate structural code differences from config artifacts.

## Summary

The two codebases share the same 7-waypoint trajectory template and the same velocity
mathematics (S-curve for stop-and-go, trapezoid with corner blending otherwise). The
geometry and profile formulas are essentially identical. The differences that matter
are in **how the descent onto the part is compensated** and in **look-ahead depth**,
and together they explain the observed speed-dependent pick offset:

1. The live cell's pick gate does **not** compensate for the pre_pick → pickup descent
   time, because that compensation was delegated to the oblique descent — which is
   **disabled** in `config.json`. The resulting miss is proportional to belt speed.
2. The PLC interpolator (and its Python port) has **no backward stop-distance pass**,
   so the arm enters the short final descent segments faster than it can stop within
   them. This adds a speed-independent baseline error at the park and place points.
3. `intercept_lead_time_s` never binds, so no timing margin absorbs either error.

## Finding 1 — uncompensated descent: miss grows linearly with belt speed (primary)

How each side budgets the dead time between "fire the grab" and "cup touches the part":

| | gate / contact lead covers | descent handling |
|---|---|---|
| repo | `command_delay + soft_start + descent` (`Scheduler.contact_delay_s`, `scheduler.py:226-259`) | S-curve time of the vertical drop, paid inside the lead. Docstring: *"forgetting the ~0.31 s descent alone costs 31 mm."* |
| modules | `command_delay (0.186 s) + gate sampling (~0.03 s)` only (`_object_pick_gate_status`, `modules/scheduler.py:1563-1588`) | deliberately **excluded** from the lead; the oblique descent (`_contact_position`) is supposed to shift the contact downstream by $v \cdot t_d$ and slant the drop to track the part |

But the live config sets `oblique_descent_enabled: false`. With the flag off,
`_contact_position` returns a straight-down contact at the predicted pick point and the
gate lead still omits $t_d$ — so between gate fire and actual contact the part travels
an uncompensated

$$\Delta u = v_{belt} \cdot t_{descent}$$

For the configured drop (pre_pick $-287$ → pickup $-303$, 16 mm), the PLC model gives
$t_d \approx 0.39$ s (`_descent_time_s`: 0.08 s soft start + 0.31 s S-curve). Even if
the PLC really bridges Pos[0] within the 80 ms soft start alone, the offset is:

| belt speed | miss at $t_d = 0.08$ s | miss at $t_d = 0.39$ s |
|---|---|---|
| 60 mm/s | ~5 mm | ~23 mm |
| 100 mm/s | ~8 mm | ~39 mm |
| 120 mm/s | ~10 mm | ~47 mm |
| 150 mm/s | ~12 mm | ~59 mm |

Because the error is linear in $v$, any config-level correction (frame calibration
shift, inflated delays) is exact at one speed only. Tuning the demo around 20–100 mm/s
absorbed a fixed-millimetre offset; past 120 mm/s the residual grows past the suction
cup's capture radius. This matches the reported symptom precisely.

A related internal inconsistency in `modules/`: `_predict_pick_position`
(`modules/scheduler.py:2284-2291`) charges the descent only `interp_soft_start_s`
(0.08 s), while `_descent_time_s` (`modules/scheduler.py:1375-1392`) models the same
move as ~0.39 s. Two contradictory descent models coexist, so even re-enabling the
oblique descent would apply a $v \cdot t_d$ correction with an unverified $t_d$.

## Finding 2 — no backward stop-distance pass in the PLC interpolator (baseline error)

The repo's `segment_schedule` (`kinetic_control.py:277-314`) runs **two passes**: a
backward pass caps every corner velocity at
$\sqrt{v_{next}^2 + 2\,d_{max}\,L_{next}}$ so a stop still fits in the remaining
runway, then a forward pass caps at what the arm can accelerate to. Its docstring
records why: a forward-only pass enters the final 5 mm descent at 291 mm/s, and
stopping from there needs 42 mm.

The real Omron function block `MC_Inter_Curve_Vel` — and its faithful Python port
`_trajectory_total_time` (`modules/scheduler.py:2578-2640`) — is **forward-only**:
$V_{end} = \min(V_{corner},\, V_{reach},\, V_{max})$, with no check that the following
segments can absorb the deceleration.

With the current geometry, the corner at P6 admits entry into the final 12 mm descent
(slope_transition $-275$ → pre_pick $-287$) at up to ~260 mm/s, while stopping from
that speed needs $v^2 / (2\,d_{max}) \approx 34$ mm. The commanded profile is
dynamically infeasible at the end of the goto: the physical arm either overshoots
below pre_pick (risking contact with the belt or part) or lags the setpoint at the
park point. A wrong park position feeds directly into the subsequent pick. This error
does not scale with belt speed, but it raises the floor under every pick and makes
config-level tuning chase a moving target. The simulation measured the same shape of
bug when it was fixed there: +0.16 s per cycle, invisible to the sim's pick-error
metric only because model and plant shared it (`doc/known-issues.md` §7).

## Secondary factors

* **No timing margin.** `intercept_lead_time_s = 0.8` never takes effect; the solver
  converges to $t_{pick} = t_{arrive} + t_{contact}$ (`doc/known-issues.md` §2), so
  every model error lands the cup *behind* the part with nothing absorbing it.
* **`argument_time` is crude but claimed ignored.** The packet times come from
  nominal speeds (`_segment_duration`, 220/220 mm/s, 0.08 s floor); comments state the
  Omron ignores `argument_time` and runs at fixed $V_{max}=300$, $A=D=1000$. Harmless
  if true — worth one on-machine verification.
* **Adaptive belt speed while a pick is in flight.** The sandbox measured mean pick
  error going from 0.44 mm to 13 mm when the speed law moves the setpoint mid-cycle
  (`doc/known-issues.md` §3). `modules/` suppresses commits only inside the ~2 s
  `gate_critical` window; `adaptive_speed_enabled` is on in the live config.

## Suggested verification (no changes applied)

1. Run `test_accuracy` (static fake objects, suction off) at 40 / 80 / 120 / 160 mm/s
   and plot pick offset against belt speed. A straight line through the origin
   confirms Finding 1, and its slope **is** the real $t_{descent}$ — measured, not
   guessed between 0.08 s and 0.39 s.
2. Then either re-enable `oblique_descent_enabled` with the measured $t_d$, or add
   $v \cdot t_d$ to the gate lead — one or the other, never both (the code comments
   already warn the combination double-counts the part's travel).
3. For Finding 2: temporarily lower the corner velocity at P6 (or lengthen the final
   descent segment) on the real cell and check whether the baseline offset shrinks.
   The root fix is a backward stop-distance pass in the PLC function block, mirroring
   the repo's `segment_schedule`.
