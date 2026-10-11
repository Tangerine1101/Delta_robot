# Plant identification of the pick cell — method and results

> Vietnamese version: [`plant-identification.vi.md`](plant-identification.vi.md).
> Data: console session `log/20261010-163210_exp_PR_DP_1010_console` (2026-10-10).
> Tool: `python3 -m modules.tools.identify_plant`. Figures: `doc/figures/identification/`
> (`sandbox/identification_figures.py`). Open items stay in [`open-issues.md`](open-issues.md);
> the decision that followed is in [`decision-log.md`](decision-log.md).

## Summary

The scheduler and the sandbox both rest on a model of the cell: how long the arm takes for a
leg, how long a pick cycle is, how the belt follows a speed command. This report measures those
terms on the cell and checks the sandbox that uses them.

| Term | Measured | Where it is used |
|---|---|---|
| Goto leg, send → arrival, minus `trajectory_time` | +0.005 s (median, 3660 legs) | `sim.goto_extra_s` |
| Pick leg, same | −0.019 s (3547 legs) | `sim.pick_extra_s` |
| Legs shorter than 120 mm | up to +0.2 s slower than the model | open-issues C5 |
| Command send → first arm motion | 0.107 s | model's soft start |
| Pick send → lowest z | 0.221 s (sd 0.004), z = −298.10 mm | gate lead, open-issues T3 |
| Back-to-back pick cycle | 2.18 s = 0.95 + 0.18 + 0.98 + 0.07 | sandbox pick interval |
| Belt, settled speed / setpoint | 1.000 | — |
| Belt, step response | 10 / 50 / 90 % at 0.023 / 0.080 / 0.126 s after the send | open-issues L9 |
| Pick error of virtual parts | 0.020 s × v + 0.41 mm | open-issues C10 |

The sandbox used to add the model's command delay (0.228 s) to every leg, which made its arm
0.28 s per pick slower than the cell's. With the measured offsets in place of that delay, the
sandbox reproduces the picks of seven cell runs within 3.3 % and the belt speed of five of them
within 10 % (§4.6).

## 1. Purpose and scope

The sandbox (`sandbox/`) runs the robot's own planners and speed laws against a simulated arm
and belt. Its conclusions carry over to the cell only as far as that plant matches the cell. The
earlier calibration was one empirical factor (`sim.carry_scale = 0.8`) with no known cause.

Identified here, from logs of the real arm and belt:

1. the arm's time per leg against the trajectory model of `core/motion.py`;
2. the pick cycle and where its time goes;
3. the descent of the cup;
4. the belt servo's response to a speed command;
5. the along-belt pick error of the gate.

Not identified: anything that involves the camera or real parts (belt scale C8, capture latency
C9, suction L6). The parts of this session are virtual (`simulate_feeder`): they are generated,
placed on the real belt and picked by the real arm, so the camera path is absent.

## 2. Data

One console session, four runs of the `simulate_feeder` scenario, all with planner `dp`,
stack-aware, 1000 parts from a Poisson feeder at 24 parts/min, seed 1001:

| Run | Speed law | Parts picked | Duration (s) |
|---|---|---|---|
| `run01` W15 | `predictive_rank`, W = 15 s | 943 | 2565 |
| `run02` W25 | `predictive_rank`, W = 25 s | 958 | 2580 |
| `run05` W50 | `predictive_rank`, W = 50 s | 846 | 2615 |
| `run06` const 20 | `constant`, 20 mm/s | 907 | 2572 |

`run03` and `run04` (4 and 0 picks) are aborted starts and are excluded: a run needs at least
100 picks (`--min-picked`). The horizon W and the law of each run were checked against its
`summary.json` (`settings`), not against its folder name.

The session folder holds three streams shared by all its runs:

| File | Content | Rate |
|---|---|---|
| `pose.csv` | arm tip position from the PLC's UDP stream, stamped at PC receive time | ~12 ms |
| `commands.jsonl` | every PLC command with send time, completion time and the packet | per command |
| `status.csv` | belt `speed_current`, position, arm state | ~40 ms (20–25 Hz) |

and, per run, `parts.csv` (one row per part: outcome, contact error, belt speed at contact).
The streams are cut into runs by time: a run ends at the `scenario … ended` line of
`console.log` and starts `duration_s` earlier. All times are the PC's monotonic clock.

## 3. Method

### 3.1 The pick cycle and the plant model

Each pick is two trajectory commands (command 3) and a rotation (command 7):

```
goto (bin → above the part)  →  wait for the part  →  pick (descend, grip, rise, carry to bin)  →  next goto
```

A command-3 leg is identified by its last waypoint: z = −287 mm ends above a part (goto),
z = −280 mm ends over a bin (pick). The scheduler predicts a leg's duration with
`trajectory_time(points)`: a soft start plus the PLC's segment profile with corner blending
(`core/motion.py`). The sandbox's plant executes a leg in

$$T_\text{leg} = T_\text{traj} + e,$$

where the overhead $e$ is what this report identifies. Before it, $e$ was the model's command delay $d$ on both legs:

$$d = d_\text{robot} + d_\text{eth} = 0.182 + 0.046 = 0.228\ \text{s}$$

with $d_\text{robot}$ = `robot_movement_delay_s` and $d_\text{eth}$ = `ethernet_delay_s`. $d$ is the
scheduler's *lead* for the pick gate, so that the cup lands on the part; it is not time the arm
spends on the flight.

### 3.2 Arm legs

For every command 3, using the pose samples between its send and the next command 3:

* $t_\text{send}$ — `t_mono_send` of the command;
* $t_\text{move}$ — first sample more than 0.5 mm from the starting pose;
* $t_\text{arr}$ — first sample, after the arm has left, within 1.0 mm of the last waypoint;
* gap $= (t_\text{arr} - t_\text{send}) - T_\text{traj}$, with $T_\text{traj}$ from the packet's
  own waypoints and the session config's `robot.interpolator`.

The gap is summarised by its median and p5–p95 per leg kind, and as a function of the path
length (the sum of the segment lengths of the packet).

### 3.3 Pick cycle

For consecutive goto → pick → goto of one run: goto flight $= t_\text{arr} - t_\text{send}$ of the
goto; wait $=$ pick send $-$ goto arrival; pick flight $=$ pick arrival $-$ pick send; gap $=$ next
goto send $-$ pick arrival. A cycle with a wait below 0.35 s is *back-to-back*: the arm did not
wait for its part, so its length is the arm's own cycle.

### 3.4 Descent

For every pick leg: send → the first sample within 0.5 mm of the lowest z, and that z. The
commanded pickup height is −300 mm.

### 3.5 Belt servo

Each command 8 sets a belt speed. A step is measured when it is at least 4 mm/s, with no other
command 8 within 1.2 s before or after. The speed before is the median of `speed_current` in the
0.6 s before the send, the speed after the median from 0.8 s after it. The times at which the
feedback crosses 10, 50 and 90 % of the step are found by linear interpolation between samples,
counted from the send. Separately, in every stretch of at least 3 s between two commands, the
median settled `speed_current` (from 1.5 s after the command) is compared with the setpoint.

### 3.6 Pick error

`error_u_mm` of every picked part (the part's tracked position against the cup's at contact,
along the belt) is regressed on the belt speed at contact: $\text{error}_u = a\,v + b$. The slope
$a$ is a time offset.

### 3.7 Validation by replay

The identified plant is checked as a whole by replaying cell runs in the sandbox
(`experiments/phase0_replay.yaml`, `phase0_replay_1001.yaml`). A replay feeds the sandbox the
cell run's own arrivals (`arrivals.csv`) under the run's own recorded settings, then compares:

| Check | Gate |
|---|---|
| parts picked | within 5 % |
| mean belt speed | within 10 % |
| parts with the same outcome, matched by arrival order | at least 85 % |

Two plants are compared on the same input: *uncalibrated* ($e = d$ on both legs) and
*identified* ($e$ = the measured medians, `carry_scale` = 1). The set of seed 110 (three runs of
2026-10-07/08) was not used for the identification, so it checks the plant on unseen runs; the
four runs of this session are the data the legs were identified from, so they check the
identification end to end, not generalisation.

## 4. Results

### 4.1 Arm legs

![Arm legs against the model](figures/identification/fig1_legs.png)

*Figure 1. (a) Every leg's measured send → arrival against the model. (b) The gap by path length.*

| Leg | n | gap p5 (s) | gap median (s) | gap p95 (s) |
|---|---|---|---|---|
| goto | 3660 | −0.015 | **+0.005** | +0.034 |
| pick | 3547 | −0.034 | **−0.019** | −0.001 |

| Path length (mm) | goto n | goto gap median (s) | pick n | pick gap median (s) |
|---|---|---|---|---|
| < 120 | 153 | +0.159 | 0 | – |
| 120–160 | 975 | +0.012 | 428 | −0.024 |
| 160–200 | 1326 | −0.001 | 1237 | −0.016 |
| 200–240 | 962 | +0.009 | 1405 | −0.016 |
| 240–280 | 228 | −0.020 | 375 | −0.031 |
| 280–400 | 16 | −0.010 | 102 | −0.019 |

Between 120 and 400 mm the model is within ±0.03 s of the cell (ratio 0.97–1.01): the values of
`robot.interpolator` (v_max 1000, a_max 2250, d_max 1750 mm/s², soft start 0.08 s) reproduce its
times. Below 120 mm the gap grows as the path shortens, to +0.2 s at 30 mm; those are 4 % of
the gotos (open-issues C5). The command write takes 0.065 s (median) and the arm starts to move
0.107 s after the send, which the model's 0.08 s soft start covers.

### 4.2 Pick cycle

![Pick cycle](figures/identification/fig2_cycle.png)

*Figure 2. (a) The cell's back-to-back cycle. (b) Median pick interval of the replayed runs: cell, uncalibrated sandbox, identified sandbox.*

| Part of the cycle (s) | p5 | median | p95 | n |
|---|---|---|---|---|
| goto flight | 0.845 | 0.945 | 1.047 | 3516 |
| wait for the part | 0.108 | 0.194 | 2.513 | 3516 |
| pick flight | 0.880 | 0.972 | 1.074 | 3543 |
| pick arrival → next goto | 0.013 | 0.072 | 0.396 | 3543 |
| goto send → next goto send | 1.941 | 2.223 | 5.343 | 3650 |

| Run | cycles | back-to-back | cycle median (s) | back-to-back median (s) |
|---|---|---|---|---|
| W15 | 942 | 675 | 2.224 | 2.180 |
| W25 | 957 | 702 | 2.240 | 2.196 |
| W50 | 845 | 645 | 2.258 | 2.215 |
| const 20 | 906 | 666 | 2.182 | 2.141 |

The cell's arm cycle is 2.18 s: 0.95 s goto, 0.18 s of waiting at the gate, 0.98 s pick, 0.07 s
to the next goto. The uncalibrated sandbox picked every 2.43–2.49 s on the same inputs (cell
2.18–2.28 s), 0.28 s too slow; with the identified legs it picks every 2.19–2.21 s. The
uncalibrated plant added $2 \times 0.228 = 0.456$ s to the two legs. Only 0.28 s of it showed up
in the interval; the rest was most likely absorbed by the wait for the part, which was not
traced separately.

### 4.3 Descent

| Quantity | p5 | median | p95 | n |
|---|---|---|---|---|
| pick send → lowest z (s) | 0.214 | 0.221 | 0.227 | 3654 |
| lowest z (mm) | −298.157 | −298.101 | −298.062 | 3654 |

The cup reaches the same depth 0.221 s after the send every time (sd 0.004 s), 1.9 mm short of
the commanded −300 mm (open-issues T3). The 0.221 s agrees with the gate's lead,
$d = 0.228$ s.

### 4.4 Belt servo

![Belt servo](figures/identification/fig3_belt.png)

*Figure 3. (a) Mean step response, normalised, by step size. (b) Time to 50 % of the step.*

| Point of the step (from the send) | p5 (s) | median (s) | p95 (s) | n |
|---|---|---|---|---|
| 10 % | −0.011 | 0.023 | 0.051 | 1159 |
| 50 % | 0.053 | 0.080 | 0.091 | 1173 |
| 90 % | 0.113 | 0.126 | 0.140 | 1173 |

| \|step\| (mm/s) | n | 10 % (s) | 50 % (s) | 90 % (s) |
|---|---|---|---|---|
| 4–10 | 711 | 0.016 | 0.076 | 0.125 |
| 10–16 | 388 | 0.044 | 0.084 | 0.127 |
| 16–22 | 71 | 0.048 | 0.089 | 0.131 |

| Setpoint band (mm/s) | settled stretches | median speed − setpoint | median speed / setpoint |
|---|---|---|---|
| 0–15 | 203 | −0.009 | 0.9981 |
| 15–25 | 141 | +0.016 | 1.0008 |
| 25–40 | 189 | +0.007 | 1.0003 |
| 40–60 | 689 | +0.004 | 1.0001 |

The feedback settles on the setpoint. A step takes about 0.04 s to start and 0.1 s to finish,
whatever its size (4–22 mm/s): the transition is a fixed time, not a rate limit of 500 mm/s²,
which would take 0.04 s for 20 mm/s. The status is sampled every ~40 ms, so the shape within a
step is not resolved. What the feedback is *of* (the encoder, not the belt surface) is not
tested here (open-issues C8).

### 4.5 Pick error

![Pick error](figures/identification/fig4_pick_error.png)

*Figure 4. Along-belt error of 3654 picked virtual parts against the belt speed at contact.*

| Belt speed (mm/s) | picks | median error (mm) | sd (mm) |
|---|---|---|---|
| 0–15 | 518 | +0.28 | 0.83 |
| 15–25 | 1294 | +0.55 | 0.95 |
| 25–40 | 609 | +0.74 | 1.08 |
| 40–60 | 1233 | +1.09 | 1.60 |

The fit is error = 0.0199 s × v + 0.41 mm: the cup lands about 20 ms late and 0.4 mm off,
so the gate and arm timing explain at most 1.6 mm at 60 mm/s. The larger errors in the figure
(up to 11 mm) are single parts; their cause was not analysed here. On real
parts the late picks of 2026-10-02 were 7–15 mm, so they lie in the camera and belt-scale terms
(open-issues C8–C10), not in the gate.

### 4.6 Validation by replay

![Replay](figures/identification/fig5_replay.png)

*Figure 5. Cell against sandbox, picks and mean belt speed; the green labels are the identified plant's difference from the cell.*

| Replay (feeder seed / run) | Picked: cell | uncal. | identified | Belt (mm/s): cell | uncal. | identified | Parts with other outcome | Gates |
|---|---|---|---|---|---|---|---|---|
| 110 / W15 dp | 1316 | 1277 | 1315 | 43.3 | 36.6 | 43.2 | 2.4 % | pass |
| 110 / W15 KIM | 1312 | 1283 | 1315 | 43.2 | 36.1 | 43.2 | 2.4 % | pass |
| 110 / W45 KIM | 1227 | 1090 | 1219 | 30.2 | 12.7 | 24.2 | 8.6 % | belt fails |
| 1001 / W15 | 943 | 912 | 946 | 42.9 | 37.4 | 44.0 | 4.5 % | pass |
| 1001 / W25 | 958 | 923 | 954 | 37.5 | 30.4 | 40.0 | 2.8 % | pass |
| 1001 / W50 | 846 | 715 | 874 | 27.1 | 9.6 | 31.3 | 9.6 % | belt fails |
| 1001 / constant 20 | 907 | 877 | 906 | 20.0 | 20.0 | 20.0 | 0.5 % | pass |

With the identified plant the picks agree within 3.3 % in all seven replays and the belt speed
within 10 % in five. The uncalibrated plant fails the belt gate in six and the picks gate in two
(W45 and W50). The two belt failures are at W ≥ 45 s, with opposite signs (sandbox 20 % below the
cell at W45, 16 % above at W50), and the picks there still agree; the
replays of §4.7 show that the first of these (W50) is the closed loop's own scatter and the second (W45) is a real
gap. In
the seed-1001 replays the sandbox's pick-gate abort on a covered target (`sim.gate_cover_abort`)
is on, as it was on the cell.

### 4.7 Noise floor and fit over time

A replay is deterministic, so a single sandbox number cannot tell a misfit from the scatter of
a chaotic closed loop. `experiments/phase0_noise.yaml` replays each cell run seven times with the
plant unchanged and the simulation clock shifted by a few milliseconds (`sim.dt_s` 4–6 ms,
`sim.replan_period_s` 45–55 ms), jitter of the size the robot's own poll and link have.

![Noise floor](figures/identification/fig7_noise_floor.png)

*Figure 7. Sandbox minus cell as a share of the cell, seven clock-shifted replays per cell run; the band is the gate.*

| Cell run | Picked: cell | sandbox mean ± sd [min–max] | Belt (mm/s): cell | sandbox mean ± sd [min–max] |
|---|---|---|---|---|
| 1001 / W15 | 943 | 943.9 ± 2.0 [941–946] | 42.88 | 44.36 ± 0.24 [44.01–44.75] |
| 1001 / W25 | 958 | 954.1 ± 1.7 [952–957] | 37.52 | 40.58 ± 0.62 [40.00–41.88] |
| 1001 / W50 | 846 | 839.6 ± 18.6 [812–874] | 27.05 | 28.24 ± 2.47 [23.35–31.31] |
| 1001 / constant 20 | 907 | 906.0 ± 0 | 19.99 | 20.00 ± 0 |
| 110 / W45 KIM | 1227 | 1198.6 ± 41.8 [1122–1233] | 30.15 | 22.86 ± 4.76 [16.89–27.94] |

| Cell run | Belt RMSE vs the cell's 10 s buckets (mm/s) | 60 s throughput RMSE (parts/min) | Pick-interval distance (KS) |
|---|---|---|---|
| 1001 / W15 | 6.1 | 1.16 | 0.060 |
| 1001 / W25 | 9.8 | 1.67 | 0.062 |
| 1001 / W50 | 8.1 | 2.91 | 0.087 |
| 1001 / constant 20 | 0.15 | 0.38 | 0.073 |
| 110 / W45 KIM | 17.0 | 3.61 | 0.160 |

What it shows:

* **W50 (seed 1001) is not a misfit.** The cell's belt speed (27.1 mm/s) and picks (846) lie inside the
  spread of the seven replays; the sandbox's own scatter at W = 50 s is ±9 % of the belt and ±2 % of the
  picks. The belt "failure" of §4.6 is the closed loop's scatter. A 10 % gate on one deterministic number
  is tighter than the loop's own repeatability at long horizons.
* **W15 and W25 (seed 1001) carry a small bias.** The sandbox belt is above the cell in every replica,
  by 3.4 % and 8.1 %, with a scatter of 0.5–1.5 %; picks agree within 0.5 %. The bias is under the gate
  and its cause is not known.
* **W45 (seed 110, KIM) is below the cell in all seven replicas** (belt −24 % on average, −7 % to −44 %),
  while its picks lie inside the spread. That run is from 2026-10-07, before the gate's covered-target
  abort and other changes, so the cell code differs from the one the sandbox follows. It is the only
  case where the gap is outside the noise, and what causes it was not traced.
* **The arm itself fits.** At constant 20 mm/s the replay does not scatter (picks 906 in all seven), the
  throughput curve is within 0.38 parts/min (1.7 % of the 22 parts/min) and 99.5 % of the parts have
  the same outcome. The pick-interval distributions differ by KS = 0.06–0.09 (0.16 at W45).
* **In time the belt does not track.** Belt RMSE is 6–10 mm/s against the 10 s buckets at W15–W50 (14–29 % of the mean), as expected when two runs of a chaotic loop follow different paths to the same average:
  the sandbox is a model of the aggregate, not of the moment.

The plant refinements the data suggest were not made. Of the goto legs 4 % are 0.16 s slower than
the model; averaged over all picks that is 0.007 s of a 2.2 s cycle, 0.3 %, well below the scatter in
the table. The cell sections of `sandbox/models/delta.yaml` are identical to the session's
`config.yaml`, and the run's `scheduling`, `speed` and feeder settings are taken from its own
`summary.json`, so no setting was left to align. The feeder's detection latency (11 ms) is not
modelled: 0.5 mm at 45 mm/s.

### 4.8 What it means for the W map

![W map](figures/identification/fig6_w_map.png)

*Figure 6. Share of arrived parts picked against the horizon W of `predictive_rank` (sandbox, KIM, 5 seeds × 1 h, mean ± sd) and the cell's three points.*

The map of `experiments/w_map.yaml` was re-run with the identified plant and differs from the
earlier one by 0.025 or less in the cells quoted in open-issues S2: at 24 parts/min the best W is 20 s (0.949), W = 45 s
gives 0.839 and W ≥ 90 s gives 0.599. The cell's three runs of this session (0.943, 0.958 and
0.846 at W = 15, 25, 50 s; 23.4 parts/min arrived) lie on the sandbox's curve, inside or at the
edge of its spread. Three runs of one seed support the shape of the curve, not its peak.

## 5. Limits of this identification

* **Virtual parts.** No camera, no belt-scale or latency terms; the grip is not judged by
  suction (open-issues L6, L17).
* **Timestamps.** `pose.csv` is stamped at PC receive time, so a constant PLC-side delay is
  absorbed in the measured times (open-issues L15). The belt status is sampled every ~40 ms.
* **One session, one day.** The identification and its end-to-end check use the same four
  runs; only the seed-110 replays are unseen data, and they predate the session.
* **Medians.** The plant takes the median overhead per leg; the spread (p5–p95 about ±0.03 s)
  and the dependence on path length (Figure 1b) are not modelled.
* **Closed-loop sensitivity.** The planner and the speed law are chaotic: a part can change
  outcome from a 10 ms difference. Judge the aggregates; the per-part agreement (90–99 %) is a
  weaker statement than the totals. Only one cell run exists per setting, so the cell's own run-to-run
  scatter is unknown; §4.7 measures the sandbox's.
* **Goto legs under 120 mm.** The cell takes up to 0.2 s longer than the trajectory model; the
  sandbox's plant uses the same model plus the median offset, so it runs these legs as fast as
  the planner expects, not as slowly as the cell does.

## 6. Reproduce

```bash
# measurements: tables, legs.csv, a four-panel chart (about 5 s)
python3 -m modules.tools.identify_plant log/20261010-163210_exp_PR_DP_1010_console

# validation (about 10 min) and the W map (about 50 min on 15 cores)
bash sandbox/run_experiments.sh phase0
bash sandbox/run_experiments.sh w_map

# the noise floor of §4.7 (about 25 min)
bash sandbox/run_experiments.sh phase0_noise

# the figures of this report
.venv/bin/python -m sandbox.identification_figures log/20261010-163210_exp_PR_DP_1010_console \
    --replay sandbox/results/phase0_replay-<time> sandbox/results/phase0_replay_1001-<time> \
    --w-map sandbox/results/w_map-<time> --noise sandbox/results/phase0_noise-<time> \
    --out doc/figures/identification
```

The sandbox constants are in `sandbox/config.yaml` (`sim.goto_extra_s`, `sim.pick_extra_s`).
The older plant is `null` for both. Logs of the cell are archived in `log/archived.zip`; the
sandbox unpacks a run from it on demand.

## 7. Where the results are recorded

| Item | Record |
|---|---|
| Trajectory model against the cell, short legs | open-issues C5 |
| Descent depth and timing | open-issues T3 |
| Gate and arm share of the pick error | open-issues C10 |
| Belt step response | open-issues L9 |
| Sandbox validated, noise floor, belt average at long W | open-issues S3 |
| Cell points of the W map | open-issues S2 |
| Why `carry_scale` was replaced | decision-log, 2026-10-10 |
