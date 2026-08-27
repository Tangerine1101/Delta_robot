# `investigation/` — pick-accuracy investigation, 2026-08-24

An ad-hoc, self-contained investigation directory. **It is not part of the `doc/` set** and
nothing in it is authoritative: `doc/context.md` §2 still describes the six documents that
are. Findings that survive should be promoted into `doc/open-issues.md` and
`doc/decision-log.md`; this directory is the working evidence behind them.

Nothing here modifies the repository. Every module imports `modules.*` read-only.

| File | Purpose |
|---|---|
| [`REPORT.md`](REPORT.md) | **Start here.** The findings, the evidence, and what blocks what. |
| `_bridge.py` | Loads this repo and the sibling sandbox `../python for scheduling` side by side under non-colliding names. |
| `test_algorithm_parity.py` | Runs both repositories' version of each shared algorithm on identical inputs. |
| `test_gate_logic.py` | Drives individual shipped functions at reachable boundaries. |
| `virtual_cell.py` | A plant — belt, delta arm, boards, camera, wire — modelling the PLC's State 10 bridge and a physically correct two-pass segment schedule. |
| `run_production_sim.py` | Wires the plant into the **real** `_run_realtime_pick_loop` and scores where the cup actually landed. |
| `experiments.py` | The eight experiments, one suspected cause each. |
| `delay_budget.py` | Every source of dead time in the loop, sorted into the four budgets, with what pays for it and how to measure what does not. |
| `results-*.txt` | Captured output of the three runners. |

```bash
python3 -m investigation.test_algorithm_parity     # ~0.1 s
python3 -m investigation.test_gate_logic           # ~0.1 s
python3 -m investigation.experiments               # ~12 min wall clock
python3 -m investigation.experiments E2 E8         # selected
python3 -m investigation.run_production_sim --belt 120 --duration 45 --verbose
python3 -m investigation.delay_budget              # instant, no hardware
```

The simulation runs in wall-clock time with the real thread structure, so a 30 s experiment
takes 30 s. `run_production_sim` also answers `open-issues.md` **L8** in passing: it is an
offline, repeatable dry run of the `production` code path, which previously had none.

Read `REPORT.md` §1 before trusting a number — in particular the paragraph on where the
virtual arm is optimistic.
