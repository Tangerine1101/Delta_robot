#!/usr/bin/env bash
# Run the pre-cell simulation experiments with one command (any shell: bash sandbox/run_experiments.sh).
#
#   bash sandbox/run_experiments.sh phase0           # does the sandbox reproduce the cell? (~10 min)
#   bash sandbox/run_experiments.sh phase1           # the W map against load (~30 min on 15 cores)
#   bash sandbox/run_experiments.sh p1               # the dp study (doc/experiment_notes.md): select, confirm, regime,
#                                                    # solve time, then the select-and-confirm analyses (~1-2 h)
#   bash sandbox/run_experiments.sh all              # phase0, then phase1
#   bash sandbox/run_experiments.sh <name> [...]     # any experiment of sandbox/experiments/
# Options (anywhere): --quick (a few minutes, coarser), --dry-run (count runs, estimate time).
# Results: sandbox/results/<experiment>-<time>/report.md plus charts and CSV files.
set -euo pipefail
cd "$(dirname "$0")/.."
PY=.venv/bin/python
[ -x "$PY" ] || PY=python3

opts=()
names=()
analyse=0
for arg in "$@"; do
  case "$arg" in
    --quick|--dry-run) opts+=("$arg") ;;
    -h|--help) sed -n '2,10p' "$0"; exit 0 ;;
    phase0) names+=(phase0_replay phase0_replay_1001) ;;
    phase1) names+=(w_map w_map_dp) ;;
    p1) names+=(p1_select p1_confirm p1_regime solve_time); analyse=1 ;;
    all) names+=(phase0_replay phase0_replay_1001 w_map w_map_dp) ;;
    *) names+=("$arg") ;;
  esac
done
[ ${#names[@]} -gt 0 ] || { sed -n '2,10p' "$0"; "$PY" -m sandbox exp --list; exit 1; }

for name in "${names[@]}"; do
  echo "=== $name ${opts[*]:-}"
  "$PY" -m sandbox exp "$name" ${opts[@]+"${opts[@]}"}
done

if [ "$analyse" = 1 ] && [ ${#opts[@]} -eq 0 ]; then
  echo "=== select and confirm: stationary load"
  "$PY" -m sandbox confirm --select p1_select --confirm p1_confirm --out sandbox/results/p1_confirmation
  echo "=== select and confirm: regime load"
  "$PY" -m sandbox confirm --select p1_regime --select-seeds 2001-2010 --confirm p1_regime --confirm-seeds 3001-3010 \
    --out sandbox/results/p1_confirmation_regime
fi
