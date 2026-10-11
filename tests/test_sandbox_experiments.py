"""The experiment machinery of the sandbox: specs, factor columns, reports, and the replay of
logged cell runs.

Run:  .venv/bin/python -m unittest tests.test_sandbox_experiments
"""

from __future__ import annotations

import csv
import json
import tempfile
import unittest
from pathlib import Path

from sandbox import replay
from sandbox.cellrun import CellRun, cell_overrides
from sandbox.exp import describe, find_spec
from sandbox.report import aggregate, best_table, build_report, label, read_runs
from sandbox.sweep import expand, factors_of, load_spec, simulate, write_run


def write_csv(path: Path, rows: list[dict]) -> None:
    with open(path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


class Specs(unittest.TestCase):
    def test_factors_are_the_settings_that_differ_and_named_by_their_last_part(self):
        cases = [("a", {"x.rate": 10, "speed.planner": "kim", "same": 1}),
                 ("b", {"x.rate": 20, "speed.planner": "dp", "same": 1}),
                 ("c", {"x.rate": 20, "scheduling.planner": "dp", "same": 1})]
        self.assertEqual(factors_of(cases), {"rate": "x.rate", "speed.planner": "speed.planner",
                                             "scheduling.planner": "scheduling.planner"})

    def test_quick_replaces_grid_and_seeds_and_merges_base(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "s.yaml"
            path.write_text("base: {sim.duration_s: 3600, a: 1}\nseeds: [1, 2, 3]\ngrid: {b: [1, 2, 3]}\n"
                            "quick: {base: {sim.duration_s: 60}, seeds: [1], grid: {b: [1]}}\n")
            full, quick = load_spec(path), load_spec(path, quick=True)
        self.assertEqual((full["seeds"], full["grid"]["b"]), ([1, 2, 3], [1, 2, 3]))
        self.assertEqual((quick["seeds"], quick["grid"]["b"]), ([1], [1]))
        self.assertEqual(quick["base"], {"sim.duration_s": 60, "a": 1})

    def test_cell_config_is_the_base_and_the_spec_wins(self):
        spec = {"cell_config": "modules/config.yaml", "base": {"scheduling.planner": "edd"},
                "grid": {"speed.law": ["constant"]}}
        (_, overrides), = expand(spec)
        self.assertEqual(overrides["scheduling.planner"], "edd")
        self.assertIn("speed.laws.predictive_rank.horizon_s", overrides)
        self.assertIn("feeder.poisson.rate_per_min", overrides)

    def test_every_shipped_experiment_resolves(self):
        """Each spec expands into configs the strict loader accepts."""
        from modules.config_io import read_config
        from sandbox.config import build_run, load_run_config

        text = describe()
        self.assertIn("w_map", text)
        for path in sorted(find_spec("w_map").parent.glob("*.yaml")):
            raw = read_config(path)
            if raw.get("kind", "sweep") != "sweep":
                continue
            for quick in (False, True) if raw.get("quick") else (False,):
                for _, overrides in expand(load_spec(path, quick)):
                    build_run(load_run_config(overrides=overrides))


class Reports(unittest.TestCase):
    def fake_sweep(self, tmp: str) -> Path:
        out = Path(tmp)
        rows = []
        for w, picked in ((15, 95), (45, 70), (1000, 60)):
            for seed in (1, 2):
                rows.append({"case": f"w={w}", "seed": seed, "horizon_s": w, "stack_aware": True,
                             "spawned": 100, "picked": picked + seed, "lost": 100 - picked - seed,
                             "covered_misses": 2, "bypassed_lost": 0, "picks_per_min": 10.0,
                             "mean_belt_mm_s": 30.0, "dip_s": 0.0})
        write_csv(out / "runs.csv", rows)
        spec = {"report": {"metrics": ["pick_share"], "labels": {"horizon_s": {1000: "inf"}},
                           "charts": [{"kind": "line", "x": "horizon_s", "y": "pick_share", "series": "stack_aware"},
                                      {"kind": "heatmap", "x": "horizon_s", "y": "stack_aware", "value": "pick_share"},
                                      {"kind": "bar", "x": "horizon_s", "y": "picks_per_min"}],
                           "best": [{"over": "horizon_s", "metric": "pick_share", "group": ["stack_aware"],
                                     "tolerance": 0.01}]}}
        (out / "manifest.json").write_text(json.dumps({
            "spec": "fake.yaml", "cases": {f"w={w}": {} for w in (15, 45, 1000)}, "seeds": [1, 2],
            "factors": {"horizon_s": "x", "stack_aware": "y"}, "spec_full": spec, "wall_s": 1, "git": "x"}))
        return out

    def test_report_writes_tables_charts_and_the_best_level(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = self.fake_sweep(tmp)
            build_report(out)
            text = (out / "report.md").read_text()
            self.assertEqual(sorted(p.name for p in out.glob("chart_*.png")),
                             ["chart_1_pick_share.png", "chart_2_pick_share.png", "chart_3_picks_per_min.png"])
            self.assertTrue((out / "summary.csv").is_file())
        self.assertIn("| True | 15 |", text.replace("  ", " "))   # best level of W
        self.assertIn("inf", text)
        self.assertIn("0.965 ± 0.007", text)                    # 15 s: 96 and 97 of 100

    def test_best_level_and_plateau(self):
        with tempfile.TemporaryDirectory() as tmp:
            rows, _ = read_runs(self.fake_sweep(tmp))
        table = best_table(rows, {"over": "horizon_s", "metric": "pick_share", "group": ["stack_aware"],
                                  "tolerance": 0.5}, {})
        self.assertIn("15, 45, 1000", table[-1])
        self.assertEqual(label({"horizon_s": {1000: "inf"}}, "horizon_s", 1000), "inf")
        self.assertEqual(list(aggregate(rows, ["horizon_s"])), [(15,), (45,), (1000,)])

    def test_a_report_naming_a_missing_column_says_so(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = self.fake_sweep(tmp)
            manifest = json.loads((out / "manifest.json").read_text())
            manifest["spec_full"]["report"]["metrics"] = ["nonsense"]
            (out / "manifest.json").write_text(json.dumps(manifest))
            with self.assertRaises(SystemExit):
                build_report(out)


class Replay(unittest.TestCase):
    def test_parts_are_matched_by_arrival_order_not_by_name(self):
        cell = [{"part_id": f"vf{n:04d}", "outcome": "picked", "reason": "gripped"} for n in (2, 1, 3)]
        sim = [{"part_id": f"p{n:05d}", "outcome": "miss_late", "reason": "never_planned"} for n in (2, 0, 1)]
        pairs = replay.match(cell, sim)
        self.assertEqual([(c["part_id"], s["part_id"]) for c, s in pairs],
                         [("vf0001", "p00000"), ("vf0002", "p00001"), ("vf0003", "p00002")])

    def test_outcomes_share_one_vocabulary(self):
        self.assertEqual(replay.category({"outcome": "miss_grip", "reason": "covered"}), "covered")
        self.assertEqual(replay.category({"outcome": "miss_grip", "reason": "cup_off_part"}), "grip_other")
        self.assertEqual(replay.category({"outcome": "lost_track", "reason": ""}), "late")
        self.assertEqual(replay.category({"outcome": "on_belt", "reason": "run_ended"}), "unfinished")

    def test_a_logged_run_replays_in_the_sandbox_and_is_compared(self):
        """Replay of a synthetic 'cell' run: the arrivals come from arrivals.csv, the cell's
        settings from the session config, and a record identical to the sandbox's own agrees."""
        with tempfile.TemporaryDirectory() as tmp:
            session = Path(tmp) / "20260101-000000_console"
            run_dir = session / "run01_000000_simulate_feeder"
            run_dir.mkdir(parents=True)
            (session / "config.yaml").write_text(Path("modules/config.yaml").read_text())
            write_csv(run_dir / "arrivals.csv",
                      [{"part_id": f"vf{i:04d}", "t_s": 2.0 + 3.0 * i, "type": "QFP", "v_mm": 60.0,
                        "heading_deg": 10.0, "regime": "poisson"} for i in range(1, 8)])
            overrides = {"scheduling.planner": "edd", "speed.law": "constant", "speed.static_mm_s": 40}
            (run_dir / "summary.json").write_text(json.dumps({"duration_s": 40.0, "overrides": overrides,
                                                              "planner": "edd", "speed_law": "constant"}))
            write_csv(run_dir / "parts.csv", [{"part_id": "vf0001", "t_seen": 2.0, "t_contact": "",
                                               "outcome": "on_belt", "reason": "", "t_closed": ""}])
            run = CellRun.load(run_dir)
            sandbox = run.overrides()
            self.assertEqual((sandbox["scheduling.planner"], sandbox["speed.static_mm_s"]), ("edd", 40))
            self.assertEqual(sandbox["feeder.kind"], "script")
            sim = simulate(sandbox)
            self.assertEqual(sim.metrics.spawned, 7)
            self.assertGreaterEqual(len(sim.belt_log), 40)
            out = Path(tmp) / "out"
            write_run(sim, sandbox, out)
            with open(out / "flow.csv") as handle:
                self.assertEqual(next(csv.reader(handle)), ["t", "input_per_min", "output_per_min", "belt_mm_s"])
            result = replay.compare(run, sim, replay.DEFAULT_GATES)
            self.assertEqual(result["parts"], 0)       # the one cell row is unfinished: nothing to compare

    def test_fit_metrics_distance_between_two_runs(self):
        from sandbox import replay

        self.assertEqual(replay.ks_statistic([1.0, 2.0, 3.0], [1.0, 2.0, 3.0]), 0.0)
        self.assertEqual(replay.ks_statistic([1.0, 2.0], [5.0, 6.0]), 1.0)
        self.assertAlmostEqual(replay.ks_statistic([1.0, 2.0, 3.0, 4.0], [3.0, 4.0, 5.0, 6.0]), 0.5)
        self.assertTrue(replay.ks_statistic([], [1.0]) != replay.ks_statistic([], [1.0]))      # nan
        self.assertAlmostEqual(replay._rmse([(1.0, 2.0), (3.0, 3.0), (5.0, 4.0)]), (2 / 3) ** 0.5)

    def test_select_on_one_set_of_seeds_confirm_on_another(self):
        from sandbox import confirm

        # selection: W 10 < 20 > 40, c 20 is best; confirmation: the same levels on other seeds
        def rows(seeds, shift):
            out = []
            for seed in seeds:
                noise = 0.002 * (seed % 5)
                for law, col, level, base in (("predictive_rank", "horizon_s", 10, 0.90), ("predictive_rank", "horizon_s", 20, 0.95),
                                              ("predictive_rank", "horizon_s", 40, 0.80), ("predictive_rank", "horizon_s", 1000, 0.60),
                                              ("constant", "static_mm_s", 20, 0.90), ("constant", "static_mm_s", 60, 0.78)):
                    for planner, delta in (("dp", 0.0), ("kim", -0.004)):
                        out.append({"case": f"{planner}|{law}|{level}", "seed": seed, "law": law, "planner": planner,
                                    "horizon_s": level if col == "horizon_s" else 50, "static_mm_s": level if col == "static_mm_s" else 10,
                                    "spawned": 1000, "picked": round(1000 * (base + shift + noise + delta))})
            return out

        with tempfile.TemporaryDirectory() as tmp:
            dirs = []
            for name, seeds in (("sel", range(2001, 2006)), ("con", range(3001, 3004))):
                d = Path(tmp) / name
                d.mkdir()
                data = rows(seeds, 0.0)
                with open(d / "runs.csv", "w", newline="") as handle:
                    writer = csv.DictWriter(handle, fieldnames=list(data[0]))
                    writer.writeheader()
                    writer.writerows(data)
                (d / "manifest.json").write_text("{}")
                dirs.append(d)
            out = Path(tmp) / "out"
            report = confirm.run(str(dirs[0]), str(dirs[1]), planner="dp", gap=0.03, out=out)
            text = report.read_text()
            self.assertIn("| predictive_rank | W = 10 | W = 20 | W = 40 |", text)
            self.assertIn("W = 20 − W = ∞", text)
            self.assertIn("dp − kim", text)
            self.assertTrue((out / "contrasts.csv").is_file())
        self.assertEqual(confirm.parse_seeds("2001-2003,3001"), {2001, 2002, 2003, 3001})
        paired = confirm.paired({1: 0.5, 2: 0.6, 3: 0.7}, {1: 0.4, 2: 0.5, 3: 0.6}, 1.5)
        self.assertAlmostEqual(paired["mean"], 0.1)
        self.assertAlmostEqual(paired["sd_d"], 0.0)

    def test_max_parts_and_solve_time_are_recorded(self):
        from sandbox.sweep import run_one

        result = run_one({"sim.duration_s": 400.0, "feeder.max_parts": 7, "scheduling.planner": "dp",
                          "speed.law": "predictive_rank"})
        self.assertEqual(result["spawned"], 7)
        self.assertGreater(result["law_ms_n"], 0)
        self.assertIsNotNone(result["law_ms_max"])

    def test_an_archived_run_is_unpacked_on_demand(self):
        import zipfile
        from unittest import mock

        from sandbox import cellrun

        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp)
            (repo / "log").mkdir()
            with zipfile.ZipFile(repo / "log" / "archived.zip", "w") as z:
                z.writestr("S_console/config.yaml", "speed: {}\n")
                z.writestr("S_console/run01_x_simulate_feeder/summary.json", "{}")
                z.writestr("T_console/run01_x_simulate_feeder/summary.json", "{}")
                z.writestr("archived/O_console/config.yaml", "speed: {}\n")      # a session archived before the zip
                z.writestr("archived/O_console/run01_x_simulate_feeder/summary.json", "{}")
            with mock.patch.object(cellrun, "REPO_DIR", repo), \
                    mock.patch.object(cellrun, "ARCHIVE", repo / "log" / "archived.zip"), \
                    mock.patch.object(cellrun, "CACHE_DIR", repo / "cache"):
                found = cellrun.resolve_run("log/S_console/run01_x_simulate_feeder")
                self.assertTrue((found / "summary.json").is_file())
                self.assertTrue((found.parent / "config.yaml").is_file())
                self.assertFalse((repo / "cache" / "T_console").exists())      # only what was asked for
                old = cellrun.resolve_run("log/O_console/run01_x_simulate_feeder")
                self.assertTrue((old / "summary.json").is_file())
                self.assertTrue((old.parent / "config.yaml").is_file())
                missing = cellrun.resolve_run("log/none/run01_simulate_feeder")
                self.assertFalse(missing.is_dir())

    def test_a_run_replays_from_the_settings_it_recorded(self):
        """summary.json carries the run's own scheduling / speed / feeder; the sandbox loads them
        and a W edited in the console between runs is not lost."""
        from modules.runtime.scenarios import run_meta
        from sandbox.config import build_run, load_run_config
        from tests.helpers import settings as load

        cfg = load(**{"speed.laws.predictive_rank.horizon_s": 33.0, "scheduling.planner": "dp"})
        meta = json.loads(json.dumps(run_meta("simulate_feeder", cfg, "2026-10-11T00:00:00", None, "camera", None)))
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "run01"
            run_dir.mkdir()
            write_csv(run_dir / "arrivals.csv", [{"part_id": "vf0001", "t_s": 1.0, "type": "QFP", "v_mm": 50.0,
                                                  "heading_deg": 0.0, "regime": "poisson"}])
            write_csv(run_dir / "parts.csv", [{"part_id": "vf0001", "outcome": "on_belt", "reason": ""}])
            (run_dir / "summary.json").write_text(json.dumps({**meta, "duration_s": 10.0}))
            overrides = CellRun.load(run_dir).overrides()
        self.assertEqual(overrides["speed.laws.predictive_rank.horizon_s"], 33.0)
        run = build_run(load_run_config(overrides=overrides))
        self.assertEqual(run.scheduling.planner, "dp")
        self.assertEqual(run.speed.laws["predictive_rank"]["horizon_s"], 33.0)

    def test_cell_overrides_carry_the_sections_the_sandbox_takes(self):
        overrides = cell_overrides("modules/config.yaml")
        self.assertIn("scheduling.planner", overrides)
        self.assertIn("speed.band.min_mm_s", overrides)
        self.assertTrue(overrides["sim.stacking"])
        self.assertNotIn("robot.limits.radius_xy_mm", overrides)


if __name__ == "__main__":
    unittest.main()
