"""Offline algorithm bench.

    python3 -m sandbox run [--set key=value ...] [--profile]   # one run, summary printed
    python3 -m sandbox sweep sandbox/experiments/<spec>.yaml [--workers N]
    python3 -m sandbox check                                   # model files vs modules/config.yaml
    python3 -m sandbox list                                    # plugins, feeders, arm models
"""

from __future__ import annotations

import argparse
import json
import sys


def _overrides(pairs: list[str]) -> dict:
    from ruamel.yaml import YAML

    yaml = YAML(typ="safe")
    out = {}
    for pair in pairs:
        key, sep, raw = pair.partition("=")
        if not sep:
            raise SystemExit(f"--set expects key=value, got {pair!r}")
        out[key.strip()] = yaml.load(raw) if raw.strip() else None
    return out


def _flatten(prefix: str, value, out: dict) -> None:
    if isinstance(value, dict):
        for key, item in value.items():
            _flatten(f"{prefix}.{key}" if prefix else str(key), item, out)
    else:
        out[prefix] = value


def check() -> int:
    """Every cell value of the sandbox model files that differs from modules/config.yaml."""
    from modules.config_io import read_config
    from sandbox.config import CELL_SECTIONS, MODELS_DIR

    operational: dict = {}
    raw = read_config()
    for section in CELL_SECTIONS:
        _flatten(section, raw.get(section), operational)
    differences = 0
    for path in sorted(MODELS_DIR.glob("*.yaml")):
        cell: dict = {}
        for section, value in (read_config(path).get("cell") or {}).items():
            _flatten(section, value, cell)
        diff = sorted(key for key in set(cell) | set(operational) if cell.get(key) != operational.get(key))
        print(f"{path.name}: {'identical to modules/config.yaml' if not diff else f'{len(diff)} difference(s)'}")
        for key in diff:
            print(f"  {key}: sandbox {cell.get(key)!r}  operational {operational.get(key)!r}")
        differences += len(diff)
    return 0 if differences == 0 else 1


def list_all() -> None:
    from modules.scheduling.registry import describe
    from modules.core.feeders import FEEDERS
    from sandbox.models import ARM_MODELS, _load

    _load()
    print(describe())
    print("Feeders (feeder.kind):")
    for name, (fn, _) in sorted(FEEDERS.items()):
        print(f"  {name:<22} {(fn.__doc__ or '').strip().splitlines()[0]}")
    print("Arm models (sandbox/models/<name>.yaml):")
    for name, (fn, _) in sorted(ARM_MODELS.items()):
        print(f"  {name:<22} {(fn.__doc__ or '').strip().splitlines()[0]}")


def main() -> int:
    parser = argparse.ArgumentParser(prog="python3 -m sandbox", description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    run = sub.add_parser("run", help="one simulated run")
    run.add_argument("--set", action="append", default=[], metavar="KEY=VALUE")
    run.add_argument("--profile", action="store_true", help="print the 15 most expensive calls")
    sw = sub.add_parser("sweep", help="an experiment spec, in parallel")
    sw.add_argument("spec")
    sw.add_argument("--workers", type=int, default=None)
    sub.add_parser("check", help="compare the model files' cell with modules/config.yaml")
    sub.add_parser("list", help="list plugins, feeders and arm models")
    args = parser.parse_args()

    if args.command == "check":
        return check()
    if args.command == "list":
        list_all()
        return 0
    if args.command == "sweep":
        from sandbox.sweep import sweep

        sweep(args.spec, args.workers)
        return 0

    from sandbox.sweep import run_one

    overrides = _overrides(args.set)
    if args.profile:
        import cProfile
        import pstats

        profiler = cProfile.Profile()
        profiler.enable()
        result = run_one(overrides)
        profiler.disable()
        pstats.Stats(profiler, stream=sys.stdout).sort_stats("cumtime").print_stats(15)
    else:
        result = run_one(overrides)
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
