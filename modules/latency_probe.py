"""Per-PLC communication latency probe.

The scheduler's ``_RoundTripTracker`` reports a single "round trip" figure, but
the IPC worker's status handler (``main.py``) reads the Omron gateway
(``get_package``) and *then* the Siemens gateway (``get_status``) in sequence,
so that figure is the SUM of both network paths. Only the Omron path actually
gates a pick dispatch (the pick command goes to the Omron), so the Siemens
latency inflates the number the scheduler calibrates against.

This tool times each PLC's read-only status call individually, plus the
sequential Omron->Siemens combo the worker performs, and reports
min/mean/median/p95/max for each. It then suggests an ``ethernet_delay_s`` from
the Omron-only mean.

Read-only: it issues only status reads (``get_package`` / ``get_status``); it
never writes a command packet, so it is safe to run against live hardware.

Usage:
    python3 -m modules.latency_probe                       # both, 100 samples
    python3 -m modules.latency_probe --count 300           # more samples
    python3 -m modules.latency_probe --target omron        # Omron only
    python3 -m modules.latency_probe --target siemens      # Siemens only
"""
from __future__ import annotations

import argparse
import statistics
import time
from typing import Callable

from modules.EthernetCom import PLCGateway, SiemensGateway, load_config


def _percentile(sorted_values: list[float], pct: float) -> float:
    """Nearest-rank percentile (pct in [0, 100]) over an already-sorted list."""
    if not sorted_values:
        return 0.0
    if len(sorted_values) == 1:
        return sorted_values[0]
    rank = pct / 100.0 * (len(sorted_values) - 1)
    lo = int(rank)
    hi = min(lo + 1, len(sorted_values) - 1)
    frac = rank - lo
    return sorted_values[lo] * (1.0 - frac) + sorted_values[hi] * frac


def _summarise(label: str, samples_ms: list[float], errors: int) -> dict[str, float]:
    """Print and return summary stats (ms) for one measured target."""
    if not samples_ms:
        print(f"  {label:<22} no successful samples ({errors} errors)")
        return {}
    ordered = sorted(samples_ms)
    stats = {
        "n": len(ordered),
        "errors": errors,
        "min": ordered[0],
        "mean": statistics.fmean(ordered),
        "median": statistics.median(ordered),
        "p95": _percentile(ordered, 95.0),
        "max": ordered[-1],
    }
    print(
        f"  {label:<22} n={stats['n']:<4} err={errors:<3} "
        f"min={stats['min']:6.2f}  mean={stats['mean']:6.2f}  "
        f"median={stats['median']:6.2f}  p95={stats['p95']:6.2f}  "
        f"max={stats['max']:6.2f}  (ms)"
    )
    return stats


def _measure(
    label: str, read_fn: Callable[[], object], count: int, warmup: int
) -> dict[str, float]:
    """Time ``read_fn`` ``count`` times (after ``warmup`` untimed calls)."""
    for _ in range(warmup):
        try:
            read_fn()
        except Exception:
            pass
    samples_ms: list[float] = []
    errors = 0
    for _ in range(count):
        t0 = time.perf_counter()
        try:
            result = read_fn()
        except Exception:
            result = None
        dt_ms = (time.perf_counter() - t0) * 1000.0
        if result is None:
            errors += 1
        else:
            samples_ms.append(dt_ms)
    return _summarise(label, samples_ms, errors)


def run(target: str, count: int, warmup: int) -> int:
    config = load_config()
    omron: PLCGateway | None = None
    siemens: SiemensGateway | None = None

    want_omron = target in ("omron", "both")
    want_siemens = target in ("siemens", "both")

    print(f"[PROBE] target={target} count={count} warmup={warmup}")
    print(f"[PROBE] omron   = {config.ip_address}:{getattr(config, 'port', 502)}")
    print(
        f"[PROBE] siemens = {getattr(config, 'siemens_ip', '192.168.250.2')}:"
        f"{getattr(config, 'siemens_port', 1502)}"
    )

    try:
        if want_omron:
            omron = PLCGateway(
                config.ip_address,
                getattr(config, "port", 502),
                getattr(config, "interpolar_points", 7),
            )
            try:
                omron.connect()
            except Exception as exc:
                print(f"[PROBE] Omron connect failed: {exc}")
                omron = None
        if want_siemens:
            siemens = SiemensGateway(
                getattr(config, "siemens_ip", None),
                getattr(config, "siemens_port", None),
            )
            if not siemens.connect():
                print("[PROBE] Siemens connect failed.")
                siemens = None

        print("\n[PROBE] Per-PLC read latency:")
        omron_stats: dict[str, float] = {}
        if want_omron and omron is not None:
            omron_stats = _measure("omron get_package", omron.get_package, count, warmup)
        if want_siemens and siemens is not None:
            _measure("siemens get_status", siemens.get_status, count, warmup)

        # Sequential combo — what the IPC worker's "status" handler actually does
        # (Omron read THEN Siemens read), for comparison against the per-PLC times.
        if target == "both" and omron is not None and siemens is not None:
            def _combo() -> object:
                o = omron.get_package()
                s = siemens.get_status()
                return o if (o is not None and s is not None) else None

            _measure("combo omron+siemens", _combo, count, warmup)

        if omron_stats:
            suggested_s = omron_stats["mean"] / 1000.0
            print(
                f"\n[PROBE] Suggested scheduler.ethernet_delay_s = {suggested_s:.4f} "
                f"(Omron-only mean; the pick path waits on the Omron, not the Siemens)."
            )
    finally:
        if omron is not None:
            try:
                omron.disconnect()
            except Exception:
                pass
        if siemens is not None:
            try:
                siemens.disconnect()
            except Exception:
                pass
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--count", type=int, default=100, help="Timed samples per target")
    parser.add_argument("--warmup", type=int, default=5, help="Untimed warmup calls per target")
    parser.add_argument(
        "--target",
        choices=("omron", "siemens", "both"),
        default="both",
        help="Which PLC(s) to probe",
    )
    args = parser.parse_args(argv)
    return run(args.target, args.count, args.warmup)


if __name__ == "__main__":
    raise SystemExit(main())
