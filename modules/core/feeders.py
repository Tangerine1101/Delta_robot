"""Feeders: when and where parts land on the belt, fixed before the run from a seed.

A feeder is one registered function `(cfg, rng, duration_s) -> list[SpawnEvent]`. The sandbox
reads its parameters from `feeder.<kind>` in its run config, the robot's `simulate_feeder`
scenario from `feeder.kinds.<kind>` in modules/config.yaml. Parts appear at the camera origin
(u = 0). The speed controller only earns its keep on irregular arrivals, so a periodic feeder
is a control, not a test.

`rate_field` names the parameter that sets a feeder's mean arrival rate (parts per minute), so
a caller can set "the rate" without knowing each feeder's own parameter names; None = the
feeder has no single rate (periodic: `interval_s`, script: explicit times).
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Callable

from modules.settings import build_section


@dataclass(frozen=True)
class SpawnEvent:
    time_s: float
    part_type: str
    v_mm: float
    heading_deg: float = 0.0
    regime: str = ""


FEEDERS: dict[str, tuple[Callable, type]] = {}
RATE_FIELDS: dict[str, str | None] = {}


def feeder(name: str, config: type, *, rate_field: str | None = None):
    def decorate(fn):
        FEEDERS[name] = (fn, config)
        RATE_FIELDS[name] = rate_field
        return fn
    return decorate


def build_events(kind: str, params: dict, rng: random.Random, duration_s: float) -> list[SpawnEvent]:
    try:
        fn, cfg_cls = FEEDERS[kind]
    except KeyError:
        raise SystemExit(f"unknown feeder '{kind}'. Known: {', '.join(sorted(FEEDERS))}") from None
    return fn(build_section(cfg_cls, params or {}, f"feeder.{kind}"), rng, duration_s)


# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ScriptConfig:
    events: tuple[tuple[float, str, float], ...] = ()   # (time_s, type, v_mm)


@feeder("script", ScriptConfig)
def script(cfg: ScriptConfig, rng: random.Random, duration_s: float) -> list[SpawnEvent]:
    """Exactly the listed events."""
    return [SpawnEvent(t, kind, v, 0.0, "script") for t, kind, v in cfg.events if t < duration_s]


@dataclass(frozen=True)
class PeriodicConfig:
    interval_s: float = 2.5
    types: tuple[str, ...] = ("QFP", "TQFP")
    lanes: tuple[float, ...] = (20.0, 60.0, 100.0)


@feeder("periodic", PeriodicConfig)
def periodic(cfg: PeriodicConfig, rng: random.Random, duration_s: float) -> list[SpawnEvent]:
    """One part every interval_s, cycling through the types and lanes (no irregularity)."""
    events, t, i = [], 0.0, 0
    while t < duration_s:
        events.append(SpawnEvent(t, cfg.types[i % len(cfg.types)], cfg.lanes[i % len(cfg.lanes)],
                                 rng.uniform(0.0, 360.0), "periodic"))
        t += cfg.interval_s
        i += 1
    return events


@dataclass(frozen=True)
class PoissonConfig:
    rate_per_min: float = 24.0
    types: tuple[str, ...] = ("QFP", "TQFP")
    lane_band_v: tuple[float, float] = (10.0, 115.0)


@feeder("poisson", PoissonConfig, rate_field="rate_per_min")
def poisson(cfg: PoissonConfig, rng: random.Random, duration_s: float) -> list[SpawnEvent]:
    """Exponential gaps, uniform lanes, no spacing check."""
    events, t = [], 0.0
    rate = cfg.rate_per_min / 60.0
    while True:
        t += rng.expovariate(rate)
        if t >= duration_s:
            return events
        events.append(SpawnEvent(t, rng.choice(cfg.types), rng.uniform(*cfg.lane_band_v),
                                 rng.uniform(0.0, 360.0), "poisson"))


@dataclass(frozen=True)
class Regime:
    weight: float = 1.0
    factor: tuple[float, float] = (1.0, 1.0)     # rate multiplier range


@dataclass(frozen=True)
class RegimeConfig:
    nominal_rate_per_min: float = 24.0
    segment_s: tuple[float, float] = (10.0, 30.0)
    types: tuple[str, ...] = ("QFP", "TQFP")
    lane_band_v: tuple[float, float] = (10.0, 115.0)
    regimes: dict[str, Regime] = field(default_factory=lambda: {
        "nominal": Regime(0.6, (0.8, 1.2)), "surge": Regime(0.2, (1.8, 2.5)), "starve": Regime(0.2, (0.2, 0.5))})


@feeder("regime", RegimeConfig, rate_field="nominal_rate_per_min")
def regime(cfg: RegimeConfig, rng: random.Random, duration_s: float) -> list[SpawnEvent]:
    """Rate shifting between regimes (nominal, surge, starve); Poisson inside each segment.
    An exponential gap cut by a segment boundary is redrawn (memoryless)."""
    names = list(cfg.regimes)
    weights = [cfg.regimes[n].weight for n in names]
    events, t = [], 0.0
    while t < duration_s:
        name = rng.choices(names, weights=weights)[0]
        rate = cfg.nominal_rate_per_min / 60.0 * rng.uniform(*cfg.regimes[name].factor)
        end = t + rng.uniform(*cfg.segment_s)
        while True:
            gap = rng.expovariate(rate) if rate > 0.0 else math.inf
            if t + gap > end:
                break
            t += gap
            if t >= duration_s:
                return events
            events.append(SpawnEvent(t, rng.choice(cfg.types), rng.uniform(*cfg.lane_band_v),
                                     rng.uniform(0.0, 360.0), name))
        t = end
    return events


@dataclass(frozen=True)
class BurstyConfig:
    mean_rate_per_min: float = 24.0
    rate_factor_centre: float = 1.0
    rate_factor_spread: float = 0.5
    rate_factor_bounds: tuple[float, float] = (0.2, 2.5)
    segment_s: tuple[float, float] = (5.0, 20.0)
    types: tuple[str, ...] = ("QFP", "TQFP")
    lane_band_v: tuple[float, float] = (10.0, 115.0)


@feeder("bursty", BurstyConfig, rate_field="mean_rate_per_min")
def bursty(cfg: BurstyConfig, rng: random.Random, duration_s: float) -> list[SpawnEvent]:
    """A bounded bell-shaped rate factor held for a whole segment, Poisson inside: a fast
    stretch yields several short gaps in a row, which independent gaps almost never do."""
    events, t = [], 0.0
    lo, hi = cfg.rate_factor_bounds
    while t < duration_s:
        while True:
            factor = rng.gauss(cfg.rate_factor_centre, cfg.rate_factor_spread)
            if lo <= factor <= hi:
                break
        rate = cfg.mean_rate_per_min / 60.0 * factor
        end = t + rng.uniform(*cfg.segment_s)
        while True:
            gap = rng.expovariate(rate)
            if t + gap > end:
                break
            t += gap
            if t >= duration_s:
                return events
            events.append(SpawnEvent(t, rng.choice(cfg.types), rng.uniform(*cfg.lane_band_v),
                                     rng.uniform(0.0, 360.0), f"x{factor:.2f}"))
        t = end
    return events
