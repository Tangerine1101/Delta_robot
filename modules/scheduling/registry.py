"""Plugin registry: dispatch rules, sequence planners, speed laws and setpoint gates.

A plugin is one decorated function. The decorator records its name, kind and (optionally) a
typed config dataclass whose fields are read from `scheduling.planners.<name>` or
`speed.laws.<name>` in config.yaml. Config selects plugins by name; an unknown name is an error
that lists the known ones.

Calling conventions (see README.md for worked examples):

    @dispatch_rule("spt")                      rule(jobs, ctx)            -> Job | None
    @planner("kim")                            planner(jobs, ctx)         -> list[ScheduledPick]
    @speed_law("x", default_gate="arm_free")   law(view)                  -> SpeedDecision
    @setpoint_gate("arm_free")                 gate(event, since_s, period_s) -> bool

A plugin declared with `config=SomeDataclass` receives its config as one more argument:
`rule(jobs, ctx, cfg)`, `planner(jobs, ctx, cfg)`, `law(view, cfg)`.
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass
from typing import Any, Callable

from modules.settings import SettingsError, build_section

from modules.scheduling.types import Job, PlanContext, ScheduledPick, SpeedDecision, SpeedView


@dataclass(frozen=True)
class PlannerSpec:
    name: str
    kind: str                                   # "rule" | "planner"
    fn: Callable[..., Any]
    config: type | None = None
    doc: str = ""


@dataclass(frozen=True)
class SpeedLawSpec:
    name: str
    fn: Callable[..., SpeedDecision]
    default_gate: str
    config: type | None = None
    # False for a law whose candidates are already discrete and step-limited: the deadband
    # would then suppress a move the law scored ("scored speed = sent speed").
    deadband: bool = True
    doc: str = ""


@dataclass(frozen=True)
class GateSpec:
    name: str
    fn: Callable[[str, float, float], bool]
    doc: str = ""


PLANNERS: dict[str, PlannerSpec] = {}
SPEED_LAWS: dict[str, SpeedLawSpec] = {}
GATES: dict[str, GateSpec] = {}


def _first_line(fn: Callable[..., Any]) -> str:
    return (fn.__doc__ or "").strip().split("\n")[0]


def _register(table: dict[str, Any], kind: str, name: str, spec: Any) -> None:
    if name in table:
        raise ValueError(f"{kind} '{name}' is registered twice")
    table[name] = spec


def dispatch_rule(name: str, *, config: type | None = None):
    """Register a rule that picks one job; the registry turns it into a planner."""
    def decorate(fn):
        _register(PLANNERS, "planner", name, PlannerSpec(name, "rule", fn, config, _first_line(fn)))
        return fn
    return decorate


def planner(name: str, *, config: type | None = None):
    """Register a sequence planner that returns the retained jobs in pick order."""
    def decorate(fn):
        _register(PLANNERS, "planner", name, PlannerSpec(name, "planner", fn, config, _first_line(fn)))
        return fn
    return decorate


def speed_law(name: str, *, default_gate: str, config: type | None = None, deadband: bool = True):
    """Register a belt speed law and the setpoint gate it runs under by default."""
    def decorate(fn):
        _register(SPEED_LAWS, "speed law", name,
                  SpeedLawSpec(name, fn, default_gate, config, deadband, _first_line(fn)))
        return fn
    return decorate


def setpoint_gate(name: str):
    """Register a rule for *when* the setpoint may move."""
    def decorate(fn):
        _register(GATES, "setpoint gate", name, GateSpec(name, fn, _first_line(fn)))
        return fn
    return decorate


def _load_plugins() -> None:
    # The plugin modules register themselves on import.
    from modules.scheduling import gates, planners, rules, speed_laws  # noqa: F401


def _lookup(table: dict[str, Any], kind: str, name: str, key: str) -> Any:
    _load_plugins()
    try:
        return table[name]
    except KeyError:
        known = ", ".join(sorted(table))
        raise SettingsError(f"{key}: unknown {kind} '{name}'. Known: {known}") from None


def _config_for(spec: Any, section: dict[str, dict[str, Any]], path: str) -> Any:
    raw = section.get(spec.name)
    if spec.config is None:
        if raw:
            raise SettingsError(f"{path}.{spec.name}: '{spec.name}' takes no parameters")
        return None
    return build_section(spec.config, raw or {}, f"{path}.{spec.name}")


# ---------------------------------------------------------------------------
# Bound plugins: the only objects the runtime and the sandbox call
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class BoundPlanner:
    """A planner with its config applied. Rules are expanded into a full ranking by applying
    the rule to the remaining jobs until none is left, so the loop can fall back to the next
    job when the first no longer validates — the same contract as a sequence planner."""

    spec: PlannerSpec
    cfg: Any = None

    @property
    def name(self) -> str:
        return self.spec.name

    def __call__(self, jobs: list[Job], ctx: PlanContext) -> list[ScheduledPick]:
        args = (ctx,) if self.cfg is None else (ctx, self.cfg)
        if self.spec.kind == "planner":
            return self.spec.fn(jobs, *args)
        remaining = [job for job in jobs if job.feasible]
        order: list[ScheduledPick] = []
        while remaining:
            chosen = self.spec.fn(remaining, *args)
            if chosen is None:
                break
            order.append(_as_scheduled(chosen, ctx))
            remaining = [job for job in remaining if job is not chosen]
        return order


def _as_scheduled(job: Job, ctx: PlanContext) -> ScheduledPick:
    intercept = job.intercept
    grab = intercept.pick_time if intercept is not None else float("inf")
    return ScheduledPick(job, ctx.now, grab, ctx.now + job.processing, job.processing, intercept)


@dataclass(frozen=True)
class BoundSpeedLaw:
    spec: SpeedLawSpec
    gate: GateSpec
    cfg: Any = None

    @property
    def name(self) -> str:
        return self.spec.name

    def __call__(self, view: SpeedView) -> SpeedDecision:
        if self.cfg is None:
            return self.spec.fn(view)
        return self.spec.fn(view, self.cfg)

    def gate_open(self, event: str, since_last_s: float, period_s: float) -> bool:
        return self.gate.fn(event, since_last_s, period_s)

    @property
    def adaptive(self) -> bool:
        """False for a law that never moves the belt (its gate is `never`)."""
        return self.gate.name != "never"


def get_planner(name: str, planners_config: dict[str, dict[str, Any]] | None = None,
                key: str = "scheduling.planner") -> BoundPlanner:
    spec = _lookup(PLANNERS, "planner", name, key)
    return BoundPlanner(spec, _config_for(spec, planners_config or {}, "scheduling.planners"))


def get_speed_law(name: str, laws_config: dict[str, dict[str, Any]] | None = None,
                  gate: str | None = None) -> BoundSpeedLaw:
    spec = _lookup(SPEED_LAWS, "speed law", name, "speed.law")
    gate_spec = _lookup(GATES, "setpoint gate", gate or spec.default_gate, "speed.setpoint_gate")
    return BoundSpeedLaw(spec, gate_spec, _config_for(spec, laws_config or {}, "speed.laws"))


def validate_plugin_config(settings: Any) -> None:
    """Start-up check: the selected plugins exist and every plugin section parses."""
    _load_plugins()
    get_planner(settings.scheduling.planner, settings.scheduling.planners)
    get_speed_law(settings.speed.law, settings.speed.laws, settings.speed.setpoint_gate)
    for name in settings.scheduling.planners:
        _config_for(_lookup(PLANNERS, "planner", name, f"scheduling.planners.{name}"),
                    settings.scheduling.planners, "scheduling.planners")
    for name in settings.speed.laws:
        _config_for(_lookup(SPEED_LAWS, "speed law", name, f"speed.laws.{name}"),
                    settings.speed.laws, "speed.laws")


def catalogue() -> dict[str, list[dict[str, str]]]:
    """Every registered plugin as data (name, kind, first doc line), for the operator console."""
    _load_plugins()
    return {
        "planners": [{"name": s.name, "kind": s.kind, "doc": s.doc}
                     for s in sorted(PLANNERS.values(), key=lambda s: (s.kind, s.name))],
        "speed_laws": [{"name": s.name, "kind": s.default_gate, "doc": s.doc}
                       for s in sorted(SPEED_LAWS.values(), key=lambda s: s.name)],
        "gates": [{"name": s.name, "kind": "gate", "doc": s.doc}
                  for s in sorted(GATES.values(), key=lambda s: s.name)],
    }


def describe() -> str:
    """Every registered plugin, one line each (`python3 -m modules.scheduling`)."""
    _load_plugins()
    lines = ["Dispatch rules and planners (scheduling.planner):"]
    for spec in sorted(PLANNERS.values(), key=lambda s: (s.kind, s.name)):
        params = f"  [scheduling.planners.{spec.name}: {', '.join(f.name for f in dataclasses.fields(spec.config))}]" if spec.config else ""
        lines.append(f"  {spec.name:<22} {spec.kind:<8} {spec.doc}{params}")
    lines.append("Speed laws (speed.law):")
    for spec in sorted(SPEED_LAWS.values(), key=lambda s: s.name):
        params = f"  [speed.laws.{spec.name}: {', '.join(f.name for f in dataclasses.fields(spec.config))}]" if spec.config else ""
        lines.append(f"  {spec.name:<22} gate={spec.default_gate:<10} {spec.doc}{params}")
    lines.append("Setpoint gates (speed.setpoint_gate):")
    for spec in sorted(GATES.values(), key=lambda s: s.name):
        lines.append(f"  {spec.name:<22} {spec.doc}")
    return "\n".join(lines)
