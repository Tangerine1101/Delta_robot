"""The sandbox run configuration: `sandbox/config.yaml` plus one model file.

`config.yaml` holds what a simulation needs — clock, feeder, which arm model, and the same
`scheduling:` / `speed:` sections as `modules/config.yaml`, so a plugin configuration moves
between the two files by copy-paste. The cell itself (robot, conveyor, object types, pick
gate) comes from `sandbox/models/<model>.yaml`, which also carries the model's own parameters
and the plant's deviations from the model. Everything is loaded with the same strict loader
as the robot's config (modules/settings.py): unknown keys are errors.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from modules.config_io import read_config
from modules.settings import (
    SchedulingSettings, Settings, SettingsError, SpeedSettings, build_section, settings_from_dict,
    with_overrides,
)

SANDBOX_DIR = Path(__file__).resolve().parent
CONFIG_PATH = SANDBOX_DIR / "config.yaml"
MODELS_DIR = SANDBOX_DIR / "models"

# The cell sections a model file carries (the same shape as in modules/config.yaml).
CELL_SECTIONS = ("robot", "conveyor", "object_types", "pick_gate")
_NO_PLC = {"omron": {"ip": "0.0.0.0", "port": 0}, "siemens": {"ip": "0.0.0.0", "port": 0}}


@dataclass(frozen=True)
class SimConfig:
    duration_s: float = 300.0
    dt_s: float = 0.005             # belt and arm clock step
    replan_period_s: float = 0.05   # the robot re-plans every poll while the arm is free
    seed: int = 1
    capture_tolerance_mm: float = 12.7   # contact within this of the part centre grips it
    lost_margin_mm: float = 30.0    # a part this far past u_max is off the belt
    stall_timeout_s: float = 10.0   # gate wait before the pick is aborted


@dataclass(frozen=True)
class ArmConfig:
    model: str = "delta"            # sandbox/models/<model>.yaml
    plant: dict[str, Any] = field(default_factory=dict)   # extra dotted overrides for the plant


@dataclass(frozen=True)
class RunConfig:
    sim: SimConfig = SimConfig()
    feeder: dict[str, Any] = field(default_factory=lambda: {"kind": "poisson"})
    arm: ArmConfig = ArmConfig()
    scheduling: SchedulingSettings = SchedulingSettings()
    speed: SpeedSettings = SpeedSettings()


@dataclass(frozen=True)
class ModelFile:
    """One `sandbox/models/<name>.yaml`."""

    builder: str                    # registered arm-model name
    params: dict[str, Any]          # the builder's own parameters
    plant: dict[str, Any]           # dotted overrides turning the model into the plant
    cell: dict[str, Any]            # robot / conveyor / object_types / pick_gate


def load_run_config(path: str | Path | None = None, overrides: dict[str, Any] | None = None) -> dict[str, Any]:
    """The raw run config with dotted overrides applied (validated by `build_run`)."""
    raw = read_config(path or CONFIG_PATH)
    for dotted, value in (overrides or {}).items():
        node = raw
        parts = dotted.split(".")
        for key in parts[:-1]:
            node = node.setdefault(key, {})
        node[parts[-1]] = value
    return raw


def load_model_file(name: str) -> ModelFile:
    path = MODELS_DIR / f"{name}.yaml"
    if not path.is_file():
        known = ", ".join(sorted(p.stem for p in MODELS_DIR.glob("*.yaml")))
        raise SystemExit(f"arm.model: no model file {path} (known: {known})")
    raw = read_config(path)
    unknown = set(raw) - {"model", "params", "plant", "cell"}
    if unknown:
        raise SettingsError(f"{path.name}: unknown top-level keys {sorted(unknown)}")
    return ModelFile(str(raw.get("model", name)), dict(raw.get("params") or {}),
                     dict(raw.get("plant") or {}), dict(raw.get("cell") or {}))


def cell_settings(model: ModelFile, run: RunConfig) -> Settings:
    """A full Settings for the simulated cell: the model file's cell, the run's plugins."""
    unknown = set(model.cell) - set(CELL_SECTIONS)
    if unknown:
        raise SettingsError(f"cell: unexpected sections {sorted(unknown)}; allowed {CELL_SECTIONS}")
    settings = settings_from_dict({"plc": _NO_PLC, **model.cell})
    return with_overrides(settings, {"scheduling": run.scheduling, "speed": run.speed})


def build_run(raw: dict[str, Any]) -> RunConfig:
    return build_section(RunConfig, raw, "")


def plant_of(model: ModelFile, run: RunConfig) -> tuple[dict[str, Any], dict[str, Any]]:
    """(cell overrides, params overrides) of the plant: the model file's `plant:` section,
    then the run's `arm.plant`. Keys under `params.` go to the model's own parameters."""
    cell: dict[str, Any] = {}
    params: dict[str, Any] = {}
    for dotted, value in {**model.plant, **run.arm.plant}.items():
        if dotted.startswith("params."):
            params[dotted[len("params."):]] = value
        else:
            cell[dotted] = value
    return cell, params
