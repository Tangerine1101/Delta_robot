"""Arm models the sandbox can simulate, by name.

A model is registered with `@arm_model("<name>", config=<dataclass>)`: a builder
`(settings, cfg) -> ArmModel`, where `settings` is the cell (robot, conveyor, object types,
pick gate) of the model's YAML file and `cfg` its own `model:` section. `delta` is the real
cell's model from `modules/core/delta.py` — the one the robot plans with.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

from modules.core.arm_model import ArmModel
from modules.core.delta import DeltaArm
from modules.core.frames import ConveyorFrame
from modules.settings import Settings, build_section

ARM_MODELS: dict[str, tuple[Callable[[Settings, Any], ArmModel], type]] = {}


def arm_model(name: str, *, config: type):
    def decorate(fn):
        ARM_MODELS[name] = (fn, config)
        return fn
    return decorate


def build_arm(name: str, settings: Settings, params: dict | None, path: str) -> ArmModel:
    _load()
    try:
        builder, cfg_cls = ARM_MODELS[name]
    except KeyError:
        raise SystemExit(f"{path}: unknown arm model '{name}'. Known: {', '.join(sorted(ARM_MODELS))}") from None
    return builder(settings, build_section(cfg_cls, params or {}, path))


def _load() -> None:
    from sandbox.models import point_mover  # noqa: F401


@dataclass(frozen=True)
class DeltaConfig:
    # The robot budgets half a gate poll and half a perception tick of staleness; the
    # sandbox evaluates the gate exactly, so a faithful model sets this to 0.
    gate_sampling_latency_s: float = 0.0


@arm_model("delta", config=DeltaConfig)
def delta(settings: Settings, cfg: DeltaConfig) -> ArmModel:
    """The cell's delta arm: 7-point templates and the PLC time model."""
    frame = ConveyorFrame.from_settings(settings.conveyor)
    return DeltaArm(settings.robot, settings.pick_gate, frame, settings.conveyor.workspace_window_uv,
                    cfg.gate_sampling_latency_s)
