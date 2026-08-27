"""Import bridge: load the delta-robot repo and the scheduling sandbox side by side.

The sandbox lives in a sibling directory whose name contains spaces and whose
modules use bare top-level names (`conveyor`, `scheduler`, `kinetic_control`)
that collide with the robot repo's `modules.*` package.  Both are loaded here
under explicit aliases so the comparison scripts can hold references to the two
implementations at once without either shadowing the other.

Nothing in this package writes to either repository.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import ModuleType

REPO_ROOT = Path(__file__).resolve().parent.parent
SANDBOX_ROOT = REPO_ROOT.parent / "python for scheduling"

if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


def _load_sandbox_module(name: str, alias: str) -> ModuleType:
    """Load one sandbox file under `alias` so it cannot shadow the robot repo.

    The sandbox modules import each other by bare name (`import conveyor`), so
    each alias is *also* registered under its bare name while the sandbox is
    being loaded.  They are plain-stdlib modules, so this is safe as long as the
    robot repo never uses those top-level names -- it doesn't; it uses
    `modules.conveyor` / `modules.scheduler`.
    """
    path = SANDBOX_ROOT / f"{name}.py"
    if not path.is_file():
        raise FileNotFoundError(f"sandbox module not found: {path}")
    spec = importlib.util.spec_from_file_location(alias, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[alias] = module
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


sandbox_kinetic = _load_sandbox_module("kinetic_control", "sandbox_kinetic_control")
sandbox_conveyor = _load_sandbox_module("conveyor", "sandbox_conveyor")
sandbox_scheduler = _load_sandbox_module("scheduler", "sandbox_scheduler")

# Drop the bare-name aliases again: from here on the robot repo owns the
# import namespace, and `modules.conveyor` must not resolve to the sandbox.
for _bare in ("kinetic_control", "conveyor", "scheduler"):
    sys.modules.pop(_bare, None)

import modules.scheduler as robot_scheduler  # noqa: E402
import modules.conveyor as robot_conveyor  # noqa: E402
from modules.EthernetCom import load_config  # noqa: E402


def robot_settings() -> "robot_scheduler.SchedulerSettings":
    return robot_scheduler.SchedulerSettings.from_config(load_config())


def sandbox_config() -> dict:
    import yaml

    with open(SANDBOX_ROOT / "config.yaml", "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


__all__ = [
    "REPO_ROOT",
    "SANDBOX_ROOT",
    "sandbox_kinetic",
    "sandbox_conveyor",
    "sandbox_scheduler",
    "robot_scheduler",
    "robot_conveyor",
    "robot_settings",
    "sandbox_config",
    "load_config",
]
