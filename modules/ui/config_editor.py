"""The configuration editor of the operator console: `modules/config.yaml` as a flat list of
dotted keys the web page can show, edit and write back.

The page shows every key of `Settings` (a key missing from the file shows its default and is
flagged), the file's own end-of-line comment as its help text, which keys the file holds that
the running console does not use yet (`pending`), and which edits only take effect after a
restart (`RESTART_PREFIXES`: the PLC link, the pose stream, the run log and the simulator are
built once at start-up). A change is checked with the strict loader of `modules/settings.py`
before anything is written, and written with `config_io.write_config`, which keeps the file's
comments.

No PLC-stack imports: the functions work on plain values and are tested without hardware.
"""

from __future__ import annotations

import copy
import dataclasses
from pathlib import Path
from typing import Any

from modules.config_io import read_comments, read_config
from modules.settings import Settings, settings_from_dict, with_overrides

# Built once at start-up: a new value reaches them only after restarting the console.
RESTART_PREFIXES = ("plc.", "pose_stream.", "logging.", "plc_sim.", "interface.port",
                    "interface.mjpeg_fps", "conveyor.position_scale_mm")
# Edited elsewhere in the page (the Teach points card).
HIDDEN_PREFIXES = ("interface.teach_points",)


def _plain(value: Any) -> Any:
    if dataclasses.is_dataclass(value) and not isinstance(value, type):
        return {f.name: _plain(getattr(value, f.name)) for f in dataclasses.fields(value)}
    if isinstance(value, dict):
        return {str(k): _plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(v) for v in value]
    return value


def flatten(value: Any, prefix: str = "") -> dict[str, Any]:
    """{dotted key: leaf} of a settings object or a plain tree. A list, an empty mapping and a
    scalar are leaves; the keys the editor does not show are left out."""
    tree = _plain(value)
    out: dict[str, Any] = {}

    def walk(node: Any, path: str) -> None:
        if isinstance(node, dict) and node:
            for key, item in node.items():
                walk(item, f"{path}.{key}" if path else str(key))
        else:
            out[path] = node

    walk(tree, prefix)
    return {k: v for k, v in out.items() if not is_hidden(k)}


def is_hidden(key: str) -> bool:
    return any(key == p or key.startswith(p + ".") for p in HIDDEN_PREFIXES)


def needs_restart(key: str) -> bool:
    return any(key.startswith(p) for p in RESTART_PREFIXES)


def differing(a: dict[str, Any], b: dict[str, Any]) -> list[str]:
    """The keys whose values differ between two flat views (a key on one side only counts)."""
    return sorted(k for k in a.keys() | b.keys() if a.get(k) != b.get(k))


def set_path(raw: dict[str, Any], dotted: str, value: Any) -> None:
    """Set `raw[a][b][c] = value` for "a.b.c", creating the mappings on the way."""
    if not dotted or is_hidden(dotted):
        raise ValueError(f"'{dotted}' cannot be edited here")
    parts = dotted.split(".")
    node = raw
    for part in parts[:-1]:
        child = node.get(part)
        if child is None:
            child = node[part] = {}
        if not isinstance(child, dict):
            raise ValueError(f"{dotted}: '{part}' is a value, not a section")
        node = child
    node[parts[-1]] = value


def apply_changes(raw: dict[str, Any], changes: dict[str, Any]) -> dict[str, Any]:
    """A copy of the plain config tree with `changes` ({dotted key: value}) set in it."""
    tree = copy.deepcopy(raw)
    for dotted, value in changes.items():
        set_path(tree, str(dotted), value)
    return tree


def snapshot(path: Path, live: Settings, overrides: dict[str, Any], loaded_mtime: float | None,
             boot: Settings) -> dict[str, Any]:
    """Everything the Config tab shows: the file's values, what is not applied yet, what is
    masked by a `--set` override, and what needs a restart."""
    out: dict[str, Any] = {"path": str(path), "stale": is_stale(path, loaded_mtime)}
    try:
        raw = read_config(path)
        in_file = set(flatten(raw))
        file_settings = settings_from_dict(raw)
        out["file_error"] = None
    except (ValueError, OSError) as exc:        # the file on disk is broken: show what is running
        raw, in_file, file_settings = {}, set(), live
        out["file_error"] = str(exc)
    shown = with_overrides(file_settings, overrides) if overrides else file_settings
    file_flat, live_flat = flatten(file_settings), flatten(live)
    out.update(
        values=file_flat,
        in_file=sorted(in_file & file_flat.keys()),
        comments=read_comments(path) if out["file_error"] is None else {},
        pending=differing(flatten(shown), live_flat),
        masked=sorted(k for k in overrides if k in file_flat),
        restart_needed=[k for k in differing(flatten(boot), live_flat) if needs_restart(k)],
        restart_prefixes=list(RESTART_PREFIXES),
    )
    return out


def is_stale(path: Path, loaded_mtime: float | None) -> bool:
    try:
        return loaded_mtime is not None and Path(path).stat().st_mtime != loaded_mtime
    except OSError:
        return False
