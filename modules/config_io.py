"""Read/write `modules/config.yaml` without losing its comments.

Every reader goes through `read_config` (plain dicts and lists out) and every writer
(camera_calibrate.py, calibrate_everything.py) through `write_config`, which merges the new
values into the existing round-trip document: unchanged values keep their comments, quoting and
flow style, so a calibration save does not erase the key documentation. No PLC-stack imports —
safe to use from standalone tools.
"""

from __future__ import annotations

import io
import os
from pathlib import Path
from typing import Any

from ruamel.yaml import YAML
from ruamel.yaml.comments import CommentedMap, CommentedSeq
from ruamel.yaml.error import YAMLError

CONFIG_PATH = Path(__file__).parent / "config.yaml"
LEGACY_JSON_PATH = Path(__file__).parent / "config.json"


class ConfigError(ValueError):
    """The config file exists but cannot be used."""


def _yaml() -> YAML:
    yaml = YAML(typ="rt")
    yaml.preserve_quotes = True
    yaml.width = 4096
    yaml.indent(mapping=2, sequence=4, offset=2)
    yaml.representer.add_representer(
        type(None), lambda rep, _: rep.represent_scalar("tag:yaml.org,2002:null", "null"))
    return yaml


def _to_plain(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _to_plain(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_to_plain(item) for item in value]
    if isinstance(value, str):
        return str(value)
    if isinstance(value, bool) or value is None:
        return value
    if isinstance(value, int):
        return int(value)
    if isinstance(value, float):
        return float(value)
    return value


def _load_document(path: Path) -> Any:
    try:
        with open(path, encoding="utf-8") as handle:
            return _yaml().load(handle)
    except YAMLError as exc:
        raise ConfigError(f"{path}: invalid YAML: {exc}") from exc


def read_config(path: Path | str = CONFIG_PATH) -> dict[str, Any]:
    """Parse the config file into plain dicts/lists. Raises FileNotFoundError on a missing
    file and ConfigError on invalid YAML or a left-over config.json."""
    path = Path(path)
    if not path.exists() and path == CONFIG_PATH and LEGACY_JSON_PATH.exists():
        raise ConfigError(f"{LEGACY_JSON_PATH} was replaced by {CONFIG_PATH}; move its values there")
    data = _load_document(path)
    if not isinstance(data, dict):
        raise ConfigError(f"{path}: top level must be a mapping")
    return _to_plain(data)


def _new_node(value: Any, like: Any = None) -> Any:
    """A round-trip node for `value`, copying the flow style of the node it replaces."""
    if isinstance(value, dict):
        node = CommentedMap()
        for key, item in value.items():
            node[key] = _new_node(item)
    elif isinstance(value, list):
        node = CommentedSeq(_new_node(item) for item in value)
    else:
        return value
    if like is not None and hasattr(like, "fa") and like.fa.flow_style():
        node.fa.set_flow_style()
    elif isinstance(value, list) and like is None and all(not isinstance(i, dict) for i in value):
        node.fa.set_flow_style()
    return node


def _append(target: CommentedMap, key: str, node: Any) -> None:
    """Append a key, moving the comment block that trails the previous last key (usually the
    header of the next section) below the new key so the key stays inside its own section."""
    last = next(reversed(target), None) if target else None
    trailing = target.ca.items.get(last) if last is not None else None
    target[key] = node
    if trailing and len(trailing) > 2 and trailing[2] is not None and "\n\n" in trailing[2].value:
        eol, _, block = trailing[2].value.partition("\n")
        trailing[2].value = eol + "\n"
        target.ca.items[key] = [None, None, type(trailing[2])("\n" + block, trailing[2].start_mark, None), None]


def _deep_update(target: CommentedMap, new: dict[str, Any]) -> None:
    for key in [key for key in target if key not in new]:
        del target[key]
    for key, value in new.items():
        if key in target and isinstance(target[key], dict) and isinstance(value, dict):
            _deep_update(target[key], value)
        elif key in target and _to_plain(target[key]) == value:
            continue
        elif key in target:
            target[key] = _new_node(value, target[key])
        else:
            _append(target, key, _new_node(value))


def write_config(data: dict[str, Any], path: Path | str = CONFIG_PATH) -> None:
    """Merge `data` into the existing file, verify the rendering parses back to exactly
    `data`, then replace the file atomically."""
    path = Path(path)
    document = _load_document(path) if path.exists() else CommentedMap()
    if not isinstance(document, CommentedMap):
        document = CommentedMap()
    _deep_update(document, data)

    buffer = io.StringIO()
    _yaml().dump(document, buffer)
    text = buffer.getvalue()
    if _to_plain(_yaml().load(text)) != data:
        raise RuntimeError("config rendering does not round-trip; refusing to write")

    tmp = path.with_name(path.name + ".tmp")
    with open(tmp, "w", encoding="utf-8") as handle:
        handle.write(text)
    os.replace(tmp, path)
