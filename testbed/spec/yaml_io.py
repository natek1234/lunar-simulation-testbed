"""One place that turns YAML dicts into frozen dataclasses (nested, tuple-ified)."""
import dataclasses
from pathlib import Path
from typing import Any, get_args, get_origin, get_type_hints

import yaml


def load_yaml(path: Path) -> dict:
    with Path(path).open(encoding="utf-8") as f:
        return yaml.safe_load(f)


def from_dict(cls, d: dict[str, Any]):
    """Recursively build a dataclass from a dict. Lists become tuples; nested
    dataclasses (including tuple[Dataclass, ...] and Optional[Dataclass]) recurse.
    Unknown keys are an error, so a typo in a YAML file fails loudly."""
    hints = get_type_hints(cls)
    fields = {f.name for f in dataclasses.fields(cls)}
    unknown = set(d) - fields
    if unknown:
        raise ValueError(f"{cls.__name__}: unknown keys {sorted(unknown)}")
    kwargs = {}
    for name, value in d.items():
        kwargs[name] = _coerce(hints[name], value)
    return cls(**kwargs)


def _coerce(hint, value):
    origin = get_origin(hint)
    args = get_args(hint)
    if value is None:
        return None
    if hint is dict:
        return dict(value)
    if origin is None and dataclasses.is_dataclass(hint):
        return from_dict(hint, value)
    if origin is tuple:
        inner = args[0] if args else Any
        return tuple(_coerce(inner, v) for v in value)
    if origin is not None and type(None) in args:  # Optional[X]
        real = next(a for a in args if a is not type(None))
        return _coerce(real, value)
    if hint is float:
        return float(value)
    return value
