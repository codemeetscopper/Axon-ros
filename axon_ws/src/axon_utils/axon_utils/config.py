"""Configuration loading and validation helpers."""
from __future__ import annotations

from dataclasses import asdict, dataclass, fields, is_dataclass
from pathlib import Path
from typing import Any, Dict, Type, TypeVar

import yaml

T = TypeVar("T")


class ConfigError(ValueError):
    """Raised when configuration validation fails."""


def load_yaml(path: str | Path) -> Dict[str, Any]:
    """Load a YAML file into a dictionary."""
    with Path(path).open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ConfigError(f"Expected mapping in YAML file: {path}")
    return data


def dataclass_from_dict(schema: Type[T], data: Dict[str, Any]) -> T:
    """Create a dataclass instance from a dict, validating keys."""
    if not is_dataclass(schema):
        raise ConfigError(f"Schema {schema} must be a dataclass")
    valid_keys = {field.name for field in fields(schema)}
    extra_keys = set(data) - valid_keys
    if extra_keys:
        raise ConfigError(f"Unknown keys in config: {sorted(extra_keys)}")
    return schema(**data)


def validate_dataclass(instance: Any) -> None:
    """Validate a dataclass by invoking its __post_init__ checks."""
    if not is_dataclass(instance):
        raise ConfigError("Instance is not a dataclass")
    dataclass_fields = {field.name for field in fields(instance)}
    missing = [name for name in dataclass_fields if getattr(instance, name, None) is None]
    if missing:
        raise ConfigError(f"Missing required fields: {missing}")


def dataclass_to_dict(instance: Any) -> Dict[str, Any]:
    """Convert a dataclass to a dict for logging or serialization."""
    if not is_dataclass(instance):
        raise ConfigError("Instance is not a dataclass")
    return asdict(instance)


@dataclass
class RetryPolicy:
    initial_backoff_s: float
    max_backoff_s: float

    def __post_init__(self) -> None:
        if self.initial_backoff_s <= 0:
            raise ConfigError("initial_backoff_s must be > 0")
        if self.max_backoff_s < self.initial_backoff_s:
            raise ConfigError("max_backoff_s must be >= initial_backoff_s")
