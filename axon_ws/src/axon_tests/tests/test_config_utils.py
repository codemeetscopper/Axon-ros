from dataclasses import dataclass

import pytest

from axon_utils.config import ConfigError, dataclass_from_dict, validate_dataclass


@dataclass
class SampleConfig:
    foo: int
    bar: str


def test_dataclass_from_dict_valid() -> None:
    config = dataclass_from_dict(SampleConfig, {"foo": 1, "bar": "x"})
    validate_dataclass(config)
    assert config.foo == 1


def test_dataclass_from_dict_extra_key() -> None:
    with pytest.raises(ConfigError):
        dataclass_from_dict(SampleConfig, {"foo": 1, "bar": "x", "extra": 2})
