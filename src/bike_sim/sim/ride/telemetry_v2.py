"""Immutable input-state forces for one physical integration interval."""

from dataclasses import dataclass
from math import isfinite
from types import MappingProxyType
from typing import Mapping

import numpy as np


@dataclass(frozen=True)
class ForceSample:
    """Forces and velocity evaluated together at the start of a MuJoCo step."""

    time_s: float
    qpos: np.ndarray
    qvel: np.ndarray
    components: Mapping[str, np.ndarray]

    def __post_init__(self) -> None:
        if not isfinite(self.time_s):
            raise ValueError("force sample time is not finite")
        for name in ("qpos", "qvel"):
            value = np.array(getattr(self, name), dtype=float, copy=True)
            if value.ndim != 1 or not np.isfinite(value).all():
                raise ValueError("force sample state shape or value mismatch")
            object.__setattr__(self, name, _frozen_copy(value))
        copied: dict[str, np.ndarray] = {}
        for name, force in self.components.items():
            value = np.array(force, dtype=float, copy=True)
            if value.shape != self.qvel.shape or not np.isfinite(value).all():
                raise ValueError("force sample shape or value mismatch")
            copied[name] = _frozen_copy(value)
        object.__setattr__(self, "components", MappingProxyType(copied))


def _frozen_copy(value: np.ndarray) -> np.ndarray:
    """Use immutable bytes as backing storage so write flags cannot be re-enabled."""
    return np.frombuffer(value.tobytes(), dtype=float).reshape(value.shape)


def component_powers(sample: ForceSample) -> dict[str, float]:
    """Instantaneous generalized power in watts at the sample's own velocity."""
    return {name: float(force @ sample.qvel) for name, force in sample.components.items()}


__all__ = ["ForceSample", "component_powers"]
