"""Named generalized-force contributions for one physical simulation step."""

from types import MappingProxyType
from typing import Mapping, Optional

import numpy as np


class ForceAccumulator:
    """Keep independent force writers additive, with no input or output aliasing."""

    def __init__(self, nv: int) -> None:
        self.nv = nv
        self._components: dict[str, np.ndarray] = {}

    def clear(self) -> None:
        self._components.clear()

    def add(self, name: str, qfrc: np.ndarray) -> None:
        value = np.asarray(qfrc, dtype=float)
        if value.shape != (self.nv,) or not np.isfinite(value).all():
            raise ValueError("invalid generalized force")
        if name in self._components:
            raise ValueError("duplicate force component")
        self._components[name] = value.copy()

    @property
    def components(self) -> Mapping[str, np.ndarray]:
        copied = {name: value.copy() for name, value in self._components.items()}
        for value in copied.values():
            value.setflags(write=False)
        return MappingProxyType(copied)

    def component(self, name: str) -> Optional[np.ndarray]:
        """Return one named contribution as a read-only view, or None if absent.

        Unlike `components`, no per-entry copies are made; callers must not
        mutate the returned array and must not outlive the accumulator's state.
        """
        value = self._components.get(name)
        if value is None:
            return None
        view = value.view()
        view.setflags(write=False)
        return view

    def total(self) -> np.ndarray:
        result = np.zeros(self.nv)
        for value in self._components.values():
            result += value
        return result


__all__ = ["ForceAccumulator"]
