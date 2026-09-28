"""Immutable SI-unit snapshots of physical wheel contact patches."""

from dataclasses import dataclass
from math import isfinite
from numbers import Integral

import numpy as np


def _frozen_vector(value: np.ndarray, name: str) -> np.ndarray:
    vector = np.array(value, dtype=float, copy=True)
    if vector.shape != (3,) or not np.isfinite(vector).all():
        raise ValueError(f"{name} must be a finite 3D vector")
    # A read-only view of owned mutable memory can have its flag re-enabled. Immutable
    # bytes keep snapshots detached even from callers holding MuJoCo array views.
    return np.frombuffer(vector.tobytes(), dtype=float)


@dataclass(frozen=True)
class ContactPatch:
    """One X-Z wheel contact, with normal and longitudinal force kept separate."""

    point_m: np.ndarray
    normal: np.ndarray
    normal_load_n: float
    tangent_force_n: float
    slip_mps: float
    couple_world_nm: np.ndarray | None = None
    native_world_force_n: np.ndarray | None = None
    source_geom: str = "terrain"

    def __post_init__(self) -> None:
        point = _frozen_vector(self.point_m, "contact point")
        normal = _frozen_vector(self.normal, "contact normal")
        if abs(np.linalg.norm(normal) - 1.0) > 1e-8 or abs(normal[1]) > 1e-8:
            raise ValueError("expected a unit normal in the X-Z plane")
        scalars = tuple(float(value) for value in (
            self.normal_load_n, self.tangent_force_n, self.slip_mps
        ))
        if not all(isfinite(value) for value in scalars) or scalars[0] < 0.0:
            raise ValueError("invalid contact load, force or slip")
        if self.source_geom not in ("terrain", "catch_plane"):
            raise ValueError("contact source must be terrain or catch_plane")
        couple = _frozen_vector(
            np.zeros(3) if self.couple_world_nm is None else self.couple_world_nm,
            "contact couple",
        )
        native_force = (
            None if self.native_world_force_n is None
            else _frozen_vector(self.native_world_force_n, "native world force")
        )
        object.__setattr__(self, "point_m", point)
        object.__setattr__(self, "normal", normal)
        object.__setattr__(self, "couple_world_nm", couple)
        object.__setattr__(self, "native_world_force_n", native_force)
        for name, value in zip(
            ("normal_load_n", "tangent_force_n", "slip_mps"), scalars
        ):
            object.__setattr__(self, name, value)

    @property
    def tangent(self) -> np.ndarray:
        return _frozen_vector(np.array([self.normal[2], 0.0, -self.normal[0]]), "tangent")

    @property
    def working_surface(self) -> bool:
        return self.source_geom == "terrain"

    @property
    def world_force_n(self) -> np.ndarray:
        if self.native_world_force_n is not None:
            return self.native_world_force_n
        return _frozen_vector(
            self.normal_load_n * self.normal + self.tangent_force_n * self.tangent,
            "contact force",
        )


@dataclass(frozen=True)
class WheelContactSnapshot:
    """Physical contact state for one wheel and one stable simulation interval."""

    time_s: float
    patches: tuple[ContactPatch, ...]
    geometric_contact: bool
    interval_id: int = 0
    backend: str = "native_reference"
    wheel_axis_m: np.ndarray | None = None

    def __post_init__(self) -> None:
        time_s = float(self.time_s)
        if not isfinite(time_s) or time_s < 0.0:
            raise ValueError("contact snapshot needs finite nonnegative time")
        if (
            isinstance(self.interval_id, bool)
            or not isinstance(self.interval_id, Integral)
            or self.interval_id < 0
        ):
            raise ValueError("contact snapshot needs a nonnegative interval ID")
        if not isinstance(self.backend, str) or not self.backend:
            raise ValueError("contact snapshot needs a backend")
        patches = tuple(self.patches)
        if not all(isinstance(patch, ContactPatch) for patch in patches):
            raise ValueError("contact snapshot needs ContactPatch entries")
        axis = _frozen_vector(
            np.zeros(3) if self.wheel_axis_m is None else self.wheel_axis_m,
            "wheel axis",
        )
        object.__setattr__(self, "time_s", time_s)
        object.__setattr__(self, "interval_id", int(self.interval_id))
        object.__setattr__(self, "patches", patches)
        object.__setattr__(self, "geometric_contact", bool(self.geometric_contact))
        object.__setattr__(self, "wheel_axis_m", axis)

    @property
    def loaded_contact(self) -> bool:
        return self.normal_load_n > 0.0

    @property
    def road_loaded_contact(self) -> bool:
        """Only working-road load may re-enable a physical drive controller."""
        return any(
            patch.working_surface and patch.normal_load_n > 0.0
            for patch in self.patches
        )

    @property
    def normal_load_n(self) -> float:
        return sum((patch.normal_load_n for patch in self.patches), 0.0)

    @property
    def world_force_n(self) -> np.ndarray:
        return _frozen_vector(
            sum((patch.world_force_n for patch in self.patches), np.zeros(3)),
            "wheel force",
        )

    @property
    def vertical_force_n(self) -> float:
        return float(self.world_force_n[2])

    @property
    def force_world_n(self) -> np.ndarray:
        """Compatibility name for the same world-force resultant."""
        return self.world_force_n

    @property
    def vertical_resultant_n(self) -> float:
        """Compatibility name for total world-Z force, including friction."""
        return self.vertical_force_n

    @property
    def normal_vertical_n(self) -> float:
        return sum(
            (patch.normal_load_n * float(patch.normal[2]) for patch in self.patches),
            0.0,
        )

    @property
    def tangent_force_n(self) -> float:
        return sum((patch.tangent_force_n for patch in self.patches), 0.0)

    @property
    def wheel_axis_moment_nm(self) -> float:
        return sum((
            float(np.cross(patch.point_m - self.wheel_axis_m, patch.world_force_n)[1]
                  + patch.couple_world_nm[1])
            for patch in self.patches
        ), 0.0)

    @property
    def effective_radius_m(self) -> float:
        if not self.patches:
            return 0.0
        weights = [patch.normal_load_n for patch in self.patches]
        if not any(weight > 0.0 for weight in weights):
            weights = [1.0] * len(self.patches)
        return max(0.0, sum(
            weight * -float(np.dot(patch.point_m - self.wheel_axis_m, patch.normal))
            for patch, weight in zip(self.patches, weights)
        ) / sum(weights))

    @property
    def slip_mps(self) -> float:
        if not self.patches:
            return 0.0
        weights = [patch.normal_load_n for patch in self.patches]
        if not any(weight > 0.0 for weight in weights):
            weights = [1.0] * len(self.patches)
        return sum(patch.slip_mps * weight for patch, weight in zip(self.patches, weights)) / sum(weights)


__all__ = ["ContactPatch", "WheelContactSnapshot"]
