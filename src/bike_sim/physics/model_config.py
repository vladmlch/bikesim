"""Mode selection and shared integration settings for ride simulations."""

from dataclasses import dataclass, field
from math import isfinite

from bike_sim.physics.physical_config import (
    TireBackendConfig, PhysicalDriveConfig, ResistanceConfig, ArticulatedConfig,
)
from bike_sim.physics.checks import scalar


@dataclass(frozen=True)
class EndStopConfig:
    """Synthetic stop calibration shared by the ride model and force path."""

    reference_force_n: float = 7000.0
    reference_deflection_m: float = 0.010
    damping_n_s_m: float = 500.0
    overtravel_m: float = 0.010
    provenance: str = "synthetic"

    def __post_init__(self) -> None:
        if (
            not all(isfinite(x) for x in (
                self.reference_force_n, self.reference_deflection_m,
                self.damping_n_s_m, self.overtravel_m,
            ))
            or self.reference_force_n <= 0
            or self.reference_deflection_m <= 0
            or self.damping_n_s_m < 0
            or self.overtravel_m <= 0
        ):
            raise ValueError("invalid end-stop calibration")
        if self.overtravel_m < self.reference_deflection_m:
            raise ValueError("end-stop overtravel must reach the reference deflection")

    @property
    def stiffness_n_m(self) -> float:
        return self.reference_force_n / self.reference_deflection_m


@dataclass(frozen=True)
class SimulationPhysicsConfig:
    """Resolved physics mode and the source of forward drive torque."""

    physics_mode: str = "legacy"
    drive_mode: str | None = None
    timestep_s: float = 0.0005
    pitch_assist: bool | None = None
    end_stops: EndStopConfig = field(default_factory=EndStopConfig)
    tires: TireBackendConfig = field(default_factory=TireBackendConfig)
    drive: PhysicalDriveConfig = field(default_factory=PhysicalDriveConfig)
    resistance: ResistanceConfig = field(default_factory=ResistanceConfig)
    articulated: ArticulatedConfig = field(default_factory=ArticulatedConfig)
    initial_speed_mps: float = 0.0

    def __post_init__(self) -> None:
        if self.physics_mode not in {"legacy", "physical"}:
            raise ValueError("unknown physics_mode")
        if not isfinite(self.timestep_s) or self.timestep_s <= 0:
            raise ValueError("timestep_s must be finite and positive")

        legacy = self.physics_mode == "legacy"
        if self.drive_mode is None:
            object.__setattr__(
                self, "drive_mode", "ideal_speed_control" if legacy else "coast"
            )
        if self.pitch_assist is None:
            object.__setattr__(self, "pitch_assist", legacy)
        if self.drive_mode not in {
            "coast",
            "ideal_speed_control",
            "crank_effort",
            "articulated_effort",
        }:
            raise ValueError("unknown drive_mode")
        if not legacy and self.pitch_assist:
            raise ValueError("external pitch assist is forbidden in physical mode")

        scalar(self.initial_speed_mps, "initial speed")
        scalar(self.timestep_s, "timestep", positive=True)
        if not isinstance(self.pitch_assist, bool):
            raise ValueError("pitch_assist must be a bool")
        for name, cls in (("end_stops", EndStopConfig), ("tires", TireBackendConfig),
                          ("drive", PhysicalDriveConfig), ("resistance", ResistanceConfig),
                          ("articulated", ArticulatedConfig)):
            if not isinstance(getattr(self, name), cls):
                raise ValueError(f"{name} needs an immutable {cls.__name__}")
