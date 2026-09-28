"""Mode selection and shared integration settings for ride simulations."""

from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class SimulationPhysicsConfig:
    """Resolved physics mode and the source of forward drive torque."""

    physics_mode: str = "legacy"
    drive_mode: str | None = None
    timestep_s: float = 0.0005
    pitch_assist: bool | None = None

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
