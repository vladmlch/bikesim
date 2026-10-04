"""Mode selection and shared integration settings for ride simulations."""

from dataclasses import dataclass, field
from math import isfinite

from bike_sim.physics.physical_config import (
    TireBackendConfig, PhysicalDriveConfig, ResistanceConfig, ArticulatedConfig,
)
from bike_sim.physics.checks import scalar
from bike_sim.physics.seated_climb import SeatedClimbConfig


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
    control_period_s: float = 0.005
    pitch_assist: bool | None = None
    end_stops: EndStopConfig = field(default_factory=EndStopConfig)
    tires: TireBackendConfig = field(default_factory=TireBackendConfig)
    drive: PhysicalDriveConfig = field(default_factory=PhysicalDriveConfig)
    resistance: ResistanceConfig = field(default_factory=ResistanceConfig)
    articulated: ArticulatedConfig = field(default_factory=ArticulatedConfig)
    seated_climb: SeatedClimbConfig = field(default_factory=SeatedClimbConfig)
    initial_speed_mps: float = 0.0
    initial_front_brake: float = 0.0
    initial_rear_brake: float = 0.0
    closure_time_constant_s: float = 0.001
    equilibrium_refine_after_s: float = 1.0
    equilibrium_refine_period_s: float = 3.0
    equilibrium_cache_enabled: bool = False

    def __post_init__(self) -> None:
        if self.physics_mode not in {"legacy", "physical"}:
            raise ValueError("unknown physics_mode")
        if not isfinite(self.timestep_s) or self.timestep_s <= 0:
            raise ValueError("timestep_s must be finite and positive")
        scalar(self.control_period_s, 'control period', positive=True)
        steps = round(self.control_period_s/self.timestep_s)
        if steps < 1 or abs(steps*self.timestep_s-self.control_period_s) > 1e-9:
            raise ValueError('control_period_s must be an integer multiple of timestep_s')

        for name in ('equilibrium_refine_after_s', 'equilibrium_refine_period_s'):
            scalar(getattr(self, name), name, positive=True)
        legacy = self.physics_mode == "legacy"
        scalar(self.closure_time_constant_s, "closure time constant", positive=True)
        if not legacy and self.closure_time_constant_s < 2.0*self.timestep_s:
            raise ValueError(
                f"closure_time_constant_s={self.closure_time_constant_s} must be at least "
                f"2*timestep_s={2.0*self.timestep_s}; otherwise MuJoCo silently changes closure stiffness"
            )
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
        if legacy and self.drive_mode in {'crank_effort','articulated_effort'}:
            raise ValueError('physical effort drive modes require physics_mode=physical')
        if not legacy and self.pitch_assist:
            raise ValueError("external pitch assist is forbidden in physical mode")

        scalar(self.initial_speed_mps, "initial speed")
        for name in ('initial_front_brake','initial_rear_brake'):
            value=scalar(getattr(self,name),name,minimum=0.)
            if value>1.: raise ValueError(f'{name} must lie in [0,1]')
        scalar(self.timestep_s, "timestep", positive=True)
        if not isinstance(self.pitch_assist, bool):
            raise ValueError("pitch_assist must be a bool")
        if not isinstance(self.equilibrium_cache_enabled, bool):
            raise ValueError("equilibrium_cache_enabled must be a bool")
        for name, cls in (("end_stops", EndStopConfig), ("tires", TireBackendConfig),
                          ("drive", PhysicalDriveConfig), ("resistance", ResistanceConfig),
                          ("articulated", ArticulatedConfig), ("seated_climb", SeatedClimbConfig)):
            if not isinstance(getattr(self, name), cls):
                raise ValueError(f"{name} needs an immutable {cls.__name__}")
        if self.seated_climb.enabled and (legacy or self.drive_mode != 'articulated_effort'):
            raise ValueError('seated climb requires the articulated physical effort drive')
        if self.drive.motor_clutch and self.drive_mode not in ('crank_effort','articulated_effort'):
            raise ValueError('drive.motor_clutch requires an effort drive mode')
        if self.drive.rotor_inertia_kgm2 > 0. and self.drive_mode not in ('crank_effort','articulated_effort'):
            raise ValueError('drive.rotor_inertia_kgm2 requires an effort drive mode')
        if self.drive.pedaling.reposition_on_stall and self.drive_mode != 'articulated_effort':
            raise ValueError('the reposition stall reflex requires articulated_effort')
