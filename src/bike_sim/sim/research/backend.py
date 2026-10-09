"""Structural research boundary. It does not select or fall back to physics."""
from os import PathLike
from pathlib import Path
from typing import Any, Protocol
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.research.sensors import SensorObservation
from bike_sim.sim.research.environment import ResearchStep


class ResearchBackend(Protocol):
    backend: str
    seed: int
    sim: Any
    config: Any
    sensor_config: Any
    observation: SensorObservation
    demand_nm: float | None
    metadata: dict
    execution: dict
    run_metadata: dict
    terminated: bool
    truncated: bool
    reason: str | None
    error: str | None
    numerically_valid: bool

    @property
    def model_valid(self) -> bool: ...
    @property
    def done(self) -> bool: ...
    @property
    def control_pending(self) -> bool: ...
    @property
    def trace(self) -> list[ResearchStep]: ...
    def begin_control(self, control: RideControl, *, front_brake_demand: float = 0.,
                      rear_brake_demand: float = 0.) -> None: ...
    def advance_control(self, *, wall_budget_s: float | None = None) -> ResearchStep | None: ...
    def step(self, control: RideControl, *, front_brake_demand: float = 0.,
             rear_brake_demand: float = 0.) -> ResearchStep: ...
    def reset(self, *, seed: int | None = None) -> SensorObservation: ...
    def save(self, directory: str | PathLike[str], *, overwrite: bool = False) -> Path: ...
    def current_metadata(self) -> dict: ...
    def stop(self) -> None: ...
    def fail_policy(self, error: Exception) -> None: ...
    def close(self) -> None: ...
