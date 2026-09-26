"""
Ride-Mode Runtime Package.

The per-step force writers of the free-rolling ride mode, plus the pieces that turn them into
a run: termination, the key map, the terminal HUD and the model livery. Each force writer is a
small object that reads MuJoCo state, asks a pure calculator for a force, and writes it into
`qfrc_applied` or `ctrl`; the orchestrator that drives them lives in `sim/ride_sim.py`.

`viewer.py` is deliberately absent from the re-exports below: it is the only module here that
imports the orchestrator at runtime, and importing it from the package would make
`sim/ride_sim.py` unimportable. Interactive callers import `bike_sim.sim.ride.viewer` directly.
"""

from bike_sim.sim.ride.braking import BrakeController
from bike_sim.sim.ride.contacts import TerrainContactQuery, TerrainContacts
from bike_sim.sim.ride.cruise import CruiseController
from bike_sim.sim.ride.forces import SuspensionForceApplier
from bike_sim.sim.ride.hud import RideHUD
from bike_sim.sim.ride.input import RideInputHandler
from bike_sim.sim.ride.livery import ModelLivery
from bike_sim.sim.ride.resistance import RollingResistance
from bike_sim.sim.ride.termination import (
    RunLimits,
    RunOutcome,
    RunTerminator,
)
from bike_sim.sim.ride.virtual_rider import CrashDetector, CrashEvent, PitchStabilizer

__all__ = [
    "SuspensionForceApplier",
    "TerrainContacts",
    "TerrainContactQuery",
    "RollingResistance",
    "CruiseController",
    "BrakeController",
    "PitchStabilizer",
    "CrashDetector",
    "CrashEvent",
    "RunLimits",
    "RunOutcome",
    "RunTerminator",
    "RideHUD",
    "RideInputHandler",
    "ModelLivery",
]
