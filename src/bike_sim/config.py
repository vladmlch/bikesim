"""
Bicycle Master Configuration Module.

This module provides the unified `BikeConfig` dataclass that aggregates frame geometry,
suspension hardware, air spring tuning, damper click setups, component masses, and rider anatomy
into a structured, cleanly decoupled architecture.
"""

from dataclasses import dataclass, field
from typing import Optional

from bike_sim.geometry.specs import FrameGeometrySpecs, SuspensionHardwareSpecs
from bike_sim.physics.air_spring import AirSpringSpecs
from bike_sim.physics.damper import DamperClickConfig
from bike_sim.physics.mass import BikeMassSpecs, RiderSpecs


@dataclass
class BikeConfig:
    """
    Master unified configuration container for 2D/3D bicycle kinematics and simulation.
    """

    geometry: FrameGeometrySpecs = field(default_factory=FrameGeometrySpecs)
    suspension: SuspensionHardwareSpecs = field(default_factory=SuspensionHardwareSpecs)
    air_spring: AirSpringSpecs = field(default_factory=AirSpringSpecs)
    damper_clicks: DamperClickConfig = field(default_factory=DamperClickConfig)
    mass: BikeMassSpecs = field(default_factory=BikeMassSpecs)
    rider: RiderSpecs = field(default_factory=RiderSpecs)
