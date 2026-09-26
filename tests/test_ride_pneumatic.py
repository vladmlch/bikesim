"""Ride-mode integration checks for the optional pneumatic tyre model."""

import json
from pathlib import Path

import mujoco
import numpy as np
import pytest

from bike_sim.geometry.hardpoints import compute_ground_z
from bike_sim.geometry.specs import BikeSpecs
from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.physics.tyre import FRONT_TYRE, TIERS, TyreConfig
from bike_sim.sim.ride.tyre.geometry import RoadProfile
from bike_sim.sim.ride.tyre.model import PneumaticTyre
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.terrain import (
    HeightFieldSpec,
    SquareEdge,
    SurfaceMap,
    TrackSpec,
    available_presets,
    build_profile,
    get_preset,
)


def _road_profile(track: TrackSpec) -> RoadProfile:
    field = HeightFieldSpec.for_track(track)
    x_m = field.track_x()
    ground_z_m = compute_ground_z(BikeSpecs()) / 1000.0
    z_m = ground_z_m + build_profile(track, x_m)
    return RoadProfile.from_samples(x_m, z_m)


def _pneumatic_config(tier: str = "fast") -> TyreConfig:
    return TyreConfig(model="pneumatic", tier=tier)


def test_pneumatic_xml_disables_only_the_wheel_contact_spheres():
    model = mujoco.MjModel.from_xml_string(
        generate_mujoco_xml(mode="ride", rider="lumped", tyre_model="pneumatic")
    )
    for name in ("geom_front_contact", "geom_rear_contact"):
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        assert geom_id >= 0
        assert model.geom_contype[geom_id] == 0
        assert model.geom_conaffinity[geom_id] == 0
        assert model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_SPHERE


@pytest.mark.parametrize("preset", available_presets())
def test_pneumatic_equilibrium_supports_system_weight_on_each_preset(preset):
    sim = RideSimulation(
        track=get_preset(preset),
        tyre=_pneumatic_config(),
        rider="none",
        target_speed_kmh=15.0,
    )
    support_n = sim.contacts.front_support_n + sim.contacts.rear_support_n
    system_weight_n = float(sim.model.body_mass.sum()) * abs(float(sim.model.opt.gravity[2]))
    assert sim.equilibrium["residual_qacc"] <= 0.05
    assert support_n == pytest.approx(system_weight_n, rel=0.005), preset
    assert sim.contacts.front_load_n > 0.0
    assert sim.contacts.rear_load_n > 0.0


def test_flat_pneumatic_traverse_completes_and_cruise_holds_target():
    track = TrackSpec(name="flat_pneumatic_short", length_m=20.0, surface="asphalt")
    sim = RideSimulation(
        track=track,
        tyre=_pneumatic_config(),
        rider="lumped",
        target_speed_kmh=15.0,
    )
    speeds_mps = []

    def record_late_speed(current: RideSimulation) -> None:
        if current.position_m >= 10.0:
            speeds_mps.append(current.speed_mps)

    outcome = sim.run(
        limits=sim.default_limits(max_wall_clock_s=30.0),
        on_step=record_late_speed,
    )
    assert outcome.completed, outcome.describe()
    assert sim.crash is None
    assert speeds_mps
    assert float(np.mean(speeds_mps[-500:])) == pytest.approx(15.0 / 3.6, abs=0.25)


def test_single_edge_traverse_shows_two_patches_and_rearward_force():
    sim = RideSimulation(
        track=get_preset("single_edge"),
        tyre=_pneumatic_config(),
        rider="lumped",
        target_speed_kmh=15.0,
    )
    edge_samples = []

    def record_edge(current: RideSimulation) -> None:
        if 18.5 <= current.position_m <= 21.5:
            outputs = current.tyre_applier.front_outputs
            if len(outputs.patches) == 2:
                edge_samples.append((outputs.force_world_n[0], outputs.patches))

    outcome = sim.run(
        limits=sim.default_limits(max_wall_clock_s=30.0),
        on_step=record_edge,
    )
    assert outcome.completed, outcome.describe()
    assert edge_samples
    assert any(force_x_n < 0.0 for force_x_n, _ in edge_samples)


def test_kicker_gap_has_a_geometric_airborne_state():
    track = get_preset("enduro_aggressive")
    road = _road_profile(track)
    wheel = PneumaticTyre(FRONT_TYRE, TIERS["detailed"])
    radius_m = FRONT_TYRE.outer_radius_mm / 1000.0
    ground_z_m = compute_ground_z(BikeSpecs()) / 1000.0
    gap_x_m = 59.5
    outputs = wheel.evaluate(
        np.asarray([gap_x_m, 0.0, ground_z_m + radius_m + 0.40]),
        np.zeros(3),
        0.0,
        road,
        SurfaceMap.uniform("hardpack"),
        dt_s=0.00025,
    )
    assert outputs.airborne
    assert outputs.patches == ()


def test_kicker_traverse_reports_both_wheels_airborne_during_flight():
    sim = RideSimulation(
        track=get_preset("enduro_aggressive"),
        tyre=_pneumatic_config(),
        rider="lumped",
        target_speed_kmh=25.0,
    )
    flight_samples = []

    def record_flight(current: RideSimulation) -> None:
        if 58.0 <= current.position_m <= 61.5:
            front = current.tyre_applier.front_outputs
            rear = current.tyre_applier.rear_outputs
            flight_samples.append((front.airborne, rear.airborne))

    outcome = sim.run(
        limits=sim.default_limits(max_wall_clock_s=60.0),
        on_step=record_flight,
    )
    assert outcome.completed, outcome.describe()
    assert any(front and rear for front, rear in flight_samples)


def test_sphere_mode_matches_the_pre_pneumatic_trace():
    trace_path = Path(__file__).resolve().parent / "golden/ride_sphere_trace.json"
    trace = json.loads(trace_path.read_text(encoding="utf-8"))
    sim = RideSimulation(
        track=get_preset(trace["track"]),
        tyre=TyreConfig(model="sphere"),
        rider=trace["rider"],
        target_speed_kmh=trace["target_speed_kmh"],
    )

    for expected in trace["samples"]:
        while sim.steps < expected["step"]:
            sim.step()
        assert sim.steps == expected["step"]
        assert sim.time_s == expected["time_s"]
        assert sim.position_m == expected["position_m"]
        assert np.array_equal(sim.data.qpos, np.asarray(expected["qpos"]))
        assert np.array_equal(sim.data.qvel, np.asarray(expected["qvel"]))
        assert np.array_equal(sim.data.ctrl, np.asarray(expected["ctrl"]))
        actual_contacts = np.asarray([
            sim.contacts.front_load_n,
            sim.contacts.rear_load_n,
            sim.contacts.front_support_n,
            sim.contacts.rear_support_n,
            sim.contacts.handlebar_load_n,
        ])
        assert np.array_equal(actual_contacts, np.asarray(expected["contacts"]))
