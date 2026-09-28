"""End-stop physics and its physical ride-mode integration."""

import mujoco
import numpy as np
import pytest

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.physics.coil_shock import CoilShock, CoilShockSpecs
from bike_sim.physics.model_config import EndStopConfig, SimulationPhysicsConfig
from bike_sim.physics.stops import end_stop
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.terrain import get_preset


def test_unloaded_compression_coil_does_not_pull_but_legacy_law_is_preserved():
    assert CoilShock().compute_spring_force(-4.0) == 0.0
    assert CoilShock(legacy_behavior=True).compute_spring_force(-4.0) == pytest.approx(-458.4)


@pytest.mark.parametrize(
    "specs",
    [
        CoilShockSpecs(preload_mm=-1.0),
        CoilShockSpecs(rate_n_m=0.0),
        CoilShockSpecs(stroke_mm=0.0),
        CoilShockSpecs(bumper_length_mm=0.0),
        CoilShockSpecs(bumper_length_mm=66.0),
    ],
)
def test_invalid_coil_geometry_or_rate_is_rejected(specs):
    with pytest.raises(ValueError):
        CoilShock(specs)


def test_top_out_pushes_into_range_and_stores_energy():
    force, energy = end_stop(-0.002, -0.1, 0.0, 0.065, 200000.0, 500.0)
    assert force == pytest.approx(450.0)
    assert energy == pytest.approx(0.4)


def test_releasing_top_out_dissipates_energy_without_tensile_contact():
    force, energy = end_stop(-0.002, 0.1, 0.0, 0.065, 200000.0, 500.0)
    assert force == pytest.approx(350.0)
    assert energy == pytest.approx(0.4)
    assert force * 0.1 + (-400.0) * 0.1 <= 0.0
    force, _ = end_stop(-0.002, 2.0, 0.0, 0.065, 200000.0, 500.0)
    assert force == 0.0


def test_unloaded_upper_boundary_has_no_contact_force():
    assert end_stop(0.065, 0.1, 0.0, 0.065, 200000.0, 500.0) == (0.0, 0.0)


@pytest.mark.parametrize("k,c", [(0.0, 500.0), (200000.0, -1.0)])
def test_invalid_end_stop_parameters_are_rejected(k, c):
    with pytest.raises(ValueError, match="end-stop"):
        end_stop(-0.002, 0.0, 0.0, 0.065, k, c)


def test_upper_handoff_preserves_force_energy_and_passivity():
    peak_n = 7000.0
    bumper_length_m = 0.010
    boundary_energy_j = peak_n * bumper_length_m / 3.0
    at_boundary = end_stop(
        0.065, 0.0, 0.0, 0.065, 700000.0, 500.0,
        upper_boundary_force_n=peak_n,
        upper_boundary_energy_j=boundary_energy_j,
    )
    just_above = end_stop(
        0.065000001, 0.0, 0.0, 0.065, 700000.0, 500.0,
        upper_boundary_force_n=peak_n,
        upper_boundary_energy_j=boundary_energy_j,
    )
    assert at_boundary == pytest.approx((-peak_n, boundary_energy_j))
    assert just_above == pytest.approx(at_boundary, abs=0.001)
    force, energy = end_stop(
        0.067, -0.1, 0.0, 0.065, 700000.0, 500.0,
        upper_boundary_force_n=peak_n,
        upper_boundary_energy_j=boundary_energy_j,
    )
    assert force == pytest.approx(-8350.0)
    assert energy == pytest.approx(boundary_energy_j + 14.0 + 1.4)
    assert force * -0.1 + 8400.0 * -0.1 <= 0.0


def test_physical_joint_has_stop_deformation_behind_working_range():
    for mode, expected in [
        ("legacy", (0.0, 0.065)),
        ("physical", (-0.010, 0.075)),
    ]:
        cfg = SimulationPhysicsConfig(physics_mode=mode)
        model = mujoco.MjModel.from_xml_string(
            generate_mujoco_xml(mode="ride", rider="none", physics_config=cfg)
        )
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "shock_stroke")
        assert tuple(model.jnt_range[jid]) == pytest.approx(expected)


def test_physical_force_channels_handoff_at_working_stroke():
    sim = RideSimulation(
        track=get_preset("flat"), rider="none",
        physics_config=SimulationPhysicsConfig(physics_mode="physical"),
    )
    dof = sim.applier.shock_dofadr
    pos = sim.applier.shock_qposadr
    sim.data.qpos[pos] = 0.065
    sim.data.qvel[dof] = 0.0
    boundary = sim.applier.compute_qfrc_components(sim.model, sim.data)
    assert boundary["shock_bumper"][dof] == pytest.approx(-7000.0)
    assert boundary["shock_upper_stop"][dof] == 0.0
    assert sim.applier.potential_energy_j["shock_bumper"] == pytest.approx(7000.0 * 0.010 / 3.0)

    sim.data.qpos[pos] = 0.067
    above = sim.applier.compute_qfrc_components(sim.model, sim.data)
    assert above["shock_bumper"][dof] == 0.0
    assert above["shock_upper_stop"][dof] == pytest.approx(-8400.0)
    assert sim.applier.potential_energy_j["shock_upper_stop"] == pytest.approx(
        7000.0 * 0.010 / 3.0 + 14.0 + 1.4
    )
    np.testing.assert_allclose(
        sim.applier.compute_qfrc(sim.model, sim.data),
        sum(above.values(), np.zeros(sim.model.nv)),
    )


def test_physical_step_registers_separate_suspension_vectors():
    sim = RideSimulation(
        track=get_preset("flat"), rider="none",
        physics_config=SimulationPhysicsConfig(physics_mode="physical"),
    )
    sim.data.qpos[sim.applier.shock_qposadr] = -0.002
    sim.data.qvel[sim.applier.shock_dofadr] = -0.1
    sim.step()
    channels = sim.last_force_snapshot[3]
    assert {
        "fork_spring", "fork_damper", "shock_coil", "shock_bumper",
        "shock_damper", "shock_top_out", "shock_upper_stop",
    } <= channels.keys()
    assert "suspension" not in channels
    assert channels["shock_top_out"][sim.applier.shock_dofadr] == pytest.approx(1450.0)
    assert channels["shock_coil"][sim.applier.shock_dofadr] == 0.0
    assert sim.applier.potential_energy_j["shock_top_out"] == pytest.approx(1.4)


def test_stop_configuration_is_immutable_and_reaches_joint_range():
    stop = EndStopConfig(overtravel_m=0.012)
    cfg = SimulationPhysicsConfig(physics_mode="physical", end_stops=stop)
    model = mujoco.MjModel.from_xml_string(
        generate_mujoco_xml(mode="ride", rider="none", physics_config=cfg)
    )
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "shock_stroke")
    assert tuple(model.jnt_range[jid]) == pytest.approx((-0.012, 0.077))
    assert stop.stiffness_n_m == pytest.approx(700000.0)
    assert stop.provenance == "synthetic"
    with pytest.raises((AttributeError, TypeError)):
        stop.overtravel_m = 0.02


def test_stop_reference_deflection_must_precede_solver_limit():
    with pytest.raises(ValueError, match="overtravel"):
        EndStopConfig(overtravel_m=0.008, reference_deflection_m=0.010)


def test_legacy_simulation_preserves_supplied_coil_mode():
    coil = CoilShock()
    sim = RideSimulation(
        track=get_preset("flat"), rider="none", coil_shock=coil
    )
    sim.data.qpos[sim.applier.shock_qposadr] = -0.004
    sim.applier.compute_qfrc(sim.model, sim.data)
    assert sim.applier.coil_shock is coil
    assert coil.legacy_behavior is False
    assert sim.applier.shock_spring_n == 0.0


@pytest.mark.parametrize("mode", ["legacy", "physical"])
def test_explicit_coil_stroke_must_match_compiled_bike_stroke(mode):
    coil = CoilShock(CoilShockSpecs(stroke_mm=60.0))
    with pytest.raises(ValueError, match=r"CoilShock.*60\.0.*BikeSpecs.*65\.0"):
        RideSimulation(
            track=get_preset("flat"), rider="none", coil_shock=coil,
            physics_config=SimulationPhysicsConfig(physics_mode=mode),
        )


def test_default_coil_uses_active_bike_stroke_and_rate():
    specs = BikeSpecs(shock_stroke=60.0, shock_stiffness=120000.0)
    sim = RideSimulation(
        track=get_preset("flat"), rider="none", specs=specs,
        physics_config=SimulationPhysicsConfig(physics_mode="physical"),
    )
    assert sim.applier.coil_shock.specs.stroke_mm == pytest.approx(60.0)
    assert sim.applier.coil_shock.specs.rate_n_m == pytest.approx(120000.0)
    jid = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_JOINT, "shock_stroke")
    assert tuple(sim.model.jnt_range[jid]) == pytest.approx((-0.010, 0.070))


def test_supplied_coil_subclass_keeps_identity_and_custom_force_method():
    class OffsetCoil(CoilShock):
        def compute_spring_force(self, stroke_mm):
            return super().compute_spring_force(stroke_mm) + 123.0

    coil = OffsetCoil(legacy_behavior=True)
    sim = RideSimulation(track=get_preset("flat"), rider="none", coil_shock=coil)
    assert sim.applier.coil_shock is coil
    assert coil.legacy_behavior is True
    sim.data.qpos[sim.applier.shock_qposadr] = -0.004
    sim.applier.compute_qfrc(sim.model, sim.data)
    assert sim.applier.shock_spring_n == pytest.approx(-458.4 + 123.0)


def test_ordinary_legacy_simulation_keeps_compiled_range_and_signed_coil():
    sim = RideSimulation(track=get_preset("flat"), rider="none")
    jid = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_JOINT, "shock_stroke")
    assert tuple(sim.model.jnt_range[jid]) == pytest.approx((0.0, 0.065))
    sim.data.qpos[sim.applier.shock_qposadr] = -0.004
    sim.applier.compute_qfrc(sim.model, sim.data)
    assert sim.applier.shock_spring_n == pytest.approx(-458.4)
