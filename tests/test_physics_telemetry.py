"""Synchronized physical force samples and mechanical accounting."""

import mujoco
import numpy as np
import pytest

from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.sim.ride.energy import EnergyLedger, mechanical_energy_terms, system_momentum
from bike_sim.sim.ride.recorder import CHANNELS, RideRecorder, read_csv
from bike_sim.sim.ride.telemetry_v2 import ForceSample, component_powers
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.terrain import get_preset


def test_force_sample_does_not_read_poststep_velocity():
    qvel = np.array([2.0])
    force = np.array([3.0])
    sample = ForceSample(0.0, np.array([0.0]), qvel, {"motor": force})
    qvel[0] = 100.0
    force[0] = 100.0
    assert component_powers(sample)["motor"] == 6.0
    with pytest.raises(ValueError):
        sample.qvel[0] = 1.0
    with pytest.raises(ValueError):
        sample.components["motor"][0] = 1.0
    with pytest.raises(ValueError):
        sample.qvel.setflags(write=True)
    with pytest.raises(TypeError):
        sample.components["other"] = np.array([1.0])


def test_force_sample_rejects_invalid_force():
    with pytest.raises(ValueError, match="force sample"):
        ForceSample(0.0, np.array([0.0]), np.array([1.0]), {"bad": np.zeros(2)})
    with pytest.raises(ValueError, match="force sample"):
        ForceSample(0.0, np.array([0.0]), np.array([1.0]), {"bad": np.array([np.nan])})


def test_loss_is_counted_once():
    ledger = EnergyLedger(10.0)
    assert ledger.residual(8.0, 0.0, 0.0, 2.0) == pytest.approx(0.0)
    assert ledger.residual(13.0, 5.0, 0.0, 2.0) == pytest.approx(0.0)
    with pytest.raises(ValueError, match="negative"):
        ledger.residual(10.0, 0.0, 0.0, -1.0)


def test_mechanical_energy_includes_compiled_mass_gravity_and_explicit_elastic():
    model = mujoco.MjModel.from_xml_string("""
    <mujoco>
      <worldbody><body><freejoint/><inertial pos="0 0 0" mass="2"
          diaginertia="1 1 1"/></body></worldbody>
    </mujoco>
    """)
    data = mujoco.MjData(model)
    data.qpos[2] = 3.0
    data.qvel[0] = 2.0
    kinetic, gravitational, elastic = mechanical_energy_terms(
        model, data, elastic_energy_j=3.0
    )
    assert kinetic == pytest.approx(4.0)
    assert gravitational == pytest.approx(58.86)
    assert elastic == pytest.approx(3.0)


def test_system_momentum_uses_each_body_and_inertial_orientation():
    model = mujoco.MjModel.from_xml_string("""
    <mujoco>
      <worldbody>
        <body name="one"><freejoint/><inertial pos="0 0 0" mass="2"
            diaginertia="1 2 3"/></body>
        <body name="two"><freejoint/><inertial pos="0 0 0" mass="2"
            diaginertia="1 2 3"/></body>
      </worldbody>
    </mujoco>
    """)
    data = mujoco.MjData(model)
    data.qpos[0:3] = (-1.0, 0.0, 0.0)
    data.qpos[7:10] = (1.0, 0.0, 0.0)
    data.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
    data.qpos[10:14] = (np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5))
    data.qvel[2] = 1.0
    data.qvel[8] = -1.0
    data.qvel[9] = 2.0  # local x rotates into world y; rotated Iy = 1.
    mujoco.mj_forward(model, data)
    linear, angular = system_momentum(model, data)
    np.testing.assert_allclose(linear, (0.0, 0.0, 0.0), atol=1e-12)
    np.testing.assert_allclose(angular, (0.0, 6.0, 0.0), atol=1e-12)

    data.qpos[[0, 7]] += 5.0
    data.qvel[[2, 8]] += 3.0
    mujoco.mj_forward(model, data)
    shifted_linear, shifted_angular = system_momentum(model, data)
    np.testing.assert_allclose(shifted_linear, (0.0, 0.0, 12.0), atol=1e-12)
    np.testing.assert_allclose(shifted_angular, angular, atol=1e-12)


def test_physical_recorder_matches_prestep_power_and_integrates_decimated_work(tmp_path):
    sim = RideSimulation(
        track=get_preset("flat"), rider="none",
        physics_config=SimulationPhysicsConfig(physics_mode="physical"),
    )
    sim.data.qvel[sim.root_x_dofadr] = 2.0
    dense = RideRecorder(sim, decimate=1)
    sparse = RideRecorder(sim, decimate=10)
    dense.record(sim)
    sparse.record(sim)
    force = np.zeros(sim.model.nv)
    force[sim.root_x_dofadr] = 3.0
    dt = float(sim.model.opt.timestep)

    sim.step(external_qfrc=force)
    dense.record(sim)
    sparse.record(sim)
    assert dense.schema_version == sparse.schema_version == 2
    assert dense.column("interval_start_s")[1] == pytest.approx(0.0)
    assert dense.column("interval_dt_s")[1] == pytest.approx(dt)
    assert dense.column(f"prestep_qvel_{sim.root_x_dofadr}")[1] == pytest.approx(2.0)
    assert np.isnan(dense.column(f"prestep_qvel_{sim.root_x_dofadr}")[0])
    assert dense.column("external_power_w")[1] == pytest.approx(6.0)
    assert dense.column("external_work_j")[1] == pytest.approx(6.0 * dt)
    assert np.isfinite(dense.column("kinetic_energy_j")[1])
    assert np.isfinite(dense.column("gravitational_energy_j")[1])
    assert np.isnan(dense.column("mechanical_energy_j")[1])
    assert np.isnan(dense.column("energy_residual_j")[1])

    for _ in range(9):
        sim.step(external_qfrc=force)
        dense.record(sim)
        sparse.record(sim)
    assert dense.component_work_j["external"] == pytest.approx(sparse.component_work_j["external"])
    assert dense.component_work_j["external"] == pytest.approx(
        sum(dense.column("external_power_w")[1:] * dt)
    )
    assert sparse.rows == 2
    assert dense.column("interval_start_s")[-1] < dense.column("time_s")[-1]
    back = read_csv(dense.write_csv(tmp_path / "physical.csv"))
    assert back["external_power_w"][1] == pytest.approx(6.0)


def test_legacy_recorder_retains_v1_columns():
    sim = RideSimulation(track=get_preset("flat"), rider="none")
    recorder = RideRecorder(sim)
    recorder.record(sim)
    assert recorder.schema_version == 1
    assert tuple(recorder.columns()) == tuple(CHANNELS)
