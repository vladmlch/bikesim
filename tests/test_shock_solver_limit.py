"""Native shock-limit forces stay separate from explicit suspension writers."""

import mujoco
import numpy as np
import pytest

from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.sim.ride.constraint_forces import (
    ConstraintForceSnapshot,
    shock_joint_limit_qfrc,
)
from bike_sim.sim.ride.recorder import RideRecorder
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.terrain import get_preset


_LIMIT_AND_CONTACT_XML = """
<mujoco>
  <option gravity="0 0 0" timestep="0.001"/>
  <worldbody>
    <geom name="floor" type="plane" size="1 1 0.1"/>
    <body pos="0 0 0.05">
      <joint name="shock_stroke" type="slide" axis="1 0 0" range="-0.01 0.01"/>
      <geom type="sphere" size="0.1" mass="1"/>
    </body>
  </worldbody>
</mujoco>
"""


def _stepped_isolated_model(shock_position_m: float, shock_velocity_mps: float):
    model = mujoco.MjModel.from_xml_string(_LIMIT_AND_CONTACT_XML)
    data = mujoco.MjData(model)
    data.qpos[0] = shock_position_m
    data.qvel[0] = shock_velocity_mps
    mujoco.mj_step(model, data)
    return model, data


def test_native_shock_limit_has_its_own_nonzero_force_and_power():
    model, data = _stepped_isolated_model(0.02, 1.0)
    limit_type = mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT
    active = [
        row for row in range(data.nefc)
        if data.efc_type[row] == limit_type and data.efc_id[row] == 0
    ]
    assert len(active) == 1
    assert data.efc_force[active[0]] > 0.0
    assert any(data.efc_type[row] != limit_type for row in range(data.nefc))

    qfrc = shock_joint_limit_qfrc(model, data)
    assert qfrc[0] == pytest.approx(-float(data.efc_force[active[0]]))
    snapshot = ConstraintForceSnapshot(
        0.0, float(data.time), np.array([1.0]), {"shock_solver_limit": qfrc}
    )
    assert snapshot.powers_w()["shock_solver_limit"] == pytest.approx(qfrc[0])
    assert snapshot.powers_w()["shock_solver_limit"] < 0.0


def test_unrelated_contact_rows_are_not_attributed_to_shock_limit():
    model, data = _stepped_isolated_model(0.0, 0.0)
    assert data.nefc > 0
    assert any(
        data.efc_type[row] != mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT
        and data.efc_force[row] > 0.0
        for row in range(data.nefc)
    )
    np.testing.assert_array_equal(shock_joint_limit_qfrc(model, data), [0.0])


def test_physical_step_exposes_poststep_constraint_snapshot_separately():
    sim = RideSimulation(
        track=get_preset("flat"), rider="none",
        physics_config=SimulationPhysicsConfig(physics_mode="physical"),
    )
    sim.step()
    assert "shock_solver_limit" not in sim.last_force_sample.components
    assert "shock_upper_stop" in sim.last_force_sample.components
    snapshot = sim.last_constraint_snapshot
    assert snapshot.interval_start_s == pytest.approx(0.0)
    assert snapshot.interval_end_s == pytest.approx(sim.data.time)
    assert snapshot.components["shock_solver_limit"].shape == (sim.model.nv,)
    assert "shock_upper_stop" not in snapshot.components


def test_v2_recorder_integrates_poststep_limit_work_for_finished_interval(monkeypatch):
    sim = RideSimulation(
        track=get_preset("flat"), rider="none",
        physics_config=SimulationPhysicsConfig(physics_mode="physical"),
    )
    recorder = RideRecorder(sim)
    recorder.record(sim)
    sim.data.qvel[sim.applier.shock_dofadr] = 2.0
    force = np.zeros(sim.model.nv)
    force[sim.applier.shock_dofadr] = -3.0
    # Inject at solved-force extraction, before the immutable interval is built.
    # Recorders must never replace actual forces from a later mutable attribute.
    from bike_sim.sim.ride import physical_runtime
    monkeypatch.setattr(physical_runtime,"shock_joint_limit_qfrc",lambda model,data: force.copy())
    sim.step()
    recorder.record(sim)
    dt = float(sim.model.opt.timestep)
    assert recorder.column("shock_solver_limit_power_w")[0] == pytest.approx(-6.0)
    assert recorder.column("shock_solver_limit_work_j")[0] == pytest.approx(-6.0 * dt)
    assert recorder.component_work_j["shock_solver_limit"] == pytest.approx(-6.0 * dt)


def test_v2_recorder_rejects_constraint_force_from_a_different_interval():
    sim = RideSimulation(
        track=get_preset("flat"), rider="none",
        physics_config=SimulationPhysicsConfig(physics_mode="physical"),
    )
    sim.data.time = 1000.0
    recorder = RideRecorder(sim)
    sim.step()
    sample = sim.last_force_sample
    sim.last_constraint_snapshot = ConstraintForceSnapshot(
        sample.time_s, float(sim.data.time + sim.model.opt.timestep), sample.qvel,
        sim.last_constraint_snapshot.components,
    )
    with pytest.raises(ValueError, match="constraint snapshot"):
        recorder.record(sim)
