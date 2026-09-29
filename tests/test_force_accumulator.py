"""Physical force ownership and pre-step sampling."""

import numpy as np
import pytest

from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.sim.ride.force_accumulator import ForceAccumulator
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.terrain import get_preset


def test_two_writers_on_one_dof_add_without_aliasing():
    acc = ForceAccumulator(2)
    source = np.array([3.0, 0.0])
    acc.add("spring", source)
    source[0] = 1000.0
    acc.add("chain", np.array([-1.0, 2.0]))
    np.testing.assert_array_equal(acc.total(), [2.0, 2.0])
    with pytest.raises(ValueError, match="duplicate"):
        acc.add("chain", np.zeros(2))
    acc.clear()
    np.testing.assert_array_equal(acc.total(), [0.0, 0.0])


def test_component_views_and_totals_cannot_change_accumulated_force():
    acc = ForceAccumulator(2)
    acc.add("spring", np.array([3.0, 1.0]))
    view = acc.components
    with pytest.raises(TypeError):
        view["chain"] = np.zeros(2)
    with pytest.raises(ValueError):
        view["spring"][0] = 100.0
    total = acc.total()
    total[0] = 100.0
    np.testing.assert_array_equal(acc.total(), [3.0, 1.0])


@pytest.mark.parametrize("invalid", [np.zeros(1), np.array([np.nan, 0.0])])
def test_invalid_generalized_force_is_rejected(invalid):
    acc = ForceAccumulator(2)
    with pytest.raises(ValueError, match="invalid generalized force"):
        acc.add("bad", invalid)


def test_physical_step_adds_external_to_suspension_and_samples_pre_step_state():
    sim = RideSimulation(
        track=get_preset("flat"),
        rider="none",
        physics_config=SimulationPhysicsConfig(physics_mode="physical"),
    )
    qpos_before = sim.data.qpos.copy()
    qvel_before = sim.data.qvel.copy()
    external = np.zeros(sim.model.nv)
    external[sim.applier.fork_dofadr] = 7.0

    sim.step(external_qfrc=external)
    time_s, qpos, qvel, components = sim.last_force_snapshot
    assert time_s == 0.0
    np.testing.assert_array_equal(qpos, qpos_before)
    np.testing.assert_array_equal(qvel, qvel_before)
    assert components["external"][sim.applier.fork_dofadr] == 7.0
    assert (
        components["fork_spring"][sim.applier.fork_dofadr]
        + components["fork_damper"][sim.applier.fork_dofadr]
    ) < 0.0
    expected = sum(components.values(), np.zeros(sim.model.nv))
    np.testing.assert_allclose(sim.data.qfrc_applied, expected)
    external[sim.applier.fork_dofadr] = 1000.0
    assert components["external"][sim.applier.fork_dofadr] == 7.0


def test_physical_steps_do_not_carry_old_generalized_or_body_forces():
    sim = RideSimulation(
        track=get_preset("flat"),
        rider="none",
        physics_config=SimulationPhysicsConfig(physics_mode="physical"),
    )
    external = np.zeros(sim.model.nv)
    external[sim.root_x_dofadr] = 9.0
    sim.step(external_qfrc=external)
    first_snapshot = sim.last_force_snapshot
    sim.data.xfrc_applied[1, 0] = 100.0
    sim.step()

    assert "external" not in sim.last_force_snapshot[3]
    # External drag and world rolling torque legitimately act on root DOFs.
    # Only named current contributions may remain; a zero root force would now
    # incorrectly require dropping real road/air interactions.
    np.testing.assert_allclose(sim.data.qfrc_applied,
        sum(sim.last_force_snapshot[3].values(), np.zeros(sim.model.nv)))
    assert sim.data.xfrc_applied[1, 0] == 0.0
    assert first_snapshot[3]["external"][sim.root_x_dofadr] == 9.0


def test_physical_step_rejects_invalid_external_shape():
    sim = RideSimulation(
        track=get_preset("flat"),
        rider="none",
        physics_config=SimulationPhysicsConfig(physics_mode="physical"),
    )
    with pytest.raises(ValueError, match="invalid generalized force"):
        sim.step(external_qfrc=np.zeros(sim.model.nv - 1))


def test_explicit_external_input_can_alias_mujoco_applied_force_array():
    sim = RideSimulation(
        track=get_preset("flat"),
        rider="none",
        physics_config=SimulationPhysicsConfig(physics_mode="physical"),
    )
    external = sim.data.qfrc_applied
    external.fill(0.0)
    external[sim.root_x_dofadr] = 11.0

    sim.step(external_qfrc=external)

    assert sim.last_force_snapshot[3]["external"][sim.root_x_dofadr] == 11.0
