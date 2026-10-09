"""Python HUD compatibility is checked against the supplied pre-C source."""
from copy import copy
from dataclasses import replace
import mujoco
import numpy as np
import pytest
from ._physical_hud_oracle import RideHUD as OriginalHUD
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.ride.hud import RideHUD
from bike_sim.sim.ride.physical_view import (
    MODEL_FIELDS, RenderReplica, apply_frame, make_physical_view, snapshot_python,
)


@pytest.mark.parametrize('steps', [0, 1, 2])
def test_existing_preview_and_hud_remain_equal(environment_factory, assert_tree, steps):
    env = environment_factory()
    for _ in range(steps):
        env.step(RideControl(0.))
    old = OriginalHUD(env.sim.model)
    hud = RideHUD(env.sim.model)
    view = make_physical_view(env.sim)
    expected = old.preview_log_row(env.sim)
    assert_tree(dict(view.preview_row), dict(expected, requested_scale=1))
    for key, value in expected.items():
        assert type(view.preview_row[key]) is type(value), key
    assert hud._physical_columns(view) == old._physical_columns(env.sim)
    assert hud.styled_line(view).plain == old.styled_line(env.sim).plain
    assert hud.physical_legend(view).plain == old.physical_legend(env.sim).plain


def test_replica_only_updates_geometry_and_never_live_state(environment_factory, monkeypatch):
    env = environment_factory()
    env.step(RideControl(0.))
    frame = snapshot_python(env.sim)
    expected = frame.integration_state.copy()
    replica = RenderReplica(copy(env.sim.model))
    def forbidden(*args, **kwargs):
        raise AssertionError('presentation performed a physics solve')
    monkeypatch.setattr(mujoco, 'mj_step', forbidden)
    monkeypatch.setattr(mujoco, 'mj_forward', forbidden)
    replica.apply(frame)
    actual = np.empty_like(expected)
    mujoco.mj_getState(replica.model, replica.data, actual, mujoco.mjtState.mjSTATE_INTEGRATION)
    np.testing.assert_array_equal(actual, expected)
    replica.data.qpos[0] += 1.
    replica.model.dof_frictionloss[:] = 0.
    np.testing.assert_array_equal(snapshot_python(env.sim).integration_state, expected)
    for name in MODEL_FIELDS:
        assert not np.shares_memory(getattr(replica.model, name), getattr(env.sim.model, name))


def test_frame_owns_nested_fields_after_reset(environment_factory):
    env = environment_factory()
    frame = snapshot_python(env.sim)
    state = frame.integration_state.copy()
    field = frame.model_fields['site_pos'].copy()
    with pytest.raises(TypeError):
        frame.view.endpoint['position_m'] = 3.
    with pytest.raises(ValueError):
        frame.integration_state[0] = 3.
    with pytest.raises(ValueError):
        frame.model_fields['site_pos'].flat[0] = 3.
    env.reset()
    env.close()
    np.testing.assert_array_equal(frame.integration_state, state)
    np.testing.assert_array_equal(frame.model_fields['site_pos'], field)


def test_incompatible_frame_is_rejected_before_model_mutation(environment_factory):
    env = environment_factory()
    frame = snapshot_python(env.sim)
    model = copy(env.sim.model)
    data = mujoco.MjData(model)
    before = model.dof_frictionloss.copy()
    fields = dict(frame.model_fields, dof_frictionloss=np.zeros_like(before), site_pos=[float('nan')])
    with pytest.raises(ValueError, match='model field'):
        apply_frame(model, data, replace(frame, model_fields=fields))
    np.testing.assert_array_equal(model.dof_frictionloss, before)
