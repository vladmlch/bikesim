"""Selected-artifact native frame gates; no implicit build and no skip."""
from copy import copy
import numpy as np
import pytest
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.ride.hud import RideHUD
from bike_sim.sim.ride.physical_view import RenderReplica, environment_snapshot, present_view
from ._physical_hud_oracle import RideHUD as OriginalHUD


def test_native_preview_matches_reference_and_is_owned(environment_pair, assert_tree):
    reference, native = environment_pair()
    original = OriginalHUD(reference.sim.model)
    for _ in range(3):
        frame = native.snapshot()
        view = present_view(frame.view, track=native.sim.track)
        assert_tree(dict(view.preview_row), dict(original.preview_log_row(reference.sim), requested_scale=1))
        assert RideHUD(reference.sim.model)._physical_columns(view) == original._physical_columns(reference.sim)
        assert_tree(view.endpoint, environment_snapshot(reference).view.endpoint)
        assert_tree(native.step(RideControl(0.)), reference.step(RideControl(0.)))
    frame = native.snapshot()
    expected = frame.integration_state.copy()
    replica = RenderReplica(native.make_render_model())
    replica.apply(frame)
    replica.data.qpos[:] += 1.
    replica.model.site_pos[:] = 10.
    np.testing.assert_array_equal(native.snapshot().integration_state, expected)
    native.reset()
    native.close()
    np.testing.assert_array_equal(frame.integration_state, expected)
    assert not frame.integration_state.flags.writeable
    with pytest.raises(TypeError):
        frame.view.preview_row['motor_torque_nm'] = 3.


def test_native_target_slice_does_not_flush_or_duplicate_request(environment_pair, assert_tree):
    reference, native = environment_pair()
    for env in (reference, native):
        env.begin_control(RideControl(0.))
        assert env.advance_control(target_step=1) is None
        assert env.sim.steps == 1
        assert env.control_pending
        assert len(env.commands_requested) == 1
        assert len(env.observations) == 1
        assert len(env.trace) == 0
        with pytest.raises(ValueError):
            env.advance_control(target_step=0)
        assert env.control_pending
    np.testing.assert_allclose(native.snapshot().integration_state,
                               environment_snapshot(reference).integration_state, rtol=1e-9, atol=1e-9)
    assert_tree(native.advance_control(), reference.advance_control())
    assert len(native.commands_requested) == len(native.trace) == 1
