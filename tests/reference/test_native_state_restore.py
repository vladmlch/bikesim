"""Native forward on a stored golden state is bitwise-equal to Python forward."""
import sys
from pathlib import Path
import mujoco
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'native' / 'build'))

from _bits import assert_bitwise_equal

def _golden(tmp_path, steps=8):
    from bike_sim.cli import research as research_cli
    from test_pinned_topology import _pinned_config
    from tools.golden_episode import capture_episode, save
    from bike_sim.sim.ride.control import RideControl
    args = research_cli.parser().parse_args([
        '--physics-config', str(_pinned_config(tmp_path)),
        '--track-file', 'examples/research/rough_uphill_savage.toml',
        '--duration', '1', '--dt', '.00125', '--diagnostic-model-limits',
        '--out', str(tmp_path/'out')])
    env = research_cli.make_environment(args)
    ep = capture_episode(env, steps, RideControl(human_torque_nm=35.))
    save(ep, tmp_path/'g', env.sim.model)
    return tmp_path/'g'

@pytest.mark.slow
def test_forward_on_stored_state_is_bitwise(tmp_path):
    bike_native = pytest.importorskip('bike_native')
    from tools.golden_episode import load_episode
    ep = load_episode(_golden(tmp_path))
    ref = mujoco.MjModel.from_binary_path(str(tmp_path/'g'/'model.mjb'))
    dref = mujoco.MjData(ref)
    st = bike_native.Stepper(str(tmp_path/'g'/'model.mjb'))
    for k in range(len(ep.state_qpos)):
        mujoco.mj_resetData(ref, dref)
        dref.qpos[:] = ep.state_qpos[k]; dref.qvel[:] = ep.state_qvel[k]
        dref.act[:] = ep.state_act[k]
        dref.qacc_warmstart[:] = ep.state_warmstart[k]
        dref.time = float(ep.state_time[k])
        mujoco.mj_forward(ref, dref)
        st.set_state(ep.state_qpos[k], ep.state_qvel[k], ep.state_act[k],
                     ep.state_warmstart[k], float(ep.state_time[k]))
        st.forward()
        assert_bitwise_equal(st.qacc, dref.qacc)
        assert_bitwise_equal(st.qfrc_constraint, dref.qfrc_constraint)
        assert_bitwise_equal(st.efc_force, dref.efc_force)

@pytest.mark.slow
def test_set_state_size_mismatch_raises_and_keeps_state(tmp_path):
    bike_native = pytest.importorskip('bike_native')
    from tools.golden_episode import load_episode
    g = _golden(tmp_path)
    ep = load_episode(g)
    st = bike_native.Stepper(str(g/'model.mjb'))
    st.set_state(ep.state_qpos[0], ep.state_qvel[0], ep.state_act[0],
                 ep.state_warmstart[0], float(ep.state_time[0]))
    st.forward()
    before = np.asarray(st.qacc).copy()
    bad_qpos = np.zeros(ep.state_qpos.shape[1] + 1)
    with pytest.raises(ValueError):
        st.set_state(bad_qpos, ep.state_qvel[0], ep.state_act[0],
                     ep.state_warmstart[0], float(ep.state_time[0]))
    # The rejected call must not clobber the previously restored state:
    # every span is width-checked before mj_resetData runs.
    assert_bitwise_equal(np.asarray(st.qacc), before)
