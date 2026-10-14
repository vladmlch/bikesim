"""Playback viewer: load a recorded episode and interpolate its stored qpos."""
import json

import numpy as np
import pytest

TRACK_TOML = ('name = "p"\nlength_m = 40.0\nsurface = "hardpack"\n'
              'grade_profile = { knots = [[0.0, 0.02], [40.0, 0.02]] }\n')


def _build_episode(tmp_path):
    """Record a short lumped-rider episode (same recipe as test_async_close._env).

    Lumped rider must NOT pass --physics-config (drive_mode mismatch)."""
    from bike_sim.cli import research as research_cli
    from bike_sim.sim.ride.control import RideControl
    track = tmp_path / 't.toml'
    track.write_text(TRACK_TOML)
    out = tmp_path / 'ep'
    argv = ['--track-file', str(track), '--duration', '0.4', '--rider', 'lumped',
            '--seed', '3', '--energy-tolerance', '1e9', '--diagnostic-model-limits',
            '--initial-speed', '2', '--record-decimation', '4', '--out', str(out)]
    args = research_cli.parser().parse_args(argv)
    env = research_cli.make_environment(args)
    while not env.done:
        env.step(RideControl(motor_torque_nm=0., human_torque_nm=0.))
    env.sim.physical.flush()
    env.save(out, overwrite=True)
    return out


@pytest.fixture(scope='module')
def episode_dir(tmp_path_factory):
    """One real recorded episode shared by every playback test in this module."""
    return _build_episode(tmp_path_factory.mktemp('episode'))


@pytest.fixture(scope='module')
def playback(episode_dir):
    """Loaded Playback for the shared episode. The lazy import keeps collection
    working while bike_sim.sim.research.playback is still being written."""
    from bike_sim.sim.research.playback import load_playback
    return load_playback(episode_dir)


def _quaternion_blocks(model):
    """qpos index ranges that must stay unit quaternions (free/ball joints)."""
    import mujoco
    blocks = []
    for j in range(model.njnt):
        adr = int(model.jnt_qposadr[j])
        jtype = int(model.jnt_type[j])
        if jtype == int(mujoco.mjtJoint.mjJNT_FREE):
            blocks.append(np.arange(adr + 3, adr + 7))      # free joint: pos(3)+quat(4)
        elif jtype == int(mujoco.mjtJoint.mjJNT_BALL):
            blocks.append(np.arange(adr, adr + 4))          # ball joint: quat(4)
    return blocks


@pytest.mark.slow
def test_playback_loads_episode(playback):
    pb = playback
    model = pb.sim.model
    times = np.asarray(pb.times_s)
    assert times.ndim == 1 and times.size > 0
    assert np.all(np.isfinite(times))
    assert np.all(np.diff(times) > 0), 'times_s must be strictly increasing'
    n = times.size
    qpos = np.asarray(pb.qpos)
    qvel = np.asarray(pb.qvel)
    assert qpos.shape == (n, model.nq)
    assert qvel.shape == (n, model.nv)
    assert np.isfinite(pb.duration_s) and pb.duration_s > 0
    for t in (0.0, 0.5 * pb.duration_s, pb.duration_s):
        pose = np.asarray(pb.pose_at(t))
        assert pose.shape == (model.nq,)
        assert np.all(np.isfinite(pose))
    assert np.array_equal(np.asarray(pb.pose_at(times[0])), qpos[0])
    assert np.array_equal(np.asarray(pb.pose_at(times[-1])), qpos[-1])


@pytest.mark.slow
def test_playback_pose_interpolates(playback):
    pb = playback
    times = np.asarray(pb.times_s)
    qpos = np.asarray(pb.qpos)
    n = times.size
    assert n >= 2, 'episode must record at least two poses to interpolate between'
    blocks = _quaternion_blocks(pb.sim.model)
    quat_idx = np.concatenate(blocks) if blocks else np.zeros(0, dtype=int)
    nonquat = np.ones(pb.sim.model.nq, dtype=bool)
    nonquat[quat_idx] = False
    for k in range(n - 1):
        t_mid = 0.5 * (times[k] + times[k + 1])
        pose = np.asarray(pb.pose_at(t_mid))
        expected = 0.5 * (qpos[k] + qpos[k + 1])
        # Linear dofs land on the exact midpoint; quaternions may be
        # renormalized (slerp/nlerp) so only their norm is asserted.
        np.testing.assert_allclose(pose[nonquat], expected[nonquat], atol=1e-12, rtol=0,
                                   err_msg=f'non-quaternion dofs diverge at sample {k}')
        for block in blocks:
            assert abs(np.linalg.norm(pose[block]) - 1.0) < 1e-9, \
                f'quaternion block {block} not normalized at sample {k}'


def test_playback_rejects_bad_dir(tmp_path):
    """Missing or empty intervals.jsonl must raise, not hang or silently succeed."""
    from bike_sim.sim.research.playback import load_playback
    acceptable = (ValueError, FileNotFoundError, OSError, KeyError, json.JSONDecodeError)
    with pytest.raises(acceptable):
        load_playback(tmp_path)                     # empty directory
    with pytest.raises(acceptable):
        load_playback(tmp_path / 'nonexistent')     # directory does not exist
    fake = tmp_path / 'fake'
    fake.mkdir()
    (fake / 'summary.json').write_text('{"name": "not-an-episode"}\n')
    with pytest.raises(acceptable):
        load_playback(fake)                         # summary.json but no intervals.jsonl


@pytest.mark.slow
def test_replay_view_dry_run(episode_dir, tmp_path):
    """--view --dry-run replays the stored trajectory without opening a window."""
    from bike_sim.cli.replay import main
    assert main([str(episode_dir), '--view', '--dry-run']) == 0
    assert main([str(tmp_path / 'nonexistent'), '--view', '--dry-run']) == 1


def test_replay_verify_still_default(tmp_path):
    """Without --view the CLI still verifies; bare/bad invocations fail nonzero."""
    from bike_sim.cli.replay import main
    try:
        code = main([])
    except SystemExit as exc:                       # argparse may exit(2) on a missing positional
        code = exc.code
    assert code != 0
    assert main([str(tmp_path / 'nonexistent')]) != 0
