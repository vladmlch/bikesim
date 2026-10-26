# tests/reference/test_equilibrium_state_file.py
"""The explicit ``equilibrium_state_file`` pose file alongside the auto-cache."""
import re

import numpy as np
import pytest

from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.physics.physical_config import TireBackendConfig
from bike_sim.physics.rider import RiderSpecs
from bike_sim.sim.ride.equilibrium_cache import save_state_file
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.terrain import get_preset


def _sim(*, rider_mass_kg=80.0, **config_kwargs):
    cfg = SimulationPhysicsConfig(
        'physical', drive_mode='coast',
        tires=TireBackendConfig(backend='compliant_2d'), **config_kwargs)
    return RideSimulation(
        track=get_preset('flat'),
        rider=RiderSpecs(variant='lumped', mass_kg=rider_mass_kg),
        physics_config=cfg)


def _settle_pose_file(path):
    """Settle the cheap flat/coast rig once and write its pose file."""
    sim = _sim()
    save_state_file(path, sim.physical, sim.data.qpos,
                    sim.equilibrium['steps'], sim.equilibrium['residual_qacc'])
    return sim


def _rewrite_npz(path, *, drop=(), replace=None):
    with np.load(path, allow_pickle=False) as source:
        arrays = {key: source[key] for key in source.files if key not in drop}
    arrays.update(replace or {})
    np.savez(path, **arrays)


def test_state_file_round_trip_loads_the_pose_without_settling(tmp_path):
    path = tmp_path / 'pose.npz'
    settled = _settle_pose_file(path)
    before = path.read_bytes()
    loaded = _sim(equilibrium_state_file=str(path))
    eq = loaded.equilibrium
    assert eq['cache_hit'] is True
    assert eq['state_file'] == str(path)
    assert eq['steps'] == settled.equilibrium['steps']
    assert eq['residual_qacc'] <= .05
    np.testing.assert_array_equal(loaded.data.qpos, settled.data.qpos)
    # The pose file is read-only at runtime: untouched by the load.
    assert path.read_bytes() == before


def test_state_file_wins_over_the_auto_cache(tmp_path, monkeypatch):
    from bike_sim.sim.ride import equilibrium_cache
    path = tmp_path / 'pose.npz'
    _settle_pose_file(path)

    def forbidden(runtime):
        raise AssertionError('the auto-cache loader ran although '
                             'equilibrium_state_file is set')
    monkeypatch.setattr(equilibrium_cache, 'load', forbidden)
    sim = _sim(equilibrium_state_file=str(path), equilibrium_cache_enabled=True)
    assert sim.equilibrium['cache_hit'] is True
    assert sim.equilibrium['state_file'] == str(path)


def test_state_file_rejects_a_different_configuration(tmp_path):
    path = tmp_path / 'pose.npz'
    _settle_pose_file(path)
    # The fingerprint covers the scenario, not the file: a rider-mass change
    # must reject the pose even though the npz itself is intact.
    with pytest.raises(ValueError, match=re.escape(str(path))):
        _sim(rider_mass_kg=75.0, equilibrium_state_file=str(path))


def test_state_file_rejects_a_state_payload_that_does_not_fit(tmp_path):
    path = tmp_path / 'pose.npz'
    _settle_pose_file(path)
    # The fingerprint still matches: only the strict all-or-nothing material
    # state check can reject this file.
    _rewrite_npz(path, drop=('state_tire_xi',))
    with pytest.raises(ValueError, match=re.escape(str(path))):
        _sim(equilibrium_state_file=str(path))


def test_state_file_rechecks_the_residual(tmp_path):
    path = tmp_path / 'pose.npz'
    sim = _settle_pose_file(path)
    root_z = sim.physical.address('root_z')[0]
    qpos = np.array(sim.data.qpos, copy=True)
    qpos[root_z] += .05
    _rewrite_npz(path, replace={'qpos': qpos})
    with pytest.raises(ValueError, match=re.escape(str(path))):
        _sim(equilibrium_state_file=str(path))


def test_state_file_names_a_missing_file(tmp_path):
    path = tmp_path / 'absent.npz'
    with pytest.raises(ValueError, match=re.escape(str(path))):
        _sim(equilibrium_state_file=str(path))


@pytest.mark.parametrize('value', ('', '   ', 3.5, ['x']))
def test_equilibrium_state_field_validation(value):
    with pytest.raises(ValueError, match='equilibrium_state_file'):
        SimulationPhysicsConfig(equilibrium_state_file=value)


def test_toml_state_file_resolves_relative_to_the_config(tmp_path):
    from bike_sim.physics.resolution import load_physics_config
    toml = tmp_path / 'physics.toml'
    toml.write_text('physics_mode = "physical"\n'
                    'equilibrium_state_file = "poses/flat.npz"\n')
    resolved = load_physics_config(toml)
    assert resolved.equilibrium_state_file == str(
        (tmp_path / 'poses' / 'flat.npz').resolve())
    absolute = tmp_path / 'elsewhere.npz'
    toml.write_text('physics_mode = "physical"\n'
                    f'equilibrium_state_file = "{absolute}"\n')
    assert load_physics_config(toml).equilibrium_state_file == str(absolute)


def test_equilibrium_override_kwargs_reach_the_solve():
    sim = RideSimulation(
        track=get_preset('flat'), rider='lumped',
        physics_config=SimulationPhysicsConfig(
            'physical', drive_mode='coast',
            tires=TireBackendConfig(backend='compliant_2d')),
        equilibrium_max_steps=4000, equilibrium_tolerance=.25)
    assert sim.equilibrium['residual_qacc'] <= .25
    with pytest.raises(ValueError, match='equilibrium_max_steps'):
        RideSimulation(equilibrium_max_steps=4000)
    with pytest.raises(ValueError, match='equilibrium_tolerance'):
        RideSimulation(
            track=get_preset('flat'), rider='lumped',
            physics_config=SimulationPhysicsConfig('physical', drive_mode='coast'),
            equilibrium_tolerance=float('nan'))


def _research_env(*, seed=0, duration_s=3.):
    """A real research environment on the cheap flat/lumped rig."""
    from bike_sim.sim.research.configuration import build_environment
    from bike_sim.sim.research.environment import ExperimentConfig
    from bike_sim.sim.research.sensors import SensorConfig
    return build_environment(
        track=get_preset('flat'), rider=RiderSpecs(variant='lumped'),
        physics_config=SimulationPhysicsConfig(
            'physical', drive_mode='crank_effort', pitch_assist=False,
            tires=TireBackendConfig(backend='compliant_2d', surface_mode='track'),
            equilibrium_cache_enabled=True),
        experiment=ExperimentConfig(seed=seed, duration_s=duration_s),
        sensors=SensorConfig())


def test_experiment_configs_produce_different_episode_keys(tmp_path, monkeypatch):
    """The poisoning regression: experiment inputs must scope what a
    post-episode re-settle publishes under one cache directory.

    The canonical fingerprint stays shared — the settled pose is a function
    of the plant alone, so env B legitimately warm-loads env A's entry — but
    the episode key binds the experiment context, so two different
    ``ExperimentConfig`` values can never publish to the same name.
    """
    monkeypatch.setenv('BIKE_SIM_EQUILIBRIUM_CACHE_DIR', str(tmp_path))
    from bike_sim.sim.ride import equilibrium_cache
    env_a = _research_env(seed=11, duration_s=1.)
    env_b = _research_env(seed=22, duration_s=2.)
    try:
        canonical_a = equilibrium_cache.fingerprint(env_a.sim.physical)
        canonical_b = equilibrium_cache.fingerprint(env_b.sim.physical)
        assert canonical_a == canonical_b
        episode_a = equilibrium_cache.episode_fingerprint(env_a.sim.physical)
        episode_b = equilibrium_cache.episode_fingerprint(env_b.sim.physical)
        assert episode_a != episode_b
        assert episode_a != canonical_a and episode_b != canonical_b
    finally:
        env_a.close()
        env_b.close()


def test_dirty_resettle_writes_the_episode_key_not_the_canonical_name(tmp_path, monkeypatch):
    """A post-episode re-settle must never republish under the load key.

    The canonical entry is deleted so ``reset()`` actually re-settles — the
    first-run-race / self-heal path where the dirty-settle poisoning vector
    fired. The result must land under the quarantined episode name and leave
    the canonical name absent, so the next consumer settles fresh instead of
    adopting a pose an episode history shaped.
    """
    monkeypatch.setenv('BIKE_SIM_EQUILIBRIUM_CACHE_DIR', str(tmp_path))
    from bike_sim.sim.ride import equilibrium_cache
    sim = _sim(equilibrium_cache_enabled=True)
    # The coast rig's strict monitor rejects linkage constraint-work numerics
    # during free stepping; the cache path under test does not depend on it.
    sim.physical.set_strict(False)
    key = equilibrium_cache.fingerprint(sim.physical)
    canonical = tmp_path / f'{key}.npz'
    assert canonical.is_file()
    for _ in range(40):
        sim.step()
    canonical.unlink()
    sim.reset()
    assert not canonical.exists()
    episode = tmp_path / f'{equilibrium_cache.episode_fingerprint(sim.physical)}.npz'
    assert episode.is_file()
    assert equilibrium_cache.load(sim.physical) is None
