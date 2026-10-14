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
