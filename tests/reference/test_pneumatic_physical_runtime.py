# tests/reference/test_pneumatic_physical_runtime.py
"""End-to-end ``tires.backend='pneumatic'`` physical ride under the python backend."""
import numpy as np

from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.physics.physical_config import PneumaticTireConfig, TireBackendConfig
from bike_sim.sim.ride.pneumatic_tire_forces import PneumaticTireForceApplier
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.terrain import get_preset


def _pneumatic_sim():
    cfg = SimulationPhysicsConfig(
        'physical', drive_mode='coast',
        tires=TireBackendConfig(backend='pneumatic', surface_mode='track',
                                pneumatic=PneumaticTireConfig(tier='fast')))
    sim = RideSimulation(track=get_preset('flat'), rider='lumped', physics_config=cfg)
    # A coasting lumped rider does no muscle or motor work, so the strict
    # monitor's absolute 1e-8 J zero-source constraint-work budget fails on
    # linkage-closure numerics -- identically under compliant_2d.
    sim.physical.set_strict(False)
    return sim


def test_pneumatic_backend_runs_a_physical_ride_end_to_end():
    sim = _pneumatic_sim()
    equilibrium = sim.equilibrium
    assert equilibrium['residual_qacc'] <= .05
    assert equilibrium['static_front_load_n'] > 0.
    assert equilibrium['static_rear_load_n'] > 0.
    tire = sim.physical.tire
    assert isinstance(tire, PneumaticTireForceApplier)
    for _ in range(400):
        sim.step()
    assert sim.steps == 400
    # The backend writes qfrc only: the legacy xfrc wheel rows stay empty and
    # the disabled wheel geoms produce no engine contacts.
    assert not sim.data.xfrc_applied.any()
    assert sim.data.ncon == 0
    for side in ('front', 'rear'):
        diagnostics = tire.diagnostics[side]
        assert diagnostics['normal_load_n'] > 0.
        assert diagnostics['patch_count'] >= 1
        assert diagnostics['surface']
        snapshot = tire.snapshots[side]
        assert snapshot.backend == 'pneumatic' and snapshot.geometric_contact
    assert sim.contacts.front_load_n > 0. and sim.contacts.rear_load_n > 0.
    channels = sim.physical.sample.channels['tires']
    for side in ('front', 'rear'):
        assert channels[side]['backend'] == 'pneumatic'
        assert channels[side]['normal_load_n'] > 0.
        assert channels[side]['patches']
    assert sim.physical.energy['elastic_energy_j']['tires'] > 0.


def test_pneumatic_tire_state_round_trips_through_the_cache_codec():
    from bike_sim.sim.ride.equilibrium_cache import _state_arrays, restore_state
    sim = _pneumatic_sim()
    for _ in range(50):
        sim.step()
    tire = sim.physical.tire
    arrays = _state_arrays(sim.physical)
    assert set(arrays) == {'state_tire_wire_names', 'state_tire_wire_row'}
    names, row = arrays['state_tire_wire_names'], arrays['state_tire_wire_row']
    assert names.dtype.kind == 'U' and row.dtype == float
    assert row.shape == tire.state_dict()['row'].shape == (len(names),)
    # A cold-start tire cannot hold the committed carcass/patch state: the wire
    # restore is what rebuilds it.
    tire.reset()
    assert not tire.states['front'].carcass_state.initialized
    assert restore_state(sim.physical, dict(arrays)) is True
    assert np.array_equal(tire.state_dict()['row'], row, equal_nan=True)
    assert tire.states['front'].carcass_state.initialized
    # Strict rejections: missing wire keys, wrong row width, unknown columns.
    missing = dict(arrays)
    del missing['state_tire_wire_row']
    assert restore_state(sim.physical, missing) is False
    short = dict(arrays)
    short['state_tire_wire_row'] = row[:-1]
    assert restore_state(sim.physical, short) is False
    unknown = dict(arrays)
    unknown['state_tire_wire_names'] = np.asarray(list(names[:-1]) + ['front.ray.000.bogus'])
    assert restore_state(sim.physical, unknown) is False


def test_pneumatic_tire_wire_round_trips_through_the_npz_cache_file(tmp_path, monkeypatch):
    """The wire keys ride through the real ``save``/``load`` npz unchanged."""
    from bike_sim.sim.ride import equilibrium_cache
    monkeypatch.setenv('BIKE_SIM_EQUILIBRIUM_CACHE_DIR', str(tmp_path))
    sim = _pneumatic_sim()
    for _ in range(50):
        sim.step()
    tire = sim.physical.tire
    equilibrium_cache.save(sim.physical, sim.data.qpos, sim.steps, 0.01)
    cached = equilibrium_cache.load(sim.physical)
    assert cached is not None
    _, qpos, steps, _, state = cached
    assert steps == sim.steps
    np.testing.assert_array_equal(qpos, sim.data.qpos)
    assert set(state) == {'state_tire_wire_names', 'state_tire_wire_row'}
    names, row = state['state_tire_wire_names'], state['state_tire_wire_row']
    assert names.dtype.kind == 'U' and row.dtype == np.float64
    wire = tire.state_dict()
    assert [str(name) for name in names] == wire['names']
    assert np.array_equal(row, wire['row'], equal_nan=True)
    # The loaded dict feeds restore_state verbatim.
    tire.reset()
    assert not tire.states['front'].carcass_state.initialized
    assert equilibrium_cache.restore_state(sim.physical, dict(state)) is True
    assert np.array_equal(tire.state_dict()['row'], row, equal_nan=True)
    assert tire.states['front'].carcass_state.initialized
