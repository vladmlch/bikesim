"""Strict wire readers and explicit fork legacy metadata, with valid controls."""
import copy
from collections import UserDict
from collections.abc import Sequence
from dataclasses import replace
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

from native_loader import load_native
from native_safety_helpers import drive_pair, freeze
from test_native_drive_policies import applier
from tools.native_config import project, project_drive_policies


def dictionaries(value, path=()):
    if isinstance(value, dict):
        yield path
        for key, item in value.items():
            yield from dictionaries(item, (*path, key))
    elif isinstance(value, (list, tuple)):
        for index, item in enumerate(value):
            yield from dictionaries(item, (*path, index))


def at(value, path):
    for key in path:
        value = value[key]
    return value


@pytest.fixture(scope='module')
def resolved(tmp_path_factory):
    from test_native_suspension import _golden
    env, directory = _golden(tmp_path_factory.mktemp('strict'), steps=1)
    config = project(env)
    config['cruise'] = dict(target_speed_kmh=25., kp_nm_per_mps=1.,
                            ki_nm_per_mps_s=.1, torque_ceiling_nm=20.)
    config['drive']['assist']['profile'] = {
        'mode_gains': {'eco': 1., 'tour': 2., 'emtb': [1., 3.], 'turbo': 3.},
        'emtb_full_gain_at_nm': 20.}
    config['drive']['assist']['mode'] = 'eco'
    # The seated plant disables the legacy rider writer. Include a valid
    # unlimited-slide path so its wire section also participates in the matrix.
    config.setdefault('rider_forces', {'paths': [dict(name='probe_path',
        joint='root_x', stiffness_n_m=1.,
        damping_ns_m=0., preload_deflection_m=0., offset_m=0., unilateral=False)]})
    if not config['tire']['surface_map']['sections']:
        config['tire']['surface_map']['sections'] = [dict(start_m=1., end_m=2.,
            surface=copy.deepcopy(config['tire']['surface_map']['surface']))]
    return env, str(directory / 'model.mjb'), config


@pytest.mark.parametrize('mutation', ['unknown', 'nonstring', 'missing'])
def test_recursive_config_key_contract(resolved, mutation):
    from tools.native_config import validate_config
    _, path, original = resolved
    load_native().Stepper(path, original)  # same path's valid control
    for location in dictionaries(original):
        candidate = copy.deepcopy(original)
        section = at(candidate, location)
        if mutation == 'missing':
            # Root writer sections and profile/curve/surface_map/pair-limit
            # are optional.
            keys = set(section) - ({'drive', 'cruise', 'suspension', 'brake',
                'resistance', 'tire', 'rider_forces', 'rider_contacts'}
                if not location else
                {'profile', 'torque_curve', 'surface_map', 'grip_pair_force_limit_n'})
            if not keys:
                continue
            key = sorted(keys)[0]
            del section[key]
        else:
            key = 'misspelled' if mutation == 'unknown' else 7
            section[key] = 1.
        expected = '.'.join(map(str, ('config', *location)))
        for reader in (validate_config, lambda value: load_native().Stepper(path, value)):
            with pytest.raises(ValueError, match=expected):
                reader(candidate)


@pytest.mark.parametrize('section', ['drive', 'cruise', 'suspension', 'brake', 'resistance', 'tire', 'rider_forces', 'rider_contacts'])
def test_present_empty_writer_requires_its_schema(resolved, section):
    from tools.native_config import validate_config
    _, path, _ = resolved
    config = {'schema': 2, section: {}}
    for reader in (validate_config, lambda value: load_native().Stepper(path, value)):
        with pytest.raises(ValueError, match=f"config.{section}"):
            reader(config)


@pytest.mark.parametrize('bad', [True, np.bool_(False), '2', 2.5, 0, 3])
def test_schema_is_closed_integer(resolved, bad):
    _, path, _ = resolved
    with pytest.raises(ValueError, match='config.schema'):
        load_native().Stepper(path, {'schema': bad})


@pytest.mark.parametrize('bad', [1, 0., 'no', np.bool_(True)])
def test_boolean_flags_are_exact(resolved, bad):
    _, path, original = resolved
    for location in [('suspension', 'shock_damper', 'lockout_firm'),
                     ('suspension', 'coil', 'legacy_behavior'),
                     ('drive', 'pedaling', 'enabled')]:
        config = copy.deepcopy(original)
        at(config, location[:-1])[location[-1]] = bad
        with pytest.raises(ValueError, match='.'.join(location)):
            load_native().Stepper(path, config)


@pytest.mark.parametrize('bad', [True, np.bool_(True), '1', float('nan'), float('inf')])
def test_numeric_types_precede_conversion(resolved, bad):
    _, path, original = resolved
    for location in [('brake', 'torque_ceiling_nm'), ('drive', 'assist', 'gain'),
                     ('resistance', 'wind_world_mps', 0)]:
        config = copy.deepcopy(original)
        at(config, location[:-1])[location[-1]] = bad
        with pytest.raises(ValueError, match='.'.join(map(str, location[:-1]))):
            load_native().Stepper(path, config)


def test_numpy_numeric_scalars_and_ordered_vectors(resolved):
    _, path, original = resolved
    config = copy.deepcopy(original)
    config['schema'] = np.int64(2)
    config['drive']['gearing']['front_teeth'] = np.int32(34)
    config['drive']['assist']['gain'] = np.float32(1.)
    config['resistance']['wind_world_mps'] = (np.int64(0), np.float64(0.), 0.)
    load_native().Stepper(path, config)


@pytest.mark.parametrize('bad', [{0., 1., 2.}, iter([0., 0., 0.]), [0., True, 0.], [0., 0.]])
def test_vectors_reject_unordered_wrong_width_and_boolean(resolved, bad):
    _, path, original = resolved
    config = copy.deepcopy(original)
    config['resistance']['wind_world_mps'] = bad
    with pytest.raises(ValueError, match='resistance.wind_world_mps'):
        load_native().Stepper(path, config)


@pytest.mark.parametrize('field,bad', [('enabled', 'false'), ('stop_time_s', True)])
def test_projection_rejects_original_mutated_attributes(field, bad):
    drive = applier()
    object.__setattr__(drive.pedaling.config, field, bad)
    with pytest.raises(ValueError, match=f'pedaling.{field}'):
        project_drive_policies(drive)


def test_projection_canonicalizes_valid_real_integer_after_validation():
    drive = applier()
    drive.assist.gain = np.int64(2)
    projected = project_drive_policies(drive)
    assert type(projected['assist']['gain']) is float
    assert type(projected['gearing']['front_teeth']) is int


@pytest.mark.parametrize('field', ['drive_mode', 'transmission_model', 'upshift_slip_mode', 'profile_mode'])
def test_standalone_projection_checks_closed_discriminators(field):
    from tools.native_config import project_drive
    from bike_sim.physics.motor_profile import BOSCH_CX_GEN4
    drive = applier()
    if field == 'drive_mode':
        drive.drive_mode = 'unsupported'
    elif field == 'transmission_model':
        object.__setattr__(drive.config, field, 'unsupported')
    elif field == 'upshift_slip_mode':
        object.__setattr__(drive.shifting.config, field, 'unsupported')
    else:
        drive.assist.profile = BOSCH_CX_GEN4
        drive.assist.mode = 'unsupported'
    with pytest.raises(ValueError, match='config.drive'):
        project_drive(drive)


@pytest.mark.parametrize('travel,legacy', [(150., True), (180., False), (150., False)])
@pytest.mark.parametrize('schema', [1, 2])
def test_schema_migration_preserves_resolved_fork_force(resolved, travel, legacy, schema):
    from bike_sim.physics.damper import Charger3Damper
    from tools.native_config import _damper_core
    from _bits import assert_bitwise_equal
    env, path, original = resolved
    damper = Charger3Damper(total_travel_mm=travel, legacy_behavior=legacy)
    assert damper.legacy_behavior is legacy
    config = copy.deepcopy(original)
    config['schema'] = schema
    config['suspension']['fork_damper'] = {**_damper_core(damper),
        'total_travel_mm': damper.total_travel_mm, 'hbo_start_mm': damper.hbo_start_mm,
        'c_hbo_base': damper.c_hbo_base}
    if schema == 2:
        config['suspension']['fork_damper']['legacy_behavior'] = legacy
    native = load_native().Stepper(path, config)
    ap = env.sim.applier
    for compression in [travel - 1., travel + 15.]:
        qpos = env.sim.data.qpos.copy()
        qvel = np.zeros(env.sim.model.nv)
        qpos[ap.fork_qposadr] = compression / 1000.
        qvel[ap.fork_dofadr] = .7
        native.set_state(qpos, qvel, np.zeros(env.sim.model.na), np.zeros(env.sim.model.nv), 0.)
        actual = native.suspension_components()['fork_damper'][ap.fork_dofadr]
        assert_bitwise_equal(actual, -damper.compute_damping_force(.7, compression))


def test_schema_two_requires_explicit_fork_flag(resolved):
    _, path, original = resolved
    config = copy.deepcopy(original)
    config['schema'] = 2
    config['suspension']['fork_damper'].pop('legacy_behavior', None)
    with pytest.raises(ValueError, match='fork_damper.legacy_behavior'):
        load_native().Stepper(path, config)


def test_projection_emits_schema_two_and_persisted_flag(resolved):
    env, _, _ = resolved
    config = project(env)
    assert config['schema'] == 2
    assert config['suspension']['fork_damper']['legacy_behavior'] is False


@pytest.mark.parametrize('key', ['misspelled', 7])
@pytest.mark.parametrize('kind', ['elastic_chain', 'ideal_mid_drive', 'geometric_ideal_mid_drive'])
def test_control_and_recursive_state_keys_are_value_errors(tmp_path, key, kind):
    _, _, _, native = drive_pair(tmp_path, kind=kind)
    native.drive_components({}, .001, 0.)
    original = native.drive_state()
    for location in dictionaries(original):
        candidate = copy.deepcopy(original)
        at(candidate, location)[key] = 1.
        with pytest.raises(ValueError, match='state'):
            native.set_drive_state(candidate)
        assert freeze(native.drive_state()) == freeze(original)
    with pytest.raises(ValueError, match='control'):
        native.drive_components({key: 1.}, .001, 0.)


def test_state_numeric_arrays_reject_boolean_before_cast(tmp_path):
    _, _, _, native = drive_pair(tmp_path)
    state = native.drive_state()
    state['ideal_hub']['coefficients'] = [True] * len(state['ideal_hub']['coefficients'])
    with pytest.raises(ValueError, match='state.ideal_hub.coefficients'):
        native.set_drive_state(state)


@pytest.mark.parametrize('bad', [[True], np.array([True]), {1.}, iter([1.])])
def test_public_numeric_arrays_are_checked_before_conversion(tmp_path, bad):
    from bike_sim.physics.checks import array
    _, _, _, native = drive_pair(tmp_path)
    with pytest.raises(ValueError):
        array(bad, 'qpos')
    with pytest.raises(ValueError, match='set_state.qpos'):
        native.set_state(bad, native.qvel, [], np.zeros(len(native.qvel)), 0.)
    with pytest.raises(ValueError, match='total'):
        native.total({'force': bad})


def test_public_state_accepts_ordered_sequences(tmp_path):
    _, _, _, native = drive_pair(tmp_path)
    native.set_state(list(native.qpos), tuple(native.qvel), [],
                     [0.] * len(native.qvel), np.float64(0.))


def test_dynamic_result_keys_are_strings(tmp_path):
    _, _, _, native = drive_pair(tmp_path)
    with pytest.raises(ValueError, match='total'):
        native.total({7: np.zeros(len(native.qvel))})


@pytest.mark.parametrize('name,args', [
    ('pedaling_update', (0., 1., 80., 10., True)),
    ('assist_ceiling', (True, 0.)),
    ('battery_draw', (True, .001)),
    ('freehub_torque', (True, 0., 0., 0.)),
])
def test_policy_call_numeric_bools_are_rejected(name, args):
    native = load_native().DrivePolicies(project_drive_policies(applier()))
    with pytest.raises(ValueError, match=name):
        getattr(native, name)(*args)


@pytest.mark.parametrize('bad', [[], None, 1, 'config'])
def test_public_config_requires_mapping_before_model_load(tmp_path, bad):
    from tools.native_config import validate_config
    for reader in (validate_config,
                   lambda value: load_native().Stepper(str(tmp_path / 'missing.mjb'), value),
                   load_native().DrivePolicies):
        with pytest.raises(ValueError, match='config'):
            reader(bad)


@pytest.mark.parametrize('location', [
    ('suspension', 'physics_mode'), ('tire', 'backend'), ('tire', 'surface_mode'),
    ('drive', 'drive_mode'), ('drive', 'transmission_model'),
    ('drive', 'shifting', 'upshift_slip_mode'), ('drive', 'assist', 'mode')])
def test_closed_discriminators_have_full_field_paths(resolved, location):
    from tools.native_config import validate_config
    _, path, original = resolved
    candidate = copy.deepcopy(original)
    at(candidate, location[:-1])[location[-1]] = 'unsupported'
    for reader in (validate_config, lambda value: load_native().Stepper(path, value)):
        with pytest.raises(ValueError, match='.'.join(('config', *location))):
            reader(candidate)


def test_schema_one_rejects_schema_two_fork_metadata(resolved):
    from tools.native_config import validate_config
    _, path, original = resolved
    candidate = copy.deepcopy(original)
    candidate['schema'] = 1
    for reader in (validate_config, lambda value: load_native().Stepper(path, value)):
        with pytest.raises(ValueError, match='fork_damper.legacy_behavior'):
            reader(candidate)


@pytest.mark.parametrize('mutation', ['unknown', 'nonstring', 'missing'])
def test_resistance_snapshot_patch_schema_is_complete(resolved, mutation):
    _, path, config = resolved
    native = load_native().Stepper(path, config)
    side = dict(patch_loads=np.array([100.]), patch_working=np.array([True]), eff_radius=.3,
                patches=[dict(normal_load_n=100., tangent_force_n=0., slip_mps=0., working_surface=True)])
    snapshots = {'front': copy.deepcopy(side), 'rear': copy.deepcopy(side)}
    native.resistance_components(snapshots)
    patch = snapshots['front']['patches'][0]
    if mutation == 'missing':
        del patch['working_surface']
    else:
        patch['misspelled' if mutation == 'unknown' else 7] = 1.
    with pytest.raises(ValueError, match='resistance_components.front.patches.0'):
        native.resistance_components(snapshots)


@pytest.mark.parametrize('name', ['chain_geometry', 'chain_center_gradient'])
def test_chain_boundary_preserves_original_numeric_types(name):
    from bike_sim.physics import chain
    oracle = getattr(chain, name)
    native = getattr(load_native(), name)
    for front in ([True, 0.], {0., 1.}, iter([0., 0.])):
        # Use a fresh iterator for each reader; neither reader consumes sets.
        if not isinstance(front, (list, set)):
            front = iter([0., 0.])
        with pytest.raises(ValueError):
            oracle(front, [-.5, .1], .068, .048)
        if not isinstance(front, (list, set)):
            front = iter([0., 0.])
        with pytest.raises(ValueError, match=name):
            native(front, [-.5, .1], .068, .048, [0., 1.], None)


@pytest.mark.parametrize('location', [('resistance', 'wind_world_mps'),
    ('drive', 'shifting', 'cassette'), ('drive', 'assist', 'torque_curve')])
def test_zero_dimensional_arrays_are_not_wire_sequences(resolved, location):
    from tools.native_config import validate_config
    _, path, original = resolved
    candidate = copy.deepcopy(original)
    at(candidate, location[:-1])[location[-1]] = np.array(1.)
    for reader in (validate_config, lambda value: load_native().Stepper(path, value)):
        with pytest.raises(ValueError, match='.'.join(('config', *location))):
            reader(candidate)


@pytest.mark.parametrize('bad', [{24, 34}, iter([24, 34]), np.array(24)])
def test_projection_does_not_turn_unordered_or_scalar_cassette_into_sequence(bad):
    drive = applier()
    object.__setattr__(drive.shifting.config, 'cassette', bad)
    with pytest.raises(ValueError, match='config.drive.shifting.cassette'):
        project_drive_policies(drive)


class OrderedRows(Sequence):
    def __init__(self, values):
        self.values = tuple(values)

    def __len__(self):
        return len(self.values)

    def __getitem__(self, index):
        return self.values[index]


@pytest.mark.parametrize('location', [('tire', 'surface_map', 'sections'), ('rider_forces', 'paths')])
@pytest.mark.parametrize('container', [OrderedRows, lambda rows: np.array(rows, dtype=object)])
def test_ordered_dictionary_collections_match_python(resolved, location, container):
    from tools.native_config import validate_config
    _, path, original = resolved
    candidate = copy.deepcopy(original)
    at(candidate, location[:-1])[location[-1]] = container(at(original, location))
    validate_config(candidate)
    load_native().Stepper(path, candidate)


def test_mapping_is_not_a_numeric_sequence(resolved):
    from tools.native_config import validate_config
    _, path, original = resolved
    candidate = copy.deepcopy(original)
    # Iterating the mapping would yield [0, 1], a valid load range, making
    # accidental mapping-to-sequence conversion observable independently of domains.
    candidate['tire']['front']['material']['valid_load_range_n'] = UserDict({0: 0., 1: 100.})
    for reader in (validate_config, lambda value: load_native().Stepper(path, value)):
        with pytest.raises(ValueError, match='tire.front.material.valid_load_range_n'):
            reader(candidate)


def test_scalar_conversion_memory_error_keeps_its_meaning(resolved):
    from tools.native_config import validate_config
    class AllocationFailure(int):
        def __float__(self):
            raise MemoryError('injected scalar conversion')
    _, path, original = resolved
    candidate = copy.deepcopy(original)
    candidate['drive']['assist']['gain'] = AllocationFailure(2)
    for reader in (validate_config, lambda value: load_native().Stepper(path, value)):
        with pytest.raises(MemoryError, match='injected scalar conversion'):
            reader(candidate)


def test_empty_boolean_ndarray_is_not_numeric_state(tmp_path):
    from bike_sim.physics.checks import array
    _, _, _, native = drive_pair(tmp_path)
    empty = np.array([], dtype=bool)
    with pytest.raises(ValueError):
        array(empty, 'act', (0,))
    with pytest.raises(ValueError, match='set_state.act'):
        native.set_state(native.qpos, native.qvel, empty, np.zeros(len(native.qvel)), 0.)


def test_huge_integer_real_conversion_and_large_finite_control():
    from bike_sim.physics.pedaling import human_crank_torque
    from _bits import assert_bitwise_equal
    for reader in (human_crank_torque, load_native().human_crank_torque):
        assert_bitwise_equal(reader(1e300, 0., 0.), 1e300)
        with pytest.raises(ValueError, match='mean'):
            reader(10**1000, 0.)


@pytest.mark.parametrize('mode', ['custom', '自定义', '\ud800'])
def test_wire_string_encoding_matches_native(mode):
    from tools.native_schema import validate_drive_policies
    config = project_drive_policies(applier())
    config['assist']['mode'] = mode
    for reader in (validate_drive_policies, load_native().DrivePolicies):
        if mode == '\ud800':
            with pytest.raises(ValueError, match='assist.mode'):
                reader(config)
        else:
            reader(config)
