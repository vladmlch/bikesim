"""Native rider-contacts config: the optional section's projection and schema.

The section mirrors what RiderContactApplier reads: every ArticulatedConfig
field its force path consumes (physical_config.py ArticulatedConfig), the
pose-resolved arm reach, and the three closed attachment discriminators.
Writer behaviour tests land with the writer wave; this file pins the wire
shape the config parser will consume.
"""
import copy
from dataclasses import replace

import mujoco
import pytest

from native_loader import load_native

bike_native = load_native()

# The documented field set (docs plan Task 3 / tools.native_schema SCHEMAS):
# every key is required except grip_pair_force_limit_n, which ArticulatedConfig
# declares as float | None.
REQUIRED_FIELDS = frozenset((
    'arm_reach_m', 'saddle_patch_half_length_m', 'pedal_patch_half_length_m',
    'support_pad_radius_m', 'support_k_n_m', 'support_c_ns_m', 'pedal_c_ns_m',
    'support_tangent_k_n_m', 'support_mu', 'support_length_m', 'grip_k_n_m',
    'grip_c_ns_m', 'grip_release_distance_m', 'grip_capture_distance_m',
    'grip_capture_speed_mps', 'pedal_attachment', 'saddle_attachment',
    'grip_attachment'))
OPTIONAL_FIELDS = frozenset(('grip_pair_force_limit_n',))


@pytest.fixture(scope='module')
def env(tmp_path_factory):
    from test_pinned_topology import _model
    return _model(tmp_path_factory.mktemp('rider_contacts'))[1]


def test_projection_copies_the_appliers_resolved_setup(env):
    from tools.native_config import project_rider_contacts
    contacts = env.sim.physical.rider_contacts
    assert contacts is not None
    section = project_rider_contacts(contacts)
    assert set(section) == REQUIRED_FIELDS | OPTIONAL_FIELDS
    cfg = contacts.config
    for key in REQUIRED_FIELDS - {'arm_reach_m'}:
        assert section[key] == getattr(cfg, key), key
    # arm_reach_m is resolved by the applier from the seated pose, not a
    # config field — the wire carries the runtime value.
    assert section['arm_reach_m'] == contacts.arm_reach > 0.
    for key, value in section.items():
        assert value is None or type(value) in (int, float, bool, str), key


def test_projection_emits_an_explicit_pair_limit(env):
    from tools.native_config import project_rider_contacts
    contacts = env.sim.physical.rider_contacts
    assert contacts.config.grip_pair_force_limit_n is None
    assert project_rider_contacts(contacts)['grip_pair_force_limit_n'] is None
    original = contacts.config
    try:
        contacts.config = replace(original, grip_pair_force_limit_n=300.)
        assert project_rider_contacts(contacts)['grip_pair_force_limit_n'] == 300.
    finally:
        contacts.config = original


def test_project_gates_the_section_on_the_applier(env):
    from tools.native_config import project
    sim = env.sim
    physical = sim.physical
    contacts = physical.rider_contacts
    assert contacts is not None
    assert 'rider_contacts' in project(env)
    try:
        physical.rider_contacts = None
        assert 'rider_contacts' not in project(env)
        sim.physical = None
        assert 'rider_contacts' not in project(env)
    finally:
        physical.rider_contacts = contacts
        sim.physical = physical


def test_schema_accepts_the_projected_section(env, tmp_path):
    from tools.native_config import project
    from tools.native_schema import validate_config
    config = project(env)
    validate_config(config)
    path = tmp_path / 'rider_contacts.mjb'
    mujoco.mj_saveModel(env.sim.model, str(path))
    bike_native.Stepper(str(path), config)


@pytest.mark.parametrize('mutation', ['missing', 'extra', 'nonstring'])
def test_schema_rejects_bad_section_keys(env, tmp_path, mutation):
    from tools.native_config import project
    from tools.native_schema import validate_config
    path = tmp_path / f'{mutation}.mjb'
    mujoco.mj_saveModel(env.sim.model, str(path))
    config = project(env)
    if mutation == 'missing':
        del config['rider_contacts']['arm_reach_m']
        expected = 'rider_contacts.arm_reach_m'
    elif mutation == 'extra':
        config['rider_contacts']['misspelled'] = 1.
        expected = 'rider_contacts.misspelled'
    else:
        config['rider_contacts'][7] = 1.
        expected = 'rider_contacts'
    for reader in (validate_config,
                   lambda cfg: bike_native.Stepper(str(path), cfg)):
        with pytest.raises(ValueError, match=expected):
            reader(config)


@pytest.mark.parametrize('field,bad', [
    ('pedal_attachment', 'pin'), ('saddle_attachment', 'spindle'),
    ('grip_attachment', 'weld'), ('pedal_attachment', 7),
    ('arm_reach_m', 0.), ('arm_reach_m', float('inf')),
    ('saddle_patch_half_length_m', -1e-3), ('support_pad_radius_m', 0.),
    ('support_k_n_m', -1.), ('support_k_n_m', float('nan')),
    ('support_c_ns_m', -1.), ('pedal_c_ns_m', -1.),
    ('support_tangent_k_n_m', 0.), ('support_mu', -1.),
    ('support_length_m', 0.), ('grip_k_n_m', 0.), ('grip_c_ns_m', -1.),
    ('grip_release_distance_m', 0.), ('grip_capture_distance_m', -1.),
    ('grip_capture_speed_mps', -1.), ('grip_pair_force_limit_n', -3.),
    ('grip_pair_force_limit_n', 'unlimited'),
])
def test_schema_rejects_bad_section_values(env, field, bad):
    from tools.native_config import project
    from tools.native_schema import validate_config
    config = project(env)
    config['rider_contacts'][field] = bad
    with pytest.raises(ValueError, match=f'rider_contacts.{field}'):
        validate_config(config)


def test_schema_accepts_every_declared_attachment(env):
    from tools.native_config import project
    from tools.native_schema import validate_config
    config = project(env)
    for field, labels in [('pedal_attachment', ('flat', 'weld', 'spindle')),
                          ('saddle_attachment', ('flat', 'weld', 'pin')),
                          ('grip_attachment', ('spring', 'connect'))]:
        for label in labels:
            config['rider_contacts'][field] = label
            validate_config(config)


def test_pair_limit_is_the_only_optional_key(env):
    from tools.native_config import project
    from tools.native_schema import validate_config
    config = project(env)
    del config['rider_contacts']['grip_pair_force_limit_n']
    validate_config(config)
    for key in REQUIRED_FIELDS:
        candidate = copy.deepcopy(config)
        del candidate['rider_contacts'][key]
        with pytest.raises(ValueError, match=f'rider_contacts.{key}'):
            validate_config(candidate)
