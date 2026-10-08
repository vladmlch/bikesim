"""Native rider-contacts config: the optional section's projection and schema,
plus the model-owned writer core (T3b-1).

The section mirrors what RiderContactApplier reads: every ArticulatedConfig
field its force path consumes (physical_config.py ArticulatedConfig), the
pose-resolved arm reach, and the three closed attachment discriminators.

The second half of this file pins the RiderContactWriter core against the
unchanged Python oracle on genuine tiny MuJoCo models: pad force assembly,
grip release/capture, diagnostics trees, probe publication, snapshot
round-trip and restoration atomicity. The T3b-2 tail adds the attachment
settle face: prepare_attachment_raw's pose capture, settle_welds' solved-
reaction latch (and the crank-torque sensor read on the next qfrc), and
attachment_samples' optional interval-state measurement.
"""
import copy
from dataclasses import asdict, replace
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

from _bits import assert_bitwise_equal
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


# --------------------------------------------------------------------------
# T3b-1 writer core: tiny genuine models, oracle-vs-native behavioral parity.
#
# The tiny plant below carries every literal name RiderContactApplier
# resolves: a planar frame (slide x/z + hinge y), a steer child, a crank
# with `crank_spin`, hinge pedals whose box geoms are the finite supports,
# freejoint rider bodies, and the five support/grip sites plus three support
# box geoms. Attachment variants add only the equalities the selected
# attachment kind consumes — exactly the conditional resolution the writer
# must reproduce.
# --------------------------------------------------------------------------

SIDES = ('left', 'right')
SUPPORT_NAMES = ('saddle', 'front_pedal', 'rear_pedal')
CONTACT_NAMES = SUPPORT_NAMES + ('grip',)
PAD_KEYS = tuple(f'{name}:{i}' for name in SUPPORT_NAMES for i in range(2))


def _contact_xml(pedal='flat', saddle='flat', grip='spring',
                 front_slide=False):
    # front_slide replaces the front foot's freejoint with a y-slide: its
    # in-plane attachment force is then genuinely unobservable (the same
    # degenerate-dof construction as _yslide_model in
    # test_native_attachment_wrench.py).
    front_joint = ('<joint name="foot_front_y" type="slide" axis="0 1 0"/>'
                   if front_slide else '<freejoint/>')
    equalities = []
    if pedal == 'weld':
        equalities += [f'<weld name="weld_foot_{s}" body1="rider_foot_{s}" body2="pedal_{s}"/>'
                       for s in ('front', 'rear')]
    elif pedal == 'spindle':
        equalities += [f'<connect name="connect_foot_{s}" body1="rider_foot_{s}" '
                       f'body2="pedal_{s}" anchor="{p}"/>' for s, p in
                       (('front', '0.25 0.04 0.795'), ('rear', '0.25 -0.04 0.51'))]
    if saddle == 'weld':
        equalities += ['<weld name="weld_saddle" body1="rider_pelvis" body2="frame"/>']
    elif saddle == 'pin':
        equalities += ['<connect name="connect_saddle" body1="rider_pelvis" body2="frame" '
                       'anchor="0 0 1.47"/>']
    if grip == 'connect':
        equalities += [f'<connect name="connect_grip_{s}" body1="rider_forearm_{s}" '
                       f'body2="steer" anchor="0.44 {y} 1.5"/>'
                       for s, y in (('left', '0.12'), ('right', '-0.12'))]
    eq = f"<equality>{''.join(equalities)}</equality>" if equalities else ''
    return f'''<mujoco>
  <option gravity="0 0 -9.81" timestep="0.0005"/>
  <worldbody>
    <body name="frame" pos="0 0 1">
      <joint name="frame_x" type="slide" axis="1 0 0"/>
      <joint name="frame_z" type="slide" axis="0 0 1"/>
      <joint name="frame_lean" type="hinge" axis="0 1 0"/>
      <inertial pos="0 0 0" mass="10" diaginertia="1 1 1"/>
      <geom name="geom_saddle" type="box" size="0.15 0.06 0.03" pos="0 0 0.45"
            contype="0" conaffinity="0"/>
      <body name="steer" pos="0.45 0 0.5">
        <joint name="steer_hinge" type="hinge" axis="0 1 0"/>
        <inertial pos="0 0 0" mass="1" diaginertia="0.05 0.05 0.05"/>
        <geom type="sphere" size="0.02" contype="0" conaffinity="0"/>
      </body>
      <body name="crank" pos="0.25 0 -0.35">
        <joint name="crank_spin" type="hinge" axis="0 1 0"/>
        <inertial pos="0 0 0" mass="1" diaginertia="0.05 0.05 0.05"/>
        <geom type="sphere" size="0.02" contype="0" conaffinity="0"/>
        <body name="pedal_front" pos="0 0.04 0.14">
          <joint name="pedal_front_tilt" type="hinge" axis="0 1 0"/>
          <inertial pos="0 0 0" mass="0.2" diaginertia="0.005 0.005 0.005"/>
          <geom name="geom_pedal_front" type="box" size="0.09 0.05 0.012"
                contype="0" conaffinity="0"/>
        </body>
        <body name="pedal_rear" pos="0 -0.04 -0.14">
          <joint name="pedal_rear_tilt" type="hinge" axis="0 1 0"/>
          <inertial pos="0 0 0" mass="0.2" diaginertia="0.005 0.005 0.005"/>
          <geom name="geom_pedal_rear" type="box" size="0.09 0.05 0.012"
                contype="0" conaffinity="0"/>
        </body>
      </body>
    </body>
    <body name="rider_pelvis" pos="0 0 1.51">
      <freejoint/>
      <inertial pos="0 0 0" mass="20" diaginertia="1 1 1"/>
      <site name="site_rider_saddle" pos="0 0 -0.04"/>
      <geom type="sphere" size="0.02" contype="0" conaffinity="0"/>
    </body>
    <body name="rider_foot_front" pos="0.25 0.04 0.83">
      {front_joint}
      <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
      <site name="site_rider_sole_front" pos="0 0 -0.035"/>
      <geom type="sphere" size="0.02" contype="0" conaffinity="0"/>
    </body>
    <body name="rider_foot_rear" pos="0.25 -0.04 0.545">
      <freejoint/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
      <site name="site_rider_sole_rear" pos="0 0 -0.035"/>
      <geom type="sphere" size="0.02" contype="0" conaffinity="0"/>
    </body>
    <body name="rider_upper_arm_left" pos="0.32 0.12 1.52">
      <freejoint/>
      <inertial pos="0 0 0" mass="1.5" diaginertia="0.02 0.02 0.02"/>
      <geom type="sphere" size="0.02" contype="0" conaffinity="0"/>
    </body>
    <body name="rider_upper_arm_right" pos="0.32 -0.12 1.52">
      <freejoint/>
      <inertial pos="0 0 0" mass="1.5" diaginertia="0.02 0.02 0.02"/>
      <geom type="sphere" size="0.02" contype="0" conaffinity="0"/>
    </body>
    <body name="rider_forearm_left" pos="0.44 0.12 1.5">
      <freejoint/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
      <site name="site_rider_grip_left"/>
      <geom type="sphere" size="0.02" contype="0" conaffinity="0"/>
    </body>
    <body name="rider_forearm_right" pos="0.44 -0.12 1.5">
      <freejoint/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
      <site name="site_rider_grip_right"/>
      <geom type="sphere" size="0.02" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  {eq}
</mujoco>'''


def _pose(shoulder=(0.4, 0., 0.2), elbow=(0.2, 0., 0.1), grip=(0., 0., 0.)):
    # geometry_pose()'s arm chain — the applier resolves arm_reach from it.
    return SimpleNamespace(shoulder=np.asarray(shoulder, float),
                           elbow=np.asarray(elbow, float),
                           grip=np.asarray(grip, float))


def _joint(model, name):
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    return int(model.jnt_qposadr[jid]), int(model.jnt_dofadr[jid])


def _body_qpos(model, name):
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    jid = int(model.body_jntadr[body])
    assert model.jnt_type[jid] == mujoco.mjtJoint.mjJNT_FREE, name
    return int(model.jnt_qposadr[jid]), int(model.jnt_dofadr[jid])


def _plant(tmp_path, pedal='flat', saddle='flat', grip='spring', pose=None,
           cfg_overrides=None, name='contacts', front_slide=False):
    from bike_sim.physics.physical_config import ArticulatedConfig
    from bike_sim.sim.ride.rider_contacts import RiderContactApplier
    from tools.native_config import project_rider_contacts
    model = mujoco.MjModel.from_xml_string(
        _contact_xml(pedal=pedal, saddle=saddle, grip=grip,
                     front_slide=front_slide))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    cfg = replace(ArticulatedConfig(), pedal_attachment=pedal,
                  saddle_attachment=saddle, grip_attachment=grip,
                  **(cfg_overrides or {}))
    applier = RiderContactApplier(model, pose or _pose(), cfg)
    applier.reset(model, data)
    section = project_rider_contacts(applier)
    path = tmp_path / f'{name}.mjb'
    mujoco.mj_saveModel(model, str(path))
    stepper = bike_native.Stepper(
        str(path), {'schema': 2, 'rider_contacts': section})
    return model, data, applier, stepper


def _mirror(stepper, data):
    """Carry the oracle's mjData state onto the Stepper's owned pair."""
    stepper.set_state(np.asarray(data.qpos), np.asarray(data.qvel),
                      np.asarray(data.act), np.asarray(data.qacc_warmstart),
                      float(data.time))
    stepper.forward()


def _reset_pair(stepper, applier, model, data):
    """reset(model, data) on both owners at the same kinematic state."""
    applier.reset(model, data)
    _mirror(stepper, data)
    stepper.rider_contacts_reset()


def assert_tree(actual, expected, path='root'):
    """Bitwise-recursive tree comparison: dicts by key set, sequences
    elementwise, ndarrays by raw bytes, floats by float64 bits, everything
    else by (type, value) — including signed zeros and inf."""
    if hasattr(expected, '__dataclass_fields__'):
        expected = asdict(expected)
    if isinstance(expected, dict):
        assert isinstance(actual, dict), f'{path}: {type(actual)} is not a dict'
        assert set(actual) == set(expected), \
            f'{path}: keys {sorted(set(actual) ^ set(expected))} differ'
        for key, value in expected.items():
            assert_tree(actual[key], value, f'{path}.{key}')
        return
    if isinstance(expected, (list, tuple)):
        assert type(actual) is type(expected), \
            f'{path}: {type(actual)} != {type(expected)}'
        assert len(actual) == len(expected), f'{path}: length differs'
        for i, (a, e) in enumerate(zip(actual, expected)):
            assert_tree(a, e, f'{path}[{i}]')
        return
    if isinstance(expected, np.ndarray):
        assert_bitwise_equal(actual, expected, path)
        return
    if isinstance(expected, (float, np.floating)):
        a, e = np.float64(actual), np.float64(expected)
        assert a.tobytes() == e.tobytes(), \
            f'{path}: {actual!r} != {expected!r}'
        return
    assert type(actual) is type(expected) and actual == expected, \
        f'{path}: {actual!r} != {expected!r}'


def oracle_state(applier):
    """The writer snapshot schema, re-derived from the oracle's own
    attributes — set_rider_contacts_state must consume this shape."""
    settled = None
    if applier._settled_welds is not None:
        welds, crank = applier._settled_welds
        settled = ({name: dict(entry) for name, entry in welds.items()}, crank)
    return {
        'enabled': {name: bool(applier.enabled[name]) for name in CONTACT_NAMES},
        'supports': {key: {'xi': float(state.xi),
                           'tangent': None if state.tangent is None
                           else np.asarray(state.tangent, float)}
                     for key, state in applier.states.items()},
        'grip_xi_local': {s: np.asarray(applier.grip_xi_local[s], float)
                          for s in SIDES},
        'grip_anchor_local': {s: None if applier.grip_anchor_local[s] is None
                              else np.asarray(applier.grip_anchor_local[s], float)
                              for s in SIDES},
        'elastic_energy_j': float(applier.elastic_energy_j),
        'loss_step_j': float(applier.loss_step_j),
        'radial_dissipation_power_w': float(applier.radial_dissipation_power_w),
        'delivered_crank_torque_nm': float(applier.delivered_crank_torque_nm),
        'last_time_s': applier.last_time_s,
        'pending_release_loss_j': float(applier.pending_release_loss_j),
        'diagnostics': copy.deepcopy(applier.diagnostics),
        'settled_welds': settled,
        'last_attachment_samples': dict(applier.last_attachment_samples),
        'last_attachment_errors': tuple(applier.last_attachment_errors),
        'probe_diagnostics': getattr(applier, 'probe_diagnostics', None),
        'probe_enabled': getattr(applier, 'probe_enabled', None),
        'probe_delivered_crank_torque_nm':
            getattr(applier, 'probe_delivered_crank_torque_nm', None),
    }


def test_flat_contacts_qfrc_and_diagnostics_bitwise(tmp_path):
    model, data, applier, stepper = _plant(tmp_path)
    _body_qpos(model, 'rider_pelvis')  # sanity: freejoint resolution works
    _, fx = _joint(model, 'frame_x')
    _, fz = _joint(model, 'frame_z')
    data.qvel[fx] = 0.06
    data.qvel[fz] = -0.08
    mujoco.mj_forward(model, data)
    _reset_pair(stepper, applier, model, data)
    dt = float(model.opt.timestep)
    expected = applier.compute_qfrc(model, data, dt)
    actual = stepper.rider_contacts_qfrc(dt)
    assert_bitwise_equal(actual, expected, 'qfrc')
    assert_tree(stepper.rider_contacts_diagnostics(), applier.diagnostics)
    assert_tree(stepper.rider_contacts_state(), oracle_state(applier))


def test_qfrc_advances_material_state_across_steps(tmp_path):
    model, data, applier, stepper = _plant(tmp_path)
    _reset_pair(stepper, applier, model, data)
    dt = float(model.opt.timestep)
    _, tilt = _joint(model, 'pedal_front_tilt')
    # Three advancing evaluations; the pedal hinge tilts between the second
    # and third so tangent transport and face-switch losses engage.
    for step, angle in enumerate((0., 0., 0.7)):
        if step:
            data.qpos[_joint(model, 'pedal_front_tilt')[0]] = angle
        _, fz = _joint(model, 'frame_z')
        _, fx = _joint(model, 'frame_x')
        data.qvel[fx] = 0.04 * (step + 1)
        data.qvel[fz] = -0.03
        data.time += dt
        mujoco.mj_forward(model, data)
        _mirror(stepper, data)
        expected = applier.compute_qfrc(model, data, dt)
        actual = stepper.rider_contacts_qfrc(dt)
        assert_bitwise_equal(actual, expected, f'qfrc[{step}]')
        assert_tree(stepper.rider_contacts_diagnostics(), applier.diagnostics)
        assert_tree(stepper.rider_contacts_state(), oracle_state(applier))


def test_detailed_false_omits_exactly_the_reference_keys(tmp_path):
    model, data, applier, stepper = _plant(tmp_path)
    _reset_pair(stepper, applier, model, data)
    dt = float(model.opt.timestep)
    expected = applier.compute_qfrc(model, data, dt, detailed=False)
    actual = stepper.rider_contacts_qfrc(dt, detailed=False)
    assert_bitwise_equal(actual, expected, 'qfrc')
    diag = stepper.rider_contacts_diagnostics()
    assert_tree(diag, applier.diagnostics)
    for name in SUPPORT_NAMES:
        assert set(diag[name]) == {'enabled', 'in_platform', 'normal_load_n',
                                   'gap_m', 'vertical_force_on_rider_n'}
    for side in SIDES:
        assert set(diag[f'grip_{side}']) == {
            'enabled', 'reachable', 'overloaded', 'trial_pair_force_n',
            'pair_force_limit_n', 'release_loss_j', 'shoulder_distance_m',
            'arm_reach_m', 'hand_gap_m', 'point_m', 'force_on_rider_n',
            'force_on_bike_n', 'elastic_energy_j'}
    assert set(diag['grip']) == {
        'enabled', 'reachable', 'overloaded', 'trial_pair_force_n',
        'pair_force_limit_n', 'release_loss_j', 'shoulder_distance_m',
        'arm_reach_m', 'hand_gap_m', 'force_on_rider_n', 'force_on_bike_n',
        'elastic_energy_j'}


def test_nonadvancing_probe_publishes_only_probe_fields(tmp_path):
    model, data, applier, stepper = _plant(tmp_path)
    _reset_pair(stepper, applier, model, data)
    dt = float(model.opt.timestep)
    applier.compute_qfrc(model, data, dt)
    stepper.rider_contacts_qfrc(dt)
    live_diag = copy.deepcopy(applier.diagnostics)
    live_state = oracle_state(applier)
    # A probe at the same timestamp must not trip the duplicate-time guard
    # nor touch live state.
    expected = applier.compute_qfrc(model, data, dt, advance=False)
    actual = stepper.rider_contacts_qfrc(dt, advance=False)
    assert_bitwise_equal(actual, expected, 'probe qfrc')
    assert_tree(stepper.rider_contacts_diagnostics(probe=True),
                applier.probe_diagnostics)
    assert_tree(stepper.rider_contacts_state()['probe_enabled'],
                applier.probe_enabled)
    state = stepper.rider_contacts_state()
    assert state['probe_delivered_crank_torque_nm'] == \
        applier.probe_delivered_crank_torque_nm
    assert applier.diagnostics == live_diag
    # Full post-probe parity (probe fields included), plus the proof that
    # publication left every live field at its pre-probe value.
    post = oracle_state(applier)
    assert_tree(state, post)
    for key, value in live_state.items():
        if key.startswith('probe_'):
            continue
        assert_tree(post[key], value, f'live.{key}')


def test_duplicate_time_guard(tmp_path):
    model, data, applier, stepper = _plant(tmp_path)
    _reset_pair(stepper, applier, model, data)
    dt = float(model.opt.timestep)
    applier.compute_qfrc(model, data, dt)
    stepper.rider_contacts_qfrc(dt)
    for advancing in (lambda: applier.compute_qfrc(model, data, dt),
                      lambda: stepper.rider_contacts_qfrc(dt)):
        with pytest.raises(ValueError, match='advances only once per timestamp'):
            advancing()


def test_restart_clock_versus_reset(tmp_path):
    model, data, applier, stepper = _plant(tmp_path)
    _reset_pair(stepper, applier, model, data)
    dt = float(model.opt.timestep)
    applier.compute_qfrc(model, data, dt)
    stepper.rider_contacts_qfrc(dt)
    # restart_clock clears only the clock: the carried xi/diagnostics stay.
    applier.restart_clock()
    stepper.rider_contacts_restart_clock()
    applier.compute_qfrc(model, data, dt)
    stepper.rider_contacts_qfrc(dt)
    assert_tree(stepper.rider_contacts_state(), oracle_state(applier))
    # reset re-derives anchors and wipes material state, clock included.
    _reset_pair(stepper, applier, model, data)
    fresh = copy.deepcopy(oracle_state(applier))
    assert_tree(stepper.rider_contacts_state(), fresh)
    assert all(stepper.rider_contacts_state()['supports'][key]['xi'] == 0.
               for key in PAD_KEYS)


def test_release_all_and_pending_release_loss(tmp_path):
    model, data, applier, stepper = _plant(tmp_path)
    _reset_pair(stepper, applier, model, data)
    dt = float(model.opt.timestep)
    # Build grip shear and pad xi before release so pending_release_loss_j
    # carries real stored energy on both paths.
    _, fx = _joint(model, 'frame_x')
    data.qvel[fx] = 0.15
    for side in SIDES:
        _, dof = _body_qpos(model, f'rider_forearm_{side}')
        data.qvel[dof + 1] = 0.1
    data.time += dt
    mujoco.mj_forward(model, data)
    _mirror(stepper, data)
    expected = applier.compute_qfrc(model, data, dt)
    actual = stepper.rider_contacts_qfrc(dt)
    assert_bitwise_equal(actual, expected, 'qfrc')
    applier.release_all()
    stepper.rider_contacts_release_all()
    assert stepper.rider_contacts_state()['pending_release_loss_j'] == \
        applier.pending_release_loss_j
    assert stepper.rider_contacts_state()['enabled'] == applier.enabled
    # The next advancing evaluation folds the pending loss into loss_step_j.
    data.time += dt
    mujoco.mj_forward(model, data)
    _mirror(stepper, data)
    expected = applier.compute_qfrc(model, data, dt)
    actual = stepper.rider_contacts_qfrc(dt)
    assert_bitwise_equal(actual, expected, 'post-release qfrc')
    assert_tree(stepper.rider_contacts_state(), oracle_state(applier))


def test_set_enabled_capture_success_and_failure(tmp_path):
    model, data, applier, stepper = _plant(tmp_path)
    _reset_pair(stepper, applier, model, data)
    dt = float(model.opt.timestep)
    applier.compute_qfrc(model, data, dt)
    stepper.rider_contacts_qfrc(dt)
    # Release both hands, then re-enable with hands still at the anchors:
    # the capture gate (gap + speed) passes on both implementations.
    applier.set_enabled('grip', False)
    assert not stepper.rider_contacts_set_enabled('grip', False)
    assert applier.set_enabled('grip', True)
    assert stepper.rider_contacts_set_enabled('grip', True)
    assert_tree(stepper.rider_contacts_state(), oracle_state(applier))
    # Release again, displace one forearm beyond the capture distance, and
    # the re-enable attempt must fail on both owners.
    applier.set_enabled('grip', False)
    assert not stepper.rider_contacts_set_enabled('grip', False)
    qpos, _ = _body_qpos(model, 'rider_forearm_left')
    data.qpos[qpos] += 0.2
    data.time += dt
    mujoco.mj_forward(model, data)
    _mirror(stepper, data)
    assert not applier.set_enabled('grip', True)
    assert not stepper.rider_contacts_set_enabled('grip', True)
    assert_tree(stepper.rider_contacts_state(), oracle_state(applier))


def test_unreachable_hand_releases_and_reports_loss(tmp_path):
    model, data, applier, stepper = _plant(tmp_path)
    _reset_pair(stepper, applier, model, data)
    dt = float(model.opt.timestep)
    applier.compute_qfrc(model, data, dt)
    stepper.rider_contacts_qfrc(dt)
    qpos, _ = _body_qpos(model, 'rider_forearm_right')
    data.qpos[qpos + 2] += 0.4  # far beyond grip_release_distance_m
    data.time += dt
    mujoco.mj_forward(model, data)
    _mirror(stepper, data)
    expected = applier.compute_qfrc(model, data, dt)
    actual = stepper.rider_contacts_qfrc(dt)
    assert_bitwise_equal(actual, expected, 'release qfrc')
    assert_tree(stepper.rider_contacts_diagnostics(), applier.diagnostics)
    assert_tree(stepper.rider_contacts_state(), oracle_state(applier))
    assert not applier.enabled['grip']


def test_pair_force_limit_releases_both_hands(tmp_path):
    model, data, applier, stepper = _plant(
        tmp_path, cfg_overrides={'grip_pair_force_limit_n': 1.})
    _reset_pair(stepper, applier, model, data)
    dt = float(model.opt.timestep)
    # Stretch the left spring past half the pair limit: the overloaded hand
    # releases and the cohesive pair drops the other hand with it.
    _, dof = _body_qpos(model, 'rider_forearm_left')
    data.qvel[dof] = 0.4
    mujoco.mj_forward(model, data)
    _mirror(stepper, data)
    expected = applier.compute_qfrc(model, data, dt)
    actual = stepper.rider_contacts_qfrc(dt)
    assert_bitwise_equal(actual, expected, 'overload qfrc')
    assert_tree(stepper.rider_contacts_diagnostics(), applier.diagnostics)
    assert not applier.enabled['grip']
    assert applier.diagnostics['grip_left']['overloaded']
    assert applier.diagnostics['grip_right']['overloaded']


def test_initialize_settled_state(tmp_path):
    model, data, applier, stepper = _plant(tmp_path)
    _reset_pair(stepper, applier, model, data)
    applier.initialize_settled_state(model, data)
    stepper.rider_contacts_initialize_settled_state()
    assert_tree(stepper.rider_contacts_state(), oracle_state(applier))
    # A used clock rejects the call on both owners.
    data.time += float(model.opt.timestep)
    mujoco.mj_forward(model, data)
    _mirror(stepper, data)
    applier.compute_qfrc(model, data, float(model.opt.timestep))
    stepper.rider_contacts_qfrc(float(model.opt.timestep))
    for call in (lambda: applier.initialize_settled_state(model, data),
                 stepper.rider_contacts_initialize_settled_state):
        with pytest.raises(ValueError, match='fresh clock'):
            call()


def test_linked_attachments_refuse_release(tmp_path):
    model, data, applier, stepper = _plant(
        tmp_path, pedal='weld', saddle='pin', grip='connect')
    _reset_pair(stepper, applier, model, data)
    dt = float(model.opt.timestep)
    expected = applier.compute_qfrc(model, data, dt)
    actual = stepper.rider_contacts_qfrc(dt)
    assert_bitwise_equal(actual, expected, 'welded qfrc')
    assert_tree(stepper.rider_contacts_diagnostics(), applier.diagnostics)
    assert_tree(stepper.rider_contacts_state(), oracle_state(applier))
    # Welded/pinned/connected contacts cannot be released: both owners
    # report True (still enabled) and leave the enabled map untouched.
    for name in CONTACT_NAMES:
        assert applier.set_enabled(name, False)
        assert stepper.rider_contacts_set_enabled(name, False)
    assert_tree(stepper.rider_contacts_state(), oracle_state(applier))


def test_welded_branch_null_latch_diagnostics(tmp_path):
    # Before any settle_welds call the latch is None: the welded branches
    # read the oracle's zeros (zero force/load/flags) plus the live
    # translation residual — pinned here against the oracle.
    for pedal, saddle, grip in (('weld', 'weld', 'connect'),
                                ('spindle', 'pin', 'connect')):
        model, data, applier, stepper = _plant(
            tmp_path, pedal=pedal, saddle=saddle, grip=grip,
            name=f'{pedal}_{saddle}')
        _reset_pair(stepper, applier, model, data)
        dt = float(model.opt.timestep)
        expected = applier.compute_qfrc(model, data, dt)
        actual = stepper.rider_contacts_qfrc(dt)
        assert_bitwise_equal(actual, expected, f'{pedal}/{saddle} qfrc')
        assert_tree(stepper.rider_contacts_diagnostics(),
                    applier.diagnostics)
        assert_tree(stepper.rider_contacts_state(), oracle_state(applier))
        diag = stepper.rider_contacts_diagnostics()
        for name in SUPPORT_NAMES:
            assert diag[name]['normal_load_n'] == 0.
            assert diag[name]['would_separate'] is False
            assert diag[name]['tangent_n'] == 0.
            assert np.asarray(diag[name]['tangent_force_n']).shape == (3,)
        assert diag['grip']['hand_gap_m'] == \
            max(diag[f'grip_{s}']['hand_gap_m'] for s in SIDES)


def test_snapshot_round_trip_and_evaluation_determinism(tmp_path):
    model, data, applier, stepper = _plant(tmp_path)
    _reset_pair(stepper, applier, model, data)
    dt = float(model.opt.timestep)
    _, fx = _joint(model, 'frame_x')
    data.qvel[fx] = 0.2
    _mirror(stepper, data)
    applier.compute_qfrc(model, data, dt)
    stepper.rider_contacts_qfrc(dt)
    snapshot = stepper.rider_contacts_state()
    assert_tree(snapshot, oracle_state(applier))
    # Restore the emitted dict on a fresh owner, plus the oracle's own
    # attribute-shaped state on a second one; the next evaluation must be
    # identical on all three.
    other_path = tmp_path / 'contacts_restored.mjb'
    mujoco.mj_saveModel(model, str(other_path))
    from tools.native_config import project_rider_contacts
    config = {'schema': 2, 'rider_contacts': project_rider_contacts(applier)}
    restored = bike_native.Stepper(str(other_path), config)
    restored.set_rider_contacts_state(copy.deepcopy(oracle_state(applier)))
    stepper.set_rider_contacts_state(copy.deepcopy(snapshot))
    data.time += dt
    mujoco.mj_forward(model, data)
    for s in (stepper, restored):
        _mirror(s, data)
    expected = applier.compute_qfrc(model, data, dt)
    assert_bitwise_equal(stepper.rider_contacts_qfrc(dt), expected,
                         'restored qfrc')
    assert_bitwise_equal(restored.rider_contacts_qfrc(dt), expected,
                         'oracle-state qfrc')
    assert_tree(restored.rider_contacts_state(), oracle_state(applier))


def test_snapshot_owns_its_storage(tmp_path):
    model, data, applier, stepper = _plant(tmp_path)
    _reset_pair(stepper, applier, model, data)
    dt = float(model.opt.timestep)
    applier.compute_qfrc(model, data, dt)
    stepper.rider_contacts_qfrc(dt)
    snapshot = stepper.rider_contacts_state()
    snapshot['supports']['saddle:0']['xi'] = 99.
    snapshot['grip_xi_local']['left'][:] = 5.
    snapshot['enabled']['grip'] = False
    fresh = stepper.rider_contacts_state()
    assert fresh['supports']['saddle:0']['xi'] != 99.
    assert fresh['enabled']['grip'] is True
    assert_bitwise_equal(fresh['grip_xi_local']['left'],
                         applier.grip_xi_local['left'])


def test_set_state_rejects_invalid_candidates_atomically(tmp_path):
    model, data, applier, stepper = _plant(tmp_path)
    _reset_pair(stepper, applier, model, data)
    dt = float(model.opt.timestep)
    applier.compute_qfrc(model, data, dt)
    stepper.rider_contacts_qfrc(dt)
    baseline = stepper.rider_contacts_state()

    def corrupt(**patch):
        candidate = copy.deepcopy(baseline)
        for key, value in patch.items():
            candidate[key] = value
        return candidate

    bad = [corrupt(enabled={'saddle': True}),
           corrupt(supports={'saddle:0': {'xi': 0., 'tangent': None}}),
           corrupt(grip_xi_local={'left': [0., 0., 0.]}),
           corrupt(elastic_energy_j=float('nan')),
           corrupt(last_time_s='now'),
           corrupt(diagnostics=[]),
           corrupt(settled_welds={'a': 1}),
           corrupt(probe_enabled={'grip': True}),
           corrupt(extra_key=0.)]
    nonfinite = copy.deepcopy(baseline)
    nonfinite['supports']['saddle:0']['xi'] = float('inf')
    bad.append(nonfinite)
    wrong_width = copy.deepcopy(baseline)
    wrong_width['supports']['saddle:0']['tangent'] = [1., 0.]
    bad.append(wrong_width)
    for candidate in bad:
        with pytest.raises(ValueError):
            stepper.set_rider_contacts_state(candidate)
    # Every rejection is all-or-nothing: the live state is untouched.
    assert_tree(stepper.rider_contacts_state(), baseline)
    restored = copy.deepcopy(baseline)
    stepper.set_rider_contacts_state(restored)
    assert_tree(stepper.rider_contacts_state(), baseline)


def test_no_writer_answers_missing_config_errors(tmp_path):
    model = mujoco.MjModel.from_xml_string(_contact_xml())
    path = tmp_path / 'bare.mjb'
    mujoco.mj_saveModel(model, str(path))
    stepper = bike_native.Stepper(str(path))
    for name in ('rider_contacts_reset', 'rider_contacts_restart_clock',
                 'rider_contacts_initialize_settled_state',
                 'rider_contacts_release_all', 'rider_contacts_stored_energy',
                 'rider_contacts_diagnostics', 'rider_contacts_state',
                 'rider_contacts_prepare_attachment_raw',
                 'rider_contacts_settle',
                 'rider_contacts_attachment_samples'):
        with pytest.raises(RuntimeError, match='rider_contacts'):
            getattr(stepper, name)()
    with pytest.raises(RuntimeError, match='rider_contacts'):
        stepper.rider_contacts_set_enabled('grip', False)
    with pytest.raises(RuntimeError, match='rider_contacts'):
        stepper.rider_contacts_qfrc(0.0005)
    with pytest.raises(RuntimeError, match='rider_contacts'):
        stepper.set_rider_contacts_state({})


# --------------------------------------------------------------------------
# T3b-2: the attachment settle/sample face.
#
# The settle face reads the SOLVED constraint arena (efc_type/efc_id/
# efc_force of the forward or step that just ran), so each parity check
# forwards both owners on the same pose and solver inputs before calling:
# _solved_mirror makes the Stepper's arena bitwise the oracle's.
# --------------------------------------------------------------------------


def _solved_mirror(stepper, model, data):
    """set_state + inputs + forward: the Stepper's efc arena is bitwise the
    arena the oracle's settle_welds reads."""
    stepper.set_state(np.asarray(data.qpos), np.asarray(data.qvel),
                      np.asarray(data.act), np.asarray(data.qacc_warmstart),
                      float(data.time))
    stepper.set_inputs(np.asarray(data.ctrl), np.asarray(data.qfrc_applied))
    stepper.forward()


def test_prepare_attachment_raw_matches_oracle(tmp_path):
    # prepare_attachment_raw (rider_contacts.py:221-269): geometry for every
    # linked support plus the welded-grip connect anchors, captured at the
    # incoming pose; unlinked attachments and spring grips produce nothing.
    for pedal, saddle, grip in (('weld', 'weld', 'connect'),
                                ('spindle', 'pin', 'connect'),
                                ('weld', 'pin', 'spring'),
                                ('spindle', 'weld', 'spring'),
                                ('weld', 'flat', 'connect'),
                                ('flat', 'flat', 'spring')):
        model, data, applier, stepper = _plant(
            tmp_path, pedal=pedal, saddle=saddle, grip=grip,
            name=f'prep_{pedal}_{saddle}_{grip}')
        _reset_pair(stepper, applier, model, data)
        expected_geometry, expected_errors = \
            applier.prepare_attachment_raw(model, data)
        geometry, errors = stepper.rider_contacts_prepare_attachment_raw()
        assert errors == expected_errors, (pedal, saddle, grip)
        assert_tree(geometry, expected_geometry)


def test_settle_welds_latches_solved_reactions(tmp_path):
    # settle_welds (rider_contacts.py:271-343): the per-support telemetry
    # dict of the CURRENT solve, latching _settled_welds and publishing
    # last_attachment_samples/last_attachment_errors.
    for pedal, saddle, grip in (('weld', 'weld', 'connect'),
                                ('spindle', 'pin', 'connect'),
                                ('weld', 'pin', 'spring'),
                                ('spindle', 'flat', 'connect'),
                                ('flat', 'flat', 'spring')):
        model, data, applier, stepper = _plant(
            tmp_path, pedal=pedal, saddle=saddle, grip=grip,
            name=f'settle_{pedal}_{saddle}_{grip}')
        _reset_pair(stepper, applier, model, data)
        # A real rider load so the equalities carry nonzero multipliers.
        _, pelvis_dof = _body_qpos(model, 'rider_pelvis')
        data.qfrc_applied[pelvis_dof + 2] = -40.
        mujoco.mj_forward(model, data)
        _solved_mirror(stepper, model, data)
        expected = applier.settle_welds(model, data)
        actual = stepper.rider_contacts_settle()
        assert_tree(actual, expected)
        # The latch and the published samples joined the snapshot.
        assert_tree(stepper.rider_contacts_state(), oracle_state(applier))


def test_settled_latch_feeds_next_qfrc(tmp_path):
    # The torque sensor and the welded-support diagnostics read the latch
    # on the NEXT compute_qfrc — linked supports must not add a second
    # physical force.
    model, data, applier, stepper = _plant(
        tmp_path, pedal='weld', saddle='weld', grip='connect', name='latch')
    _reset_pair(stepper, applier, model, data)
    mujoco.mj_forward(model, data)
    _solved_mirror(stepper, model, data)
    assert_tree(stepper.rider_contacts_settle(),
                applier.settle_welds(model, data))
    dt = float(model.opt.timestep)
    expected = applier.compute_qfrc(model, data, dt)
    actual = stepper.rider_contacts_qfrc(dt)
    assert_bitwise_equal(actual, expected, 'post-settle qfrc')
    assert_tree(stepper.rider_contacts_diagnostics(), applier.diagnostics)
    assert_tree(stepper.rider_contacts_state(), oracle_state(applier))
    # delivered_crank_torque_nm is the latched scalar, not a recomputation.
    state = stepper.rider_contacts_state()
    assert state['delivered_crank_torque_nm'] == \
        applier._settled_welds[1] != 0.


def test_attachment_samples_scalar_and_raw_match_oracle(tmp_path):
    # attachment_samples (rider_contacts.py:345-412): the scalar budget
    # dataclass per attachment, or the raw measurement block with raw=True.
    for pedal, saddle, grip in (('weld', 'weld', 'connect'),
                                ('spindle', 'pin', 'connect'),
                                ('weld', 'pin', 'spring')):
        model, data, applier, stepper = _plant(
            tmp_path, pedal=pedal, saddle=saddle, grip=grip,
            name=f'samples_{pedal}_{saddle}_{grip}')
        _reset_pair(stepper, applier, model, data)
        _, fz = _joint(model, 'frame_z')
        data.qvel[fz] = -0.05
        mujoco.mj_forward(model, data)
        _solved_mirror(stepper, model, data)
        for raw in (False, True):
            expected_samples, expected_errors = \
                applier.attachment_samples(model, data, raw=raw)
            samples, errors = \
                stepper.rider_contacts_attachment_samples(raw=raw)
            assert errors == expected_errors, (pedal, saddle, grip, raw)
            assert_tree(samples, expected_samples)


def test_settle_raw_with_prepared_uses_interval_start_geometry(tmp_path):
    # The raw period path: geometry captured at the interval-start pose is
    # paired with the post-solve multipliers — no second kinematics pass.
    model, data, applier, stepper = _plant(
        tmp_path, pedal='weld', saddle='weld', grip='connect',
        name='prepared_settle')
    _reset_pair(stepper, applier, model, data)
    expected_prepared = applier.prepare_attachment_raw(model, data)
    prepared = stepper.rider_contacts_prepare_attachment_raw()
    # Move to the interval-end pose and solve again.
    _, pelvis_qpos = _body_qpos(model, 'rider_pelvis')
    data.qpos[pelvis_qpos + 2] -= 0.01
    mujoco.mj_forward(model, data)
    _solved_mirror(stepper, model, data)
    expected = applier.settle_welds(model, data, raw=True,
                                  prepared=expected_prepared)
    actual = stepper.rider_contacts_settle(raw=True, prepared=prepared)
    # (welds dict, raw samples dict, errors tuple) — the oracle's triple.
    assert_tree(actual, expected)
    assert_tree(stepper.rider_contacts_state(), oracle_state(applier))
    # The prepared pair may also arrive as the oracle's own dataclass tuple.
    assert_tree(
        stepper.rider_contacts_settle(raw=True, prepared=expected_prepared),
        expected)


def test_attachment_samples_interval_state_restores_pose(tmp_path):
    # interval_state=(qpos, qvel) measures at the interval-start pose, then
    # restores the live buffers AND the derived kinematics on every exit.
    model, data, applier, stepper = _plant(
        tmp_path, pedal='weld', saddle='weld', grip='connect',
        name='interval')
    _reset_pair(stepper, applier, model, data)
    mujoco.mj_forward(model, data)
    _solved_mirror(stepper, model, data)
    interval_qpos = np.array(data.qpos, copy=True)
    interval_qvel = np.array(data.qvel, copy=True)
    interval_qpos[_joint(model, 'frame_x')[0]] += 0.02
    interval = (interval_qpos, interval_qvel)
    qpos_before = np.array(data.qpos, copy=True)
    qvel_before = np.array(data.qvel, copy=True)
    xpos_before = np.array(data.xpos, copy=True)
    expected_samples, expected_errors = \
        applier.attachment_samples(model, data, interval)
    # The oracle restored pose and kinematics through its try/finally.
    assert_bitwise_equal(data.qpos, qpos_before, 'oracle qpos')
    assert_bitwise_equal(data.qvel, qvel_before, 'oracle qvel')
    assert_bitwise_equal(data.xpos, xpos_before, 'oracle xpos')
    live_qpos = np.array(stepper.qpos, copy=True)
    live_qvel = np.array(stepper.qvel, copy=True)
    dt = float(model.opt.timestep)
    probe_before = stepper.rider_contacts_qfrc(dt, advance=False)
    samples, errors = stepper.rider_contacts_attachment_samples(interval)
    assert errors == expected_errors
    assert_tree(samples, expected_samples)
    assert_bitwise_equal(np.asarray(stepper.qpos), live_qpos,
                         'qpos restored')
    assert_bitwise_equal(np.asarray(stepper.qvel), live_qvel,
                         'qvel restored')
    # Kinematics were refreshed too: the pose-dependent probe is identical.
    assert_bitwise_equal(stepper.rider_contacts_qfrc(dt, advance=False),
                         probe_before, 'kinematics restored')


def test_attachment_samples_interval_restores_through_errors(tmp_path):
    # Rejected intervals never reach the buffers; an interval that still
    # collects per-attachment errors restores just like a clean one.
    model, data, applier, stepper = _plant(
        tmp_path, pedal='weld', saddle='weld', grip='connect',
        name='interval_err', front_slide=True)
    _reset_pair(stepper, applier, model, data)
    mujoco.mj_forward(model, data)
    _solved_mirror(stepper, model, data)
    live_qpos = np.array(stepper.qpos, copy=True)
    live_qvel = np.array(stepper.qvel, copy=True)
    for bad in ((np.zeros(model.nq + 1), np.zeros(model.nv)),
                (np.zeros(model.nq), np.zeros(model.nv + 1)),
                (np.full(model.nq, np.nan), np.zeros(model.nv)),
                (np.zeros(model.nq), np.full(model.nv, np.inf))):
        with pytest.raises(ValueError):
            stepper.rider_contacts_attachment_samples(bad)
    assert_bitwise_equal(np.asarray(stepper.qpos), live_qpos)
    assert_bitwise_equal(np.asarray(stepper.qvel), live_qvel)
    interval_qpos = np.array(data.qpos, copy=True)
    interval_qpos[_joint(model, 'frame_x')[0]] += 0.02
    interval = (interval_qpos, np.array(data.qvel, copy=True))
    expected_samples, expected_errors = \
        applier.attachment_samples(model, data, interval)
    samples, errors = stepper.rider_contacts_attachment_samples(interval)
    # At the shifted interval pose the live-pose wrenches no longer
    # explain — every measurement collects the generic
    # ':unobservable_attachment_wrench' spelling (never a fabricated
    # zero), and the live pose is still restored.
    assert errors == expected_errors
    assert len(errors) == 5 and samples == expected_samples == {}
    assert 'foot_front:unobservable_attachment_wrench' in errors
    assert_bitwise_equal(np.asarray(stepper.qpos), live_qpos,
                         'qpos restored through collected errors')
    assert_bitwise_equal(np.asarray(stepper.qvel), live_qvel,
                         'qvel restored through collected errors')


def test_attachment_unobservable_records_errors_not_zeros(tmp_path):
    # The y-slide front foot keeps a dof but no x/z observability: prepare
    # still captures its geometry (observable=False), the scalar path
    # collects the error, and the raw paths store the measurement.
    model, data, applier, stepper = _plant(
        tmp_path, pedal='weld', saddle='weld', grip='connect',
        name='unobservable', front_slide=True)
    _reset_pair(stepper, applier, model, data)
    mujoco.mj_forward(model, data)
    _solved_mirror(stepper, model, data)
    expected_geometry, expected_errors = \
        applier.prepare_attachment_raw(model, data)
    geometry, errors = stepper.rider_contacts_prepare_attachment_raw()
    assert errors == expected_errors == ()
    assert_tree(geometry, expected_geometry)
    assert geometry['foot_front']['observable'] is False
    # Scalar: 'foot_front' becomes an error string, not a fabricated zero.
    # (On this plant the grip measurements also collect the generic
    # ':unobservable_attachment_wrench' spelling — the wrenches fail the
    # Newton third-law check — identical on both sides either way.)
    expected_samples, expected_errors = \
        applier.attachment_samples(model, data)
    samples, errors = stepper.rider_contacts_attachment_samples()
    assert errors == expected_errors
    assert 'foot_front:unobservable_attachment_wrench' in errors
    assert 'foot_front' not in samples
    assert_tree(samples, expected_samples)
    # Raw: the measurement is stored with observable=False — the raw reader
    # defers validation (attachment_raw never validates by itself).
    expected_raw, expected_errors = \
        applier.attachment_samples(model, data, raw=True)
    raw, errors = stepper.rider_contacts_attachment_samples(raw=True)
    assert errors == expected_errors == ()
    assert raw['foot_front']['observable'] is False
    assert_tree(raw, expected_raw)
    # settle(raw=True, prepared) takes the same deferred-validation path.
    prepared_errors = ()
    expected = applier.settle_welds(
        model, data, raw=True,
        prepared=(expected_geometry, prepared_errors))
    actual = stepper.rider_contacts_settle(
        raw=True, prepared=(geometry, prepared_errors))
    assert_tree(actual, expected)
    assert actual[2] == expected[2] == ()
    assert_tree(stepper.rider_contacts_state(), oracle_state(applier))


def test_settled_snapshot_round_trip(tmp_path):
    # The latch and the published samples/errors are snapshot fields:
    # they must survive a restore onto a fresh owner bitwise, and the
    # restored owner must evaluate identically.
    model, data, applier, stepper = _plant(
        tmp_path, pedal='weld', saddle='weld', grip='connect',
        name='settled_snap')
    _reset_pair(stepper, applier, model, data)
    mujoco.mj_forward(model, data)
    _solved_mirror(stepper, model, data)
    stepper.rider_contacts_settle()
    applier.settle_welds(model, data)
    dt = float(model.opt.timestep)
    applier.compute_qfrc(model, data, dt)
    stepper.rider_contacts_qfrc(dt)
    snapshot = stepper.rider_contacts_state()
    assert_tree(snapshot, oracle_state(applier))
    from tools.native_config import project_rider_contacts
    other_path = tmp_path / 'settled_restored.mjb'
    mujoco.mj_saveModel(model, str(other_path))
    config = {'schema': 2, 'rider_contacts': project_rider_contacts(applier)}
    restored = bike_native.Stepper(str(other_path), config)
    restored.set_rider_contacts_state(copy.deepcopy(snapshot))
    # Oracle-shaped candidates restore too — the oracle dict's samples are
    # AttachmentSample dataclasses, parsed through the attribute path.
    oracle_restored = bike_native.Stepper(str(other_path), config)
    oracle_restored.set_rider_contacts_state(
        copy.deepcopy(oracle_state(applier)))
    data.time += dt
    mujoco.mj_forward(model, data)
    for s in (stepper, restored, oracle_restored):
        _mirror(s, data)
    expected = applier.compute_qfrc(model, data, dt)
    assert_bitwise_equal(restored.rider_contacts_qfrc(dt), expected,
                         'restored qfrc')
    assert_bitwise_equal(oracle_restored.rider_contacts_qfrc(dt), expected,
                         'oracle-state qfrc')
    assert_tree(restored.rider_contacts_state(), oracle_state(applier))
    # The snapshot owns its storage: mutating the emitted dict must not
    # touch the live latch or sample blocks.
    snapshot['settled_welds'][0]['saddle']['normal_n'] = -1.
    snapshot['last_attachment_samples']['foot_front']['normal_n'] = 7.
    snapshot['last_attachment_errors'] = ('bogus',)
    fresh = stepper.rider_contacts_state()
    assert fresh['settled_welds'][0]['saddle']['normal_n'] != -1.
    assert fresh['last_attachment_samples']['foot_front']['normal_n'] != 7.
    assert fresh['last_attachment_errors'] == ()


def test_settled_snapshot_rejects_malformed_attachment_fields(tmp_path):
    model, data, applier, stepper = _plant(
        tmp_path, pedal='weld', saddle='weld', grip='connect',
        name='settled_bad')
    _reset_pair(stepper, applier, model, data)
    mujoco.mj_forward(model, data)
    _solved_mirror(stepper, model, data)
    stepper.rider_contacts_settle()
    baseline = stepper.rider_contacts_state()
    bad = []
    candidate = copy.deepcopy(baseline)
    candidate['last_attachment_samples']['foot_front'] = {'kind': 'foot'}
    bad.append(candidate)
    candidate = copy.deepcopy(baseline)
    candidate['last_attachment_samples']['foot_front'] = 5.
    bad.append(candidate)
    candidate = copy.deepcopy(baseline)
    candidate['last_attachment_errors'] = ['ok', 5]
    bad.append(candidate)
    candidate = copy.deepcopy(baseline)
    candidate['settled_welds'][0]['saddle'] = {'force_on_rider_n': []}
    bad.append(candidate)
    for candidate in bad:
        with pytest.raises((ValueError, TypeError)):
            stepper.set_rider_contacts_state(candidate)
    assert_tree(stepper.rider_contacts_state(), baseline)
    # The samples map is name-open — the oracle serializes whichever
    # attachment names were measured — so an unknown-but-valid entry is
    # restored verbatim rather than rejected.
    extra = copy.deepcopy(baseline)
    extra['last_attachment_samples']['extra'] = \
        copy.deepcopy(extra['last_attachment_samples']['foot_front'])
    stepper.set_rider_contacts_state(extra)
    assert 'extra' in \
        stepper.rider_contacts_state()['last_attachment_samples']
    stepper.set_rider_contacts_state(baseline)
    assert_tree(stepper.rider_contacts_state(), baseline)


def test_set_enabled_rejects_invalid_requests(tmp_path):
    model, data, applier, stepper = _plant(tmp_path)
    _reset_pair(stepper, applier, model, data)
    for bad in (('bogus', True), ('grip', 1), ('grip', np.bool_(True)),
                (7, True)):
        with pytest.raises(ValueError, match='invalid rider contact enable'):
            applier.set_enabled(*bad)
        with pytest.raises(ValueError, match='invalid rider contact enable'):
            stepper.rider_contacts_set_enabled(*bad)


def test_evaluation_requires_initialized_anchors(tmp_path):
    model = mujoco.MjModel.from_xml_string(_contact_xml())
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    from bike_sim.physics.physical_config import ArticulatedConfig
    from bike_sim.sim.ride.rider_contacts import RiderContactApplier
    from tools.native_config import project_rider_contacts
    applier = RiderContactApplier(model, _pose(), ArticulatedConfig())
    section = project_rider_contacts(applier)
    path = tmp_path / 'unreset.mjb'
    mujoco.mj_saveModel(model, str(path))
    stepper = bike_native.Stepper(
        str(path), {'schema': 2, 'rider_contacts': section})
    _mirror(stepper, data)
    dt = float(model.opt.timestep)
    with pytest.raises(RuntimeError, match='initialize rider contact anchors'):
        applier.compute_qfrc(model, data, dt)
    with pytest.raises(RuntimeError, match='initialize rider contact anchors'):
        stepper.rider_contacts_qfrc(dt)


def test_dt_domain_rejection(tmp_path):
    model, data, applier, stepper = _plant(tmp_path)
    _reset_pair(stepper, applier, model, data)
    for bad in (0., -0.001, float('nan'), float('inf')):
        with pytest.raises(ValueError, match='invalid rider contact dt'):
            applier.compute_qfrc(model, data, bad)
        with pytest.raises(ValueError, match='invalid rider contact dt'):
            stepper.rider_contacts_qfrc(bad)
    for bad in ('dt', None, True):
        with pytest.raises(ValueError, match='finite real scalar'):
            applier.compute_qfrc(model, data, bad)
        with pytest.raises(ValueError):
            stepper.rider_contacts_qfrc(bad)


def test_stored_energy_matches_the_ledger(tmp_path):
    model, data, applier, stepper = _plant(tmp_path)
    _reset_pair(stepper, applier, model, data)
    dt = float(model.opt.timestep)
    _, fx = _joint(model, 'frame_x')
    data.qvel[fx] = 0.1
    data.time += dt
    mujoco.mj_forward(model, data)
    _mirror(stepper, data)
    applier.compute_qfrc(model, data, dt)
    stepper.rider_contacts_qfrc(dt)
    expected = applier.stored_energy(model, data)
    actual = stepper.rider_contacts_stored_energy()
    assert np.float64(actual).tobytes() == np.float64(expected).tobytes()


def test_missing_model_names_reject_construction(tmp_path):
    model = mujoco.MjModel.from_xml_string(
        _contact_xml().replace('name="geom_saddle"', 'name="geom_saddle_off"'))
    path = tmp_path / 'renamed.mjb'
    mujoco.mj_saveModel(model, str(path))
    with pytest.raises(ValueError, match="model has no 'geom_saddle'"):
        bike_native.Stepper(
            str(path),
            {'schema': 2,
             'rider_contacts': {'arm_reach_m': 0.4,
                                'saddle_patch_half_length_m': 0.045,
                                'pedal_patch_half_length_m': 0.025,
                                'support_pad_radius_m': 0.02,
                                'support_k_n_m': 1e5, 'support_c_ns_m': 500.,
                                'pedal_c_ns_m': 100.,
                                'support_tangent_k_n_m': 2e4,
                                'support_mu': 0.8, 'support_length_m': 0.1,
                                'grip_k_n_m': 4000., 'grip_c_ns_m': 150.,
                                'grip_release_distance_m': 0.12,
                                'grip_capture_distance_m': 0.02,
                                'grip_capture_speed_mps': 0.2,
                                'grip_pair_force_limit_n': None,
                                'pedal_attachment': 'flat',
                                'saddle_attachment': 'flat',
                                'grip_attachment': 'spring'}})
