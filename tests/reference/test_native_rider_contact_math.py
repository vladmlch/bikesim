"""Finite supports and grip laws: unchanged Python algorithms are byte oracles."""
import math
import os
from pathlib import Path
import sys

import numpy as np
import pytest

from bike_sim.sim.ride import support_geometry as geometry
from bike_sim.sim.ride.rider_contacts import grip_step
from bike_sim.physics.grip_release import release_if_overloaded
from _bits import assert_bitwise_equal

BUILD = Path(__file__).resolve().parents[2] / 'native' / 'build'
selected = os.environ.get('NATIVE_TEST_BUILD_DIR', '')
if selected not in ('', 'asan'):
    raise ValueError('NATIVE_TEST_BUILD_DIR must be empty or asan')
if selected:
    BUILD /= selected
sys.path.insert(0, str(BUILD))
import bike_native
assert Path(bike_native.__file__).resolve().parent == BUILD.resolve()


def rotation(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0., s], [0., 1., 0.], [-s, 0., c]])


def bits(actual, expected):
    assert_bitwise_equal(np.asarray(actual, dtype=float), np.asarray(expected, dtype=float))


@pytest.mark.parametrize('angle', [0., .2, -.7, math.pi/2, math.pi, 2.4])
@pytest.mark.parametrize('local', [[0., 0., 0.], [.1, 0., .02], [.2, 0., .04],
    [-.2, 0., -.04], [.1+1e-15, 0., .02+1e-15], [-0., -0., -0.],
    [.125, .04+1e-9, .025], [.125+1e-15, .04+1e-9+1e-15, .025]])
def test_box_pad_oracle(angle, local):
    origin, half, radius = np.array([.3, -.2, .5]), np.array([.1, .04, .02]), .025
    rot = rotation(angle)
    center = origin + rot @ local
    expected = geometry.box_pad_contact(center, radius, origin, rot, half)
    actual = bike_native.rider_box_pad_contact(center, radius, origin, rot, half)
    assert set(actual) == {'point_m', 'normal', 'tangent', 'gap_m', 'within_width', 'within_footprint'}
    for key in actual:
        wanted = getattr(expected, key)
        if isinstance(wanted, bool):
            assert actual[key] is wanted
        else:
            bits(actual[key], wanted)


def test_box_random_and_medial_ties():
    rng = np.random.default_rng(5744)
    for _ in range(200):
        rot = rotation(rng.uniform(-math.pi, math.pi))
        origin = rng.uniform(-2., 2., 3)
        half = rng.uniform(.01, .2, 3)
        center = origin + rot @ rng.uniform(-.3, .3, 3)
        expected = geometry.box_pad_contact(center, .02, origin, rot, half)
        actual = bike_native.rider_box_pad_contact(center, .02, origin, rot, half)
        for key in ('point_m', 'normal', 'tangent', 'gap_m'):
            bits(actual[key], getattr(expected, key))
        assert actual['within_width'] == expected.within_width
        assert actual['within_footprint'] == expected.within_footprint
    for angle in (0., math.pi/2, math.pi):
        rot = rotation(angle)
        expected = geometry.box_pad_contact(np.zeros(3), .01, np.zeros(3), rot, np.array([.1, .1, .1]))
        actual = bike_native.rider_box_pad_contact(np.zeros(3), .01, np.zeros(3), rot, np.array([.1, .1, .1]))
        bits(actual['normal'], expected.normal)


@pytest.mark.parametrize('angle', [0., .3, math.pi/4, math.pi/2, math.pi, -.9])
def test_upper_face_oracle(angle):
    args = (np.array([.3, .2, -.4]), rotation(angle), np.array([.1, .04, .02]))
    for actual, expected in zip(bike_native.rider_upper_box_face(*args), geometry.upper_box_face(*args)):
        bits(actual, expected)


@pytest.mark.parametrize('angle', [0., .2, .9, math.pi/2, math.pi, -.4])
@pytest.mark.parametrize('compression', [-.002, 0., .001, .02, .2])
@pytest.mark.parametrize('sole_x', [0., .07, .15, 2.])
def test_sole_height_oracle(angle, compression, sole_x):
    args = (np.zeros(3), rotation(angle), np.array([.1, .04, .02]), sole_x, .035, .012, compression)
    try:
        expected = geometry.sole_target_height(*args)
    except (geometry.UnreachableSoleTarget, RuntimeError) as exc:
        cls = bike_native.UnreachableSoleTarget if isinstance(exc, geometry.UnreachableSoleTarget) else RuntimeError
        with pytest.raises(cls, match=str(exc)):
            bike_native.rider_sole_target_height(*args)
    else:
        bits(bike_native.rider_sole_target_height(*args), expected)


@pytest.mark.parametrize('angle', [0., .4, math.pi/2, math.pi, -.8])
@pytest.mark.parametrize('compression,shear', [(-.01, 0.), (.001, .02), (.1, 0.), (.1, 3.), (.0, -3.)])
def test_project_goal_oracle(angle, compression, shear):
    args = (np.array([.3, -.2, .5]), rotation(angle), np.array([.1, .04, .02]), .035, .012, compression, shear)
    try:
        expected, diagnostic = geometry.project_sole_goal(*args)
    except (geometry.UnreachableSoleTarget, RuntimeError) as exc:
        cls = bike_native.UnreachableSoleTarget if isinstance(exc, geometry.UnreachableSoleTarget) else RuntimeError
        with pytest.raises(cls, match=str(exc)):
            bike_native.rider_project_sole_goal(*args)
        return
    actual, actual_diagnostic = bike_native.rider_project_sole_goal(*args)
    bits(actual, expected)
    assert set(actual_diagnostic) == set(diagnostic)
    for key, wanted in diagnostic.items():
        if isinstance(wanted, (bool, list)):
            assert actual_diagnostic[key] == wanted
        else:
            bits(actual_diagnostic[key], wanted)


@pytest.mark.parametrize('k,c,dt', [(4000., 150., .0005), (1., 0., .1), (1e-100, 1e-50, 1e-20)])
def test_grip_energy_oracle(k, c, dt):
    rng = np.random.default_rng(1982)
    for xi, velocity in [(np.array([.003, -.002, .001]), np.array([.1, 0., -.2])),
                          (np.array([-0., 0., -0.]), np.zeros(3))] + [tuple(rng.normal(size=(2, 3))) for _ in range(150)]:
        expected = grip_step(xi, velocity, k, c, dt)
        actual = bike_native.rider_grip_step(xi, velocity, k, c, dt)
        for got, wanted in zip(actual, expected):
            bits(got, wanted)


@pytest.mark.parametrize('force', [[3., 4., 0.], [-0., 0., -0.], [1e308, 1e308, 1e308]])
@pytest.mark.parametrize('limit', [np.nextafter(5., 0.), 5., np.nextafter(5., math.inf)])
def test_grip_release_oracle(force, limit):
    with np.errstate(over='ignore'):
        expected = release_if_overloaded(force, .2, limit)
    actual = bike_native.rider_release_if_overloaded(force, .2, limit)
    bits(actual[0], expected[0]); bits(actual[1], expected[1])
    assert actual[2] is expected[2]


def test_three_coordinate_hypot_oracle():
    rng = np.random.default_rng(98343)
    values = [(0., -0., 0.), (1e308, 1e308, 1e308), (5e-324, 5e-324, 5e-324),
              (math.inf, math.nan, 1.), (math.nan, 1., 2.)]
    values += [tuple(rng.normal(size=3)*math.ldexp(1., int(rng.integers(-1000, 1000)))) for _ in range(1000)]
    for value in values:
        actual = bike_native._rider_hypot3(value)
        expected = math.hypot(*value)
        if math.isnan(expected):
            assert math.isnan(actual)
        else:
            bits(actual, expected)


@pytest.mark.parametrize('entry,args', [
    ('rider_grip_step', ([0., 0.], [0., 0., 0.], 1., 0., .01)),
    ('rider_grip_step', ([0., math.nan, 0.], [0., 0., 0.], 1., 0., .01)),
    ('rider_grip_step', ([0., 0., 0.], [0., 0., 0.], 0., 0., .01)),
    ('rider_grip_step', ([0., 0., 0.], [0., 0., 0.], 1., -1., .01)),
    ('rider_grip_step', ([0., 0., 0.], [0., 0., 0.], 1., 0., 0.)),
    ('rider_release_if_overloaded', ([0., math.inf, 0.], 0., 1.)),
    ('rider_release_if_overloaded', ([0., 0., 0.], -1., 1.)),
    ('rider_release_if_overloaded', ([0., 0., 0.], 0., 0.)),
])
def test_invalid_grip(entry, args):
    with pytest.raises(ValueError):
        getattr(bike_native, entry)(*args)


def test_grip_overflow():
    with pytest.raises(ValueError, match='grip update overflow'):
        bike_native.rider_grip_step([1e308]*3, [1e308]*3, 1e308, 0., .1)


@pytest.mark.parametrize('field,value', [('center', [0., 0.]), ('center', [0., math.nan, 0.]),
    ('radius', 0.), ('origin', [math.inf, 0., 0.]), ('half', [0., .1, .1]),
    ('rotation', np.eye(2)), ('rotation', np.diag([-1., 1., 1.])),
    ('rotation', np.array([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]]))])
def test_invalid_box(field, value):
    args = dict(center=np.zeros(3), radius=.01, origin=np.zeros(3), rotation=np.eye(3), half=np.ones(3))
    args[field] = value
    with pytest.raises(ValueError):
        bike_native.rider_box_pad_contact(*args.values())


def test_outputs_own_storage_and_convert_dtype():
    xi = np.array([1., 2., 3.], dtype=np.float32)
    velocity = np.arange(6, dtype=np.float32)[::2]
    actual = bike_native.rider_grip_step(xi, velocity, 1., 0., .01)
    expected = grip_step(xi, velocity, 1., 0., .01)
    xi[:] = 9.; velocity[:] = 9.
    for got, wanted in zip(actual, expected):
        bits(got, wanted)
    for array in actual[:2]:
        assert array.dtype == np.float64
        assert array.flags.writeable
    first = bike_native.rider_box_pad_contact(np.zeros(3), .01, np.zeros(3), np.eye(3), np.ones(3))
    second = bike_native.rider_box_pad_contact(np.zeros(3), .01, np.zeros(3), np.eye(3), np.ones(3))
    first['normal'][:] = 999.
    bits(second['normal'], geometry.box_pad_contact(np.zeros(3), .01, np.zeros(3), np.eye(3), np.ones(3)).normal)


@pytest.mark.parametrize('joint,geom,quat', [
    ('<joint type="hinge" axis="0 1 0"/>', 'box', '1 0 0 0'),
    ('<joint type="hinge" axis="0 -1 0"/>', 'box', '1 0 0 0'),
    ('<joint type="slide" axis="1 0 1"/>', 'box', '1 0 0 0'),
    ('<joint type="hinge" axis="1 0 0"/>', 'box', '1 0 0 0'),
    ('<joint type="slide" axis="0 1 0"/>', 'box', '1 0 0 0'),
    ('<freejoint/>', 'box', '1 0 0 0'),
    ('<joint type="ball"/>', 'box', '1 0 0 0'),
    ('<joint type="hinge" axis="0 1 0"/>', 'sphere', '1 0 0 0'),
    ('<joint type="hinge" axis="0 1 0"/>', 'box', '.9238795 .3826834 0 0'),
])
def test_planar_model_validation_oracle(tmp_path, joint, geom, quat):
    import mujoco
    # An unrelated nonplanar free body must not affect the support ancestors.
    model = mujoco.MjModel.from_xml_string(f'<mujoco><worldbody><body>{joint}<geom name="support" type="{geom}" size=".1 .04 .02" quat="{quat}" mass="1"/></body><body pos="1 0 1"><freejoint/><geom type="sphere" size=".1" mass="1"/></body></worldbody></mujoco>')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    model_path = tmp_path/'support.mjb'
    mujoco.mj_saveModel(model, str(model_path))
    stepper = bike_native.Stepper(str(model_path))
    stepper.forward()
    geom_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'support'))
    try:
        geometry.validate_planar_support_model(model, data, [geom_id])
    except ValueError as exc:
        with pytest.raises(ValueError, match=str(exc)):
            bike_native._rider_validate_support_model(stepper, [geom_id])
    else:
        bike_native._rider_validate_support_model(stepper, [geom_id])
    with pytest.raises(ValueError, match='invalid rider support geom'):
        bike_native._rider_validate_support_model(stepper, [-1])
    with pytest.raises(ValueError, match='invalid rider support geom'):
        bike_native._rider_validate_support_model(stepper, [model.ngeom])


@pytest.mark.parametrize('field,value', [('k', True), ('c', '0'), ('dt', math.inf)])
def test_grip_scalar_validation_oracle(field, value):
    args = dict(xi=np.zeros(3), velocity=np.zeros(3), k=1., c=0., dt=.01)
    args[field] = value
    with pytest.raises(ValueError):
        grip_step(*args.values())
    with pytest.raises(ValueError):
        bike_native.rider_grip_step(*args.values())


@pytest.mark.parametrize('energy,limit', [(True, 2.), (False, 1.), (.2, True)])
def test_release_preserves_reference_numeric_bool_inputs(energy, limit):
    force = np.array([1., 0., -0.])
    expected = release_if_overloaded(force, energy, limit)
    actual = bike_native.rider_release_if_overloaded(force, energy, limit)
    bits(actual[0], expected[0]); bits(actual[1], expected[1])
    assert actual[2] is expected[2]
