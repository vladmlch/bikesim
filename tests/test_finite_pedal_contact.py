"""A flat pedal is a finite box, not an infinite one-sided half-space."""
from dataclasses import replace
import numpy as np
import mujoco
import pytest
from bike_sim.geometry.specs import BikeSpecs
from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.physics.rider import RiderSpecs
from bike_sim.physics.rider_segments import geometry_pose
from bike_sim.sim.ride.rider_contacts import RiderContactApplier


@pytest.fixture
def pedal_rig():
    cfg = SimulationPhysicsConfig('physical', drive_mode='articulated_effort')
    specs = BikeSpecs()
    rider = RiderSpecs(variant='articulated_planar')
    m = mujoco.MjModel.from_xml_string(generate_mujoco_xml(
        mode='ride', rider=rider, physics_config=cfg))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    contact = RiderContactApplier(m, geometry_pose(rider, specs), cfg.articulated)
    contact.reset(m, d)
    for name in ('saddle', 'rear_pedal', 'grip'):
        contact.set_enabled(name, False)
    return m, d, contact


def force_at(pedal_rig, phase, root_height):
    m, d, contact = pedal_rig
    d.qpos[m.joint('pedal_front_spin').qposadr[0]] = phase
    d.qpos[m.joint('rider_root_z').qposadr[0]] = root_height
    mujoco.mj_forward(m, d)
    contact.restart_clock()
    contact.compute_qfrc(m, d, m.opt.timestep)
    return np.array(contact.diagnostics['front_pedal']['force_on_rider_n'])


def test_inverted_platform_supports_the_foot_on_its_upper_face(pedal_rig):
    upright = force_at(pedal_rig, 0., -.003)
    inverted = force_at(pedal_rig, np.pi, -.003)
    assert upright[2] > 0.
    assert inverted[2] > 0., 'a flipped pedal must not pull a foot above it down'
    np.testing.assert_allclose(inverted, upright, atol=1e-10)


def test_a_foot_below_the_finite_platform_does_not_receive_a_ghost_force(pedal_rig):
    np.testing.assert_allclose(force_at(pedal_rig, 0., -.30), np.zeros(3), atol=1e-12)


@pytest.mark.parametrize('angle', np.linspace(-np.pi, np.pi, 33))
def test_finite_box_contact_has_no_force_at_a_distance(angle):
    from bike_sim.sim.ride.support_geometry import box_pad_contact
    R = np.array([[np.cos(angle), 0., np.sin(angle)], [0., 1., 0.],
                  [-np.sin(angle), 0., np.cos(angle)]])
    for z in (-.3, .3):
        result = box_pad_contact(np.array([0., 0., z]), .02,
                                 np.zeros(3), R, np.array([.05, .04, .008]))
        assert result.gap_m > .20


def test_rotated_corner_contact_uses_closest_feature_and_finite_width():
    from bike_sim.sim.ride.support_geometry import box_pad_contact
    result = box_pad_contact(np.array([.055, 0., .013]), .02,
                             np.zeros(3), np.eye(3), np.array([.05, .04, .008]))
    np.testing.assert_allclose(result.point_m, [.05, 0., .008], atol=1e-12)
    np.testing.assert_allclose(result.normal, [2**-.5, 0., 2**-.5], atol=1e-12)
    assert result.gap_m == pytest.approx(np.sqrt(2)*.005-.02)
    outside = box_pad_contact(np.array([0., .10, .008]), .02,
                              np.zeros(3), np.eye(3), np.array([.05, .04, .008]))
    assert not outside.within_width


def test_support_target_does_not_follow_an_inverted_normal(pedal_rig):
    from bike_sim.sim.ride.rider_control import ArticulatedRiderController
    m, d, contact = pedal_rig
    ctrl = ArticulatedRiderController(m, contact.pose, contact.config, .165)
    a = ctrl._targets(m, d, 'front')
    d.qpos[m.joint('pedal_front_spin').qposadr[0]] = np.pi
    mujoco.mj_forward(m, d)
    b = ctrl._targets(m, d, 'front')
    np.testing.assert_allclose(a, b, atol=1e-10)


@pytest.mark.parametrize('angle', (0., .15, np.pi, np.pi+.15))
def test_finite_contact_force_is_the_gradient_of_its_stored_energy(pedal_rig, angle):
    m, d, contact = pedal_rig
    contact.config = replace(contact.config, support_c_ns_m=0., support_mu=0.)
    d.qpos[m.joint('pedal_front_spin').qposadr[0]] = angle
    d.qpos[m.joint('rider_root_z').qposadr[0]] = -.005
    mujoco.mj_forward(m, d)
    q0 = d.qpos.copy()
    direction = np.zeros(m.nv)
    for name, value in (('rider_root_x', .2), ('rider_root_z', .7),
                        ('pedal_front_spin', -.3), ('rider_ankle_front', .15)):
        direction[m.joint(name).dofadr[0]] = value
    force = contact.compute_qfrc(m, d, m.opt.timestep, advance=False)
    energies = []
    eps = 1e-7
    for sign in (1., -1.):
        d.qpos[:] = q0+sign*eps*direction
        mujoco.mj_forward(m, d)
        energies.append(contact.stored_energy(m, d))
    d.qpos[:] = q0
    mujoco.mj_forward(m, d)
    assert float(force@direction) == pytest.approx(
        -(energies[0]-energies[1])/(2*eps), rel=2e-7, abs=1e-6)


def test_upper_face_target_does_not_jump_at_45_degrees():
    from bike_sim.sim.ride.support_geometry import upper_box_face
    values = []
    for theta in (np.pi/4-1e-6, np.pi/4+1e-6):
        R = np.array([[np.cos(theta), 0., np.sin(theta)], [0., 1., 0.],
                      [-np.sin(theta), 0., np.cos(theta)]])
        point,normal,_ = upper_box_face(np.zeros(3), R, [.05, .04, .008])
        values.append(point+.017*normal)
    assert np.linalg.norm(values[1]-values[0]) < 1e-5


@pytest.mark.parametrize('angle', np.radians([0., 30., 60., 75., 84.4, 90., 120., 180.]))
def test_foot_ik_accounts_for_both_finite_pressure_pads(pedal_rig, angle):
    from bike_sim.sim.ride.rider_control import ArticulatedRiderController
    model, data, contacts = pedal_rig
    controller = ArticulatedRiderController(model, contacts.pose, contacts.config, .165)
    data.qpos[model.joint('pedal_front_spin').qposadr[0]] = angle
    mujoco.mj_forward(model, data)
    desired = controller._targets(model, data, 'front', compression_m=.003)
    for joint, value in zip(('hip', 'knee', 'ankle'), desired):
        data.qpos[controller.joints[f'rider_{joint}_front'][0]] = value
    mujoco.mj_forward(model, data)
    assert not controller.saturated_ik['front']
    pads = list(contacts._pads(model, data, 'front_pedal', contacts.supports['front_pedal']))
    compression = sum(max(0., -pad[4]) for pad in pads if pad[5])
    assert compression == pytest.approx(.006, abs=1e-10)
    for pad in pads:
        if pad[4] < 0.:
            assert pad[1][2] >= data.geom_xpos[controller.pedal_geoms['front'],2]-1e-10


def test_sole_target_at_horizontal_platform_is_the_declared_compression():
    from bike_sim.sim.ride.support_geometry import sole_target_height
    height = sole_target_height(np.zeros(3), np.eye(3), np.array([.05,.04,.008]),
                                0., .025, .02, .003)
    assert height == pytest.approx(.005, abs=1e-12)


def test_preview_and_accounted_contacts_expose_the_same_underside_recovery_signal(pedal_rig):
    model, data, contacts = pedal_rig
    data.qpos[model.joint('rider_root_z').qposadr[0]] = -.04
    mujoco.mj_forward(model, data)
    full = contacts.compute_qfrc(model, data, model.opt.timestep, advance=False, detailed=True)
    detailed = contacts.probe_diagnostics['front_pedal']
    preview = contacts.compute_qfrc(model, data, model.opt.timestep, advance=False, detailed=False)
    compact = contacts.probe_diagnostics['front_pedal']
    assert compact['vertical_force_on_rider_n'] < -1.
    assert compact['vertical_force_on_rider_n'] == pytest.approx(detailed['force_on_rider_n'][2])
    np.testing.assert_allclose(preview, full, atol=1e-12)


def test_reachable_sole_goal_projection_preserves_requested_compression():
    from bike_sim.sim.ride.support_geometry import project_sole_goal
    goal, diagnostic = project_sole_goal(np.zeros(3), np.eye(3), np.array([.05, .04, .008]),
        .025, .02, .003, .001)
    np.testing.assert_allclose(goal, [.001, 0., .005], atol=1e-12)
    assert not diagnostic['saturated']
    assert diagnostic['applied_compression_m'] == .003


def test_excessive_pressure_projects_the_goal_without_changing_contact_material():
    from bike_sim.sim.ride.support_geometry import project_sole_goal, box_pad_contact
    origin = np.zeros(3)
    rotation = np.eye(3)
    half = np.array([.05, .04, .008])
    goal, diagnostic = project_sole_goal(origin, rotation, half, .025, .02, .2, .08)
    assert diagnostic['saturated']
    assert 0. <= diagnostic['applied_compression_m'] < .2
    assert abs(diagnostic['applied_shear_m']) <= .08
    assert np.isfinite(goal).all()
    compression = 0.
    for offset in (-.025, .025):
        pad = box_pad_contact(goal + np.array([offset, 0., .02]), .02, origin, rotation, half)
        if pad.within_footprint and pad.gap_m < 0.:
            compression -= pad.gap_m
            assert pad.point_m[2] >= origin[2] - 1e-10
    assert compression == pytest.approx(2.*diagnostic['applied_compression_m'], abs=1e-9)
