"""Parity between the native spindle kernels/controller and the Python oracle.

The pure kernels are bound as module functions; the stateful
SpindleController port is exercised through NativeTestAdapter's
``spindle_controller`` section on the pinned welded plant (the same model
and config the Python oracle runs). Every comparison is against the live
Python controller on an identically staged mjData — no recorded dumps.
"""
import math
from dataclasses import asdict

import mujoco
import numpy as np
import pytest

from native_loader import load_native

from bike_sim.physics.rider_posture import RiderPosture
from bike_sim.physics.seated_climb import crank_effort_ceiling
from bike_sim.sim.ride.physical_samples import plain
from bike_sim.sim.ride.rider_control import (
    ArticulatedRiderController, RiderCommand, pedal_torque_waveform)


@pytest.mark.parametrize("phase", [0.0, math.pi / 2, math.pi, 2 * math.pi - 1e-7])
def test_spindle_waveform_matches_python(phase):
    native = load_native()
    actual = native.spindle_torque_waveform(60.0, phase, 0.5)
    assert actual == pytest.approx(
        pedal_torque_waveform(60.0, phase, 0.5), rel=1e-12, abs=1e-12)


@pytest.mark.parametrize("rate", [-3.0, 0.0, 8.0, 20.0])
def test_effort_ceiling_matches_python(rate):
    assert load_native().crank_effort_ceiling(250.0, 60.0, rate) == pytest.approx(
        crank_effort_ceiling(250.0, 60.0, rate), rel=1e-12, abs=1e-12)


def test_spindle_waveform_rejects_bad_inputs():
    native = load_native()
    with pytest.raises(Exception):
        native.spindle_torque_waveform(-1.0, 0.0, 0.5)
    with pytest.raises(Exception):
        native.spindle_torque_waveform(60.0, 0.0, 1.0)
    with pytest.raises(Exception):
        native.spindle_torque_waveform(60.0, float("nan"), 0.5)


def test_effort_ceiling_rejects_bad_inputs():
    native = load_native()
    with pytest.raises(Exception):
        native.crank_effort_ceiling(-1.0, 60.0, 8.0)
    with pytest.raises(Exception):
        native.crank_effort_ceiling(250.0, -60.0, 8.0)
    with pytest.raises(Exception):
        native.crank_effort_ceiling(250.0, 60.0, float("nan"))


# ---- A2.3: SpindleController through the native-only adapter ---------------

def _damped_config(tmp_path):
    """The pinned TOML plus a nonzero activation time constant and joint
    passive damping — the pinned profile leaves both at zero, which makes
    the activation filter and passive-damping terms vacuous."""
    from test_pinned_topology import _pinned_config
    text = _pinned_config(tmp_path).read_text()
    assert 'joint_passive_damping_nms_rad = 0.0' in text
    text = text.replace(
        'joint_passive_damping_nms_rad = 0.0',
        'joint_passive_damping_nms_rad = 2.5\nactivation_tau_s = 0.05')
    path = tmp_path / 'physics_damped.toml'
    path.write_text(text)
    return path


def _plant(tmp_path, config_path=None):
    """The pinned nine-actuator plant: Python controller + native adapter
    over the identical compiled model and wire config."""
    from test_pinned_topology import _pinned_config
    from bike_sim.geometry.specs import BikeSpecs
    from bike_sim.mujoco.builder import generate_mujoco_xml
    from bike_sim.native.setup import rider_controller_config
    from bike_sim.physics.resolution import load_physics_config
    from bike_sim.physics.rider import RiderSpecs
    from bike_sim.physics.rider_segments import geometry_pose
    cfg = load_physics_config(
        _pinned_config(tmp_path) if config_path is None else config_path)
    specs = BikeSpecs()
    rider = RiderSpecs(variant='articulated_planar')
    pose = geometry_pose(rider, specs)
    model = mujoco.MjModel.from_xml_string(generate_mujoco_xml(
        specs, mode='ride', rider=rider, crank_joint=True,
        physics_config=cfg))
    crank_m = specs.crank_length / 1000.
    controller = ArticulatedRiderController(
        model, pose, cfg.articulated, crank_m)
    path = tmp_path / 'plant.mjb'
    mujoco.mj_saveModel(model, str(path))
    probe = load_native().NativeTestAdapter(str(path), {
        'spindle_controller': {
            'config': rider_controller_config(cfg.articulated, controller),
            'pose': plain(asdict(pose)),
            'crank_length_m': crank_m}})
    return model, controller, probe


def _command(posture=None, **kwargs):
    return RiderCommand(**kwargs) if posture is None else \
        RiderCommand(posture=posture, **kwargs)


def _command_dict(command):
    posture = command.posture
    return {'mean_crank_torque_nm': command.mean_crank_torque_nm,
            'enabled': command.enabled,
            'posture': {'torso_lean_rad': posture.torso_lean_rad,
                        'pelvis_pitch_rad': posture.pelvis_pitch_rad,
                        'pelvis_offset_m': posture.pelvis_offset_m,
                        'use_saddle': posture.use_saddle},
            'crank_target_phase_rad': command.crank_target_phase_rad,
            'crank_target_rate_rad_s': command.crank_target_rate_rad_s}


def _staged(model, probe, crank_q=0.0, crank_rate=0.0, root_pitch=0.0,
            joint_rates=None):
    """Identical mjData on both sides: qpos/qvel staged, then forward."""
    data = mujoco.MjData(model)
    crank = model.joint('crank_spin')
    pitch = model.joint('root_pitch')
    data.qpos[crank.qposadr[0]] = crank_q
    data.qpos[pitch.qposadr[0]] = root_pitch
    data.qvel[crank.dofadr[0]] = crank_rate
    for name, rate in (joint_rates or {}).items():
        data.qvel[model.joint(name).dofadr[0]] = rate
    mujoco.mj_forward(model, data)
    probe.set_state(np.asarray(data.qpos), np.asarray(data.qvel),
                    np.asarray(data.act), np.asarray(data.qacc_warmstart),
                    float(data.time))
    probe.forward()
    return data


def _compare_terms(native_state, controller):
    for name, term in controller.last_terms.items():
        native_term = native_state['last_terms'][name]
        for key, value in term.items():
            if isinstance(value, bool):
                assert native_term[key] == value
            else:
                assert native_term[key] == pytest.approx(
                    value, rel=1e-12, abs=1e-12), name + '.' + key


def _compare_diagnostics(native_state, controller):
    allocation = native_state['allocation_diagnostics']
    expected = controller.allocation_diagnostics
    assert allocation['invalid_controller'] == expected['invalid_controller']
    assert allocation['crank_task_nm'] == pytest.approx(
        expected['crank_task_nm'], rel=1e-12, abs=1e-12)
    assert allocation['crank_task_shortfall_nm'] == pytest.approx(
        expected['crank_task_shortfall_nm'], rel=1e-12, abs=1e-12)
    np.testing.assert_allclose(
        allocation['solution_excitation_nm'],
        expected['solution_excitation_nm'], rtol=1e-12, atol=1e-12)
    assert (native_state['support_diagnostics']['stance'] ==
            controller.support_diagnostics['stance'])
    effort = native_state['effort_diagnostics']
    expected_effort = controller.effort_diagnostics
    for key in ('rider_active_request_nm', 'rider_active_delivered_nm'):
        for name, value in expected_effort[key].items():
            assert effort[key][name] == pytest.approx(
                value, rel=1e-12, abs=1e-12), name + '.' + key
    for key in ('rider_positive_power_w', 'rider_passive_power_w'):
        assert effort[key] == pytest.approx(
            expected_effort[key], rel=1e-12, abs=1e-12)
    assert (effort['rider_activation_saturated'] ==
            expected_effort['rider_activation_saturated'])
    assert (tuple(effort['rider_strength_limited']) ==
            tuple(expected_effort['rider_strength_limited']))
    assert (effort['rider_effort_budget_exceeded'] ==
            expected_effort['rider_effort_budget_exceeded'])
    assert (effort['rider_active_positive_power_limit_w'] ==
            expected_effort['rider_active_positive_power_limit_w'])
    assert (effort['rider_effort_observation'] ==
            expected_effort['rider_effort_observation'])
    np.testing.assert_allclose(native_state['active_state'],
                               controller.active_state, rtol=1e-12,
                               atol=1e-12)
    assert (native_state['activation_time_s'] ==
            controller.activation_time_s)


@pytest.mark.parametrize("crank_q", [0.0, 0.7, math.pi / 2, 2.4])
@pytest.mark.parametrize("effort", [0.0, 40.0, 120.0])
def test_spindle_compute_matches_python(tmp_path, crank_q, effort):
    model, controller, probe = _plant(tmp_path)
    data = _staged(model, probe, crank_q=crank_q, crank_rate=6.0)
    command = _command(mean_crank_torque_nm=effort)
    expected = controller.compute(model, data, command, dt_s=.00125)
    actual = probe.spindle_compute(_command_dict(command), True, .00125, False)
    for name, value in expected.items():
        assert actual['torques'][name] == pytest.approx(
            value, rel=1e-12, abs=1e-12), name
    _compare_terms(actual['state'], controller)
    _compare_diagnostics(actual['state'], controller)


def test_spindle_disabled_command_matches_python(tmp_path):
    model, controller, probe = _plant(tmp_path)
    data = _staged(model, probe, crank_q=.3, crank_rate=2.)
    command = _command(enabled=False, mean_crank_torque_nm=80.)
    expected = controller.compute(model, data, command, dt_s=.00125)
    actual = probe.spindle_compute(_command_dict(command), True, .00125, False)
    assert not any(expected.values())
    assert not any(actual['torques'].values())
    assert (actual['state']['support_diagnostics']['stance'] ==
            {'front': False, 'rear': False})
    assert actual['state']['command_enabled'] is False


def test_spindle_steady_state_bypasses_activation(tmp_path):
    model, controller, probe = _plant(tmp_path)
    data = _staged(model, probe, crank_q=1.1, crank_rate=7.)
    command = _command(mean_crank_torque_nm=90.)
    expected = controller.compute(model, data, command, dt_s=.00125,
                                  steady_state=True)
    actual = probe.spindle_compute(_command_dict(command), True, .00125, True)
    for name, value in expected.items():
        assert actual['torques'][name] == pytest.approx(
            value, rel=1e-12, abs=1e-12), name
    _compare_diagnostics(actual['state'], controller)


def test_spindle_activation_rejects_repeated_timestamp(tmp_path):
    # The damped fixture carries activation_tau_s > 0 so the guard is real.
    model, controller, probe = _plant(tmp_path, _damped_config(tmp_path))
    assert controller.config.activation_tau_s > 0.
    data = _staged(model, probe, crank_q=.4, crank_rate=5.)
    command = _command(mean_crank_torque_nm=60.)
    controller.compute(model, data, command, dt_s=.00125)
    probe.spindle_compute(_command_dict(command), True, .00125, False)
    with pytest.raises(Exception):
        controller.compute(model, data, command, dt_s=.00125)
    with pytest.raises(Exception):
        probe.spindle_compute(_command_dict(command), True, .00125, False)


def test_spindle_activation_and_passive_damping_match_python(tmp_path):
    """Nonzero activation_tau_s + joint_passive_damping_nms_rad: the
    activation filter state, activation_time_s bookkeeping and the
    -damping*speed passive terms all evolve against the oracle."""
    model, controller, probe = _plant(tmp_path, _damped_config(tmp_path))
    assert controller.config.activation_tau_s > 0.
    assert controller.config.passive_damping_nms_rad > 0.
    data = _staged(model, probe, crank_q=.4, crank_rate=5.,
                   joint_rates={'rider_hip_front': 1.5,
                                'rider_knee_rear': -2.})
    for time_s, effort in ((.00125, 60.), (.00250, 80.), (.00375, 40.)):
        data.time = time_s
        mujoco.mj_forward(model, data)
        command = _command(mean_crank_torque_nm=effort)
        expected = controller.compute(model, data, command, dt_s=.00125)
        probe.set_state(np.asarray(data.qpos), np.asarray(data.qvel),
                        np.asarray(data.act),
                        np.asarray(data.qacc_warmstart), float(data.time))
        probe.forward()
        actual = probe.spindle_compute(_command_dict(command), True,
                                       .00125, False)
        for name, value in expected.items():
            assert actual['torques'][name] == pytest.approx(
                value, rel=1e-12, abs=1e-12), f'{time_s}:{name}'
        _compare_terms(actual['state'], controller)
        _compare_diagnostics(actual['state'], controller)
        assert actual['state']['activation_time_s'] == \
            pytest.approx(time_s, rel=0., abs=1e-15)
        assert any(
            term['passive_damping_nm'] != 0.
            for term in controller.last_terms.values())


def test_spindle_initialize_matches_python(tmp_path):
    model, controller, probe = _plant(tmp_path)
    data = mujoco.MjData(model)
    crank = model.joint('crank_spin')
    data.qpos[crank.qposadr[0]] = 1.3
    mujoco.mj_forward(model, data)
    incoming_qpos = np.asarray(data.qpos).copy()
    incoming_qvel = np.asarray(data.qvel).copy()
    controller.initialize(model, data)
    probe.set_state(incoming_qpos, incoming_qvel, np.asarray(data.act),
                    np.asarray(data.qacc_warmstart), float(data.time))
    probe.spindle_initialize()
    np.testing.assert_allclose(probe.qpos, np.asarray(data.qpos),
                               rtol=1e-12, atol=1e-12)


def test_spindle_initialize_velocity_matches_python(tmp_path):
    model, controller, probe = _plant(tmp_path)
    data = mujoco.MjData(model)
    crank = model.joint('crank_spin')
    data.qpos[crank.qposadr[0]] = .8
    data.qvel[crank.dofadr[0]] = 9.0
    mujoco.mj_forward(model, data)
    incoming_qpos = np.asarray(data.qpos).copy()
    incoming_qvel = np.asarray(data.qvel).copy()
    controller.initialize_velocity(model, data)
    probe.set_state(incoming_qpos, incoming_qvel, np.asarray(data.act),
                    np.asarray(data.qacc_warmstart), float(data.time))
    probe.forward()
    probe.spindle_initialize_velocity()
    np.testing.assert_allclose(probe.qvel, np.asarray(data.qvel),
                               rtol=1e-10, atol=1e-10)


def test_spindle_envelope_forces_match_python(tmp_path):
    model, controller, probe = _plant(tmp_path)
    data = mujoco.MjData(model)
    controller.initialize(model, data)
    probe.set_state(np.asarray(data.qpos), np.asarray(data.qvel),
                    np.asarray(data.act), np.asarray(data.qacc_warmstart),
                    float(data.time))
    probe.forward()
    force, stored = controller.envelope_forces(model, data)
    actual = probe.spindle_envelope_forces()
    np.testing.assert_allclose(actual['force'], force, rtol=1e-12,
                               atol=1e-12)
    assert actual['stored_j'] == pytest.approx(stored, rel=1e-12, abs=1e-12)


def test_spindle_write_matches_python(tmp_path):
    model, controller, probe = _plant(tmp_path)
    data = _staged(model, probe, crank_q=.5)
    command = _command(mean_crank_torque_nm=70.)
    torques = controller.compute(model, data, command, dt_s=.00125)
    controller.write(data, torques)
    probe.spindle_compute(_command_dict(command), True, .00125, False)
    probe.spindle_write(dict(torques))
    np.testing.assert_allclose(probe.ctrl, np.asarray(data.ctrl),
                               rtol=1e-12, atol=1e-12)
    assert np.count_nonzero(probe.ctrl) == len(torques)


def test_spindle_state_round_trip(tmp_path):
    model, controller, probe = _plant(tmp_path)
    data = _staged(model, probe, crank_q=.9, crank_rate=4.)
    command = _command(mean_crank_torque_nm=55.)
    controller.compute(model, data, command, dt_s=.00125)
    probe.spindle_compute(_command_dict(command), True, .00125, False)
    state = controller.state_dict()
    probe.spindle_restore(state)
    exported = probe.spindle_state()
    np.testing.assert_allclose(exported['active_state'],
                               state['active_state'], rtol=0., atol=0.)
    assert exported['enabled'] == state['enabled']
    assert exported['command_enabled'] == state['command_enabled']
    assert exported['saturated_ik'] == state['saturated_ik']
    assert exported['lean_limit_rad'] == pytest.approx(
        state['lean_limit_rad'], rel=1e-12, abs=1e-12)
    for name, terms in state['last_terms'].items():
        for key, value in terms.items():
            if isinstance(value, bool):
                assert exported['last_terms'][name][key] == value
            else:
                assert exported['last_terms'][name][key] == pytest.approx(
                    value, rel=1e-12, abs=1e-12), name + '.' + key
    for side, recovery in state['pedal_recovery'].items():
        for key, value in recovery.items():
            assert exported['pedal_recovery'][side][key] == value
    # A restored controller produces the identical next decision. Time
    # must advance past activation_time_s or both sides raise the
    # repeated-timestamp guard.
    data2 = mujoco.MjData(model)
    data2.time = .00125
    data2.qpos[model.joint('crank_spin').qposadr[0]] = 1.05
    data2.qvel[model.joint('crank_spin').dofadr[0]] = 5.
    mujoco.mj_forward(model, data2)
    probe.set_state(np.asarray(data2.qpos), np.asarray(data2.qvel),
                    np.asarray(data2.act), np.asarray(data2.qacc_warmstart),
                    float(data2.time))
    probe.forward()
    expected = controller.compute(model, data2, command, dt_s=.00125)
    actual = probe.spindle_compute(_command_dict(command), True, .00125, False)
    for name, value in expected.items():
        assert actual['torques'][name] == pytest.approx(
            value, rel=1e-12, abs=1e-12), name


def test_spindle_solved_effort_matches_python(tmp_path):
    from bike_sim.sim.ride.rider_effort import solved_effort
    model, controller, probe = _plant(tmp_path)
    data = _staged(model, probe, crank_q=.6, crank_rate=8.)
    command = _command(mean_crank_torque_nm=75.)
    torques = controller.compute(model, data, command, dt_s=.00125)
    probe.spindle_compute(_command_dict(command), True, .00125, False)
    controller.write(data, torques)
    incoming_qpos = np.asarray(data.qpos).copy()
    incoming_qvel = np.asarray(data.qvel).copy()
    mujoco.mj_forward(model, data)
    expected = solved_effort(controller, data,
                             (incoming_qpos, incoming_qvel), .00125)
    probe.set_inputs(np.asarray(data.ctrl), np.zeros(model.nv))
    probe.forward()
    actual = probe.spindle_solved_effort(incoming_qpos, incoming_qvel,
                                         .00125)
    assert set(actual.keys()) == set(expected.keys())
    for key, value in expected.items():
        if isinstance(value, dict):
            for name, entry in value.items():
                assert actual[key][name] == pytest.approx(
                    entry, rel=1e-12, abs=1e-12), key + '.' + name
        elif isinstance(value, tuple):
            assert tuple(actual[key]) == value, key
        elif isinstance(value, bool) or value is None or isinstance(value, str):
            assert actual[key] == value, key
        else:
            assert actual[key] == pytest.approx(value, rel=1e-12,
                                                abs=1e-12), key


@pytest.mark.parametrize("posture", [
    RiderPosture(),
    RiderPosture(torso_lean_rad=.3, pelvis_pitch_rad=.2),
    RiderPosture(torso_lean_rad=-.2, pelvis_offset_m=(-.05, .1)),
    RiderPosture(pelvis_pitch_rad=-.4, use_saddle=False)])
def test_spindle_compute_posture_matches_python(tmp_path, posture):
    model, controller, probe = _plant(tmp_path)
    data = _staged(model, probe, crank_q=1.9, crank_rate=7.)
    command = _command(posture=posture, mean_crank_torque_nm=70.)
    expected = controller.compute(model, data, command, dt_s=.00125)
    actual = probe.spindle_compute(_command_dict(command), True, .00125, False)
    for name, value in expected.items():
        assert actual['torques'][name] == pytest.approx(
            value, rel=1e-12, abs=1e-12), name
    _compare_diagnostics(actual['state'], controller)


def test_spindle_coasting_command_matches_python(tmp_path):
    model, controller, probe = _plant(tmp_path)
    data = _staged(model, probe, crank_q=.35, crank_rate=4.)
    command = _command(mean_crank_torque_nm=0.,
                       crank_target_phase_rad=1.2,
                       crank_target_rate_rad_s=6.)
    expected = controller.compute(model, data, command, dt_s=.00125)
    actual = probe.spindle_compute(_command_dict(command), True, .00125, False)
    for name, value in expected.items():
        assert actual['torques'][name] == pytest.approx(
            value, rel=1e-12, abs=1e-12), name
    _compare_diagnostics(actual['state'], controller)


def test_spindle_reset_activation_matches_python(tmp_path):
    model, controller, probe = _plant(tmp_path)
    data = _staged(model, probe, crank_q=.2, crank_rate=3.)
    command = _command(mean_crank_torque_nm=50.)
    controller.compute(model, data, command, dt_s=.00125)
    probe.spindle_compute(_command_dict(command), True, .00125, False)
    controller.reset_activation()
    probe.spindle_reset_activation()
    actual = probe.spindle_state()
    np.testing.assert_allclose(actual['active_state'],
                               np.zeros(len(controller.joints)),
                               rtol=0., atol=0.)
    assert actual['activation_time_s'] is None
    assert actual['effort_diagnostics'] == {}
    assert actual['sole_goal_diagnostics'] == {}


def test_spindle_strength_probes_match_python(tmp_path):
    model, controller, probe = _plant(tmp_path)
    data = _staged(model, probe, crank_q=.4)
    qpos = np.asarray(data.qpos)
    qvel = np.asarray(data.qvel)
    for name, (qa, dof, _) in controller.joints.items():
        if controller.strength is None:
            continue
        angle = controller.anatomical_joint_angle(name, qpos[qa])
        assert probe.spindle_anatomical(name, float(qpos[qa])) == \
            pytest.approx(angle, rel=1e-12, abs=1e-12), name
        for velocity, torque in ((0., 30.), (2.5, -20.), (-1.5, 45.)):
            expected = controller.strength_capacity(
                name, angle, velocity, torque)
            actual = probe.spindle_capacity(
                name, angle, velocity, torque)
            if math.isinf(expected):
                assert math.isinf(actual)
            else:
                assert actual == pytest.approx(
                    expected, rel=1e-12, abs=1e-12), name
    torques = np.array(
        [120. if 'knee' in n else -90. for n in controller.joints])
    clipped, limited = controller.strength_limited(torques, qpos, qvel)
    actual = probe.spindle_strength_limited(torques, qpos, qvel)
    np.testing.assert_allclose(actual['torques'], clipped,
                               rtol=1e-12, atol=1e-12)
    assert tuple(actual['limited']) == limited


def test_spindle_strength_rejects_malformed_state(tmp_path):
    """Caller-supplied qpos/qvel narrower than nq/nv must reject like the
    oracle's numpy indexing — never an out-of-bounds span read."""
    from bike_sim.sim.ride.rider_effort import solved_effort
    model, controller, probe = _plant(tmp_path)
    data = _staged(model, probe, crank_q=.4)
    command = _command(mean_crank_torque_nm=70.)
    torques = controller.compute(model, data, command, dt_s=.00125)
    controller.write(data, torques)
    mujoco.mj_forward(model, data)
    probe.spindle_compute(_command_dict(command), True, .00125, False)
    probe.set_inputs(np.asarray(data.ctrl), np.zeros(model.nv))
    probe.forward()
    qpos = np.asarray(data.qpos)
    qvel = np.asarray(data.qvel)
    flat = np.zeros(len(controller.joints))
    with pytest.raises(Exception):
        controller.strength_limited(flat, qpos[:2], qvel)
    with pytest.raises(Exception):
        probe.spindle_strength_limited(flat, qpos[:2], qvel)
    with pytest.raises(Exception):
        controller.strength_limited(flat, qpos, qvel[:2])
    with pytest.raises(Exception):
        probe.spindle_strength_limited(flat, qpos, qvel[:2])
    with pytest.raises(Exception):
        solved_effort(controller, data, (qpos[:2], qvel), .00125)
    with pytest.raises(Exception):
        probe.spindle_solved_effort(qpos[:2], qvel, .00125)
    with pytest.raises(Exception):
        solved_effort(controller, data, (qpos, qvel[:2]), .00125)
    with pytest.raises(Exception):
        probe.spindle_solved_effort(qpos, qvel[:2], .00125)


def test_spindle_lookups_reject_unknown_joints(tmp_path):
    """A name outside the configured rider joints must reject like the
    oracle's self.joints[name]/table lookups — never an unchecked index
    into the joint arrays. The locked ankle references are absent from
    self.joints on this nine-actuator plant, and 'rider_wrist' is absent
    from every table."""
    model, controller, probe = _plant(tmp_path)
    data = _staged(model, probe, crank_q=.4)
    for name in ('rider_ankle_front', 'rider_wrist'):
        with pytest.raises(Exception):
            controller.write(data, {name: 0.})
        with pytest.raises(Exception):
            probe.spindle_write({name: 0.})
    with pytest.raises(Exception):
        controller.anatomical_joint_angle('rider_wrist', 0.)
    with pytest.raises(Exception):
        probe.spindle_anatomical('rider_wrist', 0.)
    with pytest.raises(Exception):
        controller.strength_capacity('rider_wrist', 0., 0., 5.)
    with pytest.raises(Exception):
        probe.spindle_capacity('rider_wrist', 0., 0., 5.)


def test_spindle_probe_rejects_bad_input(tmp_path):
    from bike_sim.geometry.specs import BikeSpecs
    from bike_sim.native.setup import rider_controller_config
    from bike_sim.physics.rider import RiderSpecs
    from bike_sim.physics.rider_segments import geometry_pose
    model, controller, probe = _plant(tmp_path)
    pose = plain(asdict(geometry_pose(
        RiderSpecs(variant='articulated_planar'), BikeSpecs())))
    with pytest.raises(Exception):
        probe.spindle_compute({}, True, None, False)
    with pytest.raises(Exception):
        probe.spindle_compute(
            _command_dict(_command(mean_crank_torque_nm=-1.)),
            True, None, False)
    with pytest.raises(Exception):
        probe.spindle_restore({'enabled': True})
    # An unsupported attachment mode is rejected at decode, not routed.
    config = rider_controller_config(controller.config, controller)
    config['pedal_attachment'] = 'flat'
    path = tmp_path / 'flat.mjb'
    mujoco.mj_saveModel(model, str(path))
    with pytest.raises(Exception):
        load_native().NativeTestAdapter(str(path), {
            'spindle_controller': {
                'config': config,
                'pose': pose,
                'crank_length_m': 0.17}})
    # A missing pose/config is rejected at adapter construction.
    with pytest.raises(Exception):
        load_native().NativeTestAdapter(str(path), {
            'spindle_controller': {'config': config}})

