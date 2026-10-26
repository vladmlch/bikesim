"""Parity between the native spindle kernels/controller and the Python oracle.

The pure kernels are bound as module functions; the stateful
SpindleController port is exercised through NativeTestAdapter's
``spindle_controller`` section on the pinned welded plant (the same model
and config the Python oracle runs). Every comparison is against the live
Python controller on an identically staged mjData — no recorded dumps.
"""
import json
import math
from dataclasses import asdict

import _golden
import mujoco
import numpy as np
import pytest

from native_loader import load_native

from bike_sim.physics.rider_posture import RiderPosture
from bike_sim.physics.seated_climb import crank_effort_ceiling
from bike_sim.sim.ride.physical_samples import plain
from bike_sim.sim.ride.rider_control import (
    ArticulatedRiderController, RiderCommand, pedal_torque_waveform)


# ---------------------------------------------------------------------------
# ADR-0003 golden capture — same contract as test_native_rider_intent.py.
# `input.<op>.k` entries carry the canonical-JSON argument document (UTF-8
# bytes packed one-per-f64-lane for bt::oracle::Node::f64_input), `output.*`
# entries are oracle-side results compared byte-for-byte (|u1 canonical JSON)
# or under the source assertion's own tolerance (close f64 arrays/scalars),
# and `error.<op>.k` empty |u1 markers freeze "the wire call raised"
# outcomes. Non-finite floats use the recording codec's tagged-float64
# spelling; numpy arrays/bools canonicalize through _canon.
_SEQ: dict = {}


@pytest.fixture(autouse=True)
def _golden_seq():
    _SEQ.clear()
    yield
    _SEQ.clear()


def _next(tag):
    index = _SEQ.get(tag, 0)
    _SEQ[tag] = index + 1
    return f'{tag}.{index}'


def _normalize(value):
    if isinstance(value, dict):
        return {key: _normalize(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_normalize(item) for item in value]
    if isinstance(value, np.ndarray):
        return _normalize(value.tolist())
    if isinstance(value, np.bool_):
        return bool(value)
    return value


def _canon(value):
    if isinstance(value, dict):
        return {key: _canon(item) for key, item in value.items()}
    if isinstance(value, np.ndarray):
        return _canon(value.tolist())
    if isinstance(value, (list, tuple)):
        return [_canon(item) for item in value]
    if isinstance(value, np.bool_):
        return bool(value)
    if isinstance(value, np.integer):
        return {'npint': int(value)}
    if isinstance(value, np.floating):
        value = float(value)
    if isinstance(value, float) and not math.isfinite(value):
        spelling = 'nan' if math.isnan(value) else (
            '+inf' if value > 0. else '-inf')
        return {'$bike_recording': 'float64', 'value': spelling}
    return value


def _golden_json(value):
    return json.dumps(_canon(value), sort_keys=True, separators=(',', ':'),
                      allow_nan=False).encode('utf-8')


def _rec_in(tag, value):
    _golden.record(
        np.frombuffer(_golden_json(value), np.uint8).astype('<f8'),
        _next(f'input.{tag}'))


def _rec_out(tag, value):
    _golden.record(np.frombuffer(_golden_json(value), np.uint8),
                   _next(f'output.{tag}'))


def _rec_scalar(tag, value):
    _golden.record(np.float64(value), _next(f'output.{tag}'))


def _rec_close(tag, value, rtol, atol):
    """A source-tolerance float comparison — np.testing/pytest.approx."""
    _golden.record(np.asarray(value, dtype='<f8'), _next(f'output.{tag}'),
                   kind='close', rtol=rtol, atol=atol)


def _rec_err(tag):
    _golden.record(np.zeros(0, np.uint8), _next(f'error.{tag}'))


def _adapter(path, section):
    """Recorded NativeTestAdapter construction (spindle_controller section)."""
    _rec_in('adapter', section['spindle_controller'])
    try:
        return load_native().NativeTestAdapter(str(path), section)
    except Exception:
        _rec_err('adapter')
        raise


@pytest.mark.parametrize("phase", [0.0, math.pi / 2, math.pi, 2 * math.pi - 1e-7])
def test_spindle_waveform_matches_python(phase):
    native = load_native()
    _rec_in('waveform', {'mean_nm': 60., 'phase_rad': phase, 'ripple': .5})
    actual = native.spindle_torque_waveform(60.0, phase, 0.5)
    expected = pedal_torque_waveform(60.0, phase, 0.5)
    _rec_close('waveform', expected, 1e-12, 1e-12)
    assert actual == pytest.approx(expected, rel=1e-12, abs=1e-12)


@pytest.mark.parametrize("rate", [-3.0, 0.0, 8.0, 20.0])
def test_effort_ceiling_matches_python(rate):
    _rec_in('ceiling', {'power_w': 250., 'torque_limit_nm': 60.,
                        'crank_rate_rad_s': rate})
    actual = load_native().crank_effort_ceiling(250.0, 60.0, rate)
    expected = crank_effort_ceiling(250.0, 60.0, rate)
    _rec_close('ceiling', expected, 1e-12, 1e-12)
    assert actual == pytest.approx(expected, rel=1e-12, abs=1e-12)


def test_spindle_waveform_rejects_bad_inputs():
    native = load_native()
    for args in ((-1., 0., .5), (60., 0., 1.), (60., float('nan'), .5)):
        _rec_in('waveform', {'mean_nm': args[0], 'phase_rad': args[1],
                             'ripple': args[2]})
        with pytest.raises(Exception):
            native.spindle_torque_waveform(*args)
        _rec_err('waveform')


def test_effort_ceiling_rejects_bad_inputs():
    native = load_native()
    for args in ((-1., 60., 8.), (250., -60., 8.), (250., 60., float('nan'))):
        _rec_in('ceiling', {'power_w': args[0], 'torque_limit_nm': args[1],
                            'crank_rate_rad_s': args[2]})
        with pytest.raises(Exception):
            native.crank_effort_ceiling(*args)
        _rec_err('ceiling')


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
    probe = _adapter(path, {'spindle_controller': {
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
    _rec_in('state', _state_doc(data))
    probe.set_state(np.asarray(data.qpos), np.asarray(data.qvel),
                    np.asarray(data.act), np.asarray(data.qacc_warmstart),
                    float(data.time))
    _rec_in('forward', {})
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


def _state_doc(data):
    """The recorded adapter set_state argument document."""
    return {'qpos': list(map(float, data.qpos)),
            'qvel': list(map(float, data.qvel)),
            'act': list(map(float, data.act)),
            'qacc_warmstart': list(map(float, data.qacc_warmstart)),
            'time': float(data.time)}


def _compute(probe, controller, model, data, command, *, dt_s=.00125,
             steady_state=False, record_outputs=True):
    """Recorded spindle_compute parity call. `record_outputs` mirrors which
    source tests assert the returned torques/state — calls whose result the
    oracle never compares record only the input document."""
    _rec_in('compute', {'command': _command_dict(command), 'advance': True,
                        'dt_s': dt_s, 'steady_state': steady_state})
    expected = controller.compute(model, data, command, dt_s=dt_s,
                                  steady_state=steady_state)
    actual = probe.spindle_compute(_command_dict(command), True, dt_s,
                                   steady_state)
    if record_outputs:
        _rec_out('torques', _normalize(expected))
        _rec_out('state', _normalize(controller.state_dict()))
    return expected, actual


@pytest.mark.parametrize("crank_q", [0.0, 0.7, math.pi / 2, 2.4])
@pytest.mark.parametrize("effort", [0.0, 40.0, 120.0])
def test_spindle_compute_matches_python(tmp_path, crank_q, effort):
    model, controller, probe = _plant(tmp_path)
    data = _staged(model, probe, crank_q=crank_q, crank_rate=6.0)
    command = _command(mean_crank_torque_nm=effort)
    expected, actual = _compute(probe, controller, model, data, command)
    for name, value in expected.items():
        assert actual['torques'][name] == pytest.approx(
            value, rel=1e-12, abs=1e-12), name
    _compare_terms(actual['state'], controller)
    _compare_diagnostics(actual['state'], controller)


def test_spindle_disabled_command_matches_python(tmp_path):
    model, controller, probe = _plant(tmp_path)
    data = _staged(model, probe, crank_q=.3, crank_rate=2.)
    command = _command(enabled=False, mean_crank_torque_nm=80.)
    expected, actual = _compute(probe, controller, model, data, command)
    assert not any(expected.values())
    assert not any(actual['torques'].values())
    assert (actual['state']['support_diagnostics']['stance'] ==
            {'front': False, 'rear': False})
    assert actual['state']['command_enabled'] is False


def test_spindle_steady_state_bypasses_activation(tmp_path):
    model, controller, probe = _plant(tmp_path)
    data = _staged(model, probe, crank_q=1.1, crank_rate=7.)
    command = _command(mean_crank_torque_nm=90.)
    expected, actual = _compute(probe, controller, model, data, command,
                                steady_state=True)
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
    _compute(probe, controller, model, data, command, record_outputs=False)
    _rec_in('compute', {'command': _command_dict(command), 'advance': True,
                        'dt_s': .00125, 'steady_state': False})
    with pytest.raises(Exception):
        controller.compute(model, data, command, dt_s=.00125)
    with pytest.raises(Exception):
        probe.spindle_compute(_command_dict(command), True, .00125, False)
    _rec_err('compute')


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
        _rec_in('state', _state_doc(data))
        probe.set_state(np.asarray(data.qpos), np.asarray(data.qvel),
                        np.asarray(data.act),
                        np.asarray(data.qacc_warmstart), float(data.time))
        _rec_in('forward', {})
        probe.forward()
        _rec_in('compute', {'command': _command_dict(command),
                            'advance': True, 'dt_s': .00125,
                            'steady_state': False})
        actual = probe.spindle_compute(_command_dict(command), True,
                                       .00125, False)
        _rec_out('torques', _normalize(expected))
        _rec_out('state', _normalize(controller.state_dict()))
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
    _rec_in('state', _state_doc(data))
    probe.set_state(incoming_qpos, incoming_qvel, np.asarray(data.act),
                    np.asarray(data.qacc_warmstart), float(data.time))
    _rec_in('initialize', {})
    probe.spindle_initialize()
    _rec_close('qpos', np.asarray(data.qpos), 1e-12, 1e-12)
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
    _rec_in('state', _state_doc(data))
    probe.set_state(incoming_qpos, incoming_qvel, np.asarray(data.act),
                    np.asarray(data.qacc_warmstart), float(data.time))
    _rec_in('forward', {})
    probe.forward()
    _rec_in('initialize_velocity', {})
    probe.spindle_initialize_velocity()
    _rec_close('qvel', np.asarray(data.qvel), 1e-10, 1e-10)
    np.testing.assert_allclose(probe.qvel, np.asarray(data.qvel),
                               rtol=1e-10, atol=1e-10)


def test_spindle_envelope_forces_match_python(tmp_path):
    model, controller, probe = _plant(tmp_path)
    data = mujoco.MjData(model)
    controller.initialize(model, data)
    _rec_in('state', _state_doc(data))
    probe.set_state(np.asarray(data.qpos), np.asarray(data.qvel),
                    np.asarray(data.act), np.asarray(data.qacc_warmstart),
                    float(data.time))
    _rec_in('forward', {})
    probe.forward()
    force, stored = controller.envelope_forces(model, data)
    _rec_in('envelope', {})
    actual = probe.spindle_envelope_forces()
    _rec_close('force', np.asarray(force), 1e-12, 1e-12)
    _rec_close('stored_j', np.float64(stored), 1e-12, 1e-12)
    np.testing.assert_allclose(actual['force'], force, rtol=1e-12,
                               atol=1e-12)
    assert actual['stored_j'] == pytest.approx(stored, rel=1e-12, abs=1e-12)


def test_spindle_write_matches_python(tmp_path):
    model, controller, probe = _plant(tmp_path)
    data = _staged(model, probe, crank_q=.5)
    command = _command(mean_crank_torque_nm=70.)
    torques = controller.compute(model, data, command, dt_s=.00125)
    controller.write(data, torques)
    _rec_in('compute', {'command': _command_dict(command), 'advance': True,
                        'dt_s': .00125, 'steady_state': False})
    probe.spindle_compute(_command_dict(command), True, .00125, False)
    _rec_in('write', {'torques': dict(torques)})
    probe.spindle_write(dict(torques))
    _rec_close('ctrl', np.asarray(data.ctrl), 1e-12, 1e-12)
    np.testing.assert_allclose(probe.ctrl, np.asarray(data.ctrl),
                               rtol=1e-12, atol=1e-12)
    assert np.count_nonzero(probe.ctrl) == len(torques)


def test_spindle_state_round_trip(tmp_path):
    model, controller, probe = _plant(tmp_path)
    data = _staged(model, probe, crank_q=.9, crank_rate=4.)
    command = _command(mean_crank_torque_nm=55.)
    _compute(probe, controller, model, data, command, record_outputs=False)
    state = controller.state_dict()
    _rec_in('restore', {'state': _normalize(state)})
    probe.spindle_restore(state)
    exported = probe.spindle_state()
    _rec_out('state', _normalize(state))
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
    _rec_in('state', _state_doc(data2))
    probe.set_state(np.asarray(data2.qpos), np.asarray(data2.qvel),
                    np.asarray(data2.act), np.asarray(data2.qacc_warmstart),
                    float(data2.time))
    _rec_in('forward', {})
    probe.forward()
    _rec_in('compute', {'command': _command_dict(command), 'advance': True,
                        'dt_s': .00125, 'steady_state': False})
    expected = controller.compute(model, data2, command, dt_s=.00125)
    actual = probe.spindle_compute(_command_dict(command), True, .00125, False)
    _rec_out('torques', _normalize(expected))
    for name, value in expected.items():
        assert actual['torques'][name] == pytest.approx(
            value, rel=1e-12, abs=1e-12), name


def test_spindle_solved_effort_matches_python(tmp_path):
    from bike_sim.sim.ride.rider_effort import solved_effort
    model, controller, probe = _plant(tmp_path)
    data = _staged(model, probe, crank_q=.6, crank_rate=8.)
    command = _command(mean_crank_torque_nm=75.)
    torques = controller.compute(model, data, command, dt_s=.00125)
    _rec_in('compute', {'command': _command_dict(command), 'advance': True,
                        'dt_s': .00125, 'steady_state': False})
    probe.spindle_compute(_command_dict(command), True, .00125, False)
    controller.write(data, torques)
    incoming_qpos = np.asarray(data.qpos).copy()
    incoming_qvel = np.asarray(data.qvel).copy()
    mujoco.mj_forward(model, data)
    expected = solved_effort(controller, data,
                             (incoming_qpos, incoming_qvel), .00125)
    _rec_in('inputs', {'ctrl': list(map(float, data.ctrl)),
                       'qfrc_applied': [0.] * model.nv})
    probe.set_inputs(np.asarray(data.ctrl), np.zeros(model.nv))
    _rec_in('forward', {})
    probe.forward()
    _rec_in('solved_effort', {'qpos': list(map(float, incoming_qpos)),
                              'qvel': list(map(float, incoming_qvel)),
                              'dt_s': .00125})
    actual = probe.spindle_solved_effort(incoming_qpos, incoming_qvel,
                                         .00125)
    _rec_out('effort', _normalize(expected))
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
    expected, actual = _compute(probe, controller, model, data, command)
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
    expected, actual = _compute(probe, controller, model, data, command)
    for name, value in expected.items():
        assert actual['torques'][name] == pytest.approx(
            value, rel=1e-12, abs=1e-12), name
    _compare_diagnostics(actual['state'], controller)


def test_spindle_reset_activation_matches_python(tmp_path):
    model, controller, probe = _plant(tmp_path)
    data = _staged(model, probe, crank_q=.2, crank_rate=3.)
    command = _command(mean_crank_torque_nm=50.)
    # The compute call's divergent torques/state are unasserted here —
    # only the reset's cleared fields are compared below.
    _compute(probe, controller, model, data, command, record_outputs=False)
    controller.reset_activation()
    _rec_in('reset_activation', {})
    probe.spindle_reset_activation()
    actual = probe.spindle_state()
    _rec_close('active_state', np.zeros(len(controller.joints)), 0., 0.)
    _rec_out('state_meta', _normalize(
        {'activation_time_s': controller.activation_time_s,
         'effort_diagnostics': dict(controller.effort_diagnostics),
         'sole_goal_diagnostics': dict(
             (side, goal) for side, goal
             in (controller.sole_goal_diagnostics or {}).items())}))
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
        _rec_in('anatomical', {'joint': name, 'qpos_value': float(qpos[qa])})
        actual = probe.spindle_anatomical(name, float(qpos[qa]))
        _rec_close('anatomical', angle, 1e-12, 1e-12)
        assert actual == \
            pytest.approx(angle, rel=1e-12, abs=1e-12), name
        for velocity, torque in ((0., 30.), (2.5, -20.), (-1.5, 45.)):
            expected = controller.strength_capacity(
                name, angle, velocity, torque)
            _rec_in('capacity', {'joint': name, 'angle_rad': angle,
                                 'velocity_rad_s': velocity,
                                 'torque_nm': torque})
            actual = probe.spindle_capacity(
                name, angle, velocity, torque)
            _rec_close('capacity', expected, 1e-12, 1e-12)
            if math.isinf(expected):
                assert math.isinf(actual)
            else:
                assert actual == pytest.approx(
                    expected, rel=1e-12, abs=1e-12), name
    torques = np.array(
        [120. if 'knee' in n else -90. for n in controller.joints])
    clipped, limited = controller.strength_limited(torques, qpos, qvel)
    _rec_in('strength_limited', {'torques': list(map(float, torques)),
                                 'qpos': list(map(float, qpos)),
                                 'qvel': list(map(float, qvel))})
    actual = probe.spindle_strength_limited(torques, qpos, qvel)
    _rec_close('clipped', np.asarray(clipped), 1e-12, 1e-12)
    _rec_out('limited', _normalize(list(limited)))
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
    _rec_in('compute', {'command': _command_dict(command), 'advance': True,
                        'dt_s': .00125, 'steady_state': False})
    probe.spindle_compute(_command_dict(command), True, .00125, False)
    _rec_in('inputs', {'ctrl': list(map(float, data.ctrl)),
                       'qfrc_applied': [0.] * model.nv})
    probe.set_inputs(np.asarray(data.ctrl), np.zeros(model.nv))
    _rec_in('forward', {})
    probe.forward()
    qpos = np.asarray(data.qpos)
    qvel = np.asarray(data.qvel)
    flat = np.zeros(len(controller.joints))
    _rec_in('strength_limited', {'torques': list(map(float, flat)),
                                 'qpos': list(map(float, qpos[:2])),
                                 'qvel': list(map(float, qvel))})
    with pytest.raises(Exception):
        controller.strength_limited(flat, qpos[:2], qvel)
    with pytest.raises(Exception):
        probe.spindle_strength_limited(flat, qpos[:2], qvel)
    _rec_err('strength_limited')
    _rec_in('strength_limited', {'torques': list(map(float, flat)),
                                 'qpos': list(map(float, qpos)),
                                 'qvel': list(map(float, qvel[:2]))})
    with pytest.raises(Exception):
        controller.strength_limited(flat, qpos, qvel[:2])
    with pytest.raises(Exception):
        probe.spindle_strength_limited(flat, qpos, qvel[:2])
    _rec_err('strength_limited')
    _rec_in('solved_effort', {'qpos': list(map(float, qpos[:2])),
                              'qvel': list(map(float, qvel)),
                              'dt_s': .00125})
    with pytest.raises(Exception):
        solved_effort(controller, data, (qpos[:2], qvel), .00125)
    with pytest.raises(Exception):
        probe.spindle_solved_effort(qpos[:2], qvel, .00125)
    _rec_err('solved_effort')
    _rec_in('solved_effort', {'qpos': list(map(float, qpos)),
                              'qvel': list(map(float, qvel[:2])),
                              'dt_s': .00125})
    with pytest.raises(Exception):
        solved_effort(controller, data, (qpos, qvel[:2]), .00125)
    with pytest.raises(Exception):
        probe.spindle_solved_effort(qpos, qvel[:2], .00125)
    _rec_err('solved_effort')


def test_spindle_lookups_reject_unknown_joints(tmp_path):
    """A name outside the configured rider joints must reject like the
    oracle's self.joints[name]/table lookups — never an unchecked index
    into the joint arrays. The locked ankle references are absent from
    self.joints on this nine-actuator plant, and 'rider_wrist' is absent
    from every table."""
    model, controller, probe = _plant(tmp_path)
    data = _staged(model, probe, crank_q=.4)
    for name in ('rider_ankle_front', 'rider_wrist'):
        _rec_in('write', {'torques': {name: 0.}})
        with pytest.raises(Exception):
            controller.write(data, {name: 0.})
        with pytest.raises(Exception):
            probe.spindle_write({name: 0.})
        _rec_err('write')
    _rec_in('anatomical', {'joint': 'rider_wrist', 'qpos_value': 0.})
    with pytest.raises(Exception):
        controller.anatomical_joint_angle('rider_wrist', 0.)
    with pytest.raises(Exception):
        probe.spindle_anatomical('rider_wrist', 0.)
    _rec_err('anatomical')
    _rec_in('capacity', {'joint': 'rider_wrist', 'angle_rad': 0.,
                         'velocity_rad_s': 0., 'torque_nm': 5.})
    with pytest.raises(Exception):
        controller.strength_capacity('rider_wrist', 0., 0., 5.)
    with pytest.raises(Exception):
        probe.spindle_capacity('rider_wrist', 0., 0., 5.)
    _rec_err('capacity')


def test_spindle_probe_rejects_bad_input(tmp_path):
    from bike_sim.geometry.specs import BikeSpecs
    from bike_sim.native.setup import rider_controller_config
    from bike_sim.physics.rider import RiderSpecs
    from bike_sim.physics.rider_segments import geometry_pose
    model, controller, probe = _plant(tmp_path)
    pose = plain(asdict(geometry_pose(
        RiderSpecs(variant='articulated_planar'), BikeSpecs())))
    _rec_in('compute', {'command': {}, 'advance': True, 'dt_s': None,
                        'steady_state': False})
    with pytest.raises(Exception):
        probe.spindle_compute({}, True, None, False)
    _rec_err('compute')
    # The negative-effort document is spelled directly: routing it through
    # _command() would raise in RiderCommand.__post_init__ before the wire
    # call ever ran, leaving the adapter's own rejection unexercised.
    bad_command = {'mean_crank_torque_nm': -1., 'enabled': True,
                   'posture': {'torso_lean_rad': 0., 'pelvis_pitch_rad': 0.,
                               'pelvis_offset_m': None, 'use_saddle': True},
                   'crank_target_phase_rad': None,
                   'crank_target_rate_rad_s': 0.}
    _rec_in('compute', {'command': bad_command, 'advance': True,
                        'dt_s': None, 'steady_state': False})
    with pytest.raises(Exception):
        probe.spindle_compute(bad_command, True, None, False)
    _rec_err('compute')
    _rec_in('restore', {'state': {'enabled': True}})
    with pytest.raises(Exception):
        probe.spindle_restore({'enabled': True})
    _rec_err('restore')
    # A schema-valid non-spindle attachment routes to the ported legacy
    # controller rather than rejecting at decode.
    config = rider_controller_config(controller.config, controller)
    config['pedal_attachment'] = 'flat'
    path = tmp_path / 'flat.mjb'
    mujoco.mj_saveModel(model, str(path))
    flat_adapter = _adapter(path, {'spindle_controller': {
        'config': config,
        'pose': pose,
        'crank_length_m': 0.17}})
    _rec_in('compute', {'command': _command_dict(
        _command(mean_crank_torque_nm=5.0)), 'advance': True, 'dt_s': None,
                        'steady_state': False})
    flat_adapter.spindle_compute(
        _command_dict(_command(mean_crank_torque_nm=5.0)),
        True, None, False)
    # A mode outside the schema set is still rejected at decode.
    config['pedal_attachment'] = 'pin'
    path = tmp_path / 'pin.mjb'
    mujoco.mj_saveModel(model, str(path))
    with pytest.raises(Exception):
        _adapter(path, {'spindle_controller': {
            'config': config,
            'pose': pose,
            'crank_length_m': 0.17}})
    # A missing pose/config is rejected at adapter construction.
    with pytest.raises(Exception):
        _adapter(path, {'spindle_controller': {'config': config}})

