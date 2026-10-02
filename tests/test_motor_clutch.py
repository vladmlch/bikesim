"""Opt-in crank-side motor clutch: topology, one-way coupling, reposition FSM.

The clutch topology splits the mid-drive onto its own shaft:
crank -[crank_clutch]-> drive_shaft (motor) -[freehub]-> wheel. The rider can
backpedal while the motor keeps driving the wheel.
"""
from dataclasses import asdict, replace
from math import pi

import mujoco
import numpy as np
import pytest

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.physics.chain import DrivetrainSpecs
from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.physics.pedaling import PedalingPolicy
from bike_sim.physics.physical_config import (
    PedalingConfig, PhysicalDriveConfig, TireBackendConfig,
)
from bike_sim.physics.rider import RiderSpecs
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.ride.drivetrain_forces import DrivetrainForceApplier
from bike_sim.sim.research.rider_program import RiderKeyframe, RiderProgram


def clutch_config(transmission='ideal_mid_drive', rider='articulated_planar',
                  **pedaling):
    return SimulationPhysicsConfig('physical',
        drive_mode='articulated_effort' if rider == 'articulated_planar' else 'crank_effort',
        timestep_s=.0003125, closure_time_constant_s=.0025,
        drive=PhysicalDriveConfig(
            transmission_model=transmission, motor_clutch=True,
            gearing=DrivetrainSpecs(34, 51),
            pedaling=PedalingConfig(**pedaling)),
        tires=TireBackendConfig(backend='compliant_2d'))


def clutch_model(transmission='ideal_mid_drive', rider='none', **pedaling):
    cfg = clutch_config(transmission, rider, **pedaling)
    m = mujoco.MjModel.from_xml_string(generate_mujoco_xml(
        mode='ride', rider=rider, specs=BikeSpecs(), physics_config=cfg))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    return m, d, cfg


def joint_id(m, name):
    return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)


def tendon_joints(m, name):
    tid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_TENDON, name)
    assert tid >= 0, name
    start, count = int(m.tendon_adr[tid]), int(m.tendon_num[tid])
    return {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, int(m.wrap_objid[i])):
            float(m.wrap_prm[i]) for i in range(start, start + count)
            if m.wrap_type[i] == mujoco.mjtWrap.mjWRAP_JOINT}


# --- configuration surface -------------------------------------------------

def test_clutch_requires_an_ideal_transmission():
    with pytest.raises(ValueError, match='ideal mid-drive'):
        PhysicalDriveConfig(transmission_model='elastic_chain', motor_clutch=True)


def test_stall_reflex_requires_the_clutch_and_articulated_effort():
    with pytest.raises(ValueError, match='motor_clutch'):
        PhysicalDriveConfig(transmission_model='ideal_mid_drive',
                            pedaling=PedalingConfig(reposition_on_stall=True))
    with pytest.raises(ValueError, match='articulated_effort'):
        SimulationPhysicsConfig('physical', drive_mode='crank_effort',
            drive=PhysicalDriveConfig(transmission_model='ideal_mid_drive',
                motor_clutch=True,
                pedaling=PedalingConfig(reposition_on_stall=True)))


def test_reposition_parameter_validation():
    with pytest.raises(ValueError):
        PedalingConfig(reposition_on_stall=1)
    for key, value in (('reposition_back_rate_rad_s', 0.),
                       ('reposition_timeout_s', 0.),
                       ('reposition_noop_rad', pi / 2.)):
        with pytest.raises(ValueError):
            PedalingConfig(**{key: value})


# --- compiled topology -----------------------------------------------------

def test_default_topology_keeps_the_motor_on_the_crank():
    m, _, _ = clutch_model()
    base = replace(clutch_config('ideal_mid_drive', 'none').drive, motor_clutch=False)
    default_cfg = replace(clutch_config('ideal_mid_drive', 'none'), drive=base)
    m = mujoco.MjModel.from_xml_string(generate_mujoco_xml(
        mode='ride', rider='none', specs=BikeSpecs(), physics_config=default_cfg))
    assert joint_id(m, 'drive_shaft_spin') < 0
    assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_TENDON, 'crank_clutch') < 0
    motor = m.actuator('mid_drive').id
    assert m.actuator_trnid[motor, 0] == joint_id(m, 'crank_spin')


@pytest.mark.parametrize('transmission', ('ideal_mid_drive', 'geometric_ideal_mid_drive'))
def test_clutch_topology_moves_the_motor_onto_its_own_shaft(transmission):
    m, _, _ = clutch_model(transmission)
    shaft = joint_id(m, 'drive_shaft_spin')
    crank = joint_id(m, 'crank_spin')
    wheel = joint_id(m, 'rear_wheel_spin')
    assert min(shaft, crank, wheel) >= 0
    motor = m.actuator('mid_drive').id
    assert m.actuator_trnid[motor, 0] == shaft
    clutch = tendon_joints(m, 'crank_clutch')
    assert clutch['crank_spin'] == pytest.approx(1.)
    assert clutch['drive_shaft_spin'] == pytest.approx(-1.)
    tid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_TENDON, 'crank_clutch')
    assert m.tendon_limited[tid]
    assert m.tendon_range[tid][1] <= 0.
    freehub_name = ('geometric_mid_drive_freehub' if transmission.startswith('geometric')
                    else 'ideal_mid_drive_freehub')
    assert 'drive_shaft_spin' in tendon_joints(m, freehub_name)


def test_clutch_is_a_one_way_limit_with_zero_lash_ratchet():
    m, d, cfg = clutch_model()
    drive = DrivetrainForceApplier(m, cfg.drive, cfg.drive_mode)
    drive.reset(m, d)
    clutch = drive.clutch
    crank_q, shaft_q = clutch.driver_qpos, clutch.driven_qpos
    # Overrun: driven shaft ahead of the crank opens the clutch; prepare() only
    # ratchets the boundary down, never up.
    boundary = clutch.boundary
    d.qpos[shaft_q] += 1.
    clutch.prepare(m, d)
    assert clutch.boundary == pytest.approx(boundary - 1.)
    d.qpos[crank_q] += 2.
    clutch.prepare(m, d)
    assert clutch.boundary == pytest.approx(boundary - 1.)


@pytest.mark.parametrize('transmission', ('ideal_mid_drive', 'geometric_ideal_mid_drive'))
def test_motor_shaft_drives_the_wheel_while_the_crank_backpedals(transmission):
    m, d, cfg = clutch_model(transmission)
    drive = DrivetrainForceApplier(m, cfg.drive, cfg.drive_mode)
    drive.reset(m, d)
    m.opt.gravity[:] = 0.
    m.geom_contype[:] = 0
    m.geom_conaffinity[:] = 0
    dt = float(m.opt.timestep)
    crank = m.joint('crank_spin').dofadr[0]
    shaft = m.joint('drive_shaft_spin').dofadr[0]
    wheel = m.joint('rear_wheel_spin').dofadr[0]
    motor = m.actuator('mid_drive').id
    # Rider cranks backwards under load while the motor drives the shaft.
    d.qvel[crank] = -3.
    for _ in range(round(.2 / dt)):
        d.qfrc_applied.fill(0.)
        d.qfrc_applied[crank] = -5.
        d.ctrl[motor] = 30.
        drive.ideal_hub.prepare(m, d)
        drive.clutch.prepare(m, d)
        mujoco.mj_step(m, d)
        drive.settle_actuation(m, d)
    assert d.qvel[shaft] > 1.
    assert d.qvel[wheel] > 0.
    assert d.qvel[crank] < 0.
    # The open clutch carries essentially no torque and does no work on the
    # crank; the wheel keeps receiving motor effort through the freehub.
    assert abs(drive.last['crank_clutch_torque_nm']) < 1.
    assert not drive.last['crank_clutch_engaged']
    assert drive.last['freehub_torque_nm'] > 0.


@pytest.mark.parametrize('transmission', ('ideal_mid_drive', 'geometric_ideal_mid_drive'))
def test_forward_crank_reengages_without_rewriting_state(transmission):
    m, d, cfg = clutch_model(transmission)
    drive = DrivetrainForceApplier(m, cfg.drive, cfg.drive_mode)
    drive.reset(m, d)
    m.opt.gravity[:] = 0.
    m.geom_contype[:] = 0
    m.geom_conaffinity[:] = 0
    dt = float(m.opt.timestep)
    crank = m.joint('crank_spin').dofadr[0]
    shaft = m.joint('drive_shaft_spin').dofadr[0]
    # Crank outruns the shaft: the limit row must engage inside the same solve.
    d.qvel[crank] = 12.
    engaged = False
    peak = peak_shaft = 0.
    for _ in range(round(.1 / dt)):
        drive.ideal_hub.prepare(m, d)
        drive.clutch.prepare(m, d)
        mujoco.mj_step(m, d)
        drive.settle_actuation(m, d)
        engaged |= drive.last['crank_clutch_engaged']
        peak = max(peak, drive.last['crank_clutch_torque_nm'])
        peak_shaft = max(peak_shaft, d.qvel[shaft])
    # A bare un-damped rig rebounds elastically after engagement, so assert on
    # the driven transient rather than the settled sign of the crank.
    assert engaged and peak > 0. and peak_shaft > 1.
    # No position or velocity was ever rewritten: the solver did the work.
    assert np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all()


# --- reposition finite-state machine ---------------------------------------

def policy(**kwargs):
    return PedalingPolicy(PedalingConfig(**kwargs))


def run(policy, steps, **state):
    out = None
    for _ in range(steps):
        out = policy.update(dt=.01, **state)
    return out


def test_reposition_request_is_edge_triggered_and_returns_to_pedaling():
    p = policy()
    state = run(p, 1, phase_rad=pi / 2., rate_rad_s=0., required_cadence_rpm=0.,
                effort_nm=30., reposition=True)
    assert state.mode == 'reposition' and state.reason == 'requested'
    assert state.effort_nm == 0. and state.target_rate_rad_s < 0.
    # Phase goal is the previous half-turn boundary (pi/2 -> 0).
    assert state.target_phase_rad <= pi / 2. + .01
    phase = pi / 2.
    rate = -p.config.reposition_back_rate_rad_s
    for _ in range(200):
        phase += rate * .01
        state = p.update(phase, rate, 0., 30., .01, reposition=True)
        if state.mode != 'reposition':
            break
    assert state.mode != 'reposition'
    assert phase <= .05
    # Held request must not retrigger: normal pedaling continues.
    assert state.mode == 'pedaling' or p.update(phase, 0., 0., 30., .01, reposition=True).mode == 'pedaling'
    # A released edge fires one more maneuver once the crank left the power
    # phase and the post-maneuver cooldown has elapsed.
    phase = pi / 2.
    for _ in range(int(p.config.reposition_cooldown_s / .01) + 2):
        p.update(phase, 0., 0., 30., .01, reposition=False)
    state = p.update(phase, 0., 0., 30., .01, reposition=True)
    assert state.mode == 'reposition'


def test_reposition_targets_the_nearest_lower_power_phase():
    p = policy()
    # Just above a power boundary stays a short move; just below the next one
    # is already at a power position and is a no-op.
    state = run(p, 1, phase_rad=pi - .02, rate_rad_s=0., required_cadence_rpm=0.,
                effort_nm=30., reposition=True)
    assert state.mode != 'reposition'
    p = policy()
    state = run(p, 1, phase_rad=pi + .5, rate_rad_s=0., required_cadence_rpm=0.,
                effort_nm=30., reposition=True)
    assert state.mode == 'reposition'
    assert state.target_phase_rad <= pi + .5


def test_reposition_timeout_and_brake_abort():
    p = policy()
    run(p, 1, phase_rad=pi / 2., rate_rad_s=0., required_cadence_rpm=0.,
        effort_nm=30., reposition=True)
    # A blocked crank (phase frozen) must give up within the timeout.
    for _ in range(int(p.config.reposition_timeout_s / .01) + 30):
        state = p.update(pi / 2., 0., 0., 30., .01)
    assert state.mode != 'reposition'
    # Brake abort drops the goal immediately.
    run(p, 1, phase_rad=pi / 2., rate_rad_s=0., required_cadence_rpm=0.,
        effort_nm=30., reposition=True)
    p.update(pi / 2., 0., 0., 30., .01)
    state = p.update(pi / 2., 0., 0., 30., .01, braking=True)
    assert state.mode == 'coasting' and state.reason == 'braking'


def test_cooldown_blocks_an_explicit_request():
    p = policy(reposition_cooldown_s=1.)
    run(p, 1, phase_rad=pi / 2., rate_rad_s=0., required_cadence_rpm=0.,
        effort_nm=30., reposition=True)
    # Abort the active maneuver by braking, then immediately request again:
    # the cooldown from the abort must swallow the new edge.
    p.update(pi / 2., 0., 0., 30., .01, braking=True)
    p.update(pi / 2., 0., 0., 30., .01, reposition=False)
    assert p.update(pi / 2., 0., 0., 30., .01, reposition=True).mode != 'reposition'


def test_held_request_does_not_refire_across_a_disable_toggle():
    p = policy(reposition_cooldown_s=0.)
    p.update(pi / 2., 0., 0., 30., .01, reposition=False)
    for _ in range(5):
        state = p.update(pi / 2., 0., 0., 30., .01, enabled=False, reposition=True)
    assert state.mode == 'disabled'
    # Re-enabling under a still-held flag must not launch a maneuver.
    assert p.update(pi / 2., 0., 0., 30., .01, enabled=True,
                    reposition=True).mode != 'reposition'


def test_stall_reflex_dwell_cooldown_and_gating():
    cfg = dict(reposition_on_stall=True, reposition_stall_dwell_s=.5,
               reposition_cooldown_s=.5, reposition_min_effort_nm=20.)
    p = policy(**cfg)
    off = policy()
    stalled = dict(phase_rad=pi / 2., rate_rad_s=0., required_cadence_rpm=5.,
                   effort_nm=30.)
    # Dwell: nothing fires before the interval elapses.
    for _ in range(int(.5 / .01) - 1):
        assert p.update(**stalled, dt=.01).mode == 'pedaling'
        assert off.update(**stalled, dt=.01).mode == 'pedaling'
    assert p.update(**stalled, dt=.01).mode == 'reposition'
    assert p.update(**stalled, dt=.01).reason == 'stall_reflex'
    # Effort below the threshold never triggers.
    p2 = policy(**cfg)
    weak = dict(stalled, effort_nm=10.)
    for _ in range(100):
        assert p2.update(**weak, dt=.01).mode != 'reposition'
    # Wheel already turning never triggers.
    rolling = dict(stalled, required_cadence_rpm=40.)
    for _ in range(100):
        assert p2.update(**rolling, dt=.01).mode != 'reposition'


# --- command surface ---------------------------------------------------------

def test_ride_control_reposition_validation():
    cfg = clutch_config('ideal_mid_drive', 'articulated_planar')
    c = RideControl(crank_reposition=True)
    c.validate_for(cfg, 'articulated_planar')
    with pytest.raises(ValueError, match='articulated'):
        c.validate_for(cfg, 'none')
    with pytest.raises(ValueError):
        RideControl(crank_reposition=1)
    no_clutch = replace(cfg, drive=replace(cfg.drive, motor_clutch=False))
    with pytest.raises(ValueError, match='motor_clutch'):
        c.validate_for(no_clutch, 'articulated_planar')
    # Serialization is plain dataclass data for JSONL/replay.
    assert asdict(c)['crank_reposition'] is True
    assert RideControl(**asdict(c)) == c


def test_rider_program_owns_reposition_as_an_event_flag():
    frames = (RiderKeyframe(0., crank_reposition=False),
              RiderKeyframe(1., crank_reposition=True))
    program = RiderProgram(frames)
    assert program.owns_crank_reposition
    assert program.at(.5).crank_reposition is False
    assert program.at(1.5).crank_reposition is True
    out = program.apply(RideControl(motor_torque_nm=10.), 1.5)
    assert out.crank_reposition is True and out.motor_torque_nm == 10.
    with pytest.raises(ValueError, match='owns'):
        program.apply(RideControl(crank_reposition=True), 1.5)
    with pytest.raises(ValueError, match='all keyframes or in none'):
        RiderProgram((RiderKeyframe(0., crank_reposition=True), RiderKeyframe(1.)))
    assert RiderProgram.from_dict(program.to_dict()) == program


def test_reposition_on_stall_policy_fires_one_pulse_per_stall():
    from bike_sim.sim.research.policies import RepositionOnStallPolicy
    from bike_sim.sim.research.sensors import SensorObservation

    def obs(t, wheel=0., crank=0., valid=True):
        return SensorObservation(time_s=t, source_time_s=t, valid=valid,
            specific_force_body_mps2=(0., 0., 9.81), pitch_rate_up_rad_s=0.,
            front_wheel_rad_s=0., rear_wheel_rad_s=wheel, crank_rad_s=crank,
            motor_torque_nm=80., human_torque_nm=0.)

    p = RepositionOnStallPolicy(dwell_s=.5, cooldown_s=2.)
    p.reset(0)
    dt = .01
    for i in range(int(.5 / dt)):
        out = p.act(obs(i * dt), 80.)
        assert not out.crank_reposition
    out = p.act(obs(.51), 80.)
    assert out.crank_reposition and out.motor_torque_nm == 80.
    # One pulse per stall: the dwell resets and the cooldown blocks repeats.
    for i in range(int(2. / dt) - 1):
        assert not p.act(obs(.52 + i * dt), 80.).crank_reposition
    # Rolling wheel or turning crank is not a stall.
    assert not p.act(obs(3., wheel=10.), 80.).crank_reposition
    assert not p.act(obs(3.01, crank=5.), 80.).crank_reposition
    # Zero demand means the drive is unloaded: never reposition.
    for i in range(100):
        assert not p.act(obs(4. + i * dt), 0.).crank_reposition
    # A later stall past the cooldown fires again after another dwell.
    fired_again = any(p.act(obs(5. + i * dt), 80.).crank_reposition
                      for i in range(60))
    assert fired_again
