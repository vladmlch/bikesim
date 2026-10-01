"""Tests for articulated pedal_attachment='weld' (clipless pedal mode)."""
import mujoco
import numpy as np
import pytest

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.physics.physical_config import (
    ArticulatedConfig, PedalingConfig, PhysicalDriveConfig)
from bike_sim.physics.resolution import load_physics_config
from bike_sim.physics.rider import RiderSpecs
from bike_sim.physics.rider_segments import geometry_pose
from bike_sim.sim.ride.rider_control import ArticulatedRiderController


def _config(attachment='weld', **kwargs):
    values = dict(drive_mode='articulated_effort', timestep_s=.00125,
                  closure_time_constant_s=.0025,
                  articulated=ArticulatedConfig(pedal_attachment=attachment),
                  drive=PhysicalDriveConfig(
                      transmission_model='ideal_mid_drive',
                      human_torque_nm=20.))
    values.update(kwargs)
    physics_mode = values.pop('physics_mode', 'physical')
    return SimulationPhysicsConfig(physics_mode, **values)


def welded_rig(transmission='ideal_mid_drive', human=20., speed=0.):
    """Compiled physical model + posed controller with welded feet."""
    specs = BikeSpecs()
    cfg = _config(initial_speed_mps=speed,
                  drive=PhysicalDriveConfig(transmission_model=transmission,
                                            human_torque_nm=human))
    rider = RiderSpecs(variant='articulated_planar')
    model = mujoco.MjModel.from_xml_string(generate_mujoco_xml(
        mode='ride', rider=rider, physics_config=cfg))
    data = mujoco.MjData(model)
    controller = ArticulatedRiderController(
        model, geometry_pose(rider, specs), cfg.articulated,
        specs.crank_length / 1000.)
    controller.initialize(model, data)
    mujoco.mj_forward(model, data)
    return model, data, controller, cfg


def test_pedal_attachment_defaults_to_flat():
    assert ArticulatedConfig().pedal_attachment == 'flat'


def test_pedal_attachment_accepts_weld():
    assert ArticulatedConfig(pedal_attachment='weld').pedal_attachment == 'weld'


def test_pedal_attachment_rejects_unknown_values():
    with pytest.raises(ValueError, match='pedal_attachment'):
        ArticulatedConfig(pedal_attachment='clipless')


def test_physics_toml_loads_articulated_pedal_attachment(tmp_path):
    path = tmp_path / 'physics.toml'
    path.write_text(
        'physics_mode = "physical"\n'
        'drive_mode = "articulated_effort"\n'
        'timestep_s = 0.00125\n'
        'closure_time_constant_s = 0.0025\n'
        '[articulated]\n'
        'pedal_attachment = "weld"\n')
    cfg = load_physics_config(str(path), {})
    assert cfg.articulated.pedal_attachment == 'weld'


def _equality_names(model):
    return {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_EQUALITY, i)
            for i in range(model.neq)}


def test_weld_mode_emits_foot_pedal_equalities():
    model, _, _, _ = welded_rig()
    for side in ('front', 'rear'):
        eq = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY,
                                   f'weld_foot_{side}'))
        assert eq >= 0
        assert model.eq_type[eq] == mujoco.mjtEq.mjEQ_WELD
        foot = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                                 f'rider_foot_{side}')
        pedal = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                                  f'pedal_{side}')
        assert {int(model.eq_obj1id[eq]), int(model.eq_obj2id[eq])} == {foot, pedal}
        # solref[0] = closure time constant: >= 2*dt, critically damped row.
        assert model.eq_solref[eq][0] == pytest.approx(
            max(2. * model.opt.timestep, .0025))
        assert model.eq_solref[eq][1] == pytest.approx(1.)


def test_flat_mode_keeps_physical_feet_unwelded():
    cfg = _config('flat')
    model = mujoco.MjModel.from_xml_string(generate_mujoco_xml(
        mode='ride', rider=RiderSpecs(variant='articulated_planar'),
        physics_config=cfg))
    assert not any(name.startswith('weld_foot_')
                   for name in _equality_names(model) if name)


def test_weld_mode_rejects_non_articulated_riders():
    # drive_mode='coast' avoids the earlier 'articulated_effort requires
    # articulated_planar' check so the weld guard itself is what raises.
    cfg = _config('weld', drive_mode='coast')
    with pytest.raises((ValueError, RuntimeError)):
        generate_mujoco_xml(mode='ride', rider=RiderSpecs(variant='lumped'),
                            physics_config=cfg)


def test_weld_mode_rejects_nonphysical_mode():
    # legacy + drive_mode='coast' constructs cleanly (legacy rejects the
    # articulated_effort default at config level); the lumped rider avoids the
    # earlier 'articulated_planar requires physical' check, so the weld guard
    # itself is what raises.
    cfg = _config('weld', physics_mode='legacy', drive_mode='coast')
    with pytest.raises((ValueError, RuntimeError)):
        generate_mujoco_xml(mode='ride',
                            rider=RiderSpecs(variant='lumped'),
                            physics_config=cfg)


def test_pedal_welds_reader_reports_forces_and_residual():
    from bike_sim.sim.ride.weld_pedals import PedalWelds
    model, data, _, _ = welded_rig()
    welds = PedalWelds(model)
    foot = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                             'rider_foot_front')
    data.xfrc_applied[foot] = [0., 0., -300., 0., 0., 0.]
    for _ in range(20):
        mujoco.mj_step(model, data)
    force_on_rider = welds.force_on_rider_n(model, data, 'front')
    assert force_on_rider[2] > 0.          # weld pushes the loaded foot up
    assert welds.translation_residual_m(model, data, 'front') < .003
    # Downward force on the front pedal (arm at 3 o'clock at design pose)
    # must read as positive forward crank torque — the torque-sensor feed.
    assert welds.delivered_crank_torque_nm(model, data) > 0.


def test_pedal_welds_reader_is_quiet_when_unloaded():
    from bike_sim.sim.ride.weld_pedals import PedalWelds
    model, data, _, _ = welded_rig()
    welds = PedalWelds(model)
    for side in ('front', 'rear'):
        assert welds.translation_residual_m(model, data, side) < .001
    assert abs(welds.delivered_crank_torque_nm(model, data)) < 5.


def test_weld_mode_applier_reports_weld_diagnostics_and_no_pad_qfrc():
    from bike_sim.sim.ride.rider_contacts import RiderContactApplier
    model, data, controller, cfg = welded_rig()
    applier = RiderContactApplier(model, controller.pose, cfg.articulated)
    applier.reset(model, data)
    applier.initialize_settled_state(model, data)
    qfrc = applier.compute_qfrc(model, data, model.opt.timestep)
    diag = applier.diagnostics['front_pedal']
    assert diag['in_platform'] and diag['enabled']
    assert diag['normal_load_n'] >= 0.
    assert diag['gap_m'] < .003          # weld translation residual
    # The weld is solver-side: the applier must not double-apply its force.
    crank_dof = int(model.joint('crank_spin').dofadr[0])
    assert qfrc[crank_dof] == 0.
    # delivered_crank_torque_nm is weld-derived; saddle/grip entries survive.
    from bike_sim.sim.ride.weld_pedals import PedalWelds
    assert applier.delivered_crank_torque_nm == pytest.approx(
        PedalWelds(model).delivered_crank_torque_nm(model, data))
    assert 'saddle' in applier.diagnostics and 'grip' in applier.diagnostics


def test_weld_mode_pedal_release_is_a_noop():
    from bike_sim.sim.ride.rider_contacts import RiderContactApplier
    model, data, controller, cfg = welded_rig()
    applier = RiderContactApplier(model, controller.pose, cfg.articulated)
    applier.reset(model, data)
    assert applier.set_enabled('front_pedal', False) is True
    assert applier.enabled['front_pedal']


def test_weld_mode_sole_targets_track_the_welded_anchor():
    model, data, controller, _ = welded_rig()
    from bike_sim.sim.ride.rider_control import RiderCommand
    controller.compute(model, data, RiderCommand(20.),
        contact_loads={'front':100., 'rear':100., 'saddle':400., 'grip':True})
    for side in ('front', 'rear'):
        # The target must be the welded sole position, not a footprint
        # projection: it coincides with the actual sole site (< 3 mm residual).
        target = np.asarray(controller.sole_targets[side])
        actual = np.asarray(data.site_xpos[controller.soles[side]])
        assert np.linalg.norm(target - actual) < .003


def test_weld_mode_never_enters_pedal_recovery():
    model, data, controller, _ = welded_rig()
    from bike_sim.sim.ride.rider_control import RiderCommand
    # Force the sole below the pedal platform — the flat-mode recovery trigger.
    data.qpos[model.joint('rider_root_z').qposadr[0]] -= .04
    mujoco.mj_forward(model, data)
    controller.compute(model, data, RiderCommand(20.),
        contact_loads={'front':100., 'rear':100., 'saddle':400., 'grip':True},
        support_states={'front_pedal':{'normal_load_n':200.,'in_platform':True,
                                       'force_on_rider_n':[0.,0.,-50.]},
                        'rear_pedal':{'normal_load_n':200.,'in_platform':True,
                                      'force_on_rider_n':[0.,0.,-50.]}})
    feet = controller.support_diagnostics['feet']
    assert all(entry['recovery_stage'] == 'none' for entry in feet.values())


def test_weld_mode_feet_stay_pinned_when_crank_is_kicked():
    model, data, controller, _ = welded_rig()
    from bike_sim.sim.ride.weld_pedals import PedalWelds
    welds = PedalWelds(model)
    data.qvel[controller.crank_spin_dof] = 8.   # sudden cadence jump
    # A raw qvel write is not a weld-consistent velocity: the soft equality
    # (solref >= 2*dt, the stiffest stable setting) bleeds the relative-mode
    # impulse over ~25 ms. Also, mj_objectVelocity(BODY) reads data.cvel --
    # the velocity a step's forward pass saw on *entry* -- so asserted
    # iterations only ever inspect velocities the previous solve produced.
    # Assert the position bound through the impulse itself, then the shared
    # angular velocity once the bodies' motion has equalized.
    for _ in range(40):
        mujoco.mj_step(model, data)
        assert welds.translation_residual_m(model, data, 'front') < .003
        assert welds.translation_residual_m(model, data, 'rear') < .003
    for _ in range(100):
        mujoco.mj_step(model, data)
        assert welds.translation_residual_m(model, data, 'front') < .003
        assert welds.translation_residual_m(model, data, 'rear') < .003
        for side in ('front', 'rear'):
            # mj_objectVelocity fills res[0:3] = angular, res[3:6] = linear:
            # welded bodies must share angular velocity (residual covers linear).
            foot_v = np.zeros(6); pedal_v = np.zeros(6)
            mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY,
                int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                    f'rider_foot_{side}')), foot_v, 0)
            mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY,
                int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                    f'pedal_{side}')), pedal_v, 0)
            np.testing.assert_allclose(foot_v[:3], pedal_v[:3], atol=.05)


@pytest.mark.slow
def test_welded_ride_holds_feet_through_pedaling_and_coast():
    from bike_sim.terrain import get_preset
    from bike_sim.sim.ride_sim import RideSimulation
    from bike_sim.sim.ride.weld_pedals import PedalWelds
    cfg = SimulationPhysicsConfig(
        'physical', drive_mode='articulated_effort', timestep_s=.00125,
        closure_time_constant_s=.0025, initial_speed_mps=4.,
        articulated=ArticulatedConfig(pedal_attachment='weld'),
        drive=PhysicalDriveConfig(
            transmission_model='ideal_mid_drive', human_torque_nm=20.,
            # Cadence-triggered coasting exercises pedaling first only if
            # coast_above_rpm sits between the wheel-synced launch cadence
            # (~77 rpm at 4 m/s -- below it the drivetrain starts already
            # coasting) and the cadence the assisted rider spins up to
            # (~111 rpm here). 95/80 rpm lands pedal->coast->pedal->coast.
            pedaling=PedalingConfig(enabled=True, coast_above_rpm=95.,
                                    resume_below_rpm=80., stop_time_s=.35)))
    sim = RideSimulation(track=get_preset('flat'), rider='articulated_planar',
                         physics_config=cfg)
    rt = sim.physical
    welds = PedalWelds(sim.model)
    worst_residual = 0.
    modes = set()
    max_sensed = 0.
    for _ in range(int(6. / cfg.timestep_s)):
        sim.step()
        modes.add(rt.drive.last.get('rider_mode'))
        max_sensed = max(max_sensed, abs(float(rt.drive.last['human_sensor_nm'])))
        for side in ('front', 'rear'):
            worst_residual = max(worst_residual,
                welds.translation_residual_m(sim.model, sim.data, side))
        feet = rt.rider_control.support_diagnostics['feet']
        assert all(entry['recovery_stage'] == 'none' for entry in feet.values())
    assert worst_residual < .003
    # The name promises both halves: a real pedaling phase AND a real coast.
    assert {'pedaling', 'coasting'} <= modes, f'observed rider modes: {modes}'
    # The weld-mode torque sensor must feed the drivetrain observer a real
    # nonzero weld-derived signal, not merely exist (weld reactions peak
    # >100 Nm through the pedal/coast transitions; .5 Nm is far inside that).
    assert max_sensed > .5, f'max |human_sensor_nm| observed: {max_sensed}'
