# tests/reference/test_pneumatic_tire_forces.py
"""The physical-runtime pneumatic applier: qfrc surface, snapshot schema, state wire."""
import numpy as np
import mujoco
import pytest

from bike_sim.physics.physical_config import PneumaticTireConfig, TireBackendConfig
from bike_sim.physics.tyre import FRONT_TYRE, REAR_TYRE, TIERS
from bike_sim.sim.ride.contact_state import ContactPatch, WheelContactSnapshot
from bike_sim.sim.ride.pneumatic_tire_forces import PneumaticTireForceApplier
from bike_sim.sim.ride.physical_mapping import point_jacobian
from bike_sim.sim.ride.tire_forces import interval_id
from bike_sim.sim.ride.tyre.geometry import RoadProfile
from bike_sim.terrain.surface import SurfaceMap

FRONT_R = FRONT_TYRE.outer_radius_mm / 1000.0
REAR_R = REAR_TYRE.outer_radius_mm / 1000.0


def _wheel_rig(dt_s=5e-4, *, tier='fast', front_drop_m=0.004, rear_z_m=5.0):
    """Two named wheels on a flat road; the rear one starts airborne by default."""
    xml = f'''<mujoco><compiler angle="radian"/>
      <option timestep="{dt_s}" gravity="0 0 -9.81" integrator="Euler"/>
      <worldbody>
        <body name="front_wheel" pos="0 0 {FRONT_R - front_drop_m}">
          <joint name="front_root_x" type="slide" axis="1 0 0"/>
          <joint name="front_root_z" type="slide" axis="0 0 1"/>
          <joint name="front_wheel_spin" type="hinge" axis="0 1 0"/>
          <inertial mass="6" pos="0 0 0" diaginertia=".3 .5 .3"/>
          <geom name="geom_front_contact" type="sphere" size="{FRONT_R}"
                contype="0" conaffinity="0" mass="0"/>
        </body>
        <body name="rear_wheel" pos="0.8 0 {rear_z_m}">
          <joint name="rear_wheel_spin" type="hinge" axis="0 1 0"/>
          <inertial mass="6" pos="0 0 0" diaginertia=".3 .5 .3"/>
          <geom name="geom_rear_contact" type="sphere" size="{REAR_R}"
                contype="0" conaffinity="0" mass="0"/>
        </body>
      </worldbody></mujoco>'''
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    x = np.linspace(-2.0, 3.0, 1001)
    profile = RoadProfile.from_samples(x, np.zeros_like(x))
    config = TireBackendConfig(
        backend='pneumatic',
        pneumatic=PneumaticTireConfig(tier=tier, surface='asphalt'))
    tire = PneumaticTireForceApplier(model, profile, config)
    return model, data, tire


def test_constructor_rejects_wrong_backend_and_missing_surface():
    model, data, _ = _wheel_rig()
    x = np.linspace(-2.0, 3.0, 101)
    profile = RoadProfile.from_samples(x, np.zeros_like(x))
    with pytest.raises(ValueError, match='pneumatic backend'):
        PneumaticTireForceApplier(model, profile, TireBackendConfig())
    with pytest.raises(ValueError, match='surface map'):
        PneumaticTireForceApplier(
            model, profile,
            TireBackendConfig(backend='pneumatic',
                              pneumatic=PneumaticTireConfig(surface=None,
                                                            tier='fast'),
                              surface_mode='track'))


def test_constructor_validates_contact_geom_contype_and_radius():
    model, data, tire = _wheel_rig()
    config = tire.config
    x = np.linspace(-2.0, 3.0, 101)
    profile = RoadProfile.from_samples(x, np.zeros_like(x))
    geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'geom_rear_contact')
    model.geom_contype[geom] = 1
    with pytest.raises(ValueError, match='collisions must be disabled'):
        PneumaticTireForceApplier(model, profile, config)
    model.geom_contype[geom] = 0
    model.geom_size[geom, 0] += 1e-4
    with pytest.raises(ValueError, match='does not'):
        PneumaticTireForceApplier(model, profile, config)


def test_qfrc_sanity_on_a_physical_model():
    """The compiled physical bike: both tyres loaded, qfrc is J^T F per patch."""
    from bike_sim.geometry.hardpoints import compute_ground_z
    from bike_sim.geometry.specs import BikeSpecs
    from bike_sim.mujoco.builder import generate_mujoco_xml
    from bike_sim.physics.model_config import SimulationPhysicsConfig
    specs = BikeSpecs()
    config = SimulationPhysicsConfig(
        physics_mode='physical', drive_mode='coast',
        tires=TireBackendConfig(
            backend='pneumatic',
            pneumatic=PneumaticTireConfig(tier='fast', surface='asphalt')))
    model = mujoco.MjModel.from_xml_string(generate_mujoco_xml(
        specs, mode='ride', rider='none', physics_config=config))
    for side in ('front', 'rear'):
        geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM,
                                 f'geom_{side}_contact')
        model.geom_contype[geom] = 0
        model.geom_conaffinity[geom] = 0
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    bodies = {s: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f'{s}_wheel')
              for s in ('front', 'rear')}
    xs = [data.xpos[bodies[s]][0] for s in ('front', 'rear')]
    x = np.linspace(min(xs) - 1.0, max(xs) + 1.0, 2001)
    profile = RoadProfile.from_samples(
        x, np.full_like(x, compute_ground_z(specs) / 1000.0))
    tire = PneumaticTireForceApplier(model, profile, config.tires)
    # Press the chassis down 15 mm so both tyres carry load.
    root_z = model.jnt_qposadr[
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'root_z')]
    data.qpos[root_z] -= 0.015
    spin = model.jnt_dofadr[
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'rear_wheel_spin')]
    data.qvel[spin] = 3.0
    mujoco.mj_forward(model, data)
    dt = float(model.opt.timestep)
    qfrc = tire.compute_qfrc(model, data, dt)
    assert qfrc.shape == (model.nv,) and np.isfinite(qfrc).all()
    assert np.abs(qfrc).sum() > 0.0
    expected = np.zeros(model.nv)
    for side in ('front', 'rear'):
        snapshot = tire.snapshots[side]
        assert snapshot.normal_load_n > 0.0 and snapshot.patches
        assert snapshot.geometric_contact and snapshot.backend == 'pneumatic'
        for patch in snapshot.patches:
            jp, _ = point_jacobian(model, data, tire.bodies[side], patch.point_m)
            expected += jp.T @ patch.world_force_n
    np.testing.assert_allclose(qfrc, expected, rtol=1e-9, atol=1e-8)


def test_snapshot_schema_and_signed_slip():
    model, data, tire = _wheel_rig()
    dt = float(model.opt.timestep)
    tire.compute_qfrc(model, data, dt)
    front, rear = tire.snapshots['front'], tire.snapshots['rear']
    assert isinstance(front, WheelContactSnapshot) and front.backend == 'pneumatic'
    assert front.geometric_contact and front.patches
    assert front.interval_id == interval_id(0.0, dt) == 0
    for patch in front.patches:
        assert isinstance(patch, ContactPatch)
        assert patch.native_world_force_n is None
        np.testing.assert_allclose(
            patch.world_force_n,
            patch.normal_load_n * patch.normal + patch.tangent_force_n * patch.tangent,
            rtol=0.0, atol=0.0)
    outputs = tire.states['front'].last_outputs
    assert len(front.patches) == len(outputs.patches)
    for snap_patch, out_patch in zip(front.patches, outputs.patches):
        assert snap_patch.slip_mps == out_patch.slip_speed_mps
        assert snap_patch.normal_load_n == out_patch.normal_load_n
    assert not rear.geometric_contact and not rear.patches
    diag = tire.diagnostics['front']
    assert set(diag) == {
        'normal_load_n', 'support_n', 'tangent_force_n', 'slip_mps', 'slip_ratio',
        'transient_slip_ratio', 'friction_coefficient', 'penetration_m',
        'contact_length_m', 'patch_count', 'multi_support',
        'supports_multiple_contacts', 'fully_sliding', 'airborne',
        'coverage_event', 'rim_strike_active', 'rim_event_x_m',
        'rim_event_speed_mps', 'rim_event_peak_load_n', 'rim_event_peak_force_n',
        'rim_event_energy_j', 'pressure_bar', 'dissipated_power_w',
        'brush_loss_j', 'radial_energy_j', 'shear_energy_j',
        'outside_material_load_range', 'surface'}
    assert diag['supports_multiple_contacts'] is True
    assert diag['outside_material_load_range'] is False
    assert diag['multi_support'] == (diag['patch_count'] > 1)
    assert diag['surface'] == 'asphalt'
    assert all(not isinstance(v, (list, tuple, np.ndarray)) for v in diag.values())
    assert tire.diagnostics['rear']['surface'] == 'asphalt'
    assert tire.diagnostics['rear']['airborne'] is True


def test_state_wire_names_and_round_trip():
    model, data, tire = _wheel_rig()
    n = TIERS['fast'].n_rays
    names = tire.state_names()
    width = (n + 1) // 2
    assert len(names) == 2 * (4 * n + 8 + 3 * width)
    assert names[:4] == ['front.ray.000.delta_prev_m', 'front.ray.000.elastic_prev_n',
                         'front.ray.000.maxwell_n', 'front.ray.000.bristle_m']
    assert names[4 * n] == 'front.carcass.initialized'
    assert names[4 * n + 1] == 'front.rim.active'
    assert names[4 * n + 7] == 'front.patch.count'
    assert names[4 * n + 8] == 'front.patch.00.start'
    assert names[-1] == f'rear.patch.{width - 1:02d}.kappa_prime'
    tire.compute_qfrc(model, data, float(model.opt.timestep))
    wire = tire.state_dict()
    assert wire['names'] == names and wire['row'].shape == (len(names),)
    front_count = int(wire['row'][names.index('front.patch.count')])
    assert front_count >= 1
    assert wire['row'][names.index('front.patch.00.stop')] > \
        wire['row'][names.index('front.patch.00.start')]
    assert np.isnan(wire['row'][names.index('front.ray.000.bristle_m')])
    assert wire['row'][names.index('rear.patch.count')] == 0.0
    model2, data2, tire2 = _wheel_rig()
    tire2.set_state(wire['names'], wire['row'])
    again = tire2.state_dict()
    assert np.array_equal(again['row'], wire['row'], equal_nan=True)
    state = tire2.states['front']
    assert np.array_equal(state.carcass_state.delta_prev_m,
                          tire.states['front'].carcass_state.delta_prev_m)
    assert state.carcass_state.initialized
    assert set(state._patch_states) == set(tire.states['front']._patch_states)
    with pytest.raises(ValueError, match='rear.patch'):
        tire2.set_state(wire['names'][:-1], wire['row'][:-1])
    with pytest.raises(ValueError, match='front.ray.000.bogus'):
        tire2.set_state(['front.ray.000.bogus'], [0.])
    with pytest.raises(ValueError, match='cells for'):
        tire2.set_state(wire['names'], wire['row'][:-1])


def test_detailed_tier_bristle_wire():
    model, data, tire = _wheel_rig(tier='detailed')
    dt = float(model.opt.timestep)
    tire.compute_qfrc(model, data, dt)
    names = tire.state_names()
    assert len(names) == 2 * (4 * TIERS['detailed'].n_rays + 8
                            + 3 * ((TIERS['detailed'].n_rays + 1) // 2))
    wire = tire.state_dict()
    bristle = np.array([wire['row'][names.index(f'front.ray.{i:03d}.bristle_m')]
                        for i in range(TIERS['detailed'].n_rays)])
    finite = np.isfinite(bristle)
    assert finite.any() and not finite.all()
    for (start, stop), patch in tire.states['front']._patch_states.items():
        np.testing.assert_array_equal(
            bristle[start:stop], patch.detailed.bristle_deflection_m)
    # Round trip rebuilds DiscretisedBrush objects with the same bristle cells.
    model2, data2, tire2 = _wheel_rig(tier='detailed')
    tire2.set_state(wire['names'], wire['row'])
    assert np.array_equal(tire2.state_dict()['row'], wire['row'], equal_nan=True)
    for (start, stop), patch in tire.states['front']._patch_states.items():
        rebuilt = tire2.states['front']._patch_states[(start, stop)]
        np.testing.assert_array_equal(rebuilt.detailed.bristle_deflection_m,
                                      patch.detailed.bristle_deflection_m)
        assert rebuilt.detailed.kappa_prime == patch.detailed.kappa_prime
    # Lumped tier writes all-NaN bristle cells.
    model_f, data_f, tire_f = _wheel_rig()
    tire_f.compute_qfrc(model_f, data_f, float(model_f.opt.timestep))
    names_f = tire_f.state_names()
    row_f = tire_f.state_dict()['row']
    assert np.isnan([row_f[names_f.index(f'front.ray.{i:03d}.bristle_m')]
                     for i in range(64)]).all()


def test_rim_event_wire_round_trip():
    """A deep press drives rays past rim_strike_deflection_mm and fills rim cells."""
    model, data, tire = _wheel_rig(front_drop_m=0.055)
    tire.compute_qfrc(model, data, float(model.opt.timestep))
    names = tire.state_names()
    wire = tire.state_dict()
    assert tire.diagnostics['front']['rim_strike_active'] is True
    assert wire['row'][names.index('front.rim.active')] == 1.0
    for leaf in ('x_m', 'speed_mps', 'peak_load_n', 'peak_force_n', 'energy_j'):
        assert np.isfinite(wire['row'][names.index(f'front.rim.{leaf}')])
    model2, data2, tire2 = _wheel_rig()
    tire2.set_state(wire['names'], wire['row'])
    assert np.array_equal(tire2.state_dict()['row'], wire['row'], equal_nan=True)
    rim = tire2.states['front'].carcass_state
    assert rim.rim_strike_active
    assert rim.rim_event_x_m == tire.states['front'].carcass_state.rim_event_x_m


def test_probe_leaves_state_and_clock_untouched():
    model, data, tire = _wheel_rig()
    dt = float(model.opt.timestep)
    tire.compute_qfrc(model, data, dt)
    before = tire.state_dict()['row'].copy()
    states_id = id(tire.states['front'])
    clock = tire.last_time_s
    force = tire.compute_qfrc(model, data, dt, advance=False)
    assert np.isfinite(force).all()
    assert np.array_equal(tire.state_dict()['row'], before, equal_nan=True)
    assert tire.last_time_s == clock and tire.probe_snapshots
    assert id(tire.states['front']) == states_id
    assert set(tire.probe_snapshots) == {'front', 'rear'}
    with pytest.raises(ValueError, match='once per increasing timestamp'):
        tire.compute_qfrc(model, data, dt)


def test_interval_gate_and_restart_clock():
    model, data, tire = _wheel_rig()
    dt = float(model.opt.timestep)
    tire.compute_qfrc(model, data, dt)
    with pytest.raises(ValueError, match='once per increasing timestamp'):
        tire.compute_qfrc(model, data, dt)
    data.time += dt
    tire.compute_qfrc(model, data, dt)
    with pytest.raises(ValueError, match='once per increasing timestamp'):
        tire.compute_qfrc(model, data, dt)
    tire.restart_clock()
    tire.compute_qfrc(model, data, dt)


def test_reset_clears_history_and_counters():
    model, data, tire = _wheel_rig()
    dt = float(model.opt.timestep)
    tire.compute_qfrc(model, data, dt)
    assert tire.snapshots and tire.states['front'].carcass_state.initialized
    tire.reset()
    assert tire.snapshots == {} and tire.diagnostics == {}
    assert tire.last_time_s is None
    assert tire.elastic_energy_j == tire.brush_loss_step_j == 0.
    assert tire.radial_dissipation_power_w == 0.
    for tyre in tire.states.values():
        assert not tyre.carcass_state.initialized
        assert not np.any(tyre.carcass_state.delta_prev_m)
        assert tyre._patch_states == {}
        assert tyre.last_outputs.patches == ()
    tire.compute_qfrc(model, data, dt)
    assert tire.snapshots['front'].normal_load_n > 0.


def test_stored_energy_matches_committed_counter():
    model, data, tire = _wheel_rig()
    dt = float(model.opt.timestep)
    # The front wheel starts pre-loaded: the pose alone already stores energy.
    assert tire.stored_energy(model, data) > 0.
    tire.compute_qfrc(model, data, dt)
    assert tire.stored_energy(model, data) == tire.elastic_energy_j > 0.
    assert tire.brush_loss_step_j >= 0.
    data.time += dt
    tire.compute_qfrc(model, data, dt)
    assert tire.stored_energy(model, data) == tire.elastic_energy_j


def test_failure_atomicity_restores_front_state():
    model, data, tire = _wheel_rig(rear_z_m=REAR_R - 0.004)
    dt = float(model.opt.timestep)
    # A corrupted carcass state on the rear wheel raises inside evaluate_carcass
    # after the front wheel has already advanced in place.
    tire.states['rear'].carcass_state.n_rays += 1
    with pytest.raises(ValueError, match='rays'):
        tire.compute_qfrc(model, data, dt)
    front = tire.states['front']
    assert not front.carcass_state.initialized
    assert not np.any(front.carcass_state.delta_prev_m)
    assert front._patch_states == {}
    assert front.last_outputs.patches == ()
    assert tire.last_time_s is None and tire.snapshots == {}
    # The corrupted object is discarded wholesale; the next valid call runs clean.
    tire.states['rear'].carcass_state.reset()
    tire.states['rear'].carcass_state.n_rays = tire.states['rear'].ring.n
    tire.compute_qfrc(model, data, dt)
    assert tire.snapshots['rear'].normal_load_n > 0.


def test_oracle_parity_with_xfrc_writer():
    """The xfrc writer and this applier step the same PneumaticTyre, bit-for-bit."""
    from bike_sim.physics.tyre import TyreConfig
    from bike_sim.sim.ride.tyre.applier import TyreForceApplier
    model, data, tire = _wheel_rig()
    model_l, data_l, _ = _wheel_rig()
    x = np.linspace(-2.0, 3.0, 1001)
    legacy = TyreForceApplier(
        model_l, TyreConfig(model='pneumatic', tier='fast', surface='asphalt'),
        RoadProfile.from_samples(x, np.zeros_like(x)), SurfaceMap.uniform('asphalt'))
    dt = float(model.opt.timestep)
    for m, d in ((model, data), (model_l, data_l)):
        spin = m.jnt_dofadr[
            mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'front_wheel_spin')]
        d.qvel[spin] = 2.0
        mujoco.mj_forward(m, d)
    for _ in range(4):
        legacy.apply(model_l, data_l)
        qfrc = tire.compute_qfrc(model, data, dt)
        expected = np.zeros(model.nv)
        for side in ('front', 'rear'):
            a, b = tire.states[side], getattr(legacy, f'{side}_tyre')
            for leaf in ('delta_prev_m', 'elastic_prev_n', 'maxwell_force_n'):
                np.testing.assert_array_equal(getattr(a.carcass_state, leaf),
                                              getattr(b.carcass_state, leaf))
            assert a.carcass_state.initialized == b.carcass_state.initialized
            assert set(a._patch_states) == set(b._patch_states)
            for key, state in a._patch_states.items():
                assert state.kappa_prime == b._patch_states[key].kappa_prime
            for pa, pb in zip(a.last_outputs.patches, b.last_outputs.patches):
                np.testing.assert_array_equal(pa.force_world_n, pb.force_world_n)
                assert pa.slip_speed_mps == pb.slip_speed_mps
                jp, _ = point_jacobian(model, data, tire.bodies[side],
                                       pa.centroid_world_m)
                expected += jp.T @ pa.force_world_n
        assert np.array_equal(qfrc, expected)
        data.time += dt
        data_l.time += dt


def test_determinism_across_identical_rigs():
    a = _wheel_rig()
    b = _wheel_rig()
    dt = float(a[0].opt.timestep)
    for model, data, tire in (a, b):
        for _ in range(3):
            data.qfrc_applied[:] = tire.compute_qfrc(model, data, dt)
            mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)
    fa = a[2].compute_qfrc(a[0], a[1], dt, advance=False)
    fb = b[2].compute_qfrc(b[0], b[1], dt, advance=False)
    assert np.array_equal(fa, fb)
    assert np.array_equal(a[2].state_dict()['row'], b[2].state_dict()['row'],
                          equal_nan=True)
