"""Owning native drivetrain: unchanged per-call and genuine solved oracles."""
import copy
import os
from dataclasses import asdict, replace
from pathlib import Path
import sys

import mujoco
import numpy as np
import pytest

from _bits import assert_bitwise_equal
from bike_sim.physics.chain import chain_geometry, chain_center_gradient
from bike_sim.physics.physical_config import PhysicalDriveConfig, PedalingConfig, ShiftingConfig, BatteryConfig
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.ride.drivetrain_forces import DrivetrainForceApplier
from bike_sim.sim.ride.ideal_freehub import IdealFreehubConstraint
from bike_sim.sim.ride.geometric_freehub import GeometricFreehubConstraint
from test_native_drive_policies import assert_tree, oracle_config, KEYS

from native_loader import load_native
bike_native = load_native()


def model_xml(kind, mode='crank_effort', topology='plain', sparse=False):
    names = ['root_x', 'frame_pitch', 'crank_spin', 'rear_carrier', 'rear_wheel_spin', 'front_wheel_spin', 'pedal_front_spin', 'pedal_rear_spin']
    extra = ''
    if kind == 'elastic_chain':
        extra += '<body name="cassette" pos="-.5 0 0"><joint name="cassette_spin" axis="0 1 0"/><geom size=".04" mass="1"/></body>'
        names.append('cassette_spin')
    if topology == 'clutch':
        extra += '<body name="drive_shaft"><joint name="drive_shaft_spin" axis="0 1 0"/><geom size=".04" mass="1"/></body>'
        names.append('drive_shaft_spin')
    if topology == 'rotor':
        extra += '<body name="rotor"><joint name="rotor_spin" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia=".2 .2 .2"/></body>'
        names.append('rotor_spin')
    driver = 'drive_shaft_spin' if topology == 'clutch' else 'crank_spin'
    tendon = ''
    if kind == 'ideal_mid_drive':
        tendon = f'<fixed name="ideal_mid_drive_freehub" limited="true" range="-100 0"><joint joint="{driver}" coef="1.4166666666666667"/><joint joint="rear_wheel_spin" coef="-1"/></fixed>'
    elif kind == 'geometric_ideal_mid_drive':
        tendon = '<fixed name="geometric_mid_drive_freehub" limited="true" range="-100 0">' + ''.join(f'<joint joint="{n}" coef=".1"/>' for n in names) + '</fixed>'
    if topology == 'clutch':
        tendon += '<fixed name="crank_clutch" limited="true" range="-100 0"><joint joint="crank_spin" coef="1"/><joint joint="drive_shaft_spin" coef="-1"/></fixed>'
    if topology == 'rotor':
        tendon += '<fixed name="motor_freewheel" limited="true" range="-100 0"><joint joint="rotor_spin" coef="1"/><joint joint="crank_spin" coef="-1"/></fixed>'
    # A real unrelated joint limit row exercises tendon-row selection.
    human = '<motor name="human_crank" joint="crank_spin"/>' if mode == 'crank_effort' else ''
    motor_joint = 'rotor_spin' if topology == 'rotor' else driver
    return f'''<mujoco><option timestep=".0002" gravity="0 0 0" jacobian="{'sparse' if sparse else 'dense'}"/>
    <default><geom type="sphere" size=".05" mass="1" contype="0" conaffinity="0"/><joint damping="0"/></default>
    <worldbody><body name="frame"><joint name="root_x" type="slide" axis="1 0 0"/><joint name="frame_pitch" axis="0 1 0"/><geom/>
      <body name="crank"><joint name="crank_spin" axis="0 1 0"/><geom/></body>
      <body name="rear_wheel" pos="-.5 0 .1"><joint name="rear_carrier" type="slide" axis="0 0 1" limited="true" range="-.01 .01"/><joint name="rear_wheel_spin" axis="0 1 0"/><geom/></body>
      <body name="front_wheel" pos=".6 0 0"><joint name="front_wheel_spin" axis="0 1 0"/><geom/></body>
      <body name="pedal_front"><joint name="pedal_front_spin" axis="0 1 0"/><geom/></body>
      <body name="pedal_rear"><joint name="pedal_rear_spin" axis="0 1 0"/><geom/></body>{extra}
    </body></worldbody><tendon>{tendon}</tendon><actuator>{human}<motor name="mid_drive" joint="{motor_joint}"/></actuator></mujoco>'''


def pair(tmp_path, kind='ideal_mid_drive', mode='crank_effort', topology='plain', sparse=False, **overrides):
    cfg = replace(PhysicalDriveConfig(transmission_model=kind, human_torque_nm=20.,
        motor_clutch=topology == 'clutch', rotor_inertia_kgm2=.2 if topology == 'rotor' else 0.), **overrides)
    m = mujoco.MjModel.from_xml_string(model_xml(kind, mode, topology, sparse))
    d = mujoco.MjData(m)
    d.qpos[m.joint('rear_carrier').qposadr[0]] = .013
    d.qvel[m.joint('crank_spin').dofadr[0]] = 4.
    d.qvel[m.joint('rear_wheel_spin').dofadr[0]] = 5.
    if topology != 'plain':
        d.qvel[m.joint('rotor_spin' if topology == 'rotor' else 'drive_shaft_spin').dofadr[0]] = 4.5
    mujoco.mj_forward(m, d)
    drive = DrivetrainForceApplier(m, cfg, mode)
    # Independent resolved projection permits clean missing-API RED.
    config = {**oracle_config(drive), 'drive_mode':mode, 'transmission_model':kind,
        **{k: float(getattr(cfg,k)) for k in ('human_torque_nm','torque_ripple','crank_phase_rad','chain_k_n_m','chain_c_ns_m','bearing_c_nms_rad','rotor_inertia_kgm2')},
        'motor_clutch':cfg.motor_clutch}
    path = tmp_path / 'drive.mjb'
    mujoco.mj_saveModel(m, str(path), None)
    native = bike_native.Stepper(str(path), {'schema':1, 'drive':config})
    native.set_state(d.qpos, d.qvel, d.act, d.qacc_warmstart, d.time)
    native.forward()
    drive.reset(m, d)
    native.drive_reset()
    return m,d,drive,native


@pytest.mark.parametrize('seed', range(12))
def test_chain_core(seed):
    rng = np.random.default_rng(seed)
    cf = rng.normal(size=2); cr = cf + [-.5,.1]; up = rng.normal(size=2)
    rf,rr,ref = .068,.048,float(rng.uniform(-50,50))
    expected = chain_geometry(cf,cr,rf,rr,up_xz=up,psi_reference=ref)
    assert_tree(bike_native.chain_geometry(cf,cr,rf,rr,up,ref), expected)
    assert_bitwise_equal(bike_native.chain_center_gradient(cf,cr,rf,rr,up,ref),chain_center_gradient(cf,cr,rf,rr,up_xz=up,psi_reference=ref))


@pytest.mark.parametrize('kind', ['elastic_chain','ideal_mid_drive','geometric_ideal_mid_drive'])
@pytest.mark.parametrize('mode', ['crank_effort','articulated_effort','coast','ideal_speed_control'])
def test_force_staging(tmp_path, kind, mode):
    m,d,p,n = pair(tmp_path,kind,mode)
    expected = p.compute_components(m,d,.002,speed_mps=2., control=RideControl())
    assert_tree(n.drive_components({},.002,2.), expected)
    assert_bitwise_equal(n.ctrl,d.ctrl)
    assert_tree(n.drive_diagnostics(),p.last)
    assert_tree(n.drive_stored_energy(),p.stored_energy(m,d))


@pytest.mark.parametrize('kind,topology', [('elastic_chain','plain')] + [(k,t) for k in ('ideal_mid_drive','geometric_ideal_mid_drive') for t in ('plain','clutch','rotor')])
@pytest.mark.parametrize('sparse', [False,True])
@pytest.mark.parametrize('mode', ['crank_effort','articulated_effort'])
def test_solved_intervals(tmp_path,kind,topology,sparse,mode):
    m,d,p,n = pair(tmp_path,kind,mode,topology=topology,sparse=sparse)
    for _ in range(10):
        expected = p.compute_components(m,d,.0002,speed_mps=2.,sensed_human_nm=20.)
        actual = n.drive_components({},.0002,2.,sensed_human_torque_nm=20.)
        assert_tree(actual,expected)
        assert_tree(n.drive_diagnostics(),p.last)
        d.qfrc_applied[:] = sum(expected.values())
        n.set_inputs(d.ctrl, sum(actual.values()))
        mujoco.mj_step(m,d); n.step()
        assert_bitwise_equal(n.drive_settle_actuation(),p.settle_actuation(m,d))
        assert_tree(n.drive_diagnostics(),p.last)
        assert_bitwise_equal(n.qpos,d.qpos)
        assert_bitwise_equal(n.qvel,d.qvel)
        assert_tree(n.drive_state(), python_state(p,m))


def test_atomic_state_inputs_and_probes(tmp_path):
    m,d,p,n = pair(tmp_path,'geometric_ideal_mid_drive')
    before = n.drive_state()
    expected = p.compute_components(m,d,.002,speed_mps=2.,advance=False)
    assert_tree(n.drive_components({},.002,2.,advance=False),expected)
    assert_tree(n.drive_diagnostics(probe=True),p.probe_last)
    assert_tree(n.drive_state()['policies'],before['policies'])
    state = n.drive_state(); n.set_drive_state(copy.deepcopy(state)); assert_tree(n.drive_state(),state)
    broken = copy.deepcopy(state); broken['policies']['battery']['energy_j'] = float('nan')
    with pytest.raises(ValueError): n.set_drive_state(broken)
    assert_tree(n.drive_state(),state)
    ctrl = n.ctrl.copy()
    with pytest.raises(ValueError): n.set_inputs(np.ones(len(ctrl)),np.full(len(n.qvel),np.inf))
    assert_bitwise_equal(n.ctrl,ctrl)


def test_projection(tmp_path):
    from tools.native_config import project_drive
    _,_,p,n = pair(tmp_path)
    projected = project_drive(p)
    assert projected['drive_mode'] == p.drive_mode
    assert projected['gearing']['rear_teeth'] == p.config.gearing.rear_teeth
    assert_tree(projected['assist'],oracle_config(p)['assist'])


def test_no_drive(tmp_path):
    m = mujoco.MjModel.from_xml_string(model_xml('ideal_mid_drive'))
    path = tmp_path / 'bare.mjb'; mujoco.mj_saveModel(m,str(path),None)
    n = bike_native.Stepper(str(path))
    with pytest.raises(RuntimeError,match='drive config'): n.drive_reset()
    n.set_inputs(np.zeros(m.nu),np.zeros(m.nv)); n.step()


def python_state(p, model):
    """Every mutable oracle field; engine-address objects never cross the bridge."""
    policies = {}
    for name, keys in KEYS.items():
        obj = getattr(p,name)
        policies[name] = None if obj is None else {k:vars(obj)[k] for k in keys}
    out = {k:getattr(p,k,None) for k in ('shift_time_s','last_time_s','reference','psi','angles','last','probe_last')}
    pending = p.pending_actuation
    out['pending_actuation'] = None if pending is None else dict(zip(('requested','omega','dt','enabled'),pending))
    out['policies'] = policies
    for name in ('ideal_hub','clutch','freewheel'):
        t = getattr(p,name)
        if t is None:
            out[name] = None; continue
        geometric = isinstance(t,GeometricFreehubConstraint)
        indices = [t.coefficients[i] for i in range(model.nv)] if geometric else [t.driver_coefficient]
        state = {'ratio':t.ratio, 'rear_teeth':t.gearing.rear_teeth if geometric else (p.config.gearing.rear_teeth if name == 'ideal_hub' else 3),
            'boundary':t.boundary, 'prepared':None, 'diagnostics':getattr(t,'diagnostics',{}),
            'shift_pending':getattr(t,'shift_pending',False),
            'shift_parameter_work_j':getattr(t,'shift_parameter_work_j',0.),
            'shift_constraint_work_j':getattr(t,'shift_constraint_work_j',0.),
            'last_tension_n':getattr(t,'last_tension_n',0.),
            'range':model.tendon_range[t.tendon_id].copy(), 'coefficients':model.wrap_prm[indices].copy()}
        if geometric and t.prepared is not None:
            phi,j,q,time = t.prepared
            state['prepared'] = {'phi':phi,'jacobian':j,'qpos':q,'time':time}
        out[name] = state
    return out


@pytest.mark.parametrize('kind', ['ideal_mid_drive','geometric_ideal_mid_drive'])
@pytest.mark.parametrize('operation', ['reset','prepare','ratio'])
def test_transmission_core(tmp_path,kind,operation):
    m = mujoco.MjModel.from_xml_string(model_xml(kind))
    d = mujoco.MjData(m)
    d.qpos[:] = [.12, .15, 40., .006, 46., -.2, .1, .2]
    mujoco.mj_forward(m,d)
    t = GeometricFreehubConstraint(m,PhysicalDriveConfig().gearing) if kind.startswith('geometric') else IdealFreehubConstraint(m,34./24.)
    path = tmp_path / 'core.mjb';mujoco.mj_saveModel(m,str(path),None)
    n = bike_native.Stepper(str(path));n.set_state(d.qpos,d.qvel,d.act,d.qacc_warmstart,.3);n.forward();d.time=.3
    if operation == 'ratio':
        t.reset(m,d);t.set_ratio(m,d,34./28.)
    else:
        getattr(t,operation)(m,d)
    actual = n._drive_core(kind,operation,34./28.)
    assert_bitwise_equal(actual['boundary'],t.boundary)
    assert_bitwise_equal(actual['ratio'],t.ratio)
    assert_bitwise_equal(actual['range'],m.tendon_range[t.tendon_id])
    indices = [t.coefficients[i] for i in range(m.nv)] if kind.startswith('geometric') else [t.driver_coefficient]
    assert_bitwise_equal(actual['coefficients'],m.wrap_prm[indices])
    if kind.startswith('geometric'):
        phi,j,q,time = t.prepared
        assert_bitwise_equal(actual['prepared']['phi'],phi);assert_bitwise_equal(actual['prepared']['jacobian'],j)
        assert_bitwise_equal(actual['prepared']['qpos'],q);assert_bitwise_equal(actual['prepared']['time'],time)


@pytest.mark.parametrize('kind',['elastic_chain','ideal_mid_drive','geometric_ideal_mid_drive'])
@pytest.mark.parametrize('seed',range(8))
def test_random_multiturn_staging(tmp_path,kind,seed):
    m,d,p,n = pair(tmp_path,kind)
    rng = np.random.default_rng(seed)
    for _ in range(12):
        d.qpos[:] = rng.uniform(-.006,.006,m.nq)
        d.qpos[m.joint('crank_spin').qposadr[0]] = float(rng.uniform(-90,90))
        d.qpos[m.joint('rear_wheel_spin').qposadr[0]] = float(rng.uniform(-90,90))
        if kind == 'elastic_chain':
            d.qpos[m.joint('cassette_spin').qposadr[0]] = float(rng.uniform(-90,90))
        d.qvel[:] = rng.uniform(-7,7,m.nv);d.time += .01
        mujoco.mj_forward(m,d)
        n.set_state(d.qpos,d.qvel,d.act,d.qacc_warmstart,d.time);n.forward()
        p.pending_actuation = None
        ns=n.drive_state();ns['pending_actuation']=None;n.set_drive_state(ns)
        expected = p.compute_components(m,d,.01,speed_mps=-1.,active=True)
        actual = n.drive_components({},.01,-1.)
        assert_tree(actual,expected);assert_tree(n.drive_diagnostics(),p.last)
        assert_tree(n.drive_stored_energy(),p.stored_energy(m,d));assert_tree(n.drive_state(),python_state(p,m))


@pytest.mark.parametrize('kind',['ideal_mid_drive','geometric_ideal_mid_drive'])
@pytest.mark.parametrize('contact,slip',[(True,None),(False,None),(True,5.),(True,-5.)])
def test_shifting_prepared_state_and_clock(tmp_path,kind,contact,slip):
    shift = ShiftingConfig(enabled=True,cadence_smoothing_tau_s=0.,shift_cooldown_s=.3,shift_cut_duration_s=.1)
    m,d,p,n = pair(tmp_path,kind,shifting=shift,pedaling=PedalingConfig(enabled=True))
    d.qvel[m.joint('crank_spin').dofadr[0]] = 15.
    d.qvel[m.joint('rear_wheel_spin').dofadr[0]] = 25.
    n.set_state(d.qpos,d.qvel,d.act,d.qacc_warmstart,d.time);n.forward();mujoco.mj_forward(m,d)
    # Both preparations leave live engine state intact despite mj_setConst.
    for _ in range(4):
        ps = p.prepare_pedaling(d,.15,RideControl(),model=m,rear_in_contact=contact,rear_slip_mps=slip,effort_ceiling_nm=7.)
        ns = n.drive_prepare_pedaling({},.15,rear_in_contact=contact,rear_slip_mps=slip,effort_ceiling_nm=7.)
        assert_tree(ns,asdict(ps))
        assert_tree(n.drive_state(),python_state(p,m))
        assert_bitwise_equal(n.qpos,d.qpos);assert_bitwise_equal(n.qvel,d.qvel)
        assert_tree(n.drive_components({},.15,2.,pedaling_state=ns),p.compute_components(m,d,.15,speed_mps=2.,pedaling_state=ps))
        assert_tree(n.drive_state(),python_state(p,m))
        with pytest.raises(ValueError,match='once per timestamp'): n.drive_components({},.15,2.)
        with pytest.raises(ValueError,match='once per timestamp'): p.compute_components(m,d,.15,speed_mps=2.)
        # Clear only pending to advance the force oracle without an invented solve.
        p.pending_actuation=None;s=n.drive_state();s['pending_actuation']=None;n.set_drive_state(s)
        d.time += .15;n.set_state(d.qpos,d.qvel,d.act,d.qacc_warmstart,d.time);n.forward()
    old=n.drive_state();p.restart_clock();n.drive_restart_clock()
    assert_tree(n.drive_state(),python_state(p,m));assert_tree(n.drive_state()['policies']['battery'],old['policies']['battery'])
    p.reset(m,d);n.drive_reset();assert_tree(n.drive_state(),python_state(p,m))


@pytest.mark.parametrize('kind',['elastic_chain','ideal_mid_drive','geometric_ideal_mid_drive'])
@pytest.mark.parametrize('enabled,energy',[(True,0.),(True,.00001),(True,1.),(False,0.)])
def test_energy_ceiling_and_actual_force(tmp_path,kind,enabled,energy):
    b=BatteryConfig(enabled=enabled,energy_j=energy)
    m,d,p,n=pair(tmp_path,kind,battery=b)
    control=RideControl(human_torque_nm=50.,motor_torque_nm=40.,motor_limit_nm=8.)
    for i in range(10):
        actual=n.drive_components(asdict_control(control),.0002,2.)
        expected=p.compute_components(m,d,.0002,speed_mps=2.,control=control)
        assert_tree(actual,expected);assert_tree(n.drive_diagnostics(),p.last)
        # Finite actual actuator effort may be below reserved request.
        d.ctrl[p.actuators['mid_drive']] *= .5
        d.qfrc_applied[:]=sum(expected.values());n.set_inputs(d.ctrl,d.qfrc_applied)
        mujoco.mj_step(m,d);n.step()
        assert_bitwise_equal(n.actuator_force,d.actuator_force)
        assert_bitwise_equal(n.drive_settle_actuation(),p.settle_actuation(m,d))
        assert_tree(n.drive_state(),python_state(p,m))


def asdict_control(control):
    return {k:getattr(control,k) for k in ('motor_torque_nm','motor_limit_nm','human_torque_nm','rider_enabled')}


@pytest.mark.parametrize('kind',['elastic_chain','ideal_mid_drive','geometric_ideal_mid_drive'])
@pytest.mark.parametrize('active,braking',[(True,False),(False,False),(True,True)])
def test_probe_does_not_touch_ratchets_or_live_state(tmp_path,kind,active,braking):
    m,d,p,n=pair(tmp_path,kind)
    before=n.drive_state()
    for phase in [0.,40.,-40.]:
        d.qpos[m.joint('crank_spin').qposadr[0]]=phase
        mujoco.mj_forward(m,d);n.set_state(d.qpos,d.qvel,d.act,d.qacc_warmstart,d.time);n.forward()
        expected=p.compute_components(m,d,.002,speed_mps=2.,active=active,braking=braking,advance=False)
        assert_tree(n.drive_components({},.002,2.,active=active,braking=braking,advance=False),expected)
        assert_tree(n.drive_diagnostics(probe=True),p.probe_last)
        assert_bitwise_equal(n.ctrl,d.ctrl)
        after=n.drive_state();saved=copy.deepcopy(after);saved['probe_last']=before['probe_last']
        assert_tree(saved,before)
        assert_tree(after,python_state(p,m))


@pytest.mark.parametrize('key,value',[('drive_mode','unknown'),('transmission_model','unknown'),('torque_ripple',1.),('motor_clutch',1),('chain_k_n_m',0.),('rotor_inertia_kgm2',float('inf'))])
def test_invalid_config(tmp_path,key,value):
    from tools.native_config import project_drive
    m,d,p,n=pair(tmp_path)
    config=project_drive(p);config[key]=value
    path=tmp_path/'bad.mjb';mujoco.mj_saveModel(m,str(path),None)
    with pytest.raises(ValueError):bike_native.Stepper(str(path),{'schema':1,'drive':config})


@pytest.mark.parametrize('field,value',[('dt',0.),('dt',float('nan')),('speed_mps',float('inf')),('braking',1),('active','true'),('advance',1),('sensed_human_torque_nm',float('nan')),('control',{'rider_enabled':1}),('control',{'motor_limit_nm':-1.})])
def test_invalid_calls_are_atomic(tmp_path,field,value):
    _,_,_,n=pair(tmp_path)
    before=n.drive_state();ctrl=n.ctrl.copy();args={'control':{},'dt':.002,'speed_mps':2.};args[field]=value
    with pytest.raises((ValueError,TypeError)):n.drive_components(**args)
    assert_tree(n.drive_state(),before);assert_bitwise_equal(n.ctrl,ctrl)


@pytest.mark.parametrize('section,key,value',[
    ('root','angles',[0.,1.]),('root','reference',float('nan')),('root','last_time_s',float('inf')),
    ('ideal_hub','ratio',0.),('ideal_hub','shift_pending',1),('ideal_hub','coefficients',[0.]),
    ('shifting','shift_count',2**40),('pedaling','coasting',1),('shifting','direction','bad'),
    ('battery','energy_j',-1.),('assist','torque',float('nan'))])
def test_invalid_restoration_is_atomic(tmp_path,section,key,value):
    _,_,_,n=pair(tmp_path,'geometric_ideal_mid_drive')
    before=n.drive_state();bad=copy.deepcopy(before)
    target=bad if section=='root' else bad[section] if section=='ideal_hub' else bad['policies'][section]
    target[key]=value
    with pytest.raises(ValueError):n.set_drive_state(bad)
    assert_tree(n.drive_state(),before)


def test_set_inputs_cross_aliases_and_ownership(tmp_path):
    _,_,_,n=pair(tmp_path)
    initial=np.arange(len(n.qvel),dtype=float)+1
    n.set_inputs(np.array([17.,18.]),initial)
    ctrl=n.ctrl.copy();force=n.qfrc_applied.copy()
    # First input aliases force, and second includes ctrl via a full native view.
    n.set_inputs(n.qfrc_applied[:len(ctrl)],n.qfrc_applied)
    assert_bitwise_equal(n.ctrl,force[:len(ctrl)]);assert_bitwise_equal(n.qfrc_applied,force)
    for c,f in [(np.zeros(1),force),(ctrl,np.zeros(1)),(ctrl,np.full(len(force),np.nan))]:
        old=n.ctrl.copy();oldf=n.qfrc_applied.copy()
        with pytest.raises(ValueError):n.set_inputs(c,f)
        assert_bitwise_equal(n.ctrl,old);assert_bitwise_equal(n.qfrc_applied,oldf)
    components=n.drive_components({},.002,2.)
    saved={k:v.copy() for k,v in components.items()};del n
    assert_tree(components,saved)


@pytest.mark.parametrize('kind',['elastic_chain','ideal_mid_drive','geometric_ideal_mid_drive'])
def test_injected_valid_state_and_replay(tmp_path,kind):
    m,d,p,n=pair(tmp_path,kind)
    p.pedaling._effort=13.;p.pedaling._cadence_ema=-3.
    p.shifting.cooldown_s=.07;p.shifting.cut_remaining_s=.03;p.shifting.shift_count=4;p.shifting.direction='up';p.shifting.from_teeth=28
    p.assist.torque=2.;p.assist.last_gain=.4;p.battery.energy_j=17.;p.battery.drawn_energy_j=3.
    p.shift_time_s=-1.;p.last_time_s=-.5;p.angles=tuple(x+2*np.pi for x in p.angles)
    if p.hub:
        p.hub.boundary=-.03;p.hub.energy_j=.04;p.hub.torque_nm=2.
        p.reference += .001
    if p.ideal_hub:
        p.ideal_hub.boundary += .02
        if isinstance(p.ideal_hub,GeometricFreehubConstraint):
            p.ideal_hub.last_tension_n=2.;p.ideal_hub.shift_pending=True
            p.ideal_hub.shift_parameter_work_j=-.1;p.ideal_hub.shift_constraint_work_j=.2
    injected=python_state(p,m)
    n.set_drive_state(injected)
    assert_tree(n.drive_state(),injected)
    # Owning input and output snapshots remain detached after modification.
    injected['policies']['battery']['energy_j']=999.
    assert n.drive_state()['policies']['battery']['energy_j']==17.
    for _ in range(8):
        a=n.drive_components({},.0002,2.);e=p.compute_components(m,d,.0002,speed_mps=2.)
        assert_tree(a,e);assert_tree(n.drive_state(),python_state(p,m))
        d.qfrc_applied[:]=sum(e.values());n.set_inputs(d.ctrl,d.qfrc_applied)
        mujoco.mj_step(m,d);n.step()
        assert_bitwise_equal(n.drive_settle_actuation(),p.settle_actuation(m,d))
        assert_tree(n.drive_state(),python_state(p,m))


@pytest.mark.parametrize('kind',['ideal_mid_drive','geometric_ideal_mid_drive'])
def test_pending_and_duplicate_settlement_guards(tmp_path,kind):
    m,d,p,n=pair(tmp_path,kind)
    p.compute_components(m,d,.0002,speed_mps=2.);n.drive_components({},.0002,2.)
    d.time=.1;n.set_state(d.qpos,d.qvel,d.act,d.qacc_warmstart,d.time);n.forward()
    with pytest.raises(RuntimeError,match='not settled'):p.compute_components(m,d,.0002,speed_mps=2.)
    with pytest.raises(RuntimeError,match='not settled'):n.drive_components({},.0002,2.)
    assert_tree(n.drive_state(),python_state(p,m))
    n.set_inputs(d.ctrl,np.zeros(m.nv));mujoco.mj_step(m,d);n.step()
    assert_bitwise_equal(n.drive_settle_actuation(),p.settle_actuation(m,d));first=n.drive_state()
    assert_bitwise_equal(n.drive_settle_actuation(),p.settle_actuation(m,d))
    assert_tree(n.drive_state(),python_state(p,m));assert_tree(n.drive_state()['policies']['battery'],first['policies']['battery'])


@pytest.mark.parametrize('mode',['coast','ideal_speed_control'])
@pytest.mark.parametrize('kind',['elastic_chain','ideal_mid_drive','geometric_ideal_mid_drive'])
def test_passive_topology_without_actuators(tmp_path,mode,kind):
    from tools.native_config import project_drive
    xml=model_xml(kind,mode);xml=xml[:xml.index('<actuator>')]+ '</mujoco>'
    m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    p=DrivetrainForceApplier(m,PhysicalDriveConfig(transmission_model=kind),mode)
    path=tmp_path/'passive.mjb';mujoco.mj_saveModel(m,str(path),None)
    n=bike_native.Stepper(str(path),{'schema':1,'drive':project_drive(p)})
    p.reset(m,d);n.drive_reset()
    assert_tree(n.drive_components({},.002,0.),p.compute_components(m,d,.002,speed_mps=0.))
    n.set_inputs(np.empty(0),np.zeros(m.nv));mujoco.mj_step(m,d);n.step()
    assert_bitwise_equal(n.drive_settle_actuation(),p.settle_actuation(m,d));assert_tree(n.drive_state(),python_state(p,m))


@pytest.mark.parametrize('key,value',[('shift_direction','invalid'),('transmission_model','invalid'),('motor_enabled',1),('gear_rear_teeth',2**40),('nonsense',1.),('motor_control_source','invalid')])
def test_diagnostic_state_validation(tmp_path,key,value):
    _,_,_,n=pair(tmp_path)
    n.drive_components({},.002,2.)
    before=n.drive_state();bad=copy.deepcopy(before);bad['last'][key]=value
    with pytest.raises(ValueError):n.set_drive_state(bad)
    assert_tree(n.drive_state(),before)


def test_prepared_state_requires_boundary(tmp_path):
    _,_,_,n=pair(tmp_path,'geometric_ideal_mid_drive')
    before=n.drive_state();bad=copy.deepcopy(before);bad['ideal_hub']['boundary']=None
    with pytest.raises(ValueError):n.set_drive_state(bad)
    assert_tree(n.drive_state(),before)


def test_real_solved_request_violation(tmp_path):
    m,d,p,n=pair(tmp_path)
    p.compute_components(m,d,.0002,speed_mps=2.);n.drive_components({},.0002,2.)
    d.ctrl[p.actuators['mid_drive']]=1000.
    n.set_inputs(d.ctrl,np.zeros(m.nv));mujoco.mj_step(m,d);n.step()
    with pytest.raises(ArithmeticError,match='reserved effort ceiling'):p.settle_actuation(m,d)
    with pytest.raises(ArithmeticError,match='reserved effort ceiling'):n.drive_settle_actuation()


@pytest.mark.parametrize('topology',['clutch','rotor'])
def test_invalid_elastic_topology_rejects(tmp_path,topology):
    from tools.native_config import project_drive
    with pytest.raises(ValueError):PhysicalDriveConfig(transmission_model='elastic_chain',motor_clutch=topology=='clutch',rotor_inertia_kgm2=.2 if topology=='rotor' else 0.)
    m,d,p,n=pair(tmp_path)
    cfg=project_drive(p);cfg['transmission_model']='elastic_chain';cfg['motor_clutch']=topology=='clutch';cfg['rotor_inertia_kgm2']=.2 if topology=='rotor' else 0.
    path=tmp_path/'invalid.mjb';mujoco.mj_saveModel(m,str(path),None)
    with pytest.raises(ValueError):bike_native.Stepper(str(path),{'schema':1,'drive':cfg})


def test_optional_project_and_initial_gearing(tmp_path):
    from types import SimpleNamespace
    from bike_sim.sim.ride_sim import RideSimulation
    from tools.native_config import project,project_drive
    _,_,p,_=pair(tmp_path)
    sim=RideSimulation()
    original=getattr(sim,'physical',None)
    sim.physical=SimpleNamespace(drive=p)
    p.shifting.rear_teeth=28
    assert_tree(project(sim)['drive'],project_drive(p))
    assert project(sim)['drive']['gearing']['rear_teeth']==24
    sim.physical=SimpleNamespace(drive=None);assert 'drive' not in project(sim)
    sim.physical=None;assert 'drive' not in project(sim)
    del sim.physical;assert 'drive' not in project(sim)
    sim.physical=original


def test_set_inputs_both_cross_buffer_aliases(tmp_path):
    m=mujoco.MjModel.from_xml_string('<mujoco><worldbody><body><joint name="a"/><geom size=".1"/><body><joint name="b"/><geom size=".1"/></body></body></worldbody><actuator><motor joint="a"/><motor joint="b"/></actuator></mujoco>')
    path=tmp_path/'alias.mjb';mujoco.mj_saveModel(m,str(path),None)
    n=bike_native.Stepper(str(path));n.set_inputs(np.array([17.,18.]),np.array([1.,2.]))
    n.set_inputs(n.qfrc_applied,n.ctrl)
    assert_bitwise_equal(n.ctrl,[1.,2.]);assert_bitwise_equal(n.qfrc_applied,[17.,18.])


@pytest.mark.parametrize('key,value',[('gear_ratio',0.),('gear_front_teeth',2),('shift_count',-1)])
def test_diagnostic_ratio_and_integer_ranges(tmp_path,key,value):
    _,_,_,n=pair(tmp_path);before=n.drive_state();bad=copy.deepcopy(before);bad['last'][key]=value
    with pytest.raises(ValueError):n.set_drive_state(bad)
    assert_tree(n.drive_state(),before)


@pytest.mark.parametrize('kind',['elastic_chain','ideal_mid_drive','geometric_ideal_mid_drive'])
def test_genuine_wheel_and_rotor_overrun(tmp_path,kind):
    topology='plain' if kind=='elastic_chain' else 'rotor'
    m,d,p,n=pair(tmp_path,kind,topology=topology)
    d.qvel[m.joint('rear_wheel_spin').dofadr[0]]=12.
    if topology=='rotor':d.qvel[m.joint('rotor_spin').dofadr[0]]=2.
    n.set_state(d.qpos,d.qvel,d.act,d.qacc_warmstart,d.time);n.forward();mujoco.mj_forward(m,d)
    for _ in range(8):
        a=n.drive_components({},.0002,2.);e=p.compute_components(m,d,.0002,speed_mps=2.)
        assert_tree(a,e);d.qfrc_applied[:]=sum(e.values());n.set_inputs(d.ctrl,d.qfrc_applied)
        mujoco.mj_step(m,d);n.step()
        # Unrelated actual engine rows remain present while one-way hubs overrun.
        assert np.any(d.efc_type[:d.nefc]==mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT)
        assert_bitwise_equal(n.drive_settle_actuation(),p.settle_actuation(m,d))
        assert_tree(n.drive_state(),python_state(p,m))
        assert n.drive_diagnostics()['freehub_torque_nm']==0.


@pytest.mark.parametrize('operation', ['prepare', 'reset', 'shift', 'restore'])
def test_geometric_prepared_buffers_reuse_storage(tmp_path, operation):
    shifting = ShiftingConfig(enabled=True, cadence_smoothing_tau_s=0.,
        shift_cooldown_s=.3, shift_cut_duration_s=.1)
    m,d,p,n = pair(tmp_path, 'geometric_ideal_mid_drive', shifting=shifting)
    # Force an actual shift in the shift case; other cases retain the normal gear.
    if operation == 'shift':
        d.qvel[m.joint('crank_spin').dofadr[0]] = 15.
        d.qvel[m.joint('rear_wheel_spin').dofadr[0]] = 25.
        n.set_state(d.qpos,d.qvel,d.act,d.qacc_warmstart,d.time);n.forward()
        mujoco.mj_forward(m,d)
    if operation == 'restore':
        n.set_drive_state(copy.deepcopy(n.drive_state()))
    baseline = n._drive_prepared_storage()
    assert set(baseline) == {'jacobian_generation', 'qpos_generation',
                           'jacobian_capacity', 'qpos_capacity'}
    assert baseline['jacobian_generation'] and baseline['qpos_generation']
    assert baseline['jacobian_capacity'] >= m.nv
    assert baseline['qpos_capacity'] >= m.nv
    initial_teeth = p.shifting.rear_teeth
    if operation in ('prepare', 'restore'):
        # Sequential live preparations in genuine solved intervals.
        for _ in range(8):
            expected = p.compute_components(m,d,.0002,speed_mps=2.)
            assert_tree(n.drive_components({},.0002,2.),expected)
            assert n._drive_prepared_storage() == baseline
            assert_tree(n.drive_state(),python_state(p,m))
            d.qfrc_applied[:]=sum(expected.values());n.set_inputs(d.ctrl,d.qfrc_applied)
            mujoco.mj_step(m,d);n.step()
            assert_bitwise_equal(n.drive_settle_actuation(),p.settle_actuation(m,d))
    elif operation == 'reset':
        for _ in range(8):
            p.reset(m,d);n.drive_reset()
            assert n._drive_prepared_storage() == baseline
            assert_tree(n.drive_state(),python_state(p,m))
    else:
        expected=p.prepare_pedaling(d,.15,RideControl(),model=m)
        assert_tree(n.drive_prepare_pedaling({},.15),asdict(expected))
        assert p.shifting.rear_teeth != initial_teeth
        assert n._drive_prepared_storage() == baseline
        assert_tree(n.drive_state(),python_state(p,m))


def test_geometric_prepared_sentinel_owns_reused_storage(tmp_path):
    m,d,p,n = pair(tmp_path, 'geometric_ideal_mid_drive')
    baseline=n._drive_prepared_storage()
    state=n.drive_state();old_jacobian=state['ideal_hub']['prepared']['jacobian'].copy()
    # Diagnostics expose storage generations/capacities only — no raw
    # address crosses the bridge, so public snapshots cannot alias the
    # private native working buffers.
    assert set(baseline) == {'jacobian_generation', 'qpos_generation',
                           'jacobian_capacity', 'qpos_capacity'}
    assert baseline['jacobian_generation'] and baseline['qpos_generation']
    none_state=copy.deepcopy(state);none_state['ideal_hub']['prepared']=None
    n.set_drive_state(none_state);assert n.drive_state()['ideal_hub']['prepared'] is None
    assert n._drive_prepared_storage()==baseline
    with pytest.raises(RuntimeError,match='before solving'):n.drive_settle_actuation()
    n.set_drive_state(state)
    assert_tree(n.drive_state(),state);assert n._drive_prepared_storage()==baseline
    state['ideal_hub']['prepared']['jacobian'][:]=99.
    assert_bitwise_equal(n.drive_state()['ideal_hub']['prepared']['jacobian'],old_jacobian)
    p.reset(m,d);n.drive_reset()
    assert_tree(n.drive_state(),python_state(p,m));assert n._drive_prepared_storage()==baseline
