"""Finalize physical topology after the original component budgets are assigned.

Mass is transferred, never duplicated. The rear synthetic core is split into
15% hub/rotor and 10% cassette, retaining the previous combined mass, CoM and
locked-assembly inertia exactly. That split is not a measured cassette profile.
The optional motor rotor adds its declared synthetic 0.3 kg mass and reflected
inertia; it is separate from these original component budgets.
"""
import xml.etree.ElementTree as ET
import numpy as np
from bike_sim.physics.checks import scalar
from bike_sim.physics.inertia import (
    WheelComponent, REAR_WHEEL_PROFILE, wheel_body_inertia,
    add_body_inertial, ring_inertia, parallel_axis,
)
from bike_sim.physics.rider_segments import segment_inertia


def _find(root, kind, name):
    element=root.find(f".//{kind}[@name='{name}']")
    if element is None:
        raise ValueError(f'physical topology needs {name!r}')
    return element


def _set_inertia(body,mass,com,tensor):
    for old in body.findall('inertial'):
        body.remove(old)
    add_body_inertial(body,mass,com,tensor)
    for geom in body.findall('geom'):
        geom.set('mass','0')


def finish_physical_topology(root,specs,mass_specs,physics_config):
    """Called only for ride/physical, before serialization and compilation."""
    if physics_config.physics_mode!='physical':
        raise ValueError('physical topology cannot modify the legacy model')
    # Steering is fixed in this explicitly sagittal model. A stiff oblique
    # hinge is not a planar constraint and leaks small lateral velocities.
    steer = _find(root, 'body', 'steer')
    for joint in list(steer.findall('joint')):
        if joint.get('name') == 'steer_joint':
            steer.remove(joint)
    sensors = root.find('sensor')
    if sensors is not None:
        for sensor in list(sensors):
            if sensor.get('joint') == 'steer_joint':
                sensors.remove(sensor)
    # The legacy 0.5 ms solref was clamped by MuJoCo at dt=0.5 ms to
    # 2*dt=1 ms. Preserve that effective reference, explicitly and identically
    # across all refinement grids instead of changing the linkage stiffness.
    equality = root.find('equality')
    if equality is not None:
        for constraint in equality:
            constraint.set('solref', f'{physics_config.closure_time_constant_s:.17g} 1')
    crank=_find(root,'body','crank')
    crank_length=scalar(specs.crank_length/1000.,'crank length',positive=True)
    budget=scalar(mass_specs.crank_pedals_mass,'crank mass',positive=True)
    clutch=physics_config.drive.motor_clutch
    # Physical dimensions below are explicit synthetic component inputs, not
    # radii or sizes read back from decorative cylinders and pedal meshes.
    arm_y=.075
    spindle_mass=budget*.10/.85
    arm_mass=budget*.20/.85
    pedal_mass=budget*.175/.85
    shaft_mass=0.
    if clutch:
        # The chainring and motor output shaft ride on their own body ahead of
        # the crank clutch: a share of the spindle budget, not new mass.
        shaft_mass=spindle_mass*.6
        spindle_mass-=shaft_mass
    tensor=ring_inertia(spindle_mass,0.,.016,2*arm_y)
    for sign in (-1.,1.):
        tensor+=segment_inertia(arm_mass,[crank_length,0.,0.],.012)
        tensor+=parallel_axis(arm_mass,[sign*crank_length/2,-sign*arm_y,0.])
    _set_inertia(crank,spindle_mass+2*arm_mass,np.zeros(3),tensor)
    _find(root,'joint','crank_spin').set('damping','0')
    if clutch:
        # The motor-side shaft is coaxial with the crank: the crank body owns
        # arms and pedals, the shaft owns the chainring and motor output.
        frame_body=_find(root,'body','frame')
        shaft=ET.SubElement(frame_body,'body',name='drive_shaft',pos='0 0 0')
        ET.SubElement(shaft,'joint',name='drive_shaft_spin',type='hinge',
                      pos='0 0 0',axis='0 1 0',damping='0')
        ring_radius=physics_config.drive.gearing.front_radius_m
        ET.SubElement(shaft,'geom',name='geom_drive_shaft_ring',type='cylinder',
                      fromto='0 -0.015 0 0 0.015 0',size=f'{ring_radius:.17g}',
                      mass='0',material='mat_metal',contype='0',conaffinity='0')
        _set_inertia(shaft,shaft_mass,np.zeros(3),
                     ring_inertia(shaft_mass,.015,ring_radius,.03))
    rotor_inertia=physics_config.drive.rotor_inertia_kgm2
    if rotor_inertia > 0.:
        # Declared inertia is reflected to crank speed. Nominal 0.3 kg mass
        # prevents an artificial ring's mass from determining this inertia.
        frame_body=_find(root,'body','frame')
        rotor=ET.SubElement(frame_body,'body',name='motor_rotor',pos='0 0 0')
        ET.SubElement(rotor,'joint',name='rotor_spin',type='hinge',
                      pos='0 0 0',axis='0 1 0',damping='0')
        _set_inertia(rotor,.3,np.zeros(3),np.diag([rotor_inertia/2,rotor_inertia,rotor_inertia/2]))
    for side,sign in (('front',-1.),('rear',1.)):
        pedal=_find(root,'body','pedal_'+side)
        joint=_find(pedal,'joint','pedal_spin_'+side)
        joint.set('name','pedal_'+side+'_spin')
        joint.set('damping','0')
        hx,hy,hz=.05,.04,.008
        pedal_tensor=pedal_mass/3*np.diag([hy*hy+hz*hz,hx*hx+hz*hz,hx*hx+hy*hy])
        _set_inertia(pedal,pedal_mass,[0.,sign*.04,0.],pedal_tensor)
    wheel=_find(root,'body','rear_wheel')
    parent=next(p for p in root.iter('body') if wheel in list(p))
    if physics_config.drive.transmission_model == 'elastic_chain':
        cassette=ET.SubElement(parent,'body',name='cassette',pos=wheel.get('pos','0 0 0'))
        ET.SubElement(cassette,'joint',name='cassette_spin',type='hinge',axis='0 1 0',damping='0')
        geom=_find(wheel,'geom','geom_rear_cassette'); wheel.remove(geom); cassette.append(geom)
        ring,core=REAR_WHEEL_PROFILE
        # Renormalize fractions inside the remaining 90% wheel body.
        profile=(ring._replace(mass_fraction=.75/.9),core._replace(mass_fraction=.15/.9))
        wheel_mass=mass_specs.rear_wheel_mass*.9
        com,tensor=wheel_body_inertia(wheel_mass,profile)
        _set_inertia(wheel,wheel_mass,com,tensor)
        cassette_mass=mass_specs.rear_wheel_mass*.1
        tensor=ring_inertia(cassette_mass,core.inner_radius_m,core.outer_radius_m,core.width_m)
        _set_inertia(cassette,cassette_mass,core.offset_m,tensor)
    else:
        # The ideal mid-drive keeps the complete rear wheel as one body. The
        # decorative cassette remains attached to that body and has no rotor DOF.
        com,tensor=wheel_body_inertia(mass_specs.rear_wheel_mass,REAR_WHEEL_PROFILE)
        _set_inertia(wheel,mass_specs.rear_wheel_mass,com,tensor)
    for side in ('front','rear'):
        body=_find(root,'body',side+'_wheel')
        joint=_find(body,'joint',side+'_wheel_spin')
        joint.set('damping','0')
        joint.set('frictionloss','0')
        joint.set('solreffriction','0.005 1')
        joint.set('solimpfriction','0.9999 0.9999 0.001 0.5 2')
        contact_geom = _find(body, 'geom', 'geom_'+side+'_contact')
        contact_geom.set('priority', '1')
        contact_geom.set('friction', f'{getattr(physics_config.tires, side).mu:.17g} 0.005 0.0001')
        if physics_config.tires.backend in ('compliant_2d','distributed_2d_reference'):
            for geom in body.iter('geom'):
                geom.set('contype','0'); geom.set('conaffinity','0')
    contact=root.find('contact')
    if contact is None:
        contact=ET.SubElement(root,'contact')
    existing={frozenset((e.get('body1'),e.get('body2'))) for e in contact.findall('exclude')}
    names=[b.get('name') for b in root.iter('body') if b.get('name')]
    moving_bodies=('crank','pedal_front','pedal_rear')
    if physics_config.drive.transmission_model == 'elastic_chain':
        moving_bodies += ('cassette',)
    if clutch:
        moving_bodies += ('drive_shaft',)
    for moving in moving_bodies:
        for name in names:
            pair=frozenset((moving,name))
            if moving==name or name.startswith('rider_') or pair in existing:
                continue
            ET.SubElement(contact,'exclude',body1=moving,body2=name)
            existing.add(pair)
    actuators=root.find('actuator')
    if actuators is None:
        actuators=ET.SubElement(root,'actuator')
    for actuator in list(actuators):
        name=actuator.get('name')
        if name=='crank_drive' or (name=='rear_drive' and physics_config.drive_mode!='ideal_speed_control'):
            actuators.remove(actuator)
    if physics_config.drive_mode in ('crank_effort','articulated_effort'):
        ratio=(physics_config.drive.gearing.front_teeth /
               physics_config.drive.gearing.rear_teeth)
        # Real mid-drive: the chainring is on the crank spindle. The legacy
        # clutch topology instead keeps its chainring on drive_shaft.
        driver_joint='drive_shaft_spin' if clutch else 'crank_spin'
        motor_joint='rotor_spin' if rotor_inertia > 0. else driver_joint
        tendons = root.find('tendon')
        if physics_config.drive.transmission_model == 'ideal_mid_drive':
            if tendons is None:
                tendons = ET.SubElement(root, 'tendon')
            freehub = ET.SubElement(tendons, 'fixed', name='ideal_mid_drive_freehub',
                limited='true', range='-1e12 0', margin='0',
                solreflimit=f'{physics_config.closure_time_constant_s:.17g} 1')
            ET.SubElement(freehub, 'joint', joint=driver_joint, coef=f'{ratio:.17g}')
            ET.SubElement(freehub, 'joint', joint='rear_wheel_spin', coef='-1')
        if clutch:
            if tendons is None:
                tendons = ET.SubElement(root, 'tendon')
            crank_clutch = ET.SubElement(tendons, 'fixed', name='crank_clutch',
                limited='true', range='-1e12 0', margin='0',
                solreflimit=f'{physics_config.closure_time_constant_s:.17g} 1')
            ET.SubElement(crank_clutch, 'joint', joint='crank_spin', coef='1')
            ET.SubElement(crank_clutch, 'joint', joint='drive_shaft_spin', coef='-1')
        if rotor_inertia > 0.:
            if tendons is None:
                tendons = ET.SubElement(root, 'tendon')
            freewheel=ET.SubElement(tendons,'fixed',name='motor_freewheel',
                limited='true',range='-1e12 0',margin='0',
                solreflimit=f'{physics_config.closure_time_constant_s:.17g} 1')
            ET.SubElement(freewheel,'joint',joint='rotor_spin',coef='1')
            ET.SubElement(freewheel,'joint',joint='crank_spin',coef='-1')
        if physics_config.drive.transmission_model == 'geometric_ideal_mid_drive':
            finalize_geometric_transmission(root,physics_config)
        ET.SubElement(actuators,'motor',name='mid_drive',joint=motor_joint,gear='1',
                      ctrllimited='true',ctrlrange=f'0 {physics_config.drive.assist.effective_max_torque:.17g}')
    if physics_config.drive_mode=='crank_effort':
        # The request is validated in the configuration. No contact torque cap
        # is imposed here; the wheel can spin through a saturated tire force.
        ET.SubElement(actuators,'motor',name='human_crank',joint='crank_spin',gear='1')


def finalize_geometric_transmission(root,physics_config):
    """Run again after adding the independent rider to include its zero rows."""
    if (physics_config.drive.transmission_model != 'geometric_ideal_mid_drive'
            or physics_config.drive_mode not in ('crank_effort','articulated_effort')):
        return
    tendons=root.find('tendon')
    if tendons is None:tendons=ET.SubElement(root,'tendon')
    freehub=tendons.find("fixed[@name='geometric_mid_drive_freehub']")
    if freehub is None:
        freehub=ET.SubElement(tendons,'fixed',name='geometric_mid_drive_freehub',limited='true',
            range='-1e12 0',margin='0',solreflimit=f'{physics_config.closure_time_constant_s:.17g} 1')
    for child in list(freehub):freehub.remove(child)
    gearing=physics_config.drive.gearing
    driver_joint='drive_shaft_spin' if physics_config.drive.motor_clutch else 'crank_spin'
    for joint in root.find('worldbody').iter('joint'):
        if joint.get('type','hinge') not in ('slide','hinge'):
            raise ValueError('geometric ideal drive requires scalar planar joints')
        name=joint.get('name')
        if not name:raise ValueError('geometric tendon requires named coordinates')
        coefficient=(gearing.front_radius_m if name==driver_joint else
                     -gearing.rear_radius_m if name=='rear_wheel_spin' else 0.)
        ET.SubElement(freehub,'joint',joint=name,coef=f'{coefficient:.17g}')
