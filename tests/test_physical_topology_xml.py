"""XML contract tests; these do not replace compiling the model with MuJoCo."""
from types import SimpleNamespace
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from bike_sim.physics.physical_config import PhysicalDriveConfig, TireBackendConfig, ArticulatedConfig
from bike_sim.physics.rider_segments import ArticulatedPose, segment_masses
from bike_sim.mujoco.articulated_rider import build_articulated_rider
from bike_sim.mujoco.physical_topology import finish_physical_topology


def pose():
    return ArticulatedPose(
        saddle=SimpleNamespace(top_center=np.array([0.,0.,.69])),
        hip=np.array([0.,0.,.75]), shoulder=np.array([.3,0.,1.05]),
        elbow=np.array([.45,0.,.9]), grip=np.array([.55,0.,.75]),
        head_center=np.array([.3,0.,1.25]),
        knee_front=np.array([.3,0.,.45]), knee_rear=np.array([.1,0.,.4]),
        ankle_front=np.array([.165,0.,.115]), ankle_rear=np.array([-.165,0.,.115]),
        pedal_front=np.array([.165,0.,0.]), pedal_rear=np.array([-.165,0.,0.]),
    )


def root_fixture():
    root=ET.Element('mujoco'); world=ET.SubElement(root,'worldbody')
    frame=ET.SubElement(world,'body',name='frame')
    crank=ET.SubElement(frame,'body',name='crank',pos='0 0 0')
    ET.SubElement(crank,'joint',name='crank_spin',type='hinge',axis='0 1 0')
    ET.SubElement(crank,'geom',name='geom_crank_spindle',mass='.1')
    for s, sign in (('front',1),('rear',-1)):
        ET.SubElement(crank,'geom',name='geom_crank_arm_'+s,mass='.2')
        pedal=ET.SubElement(crank,'body',name='pedal_'+s,pos=f'{sign*.165} {-sign*.075} 0')
        ET.SubElement(pedal,'joint',name='pedal_spin_'+s,axis='0 1 0')
        ET.SubElement(pedal,'geom',name='geom_pedal_'+s,pos=f'0 {-sign*.04} 0',mass='.175')
        ET.SubElement(pedal,'site',name='site_pedal_'+s,pos=f'0 {-sign*.04} 0')
    seatstay=ET.SubElement(frame,'body',name='seatstay')
    rear=ET.SubElement(seatstay,'body',name='rear_wheel',pos='-.4 0 0')
    front=ET.SubElement(frame,'body',name='front_wheel')
    for s, wheel in (('front',front),('rear',rear)):
        ET.SubElement(wheel,'joint',name=s+'_wheel_spin',damping='.01')
        ET.SubElement(wheel,'geom',name=f'geom_{s}_contact',contype='1',conaffinity='1')
        ET.SubElement(wheel,'inertial',mass='2.8' if s=='rear' else '2.4',pos='0 0 0',diaginertia='.1 .2 .1')
    ET.SubElement(rear,'geom',name='geom_rear_cassette',mass='0',size='.095')
    ET.SubElement(frame,'geom',name='frame_crash',contype='1',conaffinity='1',mass='1')
    actuators=ET.SubElement(root,'actuator')
    for name,joint in (('rear_drive','rear_wheel_spin'),('crank_drive','crank_spin'),
                       ('front_brake','front_wheel_spin'),('rear_brake','rear_wheel_spin')):
        ET.SubElement(actuators,'motor',name=name,joint=joint)
    return root


@pytest.mark.parametrize('drive',['coast','ideal_speed_control','crank_effort','articulated_effort'])
def test_actuators_reactions_mass_and_backend_ownership(drive):
    root=root_fixture()
    cfg=SimpleNamespace(physics_mode='physical',drive_mode=drive,drive=PhysicalDriveConfig(),
                        tires=TireBackendConfig(backend='compliant_2d'), articulated=ArticulatedConfig())
    finish_physical_topology(root,SimpleNamespace(crank_length=165.),
                             SimpleNamespace(crank_pedals_mass=.85,rear_wheel_mass=2.8),cfg)
    assert root.find(".//joint[@name='cassette_spin']") is not None
    assert root.find(".//joint[@name='pedal_front_spin']") is not None
    assert (root.find(".//motor[@name='rear_drive']") is not None)==(drive=='ideal_speed_control')
    assert (root.find(".//motor[@name='human_crank']") is not None)==(drive=='crank_effort')
    assert (root.find(".//motor[@name='mid_drive']") is not None)==(drive in ('crank_effort','articulated_effort'))
    masses={b.get('name'):float(b.find('inertial').get('mass')) for b in root.iter('body') if b.find('inertial') is not None}
    assert masses['crank']+masses['pedal_front']+masses['pedal_rear']==pytest.approx(.85)
    assert masses['rear_wheel']+masses['cassette']==pytest.approx(2.8)
    for side in ('front','rear'):
        assert root.find(f".//geom[@name='geom_{side}_contact']").get('contype')=='0'
    assert root.find(".//geom[@name='frame_crash']").get('contype')=='1'
    assert root.find(".//motor[@name='crank_drive']") is None


def test_anatomical_tree_independent_root_and_visual_mass_zero():
    world=ET.Element('worldbody'); masses=segment_masses(80.,.4)
    bodies=build_articulated_rider(world,pose(),masses)
    pelvis=bodies['pelvis']; assert pelvis in list(world)
    assert len(pelvis.findall('joint'))==3
    assert bodies['shank_front'] in list(bodies['thigh_front'])
    assert bodies['thigh_front'] in list(pelvis)
    assert bodies['head'] in list(bodies['torso'])
    total=sum(float(b.find('inertial').get('mass')) for b in bodies.values())
    assert total==pytest.approx(80.)
    assert all(g.get('mass')=='0' for g in world.iter('geom'))
    assert world.find(".//site[@name='site_rider_sole_front']") is not None
    for b in bodies.values():
        values=np.fromstring(b.find('inertial').get('fullinertia'),sep=' ')
        I=np.array([[values[0],values[3],values[4]],[values[3],values[1],values[5]],[values[4],values[5],values[2]]])
        eig=np.linalg.eigvalsh(I)
        assert eig[0]>0 and eig[-1]<=sum(eig[:2])+1e-12
