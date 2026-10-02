"""Independent planar rider tree with explicit synthetic segment inertias."""
import xml.etree.ElementTree as ET
import numpy as np
from bike_sim.physics.inertia import add_body_inertial
from bike_sim.physics.rider_segments import segment_inertia


def _vec(value):
    return ' '.join(format(float(x), '.17g') for x in value)


def build_articulated_rider(worldbody, pose, masses, *, pedal_lateral_m=.115, envelopes=None):
    expected={'pelvis','torso','head','upper_arm_pair','forearm_pair'} | {
        f'{part}_{side}' for side in ('front','rear') for part in ('thigh','shank','foot')}
    if set(masses)!=expected:
        raise ValueError('incomplete anatomical segment mass budget')
    bodies={}
    centers={}

    def segment(key, parent, start, end, radius, joint=None):
        start,end=np.asarray(start,dtype=float),np.asarray(end,dtype=float)
        origin=np.zeros(3) if parent is worldbody else centers[parent.get('name')]
        body=ET.SubElement(parent,'body',name='rider_'+key,pos=_vec(start-origin))
        centers['rider_'+key]=start
        if joint:
            attrs={}
            if envelopes is not None:
                from bike_sim.physics.rider_envelope import joint_q_range
                lo,hi=joint_q_range(envelopes[joint])
                # The parent builder explicitly uses a degree compiler.
                attrs={'limited':'true','range':_vec(np.degrees([lo,hi]))}
            ET.SubElement(body,'joint',name=joint,type='hinge',axis='0 1 0',damping='0',**attrs)
        vector=end-start
        add_body_inertial(body,masses[key],vector/2,segment_inertia(masses[key],vector,radius))
        ET.SubElement(body,'geom',name='geom_rider_'+key,type='capsule',
                      fromto=_vec(np.r_[np.zeros(3),vector]),size=str(radius),mass='0',
                      contype='0',conaffinity='0',rgba='.35 .45 .6 1')
        bodies[key]=body
        return body

    # Pelvis is not attached to the bicycle along any degree of freedom.
    pelvis=segment('pelvis',worldbody,pose.hip,pose.hip+np.array([0.,0.,.12]),.085)
    for name,kind,axis in (('rider_root_x','slide','1 0 0'),('rider_root_z','slide','0 0 1'),
                           ('rider_root_pitch','hinge','0 1 0')):
        ET.SubElement(pelvis,'joint',name=name,type=kind,axis=axis,damping='0')
    ET.SubElement(pelvis,'site',name='site_rider_saddle',pos='0 0 -0.060',size='.004')
    torso=segment('torso',pelvis,pose.hip,pose.shoulder,.11,'rider_torso_hinge')
    head=ET.SubElement(torso,'body',name='rider_head',pos=_vec(pose.head_center-pose.hip))
    mass=masses['head']; radius=.11
    add_body_inertial(head,mass,np.zeros(3),.4*mass*radius*radius*np.eye(3))
    ET.SubElement(head,'geom',name='geom_rider_head',type='sphere',size=str(radius),
                  mass='0',contype='0',conaffinity='0',rgba='.6 .6 .6 1')
    bodies['head']=head
    arm=segment('upper_arm_pair',torso,pose.shoulder,pose.elbow,.045,'rider_shoulder')
    forearm=segment('forearm_pair',arm,pose.elbow,pose.grip,.04,'rider_elbow')
    ET.SubElement(forearm,'site',name='site_rider_grip',pos=_vec(pose.grip-pose.elbow),size='.004')
    for side,sign in (('front',-1.),('rear',1.)):
        lateral=np.array([0.,sign*pedal_lateral_m,0.])
        hip=pose.hip.copy(); hip[1]=lateral[1]
        knee=getattr(pose,'knee_'+side).copy(); knee[1]=lateral[1]
        ankle=getattr(pose,'ankle_'+side).copy(); ankle[1]=lateral[1]
        pedal=getattr(pose,'pedal_'+side).copy(); pedal[1]=lateral[1]
        thigh=segment('thigh_'+side,pelvis,hip,knee,.07,'rider_hip_'+side)
        shank=segment('shank_'+side,thigh,knee,ankle,.055,'rider_knee_'+side)
        foot=segment('foot_'+side,shank,ankle,pedal,.035,'rider_ankle_'+side)
        # A separate contact pad defines the sole; its topological attachment does
        # not weld the foot to the pedal. The platform top is 8 mm above its spindle.
        ET.SubElement(foot,'site',name='site_rider_sole_'+side,
                      pos=_vec(pedal-ankle+np.array([0.,0.,.008])),size='.004')
    return bodies


def add_rider_actuators(root, config):
    actuators=root.find('actuator')
    if actuators is None:
        actuators=ET.SubElement(root,'actuator')
    for joint in root.findall('.//body/joint'):
        name=joint.get('name','')
        if name.startswith('rider_') and not name.startswith('rider_root_'):
            limit=format(config.joint_limit_nm,'.17g')
            ET.SubElement(actuators,'general',name='act_'+name,joint=name,gear='1',
                          gaintype='fixed',gainprm='1',biastype='affine',
                          biasprm=f'0 0 {-config.joint_kd_nms_rad:.17g}',
                          ctrllimited='false',forcelimited='true',forcerange=f'-{limit} {limit}')
