"""Rigid-equivalent axle load reference, not a full-rider wheelie controller.

The analytical expression excludes wheel angular-momentum transients, moving
rider CoM, aerodynamic moments, pitch acceleration and unlike contact normals.
A negative answer means a two-support quasistatic state is inadmissible.
"""
from math import sin,cos,isfinite,asin
import numpy as np
import mujoco
from bike_sim.physics.tire import normal_contact


def quasistatic_front_load(mass_kg,wheelbase_m,com_x_m,com_h_m,slope_rad,
                          tangent_accel_mps2=0.,gravity_mps2=9.81):
    values=(mass_kg,wheelbase_m,com_x_m,com_h_m,slope_rad,tangent_accel_mps2,gravity_mps2)
    if not all(isfinite(v) and not isinstance(v,bool) for v in values):
        raise ValueError('load transfer inputs must be finite real scalars')
    if mass_kg<=0 or wheelbase_m<=0 or com_h_m<0 or gravity_mps2<=0:
        raise ValueError('invalid rigid-equivalent geometry')
    return mass_kg/wheelbase_m*(gravity_mps2*(com_x_m*cos(slope_rad)-com_h_m*sin(slope_rad))
                               -tangent_accel_mps2*com_h_m)


def load_transfer_rig(dt_s,slope_rad,com_x_m,com_h_m):
    mass=100.;L=1.2;g=9.81;k=200000.;c=3000.;kt=100000.;ct=1500.;duration=.5
    front=quasistatic_front_load(mass,L,com_x_m,com_h_m,slope_rad)
    rear=mass*g*cos(slope_rad)-front
    if front<=0 or rear<=0:
        raise ValueError('two loaded supports do not exist for this rigid-equivalent fixture')
    if not isfinite(dt_s) or dt_s<=0 or abs(round(duration/dt_s)*dt_s-duration)>1e-10:
        raise ValueError('stand duration must have integer positive intervals')
    xml=f'''<mujoco><compiler angle="radian"/><option timestep="{dt_s}" gravity="0 0 {-g}"/>
    <worldbody><body name="rigid_bike"><joint name="x" type="slide" axis="1 0 0"/>
    <joint name="z" type="slide" axis="0 0 1"/><joint name="pitch" type="hinge" axis="0 1 0"/>
    <inertial mass="{mass}" pos="{com_x_m} 0 {com_h_m}" diaginertia="10 20 10"/>
    <site name="rear" pos="0 0 0"/><site name="front" pos="{L} 0 0"/>
    </body></worldbody></mujoco>'''
    m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m)
    n=np.array([-sin(slope_rad),0.,cos(slope_rad)]);t=np.array([cos(slope_rad),0.,sin(slope_rad)])
    d.qpos[:2]=(-rear/k*n)[[0,2]];d.qpos[2]=-slope_rad+asin((front-rear)/(k*L))
    mujoco.mj_forward(m,d)
    # Explicit static tread anchors carry only the physical road reaction.
    # Pitch remains a free DOF; no servo, equality or hidden stand couple.
    anchors={}
    for side,load in [('front',front),('rear',rear)]:
        p=d.site_xpos[m.site(side).id].copy()
        share=load/(mass*g*cos(slope_rad));needed=mass*g*sin(slope_rad)*share
        anchors[side]=p+t*(needed/kt)
    last={};forces=[];moment=np.zeros(3)
    for _ in range(round(duration/dt_s)):
        d.qfrc_applied.fill(0.);forces=[];moment=np.zeros(3)
        for side in ('front','rear'):
            p=d.site_xpos[m.site(side).id].copy();jp=np.zeros((3,m.nv))
            mujoco.mj_jac(m,d,jp,None,p,m.body('rigid_bike').id)
            velocity=jp@d.qvel;depth=-float(p@n)
            normal,_=normal_contact(depth,-float(velocity@n),k,c)
            shear=float((p-anchors[side])@t)
            tangent=float(np.clip(-kt*shear-ct*(velocity@t),-normal,normal))
            force=normal*n+tangent*t
            d.qfrc_applied[:]+=jp.T@force;last[side]=normal;forces.append(force)
            moment+=np.cross(p-d.xipos[m.body('rigid_bike').id],force)
        mujoco.mj_step(m,d);mujoco.mj_forward(m,d)
    error=max(abs(last['front']-front),abs(last['rear']-rear))/(mass*g)
    return {'front_load_n':last['front'],'rear_load_n':last['rear'],
            'expected_front_load_n':front,'expected_rear_load_n':rear,
            'load_error_over_weight':error,'force_balance_over_weight':float(np.linalg.norm(sum(forces)+mass*m.opt.gravity))/(mass*g),
            'moment_balance_over_weight_wheelbase':float(np.linalg.norm(moment))/(mass*g*L),
            'free_pitch_coordinate_count':1.,'hidden_stand_pitch_constraints':float(m.neq),
            'final_pitch_rate_rad_s':float(d.qvel[2])}, {
                'load_error_over_weight':(0.,.01),'force_balance_over_weight':(0.,.01),
                'moment_balance_over_weight_wheelbase':(0.,.01),'hidden_stand_pitch_constraints':(0.,0.)}
