"""Independent actual engine shaft/gear test with declared fixed carrier."""
import mujoco
import numpy as np
from bike_sim.sim.ride.ideal_freehub import IdealFreehubConstraint


def shaft_ratio_rig(dt_s: float,ratio: float=34/51,*,overrun=False):
    if not np.isfinite(ratio) or ratio<=0 or not np.isfinite(dt_s) or dt_s<=0:
        raise ValueError('positive finite ratio and time interval required')
    jc,jw=.04,.24;duration=.1;count=round(duration/dt_s)
    if count<1 or abs(count*dt_s-duration)>1e-10:raise ValueError('whole shaft fixture intervals required')
    xml=f'''<mujoco><option timestep="{dt_s}" gravity="0 0 0"/><worldbody>
      <body><joint name="crank_spin" axis="0 1 0"/><inertial mass="1" pos="0 0 0" diaginertia=".04 {jc} .04"/></body>
      <body><joint name="rear_wheel_spin" axis="0 1 0"/><inertial mass="1" pos="0 0 0" diaginertia=".24 {jw} .24"/></body>
      </worldbody><tendon><fixed name="ideal_mid_drive_freehub" limited="true" range="-1e12 0"
        solreflimit=".0025 1" solimplimit=".9999 .9999 .001 .5 2">
        <joint joint="crank_spin" coef="{ratio}"/><joint joint="rear_wheel_spin" coef="-1"/>
      </fixed></tendon><actuator><motor name="mid_drive" joint="crank_spin" gear="1"/></actuator></mujoco>'''
    m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m);hub=IdealFreehubConstraint(m,ratio)
    torque=0. if overrun else 20.
    if overrun:d.qvel[1]=20.
    hub.reset(m,d);last=[];power_error=0.;max_reaction=0.
    for i in range(count):
        hub.prepare(m,d);velocity=d.qvel.copy();d.ctrl[0]=torque
        mujoco.mj_step(m,d);reaction=hub.solved_qfrc(m,d)
        actual=float(d.actuator_force[0]);actual_generalized=float(d.qfrc_actuator[0])
        power_error=max(power_error,abs(actual*velocity[0]-float(d.qfrc_actuator@velocity)))
        max_reaction=max(max_reaction,float(np.linalg.norm(reaction)))
        if i>=count//2:last.append((float(d.qacc[0]),float(d.qacc[1]),actual_generalized))
    acceleration=np.mean(last,axis=0);expected=torque/(jc+ratio*ratio*jw)
    speed_error=0. if overrun else abs(float(d.qvel[1]/d.qvel[0])-ratio)/ratio
    metrics={'gear_ratio':ratio,'delivered_crank_torque_nm':float(acceleration[2]),
        'crank_acceleration_rad_s2':float(acceleration[0]),'expected_crank_acceleration_rad_s2':expected,
        'acceleration_relative_error':abs(acceleration[0]-expected)/max(1.,abs(expected)),
        'speed_ratio_relative_error':speed_error,'shaft_power_identity_error_w':power_error,
        'max_transmission_reaction_norm':max_reaction,'overrun':float(overrun)}
    bounds={'acceleration_relative_error':(0.,.001),'speed_ratio_relative_error':(0.,.001),
            'shaft_power_identity_error_w':(0.,1e-10)}
    if overrun:bounds['max_transmission_reaction_norm']=(0.,1e-10)
    return metrics,bounds
