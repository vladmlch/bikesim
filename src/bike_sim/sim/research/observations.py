"""Convert frozen physical sensor channels to policy inputs, without truth."""
from bike_sim.sim.research.sensors import SensorObservation
from bike_sim.sim.ride.physical_observations import sensor_channels


def raw_observation(sim, sample=None):
    if sample is None:
        channels = sensor_channels(sim.physical, drive_channels=sim.physical.drive.probe_last)
        t = sim.time_s
    else:
        channels = sample.channels['sensors']
        t = sample.time_s
    enc = channels['encoders_rad_s']
    return SensorObservation(time_s=t, source_time_s=t, valid=True,
        specific_force_body_mps2=tuple(channels['frame_specific_force_body_mps2']),
        pitch_rate_up_rad_s=-float(channels['frame_gyro_body_rad_s'][1]),
        front_wheel_rad_s=float(enc['front_wheel']), rear_wheel_rad_s=float(enc['rear_wheel']),
        crank_rad_s=float(enc['crank']), motor_torque_nm=float(channels['motor_torque_nm']),
        human_torque_nm=float(channels['human_torque_nm']))
