"""One physics-clock owner of automatic rider intent and fieldwise overrides."""
import copy
from dataclasses import replace

from bike_sim.physics.checks import scalar
from bike_sim.physics.seated_climb import (
    SeatedClimbConfig, SeatedClimbPolicy, SeatedClimbSignals, SeatedClimbIntent,
)
from bike_sim.physics.rider_posture import RiderPosture
from bike_sim.sim.ride.control import RideControl


def signals_from_channels(channels) -> SeatedClimbSignals:
    return SeatedClimbSignals(
        pitch_rate_up_rad_s=-float(channels['frame_gyro_body_rad_s'][1]),
        specific_force_body_mps2=tuple(channels['frame_specific_force_body_mps2']),
        crank_rate_rad_s=float(channels['encoders_rad_s']['crank']),
        human_crank_torque_nm=float(channels['human_torque_nm']))


class RiderIntentResolver:
    def __init__(self, config: SeatedClimbConfig, dt_s: float):
        self.dt_s = scalar(dt_s, 'rider physics timestep', positive=True)
        self.period_steps = round(config.period_s/self.dt_s)
        if config.enabled and (self.period_steps < 1
                or abs(self.period_steps*self.dt_s-config.period_s) > 1e-9):
            raise ValueError('rider intention period must be an integer multiple of timestep')
        self.policy = SeatedClimbPolicy(config)
        self.reset()

    def reset(self):
        self.policy.reset()
        self._last_tick_step = -1
        self.intent = SeatedClimbIntent(RiderPosture(), 0.)

    def resolve(self, control: RideControl, signals: SeatedClimbSignals, *, step: int,
                active: bool = True, advance: bool = True) -> RideControl:
        if not isinstance(control, RideControl):
            raise ValueError('expected a rider control')
        if not self.policy.config.enabled or not active:
            return control
        if type(step) is not int or step < 0:
            raise ValueError('rider clock requires a nonnegative physics step')
        if step < self._last_tick_step:
            raise ValueError('reset rider intention before rewinding physics')
        if not advance:
            return copy.deepcopy(self).resolve(control, signals, step=step)
        if step % self.period_steps == 0 and step != self._last_tick_step:
            expected = 0 if self._last_tick_step < 0 else self._last_tick_step+self.period_steps
            if step != expected:
                raise ValueError('rider intention clock skipped an acquisition')
            self.intent = self.policy.update(signals, self.policy.config.period_s)
            self._last_tick_step = step
        if not control.rider_enabled:
            return control
        return replace(control,
            posture=self.intent.posture if control.posture is None else control.posture,
            human_torque_nm=(self.intent.effort_ceiling_nm
                             if control.human_torque_nm is None else control.human_torque_nm))
