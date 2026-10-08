"""Release an unstrapped foot beneath a pedal through finite external waypoints."""
import numpy as np

from bike_sim.physics.checks import array, scalar


class PedalRecovery:
    def __init__(self, half_length_m, pad_radius_m, clearance_m):
        self.half_length_m = scalar(half_length_m, 'recovery sole half length', positive=True)
        self.pad_radius_m = scalar(pad_radius_m, 'recovery pad radius', positive=True)
        self.clearance_m = scalar(clearance_m, 'recovery clearance', positive=True)
        self.reset()

    def reset(self):
        self.stage = 'none'
        self.direction = 1.
        self.release_offset_x_m = 0.

    def state_dict(self) -> dict:
        """Owned recovery-observer snapshot for the runtime bootstrap."""
        return {'stage': self.stage, 'direction': self.direction,
                'release_offset_x_m': self.release_offset_x_m}

    def observe(self, origin_m, rotation, half_size_m, sole_position_m, force_on_rider_n):
        origin = array(origin_m, 'pedal origin', (3,))
        orientation = array(rotation, 'pedal rotation', (3, 3))
        half = array(half_size_m, 'pedal dimensions', (3,))
        sole = array(sole_position_m, 'recovery sole', (3,))
        force = array(force_on_rider_n, 'recovery force', (3,))
        extent = np.abs(orientation) @ half
        if self.stage == 'none':
            if force[2] < -1. and sole[2]+self.pad_radius_m < origin[2]:
                self.stage = 'release'
                self.release_offset_x_m = float(sole[0]-origin[0])
                self.direction = -1. if self.release_offset_x_m < 0. else 1.
        elif self.stage == 'release':
            if sole[2]+2.*self.pad_radius_m < origin[2]-extent[2]-.5*self.clearance_m:
                self.stage = 'escape'
        elif self.stage == 'escape':
            if self.direction*(sole[0]-origin[0]) > extent[0]+self.half_length_m+self.pad_radius_m:
                self.stage = 'raise'
        elif self.stage == 'raise':
            if sole[2] > origin[2]+extent[2]+.5*self.clearance_m:
                self.stage = 'return'
        elif self.stage == 'return':
            if abs(sole[0]-origin[0]) < self.clearance_m and sole[2] > origin[2]+extent[2]:
                self.reset()

    def goal(self, origin_m, rotation, half_size_m):
        if self.stage == 'none':
            return None
        origin = np.asarray(origin_m)
        extent = np.abs(np.asarray(rotation)) @ np.asarray(half_size_m)
        lower = origin[2]-extent[2]-2.*self.pad_radius_m-self.clearance_m
        upper = origin[2]+extent[2]+self.clearance_m
        escape = self.direction*(extent[0]+self.half_length_m+self.pad_radius_m+self.clearance_m)
        offset = self.release_offset_x_m if self.stage == 'release' else 0. if self.stage == 'return' else escape
        return np.array([origin[0]+offset, origin[1],
                         lower if self.stage in ('release', 'escape') else upper])
