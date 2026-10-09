"""Setup-time NumPy streams shared by the Python and native sensor models.

Reference: supplied NumPy random/_generator.pyx, bit_generator.pyx and
random/src/distributions/distributions.c. In particular scale=0 still draws
from the normal stream. Do not substitute std::normal_distribution in C++.
"""
from dataclasses import dataclass, replace
import numpy as np


def _seed(value):
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError('sensor seed must be a nonnegative integer')
    return value


@dataclass(frozen=True)
class NoiseTape:
    noise: np.ndarray
    dropout_uniform: np.ndarray

    def __post_init__(self):
        for values in (self.noise, self.dropout_uniform):
            if np.asarray(values).dtype.kind not in "fiu":
                raise ValueError("noise tape must contain real numeric values, not booleans or objects")
        noise = np.array(self.noise, dtype=np.float64, order='C', copy=True)
        dropout = np.array(self.dropout_uniform, dtype=np.float64, order='C', copy=True)
        if noise.ndim != 2 or noise.shape[1] != 9 or dropout.shape != (noise.shape[0],):
            raise ValueError('noise tape requires (count, 9) noise and (count,) dropout')
        if not np.isfinite(noise).all() or not np.isfinite(dropout).all():
            raise ValueError('noise tape must be finite')
        if np.any((dropout < 0.) | (dropout >= 1.)):
            raise ValueError('dropout uniforms must lie in [0, 1)')
        noise.setflags(write=False)
        dropout.setflags(write=False)
        object.__setattr__(self, 'noise', noise)
        object.__setattr__(self, 'dropout_uniform', dropout)


class SensorNoiseSource:
    def __init__(self, config, *, seed=0):
        from bike_sim.sim.research.sensors import SensorConfig
        if not isinstance(config, SensorConfig):
            raise ValueError('expected SensorConfig')
        self.config, self.seed = replace(config), _seed(seed)
        noise_seed, dropout_seed = np.random.SeedSequence(self.seed).spawn(2)
        self._rng = np.random.default_rng(noise_seed)
        self._dropout_rng = np.random.default_rng(dropout_seed)

    def draw(self):
        c, rng = self.config, self._rng
        # Four separate calls, including the scalar gyro call. The disabled
        # IMU and zero scales do not skip any of them.
        acceleration = rng.normal(0., c.acceleration_std_mps2, 3)
        gyro = rng.normal(0., c.gyro_std_rad_s)
        encoder = rng.normal(0., c.encoder_std_rad_s, 3)
        torque = rng.normal(0., c.torque_std_nm, 2)
        noise = np.concatenate((acceleration, np.atleast_1d(gyro), encoder, torque))
        return noise, float(self._dropout_rng.random())


def build_noise_tape(config, *, seed, count):
    if type(count) is not int or count < 0:
        raise ValueError('noise tape count must be a nonnegative integer')
    source = SensorNoiseSource(config, seed=seed)
    noise = np.empty((count, 9), dtype=np.float64)
    dropout = np.empty(count, dtype=np.float64)
    for index in range(count):
        noise[index], dropout[index] = source.draw()
    return NoiseTape(noise, dropout)
