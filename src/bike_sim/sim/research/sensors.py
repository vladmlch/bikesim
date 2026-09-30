"""Explicit causal IMU/encoder/torque sensor model; no privileged terrain data."""
from collections import deque
from dataclasses import dataclass, fields, replace
import numpy as np
from bike_sim.physics.checks import array, scalar


@dataclass(frozen=True)
class SensorConfig:
    latency_s: float = .01
    acceleration_std_mps2: float = .05
    gyro_std_rad_s: float = .002
    encoder_std_rad_s: float = .01
    torque_std_nm: float = .05
    # An absent sensor reports exact zeros, not noise: a policy must not be able
    # to mistake a missing IMU for a quiet one. Keep this field LAST so
    # ideal()'s positional cls(0., 0., 0., 0., 0.) stays valid.
    imu_enabled: bool = True

    def __post_init__(self):
        for f in fields(self):
            if f.name != 'imu_enabled':
                scalar(getattr(self, f.name), f.name, minimum=0.)
        if type(self.imu_enabled) is not bool:
            raise ValueError('imu_enabled must be a bool')

    @classmethod
    def ideal(cls):
        return cls(0., 0., 0., 0., 0.)


@dataclass(frozen=True)
class SensorObservation:
    time_s: float
    source_time_s: float
    valid: bool
    specific_force_body_mps2: tuple[float, float, float]
    pitch_rate_up_rad_s: float
    front_wheel_rad_s: float
    rear_wheel_rad_s: float
    crank_rad_s: float
    motor_torque_nm: float
    human_torque_nm: float

    def __post_init__(self):
        scalar(self.time_s, 'observation time', minimum=0.)
        scalar(self.source_time_s, 'sensor source time', minimum=0.)
        if self.source_time_s > self.time_s+1e-12:
            raise ValueError('sensor observation cannot come from the future')
        if not isinstance(self.valid, bool):
            raise ValueError('sensor validity must be a bool')
        force = array(self.specific_force_body_mps2, 'proper acceleration', (3,))
        object.__setattr__(self, 'specific_force_body_mps2', tuple(map(float, force)))
        for name in ('pitch_rate_up_rad_s', 'front_wheel_rad_s', 'rear_wheel_rad_s',
                     'crank_rad_s', 'motor_torque_nm', 'human_torque_nm'):
            scalar(getattr(self, name), name)


class SensorPipeline:
    """Noise is drawn once on push. read() is idempotent and consumes no RNG."""
    def __init__(self, config=None, *, seed=0):
        self.config = SensorConfig() if config is None else config
        if not isinstance(self.config, SensorConfig):
            raise ValueError('expected SensorConfig')
        if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
            raise ValueError('sensor seed must be a nonnegative integer')
        self.seed = seed
        self._queue = deque()
        self._last_time = None
        self._delivery_time = None
        self._rng = np.random.default_rng(seed)

    def reset(self, initial):
        self._queue.clear()
        self._last_time = None
        self._delivery_time = None
        self._rng = np.random.default_rng(self.seed)
        self.push(initial)

    def push(self, raw):
        if not isinstance(raw, SensorObservation):
            raise ValueError('expected SensorObservation')
        t = raw.source_time_s
        if self._last_time is not None and t <= self._last_time:
            raise ValueError('sensor input timestamps must strictly increase')
        c, rng = self.config, self._rng
        acceleration = np.asarray(raw.specific_force_body_mps2)+rng.normal(0., c.acceleration_std_mps2, 3)
        gyro = raw.pitch_rate_up_rad_s+float(rng.normal(0., c.gyro_std_rad_s))
        encoder_noise = rng.normal(0., c.encoder_std_rad_s, 3)
        torque_noise = rng.normal(0., c.torque_std_nm, 2)
        measured = replace(raw, specific_force_body_mps2=tuple(acceleration), pitch_rate_up_rad_s=gyro,
            front_wheel_rad_s=raw.front_wheel_rad_s+float(encoder_noise[0]),
            rear_wheel_rad_s=raw.rear_wheel_rad_s+float(encoder_noise[1]),
            crank_rad_s=raw.crank_rad_s+float(encoder_noise[2]),
            motor_torque_nm=raw.motor_torque_nm+float(torque_noise[0]),
            human_torque_nm=raw.human_torque_nm+float(torque_noise[1]))
        if not c.imu_enabled:
            # The noise above is still drawn (and discarded) so toggling the IMU
            # does not shift the encoder/torque noise stream for a given seed.
            measured = replace(measured, specific_force_body_mps2=(0., 0., 0.), pitch_rate_up_rad_s=0.)
        self._queue.append(measured)
        self._last_time = t

    def read(self, time_s):
        now = scalar(time_s, 'sensor delivery time', minimum=0.)
        if self._delivery_time is not None and now < self._delivery_time:
            raise ValueError('sensor delivery clock cannot go backwards')
        if not self._queue:
            raise RuntimeError('reset the sensor pipeline before reading it')
        cutoff = now-self.config.latency_s
        # Keep the newest eligible sample and all not-yet-deliverable ones.
        while len(self._queue) > 1 and self._queue[1].source_time_s <= cutoff+1e-12:
            self._queue.popleft()
        selected = self._queue[0]
        if selected.source_time_s > now+1e-12:
            raise ValueError('cannot deliver a future sensor sample')
        self._delivery_time = now
        return replace(selected, time_s=now, valid=selected.source_time_s <= cutoff+1e-12)
