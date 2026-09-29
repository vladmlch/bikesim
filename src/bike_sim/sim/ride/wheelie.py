"""Contact-based research truth and physics-rate event accounting.

A wheelie candidate is NOT proof of motor causation. Crests and rider impulses
can produce the same rear-supported front lift. Absolute pitch alone is never
used to classify wheelies, and both-wheel flight always has its own state.
"""
from dataclasses import dataclass, fields
from math import atan2, cos, sin
import numpy as np
from bike_sim.physics.checks import scalar


def classify_contact(*, front_load_n, rear_load_n, front_clearance_m,
                     rear_clearance_m, relative_pitch_rad, load_threshold_n=5.,
                     clearance_threshold_m=.01, pitch_threshold_rad=.035):
    values = (front_load_n, rear_load_n, front_clearance_m, rear_clearance_m,
              load_threshold_n, clearance_threshold_m, pitch_threshold_rad)
    for value in values:
        scalar(value, 'contact classification input', minimum=0.)
    scalar(relative_pitch_rad, 'road-relative pitch')
    front = front_load_n > load_threshold_n
    rear = rear_load_n > load_threshold_n
    if front and rear:
        return 'two_wheels'
    if not front and not rear:
        return ('flight' if min(front_clearance_m, rear_clearance_m) > clearance_threshold_m
                else 'unsupported')
    if not rear:
        return 'rear_lift' if rear_clearance_m > clearance_threshold_m else 'rear_unloaded'
    if front_clearance_m <= clearance_threshold_m:
        return 'front_unloaded'
    return 'wheelie_candidate' if relative_pitch_rad > pitch_threshold_rad else 'front_lift'


@dataclass(frozen=True)
class WheelieTruth:
    time_s: float
    front_load_n: float = 0.
    rear_load_n: float = 0.
    front_clearance_m: float = 0.
    rear_clearance_m: float = 0.
    relative_pitch_rad: float = 0.
    position_m: float = 0.
    speed_mps: float = 0.
    pitch_up_rad: float = 0.
    pitch_rate_up_rad_s: float = 0.
    axle_pitch_rad: float = 0.
    road_pitch_rad: float = 0.
    front_slip_mps: float = 0.
    rear_slip_mps: float = 0.
    com_x_m: float = 0.
    com_z_m: float = 0.

    def __post_init__(self):
        for f in fields(self):
            scalar(getattr(self, f.name), f.name)
        for name in ('time_s', 'front_load_n', 'rear_load_n', 'front_clearance_m', 'rear_clearance_m'):
            scalar(getattr(self, name), name, minimum=0.)

    @property
    def contact_state(self):
        return classify_contact(front_load_n=self.front_load_n, rear_load_n=self.rear_load_n,
            front_clearance_m=self.front_clearance_m, rear_clearance_m=self.rear_clearance_m,
            relative_pitch_rad=self.relative_pitch_rad)


def truth_from_sample(sim, sample):
    """Use only immutable incoming-state channels, never endpoint qpos/velocity."""
    tires = sample.channels['tires']
    front, rear = tires['front'], tires['rear']
    f, r = np.asarray(front['wheel_axis_m']), np.asarray(rear['wheel_axis_m'])
    vertices = sim.physical.vertices
    zf, zr = np.interp([f[0], r[0]], vertices[:, 0], vertices[:, 1])
    dx = float(f[0]-r[0])
    axle_pitch = atan2(float(f[2]-r[2]), dx)
    road_pitch = atan2(float(zf-zr), dx)
    rf = float(sim.model.geom_size[sim.contact_query.front_id, 0])
    rr = float(sim.model.geom_size[sim.contact_query.rear_id, 0])
    # Different wheel radii on a uniform grade are not front lift. Account for
    # their vertical center-to-surface offset when defining the reference line.
    normal_z = max(abs(cos(road_pitch)), 1e-6)
    supported_pitch = atan2(float(zf-zr)+(rf-rr)/normal_z, dx)
    difference = axle_pitch-supported_pitch
    relative = atan2(sin(difference), cos(difference))
    query = sim.physical.tire.profile if sim.physical.tire is not None else None
    if query is None:
        from bike_sim.terrain.contact_profile import ProfileQuery
        query = ProfileQuery(vertices)
    def clearance(channel, center, radius):
        delta = channel.get('penetration_m')
        if delta is None:
            delta = query.contact(center[[0, 2]], radius).delta
        return max(0., -float(delta))
    def load(channel):
        return sum(float(p['normal_load_n']) for p in channel['patches'] if p['source_geom'] == 'terrain')
    _, pitch_dof = sim.physical.address('root_pitch')
    mass = sample.channels['mass']
    return WheelieTruth(time_s=sample.time_s, front_load_n=load(front), rear_load_n=load(rear),
        front_clearance_m=clearance(front, f, rf), rear_clearance_m=clearance(rear, r, rr),
        relative_pitch_rad=relative, position_m=float(sample.qpos[sim.root_x_qposadr]),
        speed_mps=float(sample.qvel[sim.root_x_dofadr]), pitch_up_rad=-float(sample.qpos[sim.root_pitch_qposadr]),
        pitch_rate_up_rad_s=-float(sample.qvel[pitch_dof]), axle_pitch_rad=axle_pitch,
        road_pitch_rad=road_pitch, front_slip_mps=float(front['slip_mps']),
        rear_slip_mps=float(rear['slip_mps']), com_x_m=float(mass['com_m'][0]),
        com_z_m=float(mass['com_m'][2]))


class WheelieTracker:
    """Time-based confirmation with load/clearance/angle exit hysteresis.

    Confirmed time starts after persistence_s. Candidate time retains the
    onset interval as well. A transition to rear-unloaded/flight clears a
    wheelie immediately. No metric changes a force or contact state.
    """
    def __init__(self, persistence_s=.02, load_threshold_n=5., clearance_threshold_m=.01,
                 pitch_threshold_rad=.035):
        self.persistence_s = scalar(persistence_s, 'wheelie persistence', minimum=0.)
        self.load_threshold_n = scalar(load_threshold_n, 'load threshold', positive=True)
        self.clearance_threshold_m = scalar(clearance_threshold_m, 'clearance threshold', positive=True)
        self.pitch_threshold_rad = scalar(pitch_threshold_rad, 'pitch threshold', positive=True)
        self.reset()

    def reset(self):
        self._last_end = None
        self._candidate_s = 0.
        self._active = False
        self._states = {}
        self._metrics = dict(duration_s=0., wheelie_candidate_time_s=0., wheelie_time_s=0.,
            wheelie_episodes=0, front_unloaded_time_s=0., front_lift_time_s=0., flight_time_s=0.,
            max_front_clearance_m=0., max_relative_pitch_rad=0., rear_slip_distance_m=0.,
            min_front_load_n=None)
        self.state = 'uninitialized'

    def update(self, truth, dt_s):
        if not isinstance(truth, WheelieTruth):
            raise ValueError('expected WheelieTruth')
        dt = scalar(dt_s, 'metric interval', positive=True)
        if self._last_end is not None and abs(truth.time_s-self._last_end) > 1e-9:
            raise ValueError('metric intervals must be contiguous and unrepeated')
        hysteresis = self._candidate_s > 0.
        threshold = self.load_threshold_n*(2. if hysteresis else 1.)
        # Rear support must always pass the original threshold. Hysteresis is
        # only for the front wheel; otherwise 7 N on the rear could become flight.
        raw = classify_contact(front_load_n=truth.front_load_n, rear_load_n=truth.rear_load_n,
            front_clearance_m=truth.front_clearance_m, rear_clearance_m=truth.rear_clearance_m,
            relative_pitch_rad=truth.relative_pitch_rad, load_threshold_n=self.load_threshold_n,
            clearance_threshold_m=self.clearance_threshold_m, pitch_threshold_rad=self.pitch_threshold_rad)
        candidate = (truth.rear_load_n > self.load_threshold_n and truth.front_load_n <= threshold
            and truth.front_clearance_m > self.clearance_threshold_m*(.5 if hysteresis else 1.)
            and truth.relative_pitch_rad > self.pitch_threshold_rad*(.5 if hysteresis else 1.))
        self._candidate_s = self._candidate_s+dt if candidate else 0.
        active = candidate and self._candidate_s+1e-12 >= self.persistence_s
        if active and not self._active:
            self._metrics['wheelie_episodes'] += 1
        self.state = 'wheelie' if active else raw
        self._active = active
        m = self._metrics
        m['duration_s'] += dt
        m['wheelie_candidate_time_s'] += dt if candidate else 0.
        m['wheelie_time_s'] += dt if active else 0.
        m['front_unloaded_time_s'] += dt if truth.front_load_n <= self.load_threshold_n else 0.
        m['front_lift_time_s'] += dt if raw in ('front_lift', 'wheelie_candidate') else 0.
        m['flight_time_s'] += dt if raw == 'flight' else 0.
        m['max_front_clearance_m'] = max(m['max_front_clearance_m'], truth.front_clearance_m)
        m['max_relative_pitch_rad'] = max(m['max_relative_pitch_rad'], truth.relative_pitch_rad)
        if truth.rear_load_n > self.load_threshold_n:
            m['rear_slip_distance_m'] += abs(truth.rear_slip_mps)*dt
        m['min_front_load_n'] = truth.front_load_n if m['min_front_load_n'] is None else min(m['min_front_load_n'], truth.front_load_n)
        self._states[raw] = self._states.get(raw, 0.)+dt
        self._last_end = truth.time_s+dt
        return self.state

    @property
    def metrics(self):
        return dict(self._metrics, contact_state_time_s=dict(self._states))


def quasistatic_front_load(mass_kg, wheelbase_m, com_forward_m, com_height_m,
                          road_pitch_rad, acceleration_mps2, *, gravity_mps2=9.81):
    """Signed front normal reaction for a rigid, two-contact incline model.

    Geometry is measured along/normal to the road from the rear contact.
    This diagnostic neglects suspension, pitch acceleration and tire dynamics;
    it is NEVER applied as a simulated force. Negative means no static two-wheel
    support solution. The controller does not receive this privileged estimate.
    """
    for value, name in ((mass_kg, 'mass'), (wheelbase_m, 'wheelbase'), (gravity_mps2, 'gravity')):
        scalar(value, name, positive=True)
    scalar(com_height_m, 'CoM height', minimum=0.)
    for value in (com_forward_m, road_pitch_rad, acceleration_mps2):
        scalar(value, 'load transfer input')
    return mass_kg*(gravity_mps2*cos(road_pitch_rad)*com_forward_m
                   -com_height_m*(acceleration_mps2+gravity_mps2*sin(road_pitch_rad)))/wheelbase_m
