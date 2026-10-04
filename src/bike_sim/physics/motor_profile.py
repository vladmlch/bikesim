"""Declared mid-drive data and pedelec permission.

Bosch values were recalled from memory without a web lookup, as requested;
they are unverified approximations, including the linear eMTB support law.
"""
from collections.abc import Mapping
from dataclasses import dataclass
import math
from types import MappingProxyType

from bike_sim.physics.checks import scalar


@dataclass(frozen=True)
class MotorProfile:
    name: str
    peak_torque_nm: float
    rated_power_w: float
    peak_power_w: float
    torque_tau_s: float
    mode_gains: Mapping[str, float | tuple[float, float]]
    emtb_full_gain_at_nm: float
    cadence_support_max_rpm: float
    cutoff_mps: float
    taper_width_mps: float
    gate_min_crank_rad_s: float
    provenance: str

    def __post_init__(self):
        for name in ('peak_torque_nm', 'rated_power_w', 'peak_power_w', 'torque_tau_s',
                     'emtb_full_gain_at_nm', 'cadence_support_max_rpm', 'cutoff_mps',
                     'taper_width_mps', 'gate_min_crank_rad_s'):
            scalar(getattr(self, name), name, positive=True)
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError('motor profile requires a name')
        if self.rated_power_w > self.peak_power_w:
            raise ValueError('rated motor power exceeds peak power')
        if self.taper_width_mps > self.cutoff_mps:
            raise ValueError('speed taper exceeds the cutoff speed')
        if not isinstance(self.mode_gains, Mapping) or set(self.mode_gains) != {'eco', 'tour', 'emtb', 'turbo'}:
            raise ValueError('profile must declare eco, tour, emtb and turbo')
        gains = {}
        for mode, gain in self.mode_gains.items():
            if mode == 'emtb':
                if not isinstance(gain, tuple) or len(gain) != 2:
                    raise ValueError('eMTB requires two ordered gain bounds')
                low, high = (scalar(value, 'eMTB gain', minimum=0.) for value in gain)
                if low > high:
                    raise ValueError('eMTB gain bounds must increase')
                gains[mode] = (low, high)
            else:
                gains[mode] = scalar(gain, f'{mode} gain', minimum=0.)
        # Freeze a defensive copy: dataclass freezing alone leaves dicts mutable.
        object.__setattr__(self, 'mode_gains', MappingProxyType(gains))
        if not isinstance(self.provenance, str) or not self.provenance.strip():
            raise ValueError('motor profile requires provenance')

    def __deepcopy__(self, memo):
        # Force probes copy controller state, while this immutable data can be
        # shared. MappingProxyType itself cannot be deep-copied/pickled.
        return self


BOSCH_CX_GEN4 = MotorProfile(
    name='bosch_performance_line_cx_gen4',
    peak_torque_nm=85., rated_power_w=250., peak_power_w=600., torque_tau_s=.04,
    mode_gains={'eco': .6, 'tour': 1.4, 'emtb': (1.4, 3.4), 'turbo': 3.4},
    emtb_full_gain_at_nm=40., cadence_support_max_rpm=120.,
    cutoff_mps=25/3.6, taper_width_mps=2/3.6, gate_min_crank_rad_s=math.radians(5.),
    provenance='Bosch Performance Line CX Gen 4 (no ABS); recalled from memory 2026-10-04, '
               'unverified, no web lookup by user instruction')

PROFILES = {'bosch_cx_gen4': BOSCH_CX_GEN4}


def assist_gain(profile: MotorProfile, mode: str, human_nm: float) -> float:
    """Return fixed support or the declared eMTB ramp with rider torque."""
    human = scalar(human_nm, 'human torque')
    if not isinstance(mode, str) or mode not in profile.mode_gains:
        raise ValueError(f'unknown assist mode {mode!r}')
    gain = profile.mode_gains[mode]
    if isinstance(gain, tuple):
        low, high = gain
        fraction = min(1., max(0., human)/profile.emtb_full_gain_at_nm)
        return low+(high-low)*fraction
    return gain


def pedelec_cap(human_nm, crank_rad_s, external_cap_nm, *, braking, gate_min_crank_rad_s):
    """Return zero when forbidden, otherwise the external ceiling or infinity."""
    human = scalar(human_nm, 'human torque')
    crank = scalar(crank_rad_s, 'crank rate')
    gate = scalar(gate_min_crank_rad_s, 'pedelec gate', minimum=0.)
    cap = math.inf if external_cap_nm is None else scalar(external_cap_nm, 'external motor ceiling', minimum=0.)
    if not isinstance(braking, bool):
        raise ValueError('braking must be a bool')
    return 0. if braking or crank <= gate or human <= 0. else cap
