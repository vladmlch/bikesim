"""Monotone radial tire curves with an exact force-integral elastic energy.

Tables can hold measurements or authored data; provenance is mandatory. Pressure
is metadata for THIS curve, not an unvalidated automatic stiffness multiplier.
There is no extrapolation beyond the last measured/authored deflection.
"""
from bisect import bisect_right
from dataclasses import dataclass
from bike_sim.physics.tire import _scalar, _finite_result


class MaterialRangeError(ValueError):
    """The requested state is outside the supplied material's declared domain."""


@dataclass(frozen=True)
class TabulatedTireSpec:
    deflection_m: tuple[float, ...]
    force_n: tuple[float, ...]
    radial_c_ns_m: float
    pressure_pa_gauge: float
    provenance: str
    valid_load_range_n: tuple[float, float]

    def __post_init__(self):
        try:
            x = tuple(_scalar(v, 'deflection') for v in self.deflection_m)
            y = tuple(_scalar(v, 'elastic force') for v in self.force_n)
        except TypeError as exc:
            raise ValueError('tire curve requires deflection and force sequences') from exc
        if len(x) != len(y) or len(x) < 2:
            raise ValueError('tire curve requires at least two paired samples')
        if x[0] != 0. or y[0] != 0.:
            raise ValueError('tire curve must start at zero deflection and force')
        if any(b <= a for a, b in zip(x, x[1:])):
            raise ValueError('tire deflection must strictly increase')
        if any(b < a for a, b in zip(y, y[1:])) or y[-1] <= 0.:
            raise ValueError('elastic tire force must be nondecreasing and positive at the end')
        for name in ('radial_c_ns_m', 'pressure_pa_gauge'):
            value = _scalar(getattr(self, name), name)
            if value < 0.:
                raise ValueError(f'{name} must be nonnegative')
            object.__setattr__(self, name, value)
        if not isinstance(self.provenance, str) or not self.provenance.strip():
            raise ValueError('tire curve requires explicit provenance')
        try:
            lo, hi = (_scalar(v, 'valid load') for v in self.valid_load_range_n)
        except (TypeError, ValueError) as exc:
            raise ValueError('valid load range requires two finite bounds') from exc
        if not 0. <= lo < hi <= y[-1]:
            raise ValueError('valid load range must fit within the supplied force table')
        slopes = tuple((b-a)/(v-u) for u, v, a, b in zip(x, x[1:], y, y[1:]))
        integral = [0.]
        for u, v, a, b in zip(x, x[1:], y, y[1:]):
            integral.append(integral[-1]+.5*(a+b)*(v-u))
        _finite_result(*slopes, *integral)
        object.__setattr__(self, 'deflection_m', x)
        object.__setattr__(self, 'force_n', y)
        object.__setattr__(self, 'valid_load_range_n', (lo, hi))
        # Derived caches are deliberately not dataclass fields: configuration
        # serialization contains only inputs, never redundant/rounded caches.
        object.__setattr__(self, '_slopes', slopes)
        object.__setattr__(self, '_integral', tuple(integral))

    def elastic_response(self, penetration_m: float) -> tuple[float, float]:
        delta = _scalar(penetration_m, 'tire deflection')
        if delta <= 0.:
            return 0., 0.
        if delta > self.deflection_m[-1]:
            raise MaterialRangeError(
                f'tire deflection {delta:g} m exceeds curve limit {self.deflection_m[-1]:g} m')
        i = min(bisect_right(self.deflection_m, delta)-1, len(self._slopes)-1)
        dx = delta-self.deflection_m[i]
        force = self.force_n[i]+self._slopes[i]*dx
        energy = self._integral[i]+self.force_n[i]*dx+.5*self._slopes[i]*dx*dx
        _finite_result(force, energy)
        return force, energy

    def normal_contact(self, penetration_m: float, penetration_rate_m_s: float) -> tuple[float, float]:
        delta = _scalar(penetration_m, 'tire deflection')
        rate = _scalar(penetration_rate_m_s, 'deflection rate')
        spring, energy = self.elastic_response(delta)
        if delta <= 0.:
            return 0., 0.
        total = spring+self.radial_c_ns_m*rate
        _finite_result(total)
        return max(0., total), energy

    def is_load_in_valid_range(self, load_n: float) -> bool:
        load = _scalar(load_n, 'load_n')
        lo, hi = self.valid_load_range_n
        return lo <= load <= hi
