"""Continuous road grade, independent of local obstacles (grade = dz/dx)."""
from dataclasses import dataclass
from math import isfinite
from numbers import Real
import numpy as np


@dataclass(frozen=True)
class GradeProfile:
    """Integrate a piecewise-linear grade into a C1 elevation profile.

    Knots are (station_m, dimensionless_grade), starting at station zero.
    Elevation is zero at the origin. Outside the knots the endpoint grade is
    held. A flat lead-in/lead-out therefore gives flat off-track margins.
    A grade of .15 means 15 percent, not 15 degrees. No analytic slope is fed
    to the tire: dynamics still uses the compiled heightfield cross-section.
    """
    knots: tuple[tuple[float, float], ...]

    def __post_init__(self):
        try:
            raw = tuple(tuple(p) for p in self.knots)
            if len(raw) < 2 or any(len(p) != 2 for p in raw):
                raise ValueError('grade needs at least two station/grade pairs')
            if any(isinstance(v, bool) or not isinstance(v, Real) or not isfinite(v)
                   for p in raw for v in p):
                raise ValueError('grade knots must be finite real numbers')
            knots = tuple((float(x), float(g)) for x, g in raw)
            if knots[0][0] != 0. or any(b[0] <= a[0] for a, b in zip(knots, knots[1:])):
                raise ValueError('grade stations must start at zero and strictly increase')
        except TypeError as exc:
            raise ValueError('grade knots must be station/grade pairs') from exc
        object.__setattr__(self, 'knots', knots)

    def slope(self, x_m):
        x = np.asarray(x_m, dtype=float)
        if not np.isfinite(x).all():
            raise ValueError('grade query must be finite')
        stations, grades = np.asarray(self.knots).T
        value = np.interp(x, stations, grades)
        return float(value) if x.ndim == 0 else value

    def elevation(self, x_m):
        x = np.asarray(x_m, dtype=float)
        if not np.isfinite(x).all():
            raise ValueError('grade query must be finite')
        stations, grades = np.asarray(self.knots).T
        h = np.r_[0., np.cumsum(np.diff(stations)*(grades[:-1]+grades[1:])*.5)]
        index = np.clip(np.searchsorted(stations, x, side='right')-1, 0, len(stations)-2)
        dx = x-stations[index]
        change = (grades[index+1]-grades[index])/(stations[index+1]-stations[index])
        value = h[index]+grades[index]*dx+.5*change*dx*dx
        value = np.where(x < 0., grades[0]*x, value)
        value = np.where(x > stations[-1], h[-1]+grades[-1]*(x-stations[-1]), value)
        return float(value) if x.ndim == 0 else value
