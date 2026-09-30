"""Potential-based circumferential quadrature, an experimental 2-D tire model.

Stiffness is a density per radian, not a complete tire's radial stiffness.
Coefficients can be fitted to an aggregate load-deflection experiment; a fit
that misses its declared error gate remains explicitly rejected.
"""
from dataclasses import dataclass, field, asdict
import numpy as np


@dataclass(frozen=True)
class HingeDensity:
    knots_m: tuple[float, ...] = (0., .004, .012)
    stiffness_n_m: tuple[float, ...] = (300000., 0., 0.)
    damping_ns_m: float = 2000.
    provenance: str = 'synthetic density; not fitted or measured'

    def __post_init__(self):
        a, k = np.asarray(self.knots_m, float), np.asarray(self.stiffness_n_m, float)
        if a.ndim != 1 or len(a) == 0 or a.shape != k.shape:
            raise ValueError('invalid density table shape')
        if not np.isfinite([*a, *k, self.damping_ns_m]).all():
            raise ValueError('density parameters must be finite')
        if a[0] != 0 or np.any(np.diff(a) <= 0) or np.any(k < 0) or self.damping_ns_m < 0:
            raise ValueError('density law must be monotone and dissipative')
        if not isinstance(self.provenance, str) or not self.provenance.strip():
            raise ValueError('material provenance is required')
        object.__setattr__(self, 'knots_m', tuple(float(v) for v in a))
        object.__setattr__(self, 'stiffness_n_m', tuple(float(v) for v in k))


@dataclass(frozen=True)
class DistributedTireConfig:
    station_count: int = 128
    density: HingeDensity = field(default_factory=HingeDensity)
    fitting_dataset_id: str = 'none-synthetic'
    valid_load_range_n: tuple[float, float] = (0., 2000.)
    valid_deflection_range_m: tuple[float, float] = (0., .04)
    calibration_status: str = 'experimental_unvalidated'

    def __post_init__(self):
        station_angles(self.station_count)
        if not isinstance(self.density, HingeDensity):
            raise ValueError('distributed tire needs an explicit density law')
        for field_name in ('valid_load_range_n', 'valid_deflection_range_m'):
            v = tuple(float(x) for x in getattr(self, field_name))
            if len(v) != 2 or not np.isfinite(v).all() or not 0 <= v[0] < v[1]:
                raise ValueError('invalid distributed tire applicability range')
            object.__setattr__(self, field_name, v)
        if not self.fitting_dataset_id.strip():
            raise ValueError('a fitting dataset ID or explicit synthetic ID is required')
        # Status is produced by evidence, never user-upgraded via a TOML label.
        if self.calibration_status != 'experimental_unvalidated':
            raise ValueError('distributed tire acceptance is evaluated in the validation report')


def station_angles(count: int):
    if isinstance(count, bool) or not isinstance(count, int) or count < 16:
        raise ValueError('at least sixteen tread stations required')
    return 2*np.pi*np.arange(count)/count, np.full(count, 2*np.pi/count)


def density_response(delta, material: HingeDensity):
    delta = np.asarray(delta, dtype=float)
    if not np.isfinite(delta).all():
        raise ValueError('contact penetration must be finite')
    x = np.maximum(delta[..., None]-np.asarray(material.knots_m), 0.)
    k = np.asarray(material.stiffness_n_m)
    return np.sum(k*x, axis=-1), .5*np.sum(k*x*x, axis=-1)


def normal_station_response(delta, delta_rate, weights, material: HingeDensity):
    delta, rate, w = np.broadcast_arrays(delta, delta_rate, weights)
    if not (np.isfinite(delta).all() and np.isfinite(rate).all() and np.isfinite(w).all()) or np.any(w < 0):
        raise ValueError('invalid contact quadrature')
    elastic, energy = density_response(delta, material)
    force = np.where(delta > 0, np.maximum(0., elastic+material.damping_ns_m*rate), 0.)
    return w*force, w*energy


def flat_load(deflection_m, radius_m, material: HingeDensity, station_count=512, phase_rad=0.):
    if not np.isfinite(radius_m) or radius_m <= 0:
        raise ValueError('tire radius must be positive')
    angles, weights = station_angles(station_count)
    penetration = np.asarray(deflection_m)[..., None]-radius_m*(1.+np.sin(angles+phase_rad))
    force, energy = normal_station_response(penetration, 0., weights, material)
    return np.sum(force, axis=-1), np.sum(energy, axis=-1)


def fit_density(deflections_m, loads_n, radius_m, *, knots_m=(0., .004, .012),
                station_count=512, dataset_id, damping_ns_m=0., max_relative_error=.05):
    from scipy.optimize import nnls
    d, loads = np.asarray(deflections_m, float), np.asarray(loads_n, float)
    if (d.ndim != 1 or d.shape != loads.shape or len(d) < 3 or not np.isfinite(d).all()
            or not np.isfinite(loads).all() or np.any(d <= 0) or np.any(np.diff(d) <= 0) or np.any(loads <= 0)):
        raise ValueError('fit needs ordered positive deflections and positive aggregate loads')
    if not isinstance(dataset_id, str) or not dataset_id.strip():
        raise ValueError('a fitting dataset ID is required')
    if not np.isfinite(max_relative_error) or not 0 < max_relative_error < 1:
        raise ValueError('invalid fitting error gate')
    columns = []
    for i in range(len(knots_m)):
        k = tuple(1. if i == j else 0. for j in range(len(knots_m)))
        columns.append(flat_load(d, radius_m, HingeDensity(knots_m, k, 0., 'unit basis'), station_count)[0])
    basis = np.array(columns).T
    coefficients, _ = nnls(basis/loads[:, None], np.ones(len(loads)))
    density = HingeDensity(tuple(knots_m), tuple(coefficients), damping_ns_m, 'fit: '+dataset_id)
    predicted = basis@coefficients
    errors = np.abs(predicted-loads)/loads
    return density, {'dataset_id': dataset_id, 'radius_m': float(radius_m), 'station_count': station_count,
        'max_relative_error': float(errors.max()), 'rms_relative_error': float(np.sqrt(np.mean(errors**2))),
        'error_gate': max_relative_error, 'accepted': bool(errors.max() <= max_relative_error),
        'target_load_n': loads.tolist(), 'predicted_load_n': predicted.tolist(),
        'deflection_m': d.tolist(), 'density': asdict(density),
        'calibration_status': 'synthetic_fit_only' if 'synthetic' in dataset_id.lower() else 'fit_only_not_validated'}
