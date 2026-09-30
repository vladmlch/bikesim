"""Cohesive hand release, accounting only previously stored interface energy."""
import numpy as np


def release_if_overloaded(force_n, old_energy_j: float, limit_n: float):
    force=np.asarray(force_n,dtype=float)
    if force.shape!=(3,) or not np.isfinite(force).all():
        raise ValueError('grip force must be a finite 3-vector')
    if not np.isfinite(old_energy_j) or old_energy_j<0:
        raise ValueError('invalid stored grip energy')
    if not np.isfinite(limit_n) or limit_n<=0:
        raise ValueError('grip limit must be positive')
    if np.linalg.norm(force)>limit_n:
        return np.zeros(3),float(old_energy_j),True
    return force.copy(),0.,False
