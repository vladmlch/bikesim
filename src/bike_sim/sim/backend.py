"""Fail-fast backend selection; never replace a requested native run with Python."""
from __future__ import annotations


def require_backend(backend, physics_config, rider):
    if backend not in ('python', 'native'):
        raise ValueError('backend must be python or native')
    if backend == 'python':
        return None
    from bike_sim.native.setup import validate_supported
    from bike_sim.native.artifact import load_native_extension, native_source_fingerprint
    validate_supported(physics_config, rider)
    extension = load_native_extension()
    if extension.runtime_identity()['native_source_sha256'] != native_source_fingerprint():
        raise ValueError('selected extension was not built from the current native source')
    return extension
