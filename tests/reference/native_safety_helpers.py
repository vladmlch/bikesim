"""Owning fixtures and bit-preserving snapshots for native safety contracts."""
import numpy as np


def drive_pair(tmp_path, **kwargs):
    from test_native_drivetrain import pair
    return pair(tmp_path, **kwargs)


def suspension_pair(tmp_path):
    from test_native_suspension import _golden
    from tools.native_config import project
    from native_loader import load_native
    env, directory = _golden(tmp_path, steps=1)
    config = project(env)
    return env, load_native().Stepper(str(directory / 'model.mjb'), config), directory, config


def freeze(value):
    if isinstance(value, np.ndarray):
        return value.dtype.str, value.shape, value.tobytes()
    if isinstance(value, dict):
        return tuple((key, freeze(item)) for key, item in value.items())
    if isinstance(value, (list, tuple)):
        return tuple(map(freeze, value))
    if isinstance(value, float):
        return np.float64(value).tobytes()
    return value
