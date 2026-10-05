"""Native mj_step is bitwise-identical to Python mj_step — same dylib."""
import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'native' / 'build'))
MJB = 'tools/proto_native_bench/artifacts/model.mjb'

def test_native_step_matches_python_bitwise():
    bike_native = pytest.importorskip('bike_native')
    native = bike_native.Stepper(MJB)
    ref = mujoco.MjModel.from_binary_path(MJB)
    data = mujoco.MjData(ref)
    mujoco.mj_forward(ref, data)
    for _ in range(200):
        native.step(); mujoco.mj_step(ref, data)
    assert np.array_equal(native.qpos, data.qpos)   # bitwise, not isclose
    assert np.array_equal(native.qvel, data.qvel)
