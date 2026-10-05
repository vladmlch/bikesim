"""Native mj_step is bitwise-identical to Python mj_step — same dylib."""
import os
import sys
from pathlib import Path

import mujoco
import pytest

BUILD = Path(__file__).resolve().parents[2] / 'native' / 'build'
selected = os.environ.get('NATIVE_TEST_BUILD_DIR', '')
if selected not in ('', 'asan', 'coverage', 'rtsan'):
    raise ValueError('NATIVE_TEST_BUILD_DIR must be empty or a known build dir')
if selected:
    BUILD /= selected
sys.path.insert(0, str(BUILD))
MJB = 'tools/proto_native_bench/artifacts/model.mjb'

def test_native_step_matches_python_bitwise():
    bike_native = pytest.importorskip('bike_native')
    native = bike_native.Stepper(MJB)
    ref = mujoco.MjModel.from_binary_path(MJB)
    data = mujoco.MjData(ref)
    mujoco.mj_forward(ref, data)
    for _ in range(200):
        native.step(); mujoco.mj_step(ref, data)
    from _bits import assert_bitwise_equal
    assert_bitwise_equal(native.qpos, data.qpos, 'qpos')
    assert_bitwise_equal(native.qvel, data.qvel, 'qvel')
