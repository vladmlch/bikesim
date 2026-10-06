"""Native mj_step is bitwise-identical to Python mj_step — same dylib."""
import os
import sys
from pathlib import Path

import mujoco
import pytest

from native_loader import load_native
bike_native = load_native()
MJB = 'tools/proto_native_bench/artifacts/model.mjb'

def test_native_step_matches_python_bitwise():
    native = bike_native.Stepper(MJB)
    ref = mujoco.MjModel.from_binary_path(MJB)
    data = mujoco.MjData(ref)
    mujoco.mj_forward(ref, data)
    for _ in range(200):
        native.step(); mujoco.mj_step(ref, data)
    from _bits import assert_bitwise_equal
    assert_bitwise_equal(native.qpos, data.qpos, 'qpos')
    assert_bitwise_equal(native.qvel, data.qvel, 'qvel')
