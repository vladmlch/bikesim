"""Hypothesis-driven fuzzing of the native FFI state-restore parsers.

The property under test: a mutated snapshot is either accepted or rejected
with a CONTRACTED error type — never a crash, a foreign exception, or silent
corruption (a rejected restore must leave the live state untouched). The
seed for every mutation is a genuine drive_state() snapshot, so paths that
reject only wrong widths would otherwise be vacuous.
"""
import copy
import os
from pathlib import Path
import sys

import numpy as np
import pytest

BUILD = Path(__file__).resolve().parents[2] / 'native' / 'build'
selected = os.environ.get('NATIVE_TEST_BUILD_DIR', '')
if selected not in ('', 'asan', 'coverage', 'rtsan'):
    raise ValueError('NATIVE_TEST_BUILD_DIR must be empty or a known build dir')
if selected:
    BUILD /= selected
sys.path.insert(0, str(BUILD))
import bike_native

hypothesis = pytest.importorskip('hypothesis')
from hypothesis import HealthCheck, given, settings, strategies as st

from test_native_drivetrain import pair
from test_native_drive_policies import assert_tree

# The documented FFI error surface: std::invalid_argument -> ValueError,
# nanobind cast failures -> TypeError, shift-count overflow -> OverflowError,
# projection/runtime guards -> RuntimeError. Anything else is a defect.
CONTRACT_ERRORS = (ValueError, TypeError, OverflowError, RuntimeError)

JSON_SCALAR = (st.none() | st.booleans() | st.integers() |
               st.floats(allow_nan=True, allow_infinity=True) |
               st.text(max_size=12))
JSON_VALUE = st.recursive(
    JSON_SCALAR,
    lambda c: st.lists(c, max_size=5) |
              st.dictionaries(st.text(max_size=8), c, max_size=5),
    max_leaves=25)


def dict_paths(obj, prefix=()):
    for k, v in obj.items():
        yield prefix + (k,)
        if isinstance(v, dict):
            yield from dict_paths(v, prefix + (k,))


def apply_path(obj, path, value, delete=False):
    for k in path[:-1]:
        obj = obj[k]
    if delete:
        del obj[path[-1]]
    else:
        obj[path[-1]] = value


def mutate(state, data):
    out = copy.deepcopy(state)
    ops = data.draw(st.lists(st.sampled_from(('set', 'del', 'add')),
                             min_size=1, max_size=4, unique=True))
    paths = list(dict_paths(out))
    for op in ops:
        try:
            if op == 'set':
                path = data.draw(st.sampled_from(paths))
                apply_path(out, path, data.draw(JSON_VALUE))
            elif op == 'del':
                path = data.draw(st.sampled_from(paths))
                apply_path(out, path, delete=True)
            else:
                out[f'extra_{data.draw(st.integers(0, 99))}'] = \
                    data.draw(JSON_VALUE)
        except (KeyError, TypeError):
            continue   # a previous op removed the path this op walked
    return out


@pytest.fixture(scope='module')
def drive_stepper(tmp_path_factory):
    return pair(tmp_path_factory.mktemp('fuzz'))[3]


@given(st.data())
@settings(max_examples=150, deadline=None,
          suppress_health_check=[HealthCheck.too_slow])
def test_drive_state_restore_fuzz(drive_stepper, data):
    n = drive_stepper
    baseline = n.drive_state()
    mutated = mutate(baseline, data)
    try:
        n.set_drive_state(mutated)
    except CONTRACT_ERRORS:
        assert_tree(n.drive_state(), baseline)
    else:
        # Restore the baseline so the next example starts clean — a
        # successful mutation must still leave a restorable object.
        n.set_drive_state(copy.deepcopy(baseline))


@given(st.data())
@settings(max_examples=100, deadline=None,
          suppress_health_check=[HealthCheck.too_slow])
def test_set_state_fuzz(drive_stepper, data):
    n = drive_stepper
    nq, nv = len(n.qpos), len(n.qvel)
    elems = st.floats(allow_nan=True, allow_infinity=True)
    # Most draws are real float64 arrays (reaching the width/finiteness
    # checks); the rest are arbitrary objects probing the cast layer.
    vec = lambda w: data.draw(
        st.lists(elems, min_size=0, max_size=w + 3)
        .map(np.asarray) | JSON_VALUE)
    args = [vec(nq), vec(nv), vec(3), vec(nv), data.draw(JSON_VALUE)]
    try:
        n.set_state(*args)
    except CONTRACT_ERRORS:
        pass
