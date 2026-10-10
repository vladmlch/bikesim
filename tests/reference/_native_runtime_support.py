"""Shared helpers for native-runtime reference tests.

These helpers execute fresh scalar Python intervals and read independently
captured output; they do not reconstruct native results from the same force
matrix.
"""

from collections.abc import Mapping
from pathlib import Path
import math

from bike_sim.cli import ride
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.ride.physical_samples import plain

PHYSICS = Path("examples/research/viewer_physics_welded.toml")
TRACK = Path("examples/research/rough_uphill_savage.toml")


def make_python_ride(*, strict=False, decimation=1):
    args = ride.parse_args([
        "--physics-config", str(PHYSICS), "--track", str(TRACK),
        "--headless", "--duration", "1", "--no-plots",
    ])
    sim = ride.build_physical_simulation_from_args(args)
    sim.physical.set_strict(strict)
    sim.physical.set_record_decimation(decimation)
    return sim


def advance_python(sim, steps, control=None, *, front=0.0, rear=0.0):
    command = RideControl() if control is None else control
    rows = []
    cursor = -1
    for _ in range(steps):
        sim.step(front, rear, control=command)
        for sample in sim.physical.completed_samples:
            if sample.interval_id > cursor:
                rows.append(sample.as_dict())
                cursor = sample.interval_id
    sim.physical.flush()
    for sample in sim.physical.completed_samples:
        if sample.interval_id > cursor:
            rows.append(sample.as_dict())
            cursor = sample.interval_id
    return rows


def make_python_research(*, duration=0.1, seed=210):
    from bike_sim.cli import research
    args = research.parser().parse_args([
        "--physics-config", str(PHYSICS), "--track-file", str(TRACK),
        "--duration", str(duration), "--seed", str(seed),
        "--diagnostic-model-limits", "--assist",
    ])
    return research.make_environment(args)


def assert_tree_close(actual, expected, *, atol=1e-9, rtol=1e-9, path="root"):
    actual, expected = plain(actual), plain(expected)
    if isinstance(expected, Mapping):
        assert isinstance(actual, Mapping) and actual.keys() == expected.keys(), path
        for key in expected:
            assert_tree_close(actual[key], expected[key], atol=atol, rtol=rtol,
                              path=f"{path}.{key}")
    elif isinstance(expected, (list, tuple)):
        assert isinstance(actual, (list, tuple)) and len(actual) == len(expected), path
        for index, value in enumerate(expected):
            assert_tree_close(actual[index], value, atol=atol, rtol=rtol,
                              path=f"{path}[{index}]")
    elif isinstance(expected, bool) or expected is None or isinstance(expected, str):
        assert type(actual) is type(expected) and actual == expected, path
    elif isinstance(expected, (int, float)):
        assert not isinstance(actual, bool) and isinstance(actual, (int, float)), path
        assert math.isfinite(actual) and math.isfinite(expected), path
        assert math.isclose(actual, expected, abs_tol=atol, rel_tol=rtol), path
    else:
        raise AssertionError(f"unhandled comparison type at {path}: {type(expected)}")
