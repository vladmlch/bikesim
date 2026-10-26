"""One-off Python oracle capture for completion-plan task 3.20.

Post-F2 recapture of pinned_savage_40: forty direct physical steps with
human_torque_nm = 35 over the shared pinned recipe, PLUS wire.json — the
tools.native_config.project(env) dict the C++ ports' Stepper decodes.

The authoritative producer is the in-test hook
tests/reference/test_native_brake_resistance.py::_fixture_save_episode,
run via

    BIKE_GOLDEN_CAPTURE=tests/fixtures/golden \
        uv run pytest tests/reference/test_native_brake_resistance.py \
                      tests/reference/test_native_suspension.py

This standalone script exists for provenance parity with
pinned_savage_40/freeze.py; run explicitly from the repository root on
each supported platform after execution is authorized. This source file
is not a captured artifact. Deleted with the Python capture machinery in
task 4.05.
"""
import json
import sys
from pathlib import Path
from tempfile import TemporaryDirectory

sys.path[:0] = [".", "tests/reference"]

import _golden
from bike_sim.cli import research as research_cli
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.research.recording_lifecycle import close_environment
from tools.golden_episode import capture_episode, save
from tools.native_config import project


def main():
    fixture = Path("tests/fixtures/golden/episode/pinned_savage_40_f2")
    output = (
        Path("tests/fixtures/golden") / _golden.platform_key()
        / "episode" / "pinned_savage_40_f2"
    )
    with TemporaryDirectory(prefix="bike-golden-python-") as temporary:
        args = research_cli.parser().parse_args([
            "--backend", "python", "--seed", "0",
            "--physics-config", str(fixture / "physics.toml"),
            "--track-file", "examples/research/rough_uphill_savage.toml",
            "--duration", "1", "--dt", ".00125", "--diagnostic-model-limits",
            "--out", str(Path(temporary) / "out"),
        ])
        env = research_cli.make_environment(args)
        primary = None
        try:
            episode = capture_episode(env, 40, RideControl(human_torque_nm=35.))
            save(episode, output, env.sim.model)
            (output / "wire.json").write_text(
                json.dumps(project(env), sort_keys=True) + "\n")
        except BaseException as error:
            primary = error
            raise
        finally:
            close_environment(env, primary)


if __name__ == "__main__":
    main()
