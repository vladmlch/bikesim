"""One-off Python oracle capture for completion-plan task 3.03.

Run explicitly from the repository root on each supported platform after
execution is authorized. This source file is not a captured artifact.
Deleted with the Python capture machinery in task 4.05.
"""
import sys
from pathlib import Path
from tempfile import TemporaryDirectory

sys.path[:0] = [".", "tests/reference"]

import _golden
from bike_sim.cli import research as research_cli
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.research.recording_lifecycle import close_environment
from tools.golden_episode import capture_episode, save


def main():
    fixture = Path("tests/fixtures/golden/episode/pinned_savage_40")
    output = (
        Path("tests/fixtures/golden") / _golden.platform_key()
        / "episode" / "pinned_savage_40"
    )
    if output.exists():
        raise FileExistsError(f"refusing to replace existing oracle artifact: {output}")
    # Pin the oracle backend and the existing parser default seed=0
    # explicitly: request.toml uses exactly seed_decimal="0". The CLI's
    # incidental output is private to this capture, not a shared /tmp path.
    with TemporaryDirectory(prefix="bike-golden-python-") as temporary:
        args = research_cli.parser().parse_args([
            "--backend", "python", "--seed", "0",
            "--physics-config", str(fixture / "physics.toml"),
            "--track-file", "examples/research/rough_uphill_savage.toml",
            "--duration", "1", "--dt", ".00125", "--diagnostic-model-limits",
            "--out", str(Path(temporary) / "out"),
        ])
        env = research_cli.make_environment(args, capture_mode="legacy")
        primary = None
        try:
            episode = capture_episode(env, 40, RideControl(human_torque_nm=35.))
            save(episode, output, env.sim.model)
        except BaseException as error:
            primary = error
            raise
        finally:
            close_environment(env, primary)


if __name__ == "__main__":
    main()
