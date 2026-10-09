"""Verify a saved research episode by physically replaying every recorded input."""
import argparse
import json
from pathlib import Path
import sys
from bike_sim.sim.research.replay import replay_episode


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('--viewer', action='store_true', help='show the same checked replay in a viewer')
    parser.add_argument('--time-scale', type=int, choices=(1, 2, 4, 8), default=None,
                        help='viewer playback speed; does not change recorded physics')
    args = parser.parse_args(argv)
    if args.time_scale is not None and not args.viewer:
        parser.error('--time-scale requires --viewer')
    try:
        if args.viewer:
            from bike_sim.sim.playground import ensure_macos_mjpython
            ensure_macos_mjpython()
            from bike_sim.sim.research.replay_session import ReplaySession
            from bike_sim.sim.research.replay_viewer import run_replay_viewer
            return run_replay_viewer(ReplaySession(args.recording),
                                     time_scale=1 if args.time_scale is None else args.time_scale)
        print(json.dumps(replay_episode(args.recording), indent=2, allow_nan=False))
        return 0
    except (ValueError, RuntimeError, ArithmeticError, OSError, KeyError, TypeError, ImportError) as exc:
        print(f'REPLAY ERROR: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
