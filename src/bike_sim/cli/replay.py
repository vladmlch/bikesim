"""Verify a saved research episode by physically replaying every recorded input."""
import argparse
import json
from pathlib import Path
import sys
from bike_sim.sim.research.replay import replay_episode


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    args = parser.parse_args(argv)
    try:
        print(json.dumps(replay_episode(args.recording), indent=2, allow_nan=False))
        return 0
    except (ValueError, RuntimeError, ArithmeticError, OSError, KeyError) as exc:
        print(f'REPLAY ERROR: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
