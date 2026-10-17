"""
CLI Entrypoint for the Interactive Suspension Test Stand Playground.
"""

import argparse
import sys
from bike_sim.sim.playground import run_interactive_playground


def main(argv=None) -> int:
    """CLI Entrypoint for the test stand playground runner."""
    parser = argparse.ArgumentParser(description="2D Mountain Bike Suspension Test Stand Playground in MuJoCo.")
    parser.parse_args(argv)
    from bike_sim.viewer_runtime import ViewerUnavailableError, prepare_viewer
    try:
        prepare_viewer(argv=argv, module='bike_sim.cli.playground')
        run_interactive_playground()
    except ViewerUnavailableError as error:
        print(f'PLAYGROUND ERROR: {error}', file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
