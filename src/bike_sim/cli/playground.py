"""
CLI Entrypoint for the Interactive Suspension Test Stand Playground.
"""

import argparse
from bike_sim.sim.playground import run_interactive_playground


def main() -> None:
    """CLI Entrypoint for the test stand playground runner."""
    parser = argparse.ArgumentParser(description="2D Mountain Bike Suspension Test Stand Playground in MuJoCo.")
    args = parser.parse_args()
    run_interactive_playground()


if __name__ == "__main__":
    main()
