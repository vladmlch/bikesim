# bike-sim

Bicycle suspension/kinematics, physical ride simulation, torque-control
research and checked replay. Python code is in `src/bike_sim`; the native
runtime is in `native`.

## Development commands

`tools/dev.py` is the explicit cross-platform development driver:
`bootstrap`, `configure`, `build`, `doctor` and `test` (the last delegates
the quick/native/full profiles to `tools/run_tests.sh`). All commands run
through the frozen uv environment, e.g.:

```sh
uv run --frozen --group native --no-sync python tools/dev.py --help
```

Measured development hosts today are **macOS arm64** and **Linux aarch64**
(each guide records its qualified toolchain); other architectures and Linux
x86_64 remain unqualified until their own gates pass. Follow the platform
guide for the host you are on:

- [Linux development guide](docs/development/linux.md)
- [macOS development guide](docs/development/macos.md)
- [Failure diagnosis](docs/development/troubleshooting.md)

## Documentation

- [Physical ride, research and replay](docs/RIDE.md)
- [Run recording, SQL analysis and replay formats](docs/RECORDING.md)
- [Test profiles and native tooling](docs/TESTING.md)
- [Installation modes: Python, source-native, offline](docs/development/install-modes.md)
- [Dependency wheel matrix](docs/development/dependency-matrix.md) and
  [numerical provider profiles](docs/development/numerical-profiles.md)
- [Engineering instructions](AGENTS.md)

The command entrypoints include `bike-ride`, `bike-research`, `bike-replay`
and `bike-recording`. Persistent physical/research recording is opt-in with
`--record`; `--out` alone does not retain or publish a run. Recording keeps
all episodes in RAM and publishes once when the launch ends.
