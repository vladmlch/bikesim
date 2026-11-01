# bike-sim

Bicycle suspension/kinematics, physical ride simulation, torque-control
research and checked replay — a native C++ product (the Python runtime was
removed at T63; `native/` holds the product and its migration evidence).

## Development commands

`bash tools/run_tests.sh` is the test entry point — `quick`, `native` and
`full` profiles drive the root CMake build (configure presets live in
`CMakePresets.json`; every dependency resolves from the locked `.deps`
tree).

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
