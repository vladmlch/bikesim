# bike-sim

Bicycle suspension/kinematics, physical ride simulation, torque-control
research and checked replay. Python code is in `src/bike_sim`; the native
runtime is in `native`.

## Documentation

- [Physical ride, research and replay](docs/RIDE.md)
- [Run recording, SQL analysis and replay formats](docs/RECORDING.md)
- [Test profiles and native tooling](docs/TESTING.md)
- [Engineering instructions](AGENTS.md)

The command entrypoints include `bike-ride`, `bike-research`, `bike-replay`
and `bike-recording`. Persistent physical/research recording is opt-in with
`--record`; `--out` alone does not retain or publish a run. Recording keeps
all episodes in RAM and publishes once when the launch ends.
