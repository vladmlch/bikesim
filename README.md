# bike-sim

Bicycle suspension/kinematics, physical ride simulation, torque-control research
and checked replay. Python code is in `src/bike_sim`; the native runtime is in
`native`.

## Documentation

- [Physical ride, research and replay](docs/RIDE.md)
- [Run recording, SQL analysis and replay formats](docs/RECORDING.md)
- [Test profiles and native tooling](docs/TESTING.md)
- [Engineering instructions](AGENTS.md)

The command entrypoints include `bike-ride`, `bike-research`, `bike-replay` and
`bike-recording`. Persistent physical/research recording is opt-in with `--record`;
`--out` alone does not retain or publish a run. Recording keeps all episodes in RAM
and publishes once when the launch ends.

This repomix-derived delivery contains source changes, tests and documentation.
No project compilation, tests, simulation, GUI or timing was executed. The
[delivery ledger](docs/superpowers/audits/2026-10-13-run-recording-source-completion.md)
separates implementation from the still-open acceptance gates. Restore ignored
assets, the dependency lock and a matching native artifact from the complete
checkout before following native execution examples; they are not manufactured
by unpacking a text-only repository snapshot.
