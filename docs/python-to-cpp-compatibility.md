# Python-to-C++ command compatibility map

This table is generated from `migration/command-map.json` (entry status
`native` after T63) and `migration/cli-contracts.json` (`status: native`).
It maps every interpreter-era command surface to the produced C++
executable that replaced it. The Python invocations below are historical
spelling only — no interpreter exists in this tree; they are never
executed and appear here solely so old scripts, notes and muscle memory
can be translated.

## Entrypoints

| Historical interpreter invocation | Native replacement | Module of record |
|---|---|---|
| `uv run bike-sim …` / `python -m bike_sim.cli.main …` | `build/<preset>/bin/bike-sim …` (installed `bike-sim`) | `bike_sim.cli.main` |
| `uv run bike-playground …` / `python -m bike_sim.cli.playground …` | `bike-playground …` | `bike_sim.cli.playground` |
| `uv run bike-export …` / `python -m bike_sim.cli.export …` | `bike-export …` | `bike_sim.cli.export` |
| `uv run bike-ride …` / `python -m bike_sim.cli.ride …` | `bike-ride …` | `bike_sim.cli.ride` |
| `uv run bike-settle …` / `python -m bike_sim.cli.settle …` | `bike-settle …` | `bike_sim.cli.settle` |
| `uv run bike-research …` / `python -m bike_sim.cli.research …` | `bike-research …` | `bike_sim.cli.research` |
| `uv run bike-batch …` / `python -m bike_sim.cli.batch …` | `bike-batch …` | `bike_sim.cli.batch` |
| `uv run bike-replay …` / `python -m bike_sim.cli.replay …` | `bike-replay …` | `bike_sim.cli.replay` |
| `uv run bike-recording …` / `python -m bike_sim.cli.recording …` | `bike-recording …` | `bike_sim.cli.recording` |
| `mjpython …` (macOS GUI re-exec) | run the produced executable directly; the native viewer loads MuJoCo in-process | interpreter shim, deleted |

## Developer tools

| Historical interpreter invocation | Native replacement |
|---|---|
| `python tools/golden_episode.py …` | `bike-recording golden freeze …` |
| `python tools/episode_compare.py …` | `bike-recording golden compare …` |
| `python tools/measure_realtime.py …` | `bike-measure-realtime …` |
| `python -m bike_sim.validation.benchmarks …` | `bike-validate …` |
| `python tools/research_sweep.py …` | `bike-research-sweep …` |
| `python tools/native_checks.py` / `tools/dev.py check --group native` | `build/<preset>/bin/bike-native-checks --kind <kind> --build build/<preset>` or `bash tools/run_native_checks.sh build/<preset>` |
| `uv run pytest …` | `ctest --test-dir build/<preset> …` (the gate profiles in `tools/run_tests.sh`) |
| `python tools/check_native_contracts.py` / `tools/check_offline_sources.py` | covered by the produced `bike-native-checks` kinds and the T56 `bike-python-audit` source/documentation/package modes |
| `python -m pip` / `uv pip` / `.venv` setup | no equivalent: install is a CPack package or a source build via `cmake --workflow --preset <preset>` |

## Removed flags and environment variables

| Interpreter-era surface | Post-T63 behavior |
|---|---|
| `--backend python` ride flag | removed; there is exactly one physics backend |
| `BIKE_NATIVE_BUILD_PATH` | retired with the binding build; the build dir is the CMake preset's |
| `NATIVE_TEST_BUILD_PATH` | retired; tests address `build/<preset>` directly |
| `UV_PROJECT_ENVIRONMENT`, `VIRTUAL_ENV`, `.venv` | retired; no interpreter environment exists |
| `tools/dev.py bootstrap …` | retired; `tools/extract_deps.sh` fetches `.deps` and `cmake --preset <preset>` configures |
| research `module:factory` policy strings | rejected with an actionable native-policy error — never imported; use `builtin:<name>` or `plugin:/absolute/path` (docs/NATIVE_POLICIES.md) |

## Config and output compatibility

- Simulation, track, profile and experiment TOML/JSON inputs keep their
  documented schemas; the native readers are the same contracts the
  transition tests pinned.
- Recordings carry their recorded execution identity; replay checks the
  recorded source/runtime/native-artifact identity against the current
  build rather than assuming cross-era compatibility
  (docs/RECORDING.md).
- Golden fixtures under `tests/fixtures/golden/<platform>/` are the
  immutable post-ADR-0003 reference; see
  docs/reference/README-provenance.md.
