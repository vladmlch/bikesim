# Native recording and application layer

This document describes the source implementation of T38–T51 delivered on
2026-10-17. Commands below are usage contracts for a later built installation.
**They were not executed in this delivery.** Compilation, tests, simulation and
GUI execution were expressly prohibited by the user.

## Command inventory

| Command | Purpose |
|---|---|
| `bike-sim` | Kinematics tables, geometry/damper plots, model exports and optional playground |
| `bike-export` | Native geometry JSON and playground MJCF exports |
| `bike-playground` | Native interactive playground |
| `bike-ride` | Legacy, physical or research ride frontend |
| `bike-settle` | Native equilibrium solve and reusable state artifact |
| `bike-research` | Research episodes with fixed commands or the native policy owner |
| `bike-recording` | Standalone read-only recording analyst |
| `bike-replay` | Checked re-simulation or stored-state inspection |
| `bike-batch` | TOML grids and explicit research jobs |
| `bike-research-sweep` | Decimal-seed sweep with native process ownership |

The source backend is native. A Python backend spelling is rejected; it never
causes interpreter re-execution. `bike-recording` has its own small link graph
and does not initialize a model, policy or display.

## Recording lifecycle

Physical and research recording is opt-in through `--record`. Giving `--out`
sets a destination but does not enable recording. Without recording, no growing
capture or preview artifact is retained. Legacy ride keeps its separate legacy
artifact behavior.

Recording retains immutable evidence in RAM while a launch runs. Reset seals
the previous episode in memory and creates another episode identity. Only final
publication creates the durable run directory. `manifest.json` is the final
sealing marker; a partial directory without that marker is not a recording.

The sealed directory contains `run.duckdb`, a manifest of closed-file hashes
and row counts, and a successful summary report when report generation succeeds.
The database contains typed physical/research streams, metadata, event provenance,
coordinate/alias catalogs, integration state and captured byte assets. Summary
generation failure is represented explicitly; it need not invalidate the database.

Capture is not crash recovery. Killing a process before finalization can lose
the retained RAM evidence. In-process publication failures keep evidence for a
retry; a later successful finalization is idempotent.

## Analyst operations

The analyst supports `overview`, `fields`, `events`, `window`, `compare`, `sql`
and `export`. It first validates the manifest, payload hashes, database shape and
relationships. Normal queries are bounded; explicit CSV/JSONL exports stream
the selected rows. Large integer values are not converted through binary64.
Null, absence and nonfinite values keep their distinct representations.

Examples for a later installation:

```text
bike-recording overview PATH --json
bike-recording fields PATH --limit 200
bike-recording events PATH --episode 0 --kind episode_ended
bike-recording window PATH --episode 0 --start 0 --end 1 --fields x_m,powers_w.chain
bike-recording sql PATH --query "SELECT episode_id, count(*) FROM physics GROUP BY episode_id"
bike-recording export PATH --episode 0 --table metrics --format jsonl --output metrics.jsonl
```

The SQL entry point accepts a single read-only statement. Output failures do
not mutate the source recording. Explicit exports refuse an existing output
file instead of silently truncating it.

## Replay and display

`bike-replay PATH` performs checked replay. `--viewer` adds visualization to
that verification. Completion means the final state/trace checks succeeded;
closing early remains incomplete. `--view` inspects saved states, without
stepping the physics. It supports seeking and reverse/zero/forward playback
speeds independently of checked-replay time scaling.

The replay contract checks truthful native execution, source and numerical
provider identities. A historical Python/extension identity is not treated as
the identity of a native executable. Stored-state playback can still be useful
when checked re-simulation is incompatible.

The renderer owns a separate model/data copy. Input and graphics live on the
main thread. Physical Space toggles braking; research/replay Space pauses.
Reset, speed, camera and model-specific tuning controls follow the displayed
help. The headless build does not initialize GLFW.

## Plots and optional dependencies

PNG and SVG generation consume native typed plot data. HTML is self-contained
and can be opened offline. The default `native-html-v1` backend offers hover,
legend toggles, pan/zoom and exports. Its backend identity is explicit. A real
local Plotly bundle may be supplied through `bike::viz::set_plotly_resource()`;
the bundle is not included in this source snapshot.

The built-in PNG path has no display dependency. A separately acquired and
explicitly fingerprinted Cairo 1.18.4 SDK can supply Unicode raster text.
`native/dependencies.md` describes the flags and unresolved packaging gates.
No plot was generated or visually checked in this delivery.

## Batch specification and outputs

Batch flags are the research command's long flags. Relative file inputs keep
the parent process's working directory as their base. `out` is reserved because
the batch assigns a separate destination to every job.

```toml
[base]
scenario = "rough_uphill"
duration = 3.0
record = true

[grid]
seed = { start = 0, count = 4 }
rider-mass = [60.0, 80.0]

[[job]]
seed = 12
rider-random = true
```

Grid order and explicit-job order determine IDs before scheduling. The job
arguments are validated before processes launch. `--dry-run` only prints the
expanded plan. `--serial` runs one isolated native child at a time; `--jobs`
sets parallelism. Every invocation passes an argv array directly, without shell
evaluation. The results JSONL is written incrementally, and per-job stdout and
stderr logs retain diagnostics. Interrupting the batch cancels queued jobs and
reaps active child groups before the parent returns.

Seed values and integer ranges preserve decimal identity without binary64 or
int64 narrowing. The expansion is bounded to one million jobs/visited seeds.
Existing nonempty outputs require an explicit supported overwrite choice;
sealed run directories are never silently adopted or replaced.

## Qualification state

The registered tests are `migration_T38` through `migration_T51`; their source
bodies are present but unrun. `bike-test-child` is a bounded test-only helper.
The implementation, source-card mappings, explicit compatibility differences
and pending gates are recorded in:

- `migration/recording-application-implementation.json`
- `migration/recording-application-review.md`
- `migration/recording-application-static-audit.json`

No clean build, runtime parity, database round trip, successful replay, graphical
result or platform acceptance is implied by source registration. Old Python
source remains available until the original T63 deletion gates are satisfied.
