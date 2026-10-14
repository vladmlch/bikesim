# Run Recording Implementation Plan

> **2026-10-13 delivery:** ordinary and research source changes and regression sources are supplied. See the [task/R01-R24 source-completion ledger](../audits/2026-10-13-run-recording-source-completion.md) and [usage/migration guide](../../RECORDING.md). No builds, imports, tests, analyzers, simulator/GUI runs or benchmarks were executed. Execution-dependent completion checkboxes below remain open; this is not an accepted A6/B4 runtime gate. Stage-B source was completed under the no-execution instruction, not after a claimed green stage-A run.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce opt-in, full-step DuckDB recordings that agents can inspect after ordinary native and research launches.

**Architecture:** One launch owner retains owned episodes and input events in RAM. Finalization constructs and seals a DuckDB recording plus a deterministic report. Ordinary and research adapters share storage/analysis while research keeps its sensor, command, acknowledgement and checked-replay semantics.

**Tech Stack:** Python >=3.13 through uv, existing C++23/nanobind/MuJoCo 3.12.0 runtime, DuckDB 1.5.6, PyArrow 26.0.0, NumPy, pytest and the repository verification launcher.

**Spec:** [Run recording design](../specs/2026-10-12-run-recording-design.md). Read it before either stage. Its R01–R24 contracts are authoritative.

## Global Constraints

- Invoke Python and Python tools through `uv`.
- Use Python >=3.13, C++23, nanobind, and MuJoCo 3.12.0 for native execution.
- Preserve valid-input binary64 operation order, physics timestep, controller/sensor scheduling, and existing comparison tolerances.
- Store physical samples at every accounted physics interval when recording is enabled; do not downsample or quantize the canonical recording.
- Collect the whole launch in RAM and write only during launch finalization.
- Make persistent recording opt-in with `--record`, default false.
- Keep database access, Python boxing, report generation, and disk I/O outside realtime kernels.
- Preserve user-owned changes, unrelated plans, skills, and output; do not delete or overwrite an existing run.

## Execution scope and baseline

Checkout: `/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/cpp-port-p2`.
Planning inspected HEAD `c5dafe06b5ce4c4f0b5020586eeec40a6973cad3`; tracked changes were absent, and unrelated untracked skills/docs/output existed. Earlier conversation inspections used an older HEAD. Re-establish facts at execution, including native engineering rules and the active selected artifact. This planning session creates documents only; scratch numerical storage timing is not simulator/test verification.

The user approved DuckDB, opt-in recording, whole-launch RAM retention and end-only publication. The user explicitly replaced earlier crash-prefix/periodic-write suggestions. Do not reintroduce a periodic writer or require flushing after viewer reset.

| Stage | Open when | Deliverable |
|---|---|---|
| [A: ordinary native and shared recording](2026-10-12-run-recording-ordinary.md) | Starting implementation | Capture/storage schema, ordinary native viewer/headless, report and analysis CLI |
| [B: research and replay](2026-10-12-run-recording-research.md) | A1–A6 accepted | Both research backends/entrypoints, deduplicated retention/export and old/new checked replay |

Implement substantial tasks sequentially through shared runtime files. Follow the user's current delegation constraints; this plan does not mandate fan-out or a new worktree. Reuse this existing isolated checkout. Physical tests and benchmarks run sequentially. Each task ends with a reviewable checkpoint; stage gates require their named verification evidence. Checkpoints are narrowly scoped commits when committing is authorized, otherwise reviewable diffs. Never include unrelated user files.

## A0. Establish implementation baseline

**Files:** Read `AGENTS.md`, `docs/agents/native-engineering.md`, `docs/TESTING.md`, `tools/run_tests.sh`, both stage plans and the spec. Create `docs/superpowers/audits/2026-10-12-run-recording.md` during execution, not during planning.

- [ ] Record checkout, branch, HEAD, dirty/untracked paths and selected build; preserve existing work.

```bash
git rev-parse --show-toplevel
git branch --show-current
git rev-parse HEAD
git status --short
```

- [ ] Check that the desired profile exists and is supported. The user's `viewer_physics_distributed.toml` was absent; do not manufacture it or substitute welded results as distributed acceptance. The available welded profile is suitable for source-backed regression controls.
- [ ] Establish the required current baseline once:

```bash
NATIVE_TEST_BUILD_PATH="$PWD/native/build/release" bash tools/run_tests.sh full
```

At planning time `full` calls the same native preflight as `native` (build, static sweeps, CTest and selected-artifact checks). Inspect the launcher again: if that changes, run both profiles as required by the native rules. Missing mandatory tools must be installed. Record exact baseline failures rather than weakening physical gates or declaring success. Continue source work around a documented environment baseline only within the execution instruction; final acceptance remains open until required gates pass.

- [ ] Create the audit ledger with A0, A1–A6, B1–B4 rows, requirement IDs, exact commands/results, selected artifact and unresolved errors. Leave implementation checkboxes open until their completion criteria are met.

## Requirements coverage

| Requirements | Primary implementation tasks |
|---|---|
| R01 identity, R02 opt-in, R03 lifecycle | A1, A3, B2 |
| R04 error/prefix/publication outcomes | A2–A4, B1–B3 |
| R05 passivity, R06 measurement separation | A2, A6, B1, B4 |
| R07 ownership/transaction safety | A2, A3, B1 |
| R08 physical payload, R09 catalog, R10 values/presence | A1, A2, A4, B1 |
| R11 events/commands | A3, B1, B2 |
| R12 provenance/assets | A1, A3, A4, B2, B3 |
| R13 research semantics | B1–B4 |
| R14 common owner/interfaces | A1–A4, B1, B2 |
| R15 database/schema | A1, A4, B2 |
| R16 publication | A4, B2 |
| R17 report, R18 CLI, R19 comparison | A5, A6, B4 |
| R20 replay/legacy compatibility | B3, B4 |
| R21 research optimization | B1–B4 |
| R22 explicit check policy | A3, B2 |
| R23 verification, R24 performance | A0, A6, B4 |

## Preserved exploratory storage evidence

The conversation measured one existing numerical CSV: `output/native-visual-acceptance/ride-viewer/research_rough_uphill_savage_articulated_effort_5227f30cf79e27d7/generation-0001/telemetry.csv`, 24,943 rows, 924 float64 columns and 196,798 null cells. Column aliases were retained. One parse was excluded. Seven batches of at most 4096 rows were persisted. Arrow/Parquet timers included close, file fsync, rename and directory fsync; database timers included commits, checkpoint and close. These were an earlier periodic-batch experiment, not the selected final-only export implementation.

Environment: Python 3.14.3, DuckDB 1.5.6, PyArrow 26.0.0, NumPy 2.5.4, SQLite 3.50.4. Warm query medians exclude database opening. No cold OS-cache control, process-crash injection, power-loss probe or full JSONL/research measurement was performed.

| Format | Write time | Final size | AVG over ten columns |
|---|---:|---:|---:|
| DuckDB | 2.002 s | 122.0 MiB | 0.86 ms |
| Arrow IPC LZ4 | 0.121 s | 93.5 MiB | 8.75 ms through DuckDB |
| Parquet Zstd level 1 | 0.717 s | 104.6 MiB | 9.24 ms through direct DuckDB scan |
| SQLite WAL FULL | 1.292 s, plus 0.545 s row preparation | 146.2 MiB | 39.33 ms |
| NPY numeric plus validity | 0.117 s, plus 0.160 s conversion | 197.9 MiB | Not measured |

SQLite's indexed 200-row/four-column window was faster than DuckDB (0.13 versus 0.69 ms). The comparison query was a self-join, not two independent physical runs. The measured Arrow codec result is not a universal compression ranking. DuckDB was selected for SQL analytics, typed multi-stream storage and a simpler shared final exporter; no fixed integration speed threshold follows from this table.

Temporary scripts/results under `/private/tmp` may disappear. Future acceptance creates fresh evidence on controlled owned data, recording capture cost and final export separately. Do not rely on the temporary environment or copy its Python version into project requirements.

## Completion

- [ ] A0–A6 meet the ordinary-stage gate and all R01–R19/R22 stage-A obligations.
- [ ] B1–B4 meet the research/replay gate and R13/R20/R21, including both research backends and legacy readers.
- [ ] The audit records the final full-profile output, selected artifact, focused failure-injection results, real viewer checks, and separated capture/export/RSS timing.
- [ ] Public documentation shows opt-in flags, final-only persistence, saved-run inspection and legacy replay selection. Examples use existing supported profiles and `uv`.
- [ ] No unchecked required gate is represented as completed. Handoff names remaining failures/limitations precisely.
