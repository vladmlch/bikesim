# Run recording, analysis and checked replay

## Delivery status

The ordinary-native and Python/native research paths are implemented in source in
this delivery. Compilation, tests, analyzers, simulations, GUI checks and timing
were **not run**: the delivery instruction explicitly forbids execution. This is
not a claim that the A6/B4 acceptance gates are green. See the
[source-completion ledger](superpowers/audits/2026-10-13-run-recording-source-completion.md)
for the requirement-to-code/test map and the unrun gates.

The examples below are usage references, not commands executed for this delivery.
Use a complete project checkout with its dependency lock and the matching native
artifact. The supplied repomix did not include `uv.lock`, `.git`, compiled native
artifacts or all ignored/untracked assets. No lock or binary identity was invented.

## Opt-in and supported paths

`--record` defaults to false. `--out` selects an output root but does not enable
recording. Ordinary physical `bike-ride --backend native` supports capture in
headless and viewer modes. `bike-research` and `bike-ride --research` support
Python and native capture. Ordinary physical Python execution remains available
without recording; requesting its new recording adapter is an explicit error,
not an implicit switch to native. Legacy/nonphysical and preview modes reject the
new recording flag.

Without recording, physical accounting, live views, checks, sensor delay queues
and running metrics remain active. Optional sample/command/observation histories,
preview CSV, recording bundles and recording-derived HTML are not accumulated or
written by the normal physical/research CLI.

```bash
# Ordinary native, using the supplied reference physical profile.
BIKE_NATIVE_BUILD_PATH="$PWD/native/build/release" uv run --frozen --group native bike-ride \
  --backend native --physics-config examples/research/viewer_physics_welded.toml \
  --track examples/research/rough_uphill_savage.toml --headless --duration 1 \
  --check-policy strict --record --out output/ride

# Python research. The same invocation accepts --backend native with its artifact selected.
uv run --frozen bike-research --backend python \
  --physics-config examples/research/viewer_physics_welded.toml \
  --scenario flat --duration 1 --check-policy diagnostic --record --out output/research

# Interactive research: retain reset-separated episodes, publish on viewer close.
BIKE_NATIVE_BUILD_PATH="$PWD/native/build/release" uv run --frozen --group native bike-ride \
  --backend native --research --physics-config examples/research/viewer_physics_welded.toml \
  --track examples/research/rough_uphill_savage.toml --duration 1 --record
```

`--check-policy strict|diagnostic` is independent of recording. Ordinary headless
and research headless retain strict defaults; interactive paths retain diagnostic
defaults. Research's existing `--diagnostic-model-limits` remains an alias;
combining it with explicit strict policy is rejected. Diagnostic continuation
retains sticky invalid status and the original failure facts. It does not bypass
fatal engine errors or make an invalid physical result valid for learning/replay.

## A launch is not an episode

A launch owns one UUID, UTC start time and monotonic wall-clock origin. Resets
allocate consecutive episode IDs within that run. Native generation is retained
separately; time/step/interval zero may recur in another episode without aliasing
its evidence.

Capture appends only in RAM. A terminal viewer frame seals its episode in RAM and
remains visible. Reset seals the outgoing generation before replacing the owner.
Normal completion, viewer close, Ctrl-C and caught simulation/policy failures
attempt one final publication. A stop drains only an already accepted research
control window; it never asks a policy for an extra action. An engine failure can
leave a physical suffix beyond the completed-control prefix. Both extents are
saved; no successful transition is fabricated for that suffix.

All retained intervals and reset-separated episodes remain in memory until final
publication. This is deliberately not a streaming recorder. SIGKILL, a fatal
native signal, power loss or memory exhaustion before usable finalization can
lose the entire launch. There is no background writer or crash-readable live DB.
Capture overhead can affect a binding wall-clock cap; numerical passivity tests
must use fixed step-indexed inputs, physical duration and a nonbinding wall cap.

The legacy decimation options continue to select the existing expensive
measurement/check cadence. Full recording retains **every** accounted interval,
not only measurement intervals. Energy rows identify fresh versus forwarded
measurements and their source interval/time. Full and endpoint-only mass evidence
have explicit scope. Forwarded values are not extra physical measurements.

## Published layout and failure behavior

The CLI prints the actual unique published directory. It contains:

```text
<UTC timestamp>-<run UUID>[-collision suffix]/
    run.duckdb
    summary.md           # present only if report generation succeeded
    manifest.json        # publication marker, linked last
```

The ordinary frontend may place these directories beneath its descriptive
track/configuration directory. Never infer a recording path from that description;
use the path printed at finalization. No real published example was produced in
this execution-disabled delivery.

A staging directory is created exclusively under `--out` only at finalization.
Database creation, bounded Arrow ingestion, integrity checks and COMMIT run in one
transaction; CHECKPOINT and close precede report generation. Hashes cover the
closed DB and successful report. Exclusive hard links retain an owned staging copy
until the final manifest is published. Existing runs and pre-existing empty
folders are never adopted or overwritten.

If report generation fails, the valid database may still publish with
`report_status=failed` and an explicit `report_error`. A DB/payload/manifest failure
leaves the directory unsealed and the RAM capture retryable. A missing manifest,
wrong hash, symlink payload or unknown schema is an error, not partial success.
Repeated successful `RunCapture.finalize()` returns its original path; after a
failure, an explicit retry writes a fresh exclusive destination without advancing
the simulator. The primary simulation exception is preserved, with secondary
publication/cleanup failures attached as notes. Publication failure after an
otherwise successful run is an error exit.

## Data model and type semantics

Container schema version is **1**. Research replay contract version is **3**;
these version numbers serve different purposes. Core tables are `runs`,
`episodes`, `events`, `field_catalog`, `coordinate_catalog`, `alias_catalog`,
`assets`, `integration_states` and `metrics`, with `recording_meta` for container
identity/codec. The finite physical streams are `physics`, `forces` and the
reviewed `channel_*` tables. Research uses `commands_requested`,
`commands_applied`, `observations` and `transitions`.

Each physical interval retains incoming qpos/qvel, forces, powers and available
channels. Generalized-force components are long rows keyed by component, not
extra aliases of rounded power values. Observation rows are stored once;
transitions reference the previous/new observation and request IDs. Requested
controls, applied controls and raw operator input are distinct facts. Research's
historical applied-command stream records control changes; brake demand is also
available in requests and physical channels. It is not silently redefined during
legacy migration.

Catalog entries specify canonical key, SQL type, units, physical frame, phase,
source, optionality, measurement semantics and structured children. The
coordinate catalog distinguishes qpos and DOF indices, including free/ball joint
layouts and mixed translational/rotational units. Unknown source fields reject
encoding rather than receiving guessed units. Historical scalar aliases are
views/selections, not duplicate canonical retained columns.

Booleans, integers, strings, maps, structs and vectors keep their types. Binary64
values are not quantized. Optional fields have `<field>_present`; nullable parents
have `__<parent>_present` and `__<parent>_valid`. Missing, explicit null, false, zero,
empty containers, NaN, positive/negative infinity and signed zero are not merged.
Reader validation checks presence consistency and required values, not just hashes.

JSON uses the `bike_recording.json.v1` codec with reserved `$bike_recording` tags
for nonfinite floats and typed boundary values. Objects containing that reserved
key are escaped to remain literal user data. Duplicate JSON keys, bare NaN and
unbounded numeric overflow are rejected. Use `bike_sim.recording.json_codec.loads`
to decode tagged results without losing their distinctions.

`telemetry` joins one row per physical interval using canonical names.
`legacy_telemetry` provides compatible scalar aliases. Forces are not joined into
the base view because component rows would multiply physical intervals.

## Analyst workflow

Set `RUN` to an actual published directory. These commands never construct a
simulator or import a recorded policy.

```bash
RUN=/absolute/path/to/the/published/run
uv run --frozen bike-recording overview "$RUN" --json
uv run --frozen bike-recording fields "$RUN" --pattern energy --limit 200 --json
uv run --frozen bike-recording events "$RUN" --episode 0 --json
uv run --frozen bike-recording window "$RUN" --episode 0 --start 0 --end 0.02 \
  --fields physics.time_s,physics.qvel,channel_energy.residual_j --json
uv run --frozen bike-recording sql "$RUN" \
  --query 'SELECT episode_id, interval_id, component, power_w FROM forces ORDER BY episode_id, interval_id, component' \
  --limit 200
uv run --frozen bike-recording export "$RUN" --episode 0 --format jsonl \
  --table telemetry --output intervals.jsonl
```

Ordinary query output defaults to 200 rows and includes truncation/more metadata.
`--limit` raises that bound. Explicit CSV/JSONL export instead streams the whole
selected table in bounded chunks unless its own `--limit` is given. It creates the
destination exclusively. This is an analytics export, not a new replay recipe.

Window bounds are simulation seconds, `[start,end)`, selected by interval start.
The response includes stable interval IDs and intervals overlapping the start/end
boundaries. It does not interpolate. Query field selection includes relevant
presence companions. SQL permits exactly one read-only SELECT; automatic
extension loading/installation and external filesystem/network table access are
disabled. Output limits bound returned rows, not arbitrary query CPU cost.

Exact comparison requires compatible interval/time grids, units/phases,
coordinates and measurement provenance. Integer/category/boolean comparisons are
exact; float tolerances are explicit and default to `atol=rtol=1e-9`. Results report
first mismatch, maximum deviations, unmatched extents and matching invalid values.
Matching NaNs do not establish physical validity. Summary mode compares outcomes,
metrics, events and surrounding windows without pretending changed experiments
share an identical trajectory. Neither mode silently aligns time or distance.

```bash
LEFT=/absolute/path/to/left/run
RIGHT=/absolute/path/to/right/run
uv run --frozen bike-recording compare "$LEFT" "$RIGHT" \
  --left-episode 0 --right-episode 0 --mode exact \
  --fields physics.qpos,physics.qvel --atol 1e-9 --rtol 1e-9 --json
uv run --frozen bike-recording compare "$LEFT" "$RIGHT" \
  --left-episode 0 --right-episode 0 --mode summary --json
```

`summary.md` is generated deterministically from the closed DB. It records
provenance, outcomes, extents, first failure, event references, scalar coverage,
measurement-domain extrema, final cumulative values and available force-power
integrals. Findings link to stable IDs or reproducible SQL. It does not attribute
causality from coincidence or certify physical calibration. Regeneration can use
`render_report(reader)` with `open_recording(RUN)` and write a separate file;
changing a published `summary.md` in place invalidates its manifest hash.

## Checked research replay and legacy compatibility

```bash
uv run --frozen --group native bike-replay "$RUN" --episode 0
uv run --frozen --group native bike-replay "$RUN" --episode 0 --viewer --time-scale 2
```

Episode selection is optional only when the recording contains exactly one
episode. Ordinary physical recordings are analysis evidence, not checked research
recipes. Incomplete/error/invalid version-3 prefixes remain readable for analysis
but are not eligible to claim completed successful replay.

A data-only adapter reads the new database and validates hashes, schema, assets,
normalized references, extents and state shapes. It then supplies the same
`ReplaySession` verifier used by legacy inputs. Replay selects the recorded
backend and checks source/runtime/native artifact identity before constructing a
plant. It checks startup and every requested/applied/observation/transition
boundary, outcome, metrics and the final integration state with the existing
`1e-9` tolerances. Recorded policy metadata is never imported or executed.

Legacy directories with `replay.json` schema 1/2 retain their original fixed-file
reader, checks and backend identity semantics. They are not upgraded in place or
exempted from source changes. A historical fixture can remain parseable while
physical replay requires the original compatible source/artifact. The v3 adapter
does not manufacture a temporary legacy CSV/JSONL/NPZ recording; only fixed
bundled engine input files are materialized for model construction.

Programmatic research environments default to `capture_mode='legacy'` so explicit
`save()` callers retain their established behavior. `full` supports new evidence
and explicit legacy export; `off` rejects attempts to invent discarded history.
`verify` keeps only transient unapplied-consumer suffix data needed by checked
replay. It prepares owned Python values before acknowledging/removing a consumed
applied-command prefix. Numerical clocks, noise generation and controller input
ownership are not intentionally changed by these modes.

Legacy export takes one immutable snapshot of all journals, initial/final state,
metadata and setup-time assets before I/O. Native compatibility columns are
projected only for that explicit export. New normal CLI recording does not retain
or write a duplicate legacy history.

## Verification still required

The source-completion ledger maps all R01-R24 requirements and A1-A6/B1-B4 tasks to
files and regression sources. All newly authored and adapted tests are **NOT RUN**.
Required remaining acceptance is sequential focused Python/native checks, the
repository `full` profile on the selected matching artifact, real viewer reset/
close interaction, and separate capture-on/off wall time, peak RSS, export cost,
size and query measurements at the unchanged timestep. Existing audit numbers
and stored output files are historical inputs, not results of this delivery.
