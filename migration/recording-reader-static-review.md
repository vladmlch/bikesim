# T43–T46: reader, analysis, replay, reporting and analyst review

Date: 2026-10-17. Status: **implemented, execution qualification pending**.

The implementation and regression sources described here were written and
reviewed as source text. The user explicitly prohibited compilation and
execution. No CMake configure, compiler, linker, simulator, policy, project
Python module, DuckDB round trip, CLI executable or test was run. In particular,
an authored test case is not a passing assertion or an acquired oracle.

The owning specification is
`docs/superpowers/plans/2026-10-14-python-free-cpp-06-recording-replay.md`, with
`migration/interface-contracts.md` and the individual T43–T46 source cards.
Their execution/deletion checkboxes remain open. The integrator owns the CMake
registration and shared migration ledgers; this note supplies the implementation
boundaries and the limits of the static review.

## Source ownership and native boundaries

| Package / source units | Native implementation |
|---|---|
| T43: F094 package exports; F095 analysis; F105 reader | `recording/reader.hpp/.cpp`, `analysis.cpp`, `analysis_detail.hpp`, `query.cpp`; shared DuckDB ownership in `recording/duckdb.hpp/.cpp` |
| T44: F106 replay input; F133 reconstruction/checks; F134 checked session | `replay/input.hpp/.cpp`, `input_detail.hpp`, `legacy.cpp`, `rebuild.hpp/.cpp`, `session.hpp/.cpp` |
| T44: F118 presentation clock; F128 saved-state research playback | Existing `research::PlaybackClock` in `research/policy_session.hpp/.cpp`, application/viewer pacing, and new `replay/playback.hpp/.cpp` |
| T45: F046 legacy export; F107 report | `replay/export.hpp/.cpp`, `recording/report.hpp/.cpp`, `recording/export.hpp/.cpp`; report resource `native/resources/reports/summary.md.in` |
| T46: F009 analyst command | `cli/recording.hpp/.cpp` and independent `cli/recording_main.cpp` |

The generic reader does not reconstruct a simulator, load a controller or
import a policy. Model catalog inspection is isolated in the core owner's
`recording/catalog_model.cpp`. The shared `runtime/encoding_values.hpp` contains
owned piece data only; generic validation does not include the physical
projector. The intended standalone link boundary is
`bike-recording → bike_recording_cli → bike_recording_reader → bike_io /
bike_platform / DuckDB`. This was checked from the source includes, call sites
and target declarations, not by inspecting a linked executable.

## Read-only database and typed analysis

The reader accepts the exact container-schema-1 sealing manifest and validates
its run identity, report status, fixed payload/hash set and row counts. The
DuckDB validator supplied by T41 checks the complete schema, catalog,
relationships, masks, clocks and stream extents. Asset hashes are checked before
the evidence is exposed. Detection of a container marker, including a damaged
or symlinked marker, selects the container validator; it never falls through to
the more permissive historical directory adapter.

Read-only `Database` owns an opened regular-file descriptor, requests no-follow
leaf handling, and gives DuckDB `/proc/self/fd/<n>` on Linux or `/dev/fd/<n>` on
macOS. The retained descriptor is hashed before and after relational validation;
size, modification/change times and regular-file status are checked around the
read. The fixed payload paths are also rechecked. Actual DuckDB descriptor-path
normalization and sidecar behavior require platform execution; this is not a
claim that arbitrary concurrent replacement of ancestor directories by a
same-credential process has been proved safe.

The connection disables external access and extension installation/autoload,
uses a single thread and a 512 MiB memory limit, and disables temporary spill
for the reader. Public SQL is extracted and prepared as exactly one SELECT
statement. Its result is consumed as owned batches; mutation, multiple
statements and external file readers cannot be requested through the analyst
API. `Recording::close()` is idempotent and later queries fail explicitly.

Typed conversion preserves signed/unsigned integers and wider integer decimal
text without routing through binary64; floating values use the lossless tagged
codec for nonfinite data. It also handles decimal, byte strings, UUID, enums,
dates/times/timestamps, timedeltas, lists/arrays, structs, and string-keyed maps.
Reserved codec keys are escaped by the existing JSON codec. Unsupported DuckDB
logical types fail explicitly instead of receiving a lossy string conversion.
Depth, field count, list/map size and individual byte-cell limits bound material
allocation. Those limits and the date domain 1–9999 are explicit native reader
domains and need the source-specific boundary comparisons.

Overview, fields, events, windows, SQL and comparison share positive, bounded
limits and lookahead truncation reporting. Window selection retains interval
start semantics, the separate overlap-before row, aliases, coordinate meaning,
and absent/null/present distinctions. Comparison requires compatible field
semantics, coordinates, clocks and measurement provenance before comparing
values. It does not interpolate or align different clocks. Integer categories
remain exact; finite scalar tolerances, nonfinite classification, signed-zero
statistics, first mismatch paths and unequal extents remain explicit. Two
independent connections permit comparing a reader with itself without
invalidating an active cursor.

## Checked replay and stored-state display

Container schema 1 and replay contracts 1, 2 and 3 are separate axes. The legacy
adapter keeps the original fixed-file, checksum, state-shape and execution
identity rules; version 1 does not invent a checksum for its unhashed interval
stream. Container research replay requires contract 3, an eligible complete
episode, the captured recipe and assets, matching stream identities and all
requested/applied/observation/transition extents. Loading these inputs is
data-only and does not open arbitrary recorded pathnames.

Checked reconstruction materializes only the fixed captured assets in a private
temporary directory. It recreates the real native setup, sensors and programs,
disables live equilibrium cache use during reconstruction, and checks the
canonical resolved configuration, model image, actual compiled terrain,
runtime/provider/source identity and solver qualification. An unbundled
equilibrium-state path is rejected. No policy is loaded: the recorded requested
commands drive the native runtime directly.

`ReplaySession` compares actual initial integration state and observation, each
accepted requested/applied command, observation and completed transition, final
metrics/outcome/validity and final integration state. The comparisons retain
the source distinctions between numerical trees and integration-state array
tolerances. A shape match never produces a success certificate. One accepted
command survives a budget or target-step yield; acknowledgement follows its
verified completion. Early close and a partially consumed command remain
incomplete.

`report()` is success-only; `diagnostics()` describes incomplete, incompatible,
mismatched or completed evidence. `done()` requires final verification. A
mismatch freezes the last verified display frame and advances the presentation
generation before any rollback, so the viewer can accept the rewind without
mistaking it for another physics step. Restart reconstructs fresh owners and
uses a new generation. Both interfaces expose their actual model image and
owned `runtime::RenderFrame` for the viewer boundary.

Historical Python/native-extension execution profiles have no acquired
certificate of arithmetic compatibility with the new executable. Checked
replay returns **incompatible execution** for those profiles; it does not
substitute a fabricated extension hash. Required historical checked profiles
remain a migration/deletion blocker until certified or explicitly waived by
the user, as the owner plan already requires. Native allocation-controller
solver identity must also be qualified before such a run can receive a checked
certificate.

`PlaybackSession` can display validated saved evidence, including an interrupted
or otherwise non-replay-eligible episode. Ordinary recordings use their exact
captured model image. Research/legacy presentation compiles the visual model
from its captured recipe without equilibrium settling or physics integration.
The frame path sets saved state and time, interpolates qpos, selects the source
right-neighbor qvel keyframe, and never calls a physics forward/step operation.
Quaternion normalization rejects invalid norms and aligns antipodal endpoints
before interpolation. The antipodal alignment is an explicit robustness change
from a direct linear interpolation that can produce a zero quaternion. Visual
goldens and playback edge qualification remain unrun.

## Reports and exports

Report generation reads the same database evidence through the read-only
boundary, including the internal pre-manifest report connection used by the
publisher. It distinguishes wall and simulation clocks, input and derived
events, outcomes and model validity, missing/nonfinite coverage, cumulative
values and derived recorded-power integrals. Forwarded energy is not relabeled
as a sampled peak. Rendering returns content; final-only publication owns the
exclusive write and the separate report-failure status.

Explicit CSV/JSONL export requires an episode and a fixed supported table.
Its default is the complete stream, not the convenience-query 200-row default;
an optional positive limit is explicit. Results stream in batches and output
creation is exclusive. Exact integer text, quoting/newlines/UTF-8, bytes,
structured tagged cells, null/nonfinite values and timedelta formatting have
dedicated handling. The ordinary and JSON analyst views represent the same
typed result tree.

Legacy export takes an owned snapshot before I/O, so resetting or closing the
source owner cannot invalidate a pending export. It writes the fixed historical
schema-2 bundle, source-shaped JSON/CSV streams, terrain and initial/final NPY
members, hashes the actual payloads and publishes the manifest last. The NPZ
implementation uses a valid stored ZIP with CRC and central directory; it does
not promise byte identity with Python's compressed archive. Native execution
provenance is retained truthfully and the compatibility receipt explicitly sets
`historical_python_reader_accepts_execution=false` for a new native-executable
identity. Therefore a newly exported native run is not claimed to pass the
unmodified historical Python reader's identity gate. That round-trip acceptance
requirement remains open; validated stored-state playback/export does not close
a checked-execution compatibility gate.

## Analyst CLI and shared physical metadata fixes

The standalone analyst implements overview, fields, events, window, sql, export
and compare, with the source choices/defaults, required options, long equals
form and option terminator. Help returns 0, malformed arguments 2 and operational
recording failures 1, with diagnostics on stderr. Integer options preserve plus,
ASCII whitespace and digit separators; huge syntactically valid limits fail the
storage domain rather than being mislabeled syntax errors. Float underflow and
overflow retain the source conversion to signed zero/infinity before semantic
finite/range checks. Implementation-specific NaN payload syntax is rejected.
Exact argparse help/error text, option abbreviation and broader Unicode numeric
syntax are not established by this source implementation and need explicit
source CLI comparisons before claiming complete byte/grammar parity.

The integrator requested fixes to ordinary physical capture while connecting
the replay input boundary. These shared changes are part of this handoff:

* `initialization/research_startup.hpp/.cpp` now exposes the existing read-only
  compiled physical metadata builder.
* `research/evidence.hpp` and `research/metadata.cpp` expose
  `physical_configuration_metadata()`. Both ordinary and research metadata use
  the complete resolved physical identity, compiled solver/contact/equality
  facts, mass/rider/suspension configuration and actual execution provenance.
* `cli/physical_capture.hpp/.cpp` takes the same captured launch metadata when
  creating assets. It extracts the actual compiled terrain vertices from the
  saved model, verifies the raw double-vertex hash independently from the NPY
  byte hash, and refuses profile bytes that changed from their setup digest.
  The true ideal-speed target, optional seed and async flag are recorded.
* The two corresponding calls in `cli/ride.cpp` pass the seed and immutable
  launch metadata. `cli/configuration.cpp` passes the ordinary async option.
  Finalization attaches the core owner's unchanged/changed/unavailable profile
  identity receipt without replacing captured assets.
* `io/float_format.hpp` contains the existing source-style binary64 formatter
  factored out of `configuration/schema.cpp`. Configuration still delegates to
  that same algorithm; recording uses it for source-compatible finite JSON/CSV
  text. This shared T12 source edit requires requalification with the final tree.

No numerical physics equation or tolerance was altered to obtain a replay
success. The broader root CLI and viewer changes have separate review notes.

## Authored tests and remaining gates

| Test source | Concrete cases represented; all unrun |
|---|---|
| `native/tests/migration/test_t43.cpp` | Manifest/schema corruption; SELECT-only and external-I/O refusal; lookahead limits; large integers, decimals, blobs, nested/null/nonfinite values; aliases/windows; self-comparison; no-WAL/unchanged bytes; closed-reader behavior; float formatting thresholds |
| `test_t44.cpp` | Real future native record→seal→publish→checked replay; pending command budgets; restart/early close; mismatch in rehashed transition evidence; verified-frame rollback; saved-state shapes/clamping/quaternions; historical incompatibility |
| `test_t45.cpp` | Real future default report rendering/publication; exact unlimited CSV/JSONL and exclusive output; synthetic fixed legacy bundle and NPZ round trip; truthful compatibility receipt; telemetry NaN padding; tampering, overwrite and nonfinite legacy JSON refusal |
| `test_t46.cpp` | Direct API exits/options/output semantics and standalone analyst subprocesses on a genuine future published fixture: help, overview, SQL refusal and exact-integer CSV export; integer/float syntax versus semantic range failures |
| `test_t47.cpp` additions | Actual compiled heightfield column count and distinct raw/NPY hashes; ordinary ideal-speed/seed/full metadata; refusal of a wrong recorded terrain digest; decoded explicit research string choices and rejection of literal repr quotes |

`test_t46` receives `$<TARGET_FILE:bike-recording>` as its one CTest argument and
depends on that executable. `test_t47` keeps the integrator's executable-directory
argument. The shared small recording fixture is synthetic owned evidence; the
real-runtime T44 test produces evidence only when it is eventually executed.
Neither replaces the required acquired Python/source oracle fixtures.

Pending qualification includes every added translation unit under the strict
compiler/warning policies; actual pinned DuckDB 1.5.6 headers and C API types,
nested vector reading and TIME_TZ/timestamp conversion; source-specific analyst,
report, legacy and replay oracle comparisons; real native record/replay runs;
read-only descriptor/sidecar behavior on Linux and macOS; output failure and
exception/retry probes; headless/display integration; and the repository's
required final native/full gates. The supplied snapshot does not contain the
pinned DuckDB SDK headers, and no dependency was acquired for this work.

No mandatory gate is marked passed, no source module is authorized for deletion,
and no clean-build, execution-parity, sanitizer or performance result is claimed.
