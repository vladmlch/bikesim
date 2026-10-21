# Phase 3 golden infrastructure — source implementation review

Scope: completion-plan tasks 3.01 and 3.03. This record describes source
changes only. The user's prohibition on execution covers compilation,
compiler syntax checks, CMake configure, pytest, CTest, native applications,
capture scripts and containers. None was run for this work. No generated
oracle trajectory, numerical measurement or successful qualification is
claimed here.

## Implemented source

`tests/reference/_golden.py` implements opt-in expected-value capture to one
JSON sidecar array and one concatenated binary file per pytest node. It
preserves scalar dimensions, original primitive dtype and C-order bytes,
records explicit close tolerances, truncates on a node's first call in a new
capture process, and rejects unsafe/colliding node paths and malformed
capture state. `_bits.py` retains the original dtype/shape/raw-byte
assertion and records its expected side before that assertion. Failed test
output is not qualified golden evidence.

`native/tests/support/golden.hpp/.cpp` implements the same UTF-8 node-key
sanitization/hash rule and strict sequential readers. Node identity, labels,
kind, dtype, shape/byte count, offsets, all comparison indices, tolerances,
and the final payload length are checked. Missing files and unconsumed
entries fail. Binary64 equality preserves signed zero and NaN payloads;
close comparisons retain expected-side `atol + rtol * abs(expected)`.
The explicit-shape `f64`, `f64_close`, and `bytes` overloads additionally
check the caller's full dimensions, distinguishing scalar `[]` from `[1]`
and `[1,1]`. The two-argument floating convenience overloads receive only a
span and therefore compare flattened element count, not actual dimensions;
ports with source arrays use the explicit-shape overloads.

`native/tests/reference/CMakeLists.txt` is conditional on `BUILD_TESTING`.
Its `bike_add_reference_test` helper uses `reference.<name>`, the
`reference` label, and the target directory as the working directory. The
optional `ENGINE` flag supplies startup policy and the established macOS
MuJoCo load-name rewrite. Pure reader targets avoid that rewrite because
the existing rewrite script requires a real MuJoCo linkage edge. Golden
root/platform values use a generated header; they do not introduce quoted
public compile definitions into native check-context serialization.

`native/src/io/npz.cpp` now writes deterministic ZIP32 archives containing
STORED NPY members, local headers, central directory and EOCD. It uses the
same private CRC32 routine as the reader, rejects duplicate/unsafe names,
and enforces the existing member/byte resource limits. No compression
dependency, extraction operation or subprocess is introduced.

`native/src/golden/episode.hpp/.cpp` implements all episode fields and NPZ
keys, manifest/control preservation, explicit missing channels, v1/v2
optional arrays, lossless NPY storage, exclusive artifact creation, and
row/first-failure comparisons. Old artifacts without `qfrc_applied` retain
an empty independent-observation array; force components are never used
to invent the missing measurement. MuJoCo model serialization uses the
existing `engine::invoke` error boundary.

`native/src/golden/capture.cpp` captures physical force-entry state,
force-exit `ctrl` and the actual live `qfrc_applied`, ordered component
contributions, tyre state/snapshots/diagnostics and accounted physical
sample rows. A private cold `RuntimeAccess` seam holds existing reentrancy
guards and uses ordinary physical advancement/accounting. The optional
`GoldenForceObservation` pointer defaults to null for every existing
caller; only capture requests its copies and allocations. Physics
operations were not reordered or replaced. Replay decodes all controls
and validates explicit initial-state dimensions before resetting, then
overlays the artifact's qpos, qvel, act, ctrl, warmstart and time before
the first physical force call. Writer/controller bootstrap state remains
that of the requested environment, as in the existing Python replay tool.
The artifact contains a serialization of the live model after capture,
including mutable coefficients, rather than substituting its setup image.
The independent live-exit regression has a narrow internal capture-only
callback receiving the bounded live force span immediately after
`apply_forces`. Its fields are private to the capture/step implementation;
ordinary capture and replay leave it null, and the default runtime path
still has no observation. The nonthrowing test callback uses preallocated
storage. Exact scope is recorded in `cpp-linux-phase3-episode-ports.md`.

`native/src/cli/recording_golden.cpp` implements `golden freeze` and
`golden compare`, with exit 0 for success, 1 for mismatch/operation failure,
and 2 for usage errors. Dispatch occurs before the existing flat recording
parser. Nested help documents the commands and freeze's explicit default
of 35 N m human torque (overridable with `--human-torque`). The pinned
top-level help remains as previously stored because `migration_T47` could
not be run. `RecordingApplication.cmake` builds `bike_golden`, links it into
recording CLI, and applies the existing build/install relocation path to
the engine-linked recording executable.

## Contracts corrected during source review

1. The plan's control-type fact is stale. `runtime::RideControl` exists as
   the full `rider::IntentControl` alias in `runtime/control.hpp`. The
   drivetrain-only control loses posture/rider fields. Capture/replay use
   the full actual runtime control type.
2. `np.ascontiguousarray` promotes a zero-dimensional value to a one-element
   vector. The hook uses `np.asarray(...).tobytes(order="C")` and the
   original shape, preserving the existing assertion's scalar contract.
3. Python `golden_episode.flatten_row` descends dictionaries but retains
   list-valued leaves, while `episode_compare._leaves` descends lists too.
   The C++ artifact keeps `stored_rows` as the serialization view and `rows`
   as the scalar comparison view; qpos/qvel lists are not silently lost.
4. The episode comparator uses candidate-side relative scale, unlike the
   oracle-node close reader's expected-side scale. The two APIs keep these
   distinct source contracts. Sentinel conversion, scalar numeric strings,
   missing/extra channels, ordered failure violations and whole-row diff
   truncation are covered by new test source.
5. Legacy Python JSON may contain unquoted NaN/Infinity inside list-valued
   `non_numeric` channels. The episode reader normalizes only unquoted
   complete constants to portable comparison sentinels before strict JSON
   decoding; quoted text is preserved.
6. CRC32 need not be exported merely to share it. Reader and writer are in
   the same translation unit and use the same anonymous-namespace function.

## Tests and acquisition inputs prepared, not executed

- `test_golden_hook.py`: expected capture, repeated indices, truncation,
  disabled capture, dtype/scalar shape, C-order noncontiguous arrays,
  Unicode/long/dotted node keys, suffix parsing, collisions, invalid
  tolerances and preservation of failed assertions.
- `reference.golden_support`: synthetic reader-format checks for ordering,
  missing/corrupt payloads, signed zero, NaN sign/payload, close bounds and
  explicit scalar/vector/matrix shape rejection without consuming an entry.
- `reference.golden_episode_codec`: mixed-type NPY/NPZ roundtrip, fixed ZIP
  signatures/method, CRC corruption, duplicate/unsafe names, comparison
  semantics, manifest/list/null/control roundtrip, missing live total and
  CLI usage dispatch. Its two-byte model payload is explicitly synthetic
  codec data; it is neither an engine image nor oracle evidence.
- `reference.golden_episode_tool` (`golden;slow`): requires a real
  platform-specific Python artifact; checks all state/force arrays,
  manifest/resave, native 40-step capture against that artifact, applied
  controls, first failure, explicit-state replay and zero-tolerance native
  self-replay. It leaves a named resaved artifact for a future Python
  cross-read. Absence of the Python artifact is an explicit failure.
- `reference.golden_episode_regressions` (`golden;slow`): all nine original
  functions from the three episode suites, ten cases including v1/v2.
  Preserves the distinct 50- and 40-step horizons, every-step bitwise
  component fold, real live `+.125` force mutation and actual save/replay.
  Per-function dispositions are in `cpp-linux-phase3-episode-ports.md`;
  these are source assertions, not execution results or Python goldens.
- `tests/fixtures/golden/episode/pinned_savage_40/physics.toml`,
  `request.toml`, and `freeze.py`: committed source inputs for the prescribed
  pinned configuration and forty direct physical steps. Asset paths are
  relative to these inputs. The script explicitly pins Python and seed 0,
  uses private temporary CLI output, closes through the existing research
  lifecycle, and refuses to overwrite an existing oracle directory. It has
  not been executed.

## Still required before closing tasks

Both actual Python captures under `darwin-arm64/episode/pinned_savage_40`
and `linux-aarch64/episode/pinned_savage_40` remain absent. General per-node
golden captures also remain pending. The source files above are not a
substitute for those measurements.

All compilation, compiler-policy checks, CTest/pytest runs, cross-reading
C++ output from Python, native-versus-Python array equality, CLI checks,
macOS load/install relocation and the two-platform parity/qualification
gates remain unexecuted. In particular, a numerical difference exposed by
the real episode test must be investigated under ADR-0003; no tolerance
was widened and no native-generated result was installed as an oracle.

These tasks are therefore **implemented in source, awaiting acquisition
and execution evidence**, not closed. The previously reported missing
golden infrastructure is addressed by these files, while the earlier
numerics-lock and phase 1–2 evidence blockers are outside this tranche.
