# Track B source-only delivery - 2026-10-09

## Evidence status

The user explicitly prohibited execution and compilation. Work was restricted to
archive inspection/extraction, source reading/editing, comparison of text and
packaging. No project module was imported; no Python/C++ application, build,
configure step, test, static analyzer, lint, sanitizer or benchmark was run.

The original audit already marked A4 verification as outstanding. Its historic
A0-A3 claims are not new evidence for these changes. The original phase-plan
completion checkboxes remain unchecked: B1-B3 source delivery is not B1-B3 gate
completion. There is no new commit ID: the input contains no `.git` directory.

## Delivered source scope

| Area | Source changes | Verification |
| --- | --- | --- |
| B1 | Shared NumPy draw source and immutable tape; owned native sensor clocks, queue, dropout, startup import and exhaustion handling; sensor binding. | Not run. |
| B2 | One-interval seam reused from the existing physical runtime; owned native control-window scheduler, programs, truth, quality, wheelie and torque/travel metrics; Python adapter/factory/protocol; incremental policy/operator lifecycle. | Not run. |
| B3 | Native columns/selected intervals; common staged exporter; schema-2 execution identity; strict schema-1/2 fixed file sets and rider parameter bundles. | Not run. |
| Regression sources | Independent frozen sensor oracle, parameterized sensor cases, program knots, one-second paired rollout, policy queues, publication retry, ownership/reset, strict/diagnostic outcomes, isolated engine failure, export parity and tampering. | Not collected or run. |

The Python oracle's physical integration and numerical formulas were not replaced.
Its sensor random calls were extracted without changing their valid-input order;
its external step was split into begin/resume operations. The previous save path
was moved into the common exporter. Brake validation now precedes the external
rider callback in both adapters.

The native runtime uses the pre-existing physical step/accounting implementation.
It does not call the Python research step, sensor RNG, program interpolation,
metrics, or recorder inside the GIL-released physics loop. This statement is
source-supported only; the corresponding no-Python-loop regression is unrun.

The adapter retains a completed result across Python publication failure. Native
solver/accounting failures retain their primary exception and completed evidence;
a later diagnostic-publication failure is attached as a note. These paths have
regression sources but no runtime reproduction in this delivery.

## Supplied library references actually inspected

The NumPy source root in the uploaded archive is `untitled folder/numpy`.

- `random/bit_generator.pyx`: SeedSequence child spawning.
- `random/_generator.pyx`: Generator normal/random call contracts.
- `random/src/distributions/distributions.c`: normal generation, including the
  normal draw when scale is zero. No substitute C++ random engine or approximate
  `std::normal_distribution` was introduced.
- `_core/src/multiarray/compiled_base.c`: interpolation endpoints, exact knots
  and slope/evaluation association. Research terrain interpolation reuses the
  existing native NumPy-compatible interpolation primitive.

The MuJoCo source root in the uploaded archive is `untitled folder`.

- `src/engine/engine_support.c`: matching state-bit order in `mj_getState` and
  `mj_setState`; `mj_versionString` returns static character data from the loaded
  engine. Native provenance uses `dladdr` on that data pointer, not a cast of a
  function pointer.
- `include/mujoco/mujoco.h`: corresponding public signatures.

These are source references, not proof that this delivery compiled against or
executed any binary built from those archives. The existing runtime dependency
pin remains MuJoCo 3.12.0. Neither reference archive is vendored into the output.

## Compatibility and deliberate limits

Python remains the default backend. Native selection is explicit through
`build_environment(..., backend="native")` or `create_native_research(reference)`.
Unsupported physical topology is rejected by the existing A-track setup gate.

The native facade does not expose mutable model/data objects. Custom boundary
rider behavior is deep-copied from the initialized reference; its Python state
must be deepcopy-compatible. A pending external window must be completed before
saving or directly resetting/closing the environment. PolicySession can queue
operator reset/stop/pause/brake actions for that boundary.

C3 checked native replay and the Track C viewer/CLI integration are outside this
phase. The schema-2 reader validates native recordings as data, but reconstruction
explicitly rejects substituting a Python backend for a recorded native backend.
No replay-success claim is made.

The input omits root packaging/lock files, `.git`, `tools/` (including the native
benchmark/check launchers referenced by CMake) and the original `tests/reference`
Python suite. Existing `native/tests` C++ sources are present and preserved. The
new fixtures therefore live in `tests/reference/conftest.py`; no absent upstream
`_native_runtime_support.py` was invented or overwritten. Integrating into the
full checkout must retain its existing tests/support and packaging/tool files.

## Outstanding gates

All of the following remain unexecuted, not waived:

1. A4 accounted physical rollout and failure/ownership gates on the intended
   supported artifact.
2. Configure/build with the project's strict compiler flags. Inspect the new
   generated provenance header, selected configuration and effective numerical
   flags; verify compiled and current native source fingerprints agree.
3. Collect and run the new B1 sensor sources, including the retained pre-refactor
   implementation. No numeric golden trace was generated by executing the old
   pipeline here; the frozen oracle is evaluated only when tests are authorized.
4. Run B2 transitions/observations, full-command delay/brakes, program/callback,
   strict/diagnostic, policy, snapshot/reset and isolated failure regressions.
   Require the complete one-second paired case at `atol=rtol=1e-9`, not an early
   terminal prefix presented as success.
5. Run B3 parity and schema/tamper/identity regressions. Compare summaries with
   only the explicit `research.execution` provenance difference removed.
6. Run the upstream native/full profiles, analyzer sweeps and intended sanitizer
   artifact. Measure runtime/memory behavior before making performance claims.

The original B-plan commands still apply when execution is authorized in the
complete checkout. The additional program/failure test modules should be included
alongside its listed test files. Missing tools or artifacts must be reported as
missing, not converted to passing/skipped physics checks.

## Packaging

The output is source only. It preserves the supplied project documents, examples,
Python/C++ sources and C++ contract tests, with the Track-B additions. Stale CMake
build output, Python caches, macOS metadata and compiled artifacts are excluded.
The companion patch is against the supplied source tree and uses `original/` and
`bikesim/` path prefixes (strip one component when applying in a full checkout).

## Input archive digests

```text
478e5f5a6983d5fcf2f01c1157a6fbac2bf2a4273cdbe8d895a855ef36ab7312  /mnt/data/numpy.zip
57ad67969fc4a649be553dc7d4cb82169fa13301ee1f343fd0417fd217e59979  /mnt/data/mujoco.zip
a3febcb7cc5fce1f206f82687a7182ba35e00d633d18f852e0bb8cbb87d8ef76  /mnt/data/bikesim(2).zip
```

## Post-delivery execution evidence — 2026-10-10

The six "Outstanding gates" above were executed in the complete checkout
(branch `impl/cpp-port-p2`; gate-of-record tree `8f5fb3c` = `ed75ac7` plus
three gate commits below). Host: macOS arm64, Python 3.14.3, NumPy 2.5.2
(Accelerate BLAS), MuJoCo 3.12.0, Apple clang 21. Selected artifact:
`native/build/release/bike_native.cpython-314-darwin.so` via
`NATIVE_TEST_BUILD_PATH` + `BIKE_NATIVE_BUILD_PATH`. Detail:
`.superpowers/sdd/2026-10-09-native-research-source-delivery/task-B3gate-report.md`.

### Item 1 — A4 rollout/failure/ownership gates: SATISFIED

The `full` profile below runs the entire suite including
`test_native_runtime_{accounting,rollout,ownership,step,commands,bootstrap}.py`
on the release artifact; the sanitizer selection additionally ran the
ownership/commands modules under ASan (23/23 carryover tests green, zero
sanitizer reports, process-isolated fatal probes included).

### Item 2 — provenance/fingerprint: SATISFIED

Compiled-vs-current fingerprint agreement verified in-process on both
artifacts after rebuild: `runtime_identity()['native_source_sha256']` ==
`native_source_fingerprint()` ==
`c1bc5d84dcfeaba8328cbf31e3a88d1433a8d677fae6898bcad4fd7602b165c6`.
`create_native_research` (`src/bike_sim/native/research.py:408`) raises when
they disagree, so every paired test exercises the check. Generated header:
`native/build/release/research_identity/Release/research_build_identity.hpp`
— `context_json` records `build_type=Release`, `compiler_id=AppleClang`,
`compiler_version=21.0.0.21000101`, and the effective flag set
(`-O3 -DNDEBUG`, strict `-W*` wall, `-Wframe-larger-than=8192`,
`-ffp-contract=off`, `-fstack-protector-strong`, `-fvisibility=hidden`,
`-fstrict-flex-arrays=3`, `-mbranch-protection=standard`). No git hash is
stamped by design — identity is a content hash of `native/{src,cmake}` +
`CMakeLists.txt`; HEAD at verification time was `ed75ac7`/`8f5fb3c`.
`native/build/asan/native_test_provenance.json` names the asan build and the
asan extension (its `native_test_count: 1870` is the loader's
carried-forward manifest field, not the sanitizer selection size).

### Item 3 — B1 sensor sources incl. retained pre-refactor oracle: SATISFIED

`test_sensor_noise.py` + `test_native_research_sensors.py` run in the `full`
profile and in the sanitizer selection (frozen legacy trace parity,
exact-call-order RNG draw parity, tape ownership/exhaustion, dropout/latency
matrix, seed repeat). All green.

### Item 4 — B2 regressions incl. the complete one-second paired case: SATISFIED (with clarification)

The delivered `test_one_second_pair_with_one_e_minus_nine_gate` is an honest
identical-classification gate, not a disguised completeness claim: it asserts
per-step `assert_tree` (atol=rtol=1e-9) plus **bitwise** integration-state
equality, and equal steps/reason at whatever outcome the Python oracle
reaches; its comment documents the platform truncation explicitly. Direct
probe: both backends `done` at `steps 104/800`, `reason=numerical_quality`,
identical `max_energy_residual_ratio=0.0500441062119803` — the documented
Accelerate-baseline `energy.constraint_work` violation (0.005 s, both
backends). `numerical_quality` truncation is unconditional in both
implementations (`environment.py:244-249`, `native/src/runtime/research.cpp:179-184`).

The complete case is nonetheless reachable: `--energy-tolerance` gates only
the truncation *decision*, never the integrated physics. With the established
`1e9` parity convention, **both backends complete all 800 steps with
`reason='duration'` and zero bitwise integration-state diffs** (identical
residual 0.19970166122503874). New test
`test_one_second_pair_complete_episode_gate` (commit `ff1f814`) asserts
`steps == max_steps == 800`, `reason='duration'` on both, per-step 1e-9 +
bitwise state parity, bitwise command logs, equal metrics/recorder — and
asserts the identical residual exceeds `.05`, proving the raised gate changed
no physics. The sibling truncation test is unchanged and keeps the
classification-parity coverage.

### Item 5 — B3 parity/schema/tamper/identity: SATISFIED

`test_native_research_recording.py`, `test_replay_manifest_v2.py`,
`test_replay_session.py`, `test_native_replay_session.py` all green in the
gate below — including `test_shared_export_native_rows_and_complete_provenance`
(summary compared with only the explicit `research.execution` difference
excluded) and the tamper/identity matrices; the sanitizer selection adds the
recording/export/identity tests under ASan.

### Item 6 — upstream profiles, sweeps, sanitizer artifact: SATISFIED

`bash tools/run_tests.sh full` at `8f5fb3c` (see numbers below): build +
`check_frontends`, `check_tidy` (findings none, 2 baseline nanobind
suppressions), `check_analyzer` (alpha report-only), `check_cppcheck`,
`check_odr`, `check_scripts`, headers, diagnostic-controls, context SDK,
CTest 4/4, `check_native_contracts` — all green; 2297 native extension items
collected.

Sanitizer: `native/build/asan` was stale (predated all B/C TUs). Rebuild
exposed two `-Wframe-larger-than=8192` failures in `research_binding.cpp`
(`bind_research` 34912 B, `bind_sensor_pipeline` 11200 B) — fixed with the
documented scoped `NATIVE_DIAG_IGNORE` waiver (`5696953`). A second
known-class defect: `test_native_research_failures.py`'s fatal-probe child
lacked `NATIVE_TEST_SANITIZER_RUNTIME` reinjection (dyld strips
`DYLD_INSERT_LIBRARIES` from the parent) — fixed same commit. ASan/UBSan
selection: **72 passed, 0 failed, zero sanitizer reports**, with
`verify_sanitizer_runtime` proving the Xcode clang-21 ASan dylib mapped
in-process; selection covers the new research/sensors/programs/recording/
presentation/replay TUs including both one-second gates under sanitizers.

### New `full` failures found and fixed by this gate

The first gate run at `ed75ac7` produced **31 failed / 2966 passed** — 7 IDs
beyond the documented 24-ID baseline, all test skew from the `198cfd0` /
`0ae8d99` interface refactors (the `native` profile never collects these
files, so B/C's green `native` run could not see them):

- `test_monitor_strictness::test_headless_tail_failure_...` (3 params) —
  stub `args` lacked `resolved_physics`; `sim.step` signature, driver-bound
  `PhysicalRecorder`/`physical_run_dir_name` bindings, `'Type: message'`
  failure format and the `outcome` summary key were all stale.
- `test_monitor_strictness::test_research_cleanup_...` (2 params) and
  `test_seated_pedaling_cycle::test_cadence_wish_is_forwarded_...` —
  `__new__` env stubs lacked `_closed`/`_window`/`_stop_requested` from the
  begin/advance split.
- `test_rider_information_boundary::test_motor_policy_cannot_...` —
  `PolicySession.__new__` stub lacked `_paused`/`_brakes`/`_queued` and the
  env stub lacked `control_pending`/`fail_policy`.

All fixed at `8f5fb3c` by completing the stubs to the current contracts —
assertions unchanged except the failure-string format now matching the
codebase's `Type: message` convention. No production code was touched by
these test fixes.

### Gate-of-record result (see §5 of the report for the command)

`bash tools/run_tests.sh full` at `8f5fb3c`, release artifact, log
`/tmp/b3-full-gate-final.log`: **24 failed, 2973 passed, 195 warnings in
792.19 s** (2997 collected, 2297 native-extension-selected). Sorted `FAILED`
IDs diffed against `/tmp/a4c-full-gate-final.log`: **byte-identical to the
24 documented baseline IDs — zero NEW failures.** All preflight sweeps
green (frontends/tidy/analyzer-report-only/cppcheck/odr/scripts/headers/
diag-ctrls/context/CTest 4/4/`check_native_contracts`).

### Gate commits

- `5696953` `fix:` asan build + failure-probe sanitizer reinjection
- `ff1f814` `test:` complete one-second paired gate (+ `conftest.py`
  `energy_tolerance` passthrough)
- `8f5fb3c` `test:` stub-probe contract updates

### Remaining limitations

- No performance claim is asserted anywhere; nothing was benchmarked.
- The 24 documented baseline failures remain red — platform physics
  divergence and test/code skew predating this delivery, unchanged by B/C.
- The `slow`-marked 80-transition parity test runs in `full` (release), not
  in the 72-test sanitizer selection.
- C3 native replay/viewer CLI integration remains out of scope; the
  schema-2 reader still rejects Python-backend substitution for native
  recordings — replay-success was not claimed before and is claimed only to
  the extent `test_native_recording_replays_same_backend_and_restarts`
  covers (same-backend replay round-trip).
