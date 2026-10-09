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
