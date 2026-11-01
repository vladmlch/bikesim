# Lane H4 — native `frontends` sweep lint fixes (PHASE-3 port sources)

**Lane**: `.worktrees/pfc-lane-h`, branch `lane/h4`, integration tip
`b9d53842` ("merge: lane/e3 4.02 run_tests.sh native-only gate").
DARWIN-ONLY (macOS arm64). No container use.

Scope: the 4.02 `native` gate runs `tools/run_native_checks.sh
build/native-release`, whose `frontends` kind `-fsyntax-only`-compiles every
translation unit in the root graph (512 TUs, 505 unique sources) under
`/opt/homebrew/bin/g++-16` with `-Werror`. PHASE-3 validation/test ports
landed with warnings the legacy `native/build` graph never saw — its 334-TU
manifest contains no `native/src/validation/*` or reference-test units, so
`cmake --build native/build --target check_frontends` was green while the
real gate was red. This task cleared all 123 diagnostics.

## Baseline (verified, not assumed)

Fresh root-graph sweep, log dir
`build/native-release/native_check_logs/frontends/20261031T230258514801Z/`:
**123 diagnostics across 37 TUs** — matching the reported count exactly.

| class | count | approach |
|---|---:|---|
| `-Werror=misleading-indentation` | 52 | Braced single-statement bodies (`{throw…;}`, `{continue;}`…) or split trailing statements onto their own line (compressed one-liner `if/for/else` chains in validate_main, acceptance, benchmark, drivetrain, evidence, experiments, registry, rider, rigs, runtime_access, scenarios, test_t54). Zero statement reordering — every trailing statement was already unguarded. |
| `-Werror=missing-field-initializers` | 34 | Added explicit designators for the members GCC names, in declaration order: `terrain::TrackSpec` `.obstacles={}`/`.description={}`/`.grade_profile=std::nullopt`/`.surface_sections={}`; `rider::IntentControl` (= `runtime::RideControl`) `.motor_limit_nm`/`.crank_target_rate_rad_s`/`.posture=std::nullopt`; `rider::RiderCommand` `.posture={}`/`.crank_target_phase_rad=std::nullopt`; `drivetrain::RideControl` `.motor_limit_nm=std::nullopt`. All value-initialize identically to the omitted member — bitwise no-op. Members with NSDMIs (`surface`, `rider_enabled`, `enabled`, `crank_target_rate_rad_s`, `length_m`) stay omitted, matching the file-local NOLINT'd terse style. |
| `-Werror=useless-cast` | 26 (12 unique sites) | Removed casts where target type == expression type: `static_cast<std::int64_t>(std::llround(…))` (llround→long long == int64_t on darwin LP64; on Linux LP64 the variable widens to `long long`, same 64-bit signed arithmetic) in rider_control_reference_support.hpp:336, test_native_coupled_rider_task.cpp:206, test_native_rider_allocation.cpp:904/1057, test_planar_arms.cpp:460; `std::string(source.substr(…))` in replay_reference_support.hpp:372; inner `static_cast<float>(.3F)` in test_alpine_track.cpp:160 (outer float→double cast kept — it carries the np.float32-widening intent); `static_cast<std::int32_t>(base+offsets[i])` test_native_encode_columns.cpp:833; `static_cast<std::size_t>(std::stoul(…))` test_native_pneumatic_model.cpp:698; `std::vector<int64_t>([]…())` test_native_recording_retention.cpp:75/115; `static_cast<sig_atomic_t>(shield.deferred())` test_recording_capture.cpp:402. The 26-count is TU-multiplied: the two header sites reappear across including TUs. |
| `-Werror=dangling-reference` | 7 | test_joint_strength.cpp:521/523/531/593/603/608/612 — `const auto& r = component_row(last, tmp)` / `curve_at(profile, tmp, dir)`. **GCC false positive**: both helpers return `const T&` aliasing their *first* argument (`raw.components` rows / `profile` map entries), never the temporary `name` — verified by reading the callee bodies (test_joint_strength.cpp:484-490, 67-77). Bound the results **by value** (`const auto r = …`), eliminating the hazard class outright; copies of read-only test data, semantically identical. |
| `-Werror=unused-but-set-variable` | 1 | test_native_cruise.cpp:269 — `CruiseState detached = snapshot; detached.integral_mps_s = 999.;` was never read. Added `detached.integral_mps_s == 999.` to the require — mirrors the oracle's "the detached copy actually took the mutation" half of the assertion. |
| `-Werror=type-limits` | 1 | test_native_encode_columns.cpp:2040 — `left >= 0` on `std::size_t` was vacuous. Replaced with `left < feet.s.size()` — the real "found" check. **Latent bug**: had `'left'` been absent, `left` would equal `feet.s.size()` and the subsequent `.b[left]`/`goal.children…b[left]` indexing would read out of bounds; the vacuous check guarded nothing. |
| `-Werror=redundant-tags` | 1 | benchmark.cpp:284 — `struct utsname hardware{}` → `utsname hardware{}` (redundant class-key; same type). |
| `-Werror=range-loop-construct` | 1 | test_native_runtime_step.cpp:192 — `for (const auto [front, rear] : demands)` copied each `std::pair`; now `const auto &[front, rear]` — read-only refs, same values. |

Real bugs found: **one** — the vacuous `left >= 0` guard above (test-only
robustness hole, no runtime/physics impact). The 7 dangling-reference hits
are a documented GCC heuristic false-positive class and were still hardened
by value-binding.

## Files changed (25)

`native/src/tools/validate_main.cpp`; `native/src/validation/{acceptance,
benchmark, drivetrain, evidence, experiments, registry, rider, rigs,
runtime_access, scenarios}.cpp`; `native/tests/migration/test_t54.cpp`;
`native/tests/reference/{replay_reference_support.hpp,
rider_control_reference_support.hpp, test_alpine_track.cpp,
test_joint_strength.cpp, test_native_coupled_rider_task.cpp,
test_native_cruise.cpp, test_native_encode_columns.cpp,
test_native_pneumatic_model.cpp, test_native_recording_retention.cpp,
test_native_rider_allocation.cpp, test_native_runtime_step.cpp,
test_planar_arms.cpp, test_recording_capture.cpp}`.

No suppressions added; no `NATIVE_DIAG_*` pragmas touched (clang-only,
no-op under GCC anyway); pre-existing scoped NOLINTs for the intentional
terse-designated style retained.

## Verification

| gate | result |
|---|---|
| `bike-native-checks --kind frontends --build build/native-release` | **findings: none**, tool health ok, rc=0 — log dir `…/frontends/20261031T234243756326Z/` (123 → 1 → 0; the single intermediate residual, rigs.cpp:221, was re-split and verified rc=0 before the rerun) |
| `cmake --build native/build --target check_frontends` | **findings: none**, tool health ok (334 TUs — no validation/reference units; confirms no regression in shared sources) |
| `ctest -R 'validation\|T54\|migration_T54'` (build/native-release) | `migration_T54` **Passed** 1.18 s — the only node matching the pattern; carries `validation`/`t54` labels; exercises `run_case` over the edited validation registry |
| Edited-test CTest set (11 executables) | All **Passed**: native_cruise 0.45 s, native_pneumatic_model 0.57 s, alpine_track 1.12 s, native_encode_columns 0.92 s, native_rider_allocation 54.09 s, native_coupled_rider_task 349.15 s, joint_strength 3.20 s, planar_arms 2.99 s, native_runtime_step 1.46 s, native_recording_retention 2.98 s, recording_capture 2.55 s — incl. golden-compared cruise/alpine/pneumatic (bitwise evidence) |
| `bash tools/run_tests.sh native` (full gate) | **rc=1** — build + `frontends` green inside the run, then aborted at `tidy` per gate order; documented-baseline red, not authored here (see below) |

## Wider-gate status (post-frontends kinds, all manually run per-kind)

`run_native_checks.sh` runs kinds in fixed order and aborts at the first
blocking result, so the gate cannot pass the `tidy` kind. Every kind was
also run standalone for evidence:

| kind | result | attribution |
|---|---|---|
| `frontends` | **green** — tool health ok, findings none (this task's deliverable) | — |
| `tidy` | blocking — 2,654 unsuppressed findings across 132 TUs (clang-tidy LLVM 23.1.2; `misc-const-correctness` 561, `modernize-use-designated-initializers` 437, `bugprone-unchecked-optional-access` 225, `bugprone-easily-swappable-parameters` 181, …) | **pre-existing**: flagged files are overwhelmingly untouched by this diff; the only 2 diagnostics on edited lines (`test_native_recording_retention.cpp:76/116`, `modernize-use-ranges` on `std::iota`) are the identical warnings that existed at baseline — same iota call inside the unchanged lambda body, column shifted by the removed `std::vector` wrapper. Prior reports document this recurring Darwin tidy red (task-C320: "317 unsuppressed findings across 102"). |
| `analyzer` | blocking — 2 stable-pass findings (`test_native_platform_recording.cpp:39` UndefReturn; `test_seated_topology.cpp:90,93` CallAndMessage); 446 alpha records report-only | **pre-existing** — both flagged files untouched by this diff |
| `cppcheck` | blocking — 22 warnings + 33 errors across 42 TUs (cppcheck 2.22.0; two independent runs identical) | **pre-existing**: zero diagnostics on edited lines (line-intersect checked vs `git diff -U0`); flagged files are untouched (`test_async_close.cpp`, `tests/support/golden.cpp`, …). |
| `odr` | blocking — final `-Wodr` `-r` link fails on duplicate `typeinfo`/`vtable`/dtor symbols for `research_reference::CountingPolicy` (research_reference_support.hpp, emitted from ≥7 TUs) | **pre-existing** — header untouched by this diff; a genuine missing-`inline` ODR-adjacent issue worth a follow-up lane |
| `scripts` | green — shellcheck clean | — |
| `headers` | green — findings none, tool health ok | — |
| `diagnostic-controls` | green — 6 controls, 0 failed | — |
| `context` | green — 5/5 SDK controls passed | — |
| `contracts` | **passed** | — |

## CTest leg (evidence run, `ctest -LE 'slow|audit|package'`)

175/181 passed; the 6 reds are all documented baseline failures in
**untouched physics-oracle code** — this diff contains zero runtime/physics
changes (braces, explicit value-init equal to the implicit default, no-op
cast removal, read-only value copies):

- `migration_T09` (`post-advance integration vector matches the oracle`) +
  `migration_T10` (`first transition matches the Python oracle`) — the
  frozen-oracle baseline pair, verbatim signatures, cited as baseline red
  in ≥10 prior lane reports (C317, C320, C325, C326, C336t2a/b, F4, F5,
  tail5, …).
- `reference.native_runtime_commands` —
  `test_gear_shift_and_coast_resume_match_python` bitwise mismatch at
  interval 120 — documented `planned` red (task-C325: all four
  `test_native_runtime_commands.py` reds share the propagating
  bitwise-drift disposition).
- `reference.native_runtime_rollout` +
  `reference.native_research_environment` — golden/gate bitwise
  mismatches in the documented task-F2 baseline tail concentrations
  (`runtime_*` ×14, `research_environment` ×17).
- `reference.native_atomicity` — `committed shift lands the 34/28 ratio
  atomically` — same frozen-parity-drift family (golden_platforms
  precedent cited in task-F5). Its TU does transitively include an edited
  header, but the only edit there (`static_cast<int64_t>(llround(…))` →
  `llround(…)`) is a same-type cast — bit-identical codegen, cannot move
  a shift ratio.

Toolchain note: `build/native-release` held a stale `Unix Makefiles`
configure while the `release` preset pins `Ninja`; `CMakeCache.txt` +
`CMakeFiles` were cleared (build artifacts only) so the gate's own
`cmake --preset release` drives the correct generator.

## Concerns

- None residual in the diff: every hunk is braces-only regrouping of
  same-line statements, explicit value-initialization equal to the implicit
  default, or removal of a no-op cast — verified by full `git diff` review
  and by the green reference/golden tests.
- GCC's `-Wmisleading-indentation` reads `else{…} next-stmt` on one line as
  guarded; the fix pattern used (brace or hard line-break before the
  trailing statement) is mechanical and preserves evaluation order.
- The `llround` → `int64_t` casts were only *useless* on darwin
  (`int64_t == long long`); on LP64 Linux removal widens the `auto` variable
  to `long long` — same width, same arithmetic, no bitwise difference, but
  the darwin gate cannot observe it directly.
