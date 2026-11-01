# Task — Darwin native sweep debt clearance (lane/h5)

**Lane**: `.worktrees/pfc-lane-h`, branch `lane/h5`, integration tip
`bc637988` (`merge: lane/h4 frontends lint debt cleared`). DARWIN-ONLY
(macOS arm64, AppleClang, libc++ hardening EXTENSIVE). No container use;
`container.sh` guard untouched. Tooling: Homebrew LLVM 23.1.2
(clang-tidy/clang-analyzer), `/opt/homebrew/bin/cppcheck`, `g++-16`
(frontends/odr passes).

## Scope

Clear every blocking static-analysis finding on the Darwin arm64 native
release graph so `bash tools/run_tests.sh native` passes all nine
`check_<kind>` sweeps and reaches CTest, with behavior bitwise-identical
and no unjustified suppressions. Sweep kinds:
`context, contracts, diagnostic-controls, frontends, headers, odr,
scripts, tidy, cppcheck, analyzer`.

## Per-kind before/after

| sweep | before | after | tool health |
|---|---|---|---|
| `check_odr` | duplicate `research_reference::CountingPolicy` typeinfo/vtable/dtor symbols across TUs | `findings: none` | ok |
| `check_tidy` | ~2,654 raw diagnostics across 132 TUs (initial), 506 at hand-off | `findings: none` (suppressions: `misc-no-recursion`×15, all SDK `invoke.h`) | ok |
| `check_cppcheck` | 22 warnings + 33 errors across 42 TUs (initial), 1 at hand-off | `findings: none` | ok |
| `check_analyzer` | 3 stable-pass blockers (see below) + ~4,567 alpha report-only | stable blockers: 0 (alpha unchanged, report-only by policy) | ok |
| `check_context` `contracts` `diagnostic-controls` `frontends` `headers` `scripts` | already green at lane/h4 tip | still green after this lane's edits | ok |

## Fix-vs-suppress decisions

### ODR (commit `a891e1de`)

`research_reference_support.hpp` defined `CountingPolicy` member
functions out-of-class in the header, emitting duplicate
typeinfo/vtable/destructor symbols into every including TU. The
implementation moved into `research_reference_support.cpp`, compiled
once through the OBJECT-library path in `Phase3Ports.cmake`. Real fix —
the symbol set is emitted exactly once.

### tidy (commit `73b380e5`, 131 files, +5,391/−4,481)

Bulk `clang-tidy --fix` pass (2,654 → ~1,160) then per-class work.
Mechanical fixes preferred wherever semantics were preserved:

- `bugprone-unchecked-optional-access` (213 raw / 130 sites): converted
  guarded `->`/`*` optional accesses to `.value()` so violations throw
  loudly; each surviving site carries a per-line NOLINT with the
  fixture-contract justification.
- `cppcoreguidelines-missing-std-forward`: real `std::forward` at the
  single/last use. In `replay_reference_support.hpp` `patch_execution`/
  `patch_legacy_execution` invoke the callable twice — only the last
  use is forwarded; earlier calls stay lvalue so a moved-from callable
  is never re-invoked.
- `misc-const-correctness`: `const` added where the check proved no
  mutation (Stepper handles, TemporaryDirectory dirs, ostringstream
  capture pair, IntentSignals splits).
- `modernize-loop-convert`: the two indexed-write loops over mutable
  span buffers keep index form; NOLINT explains that range-for binds a
  copy and the write would not reach the buffer. NOLINT sits on the
  `for` line — NOLINTNEXTLINE two comment lines above was dead weight
  (applies to the literally-next line only).
- `modernize-use-structured-binding`: real `[s0, e0]` decomposition.
- `cppcoreguidelines-avoid-c-arrays` + `pro-bounds-array-to-pointer-decay`:
  the `cases[]` table in `test_native_spindle_controller.cpp:520` was
  converted to `std::array` (also fixed a copy-pasted "snprintf buffer"
  NOLINT justification). snprintf label buffers elsewhere keep the C
  API by design — NOLINT'd per line.
- `cert-err33-c` + `pro-type-vararg`: bounded `snprintf` is the sized-
  label API; ignored return intentional — NOLINT'd per line.
- `reinterpret_cast` byte→char I/O views, `const_cast` mutation seams,
  `concurrency-mt-unsafe` (single-threaded test envs), `signed-bitwise`
  fixed-width literals, `use-after-move` (the moved-handle read IS the
  test contract), `bugprone-random-generator-seed` (deterministic
  golden parity seed): NOLINT'd per line with justifications.
- Small enums → `std::uint8_t`; `bugprone-misplaced-widening-cast`,
  `special-member-functions`, `throwing-static-initialization`
  (startup fixture tables) handled mechanically or suppressed per line.

Manifest suppression (tool-scoped, with recorded reason):
`misc-no-recursion` attributed by libc++23 to the `std::__invoke`
dispatch frame at `invoke.h:88` — 15 diagnostics across TUs, all in
the SDK header which cannot be NOLINT'd. The first-party recursive
Wire/tree walkers carry their own NOLINTs.

Totals: 434 NOLINT lines added across ~100 files, every one carrying a
justification; zero file-level blankets.

### cppcheck (commit `91dd8d80`, 34 files, +283/−210)

- `mismatchingContainerExpression` in
  `test_native_rider_information_boundary.cpp:190`: **real defect** —
  `only_x.erase(std::ranges::unique(only_x).begin(), only_x.end())`
  mixed iterators from the returned subrange and the container.
  Rebound to the subrange's own iterators.
- `throwInEntryPoint`: per-line suppressions — test failures propagate
  as exceptions into `bt::run`'s FAIL catch by design.
- `accessMoved`: reads of moved-from `unique_ptr` handles are the test
  contract under test.
- `unknownMacro`: `NATIVE_DIAG_*` is the clang `_Pragma` wrapper from
  `diag.hpp`; cppcheck does not resolve it through `-isystem`.
- `nullPointerRedundantCheck`, `mismatchingContainers`: per-line
  suppressions where the idiom is intentional.
- `redundantCopyLocalConst` (found this session): reverting a
  `const TierCfg` copy to `const TierCfg &tier` — references, not
  copies, for the constexpr tier constants.

### analyzer (commit `e415ddad`, 5 files, +41/−11)

Stable-pass blockers were all `bt::require`-guard false positives:
`bt::require`'s `throw` lives in the harness TU, so the analyzer cannot
prove the guarded dereference unreachable:

- `core.uninitialized.UndefReturn` at
  `test_native_platform_recording.cpp:39` — "Returning null reference"
  on `return *slot` after `bt::require(slot != nullptr, …)`. Fixed by
  spelling out the identical `std::runtime_error` guard in-TU
  (same failure mode, same message).
- `core.CallAndMessage` at `test_seated_topology.cpp:90,93` —
  `equality->`/`connect->` after `bt::require(… != nullptr)`. Same
  in-TU guard fix.
- `core.CallAndMessage` at `test_native_pneumatic_model.cpp:621`
  (appeared in the fresh sweep — "2nd function call argument is an
  uninitialized value" on `make_model(tyre, tier)` inside a `[&]`
  lambda): the `const TierCfg &tier` binding to constexpr `kDetail64`
  is lost by the analyzer through the capture. Fixed by passing
  `kDetail64` directly in the lambda call — no suppression needed.
  (N NOLINT works for `clang --analyze`, so the NOLINT that had been
  tried for the tidy pass was replaced by this structural fix.)
- `tools/native_checks.py`: `_SNIPPET_GUTTER` — clang-family diagnostic
  echoes re-print source lines in `   NN | <src>` gutter form; source
  text containing a literal `error:` was previously counted as a
  diagnostic. Gutter lines are now excluded (lane/d3 `a08a64ef`
  equivalent).

Alpha findings (`alpha.*`: IteratorRange, UseAfterLifetimeEnd,
CloneChecker, UnreachableCode, StdVariant, StoreToImmutable, webkit.*)
remain report-only by policy — ~4,567 recorded, none blocking.

## Real bugs found besides the ODR violation

1. `mismatchingContainerExpression` iterator mix in
   `test_native_rider_information_boundary.cpp` (above) — genuine
   cppcheck catch.
2. clang-tidy `--fix` converted `const RiderKinematicState frozen =
   state;` (a load-bearing owned copy — the test asserts the frozen
   snapshot survives mutation) into `const&`, which would have made the
   require tautological. Reverted to the copy + justified NOLINT.
3. Same `--fix` pass rewrote an indexed write over a mutable
   `std::span<double>` into `for (double force : forces)` — writes
   would have gone into copies. Restored indexed form (2 sites).
4. `--fix` produced invalid `.value()->` (optional::value returns T&)
   and one malformed `(text).value()(env->status(), "reason")` call —
   all repaired before any sweep ran green.

## Commits on `lane/h5`

- `a891e1de` `fix(odr): single-compile research_reference_support.cpp via OBJECT library`
- `91dd8d80` `style(sweeps): clear check_cppcheck debt — suppressions + real fixes`
- `e415ddad` `fix(analyzer): clear stable blocking findings — visible-in-TU null guards`
- `73b380e5` `style(sweeps): clear check_tidy debt — mechanical fixes + justified NOLINTs`

Commits are per-sweep-kind at hunk granularity; one hunk in
`test_seated_topology.cpp` carries a `const*` fix alongside the
analyzer guard (noted).

## Verification

- `check_tidy`: `findings: none`, health ok, 15 manifest suppressions
  (run `20261101T051244720775Z` and final gate run).
- `check_cppcheck`: `findings: none`, health ok
  (run `20261101T062532088932Z`-era final pass).
- `check_analyzer`: stable pass clean; last blocker verified per-TU
  (`clang++ --analyze` on `test_native_pneumatic_model.cpp`, rc=0,
  zero diagnostics); alpha report-only unchanged.
- `check_context`, `check_contracts` (6/6), `check_diagnostic_controls`
  (6/6), `check_frontends`, `check_headers`, `check_odr`,
  `check_scripts`: all green.
- Scoped CTest on the semantically-edited binaries
  (`-R 'numeric_completion|pneumatic_model|native_tire|rider_dynamics|
  rider_information_boundary|attachment_wrench|rider_forces|
  spindle_controller|seated_posture_program|seated_topology|
  articulated_config|native_benchmark|runtime_accounting|state_bundle|
  recording_catalog|research_capture|research_recording_lifecycle|
  async_close|platform_recording'`): **20/20 passed**.
- Final `bash tools/run_tests.sh native` (2026-11-01, ~2h40m end to
  end): **sweep stage exit 0** — launcher printed `all produced sweeps
  passed`; per-kind finals from fresh summaries:

  | kind | health | findings |
  |---|---|---|
  | frontends | ok | none |
  | tidy | ok | none (suppress `misc-no-recursion`×15) |
  | analyzer | ok | report-only (stable 0; suppress `cplusplus.Move`×1, `cplusplus.NewDelete`×2) |
  | cppcheck | ok | none |
  | odr | ok | none |
  | scripts | ok | none |
  | headers | ok | none |
  | diagnostic-controls | ok | none |
  | context | ok | none |
  | contracts | — | `status: passed` (all 7 check groups) |

- CTest leg `ctest -LE 'slow|audit|package'`: **179/181 passed**
  (67.36 s). The 2 reds are `migration_T09`
  (`post-advance integration vector matches the oracle`) and
  `migration_T10` (`first transition matches the Python oracle`) —
  the documented frozen-oracle baseline pair, verbatim signatures
  identical to ≥10 prior lane reports (C317, C320, C325, C326,
  C336t2a/b, F4, F5, tail5); untouched physics-oracle code, not a
  regression from this lane's lint/ODR edits.
- Process exit code: **8** (CTest stage; the sweep stage itself is
  green). The gate is as green as the documented baseline allows —
  exit 0 requires resolving the pre-existing oracle pair, a separate
  physics task outside this lane's scope.

## Remaining concerns

- `migration_T09`/`migration_T10` remain baseline-red in the CTest
  leg (frozen-oracle drift in physics captures; documented pair).
  They predate this lane and fail identically at the base — the
  final gate therefore exits 8, not 0.
- ~4,567 alpha analyzer findings stay report-only; none are new this
  lane. If any later get promoted to stable they will block.
- The NOLINT count is large (434) — every suppression carries an
  in-line justification and no file-level blanket exists. The bulk of
  them are three intentional-idiom classes: optional-access under
  fixture contract (118), snprintf bounded-label buffers (115),
  positional-parameter API mirrors (49+).
- `misc-no-recursion` manifest entry is tied to libc++23's
  `invoke.h:88` attribution point; on clang-18/Linux the same
  diagnostics attribute differently and the first-party walkers'
  NOLINTs already cover them — the manifest entry is version-scoped
  and documented in the manifest file.
- Bitwise-identity: all changes are comments/NOLINTs, const/ref,
  forward, array-ification of test tables, or the ODR single-compile —
  no physics or numeric path was touched (`native/src` edits are the
  same NOLINT/fix class in `controls.cpp`, `native_contacts.cpp`,
  `check_reports.cpp`).
