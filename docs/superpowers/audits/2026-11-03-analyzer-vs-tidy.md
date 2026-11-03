# Standalone analyzer pass vs clang-tidy — measurement audit (task 11)

Date: 2026-11-03. Branch: `main3`. HEAD: `6485b957` (`build: use ccache when installed; record it as build_launcher`).
Plan: [docs/superpowers/plans/2026-11-03-verification-speed.md](../plans/2026-11-03-verification-speed.md).
Baseline: [2026-11-03-verification-speed-baseline.md](2026-11-03-verification-speed-baseline.md).

Question: does the dedicated `analyzer` sweep (`clang --analyze`, default checkers
plus `-analyzer-checker=optin` and `-analyzer-checker=cplusplus.Move`;
`native/src/tools/check_frontends.cpp:241-271`) find anything `clang-tidy`
(`native/.clang-tidy`, which enables `clang-analyzer-*` plus explicit
`optin.core`/`optin.cplusplus`/`optin.taint` subsets) does not? Measurement
only — the pass is **not** removed.

## Step 1 — full runs of both kinds on the same code

Commands (run back-to-back on the same working tree, `build/native-release`,
512 translation units each, `jobs: 16`):

```bash
/usr/bin/time build/native-release/bin/bike-native-checks --kind tidy --build build/native-release
/usr/bin/time build/native-release/bin/bike-native-checks --kind analyzer --build build/native-release
```

| kind | exit | wall | user | sys | tool health | findings | suppressions |
|---|---|---|---|---|---|---|---|
| tidy | 0 | 287.62 s (~4.79 min) | 3111.53 s | 124.74 s | ok | none | misc-no-recursion=15 |
| analyzer | 0 | 243.48 s (~4.06 min) | 2523.00 s | 94.61 s | ok | none | cplusplus.NewDelete=1 |

Tools: `clang-tidy` and `clang++` from Homebrew LLVM 23.1.2
(`/opt/homebrew/opt/llvm/bin`, arm64-apple-darwin27.0.0). Summaries:
`build/native-release/native_check_summaries/{tidy,analyzer}.json`
(run_ids `20261103T232958821097Z`, `20261103T233446476163Z`).

## Step 2 — findings compared by location and checker

Comparison unit: `file:line checker` over **all** `records[].diagnostics`
(suppressed included — detection capability, not current suppressions), with
`clang-analyzer-` stripped from tidy checker names. The tidy side is filtered to
analyzer-class checkers (`core|cplusplus|deadcode|optin|security|unix|
nullability|osx|valist|webkit|fuchsia`). Raw diagnostic counts: analyzer 1,
tidy 15 — all 15 tidy diagnostics are `misc-no-recursion` at
`MacOSX.sdk/.../c++/v1/__type_traits/invoke.h:88` (one per TU reaching it;
a tidy-native checker, excluded by the class filter).

| set | count |
|---|---|
| analyzer normalized findings (`an.txt`) | 1 |
| tidy analyzer-class findings (`tidy-an.txt`) | 0 |
| only-analyzer.txt | 1 |
| only-tidy.txt | 0 |

`only-analyzer.txt`, complete:

```
native/tests/sanitizer_control.cpp:49 cplusplus.NewDelete
```

### Per-finding verdict

**`native/tests/sanitizer_control.cpp:49 cplusplus.NewDelete` — REAL
(true positive by design), suppressed on the analyzer side, not emitted by
clang-tidy.**

The diagnostic is `Use of memory after it is released` on the read through
`escaped[index]` — the intentional injected defect of the T15 use-after-free
control: the TU exists to prove the real ASan heap-use-after-free fires in a
fresh subprocess, so the defect must stay a true positive. It is suppressed in
`native/analysis_suppressions.json` (checker `cplusplus.NewDelete`,
`tool_version: "23.1.2"`, with `remove_when` tied to the control's existence),
which is why the sweep reports `findings: none`.

clang-tidy does **not** emit this diagnostic on the same TU even though
`clang-analyzer-cplusplus.NewDelete` is enabled in its checker set (verified in
`--list-checks`, Step 3). A direct single-TU rerun —
`clang-tidy -p build/native-release --extra-arg=-Wno-error --quiet
native/tests/sanitizer_control.cpp` — reproduces the sweep result: zero
diagnostics. So the gap is not checker enablement; the two drivers run the same
checker differently (`clang --analyze`'s CSA invocation vs clang-tidy's
embedded analyzer path), and on this toolchain the finding survives only under
`clang --analyze`. There is no `clang-analyzer-cplusplus.NewDelete` manifest
entry — none is needed because tidy never produces the diagnostic; if a future
toolchain makes tidy emit it, the sweep would fail unsuppressed and a twin
suppression record would be required (the same pattern already used for
`cplusplus.Move`/`core.CallAndMessage`, which carry both spellings).

## Step 3 — enabled checker sets compared

```bash
clang-tidy -p build/native-release --list-checks native/src/platform/process.cpp   # enabled set
clang -cc1 -analyzer-checker-help                                                  # analyzer packages
```

| set | count |
|---|---|
| checkers enabled in clang-tidy (`clang-analyzer-*` set) | 131 |
| `-cc1` checkers in audited packages (`core|cplusplus|deadcode|optin|security|unix|nullability|osx|valist`) | 108 |
| **analyzer-only-checkers.txt** (`optin.*` ∪ `cplusplus.Move` not enabled in tidy) | **0** |

The analyzer-only list is **empty**: `clang-analyzer-*` in `native/.clang-tidy`
already enables every CSA checker clang-tidy exposes — all 16 `optin.*`
checkers (including `optin.mpi`, `optin.osx.*`, `optin.performance.*`,
`optin.portability.UnixAPI`, beyond the three explicit subsets in the config —
those lines are redundant documentation) and `cplusplus.Move` (which
`check_contracts.cpp:1788-1806` requires `.clang-tidy` to keep enabled).

Reverse direction, for completeness: 22 tidy-enabled checkers do not appear in
the filtered `-analyzer-checker-help` list — hidden/modeling checkers the glob
picks up (`apiModeling.*`, `core.builtin.*`, `core.DynamicTypePropagation`,
`core.NonnilStringConstants`, `cplusplus.SelfAssignment`,
`cplusplus.SmartPtrModeling`, `osx.cocoa.RetainCountBase`,
`security.insecureAPI.SecuritySyntaxChecker`, `unix.DynamicMemoryModeling`,
`unix.cstring.CStringModeling`, the deliberately-excluded `webkit.*`/`fuchsia.*`
packages, and hidden `optin.taint.TaintPropagation`). Most are default-on
modeling checkers, so the analyzer pass runs them too; whether `clang
--analyze` runs `webkit.*`/`fuchsia.*` is a residual reverse-coverage gap this
audit does not close.

## Conclusion for the owner

Checker-set-wise the standalone pass adds nothing: every checker it enables is
already enabled in clang-tidy (`analyzer-only-checkers.txt` is empty).
Detection-wise it is still not equivalent: the run's only analyzer finding —
the intentional T15 use-after-free control — is a real defect clang-tidy does
not surface despite having the checker enabled (verified by direct rerun). The
pass currently costs ~4.1 min per `native`/`full` run and yields zero
unsuppressed findings on the current code.

| Option | Action | Cost |
|---|---|---|
| A | Keep the standalone analyzer pass | ~4.1 min (measured 243.48 s wall) on every `native`/`full` sweep; keeps the only demonstrated detection of `cplusplus.NewDelete`-class defects (T15 control evidence) that clang-tidy misses on this toolchain |
| B | Remove the pass, add `analyzer-only-checkers` to `native/.clang-tidy` | The checker list is empty — nothing to add; cost is instead: rework `check_contracts.cpp` (the `cplusplus.Move` positive control at :265-347 drives `clang --analyze`; the tidy-enablement contract at :1788-1806), docs/TESTING.md and check tooling tests; measured downside: `cplusplus.NewDelete` detection on `sanitizer_control.cpp:49` is lost — tidy never emits it, so the T15 control loses its analyzer-side true-positive evidence |
| C | Keep the pass but gate it to the `full` profile only | ~4.1 min removed from `native` runs; same T15-evidence risk as B on the `native` lane — analyzer regressions would surface only under `full` |

No option selected — owner decides.
