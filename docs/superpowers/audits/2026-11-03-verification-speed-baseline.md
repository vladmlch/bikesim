# Verification speed-up — baseline audit (task 0)

Date: 2026-11-03. Branch: `main3`. Plan:
[docs/superpowers/plans/2026-11-03-verification-speed.md](../plans/2026-11-03-verification-speed.md).
Design: [docs/superpowers/specs/2026-11-03-verification-speed-design.md](../specs/2026-11-03-verification-speed-design.md).

## Working copy

- HEAD: `352f51e0fbf4f2850fa8fb9a35a9e182fd201fd4` (`feat(pedaling): restore the reposition config knobs (default off)`)
- Untracked: `.agents/skills/cass/`, `.devin/skills/`, `.junie/`,
  `docs/superpowers/plans/2026-11-03-crank-reposition-native.md`,
  `docs/superpowers/plans/2026-11-03-verification-speed.md`,
  `docs/superpowers/specs/` — left alone, as expected.
- Host: macOS (Darwin), 16 online cores.

## Ordinary tests (ctest -LE 'slow|package' -j8)

Command: `cmake --preset release && cmake --build --preset release && ctest --test-dir build/native-release -LE 'slow|package' -j8 --output-on-failure`

Result: `99% tests passed, 2 tests failed out of 183`, `Total Test time (real) = 46.81 sec`.

Failures (the two the plan names — pre-existing, fixed in tasks 1 and 2):

- `208 - reference.native_research_recording (Failed)`
- `230 - reference.native_numeric_portability (Failed)`

## Tooling tests

Command: `ctest --test-dir build/native-release -R '^(tooling_checks|tooling_platform|tooling_toolchain|tooling_install)$' --output-on-failure`

| Test | Result | Time |
|---|---|---|
| tooling_platform | Passed | 137.66 s |
| tooling_toolchain | Passed | 2.54 s |
| tooling_install | Passed | 0.41 s |
| tooling_checks | **Failed** | 1401.99 s |

`75% tests passed, 1 tests failed out of 4`; `Total Test time (real) = 1542.62 sec`.

`tooling_checks` fixture-contract failures, verbatim (pre-existing — recorded, not
fixed; tasks 3, 5, 7 compare against this list):

    t56_ctx_binding_null_interpreter: stderr missing 'configured Python executable is missing or is not a path':

    t56_ctx_binding_missing_interpreter: stderr missing 'configured Python executable is missing or is not a path':

    t56_ctx_binding_null_identity: stderr missing 'binding native check context python must be an object':

    t56_contracts_stale_extension: stderr missing 'expected exactly one bike_native extension':
    native contract checks failed: cannot read CTest registration file
    build/native-release/tooling_checks/contracts-stale-extension/CTestTestfile.cmake: unreadable

    b29_ext_zero: stderr missing 'expected exactly one bike_native extension':
    native contract checks passed: build/native-release/tooling_checks/b29fx-extzero/build
    (contract list printed; engine_call_wrappers: pending)

    b29_rtsan_qualified: control build failed (2):
    make: *** No rule to make target `native_rtsan_control'.  Stop.

Also observed: `b29_cov_non_clang: GNU driver on Darwin is rejected before the
coverage-family gate — skipped` (a skip, not a failure).

## After task 5

(to be filled by task 5: real analyzer run time after the alpha half is removed)

## After

(to be filled by task 13: stage table before/after, level-1 time, final timings.tsv line, deferred items)
