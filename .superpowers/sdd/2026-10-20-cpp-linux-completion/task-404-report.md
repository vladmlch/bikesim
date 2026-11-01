# Task 4.04 report — No-Python enforcement (T62)

**Lane**: `.worktrees/pfc-lane-f`, branch `lane/f3`, base `907687f1`
(integration tip). Darwin-arm64 only (container.sh guarded; no docker leg).
Build tree `build/native-release` (Ninja, AppleClang, MuJoCo 3.12.0,
existing configure).

Scope per plan `2026-10-20-cpp-linux-completion.md` Task 4.04 and plan-09
(`2026-10-14-python-free-cpp-09-qualification-removal.md`) T62: wire the
T56-produced `bike-python-audit` into ctest with measured-failure
source/documentation gates, an installed-tree positive gate, and
per-class negative controls. The audit binary and
`migration/no-python-policy.json` already existed (T56); `package_smoke`
already ran `--mode package` inline (Task 3.44). This task adds the
registrations and the detection-proof suite.

## Semantics chosen (per the plan)

- `python_audit_source` — `bike-python-audit --mode source --root
  ${CMAKE_SOURCE_DIR} --policy migration/no-python-policy.json`,
  `WILL_FAIL TRUE`, label `audit`. Reports the remaining transition
  surface: removal-ledger paths still present + interpreter-invocation
  edges in first-party shell/CMake/CI surfaces. Task 4.06 removes
  `WILL_FAIL` after deletion; registered in `native/cmake/Checks.cmake`.
- `python_audit_documentation` — same root/policy, `--mode
  documentation`; `WILL_FAIL TRUE`, label `audit`; Task 4.07 removes
  `WILL_FAIL` after the docs rewrite.
- `no_python` — plan-09 T62 script `native/tests/package/no_python.cmake`
  (new): CPack TGZ → extract → `--mode package` over the real install
  tree, exit-0 contract. Registered in `packaging/CMakeLists.txt` beside
  `package_smoke` (it needs `_package_binary_file_name`), labels
  `audit;package`, `RUN_SERIAL`.
- `audit_negative_controls` — plan-09 T62 script
  `native/tests/package/audit_negative_controls.cmake` (new): CPack TGZ →
  one working copy → inject each artifact class **one at a time**, audit
  must exit non-zero AND name the injected edge; removal must restore
  clean. Labels `audit;package`, `RUN_SERIAL`.

The tests pass **now** (before T63) by construction: the package audits
run against the *staged package output* (Python-free already), and the
source/documentation audits are `WILL_FAIL` measured failures — matching
checkpoint 4-A's expected `-L audit` outcome (`audit_negative_controls`
passes; the two source/doc audits pass by failing).

## WILL_FAIL + FAIL_REGULAR_EXPRESSION — verified semantics

Empirically probed (scratch ctest project, cmake 4.x host): `WILL_FAIL`
inverts ctest's **determined** result, including regex results —
`WILL_FAIL + FAIL_REGULAR_EXPRESSION` with matching output **passes**.
So `FAIL_REGULAR_EXPRESSION` is set to the audit's own violation summary
(`python source audit: [0-9]+ remaining edge`,
`python documentation audit: [0-9]+ active invocation`): the test passes
only when the audit fails *with a real findings report*. Exit 0 (clean),
exit 2 (usage error), `audit failed` exceptions and crashes produce no
summary match → determined pass → inverted → **test fails**. A vacuous
or broken audit can never pass as an expected failure. (The intuitive
alternative — FAIL regex on the error signature — would have inverted
usage errors into passes.)

## Binary change (native/src/tools/python_audit_main.cpp, +9 lines)

An explicitly supplied `--policy` that does not exist now throws
`AuditError` → `audit failed: missing no-python policy: <path>` exit 1.
Previously any missing policy silently degraded to the empty policy
object — an empty `package_mode` contains no forbidden classes, so a
typo'd `AUDIT_POLICY` would have made `no_python`/`package_smoke` pass
**vacuously** (the T62 fail-closed contract forbids that; the default
policy path still degrades legitimately since package roots lack the
repo-resident file). Verified: `--policy /nonexistent.json` → exit 1,
message matches the ctest error signature.

## Negative-control results (darwin-arm64, real package copy)

Baseline clean → each injection rejected with its named edge → restored
clean. All detections on stderr `python package audit:` lines:

| Control | Injected artifact | Edge reported |
|---|---|---|
| py_source | `share/bike_sim/injected_cycle.py` | `injected_cycle.py` |
| pyi_stub | `share/bike_sim/injected_stubs.pyi` | `injected_stubs.pyi` |
| pyc_bytecode | `share/bike_sim/injected_bytecode.pyc` | `injected_bytecode.pyc` |
| wheel | `share/wheels/bike_sim-1.0-py3-none-any.whl` | `bike_sim-1.0-py3-none-any.whl` |
| notebook | `share/notebooks/injected.ipynb` | `injected.ipynb` |
| shebang_env | `bin/bike-tool-env` `#!/usr/bin/env python3` | `interpreter shebang '#!/usr/bin/env python3'` |
| shebang_absolute | `bin/bike-tool-abs` `#!/usr/bin/python3.13` | `interpreter shebang '#!/usr/bin/python3.13'` |
| site_packages | `lib/site-packages/` | `site-packages` |
| venv_marker | `.venv/` | `.venv` |
| dist_info | `lib/bike_sim-1.0.dist-info/` | `.dist-info` |
| python_version_dir | `lib/python3.13/` | `python3.13` |
| linked_libpython | stub `-dynamiclib -install_name /Library/Frameworks/Python.framework/Versions/3.13/Python` + exe linked against it, into `bin/`/`lib/` | `Python.framework` (via `otool -L`) |

Benign controls (must NOT trip): `share/pyfloat.hpp` and
`share/notes/python_history.md` prose — accepted. The stub needs no
CPython (per-plan: Darwin `-dynamiclib -install_name`, Linux
`-shared -Wl,-soname,libpython3.13.so.1.0` — both branches implemented,
Darwin leg verified; the exe calls the stub function so the link edge is
load-bearing).

## ctest results

```
ctest --test-dir build/native-release -R 'python_audit|no_python|audit_negative' --output-on-failure
  19 python_audit_source          Passed  8.96 s   (WILL_FAIL — measured failure)
  20 python_audit_documentation   Passed  6.29 s   (WILL_FAIL — measured failure)
 234 no_python                    Passed 10.12 s   (package audit clean)
 235 audit_negative_controls      Passed 22.62 s   (12 detections + benign + restore)
```

`ctest -N -L audit` lists exactly these four (`golden_completeness` is
Task 4.01's, not in this lane). Exit codes from direct runs: source → 1,
documentation → 1, package on real tree → 0, package per injection → 1.

## Source-mode stderr list (measured-failure evidence)

610 edges total under `${CMAKE_SOURCE_DIR}`:

- **535 removal-ledger rows** (`tracked Python source still present`),
  top owners: T53 ×99, T52 ×88, T26 ×58, T63 ×33, T54 ×30, T49 ×13,
  T47 ×12, T57 ×11, T55 ×11, T15 ×11, T12 ×11, T34 ×10 (remainder <10).
- **75 interpreter-invocation edges** (`interpreter invocation token`):
  `native/tests/migration/migration_T56_check.cmake` ×14 (its own audit
  fixtures — 4.06 already plans its exemption), `tools/run_tests.sh` ×13,
  `migration_T53_portability.cmake` ×10, `tools/native_coverage.sh` ×9,
  `tools/make_offline_bundle.sh` ×4, `native/cmake/BikeDependencies.cmake`
  ×4, `native/tests/package/audit_negative_controls.cmake` ×3 (this
  task's fixture strings — see concerns), `migration_T53_numerics` ×2,
  `migration_T336_portability` ×2, `native/CMakeLists.txt` ×2,
  `tools/linux/container.sh`, `Provenance.cmake`,
  `BikeCompilerPolicy.cmake`, `BikeCheckContext.cmake` ×1 each, plus 8
  hits in stale `native/build*/` scratch trees (4.06's expanded
  `exempt_globs` covers `native/build*/**`).

Documentation mode: 84 active invocation(s) — `README.md`,
`docs/development/{dependency-matrix,linux,...}.md`,
`tools/research/alpine_osm_dem/README.md` etc. (`docs/superpowers/**`,
`docs/archive/**`, `migration/**` exempt).

## Files

- `native/src/tools/python_audit_main.cpp` — +9 lines (explicit `--policy`
  fail-closed check).
- `native/cmake/Checks.cmake` — +`python_audit_source`,
  `python_audit_documentation` registrations (WILL_FAIL + summary-regex +
  label `audit`), header bullet.
- `native/tests/package/no_python.cmake` — new (installed-tree audit
  script test).
- `native/tests/package/audit_negative_controls.cmake` — new (per-class
  controls above).
- `packaging/CMakeLists.txt` — +`no_python`, `audit_negative_controls`
  registrations.
- No `migration/python-removal.json`/`test-port-map.json` rows: T62 owns
  no Python source rows or ported-test records (infra task; the ledgers
  carry no T62 rows — verified).

## Verification

- `cmake --build build/native-release --target bike-python-audit` clean.
- The four registrations green (above); `-L audit` selector correct.
- WILL_FAIL semantics probed in a scratch ctest project (both regex and
  exit-code paths).
- `bash tools/run_tests.sh native` on the selected legacy build
  (`native/build`, bindings-on — the gate's ctest surface does not include
  the root-graph registrations or the audit TU):
  - all six `check_*` sweeps green: `frontends` 334 TU none, `tidy` none,
    `analyzer` report-only (alpha never blocks), `cppcheck` none,
    `odr` none, `scripts` clean;
  - `headers` 9570 none, `diagnostic-controls` none, `context` 52 none;
  - legacy `ctest --test-dir native/build`: **60/62 (exit 8)** —
    `migration_T09` "post-advance integration vector matches the oracle"
    and `migration_T10` "first transition matches the Python oracle" are
    verbatim the documented frozen-oracle baseline pair
    (`migration/full-gate-2026-10-31.md` Darwin table "60/62 — T09, T10
    (documented baseline)"). Zero new reds. The script aborts at this
    ctest step (`set -e`), so contracts + the pytest tail did not run in
    this invocation; nothing in the diff is reachable by them (the audit
    tool is not a Python extension and the cmake additions are test
    registrations only). The failed binaries live in the bindings-on
    graph which does not compile `python_audit_main.cpp`, so the reds
    are causally independent of this diff — no stash+rerun was needed.
- Out-of-scope observation (recorded, not caused): a manual
  `check_frontends` on `build/native-release` (the standalone graph —
  505 sources / 512 TUs via `/opt/homebrew/bin/g++-16` dialect probes) is
  **red at HEAD**: 37 TUs with `useless-cast`/`range-loop-construct`
  diagnostics, all in recent `native/tests/reference/*` port TUs
  (e.g. `replay_reference_support.hpp:372`, `rider_control_reference_support.hpp:336`,
  `test_native_runtime_step.cpp:192`, `test_planar_arms.cpp:460`,
  `test_recording_capture.cpp:402`). `python_audit_main.cpp`'s own
  frontend log is empty (clean under both AppleClang and the g++
  dialect). The `native` profile does not run the standalone-graph
  frontends sweep; flagged for the integration lane.

## Concerns / handoff notes

1. **4.06 exemption**: `native/tests/package/audit_negative_controls.cmake`
   trips the source audit ×3 (its own `python3`/shebang fixture strings) —
   same class as the T56 script. When 4.06 rewrites
   `source_mode.exempt_globs`, add this file alongside
   `native/tests/migration/migration_T56_check.cmake` (its `python3.13`
   dir-injection string and the two shebang literals are detection
   fixtures, not invocations).
2. `WILL_FAIL` inversion subtlety is documented in `Checks.cmake`; if
   anyone switches to `PASS_REGULAR_EXPRESSION` later the contract
   silently flips (determined-pass inverts to fail).
3. Linux leg: the `else()` branch builds `libpython3.13.so.1.0` with
   `-fPIC -shared -Wl,-soname` and expects `readelf` on PATH — untested
   here (darwin-only lane). The audit itself throws `AuditError` if the
   platform inspection tool is missing, so a tool gap surfaces loudly.
4. `no_python` and `audit_negative_controls` each re-run CPack (~10 s +
   ~23 s observed) — self-contained by design so `-R` selection works;
   `RUN_SERIAL` guards the shared CPack scratch conventions.
