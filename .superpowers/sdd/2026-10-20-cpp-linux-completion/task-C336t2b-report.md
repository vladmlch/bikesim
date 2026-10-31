# Task C336t2b — test-port 3.36 tranche-B report (portability/tooling + execution identity; T53)

**Batch**: 3.36 tranche B — 179 test-port-map records across 11 files
(173 `planned` at lane start; 6 `test_dependency_policy.py` records were
already dispositioned on the integration tip). Files:

| source file | records | port | drop |
|---|---|---|---|
| tests/portability/test_ci_lanes.py | 34 | 22 | 12 |
| tests/portability/test_release_gate.py | 34 | 30 | 4 |
| tests/portability/test_native_dependencies.py | 28 | 18 | 10 |
| tests/portability/test_native_coverage_tools.py | 12 | 11 | 1 |
| tests/portability/test_viewer_launch_platform.py | 11 | 5 | 6 |
| tests/portability/test_worker_platform.py | 11 | 4 | 7 |
| tests/portability/test_development_docs.py | 9 | 9 | 0 |
| tests/portability/test_baseline.py | 4 | 3 | 1 |
| tests/portability/test_headless_imports.py | 3 | 1 | 2 |
| tests/portability/test_dependency_policy.py | 7 | 3 | 4 |
| tests/test_execution_identity.py | 26 | 20 | 6 |
| **total** | **179** | **126** | **53** |

**Lane**: `.worktrees/pfc-lane-c`, branch `lane/c`, base `d20ea74c`
(post-3.36b integration tip). No merge, no push.

## Status: 126 `port`, 53 `drop`, 0 `planned`

Ported surface lands on five native test IDs:

- `migration_T53_portability` — the existing `cmake -P` contract script
  `native/tests/migration/migration_T53_portability.cmake` was extended by
  ~800 lines (now 2361 lines, 86 named cases on darwin-arm64 / 80
  executable + 6 skip-recorded on linux-aarch64) to cover the CI-lane
  definitions and command contracts (`.github/workflows/portability.yml`,
  `tools/ci_presets.sh`, `tools/linux/container.sh`), the release gate /
  checklist / qualification-doc contracts (`tools/run_native_checks.sh`,
  `tools/run_tests.sh`, `docs/releases/python-free-qualification.md`,
  `docs/development/toolchain-matrix.json`), native dependency
  declarations and platform selection (`native/cmake/BikeDependencies.cmake`
  + the `.deps/` pin/lock files), native coverage-tool resolution
  (`native/cmake/CoverageTools.cmake` `bike_qualify_coverage_tools()` —
  explicit `LLVM_PROFDATA`/`LLVM_COV` override, compiler-sibling-first
  resolution, PATH candidates, Darwin-only `xcrun` fallback, LLVM-family
  compiler guard, and the real compile/run/merge/export format probe
  including fail-closed merge diagnostics), development-documentation
  contracts (`docs/development/{linux,macos,troubleshooting}.md`),
  baseline/report contracts, and the dependency-policy checks.
- `reference.execution_identity` — new
  `native/tests/reference/test_execution_identity.cpp` (512 lines) drives
  `bike::replay::validate_execution` / `check_execution` over the real
  record schemas: legacy schema-1/schema-2 records, identity-schema-2
  platform/numerics records (Darwin↔ACCELERATE, Linux↔NUMPY_OPENBLAS
  pairings, 64-char SHA-256 file identities, null `build_id`), native
  executable identity records (`native_source_sha256`,
  `execution_identity`, `mujoco_library_sha256`, `numeric_provider`,
  `executable_sha256`, `source_identity_scope`), top-level and nested
  context mutations, named validation failures, non-mutating validation,
  and field-specific diagnostics. Registered in
  `native/tests/reference/Phase3Ports.cmake` with `ENGINE`, libraries
  `bike_replay bike_recording`, labels `t53 replay portability`.
- `reference.worker_platform` — new
  `native/tests/reference/test_worker_platform.cpp` (129 lines) covers
  `bike::batch::performance_cores()`: positive worker count,
  platform-specific source selection (Darwin
  `hw.perflevel0.physicalcpu` sysctl vs Linux `sched_getaffinity`), and
  the `std::thread::hardware_concurrency()` fallback contract.
  Registered with libraries `bike_batch`, labels `t53 portability`.
- `reference.frontend_windows` / `reference.research_recording_lifecycle`
  — three viewer-launch and two recording-lifecycle records map onto
  existing landed coverage (window-launch platform contract; recording
  lifecycle), documented per record.
- `reference.recording_cli` — the one headless-import record whose
  subject survives natively (CLI launches without an interpreter preload)
  maps onto the existing recording-CLI reference test.

Dropped surface (53 records, per-record evidence in the ledger):
Python-interpreter internals with no native subject — multiprocessing
fork/spawn start-method selection, invalid start-method resource
behavior, missing-spawn fallback, threaded/`mjpython` subprocess worker
probes, argparse async-option forwarding, Python CLI sync/async exit
parity (superseded by the stronger native sample-publication/async-close
tests), interpreter import bookkeeping (`sys.modules` mutations), Python
`argparse`/`shutil.which` plumbing that the shell/CMake toolchain
contracts now cover directly, and aggregator-only bookkeeping such as
the git-porcelain dirty-tree mutation counter (native commit binding is
the read-only `git rev-parse` in `ci_presets.sh`).

No golden fixtures needed: the batch is contract/structural only.

## Verification

| gate | result |
|---|---|
| darwin-arm64 `ctest --test-dir build/native-release -R '^(migration_T53_portability\|reference\.worker_platform\|reference\.execution_identity)$'` | **3/3 Passed** — migration_T53_portability 340.62 s, worker_platform 0.03 s, execution_identity 0.76 s |
| darwin-arm64 `cmake -DREPO_ROOT=$PWD -DSCRATCH_DIR=/tmp/… -P native/tests/migration/migration_T53_portability.cmake` | **passed (86 cases)**, 4 recorded skips (Linux-only fortify gating, two doc-divergence rows, g++-alias LLVM guard) |
| linux-aarch64 `ctest --test-dir /build/clang -R '^migration_T53_portability$'` (container `bike-dev-pfc-lane-c`, Ubuntu Clang 18.1.3) | **Passed, 30.06 s** (test #15), 80 cases with recorded skips |
| linux-aarch64 scoped reference tests (`-R 'worker_platform\|execution_identity'`, `/build/clang`, 236-step build) | **2/2 Passed** |
| `bash tools/run_tests.sh native` (final gate, darwin-arm64) | build + all 9 static-analysis sweeps green; CTest **60/62** — `migration_T09` + `migration_T10`, the documented frozen-oracle baseline reds (verbatim signatures); profile exits at the ctest stage per documented gate-abort; **contracts 12/12** and the **3,086-item native/golden pytest tail run manually** — 3204 passed, 41 failed, byte-identical failure set to the pfc-gate integration worktree's `native` pytest run on this host (2026-10-30): the documented oracle-drift baseline, zero lane-authored reds |

Recorded skips on linux-aarch64: the three instrumented coverage-probe
cases (`coverage_merge_failure_names_the_command`,
`coverage_invalid_export_fails_closed`,
`coverage_real_toolchain_probe_and_resolution_record`) require an
instrumented-link toolchain; the dev container's clang-18 lacks
`libclang_rt.profile-aarch64.a`, so the control binary cannot link
`-fprofile-instr-generate`. The harness now detects this once with a
real link probe and records a named skip; the resolution-only cases
still run (explicit override, missing tool, PATH resolution,
major-version match, LLVM-family guard, xcrun-absent-on-Linux). On
darwin-arm64 the full probe cases execute.

### Port-authored fixes during red→green

- `cmake_policy(SET CMP0054 NEW)` (+ existing CMP0057 NEW) at harness
  top: literal `(`/`if(` text in captured configure output was being
  re-parsed as CMake `if()` syntax.
- Coverage scratch probes pin `CMAKE_CXX_COMPILER` to a clang-family
  compiler: default host `c++`→GCC made the LLVM-family guard fire
  before the intended code path, so the GCC-rejection case now
  exercises the real guard through `bike_qualify_coverage_tools()` and
  the probe cases always select clang++.
- Darwin `xcrun` case relaxed to a marker-only assertion (real xcrun
  always resolves on Darwin hosts).
- Linux GCC build was abandoned for qualification: GCC 13 hits
  pre-existing `-Werror=null-dereference` inside
  `native/src/initialization/artifact_identity.cpp` `std::variant`;
  the qualified Linux lane is Clang (`/usr/bin/clang++`, 18.1.3) per
  `tools/linux/container.sh`.
- `native/src/batch/summary.cpp:51` — added
  `// NOLINT(concurrency-mt-unsafe)` on the `std::getenv("OPENBLAS_NUM_THREADS")`
  read. LLVM-23 clang-tidy flags `getenv` as MT-unsafe; the codebase
  never calls `setenv`/`putenv`, so the read cannot race — same NOLINT
  precedent as `native/src/tools/check_reports.cpp:397`. First
  `run_tests.sh native` attempt stopped at `check_tidy` with this
  single unsuppressed diagnostic; standalone `check_tidy` is green
  after the fix (`findings: none`, suppressions: the pre-existing
  nanobind `clang-analyzer-cplusplus.Move=1`).

## Final gate

`bash tools/run_tests.sh native` (selected build `native/build`,
AppleClang 21, darwin-arm64; ~2.5 h wall under multi-lane host
contention): build green; sweeps — frontends `findings: none`,
tidy `findings: none` (+1 documented nanobind `clang-analyzer-
cplusplus.Move` suppression), analyzer `report-only` (alpha checkers),
cppcheck `findings: none`, odr `findings: none` (g++-16 `-flto -Wodr`),
scripts shellcheck clean, headers `findings: none` (268 headers / 9,508
header-TUs), diagnostic-controls `6 controls, 0 failed`, context
`{explicit_sdk, missing_sdk, default_sdk, default_sdk_fallback,
libcpp_hardening} all passed`. CTest **60/62, exit 8**: `migration_T09`
(`post-advance integration vector matches the oracle`) and
`migration_T10` (`first transition matches the Python oracle`) — the
**documented frozen-oracle baseline pair** (T01-era immutable oracle
captures; native physics deliberately extended past them —
`migration/cpp-linux-completion-evidence.md` documents the identical
pair red in prior final gates, signatures verbatim). Same two reds
reproduced this session on the `pfc-gate`, `pfc-lane-a` and
`pfc-lane-f` worktrees on this host; the lane diff touches no physics
TUs. Per the documented abort pattern the profile stops at CTest; the
tail ran manually: `bike-native-checks --kind contracts` **12/12
passed** (incl. `pure_core_python_free`, `hot_kernel_dependency_free`,
`ctest_contract_cases`), and the native/golden pytest tail
(`-p native_test_reporter`, `tests/reference/test_native_*.py` +
`test_golden_episode*.py`, 3,086 items): **3,204 passed / 41 failed /
1 skipped in 6,426 s**. The 41 failures are oracle-parity tests
(`test_native_runtime_step/commands/rollout`, `research_environment`,
`research_recording/capture`, `distributed_tire`, `pneumatic_tire`,
`numeric_completion`, `ride_frontend`, `viewer_snapshot`) — **the
failure set is byte-identical (41/41) to the pfc-gate integration
worktree's `native` pytest run on this same host on 2026-10-30**:
documented live-oracle drift, zero lane-authored reds. First gate
attempt had also stopped earlier at `check_tidy` on the
`summary.cpp:51` mt-unsafe diagnostic, fixed via NOLINT as described
above.

## Ledger

- `migration/test-port-map.json`: all 179 records for the 11 tranche
  files carry `disposition` (126 `port` / 53 `drop`), `native_test_id`
  naming the landing test (real `ctest::function` ids only — the merge-back
  review round re-anchored four descriptive/nonexistent
  `reference.research_recording_lifecycle`/`frontend_windows` cites onto
  `reference.native_research_recording_lifecycle::*` and real
  `frontend_windows` functions), measured `collected_node_ids` +
  `comparison_class` + `assertions_capture` (pytest --collect-only,
  325 nodes), and dated evidence entries citing the port target or
  the drop rationale; 0 records remain `planned`.
- `migration/python-removal.json`: all 11 file rows updated to
  `action: port_assertions_and_fixture_orchestration` with
  `replacement_paths` naming `migration_T53_portability.cmake`,
  `test_execution_identity.cpp`, `test_worker_platform.cpp` (and the
  pre-landed `test_recording_cli.cpp` / `test_frontend_windows*` /
  lifecycle rows where the records map onto existing coverage), plus
  fresh `sha256`/`line_count` snapshots. Python files stay in place —
  deletion is the T63 gate.

## Concerns / limitations

- The linux-aarch64 dev container cannot link instrumented coverage
  probes (`libclang_rt.profile` absent); the three real-probe cases
  record named skips there and only run on darwin-arm64. If the
  coverage CI lane qualifies on Linux, the container image needs the
  compiler-rt profile runtime.
- Two development-doc records are asserted with documented divergence
  at HEAD (linux.md gained a "Container lane" heading; the "slim
  externals" troubleshooting symptom has no native doc row) — recorded
  as skips rather than weakened assertions.
- `run_tests.sh native` runs the legacy binding graph on darwin-arm64
  only; linux-aarch64 coverage is via the container ctest runs above.

## Merge-back addendum (2026-10-31)

- **Duplicate port discovered at merge:** `tests/test_execution_identity.py`
  was already ported by the earlier-landed lane/d batch 3.35 (commit
  `0f586027`, merged `d062beef`/`b7617d01`) — 26 records, verified green on
  both platforms, registered `reference.execution_identity` first. The
  merge resolved the add/add file conflict and all 26 contested ledger
  rows to the integration (3.35) version; this lane's 512-line variant
  and its 20-port/6-drop ledger disposition were superseded. The batch's
  effective delivered scope is therefore the ten `tests/portability/`
  files (151 lane-changed ledger rows: 104 `planned → port`, 43
  `planned → drop`, plus the four pre-dispositioned
  `test_dependency_policy` drops re-measured) —
  `test_execution_identity.py` lands `planned → port` via 3.35 instead.
- Review-fix commit `094e885b` on lane/c filled measured
  `collected_node_ids`/`comparison_class`/`assertions_capture` on the 176
  placeholder rows, re-anchored four nonexistent/descriptive
  `native_test_id`s and the matching `verification_ids`, and corrected
  this report's ledger bullet.
