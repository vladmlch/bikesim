# Task 4.03 — Release qualification run (T61), darwin-arm64 wave

**Lane**: `.worktrees/pfc-lane-e`, branch `lane/e4`, integration tip
`c5e114f9159a1b5fcdf074123c72a0bb653dab84` ("merge: lane/h5 darwin
release-graph sweep debt"). **Darwin/macOS arm64 only** — no container use
(`tools/linux/container.sh` is guarded; no docker invocation was made).
Linux/aarch64 rows are deferred to the Linux wave and remain unqualified.

**Host**: Mac16,5 (Apple M4 Max, 16 cores, 64 GiB), macOS 27.0.1
(26A434), Darwin kernel 27.0.0 arm64. Toolchain: AppleClang
21.0.0.21000401, CMake 4.4.3, ninja via `cmake --workflow`. libc++
hardening `EXTENSIVE`, `-ffp-contract=off`, `-mbranch-protection=standard`
(per `output/ci/<lane>/bike_policy_report.json`).

**Date**: 2026-11-01 (CET).

## Scope executed vs. deferred

Executed (user-scoped 4.03 lanes): clean configure + build; staged
`cmake --install --prefix <scratch>`; CPack TGZ binary + source packages;
`-L package` and `-L audit` legs with `--no-tests=error`; installed-CLI
contract surface against `migration/command-map.json` /
`migration/cli-contracts.json`; `install_smoke` + `migration_T336r` /
`migration_T336` portability arms; `-L slow` enumeration of the
documented exceptions.

The plan's Step-1 L3 gate is the literal `bash tools/run_tests.sh full`
(= native profile + explicit `-L slow` / `-L audit` / `-L package` legs).
This wave ran the same stages through `ci_presets.sh release package`
(workflow + sweeps = the native profile) plus the three explicit label
legs invoked directly — identical coverage; the `slow` leg's exit-8 on
the two documented reds means a literal `full` run at this tip would
also exit non-zero, which the 2026-10-31 ruling keeps as known
exceptions, not release blockers.

Deferred (stated honestly, not implied green): `asan`, `coverage`,
`gui`, Linux lanes, offline rehearsal, and manual macOS GUI acceptance
(user action). The plan-09 T61 artifacts `migration/release-evidence.json`,
`migration/capability-matrix.json`, `migration/compatibility-approvals.json`
require the two-platform capability close-out and are **not produced** by
this Darwin-only wave — they remain T61/4.08 deliverables.

## Per-lane results

| # | Lane | Command | Exit | Evidence |
|---|---|---|---|---|
| 1 | dep acquisition (retry) | `bash tools/linux/acquire_deps.sh native/dependencies.lock.json .deps darwin-arm64` (inside `ci_presets.sh`) | 0 | 7 deps SHA-256 verified + re-extracted from `.deps/_archives` — see "Defect found" note; `/tmp/403-release-package.log` lines 1-33 |
| 2 | clean configure | `cmake --workflow --preset release` step 1 | 0 | configure 21.8 s + generate 1.1 s, AppleClang 21.0.0.21000401 (`403-release-package.log` lines 35-95) |
| 3 | clean build | workflow step 2 | 0 | 772/772 ninja steps (`log` lines 96-930); one benign `ld: warning: ignoring duplicate libraries: 'libbike_platform.a'` |
| 4 | ordinary CTest | workflow step 3 (excludes `slow\|audit\|package`) | 0 | **181/181 passed**, incl. `install_smoke` 9.53 s, `migration_T336r_portability` 1.12 s (`log` lines 933-1297) |
| 5 | native check sweeps | `bash tools/run_native_checks.sh build/native-release` | 0 | all 10 kinds green — table below (`log` lines 1328-8971) |
| 6 | staged install | `cmake --install build/native-release --prefix build/qualify-403/prefix` | 0 | 242 MiB prefix: 15 bins, `include/bike/policy_v1.h`, `lib/lib{duckdb,mujoco.3.12.0}.dylib`, `share/bike_sim` + `share/licenses` |
| 7 | `package_smoke` (workflow leg) | `ctest --test-dir build/native-release -R '^package_smoke$' --no-tests=error --output-on-failure` | 0 | 76.77 s (`log` lines 8973-8976); extracts both TGZs, exercises every packaged `bike-* --help`, diagnostic `bike-ride` run, package-mode python audit, source manifest digest check |
| 8 | binary CPack | `cpack --config build/native-release/CPackConfig.cmake -G TGZ` | 0 | `bike-sim-0.0.0-darwin-arm64.tar.gz`, 87 527 058 B |
| 9 | source CPack | `cpack --config build/native-release/CPackSourceConfig.cmake -G TGZ` | 0 | `bike-sim-0.0.0-source.tar.gz`, 202 436 223 B |
| 10 | `-L package` leg | `ctest --test-dir build/native-release -L package --no-tests=error --output-on-failure` | 0 | **3/3**: `package_smoke` 73.14 s, `no_python` 12.86 s, `audit_negative_controls` 25.33 s |
| 11 | `-L audit` leg | `ctest --test-dir build/native-release -L audit --no-tests=error --output-on-failure` | 0 | **4/4**: `python_audit_source` 33.65 s + `python_audit_documentation` 17.78 s (both WILL_FAIL, expected-fail → counted pass), `no_python` 12.01 s, `audit_negative_controls` 23.56 s |
| 12 | `-L slow` leg | `ctest --test-dir build/native-release -L slow -j4 --no-tests=error --output-on-failure` | 8 | **47/49** — the two reds are documented exceptions only; see "Exceptions" |
| 13 | install-mode arms | `ctest -R '^(install_smoke\|migration_T336_portability\|migration_T336r_portability)$' --no-tests=error --output-on-failure` | 0 | install_smoke 8.56 s, T336 4.66 s, T336r 1.03 s |
| 14 | installed CLI surface | `build/qualify-403/prefix/bin/bike-*` invocations | see below | contract transcript vs `cli-contracts.json`/`command-map.json` |
| 15 | whole workflow | `bash tools/ci_presets.sh release package` | **0** | `output/ci/Darwin-arm64-release-package/exit-code.txt` = `0`, `commit.txt` = c5e114f9; transcript `/tmp/403-release-package.log` |

## Sweep detail (workflow `run_native_checks.sh`, fixed contract order)

| sweep | tool | result |
|---|---|---|
| frontends | (built-in) | ok, findings none (505 src / 506 TUs) |
| tidy | clang-tidy | ok, findings none (505/506) |
| analyzer | `/opt/homebrew/opt/llvm/bin/clang++` | ok, findings **report-only** (alpha checkers print, never block) — see provenance note below |
| cppcheck | cppcheck | ok, findings none (505/506) |
| odr | `/opt/homebrew/bin/g++-16` | ok, findings none (505/506) |
| scripts | shellcheck 0.11.0 | ok, 18 scripts clean |
| headers | `/usr/bin/c++` | ok, findings none (253 headers / 2409 header TUs) |
| diagnostic-controls | built-in | ok, 6 controls / 0 failed |
| context | built-in | ok — `explicit_sdk`/`default_sdk`/`missing_sdk`(rejected)/`default_sdk_fallback`/`libcpp_hardening` all as designed |
| contracts | built-in | **all 12 checks passed**: provider_binding_boundary, manifest_selection, iso_language_mode_and_hardening, analysis_policy, selected_artifact_assertions, contract_source_manifest_membership, ctest_contract_cases, engine_call_wrappers, benchmark_manifest_membership, **pure_core_python_free**, **hot_kernel_dependency_free** (loader_provenance not-applicable) |

`native launcher: all produced sweeps passed` (`log` line 8971).

## Artifacts and SHA-256

| artifact | sha256 |
|---|---|
| `build/native-release/packages/bike-sim-0.0.0-darwin-arm64.tar.gz` (87 527 058 B) | `de0f6870fd6f80e0ab8ad417b2a7e3229c4ba751c7d2b4a9ce7f932490826d7b` |
| `build/native-release/packages/bike-sim-0.0.0-source.tar.gz` (202 436 223 B) | `c75d1978f082e6066c6e806713a7c63099bec20e5ba5579f6d34fdcbb163b8d1` |
| evidence dir | `output/ci/Darwin-arm64-release-package/` (`exit-code.txt`=0, `commit.txt`=c5e114f9, `Testing/`, `native_check_summaries/`, `native_check_logs/`, `ctest-junit.xml`, `package_smoke/`, `packages/`, `compile_commands.json`, `bike_policy_report.json`, `native_check_context.json`) |
| staged prefix | `build/qualify-403/prefix` (242 MiB; `LC_RPATH=@loader_path/../lib` — relocatable; zero Python load commands in all 15 binaries via `otool -L`; zero `.py`/`.pyc`/`site-packages` files) |

## Installed CLI contract (staged prefix, 15 `bike-*` binaries)

All 13 user-facing entry points from `command-map.json` plus
`native_distance_tests`/`native_station_tests`:

- `--help` → exit **0** on all 13 entry points (spot-reverified
  post-packaging: `bike-ride --list-tracks` → 0 printing the track table;
  `bike-sim`, `bike-recording`, `bike-native-ride`, `bike-research-sweep`
  `--help` → 0).
- `--bogus-flag` → exit **2** on all 13 (matches
  `cli-contracts.json` `result: subprocess, exit: 2` expectation).
- `bike-batch --dry-run --spec share/bike_sim/examples/research/batch_seed_sweep.toml --out <tmp>` → exit 0, expanded the 12 seeded jobs without executing.
- `bike-ride --list-tracks` → 0; `--dump-track flat` → 0.
- `bike-settle` → 0, wrote a valid `.npz`.
- `bike-export --output-dir … --json-only` → 0, wrote `coordinates.json`.
- `native_distance_tests` / `native_station_tests` → 0.
- Diagnostic baselines (documented, not release failures):
  `bike-ride` representative run → exit 2 with
  `outcome.reason="duration_reached"`, `first_failure="energy.constraint_work"`
  (the same documented diagnostic shape `package_smoke` asserts);
  `bike-measure-realtime` deliberately-short run → exit 1 with
  `invalid_controller`/`insufficient_simulated_window` (intentionally
  undersized window — diagnostic, not a gate failure);
  `bike-research` short run → exit 3 (invalid reference-step family) /
  diagnostic run → exit 2 (same documented energy.constraint_work shape);
  `bike-recording overview` without path → exit 2 (usage).

## Documented exceptions — honest enumeration

The ledger-ruling set (`migration/ledger-reconciliation-2026-10-31.md` §7,
2026-10-31 user ruling) holds **seven `planned` port-map records** whose
un-exception conditions are physics/contract work — *not* release blockers
by ruling. Current darwin-arm64 runtime state at c5e114f9:

| ledger record (planned) | native leg | this wave's result |
|---|---|---|
| `test_tissue_damping_does_not_eat_the_muscle_budget` | `reference.coupled_rider_task` | leg PASSED (1576.55 s) with the original `passive < 0.3·positive` premise asserted (`test_coupled_rider_task.cpp:185-186`) — the darwin red is resolved at this tip; record stays `planned` because the ruling's un-exception condition requires re-measuring **both** platforms (linux leg owed) |
| `test_pneumatic_golden_episode_event_rich` | `reference.native_pneumatic_tire` | leg PASSED — lane/h2 2026-10-31 darwin golden recapture landed; record stays `planned` pending the linux fixture recapture |
| `test_pneumatic_probe_and_clock` | `reference.native_pneumatic_tire` | **parked in source** (`test_native_pneumatic_tire.cpp:1137-1142`, `[[maybe_unused]]` — ~1e-12 `interval 1` seal_bootstrap-chain gap; no runtime leg exists) |
| `test_flat_reaches_25_kmh_within_10_s` | `reference.realistic_pedelec_acceptance` | leg PASSED; emitted metric still below old bar (`cadence_in_band=0.7651 < 0.8`) — ported contract records the measured baseline |
| `test_motor_follows_the_rider_through_a_shift_without_an_extra_cut` | same | leg PASSED; emitted `shifts=5 checked=4 failing=4 worst_ratio=0.0197` |
| `test_fifteen_percent_climb_holds_12_kmh` | same | leg PASSED; `late_min_kmh=13.488`, `late_violations=0` (was 364/538) |
| `test_savage_is_ridden_to_the_end` | same | leg PASSED; emitted `x_max=58.352 < 120 m` stall + `late_violations=354` + `reason=crash:loop_out` |

The `-L slow` run's **two actual reds** are both documented:

1. `reference.native_pneumatic_tire` →
   `test_pneumatic_observations_declare_multi_contact`
   `front.normal_load_n[0] actual=0x407aec8221e9b47c vs expected=0x407aec8221144263`
   — the lane/h2-documented ~8e-7 `seal_bootstrap`-vs-`capture_bootstrap`
   construction-path parity gap (same seal-path family as the two §7
   pneumatic records); port-map `golden` row carries the h2 evidence
   verbatim. linux fixture untouched by ruling.
2. `reference.golden_episode_tool` → `FAIL task 3.03 oracle acquisition
   required … episode.npz` — the documented 3.03 fixture-debt family
   (`golden=<missing>` on `energy.closure_*` channels), carried as a
   known baseline red since the 3.21a merge-back notes.

No *undocumented* red was observed in any leg.

## `python_audit_source` expected-failure detail

Direct run `bike-python-audit --mode source --root $PWD` → exit **1**
(expected; the ctest is WILL_FAIL): **611 remaining edge(s)** — tracked
`.py` files (636 under the tree) plus interpreter-invocation tokens in
`native/tests/migration/migration_T56_check.cmake`,
`native/tests/package/audit_negative_controls.cmake`,
`tools/linux/container.sh`, `tools/make_offline_bundle.sh`,
`tools/native_coverage.sh` and friends. This list is the T62/T63 deletion
surface; the audit gate is doing exactly its measured-failure job.
`python_audit_documentation` likewise expected-fails (docs still cite the
Python workflow) until Task 4.07.

## Defects found / fixed

**Fixed (process, not source):**
- First `ci_presets.sh release package` attempt exited 1 before configure:
  `dependency acquisition: destination already contains cairo-1.18.4`.
  Cause: `.deps/` held extracted trees while `acquire_deps.sh`
  intentionally rejects populated destinations. Repair: preserved
  `.deps/_archives`, removed extracted dirs, re-ran acquisition — all
  seven archives re-verified SHA-256 and re-extracted, no network fetch.
- Two CLI-probe measurement errors corrected during the session
  (`PIPESTATUS` mishandling): all unknown-option exits verified as real
  binary exits (2), not pipe artifacts.
- `analyzer.json` provenance (concurrency, self-inflicted): while the
  workflow ran its `headers` sweep, the parallel `-L slow` leg's
  `migration_T56` executed its designed negative controls
  (`--kind analyzer` with a crashing planted tool, `--kind
  diagnostic-controls` with a silent tool) against the same build dir,
  overwriting `native_check_summaries/analyzer.json` (→ `tool_health:
  failed`) and `diagnostic-controls.json` (→ `status: failed`) at 21:52.
  The 22:13 evidence copy captured the polluted `analyzer.json`. The
  workflow transcript itself is authoritative and unambiguous (`tool
  health: ok`, `findings: report-only`, lines 7911-7914). A standalone
  truthful re-run `build/native-release/bin/bike-native-checks --kind
  analyzer --build build/native-release` was launched to restore the
  summary; `diagnostic-controls.json`/`context.json`/`contracts.json`
  were already rewritten by the workflow's own later sweeps (and T56's
  positive produced-build controls). The restoring re-run finished:
  `--kind analyzer` standalone → **exit 0**, `tool_health: ok`,
  `findings: report-only` (matching the workflow's original result);
  the fresh `analyzer.json` + its log dir were copied over the polluted
  evidence copy, and
  `output/ci/Darwin-arm64-release-package/PROVENANCE-403.md` records
  the full three-run provenance (workflow green / T56 injection /
  restoring re-run). `scripts.json` was re-run standalone → passed.
  Lesson for future lanes: do not run `-L slow` (contains `migration_T56`
  check-machinery tests) concurrently with a sweep workflow.

**No source/packaging defect found** — no CPack/install/CLI patch was
needed; nothing was forced.

## Ledger actions

`migration/python-removal.json`: **0 rows flipped** — T61 owns zero rows
(verified: `owner_task` histogram has no T61; `task-graph.json` T61
`owned_sources: []`, targets are the four evidence files). The ledger
rows naming the observed reds are T52-owned (`golden`/`planned`) and
belong to their port lanes.

`migration/test-port-map.json`: **0 rows flipped** — same reasoning; the
seven `planned` exceptions remain `planned` by ruling (un-exception
conditions unmet), the h2 pneumatic row keeps its documented-red evidence.
No row's verification IDs were touched; nothing claimed `verified` on
4.03 evidence alone.

`docs/releases/python-free-qualification.md`: darwin rows filled (see
diff) — `macos-arm64-release` and `package-macos` closed on this run's
evidence; `macos-arm64-asan`/`coverage` marked **Not run — deferred**;
linux/offline/gui rows unchanged as deferred/pending.

`migration/cpp-linux-completion-evidence.md`: this wave appended as a
new dated section.

## Remaining blockers for Tasks 4.05–4.08

1. Python source deletion (T63) — 611 audit edges / 636 `.py` files still
   present by design; `python_audit_source` must flip from WILL_FAIL to a
   real pass only after deletion (4.06).
2. `python_audit_documentation` remains WILL_FAIL until the 4.07 doc
   rewrite; `docs/reference/README-provenance.md` (4.05) is still owed
   (22 `tools/research/**` rows cite it).
3. The seven `planned` ledger records stay blocked on physics/contract
   re-scope (FT-3/FT-4/FT-5/FT-6 territory), not ledger work.
4. `reference.native_pneumatic_tire::test_pneumatic_observations_declare_multi_contact`
   still red (~8e-7 seal-path parity gap) + `test_pneumatic_probe_and_clock`
   parked in source — both need the seal_bootstrap-chain root-cause work
   and a linux fixture recapture.
5. `reference.golden_episode_tool` needs the T03-era `episode.npz`
   oracle acquisition.
6. ASan/coverage darwin lanes and all linux-aarch64 lanes are unrun —
   the checklist cannot fully close before the linux wave and the
   sanitizer/coverage passes.
7. macOS manual GUI acceptance remains a user action on a real display.
8. plan-09 T61 evidence artifacts (`release-evidence.json`,
   `capability-matrix.json`, `compatibility-approvals.json`) still owed
   for the two-platform close-out.
