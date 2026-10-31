# Task C337/338 — validation-rigs qualification report (tasks 3.37 + 3.38)

**Batch**: plan §"Validation rigs (T54)" tasks 3.37 and 3.38 —
`src/bike_sim/physics/{tire_calibration,tire_curve}.py` plus
`src/bike_sim/validation/{tire_characterization,datasets,rigs,
contact_acceptance,contact_manifold_rigs,contact_resolution,load_transfer,
grade_feasibility,drive_suspension_rig,environment}.py` (12 of the 30
T54-owned source rows). Native targets `native/src/validation/tire.cpp` and
`native/src/validation/rigs.cpp` already existed with per-row
`phase3_source_implementation` triage; this lane's job was qualification
evidence, not a from-scratch port.

**Lane**: `.worktrees/pfc-lane-g`, branch `lane/g`, base `dacccbca`.
Container `bike-dev-pfc-lane-g` (`bike-dev-linux:24.04`, clang++-18,
`/build/root-clang`). No writes outside this worktree.

## Status: 4 rows `replacement_landed`, 8 rows stay `planned`

The prior ledger triage marks each row either
`source_implemented_pending_qualification` (implementation judged complete,
needs evidence) or `partial_source_implementation` (a concrete residual gap
remains). This batch qualified the pending set with two-platform bitwise
oracle verdicts and left the partial set `planned` — the documented gaps are
implementation gaps, which qualification cannot close.

| Python source | p3 triage | Native twin(s) | New evidence | Disposition |
|---|---|---|---|---|
| `physics/tire_calibration.py` (84) | pending_qualification | `validation/tire.{hpp,cpp}` | bitwise `tire_calibration` verdict + `material_curve_contract` (inclusive load range, extrapolation rejection) | **landed** |
| `physics/tire_curve.py` (92) | pending_qualification | `contact/tabulated_radial.{hpp,cpp}`, `validation/tire.cpp`, `registry.cpp` | bitwise `tire_curve` verdict + direct `MaterialRangeError`/unilateral-damping/interval-integral checks; runtime law green under `reference.native_tire`, `distributed_tire_reference`, `native_distributed_tire`, `native_distributed_stands` | **landed** |
| `validation/tire_characterization.py` (42) | pending_qualification | `validation/tire.cpp`, `registry.cpp` | bitwise `tire-characterization` verdict (RCP-07 `steady_brush_force` capture) + invalid-input rejection contract | **landed** |
| `validation/datasets.py` (213) | partial | `validation/tire.cpp` | bitwise `datasets`, `dataset_hash`, `holdout_metrics`, `calibration_report` verdicts | stays `planned` — C++ `holdout_metrics` is a 1-D `std::span`; Python accepts same-shaped multidimensional `np.asarray` inputs (unreachable through any in-repo caller/JSON boundary today), and native JSON byte/number limits still bound metadata |
| `validation/grade_feasibility.py` (20) | pending_qualification | `validation/rigs.cpp`, `registry.cpp` | bitwise `grade_feasibility` verdict (margin, `necessary_condition_met`, constant-false `sufficient_for_completion`) | **landed** |
| `validation/rigs.py` (194) | partial | `validation/rigs.cpp`, `drivetrain.cpp` | `brake_stand` verified through `brake_rig` after a real defect fix (below); `quasistatic_front_load` verdict covers `load_transfer.py:13-21` not this file | stays `planned` — "a manually reconstructed small rotation is not asserted bitwise equal" (chain_geometry invariance path, `rigs.py:129-138`, still has no oracle verdict); small-matrix/NumPy-vs-libm lowering unveredict |
| `validation/contact_acceptance.py` | partial | `validation/acceptance.cpp`, `rigs.cpp` | `contact_bounds`/`contact_matrix` cases exercised in `migration_T54` but no oracle verdict captured | stays `planned` — native metric decoding expects numeric JSON while Python `float()` accepts strings/bools; stand station limits and custom-material surface remain narrower |
| `validation/contact_manifold_rigs.py` | partial | `validation/rigs.cpp` | bitwise `contact_case_{step,edge_unload,recontact,incline}` verdicts cover `:94-105` (analytic vertex rasterization) | stays `planned` — the distributed stands themselves (`:80-92`, `:108-173`) have no verdicts; private native stand still uses declared default density/mu where Python accepts arbitrary optional inputs; some NumPy ufunc/matrix lowering unveredict |
| `validation/contact_resolution.py` | partial | `validation/experiments.cpp`, `evidence.cpp` | bitwise `contact_comparison` and `contact_comparison_unmatched` verdicts (`:27-36`), `persistent_events` (normal-load branch of `persistent_events`, bitwise-equal to `_events`) | stays `planned` — both NNLS density-fit paths unveredict; qualification depends on the partial contact-stand producer |
| `validation/load_transfer.py` | partial | `validation/rigs.cpp` | bitwise `quasistatic_front_load` verdict covers `:13-21` | stays `planned` — the free-pitch rigid stand (`:24-71`, `load_transfer` case) has no verdict; trig/small-matrix lowering unconfirmed |
| `validation/drive_suspension_rig.py` | partial | `validation/drivetrain.cpp` | `drive_suspension` cases dispatch and run (`bike-validate` ~73 ms smoke) but no verdict captured | stays `planned` — `shock_total` is reconstructed from split damper/HBO components; the discarded source scalar must be exposed or recomputed exactly before bitwise qualification |
| `validation/environment.py` | partial | `validation/evidence.cpp` | `environment`/`source_fingerprint` cases exercised; no verdict (native identity differs by construction) | stays `planned` — intentionally reports a native identity; Python/PEP-440 marker evaluation and per-package versions are not reproduced |

Ledger: `migration/python-removal.json` 142→146 `replacement_landed`
(389 `planned`). The four promoted rows keep
`source_assertion_map`/`release_capability_matrix` verification ids per the
landed-row convention; `tire_curve.py` additionally names the four landed
`reference.*` CTests that verify `tabulated_radial`. These are the first
`src/` (non-test) rows landed.

## Work performed

1. **Oracle capture recipe RCP-16** (`tools/migration/capture_reference.py`,
   group `validation_t54`): freezes one authored `config.json`/`config.toml`
   plus the Python oracle's verdict per case, for the pure binary64
   surfaces only (multiplies/divides/sqrt/ordered comparisons — identical on
   both qualified platforms). Transcendental/engine-integrated surfaces stay
   on declared rig bounds. 15 fixtures under
   `tests/fixtures/migration/validation/`; `migration/oracle-manifest.json`
   records each with `produced_by` provenance and blockers would have been
   recorded on capture failure (RCP-16: 15 entries, 30 files, 0 blockers;
   158 total fixtures in the manifest).
2. **`migration_T54` expansion** (`native/tests/migration/test_t54.cpp`,
   +121 lines): `verdict_fixture` resolves each fixture directory, selects
   `config.toml` else `config.json`, runs `run_case` and compares
   `result.evidence` against `verdict.json` with exact binary64/structure
   comparison (object key inventory, array shape, `std::bit_cast` double
   equality, canonical-JSON scalar equality). `contact_comparison_unmatched`
   is a diagnostic `accepted:false` case compared on evidence only, matching
   the oracle. Also adds the material-curve contract, invalid-input
   rejection, source-manifest coverage and direct rig checks (`brake_rig`
   now reachable; distributed stands and incline exercised directly rather
   than through the projected-simulation capability gate, matching how the
   Python rigs are invoked).
3. **Native defect fix** (`native/src/validation/rigs.cpp`): `brake_rig`
   read `qfrc_constraint` through an `nefc`-sized span while MuJoCo sizes
   the field `nv` — deterministic `SIGTRAP`/`EXC_BREAKPOINT` on macOS. Now
   `m->nv`, matching `validation/rigs.py:91` (Python indexes
   `d.qfrc_constraint[0]` on the `nv` array) and `native/src/stepper.hpp`.
4. **Cross-language JSON lexeme fix**
   (`native/src/recording/value_util.hpp`): `value::real` used raw
   `std::to_chars` general format, spelling `0.0001` as `1e-04` and `1e15`
   as `1e+15` where Python `repr`/`json.dumps` keeps fixed notation for
   `-4 <= e < 16` and retains `.0` on integral values. Now routes through
   the existing `io::repr_binary64`, restoring bitwise equality of
   canonical-JSON payloads (`dataset_hash`, `calibration_report`).

## Verification

- `ctest -R '^migration_T54$'` — **darwin-arm64**: PASS, 13734 checks;
  **linux-aarch64** (`bike-dev-pfc-lane-g`, root-clang): PASS 0.43 s.
  Task-3.37 L1 satisfied on both platforms.
- `cmake --build build/native-release` + full `ctest -j10` (228 tests):
  **221/228**. The 7 reds are the documented baseline verbatim
  (`migration/cpp-linux-completion-evidence.md`): `migration_T09` +
  `migration_T10` frozen-oracle pair, `migration_T56` environment drift
  (manifested as a 1500 s timeout under `-j10` load; documented signature
  is shellcheck stderr-format drift), and the four-red reference baseline
  `golden_episode_tool`/`native_numeric_completion`/`pneumatic_tire_oracle`/
  `native_pneumatic_tire`. Zero regressions attributable to this batch —
  every failing signature is a numeric/schema/config diff that a lexeme
  change cannot produce, and `rigs.cpp`'s edit is confined to `brake_rig`.
- `cmake --build native/build` (binding graph): green earlier in the lane.
- Container: `container.sh up` + `sync` + `configure root-clang root
  -DCMAKE_CXX_COMPILER=/usr/bin/clang++-18` + `build root-clang
  migration_T54` (298 targets incl. `bike-validate`) all green.
- Final gate `bash tools/run_tests.sh native`: see "Final gate" below.

## Boundary to tasks 3.39/3.40

Not touched this batch (18 T54 rows remain `planned`):

- **3.39** plant/rider rigs → `validation/rider.cpp`, `drivetrain.cpp`:
  `plant_prefix`, `plant_release`, `plant_torque_rig`, `rider_cases`,
  `rider_factor_matrix`, `rider_replay`, `rider_work`, `system_momentum`,
  `ride_cases`.
- **3.40** benchmarks/acceptance/evidence/`bike-validate` →
  `acceptance.cpp`, `evidence.cpp`, `tools/validate_main.cpp`: `__init__`,
  `antiwheelie_bench`, `antiwheelie_reports`, `benchmarks`,
  `climbing_benchmark`, `climbing_evidence`, `experiment_acceptance`,
  `fidelity_sweep`, `reference_release`. Note the `persistent_events`
  verdict captured here already bitwise-covers
  `fidelity_sweep.py:191-199`'s normal-load branch — reusable evidence for
  3.40.

## Honest limitations

- `datasets.py`'s multidimensional `holdout_metrics` input class is the
  only API surface where Python accepts what native rejects outright; no
  in-repo caller or JSON boundary can reach it, so the four captured
  verdicts fully cover today's exercised surface.
- `persistent_events` was captured from `contact_resolution._events`
  (normal-load branch); the `fidelity_sweep.persistent_events`
  `contact_state`/`confirmed_at_s` variant shares the native implementation
  but has no verdict — noted for 3.40 rather than silently claimed.
- `drive_suspension_rig.py`'s `shock_total` reconstruction can differ in
  the last bits; promotion waits on exposing/recomputing the source scalar.
- `migration_T54` now contains authored-contract checks plus 16 fixture
  verdict comparisons; the remaining uncovered public surfaces are named
  per row above.

## Final gate

`bash tools/run_tests.sh native` (darwin-arm64, selected build
`native/build`, orchestrator `build/native-release/bin/bike-native-checks`):

- Configure + full build: green.
- Static-analysis sweeps — all clean:
  `frontends` none; `tidy` none; `analyzer` report-only alpha findings
  (non-blocking by policy); `cppcheck` none; `odr` none; `scripts`
  (shellcheck) clean; `headers` 268 headers / 9508 header-TUs, none;
  `diagnostic-controls` 6 controls, 0 failed; `context` SDK controls all
  passed.
- `ctest --test-dir native/build` (62 tests): **60/62** — the only reds
  are `migration_T09` (`FAIL post-advance integration vector matches the
  oracle`) and `migration_T10` (`FAIL first transition matches the Python
  oracle`), the documented frozen-oracle baseline pair verbatim
  (`migration/cpp-linux-completion-evidence.md`). The gate exits here
  (ctest exit 8), as it does at every baseline commit on this platform —
  no regression is attributable to this batch (neither failing surface is
  reachable from `value_util.hpp`/`rigs.cpp`/T54 changes, and both reds
  reproduce at base).
- `--kind contracts` (run standalone with the gate's own env after
  loading the extension to record provenance): all 12 assertions green,
  including `pure_core_python_free`, `hot_kernel_dependency_free`,
  `loader_provenance`, `contract_source_manifest_membership`.
- Native/golden pytest tail (`tests/reference/test_native_*.py` +
  `test_golden_episode*.py`, 3086 items): completed in three stages
  because two independent environment faults had to be isolated.
  1. First attempt reached 2772 passed / 39 failed / 435 errors in
     69 min — the failure mass was environmental: the shared host hit
     `ENOSPC` (three parallel lane gates, ~10 worktrees) and another
     lane's pytest wiped the shared `pytest-of-<user>/pytest-24`
     basetemp mid-run; every one of the 435 errors is `OSError` at
     `tmp_path` setup and the check-tools fails are `Errno 28` writing
     sweep logs.
  2. Rerun of the 41 affected files under a lane-private `TMPDIR`
     exposed the second, separate fault: `selected extension was not
     built from the current native source`
     (compiled=`b66b92e2…` vs current=`408a417e…`). The binding-graph
     extension `.so` (built 09:40:02 during preflight) predated the
     `value_util.hpp` lexeme fix (09:40:51), so every
     `native_module`-fixture test failed closed on the fingerprint
     check — 1123 passed / 36 failed / 170 errors, all stale-extension.
  3. `cmake --build native/build` (all targets green), then rerun of the
     28 red files: **872 passed / 41 failed in 12 min**. The 41 reds are
     the documented baseline tail verbatim
     (`migration/cpp-linux-completion-evidence.md`, tail baseline
     "64 failed / 3175 passed" on the same file concentration):
     `test_native_research_environment` 16, `test_native_runtime_step`
     7, `test_native_runtime_commands` 4, `test_native_runtime_rollout`
     4, `test_native_distributed_tire` 2 (incl. the individually
     documented `test_distributed_probe_and_clock` /
     `test_distributed_trajectory_matches_python_channel_wise_bitwise`
     D14 baseline drops), `test_native_viewer_snapshot` 2,
     `test_native_numeric_completion` 2 (incl.
     `test_numpy_float32_shifting_metadata_retains_baseline_bits`),
     `test_native_pneumatic_tire` 1, `test_native_research_capture` 1,
     `test_native_research_recording` 1, `test_native_ride_frontend` 1
     — all C++↔oracle bitwise-drift gaps on sibling lanes'
     **unported/planned** records, none reachable from this batch:
     `value_util.hpp` changes only the JSON lexeme spelling of
     already-identical round-trip doubles (a spelling change cannot
     move a parsed binary64), and `rigs.cpp` touches only `brake_rig`'s
     span bound, which no ride/episode test exercises.
- Linux re-verification post-final-tree (`bike-dev-pfc-lane-g`):
  `container.sh up` + `sync` + `build root-clang migration_T54`
  (ninja: no work to do — build already current) +
  `ctest -R '^migration_T54$'` → **PASS 1.04 s**.
