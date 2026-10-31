# Ledger reconciliation — 2026-10-31 (Phase 4, task 4.01)

Lane: `pfc-lane-a` (`lane/a3`) at integration tip `907687f1`. Scope per the
4.01 contract: completeness sweeps over `migration/test-port-map.json` and
`migration/python-removal.json`, mechanically safe fixes only, ambiguous
cases flagged as Phase-4 blockers. Darwin-arm64 only — the Linux lane was
explicitly deferred (`tools/linux/container.sh` carries a committed
plan-change guard; no Docker run was attempted). All Python work used
`uv run`.

Companion inputs: `migration/tooling-replacement-map-2026-10-31.md`,
`migration/honest-red-inventory-2026-10-31.md`,
`migration/full-gate-2026-10-31.md`,
`docs/superpowers/audits/2026-10-28-migration-status.md`.

## 1. Counts and canonical form

| Ledger | Before | After | Notes |
|---|---|---|---|
| `test-port-map.json` records | 2394 | 2394 | header `static_test_functions`/`top_level_test_functions`/`class_test_methods` = 2394/2378/16, all consistent with the data |
| — `port` | 1533 | 1539 | +7 re-triaged from `golden`, −1 re-triaged to `golden` |
| — `golden` | 418 | 412 | symmetric inverse |
| — `drop` | 436 | 436 | unchanged |
| — `planned` | 7 | 7 | the documented baseline exceptions (§7) |
| `python-removal.json` rows | 535 | 547 | 12 missing tracked `.py` rows added (§6) |

Canonical form verified on both files: `json.dumps(indent=2,
ensure_ascii=True)` + exactly one trailing LF; authored key order preserved.
`git ls-files '*.py'` coverage: **547/547** (was 535/547 — the 12 missing
rows in §6). `checkout_reconciliation` is a dated T00 block
(`source_path_count: 146` was the T00 snapshot scope; 180 distinct
`source_path`s exist today — historical evidence, not a current-count
defect; its `project/` note remains accurate).

## 2. Golden completeness sweep

Sweep rule: for every `golden` record, every `collected_node_ids` node, and
every platform in `golden_platforms`, the pair
`tests/fixtures/golden/<platform>/<key(file)>/<key(test)>.{json,bin}` must
exist (`_key` per `tests/reference/_golden.py:30`).

| Platform | Golden records | Missing pairs after fix | Status |
|---|---|---|---|
| `darwin-arm64` (authoritative) | 412 | 6 node instances, **all documented conventions** | clean |
| `linux-aarch64` (owed/deferred) | 412 | 8 node instances: the same 6 conventions + **2 real debts** | 2 capture debts outstanding |

The 6 convention instances (documented in-record, no fixture owed):

* `test_control_clock.py::test_runtime_holds_ctrl_but_observes_solved_effort_each_step`
  — replays the committed bundle `…/control_clock/env_request/` (verified
  present), not a per-node pair.
* `test_native_state_restore.py::test_forward_on_stored_state_is_bitwise` and
  `…::test_set_state_size_mismatch_raises_and_keeps_state` — replay the
  committed `…/episode/pinned_savage_40_f2/` bundle (verified present).
* `test_native_rider_contact_math.py::test_planar_model_validation_oracle[…]`
  ×3 params — the source calls `_golden.record` only inside the `except
  ValueError` branch; non-rejecting params emit no pair by design.

**Real debt (`test_oracle_parity_with_xfrc_writer[fast]` + `[detailed]`):**
the committed tree carries **neither** platform's pairs. darwin-arm64 pairs
*exist in this worktree but are untracked* — created 2026-10-31 22:41 by an
out-of-band capture run (`git status` shows
`?? tests/fixtures/golden/darwin-arm64/tests_reference_test_pneumatic_tire_forces.py/`;
the `.json` sidecars are well-formed bitwise captures with correct node
ids, 64/68 entries). Provenance of that capture (a whole-file green run per
`_golden.py`'s commit rule) is unverified, so they were **not** committed
here — a capture/qualification lane must verify and commit them (or
re-capture). `linux-aarch64/` has no
`tests_reference_test_pneumatic_tire_forces.py/` directory at all → the
linux pairs are owed outright. Phase-4 blocker §8.

Before fixes the sweep reported 10 missing nodes per platform. Disposition
re-triage (§4) resolved 7 as misclassified `golden` records and the
annotations above cover the rest.

## 3. Native test ID / native_source resolution

For every `port`/`golden`/`planned` record (1958 `native_test_id` values):

| Resolution class | Count |
|---|---|
| Exact registered CTest name (`ctest -N`, `build/native-release`, 231 tests) | 151 |
| `suite::fn` with fn = python source symbol, or fn/case stem verified in `native_source` (incl. comma-joined case lists and `run_case(${_var})` generator names: `b29_joined_${_flag}`, `b29_hfilter_${_root_name}`, `b29_engine_${_call_name}`, `b29_override_${_t}`, `t56_ctx_root_missing_${_field}`, `t56_ctx_schema2_${_capability}_*`, `cross_compiling_configure_is_refused_${_platform}`, `record_only_*`, `live_*`) | 1806 |
| Conditional: `sanitizer_negative_control` — registered by `native/cmake/Sanitizers.cmake:16`, exists only in sanitizer-instrumented build graphs | 1 |
| **Unresolvable** | **0** |

`native_source` existence: **0 missing across all 2394 records**.
`ported::tests::*` placeholder IDs: **0** (none exist in the map).
The `reference.realtime_gate::test_realtime_factor_for_full_twenty_seconds`
id resolves — the `reference.realtime_gate` binary registers as two
case-suffixed tests (`reference.realtime_gate.rough_uphill_{savage,extreme}`)
and the named function exists at `test_realtime_gate.cpp:51`.

## 4. Port/golden record completeness

* `collected_node_ids`: **483 records backfilled** (437
  port/golden/planned + 46 drop) from a real
  `uv run pytest --collect-only` enumeration — 1877 nodes across the 33
  affected files, zero unmatched records, zero overlapping node claims.
  `assertions_capture` rewritten for 480 T01-era placeholders
  (date-stamped, collection-only claims; no oracle execution claimed) and
  appended for 3 already-measured captures. **0 empty `collected_node_ids`
  remain.**
* `comparison_class`: all 2394 records now carry one of the seven enum
  members. Fixed 151 non-enum values:
  - `native contract port` → `contract` (114)
  - `oracle-derived comparison` → `bitwise-oracle` (25; fixtures are
    `kind=bitwise`) and → `tolerance-oracle` (2: `matched_workload_…`
    asserts rtol=atol=1e-12, `tire_channel_parity…` uses
    `assert_tree_close`)
  - `source-specific; assigned by T01 before implementation` →
    `tolerance-oracle` (10 research-wave records; fixtures mix
    `kind=close` f8 leaves with bitwise canonical-JSON leaves)
* Disposition re-triage (C++-consumption-verified):
  - **golden → port ×7**: `test_native_platform_blas.py` rejects ×4 +
    `test_native_numeric_portability.py` rejects ×3 — the C++ asserts the
    rejection contract directly inside `rejection_suites()`
    (`bt::invalid`, no `oracle::Node`), and no per-node fixture exists or
    is owed. `comparison_class` → `contract`, `golden_platforms` removed.
  - **port → golden ×1**:
    `test_native_numeric_completion.py::test_mutated_drive_config_rejected_before_model_writes`
    — the C++ opens a per-parameter `oracle::Node` and replays `wrap_prm`
    bitwise; all 4 parametrized pairs exist on both platforms. Class →
    `bitwise-oracle`, `golden_platforms` → both platforms.
  - Annotated (no flip):
    `test_native_clock_contracts.py::TestTireSnapshotIntervalPath.test_overflow_is_overflowerror_and_atomic`
    stays `port` — fixture pairs exist but the C++ asserts the
    reject-preserves-state contract natively; the captured pairs are
    orphaned provenance (flagged §8).
* `golden_platforms` empty-list keys removed from 129 non-golden records
  (schema cleanup; non-golden records do not carry the field).
* `drop` evidence: **436/436 nonempty**; sample-audited one record per
  source file (55 files) plus every 3.36t2a/3.36t2b last-wave drop — all
  name the replacement surface or a specific no-loss reason. One wording
  nit (no fix): `test_publication_failure_retry_does_not_advance_twice`
  leads with "remains planned" inside a `drop` record's evidence.

## 5. Stale-path reconciliation (`python-removal.json`)

**33 distinct stale path tokens across 32 rows → all resolved except one
intentional OD-1 deferral.**

| Stale token(s) | Rows | Replacement applied |
|---|---|---|
| `.github/workflows/native.yml` | 500, 510, 511 (`ci_portability`, `portability_baseline`, `portability_qualification`) | `.github/workflows/portability.yml` |
| `native/src/tools/{test_runner,coverage}_main.cpp`, `native/tests/support/provenance.cpp` | 0, 292–295, 451 (conftest×2, `native_loader`, `native_safety_helpers`, `native_test_reporter`, `native_coverage`) | `native/cmake/Checks.cmake` + `native/tests/support/migration_test.{hpp,cpp}`; `native/cmake/{RunCoverage,CoverageTools}.cmake`; `native/cmake/Provenance.cmake` |
| `native/cmake/{CompilerPolicy,Dependencies}.cmake` | 41 (`src/bike_sim/native/__init__.py`) | `BikeCompilerPolicy.cmake`, `BikeDependencies.cmake` |
| invented reference-test names (`test_{divergence_horizon,episode_compare,golden_episode,golden_episode_v2,numba_kernels,rider_allocation,rider_dynamics,rider_information_boundary}.cpp`, `property_cases.hpp`) | 286–288, 290 (`_bits`, `_encode_oracle`, `_legacy_sensor_oracle`, `_physical_hud_oracle`) | `test_golden_episode_regressions.cpp`, `test_native_kernel_contracts.cpp`, `test_native_rider_{allocation,dynamics,information_boundary}.cpp`, `migration_test.hpp` + `golden.hpp`; `test_divergence_horizon.cpp` dropped (source records are `drop` — no twin) |
| `process_fixture.cpp`, `test_native_{check_tools,contract_checks,loader}.cpp` | 289, 291 (`_native_runtime_support`, `_recording_support`) | `migration_test.cpp`, `migration_T56_check.cmake`, `test_native_loader_contracts.cmake` |
| `packaging/{manifest.schema.json,install.sh}`, `native/src/tools/{package_verify,install}_main.cpp` | 442, 447 (`check_offline_sources`, `install_offline`) | `StageSource.cmake`, `install_{full,slim}.sh`, `package_smoke.cmake` + `python_audit_main.cpp`, `tools/linux/extract_deps.sh` |
| T55 shared invented list (`{golden_episode,episode_compare,step_profile,diag,benchmark}_main.cpp`, `state_format.{hpp,cpp}`) | 443, 453, 458 (`diag_run`, `bench.py`, `step_profile`) | `cli/recording_golden.cpp` + `golden/episode.cpp`, `measure_realtime_main.cpp` + `proto_native_bench/bench.cpp`, `cli/{ride,recording,export}.cpp`, `test_native_benchmark.cpp`, `proto_native_bench/state_format.hpp`; `divergence_horizon_main.cpp` + `finalize_bench_main.cpp` + `state_format.cpp` dropped (none exist) |
| `native/src/tools/{divergence_horizon,finalize_bench}_main.cpp` etc. on gap rows | 444, 454, 455, 456 | see delete_with_python rulings below |

**Explicit `delete_with_python` rulings applied** (map gaps 1–2, each with
`inventory_note` justification): `tools/divergence_horizon.py`,
`tools/recording_finalize_bench.py` (no native replacement — replacement
lists emptied), `tools/proto_native_bench/dump_model.py` and
`tools/proto_native_bench/state_format.py` (writer gap closed by frozen
committed `artifacts/`; replacement = `state_format.hpp` reader +
`artifacts/`).

**Intentionally unresolved:** `docs/reference/README-provenance.md` on 22
`tools/research/**` rows — the OD-1 deferred 4.05 deliverable; flagged, not
invented.

## 6. Missing ledger rows

The map named 3 missing capture-tool rows; the completeness sweep found 9
more. **All 12 added** (`delete_with_python`, T63 owner unless noted):

* `tools/migration/_f5_spindle_capture.py`, `_t25_capture.py`,
  `_t52_capture.py` — the map's named three.
* `tools/migration/_ft2_capture.py` — committed 2026-10-31, same class.
* `tests/fixtures/golden/episode/pinned_savage_40_f2/freeze.py` — mirrors
  the pinned_savage_40 freeze row (T53 owner, same gates).
* `.agents/skills/mujoco/scripts/*.py` ×7 — tracked `.py` the audit
  enumerates; `delete_with_python` rows added, policy question flagged §8.

## 7. The seven `planned` baseline exceptions (2026-10-31 user ruling)

| Record | Baseline reason | Un-exception condition |
|---|---|---|
| `test_coupled_rider_task.py::test_tissue_damping_does_not_eat_the_muscle_budget` | `reference.coupled_rider_task` red on **both** engines (`passive >= 0.3*positive` fails on the 1.5 s episode; darwin red at HEAD, 1226 s). Physics/contract premise, not a port defect | passive-damping share contract resolved (physics decision + re-scope), then re-measure both platforms |
| `test_native_pneumatic_tire.py::test_pneumatic_golden_episode_event_rich` | C++ twin exposes a real `seal_bootstrap`-vs-`capture_bootstrap` construction-path parity gap (~8e-7 front channel; committed fixture ~1.9e-5 stale vs oracle) | parity gap closed and fixture re-captured both platforms |
| `test_native_pneumatic_tire.py::test_pneumatic_probe_and_clock` | same construction-path parity family (~1e-12 interval-1) through the un-parked twin | same as above |
| `test_realistic_pedelec_acceptance.py::test_flat_reaches_25_kmh_within_10_s` | `cadence_in_band` red on **both** engines post-FT-3 (oracle 0.7447, native darwin 0.7536, native linux 0.7464 < 0.8 bar) — bar premise predates shipped behavior | premise re-scoped or physics fixed so both engines clear the bar |
| `…::test_motor_follows_the_rider_through_a_shift_without_an_extra_cut` | ride produces 4 upshifts < the test's ≥5 premise on both engines — premise conflicts with shipped multi-cog policy | premise re-scoped to multi-cog reality |
| `…::test_fifteen_percent_climb_holds_12_kmh` | oracle node green post-FT-3 but native still shows 364 late `rider_strength.*` violations (parity gap, reduced from 538) | native late-violation family driven to zero |
| `…::test_savage_is_ridden_to_the_end` | shared 25%-plateau stall: x_max 58.306 m oracle / 58.319 m native < 120 m bound — oracle-side physics shortfall, engines agree bitwise | episode physics reaches the 120 m bound or the bound is re-scoped |

## 8. Phase-4 blockers for tasks 4.02–4.08

1. **`test_oracle_parity_with_xfrc_writer` fixture debt, both platforms** —
   linux-aarch64 pairs are owed outright (deferred container lane); the
   darwin-arm64 pairs sitting untracked in this worktree need a verified
   capture-provenance review and commit (or re-capture). Blocks the
   two-platform claim on this record and leaves `??` noise in the tree.
2. **Native red at HEAD**: `reference.native_numeric_completion` fails on
   darwin-arm64 at `907687f1` —
   `config.rider_controller.leg_track_damping_nms_rad: missing required key`
   inside `test_tire_channel_parity_across_backends` (fresh rebuild
   confirms; member of the documented tire-parity red family, but the
   native-side signature is new vs the ledger's "CTest green at capture"
   note). Port-lane fix needed.
3. **`docs/reference/README-provenance.md`** — 4.05 deliverable; 22
   `tools/research/**` rows cite it (OD-1). Unresolved by design.
4. **`sanitizer_negative_control` registration** — exists only in
   sanitizer build graphs; the `test_instrumented_use_after_free_reports_asan`
   record's verification needs an asan-graph run to be checkable.
5. **Seven planned records** — §7 un-exception conditions are all
   physics/contract re-scope or parity-gap work, not ledger work.
6. **Unverifiable verification-artifact IDs** — `source_assertion_map` and
   `release_capability_matrix` (~350 rows each) name no committed file;
   either map them to real artifacts or define the ID convention.
7. **`.agents/skills/mujoco/scripts/*.py`** — rows now exist with
   `delete_with_python`; policy must decide whether agent-skill scripts
   are deleted at T63 or added to `no-python-policy.json` exempt globs.
8. **Orphaned fixture pairs** —
   `tests_reference_test_native_clock_contracts.py/TestTireSnapshotIntervalPath__test_overflow_is_overflowerror_and_atomic.{json,bin}`
   on both platforms are captured but consumed by no port. Keep as
   provenance or delete — hygiene decision for the golden-completeness
   task (a "fixture without consumer" rule does not exist yet).
9. **`project/` tree anomaly** — `project/tests/reference/test_constraint_mode_search.py`
   + `project/docs/…` are tracked outside the normal layout (committed in
   `937dffc1`); the 6 records are self-consistent and the C++ port exists,
   but the file sits outside pytest's normal collection root. Relocate or
   ratify.
10. **`verification_ids` naming** — `migration_T55`/`T57`/`T59` resolve to
    task cards (`migration/cards/*.md`), not runnable checks; if the
    field's contract is "named check", these need re-pointing (e.g. the
    T55-era rows now name `reference.native_benchmark`, `package_smoke`,
    etc. in `replacement_paths`).
11. **`bike-python-audit --mode source` census: 619 remaining edges** —
    interpreter-invocation edges across `tools/*.sh`, `native/**/*.cmake`,
    `CMakeLists.txt`, workflows (T62 enforcement surface; the new ledger
    rows are correctly enumerated as "still present" findings).
12. **Pre-existing gate reds** (not caused by this reconciliation) —
    `check_tidy`: 1 unsuppressed darwin finding (`summary.cpp:51`
    `OPENBLAS_NUM_THREADS`), 448 on linux; the documented baseline-red
    pytest/CTest families enumerated in `full-gate-2026-10-31.md`
    (`closure_signed_j` cluster, development-docs, `.python-version`
    baseline mismatch).

## 9. Verification performed (darwin-arm64 only)

* `uv run pytest --collect-only` — 33 files, 1877 nodes, rc=0; every one of
  the 483 empty-`collected_node_ids` records matched ≥1 node, none
  overlapped.
* `ctest --test-dir build/native-release -N` — 231 tests enumerated;
  every `native_test_id` resolved per §3.
* `ctest -R` spot checks on the touched suites:
  `reference.{pneumatic_tire_oracle,native_platform_blas,
  native_numeric_portability,control_clock,native_state_restore,
  native_clock_contracts,native_rider_contact_math}` — **6/7 green**;
  `reference.native_numeric_completion` red at HEAD (§8.2, pre-existing,
  rebuilt and re-confirmed — not caused by ledger edits). Caveat:
  `reference.pneumatic_tire_oracle`'s green depends on the *untracked*
  xfrc fixture pairs (§2) — on a clean checkout it would fail.
* `bike-python-audit --mode source` on the real tree — parses the edited
  `python-removal.json` cleanly; all 12 new rows enumerated (547 files
  "still present" census + 619 invocation edges, T62 surface).
* Canonical-form byte check on both ledgers after every write.
* No gate consumes `test-port-map.json` today (`golden_completeness` is a
  future task); `migration_T56` exercises the audit binary against planted
  fixtures only, so the JSON edits are not gate-visible in this tip.
  `tools/run_tests.sh` profiles cover `.py`/native changes — this change is
  ledger data + docs; the covering verification is the sweep + CTest +
  audit-binary evidence above (decision recorded per the profile rule).
