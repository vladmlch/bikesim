# Task C326 — test-port 3.26 report (runtime config, accounting, rollout, performance contract)

**Batch**: 3.26 — `tests/reference/test_native_runtime_{config,accounting,
rollout,performance_contract}.py` (44 records / 73 collected nodes;
the parametrized config capability matrix expands to 29 node ids and
the accounting parametrizations to 28).
CTest ids `reference.native_runtime_{config,accounting,rollout,
performance_contract}`; labels `t53 runtime`; port-map rows carry
`owner_task: T53`.

**Lane**: `.worktrees/pfc-lane-d`, branch `lane/d`, base `cf17978f`
(post-3.25 merge-back). No merge, no push.

## Status: landed — 38 `port`, 4 `planned` (pre-existing oracle reds), 2 `drop`

38 of 44 records ported to four new CTest executables; every ported
check compares native state against native state, native contract text,
or deterministic fake-driver measurement plumbing — no golden fixtures
were needed or created. The four live-oracle parity nodes that remain
red in the source suite were deliberately **not** reproduced (porting
them past as green would falsify a live gate); they stay `planned` with
per-node evidence. Two records are `drop` (`python-only` — the
"absent resolved section" construction is unrepresentable natively).

## Triage

| Source file | records | collected nodes | port | planned (red) | drop |
|---|---|---|---|---|---|
| `test_native_runtime_config.py` | 17 | 29 | 15 | 0 | 2 |
| `test_native_runtime_accounting.py` | 11 | 28 | 11 | 0 | 0 |
| `test_native_runtime_rollout.py` | 6 | 6 | 2 | 4 | 0 |
| `test_native_runtime_performance_contract.py` | 10 | 10 | 10 | 0 | 0 |
| **total** | **44** | **73** | **38** | **4** | **2** |

## What the ports reproduce

- **`reference.native_runtime_config`** (15 checks): the capability
  matrix against `cfg::load_physics` + `validate_native_supported()` /
  `validate_physical_capabilities()` on the welded fixture — field-path
  (not filename) rejection, structural leaves, distributed/pneumatic
  backend presence, `configured` pneumatic surface requirement, drive
  mode, transmission model, legacy physics mode, unknown tire backend,
  rider variants, tabulated tire material, scalar tuning, disabled
  features — plus `NativeRideRuntime` type exposure/construction and
  the `engine::verify_identity` dependency-mismatch check. The native
  capability matrix is intentionally broader than the oracle gate
  (details under Deviations); each record asserts the native verdict.
- **`reference.native_runtime_accounting`** (11 checks):
  `runtime::step_work` opposing-work folding and the zero-source fixed
  absolute roundoff budget in `constraint_work_ok`; non-finite /
  non-positive input rejection; `PeriodBuffer::clear` interval-marker
  survival; `WorkHistory` multi-period intervals, working-road
  (`source_geom == "terrain"`) airtime, malformed contact evidence,
  duplicate-interval rejection; `observe_effort` captured-velocity /
  strength folding and held controller state; welded-ride strict
  rejection publishing the complete period before throwing; and the
  single-warning diagnostic retention path.
- **`reference.native_runtime_rollout`** (2 checks): the two
  native-vs-native invariants — snapshot drain and time budget not
  closing a partial period, and `recorded_intervals` independence from
  record decimation. The four oracle-parity records stay `planned`
  (below).
- **`reference.native_runtime_performance_contract`** (10 checks): the
  measurement/acceptance contract over `runtime::measurement` with
  deterministic fake `Driver`s — failed-run `measured_ok`/`null` headless
  semantics, never-comparable failed runs, the exact `1.0` native
  headless RTF boundary, Release-only eligibility, the pending visual
  1x gate (`0.95` mean minimum emitted as pending), distributed-case
  gating on the same constant, artifact-identity binding in
  `measure_run`, and build-type tracking of the threshold gate.

## Individual dispositions — the four known pre-existing red nodes

All four assert bitwise equality of `NativeRideRuntime` committed
rows/state against the **live Python oracle**. All fail pre-existing on
**both** platforms with the same weld-family signature:
`root.sensors.frame_specific_force_body_mps2[0]` diverges, and the
800-interval golden episode additionally trips
`rider_strength.rider_torso_hinge` strict reference at `t=0.378750000`
(RuntimeWarning captured by the source run). This is the same
native↔oracle divergence family the 3.25 step/commands ports recorded
(1-ulp propagating integration drift; see `task-C325-report.md`). A
native↔oracle runtime divergence requiring its own fix task — the nodes
remain live oracle gates in the Python suite; each stays `planned` with
NOT-PORTED evidence in `migration/test-port-map.json`.

| Node | What it gates | Disposition |
|---|---|---|
| `test_native_full_rows_match_python_for_non_aligned_tail` | committed row bytes for a non-aligned tail vs oracle | `planned` — frame_specific_force weld-family red, both platforms |
| `test_identical_non_aligned_explicit_flush_schedule_matches_python` | identical rows under an explicit non-aligned flush schedule | `planned` — same signature |
| `test_native_recorded_columns_match_scalar_recorder_without_row_roundtrip` | `recorded_columns` vs scalar recorder without row round-trip | `planned` — same signature |
| `test_one_second_800_interval_golden_episode_all_channels` | 1 s / 800-interval all-channel golden episode | `planned` — same signature plus `rider_torso_hinge` strict trigger at t=0.37875 |

## Drops

| Node | Reason |
|---|---|
| `test_native_capability_rejects_distributed_backend_without_section` | `python-only`: the source writes `None` into the Python Optional resolved-section field; `TireBackendConfig::distributed` is a non-optional C++ member always resolved through `post_init` — the "no resolved section" state has no representation. |
| `test_native_capability_rejects_pneumatic_backend_without_section` | `python-only`: identical construction — `TireBackendConfig::pneumatic` is non-optional. |

## Two-platform qualification matrix

| Surface | darwin-arm64 | linux-aarch64 |
|---|---|---|
| `reference.native_runtime_config` | PASS (0.74 s, `build/native-release`) | PASS (0.20 s, container `bike-dev-pfc-lane-d` `/build/root-clang`) |
| `reference.native_runtime_accounting` | PASS (0.81 s) | PASS (0.26 s) |
| `reference.native_runtime_rollout` | PASS (0.36 s) | PASS (0.46 s) |
| `reference.native_runtime_performance_contract` | PASS (0.22 s) | PASS (0.09 s) |
| `ctest -R 'reference\.native_runtime_(config|accounting|rollout|performance_contract)'` | **4/4** | **4/4** |
| source suite (all 4 files) | **4 failed / 69 passed** — exactly the 4 recorded rollout reds | **4 failed / 69 passed** — identical red set and signature (`BIKE_NATIVE_BUILD_PATH=/build/native-clang`) |

Golden capture: none — every ported check is native-vs-native or a
native contract assertion; there is no oracle-derived comparison to
freeze. The welded bootstrap fixture is the pre-existing
`tests/fixtures/migration/bootstrap/welded` capture (unchanged).

## Commits

| SHA | Unit |
|---|---|
| `fc9b8fb0` | test(reference): port runtime config/accounting/rollout/perf contract suite (C3.26) — 4 CTest sources + `Phase3Ports.cmake` registration (single block, `LABELS t53 runtime`; perf target links `bike_app bike_realtime_measurement`) |
| `ab931229` | docs(migration): C3.26 runtime batch ledger — `test-port-map.json` (38 port / 4 planned-red / 2 drop, measured `collected_node_ids`, dated `assertions_capture`, per-record evidence) + `python-removal.json` (config, accounting, performance_contract → `replacement_landed`) |
| `af9496ac` | docs(migration): C3.26 evidence section + completion report (this report) |

## Deviations

1. **4 live-oracle parity nodes not reproduced** — documented above;
   they stay `planned` as live gates, not `drop`, because the gate is
   real and the divergence is unfixed.
2. **Native capability matrix intentionally broader than the oracle
   gate** — the port asserts native verdicts, not Python's: pedal welds
   are accepted (legacy-controller paths), `native_reference` is a
   recognized contact dispatch, `elastic_chain` /
   `geometric_ideal_mid_drive` transmissions are schema-accepted,
   `coast`/`crank_effort`/`ideal_speed_control` drive modes dispatch,
   attachment variants and tabulated tire material are accepted.
   Unknown/invalid values still reject by field path.
3. **Schema-coupling isolation in the config matrix**: testing
   alternate drive modes disables `seated_climb` (the welded fixture
   otherwise rejects non-articulated effort modes during schema
   validation, pre-empting the capability verdict); testing alternate
   transmission models disables automatic shifting (native schema
   requires `ideal_mid_drive` for it). Recorded per-record in the
   ledger.
4. **Accounting strict-publish port covers the contract half only**:
   publication order, dense interval ids, closure keys and
   first-failure retention under the weld-floor mutation — the
   oracle-equality half is covered by the rollout planned records.
5. **`observe_effort` port is a native-vs-native identity**: it folds
   the same channels as `SpindleController::solved_effort` on held
   captures; both controllers carry the captured
   `config.rider_controller` document — the "held controller state"
   assertion maps to document isolation, not Python object identity.
6. **Python adapter shapes not mirrored**: parametrized Python-only
   type probes (e.g. `True`/`1.5`/`'3'` targets) collapse to the typed
   native signature; the performance file's pytest fixture plumbing is
   replaced by deterministic fake `Driver` implementations against the
   same public measurement APIs.
7. **Stale extension artifacts (darwin + linux)**: the Python source
   run initially failed on a pre-imported `bike_native` artifact whose
   source fingerprint predated HEAD; rebuilt the selected artifact and
   reran — an artifact-identity gate doing its job. On Linux
   `create_native_ride` resolves via `selected_build_dir()`, so
   `BIKE_NATIVE_BUILD_PATH=/build/native-clang` is required in addition
   to `NATIVE_TEST_BUILD_PATH`.
8. **Container source-sync gaps (baseline, unmodified)**: a full
   container configure fails on unrelated omissions
   (`/src/native/tests/reference/test_crank_split.cpp`,
   `/src/native/tests/integration/test_rider_welds.cpp` not yet
   mirrored); the existing `/build/root-clang` graph was used for the
   targeted CTest scope per lane rules.

## Final gate

Lane instructions restrict broad `bash tools/run_tests.sh native` /
`full` execution on this lane; the batch gate is the targeted CTest
scope plus the source-suite status run:

- `ctest --test-dir build/native-release -R 'reference\.native_runtime_
  (config|accounting|rollout|performance_contract)'` — **4/4 PASS**
  (darwin-arm64).
- `ctest --test-dir /build/root-clang -R ...` (container
  `bike-dev-pfc-lane-d`) — **4/4 PASS** (linux-aarch64, clang).
- Source suite both platforms: **69 passed / 4 failed**, the failures
  exactly the four recorded live-oracle reds; zero unexpected reds.
- No C3.26-introduced failure exists. The documented T09/T10 frozen-
  oracle baseline pair and the other `planned`-ledger oracle reds are
  unaffected baseline state, not regressions from this batch.

## Concerns / notes for merge-back

1. The 4 `planned` records are the gate: any merge-back consumer must
   not read `planned` as "done". The weld-family divergence fix is a
   separate runtime/parity task; once it lands these nodes become
   `golden`-class ports per ADR-0003.
2. `python-removal.json` keeps `test_native_runtime_rollout.py` at
   `planned` — the file still carries live oracle gates; only
   `config`/`accounting`/`performance_contract` moved to
   `replacement_landed`.
3. The two `drop` records lose the "rejects missing resolved section"
   assertion only where the state is unrepresentable; the present-
   section acceptance and `configured`-without-surface rejection cases
   are covered by ports in the same file.
4. `test_native_runtime_config.cpp` carries the established
   `NATIVE_DIAG_PUSH` / `-Wframe-larger-than` waiver for the
   `SimulationPhysicsConfig` value type — same precedent as
   `runtime_reference_support.hpp`/`test_t09.cpp`; both platforms
   compile clean.
5. Extension-artifact identity on Linux: source-suite runs in the
   container need `BIKE_NATIVE_BUILD_PATH=/build/native-clang` (the
   build tree holding the rebuilt `bike_native` extension), not only
   `NATIVE_TEST_BUILD_PATH`.

## Fix round (2026-10-27, lane/d)

Review-verified corrections on top of `af9496ac`; no behavior change
to any ported assertion's verdict.

| Fix | Resolution |
|---|---|
| Port count 37→38 | The committed ledger always held 38 `port` records (15+11+2+10 — the triage rows were correct; only the totals/summary said 37). Corrected this report (status line, summary line, triage total) and the evidence 3.26 section (Scope dispositions, Ledger paragraph, commits-table row). Commit `ab931229`'s *subject* still reads "37 port" — immutable git text, noted in the evidence section; the Commits table here states the corrected count. |
| `expect_invalid("")` markers | `test_native_runtime_config.cpp`: `:247` `""`→`"transmission"` (schema `fail("unknown transmission model")` fires first in `post_init(PhysicalDriveConfig)`; source pins `drive\.transmission_model`), `:276` `""`→`"backend"` (schema `fail("unknown physical tire backend")`; source pins `tires\.backend`), `:189` `"surface"`→`"pneumatic"` (schema `fail("pneumatic tires require surface_mode track or a named surface")` at `TireBackendConfig::post_init` fires before the capability verdict; source pins `tires\.pneumatic\.surface` — "pneumatic" survives both the schema and capability-layer strings). All three verified by the green rerun below. The `expect_invalid("")` sites in `test_native_runtime_accounting.cpp` are faithful: the source uses bare `pytest.raises(ValueError)` with no `match=` there (its only pin, `'repeated'`, is ported). |
| Drop-record `collected_node_ids` | `test-port-map.json`: populated the measured node ids for `test_native_capability_rejects_{distributed,pneumatic}_backend_without_section` (1 collected node each, `source_path::source_symbol` — the 3.25 drop-record convention). Canonical formatting preserved (indent=2, ensure_ascii, trailing LF). |
| Commits-table reconciliation | The Commits table now lists the real `git log cf17978f..HEAD` SHAs `fc9b8fb0`/`ab931229`/`af9496ac`; the stale `(this commit)` placeholder and the evidence section's never-landed `c6b0f9bb` are corrected to `af9496ac`. |
| `tree_close` dedup (hygiene) | `accounting.cpp` and `rollout.cpp` carried byte-equivalent `tree_close`/`require_tree_close` copies; hoisted into `runtime_reference_support.hpp` beside `wire_bitwise`. rollout keeps file-local `kClose` (still used by its residual check) and drops unused `bitwise_equal`/`expect_error`/`ride` usings; accounting drops unused `expect_error` and the dead `kClose`. Header gains `<algorithm>`/`<cmath>`; rollout gains `<cmath>` for its remaining `std::abs`. |

### Fix-round verification

```
$ ctest --test-dir build/native-release -R 'reference\.native_runtime_' --output-on-failure
1/8 reference.native_runtime_bootstrap ............. Passed
2/8 reference.native_runtime_step .................. Passed
3/8 reference.native_runtime_commands .............. Passed
4/8 reference.native_runtime_ownership ............. Passed
5/8 reference.native_runtime_config ................ Passed   0.76 sec
6/8 reference.native_runtime_accounting ............ Passed   0.76 sec
7/8 reference.native_runtime_rollout ............... Passed   0.90 sec
8/8 reference.native_runtime_performance_contract .. Passed   0.51 sec
100% tests passed out of 8  (darwin-arm64, build/native-release)
```

### Fix-round commits

| SHA | Unit |
|---|---|
| `89d20efb` | fix(reference): pin field-path markers in `native_runtime_config` rejections |
| `916754f6` | refactor(reference): hoist shared `tree_close` into `runtime_reference_support` |
| (this fix-round commit) | docs(migration): C3.26 fix round — 38-port count, drop-record collected ids, commit-table reconciliation |

The four `planned` live-oracle reds, the two `drop` records, and the
two-platform qualification matrix are unchanged by this round; the
linux-aarch64 numbers above remain the recorded container results.
