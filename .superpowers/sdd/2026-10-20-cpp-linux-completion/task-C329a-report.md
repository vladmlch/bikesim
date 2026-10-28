# Task C329a — test-port 3.29a report (engine errors, native loader; `checks`, T53)

**Batch**: 3.29a — `tests/reference/test_native_engine_errors.py` (24 records /
31 collected nodes) and `tests/reference/test_native_loader.py` (20 records /
27 collected nodes). CTest ids `reference.native_engine_errors` +
`reference.native_loader.<case>`; labels `t53 checks`; port-map rows carry
`owner_task: T53`.

**Lane**: `.worktrees/pfc-lane-e`, branch `lane/e`, base `0195b091`
(post-3.23/3.30 integration tip). No merge, no push.

## Status: landed — 24 `port`, 20 `drop`, 0 `planned`

24 of 44 records ported: 21 into the new C++ executable
`reference.native_engine_errors` and 3 into the already-registered
`cmake -P` CTest script cases under `reference.native_loader.*`.
20 records are `drop` (`python-only`) — three vendored stock↔native
interop probes in the engine file and the 17 loader functions that only
exercise extension-import / pytest-reporter / sanitizer-preload machinery.
**No golden fixtures were needed or created**: the batch is contract-only
(zero `bt::oracle::Node` sites, zero live-oracle comparisons).

## Triage

| Source file | records | collected nodes | port | drop |
|---|---|---|---|---|
| `test_native_engine_errors.py` | 24 | 31 | 21 | 3 |
| `test_native_loader.py` | 20 | 27 | 3 | 17 |
| **total** | **44** | **58** | **24** | **20** |

## Engine-errors port map (`reference.native_engine_errors`)

All 21 ported C++ functions keep the Python symbol names verbatim, so
`native_test_id` is `reference.native_engine_errors::<source_symbol>`:

- `test_stock_binding_raises_fatal_error_and_survives` — framed
  `engine::try_forward` on `opt.solver=99` reports `kind==fatal` +
  "unknown solver type 99"; `engine::forward` throws `EngineFailure`;
  the frame stays reusable (two healthy forwards follow in-process). The
  stock binding's `mujoco.FatalError` is the vendor mapping of the same
  recovery — `engine::EngineFailure` is its native half.
- `test_native_fatal_constructor_preserves_process` — `Stepper(path)` on
  the invalid-solver MJB throws `EngineFailure` naming the diagnostic;
  a valid `Stepper` then constructs/forwards/steps in the same process.
  The two pytest params (`mujoco_first`/`native_first`) are import-order
  and collapse to the single native check; both collected node ids map
  to this one function.
- `test_valid_solver_still_constructs_and_forwards` — valid-input control.
- `test_runtime_fatal_poisons_stepper_until_reset` — inlines the binding's
  `_engine_test_forward_failure` mutate-tamper verbatim; forward/step throw
  the poisoned `std::logic_error`, `qpos` stays readable (size 7), `reset()`
  recovers.
- `test_set_const_fatal_poisons_owner_and_preserves_process` — inlines
  `_engine_test_set_const_failure` verbatim (body_simple/ipos tamper);
  EngineFailure message contains "simple"; poisoned; `reset()` recovers.
- `test_valid_set_state_can_recover_after_rejected_restore` — bad-width
  `set_state` throws `invalid_argument` while the owner stays poisoned;
  valid `set_state` recovers.
- `test_configured_owner_recovery_clears_pending_drive_state` — both
  recovery params (`reset` / `set_state`) on
  `drivetrain_reference::pair("geometric_ideal_mid_drive")`:
  `components()` stages `last_time_s`+`pending_actuation`, fatal injection
  poisons, recovery clears both, `components()` works again.
- `test_realtime_status_path_reports_fatal_without_throwing_inside_marked_region`
  — both ops (`forward`/`step`) via the inlined `_engine_test_try_status`
  mirror: `(false, "…unknown solver type 99")`, owner poisoned, `reset()`
  recovers.
- `test_installed_python_callback_is_rejected_before_native_construction`
  — foreign `mjcb_control` → ctor `invalid_argument` naming the callback;
  cleared → ctor+forward succeed (the "stock usable" half).
- `test_installed_callback_rejects_runtime_call_without_poison` —
  `forward()` rejects with `mjcb_control` message, `time()` unchanged,
  owner healthy after clearing.
- `test_python_time_callback_is_rejected_before_marked_runtime_step` —
  foreign `mjcb_time` → `step()` rejects, `time()` unchanged.
- `test_ctypes_time_callback_is_rejected_before_native_construction` —
  counting foreign timer, rejected by address, `calls == 0`.
- `test_ctypes_time_callback_is_rejected_before_native_recovery` —
  `reset`/`set_state`/`mutate(set_const)` all reject with `mjcb_time`,
  `calls == 0`, `time()` preserved, `step()` works after clearing.
- `test_simulate_owned_time_callback_is_accepted_for_runtime_steps` —
  **owner substitution**: `mjcb_time = &engine::native_time` (the one
  whitelisted non-null owner in the Python-free graph; `_simulate`-image
  recognition lives only in the bindings probe). ctor/step/forward/reset
  (`time()==0`)/step all accepted.
- `test_unsupported_callback_rejected_before_drive_restore` —
  `mjcb_control` + `mutate(restore(corrupted))` rejects before decode;
  `canonical_json(state)` byte-identical afterwards.
- `test_stock_timer_installed_after_native_construction_is_allowed` —
  `engine::native_time` installed after construction is accepted on the
  next `step()` (provenance re-classified per call).
- `test_plugin_model_is_rejected_before_native_engine_loading` —
  **fixture substitution** (no `mujoco.pid` binary ships in the locked
  `.deps`): valid saved image with the serialized `nplugin` field
  (offset `5·4 + 72·8`, per `contains_plugins`) patched to 1 →
  `invalid_argument` "unsupported MuJoCo plugin model". Unpatched file
  constructs as the valid-input control.
- `test_invalid_history_timestep_rejected_before_make_data` — actuator-
  delay model with `opt.timestep=0` → `invalid_argument` "history
  requires positive timestep" from `engine::make_data`'s pre-check.
- `test_bad_model_file_raises_without_exiting` — all three payloads
  (absent / empty / `not an MJB`) → `std::runtime_error`, explicitly not
  `EngineFailure`; process survives (valid owner constructs after).
- `test_actual_engine_allocation_error_is_caught_in_c_contract` — real
  subprocess isolation: `platform::run` spawns the selected build's
  `native_contract_tests` for all three `--*-allocation-failure` probes;
  exit 0 + "fatal allocation" in stdout. Binary resolved through the
  generated `bike_engine_errors_paths.hpp` macro.
- `test_configure_rejects_unsupported_installed_mujoco_version` —
  **gate substitution** (no Python package exists): scratch
  `BIKE_DEPENDENCY_ROOT` with every dep symlinked from the real `.deps`
  except `mujoco-3.12.0/BIKE_VERSION` = `3.15.0`; a real scratch
  `cmake -S . -B …` configure fails naming `'mujoco'`,
  `requires version '3.12.0'` and `'3.15.0'` — the lock's marker gate
  (fired before that dep's license/archive/hash checks).

## Engine-errors drops (3)

| Function | Justification |
|---|---|
| `test_stock_time_callback_error_preserves_python_exception` | Asserts a Python callable's exception text survives through the stock binding's `mjcb_time` into the native ctor — vendored nanobind exception-channel machinery. Native half (foreign timer rejected by address) is ported as `test_ctypes_time_callback_is_rejected_before_native_construction`. |
| `test_stock_fatal_handler_nests_inside_native_timer_callback` | Stock FatalError caught inside a timer callback while a native framed forward is in flight — needs the vendored binding inside a foreign callback; the Python-free core rejects foreign addresses pre-invocation. Native frame nesting is covered by `native_contract_tests` `nested_engine_frames`. |
| `test_nested_stock_native_warning_does_not_recurse` | Stock→native warning forwarding exactly once — stock handler's Python owner has no analog. Native forwarding to the previous TLS handler is covered by `native_contract_tests` `previous_tls_handler_restored`. |

## Loader per-function triage (20)

The whole file was triaged per instruction before porting — **not** a
blanket drop:

| Function | Fate | Justification / replacement |
|---|---|---|
| `test_selected_build_loads_the_exact_imported_extension` | drop | extension-import machinery; no `.so` exists in the Python-free graph |
| `test_load_native_records_selected_build_and_imported_extension` | drop | `load_native()` module-provenance fields — interpreter machinery |
| `test_pytest_reporter_records_selected_native_count_and_provenance` | drop | pytest reporter item counting/provenance — harness-internal |
| `test_pytest_reporter_does_not_count_native_tool_tests` | drop | pytest reporter item classification — harness-internal |
| `test_repeated_load_after_collection_preserves_native_count` | drop | `load_native()` idempotence + reporter state — Python-only |
| `test_loader_discards_invalid_or_other_artifact_test_counts` | drop | reporter count-file validation — pytest sidecar format |
| `test_custom_absolute_build_path_loads_its_extension` | drop | `BIKE_NATIVE_BUILD_PATH` extension import — cmake -S/-B selects the build natively |
| `test_legacy_build_selector_resolves_only_known_builds` | drop | legacy build-name selector env surface |
| `test_conflicting_build_selectors_fail_in_a_child_process` | drop | mutually exclusive selector envs in a Python child |
| `test_relative_build_path_is_rejected_in_a_child_process` | drop | relative build-path rejection — loader env surface |
| `test_preimported_module_from_another_build_is_rejected` | drop | `sys.modules` provenance check — interpreter module cache |
| `test_missing_extension_fails_without_import_fallback` | drop | missing `.so` import failure — extension loading |
| `test_ambiguous_extensions_fail_before_import` | drop | ambiguous `.so` rejection pre-import — extension loading |
| `test_rider_forces_import_rejects_an_earlier_wrong_build` | drop | forced re-import wrong-build rejection — import machinery |
| `test_native_profile_reporter_rejects_zero_native_items` | drop | pytest profile reporter zero-item rejection — harness-internal |
| `test_child_confirms_selected_asan_runtime_is_loaded` | drop | `DYLD_INSERT_LIBRARIES`/`LD_PRELOAD` sanitizer check in a Python child — root build links runtimes directly |
| `test_child_rejects_wrong_preload_when_selected_runtime_is_missing` | drop | wrong-preload rejection — sanitizer preload is interpreter machinery |
| `test_native_contract_require_control_fails_in_selected_build` | **port** | `reference.native_loader.test_native_contract_require_control_fails_in_selected_build` (`cmake -P test_native_loader_contracts.cmake`) — nonzero exit + "deliberate require failure" stderr |
| `test_native_contract_executable_lists_registered_cases` | **port** | `reference.native_loader.test_native_contract_executable_lists_registered_cases` — exact literal 46-case `--list` order |
| `test_contract_executable_source_is_in_cmake_manifest` | **port** | `reference.native_loader.test_contract_executable_source_is_in_cmake_manifest` — `test_contracts.cpp` appears exactly once in `native_sources.json` under target `native_contract_tests` |

## Verification evidence

- **Oracle green (darwin-arm64)**: `NATIVE_TEST_BUILD_PATH=native/build
  uv run --frozen --group native pytest -x -q
  tests/reference/test_native_engine_errors.py` — **31 passed** in
  19.21 s. The linux oracle run was not required (no live-oracle/golden
  comparisons in the batch). Collection measured for both files
  (`--collect-only -q`): 31 + 27 node ids recorded in the ledger.
- **CTest darwin-arm64** (`build/native-release`, AppleClang, Release):
  `ctest -R 'reference\.(native_engine_errors|native_loader)'` —
  **4/4 Passed** (engine_errors 7.61–8.40 s; loader cases ≈0.3 s).
  `ctest -N -L checks` lists exactly these 4 — the label is the batch.
- **CTest linux-aarch64** (container `bike-dev-pfc-lane-e`, committed
  tree synced via `git archive` → `/src`, build `/build/root-ref`
  clang++-18 Ninja Release): `ctest -R 'reference\.(native_engine_errors|
  native_loader)'` — **4/4 Passed** (engine_errors 1.54 s).
- **Red→green proof**: flipped `"unknown solver type 99"` → `98` in the
  fatal-status assertion — `FAIL fatal status preserves the engine
  diagnostic`; reverted — **PASS**. Asserts fire.
- **Static registration**: `Phase3Ports.cmake` —
  `bike_add_reference_test(native_engine_errors ENGINE … LABELS t53 checks)`
  + `add_dependencies(… native_contract_tests)` + `file(GENERATE)`
  `bike_engine_errors_paths.hpp` (same pattern as
  `bike_native_bench_path.hpp`). The 3 loader script cases were already
  registered (`foreach` over `test_native_loader_contracts.cmake`,
  `LABELS "reference;t53;checks"`, `add_test` + `cmake -P`, per the
  `migration_T03`/`migration_T56` precedent).
- **No `run_tests.sh` gate** per lane instruction; scope is the targeted
  CTest label plus the source-suite status run above.

## Ledger

- `migration/test-port-map.json`: 24 `port` (contract) + 20 `drop`
  (python-only) across 44 records — measured `collected_node_ids`,
  measured `assertions_capture`, per-record `evidence`; `native_test_id`
  uses `reference.native_engine_errors::<symbol>` for C++ ports and the
  `reference.native_loader.<case>` script-node names for the loader.
  Diff verified to touch exactly the 44 batch records.
- `migration/python-removal.json`: both source rows →
  `status: replacement_landed`, `verification_ids` set to the real CTest
  ids; the loader row's stale `phase3_source_implementation` block
  removed (matches landed-row convention). Deletion stays gated at T63.

## Defects found

1. **Scratch-configure dep iteration order** (fixed, this batch): the
   first `BIKE_DEPENDENCY_ROOT` probe failed on `cairo` before reaching
   the patched `mujoco` marker — `string(JSON … MEMBER)` iterates
   alphabetically, not in file order. Fixed by symlinking every real dep
   tree except `mujoco-3.12.0`, which carries only the doctored marker.
2. **clang-18 `-Wmissing-field-initializers`** (fixed, this batch):
   `ProcessOptions{.timeout_s = …}` designated init failed the container
   build where darwin clang accepted it; replaced with assign-after-
   default-construction.
3. **`mujoco.pid` plugin binaries absent from `.deps`** (worked around,
   documented deviation): the plugin-model record patches the serialized
   `nplugin` field instead of compiling a real plugin MJB — the same
   `contains_plugins` pre-screen rejects before `mj_loadModelBuffer`.

No baseline or unresolved defects. No live-oracle red family exists in
this batch — the source suite is fully green (31/31) and no `planned`
record remains.

## Commits

| SHA | Unit |
|---|---|
| `60dd925c` | 3.29a T53 engine-errors C++ port + Phase3Ports registration (darwin-arm64 green) |
| `d8099320` | 3.29a T53 ledger qualification evidence — 24 port + 20 drop, measured node ids, both removal rows `replacement_landed` |
| (this report) | task-C329a-report.md |
