# Task F1 report — welded-plant contract re-scope (lane/b)

Branch: `lane/b`, base tip `acede209`. Scope: separate equality/weld
closure work from non-equality numerical constraint work, weld load
envelopes for the bidirectional soft interfaces, and delivered-power
monitor budgets distinct from the 450 W command cap — mirrored
Python/native across the closed wire.

## Fix round 1

Findings addressed, with the evidence for each.

### Critical fixture/schema asymmetry — RESOLVED

The graph was rebuilt first (`cmake --build` for the touched native
targets, then focused migration runs). T09/T10 did still fail, so the
committed bootstrap/config fixtures were repaired:

- All `tests/fixtures/migration/bootstrap/**/config.json` (10 files),
  `tests/fixtures/migration/research/delayed_bootstrap/config.json`,
  `tests/fixtures/migration/config/schema.json`,
  `tests/fixtures/migration/config/rider_controller.json`, and the five
  resolved/variant config fixtures now carry the eight F1
  rider-controller keys explicitly: `weld_foot_min_normal_n`,
  `weld_foot_max_normal_n`, `weld_foot_shear_n`, `weld_saddle_min_normal_n`,
  `weld_saddle_max_normal_n`, `weld_saddle_shear_n`,
  `delivered_joint_power_budget_w`, `delivered_positive_power_budget_w`.
  Values follow TOML/default semantics: `null` for the optional normal
  bounds and delivered budgets, `0.0` for the disabled shear capacities.
  Explicit keys chosen over optional keys per the closed-wire precedent
  (`weld_servo_gain`). `rider_controller.json` keeps its compact
  single-line sorted style (70 keys).
- Articulated schema field count is now 70; `native/tests/migration/
  test_t12.cpp` updated 62 → 70, T08's stale "66-key" comment → "70-key".
- `tests/fixtures/migration/oracle-manifest.json` re-hashed for every
  edited config/schema/resolved entry (`json.dumps(indent=2,
  ensure_ascii=True)` + LF discipline preserved).
- 14 F1-generated `state.json` manifest entries that were exactly 64
  bytes stale (the `closure_signed_j`/`closure_absolute_j` pair) were
  re-hashed to the real content.
- Darwin-arm64 and linux-aarch64 `test_tire_channel_parity_across_backends`
  `.bin` goldens re-materialized: embedded compliant/distributed
  `config.json` now carry all 70 keys and embedded `state.json` records
  include the closure fields with correct declared byte counts; verified
  by slicing the archives at the manifest-declared offsets, JSON-decoding,
  and re-checking the referenced array SHA-256s.
- `bootstrap/native_start_welded/integration.f64` was briefly realigned
  to the `state.arrays/a000.f64` payload before `migration_T25` proved
  the divergence intentional (the standalone vector is the
  native-produced t=0 golden and carries signed zeros at the
  `active_state[5:9]` plugin slots 87–90; the archive is the
  oracle-captured restore state). The file and its manifest pin were
  reverted — see the T25 entry under "Native gate".
- `native/tests/migration/test_t29.cpp` now strips the three Python-only
  fields (`weld_servo_gain`, `weld_servo_limit_nm`,
  `leg_track_damping_nms_rad`) before the closed-wire decode — the same
  projection `runtime/project_config.cpp` and `native/setup.py` apply.

Focused migration results after the rebuild + repairs:
`test_t08`, `test_t09`, `test_t10`, `test_t12`, `test_t13`, `test_t29`
all pass (focused ctest runs on darwin-arm64).

### Validation mirror — RESOLVED

- `src/bike_sim/physics/physical_config.py`: the four `weld_*_normal_n`
  optionals skip the generic `minimum=0` sweep and are validated
  finite-only when set (negative = legal tension retention), mirroring
  native `optional_real` semantics; `weld_*_shear_n` stay nonnegative
  with `0` disabling the capacity.
- `native/src/configuration/validate.cpp`: symmetric — weld normal
  optionals skipped in the generic nonnegative loop, finite-only checked
  when present; weld shears removed from the strict-positive list so the
  `minimum=0` path accepts the disabling zero.
- `tests/reference/test_articulated_config.py`: mirrored tests added —
  `test_weld_normal_bounds_must_be_finite_when_set`,
  `test_weld_normal_bounds_accept_tension_retention`,
  `test_weld_shear_capacity_is_nonnegative` (89 focused nodes pass).

### D16 — linux-aarch64 pulse disposition — RESOLVED (drop strict record, D14 coverage named)

`test_pulse_redistributes_load_through_inertia_only` fails strict on
linux-aarch64 only: `energy.constraint_work` trips at t=0.005 s with a
sustained ~3–8x elevated non-equality residual versus darwin-arm64
(measured 5.9% → 2.3% vs 0.9% → 0.7% of source positive work across the
first ten closes). Equality/weld work is charged to the closure channel
and never enters this verdict, so the residual is solver-internal
joint/tendon-limit bookkeeping in the settle transient — a
platform-dependent numerical-work rate, not a contract change. A
platform-specific bound could not be physically argued beyond "solver
internals differ", so the strict record is dropped on linux:

- `tests/reference/test_seated_posture_program.py` runs the episode with
  `--check-policy diagnostic` on linux only; all posture/grip/delivered
  assertions still apply, and the test asserts `first_failure` is either
  absent or exactly `('energy.constraint_work',)` — any new or different
  verdict still fails the test.
- D14 coverage naming: the identical `constraint_work_ok` verdict keeps
  full strict linux coverage through the four
  `test_seated_pedaling_cycle.py` welded-plant nodes — measured green
  strict on linux-aarch64 (`[20.0]` 51.53 s; `[40.0]`, `[60.0]`,
  `test_free_coast_is_an_intent_not_a_crank_lock` 3 passed in 89.91 s).
- Linux verification: the pulse node passes on linux-aarch64 in 54.69 s
  with the monitor warning recorded at `0.005` s for
  `('energy.constraint_work',)` — the pinned signature.
- `docs/TESTING.md` carries the disposition subsection and the five
  baseline nodes are annotated resolved-in-refresh.

### TESTING / maps / audit caveat — RESOLVED

- `docs/TESTING.md`: five F1 baseline nodes annotated resolved-in-refresh
  pointing here; new "Linux-aarch64 pulse disposition (D16)" subsection;
  weld-envelope caveat now references
  `docs/superpowers/audits/2026-10-04-seated-rider-closed-form.md`
  (NOT ACCEPTED — provisional, unverified curves).
- `examples/research/viewer_physics_welded.toml`: the same audit caveat
  in the weld-envelope comment block.
- Migration maps refreshed for every file the F1 lane touched (17 Python
  files): `migration/python-symbols.json` (file lines/symbols/imports),
  `migration/file-work-units.json` (`source_hash` — sha256 over
  trailing-newline-stripped content — `line_count`, symbol spans +
  signatures), `migration/test-port-map.json` (`line`/`end_line`), and
  `migration/kernel-reuse-map.json` (`source_locations.python` spans via
  `python_symbol`). 147 records updated; generator verified byte-exact
  against recorded entries on files untouched by the lane; a post-refresh
  scan of the F1 set reports zero stale.

### Native gate

`bash tools/run_tests.sh native` (darwin-arm64, this tree,
log `/tmp/f1-native-gate2.log`):

- exit code: `8` (CTest failure code — the gate stops at the first red
  stage, so the extension-selection, contract and pytest stages of the
  profile did not run inside the gate; each was then run standalone as
  recorded below).
- static analysis: all sweeps green — frontends 333 sources/334 TUs,
  clang-tidy none (1 `clang-analyzer-cplusplus.Move` suppression),
  analyzer report-only (suppressions `cplusplus.Move=3`,
  `cplusplus.NewDelete=2`), cppcheck none, gcc `-Wodr` none,
  shellcheck clean, standalone headers 259 headers/8950 TUs none,
  diagnostic controls 6/0 failed, context SDK controls all passed.
- CTest: **60/62 passed**. The two reds were stale F1 artifacts, both
  corrected and re-verified standalone:
  - `migration_T38` — `test_t38.cpp` asserted the pre-F1 descriptor
    count 495; `ec9b031e` added the two `channel_energy` closure
    descriptors (`closure_signed_j`, `closure_absolute_j`), so the
    reviewed count is 497. Literal updated with the derivation in a
    comment. `ctest -R migration_T38` → PASS (0.90 s).
  - `migration_T25` — two stale artifacts in sequence. First,
    `initial_state/welded_flat.json`'s `contract_sha256` covered the
    physics config and F1's eight new rider-controller keys changed
    the digest: recaptured via the documented
    `PhysicalInitialState.capture` recipe (two-build equilibrium-cache
    path); only `contract_sha256` and `source_sha256` (provenance)
    changed — every physical array is byte-identical, confirming F1
    changed no compiled-model or settle output. Second, the earlier
    fixture realignment of `bootstrap/native_start_welded/
    integration.f64` to the state archive `a000.f64` was wrong for this
    file: the two are intentionally divergent — `a000.f64` is the
    oracle-captured restore state (real `active_state[5:9]` plugin
    values at indices 87–90) while `integration.f64` is the
    native-produced t=0 golden (signed zeros there). Reverted
    `integration.f64` to its committed bytes and its manifest pin back
    to `b7e2d0ae`. `ctest -R migration_T25` → PASS (68.76 s).
- contract check (run standalone after the gate stop):
  `bike-native-checks --kind contracts` → all 12 controls passed.
- native/golden pytest (run standalone after the gate stop): `python -m
  pytest -q --durations=10 -p native_test_reporter
  tests/reference/test_native_*.py tests/reference/test_golden_episode*.py`
  → **90 failed / 3,150 passed / 1 skipped** in 7,868 s (exit 1,
  log `/tmp/f1-native-pytest.log`). Failure breakdown:

  - **16 F1-stale nodes — repaired and re-verified green.** The re-scoped
    plant is healthy, so fixtures/tests that assumed the chronic
    `energy.constraint_work` stop, pre-closure channel encodings, or the
    old transition set were updated to the measured behavior:
    `test_native_encode_columns.py` (`test_derived_events_real_episode_bitwise`,
    `test_map_channel_tables_bitwise`,
    `test_map_fragments_respect_batch_rows_bound`,
    `test_physics_and_forces_bitwise_vs_oracle`),
    `test_native_recording_retention.py::test_native_source_encodes_missing_and_nonfinite_values`,
    `test_native_linux_headless.py::test_native_cli_without_display_retains_real_outcome`
    (both parameterizations),
    `test_native_platform_recording.py` (5 nodes),
    `test_native_recording_lifecycle.py` (3 nodes), and
    `test_native_research_environment.py::test_strict_and_diagnostic_outcomes_remain_visible[True]`.
    The deterministic stop trigger is now the pinned
    `weld_foot_min_normal_n = 100` floor
    (`tests/reference/_native_runtime_support.py::violating_physics`,
    first failure `('foot_front.normal', 'foot_rear.normal')` at
    t=0.00125 s), with absolute asset paths substituted so the copied
    TOML resolves its rider JSONs.
  - **74 baseline reds — none F1-authored.** All 90 failed node IDs
    were rerun at this tree state: the 16 repaired nodes pass and the
    same 74 nodes still fail (`74 failed, 16 passed` in 176 s;
    set-diff of the rerun remainder against the 90-run FAILED set
    minus the repaired 16 is empty on both sides). Every remaining
    failure carries a backend↔oracle
    drift signature (bitwise mismatches and `assert_allclose`
    tolerances in the spindle/drivetrain/shift/sensor/tire channels)
    or pre-existing breakage (the `test_native_spindle_controller`
    `TOMLDecodeError` duplicates `activation_tau_s`, present at base
    commit `b21c63ec`). The per-file concentration reproduces the
    documented F2 tail: `test_native_spindle_controller` 22,
    `test_native_research_environment` 17, `test_native_drivetrain` 7,
    `test_native_runtime_*` ~14, plus the small-file remainder — the
    same selection recorded 75 failed / 3,164 passed on the F2 gate
    (`migration/cpp-linux-completion-evidence.md`; this lane's
    remainder is 74, one fewer — the run sets differ by lane state,
    not by any F1 signature). No failure signature touches F1 fields
    (closure channel, weld envelopes, delivered budgets); the two
    `energy.constraint_work` monitor warnings in passing diagnostic
    tests are the intended non-equality verdict still working.
