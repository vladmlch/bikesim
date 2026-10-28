# Task C327 — test-port 3.27 report (state bundles + state restoration)

**Batch**: 3.27 — `tests/reference/test_native_state_{bundle,fuzz,
restore}.py` and `tests/reference/test_equilibrium_state_file.py`
(63 records / 121 collected nodes: 61 T01-seeded records plus two
equilibrium source tests added post-T01 by `8bb51212`; the
parametrized restore relational matrix expands to 22 nodes and the
bundle header/resolver sweeps to 20).
CTest ids `reference.{native_state_bundle,native_state_restore,
native_state_fuzz,equilibrium_state_file}`; labels `t53 state`;
port-map rows carry `owner_task: T53`.

**Lane**: `.worktrees/pfc-lane-d`, branch `lane/d`, base `41d5e081`
(post-3.26 merge-back). No merge, no push.

## Status: landed — 36 `port`, 2 `golden`, 25 `drop`, 0 `planned`

All 63 records hold concrete dispositions; no live-oracle reds exist in
scope (baseline shows the four source files green on both platforms, so
nothing stayed `planned`). Two restore records are `golden`: they replay
the pre-existing platform-qualified `pinned_savage_40_f2` episode
fixture and compare the native forward bitwise against a live MuJoCo
`mjData` oracle. The 25 `drop` records are all in the bundle file: 18
reader/resolver rejection contracts already gated end to end by the
`reference.native_benchmark` port through the real consumer, and 7
writer-side Python tooling contracts with no native owner.

## Triage

| Source file | records | collected nodes | port | golden | drop |
|---|---|---|---|---|---|
| `test_native_state_bundle.py` | 31 | 52 | 6 | 0 | 25 |
| `test_native_state_restore.py` | 19 | 54 | 17 | 2 | 0 |
| `test_native_state_fuzz.py` | 2 | 2 | 2 | 0 | 0 |
| `test_equilibrium_state_file.py` | 11 | 13 | 11 | 0 | 0 |
| **total** | **63** | **121** | **36** | **2** | **25** |

## What the ports reproduce

- **`reference.native_state_bundle`** (6 checks): the 80-byte BIKEST02
  header static pin plus a generated-image field walk (magic, version,
  flags, all five u64 dims, payload extent, both crc64 slots at their
  documented offsets); a valid `proto_native_bench::load_state` round
  trip with bitwise section equality and retained declared checksums;
  the out-of-domain u64 sweep over **all five** dimension fields (the
  benchmark CLI test samples nq only); the declared model_crc64 lie
  against intact model bytes; the full five-name hostile bundle-pointer
  sweep through the `native_bench` argv boundary including the
  exact-match `.`/`..` rejections the charset check would otherwise
  allow; and a nonexistent bundle root. In-process reader calls link the
  same `state_format.hpp` inline the consumer compiles; resolver calls
  keep the executable boundary via the generated
  `bike_native_bench_path.hpp` macro.
- **`reference.native_state_restore`** (17 checks): `Stepper::set_state`
  snapshot-before-`mj_resetData` aliasing (own views in the destination
  slots *and* a qpos view riding the qvel argument; `qacc` warmstart
  included), width/finiteness/time validation ordering before mutation,
  bitwise live-buffer preservation on every rejection arm,
  `set_inputs` nonfinite rejection, replay of the 40-row
  `pinned_savage_40_f2` platform episode with bitwise `qacc`,
  `qfrc_constraint` and `efc_force` comparison against a separate
  `mjData` forward, and the drive/policy restore relational guards —
  ratio/gear, rear-teeth, coefficient width/drift, prepared-Jacobian,
  shift clock/count, cooldown/cut relations, battery bounds, pending
  actuation ⇔ interval pairing, nonpositive pending dt, orphaned vs
  accepted `shift_pending`, cassette membership with shifter enabled,
  auxiliary clutch/freewheel topology immutability, and the standalone
  `DrivePolicies::set_state` relational matrix — every rejection leaves
  the encoded state byte-identical.
- **`reference.native_state_fuzz`** (2 checks): deterministic seeded
  splitmix64 property tests mirroring the Hypothesis mutation model —
  150 drive-snapshot examples (`set`/`del`/`add` ops over dict paths
  with recursive JSON values) where rejection must stay inside the
  sanctioned contract set (`invalid_argument`/`out_of_range`/
  `overflow_error`/`length_error`/`runtime_error`) and leave
  canonical-JSON byte-identical state, plus 100 `set_state` examples
  over arbitrary-width double spans (NaN/±Inf/subnormals/signed zero/
  extremes) where rejection leaves every live buffer bitwise unchanged;
  both arms are required non-vacuous.
- **`reference.equilibrium_state_file`** (11 checks): explicit
  state-file round trip (recorded settle steps served, residual
  rechecked, read-only file), precedence over a sentinel-poisoned
  canonical cache entry, fingerprint/configuration mismatch, payload-fit
  (strict all-or-nothing material check), residual-recheck rejection
  under a +5 cm `root_z` shift, missing-file naming, blank/wrong-kind
  field validation, TOML relative-path resolution against the declaring
  file, `EquilibriumOptions` reach-through plus the nonfinite-tolerance
  and nonpositive-budget guards, and the post-`8bb51212` lifecycle pin:
  single-shot settle rejects a second call, and
  settle→seal→advance→reset→advance→reset leaves exactly one
  byte-identical canonical entry that a fresh consumer warm-loads — the
  strictly stronger native equivalent of the episode-key quarantine.

## Individual dispositions — drops (all in `test_native_state_bundle.py`)

**Covered end to end by `reference.native_benchmark`** (the real
consumer exercises `resolve_artifacts` + `load_state` through the
executable — 18 records): `test_crc64_ecma_check_vector`
(`--selftest` asserts the published vector inside the binary),
`test_legacy_magic_is_rejected`, `test_bad_magic_is_rejected`,
`test_unknown_version_is_rejected`, `test_unknown_flags_are_rejected`,
`test_dimension_mismatch_is_rejected` (all five fields),
`test_declared_payload_length_mismatch_is_rejected`,
`test_truncated_header_is_rejected`,
`test_truncated_payload_is_rejected`, `test_trailing_bytes_are_rejected`,
`test_payload_checksum_corruption_is_rejected` (both payload-mutation
directions), `test_mixed_generation_is_rejected`,
`test_nonfinite_stored_scalar_is_rejected`,
`test_resolve_concrete_bundle_dir`, `test_resolve_root_uses_pointer`,
`test_resolve_malformed_pointer_is_rejected`,
`test_resolve_pointer_to_missing_bundle_is_rejected`,
`test_resolve_incomplete_bundle_is_rejected` (four missing-member tests).

**Python-writer tooling contracts with no native owner** (7 records):
`test_bundle_npz_matches_bin`, `test_write_bundle_publishes_pointer`,
`test_bundle_name_collision_is_rejected`,
`test_manifest_records_provenance`,
`test_write_bundle_rejects_nonfinite_snapshot`,
`test_serialize_rejects_mismatched_qfrc_extent`,
`test_serialize_rejects_bad_xfrc_extent`. The C++ consumer reads
`state.bin` + `model.mjb` and checks member existence only; the
writer, pointer publisher, manifest author and `serialize_state`
extent guards live in `tools/proto_native_bench/state_format.py`,
which removes with the Python tree at T63.

## Two-platform qualification matrix

| Surface | darwin-arm64 | linux-aarch64 |
|---|---|---|
| `reference.native_state_bundle` | PASS (0.24 s, `build/native-release`) | PASS (0.10 s, container `bike-dev-pfc-lane-d` `/build/root-clang`) |
| `reference.native_state_restore` | PASS (0.23 s) | PASS (0.13 s) |
| `reference.native_state_fuzz` | PASS (0.09 s) | PASS (0.01 s) |
| `reference.equilibrium_state_file` | PASS (4.95 s) | PASS (4.51 s) |
| `ctest -R state` (the four ids) | **4/4** | **4/4** |

Golden capture: **none new** — the two `golden` records consume the
pre-existing two-platform `pinned_savage_40_f2` episode fixture
(`tests/fixtures/golden/{darwin-arm64,linux-aarch64}/episode/`,
captured post-F2); the fuzz port is a deterministic seeded property
test by construction (no golden fuzz tests, per batch requirements).

## Commits

| SHA | Unit |
|---|---|
| `19514baf` | test(reference): port state bundle/restore/fuzz/equilibrium-state-file suite (C3.27) — 4 CTest sources + `Phase3Ports.cmake` registration (`LABELS t53 state`; bundle links `bike_app` + `native_bench` dependency for the argv boundary; restore links `bike_golden` for the episode fixture) |
| `6da60648` | docs(migration): C3.27 state batch ledger — `test-port-map.json` (36 port / 2 golden / 25 drop, 2 new post-T01 records, measured `collected_node_ids`, dated `assertions_capture`, per-record evidence) + `python-removal.json` (all four files → `replacement_landed`) |
| (this commit) | docs(migration): C3.27 evidence + completion report (this report) |

## Deviations

1. **Post-T01 source records added**: `8bb51212` introduced
   `test_experiment_configs_produce_different_episode_keys` and
   `test_dirty_resettle_writes_the_episode_key_not_the_canonical_name`
   after the ledger was seeded. Two records were appended beside the
   file's existing rows (`line`/`end_line` measured) and both ported
   through `test_post_episode_lifecycle_writes_no_cache_entry` — the
   native guarantee is strictly stronger (no post-episode settle exists
   at all: `settle` is single-shot and `reset()` replays the sealed
   bootstrap), so the port pins byte-identical canonical bytes and a
   single cache entry across the whole lifecycle rather than a
   quarantined second name. The episode-key mechanism itself is
   `python-only` — no native trigger exists.
2. **The kwargs-require-physics arm is `python-only`**: the source's
   `RideSimulation(equilibrium_max_steps=4000)` constructor rejection
   has no native twin (the request type does not carry those fields
   when unset); `EquilibriumOptions` domain guards and solve reach-
   through are ported. Recorded per-record in the ledger.
3. **The fuzz cast-layer sub-draws are `python-only`**: the source
   passes arbitrary JSON objects to `set_state`, exercising the
   nanobind cast layer; the typed C++ API has no counterpart — the port
   covers the full f64-span domain (arbitrary widths and doubles),
   which is where every width/finiteness check lives.
4. **`pitch_assist` stickiness in the field-validation port**:
   `default_physics()` resolves `pitch_assist=true` at construction
   (legacy default), so flipping `physics_mode` to `"physical"` trips
   the physical-mode pitch-assist guard before the state-file check;
   the test clears `pitch_assist` explicitly so only the field under
   test is invalid.
5. **`-Wframe-larger-than` waiver**: `test_equilibrium_state_file.cpp`
   carries the established `NATIVE_DIAG_PUSH`/`-Wframe-larger-than`
   waiver for whole decoded `StateFile` fixtures in automatic storage —
   same precedent as the T22–T26 runtime ports; both platforms compile
   clean.
6. **`-Wmissing-field-initializers` (linux clang)**: designated
   `EquilibriumOptions` initializers name both members explicitly —
   darwin accepted the partial form, the container's clang-18 sweep
   does not.

## Final gate

`bash tools/run_tests.sh native` (selected build `native/build`,
standalone `bike-native-checks` from `build/native-release`):

- Configure + build green; **all nine static-analysis sweeps green**
  (`frontends` none, `tidy` none, `analyzer` report-only alpha,
  `cppcheck` none, `odr` none, `scripts` shellcheck clean, `headers`
  none — 262-header probe set, `diagnostic-controls` 6/0, `context` SDK
  controls passed).
- `ctest --test-dir native/build`: **60/62** — reds `migration_T09`
  (`post-advance integration vector matches the oracle`) and
  `migration_T10` (`first transition matches the Python oracle`), the
  documented frozen-oracle baseline pair verbatim (same exit-8-at-ctest
  end state recorded for the prior lane gates; see
  `migration/cpp-linux-completion-evidence.md` T09/T10 entries).
- `contracts` checker standalone (the gate-abort skipped stage):
  `bike-native-checks --kind contracts --build native/build
  --repo-root .` — **all assertions passed** (`pure_core_python_free`,
  `hot_kernel_dependency_free`, `ctest_contract_cases`, …).
- Native reference pytest tail under the gate's own runtime/env
  (`native_runtime.py run --build native/build -- python -m pytest -q
  --durations=10 --ignore output tests/reference/test_native_*.py
  tests/reference/test_golden_episode*.py -p native_test_reporter`):
  **3196 passed / 43 failed / 1 skipped / 1 deselected** in 20:39.
  Every one of the 43 failures maps to a ledger record —
  **39 `planned` + 3 `drop` + 1 `golden`** — the same composition the
  previous lane gate recorded; **zero failures in C3.27 scope**.
  The 1 deselected node is
  `test_native_check_tools.py::test_standalone_header_sweep_compiles_
  first_party_headers`: a designed ~90-minute serial 9136-TU sweep whose
  in-suite duplication was running at ~14 probes/min under
  sibling-worktree CPU load (>10 h projected); the identical sweep
  passed standalone inside the gate (`bike-native-checks --kind
  headers`, 262 headers, all TUs rc=0), so the deselection removes a
  mechanical re-run, not coverage.

Zero C3.27-introduced failures: the only reds are the documented
baseline pair, and the gate-abort state is identical to the previous
lane gate's recorded verdict ("single-run exit-0 unreachable while the
documented T09/T10 baseline pair is red").

## Full standalone-tree ctest (`build/native-release`, all 166 tests)

A first full-tree run surfaced 19 stale-binary "Not Run" entries (the
gate builds only `bike-native-checks` in this graph; the lane had never
compiled the remaining test targets) plus cascading stale-binary
failures (e.g. `unknown ArticulatedConfig parameter(s)` rejections of
newer fixture keys). After `cmake --build build/native-release -j8`
(100 % green) the authoritative rerun is:

- **159/166 passed** — the four C3.27 ids green
  (`native_state_bundle` 2.36 s, `native_state_restore` 0.75 s,
  `native_state_fuzz` 0.22 s, `equilibrium_state_file` 8.80 s; label
  group `t53 state`).
- **7 reds, all documented baseline, signatures verbatim**:
  `migration_T09`/`migration_T10` (frozen-oracle pair),
  `migration_T56` (shellcheck-0.11.0 stderr-format environment drift),
  and the documented four-red reference baseline
  `reference.golden_episode_tool`, `reference.native_numeric_completion`,
  `reference.pneumatic_tire_oracle`, `reference.native_pneumatic_tire`.
- Everything else passes, including `reference.native_benchmark` (the
  consumer-side gate for 18 bundle `drop` records),
  `reference.native_coupled_rider_task` (376 s) and
  `reference.coupled_rider_task` (1555 s under sibling-worktree CPU
  contention), `install_smoke` and `package_smoke`.

## Concerns / notes for merge-back

1. All four `python-removal.json` rows moved to `replacement_landed`
   — every record in each source file is non-planned with native
   evidence; deletion still waits on the T63 gate.
2. The bundle row's `verification_ids` names
   `reference.native_benchmark` alongside
   `reference.native_state_bundle`: 18 of its records are gated by the
   benchmark executable port, so removing the source file removes no
   un-covered contract.
3. The two `golden` restore records depend on
   `pinned_savage_40_f2` under both platform dirs — a fixture captured
   in an earlier batch, not re-captured here.
4. Two new ledger records carry the 8bb51212 provenance note; consumers
   reconciling record counts against the T01 seed should expect 63 rows
   for this scope, not 61.
