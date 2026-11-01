# Task C339/340 — validation qualification report (tasks 3.39 + 3.40)

**Batch**: plan §"Validation rigs (T54)" tasks 3.39 and 3.40 —
`src/bike_sim/validation/{plant_prefix,plant_release,plant_torque_rig,
rider_cases,rider_factor_matrix,rider_replay,rider_work,system_momentum,
ride_cases}.py` plus
`src/bike_sim/validation/{__init__,antiwheelie_bench,antiwheelie_reports,
benchmarks,climbing_benchmark,climbing_evidence,experiment_acceptance,
fidelity_sweep,reference_release}.py` (19 of the 30 T54-owned source rows).
Native targets `native/src/validation/{rider,drivetrain,acceptance,
evidence}.cpp` and `native/src/tools/validate_main.cpp` already existed
with per-row `phase3_source_implementation` triage; this lane's job was
qualification evidence plus the small coercion/fidelity gaps the triage
named, not a from-scratch port.

**Lane**: `.worktrees/pfc-lane-g`, branch `lane/g2`, base `d1d26d3f`.
Darwin arm64 only; the linux-aarch64 half of every verdict claim is
deferred to the Linux lane per the two-platform convention.

## Status: 11 rows `replacement_landed`, 15 rows stay `planned`

The prior ledger triage marks each row either
`source_implemented_pending_qualification` (implementation judged
complete, needs evidence) or `partial_source_implementation` (a concrete
residual gap remains). This batch qualified the pending set with new
RCP-16 bitwise oracle verdicts and bounded stand contracts, closed three
named implementation gaps (`check_bounds` `float()` coercion,
`operator_intervention` `or`-truthiness, `shock_total` reconstruction),
and left the partial set `planned` with updated honest residual notes.

| Python source | p3 triage | Native twin(s) | New evidence | Disposition |
|---|---|---|---|---|
| `validation/plant_prefix.py` (43) | pending_qualification | `validation/evidence.cpp`, `registry.cpp` | bitwise `valid_prefix` verdict (seen/accepted, first-bad record, impulse prefix) | **landed** |
| `validation/plant_release.py` (40) | pending_qualification | `validation/acceptance.cpp`, `evidence.cpp` | bitwise `plant_release` + `trace_integral` verdicts | **landed** |
| `validation/plant_torque_rig.py` (40) | pending_qualification | `validation/rigs.cpp` | bounded `shaft_ratio`/`actual_shaft_ratio`/`actual_freehub_overrun` tendon stand contracts; `run shaft_ratio` accepted | **landed** |
| `validation/rider_cases.py` (54) | pending_qualification | `validation/rider.cpp` | bounded `airborne_internal_actuation` articulated-flight contract | **landed** |
| `validation/rider_work.py` | pending_qualification | `validation/evidence.cpp` | bitwise `rider_work` verdict (active/passive/motor channels) | **landed** |
| `validation/system_momentum.py` (128) | pending_qualification | `validation/rider.cpp`, `rigs.cpp` | bounded `momentum` + `whole_system_{passive,active}`/`motor_only_airborne` contracts; `run momentum` accepted | **landed** |
| `validation/__init__.py` (1) | pending_qualification | `validation/acceptance.hpp`, `registry.cpp` | docstring-only file; registry inventory contract proves every public id resolves | **landed** |
| `validation/climbing_evidence.py` (111) | pending_qualification | `validation/evidence.cpp` | bitwise `support_evidence` verdict (the file's only function) | **landed** |
| `validation/experiment_acceptance.py` (44) | partial → complete | `validation/tire.cpp`, `registry.cpp` | bitwise `experiment_acceptance` verdict + `is_synthetic` `str()`/bool-gate contract | **landed** — the cited residual (non-string `source` coercion) is closed via `python_str()`; `MODEL_DISCREPANCY_BUDGET` is an unconsumed declaration |
| `validation/contact_acceptance.py` (82) | partial → complete | `validation/acceptance.cpp`, `rigs.cpp` | bitwise `contact_bounds` verdict under `python_float()` + bounded `contact_matrix` cell contract | **landed** — the cited metric-decoding residual is closed via `python_float()`; the matrix exposes all five source axes at identical defaults |
| `validation/drive_suspension_rig.py` (163) | partial → complete | `validation/drivetrain.cpp`, `writers/suspension.*` | `SuspensionWriter::shock_total_n` retained exactly (no split reconstruction); bounded 3×3 transmission×torque sweep + `drive_suspension`/`drive_suspension_rig` runs green | **landed** |
| `validation/datasets.py` | partial | `validation/tire.cpp`, `registry.cpp` | bitwise `holdout_metrics_multidim` verdict (C-order multidim flattening at the JSON boundary) | stays `planned` — the public `holdout_metrics(std::span)` API remains 1-D |
| `validation/rigs.py` | partial | `validation/rigs.cpp` | every declared stand bounded-green | stays `planned` — NumPy-vs-libm scalar/small-matrix lowering has no bitwise verdict by design |
| `validation/load_transfer.py` | partial | `validation/rigs.cpp` | `rigid_load_transfer` bounded contract green | stays `planned` — trig/small-matrix lowering unconfirmed |
| `validation/contact_manifold_rigs.py` | partial | `validation/rigs.cpp` | custom density/mu now reach stands + `flat_depth` (density contract test) | stays `planned` — libm/matrix lowering unverdicted |
| `validation/contact_resolution.py` | partial | `validation/experiments.cpp` | **parity of failure**: oracle and native both reject `flat_load(...,1024)` under the shared 512-station cap | stays `planned` — NNLS fits/series depend on the partial stand producer |
| `validation/environment.py` | partial | `validation/evidence.cpp` | native identity contract exercised | stays `planned` — intentional narrow identity; no PEP-440/marker evaluation |
| `validation/antiwheelie_bench.py` | partial | `validation/experiments.cpp` | `antiwheelie-bench` module invoked (ran ~40 min, terminated; mechanics pipeline live) | stays `planned` — subordinate partial rigs/benchmarks |
| `validation/antiwheelie_reports.py` | partial | `validation/acceptance.cpp` | bitwise `paired_summary`/`assess_run` verdicts; `python_truthy()` mirrors `or`-truthiness | stays `planned` — bool/numeric cross-type seed grouping still diverges (native rejects non-string/non-number seeds where Python merges `True`/`1.0`/`1`) |
| `validation/benchmarks.py` | partial | `validation/experiments.cpp` | `run benchmarks` invoked (ran ~55 min, terminated; suite live) | stays `planned` — `jobs` 2–32 explicitly rejected |
| `validation/climbing_benchmark.py` | partial | `validation/experiments.cpp`, `evidence.cpp` | bitwise `timing_metrics` verdict | stays `planned` — full PolicySession/publication pipeline |
| `validation/fidelity_sweep.py` | partial | `validation/experiments.cpp` | bitwise `interval_integral`, `resolution_variants`, `select_verified_resolution` verdicts | stays `planned` — `jobs>1` unavailable; finalized-interval row cadence differs |
| `validation/reference_release.py` | partial | `validation/acceptance.cpp`, `experiments.cpp` | bitwise `release_gates`, `select_verified_resolution`, `classify_outcome` verdicts | stays `planned` — hashed manifests, measured-holdout authentication, 18-scenario matrix |
| `validation/ride_cases.py` | partial | `validation/scenarios.cpp`, `rider.cpp` | `flat_static`, `requested_sag`, `terrain_resolution`, `drive_low_mu` green; `surface_mode="track"` reaches writer-backed stands | stays `planned` — `brake_hold` diverges in the **oracle identically** (see below); per-step vs finalized-interval observation unqualified |
| `validation/rider_replay.py` | partial | `validation/rider.cpp`, `experiments.cpp` | — | stays `planned` — missing upstream tracks; cadence |
| `validation/rider_factor_matrix.py` | partial | `validation/rider.cpp`, `experiments.cpp` | **parity of failure**: declared `rider_resume_*.toml` tracks absent in both trees | stays `planned` — replay cadence inherits rider_replay |

Ledger: `migration/python-removal.json` 172→183 `replacement_landed`.
The 15 still-planned rows keep `planned` with refreshed
`phase3_source_implementation.verification` notes recording what was
qualified this lane and what remains.

## Work performed

1. **RCP-16 recipe expansion** (`tools/migration/capture_reference.py`,
   `_rcp16_validation_rigs`, +344 lines): 16 new authored-config +
   oracle-verdict fixtures for the pure binary64 surfaces —
   `holdout_metrics_multidim`, `contact_bounds`, `experiment_acceptance`,
   `paired_summary`, `assess_run`, `rider_work`, `trace_integral`,
   `interval_integral`, `timing_metrics`, `support_evidence`,
   `resolution_variants`, `release_gates`, `select_verified_resolution`,
   `valid_prefix`, `plant_release`, `classify_outcome`. Captured against
   the pinned oracle worktree (`reference-oracle` @ `4667639c`,
   `reference/python-oracle`), 33 group entries / 64 files / 0 blockers;
   the 16 new entries were merged into `migration/oracle-manifest.json`
   (174 total fixtures). Transcendental surfaces (`flat_load`'s `np.sin`,
   `load_transfer` small-matrix/libm paths) deliberately received no
   bitwise fixture — they stay on declared bounds.
2. **`migration_T54` expansion** (`native/tests/migration/test_t54.cpp`):
   16 `verdict_fixture` lines + `airborne_internal_actuation` added to
   the bounded-stand sweep. PASS 15043 checks, exit 0.
3. **Python coercion parity** (`native/src/validation/common.hpp`,
   `acceptance.cpp`, `registry.cpp`, `tire.cpp`):
   `python_float()` (numbers/bools/parseable strings incl. `1_000.5`,
   `inf`, `nan`, exponent, whitespace — fail-closed elsewhere) now backs
   `check_bounds` and multidim `holdout_metrics` leaf coercion;
   `python_truthy()` mirrors `or`-truthiness for
   `operator_intervention`; `python_str()` approximates `str(source)` for
   scalar JSON in `is_synthetic`. JSON recursive shape/flattening covers
   `np.asarray` same-shaped multidimensional holdout inputs.
4. **`shock_total` exact retention** (`native/src/writers/suspension.*`,
   `validation/drivetrain.cpp`): `SuspensionWriter` stores the computed
   `shock_total_n` scalar and exposes `shock_total_n()`; the drive-
   suspension report consumes it instead of reconstructing from split
   damper/HBO component vectors (the reordering changed last bits vs the
   source scalar).
5. **`surface_mode` reachability** (`validation/scenarios.cpp`):
   writer-backed tire backends (`compliant_2d`,
   `distributed_2d_reference`, `pneumatic`) now select
   `surface_mode="track"` for the rig requests — required by the native
   capability gate. At the rolling/static slips these stands measure,
   `min(tire.mu, surface.mu)` binds at the tire cap, preserving the
   oracle's `configured`-mode semantics. Documented in-tree.
6. **Custom material plumbing** (`validation/rigs.*`): distributed
   stands accept optional `mu` and a `configuration::HingeDensity`
   (field-wise conversion with synthetic fallback), closing part of the
   `contact_manifold_rigs` custom-material gap.

## Upstream parity findings (not native defects)

- `brake_hold`: the Python oracle fails identically —
  `RuntimeError: physical equilibrium did not converge after 160000
  steps` on `rear_wheel_spin` (native residual 0.094492 vs oracle
  0.113611, same dof signature). The case is divergent upstream on this
  incline+brakes+heightfield fixture in both implementations.
- `contact_resolution` (module): oracle `python -m
  bike_sim.validation.contact_resolution` raises `ValueError: at most
  512 tread stations supported` at its own `flat_load(...,1024)` call —
  the native module errors identically.
- `rider_factor_matrix`/`rider_replay`: the declared
  `examples/research/rider_resume_{flat,incline}.toml` tracks do not
  exist anywhere in the tree; the oracle errors identically on the
  missing file.

## Verification commands and exit codes

- `uv run --frozen --group native python tools/migration/capture_reference.py
  --output tests/fixtures/migration --manifest /tmp/t54-manifest.json
  --groups validation_t54 --oracle-root
  .../reference-oracle` → `captured 33 fixture entries / 64 files /
  0 blockers`, exit 0.
- `cmake --build build/native-release --target migration_T54` → exit 0.
- `./build/native-release/migration_T54` → `PASS 15043 checks`, exit 0.
- `ctest --test-dir build/native-release -R
  "migration_T54|native_suspension|distributed_tire|native_tire|
  ride_backend|rider_contacts|native_rider" --output-on-failure` →
  13/13 passed, exit 0 (includes the 68.9 s `native_rider_allocation`).
- `cmake --build native/build` → 100%, exit 0.
- `bike-validate run` direct checks: `momentum`/`shaft_ratio`/
  `load_transfer`/`drive_suspension`/`support_evidence`/
  `resolution_variants`/`experiment_acceptance`/
  `select_verified_resolution`/`drive_suspension_rig` all
  `accepted:true`; diagnostic cases return `accepted:false` with
  evidence as designed.
- `bash tools/run_tests.sh native` (final gate, darwin-arm64):
  configure + `native/build` compile 100%; all six static-analysis
  targets clean (`check_frontends`, `check_tidy`, `check_analyzer` —
  alpha findings report-only, `check_cppcheck`, `check_odr`,
  `check_scripts`); check orchestrator `headers` (269 headers / 9,570
  TUs), `diagnostic-controls` (6/0), `context` (5 SDK controls) all
  passed; sanitizer-runtime preflight ok; **CTest 60/62** —
  `migration_T09` (`FAIL post-advance integration vector matches the
  oracle`) and `migration_T10` (`FAIL first transition matches the
  Python oracle`), the documented frozen-oracle baseline pair with
  verbatim signatures (see `migration/cpp-linux-completion-evidence.md`,
  `task-C327/C336t2b/FT1/C337338` reports); the profile exits at the
  ctest stage per the documented gate-abort. The skipped tail phases ran
  manually with the gate's own env:
  `bike-native-checks --kind contracts` → **12/12 passed**;
  `load_native()` → `native/build/bike_native.cpython-313-darwin.so`;
  the 3,086-item native/golden pytest tail (`-p native_test_reporter`,
  `test_native_*.py` + `test_golden_episode*.py`, gate env incl.
  `MallocScribble`/`PYTHONDEVMODE`) → **3,243 passed / 2 failed /
  1 skipped in 8,065 s**:

  - `test_native_numeric_completion.py::test_numpy_float32_shifting_
    metadata_retains_baseline_bits` (`rear_teeth` 14 vs pinned 18) —
    the ledger-documented stale pin (`migration/test-port-map.json`
    disposition `drop`; `migration/full-gate-2026-10-31.md` and
    `cpp-linux-completion-evidence.md` record it: oracle `shifting.py`
    was rewritten multi-cog by `b21c63e` while the native port keeps
    the previous rule). Not lane-authored.
  - `test_native_recording_retention.py::test_strict_flush_rejection_
    preserves_retained_evidence` (`DID NOT RAISE InvalidReferenceRun`)
    — **verified pre-existing at the merge base**: rerun on a stashed
    `native/` tree (base `d1d26d3f`, rebuilt `native/build`) fails with
    the identical signature. The node was `port`ed green on 2026-10-27
    (3.30 lane/e) when a live reference violation still fired; the
    strict monitor now accepts all 800 committed intervals, consistent
    with `6fe7ab22`'s `runtime/step.cpp` constraint-partition refactor
    landing after that measurement. Not lane-authored; flagged for
    whoever owns the runtime-monitor baseline.
  - The C336t2b-documented 41-failure oracle-drift set shrank to these
    2 — the runtime/research/replay drift families now pass under the
    merged base.

## Concerns

- `brake_hold`, `contact_resolution`, `rider_factor_matrix`/
  `rider_replay` fail **identically upstream** — parity of failure is
  recorded but these surfaces never produce evidence anywhere; they
  remain `planned` rows for their own independent gaps.
- `airborne_internal_actuation` added to the bounded sweep raised
  `migration_T54` wall time to ~17 min on this machine (2 s articulated
  horizon at dt=0.0005 plus controller init). The sweep preserves source
  horizons by convention; no shortened substitute exists.
- `benchmarks` and `antiwheelie-bench` module runs exceeded the
  session's bounded window and were terminated at ~55 min / ~40 min CPU
  to reduce gate contention — the suite/matrix is real sim work, not a
  hang (both had been consuming CPU steadily). Their rows stay `planned`
  regardless, so no qualification claim rests on them.
- Other worktrees (`pfc-lane-c`, `pfc-lane-h`, `pfc-lane-e`) ran native
  gates and pytest tails concurrently; all lanes proceeded under CPU
  contention, stretching wall times (the `headers` kind ran ~70 min
  standalone and ~75 min inside the pytest tail; the tail took 2h14m).
- `test_strict_flush_rejection_preserves_retained_evidence` regressed
  somewhere between the 10-30/10-31 gates and this base — verified
  red at `d1d26d3f` with my native changes stashed, so it is a
  merge-introduced baseline regression outside this lane's scope
  (see Verification). The port-map row still claims `port`+green.
- `environment.py` remains intentionally narrowed — native identity only.

## Deferred Linux status

All verdict fixtures are pure binary64 arithmetic and are expected
bitwise-stable on linux-aarch64, but this lane produced no Linux
measurement; `linux-aarch64 deferred to the Linux lane` is recorded in
every promoted row's verification note.

## Commit

`69b90ed5` on `lane/g2` — 47 files, +1296/−100.
