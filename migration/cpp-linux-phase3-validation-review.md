# Phase 3 validation source review — Tasks 3.37–3.40

Date: 2026-10-21. Scope: the extracted `phase3-project` snapshot.

## Evidence status

This change adds native implementations and authored regressions. It does **not**
close Tasks 3.37–3.40. The user explicitly prohibited compilation and execution.
No compiler, linker, CMake configure, tests, oracle, benchmark, application,
container, project script, or syntax-only compile was run for this work.
No new numerical fixture, measured timing, passed physical gate, empirical
calibration, or platform qualification is claimed.

The count in the completion-plan heading is inconsistent with its own file list.
The supplied source contains **28 files in `src/bike_sim/validation/`**, including
the package marker, and **two owned files in `src/bike_sim/physics/`**. Plan 08
T54 explicitly says 28 validation modules. The table below accounts for all
30 actual files. None was deleted.

`full` below means all public operations have source implementations or an
identified existing native producer, with no known omitted operation in that
module. It is **not** a claim of compilation, bitwise equivalence, a landed
qualified replacement, or passing tests. `partial` names a concrete remaining
operation, input-domain difference, orchestration difference, or numerical
issue. There are **12 full source mappings and 18 partial source mappings**.
No module is classified `missing`; several are still partial. The removal
ledger must remain open until the required captures and checks are actually
performed.

## Per-module mapping

All paths in the native column are relative to `native/src/`. Python paths with
no prefix below are relative to `src/bike_sim/validation/`.

| Python module | Source implementation | Source status | Public operations and remaining limit |
|---|---|---|---|
| `../physics/tire_calibration.py:1–84` | `validation/tire.{hpp,cpp}` | full | `TireCalibration`, record/file loading, geometry/provenance, explicit curve and inclusive load range. No implicit pressure scaling. |
| `../physics/tire_curve.py:1–92` | existing `contact/tabulated_radial.{hpp,cpp}`, `validation/tire.cpp`, `registry.cpp` | full | Monotone table, interval force integral, unilateral damping, no extrapolation and range errors. CLI `tire_curve` retains energy in joules. |
| `__init__.py:1` | `validation/acceptance.hpp`, `registry.cpp` | full | Package organization becomes an explicit native library and registry; no Python module discovery. |
| `antiwheelie_bench.py:1–133` | `validation/benchmark.cpp` | partial | Version/profile identity, evidence matching, bounded results, 14 load/shaft stands and five-operation mechanics pipeline are implemented. Native provenance replaces Python package versions; subordinate partial rigs/benchmarks prevent a complete source-equivalence claim. |
| `antiwheelie_reports.py:1–70` | `validation/acceptance.cpp` | partial | `assess_run`, tuple grouping, numeric integer-seed ordering, candidate/baseline exclusions and horizon checks. Mixed unusual seed types and Python truthiness of non-boolean intervention values are narrower at the native typed boundary. |
| `benchmarks.py:1–240` | `validation/benchmark.cpp`, `registry.cpp` | partial | All 41 registry entries, original bounds, convergence keys/floors, mandatory-vs-diagnostic selection, complete-suite and exact-dt gates. `jobs=1` executes sequentially; source isolated worker execution for jobs 2–32 is explicitly rejected. Native versions are declared as native versions. |
| `climbing_benchmark.py:1–166` | `validation/benchmark.cpp`, `evidence.cpp` | partial | Accounted physical steps and real research `PolicySession` ticks, setup/warmup/latency, output-directory exclusivity, validity, hashes and recording publication. Native research capture uses the existing native recording contract; equivalence of setup/retention costs and sampled diagnostics requires captures. Source `preview_mode()` is already removed and raises `preview path removed`; the port preserves that rejection. |
| `climbing_evidence.py:1–111` | `validation/evidence.cpp` | full | Ordered/contiguous support intervals, observed-vs-unknown support, model/energy invalidity, first bad/unknown time, reason counts, valid prefix and separate completeness. Unknown observations are never invented as zero-valued evidence. |
| `contact_acceptance.py:1–82` | `validation/acceptance.cpp`, `rigs.cpp` | partial | Bounds and flat/step/incline matrix over actual distributed stands. Native metric decoding currently expects numeric JSON values, while Python `float()` also accepts some strings/bools. Stand station limits and custom-material surface remain narrower; empty CLI bounds do not constitute accepted evidence. |
| `contact_manifold_rigs.py:1–173` | `validation/rigs.cpp` | partial | Two-body wheel stand, kinetic/material ledger, flat/step/incline rigs, six road profiles and retained trajectories. The private native stand currently uses the declared default density/mu; the Python helper's arbitrary optional density/mu inputs are not exposed. Some NumPy scalar ufunc/matrix lowering still needs a dedicated bitwise comparison. |
| `contact_resolution.py:1–95` | `validation/experiments.cpp`, `evidence.cpp` | partial | Both source NNLS density fits, 12 flat and three incline stands, all six roads, time/road/station axes, persistent events and finest-pair acceptance. Numerical qualification depends on the partial contact stand producer; report/provenance serialization is native. |
| `datasets.py:1–237` | `validation/tire.cpp` | partial | Seven unit schemas, whole-experiment splits, measured conditions, sensor/estimate provenance, per-experiment errors, declared parameter/error budgets and canonical ASCII hashing. The public C++ holdout API is a one-dimensional span; Python accepts same-shaped multidimensional NumPy inputs. Existing native JSON byte/number limits also bound metadata. |
| `drive_suspension_rig.py:1–163` | `validation/drivetrain.cpp` | partial | Actual compiled suspension fixture, four-joint bounded closure continuation, 784.8 N frame load, viscous rollers, three transmissions, all stroke/torque/dt combinations and original force gates. `shock_total` is presently reconstructed from split damper/HBO components: subtracting then adding HBO can change the last bits. The discarded source scalar must be exposed or recomputed exactly before bitwise qualification. |
| `environment.py:1–82` | `validation/evidence.cpp` | partial | Ordered length-delimited source hashing and actual native build lock/MuJoCo/provider identity. This intentionally reports a native identity; Python/PEP 440 marker evaluation and every direct package version are not reproduced. Current lock verification covers the build-lock digest and loaded MuJoCo, not every transitive image. |
| `experiment_acceptance.py:1–44` | `validation/tire.cpp`, `registry.cpp` | partial | Split rejection, synthetic declaration and explicit boolean calibration gates. Source `is_synthetic` applies Python `str()` to an arbitrary `source` object; the C++ record boundary requires a string. The discrepancy-budget constants remain in the Python authority and are not automatically applied or invented as new thresholds. |
| `fidelity_sweep.py:1–409` | `validation/evidence.cpp`, `scenarios.cpp`, `experiments.cpp` | partial | Resolution validation, interval integration, all comparison criteria, seven cases, physical seeds, periodic station projection, rows/events/metrics and finest-pair summaries. Jobs 2–16 are explicitly unavailable. Runtime rows consume each immutable finalized interval once; this differs from the source's repeated `sim.physical.sample` read after each physics step. Initial state files preserve the exact payload bytes used for their hashes. |
| `grade_feasibility.py:1–20` | `validation/rigs.cpp`, `registry.cpp` | full | Explicit scalar domains and `mu * rear_fraction - grade`; result is labelled necessary and never sufficient for a completed climb. |
| `load_transfer.py:1–71` | `validation/rigs.cpp` | partial | Quasistatic load formula and full free-pitch rigid stand with normal/tangent reaction and moment checks. Source NumPy scalar trig lowering and small matrix products need confirmation against platform captures. |
| `plant_prefix.py:1–43` | `validation/evidence.cpp`, `registry.cpp` | full | `ValidPrefix::observe`: seen/accepted counters, first invalid record, strict evidence booleans, positive interval, loads and impulse prefix. A later valid row cannot resume acceptance. |
| `plant_release.py:1–28` | `validation/acceptance.cpp`, `evidence.cpp` | full | All eight independent SIL gates, optional completion, no calibration/safety promotion and ordered trapezoidal trace integral. |
| `plant_torque_rig.py:1–40` | `validation/rigs.cpp` | full | Actual tendon-based shaft/freehub stand, arbitrary positive ratio, overrun, constraint reaction, angular acceleration and shaft-power identity. Uses actual MuJoCo actuation/constraint results. |
| `reference_release.py:1–289` | `validation/acceptance.cpp`, `experiments.cpp` | partial | Required release gates, candidate choice, hashed evidence manifests, measured-holdout authentication, outcome ordering and all 18 scenarios. Matrix execution inherits the immutable-interval and native-seed differences; no reference release was created. |
| `ride_cases.py:1–239` | `validation/scenarios.cpp`, `runtime_access.cpp` | partial | Flat/native static, exact requested-sag fitting bounds, brake hold, low mu, airborne passive/released rider, road runs and terrain refinement. Road/refinement fork travel comes from each committed endpoint, preserving the source's `fork_travel_mm / 1000` operation. All source horizons/thresholds and nominal-dt integral arithmetic remain. Each published row represents one physics step; processing all rows after publication differs from the source's per-step access to its latest published sample. |
| `rider_cases.py:1–54` | `validation/rider.cpp` | full | Compiled airborne articulated tree, actual rider/internal forces, ballistic/momentum/work invariants and original two-second horizon. |
| `rider_factor_matrix.py:1–40` | `validation/scenarios.cpp` | partial | Nine original factors, ten-second horizon, baseline-resumed and causal-replay-resolved gates. Inherits replay observation differences; unsupported motor-only replay is retained as an explicit failed experiment. |
| `rider_replay.py:1–295` | `validation/scenarios.cpp`, `evidence.cpp` | partial | Schedules, profile loading, diagnostic rows, replay, coast/resume diagnosis and JSON/gzip reports. All seven modes remain; motor-only open loop rejects. Native report writing uses actual gzip stored blocks and atomic native publication. Row cadence is the distinct finalized interval stream rather than repeated/stale `sample` access. |
| `rider_work.py:1–10` | `validation/evidence.cpp` | full | Exact component-name selection, CPython-style active-work sum, separate passive, net and motor work. |
| `rigs.py:1–194` | `validation/rigs.cpp`, `drivetrain.cpp` | partial | Every public stand and compiled chain derivative is implemented against actual compiled models/production laws. Small-matrix and NumPy-vs-libm scalar lowering still needs direct comparison; a manually reconstructed small rotation is not asserted bitwise equal. |
| `system_momentum.py:1–128` | `validation/rider.cpp` | full | Independent body inertial Jacobians, world inertia, CoM/linear/angular momentum, actual complete airborne tree, passive/active/motor-only cases, gravity impulse, free-pitch conservation and separate roll/yaw constraint evidence. Task 3.19 reference tests reuse this API. |
| `tire_characterization.py:1–42` | `validation/tire.cpp`, `registry.cpp` | full | Pure brush-step loop, exact slip convention, Stribeck cap, loss/storage/stiffness metadata and integer horizon. Default-material provenance is explicit through a supplied tire record or physics profile. |

## Native integration

`bike_validation` contains exactly these translation units:

```text
native/src/validation/acceptance.cpp
native/src/validation/evidence.cpp
native/src/validation/tire.cpp
native/src/validation/rigs.cpp
native/src/validation/drivetrain.cpp
native/src/validation/rider.cpp
native/src/validation/runtime_access.cpp
native/src/validation/scenarios.cpp
native/src/validation/experiments.cpp
native/src/validation/benchmark.cpp
native/src/validation/registry.cpp
```

It links the existing `bike_cli` graph, because the rigs use its setup, physical
runtime, research policies, recordings and model catalogue. The native source
include directory is public. The coordinator owns CMake integration in
`native/cmake/NativeValidation.cmake`. CLI entry point:
`native/src/tools/validate_main.cpp`.

Only two existing engine headers were extended: `runtime/runtime.hpp` and
`runtime/step.hpp` name the validation access friend. This was coordinated with
the golden-capture owner. The capture observation flow was not changed.
`RuntimeRigAccess` is an exclusively guarded, cold fixture surface. It uses
actual owned model/stepper/physical objects; airborne rebasing and physical seed
capture require step/time zero. A released-rider external impulse fixture
accepts only the initial interval. Ordinary runtime input checks remain in
place. This new seam's rollback/reset/poison integration is **source-reviewed
only** and requires dedicated runtime checks before production use.

### CLI

`bike-validate list` and `discover` enumerate explicit C++ records.
`bike-validate run CASE --config FILE` handles JSON/TOML operation inputs. The
module form accepts the original Python main/argparse names for rider replay,
factors, drive comparison, contact resolution, fidelity, reference matrix,
climbing and antiwheelie mechanics. General pure-data operations use `--config`;
paths resolve under `--root`. `--report` writes the complete envelope.

`physical_validity`, `numerical_validity`, `numerical_parity`, and
`runtime_performance` are separate envelope members. Missing evidence stays
null. An aggregate case acceptance is not copied into those fields. In
particular numerical parity stays unknown because a physical validation run is
not an oracle/golden comparison. Diagnostic completion and passed bounds have
distinct exit handling; reference-matrix numerical/model failures retain exit
2. `validation/cli_options.hpp` restricts recognized options to the exact module
that consumes them. Shared native input/root/report/output options are explicit;
pure-data inputs otherwise come from the case configuration. `--dt` and
`--timestep` address the same field, and supplying both is rejected as a
duplicate before file loading or case execution. All four independent report
axes are also present in error envelopes. Non-boolean values cannot populate
the performance axis.

## Authored regression and capture requirements

`native/tests/migration/test_t54.cpp` adds:

- The existing `validation/tire` fixture comparison with exact binary64 bits,
  shape and field inventory. `capture_reference.py:2572–2576` loads an additional
  `config/viewer_physics_welded.toml` fixture; the test supplies that dependency
  explicitly. The short grid TOML omits tire friction/stiffness/length and cannot
  establish them by itself. No guessed material defaults were inserted.
- Curve integration/unilateral contact, range rejection, synthetic holdout
  rejection/preservation, whole-experiment leakage, missing units/uncertainty,
  invalid dates, and Unicode canonical dataset hashing.
- Invalid-prefix persistence, empty/unknown support, missing release gates,
  non-boolean truthiness, empty ad-hoc bounds, unsupported control schedules,
  identity absence, interval gaps, load/work/timing units, integer station
  counts, numeric pair ordering and explicit registry inventory.
- The actual frontend's module-specific option grammar, unknown/irrelevant
  options, repeated options, both timestep-alias orders and all original module
  argument sets. These checks use the shared parser guard without launching
  the CLI or a subprocess. Boolean report axes reject numeric/string truthiness.
- Publication coverage independent of batch sizes: duplicate/skipped interval
  IDs, missing trailing samples, discontinuous times, wrong final boundaries,
  and an explicitly nonzero starting boundary.
- Real original-horizon wheel inertia, compiled chain Jacobian and actual
  shaft/freehub stands. These are authored expectations, not measured results.

The coordinator registered `migration_T54` in root `CMakeLists.txt:607–616`
under `BUILD_TESTING`, linked `bike_migration_test` and `bike_validation`, applied
the existing startup policy/MuJoCo load-name helper, and added the `bike-validate`
dependency. It takes no arguments and carries labels
`migration;t54;validation;slow`. `reference.system_momentum_drive` uses the same
validation momentum producer through `native/tests/reference/Phase3Ports.cmake`.
These statements describe the reviewed CMake source, not a successful configure
or build.

No test or report here supplies the missing two-platform captures required by
ADR-0003 and completion-plan method step 2. The existing migration fixture is
historical evidence with its own manifest provenance; it does not replace the
new per-platform Phase 3 golden inventory. `replacement_landed`, L1, L2, L3,
release readiness and calibration flags were not updated by this work.

## Source-review findings and remaining risks

Source review caught and corrected native enum/member mismatches, incompatible
`auto` multi-declarations, native research construction, exact requested-sag
lower/upper bounds, numeric seed sorting, typed station counts, tire-curve energy
units, fixture input dependency, HingeDensity report fields, and hash-preserving
physical seed output, four nested JSON initializer scopes, module-specific CLI options, duplicate aliases,
explicitly typed report axes and endpoint fork-travel sampling in road/refinement
cases. Public reads use the existing bounded no-follow regular
file reader; file hashes stream through the established platform helper.

The principal unresolved behavior is observer cadence. The supplied Python
validation drivers frequently read `sim.physical.sample` immediately after
every physics step; the current runtime publishes samples at accounting-period
boundaries. Reusing a stale sample, or dereferencing no first sample, is a
different observable behavior from draining completed immutable intervals once.
The native code chooses the interval stream and retains full duration/error
evidence. It cannot be described as a literal numerical port of those drivers
until the source intent, captures and regression expectations are reconciled.
This publication delay does **not** make one native row a whole accounting
period. `runtime/step.cpp:1989–1991` preserves each raw physics step's ID and
start/end times; `runtime/accounting.cpp:705–719` creates an individual published
sample from every raw. `Simulation` requests full retention and measurement
decimation 1, so draining a period visits all of its physics intervals once.
The ride-case accumulators retain the Python nominal-`dt` arithmetic. Replacing
that multiplier by `end_time_s - time_s` would change rounding and would not be
a justified literal source port. The remaining gap is which observations the
Python driver consumes, not a demonstrated accounting-period scaling error.

A separate, confirmed endpoint mismatch was corrected without changing the
physical trajectory. `ride_cases.py:176,210` reads live `sim.fork_travel_mm`
after each step; its property reads current qpos and multiplies by 1000
(`sim/ride_sim.py:751–753`). The sample's suspension channel instead contains
the interval's incoming qpos (`runtime/step.cpp:1880–1884`). The private replay
helper now has an optional boundary observer, called after a successfully
advanced step and before draining published samples. Road and terrain-refinement
cases use that observer to read `render_frame().presentation.fork_travel_mm / 1000`
once per endpoint, then retain the original mean reduction. The presentation
cache is refreshed from current qpos on every physics step
(`runtime/presentation.cpp:118–134`, `runtime/runtime.cpp:653–661`), independently
of the accounting close. Load sampling, nominal-dt multipliers, period closes,
and the remaining general observation-cadence differences are unchanged.

The replay collector now also checks coverage without changing sample cadence.
`IntervalCoverage` requires consecutive interval IDs from the starting step,
uses the existing `PeriodBuffer` continuity tolerance, and checks the final
drained ID/time against the runtime's final committed boundary after `flush()`.
A period may still publish several rows together; no sample is required after
each individual advance. Missing/duplicated intervals cannot produce accepted
fixed-duration evidence. Fixed-duration ride cases additionally reject a
shortened horizon or an outcome before returning their metrics and bounds.
The diagnostic `run_replay` path explicitly retains prefix reporting and its
separate `completed_requested_duration` evidence.

The `shock_total` scalar recovery, omitted process-worker path, custom contact
material helper parameters, accepted Python coercion edge cases, multidimensional
holdout input, full native dependency-identity closure, and small-matrix/ufunc
rounding are explicit remaining work. None is
covered by invented tolerances, accepted old failures, substituted fixture
bytes or shortened unstable trajectories.

Compilation, strict warning sweeps, runtime invariant checks, two-platform
goldens, conservation tests, measured convergence, timing runs and release
qualification remain entirely unperformed under the user's execution ban.

The coordinator's final file-level delimiter inspection also corrected three
remaining nested `Value` initializers: factor-matrix summary, resolution-variant
return value and contact-matrix result row. Together with the earlier initializer
fixes, these are source text repairs. No compiler was invoked to validate them.
