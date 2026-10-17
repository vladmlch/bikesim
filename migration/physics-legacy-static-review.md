# T31–T33 implementation and static review

Date: 2026-10-17. Scope: the legacy ride, rider/measurement helpers and headless suspension playground in the **Physics and research** group.

## Evidence status

The user explicitly prohibited compilation and execution. No configure, build, simulator, test executable, Python module, oracle generator, formatter, linter, sanitizer or performance command was run for this work. Source and declarations were read and edited. The C++ regressions below are authored tests, not passing-test evidence.

The work therefore provides implementation and a static source mapping. It does **not** close bitwise parity, test, performance, sanitizer, runtime capability, release or Python-removal gates. The original Python files remain intact. Do not mark the corresponding removal rows `ready_to_delete` or their cards verified from this document.

The repository's referenced `docs/superpowers/specs/2026-10-06-native-safety.md` was not present in the supplied snapshot. The implementation follows the available `AGENTS.md`, `docs/agents/native-engineering.md`, common migration specification, interface contracts and existing native owner boundaries. The missing specification still needs to be recovered for the final gate review.

## Native entry points and actual wiring

`bike::legacy::make_ride(RideOptions)` and its request-path overload create a `RideSimulation` that owns its `Stepper`, configuration, model image, force/controller state and sensors. It uses the existing native MJCF generator, road raster builder, configuration projection and legacy equilibrium bridge. There is no interpreter bootstrap or Python object in this route.

The owner handles sphere and legacy pneumatic tyres, motor, pedal and pedelec drive, seated/rigid or articulated legs, cruise, rolling resistance, brakes, airborne pitch stabilization, crash detection, reset, bounded run limits and a callback-driven run. A tick assembles the historical assigning forces and performs one authoritative engine step. The native physical `PhysicalStep` continues to own the separate physical stepping route.

The explicit `RideRecorder` owns growing history. Its 65-column CSV schema follows `sim/ride/recorder.py`, including absent-pedalling NaNs and sphere-mode zero tyre columns. It samples rim events and wheelspin/lock timing on every callback before row decimation. `to_csv()` and the file exporter use the same serializer.

`summarize_ride` produces owned `RideSummary` values, versioned JSON and the source table layout. Its numerical filter uses a native Butterworth design, `lfilter_zi`, odd extension, and forward/backward direct-form filtering. The steady-state solve uses the same pinned ILP64 LAPACK entry point already used by `kinematics/curves.cpp`; it adds no new dependency. Equality to SciPy remains an unexecuted oracle gate.

`make_playground()` creates an owned `Playground` for `standard`, `stand` and `playground` model modes. It provides model switching, travel targets, reset, deterministic injected sweep time, queued controls, damper presets/clicks, air pressure/tokens, in-place rider/marker toggles and owned telemetry. No window, GUI thread, Python main-thread trampoline or rendering loop enters this target. The graphical application, its HUD/help rendering and window ownership remain T48, as required by the group boundary.

## Source-card mapping

| Card | Source | Native coverage / reused implementation |
|---|---|---|
| T31 / F093 | `physics/tyre.py` | `legacy/tyre.hpp/.cpp`: `TyreSpecs` aliases the common native material payload with its radius/pressure properties; `make_tyre_specs` restores source optional constructor defaults; stock tyres, tier records, pressure copying, live clamps, literature constants, stiffness and Crr targets, `TyreConfig` validation and pressure copies. Existing `config_validation.hpp` remains the material validation boundary. |
| T31 / F143 | `sim/ride/__init__.py` | Typed native headers replace runtime re-exports. Controls, tyre, force, termination and recorder APIs are directly linked. HUD/input/livery renderer imports are not introduced into headless libraries; graphical consumers belong to T48. There is no import-time registration needed by these force owners. |
| T31 / F165 | `sim/ride/forces.py` | Existing `writers/suspension.cpp/.hpp` and `model/suspension_setup.*` supply the spring, damper, bumper and stop equations. `RideSimulation::Impl::suspension_forces` calls the legacy component path and writes the fork/shock slide forces without a second leverage-ratio multiplier. |
| T31 / F198 | `sim/ride/resistance.py` | `legacy::rolling_torque` uses raw vertical support, the three-system-weight load ceiling, compiled sphere radius, Crr and the low-speed taper. `RideSimulation::step` invokes it only for the sphere route; pneumatic material losses stay in the tyre model. No rolling-history state is invented. |
| T31 / F217 | `sim/ride/tire_forces.py` | Reuse `tyre/profile.*`, `writers/tire.*`, `writers/tire_common.*`, `contact/*` and the physical runtime's clock/state/force ownership. T26/T27 integration retains the physical compliant path. The historical pneumatic `xfrc_applied` writer is implemented separately in `legacy/tyre.cpp`, avoiding a second physical force assembly. This reuse still needs the existing tire oracle and clock suites to be run. |
| T31 / F230 | `sim/ride/wheels.py` | `legacy::WheelSpin`, `resolve_wheel_spin`, `resolve_hinge`, `resolve_scalar_joint`, `validate_actuator_target`, `opposing_torque`; topology checks reuse `model_topology.hpp`. The ride/drivetrain use these helpers. The resistance rest branch returns positive zero and preserves source multiplication order. Existing `BrakeWriter` reuses its already validated wheel/actuator path. |
| T31 / F231 | `sim/ride_sim.py` | `legacy/ride.cpp` assembles real model/configuration/road/equilibrium/force/drive/contact/measurement owners. `make_ride`, `reset`, `step`, setters, limits/run, model-image, metadata and snapshots form the headless legacy route. Common app dispatch is registered by the integration owner. Full physical recording and imported physical initial states retain explicit mode gates. GUI-only callbacks remain with T48. |
| T32 / F161 | `sim/ride/energy.py` | `legacy/rider.cpp`: `mechanical_energy_terms` expands the mass matrix and uses the same BLAS association; potential energy retains the source flattened NumPy reduction. `EnergyLedger` preserves work/loss signs and validation. |
| T32 / F172 | `sim/ride/leg_drive.py` | `legacy/legs.cpp`: validated handles, initialization, unreachable IK fallback to measured pose, counter-rotating pedals, antiphase torque shares, virtual-work Jacobian, reference-velocity PD terms and bias compensation. The actual legacy step invokes it with the rider crank torque. |
| T32 / F173 | `sim/ride/leg_loop.py` | The public `legacy/legs.hpp` reuses `spindle::{LegLoopGeometry,leg_loop_geometry,leg_joint_q,leg_loop_jacobian,spindle_xz,planar_angle}`; the articulated legacy drive consumes the existing native `model::{solve_leg_qpos,leg_jacobian,crank_torque_share}`. No independently approximated leg geometry is added. |
| T32 / F176 | `sim/ride/metrics.py` | `legacy/metrics.hpp/.cpp`, `numerics/filter.hpp/.cpp`: run-up masking; travel/stop/percentile statistics; filtered acceleration; rider loads/lift events; bob-frequency least squares; obstacle effective drop; rim episodes; tyre losses; JSON and fixed-width table. Short and Nyquist filter branches precede filter construction, as in the source. Missing optional tyre columns default to source-sized zero arrays. |
| T32 / F229 | `sim/ride/wheelie.py` | Shared `runtime/research_metrics.hpp/.cpp`: truth validation, profile-contact clearance fallback, raw classification, configurable load/clearance/pitch thresholds, persistence and front-only exit hysteresis, custom onset context, reset, episode and aggregate metrics, and signed quasistatic load diagnostic. Existing research callers retain the one-argument tracker constructor and delivered/applied-torque overload. |
| T32 / F232 | `sim/telemetry.py` | `legacy/telemetry.cpp`: `SimModelHandles`, kinematic path selection, rear/fork/shock metrics, damper/air settings and labels, closure residual, rider mass/CoM and axle balance. `TelemetryProvider::compute` reads existing derived state and does not forward or step it. Compiled with `bike_playground` and exercised by the T33 controller test. |
| T33 / F114 | `sim/controllers.py` | `legacy/suspension_controller.cpp`: nonlinear fork air force plus damper and source finite gate; stand shock spring plus damper. Existing native air-spring and damper calculators remain the numerical implementations. |
| T33 / F119 | `sim/playground.py` | `legacy/playground.cpp`, `legacy/static_equilibrium.cpp`: owner/lifecycle, all three model modes, cached frame mass properties, in-place rider/debug geometry changes, queued controls, targets, sweep and telemetry. `solve_static_equilibrium` delegates to the shared existing initializer. Python process relaunch and graphical event-loop functions are superseded by the native application boundary and scheduled renderer task T48. |

## Cross-card dependencies completed in this owner

| Source responsibility | API and consumer |
|---|---|
| `physics/drivetrain.py`, legacy branch of `sim/ride/drivetrain.py` | `DrivetrainSpecs`, ripple/ceiling/cutoff/lag helpers and `PedalDrivetrain`; source gearing, cassette shifting/cooldown/cut, assist, equality re-datum, coasting/freewheel, brake demand and telemetry. `RideSimulation` owns the controller for pedal/pedelec. |
| `sim/ride/contacts.py` | `TerrainContactQuery` wraps the shared `RuntimeContactQuery` for sphere contact patches, wrenches, slip, raw support, dropout bridging and grounded state. Its pneumatic adapter builds source snapshots from actual legacy material patches. It does not advance physical tyre state. |
| `sim/ride/tyre/applier.py`, `sim/ride/tyre/geometry.py` | `TyreForceApplier` and `build_road_profile`; existing `PneumaticTyreModel`/`RoadProfile` supply fast/detailed equations. `first_crossing_reference` supplies the slow reference ray query. The applier stages both wheel wrenches, restores material states on failure, uses body-CoM moment arms and commits only finite results. |
| `sim/ride/rider_forces.py` | `RiderForces` reuses validated native rider paths and updates the source's mutable pedal offsets; unilateral saddle separation, spring/damper forces, load telemetry and stored energy remain accessible. |
| `sim/ride/virtual_rider.py`, `sim/ride/termination.py` | `pitch_step`, `detect_crash`, `RunLimits`, `termination_reason`, `RunOutcome`; source priorities, clipping and integral/work channels. |
| `sim/ride/recorder.py`, `sim/ride/rim_events.py` | `RideRecorder`, rim-history capture and `count_completed_impact_episodes`; events remain observable across CSV decimation and the run-up boundary. |
| `validation/system_momentum.py` | `system_momentum`: native body Jacobians, mass-weighted CoM and linear/angular momentum with full rotated inertia tensor. This diagnostic refreshes the explicitly supplied `mjData`, matching its source; it is not called by read-only ride telemetry. |

## Static review details

* Native includes were inspected. An initial accidental include of the historical nanobind `config.hpp` was removed; legacy creation uses `writer_config.hpp` and `runtime/project_config.hpp`.
* Model/data lifetimes precede all borrowed controller/writer lifetimes. Destruction order releases controllers before the owning Stepper. `Playground::State` owns independent model/data, and failed model selection does not replace the live state.
* Resolved scalar indices use explicit size conversions for span access. Source hinge/axis/actuator topology gates now run before legacy wheel/drivetrain use. Wheel radius comes from the compiled contact sphere.
* Engine calls added for pneumatic body velocity, mass-matrix expansion and momentum Jacobians use the existing engine error trampoline. Playground engine failure poisons the owning state; it can be replaced by a new model owner.
* Pneumatic force commits are transactional across the two wheels. Invalid brake input is checked before a ride tick. ForceAccumulator validates both total and named-component overflow before mutation.
* Source-specific reduction choices are explicit: Python compensated sum for body/support reductions where the source uses builtin `sum`, NumPy pairwise sum for metric/gravity reductions, CBLAS for Jacobian/momentum/energy products, the native `np_interp` scalar lowering for rear-travel interpolation, and LAPACK for the filter initial state.
* The rear-travel table retains the source 0.1 mm spacing rule and linear endpoint extrapolation. Interior interpolation uses the existing NumPy-compatible scalar kernel.
* The common `research_quality` finite/positive threshold validation supplied by the research owner was preserved while extending the wheelie API.

These observations are static evidence only. They do not establish absence of compiler diagnostics or runtime faults.

## Independent review of T26/T27/T30 integration

The legacy owner also read the integrator's `runtime/native_contacts.cpp`, the contact branches in `runtime/control.*` and `runtime/step.*`, the tabulated material/configuration projection, drivetrain helper additions, and `native/cmake/NativePhysicsResearch.cmake` against their source responsibilities.

The native-reference path retains MuJoCo's geom2 wrench sign, transposed contact-frame rotation for force and couple, raw vertical normal support, planar normal projection, incoming-velocity Jacobian slip and separate terrain-only controller grounding. `PhysicalStep::refresh_contacts` matches the source's distinct query and physical filter owners. A native-reference run does not enter the applied tyre force writer or duplicate its resistance path.

The reviewer identified and, with the integration owner's agreement, fixed three contact-boundary gaps:

* `mj_contactForce` and `mj_jac` in the new query now use `engine::invoke` directly. A caller outside an enclosing Stepper mutation still receives the native engine failure boundary.
* Both aggregate slip channels are validated after weighted reduction and before bridge/filter state is committed. Finite patch fields alone do not establish that their weighted sum is finite.
* `handlebar_load` validates the construction-model identity, nonnull data, contact row count and finite accumulated load, and protects its engine call. Its header records the same model/data lifetime contract as the full query. Two rejection assertions were added to the authored `test_native_contacts.cpp`.

The material review found the source `is_load_in_valid_range` public helper missing. This was reported to the integration owner, who owns its implementation and associated source row. Table monotonicity, prefix integrals, non-extrapolation, no-tension damping, declared load/pressure metadata and production `TireWriter` dispatch were inspected and no further static mismatch was identified in those reviewed expressions.

Both build declarations register the added elastic-chain and motor-clutch translation units; the shared physics module registers the tabulated material and native contact query once per core target. `bike_legacy` and `bike_playground` lists match the implementation files. The source namespace review found the legacy least-squares uses already explicitly qualified as `::rider`, avoiding the new `bike::rider` namespace shadow.

This independent review did not configure a graph, link a target, execute the added contact test or establish a numerical/performance pass.

## Authored regressions

| Source file | Assertions intended for a later run |
|---|---|
| `native/tests/migration/test_t31.cpp` | Source CSV ordering/precision/NaN/signed zero and decimation; tyre defaults/unit properties; pressure/tier branches; reference ray crossing; both material tiers; legacy startup/reset determinism and immutable observation; all pedalled drive modes; mode gates; exact original bootstrap integration-state and first-step fixtures. |
| `native/tests/migration/test_t32.cpp` | Short-series bit copy; constant and impulse filtering; Nyquist fallback; rising-edge events, stop and acceleration signs; run-up window; bob amplitude; rim-event merging/invalid overlap; source torque/cutoff/leg-share equations; zero-speed resistance sign; pitch/crash/termination/energy boundaries; configurable wheelie hysteresis and reset; exact original filter fixtures. |
| `native/tests/migration/test_t33.cpp` | Stand spring/damper directions; headless startup/linkage closure; targets; rider mass toggle preserving qpos/qvel; click/preset controls; injected-time sweep; queued reset; generation; rejection preserving current model; all three model modes. |

The assertions supplement, rather than replace, the existing per-writer contract tests and source-specific Python parity fixtures. They have not been executed.

## Fixture and acceptance gaps

The supplied snapshot contains `tests/fixtures/migration/metrics/filter.json` and `tests/fixtures/migration/bootstrap/legacy_flat/meta.json`, but the referenced binary64 input/expected files are not present. The tests deliberately retain the original manifest and `golden_f64` paths. No replacement expected values were generated from the new implementation, and no mismatch is hidden behind an approximate comparison.

Still required after execution is authorized:

1. Restore the original oracle binaries and check manifest identity for `metrics/filter` and `bootstrap/legacy_flat`.
2. Compile the targets on the pinned native toolchain and resolve diagnostics; then run T31–T33 and the existing writer/model/clock contracts.
3. Run the bitwise filter and legacy trace comparisons. In particular, complex Butterworth coefficient construction, filter recurrence, equilibrium reset, source table formatting and long-run controller history need numerical evidence.
4. Acquire/compare the T33 stand telemetry fixture and full source-card edge/error cases; present synthetic controller tests alone do not close those declarations.
5. Check fast/detailed pneumatic traces, pressure changes, nonzero crank phase, auto-shift/coast transitions, crash/flight, rider separation and high-load rim events against the authoritative source fixtures.
6. Run sanitizer, ownership, repeated-reset, app dispatch and performance gates on the built native targets. Linux provider portability remains in the delivery group, not proven by the existing pinned Accelerate declarations.
7. Complete independent review and update the common source-removal/test/capability ledgers only with the actual resulting evidence.

## Build edges and deferred commands

The integration owner's `native/cmake/NativePhysicsResearch.cmake` registers `legacy/{controls,force_accumulator,legs,rider,tyre,ride,recorder,metrics}.cpp` and `numerics/filter.cpp` in `bike_legacy`, linked to `bike_initialization`. It registers `legacy/{playground,suspension_controller,static_equilibrium,telemetry}.cpp` in `bike_playground`, linked to `bike_legacy`. These edges add no GUI or Python dependency.

For a future authorized session only:

```bash
cmake --build build/native-release --target migration_T31 migration_T32 migration_T33
ctest --test-dir build/native-release -R '^migration_T3[123]$' --no-tests=error --output-on-failure
```

No command in this section was executed as part of the implementation.
