# Physics and research: T26–T37

This source delivery implements the native boundaries for the **Physics and
research** group against the supplied 2026-10-17 Repomix snapshot. It contains
code, build declarations, focused regression sources and migration evidence.
It has not been configured, compiled, linked or executed.

Implementation status is separate from acceptance, source deletion and
release status. The task cards remain open for executable qualification.

## Native boundaries

| Tasks | Boundary | Result |
|---|---|---|
| T26 | Existing writers/runtime, `contact/tabulated_radial.*`, `runtime/native_contacts.cpp` and `migration/kernel-reuse-map.json` | Map for 58 modules / 508 declarations; residual contact snapshots and tabulated radial material. |
| T27 | `drivetrain/elastic_chain.cpp`, `motor_clutch.cpp`, `transmission.*` | Motor topology, chain extension/Jacobian controls, unilateral constraint linearization and reaction helpers. |
| T28 | `rider/allocation.*`, `numerics/{slsqp,linear_feasibility,nnls,brent,density_fit}.*` | Constrained allocation, conservative linear feasibility, nonnegative fitting and bracketed roots. |
| T29 | `rider/{constraint_dynamics,mode_search,activation_allocation,response_allocation,legacy_controller}.*` | Remaining rider dynamics/controller path with state and diagnostic transfer. |
| T30 | `rider/support_dispatch.*`, existing support writer, `runtime/capabilities.*` | Alternative pedal/saddle/grip attachments and explicit physical capability dispatch. |
| T31 | `legacy/{ride,controls,legs,rider,tyre}.*`, recorder | Owned legacy ride, native setup, forces, reset, bounded run, tyre adapters and CSV schema. |
| T32 | `numerics/filter.*`, `legacy/metrics.*`, `runtime/research_metrics.*` | Butterworth/filtfilt processing, ride statistics, events and wheelie analysis. |
| T33 | `legacy/playground.*`, suspension controller, static equilibrium, telemetry | Headless playground/stand model, controls, sweep and measurements. |
| T34 | `research/environment.*`, configuration/program/random loaders, `initialization/research_startup.*` | Native research request, startup, sensor timing/noise and seeded reset. |
| T35 | `research/policy.*`, built-ins, plugin loader, `include/bike/policy_v1.h` | Policy selection and versioned C ABI with explicit ownership/error handling. |
| T36 | `research/policy_session.*`, rider behavior and lifecycle | Causal policy scheduling, rider-command ownership and terminal/error handling. |
| T37 | Research evidence, metadata, metrics, quality and preview CSV | Owned episode evidence, logical record IDs, assets, recipe, summaries and provenance. |

Main public entry points:

- `bike::legacy::make_ride` and `bike::legacy::RideSimulation`.
- `bike::legacy::make_playground` and `bike::legacy::Playground`.
- `bike::research::read_research_request`, `prepare_environment` and `make_environment`.
- `bike::research::load_policy` and `bike::research::PolicySession`.
- `bike::research::summarize`, `recording_batch` and `snapshot_research_assets`.
- `bike::rider::allocate` and the typed allocation/dynamics payloads.

`native/cmake/NativePhysicsResearch.cmake` extends both existing build
graphs, introduces `bike_legacy` and `bike_playground`, and adds research
sources to `bike_app`. `BIKE_BUILD_POLICY_EXAMPLES` declares the optional
native traction policy module. No build command was issued.

The small `bike-native-ride` startup executable now dispatches legacy
requests to `RideSimulation`. Its legacy options include drive, tyre, tyre
tier and assist mode. A zero-step request prepares the model without entering
the positive-step run-limit contract.

## Numerical contracts

The existing spindle controller and hot writers retain ownership of their
established paths. The additional non-spindle articulated controller uses:

`bike-dense-sqp-bfgs-v1;ftol=1e-12;maxiter=500;restarts=4`

This identity does not claim equivalence to historical SciPy SLSQP. The
nearest-target fast path and returned-point feasibility retain the source
threshold of `1e-7`. An optimizer success flag cannot make an infeasible
point acceptable. Linear pruning distinguishes proven infeasibility from an
unknown numerical outcome. Capability reporting and relevant setup/research
metadata expose the solver identity.

Density helpers use existing NumPy-compatible reductions; allocation
subproblems reuse the checked least-squares provider. NNLS, SQP convergence,
physical rider modes, filtering and full source parity still need captured
comparisons on the intended provider.

Native-reference tyre patches retain the solved world force and couple.
Slip uses the requested interval velocity. Current vertical support, bridged
normal load and controller grounded state remain distinct. Tabulated radial
material owns validated knots, slopes and integrated energy, rejects values
beyond its curve and cannot generate tensile contact. Its applicability
range flags results; it does not clip force.

## Research configuration and policy ownership

Research admits the source-supported physical effort modes:
`crank_effort` and `articulated_effort`. The historical file/CLI request has
defaults of `0.005 s` sensor period and `80` recording decimation. Typed
`SensorConfig` and `ExperimentConfig` retain their separate `0.001 s` / `20`
defaults. Explicit configuration overrides either set.

Asynchronous telemetry publication is owned by T42 in the next group.
Requests with `async_telemetry=true` are explicitly rejected before setup;
there is no silent synchronous fallback. The synchronous research route is
implemented here.

The policy ABI carries nine sensor channels, schema version, validity,
optional demand and a control presence mask. Plugins own opaque state; the
host owns input/output/error storage. The host retains the loaded module
through destruction and translates callback failures into policy errors.
Native plugins are trusted executable extensions. Known historical
Python-style policy aliases resolve to native built-ins.

## Verification and handoff

Regression targets `migration_T26` through `migration_T37` and
`migration_T30_native_contacts` are declared. T35 additionally declares real
shared-module failure fixtures. All these new tests are **unrun**. No
compiler, linker, simulator, solver, policy, project Python module, oracle
generator, sanitizer or performance test was invoked.

The text snapshot omits binary oracle inputs used by several test groups,
including legacy bootstrap and filtering. Original manifest/fixture names
are retained. These bytes must be recovered before those comparisons can
run. A referenced native safety specification is also absent from the input.

See [implementation records](../migration/physics-research-implementation.json),
[static review](../migration/physics-research-review.md),
[filesystem audit](../migration/physics-research-static-audit.json) and
[kernel reuse map](../migration/kernel-reuse-map.json).

Full recording/replay/viewer/CLI application work remains T38–T51. Linux,
packaging and delivery qualification remain T52–T60. All original Python
modules and immutable reference fixtures are retained. T61–T63 acceptance
and deletion gates are unchanged.
