# T28–T30: rider allocation, controller and support source delivery

Date: 2026-10-17. Scope owner: rider/solver implementation in the physics and research group. Integration owner maintains the shared CMake graph, runtime assembly and migration ledgers.

## Evidence status

The user explicitly prohibited compilation and execution. This work used source reading and editing only. No CMake configure, compiler, syntax-only compiler, unit test, oracle program, Python import, simulation, benchmark, plugin loading or project validation script was run. The authored tests below are pending; their presence is not passing evidence. T28, T29 and T30 cannot be accepted or used to authorize Python deletion on this report.

The source snapshot referenced `docs/superpowers/specs/2026-10-06-native-safety.md`, which was not included. The available `AGENTS.md`, `docs/agents/native-engineering.md`, common migration specification, interface contracts, remaining-physics plan and per-source cards were used. No existing Python implementation was deleted or replaced.

## Source mapping

| Source and declarations | Native implementation | Static integration evidence |
|---|---|---|
| F079 `physics/rider_allocation.py`: records, residual, inverse-dynamics rows, allocation, old-style constraints, grip branches | `native/src/rider/allocation.hpp/.cpp`, `allocation_json.cpp`, `native/src/numerics/slsqp.hpp/.cpp` | Owned matrices/callback captures; projected exact target; finite/shape checks including direct residual calls; equality/inequality/extra constraints; four warm restarts; physical residual determines feasible. Cold JSON stays outside core. |
| F199 `sim/ride/rider_activation_allocation.py`: latent/delivered activation, derivative, constraint, projection | `native/src/rider/activation_allocation.hpp/.cpp` | Existing activation and positive-power kernels; separate excitation, latent activation and delivered torque; nonlinear equality rechecked after projection. |
| F200 `sim/ride/rider_balance.py`: frame-relative internal balance request | `native/src/rider/legacy_controller.cpp` | Frame/pelvis pose and point velocity determine a bounded request; only internal hinge actuators realize it. No running root-coordinate prescription. |
| F202 `sim/ride/rider_control.py`: non-spindle setup, foot/arm targets, pressure requests, coasting, finite differences, coupled/uncoupled allocation, final effort, state | `native/src/rider/legacy_controller.hpp/.cpp`, `legacy_allocation.cpp`, `force_requests.hpp`; facade/state changes in `spindle_controller.hpp/.cpp`, `state_codec.hpp/.cpp`, `spindle_wire.cpp` | `SpindleController` delegates by pedal attachment; the original spindle computation remains. Full typed articulated configuration is decoded on both native and optional binding boundaries. |
| F203 `sim/ride/rider_dynamics.py`: model snapshot, bounded dual response, attachment force map, region traversal | `native/src/rider/constraint_dynamics.hpp/.cpp`, `mode_search.cpp` | Framed engine calls read mass/Jacobian/declared impedance; full coupled response does not read solved constraint force or prior acceleration. Every affine result carries its domain. Queue deduplication and explicit completion flag retained. |
| F207 `sim/ride/rider_response_allocation.py`: linear prefilter, crank rows, support/grip constraints, coupled response | `native/src/rider/response_allocation.hpp/.cpp`, `response_codec.hpp/.cpp`, `native/src/numerics/linear_feasibility.hpp/.cpp` | Source force/moment/grip guards, per-joint bounds, power epigraph, crank task coordinate, remembered branch and projected activation. Failed support allocation stays `invalid_controller`; incomplete search is exposed. |
| F154 `sim/ride/control.py`: one-interval physical command | Existing `native/src/runtime/control.*` plus integrator runtime dispatch changes | Drive/rider validation remains at the actual physical boundary. |
| F209 `sim/ride/rider_support.py`: gravity and pedaling support targets | `native/src/rider/support_dispatch.hpp`, `flat_contacts.cpp` | Weight and declared pitch/crank moment rows; unilateral saddle/feet with bilateral grip; pedaling reaction included in total support request. |
| F214 `sim/ride/support_geometry.py`: finite box/pad geometry and sole projection | Existing `native/src/rider/support_geometry.hpp/.cpp` and `contact_math.*`; reached by the new controller and existing contact writer | Both broad faces, finite footprint and width, projected compression/shear and diagnostics are reused. |
| F227 `sim/ride/weld_pedals.py`: equality groups, pin/weld/grip reactions | Existing `native/src/rider/equality_reactions.*`, `attachment_wrench.*`, `writers/rider_contacts.cpp`; `support_dispatch.cpp`, `weld_settle.cpp`, `grip_springs.cpp` | One contact owner handles spring capture/release, overload loss, solved equality reactions and settle. No duplicate reaction or release-energy state was added. |
| `physics/distributed_tire.py`: vector density, normal response, flat aggregate load, NNLS fit | `native/src/numerics/density_fit.hpp/.cpp`, `nnls.hpp/.cpp` | Existing station quadrature; vector knot/aggregate/RMS reductions preserve NumPy pairwise order; fitted prediction preserves Fortran-contiguous BLAS layout. Fit status cannot certify material/plant validity. |
| Numerical dependencies | `native/src/numerics/dense.hpp`, `brent.hpp/.cpp`; existing `native/src/rider/least_squares.hpp/.cpp` | Checked row-major adapter, partial-pivot square solve, existing DGELSD SVD provider, bracketed root solver with source tolerance defaults. |

## Runtime and state contract

The existing controller facade remains the public runtime owner. Its `compute` call adds optional support availability in saddle/front/rear/grip order and a pedal-recovery flag. Omitted support availability means all false, matching the source default. The runtime integrator supplies the contact writer's actual enabled state and enables recovery only during active non-initialization operation.

Non-spindle compute uses the same snapshot/restore transaction as the controller owner: an exception restores controller state. Detached target kinematics do not overwrite the running state. State now carries PD components, allocation branch/point warm starts and latent activation. Reset clears activation and allocator memory; source PD diagnostic state follows its original reset behavior. Initial velocity and pose solving remain initialization-only.

The native state codec and optional nanobind codec both preserve these source-shaped diagnostic variants:

- Full allocation output with feasibility/residual, branch, torque/activation/power/wrench data, mode-search evidence, state snapshot and declared numerical identity.
- Disabled allocation's three-field record, without fabricated solver coordinates.
- Full support targets, optional crank tracking, and actual/target foot geometry/recovery diagnostics.
- Top-level sole goals for spindle, projected flat geometry, weld and recovery.
- Omitted `requested_nm` and effort budget/limit/observation fields for the disabled non-spindle source record; later solved observations add their fields explicitly.

`initialization/finalize.cpp` preserves the allocator's latent activation separately from delivered torque at the equilibrium-to-ride boundary. A disabled controller uses the ordinary zero-request initialization path. Allocator warm memory is retained with that state.

`runtime::implemented_capabilities(config, rider)` and its compatibility alias `capabilities` identify connected source paths. They preserve resolved physical-drive requirements, reject physical pitch assistance, require articulated topology for non-flat attachments, and retain the physical seated/rigid-leg gate. Tire paths identify the actual compliant, distributed, pneumatic or native-reference implementation supplied by the integrator. `implemented` does not assert physical or numerical acceptance: `supported` remains false and `qualification_pending` true in this unexecuted delivery.

## Numerical identity and remaining qualification

The allocator declares:

```text
bike-dense-sqp-bfgs-v1;ftol=1e-12;maxiter=500;restarts=4
```

This is a native dense SQP/BFGS implementation with active-set quadratic subproblems and an L1 merit line search. It is not the historical SciPy SLSQP implementation, and no bitwise or result-equivalence claim has been made. The source objective, units, constraints, exact projected-target fast path, residual threshold, iteration limit and four-restart boundary are explicit. Status/failure and physical feasibility remain separate.

The linear prefilter is a conservative native feasibility check. It can prove a contradiction from outward-rounded interval bounds, verify a numerical feasible witness, or return unknown. It does not implement all HiGHS presolve/simplex behavior. Unknown, budget exhaustion and numerical failure never become a claim of an empty mechanical region. This may examine additional modes and can affect a finite-budget result compared with the source; that difference needs an oracle campaign.

The NNLS active-set implementation is also new. Its restricted least-squares work reuses the existing DGELSD provider; no additional Jacobi SVD is shipped. The dense square solver, new optimization control flow and any platform-dependent numerical supplier still require qualification. Planar observability uses absolute singular value `> 1e-12`, while wrench least squares uses relative `rcond=1e-12` as in the source.

Source-domain edge behavior added explicitly includes zero-time activation without division by zero and empty vector quadrature broadcast against scalar rate/weight. These typed cases need dedicated oracle decisions where the historical Python input shape or zero-time optimization behavior is narrower. Optional binding adapters currently represent newly added nested diagnostic arrays as owned Python lists, the same JSON value shape; ndarray-specific compatibility has not been measured.

No performance, stack-budget, sanitizer, warning-policy, ODR, clean-build, native/Python parity or long-run stability result follows from static inspection. The source tests do not cover every branch of all F202/F207 modes. Existing per-source acceptance rows remain unchecked.

## Authored regression coverage

| Test source | Intended future assertions |
|---|---|
| `native/tests/migration/test_t28.cpp` | Exact bounded target/warm-start priority; callback bounds; equality simplex; infeasible diagnostic; pulling circle; malformed direct residual; unknown/certified linear regions; inverse-dynamics signs; NNLS boundary optimum; Brent bracket; density potential/separation/broadcast; synthetic calibration label. |
| `native/tests/migration/test_t29.cpp` | Analytic unilateral and friction responses; mode domains; released regions; truncated/complete search; latent versus delivered activation and Jacobian; friction requests/net crank moment; warm-state, full sole geometry and disabled diagnostic roundtrips; full articulated-config native decode. |
| `native/tests/migration/test_t30.cpp` | Weight/moment support balance; pedaling force balance; both pedal faces; one-time release-energy accounting; all 18 pedal/saddle/grip combinations through model/controller/contact construction; grip release versus connected refusal; settle energy; capability rejection; support diagnostic roundtrip. |

The T30 setter assertions use the writer's actual return contract: `set_enabled` returns the resulting enabled state. A spring-grip release returns false; a connected grip refuses release and remains true.

## Actual static review performed

Read the numerical implementations, declared source counterparts, facade/config/state consumers and registered CMake source paths. Corrected a public residual shape hazard, an absolute-versus-relative rank cutoff, vector reduction order, disabled state shape, neutral/IK/recovery constants and test expectations. Text inspection of the scoped new source files found no merge markers or trailing whitespace. This is source review, not compiler validation.

A separate read-only review of `research/environment.*`, `policy.*`, `plugin_policy.cpp`, `policy_session.cpp`, `initialization/research_startup.*`, `native/include/bike/policy_v1.h`, `NativePhysicsResearch.cmake` and CTest registration confirmed the following from source: plugin callbacks use owned host buffers; partial create failure is guarded by instance destruction; the instance is destroyed before unloading its library; ABI version/size/schema/callbacks and control presence bits are checked; policy sessions keep an accepted command for retry rather than invoking `act` twice. Source startup's safe non-advancing control is preserved.

That independent review found that `async_telemetry=true` was parsed and described but not connected to a native worker. The finding was sent to the research and integration owners. They assigned the worker to the existing T42/T53 boundary and arranged explicit pre-setup rejection for this unimplemented option. No research files were edited by this scope owner.

## Deferred commands and evidence

These commands are **instructions for later qualification, not commands executed in this task**. They require a properly configured native build and lifting the current no-execution restriction.

```sh
cmake --build build/native-release --target migration_T28 migration_T29 migration_T30
ctest --test-dir build/native-release -R '^migration_T(28|29|30)$' --no-tests=error --output-on-failure
```

Then run the mapped source-oracle groups for rider allocation/dynamics/coupled task, flat/weld/pin/spring topology, pedal waveform, source information boundary, native activation/state restore, density calibration and mode-search failure cases. Include both complete and deliberately truncated mode searches, solver failures, singular/rank-deficient matrices, activation-boundary saturation, strength tables, omitted support contacts and zero/disabled controller state. Qualify the declared numerical identity separately from existing same-provider spindle results.

The required native serialized gates, strict warning/stack/ODR checks, allocation-failure transaction checks and physical energy/support/long-run scenarios remain deferred. T63 removal remains unavailable until the common plan's actual evidence is produced.
