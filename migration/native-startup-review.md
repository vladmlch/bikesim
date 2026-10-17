# Native startup wave: source implementation review

Date: 2026-10-17

## Follow-up review of the committed integration

The combined tree at `9d9e49c0`, including startup commit `9c940c96`, has
open correctness findings recorded in
[the commit review](native-waves-commit-review.md). In particular, the
subsequent physics wave admits native-reference tires without handling the
absent applied tire writer in bootstrap capture and restoration (F1).
The new speed-control route also retains a zero-torque placeholder (F2).
These findings limit the earlier source-completeness statement; startup
and physics/research acceptance remain pending. This follow-up inspected
source only and ran no tests or builds.

Scope: **T16–T18 and T20–T25**. The implementation was made in the repository reconstructed from the supplied `repomix-output.xml(6).zip` attachment. The other four management groups remain outside this delivery.

## Result and acceptance status

The source now contains the complete native assembly path from ordinary physics and track configuration through model generation, configuration projection, initialization, state capture and construction of the existing native ride runtime. A thin `bike-native-ride` executable and nine focused migration-test sources are registered in both build graphs.

**Status: implemented, execution unverified.** The user explicitly prohibited compilation and execution. No CMake configuration, build, compiler, linker, test executable, simulation, project validation tool or reference generator was run. File extraction, source editing, text inspection, manifest hashing and archive/patch operations are the only operations used in this delivery. Test source is evidence of intended assertions, not evidence that those assertions pass.

The task graph's existing `status: planned` values and the Python-removal deletion gates are retained. The new `implementation` records distinguish this source delivery from runtime qualification. No Python source is deleted; that remains T63.

## Implemented task boundaries

| Task | Native result | Main locations |
|---|---|---|
| T16 | Typed physical-model request, MJCF assembly, topology/components/rider/transmission and owned engine compilation | `native/src/model/builder.*`, `xml_writer.*`, physical component translation units |
| T17 | Standard, stand, playground and legacy model exports; visual assets, environment, sensors and geometry JSON | `native/src/model/export.*`, `legacy_topology.cpp`, `assets.cpp`, `environment.cpp`, `sensors.cpp` |
| T18 | Named model addresses, force/Jacobian mapping, writer/runtime projection and full fresh-model identity checks | `native/src/model/mapping.*`, `native/src/runtime/project_config.*` |
| T20 | Obstacles, grade/profile composition, contact safety, wheel paths, surfaces and heightfields | `native/src/terrain/types.hpp`, `profile.*`, `obstacles.cpp`, `surface.cpp`, `heightfield.cpp`, `contact_safety.cpp`, `wheelpath.cpp` |
| T21 | Road/Terrain/Alpine generation, presets, research scenarios, placement and track-file round trips using the existing native RNG | `native/src/terrain/generators.hpp`, generator translation units, `trackfile.*` |
| T22 | Owned preparation context, physical initial pose, contact settling, measured convergence/failure and final initialization accounting | `native/src/initialization/context.hpp`, `setup_owner.*`, `initial_pose.cpp`, `equilibrium.*`, `settle_contacts.cpp`, `finalize.*` |
| T23 | Bounded dense trust-region reflective least squares, physical refinement, sag fitting and typed legacy relaxation algorithm | `native/src/numerics/least_squares.*`, `native/src/initialization/refine.cpp`, `sag_fit.*`, `sag_evaluator.cpp`, `legacy_equilibrium.*` |
| T24 | Native equilibrium/state persistence, historical NPY/NPZ readers, complete material state transfer and station projection | `native/src/initialization/state_file.*`, `equilibrium_cache.cpp`, `initial_state.cpp`, `native/src/io/npy.*`, `npz.cpp` |
| T25 | Request loading, native preparation, sealed step-zero bootstrap, runtime factory, metadata and headless entry point | `native/src/app/`, `native/src/initialization/request.cpp`, `capture.*`, `setup_owner.*` |

The machine-readable implementation record and source-to-native ledger contain the additional helper files and test registrations. Existing numeric, RNG, geometry, rider, suspension, engine, writer and runtime boundaries are reused.

## Assembly and ownership decisions

`SetupOwner` owns the generated model, shared `Stepper`, physical writers and initialization context until successful settling. It does not construct a partially initialized public ride and then mutate it through test hooks. The physical-step factory is shared with the existing runtime.

The road heightfield is populated before writer projection. The generated hfield asset is named `road`; its world geom is `terrain`. The front and rear contact geom names remain `geom_front_contact` and `geom_rear_contact`. Projection reconstructs the typed request's fresh model and compares its complete engine serialization, including float32 road samples, so a same-sized foreign model is not accepted by name checks alone.

After initialization, finalization resets clocks and accumulated work/loss accounting, restores canonical tire/contact material state, applies initial speed, establishes energy and center-of-mass baselines and refreshes durable contacts. Capture serializes the integration state, permitted mutable model fields, tire and rider/controller state, drive state, runtime configuration and initial presentation into the existing bootstrap format. Sealing is allowed once, only after successful initialization. `make_ride` transfers that bootstrap into the existing `runtime::NativeRideRuntime`.

State restoration decodes and stages tire, drive, contact and controller material before publishing it. Source-specific JSON escaping retains Python ASCII/surrogate representation for historical fingerprints and initial-state contracts, while native envelopes retain the shared native codec. The added stage/commit interfaces retain the existing state codecs. A transferred initial state refreshes the physical contacts while preserving the supplied warm-start state.

Failed preparation does not create a recording directory or publish a recording manifest. Cache persistence is explicit. The new executable handles preparation and bounded headless stepping; recording, viewer, replay and the full application CLI remain assigned to T38–T51.

## Input and compatibility boundaries

The request document follows the existing schema-1 contract. Physics, track and saved-state paths are relative to the document that declares them. Overrides are resolved through the existing configuration parser. `--fresh` bypasses reading a request's saved initial state and clears automatic cache and explicit equilibrium-state reuse.

Decimal seed text is preserved without narrowing to 64 bits. A request seed overrides the seed of a generator-backed track. For a materialized track it remains request metadata; it does not invent a generator or alter its geometry. The scoped track-TOML adapter preserves large seed integers and authored table order without changing the general configuration parser.

The current ride validator retains the supported physical configuration: articulated effort dynamics, the articulated planar rider, spindle pedals, pin saddle, connected grips, ideal mid-drive, no crank clutch or motor rotor inertia, and the compliant, distributed-reference or pneumatic tire backend. Model generation/export exposes additional physical and legacy branches; full runtime behavior for those branches belongs to the subsequent physics wave. The legacy initialization algorithm has typed callbacks so future legacy writers can integrate it without a Python object.

Native cache identity contains native source/build, provider, model and complete typed input identity. Historical NPZ verification uses a separate source-compatible fingerprint, including the historical track dictionary. An optional `--legacy-source-root` supplies the actual historical source bytes for this comparison; they are read and hashed without executing Python. This compatibility path does not become a prerequisite for fresh native startup.

The original track-file writer does not serialize `SteppedClimb`; its historical rejection is retained. Complete native track identity separately covers that obstacle type for startup and cache identity.

## Tests authored, execution not performed

Each task has its own `native/tests/migration/test_tNN.cpp` source and `migration_TNN` target. Tests cover independent model fixtures and exports, full model identity and projection, analytic force mapping, terrain and seeded-generator cases, equilibrium budgets and failure diagnostics, bounded/singular solver controls, sag acceptance, state/cache validation and ownership/reset behavior. The physical T23 regression independently evaluates the selected candidate from the incoming material state after two insufficient refinements; it checks for effects left by rejected probes without requiring an incorrect whole-call rollback. Specific assertion groups are listed in `migration/native-startup-implementation.json`.

Tests use retained fixtures as comparison inputs and fail when required binary fixtures are missing. No golden value, captured state, oracle manifest or tolerance was regenerated to make the new implementation appear qualified. The compiled equilibrium acceptance remains residual acceleration at most `0.05` for the default tolerance.

The standalone root graph and transition graph share `native/cmake/NativeStartup.cmake`. Existing core object files are carried into app consumers once; the runtime is not reimplemented in another library. New numeric array IO is kept above the platform/JSON boundary to avoid a platform-to-IO dependency cycle. Two pre-existing unmatched constructs in the transition CMake file were repaired while registering the new targets.

## Static review and its limits

The source review checked the public boundaries, source field/default mapping, ownership/destruction order, model and contact names, state-transfer sequencing, request precedence, error context, CMake source registration, local include resolution, delimiter balance and diff whitespace. Independent reviews found and corrected nested-initializer delimiters, unconditional seed override of a materialized track, fresh-state loading order, stale contacts after state transfer and historical-versus-native fingerprint separation.

The final file-only audit is recorded separately in `migration/native-startup-static-audit.json`. These checks are not a C++ parser, compilation, link validation, sanitizer run or numerical test. They cannot establish that the new code compiles or that engine serialization and solver results match the pinned oracle.

The delivered delta contains 123 new or changed files. The final lexical/include audit inspected 98 C++ source/header files and checked registration of 47 new production translation units. It found no missing local includes, delimiter imbalance, missing source registrations or diff-whitespace diagnostics. Input hashes in the audit identify the exact code and CMake files inspected.

### Remaining acceptance work

1. Configure and build both intended graphs under the repository's strict warning policy, including the standalone bindings-OFF graph. None of these gates ran in this delivery.
2. Run all nine focused migration targets and the required existing regressions for shared writer/controller/runtime changes.
3. Acquire the omitted immutable binary fixtures and original optional pose files from their authoritative source. Do not regenerate a new oracle from the candidate implementation.
4. Qualify physical startup, reset and first-step outcomes against the pinned engine and numerical provider. The new least-squares implementation uses an owned dense TRF algorithm with a one-sided Jacobi SVD; bit identity with SciPy/LAPACK is **not established**. No global tolerance substitutes for that evidence.
5. Check asset-root/VFS resolution, deterministic engine model serialization, cache compatibility, failure paths and process/library traces on an actual standalone installation. Historical NPZ identity also requires the matching source bytes and resolved asset paths; relocated inputs are not falsely assigned their old identity. Dynamic-DEFLATE/large ZIP64 corpus and allocation-fault/fatal-engine replay remain qualification work.
6. Keep T11/T14/T56 follow-up gates and T57 Linux work separate from this source-only wave. They were not rerun or closed here.

The historical preset-profile capture also appears to use vertical `profile_extent` bounds as its longitudinal sample range. The new tests retain that immutable fixture contract and add full-profile invariants; the old fixture alone does not prove full-length terrain equivalence.

## Reconstructed input and applying the delivery

The attachment is a packed text snapshot, not a Git checkout. The referenced historical HEAD remains provenance from its manifest; this workspace does not establish a new Git HEAD or commit.

Repomix extraction removed terminal newlines. A separate comparison baseline restores terminal LF where manifest hashes prove it and records other inferred text endings explicitly. Of 1,510 extracted files, 1,298 terminal-newline restorations are manifest-proven, 36 files already match recorded bytes, and 161 text endings were inferred without matching manifest evidence. There are 55 existing content/hash mismatches against the historical manifests. The oracle manifest references 2,061 unique fixture files, of which 1,808 are absent from this text snapshot. Original source/oracle manifests are unchanged. Details, mismatches and unavailable fixture counts are in `migration/native-startup-reconstruction.json`.

The deliverable patch is relative to that normalized extracted baseline. Apply it to the corresponding original checkout after reviewing any local differences; a newer checkout can require normal conflict resolution. The changes archive contains only changed/new files at repository-relative paths. The full source archive contains the reconstructed tree plus this implementation; it does not include omitted binaries, acquired vendor dependencies or a compiled application.

Application examples and public interfaces are documented in `docs/native-startup.md`. Commands there are instructions for a future authorized run; they were not executed during this delivery.

The downloadable delivery consists of `native-startup.patch`, `native-startup-changes.zip`, `native-startup-source.zip` and this `native-startup-review.md`. The patch and changes archive contain the same source delta. Prefer the patch for reviewing/applying the work to the matching checkout; use the full source archive to inspect the complete reconstructed context.
