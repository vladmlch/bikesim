# Python-Free bike-sim: Migration Specification

## 1. Objective and interpretation

Remove Python from all first-party execution, initialization, analysis, tests, developer automation and distribution workflows. Implement the remaining product behavior in C++ while keeping the numerical/runtime code that is already native. The finish line is not an extension imported from Python and not a native executable that shells out to Python for setup, recording or plotting.

This specification interprets the request as a **complete first-party migration**, including infrequently used legacy modes and tools. CMake, shell launchers, declarative TOML/JSON/XML, static data and JavaScript embedded in offline HTML reports may remain. Native third-party C/C++ libraries need not be reimplemented merely because they are not written in C++ throughout. No dependency may require an interpreter at runtime or during the final qualified source build. Any separately acquired native numerical library must satisfy that same build restriction.

Python is temporarily allowed only in the immutable reference checkout and an explicitly transitional fixture-capture/binding job. It is deleted from the active first-party tree after its corresponding replacement and the release gates pass. Captured numerical arrays, models and recording fixtures are data, not a continuing Python dependency.

The proposal is intentionally conservative about behavior and explicit about changes. Existing public Python import APIs cannot survive in a Python-free product; native library/CLI interfaces replace them. Existing external Python policy implementations require native ports. Recognized built-in policy spellings can be mapped to native equivalents without importing a module. Unknown `module:factory` strings must fail clearly rather than silently fall back. Required custom policies remain migration work until ported or explicitly retired by the user.

## 2. What was actually inspected

The uploaded ZIP contains a Repomix XML snapshot. Unmixing restored **1,075 text files**, with unique relative paths checked before extraction. The inventory contains **459 Python files / 101,930 lines**: 284 application files / 56,422 lines, 155 files under tests / 39,752 lines, 18 tool files / 5,581 lines, one project test / 170 lines and root conftest.py / 5 lines. There are **157 C++ .cpp/.hpp files / 53,701 lines** across the snapshot. These are physical line counts, not executable line counts or estimates of remaining effort.

The static test inventory finds **1,611 functions named test_**, including nested methods/functions; the earlier top-level-only count is 1,595. Neither number is the pytest-collected/parameterized test count. T01 collects that count and records every actual node. Test coverage cannot be inferred from file or line counts.

The source already contains native force writers, drivetrain/rider/tire kernels, an owned ride runtime, research stepping/sensors/programs, sample retention, column encoders, direct C++ tests and safety tooling. The port is substantial but not standalone: Python still constructs the initial simulation, captures bootstrap state, supplies startup presentation, performs significant configuration/model/equilibrium work, implements unsupported modes, owns final recording publication/analysis, provides GUI/CLIs and runs the tests/tools.

This planning pass did not build the application, execute its regression suites, regenerate model binaries or certify numerical parity. The snapshot excludes Git history and may omit lockfiles, binary assets and fixtures. A snapshot inventory is not proof that the real checkout contains nothing else. T00 reconciles the actual tracked tree and T01 establishes a measured baseline before changes.

See `docs/superpowers/audits/2026-10-14-python-free-cpp-source-audit.md` for source anchors and extraction provenance; `migration/source-manifest.json` supplies all text-file hashes.

## 3. Completion claims are separate

**C1: Standalone core.** A native executable can run an existing valid frozen model/bootstrap without importing Python. This proves boundary extraction, not native startup.

**C2: Fresh native startup.** Configuration, track generation, model generation, parameter projection, equilibrium/state import and initial presentation are produced natively. It can start from normal user inputs with no Python-produced cache/model prerequisite.

**C3: Feature-complete native application.** Physical modes, legacy/nonphysical modes, research, policies, GUI, batch, export/settle/playground, recording, replay and analysis all have demonstrated equivalents.

**C4: Python-free installed product.** Runtime, reader, viewer, reporting and installed developer commands work in a clean environment lacking Python. Binary dependencies and subprocess execution are audited, not just filenames.

**C5: Python-free development/release workflow.** Source configure/build/check/test/coverage/package/install works offline using the native dependency lock and native tools. Python does not survive as a build-system convenience.

**C6: First-party removal complete.** All ledger rows are closed, no required tests or capabilities were dropped, active documentation is updated, Python adapters/packaging are deleted, and clean supported-platform release gates pass. Historical source remains only in an immutable reference tag/archive, not on an active fallback path.

Do not call C1 or C2 the completion of this migration.

## 4. Architecture and dependency direction

Keep `native/src/` as the native implementation home. Do not introduce a competing engine under a new unrelated tree. Add a root CMake entrypoint that makes standalone builds ordinary, with optional transition bindings isolated behind `BIKE_BUILD_PYTHON_BINDINGS` until T63 removes the option altogether.

The target split is:

| Target/module | Responsibility | Must not depend on |
|---|---|---|
| `bike_numeric`, existing writers/rider/drivetrain/tyre | Numerical kernels, math/provider adapters, typed state | Python, JSON parser, database, GUI, CLI |
| `bike_engine` | MuJoCo ownership/error containment/model access | Python, GUI, database |
| `bike_runtime` | Typed ride/research owners, step/control boundaries, samples/accounting | Python, UI, SQL/filesystem publication |
| `bike_io`, `bike_platform` | Lossless data codecs, hashing, bounded filesystem/process primitives | Physics execution callbacks |
| `bike_configuration`, `bike_model`, `bike_initialization`, `bike_terrain` | Input resolution, model construction, native setup | Python, GUI, recording publication |
| `bike_recording`, `bike_replay` | Catalog, DuckDB, closed evidence, publication and reader/replay coordination | Policy execution during reading/replay; live simulation for analysis |
| `bike_viewer`, `bike_viz` | Independent render replica, input/HUD, offline plots | Authoritative live force/sensor pointers |
| `bike_app`, CLIs, batch/tools | Orchestration over the above | Interpreter fallback |
| optional transition bindings | Python conversion into the same typed interfaces | Independent numerical execution logic |

Not every library must become a separately installed package. These are dependency boundaries and focused source ownership, not a demand for excessive packaging. The integration owner may combine adjacent static targets when it preserves include/link constraints and testability.

Typed core records are primary. Reuse the existing insertion-ordered `runtime::Wire`/`WireObject` for hot-path metadata. Use a separate owned `bike::io::Value` cold-path tree for exact JSON/config/recording values, including large integer lexemes and tagged nonfinite/byte/date values. Do not replace typed physics with a generic object tree and do not coerce all SQL values to strings.

### Initialization owner versus live runtime

A valid `NativeRideRuntime` currently assumes a complete t=0 bootstrap. It must not be used as the starting object for computing its own missing bootstrap. Introduce `SetupOwner`, which owns a model/Stepper and shares the existing writer/control primitives. It performs initialization-only pose, velocity, attachment and equilibrium operations. Once complete, it produces an owned `RuntimeBootstrap` with a model image, typed configuration, full integration state, mutable model fields, hidden writer/controller states and initial presentation. Construction of the live runtime occurs after that seal.

Use in-memory native model images where possible; both frozen-bootstrap loading and freshly generated models feed the same validated native model-loading path. User-requested caches/exports remain explicit. A model path is an IO concern, not an object that can be swapped behind a running owner.

### Ownership and concurrency

One owner controls each mutable runtime/model/data pair. Readable snapshots, sealed episodes and render frames own their memory and outlive reset/close. Keep generation checks, no-copy runtime ownership, transactional validation and poison semantics. Independent runtimes may run concurrently only through the existing engine error-containment rules; adding global shared scratch is forbidden.

Rendering is a replica, not another physical simulator. Keep its model/data distinct. Geometry refresh may use the source's kinematic-only refresh sequence, but it must not invoke stepping, full forward dynamics, force writers, sensor acquisition or policy evaluation. UI/main-thread restrictions are handled by the native viewer owner. A live render frame includes the six declared mutable model arrays: `dof_frictionloss`, `site_pos`, `tendon_stiffness`, `tendon_damping`, `tendon_lengthspring`, `tendon_range`.

## 5. Preserve the existing behavior surface

The nine existing command names remain: `bike-sim`, `bike-playground`, `bike-export`, `bike-ride`, `bike-settle`, `bike-research`, `bike-batch`, `bike-replay`, `bike-recording`. Capture their actual parser definitions/defaults/invalid combinations and current outputs before porting. Do not make a generic new CLI and call it compatible.

The native setup source currently restricts execution to physical/articulated-effort/ideal-mid-drive with articulated-planar rider, spindle pedals, saddle pin, grip connect, pitch assist off, motor clutch off and zero reflected rotor inertia. It accepts compliant_2d, distributed_2d_reference and pneumatic tires, with backend-specific surface rules. This is the initial reusable core, **not the final product scope**.

Remaining requirements include elastic-chain/clutch/rotor behavior, material variants, alternative supports, other drive/rider combinations, legacy tire and nonphysical ride integration, suspension playground/stand modes, model and geometry exporters, sag/equilibrium fits, validation/calibration, plots and all developer tools. The exhaustive source ledger prevents a feature from disappearing merely because it lies outside the current native setup validator.

Define a capability matrix with one row per supported mode/profile/entrypoint combination. Every row names its source authority, resolved inputs, native path, parity fixtures, platforms, recording/replay behavior and final outcome. Unsupported combinations must still fail with the captured validation behavior. Source schema acceptance is not proof of runtime support; capability validation remains explicit.

## 6. Numerical compatibility and solver decisions

Preserve existing no-fast-math/disabled FP-contraction flags and operation/reduction order. Some helpers intentionally reproduce NumPy/libm semantics. `writers/pyfloat.hpp` is C++ numerical code; its name does not make it a Python runtime dependency. Preserve the existing Accelerate/CBLAS-backed provider for the strict macOS baseline until its exact behavior is characterized.

Assign every fixture a comparison class:

| Class | Use | Acceptance |
|---|---|---|
| B: binary64/object-bit equality | Existing same-provider bitwise kernel/state contracts | Exact bytes, including signed zero and applicable NaN payloads |
| N: bounded numeric | Algorithms whose source tests already use numeric tolerances, newly qualified provider-specific solves | Explicit per-field atol/rtol/ULP/residual from source or reviewed evidence; never one project-wide epsilon |
| S: structural/exact | Configs, enums, names/IDs, command schedules, schema/SQL types, events | Equal semantic structure/order/presence and exact values where prescribed |
| Q: physical qualification | Long runs, conservation, validity and release rigs | Original requirements and first-failure semantics independently checked |
| P: performance | Startup/stepping/recording/UI workloads after B/N/S/Q pass | Measured baseline comparison with actual completed valid work and machine identity |

Existing reference tests rebind Numba wrappers to `_py` kernels. Preserve that precise oracle identity for bitwise cases and capture production-Numba end-to-end cases separately. A reference run from a different implementation/provider is not automatically the same oracle.

Port bounded least-squares behavior deliberately: the equilibrium path uses three-point finite differences, Jacobian scaling, a bounded evaluation budget and planar state assumptions. Preserve auxiliary tire/contact/freehub variables, accepted-update rules and residual evaluation. T23 first builds solver control fixtures, then integrates equilibrium and sag. A generic unconstrained Levenberg-Marquardt replacement does not satisfy this contract.

The allocation objective is nearest feasible point to a target. Preserve box projection, exact fast path at residual <=1e-7, equalities/linear inequalities/nonlinear-extra ordering, SLSQP settings (`ftol=1e-12`, `maxiter=500`) and up to four warm restarts. Feasibility is remeasured from the returned point; an optimizer success flag is not sufficient. Linear-feasibility pruning must distinguish proven infeasible from solver failure/unknown. NNLS used in tire fitting retains nonnegativity and its tested behavior. T28 owns these solver seams; no other agent introduces an independent allocator.

The implementation choice is a narrow native implementation/provider of the required source algorithms, with licensing and exact version tracked. Do not take a dependency on the full SciPy Python stack. Where an algorithmic path cannot match a required exact historical profile, the task stays open while the solver/provider is corrected; changing expected outputs is not a completion strategy.

Legacy filtering retains source Butterworth/filtfilt order, cutoff, padding and short-input rules. The ride metrics source uses order 4/cutoff 100 Hz, with its ramp-exclusion and gravity conventions. Test at valid sample rates (for example 1000 Hz); 100 Hz input sampling cannot support a 100 Hz cutoff.

### Deterministic randomness

Native RNG work must reproduce the actual seeded contract: NumPy seed interpretation, SeedSequence state/spawn keys, PCG64 raw sequence, distribution transforms, caching/draw consumption and stream separation. `std::normal_distribution` is not an acceptable replacement just because it uses the same integer seed. Sensor noise/dropout spawning and terrain random draws have independent fixture groups.

Capture raw generator words, uniform/normal outputs, zero-scale and shape/draw-order boundaries, startup samples, reset behavior and complete generated terrain for representative seeds. A prerecorded noise tape solves replay of that recording, not generation of a new equivalent seeded experiment. Record algorithm/version identity in provenance. Batch scheduling never changes job seeds or draw order.

## 7. Model generation, configuration and startup

Source dataclass fields/defaults/validators are preserved in `migration/dataclass-contracts.md`. Explicitly distinguish missing, null, false, zero and empty values. Preserve profile/CLI merge precedence, asset-relative resolution, mode validation, units and serialized provenance. Public defaults must come from the actual input schema rather than zero-initialized C++ wire structs.

Port all MJCF branches with deterministic names/order. Compare compiled model arrays and named topology, not just successful XML parsing. Preserve joint types, nq/nv distinction, qpos/DOF addresses, inertia tensor orientation, equality/tendon/contact setup, actuator/sensor names and nonzero initial marker positions. The annular-cylinder inertia helper returns a full 3x3 tensor, with four physical inputs and the source validation domain.

Initialization includes road slope/profile/wheel clearance, rider pose and supports, crank phase, tire/freehub/controller state, integration state and all mutable model changes. The physical-equilibrium default residual tolerance is .05; source maximum steps are `max(40000, round(80.0/dt))` when not supplied. Preserve initialization-only velocity zeroing and refinement/retry behavior. Do not insert a runtime stabilizer to make a startup test pass.

Explicit state files outrank auto-cache selection. Preserve their existing validation distinctions and record them; a migration is not permission to silently equalize them. Implement bounded native read-only import of legacy NPZ/NPY variants, rejecting pickle/object execution. Check file dimensions/hash/provider/model identity before restore. New formats require explicit versioning and old readers remain where the source promised them.

## 8. Recording and analysis are core deliverables

Recording is opt-in through `--record`. `--out` alone does not enable it. No-record runs still perform required live checks/metrics, but they do not allocate growing optional histories. Recording keeps the original RAM-retained/final-only design; do not replace it with streaming because the Python layer is being removed.

One launch UUID contains consecutive episodes. Runtime generation is a separate reset/staleness concept. Preserve partial-period flushing, control-completed prefixes versus physical-failure suffixes, requested/applied commands, observation sequences, resets and sticky invalid status. Measurement decimation does not become recording decimation. Terminal viewer state seals memory and remains visible; normal publication happens on close/finalization.

Port the complete catalog, not only telemetry: runs, episodes, events, field/coordinate/alias catalogs, assets, integration states, metrics, recording metadata, physical/channel streams and research command/observation/transition tables. Preserve exact names/types/units/frames/phases, nested struct/list/map offsets, parent validity and optional presence columns. Coordinate catalogs must handle scalar/free/ball joints, not assume nq==nv globally.

The current encoder already creates typed owned `Piece` buffers. Separate it from nanobind boxing and ingest native DuckDB vectors/data chunks at publication time. Do not retain duplicate row-oriented histories or route every value through JSON. DuckDB ownership/error returns must be checked, appenders closed, transactions validated and database handles checkpointed/closed before hashing.

**Publication protocol:** create exclusive staging/destination objects, write the DB transaction, validate tables/presence/order, commit/checkpoint/close, write/report and hash payloads, and publish the manifest last using the source's exclusive hard-link discipline. Existing destinations, even empty ones, are never adopted. Validate opened regular files and symlinks; do not rely solely on a pre-open existence check. Hashes detect integrity errors, not authenticity.

A summary-report failure may produce a valid DB with `report_status=failed`. DB/payload/manifest failures leave the capture retryable and unsealed output recognizable; retries use a fresh exclusive destination. A successful repeated finalize returns the original path. Repeated SIGINT cannot interrupt the critical owning finalizer. Signal handlers themselves do no allocation, SQL, logging or publication. Preserve the original simulation error alongside secondary close errors. RAM-only runs cannot guarantee data recovery after SIGKILL/power loss; do not claim otherwise.

The container schema is **1** and research replay contract is **3**; never combine these version axes. Lossless tagged JSON follows `bike_recording.json.v1`, including `$bike_recording` escaping, nonfinite floats, bytes, decimals and temporal values. Native decoding preserves large integers without an intermediate double and rejects duplicate keys/bare nonstandard NaN.

`bike-recording` remains an independent read-only analyst command with `overview`, `fields`, `events`, `window`, `sql`, `export`, `compare`. The actual analyst export formats are **CSV and JSONL**, not Parquet. Preserve required episode selection, exclusive output creation, default unlimited export, source batch sizing and tagged structured/null/nonfinite values. Reading/exporting reports must not create a simulator, require a display, download an extension or execute a recorded policy.

## 9. Research policies and replay

Keep research's requested/applied control distinction, accepted pending window, begin/advance/acknowledge lifecycle, sensor delivery/noise/latency/dropout, metrics and validity. Policy actions occur exactly once per accepted boundary, even across a yield, pause, stop or allocation failure. A stop can drain only already accepted work. Reset restores startup sensor/RNG/program state without double-consuming a sample.

Native policy input is the source sensor-only observation plus the requested demand, never hidden ground-truth model state, tire forces or recording internals. Preserve built-in passthrough/zero/fixed-limit and traction-control composition semantics. The C ABI in `migration/interface-contracts.md` provides version/size checking, host-owned bounded buffers, reset/action/destroy lifetime and no exception/STL crossing. Load only explicitly selected native modules; they are trusted executable code, not sandboxed content. Record binary/config hashes. Reading a recording must never load them.

Checked replay and stored-state playback are different products. Port legacy v1/v2 input formats plus DuckDB v3, original backend/execution identity checks, immutable assets, initial state and command scheduling. Historical Python-backed checked replay requires a certified native implementation of that historical arithmetic profile; do not relabel it as native or silently relax the comparator. An uncertified profile reports explicit checked-replay incompatibility while validated stored-state playback/export remains possible. Final full compatibility stays blocked until required profiles are certified or the user explicitly approves the break.

## 10. UI, plots and batch semantics

Use the native MuJoCo visualization API with GLFW rather than embedding or re-executing `mjpython`. Capture the current Python-amended step-zero presentation natively. Keep fixed physics dt; scale 1/2/4/8 adjusts work demand, not integration/controller time. Preserve the source 8 ms cooperative compute budget, 60 Hz frame requests and debt accounting from actual completed steps. These settings do not promise sustained 8x or hard real-time performance.

Preserve separate physical/research input maps, reset/close behavior, terminal-frame hold, camera modes, HUD/invalid labels and marker meanings. A headless build excludes GUI requirements entirely. UI lifecycle semantics can be tested with fake clocks and snapshots; actual supported-machine display/HiDPI smoke checks are separate mandatory release evidence.

Replace Python plot construction with native data adapters and a bounded native drawing layer. Use a pinned native raster backend for static images and bundled licensed offline Plotly JavaScript for existing interactive HTML, where needed. Compare numerical series/labels/markers independently of pixels. Do not remove geometry, dyno, leverage, comparison and ride plotting because simulation itself is native.

Batch/sweep orchestration uses native argv-based subprocesses, bounded workers, stable per-job seeds and output identities, predictable result ordering and owned cancellation/reaping. Worker exit zero does not imply physical validity. No shell interpolation, implicit current-directory mutation or shared output folder adoption is allowed.

## 11. Build, platform and toolchain transition

Current CMake is macOS/Accelerate-specific and finds Python/MuJoCo/nanobind through the interpreter/wheel layout. Existing offline packaging targets CPython/Linux wheels. A native replacement must resolve both surfaces rather than claim portability from successful compilation on one machine.

Keep the strict macOS provider as the primary parity baseline. Qualify Linux x86_64 headless with explicit native MuJoCo/BLAS/provider binaries, ELF paths and provider-specific evidence before declaring the old offline install surface replaced. Windows and Linux GUI are not new promised targets. Record actual deployment floors from produced binaries; do not invent an OS/CPU support matrix from library marketing.

Introduce a checked native dependency lock with real versions/commits, artifact hashes and licenses. Given snapshot pins are retained initially; newly proposed libraries are resolved and locked in T03, not represented by fabricated hashes. Offline builds never auto-fetch. Audit transitive generation tools: a native library's source build may still require Python unless using a qualified pre-generated/native-only path.

Port the Python native-check controller, contract checker, coverage runner, baseline/episode/performance/diagnostic tools, proto-native benchmark asset/state tools and offline installer. Keep shell launchers thin. Preserve all nine safety sweep classes and their negative controls. `tools/run_tests.sh` currently invokes native preflight for **both native and full**; the native replacement must do the same. Test reports include the exact selected binary/build/context and reject stale artifacts and zero selected tests.

## 12. Testing, fixture governance and deletion evidence

Reference fixtures are generated before the implementation they assess. Each fixture records input/source hashes, software/provider identity, dtype/shape/endian/row order, comparison class, expected validity and output hashes. Do not write guessed expected numbers into a file merely to make a C++ test compile. Missing binary fixtures or a failed capture is a red gate, not a skipped comparison.

The supplied static test map is a seed, not a completed assertion map. T01 collects actual parameterized nodes; T52/T53 preserve every independent assertion in native tests. Test framework/proxy-specific cases may be retired only with an explicit equivalent native boundary assertion or content-level duplicate proof. Test counts alone are insufficient.

Each work package contains a first regression example, mandatory additional cases and a per-file inventory. Every leaf file/card follows the same red-test, minimal native change, green-test, source comparison, ledger update and commit cycle. Use narrow CTest selection for fast feedback and run full shared preflight at the stated integration boundaries. Sanitizer/coverage jobs complement but do not replace numerical/physical checks.

A source deletion requires: accountable owner; exact replacement path/symbol or approved retirement; test-node mapping; capability-row closure; passing comparison evidence; commit identity; no remaining import/build/runtime edge. Delete files in reviewed groups and then rebuild from a clean tree. Existing native C++ binding files may contain core types/logic: extract first, delete adapters last.

## 13. Global constraints

- Preserve all required first-party behavior; no Python fallback, embedded interpreter, subprocess interpreter or mandatory Python build/test tool in the final product.
- Preserve existing C++ kernels and the single authoritative physical stepping path; boundary extraction is not permission to change equations or operation order.
- Use C++23 and CMake >=3.24. Retain strict warnings, stack protection, numerical flags, hardening and native safety checks.
- Initially retain MuJoCo 3.12.0 with the verified private TLS error ABI and DuckDB 1.5.6; lock actual native dependency hashes before use.
- Maintain named schema/config/CLI/recording/replay contracts and every supported mode, or obtain explicit user approval for a documented compatibility break.
- Keep existing same-provider bitwise guarantees; no global epsilon, golden regeneration to hide failures, or silent cross-provider equivalence claims.
- Recording remains opt-in, RAM-retained during execution and final-only publication; no streaming redesign, silent history loss or automatic recording from --out.
- Every Python deletion requires a ledger owner, native replacement or approved retirement, assertion/capability coverage and passing evidence; final removal is T63.
- Changes to shared headers/CMake/registries go through one integration owner. Run full native gates serially per machine and record the exact build identity.
- Source examples and API snippets in this plan are proposed implementation contracts, not a claim that code was compiled or tests passed during planning.

## 14. Requirement-to-task acceptance map

| Requirement | Owning tasks | Final evidence |
|---|---|---|
| R01 source completeness/baseline | T00-T02 | hashes, actual checkout reconciliation, collected nodes/oracles |
| R02 standalone native build/error ABI | T03-T04, T56-T58 | Python-disabled configure, direct tests, provider/ABI reports |
| R03 typed runtime isolation | T07-T11 | public header/link closure and state/ownership tests |
| R04 lossless cold IO and process safety | T05-T06 | codec, precision, filesystem/process/signal failures |
| R05 all config/model/geometry behavior | T12-T18 | resolved config and compiled-topology comparisons |
| R06 randomness/terrain equivalence | T19-T21 | raw RNG, draws, seeds, complete terrain fixtures |
| R07 fresh native initialization/cache | T22-T25 | uncached startup/state import and t=0 bootstrap |
| R08 reuse native kernels/missing physics | T26-T33 | branch reuse map and full mode matrix |
| R09 research/policy information boundary | T34-T37 | pending-window/sensor/policy/error tests |
| R10 full recording/reader/publication | T38-T43 | schema/data fixtures, failure injection, no-record invariants |
| R11 checked replay/legacy profiles | T44 | versioned fixtures and execution-identity decisions |
| R12 reports/export/analyst CLI | T45-T46 | exact CSV/JSONL, reports and parser/output comparisons |
| R13 all entrypoints/GUI/plots/batch | T47-T51 | CLI matrix, viewer ownership, plot data, seed/job tests |
| R14 preserve all tests/validation | T52-T54 | assertion map, native tests, independent release rigs |
| R15 developer tools/check/coverage | T55-T57 | tool schema/command equivalence and full preflight |
| R16 offline native packages/platforms | T58-T59 | clean supported-platform source/install smoke evidence |
| R17 active docs/compatibility disclosure | T60 | command map and executed examples |
| R18 end-to-end feature qualification | T61 | capability/performance/validity matrix and reviewer sign-off |
| R19 absence of Python as code or hidden edge | T56, T62-T63 | native audit negative controls, linked deps, no-interpreter execution |
| R20 safe final deletion/no lost oracle | T63 | closed removal ledger, immutable reference, clean rebuild |

## 15. External engineering references

These references verify the availability/semantics of proposed native API surfaces. They do not override the snapshot's version pins or promise numerical interchangeability. Pin headers/libraries and test actual deployed versions.

1. MuJoCo native visualization: https://mujoco.readthedocs.io/en/stable/programming/visualization.html . Its abstract scene and rendering layers support a native viewer; the application must retain its stricter non-stepping replica contract rather than copying a demo's simulation loop.
2. MuJoCo programming/model-data ownership: https://mujoco.readthedocs.io/en/stable/programming/simulation.html . Model/data compatibility and ownership remain explicit native concerns.
3. DuckDB C API and appenders: https://duckdb.org/docs/current/clients/c/api and https://duckdb.org/docs/current/clients/c/appender . Native handles, vectors/data chunks and checked appender lifecycle replace Python ingestion.
4. GLFW thread rules: https://www.glfw.org/docs/latest/intro_guide.html . Initialize, manage windows/contexts and process events according to main-thread restrictions.
5. NumPy RNG compatibility and PCG64: https://numpy.org/doc/stable/reference/random/compatibility.html and https://numpy.org/doc/stable/reference/random/bit_generators/pcg64.html . Generator identity alone does not establish complete distribution/seed compatibility; test the captured stream contract.
6. RapidJSON SAX raw numbers: https://rapidjson.org/classrapidjson_1_1_handler.html and https://rapidjson.org/reader_8h.html . `kParseNumbersAsStringsFlag` exposes number lexemes through `RawNumber`, allowing the native codec to avoid precision loss.
7. Cairo native raster/PNG surfaces: https://cairographics.org/manual/cairo-PNG-Support.html . Check surface/write status and qualify the required output subset, rather than assume the simple PNG helper provides every export control.

References checked during this planning pass on 2026-10-14. Repository facts in this specification are grounded in the supplied source, not in current upstream examples.
