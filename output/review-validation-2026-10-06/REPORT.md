**Validation of the external C++ review — 2026-10-06**

Audited checkout: `9f3a4442bc57312877c062299055813ec06ff4f5`. This is an audit of the supplied claims, not an implementation of the proposed fixes. No tracked source changed. The original report is useful as a hypothesis list, but its severity ordering and several headline assertions do not survive verification.

Each claim below was checked against implementation and a second perspective: caller reachability, parser invariants, existing oracle tests, installed library implementation, a compiler control, or an isolated reproduction. Runtime reproductions supplement those checks where practical. A source pattern alone is not treated as proof of a reachable crash. Very large allocations, capsule-allocation failure, arbitrary thread races, and unspecified “zip/clamp/value_or opportunities” were not exhaustively reproduced; those limits are explicit.

**What merits action first**

1. Fatal MuJoCo errors escape the native FFI boundary by terminating the process. Reproduced with a compiled contact model and invalid solver99: native constructor exits1, stock Python raises FatalError and survives. Importing mujoco first does not protect native calls. Missing/empty/garbage MJB controls survived with RuntimeError, so the review's broken-file example is too broad. Error interception needs a nonreturning, per-call/thread-aware mechanism and cleanup discipline; it is not a safe one-line global throwing callback. [Pinned MuJoCo interceptor](https://raw.githubusercontent.com/google-deepmind/mujoco/3.12.0/python/mujoco/errors.h), [error-handling documentation](https://mujoco.readthedocs.io/en/stable/python.html#error-handling).
2. Mandatory analyzer wrappers can falsely succeed. Missing/error-only tidy returns clean; empty CSA selection returns clean; real tidy loses header findings after changing checkout basename. These are verified failure modes, although today's baseline selected real sources.
3. A parser-valid geometric shift can leave ratio/rear teeth updated after new geometry fails. A 24→1000-tooth shift changed ratio1.4166667→0.034, while prepared phi/boundary remained old.
4. Stateful drivetrain calls can advance policies before throwing. Negative effort ceiling advanced effort0→1; two unsettled retries advanced2→3→4. Injected allocation failure likewise advanced10→20 without committing timestamp. Pending-guard mutation also exists in the Python oracle, so fixing it entails a shared behavioral decision.
5. Public malformed air/damper configs produce masked zero/NaN where Python raises. Confirmed negative chamber and zero max_lsc cases. Validation must distinguish original Python errors from deliberate NaN compatibility.

**Important rebuttals**

- The eight Stepper array properties retain the owner through nanobind's default `reference_internal` property policy. Runtime refcount increased by1 for every property, including efc_force and drive properties. `Stepper(path).qpos` is not the claimed immediate UAF. The binding's NON-OWNING lifetime comment is wrong and should be corrected. Owners do not refresh the shape/content of a stored runtime arena view.
- `req_bool` does not apply arbitrary truthiness. `'no'` is rejected. The actual problem is an untranslated `std::bad_cast` producing RuntimeError. `req_vec2/3` use sequence conversion and reject sets/iterators; the reported nondeterministic set acceptance is false.
- A missing explicit warning option does not imply missing coverage: current Apple clang umbrella flags already diagnose both disputed warning classes.
- Several “public API” crash examples are only calls to internal C++ helpers with violated preconditions. Current bindings resolve IDs, pair their own model/data, and route interpolation through validated profiles. MuJoCo's MJB loader independently rejects cyclic parents and out-of-range heightfield IDs in reproduced controls.
- C++23 feature counts are not a correctness standard. `nodiscard` does not cause validation to run: a discarded function call still executes and throws. Blanket `noexcept` would undermine recoverable error handling. P2/P3 explicitly defer the allocation-free whole tick to P4.

**Claim ledger — original section 1 (UB/crashes)**

| Original assertion | Verdict after cross-checking | Evidence / scope |
|---|---|---|
| MuJoCo fatal errors kill Python | Confirmed fatal runtime path; broken-file generalization qualified | stepper.cpp:37-45; no native interceptor; subprocess exit1 vs stock FatalError; missing/empty/garbage files survived |
| Empty ndarray owner causes immediate UAF | Rejected | binding.cpp:183-192 and drive_binding.cpp:604-613 are properties; nb_class.h default reference_internal; nb_ndarray.cpp retains self; refcount controls |
| Bench fixed buffers overflow at nv>64 / paired nv>32 | Confirmed prototype defect | bench.cpp:84,146; actual saved nv26 safe; ASan nv33 heavy and nv65 glue overflow |
| Bench state header can overflow model / negative resize | Confirmed prototype defect | bench.cpp:40-63; current dims match; negative nq length_error; oversized nq100000 vs model nq1 ASan heap overflow |
| Bench atoi/modulo causes up to4 SIGFPE | UB confirmed; signal claim rejected as universal | UBSan rewind0/abc division by zero; committed arm64 binary can survive; zero nu/no-body/empty-coordinate modes require matching model/workload |
| Bench mj_makeData unchecked | Confirmed omission; not all allocation failure returns null | bench.cpp:78; pinned engine can return null on checked size failure or terminate on allocation failure |
| Bench C++17/no CMake/no warnings/committed binary | Facts; prototype scope | Native CMake/sweeps exclude it; current binary linked to main checkout absolute dylib; C++17 alone is not unsafe |
| buffer removes const | Mechanism rejected; shallow-const concern retained | Template preserves T const; static_assert also confirms const mjData->qpos yields mutable span because C struct pointer pointee remains mutable |
| buffer negative count unchecked | True internal precondition | All present calls derive dimensions from loaded model; no negative extent API exposed to Python |
| Geometry angle/evaluate unchecked body IDs | True helper precondition; public exploit unestablished | chain.cpp:113,155; callers use resolve() or validated writer bodies; workspace not Python-bound |
| Geometry planar check lets NaN through | Literal comparison confirmed; downstream outcome qualified | abs(NaN)>tol false; finite center/up/reference validators often throw later; no reproduced successful NaN geometric linearization |
| angle reads qpos for nv without nq==nv check | Local omission; guarded callers | evaluate checks nq==nv; writer guards scalar topology; valid MuJoCo nq>=nv prevents the alleged qpos over-read even for ball/free |
| np_interp empty/mismatched spans | True unchecked direct helper; current caller protected | profile constructor checks >=2/equal sizes; contact validates query; no direct Python binding |
| Heightfield ID unchecked | Local omission; loaded model validation protects tested bad ID | MuJoCo validates references before Stepper writer construction; raw/later-mutated C++ model remains a trust boundary |
| rows*cols int overflow bypasses check | Unsafe expression for invalid models; current validated load route protected | Engine caps nhfielddata<INT_MAX and validates extent in mjtSize; no valid huge raster can pass that loader then overflow this product |
| Parent traversal lacks range/cycle guard | Local omission; tested corrupt load rejected | support_geometry.cpp:281-285; MuJoCo enforces ordered parents; actual Stepper cannot receive a foreign mutable parent array through its Python API |
| candidates(hi<lo) reserve wraps | Rejected as reachable current bug | Private helper; sorted profile and positive radius make caller lo<=hi; source and interval reasoning checked |
| Writers accept foreign-model mjData | True C++ pairing precondition; no Python route | Stepper always supplies its mj_makeData(model); pairing is standard MuJoCo API caller responsibility |
| Moved-from Transmission null-constant crash | Direct C++ hardening issue; reported immediate cause incomplete | Type is movable; moved-from operation traps on empty coefficient vector before null constant_; production ownership never moves it |
| Motor empty curve UB | Direct C++ hardening issue; normal FFI protected | Empty engaged curve traps; both current config entry points enforce >=2 rows |

**Claim ledger — original section 2 (exception safety)**

| Original assertion | Verdict after cross-checking | Evidence / scope |
|---|---|---|
| set_ratio commits before geometry throws | Confirmed reachable with valid parsed config | transmission.cpp:164-167; shift repro and old/new state comparison |
| compute advance before allocating diagnostics permits double advance | Confirmed; observed failure exists without OOM too | writer.cpp:264,346,367,437; native operator-new counter54; injected failure and repeated pending-guard failures |
| settle may double draw battery on retry | Conditionally confirmed | draw491 precedes maps493-498 and pending reset502; sparse allowed last={} restoration + allocation failure duplicated debit; normal complete ideal settlement allocated0 |
| raw new then capsule leaks if capsule creation throws | Confirmed conditional ownership gap; no allocation fault injection | Three binding families plus as_owned_bool; PyCapsule_New failure raises without acquiring ownership; subsequent ndarray failure after capsule ownership is guarded |

**Claim ledger — original section 3 (validation/parity)**

| Original assertion | Verdict after cross-checking | Evidence / scope |
|---|---|---|
| AirSpringSpec alone lacks all validation | Specific air hole confirmed; exclusivity rejected | Brake/rider have other holes; many other sections validate; Python air specs also unchecked |
| Negative air chamber becomes masked zero vs Python TypeError | Confirmed malformed-config parity difference | Native -0.0 vs live Python complex-to-float TypeError |
| Damper zero maxima/knees lead NaN vs ZeroDivisionError | Confirmed malformed-config difference | max_lsc0 reproduced; Python base attrs unchecked too; normal Python subclasses supply positive constants |
| Damper has no constructor validation at all | Overstated | Native validates travel/stroke and modern HBO; legacy fork validation is actually too strict in another case |
| req_f64 admits NaN/inf | Fact; semantics checked per writer | Resistance/tire/cruise/coil validators reject relevant nonfinite values; central casting alone does not establish a bug |
| Brake ceiling NaN / rider scalars unchecked / legacy HBO lacks isfinite | Concrete holes or inherited policy | NaN brake produces NaN torques; rider negative parameters accepted; unchanged Python shares validation weakness; legacy HBO differences deliberately preserve legacy behavior |
| Drivetrain typed ctors don't enforce invariants | Direct C++ issue; present parser coverage substantial | stop_time/cassette/curve/tau/etc validation prevents normal FFI failures; deliberately zero-valued “disabled filter” parameters must remain allowed where parser/oracle permit them |
| NaN orientation can be written to tendon/solver | Insufficient proof of proposed geometric path | finite geometry validators invalidate many such inputs before linearize; ideal relative coordinates can propagate nonfinite restored qpos, which is a broader accepted state/parity policy |
| Stepper.set_state lacks finite checks unlike set_inputs | Confirmed accepted input behavior, not automatically a defect | Public runtime accepts NaN qpos/time; Python snapshot assignment likewise accepts; expected policy must be decided explicitly |
| Unknown motor mode maps to emtb | Qualified | Profile parser restricts modes; profile-less custom mode is intentionally supported and tested by Python oracle |
| check_width always says set_state | Confirmed minor diagnostic defect | set_inputs wrong width reports “set_state: ctrl…” in runtime control |
| Tire NaN time exception differs | Confirmed | Native OverflowError('infinity') vs Python ValueError('NaN') |
| nearbyint follows FP rounding mode unlike round | Confirmed conditional | FE_UPWARD: native interval3 vs Python round(2.5)=2; default mode matches |

**Claim ledger — original section 4 (config/FFI)**

| Original assertion | Verdict after cross-checking | Evidence / scope |
|---|---|---|
| Unknown config keys silently ignored | Confirmed root/reader schema weakness | schema1+suspenson accepted and writer remains disabled; readers fetch known keys without exact-set checks; state parsers intentionally have stricter and sometimes partial schemas |
| req_vec2/3 accept any iterable, including unordered sets | Rejected | Installed seq_get uses PySequence_Check; runtime set and iterator rejected; list/tuple accepted |
| req_bool coerces 'no' to true | Rejected | Installed exact bool caster; runtime 'no' raises RuntimeError std::bad_cast |
| Nonstring dict keys throw bad_cast RuntimeError | Confirmed minor boundary inconsistency | Control/diagnostics/total controls reproduced; these parser failures precede writer mutation |
| _drive_core any unknown kind selects ideal / corrupts live cache | Confirmed private-hook issue | Unknown kind ratio operation changed shared coefficient1.416→1.214 while writer ratio remained1.416; current tests use bare Stepper; no runtime application caller |
| _drive_prepared_storage exposes uintptr addresses, latent UAF | Addresses confirmed; UAF allegation unproved | Test-only storage/capacity instrumentation; exposes no dereference operation; ctypes caller can misuse any raw pointer, which does not prove present ownership failure |

**Claim ledger — original section 5 (analysis)**

| Original assertion | Verdict after cross-checking | Evidence / scope |
|---|---|---|
| tidy ignores execution failure | Confirmed | Missing/error/crash mocks clean; warning control fails; controller independently reran missing/error/positive controls |
| analyzer empty entries clean | Confirmed | Empty and relative-file compile DB selections run0 TUs yet pass; today's database lists21 native TUs |
| HeaderFilter checkout name suppresses all first-party header findings elsewhere | Confirmed header-filter flaw; “all analysis” would be overstated | Real identical integer-division header diagnostic present only under cpp-port-p2 path; main-file diagnostics unaffected |
| cppcheck suppressed includes/nonexistent fallback degrades analysis | Confirmed failure-context risk; “no AST/nothing analyzed” rejected | Real conditional null-pointer finding disappears when defining header missing; present DB has real include paths; cppcheck normally uses independent built-in parser |
| Move checker globally disabled despite filter | Confirmed coverage tradeoff, not current defect proof | Both tidy and CSA exclude it; documented third-party false-positive rationale exists; header filters do not necessarily prevent analyzer path effects through third-party code |

**Claim ledger — original section 6 (duplication)**

| Original assertion | Verdict after cross-checking | Evidence / scope |
|---|---|---|
| Five handwritten CBLAS ABI declarations / magic enums | Confirmed duplication/platform coupling | Same supported legacy Accelerate ABI; no actual inconsistent prototype found; common wrapper and mjtNum type assertion sensible preventive work |
| Duplicate drive/policy helpers and state schema copies | Confirmed maintenance risk | Current differing signatures/types sometimes intentional; no blanket bug established by copying alone |
| Two CPython norm ports | Confirmed duplication; numeric semantics matter | Norm/hypot and BLAS paths intentionally reproduce different oracle operations; unify only identical rounding contracts |
| Three finite/positive validator styles | Confirmed consistency issue | policy_config return-value helpers, cruise local void helpers, brake comparison checks; pyfloat currently pow shim, placing validation there is a design choice |
| Duplicate resolve/jacobian helpers brake/tire | Confirmed overlapping logic | Some topology/error semantics differ; no current divergence defect independently shown |

**Claim ledger — original section 7 (hot path)**

| Original assertion | Verdict after cross-checking | Evidence / scope |
|---|---|---|
| Drivetrain50–60 allocations per tick / strings / angles / probe copies | Count confirmed for representative call; absolute generalization qualified | Instrumented ideal compute54 native new calls, excluding Python output; parameter/model dependent; probe copies real, “dead” is an optimization judgment |
| Suspension7–8 vectors and strings per call | Confirmed returned-component allocations | No counter proving allocation-free writer contract; P2/P3 per-call outputs own storage |
| Tire local qfrc/tmp, radius vectors, strings | Confirmed allocation sites | Existing persistent Jacobians do not imply every temporary must already be cached; profile candidates also allocate |
| Resistance row allocations/components | Confirmed | Source and returned ownership contracts; future whole-tick work |
| Rider per-call vector | Confirmed | Owned nv output by API design |
| engaged logic_error allocates and future RTSan abort masks error | Conditional future hazard | Throw path outside currently marked step/forward; SSO does not govern every std::exception implementation; no present RTSan failure shown |
| mutable const methods cause races | Conditional C++ concurrency limitation | Shared scratch non-threadsafe; Python bindings hold GIL, simulation is not promised concurrent; const does not itself imply thread safety |
| brake/suspension last_ write-only | Confirmed local dead telemetry | No accessor found; rider last_ is read for force output and must not be included in that claim |

**Claim ledger — original section 8 (modernization)**

| Original assertion | Verdict after cross-checking | Evidence / scope |
|---|---|---|
| Massive missing noexcept/nodiscard/constexpr implies safety defect | Rejected as blanket inference | Many getters already nodiscard; math deliberately throws; C++23 math constexpr availability limited; missing attribute alone proves no bad behavior |
| Stepper step/forward “provably noexcept” fixes RT contract | Unsupported recommendation | BIKE_NONBLOCKING addresses allocation/locking, not exception behavior; an error bridge must preserve recoverable fatal errors; cannot blanket-add noexcept |
| Discarding req_* result skips throw | False | Discarded call still executes and raises; nodiscard detects unused values, not missing validation execution |
| optional<Transmission> superior to unique_ptr | Design choice | Unique_ptr valid optional ownership/deep object seam; replacement changes movement/storage properties and does not fix moved-from invariants by itself |
| Sentinels -1/NaN/inf / cassette0 are defects | Facts or future readability work | Checked absent-index/initialization sentinels; cassette accessed only elastic path after resolution; current caller invariants hold |
| String states should become enums | Reasonable selective proposal | Existing FFI schema intentionally string; parser restricts closed sets; synthetic mode deliberately open; no erroneous current exhaustive branch shown |
| Magic components/strides/literals lack mdspan/constants | Fact/style proposal | Current insertion order matches uses; 3/9 encode MuJoCo dimensions; mdspan does not automatically establish ownership or bounds |
| Raw point_jacobian_into pointer has no extent | True private helper precondition | Caller-sized construction-owned3*nv scratch and MuJoCo raw C API; no demonstrated wrong extent in current callers |
| matvec(bool transpose) opaque | Interface readability proposal | Both branch semantics and caller argument convention inspected; no incorrect call established |
| raw optional dereferences should use engaged | Guarded current uses; consistency suggestion | Pedaling initializes cadence EMA before dereference; shifting initializes paired EMAs and restore rejects missing pair; no current empty dereference |
| chain/rider nb::arg names absent | Confirmed public API usability limitation | Positional binding signatures lack keywords/defaults in these functions; not memory UB |
| drivetrain.hpp not self-contained | Compilation-failure inference rejected | Standalone compile succeeds with pinned SDK; relies on includes from other project headers; direct include hygiene can still improve |
| Global using / include hygiene | Confirmed style facts | using DrivetrainWriter after namespace; no current name conflict/ODR error found |
| zip/clamp/value_or unspecified opportunities | Not independently assessable as specific changes | Supplied report provides no exact sites; these are optional refactoring proposals; generic clamp can change ordering/NaN/signed-zero parity |
| CruiseState target lacks default initializer | True type-level footgun; current initialization protected | Constructor designated target0 and binding restore designated target; uninitialized direct default local would be misuse; no current read of uninitialized target |

**Claim ledger — original section 9 (build/config)**

| Original assertion | Verdict after cross-checking | Evidence / scope |
|---|---|---|
| CXX_EXTENSIONS unset means gnu++23 | Confirmed build mode; functional defect unproved | Actual compile DB uses gnu++23; independent GCC sweep uses strict c++23; pedantic-errors remain |
| FAST release weaker than EXTENSIVE debug | Fact/policy choice | Default current build type is empty, also FAST; ADR explicitly chooses FAST; no measured production regression from that choice |
| Missing Wbitwise-instead-of-logical / Wextra-semi is coverage gap | Rejected | Real Apple clang controls diagnosed both under existing umbrella flags; GCC explicitly has extra-semi |
| Cruise cached dt differs from live Python dt | Confirmed semantic difference; current API scope limited | Mutable direct C++ model can trigger; Stepper Python owns separate loaded model with no timestep setter; external Python applier mutation does not mutate its copy |
| root_x hinge gives wrong physical units | Inherited topology acceptance | Native permits hinge; unchanged Python accepts it too; physically stricter model contract would be shared change |
| Brake wheel spin lacks joint type check | Confirmed inherited topology hole | Python wheel resolver likewise does not require hinge; legitimate addresses avoid memory OOB, but wrong model semantics possible |
| frontend hardcodes gcc16/no override | Confirmed portability limitation | Tool exists here and missing tool fails loudly; CMake/uv interpreter context can diverge outside current config |
| coverage glob lacks cardinality check | Confirmed latent tooling defect | Multiple .so matches become one invalid object filename; present normal dir has one extension |

**Additional observations absent or insufficiently developed in the original report**

| Observation | Verified impact / limits |
|---|---|
| Wrong lifetime documentation | Binding comment contradicts installed nanobind behavior and refcount evidence; can mislead callers/maintainers |
| Legacy fork150 rejected | Python legacy Charger3 accepts travel150 and HBO start160; native unconditional zone check rejects; resolved config discarded flag needed to distinguish modes |
| Boolean numeric acceptance inconsistent | Native tire material True→1.0 passes where Python TireSpec rejects; drive prepared qpos/Jacobian bool elements and optional effort ceiling/slip similarly bypass strict scalar checks |
| Restore accepts incompatible ratios/coefficients | Positive ideal ratio2 can coexist with shifting ratio/model coefficient1.416; current schema validates fields individually but not relational consistency. Decide intended state-injection contract before tightening |
| Tire snapshot interval narrower than Python | Finite time1e20 accepted by Python arbitrary integer interval; native int64/9.2e18 cutoff rejects, including some values inside int64 capacity. Extreme-scale contract issue |
| Alpha analyzer execution failures disappear | Report-only findings authorized, but uppercase LLVM ERROR execution failure silently lost; separate tool health from alpha finding severity |
| Frontend discovery failure passes | Nonexistent source directory makes find error yet prints gcc sweep clean through process substitution; current CMake source path exists |
| Rider-forces test ignores build selector | Standalone fresh process with coverage selector imports regular extension; sanitizer/coverage correctness depends on test import order |
| Custom coverage directory routes tests elsewhere | Script builds caller dir but tests fixed native/build/coverage; .so glob cardinality compounds the problem |
| Full profile is not native preflight | Full runs all pytest files without native rebuild/sweeps/import check. For overlapping native/physics work, running full alone does not prove the native artifact was rebuilt |
| ASan launcher documentation overclaims | Selector/options alone do not inject runtime or preflight selected module; safer uv env injection pattern is already present in P3 plan; no broad ASan suite claimed here |
| Prototype regeneration leaves state.bin stale | dump_model emits model.mjb/state.npz but C++ reads state.bin; no committed generator for it found, so normal regeneration can invalidate benchmark inputs |
| Benchmark workloads/numerics differ | Heavy sink NaN on current artifacts; C++ energy/heavy solve/warmup differs from Python. Useful operation-mix timing must not be described as numerical parity |
| libc++ DEBUG rationale inaccurate for this toolchain | CMake claims layout change; tested container sizes identical for all3 modes; local libc++ hardening ABI tags contradict blanket explanation. Policy FAST vs EXTENSIVE remains an independent performance choice |

**Verification evidence and boundaries**

- Fresh `bash tools/run_tests.sh native` completed with exit0: **697 passed, 15 warnings**, pytest14.53s, after native rebuild and frontend/tidy/CSA/cppcheck/ODR/script sweeps. Full baseline log: [native-baseline.log](native-baseline.log). Alpha findings are intentionally report-only. The first sandboxed attempt failed because hidden uv cache permission errors forced `/nonexistent` header fallback; rerun outside sandbox resolved the environment problem. It is not counted as a code/compiler failure.
- Controller reran both drivetrain and physics reproduction scripts against the rebuilt extension and independently repeated array refcounts, parser rejection, fatal/error controls, corrupted-MJB rejection, static const assertions, tidy/CSA controls, and real header-filter behavior.
- Native allocation interposer rerun reproduced54 new calls, advancement before bad_alloc, and conditional battery double draw. ASan/UBSan benchmark controls were isolated from committed artifacts and corroborated against raw source/model dimensions.
- No physical/full-episode acceptance, broad ASan/RTSan build, arbitrary OOM stress, or thread-race execution is claimed. The passing baseline validates existing gates and oracle cases; it does not invalidate the newly reproduced cases or the demonstrated gate failure modes.

Detailed source/caller/oracle evidence is in [drive-audit.md](drive-audit.md), [physics-audit.md](physics-audit.md), and [infra-audit.md](infra-audit.md). Controller qualifications above take precedence where a subsidiary note is broader. Structured infrastructure outputs are saved as [infra-repro-results.json](infra-repro-results.json), [infra-extra-results.json](infra-extra-results.json), and [bench-mismatch-asan.log](bench-mismatch-asan.log). Reproduction sources are saved alongside this report; their original commands/paths are retained so the evidence is traceable.
