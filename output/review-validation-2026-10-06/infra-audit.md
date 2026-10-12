# Native infrastructure and prototype claim audit

Read-only audit on 2026-10-06 in `/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.worktrees/cpp-port-p2`. No tracked source was edited. No broad native build or full test sweep was run by this agent (the controller owns the baseline native run). Reproductions use copied/generated artifacts in `/private/tmp`, with Python invoked through `uv`. Findings distinguish actual defects, limitations, and claims that the local evidence contradicts.

Reproduction scripts and complete structured output:

- `/private/tmp/infra_audit_repro.py`, `/private/tmp/infra_audit_repro/results.json`: tool exit-status mocks, real clang-tidy header filtering, and ASan/UBSan benchmark cases.
- `/private/tmp/infra_audit_extra.py`, `/private/tmp/infra_audit_extra/results.json`: real cppcheck missing-header control, missing frontend sources, compiler-warning controls, hardening layout controls, coverage routing mocks, standalone rider-forces import, and committed benchmark argument cases.
- `/private/tmp/infra_audit_extra/mismatch.log`: corrected working-directory reproduction of the positive oversized state/model mismatch, producing ASan heap-buffer-overflow. The earlier `bench-mismatched-large-nq` record in extra/results.json used the wrong cwd and reports a missing model; use this separate log as the actual evidence.

## Highest-confidence infrastructure defects

### 1. clang-tidy can fail, crash, or be absent and still pass the mandatory gate — confirmed

**Source:** `tools/check_native_tidy.sh:9-16` redirects every worker's output, discards the complete pipeline status with `|| true`, and fails only on the literal substring `warning:`. It does not check errors, worker return statuses, or whether a worker ran. Missing binary diagnostics and error-only clang parsing failures are hidden by the success message. `set -euo pipefail` does not help because this pipeline explicitly ignores failure.

**Caller/contract:** `native/CMakeLists.txt:190-196` wires the script to `check_tidy`; `tools/run_tests.sh:42` considers target success a prerequisite for native tests. `AGENTS.md` explicitly requires missing sweep tools to fail. Thus this is a correctness-gate defect, not merely cosmetic logging.

**Reproduction:** one fake TU, with `CLANG_TIDY` pointing respectively to a nonexistent file, a script exiting1 with `error: mocked parse error`, and a script killing itself with SIGSEGV, all return0 and print `tidy clean`. A warning-emitting control returns1. These prove the gate behaves incorrectly without needing to corrupt production compilation.

**Priority:** high for verification reliability. Preserve all worker statuses, fail on tool execution/parse errors, report output on failure, and verify a nonempty expected TU set. Treat diagnostic findings independently from successful tool execution.

### 2. path-specific clang-tidy header filter hides own header findings in other checkouts — confirmed

**Source:** `native/.clang-tidy:47` is `HeaderFilterRegex: '.*cpp-port-p2/native/src/.*'`. It is not an absolute regex, but it hardcodes a worktree basename. Any clone/branch checkout not containing this path segment loses header diagnostics in its own tree. Main-file diagnostics remain enabled; the claim must not say the entire sweep is disabled.

**Caller/config:** the CMake target runs source files from arbitrary `${CMAKE_SOURCE_DIR}/src`. Neither CMake nor the wrapper adjusts the filter for another checkout. The prototype filename is unrelated; actual native headers live in the filtered tree.

**Reproduction:** real `/opt/homebrew/opt/llvm/bin/clang-tidy`, identical `test.hpp` with `inline double divide(int a,int b) { return a/b; }`, identical source including it, native config copied into a temp checkout, and `--checks=-*,bugprone-integer-division`. Under `/private/tmp/infra_audit_repro/cpp-port-p2/native/src` it emits the warning; under `other-checkout/native/src` it emits nothing. Both tool calls return0 (normal tidy finding exit behavior); the wrapper would fail only the first case.

**Priority:** medium/high, because moving the checkout silently narrows mandatory analysis coverage. Match native/source headers independently of the checkout basename, or pass an escaped source-derived filter.

### 3. analyzer can analyze zero translation units and declare clean — confirmed

**Source:** `tools/check_analyzer.sh:21-23,54-73` filters raw `file` values using an absolute prefix, then loops without checking count/completeness. `file` paths are not normalized relative to their entry's `directory`; valid compile databases can contain relative paths. The script also accepts stale/empty databases and source/database mismatches.

**Caller/config:** CMake provides the current source directory, but the script independently trusts an existing compile database. Current repository entries are absolute, and the current database contains all 21 listed core TUs; the zero-entry bug is a latent failure mode, not evidence that today's actual baseline skipped every TU.

**Reproduction:** `compile_commands.json=[]` and a valid record with `file='../native/src/test.cpp'` both return0 and `csa clean`, even with `ANALYZER_CLANG` pointing to a nonexistent binary. A tool cannot have run in these cases.

**Additional compatibility limit:** `base_cmd` reads only `e['command']` (`:26`); the compilation database also permits an `arguments` array. An arguments-only entry would fail loudly with KeyError rather than falsely pass. That is separate robustness work, not a reason to claim false cleanliness in the current database.

**Priority:** medium/high for verification reliability. Resolve per-entry paths and verify the selected source set against expected TUs before analysis.

### 4. alpha analyzer execution failures can disappear completely — confirmed extra issue

**Source:** `tools/check_analyzer.sh:60-66` assigns `rc` but ignores it, and prints only lines containing lowercase `warning:` or `error:`. Crashes reported as uppercase `LLVM ERROR`, or no diagnostic at all, disappear. Genuine alpha findings being report-only is explicitly authorized by AGENTS.md; silent tool failure is a different issue.

**Reproduction:** mock analyzer exits0 for stable and exits1 with `LLVM ERROR: mocked analyzer crash` for alpha. Whole script returns0 with only `csa clean`. A warning/error diagnostic alpha finding is printed, so this is a reporting blind spot rather than all alpha output being suppressed.

**Priority:** medium. Report alpha execution failures distinctly; no need to make unstable alpha findings blocking to fix this.

### 5. frontend source discovery failures also return clean — confirmed extra issue

**Source:** `tools/check_native_frontends.sh:8-29` feeds the loop via process substitution `done < <(find ... | sort)`. The parent shell does not receive that process substitution's failure as the loop's exit status, and the script does not check a nonempty set.

**Reproduction/caller:** invoking the real script with a nonexistent source directory returns0, prints `gcc sweep clean`, and also emits `find: ... No such file or directory`. CMake normally supplies an existing `${CMAKE_SOURCE_DIR}/src`, so current source discovery is not proven broken. This does prove a path/config failure can be mistaken for successful verification.

**Priority:** medium/low. Check discovery status and the expected TU count, ideally share source enumeration with CMake.

## Prototype benchmark claims

The file explicitly labels itself a throwaway microbenchmark. It is not the production Stepper loop. Prioritize prototype defects according to whether the benchmark continues to provide evidence for port decisions, not as if every issue currently affects the production engine. Native tests consume the committed MJB model, but do not compile or exercise `bench.cpp`.

### 6. fixed 64-wide buffers and second jacobian write overflow — confirmed, latent on different models

**Source:** `tools/proto_native_bench/bench.cpp:84` allocates `srcv[8][64]`, `acc[64]`, `jacp[192]`, `jacr[192]`. Glue writes `nv` elements (`:97-99,120-122`) and `mj_jac` writes3*nv (`:102,126`). Heavy writes another 3*nv at `jacp+3*nv` (`:146`), requiring6*nv<=192; therefore glue requires nv<=64 and heavy requires nv<=32. No guards assert either constraint.

**Artifacts/callers:** actual saved model and state agree: nq26, nv26, na0, nu12, nbody28; state header `(0xBEA0,26,26,0,12,28)`. These buffers are large enough for the currently committed model. `dump_model.py` regenerates an arbitrary model from simulation configurations without documenting these bounds.

**Reproduction:** compile unmodified source with Apple clang21 ASan+UBSan, pinned SDK26.5, proper headerpad and MuJoCo install-name rewrite. Generated valid independent slide models and matching state files: nv33 glue returns0; nv33 heavy produces an ASan stack-buffer-overflow writing792 bytes from the second `mj_jac`; nv65 glue produces an ASan stack-buffer-overflow writing520 bytes to acc. Boundaries and specific storage are therefore measured, not guessed.

**Priority:** medium for an actively used benchmark; allocate once using actual nv or reject unsupported dimensions before any buffer use. No per-step allocation is necessary.

### 7. state header accepts negative and incompatible dimensions — confirmed

**Source:** loader (`:40-50`) checks only header length and magic, then copies signed counts into vector lengths. `restore` (`:55-63`) copies header-derived byte counts into model-sized buffers without comparing nq/nv/na/nu/nbody. `6*s.nx` and byte-size expressions also use signed arithmetic, so sufficiently large positive counts introduce integer-overflow hazards before conversion. Native Stepper validates widths separately; these are prototype vulnerabilities, not missing guards in `Stepper::set_state`.

**Artifacts/control:** current header dimensions match current MJB exactly. This explains why ordinary runs survive; it does not validate the parser for a regenerated/mismatched state file.

**Reproduction:** changing only nq to-1 yields an uncaught `std::length_error` and abort. Pairing the original state with a tiny nq1 model returns0 despite writing the wrong model widths (intra-arena writes can evade ASan). With positive nq100000 and a fully present state payload for an nq1 model, ASan reports heap-buffer-overflow in `restore`, writing800000 bytes past a4224-byte mjData buffer; see separate mismatch.log. Rejecting malformed dimensions must precede allocation/copy, and exact compatibility should be checked.

**Priority:** medium if local artifacts can change or be supplied externally; current trusted committed artifacts avoid the trigger.

### 8. atoi and zero modulo — confirmed undefined behavior; unconditional SIGFPE claim contradicted

**Source:** `:68-69` use `atoi` without validation; `i % rewind` runs in warmup at `:95` even when steps=0. Both rewind `0` and nonnumeric `abc` become zero. Negative/zero steps also produce invalid timing statistics at the final division; unknown mode silently chooses bare.

**Reproduction:** sanitized unmodified source aborts with UBSan division-by-zero at line95 for rewind0 and abc. The committed unsanitized arm64 binary with `bare 1 0` returns0 and prints timings; therefore asserting it necessarily raises SIGFPE is wrong on this environment. C++ UB is real even where the architecture/compiler happens to continue. `bare 0` prints NaN timings and returns0.

**Priority:** low/medium CLI correctness. Parse/range-check counts and mode before loading or timing.

### 9. mj_makeData return unchecked — confirmed, with constrained failure explanation

**Source/caller:** `bench.cpp:78-80` immediately passes the returned pointer to `restore`, which dereferences it at line56. Production `native/src/stepper.cpp:40-44` checks and throws; do not generalize the prototype omission to the native owner.

**Primary source:** MuJoCo3.12.0 `mj_makeData` initializes the pointer to NULL, calls `mj_makeRawData`, and returns it. `mj_makeRawData` can return after a checked buffer-size overflow before setting the destination; ordinary allocation failures call MuJoCo's fatal error path instead. Thus "unchecked nullable result" is supported, while "all OOM always returns null" is not. [Pinned MuJoCo source](https://raw.githubusercontent.com/google-deepmind/mujoco/3.12.0/src/engine/engine_io.c).

**Priority:** low/medium for prototype; add null check without claiming it replaces engine fatal-error handling. No actual OOM was induced.

### 10. c++17 build comment, no CMake target, committed binary — facts confirmed; implications need scope

**Source/config:** bench.cpp:18-22 documents direct clang++ c++17 compilation. Native CMake builds only native/src; no target compiles benchmark source. Every sweep enumerates native/src (or its compilation database), so the prototype does not share the production warning/sanitizer contract. Prototype uses C++17-compatible facilities; c++17 alone is not a correctness defect.

**Repository/runtime:** `git ls-files tools/proto_native_bench` includes the executable and model/state artifacts. `otool -L` shows the executable records `/Users/vladislav.molchanov/Desktop/tmp/mujoco_clean/.venv/lib/python3.14/site-packages/mujoco/libmujoco.3.12.0.dylib` from the MAIN checkout. It works here because that path exists; it is not a relocatable build artifact.

**Measured documentation failure:** the documented compile recipe initially fails to link on this host because the default CLT SDK27.0 contains arm64e.x1 entries the selected linker cannot parse. Pinning SDK26.5 solves compilation, but the documented rpath does not resolve MuJoCo's framework install name; a plain resulting executable cannot start. A proper install-name rewrite requires headerpad, as already implemented in native CMake. These additional steps were necessary for the isolated source reproduction.

**Extra consistency defect:** `dump_model.py:53-60` writes model.mjb and state.npz, but never writes state.bin. `bench.cpp:79` reads state.bin; `bench.py` reads state.npz. Rerunning the documented artifact generator therefore updates the C++ model and Python state while leaving C++ state.bin stale. Repository search found no committed state.bin generator. This concretely makes the unchecked shape/state compatibility relevant to ordinary prototype regeneration.

**Priority:** medium if benchmark results are still referenced: make regeneration/build explicit and reproducible; do not classify every committed prototype artifact as inherently wrong without considering the throwaway purpose.

### 11. current heavy sink is NaN and workload is not numerically identical to Python — confirmed extra limitation

**Source comparison:** C++ energy uses actuator_force[j%nu]*qvel[j] for every nv (`:105,129`), while bench.py dots actuator_force with the first nu velocities once. C++ heavy b assembly uses only three force coordinates (`:153`), while Python computes full jacp@qfrc_constraint. C++ batch Gaussian system has no diagonal regularizer or robust pivot/rank handling, while Python adds1e-6*I and uses lstsq; the simulated rows come from a3-row jacobian, making rank deficiency unsurprising. Python performs heavy work in warmup; C++ warmup never enters the heavy block. Allocation/record workloads differ deliberately as well.

**Measured artifact:** the unmodified sanitized C++ heavy source on the committed nv26 model with one measured step returns0 and prints `sink nan`. This is not memory UB, and by itself does not prove the timing is unusable; it does contradict treating sink as an end-to-end numerical parity check. The file describes operation MIX; compare cost approximations accordingly rather than claiming the two loops compute identical results.

## Analyzer and build-policy claims requiring qualification

### 12. cppcheck suppresses missingIncludeSystem — fact confirmed; "nonexistent AST" is not an accurate diagnosis

**Source:** `tools/check_cppcheck.sh:31-41` imports compile_commands, uses its built-in parser, enables warning/performance/portability, and suppresses missingIncludeSystem. No --clang AST import is requested; that is normal cppcheck usage and preserves an independent analyzer engine. The installed cppcheck2.22 help confirms --clang is an optional experimental parser. Absence of that option does not mean there is no useful parse/analysis.

**Reproduction:** a temp native TU includes `<required.hpp>` that defines FEATURE, with a definite null dereference inside #ifdef FEATURE. With the header absent from the compile record, the real script returns0/clean. With the include directory present, the identical source reports nullPointer and returns1. This proves missing-header/preprocessor context can hide own source paths. Even removing the suppression would not necessarily make missing includes block: the script does not enable missingInclude/information diagnostics. The fallback nanobind path is `/nonexistent`, making a failed interpreter/header lookup similarly unobservable until it causes some other error.

**Current limits:** current compilation entries provide real nanobind/MuJoCo/Python headers, and this audit does not prove a concrete present TU was skipped. Missing system-library models are a common analyzer limitation; prioritize missing required project/third-party context rather than assuming all standard includes must parse perfectly. A separate --check-config pass with carefully classified required includes can detect regressions.

### 13. cplusplus.Move is globally disabled — confirmed tradeoff, no specific present use-after-move established

**Source:** native/.clang-tidy:43 disables clang-analyzer-cplusplus.Move; check_analyzer.sh:47,51 disables cplusplus.Move in stable and alpha. Thus the dedicated CSA pass does not restore this checker, and real first-party use-after-move would also escape this check.

**Caller/code angle:** actual first-party drivetrain code contains std::move, and both drive_binding.cpp and rider/contact_binding.cpp instantiate nanobind's std::array caster. The code comments provide a specific third-party false-positive rationale. The global disable is broader than the cited caster false positive, but this audit did not re-trigger that specific historical false positive or identify an actual moved-from misuse in production. This is a documented analyzer coverage decision, not independently sufficient evidence of a correctness bug.

**Suggestion:** seek a narrow suppression/model or checker-enabled sweep of non-binding core TUs; retain the rationale until its underlying diagnostic is reproduced. Avoid claiming all move safety is unchecked: types/RAII/other compiler checks remain.

### 14. CXX_EXTENSIONS defaults to GNU++23 — confirmed mode, defect not established

**Source/build:** CMake sets STANDARD23/REQUIRED but never CXX_EXTENSIONS OFF. Actual compile_commands contain `-std=gnu++23`. Frontend sweep explicitly uses `-std=c++23`; -Wpedantic/-Werror and -Wlanguage-extension-token already reject many extensions.

**Assessment:** strict ISO mode would make intent clearer and better align frontends, but no actual GNU extension dependency or runtime difference was found. This is a portability/config consistency improvement, not a demonstrated functional defect.

### 15. FAST outside Debug / EXTENSIVE in Debug — confirmed; DEBUG layout justification wrong on this toolchain

**Source/build:** native/CMakeLists.txt:108-112 explicitly selects EXTENSIVE only for Debug and FAST otherwise. Current native/build/CMakeCache.txt has an EMPTY CMAKE_BUILD_TYPE, so even the unoptimized default local build uses FAST. Do not call it a release build just because its hardening mode is FAST. Actual compile_commands confirm the FAST definition.

**Toolchain angle:** local SDK26.5 libc++ `__configuration/hardening.h` and `__assert` show FAST enables valid input ranges and element access; EXTENSIVE adds nonnull/overlap/domain/allocator etc checks; DEBUG adds semantic/internal checks. FAST is not no hardening. Switching release to EXTENSIVE is a performance/diagnostics policy choice, not a proven required fix.

**Incorrect comment:** CMake says DEBUG changes libc++ layout and would cause ODR clashes. For this current libc++, local `__config:303-329,338-368` includes the hardening mode in `_LIBCPP_ODR_SIGNATURE`/ABI tags specifically to avoid collisions between differently configured inline methods. Native test programs compiled with FAST, EXTENSIVE, and DEBUG all print the identical sizes `vector<int>=24, vector<int>::iterator=8, string=24, span<int>=16`; DEBUG does not imply debug-iterator ABI switches. These measured/header facts contradict the blanket layout assertion. This does not prove every imaginable library configuration is ABI-compatible; it does support correcting this mode's rationale. Use the actual toolchain's documented performance/testing tradeoff.

### 16. -Wbitwise-instead-of-logical and -Wextra-semi absent explicitly — confirmed spelling, claimed diagnostic gap rejected

**Source:** native CMake does not spell either exact option. It does use -Wall/-Wextra/-Wpedantic and -Wextra-semi-stmt, and the GCC frontend explicitly uses -Wextra-semi.

**Actual compiler control:** Apple clang21 syntax-checking `bool c(){return a() & b();}` and `struct S {int n;;};` with the current umbrella flags emits both exact diagnostics (`-Wbitwise-instead-of-logical` and `-Wextra-semi`). Adding the two explicit options produces the same two diagnostics. In the production -Werror build they are errors. Counting exact option spellings therefore produces a false coverage finding here.

### 17. frontend hardcodes GCC16/SDK and re-discovers Python env — facts confirmed, portability limitation

**Source/config:** check_native_frontends.sh:5-10 hardcodes SDK26.5 and g++-16. It calls `uv run` to discover Python/nanobind; CMake's target passes only source and MuJoCo includes. By contrast check_odr accepts `${GXX:-g++-16}` and the CMake-resolved Python interpreter. A CMake build intentionally using a different Python interpreter or SDK need not be swept with that same context.

**Current environment:** g++-16 and the SDK exist on this host; current CMake-resolved interpreter and uv project interpreter refer to the same3.14 environment, so a current mismatch is not established. Hardcoded tool choice fails loudly when unavailable (unlike tidy), which agrees with the missing-tool rule. A compiler override and explicit context would improve portability/reproducibility; do not call a macOS-specific pinned project automatically broken for not supporting Linux.

## Coverage and test-selection defects

### 18. native_coverage.sh accepts a build argument but tests a different fixed directory — confirmed

**Source:** tools/native_coverage.sh:15,18,26-29 use caller BUILD_DIR, but :21 always sets NATIVE_TEST_BUILD_DIR=coverage. Native tests resolve that selector only under repo/native/build. Merely requiring the supplied directory basename be coverage (:9) does not fix a different parent directory.

**Reproduction:** copy script to isolated temp project and mock only cmake/uv/sysctl/xcrun. Give `/private/tmp/infra_audit_extra/custom/coverage`; build and report route there, while the test invocation logs selector=coverage with cwd the separate coverage-project repository. Real test loader code resolves that to coverage-project/native/build/coverage. Thus an allowed custom path can exercise an existing different instrumented module or fail; the requested build is not necessarily the one under test.

**Priority:** medium. Derive/validate the selected module path from BUILD_DIR or reject any path except the actual fixed supported directory before building.

### 19. .so glob merges multiple matches into one filename — confirmed latent robustness defect

**Source:** native_coverage.sh:28 assigns `SO=$(echo "$BUILD_DIR"/bike_native.*.so)` then passes `"$SO"` at :29. Zero matches remain a literal glob; multiple matches become a single space-separated filename.

**Reproduction/config:** isolated coverage fixture with cpython313 and314 extension files causes mocked xcrun to receive argc4: llvm-cov, report, ONE object argument containing both paths, and profile. Current real coverage directory has exactly one matching .so, so current reporting is not proved wrong. A stale ABI artifact from a Python upgrade can trigger it. Select the extension actually imported by the tests and enforce uniqueness/existence.

### 20. rider-forces standalone tests ignore sanitizer/coverage build selector — confirmed extra issue

**Source:** tests/reference/test_native_rider_forces.py:25 prepends regular native/build unconditionally; subsequent tests use pytest.importorskip and never assert extension __file__. Neighboring native test files validate/resolve NATIVE_TEST_BUILD_DIR and some assert the imported directory; docs/TESTING.md claims no fallback across native tests.

**Reproduction:** a fresh uv Python process with NATIVE_TEST_BUILD_DIR=coverage, execute this module via runpy then import bike_native; actual __file__ is regular native/build/bike_native.cpython-314-darwin.so. The coverage extension exists. Same routing defect applies to standalone asan/rtsan selection. A combined suite may appear to work when another test imported the selected extension earlier because Python caches it; that is import-order dependence, not a reliable selector contract.

**Priority:** medium/high for claims of sanitized coverage. Share selection and assert the module path in this file; ideally use a common test loader/preflight to remove collection-order effects.

### 21. full profile does not inherit native build/sweep chain — confirmed extra limitation

**Source/caller:** tools/run_tests.sh:53-54 full branch does nothing before invoking pytest; only native runs build+sweeps+import preflight. AGENTS.md asks full for physics/realtime and native for changes under native/tools. A full run after a native physics change can therefore run an existing .so without rebuilding it and without analyzer checks unless native was also run. The all-tests set does not imply the stronger native preparation contract.

**Assessment:** do not assert every full run is wrong: this task's controller deliberately runs native, and a user may run both profiles. Documentation should explicitly require both for overlapping scopes, or full should arrange the necessary native preflight. This is a verified control-flow limitation, not an observed stale-result failure in the current run.

### 22. documented native ASan launcher path is inconsistent — confirmed source limitation, not rerun broadly

**Source:** docs/TESTING.md correctly notes uv strips DYLD_INSERT_LIBRARIES and shows a direct interpreter recipe, but later claims `NATIVE_TEST_BUILD_DIR=asan` makes the native profile alone a valid ASan run. tools/run_tests.sh always builds/sweeps regular native/build (:40-46), pre-imports regular extension through PYTHONPATH (:47-49), and finally executes uv (:74); its ASAN_OPTIONS block only sets allocator/options, not the sanitizer runtime injection or correct selected preflight import.

**Assessment:** the script alone cannot establish a complete ASan run just by setting NATIVE_TEST_BUILD_DIR; explicit runtime activation and selected module preflight matter. This agent did not run the whole ASan suite or an additional build because the controller owns broad verification. Rider-forces selector reproduction above independently demonstrates that test-file selection also needs repair. Keep sanitizer instructions honest rather than assuming the regular native profile proves the selected instrumented build.

## Modernization claims: evaluate behavior/contracts, not feature counts

### 23. missing modernize/use-auto/nodiscard/noexcept/constexpr/mdspan is not a sufficient correctness finding

**Config/code facts:** .clang-tidy enables modernize-* broadly but explicitly excludes use-auto/use-nodiscard/trailing-return-type with rationales. Native headers already use nodiscard on many state/getter functions; constexpr constants and if constexpr appear in actual code. Core uses span, std::array, owning vectors, unique_ptr, variants/options, and designated initialization; absence of one C++23 facility is not evidence of unsafe or outdated design.

**Contract/caller angle:** functions such as grip_step/release_if_overloaded deliberately throw invalid_argument on bad finite/domain inputs, and other writers return allocated vectors or parse Python data; adding noexcept would convert legitimate errors/allocation failures into process termination. Public void/numeric mutation methods and optional-returning values require call-site meaning, not a blanket nodiscard policy. A pure small vector add/multiply could reasonably be constexpr/noexcept, but that offers documentation/compile-time use, not a measured behavioral correction here. mdspan would not own MuJoCo buffers or validate their model dimensions by itself; flat std::array and spans already encode useful extents/ownership.

**Stage constraint:** ADR discusses a future nonthrowing allocation-free hot core, whereas docs/TESTING.md explicitly says current per-call stages do not establish a complete allocation-free P4 tick or episode integration. Evaluate missing future properties against the implemented stage, and require a concrete bad call/state transition before assigning a correctness priority. Tiny pure-helper improvements can be considered when useful, separately from the verified gate/parser defects above.
