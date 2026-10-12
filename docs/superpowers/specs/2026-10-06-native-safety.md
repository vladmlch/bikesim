# Native safety, strict validation, and trustworthy verification

This is the implementation contract for the [master plan](../plans/2026-10-06-native-safety-master.md). The user approved strict rejection of malformed inputs in **both Python and C++**, while preserving numerical parity for valid inputs. The external review is a hypothesis list, not evidence. The audit baseline is commit `9f3a4442bc57312877c062299055813ec06ff4f5`; subsequent executors must check current code before reproducing a finding.

## Global constraints

- Use `uv` for every Python command.
- Retain Python >=3.13, CMake >=3.24, and nanobind >=3.1.0; native platform scope remains Apple arm64/Accelerate.
- Preserve valid-input numerical parity, including operation order, intentional saturation, and documented legacy behavior.
- Reject malformed inputs consistently in Python and C++; update both implementations in the same task.
- Compile first-party native code as ISO C++23 with extensions disabled.
- Use the existing MuJoCo 3.12.0 dependency; fail configuration if the error-interception ABI is unavailable.
- Keep alpha analyzer findings report-only; report analyzer execution failures separately from findings.
- Run `native` for native code or shell tooling changes; also run `full` for physics or realtime changes.
- Preserve existing AGENTS.md behavioral-stability requirements.
- Keep allocation, Python boxing, logging, and exception construction outside marked realtime regions.
- Make each task independently reviewable; record exact commands, selected binary, tool health, and test results.

## Required behavior

### S1. Verification must prove that it examined the intended code

Every mandatory sweep checks tool availability, complete first-party translation-unit selection, valid dependency paths, process return codes, and diagnostics. Zero selected units, duplicate units, missing units, malformed compilation databases, tool crashes, and missing required project headers fail. Header diagnostics work after moving the checkout. Stable findings block; experimental findings print with an explicit report-only status. Tool failure never becomes a clean finding result.

CMake supplies the authoritative source manifest and build context. Wrappers accept documented compiler overrides and use that context rather than probing inaccessible caches or substituting `/nonexistent`. The default build and sanitizer/coverage builds run the same applicable gates. Analyzer suppressions identify checker, affected origin, reason, and expiry condition; first-party move analysis remains enabled.

Retain the project's Python >=3.13, CMake >=3.24, and nanobind >=3.1.0 floors. Add an explicit native dependency group containing MuJoCo ==3.12.0 so a private-ABI-dependent native build is reproducible from uv.lock; Python-only support outside that group retains its current package requirement. Native platform scope remains the existing Apple arm64/Accelerate toolchain. Respect an explicitly supplied SDK/toolchain and check its compatibility rather than unconditionally overwriting it with a machine path. Dependency/ABI upgrades require new interceptor, numerical, sanitizer, and analyzer controls.

### S2. Tests must load the selected native artifact

One loader resolves `NATIVE_TEST_BUILD_PATH`, with compatibility for existing `NATIVE_TEST_BUILD_DIR` values. Conflicting selectors fail. Native tests verify the actual imported extension path, including tests currently rewriting `sys.path` or using `importorskip`. Native/full profiles build and sweep the selected directory before tests. Sanitizer execution injects its runtime into the Python process after `uv` launches it. Coverage reports use the same extension and build directory as the test process, with exactly one resolved extension artifact.

### S3. Input contracts are explicit and shared

At public config/state/control boundaries, accept documented mappings, ordered sequences, exact booleans, finite real scalars, and integers with the required range. Reject boolean values in numeric fields and numeric arrays. A NumPy numeric scalar is accepted where its Python implementation accepts it. Reject unordered sets, iterators without the promised sequence interface, nonstring keys, unknown keys, missing required keys, incorrect widths, and unsupported discriminators with `ValueError`. Allocation failures retain their normal memory-error meaning.

Disabled writers remain explicit empty sections or absent optional sections according to the existing schema. A present nonempty section must satisfy its complete schema. Intentional click/token clamping remains documented normalization of integer inputs; validation precedes normalization. Profile-less custom assist modes remain valid; profiled assist modes form a closed four-mode set.

Extract Python-free native config types and validators so typed C++ construction enforces the same invariants as FFI parsing. Constructors and mutators enforce invariants, including after public Python attributes/specs have been changed. Validate before state mutation. Reuse scalar/type helpers without erasing field-specific domains.

Introduce native config schema 2 to carry an explicit fork `legacy_behavior` flag. New config producers emit schema 2. Keep an explicit schema-1 migration: infer the historical fork legacy branch when resolved `hbo_start_mm == 160.0`; the ambiguous modern 180 mm fork has identical resolved damping parameters, so preserve those parameters rather than reconstructing origin. Require schema-2 callers to provide the flag. Document this migration and test old project configurations and hand-written schema-1 fixtures. Do not rewrite golden numerical expectations merely to hide a regression.

### S4. Numeric domains

| Area | Required domain and retained behavior |
|---|---|
| Air spring | Finite positive diameters, chamber lengths, total travel, gamma, and atmospheric pressure; finite nonnegative token volume and gauge pressure; integer token counts; evaluated chamber volumes strictly positive before fractional powers. Keep existing integer token clamping. Do not reject a usable spring solely because some unreachable declared travel would exhaust a chamber. |
| Damper | Positive maximum click denominators and velocity knees; finite nonnegative damping constants with ordered minima/maxima; integer click inputs retain documented clamping; positive travel/stroke. Modern HBO start lies within its working travel; legacy HBO can start beyond travel but remains finite and nonnegative. Persist the fork legacy flag in Python. |
| Brake/rider/end stops | Finite nonnegative physical stiffness, damping, preload and torque ceilings; finite geometric offsets; positive taper/length denominators where used. Boolean policy fields use exact booleans. |
| Coil | Finite positive spring rate, stroke, bumper length and peak; nonnegative preload; bumper length no greater than stroke; exact legacy flag. Retain existing legacy force formula. |
| Resistance | Finite nonnegative crr/rho/cda; positive rolling taper; finite vec3 wind/application point with their planar y component zero; valid resolved body IDs. |
| Tire/material/surface | Positive radial/tangent stiffness and relaxation length; nonnegative damping/pressure/friction; nonblank provenance; load range 0 <= lower < upper; significant delta nonnegative, fraction in [0,1], normal angle in (0,180). Positive surface coefficients/speed with slide friction <= peak; ordered, nonoverlapping section intervals 0 <= start < end. Native writer remains compliant_2d; configured/track surface modes remain supported. |
| Pedaling | Positive stop time, nonnegative coast/filter/slew values where zero disables a feature; ordered resume/coast thresholds; positive mash cadence when mash torque is enabled. |
| Shifting/gearing | Nonempty unique cassette teeth, each at least 3; valid front/rear teeth; positive pitch and cadence thresholds; ordered cadence band; cooldown/cut timing consistent; torque factor in [0,1]; finite nonnegative smoothing/slip limits; closed slip mode. Enabled rear gear belongs to cassette. |
| Assist/battery/freehub | Positive time/slew/taper denominators and profile full-gain torque; finite nonnegative physical gains, torque/power/energy/loss limits; ordered nonnegative curve rows with at least two samples; taper does not exceed cutoff; positive hub stiffness and nonnegative damping. Keep intentional zero-disable values. |
| Model topology | Cruise longitudinal joint is a slide joint with the supported longitudinal axis; wheel-spin joints are hinge joints with supported wheel axis. Geometry helpers validate IDs, scalar topology, and finite transforms. |
| State/runtime | Exact shapes, finite qpos/qvel/act/warmstart/control/time; time is nonnegative. Validate every control before advancing a policy. Tire unset-state sentinels are allowed only at explicitly named sentinel fields, with relational checks. |

Tire interval conversion uses binary64 `time / timestep`, then half-even rounding independent of the process floating-point rounding mode. Both implementations accept the resulting interval in `[0, INT64_MAX]`. Nonfinite or negative input time is `ValueError`; a finite time whose interval exceeds this domain is `OverflowError`. Replace the approximate `9.2e18` cutoff with an exact conversion boundary. Preserve Python integer behavior elsewhere. Cruise reads the active model timestep on each calculation; if a timestep setter is introduced, reject invalid values and updates during pending actuation.

Before publishing forces, controls, energy, geometry, or model coefficients, require finite derived values. Finite inputs that exceed the representable computation domain raise `OverflowError` in both implementations before live writes. Keep existing domain-specific arithmetic exceptions for geometry failures. Legacy HBO retains its historical negative-denominator/clamped branch; an activated zero-denominator branch is rejected with `ValueError` before division.

### S5. Fatal engine errors are recoverable at the FFI boundary

Use a pinned, thread-local MuJoCo interceptor compatible with stock Python bindings. The engine handler is nonreturning. A narrow C-compatible operation frame uses fixed storage and does not jump across C++ destructors or Python frames. Restore the prior thread-local handler on success and failure. Never throw a C++ exception through engine C frames.

Catch fatal errors from loading, data creation, forward, step, reset, and model-constant recomputation, including calls made by writers and private adapters. Translate outside realtime code into `mujoco.FatalError` (also exported by `bike_native`). On runtime engine failure, poison the owning Stepper and reject subsequent mutating calls until explicit reset/reinitialization succeeds. Do not promise rollback of partially executed engine code. Construction failure cleans owned resources and leaves no published object.

Pinned implementation references: [Python interceptor](https://raw.githubusercontent.com/google-deepmind/mujoco/3.12.0/python/mujoco/errors.h), [engine log/error mechanism](https://raw.githubusercontent.com/google-deepmind/mujoco/3.12.0/src/engine/engine_util_errmem.c), [model loader](https://raw.githubusercontent.com/google-deepmind/mujoco/3.12.0/src/engine/engine_io.c). Validate these against the installed binary before adopting private symbols.

### S6. Failed stateful operations do not commit partial updates

Validation, geometry, diagnostic allocation, and Python-result allocation happen before commit. Transmission stages rear teeth, ratio, coefficients, ranges, prepared geometry, and linearization state together. Drivetrain stages pedaling, shifting, assist, ctrl, diagnostics, time, pending actuation, and transmission updates together. Settlement stages battery debit, telemetry, and pending removal together. Retry after rejected input or allocation failure has the same result as one successful call from the original state.

Probe operations leave policy state, model coefficients, applied controls, telemetry, and clocks unchanged. Restoring state validates relational invariants: gear/ratio/prepared coefficients agree; pending timestamps and battery/energy state are consistent; optional sentinel groups have valid combinations. Python oracle operations obey the same failure atomicity. Fatal engine failures follow S5's poison contract instead of the stronger rollback promise.

### S7. Ownership and internal C++ safety

Make array-property `reference_internal` policy explicit and correct the misleading lifetime comments. Existing eight properties already retain Stepper through nanobind; there is no immediate temporary-Stepper UAF to fix. Arena arrays are documented as a snapshot of the exposed shape, valid only under the declared storage-stability contract; use owned copies for variable-size `efc_force` if stability across forward/step cannot be demonstrated. Test temporary owners, owner deletion, and runtime arena changes.

Capsule helpers hold allocation in `unique_ptr` until capsule construction succeeds. Share those helpers across binding families. Private core adapters accept a closed kind set and operate on isolated scratch model/data or reject live writer use before mutation. Replace raw address test instrumentation with owning opaque tokens or nonaddress identity/capacity diagnostics; no production FFI consumer depends on the current addresses.

Internal raw model/data pairing is a documented C++ precondition unless the interface can carry an owning context. MuJoCo does not expose a dependable owner identity in arbitrary `mjData*`; comparing dimensions alone does not prove pairing. Stepper and owned contexts keep that invariant. Bounds, negative extents, scalar topology, finite rotations, interpolation lengths, raster products, parent traversal, and sorted candidate intervals are checked where untrusted input enters. Convert shallow-const struct access into explicit read-only spans at read-only boundaries.

Transmission is explicitly noncopyable and nonmovable under its current unique ownership. Shared CBLAS declarations use the supported Accelerate ABI and `static_assert(std::is_same_v<mjtNum, double>)`. Consolidate only helpers with identical contracts, including exact norm rounding. Headers compile independently. Use local closed enums for profiled modes/slip directions, named strides, explicit transpose enums, and span extents. Add `nodiscard`, `constexpr`, and `noexcept` only where they express proved behavior. Remove write-only brake/suspension `last_`; document scratch ownership and concurrency.

### S8. Runtime and benchmark scope

Prepare owning scratch at construction for rider, tire, resistance, and suspension. Provide `compute_into`/typed component interfaces used by the owning runtime, with Python convenience allocation outside the core. Replace tick-time map/string rebuilding with typed diagnostics and stage banks. Verify zero core allocations after warmup for every supported backend and advance/probe/settle path, including failure-status paths. RTSan regions return status; they do not construct allocating exceptions. Full realtime scheduling remains the existing P4 project; this work removes known allocation obstacles and must not claim an allocation-free Python tick.

Integrate the prototype bench into CMake, warning/analyzer/sanitizer selection, and CTest. Use model-sized buffers, strict CLI parsing, checked data creation, and model/state compatibility checks. Reject corrupted headers, mismatched models, truncated payloads, zero divisors, overflow, and unsupported workload/model combinations before executing kernels. Generate `model.mjb`, `state.npz`, and `state.bin` from one state with a versioned header and model fingerprint; use atomic replacement of a complete artifact set. Align paired Python/C++ workloads and finite checksum expectations. Retain the old operation-mix benchmark only under an explicitly approximate name. Remove the tracked platform binary after a reproducible build command exists.

## Evidence and disposition ledger

This ledger is self-contained so executors need not retain untracked audit output. “Internal” means a C++ precondition hardening task, not a demonstrated Python exploit. Every row has an implementation owner in the master plan.

| ID | Finding and cross-check | Disposition |
|---|---|---|
| A01 | Invalid contact solver99 exits native process; stock bindings raise FatalError. Missing/empty/garbage MJB controls survive. | Recover fatal engine path; retain normal load errors. |
| A02 | All eight property views increment owner refcount; nanobind property policy confirms reference_internal. Stored efc shape stays stale. | Correct comments/policy; verify storage or return copy for arena view. |
| A03 | Bench nv33 paired and nv65 glue overflow under ASan; current saved model nv26 is safe. Header/model mismatch writes out of bounds. | Integrate and harden prototype. |
| A04 | Zero rewind gives UBSan division-by-zero; arm64 signal is not consistently SIGFPE. Negative header throws length_error. | Validate CLI/state before work. |
| A05 | buffer preserves T const; const mjData pointer still exposes mutable pointer members. Negative count is unchecked. | Explicit read-only boundary spans and extent guards. |
| A06 | Geometry IDs/transforms unchecked internally; bindings resolve IDs. Valid nq>=nv; scalar writer topology checked. | Check helper contracts without alleging exposed qpos over-read. |
| A07 | Profile construction protects interpolation; loaded corrupt heightfield IDs/parents are rejected by engine. Raw mutations remain unsafe. | Defensive helper validation and checked raster multiplication. |
| A08 | Candidate lo>hi is excluded by current sorted-profile caller; arbitrary model/data mismatch has no Python route. | Prove/document callers; explicit owned context for future APIs. |
| A09 | Moved-from Transmission traps before its alleged null mj_setConst; production never moves it. Empty curve is parser-protected. | Enforce class construction and ownership invariants. |
| A10 | Valid geometric shift 24→1000 commits ratio before geometry failure. | Stage and atomically commit Transmission. |
| A11 | Negative effort ceiling and unsettled retries advance effort before throwing; Python shares pending-guard mutation. Injected allocation failure advances state. | Shared atomic compute/probe contract. |
| A12 | Sparse restored last={} plus allocation failure after draw causes repeated battery debit; ordinary ideal settle allocates zero. | Shared atomic settlement and fault-injection tests. |
| A13 | Raw new before capsule creation has conditional leak in binding, contact, and drive helpers. | Shared RAII capsule helper. |
| A14 | Negative air chamber masks failure as native -0.0; zero damper max_lsc produces NaN vs Python exception. Both lack complete validation. | Strict common numeric domains. |
| A15 | Brake NaN, rider negative parameters, legacy HBO finiteness, and unchecked state are real holes; many other parsers already validate. | Complete field-by-field validation. |
| A16 | Profiled motor parser is closed; profile-less custom modes are deliberate. Width error names wrong API. | Retain custom mode; correct diagnostics. |
| A17 | Tire NaN time exception differs; FE_UPWARD changes nearbyint tie; huge finite time exceeds native integer range. | Shared explicit clock/rounding domain. |
| A18 | Unknown keys accepted, nonstring keys leak bad_cast. Set/iterator vectors and 'no' boolean are rejected already. Numeric bool slips through other readers. | Exact key/type checks and ValueError translation. |
| A19 | _drive_core unknown kind chooses ideal and changes live model coefficients while writer cache stays old. Raw address hook is instrumentation, not proven UAF. | Isolate/guard private hooks and remove address contract. |
| A20 | Tidy failure-only mocks and zero-CSA-selection mocks pass; relocated checkout loses real header diagnosis. | Fail-closed sweeps and portable filters. |
| A21 | Missing include hides a conditional cppcheck finding; built-in AST still exists. Global Move exclusion has third-party rationale. | Required build context and scoped suppression. |
| A22 | Five CBLAS declarations, duplicated FFI/norm/validation/resolve helpers; no wrong ABI found. | Consolidate only matching contracts. |
| A23 | Representative compute has 54 native allocations; other writers allocate scratch and strings. P4 deferral is documented. | Owning scratch, typed telemetry, measured allocation budget. |
| A24 | Mutable const scratch is not thread-safe; Python bindings retain GIL. brake/suspension last_ is write-only. | Document context ownership; remove dead telemetry. |
| A25 | Header standalone compilation works, but some types/includes are transitive. Cruise target initialized by current constructors. Blanket noexcept/nodiscard rationales are false. | Selective hygiene and explicit initialization. |
| A26 | Native CMake uses gnu++23; strict frontend already c++23. FAST is default outside Debug; hardening layout comment is inaccurate. | ISO configuration and measured EXTENSIVE policy. |
| A27 | Current warning umbrellas already catch disputed bitwise and extra-semicolon classes. Frontend missing build dir returns clean via process substitution. | Diagnostic controls and tool/manifest health rather than flag counts. |
| A28 | Cruise caches timestep while Python reads live; hinge root_x and wrong wheel-spin topology accepted by both. | Shared physical topology checks and live timestep. |
| A29 | dump_model regenerates npz but not bin; binary has absolute dylib; heavy checksum NaN and workload mismatch. | Reproducible artifacts, workload labels, finite controls. |
| A30 | Coverage custom build tests regular path; extension glob is unchecked. Rider test overrides selected build. Full omits native preflight; ASan uv environment recipe is incomplete. | Unified loader, profiles, and coverage selection. |
| A31 | Valid legacy fork travel150/start160 accepted by Python but rejected native; Python fork loses legacy flag. | Schema-2 flag and explicit schema-1 migration. |
| A32 | Sparse restore accepts ratio/gear inconsistency; optional numeric and prepared-array bool accepted. | Shared relational restore and scalar/array type contract. |
| A33 | Alpha analyzer execution crash can disappear even though findings are intentionally report-only. | Separate health from finding severity. |

Optional modernization (zip, mdspan, optional-for-unique_ptr, broad enum conversion) is not a completion requirement. Adopt only when a task shows a concrete safety or maintainability benefit and preserved numerical behavior. No source-wide feature-count migration is authorized by this spec.

Successful standalone prepare calls currently advance policy without recording the compute clock; the normal caller uses prepare then compute with the returned state. E3 must document and test that two-phase protocol. Rejected calls and retries are atomic; redefining repeated successful prepares as idempotent is a separate API change, not an assumed fix.
