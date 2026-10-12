# Native C++ Contracts Implementation Plan

**Goal:** Enforce internal constructor, geometry, ownership, numerical ABI, and header contracts without gratuitous
C++23 rewrites.

**Architecture:** Python-free validators protect typed construction. Checked model-access helpers validate extents and
IDs at module boundaries. Shared numerical helpers encode ABI and exact oracle semantics; local enums/attributes make
established contracts visible to the compiler.

**Tech Stack:** C++23, MuJoCo, Accelerate CBLAS, CTest, clang/GCC analysis, ASan/UBSan, Python parity tests through uv.

**Spec:** [Native safety specification](../specs/2026-10-06-native-safety.md), S3 and S7.

## Global Constraints

- Use `uv` for every Python command.
- Retain Python >=3.13, CMake >=3.24, and nanobind >=3.1.0; native platform scope remains Apple arm64/Accelerate.
- Preserve valid-input numerical parity, including operation order, intentional saturation, and documented legacy
  behavior.
- Reject malformed inputs consistently in Python and C++; update both implementations in the same task.
- Compile first-party native code as ISO C++23 with extensions disabled.
- Use the existing MuJoCo 3.12.0 dependency; fail configuration if the error-interception ABI is unavailable.
- Keep alpha analyzer findings report-only; report analyzer execution failures separately from findings.
- Run `native` for native code or shell tooling changes; also run `full` for physics or realtime changes.
- Preserve existing AGENTS.md behavioral-stability requirements.
- Keep allocation, Python boxing, logging, and exception construction outside marked realtime regions.
- Make each task independently reviewable; record exact commands, selected binary, tool health, and test results.

---

## File map

Create `native/src/model_access.hpp`, `cblas_abi.hpp`, and `numeric_norm.hpp`. Extend V3's C++ contract executable; keep
tests independent of nanobind and Python where possible. Modify classes/helper boundaries in place rather than
redesigning every writer. Shared types from F1 carry no Python dependency. Build-context/header checks come from V1/V2.

Focused commands for every task:

```bash
uv run cmake --build native/build --target native_contract_tests
uv run ctest --test-dir native/build --output-on-failure
uv run python tools/native_checks.py --kind headers --build native/build
bash tools/run_tests.sh native
bash tools/run_tests.sh full
```

Run new constructor/extent/ABI controls before implementation and expect the specific contract failure; run again after
implementation and require all CTest cases pass with NDEBUG. ASan/UBSan selection follows the corrected V3 launcher
recipe, not the old regular-build import path.

### C1: Enforce typed construction and lifecycle invariants

**Files:** Modify `native/src/drivetrain/transmission.hpp`, `pedaling.hpp/.cpp`, `shifting.hpp/.cpp`, `motor.hpp/.cpp`,
`freehub.hpp/.cpp`, `policy_config.hpp`; modify writer constructors and `cruise.hpp`; test
`native/tests/test_contracts.cpp`.

**Interfaces:** Reuse F1/F2 `validate(const Config&)` overloads; constructors validate before use. Transmission
explicitly deletes copy/move constructor and assignment. Keep `std::unique_ptr<Transmission>` ownership. Initialize
every scalar/optional/sentinel member to its declared valid reset state.

- [x] Add compile-time copy/move checks and direct constructor invalid cases: stop time0, empty/one-row motor curve,
  unsorted/duplicate rows, rear teeth0, empty cassette, invalid tau/slew/taper, invalid damping maxima, and nonfinite
  air/rider/brake fields. Add zero-disable and valid empty-optional-curve controls. Tests call constructors directly,
  bypassing wire parsing.

```cpp
static_assert(!std::is_move_constructible_v<drivetrain::Transmission>);
static_assert(!std::is_move_assignable_v<drivetrain::Transmission>);
static_assert(!std::is_copy_constructible_v<drivetrain::Transmission>);
```

- [x] Run CTest/compile controls before changes; expect movable Transmission and invalid typed constructor failures. Add
  validator calls in inline constructors before storing or computing state, moving validation into out-of-line
  implementation when that keeps headers small. Avoid double physics normalization between parser and constructor.

```cpp
explicit PedalingPolicy(PedalingConfig config) : config_(std::move(config)) {
    validate(config_);
}
```

- [x] Initialize `CruiseState::target_speed_mps` explicitly, set absent cassette body to a named invalid ID rather than
  worldbody0, and validate before each optional scratch dereference. Use `engaged` only where its throwing contract is
  appropriate outside realtime regions; fixed core paths return status for missing state.
- [x] Run direct C++ cases under Release NDEBUG and ASan/UBSan, standalone headers, native/full policy tests.
  Review/commit when typed and parsed constructors accept the same valid domain and moved-from misuse is structurally
  impossible.

### C2: Checked extents, finite geometry, and model traversal

**Files:** Create `native/src/model_access.hpp`; modify `native/src/drivetrain/chain.hpp/.cpp`, `tyre/profile.hpp/.cpp`,
`rider/support_geometry.cpp`, `writers/tire.cpp`, `brake.cpp`, `rider_forces.cpp`, `suspension.cpp`; test C++ contracts
and existing chain/profile/contact tests.

**Interfaces:** `readonly_buffer(const T* data, mjtSize count) -> std::span<const T>`;
`mutable_buffer(T* data, mjtSize count) -> std::span<T>`;
`require_id(int id, mjtSize count, std::string_view field) -> void`;
`checked_product(std::size_t a, std::size_t b, std::size_t limit) -> std::size_t`. Helpers reject negative extents, null
with positive extent, and invalid products before range creation. Owning Stepper context carries paired model/data;
arbitrary raw pairing is a documented caller precondition.

- [x] Add direct C++ tests for negative counts, null/nonzero buffers, compile-time readonly element type,
  body/front/rear/frame IDs at -1/count, nq!=nv scalar geometry, NaN/inf rotation entries, empty/mismatched/nonmonotonic
  interpolation, invalid heightfield ID/rows/cols/data extent, invalid/cyclic parents, and lo>hi candidate interval.
  Construct invalid models only inside isolated tests; keep loaded-valid-model controls.

```cpp
static_assert(std::is_same_v<
    decltype(readonly_buffer(static_cast<const mjtNum *>(nullptr), mjtSize{0})),
    std::span<const mjtNum>>);
require_throws_invalid_argument([] {
    readonly_buffer(static_cast<const mjtNum *>(nullptr), mjtSize{-1});
});
```

Define `require_throws_invalid_argument` as a test helper that invokes a callable, records that exact exception, and
fails through `require` if it returns or throws another type.

- [x] Run cases before implementation. Add checked read-only/mutable buffer helpers and use read-only spans for
  shallow-const model/data pointer members. Reject nonfinite transforms before tolerance comparisons. Geometry
  angle/evaluate validate IDs and supported scalar topology before indexing; do not report nq>=nv valid models as a qpos
  overflow.

```cpp
if (a != 0 && b > limit / a)
    throw std::invalid_argument("heightfield extent exceeds data storage");
return a * b;
```

Cast validated positive rows/cols before multiplication; compare result to `nhfielddata` and offsets with checked
addition. Use exact signed-size limits from the engine version, not `int` intermediate products.

- [x] Validate interpolation widths and sorted abscissas once in owning profile construction; keep a private unchecked
  inner loop for validated profiles if needed. At exported helper boundaries check empty/mismatch explicitly. Bound
  parent traversal by nbody and validate each ID; where ordered-parent invariant applies, check `0 <= parent < body`
  instead of allocating a visited set. Candidate reserve validates ordered bounds or returns the declared empty
  interval, with one chosen contract and tests.
- [x] Add API comments to raw writers stating model/data pairing and single-context ownership. Do not fabricate an
  ownership check from dimensions. Use explicit owning context for any new API that accepts data not created by its
  Stepper. Replace raw Jacobian destination pointers at writer boundaries with `std::span<mjtNum>` and exact `3*nv`
  checks; engine ABI call receives `.data()` only inside the checked helper.
- [x] Run CTest, native/full geometry/profile parity, ASan/UBSan malformed helper probes, and allocation controls for
  parent walks. Review/commit when checks cover entry boundaries and valid kernels preserve bitwise results and intended
  allocation behavior.

### C3: One numerical ABI and matching numerical helpers

**Files:** Create `native/src/cblas_abi.hpp`, `numeric_norm.hpp`; modify `writers/resistance.cpp`, `tire.cpp`,
`tyre/profile.cpp`, `rider/contact_math.cpp`, `drivetrain/chain.cpp`, `writers/brake.cpp`, and `model_access.hpp`; test
C++ contracts and native contact/profile/brake/tire parity.

**Interfaces:** `blas::Order`, `blas::Transpose` enumerate supported CBLAS integer ABI values; common declarations use
the installed Accelerate ABI. `rider::matvec(const Mat3&, const Vec3&, blas::Transpose = blas::Transpose::no) -> Vec3`
replaces its opaque transpose bool. `numeric::python_vector_norm(std::span<const double>) -> double` is used only where
both current ports reproduce the same CPython vector-norm contract. Shared model helpers resolve names and check joint
types with explicit caller-supplied physical requirements.

- [x] Add compile/link ABI controls, `mjtNum` type assertion, row/column-major transpose examples, zero-length vector,
  signed zero, huge/subnormal values, near-cancellation, and repeated-value norm cases. Compare norm bytes to Python's
  actual oracle, not to an unrelated `hypot` formula.

```cpp
static_assert(std::is_same_v<mjtNum, double>);
enum class Transpose : int { no = 111, yes = 112 };
enum class Order : int { row_major = 101, column_major = 102 };
```

- [x] Run controls against existing implementations and preserve a corpus of exact expected bits. Include the real
  supported CBLAS header where warning/build constraints permit; otherwise declare the pinned supported ABI once in
  `cblas_abi.hpp`, with integer-width and symbol link checks. Remove all five TU-local declarations and magic
  order/transpose literals.
- [x] Extract the matching CPython norm implementation verbatim first; keep compensated sums and BLAS reductions
  separate where they have distinct oracle operation order. Reuse F2's finite validators; do not put general config
  validation in `pyfloat.hpp`, whose responsibility remains Python pow semantics.

```cpp
inline constexpr int kXYZ = 3;
inline constexpr int kRotationElements = 9;
inline constexpr int kQuaternionElements = 4;
```

Introduce stride constants in the focused model-access module or local owning types. Use them only where shape meaning
matches; a coefficient-table count is not a spatial dimension.

- [x] Consolidate brake/tire resolve-actuator/Jacobian code through model_access helpers after recording each existing
  topology/error contract. Give physical joint requirements explicit parameters and destination spans, retaining
  different permissible model types where needed.
- [x] Run ABI/link controls, all relevant bitwise tests, native/full and header analysis. Review/commit when handwritten
  ABI exists in one place and extracted math is byte-identical on its oracle corpus. Any helper with different rounding
  semantics stays separate with a documented name.

### C4: Compiler-visible API contracts and dead-state cleanup

**Files:** Modify `native/src/writers/drivetrain.hpp/.cpp`, `brake.hpp/.cpp`, `suspension.hpp/.cpp`,
`resistance.hpp/.cpp`, `rider_forces.hpp/.cpp`, `drivetrain/pedaling.hpp/.cpp`, `motor.hpp/.cpp`, `shifting.hpp/.cpp`,
`contact/laws.hpp`, `rtsan.hpp`, `native/.clang-tidy`; test header controls, direct contracts, and writer parity tests.

**Interfaces:** Local enums for closed profiled assist mode, shift slip mode, and fixed internal directions; serializers
keep existing Python strings. Profile-less custom assist labels remain owned strings at the config/FFI boundary. Remove
global `using drivetrain::DrivetrainWriter`; qualify consuming declarations. Keep logically mutable scratch explicitly
owned by the instance.

- [x] Add enum parsing/exhaustiveness controls and unknown-mode rejection only for profiled modes. Compile headers
  independently and nodiscard/noexcept controls for selected interfaces. Capture writer diagnostics/state outputs before
  dead-state removal so the test asserts external behavior rather than private structure.

```cpp
enum class ProfileMode { eco, tour, emtb, turbo };
double profile_gain(const drivetrain::MotorProfile &p, ProfileMode mode, double sensed) noexcept {
    switch (mode) {
        case ProfileMode::eco: return p.eco;
        case ProfileMode::tour: return p.tour;
        case ProfileMode::emtb: {
            const double fraction = std::min(1.0, std::max(0.0, sensed) / p.emtb_full_gain_at_nm);
            return p.emtb_low + (p.emtb_high - p.emtb_low) * fraction;
        }
        case ProfileMode::turbo: return p.turbo;
    }
    std::unreachable();
}
```

The config and sensed torque are prevalidated by F2. Keep open custom-label semantics outside this closed switch; the
existing MotorProfile structure remains the typed source of gains.

- [x] Run controls before changes. Add direct standard includes for every used type/function and remove global alias.
  Mark meaningful result-producing math/state queries nodiscard; validation calls still execute when their results are
  discarded. Mark math constexpr only where supported operations and compiler evaluation preserve runtime behavior;
  noexcept only where the implementation and its dependencies cannot throw.
- [x] Remove write-only brake/suspension last_ and writes after verifying `rg` references and state emitters. Retain
  rider last_ because it is read. Name fixed force-component positions with an enum/index helper rather than relying on
  emplace order. Use explicit transpose enum from C3, named shape constants, and checked optional access appropriate to
  the region.
- [x] Document one-instance-per-context scratch semantics and the GIL-held binding contract. Const methods may update
  private scratch if that ownership is clear; change constness when it falsely advertises observable purity. Keep
  exceptions outside nonblocking kernels; invalid core state returns status and is translated by the caller.
- [x] Run header/enum controls, native/full writer tests, and RTSan marked-region checks. Review/commit when
  compiler-visible contracts reflect actual behavior, public serialization is stable, dead state is absent, and no
  attribute causes termination where recovery was required.
