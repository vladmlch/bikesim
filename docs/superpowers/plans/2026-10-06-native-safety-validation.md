# Strict FFI and Physics Validation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:
> executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reject malformed public inputs consistently while retaining all valid numerical behavior and documented legacy
branches.

**Architecture:** Separate Python-free config types and numeric invariants from nanobind readers. Share strict boundary
readers and capsule ownership. Version the missing fork metadata explicitly and test Python/native domains and clocks as
paired contracts.

**Tech Stack:** uv, pytest, NumPy, Python dataclasses, C++23, nanobind, MuJoCo.

**Spec:** [Native safety specification](../specs/2026-10-06-native-safety.md), S3–S4 and S7.

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

## File map and shared test helpers

Create `native/src/config_types.hpp`, `validation.hpp`, `binding_readers.hpp`, and `binding_arrays.hpp`; leave parsing
in existing binding/config files. Add `tests/reference/test_native_input_contracts.py`, `test_native_view_contracts.py`,
`test_native_clock_contracts.py`, and `native_safety_helpers.py`. The helpers provide `drive_pair(tmp_path, **kwargs)`
by calling the existing `test_native_drivetrain.pair`; `suspension_pair(tmp_path)` uses the existing `_golden` helper
and `tools.native_config.project`; `freeze(value)` recursively converts arrays into `(dtype.str, shape, tobytes())` and
dicts/lists into immutable structures, retaining float bit patterns. This is test-only factoring, not a production
adapter.

Focused commands, after writing the named tests:

```bash
uv run python -m pytest tests/reference/test_native_input_contracts.py -q
uv run python -m pytest tests/reference/test_native_view_contracts.py -q
uv run python -m pytest tests/reference/test_native_clock_contracts.py -q
uv run python -m pytest tests/reference/test_native_state_restore.py tests/reference/test_native_state_fuzz.py -q
bash tools/run_tests.sh native
bash tools/run_tests.sh full
```

Use V3's selected-build loader for each command. New rejection/clock regressions are red before their changes; existing
owner/valid-golden controls stay green throughout. After implementation all focused tests and required profiles pass.

### F1: Strict readers, typed config separation, and schema 2

**Files:** Modify `native/src/config.hpp`, `native/src/drivetrain/drive_binding.cpp`, `policy_binding.cpp`,
`policy_config.hpp`, `tools/native_config.py`, `src/bike_sim/physics/damper.py`; create config types/readers/validation
headers and input-contract tests/helpers.

**Interfaces:** `wire::finite_real(nb::handle, std::string_view) -> double`;
`wire::integer(nb::handle, std::string_view) -> std::int64_t`; `wire::boolean(nb::handle, std::string_view) -> bool`;
`wire::exact_keys(const nb::dict&, std::span<const std::string_view> required, std::span<const std::string_view> optional, std::string_view path) -> void`.
`parse_drive_policy_config(const nb::dict&, std::span<const std::string_view> extra_root_keys = {}) -> DrivePolicyConfig`
allows the full drive parser's known root extension set without treating unrelated keys as valid.

- [x] Write recursive unknown-key tests for every root/section/subsection, nonstring keys, required-key omissions,
  empty/disabled sections, unsupported schema/kind/mode, exact bool rejection, numeric bool rejection, NumPy numeric
  scalars, list/tuple acceptance, and set/iterator rejection. Enumerate actual config/state keys from the emitters and
  parsers into typed constant arrays; generate one mutation case per dictionary path.

```python
def test_unknown_root_key_is_error(tmp_path):
    from native_loader import load_native
    path = tmp_path / "empty.mjb"
    model = mujoco.MjModel.from_xml_string("<mujoco/>")
    mujoco.mj_saveModel(model, str(path), None)
    with pytest.raises(ValueError, match="config.suspenson"):
        load_native().Stepper(str(path), {"schema": 1, "suspenson": {}})
```

- [x] Run `uv run pytest tests/reference/test_native_input_contracts.py -q`; expect current unknown-key/bool/error-class
  failures. Move typed structs into `config_types.hpp` without Python includes; move finite/nonnegative/positive helpers
  into `validation.hpp`. Readers perform explicit type checks and translate cast failures into field-path
  `std::invalid_argument`; preserve genuine allocation exceptions.

```cpp
inline bool boolean(nb::handle value, std::string_view path) {
    if (value.ptr() == Py_True) return true;
    if (value.ptr() == Py_False) return false;
    throw std::invalid_argument(std::string(path) + ": expected bool");
}
```

Implement numeric readers using Python/NumPy real/integral classification consistent with `physics/checks.py`; exclude
bool and np.bool explicitly before conversion. Ordered fixed sequences have exact width; bool-containing numeric
sequences fail before ndarray force-casting can hide their original type.

- [x] Apply exact-key validation at every config/control/result/state dictionary, respecting optional restore fields
  intentionally allowed today. Add required/optional schema constants next to each emitter/parser pair. Policy parsing
  validates either its own root or the explicitly supplied drive root union. Preserve existing mutually exclusive
  topology rules and disabled-writer conventions.
- [x] Validate resolved Python projection inputs before `float`, `int`, `bool`, or `str` coercion. Test a mutated
  resolved boolean flag containing a string and a numeric field containing True; project must reject the original
  malformed value rather than emit an apparently valid wire dictionary. Keep explicit conversion only after the shared
  type/domain checks succeed.
- [x] Persist `Charger3Damper.legacy_behavior`, add native `Charger3Config::legacy_behavior`, emit schema2 with that
  flag, and migrate schema1 according to the spec. Test legacy travel150/start160, modern travel180/start160, modern
  shorter travel, project output, unknown schema, and existing schema1 fixtures. Check inferred schema1 behavior uses
  resolved parameters without re-running a different default branch.
- [x] Run input tests, native/full profiles, and standalone config-header C++ compilation. Review/commit when key/type
  matrices match between Python/native and both schema versions preserve valid behavior. Update projection docstring and
  TESTING.md with schema2 and migration rules.

### F2: Shared numeric domains and physical topology

**Files:** Modify `src/bike_sim/physics/checks.py`, `air_spring.py`, `damper.py`, `coil_shock.py`, `model_config.py`,
`physical_config.py`, `pedaling.py`, `shifting.py`, `motor.py`, `motor_profile.py`, `battery.py`; modify corresponding
`native/src/drivetrain/*.cpp`, `native/src/writers/suspension.cpp`, `brake.cpp`, `rider_forces.cpp`, `cruise.cpp`, and
`config_types.hpp`; modify `src/bike_sim/sim/ride/braking.py`, `cruise.py`, `rider_forces.py`; test input contracts and
existing writer/policy tests.

**Interfaces:** Python `checks.scalar(value, name, *, minimum=None, positive=False) -> float` remains the central finite
scalar interface; add exact integer/bool helpers where absent. C++ overloads `validate(const AirSpringSpec&)`,
`validate(const DamperCore&)`, and each typed config return `void` and throw `std::invalid_argument` with field paths.
Constructors call them after candidate config creation and before engine/model mutation.

- [x] Create parameterized invalid-domain cases from every S4 row: NaN/±inf, zero denominators, negative physical
  quantities, inverted min/max, invalid cassette/profile/timing relationships, unsupported axis/type, and
  bool-for-number. Add valid controls for zero-disable fields, clipped clicks/tokens, and legacy HBO beyond travel.
  Include finite extreme parameters that overflow a force/energy intermediate and require OverflowError before
  publication. Keep current resistance/tire/coil validators while closing type gaps; this task does not add a new native
  tire backend.

```python
@pytest.mark.parametrize("length", [-5.0, 0.0, float("nan"), float("inf")])
def test_air_chamber_rejected(length):
    from bike_sim.physics.air_spring import AirSpringSpecs, ForkAirSpring
    with pytest.raises(ValueError, match="neg_chamber_length_mm"):
        ForkAirSpring(AirSpringSpecs(neg_chamber_length_mm=length))
```

Run the same values through a projected native suspension config. Mutate a valid Python damper's `max_lsc` to0 before
force calculation to verify evaluation validates mutable attributes as well as constructor arguments.

- [x] Run the new matrix to prove masked-air and zero-damper cases fail. Implement validators using the spec's exact
  domains; retain the original formula order after validation. Check initial and evaluated chamber volumes before
  fractional powers. Preserve current token/click integer saturation and pressure-setting normalization where
  documented, with finite/nonnegative input checks first.

```cpp
inline double positive(double value, std::string_view path) {
    if (!std::isfinite(value) || value <= 0.0)
        throw std::invalid_argument(std::string(path) + ": expected finite positive value");
    return value;
}
```

- [x] Validate every public Python constructor/setter and native typed constructor, including AirSpringSpec, DamperCore,
  brake, rider, end stops, legacy HBO, profile curves, and policy configs. Zero smoothing/coast tau stays allowed only
  where the current implementation has an explicit disabled branch. Full-gain torque, stop time, damping knees, and
  ratio denominators stay positive. Preserve legacy HBO's negative-denominator/clamped result where historically
  defined; reject an activated zero-denominator branch before division in both implementations. Reject nonfinite derived
  force/energy/control values before publication with the shared OverflowError contract.
- [x] Require cruise `root_x` slide/longitudinal axis and wheel-spin hinge/supported axle in Python/native model lookup.
  Build minimal models with hinge root_x, ball/free spin joints, wrong axes, missing actuator target, and valid scalar
  topology. Reject before attaching the writer; preserve model/data on construction failure.
- [x] Run focused writer/policy tests plus `native` and `full`. Review/commit only when every S4 field is either tested
  and validated or explicitly documented as intentional normalization; valid modern and legacy golden components remain
  bitwise equal.

### F3: Runtime/state validation and relational restore

**Files:** Modify `native/src/stepper.cpp`, `binding.cpp`, `drivetrain/drive_binding.cpp`, `policy_binding.cpp`,
`writers/tire.cpp`, `drivetrain/transmission.cpp`; modify corresponding Python state/control entry points in
`src/bike_sim/sim/ride/drivetrain_forces.py`, `tire_forces.py`, `cruise.py`, and `src/bike_sim/physics/*` policy restore
methods; test `test_native_input_contracts.py`, `test_native_state_restore.py`, `test_native_state_fuzz.py`.

**Interfaces:** `validate(const DriveSnapshot&, const DriveConfig&, const mjModel&) -> void`; each owning operation
validates its declared inputs before staging. Change `check_width` to
`check_width(std::span<const double> values, std::string_view api, std::string_view field, mjtSize expected) -> void`;
maintain Stepper's existing public method signatures. Native/Python restore validates a complete candidate snapshot
before applying any member.

- [x] Test wrong shapes, NaN/inf in every numeric state/control field, negative time, numeric bool elements, optional
  bool-as-slip/ceiling, invalid pending state, partial tire sentinel groups, and gear/ratio/prepared-J inconsistencies.
  Keep named all-unset tire fields valid and test their first advance.

```python
def test_state_rejection_preserves_buffers(tmp_path):
    _, data, _, native = drive_pair(tmp_path)
    before = tuple(a.copy() for a in (native.qpos, native.qvel, native.ctrl))
    qpos = native.qpos.copy()
    qpos[0] = np.nan
    with pytest.raises(ValueError, match="set_state.qpos"):
        native.set_state(qpos, native.qvel, data.act, np.zeros_like(native.qvel), 0.0)
    for expected, actual in zip(before, (native.qpos, native.qvel, native.ctrl)):
        assert expected.tobytes() == actual.tobytes()
```

Add cross-buffer alias controls for qpos/qvel/warmstart to retain current staging behavior.

- [x] Run focused cases and capture current nonfinite acceptance and bool/coefficient inconsistency. Validate type and
  finiteness before copying/resetting. Snapshot aliased inputs only after validation and before any engine reset.
  Control validation precedes policy advance; errors name `set_inputs` rather than `set_state`.

```cpp
for (double value : qpos) {
    if (!std::isfinite(value))
        throw std::invalid_argument("set_state.qpos: expected finite values");
}
```

- [x] Define restore invariants from the emitter's legitimate states: selected gear agrees with ratio/front teeth;
  prepared coefficient topology agrees with selected transmission; pending request/omega/dt fields are finite with
  positive dt; clocks have consistent ordering; battery energy stays within configured bounds. Reject inconsistent
  sparse snapshots instead of reconstructing hidden unrelated values. Preserve legitimate optional telemetry omissions
  until E3 converts telemetry storage.
- [x] Mirror checks in Python restore/runtime methods. Test roundtrip state from reset, probe, advanced-but-unsettled,
  settled, shifted, clutch and rotor topologies. Fuzz individual fields and relationships using existing fuzz harness;
  compare exception classes and unchanged state.
- [x] Run focused tests, native/full, and selected ASan state smoke tests. Review/commit when invalid input never
  mutates state and every emitted valid snapshot restores successfully in both implementations.

### F4: Explicit view ownership, capsule RAII, and private hooks

**Files:** Create `native/src/binding_arrays.hpp`; modify `native/src/binding.cpp`, `rider/contact_binding.cpp`,
`drivetrain/drive_binding.cpp`, `stepper.hpp/.cpp`; test `tests/reference/test_native_view_contracts.py`, existing
contact/drive storage tests.

**Interfaces:** Shared `owned_array<T>(std::unique_ptr<T[]> storage, std::size_t count)` returns the existing nanobind
ndarray type; fixed Vec2/Vec3 overloads retain current shape. Properties explicitly use
`nb::rv_policy::reference_internal`. Private `_drive_prepared_storage` emits `jacobian_generation`, `qpos_generation`,
and capacities, replacing addresses; generations increment only when storage identity changes.

- [x] Verify all eight current array properties retain a temporary Stepper and survive owner deletion, using weakrefs if
  supported or reference-count controls. Add efc arena-growth tests across contact changes and step/forward; access a
  previously retained result after growth under ASan. Record expected shape/value contract.

```python
def test_temporary_stepper_qpos_retains_owner(saved_model):
    from native_loader import load_native
    array = load_native().Stepper(str(saved_model)).qpos
    gc.collect()
    assert np.isfinite(array).all()
    assert not array.flags.writeable
    assert array.shape == (1,)
```

Create `saved_model` fixture from an inline one-slide-joint MJCF and `mj_saveModel`; test already passes today and
guards against a false-fix regression.

- [x] Add private-hook tests: unknown kind ValueError and unchanged model/writer state; valid core request on a bare
  Stepper; attempted mutation with installed drivetrain rejected or isolated; storage generation stable over normal
  prepares. Reproduce current live-cache mismatch before implementation.
- [x] Replace raw-new ownership gaps in all three binding families, including bool-array helper. Transfer allocation
  only after successful capsule creation.

```cpp
auto allocation = std::make_unique<Vec3>(value);
nb::capsule owner(allocation.get(), [](void *p) noexcept {
    delete static_cast<Vec3 *>(p);
});
auto *data = allocation.release();
return Array(data->data(), {3}, owner);
```

Choose one consistent allocation/container form for dynamic and fixed arrays; the capsule owns exactly the matching
allocated type. Add a CPython capsule-creation-failure seam in the isolated test harness, not the public module API, to
exercise leak cleanup.

- [x] Make property policy explicit and replace incorrect nonowning comments. Return owned `efc_force` arrays if the
  runtime-arena stability test cannot establish storage validity across supported operations; retain zero-copy for fixed
  allocation-stable buffers. Document copy/view distinction. Close hook kind set; use isolated copied model/data or
  reject active writer use before mutation. Replace raw address checks with generations/capacities and update tests.
- [x] Add `nb::arg` names/defaults to chain/rider APIs, matching current positional behavior and Python counterparts.
  Run view/private-hook/contact tests, native profile, and ASan ownership controls. Review/commit when capsule failure
  has no leak, temporary owners survive, and private hooks cannot desynchronize a live writer.

### F5: Deterministic interval IDs and live timestep

**Files:** Modify `native/src/writers/tire.cpp`, `cruise.cpp/.hpp`, `src/bike_sim/sim/ride/tire_forces.py`, `cruise.py`;
create `native/src/interval_clock.hpp`, `tests/reference/test_native_clock_contracts.py`; extend
`native/tests/test_contracts.cpp`.

**Interfaces:** `interval_id(double time, double dt) -> std::int64_t` in native; Python
`interval_id(time: float, dt: float) -> int` in the tire force module. Both validate S4's interval domain. Cruise uses
`model_->opt.timestep` for each compute; no new public timestep setter is required to fix the direct C++ contract.

- [x] Test ties 0.5/1.5/2.5/3.5, adjacent nextafter values, zero/-0, NaN/±inf/negative time, nonpositive dt, and large
  binary64 values around `0x1p63`. Run native C++ controls under every supported FE mode, restoring the previous mode
  even on failure; compare against Python half-even on the same computed quotient. Test changing model.opt.timestep
  between cruise calls in Python and a direct native C++ case.

```python
@pytest.mark.parametrize("q, expected", [(0.5, 0), (1.5, 2), (2.5, 2), (3.5, 4)])
def test_interval_half_even(q, expected):
    from bike_sim.sim.ride.tire_forces import interval_id
    assert interval_id(q, 1.0) == expected
```

- [x] Run focused controls; expect absent helper, rounding-mode, range, and cached-dt failures. Implement finite/domain
  checks first, then rounding independent of nearbyint/fenv.

```cpp
double base = std::floor(quotient);
double fraction = quotient - base;
if (fraction > 0.5 || (fraction == 0.5 && std::fmod(base, 2.0) != 0.0))
    base += 1.0;
if (base >= 0x1p63) throw std::overflow_error("tire.interval_id");
return static_cast<std::int64_t>(base);
```

Check quotient overflow before floor and validate negative/nonfinite time before the snippet. Python checks rounded
integer against `(1 << 63) - 1`; binary64 spacing near the boundary is part of the test matrix.

- [x] Replace tire snapshot's conversion/error order and Python interval calculation with the shared-domain helpers.
  Retain binary64 division order and snapshot schema. Remove cached cruise dt or use it only for immutable validated
  config unrelated to integration.
- [x] Run clock tests, CTest, native/full, and golden tire/cruise tests. Review/commit when FE modes do not change tie
  conversion, bounds are exact, errors match Python/native, and valid integration with a changed timestep agrees.
