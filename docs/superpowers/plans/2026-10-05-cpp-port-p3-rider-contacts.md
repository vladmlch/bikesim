# Native rider contacts P3 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Port rider support/grip forces, solved attachment measurements, release/capture, and complete contact state into C++ for the remaining P3 runtime.

**Architecture:** Python-independent contact math and attachment measurement modules feed a model-owned `RiderContactWriter`. Plain setup/snapshot dictionaries cross nanobind, with no Python callbacks in numerical methods. This increment completes the contact applier seam; the rider controller and P4 scheduler/viewer remain subsequent work under the same accepted ADR.

**Tech Stack:** C++23, nanobind, MuJoCo 3.12.0, Apple Accelerate CBLAS/LAPACK, uv/pytest, clang/GCC, ASan/UBSan.

**Spec:** `docs/adr/0001-native-port-mujoco-core.md`, remaining P3 rider contacts; unchanged `src/bike_sim/sim/ride/rider_contacts.py`, `support_geometry.py`, `weld_pedals.py`, `attachment_wrench.py` are the behavioral reference.

## Global Constraints

- C++23 with existing strict warning flags and `-ffp-contract=off`; no warning suppression across the target.
- Typed numerical modules contain no Python includes, objects, callbacks, computations, or external interpreter calls. Config, scratch, and snapshots own their storage.
- Preserve schema 1 with optional `rider_contacts`; no-section construction retains every existing Stepper behavior.
- Preserve expression/accumulation order, signed zeros, `None` sentinels, enabled flags, causal solved-force timing, and exceptions on valid-domain overflow. Compare outputs/state raw bytewise on this platform; never relax a mismatch without diagnosis and a recorded ruling.
- Use the same MuJoCo dylib, system libm, Accelerate operation shapes, and `writers/pyfloat.hpp` for Python exponentiation. NumPy 2.5.2 imports `dgelsd$NEWLAPACK$ILP64`; least-squares recovery must match that LAPACK algorithm/ABI with `rcond=1e-12`.
- Model-size Jacobian, generalized-force, solver multiplier, and least-squares work buffers are owned/reused. Setup/snapshot/FFI conversion may allocate. This does not claim a complete allocation-free P4 tick or a speedup.
- Python commands always use uv; `UV_CACHE_DIR=/tmp/cpp-port-p3-uv`. Tests assert the exact local regular or explicitly selected sanitizer extension path.
- Use unchanged Python algorithms with tiny real compiled models and genuine solved MuJoCo constraints as independent per-call oracles. Never inject Python engine addresses or fabricated efc arrays. Full-episode/golden acceptance stays in P4.
- Focused tests during iteration, affected tests/compiler gate per task, one combined sanitizer gate, and one quick profile after the increment. Do not rerun the known failing eight-minute full suite.
- Work in existing isolated `impl/cpp-port-p2`; preserve user-dirty `CLAUDE.md` and unrelated files. Scoped commits, no merge/push.

---

### Task 1: Shared contact laws and finite support geometry

**Files:**
- Create: `native/src/contact/laws.hpp` — shared typed `_normal_contact` and `_brush_step`, preserving current tire implementation and exception behavior.
- Modify: `native/src/writers/tire.cpp` — consume these same shared laws instead of retaining duplicate implementations; other tire calculations remain unchanged.
- Create: `native/src/rider/contact_math.hpp`, `contact_math.cpp` — fixed-size vectors/rotations, grip step/release, Python-faithful three-coordinate hypot where needed.
- Create: `native/src/rider/support_geometry.hpp`, `support_geometry.cpp` — box pad, upper face, sole target/projected goal, planar model validation.
- Create: `native/src/rider/contact_binding.hpp`, `contact_binding.cpp` — pure diagnostic bindings plus `bind_rider_contact_math(nanobind::module_&)`.
- Modify: `native/src/binding.cpp`, `native/CMakeLists.txt` — register/build new modules.
- Test: `tests/reference/test_native_rider_contact_math.py`.
- Modify: `docs/TESTING.md` — focused command and numerical contract.

**Interfaces:**

Python-independent namespace `rider` defines `Vec3=std::array<double,3>`, `Mat3=std::array<double,9>` row-major; `BoxPadContact {point_m, normal, gap_m, within_width, within_footprint}` with tangent `[normal[2],0.,-normal[0]]`. Core functions mirror the Python positional inputs and typed results, using `std::optional` for missing values. Public diagnostic entrypoints:

```python
bike_native.rider_box_pad_contact(center_m, radius_m, origin_m, rotation, half_size_m)
# owning dict: point_m, normal, tangent, gap_m, within_width, within_footprint
bike_native.rider_upper_box_face(origin_m, rotation, half_size_m)  # point, normal, tangent
bike_native.rider_sole_target_height(origin, rotation, half, sole_x_m,
    pad_half_length_m, pad_radius_m, compression_m)  # float
bike_native.rider_project_sole_goal(origin, rotation, half, pad_half_length_m,
    pad_radius_m, compression_m, shear_m)  # (owning vector, exact diagnostic dict)
bike_native.rider_grip_step(xi, relative_velocity, k, c, dt)
# (owning new vector, owning force vector, energy, loss)
bike_native.rider_release_if_overloaded(force_n, old_energy_j, limit_n)
# (owning vector, release_loss, overloaded bool)
```

Core exports `box_pad_contact` (validated), `box_pad_contact_unchecked` for previously validated compiled geometry, `upper_box_face`, `sole_target_height`, `project_sole_goal`, `grip_step`, `release_if_overloaded`, `validate_planar_support_model(model,data,geom_ids)`. Shared `contactlaw::normal_contact` returns `(force,energy)` and `contactlaw::brush_step` returns `(xi,force,loss)` with the existing tire finite/passivity guards. The later writer calls these same functions directly. Preserve Python `UnreachableSoleTarget` semantics via a registered ValueError subclass, keeping non-convergence a RuntimeError.

- [ ] **Step 1: Write raw-byte oracle tests before implementation.** Use independent `support_geometry`, `grip_step`, and `grip_release` imports. Example:

```python
def test_grip_energy_oracle():
    xi = np.array([.003, -.002, .001])
    velocity = np.array([.1, 0., -.2])
    expected = grip_step(xi, velocity, 4000., 150., .0005)
    actual = bike_native.rider_grip_step(xi, velocity, 4000., 150., .0005)
    for got, wanted in zip(actual, expected):
        assert_bitwise_equal(np.asarray(got), np.asarray(wanted))
```

Cover planar rotations, flipped/vertical boxes, finite edges/corners, medial-axis ties, width/footprint boundaries, signed zeros and near-zero deltas. Cover upper face ray, both sole-target branches, unreachable compression/shear projection and limiting-reason order, convergence errors; random valid points. Grip cases include release at/below/above strength, overflow, zero damping, three-coordinate distances against `math.hypot`. Check owning outputs/dtype conversions, invalid shapes/nonfinite/config ranges and improper/nonplanar rotations. Do not change existing Python references.

- [ ] **Step 2: Run RED.**

```bash
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run pytest tests/reference/test_native_rider_contact_math.py -q
```

Expected missing native API failures, not collection skips or erroneous fixtures. Keep evidence.

- [ ] **Step 3: Extract existing tire laws literally, then port math/geometry.** Preserve this `_box_pad_contact` branch and ordering:

```cpp
const double closest_x = std::min(std::max(local[0], -half[0]), half[0]);
const double closest_z = std::min(std::max(local[2], -half[2]), half[2]);
const double distance = std::hypot(local[0] - closest_x, local[2] - closest_z);
```

Matrix-vector products use the same Accelerate shape as NumPy. Grip energy/loss computes both `u@u` occurrences with the same dot operation and order. CPython three-argument `math.hypot` is not assumed identical to `std::hypot(x,y,z)`; diagnose/port its compensated norm if tests show a difference. Do not substitute a different sole-target algorithm or alter fixed iteration counts (32/18) and tolerances from the source.

- [ ] **Step 4: Build, focused GREEN, affected tire gate, compiler sweep.**

```bash
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run cmake --build native/build -j4
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run pytest tests/reference/test_native_rider_contact_math.py tests/reference/test_native_tire.py -q
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run cmake --build native/build --target check_frontends
```

Report raw-byte/math-hypot evidence and exact typed interfaces for downstream tasks. Escalate any unresolved library rounding issue.

- [ ] **Step 5: Self-review and scoped commit.**

```bash
git add native/src/contact native/src/rider native/src/writers/tire.cpp native/src/binding.cpp native/CMakeLists.txt tests/reference/test_native_rider_contact_math.py docs/TESTING.md
git commit -m "feat(native): port finite rider support geometry and shared contact laws"
```

### Task 2: Equality reactions and attachment wrench measurement

**Files:**
- Create: `native/src/rider/equality_reactions.hpp`, `equality_reactions.cpp` — fresh equality row grouping, signed forces/residuals/crank torque, selected generalized force.
- Create: `native/src/rider/least_squares.hpp`, `least_squares.cpp` — owning/reusable Accelerate ILP64 DGELSD workspace and NumPy-faithful results.
- Create: `native/src/rider/attachment_wrench.hpp`, `attachment_wrench.cpp` — prepared geometry, raw data, spatial/planar wrench recovery, budget sample decomposition.
- Create: `native/src/rider/attachment_binding.hpp`, `attachment_binding.cpp` — dictionary/array conversion and diagnostic API registration.
- Modify: `native/src/stepper.hpp`, `binding.cpp`, `native/CMakeLists.txt` — diagnostic methods on the existing owner, no second simulation owner.
- Test: `tests/reference/test_native_attachment_wrench.py`.
- Modify: `docs/TESTING.md` — supported measurement/error semantics.

**Interfaces:**

Core uses Task 1 `rider::Vec3/Mat3`; defines owning `AttachmentGeometry`, `AttachmentRaw`, `AttachmentSample` with exactly the reference dataclass fields and array shapes. Column indices are owned checked integer vectors. `EqualityReactions(model)` owns nv/nefc work storage; each evaluation derives row membership from current `efc_type/efc_id`, never from an earlier solve. It implements row grouping, `force_on_rider`, `translation_residual`, `equality_qfrc`, and sum-of-pedal-equalities torque at crank dof. Missing current rows produce the reference zero output, not an invented measurement.

`LeastSquaresWorkspace(max_rows,max_columns,max_rhs)` invokes the same `dgelsd$NEWLAPACK$ILP64` symbol with checked signed 64-bit dimensions and workspace query; copy inputs to its column-major A and padded B buffers. Return solution/rank/singular values needed by the reference checks. Validate dimensions before copying; nonzero LAPACK info fails explicitly. Preserve distinct rank behavior: `recover_wrench` requires full column rank; spatial body-wrench recovery accepts minimum-norm underdetermined results if observable/reconstruction checks pass.

Diagnostic APIs reuse `Stepper.model()/data()` ownership; no added model or fake efc restoration:

```python
bike_native.rider_recover_wrench(relative_jacobian, qfrc)  # owning planar vector
bike_native.rider_decompose_wrench(wrench6, normal, kind, rotational,
    half_patch_m=0., gap_m=0., pull_direction=None)  # exact AttachmentSample dict
stepper.rider_equality_qfrc(eq_id)  # owning nv-vector
stepper.rider_relative_planar_jacobian(body_a, body_b, point, rotational)
stepper.rider_prepare_attachment(eq_id, body_rider, body_bike, point, normal,
    kind, rotational, half_patch_m=0., pull_direction=None)  # owning geometry dict
stepper.rider_attachment_raw(eq_id, body_rider, body_bike, point, normal,
    kind, rotational, half_patch_m=0., pull_direction=None)  # owning raw dict
stepper.rider_attachment_raw_from_geometry(geometry, validate_wrench=True)
stepper.rider_attachment_sample(eq_id, body_rider, body_bike, point, normal,
    kind, rotational, half_patch_m=0., pull_direction=None)  # owning sample dict
```

The typed core exports the corresponding operations for Task 3 without constructing an FFI wrapper. Preserve exact raw/prepared schemas, normal/pull projections, body-specific soft-connect anchors, disjoint support checks, least-squares rank/reconstruction/third-law/planarity failures and gap from current `efc_pos`. Sample normal/tangent/moment/gap/pull/half-patch fields match `AttachmentSample`. Raw from prepared geometry with `validate_wrench=False` defers spatial checks exactly like Python.

- [ ] **Step 1: Add independent failing numerical and genuine-solve tests.**

```python
def test_native_recover_wrench():
    jac = np.array([[1., 0., 0., -1.], [0., 1., 0., 0.], [0., 0., 1., 0.]])
    force = jac.T @ np.array([20., 100., -2.])
    assert_bitwise_equal(bike_native.rider_recover_wrench(jac, force),
                         recover_wrench(jac, force))
```

Add random tall/wide/rank-deficient matrices and near rcond thresholds, inconsistent targets and nonfinite/shape errors. Use small independent rider/bike free-body or scalar planar weld/connect models (existing `test_attachment_wrench.py::_pair_model` is useful) with genuine applied nv forces via `set_inputs`. Compare every prepared/raw/sample field against unchanged Python after identical forward/8-step solves. Cover dense/sparse layout, unrelated contact/limit rows, missing equality rows, soft-connect nonzero anchor gap, shared supports, out-of-plane force, unobservable DOFs, Newton-law/reconstruction errors, owning prepared snapshots and validation-disabled raw path. The solved multipliers must come from MuJoCo; no direct efc writes in native fixtures.

- [ ] **Step 2: Run RED.**

```bash
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run pytest tests/reference/test_native_attachment_wrench.py -q
```

- [ ] **Step 3: Implement row extraction, DGELSD recovery, then attachment measurement.** Preserve selected multipliers and engine mapping:

```text
scan current equality rows -> zero multiplier scratch
-> copy forces only for requested equality -> mj_mulJacTVec
-> prepare body Jacobians/column support -> recover body-specific wrenches
-> reconstruction/observability/Newton/planarity checks -> decompose sample
```

For connect constraints each body's own compiled anchor is the force point. Prepared geometry contains pre-step Jacobians and columns; combine with post-solve multipliers/residuals without a new forward. Preserve the source's `np.allclose` asymmetry/tolerances exactly, not a norm-only replacement. Static rank/error threshold cases must agree with the same LAPACK routine; diagnose a raw-bit difference before changing any tolerance.

- [ ] **Step 4: Build, focused GREEN, frontend sweep.**

```bash
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run cmake --build native/build -j4
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run pytest tests/reference/test_native_attachment_wrench.py tests/reference/test_native_rider_contact_math.py -q
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run cmake --build native/build --target check_frontends
```

Document exact LAPACK symbol/ABI evidence and all exported typed interfaces for the final writer.

- [ ] **Step 5: Self-review and scoped commit.**

```bash
git add native/src/rider native/src/stepper.hpp native/src/binding.cpp native/CMakeLists.txt tests/reference/test_native_attachment_wrench.py docs/TESTING.md
git commit -m "feat(native): port solved rider attachment wrench measurements"
```

### Task 3: Model-owned contact applier, complete state and integration

**Files:**
- Create: `native/src/rider/contact_config.hpp` — typed resolved contact setup and complete snapshot/diagnostic structs, no Python includes.
- Create: `native/src/writers/rider_contacts.hpp`, `rider_contacts.cpp` — force assembly, release/capture, linked reaction latch, complete typed state.
- Create: `native/src/rider/rider_contact_binding.hpp`, `rider_contact_binding.cpp` — optional config parser, snapshots/diagnostics/prepared-raw conversion and Stepper method registration.
- Modify: `native/src/config.hpp`, `stepper.hpp`, `stepper.cpp`, `binding.cpp`, `native/CMakeLists.txt` — optional owning writer and lifecycle.
- Modify: `tools/native_config.py` — `project_rider_contacts(applier)` and optional `project()` section only when `sim.physical.rider_contacts` exists.
- Test: `tests/reference/test_native_rider_contacts.py`.
- Modify: `docs/TESTING.md` — complete contact API/snapshot/stage order and acceptance limits.

**Interfaces:**

`rider_contacts` section carries `arm_reach` already resolved by runtime from `pose`; copied config fields are `saddle_patch_half_length_m`, `pedal_patch_half_length_m`, `support_pad_radius_m`, `support_k_n_m`, `support_c_ns_m`, `pedal_c_ns_m`, `support_tangent_k_n_m`, `support_mu`, `support_length_m`, `grip_k_n_m`, `grip_c_ns_m`, `grip_release_distance_m`, optional `grip_pair_force_limit_n`, `grip_capture_distance_m`, `grip_capture_speed_mps`, `pedal_attachment`, `saddle_attachment`, `grip_attachment`. Names/topology match Python literal body/site/geom/equality names, not a guess at source JSON or pose files. Projection copies config only; current material/latch/anchor state crosses via snapshot restoration.

The writer consumes typed modules from Tasks 1–2, `mjModel*`, `mjData*`, typed config. Exact public APIs:

```python
stepper.rider_contacts_reset()  # initialize anchors on current model/data
stepper.rider_contacts_restart_clock()
stepper.rider_contacts_initialize_settled_state()
stepper.rider_contacts_set_enabled(name, enabled)  # bool, linked contacts cannot release
stepper.rider_contacts_release_all()
stepper.rider_contacts_qfrc(dt, advance=True, detailed=True)  # owning nv-vector
stepper.rider_contacts_stored_energy()  # float
stepper.rider_contacts_prepare_attachment_raw()  # (geometry dict by name, tuple errors)
stepper.rider_contacts_settle(interval_state=None, raw=False, prepared=None)
# exact Python settle_welds output dict, or (dict, samples, errors) when raw
stepper.rider_contacts_attachment_samples(interval_state=None, raw=False)
# (owning samples dict, errors tuple)
stepper.rider_contacts_diagnostics(probe=False)  # owning dict
stepper.rider_contacts_state()  # complete owning snapshot
stepper.set_rider_contacts_state(state)  # validated atomic restoration
```

`interval_state` is optional `(qpos,qvel)` with model widths; `prepared` matches Task 2 geometry/raw dictionary schemas plus error tuple. Preserve public return shapes and missing-writer errors. Model/body caches, reusable scratch, and owned detached data are implementation state, not serialized pointers.

Snapshot includes every mutable field: enabled map (four names), six support states (`xi`, optional tangent), left/right grip xi/optional anchors, elastic/loss/radial-power/delivered-torque fields, diagnostics, optional clock, pending-release loss, optional settled support dict+crank torque, last attachment samples/errors, optional probe diagnostics/enabled/delivered torque. Preserve constructor/reset sentinels; parse full candidates, validate shape/type/finite/enums/topology/clock and all allocations before committing. Returned samples/states and converted input buffers must outlive FFI temporaries. Restore no derived efc arrays. mjData restore stays independent of contact state. Release after integration refreshes kinematics only; capture refreshes kinematics/comPos and checks both hands before zeroing springs.

- [ ] **Step 1: Write failing full contact oracle tests using tiny genuine models.** Name required frame/steer/rider pelvis, two feet/forearms/upper arms, sole/saddle/grip sites, support box geoms, crank joint and optional weld/connect equalities exactly. Each model has independently moving planar bike/rider branches; don't invoke research equilibrium relaxation. Save MJB and construct separate owners, identical state, explicit reset.

```python
expected = python_contacts.compute_qfrc(model, data, dt, advance=True, detailed=True)
actual = native.rider_contacts_qfrc(dt)
assert_bitwise_equal(actual, expected)
assert_tree(native.rider_contacts_diagnostics(), python_contacts.diagnostics)
assert_tree(native.rider_contacts_state(), python_snapshot(python_contacts))
```

Test all pedal flat/weld/spindle, saddle flat/weld/pin, and grip spring/connect modes with a bounded representative matrix and mixed configurations. Test exact forces/diagnostics/state/energy, tilted/flipped finite supports, contact/shear transport and material face switch, zero normal/sliding, asymmetric hand reach/overload with left-then-right update order, release/capture success/failure/blocked linked releases, pending release energy at incoming/outgoing poses, initialize-settled-state, restart versus reset, both detailed flags, duplicate-time guard, non-advancing probes and their outputs, state ownership/aliasing/invalid restore, optional projection and no-contact Stepper. Short genuine solves latch equality reactions and next-step crank sensor; compare prepared/raw/scalar attachment measurements/errors across dense/sparse layouts, unrelated rows, optional interval-start pose. Check public interval-state measurement restores live qpos/qvel and poses even on an error.

- [ ] **Step 2: Run RED against missing contact APIs/projection.**

```bash
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run pytest tests/reference/test_native_rider_contacts.py -q
```

- [ ] **Step 3: Implement the unchanged applier with typed state and causal stages.**

```text
reset anchors -> compute incoming support/grip forces
-> prepare attachment geometry before integration -> genuine MuJoCo solve
-> latch equality reactions/raw inputs from that solve
-> next compute reads the latched force and crank sensor
```

For pads use one relative Jacobian pair for velocity and force mapping at a common point; add `jrel.T @ f` once, preserve Newton pairs and accumulation order. Linked supports add no second physical force. Detailed=False must preserve material/enable/force/energy behavior and omit exactly the reference diagnostics. Probes copy material/enabled state and leave live latches/clocks/anchors intact while publishing only probe values. Do not fix apparent Python order quirks (including asymmetric hand release); expose exact oracle behavior. `prepare_attachment_raw` and scalar sampling preserve errors rather than substituting zero-force samples. Own reusable nv scratch and destroy writer before model on normal and failed construction paths.

- [ ] **Step 4: Build, focused GREEN, frontend and one sanitizer gate.**

```bash
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run cmake --build native/build -j4
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run pytest tests/reference/test_native_rider_contacts.py tests/reference/test_native_attachment_wrench.py tests/reference/test_native_rider_contact_math.py -q
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run cmake --build native/build --target check_frontends
UV_CACHE_DIR=/tmp/cpp-port-p3-uv uv run cmake --build native/build/asan -j4
```

Run all three new modules under the actual ASAN extension with `NATIVE_TEST_BUILD_DIR=asan`, `ASAN_OPTIONS=detect_leaks=0` and existing clang-21 ASAN library injected via `uv run env ... python`; assert/print exact resolved module path before pytest. Report exact counts/times/RED-GREEN/projection/state schemas and remaining limitations. Root runs one `bash tools/run_tests.sh quick` after task fixes, then final increment review. No full-suite, speed, viewer or full-P3 claim.

- [ ] **Step 5: Self-review and scoped commit.**

```bash
git add native/src/rider native/src/writers/rider_contacts.hpp native/src/writers/rider_contacts.cpp native/src/config.hpp native/src/stepper.hpp native/src/stepper.cpp native/src/binding.cpp native/CMakeLists.txt tools/native_config.py tests/reference/test_native_rider_contacts.py docs/TESTING.md
git commit -m "feat(native): port model-owned rider contacts and release state"
```

## Completion evidence

Three task gates and a final increment review resolve correctness findings. Collect all ledger rulings/costs before removing only this plan's generated scratch directory. Retain branch/worktree for rider controller and P4; no merge/push. Final report distinguishes contact completion from unfinished controller/full-viewer/full-suite acceptance.
