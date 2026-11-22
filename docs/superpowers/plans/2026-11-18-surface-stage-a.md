# Track Surface Simulation — Stage A Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Deliver Stage A of track surface simulation for `pneumatic` and `distributed_2d_reference` tyres — calibrated `crr_delta` and `load_sensitivity` in `SurfaceSpec`, $C^1$-smooth `blend_m` transitions in `SurfaceMap`, ray-/station-resolved surface physics, energy-conserving shear stiffness scaling in `distributed_2d_reference`, and exact radial + rolling energy-balance closure in `pneumatic` — while keeping `compliant_2d` bitwise untouched.

**Architecture:** All "tyre × surface" mechanics reside inside `PneumaticTireWriter`/`PneumaticTyreModel` and `DistributedTireWriter`, backed by one allocation-free stateless evaluator in `tirecommon`. Rolling resistance (`tread_loss_crr_at_pressure + crr_delta` in `pneumatic`, `crr_delta` in `distributed_2d_reference`) is applied as a pure world-frame wheel-body moment via MuJoCo's rotational Jacobian `jr` (reaction to world, zero artificial pitch torque on the frame), while `pneumatic` radial stored energy is closed via a cancellation-free air-chord integral $W(d)$ and a nonsingular Bregman–Legendre Maxwell potential $U_M(d, \xi_e)$.

**Tech Stack:** C++23, MuJoCo 3.12, OpenBLAS/CBLAS (`cblas_abi.hpp`), CMake + Ninja + CTest.

**Spec:** `docs/superpowers/specs/2026-11-18-surface-stage-a-design.md`

## Global Constraints

- **Target tyre backends:** `pneumatic` and `distributed_2d_reference` ONLY. `TireWriter` (`compliant_2d`), `ResistanceWriter`, and `native/src/gpu/` MUST NOT change behavior by a single bit.
- **Wire & fixture preservation:** `crr_delta`, `load_sensitivity` (on `surface`) and `blend_m` (on `tire.surface_map.sections`) are optional wire keys defaulting to `0.0`. `project_config.cpp` MUST NOT emit `crr_delta` or `load_sensitivity` when `tires.backend == "compliant_2d"`, and `blend_m` is emitted in `project_config.cpp` and `trackfile.cpp` ONLY when `!= 0.0` so all ~35 existing JSON/TOML fixtures and `track_sha256` hashes remain byte-identical.
- **Aggregate initialization & warnings:** `nativecfg::SurfaceMap` remains a 2-field aggregate `{surface, sections}`; new fields on `SurfaceSpec` (`crr_delta = 0.0`, `load_sensitivity = 0.0`) and `SurfaceSection` (`blend_m = 0.0`) carry default member initializers so `-Werror -Wmissing-field-initializers` passes across all 113 compiler flags.
- **Realtime safety (`BIKE_NONBLOCKING`):** Zero heap allocations inside `PneumaticTireWriter::try_compute_into` and `DistributedTireWriter::try_compute_into`; every scratch vector (`jr_`, `contact_x_`, `slips_`, `surface_samples_`) and string capacity (`surface_capacity >= 10`) is sized at construction.
- **Wheel-moment physics:** Rolling-resistance moments MUST map through the wheel body's world rotational Jacobian `row = jr.T @ axis` with world spin `omega_world = row @ qvel` and `kWheelRollingTaperRadps = 0.2` rad/s — NEVER through `out[spin_dof]`.
- **Precision & strict compiler flags:** `-ffp-contract=off`, `-Werror`, `-Wunsafe-buffer-usage`, `-Wframe-larger-than=8192` on all production TUs.

## Test Budget

Baseline timings from `<git-common-dir>/bike-gate/timings.tsv` (16-core reference: incremental build ~17 s, full `ctest` ~69 s, `cppcheck` ~13 s, `contracts` ~4 s, `headers` ~32 s):
- **Per task (Tasks 1–5):** targeted incremental build + `ctest --test-dir build/native-release -R '<pattern>' --output-on-failure` (~15–30 s per task, ~2.5 min total).
- **Task 6 (Golden refresh & completion verification):** re-record the 6 affected `linux-aarch64` reference targets (~2 min), run all `terrain|config|pneumatic|distributed|native_tire|input_contracts|numeric_completion` suites (~2 min), and run `bike-native-checks --kind scripts` + `cppcheck` + `headers` on touched TUs (~3 min). Total budget: ~10–12 minutes after initial worktree compile.

---

### Task 1: Surface Domain, Validation, Wire Schema & Track Parser

**Files:**
- Modify: `native/src/platform/process.cpp:122-129`
- Modify: `native/src/terrain/types.hpp:29-61`
- Modify: `native/src/terrain/surface.cpp:10-76`
- Modify: `native/src/terrain/trackfile.cpp:233-241, 417-421`
- Modify: `native/src/terrain/generators.hpp` + `native/src/terrain/spec_io.hpp` + `native/src/terrain/alpine.cpp:107-116` (optional `surface_blend_m` only if needed; keep `AlpineGenSpec` default `spec_sha256` byte-identical!)
- Modify: `native/src/config_types.hpp:261-277`
- Modify: `native/src/config_validation.hpp:344-375`
- Modify: `native/src/configuration/schema.cpp:586-594`
- Modify: `native/src/writer_config.hpp:458-475` (+ `decode_surface` / `decode_surface_section` helpers in `writer_config.hpp`)
- Modify: `native/src/runtime/project_config.cpp:190-196, 231-256`
- Modify: `examples/research/viewer_physics_pneumatic.toml` (add `[resistance]\ncrr = 0.0`)
- Modify: `tests/fixtures/config/viewer_physics_pneumatic.toml` and `tests/fixtures/config/resolved/pneumatic.json` (`crr = 0.0`)
- Modify: `native/tests/reference/test_native_input_contracts.cpp:180-190`
- Modify: `native/tests/reference/pneumatic_reference_support.hpp:150-160`, `native/tests/reference/pneumatic_tire_fixture.hpp:80-84`, `native/tests/reference/test_tyre_road_profile.cpp:28-38`
- Test: `native/tests/terrain/test_generators.cpp`, `native/tests/terrain/test_presets.cpp`, `native/tests/config/test_schema.cpp`, `native/tests/config/test_resolution.cpp`, `native/tests/reference/test_native_input_contracts.cpp`

**Interfaces:**
- Produces:
  - `bike::terrain::SurfaceSpec` with `double crr_delta = 0.0; double load_sensitivity = 0.0;` and calibrated 5-surface registry (`asphalt`: `0.0, 0.06`; `hardpack`: `0.0025, 0.08`; `loose`: `0.0060, 0.12`; `wet`: `0.0035, 0.12`; `scree`: `0.0080, 0.15`).
  - `bike::terrain::SurfaceSection` with `double blend_m = 0.0;`.
  - `nativecfg::SurfaceSpec` (`double crr_delta = 0.0, load_sensitivity = 0.0;`) and `nativecfg::SurfaceSection` (`double blend_m = 0.0;`).
  - Synchronized validation in `bike::terrain::SurfaceMap::validate()` and `nativecfg::validate(const SurfaceMap&)` enforcing Spec §2.1–2.2 (`crr_delta >= 0`, `0 <= load_sensitivity <= 0.5`, `blend_m >= 0`, `sec[0].start_m == 0.0` has no left transition else `start_m >= 0.5 * blend_m`, adjacent `end_m == next.start_m` requires equal `blend_m`, separated `next.start_m - end_m >= 0.5 * (b_i + b_{i+1})`, and `end_m - start_m >= 0.5 * b_left + 0.5 * b_right`).

- [ ] **Step 1: Unblock `libbike_platform.a` on glibc C++23 & write failing domain/track/wire tests**

In `native/src/platform/process.cpp:122-129`, guard `::posix_spawn_file_actions_addchdir` on `__GLIBC__` by `#if defined(_POSIX_C_SOURCE) && _POSIX_C_SOURCE >= 202405L` so `_GNU_SOURCE` C++23 builds use `::posix_spawn_file_actions_addchdir_np`.

Add unit tests in `native/tests/terrain/test_generators.cpp`:
```cpp
// Stage A SurfaceSpec + SurfaceMap blend_m validation & trackfile roundtrip
const auto& asphalt = t::get_surface("asphalt");
const auto& scree = t::get_surface("scree");
bt::require(asphalt.crr_delta == 0.0 && asphalt.load_sensitivity == 0.06, "asphalt stage-a fields");
bt::require(scree.crr_delta == 0.0080 && scree.load_sensitivity == 0.15, "scree stage-a fields");
bt::invalid([] {
    t::SurfaceSpec{.name = "bad", .mu_peak = 0.8, .mu_slide = 0.6,
                   .slip_stiffness_per_load = 10.0, .crr_delta = -0.001}.validate();
});
bt::invalid([] {
    t::SurfaceSpec{.name = "bad", .mu_peak = 0.8, .mu_slide = 0.6,
                   .slip_stiffness_per_load = 10.0, .load_sensitivity = 0.55}.validate();
});
// Adjacent touching sections with mismatched blend_m reject; equal blend_m accepts.
bt::invalid([] {
    (void)t::SurfaceMap(t::get_surface("hardpack"),
        {{.start_m = 10.0, .end_m = 20.0, .surface = "loose", .blend_m = 0.4},
         {.start_m = 20.0, .end_m = 30.0, .surface = "scree", .blend_m = 0.8}});
});
(void)t::SurfaceMap(t::get_surface("hardpack"),
    {{.start_m = 0.0, .end_m = 5.0, .surface = "asphalt", .blend_m = 1.0},
     {.start_m = 5.0, .end_m = 12.0, .surface = "scree", .blend_m = 1.0}});
bt::invalid([] {
    (void)t::SurfaceMap(t::get_surface("hardpack"),
        {{.start_m = 0.2, .end_m = 5.0, .surface = "asphalt", .blend_m = 0.6}});
});
```

- [ ] **Step 2: Build `terrain_generators` to verify RED**

Run: `cmake --build build/native-release --target terrain_generators`
Expected: FAIL (missing `crr_delta`, `load_sensitivity`, `blend_m` members).

- [ ] **Step 3: Implement `SurfaceSpec`, `SurfaceSection`, validation, wire decode/schema, trackfile & pneumatic `crr = 0.0`**

1. Add `double crr_delta = 0.0; double load_sensitivity = 0.0;` to `bike::terrain::SurfaceSpec` and `nativecfg::SurfaceSpec`, and `double blend_m = 0.0;` to `bike::terrain::SurfaceSection` and `nativecfg::SurfaceSection`.
2. Populate the 5 registry entries in `native/src/terrain/surface.cpp` with the exact Spec §2.1 table values (with a brief comment explaining why `wet` is stiffer and lower-`crr_delta` than `loose` despite lower `mu_peak`).
3. Implement exact Spec §2.1–2.2 validation in both `bike::terrain::SurfaceMap::validate()` and `nativecfg::validate(const SurfaceMap&)` (`native/src/config_validation.hpp`).
4. Update `native/src/terrain/trackfile.cpp`: accept optional `blend_m` in `surface_sections` (rejecting unknown keys with `TrackFileError`), and emit `"blend_m"` in `track_value` only when `s.blend_m != 0.0`. Also add `double surface_blend_m = 0.0;` to `AlpineGenSpec` (`native/src/terrain/generators.hpp`, `alpine.cpp`): validate `0.0 <= surface_blend_m`, omit `"surface_blend_m"` from `AlpineGenSpec::to_dict()` when `0.0` so all existing `spec_sha256` hashes remain byte-identical, and pass `.blend_m = s.surface_blend_m` (clamped to `<= 0.5 * section_length` when `surface_blend_m > 0`) when generating `scree`/`wet` sections in `generate_alpine_track`.
5. Update `native/src/configuration/schema.cpp` (`surface` optional `"crr_delta load_sensitivity"`, `tire.surface_map.sections` optional `"blend_m"`) and `native/src/writer_config.hpp` (`exact_at` optional keys + `opt_num(d, "crr_delta", 0.0)`, `opt_num(d, "load_sensitivity", 0.0)`, `opt_num(d, "blend_m", 0.0)`).
6. Update `native/src/runtime/project_config.cpp`: emit `crr_delta` and `load_sensitivity` (when `!= 0.0`) ONLY when `tires.backend != "compliant_2d"`, and emit `"blend_m"` on sections ONLY when `s.blend_m > 0.0`. Register `"crr_delta"`, `"load_sensitivity"`, `"blend_m"` in `optional_nested` in `native/tests/reference/test_native_input_contracts.cpp`.
7. Forward `.crr_delta` and `.load_sensitivity` in `pneumatic_reference_support.hpp`, `pneumatic_tire_fixture.hpp`, and `test_tyre_road_profile.cpp`.
8. Add `[resistance]\ncrr = 0.0` to `examples/research/viewer_physics_pneumatic.toml` and `tests/fixtures/config/viewer_physics_pneumatic.toml`, and set `"crr": 0.0` in `tests/fixtures/config/resolved/pneumatic.json`.

- [ ] **Step 4: Build & run targeted tests to verify GREEN**

Run:
```bash
cmake --build build/native-release --target terrain_generators terrain_presets config_schema config_resolution reference.native_input_contracts reference.native_pneumatic_wire reference.native_tire reference.tyre_road_profile
ctest --test-dir build/native-release -R "^(terrain_generators|terrain_presets|config_schema|config_resolution|reference\.native_input_contracts|reference\.native_pneumatic_wire|reference\.native_tire|reference\.tyre_road_profile)$" --no-tests=error --output-on-failure
```
Expected: 100% PASS (`compliant_2d` `reference.native_tire` and preset hashes completely green).

- [ ] **Step 5: Commit**

```bash
git add native/src/platform/process.cpp native/src/terrain/types.hpp native/src/terrain/surface.cpp native/src/terrain/trackfile.cpp native/src/terrain/generators.hpp native/src/terrain/alpine.cpp native/src/config_types.hpp native/src/config_validation.hpp native/src/configuration/schema.cpp native/src/writer_config.hpp native/src/runtime/project_config.cpp examples/research/viewer_physics_pneumatic.toml tests/fixtures/config/viewer_physics_pneumatic.toml tests/fixtures/config/resolved/pneumatic.json native/tests/terrain/test_generators.cpp native/tests/reference/test_native_input_contracts.cpp native/tests/reference/pneumatic_reference_support.hpp native/tests/reference/pneumatic_tire_fixture.hpp native/tests/reference/test_tyre_road_profile.cpp
git commit -m "feat(terrain,config): extend SurfaceSpec/SurfaceMap with crr_delta, load_sensitivity and blend_m"
```

---

### Task 2: Canonical `tirecommon` Surface Evaluator (`eval_surface`, `uniform_surface_for_span`, Load-Sensitive Friction)

**Files:**
- Modify: `native/src/writers/tire_common.hpp:96-141`
- Modify: `native/src/writers/tire_common.cpp:12-43`
- Create: `native/tests/terrain/test_surface_eval.cpp`
- Modify: `native/tests/terrain/CMakeLists.txt`

**Interfaces:**
- Consumes: `nativecfg::SurfaceSpec`, `nativecfg::SurfaceSection`, `nativecfg::SurfaceMap` from Task 1.
- Produces (in `namespace tirecommon`, `native/src/writers/tire_common.hpp`):
  ```cpp
  inline constexpr double kSurfaceReferenceLoadN = 500.0;
  inline constexpr double kSurfaceLoadSmoothDelta = 0.1;
  inline constexpr double kReferenceSlipStiffnessPerLoad = 15.0;
  inline constexpr double kWheelRollingTaperRadps = 0.2;

  struct SurfaceSample {
      std::string_view name{};
      double mu_peak = 0.0;
      double mu_slide = 0.0;
      double slip_stiffness_per_load = 0.0;
      double stribeck_speed_mps = 4.5;
      double crr_delta = 0.0;
      double load_sensitivity = 0.0;
  };

  [[nodiscard]] SurfaceSample sample_from_spec(const nativecfg::SurfaceSpec &s) noexcept;
  [[nodiscard]] SurfaceSample eval_surface(const nativecfg::SurfaceMap &map, double x_m);
  [[nodiscard]] const nativecfg::SurfaceSpec *
  uniform_surface_for_span(const nativecfg::SurfaceMap &map, double x_min_m, double x_max_m);
  [[nodiscard]] double load_sensitivity_factor(double wheel_normal_load_n, double k_z) noexcept;
  [[nodiscard]] double sample_mu(const SurfaceSample &s, double sliding_speed_mps,
                                 double wheel_normal_load_n);
  ```

- [ ] **Step 1: Write the failing unit test `native/tests/terrain/test_surface_eval.cpp`**

Register `terrain_surface_eval` in `native/tests/terrain/CMakeLists.txt` (linking `bike_writers` or compiling `tire_common.cpp` cleanly) testing:
1. Bitwise identity `eval_surface(map, x)` vs `sample_from_spec(surface_at(map, x))` outside transition bands and when `blend_m == 0.0`, plus `L.name == R.name` exact identity inside a nominal transition band.
2. $C^1$ continuity at both edges $x_b - 0.5 b$ and $x_b + 0.5 b$ (centered finite-difference slope $< 10^{-6}$ at boundaries, midpoint $x_b$ equals exact average `0.5 * (L + R)`, and `name == surface_at(map, x).name`).
3. `sec[0].start_m == 0.0` with `blend_m = 1.0`: at $x = -0.5, 0.0, +0.2$, `eval_surface` returns pure `sec[0].surface` with no blending at $x = 0$.
4. `uniform_surface_for_span`: returns `&sec.surface` when strictly inside `[start_m + 0.5*b_left, end_m - 0.5*b_right)`, and `nullptr` as soon as the span touches a transition window or section boundary.
5. `load_sensitivity_factor` & `sample_mu`: bitwise `1.0` for `wheel_normal_load_n <= 500.0` or `k_z == 0.0`, $C^1$-smooth at `500.0` N, and strictly increasing `F_z * sample_mu(s, v_s, F_z)` up to `F_z = 5000.0` N for `k_z = 0.5`.

- [ ] **Step 2: Build `terrain_surface_eval` to verify RED**

Run: `cmake --build build/native-release --target terrain_surface_eval`
Expected: FAIL (symbols `eval_surface`, `uniform_surface_for_span`, `sample_mu` undefined).

- [ ] **Step 3: Implement `eval_surface`, `uniform_surface_for_span`, `load_sensitivity_factor`, and `sample_mu`**

Implement in `native/src/writers/tire_common.hpp/.cpp` using zero heap allocations (iterating `map.sections` in place):
- Transition boundary scan over `map.sections`:
  - For section $i$: if $i == 0$: when `sec[0].start_m > 0.0 && sec[0].blend_m > 0.0`, boundary at `sec[0].start_m` of width `sec[0].blend_m` (`map.surface -> sec[0].surface`).
  - Between section $i$ and $i+1$ (or end of `sec[last]`): if $i + 1 < N$ and `sec[i].end_m == sec[i+1].start_m`, single boundary at `sec[i].end_m` of width `sec[i].blend_m` (`sec[i].surface -> sec[i+1].surface`); else boundary at `sec[i].end_m` of width `sec[i].blend_m` (`sec[i].surface -> map.surface`), and if $i + 1 < N$ with `sec[i+1].blend_m > 0.0`, boundary at `sec[i+1].start_m` of width `sec[i+1].blend_m` (`map.surface -> sec[i+1].surface`).
- Hermite smoothstep $s(t) = t * t * (3.0 - 2.0 * t)$ for $t = (x - x_0) / (x_1 - x_0) \in [0, 1]$, short-circuiting to `sample_from_spec(L)` when `L.name == R.name`.
- Keep `tirecommon::effective_friction` unchanged so `TireWriter` (`compliant_2d`) remains 100% untouched.

- [ ] **Step 4: Build & run `terrain_surface_eval` to verify GREEN**

Run:
```bash
cmake --build build/native-release --target terrain_surface_eval
ctest --test-dir build/native-release -R "^terrain_surface_eval$" --no-tests=error --output-on-failure
```
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add native/src/writers/tire_common.hpp native/src/writers/tire_common.cpp native/tests/terrain/test_surface_eval.cpp native/tests/terrain/CMakeLists.txt
git commit -m "feat(writers): add C1-smooth SurfaceMap evaluator and wheel-load friction law"
```

---

### Task 3: `DistributedTireWriter` Surface Physics (`r_k` Shear Scaling, Load Saturation & World Rolling Moment)

**Files:**
- Modify: `native/src/writers/distributed_tire.hpp:235-270`
- Modify: `native/src/writers/distributed_tire.cpp:240-296, 615-895`
- Test: `native/tests/reference/test_native_distributed_tire.cpp`

**Interfaces:**
- Consumes: `tirecommon::SurfaceSample`, `tirecommon::eval_surface`, `tirecommon::sample_mu`, `tirecommon::kReferenceSlipStiffnessPerLoad`, `tirecommon::kWheelRollingTaperRadps` from Task 2.
- Produces: updated `DistributedTireWriter` implementing Spec §3.2 with unchanged public C++ signature and unchanged 4-column `state()`/`set_state()` wire format.

- [ ] **Step 1: Add failing physics & energy tests in `native/tests/reference/test_native_distributed_tire.cpp`**

Add test functions verifying:
1. **Shear stiffness scaling ($3.33\times$ `asphalt` vs `scree`) at standstill ($v_{\text{roll}} = 0$) and rolling:** under an identical small horizontal displacement / spin step well below the friction limit, tangential force on `scree` (`C_k/F_z = 4.5`) equals $(4.5 / 15.0) = 0.3\times$ tangential force on `asphalt` (`15.0`) within `1e-9`.
2. **Energy conservation across a `blend_m = 0.4` transition & `state()`/`set_state()` roundtrip:** stepping a loaded wheel across `hardpack -> scree` (`blend_m = 0.4`) satisfies discrete passivity (`losses >= 0`, `stored_energy(d) == elastic_energy_j()` bitwise, and total mechanical work + $\Delta E_{\text{stored}}$ + `brush_loss_step_j` + `radial_dissipation_power_w * dt` balances to `< 1e-3` J).
3. **Wheel-level load sensitivity & world rolling moment:**
   - at wheel normal load $> 800$ N on `scree`, effective friction saturates below `0.45` (`diag.friction_coefficient < 0.38 + (0.45 - 0.38)`), whereas at `< 500` N it matches Stribeck exactly;
   - on `scree` (`crr_delta = 0.008`), rolling wheel receives negative world spin torque via `couple_world_nm[1]`, `qfrc` at root/frame pitch has zero artificial internal caliper reaction (`qfrc == sum(J_p^T F_patch + J_r^T couple_patch)`), and under rollback with forward wheel spin ($v_x < 0, \omega > 0$) `brush_loss_step_j() > 0` (strictly passive).

- [ ] **Step 2: Build & run `reference.native_distributed_tire` to verify RED**

Run:
```bash
cmake --build build/native-release --target reference.native_distributed_tire
./build/native-release/native/tests/reference/reference.native_distributed_tire
```
Expected: FAIL on the new assertions.

- [ ] **Step 3: Implement Spec §3.2 in `DistributedTireWriter`**

1. In `distributed_tire.hpp` and constructor (`distributed_tire.cpp:248-294`):
   - add `std::vector<double> contact_x_, slips_;` sized `n` at construction;
   - compute `surface_capacity` (minimum `std::string_view("configured").size()` = 10, plus `surface_map_` names) and `reserve` `diagnostics_[s].surface` and `diagnostics_stage_[s].surface`.
2. In `accumulate_validated` (`distributed_tire.cpp:649-890`), split the station loop per wheel into Pass 1 and Pass 2:
   - **Pass 1:** query geometry, build `jac_station_` and `velocity`, compute radial `station_response`, fill `loads_[j]`, `energies_[j]`, `depths_[j]`, `normals_[3*j..]`, `contact_x_[j] = geometry.point[0]`, `slips_[j] = dot3(velocity, tangent)`, and accumulate `radial_power` and `nonsmooth`. Compute `const double normal_total = numeric::numpy_sum(std::span<const double>(loads_).first(count));`.
   - **Pass 2:** for each `j < count` (`sid = active_ids_[j]`):
     - evaluate `SurfaceSample` when `surface_map_` is engaged: `mu = std::min(pcfg.mu, tirecommon::sample_mu(sample, slips_[j], normal_total))`, `r_k = sample.slip_stiffness_per_load / tirecommon::kReferenceSlipStiffnessPerLoad`, accumulate `crr_weighted_sum += loads_[j] * sample.crr_delta`, track max-load station's `sample.name`; when `!surface_map_`: `mu = pcfg.mu`, `r_k = 1.0`;
     - rescale `xi_in = (r_k == 1.0) ? transported : (transported / std::sqrt(r_k))`, call `tirecommon::brush_step(xi_in, slips_[j], dot3(center_velocity, tangent), loads_[j], k_station * r_k, mu, pcfg.relaxation_length_m, dt)`, set `xi_stored = (r_k == 1.0) ? xi_out : (xi_out * std::sqrt(r_k))`, accumulate `losses += brush_loss` and `shear_energy += 0.5 * k_station * pyfloat::pow(xi_stored, 2.0)`;
     - rebuild `jac_station_ = jac_center_ + jr_ × arm` and accumulate `out += jac_station_.T @ force`, staging `snap.patches`.
   - **World rolling moment:** after `snap.effective_radius_m` (`line 854`), if `normal_total > 0.0 && snap.effective_radius_m > 0.0 && crr_weighted_sum > 0.0`:
     - `const double crr_bar = crr_weighted_sum / normal_total;`
     - extract wheel Y-axis `axis` from `d->xmat` for `body`, compute `tmp_ = jr_.T @ axis` via `blas::dgemv`, `omega_world = blas::ddot(nv_, tmp_.data(), 1, qvel.data(), 1)`;
     - `const double m_roll = -crr_bar * normal_total * snap.effective_radius_m * std::tanh(omega_world / tirecommon::kWheelRollingTaperRadps);`
     - `for (std::size_t i = 0; i < nv; ++i) out[i] += m_roll * tmp_[i];`
     - distribute `snap.patches[j].couple_world_nm = {0.0, m_roll * (loads_[j] / normal_total), 0.0};`
     - `losses += std::max(0.0, -m_roll * omega_world) * dt;`
   - Assign `diagnostics_stage_[s].surface` from max-load station's name (or `eval_surface(*surface_map_, center[0]).name` when airborne, or `"configured"`).

- [ ] **Step 4: Verify new tests pass (temporarily skipping or before Task 6 golden re-record on the existing `hardpack` golden node)**

Run:
```bash
cmake --build build/native-release --target reference.distributed_tire_reference reference.native_distributed_stands reference.native_distributed_tire
ctest --test-dir build/native-release -R "^reference\.(distributed_tire_reference|native_distributed_stands)$" --no-tests=error --output-on-failure
```
Expected: `distributed_tire_reference` and `native_distributed_stands` (`configured` mode) pass 100% bitwise; new physics assertions in `test_native_distributed_tire` pass (golden node re-recorded in Task 6, or re-record `native_distributed_tire` here once verified).

- [ ] **Step 5: Commit**

```bash
git add native/src/writers/distributed_tire.hpp native/src/writers/distributed_tire.cpp native/tests/reference/test_native_distributed_tire.cpp
git commit -m "feat(distributed-tire): add energy-invariant surface shear scaling, load sensitivity and crr_delta moment"
```

---

### Task 4: `Pneumatic` Radial Energy Closure (Air-Chord Potential $W(d)$, Bregman–Legendre Maxwell Potential $U_M$, Single Rim Hysteresis)

**Files:**
- Modify: `native/src/pneumatic/carcass.hpp:150-265`
- Modify: `native/src/pneumatic/carcass.cpp:480-560`
- Modify: `native/src/pneumatic/model.hpp:78-118, 337-350`
- Modify: `native/src/pneumatic/model.cpp:463-520, 725-750`
- Modify: `native/src/writers/pneumatic_tire.hpp:290-335`
- Modify: `native/src/writers/pneumatic_tire.cpp:719-741, 1090-1106`
- Create: `native/tests/reference/test_pneumatic_energy_balance.cpp`
- Modify: `native/tests/reference/Phase3Ports.cmake`

**Interfaces:**
- Produces (in `namespace biketyre`, `native/src/pneumatic/carcass.hpp`):
  ```cpp
  // Cancellation-free integral W(d) = \int_0^d chord(u) du for d >= 0.
  [[nodiscard]] double casing_chord_integral_m2(double deflection_m, double casing_radius_m) noexcept;
  // Total per-ray elastic energy E_e(d) = (area_pressure * W(d) + 0.5 * k_c * d^2) * ds_m.
  [[nodiscard]] double ray_elastic_energy_j(double deflection_m,
                                            const nativecfg::PneumaticTyreSpec &tyre,
                                            double ds_m) noexcept;
  // Nonsingular Bregman-Legendre Maxwell stored energy U_M(d, xi_e) >= 0.
  [[nodiscard]] double ray_maxwell_energy_j(double deflection_m, double elastic_prev_n,
                                            double maxwell_force_n,
                                            const nativecfg::PneumaticTyreSpec &tyre,
                                            double ds_m) noexcept;
  ```
- Updates `CarcassBuffers` with `std::span<double> rim_hysteresis_force_n` (allocated in `PneumaticTyreModel::CarcassScratch`) and `WheelOutputs` with `double carcass_dissipated_power_w = 0.0;`.

- [ ] **Step 1: Create failing test `native/tests/reference/test_pneumatic_energy_balance.cpp` and register in `Phase3Ports.cmake`**

Write unit tests covering Spec §3.3.3:
1. **Grazing & deep deflection air-chord potential:** for $d \in \{0.0, 10^{-18}, 10^{-12}, 10^{-6}, 0.005, 0.030, 0.045\}$ m, `casing_chord_integral_m2(d, rho) >= 0.0`, continuous across $d = \rho$, and `ray_elastic_energy_j(d, spec, ds)` agrees with 10,000-step midpoint quadrature of `elastic_force_n` to `< 1e-11` J.
2. **Bregman–Legendre Maxwell closed cycle (`load -> hold -> unload -> airborne decay`):**
   - across static hold ($d = \text{const}$), `U_M(d, pre) - U_M(d, post) == P_M_loss * dt` to `< 1e-14` J;
   - across a full dynamic compression-rebound-airborne cycle (`advance_unloaded` down to `< 1e-12` J), $\int f_{\text{total}}\,\dot{d}\,dt$ equals $\Delta E_{\text{stored}} + \int P_{\text{carcass\_loss}}\,dt$ within `O(dt)` quadrature (`< 1e-3` J).
3. **Single rim-strike hysteresis:** for $d > \delta_{\text{rim}}$, `total_element_force_n` includes `rim_hysteresis` once (not twice), and `carcass_dissipated_power_w` matches `(carcass_hysteresis + rim_hysteresis) * material_rate_mps`.
4. **Bitwise `stored_energy(d) == elastic_energy_j()`** on `PneumaticTireWriter` after advancing loaded and airborne steps.

- [ ] **Step 2: Build `reference.pneumatic_energy_balance` to verify RED**

Run: `cmake --build build/native-release --target reference.pneumatic_energy_balance`
Expected: FAIL (missing radial energy helpers and `carcass_dissipated_power_w`).

- [ ] **Step 3: Implement Spec §3.3.3 in `carcass.cpp`, `model.cpp`, and `writers/pneumatic_tire.cpp`**

1. In `carcass.cpp`:
   - fix line 500: `out.hysteresis_force_n[i] = carcass_hysteresis; out.rim_hysteresis_force_n[i] = rim_hysteresis;` (`out.rim_force_n[i] = rim_elastic + rim_hysteresis` stays single-counted in `total`);
   - implement `casing_chord_integral_m2` using $\psi = 4\arcsin(\sqrt{d / (2\rho)}) \in (0, \pi]$ with the $\psi < 0.02$ Taylor expansion $\frac{1}{2}\rho^2 \frac{\psi^3}{6}(1 - \frac{\psi^2}{20} + \frac{\psi^4}{840})$ and exact branch $\frac{1}{2}\rho^2(\psi - \sin\psi)$ for $\psi \ge 0.02$, seamlessly joining $\frac{\pi}{2}\rho^2 + 2\rho(d - \rho)$ for $d \ge \rho$;
   - implement `ray_elastic_energy_j` and `ray_maxwell_energy_j` using the 4-step trigonometric Newton inversion $\delta^* = \rho(1 - \cos\phi)$ on $(2 p_{\text{area}}\rho\sin\phi + k_c\rho(1 - \cos\phi))\Delta s = \xi_e$ for $0 < \xi_e < f_e(\rho)$.
2. In `model.cpp`:
   - before `advance_unloaded` (`line 468`) and before `evaluate_carcass_into` (`line 480`), if `carcass_state_.initialized() && tyre_.rate_stiffening > 0.0`, sum pre-step Maxwell energy $\sum_i U_M(d_i, \xi_{e,i}^{\text{pre}})$ (`d_i = 0` on the airborne early-out, `hits.delta_m[i]` otherwise), and after `carcass_state_` updates sum $\sum_i U_M(d_i, \xi_{e,i}^{\text{post}})$, setting `maxwell_loss_w = std::max(0.0, pre_um - post_um) / dt_s`;
   - include `(carcass.hysteresis_force_n[i] + carcass.rim_hysteresis_force_n[i]) * carcass.material_rate_mps[i]` plus `maxwell_loss_w` in `carcass_loss_w`, and publish `out.carcass_dissipated_power_w = carcass_loss_w` (including in `write_unloaded` when `maxwell_loss_w > 0`).
3. In `writers/pneumatic_tire.cpp`:
   - update `radial_energy_j` to sum `ray_elastic_energy_j + ray_maxwell_energy_j + 0.5 * k_rim * overlap^2 * ds_m`;
   - replace line 1104 `radial_power += outputs.dissipated_power_w - brush_power_w;` with `radial_power += outputs.carcass_dissipated_power_w;`.

- [ ] **Step 4: Build & run `reference.pneumatic_energy_balance` to verify GREEN**

Run:
```bash
cmake --build build/native-release --target reference.pneumatic_energy_balance
ctest --test-dir build/native-release -R "^reference\.pneumatic_energy_balance$" --no-tests=error --output-on-failure
```
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add native/src/pneumatic/carcass.hpp native/src/pneumatic/carcass.cpp native/src/pneumatic/model.hpp native/src/pneumatic/model.cpp native/src/writers/pneumatic_tire.hpp native/src/writers/pneumatic_tire.cpp native/tests/reference/test_pneumatic_energy_balance.cpp native/tests/reference/Phase3Ports.cmake
git commit -m "fix(pneumatic): close radial energy accounting for air spring, Maxwell branch and rim hysteresis"
```

---

### Task 5: `Pneumatic` Surface Physics (Ray-Resolved Patches, `shear_energy_j`, World Rolling Moment & 60 m Strict Ride Gate)

**Files:**
- Modify: `native/src/pneumatic/brush.hpp:115-163`
- Modify: `native/src/pneumatic/brush.cpp:284-414`
- Modify: `native/src/pneumatic/model.hpp:59-117, 337-353`
- Modify: `native/src/pneumatic/model.cpp:519-750`
- Modify: `native/src/writers/pneumatic_tire.hpp:290-335`
- Modify: `native/src/writers/pneumatic_tire.cpp:290-300, 746-783, 940-1106`
- Modify: `native/tests/reference/test_pneumatic_tire_forces.cpp:82-92`
- Modify: `native/tests/reference/test_pneumatic_tire_oracle.cpp:64-73`
- Modify: `native/tests/reference/test_native_pneumatic_model.cpp:712-797`
- Modify: `native/tests/reference/test_pneumatic_energy_balance.cpp`

**Interfaces:**
- Consumes: `tirecommon::SurfaceSample`, `tirecommon::eval_surface`, `tirecommon::uniform_surface_for_span`, `tirecommon::load_sensitivity_factor`, `tirecommon::sample_mu`, `tirecommon::kWheelRollingTaperRadps` (Task 2) and radial energy accounting (Task 4).
- Produces:
  - `DiscretisedBrush::step_heterogeneous(std::span<const double> normal_loads_n, double half_length_m, double v_x_mps, double v_s_mps, double sigma_m, std::span<const tirecommon::SurfaceSample> samples, double wheel_normal_load_n, double dt_s)` using preallocated member scratch (`mu_scratch_`, `k_scratch_`, zero allocations).
  - `TyrePatchOutput::shear_energy_j` and `TyrePatchOutput::rolling_crr`, `WheelOutputs::rolling_crr`.
  - World-Jacobian rolling moment in `PneumaticTireWriter::evaluate_wheels` writing `couple_world_nm` on `snap.patches` and exact `rolling_power_w` into `diag.dissipated_power_w`, `diag.brush_loss_j`, and `brush_loss`.

- [ ] **Step 1: Extend `test_pneumatic_energy_balance.cpp` with failing surface & 60 m ride tests**

Add tests to `native/tests/reference/test_pneumatic_energy_balance.cpp`:
1. **World rolling moment & zero fake frame pitch reaction:** spinning wheel on `scree` (`crr_delta = 0.008` + `tread_loss_crr`) satisfies `qfrc == sum(J_p^T F_patch + J_r^T couple_patch)` across all `nv` DOFs, zero internal reaction on fork/swingarm pitch at zero hub translation, and positive loss even when `v_x < 0, omega > 0`.
2. **Smooth boundary crossing (`hardpack -> scree` with `blend_m = 0.0` AND `blend_m = 0.4`):** as a loaded wheel rolls across a boundary at `x = 5.0` m, tangential force `F_x(x)` transitions monotonically across the ~13 cm footprint without a single-step jump, and on `detailed` tier inside `blend_m = 0.4`, `stored_energy(d) == elastic_energy_j()` holds bitwise.
3. **Full 60 m ride strict energy gate on `examples/research/viewer_physics_pneumatic.toml`:** normal rider rides 60 m on flat/climb track with mixed `hardpack` / `scree` / `wet` zones (`blend_m = 0.4`), asserting `model_valid`, zero wheel-moment passivity violations, and strict relative energy residual `|residual_j| / energy_scale_j <= 0.05`.

- [ ] **Step 2: Build & run `reference.pneumatic_energy_balance` to verify RED**

Run: `cmake --build build/native-release --target reference.pneumatic_energy_balance && ./build/native-release/native/tests/reference/reference.pneumatic_energy_balance`
Expected: FAIL (60 m ride still has ~333 J residual from linear `tread_loss_force_n` and step jump at `centroid_world_m[0]`).

- [ ] **Step 3: Implement Spec §3.3.1–3.3.2 & update the 3 legacy `qfrc`/`tread_loss` test assertions**

1. Add `DiscretisedBrush::step_heterogeneous` in `pneumatic/brush.hpp/.cpp` (preallocating `k_scratch_` in constructor so `prewarm_detailed_brushes()` stays 100% allocation-free), keeping `DiscretisedBrush::step(..., const SurfaceSpec&, ...)` untouched so `reference.native_pneumatic_brush` stays bitwise identical.
2. In `PneumaticTyreModel` (`model.hpp/.cpp`):
   - preallocate `std::vector<tirecommon::SurfaceSample> sample_scratch_` of size `ring_.n()` in constructor;
   - pre-sum Euclidean patch normal loads across `carcass.patch_count` before the patch loop to obtain exact `wheel_normal_load_n`;
   - implement the uniform fast-path (`uniform_surface_for_span(surface_map, x_min, x_max)`) vs ray-weighted (`fast`) / `step_heterogeneous` (`detailed`) evaluation;
   - compute `po.shear_energy_j` at the end of each `detailed` patch step (`0.5 * k_px * dx_m * numpy_sum(u^2)` on uniform fast-path, `0.5 * numpy_sum(k_i * u_i^2)` on heterogeneous path);
   - remove `tread_loss_force_n` from `po.tangential_force_n` (`po.tangential_force_n = brush_result.force_n`, `po.dissipated_power_w = slip_loss_w`), and export `po.rolling_crr = tread_loss_crr_at_pressure(tyre_) + patch_crr_delta` and load-weighted `out.rolling_crr`.
3. In `PneumaticTireWriter` (`writers/pneumatic_tire.hpp/.cpp`):
   - preallocate `jr_.resize(3 * nv_)` in constructor;
   - simplify `shear_energy_j(tyre)` to sum committed `p.shear_energy_j` across matched `PatchBrushState` slots;
   - after `snap.effective_radius_m` (`line 997`), apply world rotational Jacobian moment $M_{\text{roll}} = -\text{outputs.rolling\_crr} \cdot \text{outputs.normal\_load\_n} \cdot R_{\text{eff}} \cdot \tanh(\omega_{\text{world}} / 0.2)$ via `jr_`, write `couple_world_nm` proportionally into `snap.patches`, and add `rolling_power_w = std::max(0.0, -M_roll * omega_world)` to `diag.dissipated_power_w`, `diag.brush_loss_j` (`* dt`), and `brush_loss` (`* dt`).
4. Update `test_pneumatic_tire_forces.cpp:87-91` and `test_pneumatic_tire_oracle.cpp:68-73` so `expected` includes `jr.T @ patch.couple_world_nm`, and update `test_native_pneumatic_model.cpp:712-797` to verify zero patch tangential offset at pure free rolling + exported `rolling_crr`.

- [ ] **Step 4: Build & run pneumatic tests to verify GREEN**

Run:
```bash
cmake --build build/native-release --target reference.pneumatic_energy_balance reference.native_pneumatic_brush reference.native_pneumatic_geometry reference.pneumatic_tire_forces
ctest --test-dir build/native-release -R "^reference\.(pneumatic_energy_balance|native_pneumatic_brush|native_pneumatic_geometry|pneumatic_tire_forces)$" --no-tests=error --output-on-failure
```
Expected: 100% PASS (including the 60 m ride strict energy balance gate!).

- [ ] **Step 5: Commit**

```bash
git add native/src/pneumatic/brush.hpp native/src/pneumatic/brush.cpp native/src/pneumatic/model.hpp native/src/pneumatic/model.cpp native/src/writers/pneumatic_tire.hpp native/src/writers/pneumatic_tire.cpp native/tests/reference/test_pneumatic_tire_forces.cpp native/tests/reference/test_pneumatic_tire_oracle.cpp native/tests/reference/test_native_pneumatic_model.cpp native/tests/reference/test_pneumatic_energy_balance.cpp
git commit -m "feat(pneumatic): add ray-resolved surface evaluation and world rolling-resistance moment"
```

---

### Task 6: Re-record Affected Goldens, Update Documentation & Run Completion Checks

**Files:**
- Modify: `tests/fixtures/golden/linux-aarch64/{native_pneumatic_carcass,native_pneumatic_model,native_pneumatic_tire,pneumatic_tire,native_distributed_tire,native_numeric_completion}/**`
- Modify: `docs/agents/simulation-decisions.md:835-850` (note closure of «Не решено», п. 2 for `pneumatic`)
- Modify: `docs/research/tyre-calibration.md` (§0 / §3.1 / §4.1 note on world-axle rolling moment and Stage A surface extensions)

**Interfaces:**
- Consumes: completed Stage A implementation from Tasks 1–5.
- Produces: green reference & regression suites on `linux-aarch64`, clean static analysis sweeps on touched TUs, and updated documentation.

- [ ] **Step 1: Re-record the 6 affected `linux-aarch64` golden targets & initial states**

Run with `BIKE_GOLDEN_RECORD=1` (and initial-state recording helper where used by `native_pneumatic_tire` / `native_distributed_tire`) ONLY for the 6 targets enumerated in Spec §4.1.3:
- `reference.native_pneumatic_carcass`
- `reference.native_pneumatic_model`
- `reference.pneumatic_tire_oracle`
- `reference.native_pneumatic_tire`
- `reference.native_distributed_tire`
- `reference.native_numeric_completion`

Verify `git status` touches ONLY `tests/fixtures/golden/linux-aarch64/` for those 6 suites (never `darwin-arm64/` and never `native_tire/` or `compliant_2d` goldens).

- [ ] **Step 2: Re-run all 6 targets WITHOUT `BIKE_GOLDEN_RECORD` plus `compliant_2d` non-interference & static checks**

Run:
```bash
ctest --test-dir build/native-release -R "^(terrain_.*|config_.*|reference\.(native_pneumatic_.*|pneumatic_.*|native_distributed_.*|distributed_tire_reference|native_tire|native_input_contracts|native_numeric_completion|tyre_road_profile|alpine_track|savage_track_surface))$" --no-tests=error --output-on-failure
```
Also build `bike-native-checks` and run standalone sweeps (`--kind scripts`, `--kind cppcheck`, `--kind headers`, `--kind contracts`) verifying zero findings on all touched files.

- [ ] **Step 3: Update `docs/agents/simulation-decisions.md` («Не решено», п. 2) and `docs/research/tyre-calibration.md`**

Record in `docs/agents/simulation-decisions.md` («Не решено», п. 2) that double-counted `crr` and radial/rolling energy accounting for `pneumatic` are closed in Stage A (`[resistance] crr = 0.0`, world-Jacobian rolling moment `tread_loss_crr_at_pressure + crr_delta`, air-chord $W(d)$ and Bregman–Legendre Maxwell energy $U_M$), and note the pending `darwin-arm64` twin golden refresh command for the 6 targets when merging on macOS. Update `docs/research/tyre-calibration.md` §3.1/§4.1 accordingly.

- [ ] **Step 4: Commit**

```bash
git add tests/fixtures/golden/linux-aarch64 docs/agents/simulation-decisions.md docs/research/tyre-calibration.md
git commit -m "test(golden),docs: refresh linux-aarch64 Stage A tyre goldens and record pneumatic energy closure"
```
