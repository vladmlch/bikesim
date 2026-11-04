# Remove Legacy Mode, the Stand and the Seated Rider — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
> **This session's user forbids subagents** ("не запускай агентов, делай все сам"): execute inline with superpowers:executing-plans.

**Goal:** Leave `physics_mode = "physical"` as the only simulation mode. Delete the old (legacy) simulation, the `bike-playground` stand with its two stand model modes, and the seated rider variant with its spring-dampers. The physical simulation stays bit-identical.

**Architecture:** First move the four helpers that physical code and tests borrow from `native/src/legacy/` into their real homes. Then cut the legacy branches out of each entry point (`bike-ride`, `bike-native-ride`, viewer, plots, `bike-sim`) and delete `native/src/legacy/` with its CMake targets, tests and fixtures. After that, narrow the configuration (`physics_mode`, suspension `legacy_behavior`) to the physical value only. Last, remove the seated rider variant and the `rider_forces` writer. Every task keeps the physical goldens unchanged; a changed physical golden stops the work.

**Tech Stack:** C++23, CMake/CTest, MuJoCo, the native test harness (`native/tests/support/fixture_test.hpp`, `bt::require`/`bt::invalid`/`bt::run`), golden fixtures under `tests/fixtures/golden/darwin-arm64/`.

**Spec:** No separate design document. The decisions below come from the user (2026-11-04) and from the investigation in this session. The normative model limits are in `AGENTS.md` ("Ограничения модели райдера и мотора").

## Decisions (user, 2026-11-04)

- D1. Legacy mode is removed as a separate task. The user runs only physical mode:
  `bike-ride --physics-config examples/research/viewer_physics_pneumatic.toml --track examples/research/rough_uphill_savage.toml`.
- D2. "переведи на режим physical": everything that stays runs in physical mode; nothing keeps a legacy code path.
- D3. The `bike-playground` stand is deleted ("Удалить стенд"), together with `bike-sim --playground`.
- D4. The seated rider variant (`--rider seated`) is deleted entirely, including from physical mode. What remains: `none`, `lumped`, `articulated_planar`. No variant can stand up from the saddle (AGENTS.md); `articulated_planar` is not affected.
- D5. The stand model modes go too ("да можно удалить", option 1, 2026-11-04). `bike-sim --export-mujoco` and `bike-export` stop writing `bike_playground_stand.xml` (mode `stand`) and `bike_playground.xml` (mode `playground`). They keep writing `bike_model.xml` (mode `standard`), `bike_ride.xml` (mode `ride`) and `coordinates.json`.

## Global Constraints

- Physical behaviour stays bit-identical. Every golden that a physical test consumes passes **without** `BIKE_GOLDEN_RECORD`. A golden deleted together with a deleted test is the only allowed golden change. The one exception is Task 11's sensor list, which is checked and explained separately.
- Linux is deferred ("линукс пока не делай"). `tests/fixtures/golden/linux-aarch64/` is edited only where a deleted test's golden directory is deleted on both platforms.
- No subagents. Commit only when the user asks. Branch from the committed `no-joint-damping` work (see Task 0).
- Run tests with `ctest --timeout 120` so a hang cannot block the session.
- Before native changes, read `docs/agents/native-engineering.md` (AGENTS.md rule 1).
- After every task, run `bash tools/check_change.sh <commit before the task>` (level 1). Run `bash tools/run_tests.sh full` once, at the end (Task 13). A run started with `--from` is partial and never counts as the final check.
- Reference audits ("is this the last user?") use mcp-steroid PSI when the session preflight says `reachable`. Otherwise use `git grep -n -w` and say in the task report that grep cannot prove absence (CLAUDE.md).
- The word "legacy" has other meanings that are **out of scope** and must not be touched: legacy recordings and replay inputs (`replay/legacy.cpp`, `tests/fixtures/replay/legacy_v1|legacy_v2`, `load_legacy_input`), `RetentionMode::legacy`, `ResearchCaptureMode::legacy` / `capture_mode='legacy'`, `configuration/legacy_battery.hpp`, the historical NPZ equilibrium inventory (`legacy_equilibrium_fingerprint`, `legacy_source_root`), `rider/legacy_allocation.cpp`, `rider/legacy_controller.cpp` (closed-form plan Task 11), and `model/legacy_topology.cpp` (it also builds the lumped rider; only its seated branch goes).

## Out of scope (state these in the final report)

- **Inert seated-rider fields of `RiderSpecs`:** `torso_resonance_hz`, `leg_resonance_hz`, `arm_resonance_hz`, `damping_ratio`, `saddle_interface_k_n_m`, `saddle_interface_c_ns_m`, and `legs`. Every recording writes them into its resolved config, and the equilibrium-cache and initial-state fingerprints hash the rider record (`initialization/equilibrium_cache.cpp:174`). Deleting them changes recorded bytes and makes stored states and caches invalid. That needs its own migration plan that follows the `legacy_battery.hpp` pattern. After this plan they have no effect: Task 11 adds a test that proves it.
- **The seated pose solver** (`solve_seated_pose`, `SeatedPose`, `seated_pose`, `spring_params`, `saddle_path_apparent_mass`). `geometry_pose()` (`model/rider_geometry.cpp:1228`) builds the `articulated_planar` geometry from it, and `compute_rider_centers_of_mass` uses it. It stays; renaming it is a separate task.
- Re-recording linux-aarch64.

## File map

Created:
- `native/src/runtime/run_limits.hpp` — header-only `bike::runtime::RunLimits` (moved from `legacy/controls.hpp`).
- `native/src/terrain/pothole_report.hpp`, `native/src/terrain/pothole_report.cpp` — `PotholeReport`, `pothole_reports()` (moved from `legacy/metrics.*`).
- `native/tests/reference/reference_tyres.hpp` — test-only reference tyres, tiers and the world-grid road profile (moved from `legacy/tyre.*`).
- `native/tests/runtime/test_run_limits.cpp`, `native/tests/terrain/test_pothole_report.cpp`.
- `docs/superpowers/audits/2026-11-04-legacy-inventory.md` — the reference inventory (Task 1).

Deleted:
- `native/src/legacy/` (all 19 files), `native/src/cli/playground.cpp`, `native/src/initialization/legacy_equilibrium.{hpp,cpp}`, `native/src/writers/rider_forces.{hpp,cpp}`.
- `native/tests/legacy/` (3 tests + CMakeLists), `native/tests/reference/test_native_rider_forces.cpp`.
- Fixtures: `tests/fixtures/config/legacy.toml`, `tests/fixtures/requests/legacy_flat.toml`, `tests/fixtures/bootstrap/legacy_flat/`, `tests/fixtures/metrics/filter_*`, `tests/fixtures/golden/*/native_rider_forces/`, `tests/fixtures/golden/*/native_suspension/legacy_env/` (only if Task 10's audit shows nothing physical reads it).

Modified (main ones): `native/src/cli/{ride,configuration,simulation,application,argument_contracts.inc}`, `native/src/app/native_ride_main.cpp`, `native/src/viewer/{ride.cpp,session.hpp,input.hpp,input.cpp}`, `native/src/viz/{ride.cpp,plot.hpp}`, `native/src/numerics/filter.{hpp,cpp}`, `native/src/runtime/{realtime_driver,project_config,capabilities,step}.cpp`, `native/src/stepper.{hpp,cpp}`, `native/src/configuration/{model_config.hpp,validate.cpp,schema.cpp,rider_spec.hpp}`, `native/src/{config_types,config_validation,writer_config}.hpp`, `native/src/writers/suspension.cpp`, `native/src/model/{builder,legacy_topology,sensors,mass,rider_geometry,environment,drivetrain,linkage,export}.cpp`, CMake files in `native/cmake/` and `native/tests/`, docs `docs/RIDE.md`, `docs/TESTING.md`, `docs/native-workflow.md`, `docs/development/{macos,linux}.md`.

---

### Task 0: Branch and baseline

**Files:** none in the tree. Evidence goes to `/tmp/rlm/`.

- [ ] **Step 1: Make sure the no-joint-damping work is committed.** That needs the user's go-ahead; if there is none, stop and ask. Run:

```bash
cd .worktrees/no-joint-damping && git status --short | grep -v '^??' | wc -l
```

Expected: `0`. Any other number means it is not committed yet.

- [ ] **Step 2: Create the worktree.**

```bash
git worktree add .worktrees/remove-legacy-mode -b remove-legacy-mode no-joint-damping
cd .worktrees/remove-legacy-mode
cmake --preset release && cmake --build build/native-release -j
```

- [ ] **Step 3: Record the baseline test list and result.**

```bash
mkdir -p /tmp/rlm
ctest --test-dir build/native-release -N | sed -n 's/^ *Test *#[0-9]*: //p' | sort > /tmp/rlm/tests-before.txt
ctest --test-dir build/native-release --timeout 120 -j8 -LE 'slow|package' --output-log /tmp/rlm/ctest-before.log
ctest --test-dir build/native-release --timeout 600 -j4 -L slow --output-log /tmp/rlm/ctest-slow-before.log
tail -3 /tmp/rlm/ctest-before.log /tmp/rlm/ctest-slow-before.log
```

Expected: `100% tests passed` in both. A red test here is fixed or reported before anything else.

- [ ] **Step 4: Record the physical reference ride (the user's command).**

```bash
build/native-release/bin/bike-ride --physics-config examples/research/viewer_physics_pneumatic.toml \
  --track examples/research/rough_uphill_savage.toml --headless --duration 8 --no-plots \
  > /tmp/rlm/ride-before.json 2> /tmp/rlm/ride-before.err; echo "exit=$?"
```

Record the exit code in `/tmp/rlm/notes.txt`. The JSON summary is compared byte for byte in Task 13; only fields that hold wall-clock time may differ. Write those field names into `/tmp/rlm/notes.txt` now. To find them, run the command a second time and `diff` the two outputs.

---

### Task 1: Reference inventory

**Files:**
- Create: `docs/superpowers/audits/2026-11-04-legacy-inventory.md`

- [ ] **Step 1: List every user of each symbol below outside the code being deleted.** Use PSI find-usages (`mcp-steroid`, recipe "call hierarchy / find usages" in `.claude/docs/mcp-steroid/recipes.md`) when reachable, otherwise `git grep -n -w <symbol> -- native tools docs examples`. Symbols: `bike::legacy` (namespace), `RunLimits`, `termination_reason`, `pothole_reports`, `PotholeReport`, `front_tyre`, `rear_tyre`, `tier_spec`, `TierSpec`, `TYRE_TIERS`, `TyreConfig`, `build_road_profile`, `first_crossing_reference`, `kGravityMps2`, `kRampExclusionM`, `legacy_lowpass`, `legacy_ride_filter`, `travel_plot`, `shaft_velocity_plot`, `acceleration_plot`, `tyre_plot`, `plot_ride`, `track_profile`, `run_legacy`, `run_playground`, `legacy_frame`, `viewer::Mode::legacy`, `viewer::Mode::playground`, `playground_command`, `make_playground`, `StaticEquilibriumContext`, `StaticEquilibriumOptions`, `BuildRequest::tyre_model`, `RiderForcesWriter`, `RiderForcesConfig`, `rider_forces`, `has_rider_forces`, `rider_forces_qfrc`, `build_seated_rider`, `saddle_top_z_for_rider`, `legacy_behavior`, `sd_legacy_damping_force`.

- [ ] **Step 2: Write the inventory file.** Use one table per symbol (file:line, kept or deleted, the task that handles it), and add a line naming the tool used (PSI or grep). Expected result from this session's grep (verify, do not trust): physical code uses only `RunLimits::for_track` (`cli/ride.cpp:130`, `runtime/realtime_driver.cpp:49,131`), `pothole_reports` (`viz/ride.cpp:131` inside `track_profile`, used by `bike-ride --preview`), and nothing else from `bike::legacy`. Tests use `front_tyre`, `rear_tyre`, `tier_spec`, `TYRE_TIERS`, `TyreConfig`, `build_road_profile`, `TyreForceApplier`.

- [ ] **Step 3: Stop if the inventory finds a physical user that this plan does not list.** Add a task for it before going on.

---

### Task 2: Move `RunLimits` to the runtime

**Files:**
- Create: `native/src/runtime/run_limits.hpp`
- Create: `native/tests/runtime/test_run_limits.cpp`
- Modify: `native/tests/runtime/CMakeLists.txt` (append the target)
- Modify: `native/src/cli/ride.cpp:130`, `native/src/runtime/realtime_driver.cpp:49,131` and the includes of both

**Interfaces:**
- Produces: `bike::runtime::RunLimits { double finish_x_m; std::int64_t max_steps; double max_wall_clock_s = 300.; void validate() const; static RunLimits for_track(double length_m, double start_x_m, double timestep_s, double max_wall_clock_s = 300., double min_speed_mps = 15. / 3.6); }` in `runtime/run_limits.hpp`.

- [ ] **Step 1: Write the failing test.**

```cpp
// test_run_limits.cpp — RunLimits::for_track sizes the physical ride's
// step cap and wall-clock watchdog. It moved out of the removed legacy
// controls unchanged; these cases pin the same expression and errors.
#include "support/fixture_test.hpp"
#include "runtime/run_limits.hpp"

#include <cstdint>

namespace {
using bike::runtime::RunLimits;

// The cap is twice the remaining distance at the minimum speed, truncated.
void cap_matches_the_formula() {
    const double dt = .0005, speed = 15. / 3.6;
    const auto limits = RunLimits::for_track(120., 5., dt);
    bt::require(limits.finish_x_m == 120., "finish is the track length");
    bt::require(limits.max_steps == static_cast<std::int64_t>(2. * (120. - 5.) / (speed * dt)),
                "step cap is 2*(length-start)/(min_speed*dt), truncated");
    bt::require(limits.max_wall_clock_s == 300., "default wall-clock cap is 300 s");
}

// Non-positive inputs and a start at or past the finish are rejected.
void rejects_bad_inputs() {
    bt::invalid([] { (void)RunLimits::for_track(120., 5., 0.); });
    bt::invalid([] { (void)RunLimits::for_track(120., 5., .0005, 300., 0.); });
    bt::invalid([] { (void)RunLimits::for_track(5., 5., .0005); });
    bt::invalid([] { (void)RunLimits::for_track(120., 5., .0005, 0.); });
}
} // namespace

int main() { return bt::run([] { cap_matches_the_formula(); rejects_bad_inputs(); }); }
```

Append to `native/tests/runtime/CMakeLists.txt` (same pattern as `runtime_encoding`; header-only, so no engine libraries are needed):

```cmake
# RunLimits (runtime/run_limits.hpp) — header-only step/wall-clock caps of
# the physical ride, moved from the removed legacy controls.
add_executable(runtime_run_limits test_run_limits.cpp)
set_target_properties(runtime_run_limits PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF)
target_link_libraries(runtime_run_limits PRIVATE bike_test_fixtures)
bike_apply_native_policy(runtime_run_limits)
target_include_directories(runtime_run_limits SYSTEM PRIVATE
  "${_test_repo_root}/native/src")
if(BUILD_TESTING)
  add_test(NAME runtime_run_limits COMMAND runtime_run_limits)
  set_tests_properties(runtime_run_limits PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:runtime_run_limits>")
endif()
```

- [ ] **Step 2: Run it and confirm it fails.**

Run: `cmake --build build/native-release --target runtime_run_limits`
Expected: compile error `runtime/run_limits.hpp: No such file or directory`.

- [ ] **Step 3: Create the header.** Copy the body from `legacy/controls.cpp:318-331` unchanged, apart from the error text:

```cpp
#pragma once
// RunLimits — the physical ride's finish position, step cap and wall-clock
// watchdog. Moved unchanged from the removed legacy controls (2026-11-04).

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>

namespace bike::runtime {
struct RunLimits {
    double finish_x_m{};
    std::int64_t max_steps{};
    double max_wall_clock_s = 300.;

    void validate() const {
        positive(finish_x_m, "finish_x_m");
        positive(max_wall_clock_s, "max_wall_clock_s");
        if (max_steps <= 0) throw std::invalid_argument("max_steps must be positive");
    }
    [[nodiscard]] static RunLimits for_track(double length_m, double start_x_m, double timestep_s,
            double max_wall_clock_s = 300., double min_speed_mps = 15. / 3.6) {
        positive(timestep_s, "timestep_s");
        positive(min_speed_mps, "min_speed_mps");
        const double distance = length_m - start_x_m;
        positive(distance, "remaining track length");
        const double count = 2. * distance / (min_speed_mps * timestep_s);
        if (!std::isfinite(count) || count >= static_cast<double>(std::numeric_limits<std::int64_t>::max()))
            throw std::invalid_argument("run step cap exceeds int64 range");
        RunLimits result{.finish_x_m = length_m, .max_steps = static_cast<std::int64_t>(count),
                         .max_wall_clock_s = max_wall_clock_s};
        result.validate();
        return result;
    }

private:
    static void positive(double value, std::string_view name) {
        if (!std::isfinite(value) || value <= 0.)
            throw std::invalid_argument(std::string(name) + " must be finite and positive");
    }
};
} // namespace bike::runtime
```

Before you save it, read `positive()` in `legacy/controls.cpp` and copy its exact condition and message. If it differs from the one above, use the original.

- [ ] **Step 4: Switch the two physical callers.** In `cli/ride.cpp:130` replace `legacy::RunLimits::for_track` with `runtime::RunLimits::for_track` and add `#include "../runtime/run_limits.hpp"`. In `runtime/realtime_driver.cpp:49,131` replace `bike::legacy::RunLimits` with `bike::runtime::RunLimits`, replace `#include "../legacy/controls.hpp"` with `#include "run_limits.hpp"`, and check that nothing else in that file used `controls.hpp`.

- [ ] **Step 5: Run the test and the physical gates.**

```bash
cmake --build build/native-release -j && ctest --test-dir build/native-release --timeout 120 -R 'runtime_run_limits|realtime|recording_cli|ride_backend_cli|cli_' --output-on-failure
```

Expected: all pass.

- [ ] **Step 6: Level-1 check, then commit (with the user's go-ahead).**

```bash
bash tools/check_change.sh HEAD
git add native/src/runtime/run_limits.hpp native/tests/runtime native/src/cli/ride.cpp native/src/runtime/realtime_driver.cpp
git commit -m "refactor(runtime): move RunLimits out of the legacy controls"
```

---

### Task 3: Move pothole reports to the terrain library

**Files:**
- Create: `native/src/terrain/pothole_report.hpp`, `native/src/terrain/pothole_report.cpp`
- Create: `native/tests/terrain/test_pothole_report.cpp`
- Modify: `native/cmake/NativeStartup.cmake` (add the `.cpp` to `bike_terrain`, next to the other `src/terrain/*.cpp` entries)
- Modify: `native/tests/terrain/CMakeLists.txt` (register the test like `test_presets.cpp`)
- Modify: `native/src/viz/ride.cpp:2,131`

**Interfaces:**
- Produces: `bike::terrain::PotholeReport { std::string label, kind; double start_m, length_m, declared_depth_mm, effective_drop_front_mm, effective_drop_rear_mm; }` and `std::vector<PotholeReport> bike::terrain::pothole_reports(const TrackSpec&)`.

- [ ] **Step 1: Write the failing test.** It pins what the track preview draws, using a preset that has potholes. Pick the preset during the step with `bike-ride --list-tracks` and `--dump-track NAME`, and choose one whose dump shows at least one pothole. Use that name in place of `road_worn` if `road_worn` has none.

```cpp
// test_pothole_report.cpp — pothole_reports() feeds the "declared depth vs
// effective wheel drop" panel of `bike-ride --preview`. It moved from the
// removed legacy metrics unchanged; this pins its contract.
#include "support/fixture_test.hpp"
#include "terrain/pothole_report.hpp"
#include "terrain/presets.hpp"
#include "terrain/profile.hpp"

namespace {
// One report per pothole, in track order, with drops from the wheel radii.
void reports_every_pothole_in_order() {
    const auto track = bike::terrain::get_preset("road_worn");
    const auto reports = bike::terrain::pothole_reports(track);
    std::size_t potholes = 0;
    for (const auto& obstacle : track.sorted_obstacles()) potholes += obstacle.is_pothole() ? 1 : 0;
    bt::require(potholes > 0 && reports.size() == potholes, "one report per pothole");
    for (std::size_t i = 1; i < reports.size(); ++i)
        bt::require(reports[i - 1].start_m <= reports[i].start_m, "reports follow track order");
    for (const auto& r : reports) {
        bt::require(r.declared_depth_mm >= 0. && r.length_m > 0., "depth and length are physical");
        bt::require(r.effective_drop_front_mm <= r.declared_depth_mm + 1e-9 &&
                    r.effective_drop_rear_mm <= r.declared_depth_mm + 1e-9,
                    "a wheel never drops deeper than the declared hole");
    }
}
} // namespace

int main() { return bt::run([] { reports_every_pothole_in_order(); }); }
```

Check the real header names first: `get_preset` and `sorted_obstacles` are declared where `cli/configuration.cpp` and `legacy/metrics.cpp` include them from (`terrain/trackfile.hpp`, `terrain/types.hpp`, `terrain/profile.hpp`). Fix the includes to match.

- [ ] **Step 2: Run it and confirm it fails.** Build the test target. Expected: `terrain/pothole_report.hpp: No such file`.

- [ ] **Step 3: Move the code.** `pothole_report.hpp` declares the struct and the function in `bike::terrain` (field order exactly as in `legacy/metrics.hpp:27-30`). `pothole_report.cpp` holds the body from `legacy/metrics.cpp` (`awk '/pothole_reports\(const terrain::TrackSpec/,/^}/' native/src/legacy/metrics.cpp`) with the `terrain::` qualifiers dropped. Its includes are `"pothole_report.hpp"`, `"profile.hpp"`, `<variant>`. Remove the declaration and the struct from `legacy/metrics.hpp` and make `legacy/metrics.*` use `terrain::PotholeReport` until Task 8 deletes them (`using PotholeReport = terrain::PotholeReport;`).

- [ ] **Step 4: Switch the viewer plot.** In `viz/ride.cpp:131` use `terrain::pothole_reports(track)`; add `#include "../terrain/pothole_report.hpp"`.

- [ ] **Step 5: Run the tests.** `ctest --test-dir build/native-release --timeout 120 -R 'terrain|viz|plots' --output-on-failure`. Expected: pass. `viz_plots` compares the preview figure's data, if it has a golden; it must not change.

- [ ] **Step 6: Level-1 check, then commit (with the user's go-ahead).** Message: `refactor(terrain): move pothole reports out of the legacy metrics`.

---

### Task 4: Move the reference tyres into test support

The pneumatic tyre tests use the two reference tyres (Magic Mary 29×2.4, Hans Dampf 27.5×2.4), the two tiers and the world-grid road profile from `legacy/tyre.*`. Physical code takes its tyres from `[tires.pneumatic]` in the physics config, so these become test data.

**Files:**
- Create: `native/tests/reference/reference_tyres.hpp`
- Modify: `native/tests/reference/pneumatic_tire_fixture.hpp:8,59-80`, `pneumatic_reference_support.hpp:42`, `test_native_pneumatic_carcass.cpp`, `test_native_pneumatic_model.cpp`, `test_tyre_road_profile.cpp:12,51,84-85`, `test_pneumatic_tire_forces.cpp:30,137`, `test_pneumatic_tire_oracle.cpp`
- Modify: `native/tests/reference/Phase3Ports.cmake:75,77,84,95,114` (drop `bike_legacy` from `LIBRARIES`)

**Interfaces:**
- Produces (namespace `bt::tyres`): `nativecfg::PneumaticTyreSpec front_tyre()`, `nativecfg::PneumaticTyreSpec rear_tyre()`, `struct TierSpec { std::string name; std::int64_t n_rays; double half_angle_deg, timestep_s; bool discretised_brush; }`, `TierSpec tier_spec(std::string_view)`, `inline constexpr std::array<std::string_view,2> kTiers{"fast","detailed"}`, `biketyre::RoadProfile build_road_profile(const terrain::TrackSpec&, const terrain::HeightFieldSpec&, const geometry::BikeSpecs&)`.

- [ ] **Step 1: Create `reference_tyres.hpp`.** Copy the values character for character from `legacy/tyre.cpp:23-30` (`front_tyre`, `rear_tyre`), `:43-47` (`tier_spec`) and `:62-69` (`build_road_profile`), as `inline` functions:

```cpp
#pragma once
// Reference tyres for the pneumatic tyre tests: the two catalogue tyres,
// the fast/detailed tiers and the world-grid road profile (ground plane +
// build_profile, before MuJoCo's binary32 heightfield quantization). Moved
// verbatim from the removed legacy tyre owner (2026-11-04); physical runs
// take their tyres from [tires.pneumatic] in the physics config instead.
#include "config_types.hpp"
#include "geometry/specs.hpp"
#include "pneumatic/model.hpp"
#include "terrain/profile.hpp"
#include "terrain/types.hpp"

#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace bt::tyres {
[[nodiscard]] inline nativecfg::PneumaticTyreSpec front_tyre() {
    return {.name="Schwalbe Magic Mary 29x2.4", .outer_radius_mm=372., .rim_radius_mm=320., .section_height_mm=57., .pressure_bar=1.5, .crr_reference=.011,
        .casing_width_mm=60., .compressed_casing_mm=6., .area_factor=.4435, .carcass_stiffness_n_mm2=.1194, .contact_length_factor=.8589, .rate_stiffening=.2484, .rate_relaxation_s=.2, .loss_factor=.0662, .hysteresis_rate_eps_mps=.01, .relaxation_length_mm=60., .rim_stiffness_n_mm2=28., .tread_loss_crr=.00586};
}
[[nodiscard]] inline nativecfg::PneumaticTyreSpec rear_tyre() {
    return {.name="Schwalbe Hans Dampf 27.5x2.4", .outer_radius_mm=352., .rim_radius_mm=300., .section_height_mm=55., .pressure_bar=1.7, .crr_reference=.0103,
        .casing_width_mm=60., .compressed_casing_mm=6., .area_factor=.4576, .carcass_stiffness_n_mm2=.1225, .contact_length_factor=.8801, .rate_stiffening=.2498, .rate_relaxation_s=.2, .loss_factor=.0690, .hysteresis_rate_eps_mps=.01, .relaxation_length_mm=60., .rim_stiffness_n_mm2=28., .tread_loss_crr=.00481};
}
struct TierSpec {
    std::string name;
    std::int64_t n_rays;
    double half_angle_deg, timestep_s;
    bool discretised_brush;
};
inline constexpr std::array<std::string_view, 2> kTiers{"fast", "detailed"};
[[nodiscard]] inline TierSpec tier_spec(std::string_view name) {
    if (name == "fast") return {.name="fast", .n_rays=64, .half_angle_deg=75., .timestep_s=.0005, .discretised_brush=false};
    if (name == "detailed") return {.name="detailed", .n_rays=256, .half_angle_deg=75., .timestep_s=.00025, .discretised_brush=true};
    throw std::invalid_argument("unknown reference tyre tier '" + std::string(name) + "'");
}
[[nodiscard]] inline biketyre::RoadProfile build_road_profile(const bike::terrain::TrackSpec& track,
        const bike::terrain::HeightFieldSpec& field, const bike::geometry::BikeSpecs& specs) {
    const auto x = field.track_x();
    auto z = bike::terrain::build_profile(track, x);
    const double ground = bike::geometry::compute_ground_z(specs) / 1000.;
    for (auto& height : z) height = ground + height;
    return {x.front(), x[1] - x[0], std::move(z)};
}
} // namespace bt::tyres
```

After writing it, run `diff <(sed -n 23,30p native/src/legacy/tyre.cpp) <(grep -A3 'front_tyre()\|rear_tyre()' native/tests/reference/reference_tyres.hpp)` and check by eye that every number matches.

- [ ] **Step 2: Switch the tests.** Replace `bike::legacy::front_tyre` → `bt::tyres::front_tyre`, `bike::legacy::rear_tyre` → `bt::tyres::rear_tyre`, `bike::legacy::tier_spec` → `bt::tyres::tier_spec`, `bike::legacy::TYRE_TIERS` → `bt::tyres::kTiers`, `bike::legacy::build_road_profile` → `bt::tyres::build_road_profile`, and `#include "legacy/tyre.hpp"` → `#include "reference_tyres.hpp"`. The files are listed under **Files**.

- [ ] **Step 3: `test_pneumatic_tire_forces.cpp:30`.** Delete the `static_assert(!std::is_constructible_v<PneumaticTireWriter, ..., bike::legacy::TyreConfig>)` line. The type it guards against no longer exists. Line 137 uses `bt::tyres::tier_spec("fast")`.

- [ ] **Step 4: `test_pneumatic_tire_oracle.cpp`.** The golden `pneumatic_tire_forces/oracle_parity_with_xfrc_writer/<tier>` is written only from the physical writer (`rig`). The legacy `TyreForceApplier` only supplied the second half of two cross-checks. So:
  - delete `legacy_rig`, `config`, `legacy`, `legacy->apply(...)`, `legacy_rig.engine.data->time += dt`, and `other` with its two `bt::require`/`numeric_equal` lines;
  - keep every `vector(node, ...)`/`scalar(node, ...)`/`keys(...)` call, `add_jtf(...)` and `numeric_equal(actual, expected)`: the generalized force must still equal the sum of Jᵀ·F over the writer's own patch outputs;
  - keep the node name, so the golden is read unchanged;
  - change the loop to `for (const auto tier : bt::tyres::kTiers)`;
  - update the comment above the test: "the qfrc writer's carcass, patch and output state per step, and qfrc equal to the sum of the patch wrenches mapped through the Jacobian. The legacy xfrc-writer parity half was removed with legacy mode (2026-11-04)."

- [ ] **Step 5: Drop `bike_legacy` from the five `LIBRARIES` lists in `Phase3Ports.cmake`.**

- [ ] **Step 6: Run the tyre tests in compare mode.**

```bash
cmake --build build/native-release -j && ctest --test-dir build/native-release --timeout 600 -R 'pneumatic|tyre|tire' --output-on-failure
```

Expected: all pass with no golden change (`git status tests/fixtures` clean).

- [ ] **Step 7: Level-1 check, then commit (with the user's go-ahead).** Message: `test(tyre): own the reference tyres outside legacy code`.

---

### Task 5: `bike-ride` runs physical only

**Files:**
- Modify: `native/src/cli/ride.cpp` (delete `drivetrain_value`, `ride_track_seed`, `legacy_directory_name`, `legacy_options`, `legacy_headless`, `legacy_run`; includes `../legacy/metrics.hpp`, `../legacy/ride.hpp`; final dispatch line 355)
- Modify: `native/src/cli/configuration.cpp:73,84-88,103,123-170`
- Modify: `native/src/cli/argument_contracts.inc` (`bike-ride` options)
- Test: `native/tests/cli/test_capture.cpp:44`, `native/tests/reference/test_recording_cli.cpp:103,155,190`, `native/tests/reference/test_ride_backend_cli.cpp:187,191`, plus the new cases below

- [ ] **Step 1: Write the failing tests** (in `native/tests/reference/test_ride_backend_cli.cpp`, next to the existing `resolve(...)` cases; reuse its `resolve` helper and `bt::invalid`):

```cpp
// Legacy mode is gone: `--physics legacy` is not an accepted choice and the
// legacy-only drivetrain/tyre flags are no longer bike-ride options.
void legacy_mode_is_not_selectable() {
    bt::invalid([] { (void)resolve({"--physics", "legacy"}); });
    for (const auto* flag : {"--drive-mode", "--tyre-model", "--tyre-tier", "--tyre-pressure",
                             "--surface", "--gearing", "--ripple-depth", "--legs"})
        bt::invalid([flag] { (void)resolve({flag, "x"}); });
    bt::invalid([] { (void)resolve({"--visual-pedalling"}); });
}
// Without --physics-config or --physics the ride resolves to physical mode.
void default_ride_is_physical() {
    const auto ride = resolve({"--duration", "1"});
    bt::require(ride->setup.model.physics.value().physics_mode == "physical",
                "an unconfigured bike-ride is a physical run");
    bt::require(ride->setup.model.rider.value().variant == "lumped",
                "the default physical rider is lumped");
}
```

Call both from `main`. Check the `resolve` helper's real signature in the file before you paste this, and adapt the call syntax to it.

- [ ] **Step 2: Run and confirm they fail.** `ctest ... -R ride_backend_cli --output-on-failure`. Expected: `default_ride_is_physical` fails (the default is still legacy), and `--drive-mode x` is still parsed.

- [ ] **Step 3: Change the parser contract.** In `argument_contracts.inc`, under `"bike-ride"`:
  - delete the option objects whose `dest` is `legs`, `tyre_model`, `tyre_tier`, `tyre_pressure`, `drive_mode`, `gearing`, `ripple_depth`, `visual_pedalling`, `surface`;
  - change `physics`'s `"choices"` to `["physical"]`.
  Keep `assist`, `crank_phase`, `speed`, `decimate` and `sag`: physical mode uses them (`configuration.cpp:76-88`, `ride.cpp:122`). After the edit, `grep -c '"dest"'` on the `bike-ride` block must drop by exactly 9.

- [ ] **Step 4: Change `cli/configuration.cpp`.**
  - `load_ride_physics`: when no `--physics-config` is given, start from `configuration::default_physics()` (from Task 9 on, that is physical). Until Task 9 lands, set `physics_mode = "physical"` on the default-constructed config here and add the comment `// physical is the only mode (legacy removed 2026-11-04)`.
  - `ride_physics`: the `--assist` override is applied unconditionally (the `physics_mode == "physical" &&` condition goes).
  - `resolve_ride`: delete the `else { ... }` legacy branch (lines 150-160) and the legacy-only checks inside `if (physical)` (`drive_mode`/`visual_pedalling`, `tyre_*`/`surface`, `gearing`/`ripple_depth`, `legs`), because the parser now rejects those flags. Drop the `if (physical)` wrapper itself. Replace every `!physical` condition with its physical outcome, so `--time-scale`, `--close-on-finish`, `--backend`, `--record` and `--check-policy` keep only their preview/headless checks. Delete the defaults for `tyre_tier`/`tyre_pressure` (lines 162-163).
  - `ride_setup_request`: the `physical` parameter goes away; `request.model.crank_joint = true`. Change the rider fallback `"seated"` (line 103) to `"lumped"`; `resolve_ride` already sets the physical default before this point.

- [ ] **Step 5: Change `cli/ride.cpp`.** Delete the legacy functions listed under **Files** and their includes. Change the last line of `ride_command` to `return physical_run(std::move(resolved));`. Remove `ride_track_seed` only if Task 1's inventory confirms that `legacy_run` was its only caller.

- [ ] **Step 6: Update the existing tests that passed `--physics legacy`.**
  - `test_capture.cpp:44`: delete the line (`--close-on-finish` with legacy mode can no longer be formed).
  - `test_recording_cli.cpp:103,155,190` and `test_ride_backend_cli.cpp:187,191`: delete those cases. Each checked that physical-only flags reject legacy mode, and that combination no longer exists. If one of them also checks something that still applies (for example the `--preview` half of a message), keep that half with `--preview`.

- [ ] **Step 7: Run the CLI tests.** `ctest --test-dir build/native-release --timeout 120 -R 'cli|capture|recording_cli|ride_backend|argument' --output-on-failure`. Expected: pass, including the two new cases.

- [ ] **Step 8: Run the physical reference ride and compare.**

```bash
build/native-release/bin/bike-ride --physics-config examples/research/viewer_physics_pneumatic.toml \
  --track examples/research/rough_uphill_savage.toml --headless --duration 8 --no-plots > /tmp/rlm/ride-t5.json; echo "exit=$?"
diff /tmp/rlm/ride-before.json /tmp/rlm/ride-t5.json
```

Expected: the same exit code; the diff shows only the wall-clock fields recorded in Task 0.

- [ ] **Step 9: Level-1 check, then commit (with the user's go-ahead).** Message: `feat(ride): bike-ride runs physical mode only`.

---

### Task 6: `bike-native-ride` runs physical only

**Files:**
- Modify: `native/src/app/native_ride_main.cpp:106-107,136-178,195-196,236-241` and its option parser (`--legacy-drive`, `--assist-mode`, `--legacy-tyre`, `--legacy-tyre-tier`)
- Modify: `native/cmake/NativePhysicsResearch.cmake:70` (drop `bike_legacy` from `bike-native-ride`)
- Test: the CTest cases that run `bike-native-ride`. Find them with `git grep -n "bike-native-ride\|legacy-drive\|legacy-tyre" -- native/tests tools`

- [ ] **Step 1: Write the failing test.** Find the existing test for `bike-native-ride` argument parsing (with the grep above). Add a case that runs the program with `--physics-config tests/fixtures/config/<a physical TOML> --legacy-drive motor` and expects exit code 2 with `unknown option` on stderr; copy the string the parser really prints for an unknown flag. If no such test exists, add one to `native/tests/cli/test_capture.cpp` using `bike::platform::run` the same way as line 118.

- [ ] **Step 2: Run and confirm it fails** (today the flag is accepted, then rejected for physical with "legacy drive/tyre/assist options require physics_mode='legacy'").

- [ ] **Step 3: Edit `native_ride_main.cpp`.**
  - Delete `run_legacy`, the four legacy fields of `Options` and their parsing.
  - Delete the two "Legacy mode:" lines from `usage()`.
  - Delete the check at the top of `run_physical`.
  - Change the rider default (line 106) to `options.rider.value_or("articulated_planar")`.
  - Make `main` call `run_physical` directly.
  - If `run_physical` needs a guard that the loaded config is physical, Task 9's validation already gives it, so add none.
  - Remove `#include` lines for `legacy/*`.

- [ ] **Step 4: Drop `bike_legacy` from line 70 of `NativePhysicsResearch.cmake`.**

- [ ] **Step 5: Run the tests.** `ctest --test-dir build/native-release --timeout 120 -R 'native_ride|bootstrap|welded|capture' --output-on-failure`. Expected: pass. `reference.welded_profile_config` and the `native_start_welded` golden must not change.

- [ ] **Step 6: Level-1 check, then commit (with the user's go-ahead).** Message: `feat(native-ride): drop the legacy run path`.

---

### Task 7: Viewer and plots without legacy and the stand

**Files:**
- Modify: `native/src/viewer/session.hpp:58-60`, `native/src/viewer/ride.cpp:185-241`, `native/src/viewer/input.hpp:9`, `native/src/viewer/input.cpp`
- Modify: `native/src/viz/ride.cpp` (delete `travel_plot`, `shaft_velocity_plot`, `acceleration_plot`, `tyre_plot`, `plot_ride`, and the helpers only they use: check `percentile`, `evidence_markers`, `add`), `native/src/viz/plot.hpp:77-80,86`
- Modify: `native/src/numerics/filter.{hpp,cpp}` (delete `legacy_lowpass`, `legacy_ride_filter`, if Task 1 shows no other user)
- Modify: `native/cmake/RecordingApplication.cmake:88,120` (drop `bike_playground` from `bike_viewer` and `bike_legacy` from `bike_viz`)
- Test: `native/tests/viewer/test_hud.cpp`, `native/tests/viz/test_plots.cpp`

- [ ] **Step 1: Change the tests first.** In `test_hud.cpp` delete the cases that build `viewer::Mode::legacy` HUDs (the `Mode::playground` cases go in Task 8). In `test_plots.cpp` delete the cases for the five removed plot functions. Keep the cases for `track_profile`, `physical_ride_plots`, `physical_ride_figures` and `plot_physical_ride_html`. Run `ctest -R 'hud|plots'` and confirm the remaining cases still pass before touching the sources.

- [ ] **Step 2: Remove the legacy viewer path.** Delete `legacy_frame` and `run_legacy` from `viewer/ride.cpp` and their declarations from `session.hpp`. Remove `legacy` from `enum class Mode` in `input.hpp`, and every `case`/comparison on it in `input.cpp` and `viewer/ride.cpp`. **Keep `run_playground` and `Mode::playground` for now:** `cli/playground.cpp` still calls them until Task 8 deletes the stand, so removing them here would break the build between tasks. Leave `Mode::research`, `inspection` and `checked_replay` untouched. Changing the enum's underlying values is fine; it is never serialized (check with `git grep -n "Mode::" -- native/src/viewer native/src/recording` before deleting).

- [ ] **Step 3: Remove the legacy-only plots** and their declarations. Delete the `#include "../legacy/tyre.hpp"` and `"../legacy/metrics.hpp"` lines from `viz/ride.cpp` (Task 3 already moved `pothole_reports`). Delete the legacy filters from `numerics/filter.*` and the `numerics_*` test cases that cover only them, if there are any.

- [ ] **Step 4: Fix the CMake link** of `bike_viz` (`RecordingApplication.cmake:120`: drop `bike_legacy`). `bike_viewer` keeps `bike_playground` until Task 8.

- [ ] **Step 5: Build and test.** `cmake --build build/native-release -j && ctest --test-dir build/native-release --timeout 120 -R 'viewer|hud|viz|plots|numerics|preview' --output-on-failure`. Expected: pass. Then run `bike-ride --preview --track examples/research/rough_uphill_savage.toml --out /tmp/rlm/preview` and check that the PNG is written (exit code 0).

- [ ] **Step 6: Level-1 check, then commit (with the user's go-ahead).** Message: `refactor(viewer): drop the legacy and stand viewer modes`.

---

### Task 8: Delete the stand and the legacy library

**Files:**
- Delete: `native/src/legacy/` (all), `native/src/cli/playground.cpp`, `native/src/initialization/legacy_equilibrium.{hpp,cpp}`
- Delete: `native/tests/legacy/` (all), `tests/fixtures/requests/legacy_flat.toml`, `tests/fixtures/bootstrap/legacy_flat/`, `tests/fixtures/metrics/filter_*`
- Modify: `native/cmake/NativePhysicsResearch.cmake:58-78` (delete `bike_legacy`, `bike_playground`), `native/cmake/NativeStartup.cmake:83` (drop `legacy_equilibrium.cpp`), `native/cmake/RecordingApplication.cmake:171,180` (drop `cli/playground.cpp` and `bike-playground`), the `add_subdirectory(legacy)` line in the tests CMake (find it with `git grep -n "legacy" -- native/tests/CMakeLists.txt`)
- Modify: `native/src/cli/application.cpp:20`, `native/src/cli/application.hpp` (`playground_command` declaration), `native/src/cli/simulation.cpp:15,28`, `native/src/cli/argument_contracts.inc` (the `"bike-playground"` block at line 117 and the `bike-sim` `--playground` option at line 70)
- Modify: the packaging and install lists that name `bike-playground` (`git grep -n "bike-playground" -- . ':!docs/superpowers/**'`)
- Modify: the structural contracts check, if it lists `native/src/legacy` or `bike-playground` (`git grep -n "legacy/\|bike-playground\|bike_legacy" -- tools native/tools native/src/checks 2>/dev/null`)

- [ ] **Step 1: Write the failing test.** In the test that checks the executable list, or in `test_capture.cpp`, add a check that `bike-sim --playground` is rejected as an unknown option, and that `bike-playground` is not among the installed or built commands. Find the command-list test with `git grep -n "bike-batch" -- native/tests`; it lists all eight commands.

```cpp
// The stand simulator was removed (2026-11-04): bike-sim has no
// --playground option and bike-playground is not a built command.
bt::invalid([] { (void)parse("bike-sim", {"--playground"}); });
```

- [ ] **Step 2: Run and confirm it fails.**

- [ ] **Step 3: Delete the files, edit the CMake files and edit the dispatch.** In `simulation.cpp` drop `args.flag("playground")` from `defaults` and make the function return `0`. In `application.cpp` delete the `bike-playground` line. In the viewer, delete `run_playground` (`viewer/ride.cpp`, `session.hpp`), remove `playground` from `enum class Mode` (`input.hpp`) with every `case`/comparison on it (`input.cpp`, `viewer/ride.cpp`), delete the `Mode::playground` cases in `native/tests/viewer/test_hud.cpp`, and drop `bike_playground` from `bike_viewer` (`RecordingApplication.cmake:88`).

- [ ] **Step 4: Prove nothing still includes the deleted code.**

```bash
git grep -n 'legacy/\(controls\|force_accumulator\|legs\|metrics\|playground\|recorder\|ride\|rider\|static_equilibrium\|suspension_controller\|telemetry\|tyre\)\|legacy_equilibrium.hpp\|bike_legacy\|bike_playground\|bike::legacy\|playground_command' -- native tools packaging CMakeLists.txt CMakePresets.json
```

Expected: no output. Then run the PSI check from Task 1 for `bike::legacy`, if PSI is available.

- [ ] **Step 5: Configure from scratch, build, list the tests.**

```bash
cmake --preset release --fresh && cmake --build build/native-release -j
ctest --test-dir build/native-release -N | sed -n 's/^ *Test *#[0-9]*: //p' | sort > /tmp/rlm/tests-t8.txt
diff /tmp/rlm/tests-before.txt /tmp/rlm/tests-t8.txt
```

Expected: the diff removes only `legacy_metrics`, `legacy_playground`, `legacy_ride` and adds `runtime_run_limits`, `terrain_pothole_report` (or whatever name you registered).

- [ ] **Step 6: Run all ordinary tests.** `ctest --test-dir build/native-release --timeout 120 -j8 -LE 'slow|package' --output-on-failure`. Expected: 100% passed, and `git status --short tests/fixtures` shows only the deletions above.

- [ ] **Step 7: Level-1 check, then commit (with the user's go-ahead).** Message: `refactor: delete the legacy simulation and the stand`.

---

### Task 8a: Remove the stand model modes (D5)

The MJCF builder still has two modes that exist only for the stand. `stand` puts the bike on a stand post with position actuators on the suspension; `playground` adds a weld `stand_clamp` that fixes the frame to the world, plus debug markers. Only the deleted stand simulator and `export_playground_models` used them.

**Files:**
- Modify: `native/src/model/export.cpp:174-186` (write only `bike_model.xml` and `bike_ride.xml`; rename `export_playground_models` → `export_models` in `export.cpp`, `export.hpp:13-17`, `cli/export.cpp:12`, `cli/simulation.cpp:27`, using the IDE rename when PSI is reachable)
- Modify: `native/src/model/builder.cpp:53` (timestep branch), `native/src/model/environment.cpp:15-20` (stand geoms), `native/src/model/drivetrain.cpp:80-83` (stand actuators), `native/src/model/linkage.cpp:74,180` (mode list; `stand_clamp` weld), `native/src/model/legacy_topology.cpp:132` (lumped rider condition)
- Modify: `native/src/model/builder.hpp:28` and its readers `legacy_topology.cpp:115`, `linkage.cpp:97,117` — delete `debug_markers` only if Task 1's inventory shows that the playground export was its only product setter
- Modify: the `bike-sim` and `bike-export` option help in `argument_contracts.inc`, if it names the stand files
- Test: `native/tests/model/test_export.cpp:34-40,76-77,84`, `native/tests/model/test_mjcf.cpp:181-183,224-225`

- [ ] **Step 1: Write the failing tests** (in `test_export.cpp`):

```cpp
// The stand model modes were removed with the stand (2026-11-04): the
// builder accepts only the free-standing and ride models, and the export
// writes exactly those two MJCF files plus nothing stand-related.
void stand_modes_are_gone() {
    for (const char* mode : {"stand", "playground", "no_such_mode"}) {
        mdl::BuildRequest request; request.mode = mode;
        bt::invalid([&] { (void)mdl::generate_mjcf(request); });
    }
}
void export_writes_only_free_and_ride_models() {
    const auto out = bt::temporary_directory("export_models");   // the file's existing temp-dir helper
    mdl::export_models(mdl::BuildRequest{}, out);
    for (const char* name : {"bike_model.xml", "bike_ride.xml"})
        bt::require(std::filesystem::is_regular_file(out / name), std::string(name) + " is exported");
    for (const char* name : {"bike_playground_stand.xml", "bike_playground.xml"})
        bt::require(!std::filesystem::exists(out / name), std::string(name) + " is no longer exported");
}
```

Use the temp-directory helper that `test_export.cpp` already uses around line 84, under its real name.

- [ ] **Step 2: Run and confirm they fail.** `ctest --test-dir build/native-release --timeout 120 -R 'model_export' --output-on-failure`. Expected: `stand` still builds, and `export_models` is not declared.

- [ ] **Step 3: Make the builder reject unknown modes.** At the top of `generate_mjcf` (`builder.cpp`, before line 16), add:

```cpp
    // Two model modes remain: the free-standing bike ("standard") and the
    // ride plant ("ride"). The stand modes were removed with the stand
    // simulator (2026-11-04); an unknown mode fails instead of silently
    // building the standard model.
    if(request.mode!="standard" && request.mode!="ride") throw std::invalid_argument("unknown model mode '"+request.mode+"'; available: standard, ride");
```

- [ ] **Step 4: Delete the stand branches.**
  - `builder.cpp:53`: drop the `stand || playground` timestep branch.
  - `environment.cpp:15-20`: delete the stand geoms.
  - `drivetrain.cpp:80-83`: delete the stand actuator block; the `else if(mode=="ride")` becomes `if`.
  - `linkage.cpp:74`: the condition becomes `mode=="ride"`. First check with `git grep -n '"dynamic"' -- native` that `"dynamic"` is set nowhere; if it is set somewhere, stop and report.
  - `linkage.cpp:180`: delete the `stand_clamp` weld.
  - `legacy_topology.cpp:132`: the condition becomes `c.request.mode=="ride"`.
  - Change `export.cpp` and rename the function as listed under **Files**.

- [ ] **Step 5: Update the old tests.** In `test_export.cpp:34-40`, loop over `{"standard","ride"}` and delete the `stand_clamp` assertion. At `:76-77`, delete the `mode="stand"` rejection: Step 1's test now covers it. At `:84`, the expected file list becomes `{"bike_model.xml","bike_ride.xml"}`. In `test_mjcf.cpp`, delete the three `stand`/`playground` cases of `no_joint_damping_in_any_mode()` (`:181-183`); keep `errors_and_ownership`'s `mode="stand"` rejection (`:224-225`), which still holds.

- [ ] **Step 6: Run the model tests, then all ordinary tests.**

```bash
cmake --build build/native-release -j
ctest --test-dir build/native-release --timeout 120 -R 'model|export|mjcf|inertia' --output-on-failure
ctest --test-dir build/native-release --timeout 120 -j8 -LE 'slow|package' --output-on-failure
git status --short tests/fixtures
```

Expected: pass, and no fixture is modified. A ride-mode golden must not change: no stand branch reaches mode `ride`.

- [ ] **Step 7: Run the export by hand.** `build/native-release/bin/bike-export --output-dir /tmp/rlm/export && ls /tmp/rlm/export` (check the real option name with `--help`). Expected: `bike_model.xml`, `bike_ride.xml`, `coordinates.json`.

- [ ] **Step 8: Level-1 check, then commit (with the user's go-ahead).** Message: `refactor(model): drop the stand model modes`.

---

### Task 9: `physics_mode` accepts only "physical"

**Files:**
- Modify: `native/src/configuration/model_config.hpp:46`, `native/src/configuration/validate.cpp:1037-1080`, `native/src/runtime/capabilities.cpp:16-17`, `native/src/model/builder.cpp:16-18` (the legacy pneumatic `tyre_model` check), `native/src/model/builder.hpp` (`BuildRequest::tyre_model`, if Task 1 shows only legacy users), `native/src/model/{drivetrain,linkage,export}.cpp` (their `tyre_model` branches)
- Delete: `tests/fixtures/config/legacy.toml`
- Test: `native/tests/config/test_schema.cpp` (`:182`, `:587`, `:650`, `:774`, `:1012`, `:1133`, `:1173`), `native/tests/config/test_resolution.cpp:318,559-571,734-745`, `native/tests/reference/test_native_runtime_config.cpp`, `native/tests/model/test_mjcf.cpp`, `native/tests/model/test_export.cpp:78`

- [ ] **Step 1: Write the failing tests** (in `test_schema.cpp`):

```cpp
// Legacy mode was removed (2026-11-04): physical is the declared default
// and the only accepted value; an old "legacy" document fails loudly.
void physics_mode_is_physical_only() {
    cfg::SimulationPhysicsConfig declared;
    bt::require(declared.physics_mode == "physical", "the declared default mode is physical");
    auto resolved = cfg::default_physics();
    bt::require(resolved.physics_mode == "physical" && resolved.drive_mode == "coast" &&
                resolved.pitch_assist == false, "defaults resolve as a physical coast run");
    auto old = cfg::default_physics();
    old.physics_mode = "legacy";
    bt::invalid([&] { cfg::post_init(old); });
}
```

- [ ] **Step 2: Run and confirm it fails.**

- [ ] **Step 3: Change the configuration.**
  - `model_config.hpp:46`: `std::string physics_mode = "physical";`
  - `validate.cpp`: change the check to `if (config.physics_mode != "physical") fail("unknown physics_mode '" + config.physics_mode + "'; legacy mode was removed, use physical");`. Delete `const bool legacy` and every branch on it: the closure check becomes unconditional, `drive_mode` defaults to `"coast"`, `pitch_assist` defaults to `false`, and the "effort drive modes require physical" check goes.
  - `capabilities.cpp:17`: change the message to `"physical runtime requires 'physical'"`.
  - `builder.cpp`: delete the `tyre_model` checks. If Task 1 shows `BuildRequest::tyre_model` is read only by the legacy pneumatic MJCF branches, delete the field and those branches in `drivetrain.cpp`, `linkage.cpp` and `export.cpp`. Each such branch is an `if (request.tyre_model == "pneumatic")` block; the sphere path stays.

- [ ] **Step 4: Update the tests that encoded legacy defaults.** Every test that expects `physics_mode == "legacy"`, `drive_mode == "ideal_speed_control"` or `pitch_assist == true` from defaults now expects `physical`/`coast`/`false`. Every test that loads `config/legacy.toml` is deleted, along with the fixture. In `test_schema.cpp:1173` drop the pair from the list. Check every edited assertion against the new rules and keep only the ones that still describe real behaviour.

- [ ] **Step 5: Run the configuration and model tests, then everything ordinary.**

```bash
ctest --test-dir build/native-release --timeout 120 -R 'config|schema|resolution|runtime_config|mjcf|export' --output-on-failure
ctest --test-dir build/native-release --timeout 120 -j8 -LE 'slow|package' --output-on-failure
git status --short tests/fixtures
```

Expected: pass, and no golden is modified. A changed golden here means some physical path read the old default. Stop, find that caller (`project_config.cpp:38` `value_or(default_physics())` is the first suspect), and report it.

- [ ] **Step 6: Level-1 check, then commit (with the user's go-ahead).** Message: `feat(config): physics_mode is physical only`.

---

### Task 10: Suspension without the legacy damper behaviour

`runtime/project_config.cpp:119-130` switches the dampers to the legacy curves (`legacy_behavior = true`) only for non-physical configs. After Task 9 that branch cannot run. The wire field `legacy_behavior` stays, always `false`, so every physical wire config keeps its exact bytes.

**Files:**
- Modify: `native/src/runtime/project_config.cpp:119-130,153,169`
- Modify: `native/src/writers/suspension.cpp` (`sd_legacy_damping_force`, the `legacy_behavior` branches at `:230,262,272,411,421`, and `physical_`, which becomes always true: keep the physical arm of each `physical_ ? a : b` and drop the member)
- Modify: `native/src/config_validation.hpp:92`, `native/src/writer_config.hpp:531-534`, `native/src/configuration/schema.cpp:1108` (accept `physics_mode == "physical"` only; reject `legacy_behavior == true` with `"config.suspension.<damper>.legacy_behavior: the legacy damper curve was removed"`)
- Test: `native/tests/reference/*suspension*`, `native/tests/test_contracts.cpp` (suspension sections), the golden `native_suspension/legacy_env`

- [ ] **Step 1: Find who reads the legacy curve.** Run `git grep -n "legacy_env\|legacy_behavior" -- native/tests`. List each test case that drives the suspension writer with `legacy_behavior = true` or `physics_mode = "legacy"`. Those cases, and only those, are deleted. A case that sets `legacy_behavior = false` and checks the physical curve stays.

- [ ] **Step 2: Write the failing test** (in the suspension writer's config test, next to the existing rejection cases):

```cpp
// The legacy damper curve was removed with legacy mode (2026-11-04): a wire
// config that asks for it is rejected at ingress, never silently modernized.
void legacy_damper_curve_is_rejected() {
    auto wire = physical_suspension_wire();           // the file's existing physical fixture builder
    set_path(wire, {"suspension", "fork_damper", "legacy_behavior"}, bike::io::Value{true});
    bt::invalid([&] { (void)nativecfg::decode(wire); });
    auto mode = physical_suspension_wire();
    set_path(mode, {"suspension", "physics_mode"}, bike::io::Value{std::string("legacy")});
    bt::invalid([&] { (void)nativecfg::decode(mode); });
}
```

Replace `physical_suspension_wire` and `set_path` with the helper names the chosen test file already uses (for example `resolved_wire()` and `set(at(wire, {...}), key, value)` in `test_native_numeric_completion.cpp:764-766`).

- [ ] **Step 3: Run and confirm it fails.**

- [ ] **Step 4: Change the sources.** Edit as listed under **Files**. In `project_config.cpp` delete the `if (physics_config.physics_mode != "physical") { ... }` block and write `Value{false}` for the coil's `legacy_behavior`. Keep `{"physics_mode", text(physics_config.physics_mode)}`: it still writes `"physical"`.

- [ ] **Step 5: Delete the legacy suspension cases** found in Step 1 and the golden directory `tests/fixtures/golden/{darwin-arm64,linux-aarch64}/native_suspension/legacy_env/`, but only if no remaining case reads it. Check this by running the suspension tests; a missing golden fails loudly.

- [ ] **Step 6: Run the tests.** `ctest --test-dir build/native-release --timeout 600 -R 'suspension|contracts|numeric_completion|input_contracts|runtime_config|stepper' --output-on-failure`, then all ordinary tests. Expected: pass; `git status tests/fixtures` shows only the `legacy_env` deletion.

- [ ] **Step 7: Level-1 check, then commit (with the user's go-ahead).** Message: `refactor(suspension): drop the legacy damper curve`.

---

### Task 11: Remove the seated rider variant and its spring-dampers

**Files:**
- Modify: `native/src/configuration/rider_spec.hpp:40-41,62` (variants `none, lumped, articulated_planar`; `kDefaultRiderVariant = "lumped"`; `variant = "lumped"`), `native/src/configuration/validate.cpp:864`
- Modify: `native/src/model/builder.cpp:24-26,34`, `native/src/model/builder_internal.hpp:44`, `native/src/model/legacy_topology.cpp:19-130` (delete `build_seated_rider` and its call at `:131`), `native/src/model/sensors.cpp:32-35`, `native/src/model/mass.cpp:512-518` (`saddle_top_z_for_rider` returns `std::nullopt`; delete it if Task 1 shows its callers handle `nullopt` only), `native/src/model/rider_geometry.hpp:289-296` (`resolve_rider` default `"lumped"`), `native/src/model/rider_geometry.cpp:901-907` (`seated_pose(rider, specs)` stops checking `variant == "seated"`: its only remaining callers are `geometry_pose` and `compute_rider_centers_of_mass`, which pass their own record)
- Modify: `native/src/runtime/capabilities.cpp:29`, `native/src/runtime/project_config.cpp:425-437`, `native/src/runtime/step.cpp:31,1304-1307,2339-2341,2482-2485`, `native/src/stepper.{hpp,cpp}` (`rider_forces_`, `rider_forces()`, `has_rider_forces()`, `rider_forces_qfrc()`, `rider_out_`)
- Modify: `native/src/writer_config.hpp:47,57,324,476-485,864-880,1183-1184`, `native/src/config_types.hpp:344-365`, `native/src/config_validation.hpp:213-222`, `native/src/configuration/schema.cpp:505,593-596`
- Modify: `native/src/initialization/legacy_equilibrium.*` — already deleted in Task 8
- Delete: `native/src/writers/rider_forces.{hpp,cpp}` and their entry in the CMake source list (`git grep -n "rider_forces.cpp" -- native/cmake`), `native/tests/reference/test_native_rider_forces.cpp` and its `Phase3Ports.cmake:245-246` entry, `tests/fixtures/golden/{darwin-arm64,linux-aarch64}/native_rider_forces/`
- Test: `native/tests/model/test_mjcf.cpp:185-186`, `native/tests/model/test_export.cpp:78`, `native/tests/model/test_inertia.cpp` (only the cases that build an MJCF or pick `variant = "seated"` for a model; the pose-solver cases stay), `native/tests/config/test_schema.cpp:1350`, `native/tests/reference/test_native_runtime_config.cpp:288`, `native/tests/reference/test_pinned_topology.cpp:79`, `native/tests/config/test_writer_config.cpp:79,91,183`, `native/tests/reference/test_native_input_contracts.cpp:139,182,219,570`, `native/tests/reference/test_native_numeric_completion.cpp:143,181,534,764-766`, `native/tests/test_contracts.cpp:21,395,442-449,2640,2709,3908-3914`

- [ ] **Step 1: Write the failing tests** (in `native/tests/model/test_mjcf.cpp`):

```cpp
// The seated rider variant was removed (2026-11-04): it no longer resolves,
// and the remaining variants build without slide-joint spring-dampers.
void seated_variant_is_gone() {
    bt::invalid([] { (void)bike::model::rider_from_args("seated"); });
    bike::configuration::RiderSpecs declared;
    bt::require(declared.variant == "lumped", "the declared default rider is lumped");
}

// The seated-only parameters (resonances, damping ratio, saddle interface)
// do not reach any remaining model: changing them leaves the MJCF of the
// lumped and articulated_planar riders byte-identical. This is what lets
// their removal from RiderSpecs wait for a separate migration.
void seated_parameters_are_inert() {
    for (const auto* variant : {"lumped", "articulated_planar"}) {
        auto request = ride_request(variant);       // the file's existing ride-mode BuildRequest helper
        const auto before = bike::model::generate_mjcf(request);
        auto& rider = request.rider.value();
        rider.torso_resonance_hz = 9.; rider.leg_resonance_hz = 8.; rider.arm_resonance_hz = 7.;
        rider.damping_ratio = .9; rider.saddle_interface_k_n_m = 1.; rider.saddle_interface_c_ns_m = 1.;
        bt::require(bike::model::generate_mjcf(request) == before,
                    std::string(variant) + ": seated-only parameters do not change the model");
    }
}
```

`articulated_planar` needs a physical `request.physics` (builder.cpp:30). Build `ride_request` the way `no_joint_damping_in_any_mode()` in the same file builds its `Case`s. In `no_joint_damping_in_any_mode()`, delete the two `seated` cases (`:185-186`).

- [ ] **Step 2: Run and confirm it fails.** `seated_variant_is_gone` fails; `seated_parameters_are_inert` should already pass. If it does not, a seated parameter leaks into a remaining model: stop and report it.

- [ ] **Step 3: Remove the variant and the writer** as listed under **Files**. Order: configuration (variant list, default) → builder/topology/sensors/mass → projection (`project_config.cpp`) → writer config and schema (drop the optional `rider_forces` section so an old wire config with it fails as an unknown key) → stepper and step hooks → delete the writer.

- [ ] **Step 4: Update the tests** listed under **Files**. Delete the cases that exist only for the seated variant or the `rider_forces` writer. Change a case that only uses `"seated"` as a sample variant to `"lumped"` if what it checks still applies. In `test_contracts.cpp`, delete the `RiderForcesWriter` sections.

- [ ] **Step 5: Check the sensor list of the remaining models.** The seated torso and pelvis accelerometers (`sensors.cpp:32-35`) were added only for `variant == "seated"`, so lumped and articulated models must keep exactly their sensors. Run `ctest -R 'mjcf|export|catalog|sensor' --output-on-failure` and confirm no MJCF golden or catalog fixture changed.

- [ ] **Step 6: Prove nothing still references the removed parts.**

```bash
git grep -n -w 'RiderForcesWriter\|RiderForcesConfig\|has_rider_forces\|rider_forces_qfrc\|build_seated_rider' -- native
git grep -n '"seated"' -- native/src native/tests
```

Expected: no output from the first command. The second may only show `validation/acceptance.cpp:237` (a posture name, not the rider variant) and the `geometry_pose` line `neutral.variant = "seated"`. Change that line to a comment explaining that the pose solver takes the record as geometry input, and drop the assignment if `seated_pose` no longer checks the variant (Step 3). Run the PSI check if available.

- [ ] **Step 7: Run all ordinary tests, then the slow set.**

```bash
ctest --test-dir build/native-release --timeout 120 -j8 -LE 'slow|package' --output-on-failure
ctest --test-dir build/native-release --timeout 600 -j4 -L slow --output-on-failure
git status --short tests/fixtures
```

Expected: 100% passed; fixture changes only the `native_rider_forces` deletions.

- [ ] **Step 8: Level-1 check, then commit (with the user's go-ahead).** Message: `feat(rider): remove the seated rider variant and its dampers`.

---

### Task 12: Documentation

**Files:**
- Modify: `docs/RIDE.md` (`:95`, and every place that describes `--physics legacy`, the legacy drive/tyre flags, `--rider seated` or the legacy headless output `telemetry.csv`/`summary.json`), `docs/native-workflow.md`, `docs/development/macos.md`, `docs/development/linux.md`, `docs/TESTING.md` (the rows naming `legacy_ride`, `legacy_metrics`, `legacy_playground`, `native_rider_forces`, `suspension_legacy`, and the legacy-mode sentences; leave the historical F1 records as they are), `docs/RECORDING.md:29` ("Legacy/nonphysical … modes reject …" → only preview), `AGENTS.md` (only if a line there names legacy mode or the seated variant; today none does)
- Modify: `docs/adr/0003-native-authoritative-python-removal.md` — add a short dated note that legacy mode, the stand and the seated variant were removed and their goldens deleted

- [ ] **Step 1: Find every mention.**

```bash
git grep -n -i 'physics legacy\|physics_mode *= *"legacy"\|legacy mode\|bike-playground\|--playground\|bike_playground_stand\.xml\|bike_playground\.xml\|--rider seated\|seated rider\|--drive-mode\|--tyre-pressure\|--tyre-model\|--visual-pedalling\|--legs\b' -- docs README.md examples AGENTS.md ':!docs/superpowers/**'
```

- [ ] **Step 2: Edit each hit** to describe the physical-only behaviour, or delete the sentence when it describes only removed behaviour. Keep historical audit/plan files under `docs/superpowers/` unchanged.

- [ ] **Step 3: Check that the help output matches the docs.** Run `build/native-release/bin/bike-ride --help` and `build/native-release/bin/bike-native-ride --help`, and confirm no removed flag is printed.

- [ ] **Step 4: Commit (with the user's go-ahead).** Message: `docs: describe the physical-only ride`.

---

### Task 13: Final verification and report

- [ ] **Step 1: Run the reference ride once more.**

```bash
build/native-release/bin/bike-ride --physics-config examples/research/viewer_physics_pneumatic.toml \
  --track examples/research/rough_uphill_savage.toml --headless --duration 8 --no-plots > /tmp/rlm/ride-after.json; echo "exit=$?"
diff /tmp/rlm/ride-before.json /tmp/rlm/ride-after.json
```

Expected: the same exit code; only the wall-clock fields differ.

- [ ] **Step 2: Compare the test lists.** `diff /tmp/rlm/tests-before.txt <(ctest --test-dir build/native-release -N | sed -n 's/^ *Test *#[0-9]*: //p' | sort)`. Every removed test is one of `legacy_ride`, `legacy_metrics`, `legacy_playground`, `reference.native_rider_forces`, plus the suspension legacy cases if they were their own CTest entries. Every added test is from Tasks 2–3.

- [ ] **Step 3: Check the goldens.** `git diff --stat <Task 0 commit> -- tests/fixtures`. Expected: deletions only (`bootstrap/legacy_flat`, `requests/legacy_flat.toml`, `config/legacy.toml`, `metrics/filter_*`, `golden/*/native_rider_forces`, `golden/*/native_suspension/legacy_env`). Any modified golden is a failure of the "physical stays bit-identical" constraint and must be explained or fixed before going on.

- [ ] **Step 4: Run the absence audit** from Task 8 Step 4 and Task 11 Step 6 on the final tree, with PSI if reachable.

- [ ] **Step 5: Run the full gate** once, on an idle machine:

```bash
bash tools/run_tests.sh full
```

Expected: exit 0. A failure is fixed and the complete run is repeated; a `--from` run does not count.

- [ ] **Step 6: Report to the user, in plain Russian:**
  - what was removed and what was kept;
  - the evidence (the ride diff, the test-list diff, the fixture diff, the full gate result);
  - what was **not** done: Linux goldens; the inert `RiderSpecs` fields;
  - a question: should a migration plan for the inert rider fields be written now?
