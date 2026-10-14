# Native commands, visualization and batch workflows Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Complete T47 through T51 with source-backed parity and independently reviewed test cycles.

**Architecture:** Preserve the existing native execution path and replace only this subsystem's Python-owned construction, behavior or orchestration. Downstream consumers use the declared owned interfaces; Python reference code stays isolated until final removal.

**Tech Stack:** C++23, CMake >=3.24, pinned native MuJoCo 3.12.0/DuckDB 1.5.6, existing numeric kernels, native IO/GUI/test tools.

**Spec:** `docs/superpowers/specs/2026-10-14-python-free-cpp.md`. Also read `migration/interface-contracts.md`, `migration/api-index.md`, and the assigned source cards.

## Global Constraints

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

---

The named execution sub-skills refer to the downstream agent's environment. When unavailable, follow the same task-by-task red/green/independent-review protocol; no capability is assumed in this handoff. All statuses start planned. The code examples are contracts/regressions to implement and validate, not patches already applied to the application.

## Package map

| Task | Deliverable | Dependencies |
|---|---|---|
| T47 | Replace all remaining Python CLI entrypoints with installed native commands | T25, T33, T36, T44, T46 |
| T48 | Implement a native render owner with independent simulation state | T25, T11, T03 |
| T49 | Port viewer controls, HUD, cameras and cooperative pacing | T48, T36, T44, T47 |
| T50 | Replace Matplotlib and Python Plotly construction with native artifact generation | T14, T15, T32, T37, T43, T06 |
| T51 | Port batch jobs and research sweeps with deterministic process ownership | T47, T37, T42, T06 |

## Task T47: Replace all remaining Python CLI entrypoints with installed native commands

**Dependencies:** T25, T33, T36, T44, T46

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/cli/arguments.hpp` |
| Create | `native/src/cli/arguments.cpp` |
| Create | `native/src/cli/application.cpp` |
| Create | `native/src/cli/ride.cpp` |
| Create | `native/src/cli/research.cpp` |
| Create | `native/src/cli/settle.cpp` |
| Create | `native/src/cli/export.cpp` |
| Create | `native/src/cli/playground.cpp` |
| Create | `native/src/cli/replay.cpp` |
| Create | `native/src/cli/main.cpp` |
| Create | `native/tests/migration/test_t47.cpp` |

Read-only source authority (do not delete during the port):

- `pyproject.toml:1-51`
- `src/bike_sim/__init__.py:1-75`
- `src/bike_sim/cli/__init__.py:1-13`
- `src/bike_sim/cli/export.py:1-49`
- `src/bike_sim/cli/main.py:1-125`
- `src/bike_sim/cli/playground.py:1-17`
- `src/bike_sim/cli/replay.py:1-71`
- `src/bike_sim/cli/research.py:1-281`
- `src/bike_sim/cli/ride.py:1-924`
- `src/bike_sim/cli/settle.py:1-188`
- `src/bike_sim/config.py:1-31`
- `src/bike_sim/sim/__init__.py:1-29`
- `src/bike_sim/sim/backend.py:1-16`

Declaration-level card: `migration/cards/T47.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T25` / `migration/api-index.md#t25`: Construct standalone native rides from user configuration.
- `T33` / `migration/api-index.md#t33`: Port suspension playground and stand controllers.
- `T36` / `migration/api-index.md#t36`: Port policy sessions, pacing boundaries and lifecycle coordination.
- `T44` / `migration/api-index.md#t44`: Port checked replay and versioned legacy compatibility.
- `T46` / `migration/api-index.md#t46`: Deliver the standalone bike-recording analyst command.

**Produces:**

```cpp
namespace bike::cli {
int dispatch(std::string_view executable, std::span<const std::string_view> arguments);
}
// Installed names: bike-sim, bike-playground, bike-export, bike-ride,
// bike-settle, bike-research, bike-batch, bike-replay, bike-recording.
// bike-batch is wired to T51; do not advertise it as implemented before T51.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t47.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "platform/process.hpp"
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

int main() {
  return bt::run([] {
    const auto result = bike::platform::run(
      {BIKE_BINARY_DIR "/bin/bike-ride", "--help"}, bt::fixture("cli"));
    bt::require(result.exit_code == 0, "native help exit");
    bt::golden_text("cli/bike-ride.help.txt", result.out);
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T47
ctest --test-dir build/native-release -R '^migration_T47$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T47` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T47` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Port each argparse surface from migration/cli-contracts.json: actual positional names, defaults, choices, repeated flags, mutually exclusive groups, parser errors, command dispatch, output placement, stdout/stderr and return code. Do not add new user-visible defaults during the port.
  - [ ] 2. Use explicit argument arrays and typed options. Resolve config/track/rider/state paths with their captured precedence. Preserve --record opt-in, --out behavior, headless/viewer selection, diagnostics and --time-scale restrictions.
  - [ ] 3. Accept only implemented runtime modes. During transition the Python command may remain available under its original environment; the native binary must fail specifically for a not-yet-ported mode, never invoke Python as a fallback.
  - [ ] 4. Replace mjpython re-execution with the native viewer path in T48/T49. Move JSON outputs to T05 lossless codecs; preserve command-specific human output rather than making every command print raw JSON.
  - [ ] 5. Keep current export and settle products and cache/state semantics. Generate a version/provenance identity from native build metadata, not from importlib.metadata.
  - [ ] 6. Install all entrypoint names through CMake. Use small per-command main functions or a tested executable-name dispatcher; include command name in usage/errors and test direct invocation from a different working directory.

### Mandatory additional regression cases

- [ ] Every captured option/default/invalid combination is exercised for each of the nine entrypoints.
- [ ] Output disabled by default; an output path alone does not enable recording.
- [ ] Paths with spaces and Unicode, locale-independent numbers and absent display.
- [ ] No shell-command injection or implicit Python fallback; CLI regression fixtures remain stable.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T47$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Run the shared integration gate serially on one machine.** Before T57, use the existing runner with the explicitly selected absolute transition build path. After T57, use the native runner below. Both profiles include the native preflight; do not run them concurrently or drop check classes.

```bash
NATIVE_TEST_BUILD_PATH="$PWD/native/build" bash tools/run_tests.sh native
NATIVE_TEST_BUILD_PATH="$PWD/native/build" bash tools/run_tests.sh full
# After the T57 cutover, use these instead, not Python:
build/native-release/bin/bike-test-runner --profile native --build build/native-release
build/native-release/bin/bike-test-runner --profile full --build build/native-release
```

Verify provenance identifies the intended binary and fixture/provider identity. The transition build must be explicitly configured with bindings ON after T03 and cannot be confused with the bindings-OFF release tree.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/cli/arguments.hpp native/src/cli/arguments.cpp native/src/cli/application.cpp native/src/cli/ride.cpp native/src/cli/research.cpp native/src/cli/settle.cpp native/src/cli/export.cpp native/src/cli/playground.cpp native/src/cli/replay.cpp native/src/cli/main.cpp native/tests/migration/test_t47.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T47 replace all remaining python cli entrypoints with installed native commands"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T48: Implement a native render owner with independent simulation state

**Dependencies:** T25, T11, T03

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/viewer/render_owner.hpp` |
| Create | `native/src/viewer/render_owner.cpp` |
| Create | `native/src/viewer/glfw_context.hpp` |
| Create | `native/src/viewer/glfw_context.cpp` |
| Create | `native/src/viewer/snapshot.cpp` |
| Create | `native/src/runtime/runtime.hpp` |
| Create | `native/src/runtime/runtime.cpp` |
| Create | `native/tests/migration/test_t48.cpp` |

Read-only source authority (do not delete during the port):

- `native/src/runtime/presentation.hpp:1-42`
- `src/bike_sim/native/runtime.py:1-232`
- `src/bike_sim/sim/ride/physical_view.py:1-191`
- `src/bike_sim/sim/ride/presentation.py:1-80`

Declaration-level card: `migration/cards/T48.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T25` / `migration/api-index.md#t25`: Construct standalone native rides from user configuration.
- `T11` / `migration/api-index.md#t11`: Separate native samples and column pieces from their Python boxers.
- `T03` / `migration/api-index.md#t03`: Build the application without finding Python or wheel directories.

**Produces:**

```cpp
namespace runtime {
struct RenderFrame { RuntimeSnapshot state; PresentationState presentation; };
// Add NativeRideRuntime::render_frame() const returning this owned record.
}
namespace bike::viewer {
class RenderOwner {
 public:
  explicit RenderOwner(const runtime::ModelImage& model);
  void apply(const runtime::RenderFrame& frame);
  void draw(int framebuffer_width, int framebuffer_height);
  ~RenderOwner();
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
// GLFW lifetime/event/window work belongs to the main thread. The initial
// PresentationState is captured by native initialization, not a Python amend().
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t48.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "app/setup.hpp"
#include "viewer/render_owner.hpp"
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

int main() {
  return bt::run([] {
    auto prepared = bike::app::prepare(bt::fixture("requests/welded_flat.toml"));
    bike::viewer::RenderOwner renderer(prepared.bootstrap.model);
    auto run = bike::app::make_ride(std::move(prepared));
    const auto before = run->snapshot();
    renderer.apply(run->render_frame());
    const auto after = run->snapshot();
    bt::require(after.step == before.step && after.integration_state.size() == before.integration_state.size(),
                "render apply is non-mutating");
    // Execute this case in the display integration group; headless core tests
    // separately verify snapshot ownership without constructing a GL context.
    for (std::size_t i=0; i<before.integration_state.size(); ++i)
      bt::require(std::bit_cast<std::uint64_t>(before.integration_state[i]) ==
                  std::bit_cast<std::uint64_t>(after.integration_state[i]),
                  "render preserves state bits");
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T48
ctest --test-dir build/native-release -R '^migration_T48$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T48` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T48` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Allocate render-only mjModel/mjData ownership using the MuJoCo native visualization API and a native GLFW context. No simulation mjData pointer is handed to UI code.
  - [ ] 2. Apply complete owned frame arrays plus the declared mutable model arrays; preserve qpos/quaternion handling and geometry refresh. Render paths must not call mj_step, mj_forward, writers, sensors or policy callbacks. Retain existing native view-contract tests as executable checks.
  - [ ] 3. Expose the current camera modes, overlays, livery/markers and UI model state without reading the live runtime. Apply frames on the main thread with their generation/episode IDs; reject incompatible dimensions and stale-reset data.
  - [ ] 4. Scale drawing to framebuffer size, not logical window size; handle resize/minimize/HiDPI, zero-size framebuffer and context destruction. Close renderer resources before the context and keep errors off the physics thread.
  - [ ] 5. Build the viewer only when BIKE_BUILD_GUI=ON. A headless binary/package never initializes GLFW or needs a display. UI startup failure does not corrupt a recording or attempt Python-based recovery.
  - [ ] 6. Use offscreen/hidden-window tests where available and a required manual main-thread/HiDPI smoke gate on the supported macOS configuration. Mark environmental skips distinctly, never convert them into a passing GUI release gate.
  - [ ] 7. Capture the source step-zero view/model fields during native SetupOwner finalization. The current Python NativeRide stores _initial_frame and amends t=0 snapshots; remove that last rendering dependency rather than filling t=0 presentation with zeros.

### Mandatory additional regression cases

- [ ] Applying frames cannot alter simulation state or invoke engine stepping.
- [ ] Wrong model/dimensions, reset generation changes, repeated close and viewer initialization failure.
- [ ] Display event thread and resource destruction ordering.
- [ ] Headless configure/link/run without GLFW or display libraries.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T48$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Complete the review gate.** Run the relevant dependency/domain tests and checks; save exact exit codes/build identity, first failing case when any, and immutable artifact paths. Full shared preflight runs at the next integration boundary; this does not exempt this unit from focused correctness, ownership and warning checks.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/viewer/render_owner.hpp native/src/viewer/render_owner.cpp native/src/viewer/glfw_context.hpp native/src/viewer/glfw_context.cpp native/src/viewer/snapshot.cpp native/src/runtime/runtime.hpp native/src/runtime/runtime.cpp native/tests/migration/test_t48.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T48 implement a native render owner with independent simulation state"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T49: Port viewer controls, HUD, cameras and cooperative pacing

**Dependencies:** T48, T36, T44, T47

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/viewer/input.hpp` |
| Create | `native/src/viewer/input.cpp` |
| Create | `native/src/viewer/hud.cpp` |
| Create | `native/src/viewer/camera.cpp` |
| Create | `native/src/viewer/pacer.hpp` |
| Create | `native/src/viewer/pacer.cpp` |
| Create | `native/src/viewer/ride.cpp` |
| Create | `native/src/viewer/research.cpp` |
| Create | `native/src/viewer/replay.cpp` |
| Create | `native/tests/migration/test_t49.cpp` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/sim/camera.py:1-99`
- `src/bike_sim/sim/hud.py:1-77`
- `src/bike_sim/sim/input_handler.py:1-152`
- `src/bike_sim/sim/research/replay_viewer.py:1-100`
- `src/bike_sim/sim/research/viewer.py:1-234`
- `src/bike_sim/sim/ride/console.py:1-73`
- `src/bike_sim/sim/ride/hud.py:1-676`
- `src/bike_sim/sim/ride/input.py:1-324`
- `src/bike_sim/sim/ride/livery.py:1-111`
- `src/bike_sim/sim/ride/physical_frontend.py:1-545`
- `src/bike_sim/sim/ride/playback_viewer.py:1-233`
- `src/bike_sim/sim/ride/session.py:1-304`
- `src/bike_sim/sim/ride/viewer.py:1-321`

Declaration-level card: `migration/cards/T49.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T48` / `migration/api-index.md#t48`: Implement a native render owner with independent simulation state.
- `T36` / `migration/api-index.md#t36`: Port policy sessions, pacing boundaries and lifecycle coordination.
- `T44` / `migration/api-index.md#t44`: Port checked replay and versioned legacy compatibility.
- `T47` / `migration/api-index.md#t47`: Replace all remaining Python CLI entrypoints with installed native commands.

**Produces:**

```cpp
namespace bike::viewer {
struct PaceDecision { std::uint64_t physics_steps; bool request_frame; };
class Pacer {
 public:
  explicit Pacer(double dt);
  ~Pacer();
  Pacer(const Pacer&) = delete;
  Pacer& operator=(const Pacer&) = delete;
  void set_scale(unsigned scale); // exactly 1, 2, 4 or 8
  PaceDecision poll(std::chrono::steady_clock::time_point now);
  void completed(std::uint64_t actual_steps);
  void reset(std::chrono::steady_clock::time_point now);
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
// Runtime advances remain bounded by the existing interval/stop/yield contract.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t49.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "viewer/pacer.hpp"
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

int main() {
  return bt::run([] {
    bike::viewer::Pacer p(0.00125);
    bt::invalid([&] { p.set_scale(3); });
    p.set_scale(8);
    const auto t0 = std::chrono::steady_clock::time_point{};
    p.reset(t0);
    auto decision = p.poll(t0 + std::chrono::milliseconds(10));
    bt::require(decision.physics_steps == 64, "8x requests steps, not larger dt");
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T49
ctest --test-dir build/native-release -R '^migration_T49$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T49` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T49` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Preserve fixed physics timestep and compute pacing from actual completed steps. The welded profile is dt=0.00125; speed scale multiplies work demand, not dt or controller clocks. Use a monotonic clock and injectable test clock.
  - [ ] 2. Port the source 8 ms cooperative compute budget and 60 Hz render-request pacing, backlog/debt treatment, display idle behavior and pause/reset barriers. Do not promise sustained 8x or hard real-time scheduling.
  - [ ] 3. Port F6/F7/F8 scale controls, C camera, 1/2 views, T terminal, G markers, Q/Escape close; preserve physical Space brake/comma-period strength/R reset and research Space pause/B brake/R reset. These are separate input maps, not a unified shortcut reinterpretation.
  - [ ] 4. Port camera constraints, HUD columns, provenance/validity labels, terminal formatting, braking state and model status. UI-only changes cannot modify forces or command sequence.
  - [ ] 5. Preserve terminal-frame hold, reset generation semantics, no extra policy interval on viewer close, replay acknowledgment and finalization-on-close. Use T42 signal/finalizer coordination.
  - [ ] 6. Capture visual layout baselines with deterministic fixture frames. Automated tests assert semantic HUD text and input dispatch; manual checks cover readability and window behavior without treating pixel equality as physics evidence.

### Mandatory additional regression cases

- [ ] Fake-clock 1/2/4/8 pacing, pause/resume, time jump, backlog and partial-step completion.
- [ ] Explicit --time-scale 1 is rejected headlessly just like other explicit scales.
- [ ] Close/stop during policy callbacks and terminal-frame hold do not add steps.
- [ ] Reset clears pacing debt and cannot display an earlier generation.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T49$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Run the shared integration gate serially on one machine.** Before T57, use the existing runner with the explicitly selected absolute transition build path. After T57, use the native runner below. Both profiles include the native preflight; do not run them concurrently or drop check classes.

```bash
NATIVE_TEST_BUILD_PATH="$PWD/native/build" bash tools/run_tests.sh native
NATIVE_TEST_BUILD_PATH="$PWD/native/build" bash tools/run_tests.sh full
# After the T57 cutover, use these instead, not Python:
build/native-release/bin/bike-test-runner --profile native --build build/native-release
build/native-release/bin/bike-test-runner --profile full --build build/native-release
```

Verify provenance identifies the intended binary and fixture/provider identity. The transition build must be explicitly configured with bindings ON after T03 and cannot be confused with the bindings-OFF release tree.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/viewer/input.hpp native/src/viewer/input.cpp native/src/viewer/hud.cpp native/src/viewer/camera.cpp native/src/viewer/pacer.hpp native/src/viewer/pacer.cpp native/src/viewer/ride.cpp native/src/viewer/research.cpp native/src/viewer/replay.cpp native/tests/migration/test_t49.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T49 port viewer controls, hud, cameras and cooperative pacing"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T50: Replace Matplotlib and Python Plotly construction with native artifact generation

**Dependencies:** T14, T15, T32, T37, T43, T06

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/viz/plot.hpp` |
| Create | `native/src/viz/plot.cpp` |
| Create | `native/src/viz/svg.cpp` |
| Create | `native/src/viz/png.cpp` |
| Create | `native/src/viz/html.cpp` |
| Create | `native/src/viz/tables.cpp` |
| Create named fixture files from recipe (data) | `native/resources/plotly/` |
| Create | `native/tests/migration/test_t50.cpp` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/viz/__init__.py:1-25`
- `src/bike_sim/viz/comparison_plot.py:1-243`
- `src/bike_sim/viz/dyno_plot.py:1-176`
- `src/bike_sim/viz/geometry_plot.py:1-321`
- `src/bike_sim/viz/leverage_plot.py:1-110`
- `src/bike_sim/viz/plots.py:1-21`
- `src/bike_sim/viz/ride_plots.py:1-616`
- `src/bike_sim/viz/tables.py:1-169`
- `src/bike_sim/viz/theme.py:1-44`

Declaration-level card: `migration/cards/T50.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T14` / `migration/api-index.md#t14`: Port geometry and Horst-link kinematics.
- `T15` / `migration/api-index.md#t15`: Port mass properties, suspension setup and rider geometry.
- `T32` / `migration/api-index.md#t32`: Port remaining legacy rider/control/measurement behavior.
- `T37` / `migration/api-index.md#t37`: Port research metrics, validity and experiment evidence.
- `T43` / `migration/api-index.md#t43`: Port read-only recording validation and analyst operations.
- `T06` / `migration/api-index.md#t06`: Introduce filesystem, subprocess and provenance primitives.

**Produces:**

```cpp
namespace bike::viz {
struct Series { std::string name; std::vector<double> x, y; };
struct Plot { std::string title, x_label, y_label; std::vector<Series> series; };
std::string to_svg(const Plot& plot);
std::string to_html(const Plot& plot);
void write_png(const Plot& plot, const std::filesystem::path& path,
               unsigned width, unsigned height);
}
// Plot is the common primitive; task-owned geometry/dyno/ride adapters retain
// domain panels, labels, units, range conventions and marker meanings.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t50.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "viz/plot.hpp"
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

int main() {
  return bt::run([] {
    bike::viz::Plot plot{"travel", "wheel [mm]", "shock [mm]",
      {{"reference", {0.0, 1.0}, {0.0, 0.5}}}};
    const auto html = bike::viz::to_html(plot);
    bt::require(html.find("reference") != std::string::npos, "series retained");
    bt::require(html.find("https://") == std::string::npos, "offline HTML");
    bt::require(bike::viz::to_svg(plot).find("<svg") != std::string::npos, "SVG output");
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T50
ctest --test-dir build/native-release -R '^migration_T50$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T50` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T50` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Inventory actual outputs of every function in viz/: geometry, linkage/leverage, damper dyno, comparison, ride plots, tables and theme. Freeze numerical series separately from pixels, including HTML invalid/termination markers covered by existing tests.
  - [ ] 2. Implement native plot layout and static vector primitives. Use a pinned native raster backend for PNG (proposed Cairo with its native PNG/font dependencies) rather than Python Matplotlib, Kaleido or a shell-launched interpreter. Lock actual transitive dependencies at T03/T59.
  - [ ] 3. Generate interactive offline HTML using locally bundled, licensed Plotly JavaScript plus C++ JSON data, or native HTML/SVG for source outputs that are not interactive. JavaScript in the output is not first-party Python and need not be rewritten as C++; no CDN is required.
  - [ ] 4. Preserve series ordering, units, axis conventions, legends, resolution defaults, invalid-prefix markers, braking/reset annotations, and command-specific paths. Escaping prevents script injection from labels/metadata; nonfinite series use explicit missing-point encoding.
  - [ ] 5. Reject incompatible x/y lengths, zero-size images, huge dimensions and invalid destinations. Test PNG signature/dimensions and semantic plot data; keep fonts/layout deterministic in package smoke tests.
  - [ ] 6. Do not recreate a general plotting library. Scope the native primitives to the plots the repository produces and retain any source-supported output format verified in T01.

### Mandatory additional regression cases

- [ ] No network or Python needed to produce/open the offline HTML report.
- [ ] Geometry/dyno/ride numerical series agree with reference independently of rendering.
- [ ] Nonfinite/missing values, single/empty series, UTF-8 and hostile labels.
- [ ] Static format, image dimensions, markers and filenames match advertised commands.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T50$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Complete the review gate.** Run the relevant dependency/domain tests and checks; save exact exit codes/build identity, first failing case when any, and immutable artifact paths. Full shared preflight runs at the next integration boundary; this does not exempt this unit from focused correctness, ownership and warning checks.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/viz/plot.hpp native/src/viz/plot.cpp native/src/viz/svg.cpp native/src/viz/png.cpp native/src/viz/html.cpp native/src/viz/tables.cpp native/tests/migration/test_t50.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T50 replace matplotlib and python plotly construction with native artifact generation"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.


---

## Task T51: Port batch jobs and research sweeps with deterministic process ownership

**Dependencies:** T47, T37, T42, T06

**Review unit:** one source card or one independently testable seam at a time. The work package is complete only when all its cards and mandatory cases pass.

### Files and ownership

| Action | Exact path |
|---|---|
| Create | `native/src/batch/plan.hpp` |
| Create | `native/src/batch/plan.cpp` |
| Create | `native/src/batch/executor.cpp` |
| Create | `native/src/batch/summary.cpp` |
| Create | `native/src/cli/batch.cpp` |
| Create | `native/src/tools/research_sweep_main.cpp` |
| Create | `native/tests/migration/test_t51.cpp` |

Read-only source authority (do not delete during the port):

- `src/bike_sim/cli/batch.py:1-300`
- `tools/research_sweep.py:1-77`

Declaration-level card: `migration/cards/T51.md`. Every owned Python file remains assigned in the removal ledger until T63.

### Interfaces

**Consumes:** the following completed producer contracts. Exact declarations are in the named API-index sections; include their owning headers and use the shared definitions from interface-contracts.md.

- `T47` / `migration/api-index.md#t47`: Replace all remaining Python CLI entrypoints with installed native commands.
- `T37` / `migration/api-index.md#t37`: Port research metrics, validity and experiment evidence.
- `T42` / `migration/api-index.md#t42`: Preserve exclusive final publication and retry semantics.
- `T06` / `migration/api-index.md#t06`: Introduce filesystem, subprocess and provenance primitives.

**Produces:**

```cpp
namespace bike::batch {
struct Job { std::string id; std::vector<std::string> argv; };
struct Result { std::string id; int exit_code; bike::io::Value summary; };
std::vector<Result> execute(std::span<const Job> jobs, unsigned workers);
}
// Results are returned in input job order; process completion order is not identity.
```

### Step-by-step execution

- [ ] **Step 1: Confirm source and capture the oracle for this unit.** Read the exact source/card ranges and applicable fixture recipe before touching implementation. Add this unit's baseline inputs/outputs/error outcomes to the immutable fixture manifest. Verify its dependencies are at the tested revisions recorded by the coordinator. A missing fixture is a blocking acquisition result, not a skipped comparison.

- [ ] **Step 2: Add the first regression.** Put this body in `native/tests/migration/test_t51.cpp` with the named public headers and T02 CTest registration. The mandatory cases below are additional required tests, not optional follow-ups.

```cpp
#include "support/migration_test.hpp"
#include "batch/plan.hpp"
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

int main() {
  return bt::run([] {
    const std::vector<bike::batch::Job> jobs = {
      {"a", {BIKE_BINARY_DIR "/bin/bike-test-child", "exit", "0"}},
      {"b", {BIKE_BINARY_DIR "/bin/bike-test-child", "exit", "3"}}};
    const auto results = bike::batch::execute(jobs, 2);
    bt::require(results.size() == 2 && results[0].id == "a", "stable ordering");
    bt::require(results[1].exit_code == 3, "worker failure preserved");
  });
}
```

- [ ] **Step 3: Observe the intended red result.** Build and run the focused regression. Before implementation, the new declaration/target may be missing or the semantic comparison must fail. Record which failure was observed. A missing fixture, missing tool, unrelated compile error or zero selected tests is an acquisition/setup failure, not a successful semantic red test.

```bash
cmake --build build/native-release --target migration_T51
ctest --test-dir build/native-release -R '^migration_T51$' --no-tests=error --output-on-failure
```

For a build/process-only package, register `migration_T51` as a CTest script test rather than a fake empty C++ executable; its script runs the command contract above and asserts exits/artifacts. Also register a custom build target named `migration_T51` depending on the native executables used by that script, so the focused build invocation is real. T02 uses the legacy native build for bootstrap; T03 introduces the root standalone build.

- [ ] **Step 4: Implement the native seam and source behaviors.** The produced declarations above are the boundary; the following source-specific requirements define implementation behavior. Make the smallest change for one card/regression, then repeat the test cycle. Do not replace the behavior with a hardcoded golden value.

  - [ ] 1. Port cli/batch.py and tools/research_sweep.py argument construction, job matrix, output naming, result summaries and partial-failure behavior. Preserve captured source behavior for default worker count and invalid inputs.
  - [ ] 2. Spawn native executables with argv vectors, independent per-job runtime/seed/output ownership and bounded workers. Never embed a Python interpreter or invoke uv/pytest for production batch work.
  - [ ] 3. Derive each job seed from its recorded identity and requested seed sequence, not worker scheduling. The same job produces the same inputs with workers=1 and workers=N.
  - [ ] 4. Handle cancellation by stopping dispatch, forwarding the requested termination to owned children and waiting/reaping them. Preserve already finalized results; do not delete diagnostic output or leave orphan processes.
  - [ ] 5. Use fresh exclusive output destinations and avoid process-global current-directory changes. Propagate return codes and failure descriptions without claiming that process exit zero alone certifies a physically valid experiment.
  - [ ] 6. Create bike-test-child as a native test support executable with explicit exit/sleep/write modes, used by T06 and this task. Its modes take typed bounded values; no arbitrary shell execution mode.

### Mandatory additional regression cases

- [ ] Single/multi-worker equivalence, seed stability and stable result order.
- [ ] Spawn failure, nonzero child, signal/cancel, partial output and output collision.
- [ ] Whitespace/Unicode paths and argument values are not split by a shell.
- [ ] Current tests/test_batch.py and test_research_sweep.py assertions survive.

- [ ] **Step 5: Run the green tests and the source comparison.**

```bash
cmake --build build/native-release
ctest --test-dir build/native-release -R '^migration_T51$' --no-tests=error --output-on-failure
```

Expected: the selected test actually runs, returns success and its declared bitwise/numeric/structural/physical comparisons pass. Preserve mismatches with field/index/source provenance. Run the existing source tests named above in the transition reference/native-binding environment until their native replacements are verified; do not claim a native-only build was tested by importing an older extension.

- [ ] **Step 6: Run the shared integration gate serially on one machine.** Before T57, use the existing runner with the explicitly selected absolute transition build path. After T57, use the native runner below. Both profiles include the native preflight; do not run them concurrently or drop check classes.

```bash
NATIVE_TEST_BUILD_PATH="$PWD/native/build" bash tools/run_tests.sh native
NATIVE_TEST_BUILD_PATH="$PWD/native/build" bash tools/run_tests.sh full
# After the T57 cutover, use these instead, not Python:
build/native-release/bin/bike-test-runner --profile native --build build/native-release
build/native-release/bin/bike-test-runner --profile full --build build/native-release
```

Verify provenance identifies the intended binary and fixture/provider identity. The transition build must be explicitly configured with bindings ON after T03 and cannot be confused with the bindings-OFF release tree.

- [ ] **Step 7: Update evidence, review and commit.** Update every affected source/card/test/capability row with actual replacement symbols, comparisons, commands and the resulting commit. Obtain independent specification and code review before marking this package complete. Keep the Python oracle until T63.

```bash
git diff --check
git add -- native/src/batch/plan.hpp native/src/batch/plan.cpp native/src/batch/executor.cpp native/src/batch/summary.cpp native/src/cli/batch.cpp native/src/tools/research_sweep_main.cpp native/tests/migration/test_t51.cpp migration/python-removal.json migration/test-port-map.json
git commit -m "feat(cpp): T51 port batch jobs and research sweeps with deterministic process ownership"
```

**Completion condition:** all required cases and owned cards pass, shared signatures match their consumers, evidence is recorded, and no new interpreter dependency or unapproved compatibility change was introduced.
