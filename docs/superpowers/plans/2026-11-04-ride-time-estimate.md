# Ride Time Estimate Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
> **This session's user forbids subagents** ("не запускай агентов, делай все сам"): execute inline with superpowers:executing-plans.

**Goal:** `bike-ride` and `bike-settle` tell the operator, while they run, how long the wait will be: whether the equilibrium must be settled and for how long at most, then how long the ride itself will take.

**Architecture:** The physics code stays clock-free. Settling reports only step counts and the residual through optional callbacks (`initialization::SettleProgress`). The CLI layer owns the wall clock and turns counts into estimates in one small, clock-injected module (`cli/run_estimate.*`). Estimates go to stderr for `bike-ride` (stdout carries the JSON summary) and to stdout for `bike-settle` (where its other progress lines already go). Nothing that reaches the simulation or a recording changes.

**Tech Stack:** C++23, CMake/CTest, the native test harness (`native/tests/support/fixture_test.hpp`: `bt::require`, `bt::invalid`, `bt::run`), `bike::platform::monotonic_seconds`, `bike::platform::run` (process runner returning `exit_code`, `out`, `err`).

**Spec:** No separate design document. The decisions below come from the user (2026-11-04) and from the investigation in this session.

## Decisions (user, 2026-11-04)

- D1. The program itself prints how long a scenario will take, so an agent sees the estimate before it starts waiting ("3 напиши отдельный план").
- D2. Marking long tests (the `slow` threshold of 30 s, a warning when a test exceeds its declared time) is a separate plan, not this one.

## Global Constraints

- The simulation stays bit-identical. The callbacks only read step counts and the residual; they never touch `mjData`, the stepper or the cache. Every golden passes without `BIKE_GOLDEN_RECORD`; `git status tests/fixtures` stays clean.
- stdout of `bike-ride` is unchanged: tests and users parse its JSON summary. New `bike-ride` lines go to stderr with the existing `[bike-ride] ` prefix.
- Estimates are honest: an upper bound is labelled "at most"; an estimate from measured speed is labelled "about"; when there is nothing to base it on, the line says so instead of printing a number.
- No subagents. Commit only when the user asks. Linux runs only on explicit request.
- Before native changes, read `docs/agents/native-engineering.md` (AGENTS.md rule 1).
- Order: execute after `2026-11-04-remove-legacy-mode.md` has merged, so `cli/ride.cpp` is edited once. If this plan runs first, the only difference is that `physical_run` still names `legacy::RunLimits`.

## Verification budget

Measured costs (`<git-common-dir>/bike-gate/timings.tsv`, 2026-11-04): ordinary ctest 45 s, `check_change.sh` ~1 min on a warm tree, `run_tests.sh native` ≈ 11 min.

- Inside a task: `mise run test '<pattern>'` with the tests the task names.
- Level 1 (`mise run check <base>`) twice: after Task 2 (stage A: estimator + settle hooks) and after Task 4 (stage B: wiring).
- At the end: one `bash tools/run_tests.sh native` (Task 5). This change touches no force, integration or contact code, only observation callbacks in the settle loop. The settle-related slow tests run as a targeted pattern in Task 2, so the full slow set is not needed.
- Expected total test time: about 15 minutes.

## File map

Created:
- `native/src/initialization/settle_progress.hpp` — `SettleProgress`, the two optional callbacks.
- `native/src/cli/run_estimate.hpp`, `native/src/cli/run_estimate.cpp` — pure estimate math and line text, plus `ProgressReporter` (clock and stream injected).
- `native/tests/cli/test_run_estimate.cpp` — unit tests with a fake clock.

Modified:
- `native/src/initialization/equilibrium.hpp:61-62`, `native/src/initialization/equilibrium.cpp:83-132` (`settle_context` calls the callbacks), `native/src/initialization/setup_owner.hpp:68`, `native/src/initialization/setup_owner.cpp:235` (new `settle` overload).
- `native/src/app/prepare.hpp`, `native/src/app/prepare.cpp` (new `prepare` overload).
- `native/src/cli/ride.cpp` (`physical_run`: settle hooks and ride progress in the headless loop), `native/src/cli/settle.cpp` (settle hooks).
- `native/cmake/RecordingApplication.cmake:166-168` (add `cli/run_estimate.cpp`), `native/tests/cli/CMakeLists.txt` (register the unit test).
- `native/tests/cli/test_capture.cpp:96-124` (end-to-end check of the new lines).
- `docs/RIDE.md` (one paragraph on the progress lines).

---

### Task 0: Baseline

- [ ] **Step 1: Record the reference ride before any change.**

```bash
mkdir -p /tmp/rte && git rev-parse HEAD > /tmp/rte/base-commit.txt
cmake --build build/native-release
for run in 1 2; do
  build/native-release/bin/bike-ride --physics-config examples/research/viewer_physics_pneumatic.toml \
    --track examples/research/rough_uphill_savage.toml --headless --duration 8 --no-plots \
    > /tmp/rte/ride-before-$run.json 2> /tmp/rte/ride-before-$run.err; echo "exit=$?"
done
diff /tmp/rte/ride-before-1.json /tmp/rte/ride-before-2.json
```

The `diff` of the two runs names the wall-clock fields; write them into `/tmp/rte/notes.txt`. Task 4 compares against `ride-before-1.json` and may differ only in those fields.

---

### Task 1: The estimate module

**Files:**
- Create: `native/src/cli/run_estimate.hpp`, `native/src/cli/run_estimate.cpp`
- Create: `native/tests/cli/test_run_estimate.cpp`
- Modify: `native/cmake/RecordingApplication.cmake` (source list next to `cli/ride.cpp`), `native/tests/cli/CMakeLists.txt`

**Interfaces:**
- Produces (namespace `bike::cli::estimate`):
  - `std::string duration_text(double seconds)`
  - `std::optional<double> realtime_factor(double sim_s, double wall_s)`
  - `struct RideState { std::int64_t step; std::int64_t max_steps; double timestep_s; double sim_s; double position_m; double finish_m; bool duration_limited; };`
  - `struct RideRate { double realtime_factor; std::optional<double> speed_mps; };`
  - `struct RideEstimate { std::optional<double> wall_s_to_end; double wall_s_to_cap; };`
  - `RideEstimate ride_estimate(const RideState&, const RideRate&)`
  - `std::string settle_started_line(bool cache_hit, std::int64_t budget_steps)`
  - `std::string settle_progress_line(std::int64_t steps, std::int64_t budget_steps, double steps_per_s, double residual, double tolerance)`
  - `std::string settle_done_line(std::int64_t steps, double wall_s)`
  - `std::string ride_line(const RideState&, const RideRate&, const RideEstimate&)`
- Produces (namespace `bike::cli`): `class ProgressReporter` (see Step 3).

- [ ] **Step 1: Write the failing tests.**

```cpp
// test_run_estimate.cpp — the wait estimates bike-ride and bike-settle
// print: the arithmetic, the honesty labels ("at most" for a bound, "about"
// for a measured-speed estimate), and the reporter's pacing under a fake
// clock. No simulation runs here.
#include "support/fixture_test.hpp"
#include "cli/run_estimate.hpp"

#include <sstream>
#include <string>

namespace {
namespace e = bike::cli::estimate;
bool has(const std::string& text, std::string_view part) { return text.find(part) != std::string::npos; }

// Durations read the way a person reads them.
void durations_read_naturally() {
    bt::require(e::duration_text(0.4) == "0.4 s", "sub-second keeps one decimal");
    bt::require(e::duration_text(45.2) == "45 s", "seconds are whole");
    bt::require(e::duration_text(200.) == "3 min 20 s", "minutes and seconds");
    bt::require(e::duration_text(3900.) == "1 h 05 min", "hours and zero-padded minutes");
}

// A realtime factor needs simulated and wall time both positive.
void realtime_factor_needs_both_clocks() {
    bt::require(e::realtime_factor(2., 1.) == 2., "2 s simulated in 1 s wall is 2x");
    bt::require(!e::realtime_factor(0., 1.) && !e::realtime_factor(1., 0.), "no factor without both clocks");
}

// A --duration ride: remaining simulated time over the realtime factor.
void duration_ride_estimate() {
    const e::RideState s{.step = 4000, .max_steps = 16000, .timestep_s = .0005, .sim_s = 2.,
                         .position_m = 10., .finish_m = 120., .duration_limited = true};
    const auto est = e::ride_estimate(s, {.realtime_factor = 2., .speed_mps = 3.});
    bt::require(est.wall_s_to_end && *est.wall_s_to_end == 12000 * .0005 / 2., "6 s simulated left at 2x is 3 s");
    bt::require(est.wall_s_to_cap == *est.wall_s_to_end, "for a duration ride the cap is the end");
    bt::require(has(e::ride_line(s, {.realtime_factor = 2., .speed_mps = 3.}, est), "about 3 s"),
                "a measured estimate says about");
}

// An end-of-track ride: remaining distance over measured speed, never past the cap.
void track_ride_estimate() {
    const e::RideState s{.step = 4000, .max_steps = 400000, .timestep_s = .0005, .sim_s = 2.,
                         .position_m = 20., .finish_m = 120., .duration_limited = false};
    const auto est = e::ride_estimate(s, {.realtime_factor = 2., .speed_mps = 4.});
    bt::require(est.wall_s_to_end && *est.wall_s_to_end == (100. / 4.) / 2., "100 m at 4 m/s, 2x realtime");
    bt::require(est.wall_s_to_cap == (400000 - 4000) * .0005 / 2., "the cap bound is the step cap");
    const auto stalled = e::ride_estimate(s, {.realtime_factor = 2., .speed_mps = 0.});
    bt::require(!stalled.wall_s_to_end, "no end-of-track estimate while standing still");
    bt::require(has(e::ride_line(s, {.realtime_factor = 2., .speed_mps = 0.}, stalled), "at most"),
                "without speed the line falls back to the cap bound");
}

// Settle lines name the cache decision and label the budget as an upper bound.
void settle_lines() {
    bt::require(has(e::settle_started_line(true, 160000), "stored pose restored"), "a cache or state-file hit says so");
    const auto fresh = e::settle_started_line(false, 160000);
    bt::require(has(fresh, "settling") && has(fresh, "at most 160000 steps"), "a fresh settle states its budget");
    const auto mid = e::settle_progress_line(30000, 160000, 3000., .8, .05);
    bt::require(has(mid, "30000/160000") && has(mid, "at most 43 s left") && has(mid, "residual 0.8 (tolerance 0.05)"),
                "progress shows steps, the bound and the convergence measure");
}

// The reporter prints the start line at once, then at most one line per interval.
void reporter_paces_lines() {
    double now = 0.;
    std::ostringstream out;
    bike::cli::ProgressReporter reporter(out, "[t] ", [&] { return now; }, 10.);
    const auto hooks = reporter.settle_hooks(.05);
    hooks.started(false, 160000);
    now = 1.;  hooks.advanced(3000, 1.);
    now = 11.; hooks.advanced(33000, .5);
    now = 12.; hooks.advanced(36000, .4);
    const auto text = out.str();
    bt::require(has(text, "[t] equilibrium: settling"), "the start line is immediate");
    bt::require(text.find("33000/160000") != std::string::npos && text.find("36000/160000") == std::string::npos,
                "one progress line per 10 s interval");
    reporter.settle_finished();
    bt::require(has(out.str(), "settled in 36000 steps"), "the finish line reports the real cost");
}

// Ride lines: none before 2 s of wall time, and the position is read only
// when a line is printed.
void reporter_reads_position_only_when_due() {
    double now = 0.;
    int reads = 0;
    std::ostringstream out;
    bike::cli::ProgressReporter reporter(out, "[t] ", [&] { return now; }, 10.);
    const auto position = [&] { ++reads; return 2. * now; };
    e::RideState s{.step = 0, .max_steps = 16000, .timestep_s = .0005, .sim_s = 0.,
                   .position_m = 0., .finish_m = 120., .duration_limited = true};
    reporter.ride_advanced(s, position);
    now = 1.; s.step = 4000; s.sim_s = 2.; reporter.ride_advanced(s, position);
    bt::require(out.str().empty() && reads == 1, "no line and no position read before 2 s");
    now = 2.; s.step = 8000; s.sim_s = 4.; reporter.ride_advanced(s, position);
    bt::require(has(out.str(), "2.0x realtime") && has(out.str(), "about 2 s to --duration") && reads == 2,
                "the first estimate after 2 s: 4 s simulated in 2 s wall, 4 s left");
}
} // namespace

int main() {
    return bt::run([] {
        durations_read_naturally(); realtime_factor_needs_both_clocks(); duration_ride_estimate();
        track_ride_estimate(); settle_lines(); reporter_paces_lines(); reporter_reads_position_only_when_due();
    });
}
```

Register it in `native/tests/cli/CMakeLists.txt`, after the `cli_capture` block:

```cmake
# wait estimates of bike-ride/bike-settle (cli/run_estimate.cpp): pure
# arithmetic and line text under a fake clock; runs no simulation.
add_executable(cli_run_estimate test_run_estimate.cpp)
bike_startup_target_policy(cli_run_estimate "${_test_repo_root}/native")
target_link_libraries(cli_run_estimate PRIVATE bike_test_fixtures bike_cli)
if(BUILD_TESTING)
  add_test(NAME cli_run_estimate COMMAND cli_run_estimate)
  set_tests_properties(cli_run_estimate PROPERTIES
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:cli_run_estimate>")
endif()
```

Check that `bike_cli` is the library that holds `cli/ride.cpp` (`git grep -n "add_library(bike_cli" -- native/cmake`). If it is named differently, use that name here.

- [ ] **Step 2: Run it and confirm it fails.** `mise run test '^cli_run_estimate$'`. Expected: compile error, `cli/run_estimate.hpp` not found.

- [ ] **Step 3: Write the module.** Header:

```cpp
#pragma once
// run_estimate — the wait estimates of bike-ride and bike-settle. Physics
// code reports only step counts and residuals (initialization::
// SettleProgress); this module owns the wall clock and the wording. An
// upper bound says "at most", a measured-speed estimate says "about".

#include "../initialization/settle_progress.hpp"

#include <cstdint>
#include <functional>
#include <optional>
#include <ostream>
#include <string>

namespace bike::cli::estimate {
[[nodiscard]] std::string duration_text(double seconds);
[[nodiscard]] std::optional<double> realtime_factor(double sim_s, double wall_s);
struct RideState {
    std::int64_t step = 0, max_steps = 0;
    double timestep_s = 0., sim_s = 0., position_m = 0., finish_m = 0.;
    bool duration_limited = false;
};
struct RideRate { double realtime_factor = 0.; std::optional<double> speed_mps; };
struct RideEstimate { std::optional<double> wall_s_to_end; double wall_s_to_cap = 0.; };
[[nodiscard]] RideEstimate ride_estimate(const RideState&, const RideRate&);
[[nodiscard]] std::string settle_started_line(bool cache_hit, std::int64_t budget_steps);
[[nodiscard]] std::string settle_progress_line(std::int64_t steps, std::int64_t budget_steps,
    double steps_per_s, double residual, double tolerance);
[[nodiscard]] std::string settle_done_line(std::int64_t steps, double wall_s);
[[nodiscard]] std::string ride_line(const RideState&, const RideRate&, const RideEstimate&);
} // namespace bike::cli::estimate

namespace bike::cli {
// Paces estimate lines: the first line of each phase at once, then at
// most one per interval. The clock is injected so tests need no sleeps.
class ProgressReporter {
public:
    using Clock = std::function<double()>;
    ProgressReporter(std::ostream& out, std::string prefix, Clock clock, double interval_s = 10.);
    [[nodiscard]] initialization::SettleProgress settle_hooks(double tolerance);
    // After settle returns: the closing line of a fresh settle (silent on a
    // cache hit). The reporter remembers the decision and the last step count.
    void settle_finished();
    // After each headless advance. `position` is read only when a line is
    // due, so the per-advance cost is one clock read.
    void ride_advanced(estimate::RideState state, const std::function<double()>& position);
private:
    void line(const std::string& text);
    std::ostream* out_;
    std::string prefix_;
    Clock clock_;
    double interval_s_, last_line_ = 0.;
    double settle_started_ = 0.;
    std::int64_t settle_budget_ = 0, settle_steps_ = 0;
    bool settle_cache_hit_ = false;
    std::optional<double> ride_started_wall_;
    estimate::RideState ride_first_{}, ride_last_{};
    double ride_last_wall_ = 0.;
};
} // namespace bike::cli
```

Implementation (`run_estimate.cpp`):

```cpp
#include "run_estimate.hpp"

#include <cmath>
#include <format>
#include <utility>

namespace bike::cli::estimate {
std::string duration_text(double seconds) {
    if (seconds < 1.) return std::format("{:.1f} s", seconds);
    const auto total = static_cast<long long>(std::llround(seconds));
    if (total < 60) return std::format("{} s", total);
    if (total < 3600) return std::format("{} min {} s", total / 60, total % 60);
    return std::format("{} h {:02} min", total / 3600, (total % 3600) / 60);
}
std::optional<double> realtime_factor(double sim_s, double wall_s) {
    if (!(sim_s > 0.) || !(wall_s > 0.)) return std::nullopt;
    return sim_s / wall_s;
}
RideEstimate ride_estimate(const RideState& s, const RideRate& r) {
    RideEstimate out;
    const double left_cap_sim = static_cast<double>(s.max_steps - s.step) * s.timestep_s;
    out.wall_s_to_cap = left_cap_sim / r.realtime_factor;
    if (s.duration_limited) { out.wall_s_to_end = out.wall_s_to_cap; return out; }
    // Below 0.1 m/s the bike is effectively standing; distance/speed is noise.
    if (r.speed_mps && *r.speed_mps > .1 && s.finish_m > s.position_m) {
        const double sim_left = (s.finish_m - s.position_m) / *r.speed_mps;
        out.wall_s_to_end = std::min(sim_left / r.realtime_factor, out.wall_s_to_cap);
    }
    return out;
}
std::string settle_started_line(bool cache_hit, std::int64_t budget) {
    // cache_hit covers both the auto-cache and an explicit equilibrium_state_file.
    if (cache_hit) return "equilibrium: stored pose restored, no settling";
    return std::format("equilibrium: settling, no cached pose (at most {} steps)", budget);
}
std::string settle_progress_line(std::int64_t steps, std::int64_t budget, double rate, double residual, double tolerance) {
    const double left = static_cast<double>(budget - steps) / rate;
    return std::format("equilibrium: {}/{} steps, {:.0f} steps/s, at most {} left; residual {:.3g} (tolerance {:.3g})",
                       steps, budget, rate, duration_text(left), residual, tolerance);
}
std::string settle_done_line(std::int64_t steps, double wall_s) {
    return std::format("equilibrium: settled in {} steps, {}", steps, duration_text(wall_s));
}
std::string ride_line(const RideState& s, const RideRate& r, const RideEstimate& e) {
    auto text = std::format("ride: x {:.1f}/{:.0f} m, {:.1f}x realtime", s.position_m, s.finish_m, r.realtime_factor);
    if (r.speed_mps) text += std::format(", {:.1f} km/h", *r.speed_mps * 3.6);
    if (s.duration_limited) return text + " -> about " + duration_text(*e.wall_s_to_end) + " to --duration";
    if (e.wall_s_to_end) return text + " -> about " + duration_text(*e.wall_s_to_end) + " to the end of the track (at most "
                                + duration_text(e.wall_s_to_cap) + ")";
    return text + " -> no end-of-track estimate while standing still; at most " + duration_text(e.wall_s_to_cap);
}
} // namespace bike::cli::estimate

namespace bike::cli {
ProgressReporter::ProgressReporter(std::ostream& out, std::string prefix, Clock clock, double interval_s)
    : out_(&out), prefix_(std::move(prefix)), clock_(std::move(clock)), interval_s_(interval_s) {}
void ProgressReporter::line(const std::string& text) {
    *out_ << prefix_ << text << '\n' << std::flush;
    last_line_ = clock_();
}
initialization::SettleProgress ProgressReporter::settle_hooks(double tolerance) {
    initialization::SettleProgress hooks;
    hooks.started = [this](bool cache_hit, std::int64_t budget) {
        settle_started_ = clock_(); settle_budget_ = budget; settle_cache_hit_ = cache_hit;
        line(estimate::settle_started_line(cache_hit, budget));
    };
    hooks.advanced = [this, tolerance](std::int64_t steps, double residual) {
        settle_steps_ = steps;
        const double now = clock_();
        if (now - last_line_ < interval_s_ || now <= settle_started_) return;
        line(estimate::settle_progress_line(steps, settle_budget_, static_cast<double>(steps) / (now - settle_started_),
                                            residual, tolerance));
    };
    return hooks;
}
void ProgressReporter::settle_finished() {
    if (settle_cache_hit_ || settle_steps_ == 0) return;
    line(estimate::settle_done_line(settle_steps_, clock_() - settle_started_));
}
void ProgressReporter::ride_advanced(estimate::RideState s, const std::function<double()>& position) {
    const double now = clock_();
    if (!ride_started_wall_) {
        s.position_m = position();
        ride_started_wall_ = now; ride_first_ = ride_last_ = s; ride_last_wall_ = now; return;
    }
    // First estimate after 2 s of wall time, then one per interval.
    const bool first = ride_last_wall_ == *ride_started_wall_;
    if (now - (first ? *ride_started_wall_ : last_line_) < (first ? 2. : interval_s_)) return;
    s.position_m = position();
    const auto rtf = estimate::realtime_factor(s.sim_s - ride_first_.sim_s, now - *ride_started_wall_);
    if (!rtf) return;
    std::optional<double> speed;
    if (s.sim_s > ride_last_.sim_s) speed = (s.position_m - ride_last_.position_m) / (s.sim_s - ride_last_.sim_s);
    const estimate::RideRate rate{.realtime_factor = *rtf, .speed_mps = speed};
    line(estimate::ride_line(s, rate, estimate::ride_estimate(s, rate)));
    ride_last_ = s; ride_last_wall_ = now;
}
} // namespace bike::cli
```

`settle_progress.hpp` (needed by the header; Task 2 wires it into the settle loop):

```cpp
#pragma once
// SettleProgress — optional observers of the equilibrium settle loop. They
// receive only step counts and the residual: the settle loop never reads a
// clock and the callbacks never touch the model, so settling stays
// bit-identical with or without them.

#include <cstdint>
#include <functional>

namespace bike::initialization {
struct SettleProgress {
    // Once, after the cache decision: whether a stored pose was restored
    // and the step budget a fresh settle may spend.
    std::function<void(bool cache_hit, std::int64_t budget_steps)> started;
    // After every relaxation segment: cumulative steps and the residual.
    std::function<void(std::int64_t steps, double residual_qacc)> advanced;
};
} // namespace bike::initialization
```

Add `"${native_dir}/src/cli/run_estimate.cpp"` to the source list in `RecordingApplication.cmake`, next to `cli/ride.cpp`.

- [ ] **Step 4: Run the test.** `mise run test '^cli_run_estimate$'`. Expected: pass. If `settle_progress_line`'s text differs from the test (for example `at most 43 s left` because 130000/3000 = 43.3), fix the code, not the test, unless the test arithmetic is wrong.

- [ ] **Step 5: Commit (with the user's go-ahead).** Message: `feat(cli): add the wait-estimate module`.

---

### Task 2: Settle reports its progress

**Files:**
- Modify: `native/src/initialization/equilibrium.hpp:61-62`, `native/src/initialization/equilibrium.cpp:83-132`, `native/src/initialization/setup_owner.hpp:68`, `native/src/initialization/setup_owner.cpp:~235`
- Test: `native/tests/cli/test_run_estimate.cpp` stays; the settle wiring is checked by the existing equilibrium tests (bit-identity) and end to end in Task 4

**Interfaces:**
- Consumes: `initialization::SettleProgress` (Task 1).
- Produces: `EquilibriumResult settle(SetupOwner&, const EquilibriumOptions&, const SettleProgress&)`; `EquilibriumResult settle_context(SetupContext&, const EquilibriumOptions& = {}, const SettleProgress* = nullptr)`.

- [ ] **Step 1: Change the declarations.** In `equilibrium.hpp`, include `"settle_progress.hpp"` and declare:

```cpp
[[nodiscard]] EquilibriumResult settle(SetupOwner&, const EquilibriumOptions& = {});
// Same settle, observed: the callbacks see step counts and the residual only.
[[nodiscard]] EquilibriumResult settle(SetupOwner&, const EquilibriumOptions&, const SettleProgress&);
[[nodiscard]] EquilibriumResult settle_context(SetupContext&, const EquilibriumOptions& = {},
                                               const SettleProgress* progress = nullptr);
```

In `setup_owner.hpp:68`, add the friend for the new overload next to the existing one:

```cpp
    friend EquilibriumResult settle(SetupOwner&, const EquilibriumOptions&, const SettleProgress&);
```

- [ ] **Step 2: Call the callbacks in `settle_context`.** Right after the cache block (after `cache_hit=true` / the `catch` that resets the pose, before `relax_step` is defined):

```cpp
    if(progress&&progress->started)progress->started(cache_hit,budget);
```

and inside the `while` loop, right after `residual` is computed by `acceleration_residual(context)` and the refine block has run (before `if(residual<=options.tolerance)break;`):

```cpp
        if(progress&&progress->advanced)progress->advanced(steps,residual);
```

Do not move or reorder any existing statement.

- [ ] **Step 3: Add the `settle` overload** in `setup_owner.cpp`, next to the existing `settle`: copy its body and pass `&progress` to `settle_context`. Make the existing `settle(owner, options)` call the new one with `SettleProgress{}`, so there is one body.

- [ ] **Step 4: Prove settling is unchanged.**

```bash
mise run test 'equilibrium|settle|bootstrap|sag|initial_state|welded|golden_episode'
git status --short tests/fixtures
```

Expected: pass, and no fixture is modified.

- [ ] **Step 5: Commit (with the user's go-ahead).** Message: `feat(init): let callers observe the settle loop`.

- [ ] **Stage A check (level 1).** `mise run check <commit before Task 1>`. Expected: exit 0.

---

### Task 3: `bike-settle` prints settle progress

**Files:**
- Modify: `native/src/cli/settle.cpp:30-55`

- [ ] **Step 1: Wire the reporter.** Its existing lines go to stdout, so the reporter writes there too:

```cpp
    ProgressReporter reporter(std::cout, "[bike-settle] ", platform::monotonic_seconds);
    ...
        equilibrium = initialization::settle(*setup, options, reporter.settle_hooks(options.tolerance));
        reporter.settle_finished();
```

Add `#include "run_estimate.hpp"`. Keep every existing `[bike-settle]` line unchanged: the closing line already reports steps and wall time.

- [ ] **Step 2: Run the settle tests.** `mise run test 'cli_capture|settle'`. Expected: pass. `cli_capture` runs `bike-settle` end to end (`test_capture.cpp:113`).

- [ ] **Step 3: Commit (with the user's go-ahead).** Message: `feat(settle): print settle progress and bound`.

---

### Task 4: `bike-ride` prints settle and ride estimates

**Files:**
- Modify: `native/src/app/prepare.hpp`, `native/src/app/prepare.cpp`
- Modify: `native/src/cli/ride.cpp` (`physical_run`)
- Test: `native/tests/cli/test_capture.cpp:96-124`

**Interfaces:**
- Consumes: `ProgressReporter`, `estimate::RideState` (Task 1); `settle(SetupOwner&, const EquilibriumOptions&, const SettleProgress&)` (Task 2).
- Produces: `std::unique_ptr<PreparedSimulation> app::prepare(std::unique_ptr<SetupRequest>, const initialization::SettleProgress&)`.

- [ ] **Step 1: Extend the end-to-end test first.** In `test_capture.cpp`, after the existing `ride` assertions (line ~123):

```cpp
    // The ride states its wait on stderr, and stdout keeps only the summary.
    bt::require(ride.err.find("[bike-ride] equilibrium:") != std::string::npos,
                "bike-ride reports the equilibrium decision (" + ride.err + ")");
    bt::require(ride.out.find("equilibrium:") == std::string::npos,
                "estimate lines never reach stdout, which carries the JSON summary");
    bt::require(settle.out.find("[bike-settle] equilibrium: settling") != std::string::npos,
                "bike-settle reports a fresh settle and its bound (" + settle.out + ")");
```

The ride there runs 0.00125 s, so it prints no ride line (the first one waits 2 s); the ride line is covered by the unit test and Step 5.

- [ ] **Step 2: Run it and confirm the bike-ride assertion fails.** `mise run test '^cli_capture$'`.

- [ ] **Step 3: Add the `prepare` overload.** In `prepare.hpp`:

```cpp
// Same preparation, with the settle loop observed (bike-ride's estimates).
[[nodiscard]] std::unique_ptr<PreparedSimulation> prepare(std::unique_ptr<SetupRequest> request,
                                                          const initialization::SettleProgress& progress);
```

In `prepare.cpp`, give `make_settled_owner` and `prepare_into` a `const initialization::SettleProgress&` parameter and call `initialization::settle(*setup, options, progress)`. The existing overloads pass `initialization::SettleProgress{}`.

- [ ] **Step 4: Wire `physical_run`.**
  - Before `app::prepare`, create the reporter and use the observed overload:

```cpp
    ProgressReporter reporter(std::cerr,"[bike-ride] ",platform::monotonic_seconds);
    auto prepared=app::prepare(std::make_unique<initialization::SetupRequest>(request),
        reporter.settle_hooks(request.equilibrium.tolerance));
    reporter.settle_finished();
```

  - In the headless loop, after each `runtime->advance(...)` (and its `release_samples()`):

```cpp
                const auto now=runtime->snapshot();
                reporter.ride_advanced({.step=now.step,.max_steps=max_steps,.timestep_s=dt,.sim_s=now.time_s,
                    .position_m=0.,.finish_m=limits.finish_x_m,.duration_limited=args.has("duration")},
                    [&]{return runtime->render_frame().presentation.position_m;});
```

    `render_frame()` is `const` (`runtime/runtime.hpp:143`) and the preview path already calls it, so reading the position does not change the simulation. The reporter calls it only when a line is due.
  - Add `#include "run_estimate.hpp"`. The viewer path gets the settle lines (the user waits for settling there too) and no ride lines: it runs at the chosen time scale.

- [ ] **Step 5: Run the tests and the reference ride.**

```bash
mise run test 'cli_capture|cli_run_estimate|ride_backend_cli|recording_cli'
build/native-release/bin/bike-ride --physics-config examples/research/viewer_physics_pneumatic.toml \
  --track examples/research/rough_uphill_savage.toml --headless --duration 8 --no-plots \
  > /tmp/rte/ride.json 2> /tmp/rte/ride.err; echo "exit=$?"; cat /tmp/rte/ride.err
```

Expected: tests pass. stderr shows the equilibrium line, then a `ride:` line ~2 s after the ride starts, then one every 10 s. `diff /tmp/rte/ride-before-1.json /tmp/rte/ride.json` shows only the wall-clock fields from `/tmp/rte/notes.txt`; any other difference means the reporting changed the run and must be fixed before going on.

- [ ] **Step 6: Commit (with the user's go-ahead).** Message: `feat(ride): print settle and ride wait estimates`.

- [ ] **Stage B check (level 1).** `mise run check <commit before Task 3>`. Expected: exit 0.

---

### Task 5: Documentation and final check

**Files:**
- Modify: `docs/RIDE.md` (the section on headless runs)

- [ ] **Step 1: Add one paragraph to `docs/RIDE.md`:**

```markdown
While it runs, `bike-ride` states the wait on stderr: first whether the
equilibrium comes from a cached pose or must be settled (with the step
budget as an upper bound and periodic progress), then, for headless runs,
the ride's realtime factor and the time to `--duration` or to the end of
the track. "at most" marks an upper bound; "about" marks an estimate from
the speed measured so far. stdout still carries only the JSON summary.
`bike-settle` prints the same settle lines on stdout.
```

- [ ] **Step 2: Run the final gate.** `bash tools/run_tests.sh native`. Expected: exit 0, about 11 min.

- [ ] **Step 3: Commit (with the user's go-ahead).** Message: `docs(ride): describe the wait estimates`.

- [ ] **Step 4: Report to the user, in plain Russian:** what each command now prints, an example from the reference ride, the test results, and that `bike-ride --research` is not covered (its loop is separate; it can reuse `ProgressReporter` later).
