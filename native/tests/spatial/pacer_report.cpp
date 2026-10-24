// W50 deterministic-clock controls: no display, physics or wall-clock wait.
// These test sources are authored here; execution belongs to V09/V10.
#include "support/migration_test.hpp"
#include "viewer/pacer.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <optional>

namespace {
namespace view = bike::viewer;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
const Clock::time_point zero{};

void close(double actual, double expected, const char* message) {
    bt::require(std::isfinite(actual) && std::abs(actual - expected) < 1e-12, message);
}

void observed_debt_and_presentations() {
    view::Pacer pacer(.125, .25);
    view::PacingMeasurement measured(zero, 2., .25);
    bt::require(pacer.poll(zero).request_frame, "initial scheduling still requests the first frame");
    measured.presented(zero, zero, 2., 0.);
    bt::require(pacer.poll(zero + 250ms).physics_steps == 2, "quarter second offers two fixed ticks");
    measured.observe(zero + 250ms, 2., pacer.debt_s());
    pacer.completed(1);
    close(pacer.debt_s_at(zero + 375ms), .25, "observational debt includes render time since the last poll");
    close(pacer.debt_s(), .125, "observing debt never changes pacing arithmetic");
    measured.presented(zero + 250ms, zero + 375ms, 2.125, pacer.debt_s_at(zero + 375ms));
    bt::require(pacer.poll(zero + 500ms).physics_steps == 3, "later offer retains only actual uncompleted work");
    measured.observe(zero + 500ms, 2.125, pacer.debt_s());
    pacer.completed(3);
    measured.presented(zero + 500ms, zero + 625ms, 2.5, pacer.debt_s_at(zero + 625ms));
    measured.stop(zero + 625ms, 2.5, pacer.debt_s_at(zero + 625ms), true);
    const auto report = measured.report();
    bt::require(report.stopped && report.terminal && report.one_x_eligible,
                "clean terminal window is eligible for timing assessment, independently of measured speed");
    close(report.wall_s, .625, "wall time ends after terminal presentation");
    close(report.simulated_s, .5, "simulation duration is relative to the initial state");
    close(report.render_wall_s, .25, "only successful presentation render intervals are accumulated");
    close(report.max_debt_s, .375, "maximum observed pacing debt is retained");
    close(report.debt_s, .125, "terminal debt includes final render time");
    bt::require(report.presented_frames == 3 && report.late_frames == 1,
                "actual presentations and late presentation intervals are counted separately");
    close(report.max_frame_lateness_s, .125, "maximum inter-frame lateness");
    close(report.total_frame_lateness_s, .125, "total inter-frame lateness");
    bt::require(report.mean_frame_lateness_s == std::optional<double>(.0625),
                "mean lateness uses presentation intervals, never physics steps");
    bt::require(report.mean_achieved_rtf.has_value() && report.frames_per_s.has_value(),
                "positive measured window supplies actual RTF and FPS");
    close(report.mean_achieved_rtf.value_or(std::numeric_limits<double>::quiet_NaN()), .8,
          "achieved RTF is simulated time divided by full viewer time");
    close(report.frames_per_s.value_or(std::numeric_limits<double>::quiet_NaN()), 4.8,
          "frame rate counts actual swaps per measured wall second");

    measured.observe(zero + 10s, 2.5, 10.);
    measured.presented(zero + 10s, zero + 11s, 2.5, 11.);
    measured.stop(zero + 20s, 2.5, 20., false);
    const auto held = measured.report();
    close(held.wall_s, report.wall_s, "terminal idle/close does not extend the measured window");
    bt::require(held.presented_frames == report.presented_frames && held.one_x_eligible,
                "terminal hold redraws do not add performance frames or invalidate the completed window");
    measured.reset(zero + 21s, 0.);
    bt::require(!measured.report().one_x_eligible && measured.report().resets == 1,
                "post-terminal reset prevents attaching first-episode qualification to another generation");
    close(measured.report().wall_s, .625, "post-terminal reset does not relabel idle time as measured physics");
}

void eligibility_controls() {
    const auto finish = [](view::PacingMeasurement& measured, bool terminal) {
        measured.presented(zero, zero + 1s, 1., 0.);
        measured.stop(zero + 1s, 1., 0., terminal);
        return measured.report();
    };
    view::PacingMeasurement control(zero, 0.);
    const auto valid = finish(control, true);
    bt::require(valid.one_x_eligible && valid.mean_achieved_rtf == std::optional<double>(1.),
                "one second of actual simulation in one measured second yields exactly 1x");
    view::PacingMeasurement early(zero, 0.);
    bt::require(!finish(early, false).one_x_eligible, "operator stop cannot qualify a complete episode");
    view::PacingMeasurement paused(zero, 0.);
    paused.mode(true, 1); paused.mode(false, 1);
    const auto paused_report = finish(paused, true);
    bt::require(paused_report.paused && !paused_report.one_x_eligible,
                "resuming does not erase a pause from acceptance history");
    view::PacingMeasurement scaled(zero, 0.);
    scaled.mode(false, 2); scaled.mode(false, 1);
    const auto scaled_report = finish(scaled, true);
    bt::require(scaled_report.scaled && !scaled_report.one_x_eligible,
                "returning to 1x does not erase a scaled segment");
    view::PacingMeasurement fast_start(zero, 0., .25, 8);
    bt::require(!finish(fast_start, true).one_x_eligible, "initial non-1x speed is not clean 1x evidence");
    view::PacingMeasurement hidden(zero, 0., .25, 1, true);
    bt::require(!finish(hidden, true).one_x_eligible, "hidden window is not real-display evidence");
    view::PacingMeasurement absent(zero, 0.);
    absent.unavailable_framebuffer();
    const auto absent_report = finish(absent, true);
    bt::require(absent_report.presented_frames == 1 && !absent_report.one_x_eligible,
                "zero-size framebuffer attempt is not counted and cannot be hidden by a later swap");
    view::PacingMeasurement no_frame(zero, 0.);
    no_frame.stop(zero + 1s, 1., 0., true);
    bt::require(!no_frame.report().one_x_eligible && no_frame.report().presented_frames == 0,
                "physics without a displayed frame cannot qualify GUI pacing");
    view::PacingMeasurement empty(zero, 0.);
    empty.presented(zero, zero, 0., 0.);
    empty.stop(zero, 0., 0., true);
    bt::require(!empty.report().mean_achieved_rtf && !empty.report().frames_per_s &&
                !empty.report().one_x_eligible, "zero wall interval has no fabricated rate");
    view::PacingMeasurement reset(zero, 0.);
    reset.observe(zero + 250ms, .25, 0.);
    reset.reset(zero + 500ms, 0.);
    reset.presented(zero + 500ms, zero + 1s, .5, 0.);
    reset.stop(zero + 1s, .5, 0., true);
    bt::require(!reset.report().one_x_eligible && reset.report().resets == 1,
                "generation reset is diagnostic even when clocks restart cleanly");
    close(reset.report().simulated_s, .75, "diagnostic multi-generation progress never subtracts reset time");
}

void invalid_clocks() {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    bt::invalid([&] { view::PacingMeasurement bad(zero, nan); });
    bt::invalid([] { view::PacingMeasurement bad(zero, 0., 0.); });
    bt::invalid([] { view::PacingMeasurement bad(zero, 0., .25, 3); });
    view::PacingMeasurement measured(zero, 0.);
    measured.observe(zero + 500ms, .5, .125);
    bt::invalid([&] { measured.observe(zero, .5, 0.); });
    bt::invalid([&] { measured.observe(zero + 1s, .25, 0.); });
    bt::invalid([&] { measured.observe(zero + 1s, .5, -1.); });
    bt::invalid([&] { measured.observe(zero + 1s, nan, 0.); });
    bt::invalid([&] { measured.presented(zero + 2s, zero + 1s, .5, 0.); });
    bt::invalid([&] { measured.reset(zero, 0.); });
    measured.presented(zero + 500ms, zero + 750ms, .5, 0.);
    bt::invalid([&] { measured.presented(zero + 500ms, zero + 1s, .5, 0.); });
    view::Pacer pacer(.125, .25);
    pacer.reset(zero + 1s);
    bt::invalid([&] { (void)pacer.debt_s_at(zero); });
    pacer.set_paused(true);
    close(pacer.debt_s_at(zero + 3s), 0., "paused debt observation does not accrue work");
    pacer.set_paused(false);
    close(pacer.debt_s_at(zero + 4s), 0., "pending pacing rebase is respected by observational debt");
}
} // namespace

int main() {
    return bt::run([] { observed_debt_and_presentations(); eligibility_controls(); invalid_clocks(); });
}
