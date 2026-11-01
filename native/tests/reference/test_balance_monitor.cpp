// Completion 3.23. Source: tests/reference/test_balance_monitor.py.
// Low speed latches a run record after grace and dwell; the active loss
// recovers with the dwell timer. Braking never contributes dwell. The
// summary/metrics record pins the latched `event` (never `current_event`
// or a later position) through the production presentation/summary read
// and the research episode_metrics surface.
#include "support/migration_test.hpp"
#include "support/golden.hpp"
#include "cli/physical_capture.hpp"
#include "configuration/load.hpp"
#include "configuration/model_config.hpp"
#include "configuration/physical_config.hpp"
#include "diag.hpp"
#include "initialization/setup_owner.hpp"
#include "io/json.hpp"
#include "model/rider_geometry.hpp"
#include "research/environment.hpp"
#include "research/evidence.hpp"
#include "runtime/control.hpp"
#include "runtime/research.hpp"
#include "runtime/runtime.hpp"
#include "runtime/samples.hpp"
#include "terrain/trackfile.hpp"
#include <cmath>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace io = bike::io;
using runtime::Wire;
using runtime::WireObject;

namespace {

// The live example document — bt::oracle::root() is <repo>/tests/fixtures/
// golden; the welded TOML sits beside the tests three levels up.
std::filesystem::path repo_file(std::string_view relative) {
    return bt::oracle::root().parent_path().parent_path().parent_path() /
           std::filesystem::path(relative);
}

constexpr double KMH = 1. / 3.6;

runtime::BalanceMonitor run(const std::vector<double> &speeds) {
    runtime::BalanceMonitor monitor(4. / 3.6, .5, 3.);
    for (std::size_t i = 0; i < speeds.size(); ++i) {
        const double t = static_cast<double>(i) * .1;
        monitor.update(t, t * speeds[i], speeds[i], true);
    }
    return monitor;
}

bool same_event(const std::optional<runtime::BalanceLostEvent> &a,
                const std::optional<runtime::BalanceLostEvent> &b) {
    if (!a.has_value() || !b.has_value()) return a.has_value() == b.has_value();
    return a->time_s == b->time_s && a->position_m == b->position_m &&
           a->speed_mps == b->speed_mps;
}

// test_balance_monitor.py::test_slow_start_within_grace_is_fine
void test_slow_start_within_grace_is_fine() {
    const auto monitor = run(std::vector<double>(29, 0.));
    bt::require(!monitor.event.has_value(),
                "29 low-speed samples inside the 3 s grace never latch");
}

// test_balance_monitor.py::test_sustained_low_speed_latches_after_half_second_from_first_low_sample
void test_sustained_low_speed_latches() {
    std::vector<double> speeds(31, 5. * KMH);
    speeds.insert(speeds.end(), 6, 3. * KMH);
    const auto monitor = run(speeds);
    bt::require(monitor.event.has_value(), "the sustained dip latches an event");
    bt::require(std::abs(monitor.event.value().time_s - 3.6) < 1e-9, // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                "the event latches at the .5 s dwell boundary");
    bt::require(std::abs(monitor.event.value().position_m - 3.) < 1e-9, // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                "the event keeps the latching sample's position");
    bt::require(same_event(monitor.event, monitor.current_event),
                "the latched event is the active event");
}

// test_balance_monitor.py::test_recovered_speed_clears_the_active_event_but_keeps_the_record
void test_recovered_speed_clears_active_event() {
    std::vector<double> speeds(31, 5. * KMH);
    speeds.insert(speeds.end(), 6, 3. * KMH);
    runtime::BalanceMonitor monitor = run(speeds);
    const auto event = monitor.event;
    bt::require(!monitor.update(10., 50., 10. * KMH, true).has_value(),
                "recovery emits no new event");
    bt::require(!monitor.current_event.has_value(),
                "recovery clears the active event");
    bt::require(same_event(monitor.event, event),
                "recovery keeps the first record");
    bt::require(monitor.low_speed_s == 0., "recovery resets the dwell timer");
    bt::require(!monitor.update(20., 60., 3. * KMH, true).has_value(),
                "a fresh dip starts a new dwell, not a new record");
    const auto active = monitor.update(20.6, 61., 3. * KMH, true);
    bt::require(active.has_value() && same_event(active, monitor.current_event),
                "the second latch returns the active event");
    bt::require(!same_event(monitor.current_event, event) &&
                    monitor.current_event.value().position_m == 61., // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                "the active event is the later episode");
    bt::require(same_event(monitor.event, event),
                "the first record is never replaced");
    bt::require(!monitor.update(21., 62., 3. * KMH, false).has_value(),
                "a braking sample emits nothing");
    bt::require(!monitor.current_event.has_value(),
                "a non-riding sample clears the active event");
    bt::require(same_event(monitor.event, event),
                "the first record survives the pause");
}

// test_balance_monitor.py::test_brief_dip_recovers_and_resets_dwell
void test_brief_dip_recovers_and_resets_dwell() {
    std::vector<double> speeds(31, 5. * KMH);
    speeds.insert(speeds.end(), 4, 3. * KMH);
    speeds.insert(speeds.end(), 5, 5. * KMH);
    const auto monitor = run(speeds);
    bt::require(!monitor.event.has_value(),
                "a .4 s dip inside the .5 s dwell never latches");
    bt::require(monitor.low_speed_s == 0., "recovery zeroes the dwell timer");
}

// test_balance_monitor.py::test_grace_starts_only_when_riding
void test_grace_starts_only_when_riding() {
    runtime::BalanceMonitor monitor(4. / 3.6, .5, 3.);
    for (int i = 0; i < 50; ++i)
        bt::require(!monitor.update(i * .1, 0., 0., false).has_value(),
                    "parked samples never start grace or latch");
    for (int i = 0; i < 29; ++i)
        bt::require(!monitor.update(5. + i * .1, 0., 0., true).has_value(),
                    "riding samples inside grace never latch");
}

// test_balance_monitor.py::test_braking_pauses_dwell_without_restarting_grace
void test_braking_pauses_dwell_without_restarting_grace() {
    runtime::BalanceMonitor monitor(4. / 3.6, .5, 3.);
    monitor.update(0., 0., 5. * KMH, true);
    monitor.update(3.1, 0., 0., true);
    monitor.update(3.4, 0., 0., true);
    bt::require(std::abs(monitor.low_speed_s - .3) < 1e-9,
                "the dwell timer counts from the first low sample");
    monitor.update(3.5, 0., 0., false);
    bt::require(monitor.low_speed_s == 0., "braking clears the dwell timer");
    monitor.update(8., 0., 0., false);
    bt::require(monitor.grace_left_s == 0., "grace was already consumed");
    bt::require(!monitor.update(8.1, 0., 0., true).has_value(),
                "the restarted dwell needs a fresh .5 s");
    bt::require(!monitor.update(8.5, 0., 0., true).has_value(),
                ".4 s of restarted dwell still holds");
    const auto latched = monitor.update(8.6, 0., 0., true);
    bt::require(latched.has_value() &&
                    same_event(latched, monitor.event),
                "the restarted dwell latches at .5 s");
    bt::require(std::abs(monitor.event.value().time_s - 8.6) < 1e-9, // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                "the event time is the latching sample's time");
}

// test_balance_monitor.py::test_reset_clears_event_and_restarts_first_ride_grace
void test_reset_clears_event_and_restarts_grace() {
    runtime::BalanceMonitor monitor = run(std::vector<double>(37, 0.));
    bt::require(monitor.event.has_value(), "37 low-speed samples latch");
    monitor.reset();
    bt::require(!monitor.event.has_value() && !monitor.current_event.has_value() &&
                    monitor.low_speed_s == 0.,
                "reset clears both events and the dwell timer");
    bt::require(!monitor.update(100., 0., 0., true).has_value(),
                "grace restarts after reset");
    bt::require(monitor.grace_left_s == 3., "the first-ride grace is restored");
}

// test_balance_monitor.py::test_floor_boundary_and_time_validation
void test_floor_boundary_and_time_validation() {
    runtime::BalanceMonitor monitor(1., .5, 0.);
    bt::require(!monitor.update(0., 0., 1., true).has_value(),
                "the floor boundary is inclusive");
    bt::require(!monitor.update(1., 0., 1., true).has_value(),
                "holding exactly at the floor stays upright");
    bt::invalid([&] { monitor.update(.5, 0., 0., true); });
}

// test_balance_monitor.py::test_summary_and_research_metrics_keep_latched_event_position
// metrics leg — a fabricated sealed episode whose committed balance_event
// carries the latched position 12 while the episode advanced to 15; the
// production metrics surface must report the latched record.
void test_summary_and_metrics_keep_latched_event_position_metrics() {
    runtime::OwnedResearchEpisode episode{};
    episode.evaluation = {
        {"time_s", Wire(1.)}, {"position_m", Wire(15.)},
        {"reason", Wire("duration")},
        {"torque_requested_nms", Wire(0.)},
        {"torque_delivered_nms", Wire(0.)},
        {"demand_integral_nms", Wire(nullptr)},
        {"balance_event", Wire(WireObject{
            {"time_s", Wire(.5)}, {"position_m", Wire(12.)},
            {"speed_mps", Wire(0.)}})},
        {"max_shock_stroke_m", Wire(0.)},
        {"max_fork_travel_m", Wire(0.)},
        {"numerically_valid", Wire(true)},
        {"max_energy_residual_ratio", Wire(0.)},
        {"metrics", Wire(WireObject{})}};
    episode.configuration.start_position_m = 10.;
    episode.configuration.metadata = io::Value{io::Value::Object{}};
    episode.final_snapshot.model_status = WireObject{};
    episode.run_metadata = io::Value{io::Value::Object{}};
    const auto metrics = bike::research::episode_metrics(episode);
    const auto *fields = std::get_if<io::Value::Object>(&metrics.data);
    bt::require(fields != nullptr, "episode_metrics returns an object");
    const auto *lost = io::find_member(*fields, "balance_lost");
    const auto *lost_at = io::find_member(*fields, "balance_lost_at_m");
    bt::require(lost != nullptr && io::as_bool(*lost, "balance_lost"),
                "episode_metrics reports the latched balance loss");
    bt::require(lost_at != nullptr &&
                    io::as_double(*lost_at, "balance_lost_at_m") == 12.,
                "episode_metrics keeps the latched event position, not the "
                "episode's final position");
}

// ride-summary leg — a real welded env ridden at standstill past grace and
// dwell: the committed monitor's latched event flows through
// balance_event() → presentation.balance_lost_at_m → cli::physical_summary's
// outcome, the same read path the Python fake pins.
void test_summary_and_metrics_keep_latched_event_position_summary() {
    auto owner = std::make_unique<bike::research::ResearchRequest>();
    auto &request = *owner;
    request.setup.model.physics = bike::configuration::load_physics(
        repo_file("examples/research/viewer_physics_welded.toml"));
    request.setup.model.mode = "ride";
    request.setup.model.crank_joint = true;
    request.setup.model.rider =
        bike::model::rider_from_args("articulated_planar");
    request.setup.track = bike::terrain::track_from_dict(
        bike::terrain::parse_track_toml(
            "name = \"probe\"\nlength_m = 200.0\nsurface = \"hardpack\"\n"
            "grade_profile = { knots = [[0.0, 0.0], [200.0, 0.0]] }\n"));
    request.setup.model.physics->initial_speed_mps = 0.;
    request.experiment.duration_s = 8.;
    request.experiment.record_decimation = 1;
    // Diagnostic mode like the other welded fixtures: the documented
    // energy.constraint_work baseline verdict must not abort the balance
    // latch this leg measures.
    request.experiment.stop_on_model_violation = false;
    request.sensors = bike::research::SensorConfig{};
    // The welded .00125 s grid: .005 is the 4-step sensor period the other
    // welded fixtures use.
    request.sensors.sample_period_s = .005;
    const auto prepared = bike::research::prepare_environment(std::move(owner));
    runtime::NativeRideRuntime runtime(std::move(prepared->bootstrap.ride));
    runtime::RideControl control{};
    // rider_enabled stays true (the monitor's riding leg needs it) but the
    // rider's default pedaling is overridden to zero torque — otherwise the
    // welded rig accelerates past the balance floor before the dwell can
    // latch.
    control.human_torque_nm = 0.;
    control.motor_torque_nm = 0.;
    // Grace 3 s + dwell .5 s at standstill on the welded .00125 s grid:
    // the event latches by step 2800; 3200 leaves margin.
    const auto result = runtime.advance(3200, control, 0., 0., std::nullopt);
    (void)result;
    const auto frame = runtime.render_frame();
    bt::require(frame.presentation.balance_lost_at_m.has_value(),
                "the welded standstill latches a balance event");
    const double latched = (frame.presentation.balance_lost_at_m).value(); // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
    const auto summary =
        bike::cli::physical_summary(runtime, io::Value{io::Value::Object{}},
                                    "duration_reached");
    const auto *summary_fields = std::get_if<io::Value::Object>(&summary.data);
    bt::require(summary_fields != nullptr, "physical_summary returns an object");
    const auto *outcome = io::find_member(*summary_fields, "outcome");
    bt::require(outcome != nullptr, "physical_summary carries an outcome");
    const auto *outcome_fields = std::get_if<io::Value::Object>(&outcome->data);
    bt::require(outcome_fields != nullptr, "the outcome is an object");
    const auto *lost = io::find_member(*outcome_fields, "balance_lost");
    const auto *lost_at = io::find_member(*outcome_fields, "balance_lost_at_m");
    bt::require(lost != nullptr && io::as_bool(*lost, "balance_lost"),
                "physical_summary reports the latched balance loss");
    bt::require(lost_at != nullptr &&
                    io::as_double(*lost_at, "balance_lost_at_m") == latched,
                "physical_summary keeps the latched event position");
}

} // namespace

// The welded-env summary leg plus the synthetic monitor legs inline into
// this one dispatcher; the aggregate frame exceeds the 8 KiB policy even
// though each callee stays within budget, so the suppression wraps main
// only (same pattern as test_period_buffer.cpp).
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
int main() {
    return bt::run([] {
        test_slow_start_within_grace_is_fine();
        test_sustained_low_speed_latches();
        test_recovered_speed_clears_active_event();
        test_brief_dip_recovers_and_resets_dwell();
        test_grace_starts_only_when_riding();
        test_braking_pauses_dwell_without_restarting_grace();
        test_reset_clears_event_and_restarts_grace();
        test_floor_boundary_and_time_validation();
        test_summary_and_metrics_keep_latched_event_position_metrics();
        test_summary_and_metrics_keep_latched_event_position_summary();
    });
}
NATIVE_DIAG_POP
