// test_rider_senses.cpp — launch part 1, task 2: the RiderSenses fold,
// the SurfaceLabels map, their strict codecs, and the senses channel's
// transport through the welded ride (fold, state restore, telemetry).
//
// RiderSenses folds what a human feels from the interval's own data: the
// hand press sign follows the rider-contacts convention (bar pushing the
// hands up is positive), labels come from the label map, and nothing from
// the tyre contacts enters (spec «Ощущения райдера»). Part 1 carries the
// record only — no behavior consumes it, so every ride leg also checks
// the trajectory stays bitwise identical under reset.

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <format>
#include <print>
#include <memory>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <mujoco/mujoco.h>

#include "io/value.hpp"
#include "launch_ride_support.hpp"
#include "model_access.hpp"
#include "rider/senses_codec.hpp"
#include "runtime/samples.hpp"
#include "stepper.hpp"
#include "support/fixture_test.hpp"
#include "terrain/trackfile.hpp"
#include "validation/runtime_access.hpp"

namespace {

namespace sw = runtime::sample_wire;
namespace terrain = bike::terrain;
namespace bio = bike::io;

// ---- label map + codec units ------------------------------------------------

// Section lookup is [start_m, end_m): the end point belongs to the
// default.
void test_surface_labels_use_half_open_sections() {
    const rider::SurfaceLabels map{
        rider::SurfaceLabel::asphalt,
        {{.start_m = 10., .end_m = 20., .label = rider::SurfaceLabel::loose}}};
    bt::require(map.at(9.999) == rider::SurfaceLabel::asphalt,
                "before the section");
    bt::require(map.at(10.) == rider::SurfaceLabel::loose,
                "the start belongs to the section");
    bt::require(map.at(20.) == rider::SurfaceLabel::asphalt,
                "the end belongs to the default");
}

// Every one of the five labels round-trips by name; an unknown name
// fails.
void test_all_five_labels_round_trip_and_unknown_fails() {
    for (const auto l : {rider::SurfaceLabel::asphalt,
                         rider::SurfaceLabel::hardpack,
                         rider::SurfaceLabel::loose, rider::SurfaceLabel::wet,
                         rider::SurfaceLabel::scree})
        bt::require(rider::surface_label(rider::surface_label_name(l)) == l,
                    "label round trip");
    bt::require(!rider::surface_label("ice").has_value(),
                "unknown label is rejected");
}

// The senses codec is exact: a missing key fails and a full record
// round-trips bitwise.
void test_senses_codec_is_exact_and_bitwise() {
    const rider::RiderSenses s{.hand_press_n = 123.5, .speed_mps = .25,
                         .rear_teeth = 51,
                         .surface_rear = rider::SurfaceLabel::loose,
                         .surface_front = rider::SurfaceLabel::wet,
                         .surface_ahead = rider::SurfaceLabel::scree,
                         .grade_ahead = .15};
    const auto back =
        rider::decode_rider_senses(rider::encode_rider_senses(s), "senses");
    bt::require(back.hand_press_n == s.hand_press_n &&
                    back.speed_mps == s.speed_mps &&
                    back.rear_teeth == 51 &&
                    back.surface_rear == s.surface_rear &&
                    back.surface_front == s.surface_front &&
                    back.surface_ahead == s.surface_ahead &&
                    back.grade_ahead == s.grade_ahead,
                "senses round-trip bitwise");
    for (const std::string_view missing_key : {"grade_ahead", "surface_rear"}) {
        auto v = rider::encode_rider_senses(s);
        auto &o = std::get<bio::Value::Object>(v.data);
        std::erase_if(o,
                      [missing_key](const auto &kv) { return kv.first == missing_key; });
        bool threw = false;
        try {
            (void)rider::decode_rider_senses(v, "senses");
        } catch (const std::invalid_argument &) {
            threw = true;
        }
        bt::require(threw, "a missing key is rejected");
    }
}

// The label-map codec validates the SurfaceMap contract: unknown keys,
// unknown labels, and overlapping/unsorted intervals all reject at the
// field path.
void test_surface_labels_codec_is_strict() {
    const rider::SurfaceLabels labels{
        rider::SurfaceLabel::hardpack,
        {{.start_m = 1., .end_m = 3., .label = rider::SurfaceLabel::loose}}};
    const auto doc = rider::encode_surface_labels(labels);
    const auto back = rider::decode_surface_labels(doc, "labels");
    bt::require(back.surface == rider::SurfaceLabel::hardpack &&
                    back.sections.size() == 1 &&
                    back.sections[0].start_m == 1. &&
                    back.sections[0].label == rider::SurfaceLabel::loose,
                "surface labels round-trip");

    auto missing_default = doc;
    {
        auto &o = std::get<bio::Value::Object>(missing_default.data);
        std::erase_if(o,
                      [](const auto &kv) { return kv.first == "surface"; });
    }
    bt::invalid([&] {
        (void)rider::decode_surface_labels(missing_default, "labels");
    });
    auto extra_key = doc;
    std::get<bio::Value::Object>(extra_key.data)
        .emplace_back("unknown", bio::Value{});
    bt::invalid([&] {
        (void)rider::decode_surface_labels(extra_key, "labels");
    });
    auto bad_label = doc;
    {
        auto &o = std::get<bio::Value::Object>(bad_label.data);
        for (auto &[k, v] : o)
            if (k == "surface") v = bio::Value{"ice"};
    }
    bt::invalid([&] {
        (void)rider::decode_surface_labels(bad_label, "labels");
    });
    // Overlapping sections reject through the constructor contract.
    auto overlap = doc;
    {
        auto &o = std::get<bio::Value::Object>(overlap.data);
        for (auto &[k, v] : o)
            if (k == "sections") {
                auto &items = std::get<bio::Value::Array>(v.data);
                items.push_back(items.front());
            }
    }
    bt::invalid([&] {
        (void)rider::decode_surface_labels(overlap, "labels");
    });
}

// ---- ride legs --------------------------------------------------------------

struct SensesRow {
    std::int64_t interval = 0;
    double hand_press_n = 0.;
    double speed_mps = 0.;
    std::int64_t rear_teeth = 0;
    std::string surface_rear, surface_front, surface_ahead;
    double grade_ahead = 0.;
    double root_x = 0.;
    double front_x = 0.;
    std::vector<double> qpos;
};

// One flat probe ride under a fixed command — the senses channel rows
// plus the qpos trajectory for the reset/determinism comparison.
[[nodiscard]] std::vector<SensesRow>
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the ride call sites
ride_rows(launch_ride::PedelecEnv &env, double front_wheel_offset,
          std::size_t windows) {
    std::vector<SensesRow> rows;
    std::int64_t last_id = -1;
    runtime::RideControl control;
    control.human_torque_nm = 10.;
    for (std::size_t w = 0; w < windows; ++w) {
        const auto result = env.runtime->advance(
            (static_cast<std::int64_t>(w) + 1) * env.control_steps,
            control, 0., 0., std::nullopt);
        if (result.outcome.has_value()) break;
        for (const auto &sample : env.runtime->recorded_intervals()) {
            const std::int64_t id = sample->interval_id;
            if (id <= last_id) continue;
            last_id = id;
            const auto *wire = sw::find(sample->channels, "rider_senses");
            bt::require(wire != nullptr,
                        "every interval publishes the rider_senses channel");
            const auto &senses = sw::object(*wire);
            SensesRow row;
            row.interval = id;
            row.hand_press_n =
                sw::number(sw::required(senses, "hand_press_n"));
            row.speed_mps = sw::number(sw::required(senses, "speed_mps"));
            row.rear_teeth = sw::integer(sw::required(senses, "rear_teeth"));
            row.surface_rear =
                sw::string(sw::required(senses, "surface_rear"));
            row.surface_front =
                sw::string(sw::required(senses, "surface_front"));
            row.surface_ahead =
                sw::string(sw::required(senses, "surface_ahead"));
            row.grade_ahead =
                sw::number(sw::required(senses, "grade_ahead"));
            row.qpos = sample->qpos;
            row.root_x = sample->qpos.at(
                static_cast<std::size_t>(env.root_x_qpos));
            row.front_x = row.root_x + front_wheel_offset;
            rows.push_back(std::move(row));
        }
    }
    return rows;
}

// The interval-start senses fold on a sectioned flat track: the senses
// channel mirrors the label map at the front wheel's start position, the
// lookahead reads ahead by road_lookahead_m, the grip latch carries a
// real hand press at rest, and a reset+replay reproduces the same
// trajectory bitwise (nothing in the fold feeds back into the dynamics).
void test_ride_senses_follow_the_label_map() {
    // The loose window sits ahead of the ~2.8 m start pose; the 2 m
    // lookahead (viewer_physics_welded.toml) senses it ~2 m before the
    // wheel does — the interval-start convention (M-P2) is what the
    // flip ordering checks.
    auto track = terrain::track_from_dict(terrain::parse_track_toml(
        "name = \"sections\"\nlength_m = 40.0\n"
        "surface = \"asphalt\"\n"
        "grade_profile = { knots = [[0.0, 0.0], [40.0, 0.0]] }\n"
        "surface_sections = [{ start_m = 6.0, end_m = 9.0,"
        " surface = \"loose\" }]\n"));
    auto env = launch_ride::environment(std::move(track), 4.);
    // The front wheel's offset from the root slide — read once on the
    // settled start pose through the validated inspect seam (P7).
    double front_wheel_offset = 0.;
    bike::validation::RuntimeRigAccess::inspect(
        *env.runtime, [&](const Stepper &stepper,
                          const runtime::PhysicalStep &) {
            const int body = mj_name2id(stepper.model(), mjOBJ_BODY,
                                        "front_wheel");
            bt::require(body >= 0, "the welded plant names its wheels");
            const auto xpos = model_access::readonly_buffer(
                stepper.data()->xpos, 3 * stepper.model()->nbody);
            const auto qpos = stepper.qpos();
            front_wheel_offset =
                xpos[3 * static_cast<std::size_t>(body)] -
                qpos[static_cast<std::size_t>(env.root_x_qpos)];
        });
    // Startup contract (P2): the first interval's record already carries
    // the settled grip load and the start pose's labels — the senses
    // record exists before any ride tick.
    const std::size_t windows = static_cast<std::size_t>(
        env.max_steps / env.control_steps + 1);
    const auto rows = ride_rows(env, front_wheel_offset, windows);
    bt::require(!rows.empty(), "the ride published intervals");
    bt::require(rows.front().hand_press_n > 0.,
                "settled grip load is a positive hand press at rest");
    bt::require(rows.front().rear_teeth > 2,
                "the startup record carries the engaged rear cog");

    bool saw_loose_rear = false, saw_loose_front = false, saw_loose_ahead = false,
         returned_front = false;
    std::int64_t ahead_flip = -1, front_flip = -1, rear_flip = -1;
    for (const auto &row : rows) {
        // The fold's surface_front tracks the label map at the front
        // wheel's interval-start position — the [6,9) loose section
        // gives one contiguous loose window. Sample qpos is the
        // interval-END pose while the fold reads the start pose, so the
        // boundary legs keep a 1 m margin against the half-step offset.
        if (row.front_x < 5.)
            bt::require(row.surface_front == "asphalt" &&
                            row.surface_rear == "asphalt",
                        "before the section the default label holds");
        else if (row.front_x < 9.) {
            if (row.surface_front == "loose") {
                saw_loose_front = true;
                if (front_flip < 0) front_flip = row.interval;
            }
        } else if (row.surface_front == "asphalt")
            returned_front = true;
        if (row.surface_rear == "loose") {
            saw_loose_rear = true;
            if (rear_flip < 0) rear_flip = row.interval;
        }
        if (row.surface_ahead == "loose") {
            saw_loose_ahead = true;
            if (ahead_flip < 0) ahead_flip = row.interval;
        }
        // A flat probe track previews a zero grade.
        bt::require(row.grade_ahead == 0., "flat track previews no grade");
        bt::require(row.rear_teeth > 2, "a real cassette cog is sensed");
    }
    bt::require(saw_loose_rear && saw_loose_front && saw_loose_ahead,
                "the ride crosses the loose section");
    bt::require(returned_front,
                "the front label returns to the default past the section");
    bt::require(ahead_flip >= 0 && front_flip >= 0 && rear_flip >= 0 &&
                    ahead_flip < front_flip && front_flip < rear_flip,
                "the lookahead senses the section before front, then rear");

    // Behavior preservation: reset() replays the stored bootstrap and the
    // identical command sequence reproduces every interval's qpos
    // bitwise — the senses fold never reaches the dynamics (P12's
    // native-vs-native leg).
    env.runtime->reset();
    const auto replay = ride_rows(env, front_wheel_offset, windows);
    bt::require(replay.size() == rows.size(),
                "the replay covers the same intervals");
    for (std::size_t i = 0; i < rows.size(); ++i) {
        bt::require(replay[i].qpos.size() == rows[i].qpos.size(),
                    "replay qpos width matches");
        for (std::size_t j = 0; j < rows[i].qpos.size(); ++j)
            bt::require(
                std::bit_cast<std::uint64_t>(replay[i].qpos[j]) ==
                    std::bit_cast<std::uint64_t>(rows[i].qpos[j]),
                "the replayed trajectory is bitwise identical");
        bt::require(
            replay[i].hand_press_n == rows[i].hand_press_n &&
                replay[i].surface_rear == rows[i].surface_rear &&
                replay[i].surface_front == rows[i].surface_front &&
                replay[i].rear_teeth == rows[i].rear_teeth,
            "the senses record restores through reset");
    }
}

// A climb probe ride: the grade preview follows the track's profile on
// the same interval-start positions — nonzero where the knots rise.
void test_grade_ahead_tracks_the_profile() {
    // A 6% ramp: steep enough for the lookahead preview to read a
    // positive grade, gentle enough that the probe keeps climbing
    // (a stall would roll the wheels back off the profile domain).
    auto env = launch_ride::environment(
        launch_ride::track("[[0.0, 0.0], [10.0, 0.0], [14.0, 0.06],"
                           " [150.0, 0.06]]", 150.),
        6.);
    const auto rows = ride_rows(
        env, 0.,
        static_cast<std::size_t>(env.max_steps / env.control_steps + 1));
    bt::require(!rows.empty(), "the climb ride published intervals");
    bool flat_seen = false, climb_seen = false;
    for (const auto &row : rows) {
        if (row.grade_ahead == 0.) flat_seen = true;
        if (row.grade_ahead > 0.) climb_seen = true;
    }
    bt::require(flat_seen, "the flat run-in previews zero grade");
    bt::require(climb_seen, "the climb previews a positive grade");
}

// One recorded interval of the reaction channel, plus whether the
// interval was measured: an unmeasured row carries the kinematic mass
// channel without com_velocity_mps (step.cpp:1694-1698).
struct ReactionRow {
    std::int64_t interval = 0;
    bool full = false;
    std::string phase;
    bool predicate = false;
    double factor = 1., lean = 0.;
};

[[nodiscard]] std::vector<ReactionRow> reaction_rows(launch_ride::PedelecEnv &env, std::size_t windows) {
    std::vector<ReactionRow> rows;
    std::int64_t last_id = -1;
    const runtime::RideControl control;   // effort and posture come from the rider intent
    for (std::size_t w = 0; w < windows; ++w) {
        const auto result = env.runtime->advance(
            (static_cast<std::int64_t>(w) + 1) * env.control_steps, control, 0., 0., std::nullopt);
        for (const auto &sample : env.runtime->recorded_intervals()) {
            if (sample->interval_id <= last_id) continue;
            last_id = sample->interval_id;
            const auto *wire = sw::find(sample->channels, "rider_reaction");
            bt::require(wire != nullptr, "every interval publishes the rider_reaction channel");
            const auto &reaction = sw::object(*wire);
            const auto &mass = sw::object(sw::required(sample->channels, "mass"));
            rows.push_back({.interval = sample->interval_id,
                            .full = sw::find(mass, "com_velocity_mps") != nullptr,
                            .phase = sw::string(sw::required(reaction, "phase")),
                            .predicate = sw::boolean(sw::required(reaction, "predicate")),
                            .factor = sw::number(sw::required(reaction, "effort_factor")),
                            .lean = sw::number(sw::required(reaction, "lean_add_rad"))});
        }
        if (result.outcome.has_value()) break;
    }
    return rows;
}

// Launch part 3a — transport of the reaction channel on a decimated ride
// (physical control period 4 steps, record decimation 8, full capture):
// every consecutive interval, measured or not, carries the channel; the
// rows obey the reaction's phase order and switch ranges; a reset replays
// them bitwise, so perception and reaction state travel through the
// bootstrap capture and restore. Both actions are on with test-only
// thresholds (hands always count as unloaded, 0.05 rad of perceived
// pitch). Whether this ride reaches acting is reported, not required: the
// exact timing contract belongs to the resolver tests above.
[[gnu::noinline]] void test_reaction_channel_on_a_decimated_ride() {
    const auto edit = [](auto &physics) {
        physics.control_period_s = .005;
        auto &r = physics.front_lift_reaction;
        r.ease_enabled = true;
        r.lean_enabled = true;
        r.hand_drop_ratio = 1.;
        r.pitch_rad = .05;
    };
    auto env = launch_ride::environment(launch_ride::track("[[0.0, 0.0], [60.0, 0.0]]", 60.), 3., edit, 8,
                                        runtime::ResearchCaptureMode::full);
    const std::size_t windows = 300;   // 3 s of the .01 s research control period
    const auto rows = reaction_rows(env, windows);
    std::size_t full = 0, unmeasured = 0;
    for (std::size_t i = 0; i < rows.size(); ++i) {
        ++(rows[i].full ? full : unmeasured);
        bt::require(i == 0 || rows[i].interval == rows[i - 1].interval + 1, "consecutive intervals are recorded");
    }
    bt::require(full > 0 && unmeasured > 0, "the ride records measured and unmeasured intervals");
    const std::set<std::pair<std::string, std::string>> allowed{
        {"idle", "idle"}, {"idle", "deciding"}, {"deciding", "deciding"}, {"deciding", "idle"},
        {"deciding", "acting"}, {"acting", "acting"}, {"acting", "releasing"}, {"releasing", "releasing"},
        {"releasing", "acting"}, {"releasing", "idle"},
        // A row is the phase at the interval's close, 8 intention ticks
        // apart: releasing -> idle -> deciding can fit inside one interval.
        {"releasing", "deciding"}};
    std::optional<std::int64_t> deciding_from, acting_at;
    for (std::size_t i = 0; i < rows.size(); ++i) {
        const auto &row = rows[i];
        if (i > 0)
            bt::require(allowed.contains({rows[i - 1].phase, row.phase}), "the phases follow the reaction's order");
        if (row.phase == "idle" || row.phase == "deciding")
            bt::require(row.factor == 1. && row.lean == 0., "no action before acting");
        bt::require(row.factor >= .3 && row.factor <= 1. && row.lean >= 0. && row.lean <= .1,
                    "the outputs stay inside their switch ranges");
        if (row.phase == "deciding" && (i == 0 || rows[i - 1].phase != "deciding") && !acting_at)
            deciding_from = row.interval;
        if (row.phase == "acting" && !acting_at) acting_at = row.interval;
    }
    // One research interval (.01 s) is 8 intention ticks (seated_climb.period_s
    // = .00125 s), so decision_s = .1 s is 80 ticks = 10 intervals. Rows are
    // sampled at interval close: a short return to idle inside one interval
    // is invisible and only makes the observed gap longer, so the bound is
    // >= 10 (exactly 10 without such a hidden return).
    if (acting_at)
        bt::require(deciding_from && *acting_at - *deciding_from >= 10, "acting starts decision_s after deciding");
    std::println("reaction ride: {} rows, {} unmeasured, acting {}", rows.size(), unmeasured,
                 acting_at ? "reached" : "not reached");
    env.runtime->reset();
    const auto replay = reaction_rows(env, windows);
    bt::require(replay.size() == rows.size(), "the replay covers the same intervals");
    for (std::size_t i = 0; i < rows.size(); ++i)
        bt::require(replay[i].interval == rows[i].interval && replay[i].phase == rows[i].phase &&
                        replay[i].predicate == rows[i].predicate &&
                        std::bit_cast<std::uint64_t>(replay[i].factor) == std::bit_cast<std::uint64_t>(rows[i].factor) &&
                        std::bit_cast<std::uint64_t>(replay[i].lean) == std::bit_cast<std::uint64_t>(rows[i].lean),
                    "the reaction replays bitwise through reset");
}

// One recorded interval of the launch channel plus the root slide's
// position for the rollback printout.
struct LaunchRow {
    std::int64_t interval = 0;
    std::string phase;
    double brake = 0.;
    double effort_nm = 0., weight = 0., lean_target_rad = 0.;
    bool preload_missed = false;
    double root_x = 0.;
};

[[nodiscard]] std::vector<LaunchRow> launch_rows(launch_ride::PedelecEnv &env, std::size_t windows) {
    std::vector<LaunchRow> rows;
    std::int64_t last_id = -1;
    const runtime::RideControl control;   // effort and posture come from the rider intent
    for (std::size_t w = 0; w < windows; ++w) {
        const auto result = env.runtime->advance(
            (static_cast<std::int64_t>(w) + 1) * env.control_steps, control, 0., 0., std::nullopt);
        for (const auto &sample : env.runtime->recorded_intervals()) {
            if (sample->interval_id <= last_id) continue;
            last_id = sample->interval_id;
            const auto *wire = sw::find(sample->channels, "rider_launch");
            bt::require(wire != nullptr, "every interval publishes the rider_launch channel");
            const auto &launch = sw::object(*wire);
            rows.push_back({.interval = sample->interval_id,
                            .phase = sw::string(sw::required(launch, "phase")),
                            .brake = sw::number(sw::required(launch, "brake")),
                            .effort_nm = sw::number(sw::required(launch, "effort_nm")),
                            .weight = sw::number(sw::required(launch, "weight")),
                            .lean_target_rad = sw::number(sw::required(launch, "lean_target_rad")),
                            .preload_missed = sw::boolean(sw::required(launch, "preload_missed")),
                            .root_x = sample->qpos.at(static_cast<std::size_t>(env.root_x_qpos))});
        }
        if (result.outcome.has_value()) break;
    }
    return rows;
}

// Launch part 3b: the rider_launch channel on a standing 15 % start with
// the plan switched on (test-only): the rows start in hold with the
// brakes on, the phases follow the plan's order, the brake only falls,
// and the ride leaves the hold. Rollback, the release time and whether the
// handover is reached are printed for the calibration, not required.
[[gnu::noinline]] void test_launch_channel_on_a_standing_climb() {
    const auto edit = [](auto &physics) { physics.rider_launch.plan_enabled = true; };
    auto env = launch_ride::environment(launch_ride::track("[[0.0, 0.15], [60.0, 0.15]]", 60.), 8., edit, 1,
                                        runtime::ResearchCaptureMode::full);
    const std::size_t windows = 800;   // 8 s of the .01 s research control period
    const auto rows = launch_rows(env, windows);
    bt::require(!rows.empty(), "the ride records intervals");
    bt::require(rows.front().phase == "hold" && rows.front().brake == 1.,
                "the first row holds with the brakes on");
    const std::set<std::pair<std::string, std::string>> allowed{
        {"idle", "idle"}, {"idle", "hold"}, {"hold", "hold"}, {"hold", "release"}, {"hold", "accelerate"},
        {"release", "release"}, {"release", "accelerate"}, {"accelerate", "accelerate"},
        {"accelerate", "handover"}, {"accelerate", "blend"}, {"handover", "handover"}, {"handover", "blend"},
        {"blend", "blend"}, {"blend", "done"}, {"done", "done"}, {"done", "hold"}};
    // A recorded interval is one physical step (interval_id = step_), not a
    // research window of control_steps steps.
    const double interval_s = .00125;
    double brake = rows.front().brake, min_x = rows.front().root_x;
    std::optional<double> release_at;
    bool left_hold = rows.front().phase != "hold", done = false;
    for (std::size_t i = 0; i < rows.size(); ++i) {
        const auto &row = rows[i];
        if (i > 0)
            bt::require(allowed.contains({rows[i - 1].phase, row.phase}), "the phases follow the plan's order");
        bt::require(row.brake <= brake || row.phase == "hold", "the brake only falls outside a new hold");
        brake = row.brake;
        min_x = std::min(min_x, row.root_x);
        if (row.phase == "release" && !release_at)
            release_at = static_cast<double>(row.interval) * interval_s;
        if (row.phase != "hold") left_hold = true;
        if (row.phase == "done") done = true;
    }
    bt::require(left_hold, "the ride leaves the hold inside 8 s");
    std::println("launch ride: {} rows to {:.3f} s, rollback {:.4f} m, release {}, done {}",
                 rows.size(), static_cast<double>(rows.back().interval + 1) * interval_s,
                 min_x - rows.front().root_x,
                 release_at ? std::format("{:.3f} s", *release_at) : std::string("not reached"),
                 done ? "reached" : "not reached");
}

} // namespace

int main() {
    // cppcheck-suppress throwInEntryPoint // test failures propagate as exceptions into bt::run's FAIL catch — the harness entry-point contract, not an unhandled throw
    return bt::run([] {
        test_surface_labels_use_half_open_sections();
        test_all_five_labels_round_trip_and_unknown_fails();
        test_senses_codec_is_exact_and_bitwise();
        test_surface_labels_codec_is_strict();
        test_ride_senses_follow_the_label_map();
        test_grade_ahead_tracks_the_profile();
        test_reaction_channel_on_a_decimated_ride();
        test_launch_channel_on_a_standing_climb();
    });
}
