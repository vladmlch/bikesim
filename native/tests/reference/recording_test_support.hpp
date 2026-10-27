// Batch 3.30 shared recording-test builders — the C++ mirror of the
// _recording_support.py synthetic sample/evidence/journal builders used by
// tests/reference/test_recording_capture.py,
// tests/reference/test_recording_publication.py,
// tests/reference/test_native_recording_retention.py and
// tests/reference/test_native_recording_lifecycle.py.
//
// Synthetic rows are canonical runtime::PhysicalSampleData owners carrying
// the same member paths the authored Python SimpleNamespace rows did;
// OwnedNativeEpisode::seal and make_physical_source are the production
// boxing/encoding seams the oracle called through nanobind.
#pragma once

#include "support/migration_test.hpp"
#include "support/golden.hpp"
#include "recording/capture.hpp"
#include "recording/catalog.hpp"
#include "recording/contracts.hpp"
#include "recording/value_util.hpp"
#include "runtime/encode.hpp"
#include "runtime/encoding_values.hpp"
#include "runtime/runtime.hpp"
#include "runtime/samples.hpp"
#include "support/temporary_directory.hpp"
#include "io/json.hpp"
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace recording_test {

namespace rec = bike::recording;
namespace sw = runtime::sample_wire;
namespace io = bike::io;

using bike::io::Value;
using runtime::SamplePtr;
using runtime::Wire;
using runtime::WireArray;
using runtime::WireObject;
using rec::value::integer;
using rec::value::real;
using rec::value::text;

// The small planar joint set of support.synthetic_joints() (see
// test_native_encode_columns.cpp, which shares these fixtures).
inline const std::vector<rec::JointCoords> &synthetic_joints() {
    static const std::vector<rec::JointCoords> joints = {
        {"root_x", "slide", 0, 0}, {"root_z", "slide", 1, 1},
        {"root_pitch", "hinge", 2, 2}, {"front_wheel_spin", "hinge", 3, 3},
        {"rear_wheel_spin", "hinge", 4, 4}, {"crank_spin", "hinge", 5, 5},
        {"fork_slide", "slide", 6, 6}, {"shock_slide", "slide", 7, 7}};
    return joints;
}

inline const rec::Catalog &synthetic_catalog() {
    static const rec::Catalog catalog = rec::build_catalog(synthetic_joints());
    return catalog;
}

// One canonical retained interval: qpos/qvel span the 8-coordinate
// synthetic joint layout; channels carry only what callers stage.
inline SamplePtr sample_row(std::int64_t interval_id, double start_s,
                            double dt_s = .00125, WireObject channels = {}) {
    auto sample = std::make_shared<runtime::PhysicalSampleData>();
    sample->interval_id = interval_id;
    sample->time_s = start_s;
    sample->end_time_s = start_s + dt_s;
    sample->qpos = std::vector<double>(8, 0.);
    sample->qvel = std::vector<double>(8, 0.);
    sample->channels = std::move(channels);
    return SamplePtr(std::move(sample));
}

inline runtime::OwnedNativeEpisode make_episode(std::vector<SamplePtr> rows,
                                                int generation = 1) {
    return runtime::OwnedNativeEpisode::seal(
        generation, std::span<const SamplePtr>(rows), runtime::ColumnLayout{});
}

// Storage-valid channel subtrees. The schema tables claim channel roots
// wholesale — a row carrying channels.drive or channels.tires must carry
// every non-optional field, so synthetic episodes that reach publication
// use these complete objects rather than the derived-diff minimum.
inline WireObject tire_channels(bool contact) {
    // Only the plan-watched raw_contact varies with `contact`; every other
    // field stays constant so the derived-transition change set is exactly
    // {front.raw_contact, rear.raw_contact}.
    const auto vec3 = [](double x, double y, double z) {
        return Wire{WireArray{Wire{x}, Wire{y}, Wire{z}}};
    };
    return WireObject{
        {"backend", Wire{std::string("planar")}},
        {"unloaded_radius_m", Wire{.34}},
        {"supports_multiple_contacts", Wire{false}},
        {"outside_material_load_range", Wire{false}},
        {"patches", Wire{WireArray{}}},
        {"normal_load_n", Wire{350.}},
        {"tangent_force_n", Wire{0.}},
        {"slip_mps", Wire{0.}},
        {"world_force_n", vec3(0., 0., 350.)},
        {"geometric_contact", Wire{true}},
        {"raw_contact", Wire{contact}},
        {"controller_grounded", Wire{true}},
        {"multi_support", Wire{false}},
        {"vertical_force_n", Wire{350.}},
        {"normal_vertical_n", Wire{350.}},
        {"effective_radius_m", Wire{.33}},
        {"wheel_axis_m", vec3(0., 1., 0.)},
        {"wheel_axis_moment_nm", Wire{0.}},
        {"omega_abs_rad_s", Wire{0.}},
        {"omega_rel_rad_s", Wire{0.}},
        {"slip_ratio", Wire{0.}},
        {"tangent_center_speed_mps", Wire{0.}}};
}

inline WireObject drive_channels(double brake) {
    return WireObject{
        {"transmission_model", Wire{std::string("ideal_hub")}},
        {"omits_suspension_coupling", Wire{true}},
        {"chain_extension_m", Wire{0.}}, {"chain_extension_rate_mps", Wire{0.}},
        {"chain_tension_n", Wire{0.}}, {"chain_energy_j", Wire{0.}},
        {"chain_dissipation_power_w", Wire{0.}}, {"chain_power_w", Wire{0.}},
        {"freehub_torque_nm", Wire{0.}}, {"freehub_energy_j", Wire{0.}},
        {"freehub_engaged", Wire{true}}, {"freehub_deflection_rad", Wire{0.}},
        {"freehub_dissipation_power_w", Wire{0.}}, {"freehub_power_w", Wire{0.}},
        {"cadence_rpm", Wire{0.}}, {"crank_rad_s", Wire{0.}},
        {"drive_shaft_rad_s", Wire{0.}}, {"crank_phase_rad", Wire{0.}},
        {"human_torque_nm", Wire{0.}}, {"human_sensor_nm", Wire{0.}},
        {"human_command_nm", Wire{0.}}, {"human_active_power_w", Wire{0.}},
        {"human_joint_power_w", Wire{0.}},
        {"assist_demand_gated", Wire{false}}, {"assist_sensor_nm", Wire{0.}},
        {"assist_mode", Wire{std::string("off")}},
        {"rider_mode", Wire{std::string("passive")}},
        {"required_cadence_rpm", Wire{0.}},
        {"motor_request_nm", Wire{0.}}, {"motor_limited_request_nm", Wire{0.}},
        {"motor_torque_nm", Wire{0.}},
        {"motor_control_source", Wire{std::string("none")}},
        {"safety_limited", Wire{false}}, {"energy_limited", Wire{false}},
        {"motor_enabled", Wire{false}}, {"battery_empty", Wire{false}},
        {"motor_freewheel_engaged", Wire{false}},
        {"motor_freewheel_torque_nm", Wire{0.}},
        {"motor_freewheel_dissipation_power_w", Wire{0.}},
        {"gear_front_teeth", Wire{std::int64_t{34}}},
        {"gear_rear_teeth", Wire{std::int64_t{51}}},
        {"gear_ratio", Wire{51. / 34.}},
        {"shift_active", Wire{false}},
        {"shift_direction", Wire{std::string("none")}},
        {"shift_from_teeth", Wire{std::int64_t{0}}},
        {"shift_count", Wire{std::int64_t{0}}},
        {"shift_torque_factor", Wire{1.}},
        {"front_brake_demand", Wire{brake}},
        {"rear_brake_demand", Wire{brake}},
        {"rollback_brake_demand", Wire{0.}},
        {"front_brake_power_w", Wire{0.}}, {"rear_brake_power_w", Wire{0.}},
        {"front_brake_torque_nm", Wire{0.}}, {"rear_brake_torque_nm", Wire{0.}}};
}

// Channels valid for publication whose only varying facts are tire contact
// and brake demand — the derived-transition diffs stay exactly on those.
inline WireObject storable_channels(bool contact, double brake) {
    return WireObject{
        {"tires", Wire{WireObject{
            {"front", Wire{tire_channels(contact)}},
            {"rear", Wire{tire_channels(contact)}}}}},
        {"drive", Wire{drive_channels(brake)}}};
}

// _recording_support.run_metadata(): deterministic synthetic metadata.
inline rec::RunMetadata run_metadata(std::string backend = "native",
                                   std::string mode = "viewer") {
    rec::RunMetadata metadata;
    metadata.run_uuid = "00000000-0000-4000-8000-000000000001";
    metadata.started_utc = "2026-01-01T00:00:00+00:00";
    metadata.backend = std::move(backend);
    metadata.mode = std::move(mode);
    metadata.values = Value{Value::Object{
        {"check_policy", text(mode == "off" ? "diagnostic" : "strict")},
        {"drive_mode", text("articulated_effort")},
        {"configuration_sha256", text(std::string(64, '0'))}}};
    metadata.wall_origin_s = 1.5;
    return metadata;
}

inline rec::InputEvent input_event(std::int64_t episode_id,
                                   std::int64_t event_id,
                                   std::string kind = "episode_start",
                                   double wall_s = 0.,
                                   std::int64_t step = 0,
                                   Value values = Value{Value::Object{}}) {
    rec::InputEvent event;
    event.episode_id = episode_id;
    event.event_id = event_id;
    event.kind = std::move(kind);
    event.received_wall_s = wall_s;
    event.processed_step = step;
    event.values = std::move(values);
    return event;
}

// The TreePhysicalSource stand-in used by capture/publication tests: the
// production source over synthetic retained rows, with the sealed episode
// kept reachable for iter_derived_events-equivalent re-reads.
inline std::shared_ptr<const rec::PhysicalSource>
make_source(std::shared_ptr<const runtime::OwnedNativeEpisode> episode) {
    return rec::make_physical_source(std::move(episode), synthetic_catalog());
}

// storage-contract evidence: summary always carries the reviewed outcome /
// model_status envelope; assets stay empty unless a test stages them.
inline std::shared_ptr<const rec::EpisodeEvidence>
make_evidence(std::int64_t episode_id, int generation = 1,
              std::vector<SamplePtr> rows = {},
              Value summary = Value{Value::Object{}}) {
    auto evidence = std::make_shared<rec::EpisodeEvidence>();
    evidence->episode_id = episode_id;
    evidence->generation = generation;
    evidence->physical = make_source(
        std::make_shared<runtime::OwnedNativeEpisode>(
            make_episode(std::move(rows), generation)));
    if (io::as_object(summary, "summary").empty()) {
        summary = Value{Value::Object{
            {"outcome", Value{Value::Object{{"reason", text("synthetic")}}}},
            {"model_status", Value{Value::Object{{"model_valid", Value{true}},
                                                 {"numerically_valid", Value{true}}}}}}};
    }
    evidence->summary = std::move(summary);
    return evidence;
}

// ---- captured-bootstrap runtime staging ------------------------------------

// Stage a copy of tests/fixtures/migration/bootstrap/welded with selected
// config.json fields re-written (retention mode, measurement cadence,
// strictness) — the cheap NativeRideRuntime path shared by the retention
// and lifecycle runtime-level ports.
struct WeldedRuntime {
    bt::TemporaryDirectory stage{"recording-welded"};
    std::unique_ptr<runtime::NativeRideRuntime> runtime;
    std::int64_t period = 0;
};

inline void replace_config_key(std::string &text, std::string_view key,
                               std::string_view from, std::string_view to) {
    const std::string needle = "\"" + std::string(key) + "\": " + std::string(from);
    const auto at = text.find(needle);
    bt::require(at != std::string::npos, "config key " + std::string(key) + " present");
    text.replace(at, needle.size(),
                 "\"" + std::string(key) + "\": " + std::string(to));
}

inline std::unique_ptr<WeldedRuntime> welded_runtime(
        std::optional<std::string> retention = std::nullopt,
        std::optional<int> decimation = std::nullopt,
        bool strict = false) {
    auto rig = std::make_unique<WeldedRuntime>();
    // state.json declares payloads capture-root-relative
    // ("bootstrap/welded/state.arrays/…"); resolve_declared climbs ancestors,
    // so the staged copy must keep the <root>/bootstrap/welded layout.
    const auto source = bt::fixture("bootstrap/welded");
    const auto staged = rig->stage.path() / "bootstrap" / "welded";
    for (const auto &entry : std::filesystem::recursive_directory_iterator(source)) {
        if (!entry.is_regular_file()) continue;
        const auto relative = std::filesystem::relative(entry.path(), source);
        const auto target = staged / relative;
        std::filesystem::create_directories(target.parent_path());
        std::filesystem::copy_file(entry.path(), target);
    }
    const auto config_path = staged / "config.json";
    auto config = bt::text(config_path);
    if (retention.has_value())
        replace_config_key(config, "record_retention", "\"legacy\"",
                           "\"" + *retention + "\"");
    if (decimation.has_value())
        replace_config_key(config, "measurement_decimation", "1",
                           std::to_string(*decimation));
    if (strict) replace_config_key(config, "strict", "false", "true");
    {
        std::ofstream out(config_path);
        out << config;
    }
    // state.json pins runtime.measurement_decimation / runtime.monitor.strict;
    // read_bootstrap rejects a staged state that disagrees with config.json.
    if (decimation.has_value() || strict) {
        const auto state_path = staged / "state.json";
        auto state = bt::text(state_path);
        if (decimation.has_value())
            replace_config_key(state, "measurement_decimation", "1",
                               std::to_string(*decimation));
        if (strict) replace_config_key(state, "strict", "false", "true");
        std::ofstream out(state_path);
        out << state;
    }
    const auto bootstrap = std::make_unique<runtime::RuntimeBootstrap>(
        runtime::read_bootstrap(staged));
    rig->period = bootstrap->config.control_period_steps;
    rig->runtime = std::make_unique<runtime::NativeRideRuntime>(std::move(*bootstrap));
    return rig;
}

inline void advance(runtime::NativeRideRuntime &runtime, std::int64_t steps) {
    const auto result = runtime.advance(steps, runtime::RideControl{}, 0., 0., std::nullopt);
    bt::require(result.step == steps, "advance committed to target");
}

// The repository root — bt::oracle::root() is <repo>/tests/fixtures/golden.
inline std::filesystem::path repo_file(std::string_view rel) {
    return bt::oracle::root().parent_path().parent_path().parent_path() /
           std::string(rel);
}

// _native_runtime_support.violating_physics: the welded TOML with
// weld_foot_min_normal_n lifted to +100 so the weld-envelope monitor trips
// a real model violation at the first retained row. The TOML's rider
// profile paths are relative to its own directory, so the staged copy
// needs the JSON siblings alongside.
inline std::filesystem::path violating_physics(const std::filesystem::path &target) {
    auto text = bt::text(repo_file("examples/research/viewer_physics_welded.toml"));
    const std::string needle = "weld_foot_min_normal_n = -1050.0";
    const auto at = text.find(needle);
    bt::require(at != std::string::npos,
                "welded TOML floor moved; re-measure the trigger");
    text.replace(at, needle.size(), "weld_foot_min_normal_n = 100.0");
    std::ofstream out(target);
    out << text;
    for (const char *sibling : {"rider_joint_envelope_anatomical.json",
                                "rider_strength_reference.json"})
        std::filesystem::copy_file(repo_file("examples/research") / sibling,
                                   target.parent_path() / sibling,
                                   std::filesystem::copy_options::overwrite_existing);
    return target;
}

// capture_rows(rows) — test_recording_storage.py's enabled one-episode
// capture with the synthetic summary the publication tests assert against.
// RunCapture is non-movable, so the owner is heap-staged like the other
// several-KB recording owners in this suite.
inline std::unique_ptr<rec::RunCapture>
capture_rows(std::vector<SamplePtr> rows, std::string mode = "viewer") {
    auto capture = std::make_unique<rec::RunCapture>(
        run_metadata("native", std::move(mode)), true);
    capture->add_episode(make_evidence(0, 1, std::move(rows)));
    return capture;
}

// pytest.raises(..., match=...) — exception type plus a message fragment.
template <class Ex, class F>
void throws(F &&action, std::string_view needle, std::string_view message) {
    try {
        std::forward<F>(action)();
    } catch (const Ex &failure) {
        bt::require(needle.empty() ||
                    std::string(failure.what()).find(needle) != std::string::npos,
                    std::string(message) + ": unexpected message '" +
                        failure.what() + "'");
        return;
    }
    bt::require(false, std::string(message) + ": no exception was raised");
}

// Wire member helpers for channel-tree assertions.
inline const Wire *member(const WireObject &object, std::string_view key) {
    return sw::find(object, key);
}
inline const Value *member(const Value::Object &object, std::string_view key) {
    return io::find_member(object, key);
}
inline std::string as_text(const Value &value) {
    return io::as_string(value, "value");
}
inline bool as_bool(const Value &value) {
    return io::as_bool(value, "value");
}
inline std::int64_t as_integer(const Value &value) {
    return io::number_i64(io::as_number(value, "value"), "value");
}
inline double as_real(const Value &value) {
    return io::number_double(io::as_number(value, "value"), "value");
}
inline const Value::Object &as_object(const Value &value) {
    return io::as_object(value, "value");
}

// kinds(events, episode_id) — the lifecycle file's per-episode kind list.
inline std::vector<std::string> kinds(const std::vector<rec::InputEvent> &events,
                                      std::int64_t episode_id = -1) {
    std::vector<std::string> out;
    for (const auto &event : events)
        if (episode_id < 0 || event.episode_id == episode_id)
            out.push_back(event.kind);
    return out;
}

} // namespace recording_test
