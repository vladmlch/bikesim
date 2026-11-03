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

#include "support/fixture_test.hpp"
#include "support/golden.hpp"
#include "recording/analysis_detail.hpp"
#include "recording/capture.hpp"
#include "recording/catalog.hpp"
#include "recording/contracts.hpp"
#include "recording/duckdb.hpp"
#include "recording/reader.hpp"
#include "recording/value_util.hpp"
#include "platform/files.hpp"
#include "runtime/encode.hpp"
#include "runtime/encoding_values.hpp"
#include "runtime/runtime.hpp"
#include "runtime/samples.hpp"
#include "support/temporary_directory.hpp"
#include "io/json.hpp"
#include <algorithm>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
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
      {.name = "root_x", .joint_type = "slide", .qposadr = 0, .dofadr = 0},
      {.name = "root_z", .joint_type = "slide", .qposadr = 1, .dofadr = 1},
      {.name = "root_pitch", .joint_type = "hinge", .qposadr = 2, .dofadr = 2},
      {.name = "front_wheel_spin",
       .joint_type = "hinge",
       .qposadr = 3,
       .dofadr = 3},
      {.name = "rear_wheel_spin",
       .joint_type = "hinge",
       .qposadr = 4,
       .dofadr = 4},
      {.name = "crank_spin", .joint_type = "hinge", .qposadr = 5, .dofadr = 5},
      {.name = "fork_slide", .joint_type = "slide", .qposadr = 6, .dofadr = 6},
      {.name = "shock_slide",
       .joint_type = "slide",
       .qposadr = 7,
       .dofadr = 7}};
  return joints;
}

inline const rec::Catalog &synthetic_catalog() {
    static const rec::Catalog catalog = rec::build_catalog(synthetic_joints());
    return catalog;
}

// One canonical retained interval: qpos/qvel span the 8-coordinate
// synthetic joint layout; channels carry only what callers stage. The
// _recording_support.sample_row parameter surface (start/dt/qpos/qvel/
// forces/powers/channels) — forces and powers_w are per-component maps
// the forces emitter explodes into long rows.
inline SamplePtr sample_row(std::int64_t interval_id, double start_s, // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
                            double dt_s = .00125,
                            std::vector<double> qpos = {},
                            std::vector<double> qvel = {},
                            runtime::ForceComponents forces = {},
                            WireObject powers_w = {},
                            WireObject channels = {}) {
    auto sample = std::make_shared<runtime::PhysicalSampleData>();
    sample->interval_id = interval_id;
    sample->time_s = start_s;
    sample->end_time_s = start_s + dt_s;
    sample->qpos = qpos.empty() ? std::vector<double>(8, 0.) : std::move(qpos);
    sample->qvel = qvel.empty() ? std::vector<double>(8, 0.) : std::move(qvel);
    sample->forces = std::move(forces);
    sample->powers_w = std::move(powers_w);
    sample->channels = std::move(channels);
    return {std::move(sample)};
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
        {"safety_limited", Wire{false}}, {"motor_enabled", Wire{false}},
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

// _recording_support.energy_channels(): the complete authored ``energy``
// root plus the sibling ``component_work_j`` map. Every required leaf
// appears so a published row survives storage validation; optional leaves
// (elastic_energy_j, constraint_work_ratio, residual_j, source_interval_id,
// source_time_s) are authored on the base row and removed per test.
// ``energy_over`` merges into the ``energy`` member map; ``other_over``
// merges at the channel-root level (the oracle's ``**overrides`` split).
inline WireObject energy_channels(const WireObject &energy_over = {}, // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
                                  const WireObject &other_over = {}) {
    WireObject energy{
            {"mechanical_energy_j", Wire{12.5}},
            {"elastic_energy_j", Wire{WireObject{{"fork_spring", Wire{.25}}}}},
            {"active_work_j", Wire{1.5}},
            {"external_work_j", Wire{-.5}},
            {"loss_j", Wire{.25}},
            {"loss_step_j", Wire{.001}},
            {"solver_constraint_work_j", Wire{0.}},
            {"energy_scale_j", Wire{10.}},
            {"muscle_signed_j", Wire{1.}},
            {"muscle_positive_j", Wire{1.25}},
            {"motor_signed_j", Wire{.5}},
            {"motor_positive_j", Wire{.5}},
            {"source_positive_work_j", Wire{1.75}},
            {"constraint_signed_j", Wire{0.}},
            {"constraint_absolute_j", Wire{0.}},
            {"closure_signed_j", Wire{0.}},
            {"closure_absolute_j", Wire{0.}},
            {"constraint_work_ratio", Wire{0.}},
            {"constraint_work_ok", Wire{true}},
            {"residual_j", Wire{-.001}},
            {"measured", Wire{true}},
            {"source_interval_id", Wire{std::int64_t{0}}},
            {"source_time_s", Wire{.001}}};
    sw::merge(energy, WireObject{energy_over});
    WireObject channels{
        {"energy", Wire{std::move(energy)}},
        {"component_work_j", Wire{WireObject{{"chain", Wire{.1}},
                                             {"tires", Wire{-.2}}}}}};
    sw::merge(channels, WireObject{other_over});
    return channels;
}

// Mutable member lookup for staged channel overrides (sw::find is const).
inline Wire &member_ref(WireObject &object, std::string_view key) {
    for (auto &entry : object) if (entry.first == key) return entry.second;
    throw std::invalid_argument("wire member missing: " + std::string(key));
}
inline WireObject &object_ref(Wire &wire, std::string_view field) {
    auto *object = std::get_if<WireObject>(&wire.value);
    bt::require(object != nullptr,
                "wire member is not an object: " + std::string(field));
    return *object;
}
inline WireObject &object_ref(WireObject &object, std::string_view key) {
    return object_ref(member_ref(object, key), key);
}
// `del channels[...]` — the oracle's member-removal staging lane.
inline void wire_erase(WireObject &object, std::string_view key) {
    const auto it = std::ranges::find(
        object, key, &std::pair<std::string, Wire>::first);
    if (it != object.end()) object.erase(it);
}

// _recording_support.run_metadata(): deterministic synthetic metadata.
inline rec::RunMetadata run_metadata(std::string backend = "native",
                                   std::string mode = "viewer") {
    rec::RunMetadata metadata;
    metadata.run_uuid = "00000000-0000-4000-8000-000000000001";
    metadata.started_utc = "2026-01-01T00:00:00+00:00";
    metadata.backend = std::move(backend);
    // Read the check-policy decision before mode moves into metadata.mode —
    // a moved-from string compares unspecified and would freeze "off" into
    // "strict".
    const std::string check_policy =
        mode == "off" ? "diagnostic" : "strict";
    metadata.mode = std::move(mode);
    metadata.values = Value{Value::Object{
        {"check_policy", text(check_policy)},
        {"drive_mode", text("articulated_effort")},
        {"configuration_sha256", text(std::string(64, '0'))}}};
    metadata.wall_origin_s = 1.5;
    return metadata;
}

inline rec::InputEvent input_event(std::int64_t episode_id, // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
                                   std::int64_t event_id,
                                   std::string kind = "episode_start",
                                   double wall_s = 0., // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
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
// kept reachable for iter_derived_events-equivalent re-reads. ``catalog``
// overrides the bound coordinate/alias map (coordinate-identity mismatch
// ports); ``ordinary`` switches on the commands_applied projection.
inline std::shared_ptr<const rec::PhysicalSource>
make_source(std::shared_ptr<const runtime::OwnedNativeEpisode> episode,
            const rec::Catalog &catalog = synthetic_catalog(),
            bool ordinary = false) {
    return rec::make_physical_source(std::move(episode), catalog, ordinary);
}

// storage-contract evidence: summary always carries the reviewed outcome /
// model_status envelope; assets stay empty unless a test stages them.
inline std::shared_ptr<const rec::EpisodeEvidence>
make_evidence(std::int64_t episode_id, int generation = 1, // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
              std::vector<SamplePtr> rows = {},
              Value summary = Value{Value::Object{}},
              const rec::Catalog &catalog = synthetic_catalog(),
              std::vector<rec::Asset> assets = {}) {
    auto evidence = std::make_shared<rec::EpisodeEvidence>();
    evidence->episode_id = episode_id;
    evidence->generation = generation;
    evidence->physical = make_source(
        std::make_shared<runtime::OwnedNativeEpisode>(
            make_episode(std::move(rows), generation)), catalog);
    if (io::as_object(summary, "summary").empty()) {
        summary = Value{Value::Object{
            {"outcome", Value{Value::Object{{"reason", text("synthetic")}}}},
            {"model_status", Value{Value::Object{{"model_valid", Value{true}},
                                                 {"numerically_valid", Value{true}}}}}}};
    }
    evidence->summary = std::move(summary);
    evidence->assets = std::move(assets);
    return evidence;
}

// ---- Batch 3.31 synthetic capture matrix (test_recording_storage.py) ----

// _recording_support.value_case_rows(dt): the reviewed optional-leaf and
// nonfinite matrix on energy.residual_j — NaN, explicit null and absent
// leaf; explicit null and absent ``energy`` parent; +0.0, -0.0, +inf and
// -inf. Every interval is self-measured so source/measurement provenance
// stays consistent.
inline std::vector<SamplePtr> value_case_rows(double dt = 1. / 8) {
    std::vector<SamplePtr> rows;
    for (std::int64_t i = 0; i < 9; ++i) {
        auto channels = energy_channels(WireObject{
            {"measured", Wire{true}},
            {"source_interval_id", Wire{i}},
            {"source_time_s", Wire{static_cast<double>(i + 1) * dt}}});
        auto &energy = object_ref(channels, "energy");
        switch (i) {
            case 0: member_ref(energy, "residual_j") =
                        Wire{std::numeric_limits<double>::quiet_NaN()}; break;
            case 1: member_ref(energy, "residual_j") =
                        Wire{nullptr}; break;
            case 2: wire_erase(energy, "residual_j"); break;
            case 3: member_ref(channels, "energy") =
                        Wire{nullptr}; break;
            case 4: wire_erase(channels, "energy"); break;
            case 5: member_ref(energy, "residual_j") = Wire{0.}; break;
            case 6: member_ref(energy, "residual_j") = Wire{-0.}; break;
            case 7: member_ref(energy, "residual_j") =
                        Wire{std::numeric_limits<double>::infinity()}; break;
            default: member_ref(energy, "residual_j") =
                        Wire{-std::numeric_limits<double>::infinity()}; break;
        }
        rows.push_back(sample_row(i, static_cast<double>(i) * dt, dt,
                                  {}, {}, {}, {}, std::move(channels)));
    }
    return rows;
}

// _recording_support._with_forces(row): hand-computed per-component powers
// (qfrc·qvel) and work (power·dt) — the chain/tires pair the forces table
// and channel_energy.component_work_j share.
inline SamplePtr with_forces(const runtime::PhysicalSampleData &row) {
    runtime::ForceComponents qfrc{
        {"chain", std::vector<double>(8, 0.)},
        {"tires", std::vector<double>(8, 0.)}};
    qfrc[0].second[0] = 3.;
    qfrc[1].second[0] = -1.;
    WireObject powers, work;
    for (const auto &[name, vector] : qfrc) {
        double power = 0.;
        for (std::size_t k = 0; k < vector.size() && k < row.qvel.size(); ++k)
            power += vector[k] * row.qvel[k];
        powers.emplace_back(name, Wire{power});
        work.emplace_back(name, Wire{power * (row.end_time_s - row.time_s)});
    }
    auto sample = std::make_shared<runtime::PhysicalSampleData>(row);
    sample->forces = std::move(qfrc);
    sample->powers_w = std::move(powers);
    sw::set(sample->channels, "component_work_j", Wire{std::move(work)});
    return {std::move(sample)};
}

// _recording_support._forwarded(rows): all but the closing interval forward
// the LAST interval's measurement provenance (measured=false); the closing
// row keeps self-measurement.
inline std::vector<SamplePtr> forwarded(const std::vector<SamplePtr> &rows) {
    bt::require(!rows.empty(), "forwarded() needs a closing interval");
    const auto &closing = *rows.back();
    std::vector<SamplePtr> out;
    out.reserve(rows.size());
    for (const auto &ptr : rows) {
        auto sample = std::make_shared<runtime::PhysicalSampleData>(*ptr);
        WireObject energy;
        if (const auto *existing = sw::find(sample->channels, "energy"))
            if (const auto *object = std::get_if<WireObject>(&existing->value))
                energy = *object;
        sw::set(energy, "measured",
                Wire{ptr->interval_id == closing.interval_id});
        sw::set(energy, "source_interval_id", Wire{closing.interval_id});
        sw::set(energy, "source_time_s", Wire{closing.end_time_s});
        sw::set(sample->channels, "energy", Wire{std::move(energy)});
        out.push_back(SamplePtr(std::move(sample)));
    }
    return out;
}

// _recording_support.synthetic_capture(): a reviewed RunCapture over
// synthetic retained rows — the oracle's TreePhysicalSource seam is the
// production OwnedNativeEpisode + make_physical_source pair natively.
// ``episodes`` carries explicit per-episode row sets (Python's sequence
// branch); ``episode_count`` shares ``rows`` over N episodes (the int
// branch, 0 seals nothing). Summaries index-align when a per-episode
// vector is staged, else the shared ``summary`` applies.
struct SyntheticCapture {
    std::vector<std::vector<SamplePtr>> episodes;
    int episode_count = 1;
    WireObject channels;
    std::vector<rec::InputEvent> events;
    std::vector<rec::Asset> assets;
    std::vector<Value> summaries;
    Value summary{Value::Object{}};
    const rec::Catalog *catalog = nullptr;
    bool forces = false, forwarded = false, value_cases = false;
    bool enabled = true;
    rec::RunMetadata metadata = run_metadata();
};

inline std::unique_ptr<rec::RunCapture>
synthetic_capture(const std::vector<SamplePtr> &rows = {},
                  SyntheticCapture o = {}) {
  auto capture =
      std::make_unique<rec::RunCapture>(std::move(o.metadata), o.enabled);
  std::vector<std::vector<SamplePtr>> per_episode;
  if (o.value_cases)
    per_episode = {value_case_rows()};
  else if (!o.episodes.empty())
    per_episode = std::move(o.episodes);
  else
    for (int i = 0; i < o.episode_count; ++i)
      per_episode.push_back(rows);
  for (std::size_t index = 0; index < per_episode.size(); ++index) {
    std::vector<SamplePtr> materialized;
    materialized.reserve(per_episode[index].size());
    for (const auto &ptr : per_episode[index]) {
      auto sample = std::make_shared<runtime::PhysicalSampleData>(*ptr);
      sw::merge(sample->channels, WireObject{o.channels});
      materialized.push_back(o.forces ? with_forces(*sample)
                                      : SamplePtr(std::move(sample)));
    }
    if (o.forwarded)
      materialized = forwarded(materialized);
    Value episode_summary =
        index < o.summaries.size() ? o.summaries[index] : o.summary;
    capture->add_episode(
        make_evidence(static_cast<std::int64_t>(index), static_cast<int>(index),
                      std::move(materialized), std::move(episode_summary),
                      o.catalog ? *o.catalog : synthetic_catalog(),
                      std::vector<rec::Asset>{o.assets}));
  }
  for (auto &event : o.events)
    capture->event(std::move(event));
  return capture;
}

// open_recording for the RAII-less container owner Recording returns.
inline std::unique_ptr<rec::Recording>
sealed(const std::filesystem::path &path) { return rec::open_recording(path); }

// The oracle's `duckdb.connect(...).execute(...); CHECKPOINT` + manifest
// re-hash reseal path: write-mutate a sealed database, then re-cover the
// payload checksum so only a semantic validator can still refuse it.
inline void mutate_database(const std::filesystem::path &recording,
                            std::string_view sql) {
    {
      rec::Database const database(recording / "run.duckdb");
      database.connection().execute(sql);
      database.connection().execute("CHECKPOINT");
    }
    auto const manifest =
        io::parse_recording_json(bt::text(recording / "manifest.json"));
    auto *hashes = const_cast<Value *>( // NOLINT(cppcoreguidelines-pro-type-const-cast) mutable write through the const lookup view mirrors the oracle's in-place mutation
        rec::detail::find(manifest, "file_sha256"));
    bt::require(hashes != nullptr, "manifest owns file_sha256");
    rec::detail::set(*hashes, "run.duckdb",
                     text(bike::platform::file_sha256(recording / "run.duckdb")));
    std::ofstream out(recording / "manifest.json", std::ios::trunc);
    out << io::recording_json(manifest) << '\n';
}

// ---- captured-bootstrap runtime staging ------------------------------------

// Stage a copy of tests/fixtures/bootstrap/welded with selected
// config.json fields re-written (retention mode, measurement cadence,
// strictness) — the cheap NativeRideRuntime path shared by the retention
// and lifecycle runtime-level ports.
struct WeldedRuntime {
    bt::TemporaryDirectory stage{"recording-welded"};
    std::unique_ptr<runtime::NativeRideRuntime> runtime;
    std::int64_t period = 0;
};

inline void replace_config_key(std::string &text, std::string_view key,
                               std::string_view from, std::string_view to) { // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
    const std::string needle = "\"" + std::string(key) + "\": " + std::string(from);
    const auto at = text.find(needle);
    bt::require(at != std::string::npos, "config key " + std::string(key) + " present");
    text.replace(at, needle.size(),
                 "\"" + std::string(key) + "\": " + std::string(to));
}

inline std::unique_ptr<WeldedRuntime> welded_runtime(
        std::optional<std::string> retention = std::nullopt,
        std::optional<int> decimation = std::nullopt,
        bool strict = false,
        bool floor_pin = false) {
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
    // The re-scoped welded plant is healthy: a strict run needs the same
    // weld_foot_min_normal_n=100 pin welded_floor_docs applies, so the
    // weld-envelope monitor trips a deterministic early violation instead
    // of relying on the pre-F1 fixture's incidental rejection.
    if (floor_pin)
        replace_config_key(config, "weld_foot_min_normal_n", "-1050.0",
                           "100.0");
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
capture_rows(std::vector<SamplePtr> rows, std::string mode = "viewer",
             Value summary = Value{Value::Object{}},
             std::vector<rec::Asset> assets = {}) {
    auto capture = std::make_unique<rec::RunCapture>(
        run_metadata("native", std::move(mode)), true);
    capture->add_episode(make_evidence(0, 1, std::move(rows),
                                     std::move(summary), synthetic_catalog(),
                                     std::move(assets)));
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
