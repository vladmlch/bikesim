// Completion 3.22a. Source: tests/reference/test_native_encode_columns.py.
//
// The ported assertions exercise the typed columnar encoder
// (runtime/encode.cpp) and the production NativePhysicalSource
// (recording/physical_source.cpp) against the frozen oracle output the
// pytest file captured through encode_rows + arrow_batch. Comparisons are
// the byte-exact `column_bits` identity from _encode_oracle.py — validity
// bitmaps, list/map offsets, payload lanes and struct member order — so
// -0.0, NaN payloads, empty-vs-absent maps and NULL-vs-absent optionals
// all stay observable. Tests consume golden entries in the same order the
// pytest node recorded them.
#include "support/fixture_test.hpp"
#include "support/golden.hpp"
#include "support/temporary_directory.hpp"
#include "diag.hpp"
#include "io/json.hpp"
#include "recording/catalog.hpp"
#include "recording/contracts.hpp"
#include "recording/encoding.hpp"
#include "runtime/encode.hpp"
#include "runtime/encoding_values.hpp"
#include "runtime/runtime.hpp"
#include "runtime/samples.hpp"
#include <format>
#include <mujoco/mujoco.h>
#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <ranges>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace bt_oracle = bt::oracle;
namespace rec = bike::recording;
namespace sw = runtime::sample_wire;

namespace {

using runtime::Piece;
using runtime::Wire;
using runtime::WireArray;
using runtime::WireObject;
using runtime::EncodedTable;
using runtime::SamplePtr;
using bike::io::Value;

const char *const NODE_SUITE = "native_encode_columns";

std::string node_id(std::string_view test) {
    auto name = std::string(test);
    if (name.starts_with("test_")) name.erase(0, 5);
    return std::string(NODE_SUITE) + "/" + name;
}

// The exception contract pytest.raises(..., match=...) expresses: type
// plus a message fragment.
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

// ---- authored canonical sample trees (_recording_support.py) --------------

// The small planar joint set of support.synthetic_joints().
const std::vector<rec::JointCoords> SYNTHETIC_JOINTS = { // NOLINT(bugprone-throwing-static-initialization,cert-err58-cpp) static fixture table built once at startup; a throw is a fatal defect
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
    {.name = "shock_slide", .joint_type = "slide", .qposadr = 7, .dofadr = 7}};

const rec::Catalog &support_catalog() {
    static const rec::Catalog catalog = rec::build_catalog(SYNTHETIC_JOINTS);
    return catalog;
}

rec::CoordinateCatalog synthetic_coordinates() {
    // Python Catalog(joints) converts JointCoords through
    // _coordinate_catalog; build_catalog runs the same conversion.
    static const rec::CoordinateCatalog coordinates =
        rec::build_catalog(SYNTHETIC_JOINTS).coordinates();
    return coordinates;
}

Wire w_arr(const std::vector<double> &values) {
  WireArray out;
  out.reserve(values.size());
  for (const double v : values)
    out.emplace_back(v);
  return Wire{std::move(out)};
}
Wire w_ints(const std::vector<std::int64_t> &values) {
  WireArray out;
  for (const std::int64_t v : values)
    out.emplace_back(v);
  return Wire{std::move(out)};
}
Wire w_strs(const std::vector<std::string> &values) {
  WireArray out;
  for (const std::string &v : values)
    out.emplace_back(v);
  return Wire{std::move(out)};
}

// The **overrides protocol of the support builders is dict.update():
// a one-level member replace — a {} override empties the member, it does
// not merge into it. Order is insertion order for new members.
void merge_into(WireObject &into, const WireObject &extra) {
    for (const auto &[key, value] : extra) {
        if (auto *slot = const_cast<Wire *>(sw::find(into, key))) // NOLINT(cppcoreguidelines-pro-type-const-cast) mutable write through the const lookup view mirrors the oracle's in-place mutation
            *slot = value;
        else
            into.emplace_back(key, value);
    }
}

WireObject energy_channels(const WireObject &energy_over = {},
                           const WireObject &other_over = {}) {
    WireObject energy = {
        {"mechanical_energy_j", Wire{12.5}},
        {"elastic_energy_j", Wire{WireObject{{"fork_spring", Wire{0.25}}}}},
        {"active_work_j", Wire{1.5}},
        {"external_work_j", Wire{-0.5}},
        {"loss_j", Wire{0.25}},
        {"loss_step_j", Wire{0.001}},
        {"solver_constraint_work_j", Wire{0.0}},
        {"energy_scale_j", Wire{10.0}},
        {"muscle_signed_j", Wire{1.0}},
        {"muscle_positive_j", Wire{1.25}},
        {"motor_signed_j", Wire{0.5}},
        {"motor_positive_j", Wire{0.5}},
        {"source_positive_work_j", Wire{1.75}},
        {"constraint_signed_j", Wire{0.0}},
        {"constraint_absolute_j", Wire{0.0}},
        {"closure_signed_j", Wire{0.0}},
        {"closure_absolute_j", Wire{0.0}},
        {"constraint_work_ratio", Wire{0.0}},
        {"constraint_work_ok", Wire{true}},
        {"residual_j", Wire{-0.001}},
        {"measured", Wire{true}},
        {"source_interval_id", Wire{std::int64_t{0}}},
        {"source_time_s", Wire{0.001}},
    };
    merge_into(energy, energy_over);
    WireObject channels{{"energy", Wire{std::move(energy)}},
        {"component_work_j",
         Wire{WireObject{{"chain", Wire{0.1}}, {"tires", Wire{-0.2}}}}}};
    merge_into(channels, other_over);
    return channels;
}

WireObject drive_channels(double speed = 0., const WireObject &drive_over = {},
                          const WireObject &other_over = {}) {
    WireObject drive;
    drive.reserve(79);
    drive.emplace_back("transmission_model", Wire{"ideal_mid_drive"});
    drive.emplace_back("omits_suspension_coupling", Wire{true});
    drive.emplace_back("chain_extension_m", Wire{0.001 + speed});
    drive.emplace_back("chain_extension_rate_mps", Wire{0.002 - speed});
    drive.emplace_back("chain_tension_n", Wire{120.0 + speed});
    drive.emplace_back("chain_energy_j", Wire{0.06 + speed});
    drive.emplace_back("chain_dissipation_power_w", Wire{-0.4 - speed});
    drive.emplace_back("chain_power_w", Wire{12.0 - speed});
    drive.emplace_back("freehub_torque_nm", Wire{8.0 + speed});
    drive.emplace_back("freehub_energy_j", Wire{0.02});
    drive.emplace_back("freehub_engaged", Wire{speed < 0.5});
    drive.emplace_back("freehub_deflection_rad", Wire{-0.05 + speed});
    drive.emplace_back("freehub_dissipation_power_w", Wire{speed != 0.0 ? -0.0 : 0.1});
    drive.emplace_back("freehub_power_w", Wire{12.1});
    drive.emplace_back("cadence_rpm", Wire{80.0 + speed});
    drive.emplace_back("crank_rad_s", Wire{8.4 + speed});
    drive.emplace_back("drive_shaft_rad_s", Wire{30.0 - speed});
    drive.emplace_back("crank_phase_rad", Wire{1.25});
    drive.emplace_back("human_torque_nm", Wire{20.0});
    drive.emplace_back("human_sensor_nm", Wire{19.8 + speed});
    drive.emplace_back("human_setpoint_nm", Wire{20.0});
    drive.emplace_back("human_command_nm", Wire{20.0});
    drive.emplace_back("human_active_power_w", Wire{168.0});
    drive.emplace_back("human_joint_power_w", Wire{170.0 - speed});
    drive.emplace_back("assist_demand_gated", Wire{speed >= 0.5});
    drive.emplace_back("assist_sensor_nm", Wire{19.7});
    drive.emplace_back("assist_mode", Wire{"assist"});
    drive.emplace_back("assist_gain", Wire{1.0});
    drive.emplace_back("rider_mode", Wire{"pedaling"});
    drive.emplace_back("coasting_reason", Wire{"downhill"});
    drive.emplace_back("required_cadence_rpm", Wire{72.0});
    drive.emplace_back("crank_target_phase_rad", Wire{1.0});
    drive.emplace_back("crank_target_rate_rad_s", Wire{8.0});
    drive.emplace_back("motor_request_nm", Wire{6.5 + speed});
    drive.emplace_back("motor_setpoint_nm", Wire{6.0});
    drive.emplace_back("motor_limit_nm", Wire{9.0});
    drive.emplace_back("motor_limited_request_nm", Wire{6.5});
    drive.emplace_back("motor_torque_nm", Wire{6.0 + speed});
    drive.emplace_back("motor_control_source", Wire{"assist"});
    drive.emplace_back("safety_limited", Wire{false});
    drive.emplace_back("motor_enabled", Wire{true});
    drive.emplace_back("motor_shaft_power_w", Wire{150.0});
    drive.emplace_back("motor_freewheel_engaged", Wire{true});
    drive.emplace_back("motor_freewheel_torque_nm", Wire{0.4});
    drive.emplace_back("motor_freewheel_dissipation_power_w", Wire{-0.05});
    drive.emplace_back("crank_clutch_torque_nm", Wire{0.3});
    drive.emplace_back("crank_clutch_engaged", Wire{false});
    drive.emplace_back("crank_clutch_dissipation_power_w", Wire{0.0});
    drive.emplace_back("gear_front_teeth", Wire{std::int64_t{44}});
    drive.emplace_back("gear_rear_teeth", Wire{std::int64_t{16}});
    drive.emplace_back("gear_ratio", Wire{2.75});
    drive.emplace_back("shift_active", Wire{false});
    drive.emplace_back("shift_direction", Wire{"none"});
    drive.emplace_back("shift_from_teeth", Wire{std::int64_t{16}});
    drive.emplace_back("shift_count", Wire{static_cast<std::int64_t>(speed)});
    drive.emplace_back("shift_time_s", Wire{0.5 + speed});
    drive.emplace_back("shift_torque_factor", Wire{1.0});
    drive.emplace_back("front_brake_demand", Wire{0.0});
    drive.emplace_back("rear_brake_demand", Wire{0.0});
    drive.emplace_back("rollback_brake_demand", Wire{0.0});
    drive.emplace_back("front_brake_power_w", Wire{-0.0});
    drive.emplace_back("rear_brake_power_w", Wire{0.0});
    drive.emplace_back("front_brake_torque_nm", Wire{0.0});
    drive.emplace_back("rear_brake_torque_nm", Wire{0.0});
    drive.emplace_back("transmission_phi_m", Wire{0.02});
    drive.emplace_back("transmission_boundary_m", Wire{0.05});
    drive.emplace_back("transmission_gap_m", Wire{0.03});
    drive.emplace_back("transmission_tension_n", Wire{40.0});
    drive.emplace_back("transmission_constraint_defect_m", Wire{1e-9});
    drive.emplace_back("transmission_interval_work_j", Wire{0.01});
    drive.emplace_back("transmission_reaction_error_n", Wire{0.001});
    drive.emplace_back("transmission_reference_status", Wire{"qualified"});
    drive.emplace_back("shift_parameter_work_j", Wire{0.0});
    drive.emplace_back("shift_interval_constraint_work_j", Wire{0.0});
    drive.emplace_back("shift_constraint_work_cumulative_j", Wire{0.0});
    merge_into(drive, drive_over);
    WireObject channels{{"drive", Wire{std::move(drive)}}};
    merge_into(channels, other_over);
    return channels;
}

WireObject sensors_channels(double speed = 0., const WireObject &over = {},
                            const WireObject &other_over = {}) {
    WireObject sensors = {
        {"frame_specific_force_body_mps2",
         w_arr({9.81 + speed, 0.0, -0.0})},
        {"frame_gyro_body_rad_s", w_arr({0.0, 0.01 * (speed + 1.0), 0.0})},
        {"encoders_rad_s", Wire{WireObject{
             {"front_wheel", Wire{4.0 + speed}},
             {"rear_wheel", Wire{4.0 + speed}},
             {"crank", Wire{1.4}}}}},
        {"motor_torque_nm", Wire{12.0 + speed}},
        {"human_torque_nm", Wire{20.0}},
    };
    merge_into(sensors, over);
    WireObject channels{{"sensors", Wire{std::move(sensors)}}};
    merge_into(channels, other_over);
    return channels;
}

WireObject mass_channels(bool measured = true) {
    WireObject channels = {
        {"mass", Wire{WireObject{
             {"mass_kg", Wire{95.0}},
             {"com_m", w_arr({0.05, 0.0, 0.9})},
             {"com_velocity_mps", w_arr({0.0, 0.0, -0.1})},
             {"linear_momentum_kg_mps", w_arr({0.0, 0.0, -9.5})},
             {"angular_momentum_kg_m2_s", w_arr({0.0, 1.0, 0.0})},
             {"kinetic_energy_j", Wire{4.5}},
             {"gravitational_energy_j", Wire{838.0}},
             {"measurement_scope", Wire{"full"}}}}},
    };
    if (measured)
        channels.emplace_back("endpoint_mass", Wire{WireObject{
            {"mass_kg", Wire{95.1}},
            {"com_m", w_arr({0.05, 0.0, 0.9})},
            {"com_velocity_mps", w_arr({0.0, 0.0, -0.1})},
            {"kinetic_energy_j", Wire{4.5}},
            {"gravitational_energy_j", Wire{838.0}}}});
    return channels;
}

WireObject control_channels(bool posture = true, bool pelvis_offset = false) {
    WireObject control = {
        {"motor_torque_nm", Wire{5.5}},
        {"motor_limit_nm", Wire{12.0}},
        {"human_torque_nm", Wire{30.0}},
        {"crank_target_rate_rad_s", Wire{8.0}},
        {"rider_enabled", Wire{true}},
    };
    if (posture) {
        WireObject value{{"torso_lean_rad", Wire{0.15}},
            {"pelvis_pitch_rad", Wire{0.02}}, {"use_saddle", Wire{true}}};
        if (pelvis_offset)
            value.emplace_back("pelvis_offset_m", w_arr({0.02, 0.01}));
        control.emplace_back("posture", Wire{std::move(value)});
    }
    return {{"control", Wire{std::move(control)}}};
}

WireObject model_status_channels(const Wire &verdict = Wire{true},
                                 bool violation = false,
                                 const WireObject &over = {}) {
    WireObject status = {
        {"model_valid", Wire{true}},
        {"model_violation_counts", Wire{WireObject{
             {"impact_load", Wire{2.0}}, {"edge_drop", Wire{1.0}}}}},
        {"numerically_valid", verdict},
        {"calibration_status", Wire{"qualified"}},
    };
    if (violation)
        status.emplace_back("first_model_violation", Wire{WireObject{
            {"interval_id", Wire{std::int64_t{4}}},
            {"time_s", Wire{0.004}},
            {"reasons", w_strs({"energy_closure"})}}});
    merge_into(status, over);
    return {{"model_status", Wire{std::move(status)}}};
}

WireObject tire_patch(double load = 160., bool couple_world = true,
                      const char *source_geom = "terrain") {
    WireObject patch = {
        {"point_m", w_arr({0.0, 0.02, 0.0})},
        {"normal", w_arr({0.0, 0.0, 1.0})},
        {"normal_load_n", Wire{load}},
        {"tangent_force_n", Wire{-1.5}},
        {"slip_mps", Wire{0.01}},
        {"world_force_n", w_arr({0.0, -1.5, load})},
        {"source_geom", Wire{source_geom}},
    };
    if (couple_world)
        patch.emplace_back("couple_world_nm", w_arr({0.0, 0.05, 0.0}));
    return patch;
}

WireObject tire_side(std::vector<WireObject> patches = {},
                     double seed = 0.) {
    const bool contact = !patches.empty();
    WireArray listed;
    for (auto &patch : patches) listed.emplace_back(std::move(patch));
    return {
        {"backend", Wire{"point"}},
        {"unloaded_radius_m", Wire{0.34}},
        {"supports_multiple_contacts", Wire{true}},
        {"outside_material_load_range", Wire{false}},
        {"outside_material_deflection_range", Wire{!contact}},
        {"patches", Wire{std::move(listed)}},
        {"geometric_contact", Wire{contact}},
        {"raw_contact", Wire{contact}},
        {"controller_grounded", Wire{contact}},
        {"multi_support", Wire{patches.size() > 1}},
        {"normal_load_n", Wire{320.0 + seed}},
        {"world_force_n", w_arr({0.0, -1.5, 320.0 + seed})},
        {"vertical_force_n", Wire{320.0 + seed}},
        {"normal_vertical_n", Wire{318.0 + seed}},
        {"tangent_force_n", Wire{-2.5}},
        {"effective_radius_m", Wire{0.335}},
        {"wheel_axis_m", w_arr({0.0, 1.05, 0.34})},
        {"wheel_axis_moment_nm", Wire{0.8}},
        {"omega_abs_rad_s", Wire{4.0 + seed}},
        {"omega_rel_rad_s", Wire{4.0 + seed}},
        {"slip_mps", Wire{0.02}},
        {"slip_ratio", Wire{0.01}},
        {"tangent_center_speed_mps", Wire{2.0}},
        {"penetration_m", Wire{contact ? 0.003 : 0.0}},
        {"surface", Wire{"asphalt"}},
        {"radial_energy_j", Wire{0.4 + seed}},
        {"station_count", Wire{std::int64_t{24}}},
    };
}

WireObject tires_channels(std::vector<WireObject> front = {},
                          std::vector<WireObject> rear = {}) {
    WireObject tires;
    tires.emplace_back("front", Wire{tire_side(std::move(front), 0.0)});
    tires.emplace_back("rear", Wire{tire_side(std::move(rear), 10.0)});
    return {{"tires", Wire{std::move(tires)}}};
}

WireObject rider_channels(const WireObject &over = {}) {
    WireObject channels = {
        {"rider", Wire{WireObject{
             {"saddle", Wire{WireObject{
                  {"enabled", Wire{true}}, {"in_platform", Wire{true}},
                  {"normal_load_n", Wire{310.0}}, {"gap_m", Wire{0.0}},
                  {"patches", Wire{WireArray{Wire{WireObject{
                       {"in_platform", Wire{true}},
                       {"normal_load_n", Wire{150.0}},
                       {"gap_m", Wire{0.0}},
                       {"point_m", w_arr({0.0, 0.0, 0.0})},
                       {"force_on_rider_n", w_arr({0.0, 0.0, -150.0})},
                       {"radial_energy_j", Wire{0.2}},
                       {"shear_energy_j", Wire{0.01}}}}}}}}}},
             {"bar", Wire{WireObject{
                  {"enabled", Wire{true}}, {"reachable", Wire{true}},
                  {"hand_gap_m", Wire{0.005}}}}}}}},
        {"rider_welds", Wire{WireObject{
             {"pedal_left", Wire{WireObject{
                  {"force_on_rider_n", w_arr({0.0, 0.0, -45.0})},
                  {"normal_n", Wire{45.0}},
                  {"would_separate", Wire{false}}}}},
             // Owned by the weld_crank_torque_nm leaf — never a map entry.
             {"crank_torque_nm", Wire{6.25}}}}},
        {"rider_support_targets", Wire{WireObject{
             {"stance", Wire{WireObject{
                  {"front", Wire{false}}, {"rear", Wire{true}}}}},
             {"requested_vertical_forces_n", Wire{WireObject{
                  {"saddle", Wire{300.0}}}}},
             {"feasible_pedal_force_on_bike_n", Wire{WireObject{
                  {"left", w_arr({0.0, 0.0, -200.0})}}}},
             {"posture", Wire{WireObject{
                  {"torso_lean_rad", Wire{0.12}},
                  {"pelvis_pitch_rad", Wire{0.0}},
                  {"use_saddle", Wire{true}}}}},
             {"crank_tracking", Wire{WireObject{
                  {"actual_phase_rad", Wire{1.0}},
                  {"actual_rate_rad_s", Wire{7.9}}}}},
             {"feet", Wire{WireObject{
                  {"left", Wire{WireObject{
                       {"ik_saturated", Wire{false}},
                       {"goal_geometry", Wire{WireObject{
                            {"saturated", Wire{false}},
                            {"weld", Wire{true}}}}}}}}}}}}}},
        {"rider_intent", Wire{WireObject{
             {"posture", Wire{WireObject{
                  {"torso_lean_rad", Wire{0.1}},
                  {"pelvis_pitch_rad", Wire{0.0}},
                  {"use_saddle", Wire{true}}}}},
             {"effort_ceiling_nm", Wire{90.0}}}}},
    };
    merge_into(channels, over);
    return channels;
}

WireObject suspension_channels(const WireObject &over = {}) {
    WireObject suspension = {
        {"fork_travel_m", Wire{0.01}},
        {"shock_stroke_m", Wire{0.005}},
        {"fork_velocity_mps", Wire{-0.2}},
        {"shock_velocity_mps", Wire{0.1}},
        {"shock_solver_limit_power_w", Wire{0.0}},
        {"linkage_closure_max_m", Wire{1e-9}},
        {"generalized_force_components_n", Wire{WireObject{
             {"fork_slide", Wire{12.5}}, {"shock_slide", Wire{-3}}}}},
    };
    merge_into(suspension, over);
    return {{"suspension", Wire{std::move(suspension)}}};
}

SamplePtr sample_row(std::int64_t interval_id = 0, double start = 0., // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
                     double dt = 0.001,
                     const std::vector<double> &qpos = {},
                     const std::vector<double> &qvel = {},
                     runtime::ForceComponents forces = {},
                     WireObject powers = {}, WireObject channels = {}) {
    auto sample = std::make_shared<runtime::PhysicalSampleData>();
    sample->interval_id = interval_id;
    sample->time_s = start;
    sample->end_time_s = start + dt;
    sample->qpos = qpos.empty() ? std::vector<double>(8, 0.) : qpos;
    sample->qvel = qvel.empty() ? std::vector<double>(8, 0.) : qvel;
    sample->forces = std::move(forces);
    sample->powers_w = std::move(powers);
    sample->channels = std::move(channels);
    return {std::move(sample)};
}

runtime::OwnedNativeEpisode make_episode(std::vector<SamplePtr> rows,
                                         int generation = 1) {
    return runtime::OwnedNativeEpisode::seal(
        generation, std::span<const SamplePtr>(rows), runtime::ColumnLayout{});
}

std::int64_t episode_rows(const runtime::OwnedNativeEpisode &episode) {
    return static_cast<std::int64_t>(episode.samples.size());
}

// ---- the column_bits mirror ------------------------------------------------

// _encode_oracle.column_bits splits an Arrow column's byte identity into
// '|'-joined parts. The piece lanes are the exact bytes the emitter fed
// pieces_to_arrow, so the same sequence is computable without Arrow:
//   scalar_[d,i,b,s] : [bitmap?] + payload (zeroed at null slots / tags)
//   list_*           : [bitmap?] + i32 offsets + child payload parts
//   map              : [bitmap?] + i32 offsets + key parts + item parts
//   struct (no offs) : [bitmap always] + member parts in catalog order
//   struct[] (offs)  : [bitmap?] + i32 offsets + [inner struct parts]
// Arrow packs validity LSB-first and only allocates a bitmap when a null
// exists (structs always read buffer 0 — an absent mask contributes an
// empty part, still visible in the join).

std::size_t piece_rows(const Piece &piece) { // NOLINT(misc-no-recursion) recursion mirrors the document tree
    if (!piece.offsets.empty()) return piece.offsets.size() - 1;
    switch (piece.kind) {
        case Piece::Kind::scalar_d: return piece.d.size();
        case Piece::Kind::scalar_i: return piece.i.size();
        case Piece::Kind::scalar_b: return piece.b.size();
        case Piece::Kind::scalar_s: return piece.s.size();
        case Piece::Kind::structure:
            if (!piece.valid.empty()) return piece.valid.size();
            if (!piece.children.empty())
                return piece_rows(piece.children.begin()->second);
            return 0;
        case Piece::Kind::list_d: case Piece::Kind::list_i:
        case Piece::Kind::list_s: case Piece::Kind::map:
            return 0; // list/map pieces always carry offsets
    }
    bt::require(false, "unhandled piece kind");
    return 0;
}

// Arrow's boolean bitmap, little-endian bit order, ceil(n/8) bytes.
std::string bitmap(const std::vector<char> &valid) {
    std::string out((valid.size() + 7) / 8, '\0');
    for (std::size_t i = 0; i < valid.size(); ++i)
        if (valid[i]) out[i >> 3] = static_cast<char>( // NOLINT(bugprone-signed-bitwise) fixed-width literal shift is the fixture contract
            out[i >> 3] | static_cast<char>(1u << (i & 7))); // NOLINT(bugprone-signed-bitwise) fixed-width literal shift is the fixture contract
    return out;
}

// Arrow normalizes nested nulls when it materializes a StructArray from
// row dicts: a member slot inside a *null* parent element is stored
// VALID (zero payload) — the parent mask alone carries the nullness.
// `ctx` is the enclosing element's arrow-validity lane (nullptr marks the
// top-level, where every slot is valid). The returned lane is this node's
// own arrow validity: bit=1 means "non-null slot" in Arrow terms.
std::vector<char> arrow_validity(const Piece &piece,
                                 const std::vector<char> *ctx) {
    const std::size_t rows = piece_rows(piece);
    std::vector<char> av(rows, 1);
    for (std::size_t i = 0; i < rows; ++i) {
        const bool valid = piece.valid.empty() ||
            (i < piece.valid.size() && piece.valid[i]);
        const bool parent_ok =
            ctx == nullptr || (i < ctx->size() && (*ctx)[i]);
        av[i] = static_cast<char>(valid || !parent_ok);
    }
    return av;
}

bool any_masked(const std::vector<char> &av) {
    return std::ranges::any_of(av, [](char v) { return v == 0; });
}

void append_i64(std::string &out, std::int64_t value) {
    const auto raw = std::bit_cast<std::array<std::byte, 8>>(value);
    for (const std::byte b : raw) out.push_back(std::to_integer<char>(b));
}
void append_f64(std::string &out, double value) {
    append_i64(out, static_cast<std::int64_t>(std::bit_cast<std::uint64_t>(value)));
}
void append_i32(std::string &out, std::int32_t value) {
    const auto raw = std::bit_cast<std::array<std::byte, 4>>(value);
    for (const std::byte b : raw) out.push_back(std::to_integer<char>(b));
}

const rec::FieldSpec BOOL_FIELD{ // NOLINT(bugprone-throwing-static-initialization,cert-err58-cpp) static fixture table built once at startup; a throw is a fatal defect
    .key = "", .dtype = "boolean", .unit = "n/a", .phase = "n/a",
    .optional = false, .measurement = "declared", .source = "test",
    .description = "generated boolean column", .frame = "n/a", .path = {},
    .aliases = {}, .shape = std::nullopt, .axes = std::nullopt,
    .except_keys = {}, .children = {}};

void bits_into(std::vector<std::string> &parts, const Piece &piece,
               const rec::FieldSpec &field, const std::vector<char> *ctx);

// Scalar payload: bitmap iff Arrow materializes a null, then one
// fixed-width lane. Masked slots carry the column_bits fill tags; slots
// under an absent parent stay valid and carry the raw lane bytes.
void scalar_bits(std::vector<std::string> &parts, const Piece &piece,
                 std::string_view dtype, const std::vector<char> *ctx) {
    const std::vector<char> av = arrow_validity(piece, ctx);
    if (any_masked(av)) parts.push_back(bitmap(av));
    const std::size_t rows = piece_rows(piece);
    std::string payload;
    if (dtype == "double")
        for (std::size_t i = 0; i < rows; ++i)
            append_f64(payload,
                       av[i] ? (i < piece.d.size() ? piece.d[i] : 0.) : 0.);
    else if (dtype == "bigint")
        for (std::size_t i = 0; i < rows; ++i)
            append_i64(payload,
                       av[i] ? (i < piece.i.size() ? piece.i[i] : 0) : 0);
    else if (dtype == "boolean")
        for (std::size_t i = 0; i < rows; ++i)
            payload.push_back(!av[i] ? '\xff'
                                     : (i < piece.b.size() && piece.b[i])
                                           ? '\x01'
                                           : '\x00');
    else
        for (std::size_t i = 0; i < rows; ++i)
            if (!av[i]) payload.push_back('\xff');
            else {
                payload += '\xfe';
                if (i < piece.s.size()) payload += piece.s[i];
            }
    parts.push_back(std::move(payload));
}

// The child arrays under a list: flat lanes with no validity of their own.
void flat_lane_bits(std::vector<std::string> &parts, const Piece &piece,
                    std::string_view member) {
    std::string payload;
    if (member == "double")
        for (const double v : piece.d) append_f64(payload, v);
    else if (member == "bigint")
        for (const std::int64_t v : piece.i) append_i64(payload, v);
    else
        for (const std::string &v : piece.s) {
            payload += '\xfe';
            payload += v;
        }
    parts.push_back(std::move(payload));
}

void bits_into(std::vector<std::string> &parts, const Piece &piece, // NOLINT(misc-no-recursion) recursion mirrors the document tree
               const rec::FieldSpec &field, const std::vector<char> *ctx) {
    const std::string &dtype = field.dtype;
    switch (piece.kind) {
        case Piece::Kind::scalar_d:
            scalar_bits(parts, piece, "double", ctx);
            return;
        case Piece::Kind::scalar_i:
            scalar_bits(parts, piece, "bigint", ctx);
            return;
        case Piece::Kind::scalar_b:
            scalar_bits(parts, piece, "boolean", ctx);
            return;
        case Piece::Kind::scalar_s:
            scalar_bits(parts, piece, "varchar", ctx);
            return;
        case Piece::Kind::list_d: case Piece::Kind::list_i:
        case Piece::Kind::list_s: {
            const std::vector<char> av = arrow_validity(piece, ctx);
            if (any_masked(av)) parts.push_back(bitmap(av));
            std::string offsets;
            for (const std::int32_t edge : piece.offsets) append_i32(offsets, edge);
            parts.push_back(std::move(offsets));
            const std::string member = dtype.substr(0, dtype.size() - 2);
            flat_lane_bits(parts, piece, member);
            return;
        }
        case Piece::Kind::map: {
            const std::vector<char> av = arrow_validity(piece, ctx);
            if (any_masked(av)) parts.push_back(bitmap(av));
            std::string offsets;
            for (const std::int32_t edge : piece.offsets) append_i32(offsets, edge);
            parts.push_back(std::move(offsets));
            // The MapArray's key column is an all-valid string array.
            flat_lane_bits(parts, piece, "varchar");
            const std::string member = dtype.substr(4, dtype.size() - 5);
            rec::FieldSpec const member_spec{
                .key = "",
                .dtype = member == "struct" ? "struct" : member,
                .unit = "n/a",
                .phase = "n/a",
                .optional = false,
                .measurement = "declared",
                .source = "test",
                .description = "map member",
                .frame = "n/a",
                .path = {},
                .aliases = {},
                .shape = std::nullopt,
                .axes = std::nullopt,
                .except_keys = {},
                .children = field.children};
            bt::require(piece.items != nullptr, "map piece without items lane");
            // Map entries are always valid, so the item lane gets an
            // all-valid context.
            bits_into(parts, *piece.items, member_spec, nullptr);
            return;
        }
        case Piece::Kind::structure: {
            const std::vector<char> av = arrow_validity(piece, ctx);
            const std::vector<char> *member_ctx = nullptr;
            if (!piece.offsets.empty()) {
                // struct[]: the list slot may be null; the inner StructArray
                // is always valid so its buffer-0 part is empty unless the
                // emitter marked element validity (it never does). Member
                // lanes are indexed by element, and list elements are never
                // null — an all-valid member context.
                if (any_masked(av)) parts.push_back(bitmap(av));
                std::string offsets;
                for (const std::int32_t edge : piece.offsets)
                    append_i32(offsets, edge);
                parts.push_back(std::move(offsets));
                parts.emplace_back();
            } else {
                parts.push_back(any_masked(av) ? bitmap(av) : std::string{});
                member_ctx = &av;
            }
            for (const rec::FieldSpec &child : field.children) {
                const auto member = piece.children.find(child.key);
                bt::require(member != piece.children.end(),
                            "struct piece is missing member " + child.key);
                bits_into(parts, member->second, child, member_ctx);
                if (child.optional) {
                    const auto present =
                        piece.children.find(child.key + "_present");
                    bt::require(present != piece.children.end(),
                                "struct piece is missing member " + child.key +
                                    "_present");
                    bits_into(parts, present->second, BOOL_FIELD, member_ctx);
                }
            }
            return;
        }
    }
    bt::require(false, "unhandled piece kind " +
                           std::to_string(static_cast<int>(piece.kind)));
}

std::string column_bits(const Piece &piece, const rec::FieldSpec &field) {
    std::vector<std::string> parts;
    bits_into(parts, piece, field, nullptr);
    std::string out;
    for (const auto &part : parts) {
        out += part;
        out += '|';
    }
    if (!out.empty()) out.pop_back();
    return out;
}

void bytes_compare(bt_oracle::Node &node, std::string_view bytes,
                   std::string_view dtype, std::size_t count,
                   const std::string &label) {
  node.bytes(std::as_bytes(std::span(bytes)), dtype,
             std::array<std::size_t, 1>{count}, label);
}

void rows_compare(bt_oracle::Node &node, std::int64_t rows,
                  const std::string &label) {
    node.bytes(std::as_bytes(std::span<const std::int64_t>(&rows, 1)),
               "<i8", std::array<std::size_t, 1>{1}, label);
}

// The _record_batch_bits consumer: `<prefix>/rows` then one
// `<prefix>/bits/<name>` per column_types(spec) member, in order. The
// episode_id column is synthesized exactly like arrow_batch's injection
// when the encoder did not emit it.
void compare_batch(bt_oracle::Node &node, const EncodedTable &table,
                   const rec::TableSpec &spec, std::int64_t episode_id,
                   const std::string &prefix) {
    rows_compare(node, table.rows, prefix + "/rows");
    const auto types = rec::column_types(spec);
    std::size_t covered = 0;
    for (const auto &[name, duck_type] : types) {
        (void)duck_type;
        const Piece *piece = nullptr;
        Piece synth;
        if (const auto found = table.columns.find(name);
            found != table.columns.end())
            piece = &found->second;
        else if (name == "episode_id") {
            synth.kind = Piece::Kind::scalar_i;
            synth.i.assign(static_cast<std::size_t>(table.rows), episode_id);
            piece = &synth;
        } else {
            // Visible-in-TU throw, not bt::require: the analyzers cannot
            // prove the `*piece` dereference below is unreachable on the
            // missing-column path. Identical failure mode —
            // std::runtime_error with the same message.
            throw std::runtime_error(spec.name + ": encoder emitted no " +
                                     name + " column");
        }
        const rec::FieldSpec *field = spec.field(name);
        const std::string bits = column_bits(*piece, field ? *field : BOOL_FIELD);
        const std::string label = std::format("{}/bits/{}", prefix, name);
        // Record mode: recorded columns this encoder no longer emits sit
        // before the next emitted one in the frozen label order (or after
        // the last one, dropped below the loop); drop them.
        while (bt_oracle::Node::recording() && node.has_next() &&
               node.peek_label() != label &&
               node.peek_label().starts_with(prefix + "/bits/")) {
            static_cast<void>(node.bytes_input(node.peek_label()));
            node.drop_consumed();
        }
        bytes_compare(node, bits, "|u1", bits.size(), label);
        ++covered;
    }
    while (bt_oracle::Node::recording() && node.has_next() &&
           node.peek_label().starts_with(prefix + "/bits/")) {
        static_cast<void>(node.bytes_input(node.peek_label()));
        node.drop_consumed();
    }
    bt::require(covered == table.columns.size() +
                               (table.columns.contains("episode_id") ? 0 : 1),
                spec.name + ": encoder emitted undeclared columns");
}

// compile_plan + encode_table_native + the RecordBatch-bit compare —
// oracle.native_record_batch + the frozen column_bits sequence.
void assert_identical(bt_oracle::Node &node, const rec::TableSpec &spec,
                      const runtime::OwnedNativeEpisode &episode,
                      const rec::Catalog &catalog, std::int64_t episode_id = 0) {
  auto const plan = rec::compile_plan(catalog, spec.name);
  auto const table =
      runtime::encode_table_native(episode, plan, 0, episode_rows(episode));
  rec::validate_pieces(table, spec);
  compare_batch(node, table, spec, episode_id, spec.name);
}

// Pieces are move-only; the merge needs a deep copy for first insertion.
Piece copy_piece(const Piece &piece) { // NOLINT(misc-no-recursion) recursion mirrors the document tree
    Piece out;
    out.kind = piece.kind;
    out.d = piece.d;
    out.i = piece.i;
    out.b = piece.b;
    out.s = piece.s;
    out.offsets = piece.offsets;
    out.valid = piece.valid;
    for (const auto &[key, child] : piece.children)
        out.children.emplace(key, copy_piece(child));
    if (piece.items) out.items = std::make_unique<Piece>(copy_piece(*piece.items));
    return out;
}

// concat_batches over emitted fragments: piece lanes concatenate, offsets
// rebase like _slice_piece + concat, validity masks widen all-valid lanes.
Piece concat_piece(const Piece &a, const Piece &b, std::size_t rows_a, // NOLINT(misc-no-recursion) recursion mirrors the document tree
                   std::size_t rows_b) {
    bt::require(a.kind == b.kind, "fragments disagree on piece kind");
    Piece out;
    out.kind = a.kind;
    out.d.assign(a.d.begin(), a.d.end());
    out.d.insert(out.d.end(), b.d.begin(), b.d.end());
    out.i.assign(a.i.begin(), a.i.end());
    out.i.insert(out.i.end(), b.i.begin(), b.i.end());
    out.b.assign(a.b.begin(), a.b.end());
    out.b.insert(out.b.end(), b.b.begin(), b.b.end());
    out.s.assign(a.s.begin(), a.s.end());
    out.s.insert(out.s.end(), b.s.begin(), b.s.end());
    if (!a.valid.empty() || !b.valid.empty()) {
        if (!a.valid.empty()) out.valid.assign(a.valid.begin(), a.valid.end());
        else out.valid.assign(rows_a, '\x01');
        if (!b.valid.empty())
            out.valid.insert(out.valid.end(), b.valid.begin(), b.valid.end());
        else out.valid.insert(out.valid.end(), rows_b, '\x01');
    }
    if (!a.offsets.empty()) {
        out.offsets = a.offsets;
        const std::int32_t base = a.offsets.back();
        for (std::size_t i = 1; i < b.offsets.size(); ++i)
            out.offsets.push_back(base + b.offsets[i]);
    }
    for (const auto &[key, child] : a.children)
        out.children.emplace(key, concat_piece(child, b.children.at(key),
                                               piece_rows(child),
                                               piece_rows(b.children.at(key))));
    if (a.items)
        out.items = std::make_unique<Piece>(concat_piece(
            *a.items, *b.items, piece_rows(*a.items), piece_rows(*b.items)));
    return out;
}

EncodedTable concat_tables(const std::vector<EncodedTable> &fragments) {
    EncodedTable merged;
    for (const auto &fragment : fragments) {
        merged.rows += fragment.rows;
        for (const auto &[name, piece] : fragment.columns) {
            const auto found = merged.columns.find(name);
            if (found == merged.columns.end())
                merged.columns.emplace(name, copy_piece(piece));
            else
                found->second = concat_piece(
                    found->second, piece, piece_rows(found->second),
                    piece_rows(piece));
        }
    }
    return merged;
}

// ---- derived scan (recording/native.py:iter_derived_events) --------------

// The _DERIVED_FIELDS (kind, path) table the Python module authors.
const std::pair<const char *, std::vector<const char *>> DERIVED_FIELDS[] = { // NOLINT(bugprone-throwing-static-initialization,cert-err58-cpp,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) static fixture table built once at startup; a throw is a fatal defect; bounded label buffer for snprintf
    {"tire_contact", {"tires", "front", "raw_contact"}},
    {"tire_contact", {"tires", "rear", "raw_contact"}},
    {"tire_contact", {"tires", "front", "controller_grounded"}},
    {"tire_contact", {"tires", "rear", "controller_grounded"}},
    {"shift", {"drive", "shift_active"}},
    {"shift", {"drive", "shift_direction"}},
    {"freehub_coast", {"drive", "coasting_reason"}},
    {"freehub_coast", {"drive", "freehub_engaged"}},
    {"drive_limits", {"drive", "safety_limited"}},
    {"drive_limits", {"drive", "motor_enabled"}},
    {"brake_gate", {"drive", "front_brake_demand"}},
    {"brake_gate", {"drive", "rear_brake_demand"}},
    {"brake_gate", {"drive", "rollback_brake_demand"}},
    {"model_validity", {"model_status", "model_valid"}},
    {"model_validity", {"model_status", "numerically_valid"}},
    {"model_validity", {"model_status", "model_violation_counts"}},
    {"model_first_failure", {"model_status", "first_model_violation"}},
    {"contact_crash", {"contact_crash_cause"}},
};

Value derived_plan_value() {
    Value::Array fields;
    for (const auto &[kind, path] : DERIVED_FIELDS) {
        Value::Array members;
        for (const char *member : path)
            members.emplace_back(Value{std::string(member)});
        fields.push_back(Value{Value::Object{
            {"kind", Value{std::string(kind)}},
            {"path", Value{std::move(members)}}}});
    }
    return Value{Value::Object{{"fields", Value{std::move(fields)}}}};
}

// The boxed derived_rows() rows, in binding member order.
std::vector<WireObject> derived_rows(const runtime::OwnedNativeEpisode &episode,
                                     const runtime::DerivedEventPlan &plan,
                                     std::int64_t start, std::int64_t limit) {
    std::vector<WireObject> out;
    for (const auto &row :
         runtime::derived_event_rows_native(episode, plan, start, limit))
        out.push_back(WireObject{
            {"interval_id", Wire{row.interval_id}},
            {"kind", Wire{"derived_transition"}},
            {"derived_kind", Wire{row.derived_kind}},
            {"changes", Wire{row.changes}}});
    return out;
}

// The iter_derived_events native branch — chunked derived_rows calls plus
// the InputEvent wrap (identity, steps, values tree) kept byte-faithful.
std::vector<WireObject> derived_events(const runtime::OwnedNativeEpisode &episode,
                                       std::int64_t episode_id, // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
                                       std::int64_t start_event_id,
                                       double received_wall_s) {
    const auto plan = runtime::parse_derived_event_plan(derived_plan_value());
    std::vector<WireObject> out;
    std::int64_t event_id = start_event_id;
    for (std::int64_t start = 0; start < episode_rows(episode); start += 1024) {
        for (const auto &row : runtime::derived_event_rows_native(
                 episode, plan, start, std::min<std::int64_t>(1024, episode_rows(episode) - start))) {
            out.push_back(WireObject{
                {"episode_id", Wire{episode_id}},
                {"event_id", Wire{event_id}},
                {"kind", Wire{"derived_transition"}},
                {"received_wall_s", Wire{received_wall_s}},
                {"processed_step", Wire{row.interval_id}},
                {"applied_step", Wire{row.interval_id}},
                {"values", Wire{WireObject{
                     {"classification", Wire{"derived_transition"}},
                     {"derived_kind", Wire{row.derived_kind}},
                     {"derivation_rule", Wire{"channel_value_change.v1"}},
                     {"interval_id", Wire{row.interval_id}},
                     {"changes", Wire{row.changes}}}}}});
            ++event_id;
        }
    }
    return out;
}

// native_encode_columns/_freeze — the typed-tree serializer the
// captured 'events' blobs are written in.
void freeze_into(std::string &out, const Wire &value) { // NOLINT(misc-no-recursion) recursion mirrors the document tree
    if (std::holds_alternative<std::monostate>(value.value)) { out += 'n'; return; }
    if (const auto *b = std::get_if<bool>(&value.value)) {
        out += 'b';
        out += *b ? '\x01' : '\x00';
        return;
    }
    if (const auto *i = std::get_if<std::int64_t>(&value.value)) {
        out += 'i';
        append_i64(out, *i);
        return;
    }
    if (const auto *d = std::get_if<double>(&value.value)) {
        out += 'f';
        append_f64(out, *d);
        return;
    }
    if (const auto *s = std::get_if<std::string>(&value.value)) {
        out += 's';
        append_i32(out, static_cast<std::int32_t>(s->size()));
        out += *s;
        return;
    }
    if (const auto *a = std::get_if<WireArray>(&value.value)) {
        out += 'l';
        append_i32(out, static_cast<std::int32_t>(a->size()));
        for (const Wire &member : *a) freeze_into(out, member);
        return;
    }
    if (const auto *o = std::get_if<WireObject>(&value.value)) {
        out += 'd';
        append_i32(out, static_cast<std::int32_t>(o->size()));
        for (const auto &[key, member] : *o) {
            out += key;
            out += '\x00';
            freeze_into(out, member);
        }
        return;
    }
    bt::require(false, "unhandled Wire alternative in freeze");
}

std::string freeze(const Wire &value) {
    std::string out;
    freeze_into(out, value);
    return out;
}
std::string freeze(const std::vector<WireObject> &rows) {
    WireArray wrapped;
    for (const auto &row : rows)
      wrapped.emplace_back(row);
    return freeze(Wire{std::move(wrapped)});
}

// assert_plain_bitwise — chunked vs whole walk that consumes the recorded
// scalar leaves. Labels mirror the pytest `what[key!r]` / `what[index]`
// convention so the golden cursor lands on each captured f64 scalar.
void plain_bitwise(bt_oracle::Node *node, const Wire &actual, // NOLINT(misc-no-recursion) recursion mirrors the document tree
                   const Wire &expected, const std::string &what) {
    if (const auto *e_obj = std::get_if<WireObject>(&expected.value)) {
        const auto *a_obj = std::get_if<WireObject>(&actual.value);
        bt::require(a_obj != nullptr, what + ": expected a mapping");
        std::vector<std::string> a_keys, e_keys;
        for (const auto &[key, member] : *a_obj) a_keys.push_back(key);
        for (const auto &[key, member] : *e_obj) e_keys.push_back(key);
        bt::require(a_keys == e_keys, what + ": member order differs");
        for (const auto &[key, member] : *e_obj)
            plain_bitwise(node, *sw::find(*a_obj, key), member,
                          what + "['" + key + "']"); // NOLINT(performance-inefficient-string-concatenation) golden label assembly mirrors the Python oracle's label building
        return;
    }
    if (const auto *e_arr = std::get_if<WireArray>(&expected.value)) {
        const auto *a_arr = std::get_if<WireArray>(&actual.value);
        bt::require(a_arr != nullptr && a_arr->size() == e_arr->size(),
                    what + ": expected a same-length list");
        for (std::size_t i = 0; i < e_arr->size(); ++i)
            plain_bitwise(node, (*a_arr)[i], (*e_arr)[i],
                          what + "[" + std::to_string(i) + "]");
        return;
    }
    const bool a_num = std::holds_alternative<double>(actual.value) ||
                       std::holds_alternative<std::int64_t>(actual.value);
    const bool e_num = std::holds_alternative<double>(expected.value) ||
                       std::holds_alternative<std::int64_t>(expected.value);
    if (std::holds_alternative<double>(expected.value) ||
        std::holds_alternative<double>(actual.value)) {
        bt::require(a_num && e_num, what + ": numeric type mismatch");
        const double a = std::holds_alternative<double>(actual.value)
            ? std::get<double>(actual.value)
            : static_cast<double>(std::get<std::int64_t>(actual.value));
        const double e = std::holds_alternative<double>(expected.value)
            ? std::get<double>(expected.value)
            : static_cast<double>(std::get<std::int64_t>(expected.value));
        if (node != nullptr) {
            const std::array<double, 1> leaf{a};
            node->f64(leaf, std::span<const std::size_t>{}, what);
        } else {
            bt::require(std::bit_cast<std::uint64_t>(a) ==
                            std::bit_cast<std::uint64_t>(e),
                        what + ": float bits differ");
        }
        return;
    }
    bt::require(actual.value.index() == expected.value.index(),
                what + ": type tag differs");
    // Wire has no operator== (recursive type); the non-float leaves are
    // bool/int64/string/monostate scalars here.
    bool equal = false;
    if (const auto *expected_b = std::get_if<bool>(&expected.value))
        equal = *std::get_if<bool>(&actual.value) == *expected_b;
    else if (const auto *expected_i = std::get_if<std::int64_t>(&expected.value))
        equal = *std::get_if<std::int64_t>(&actual.value) == *expected_i;
    else if (const auto *expected_s = std::get_if<std::string>(&expected.value))
        equal = *std::get_if<std::string>(&actual.value) == *expected_s;
    else
        equal = std::holds_alternative<std::monostate>(expected.value);
    bt::require(equal, what + ": values differ");
}

// ---- the staged bootstrap replay (numeric_completion precedent) -----------

std::string input_bytes(std::span<const double> payload) {
    const auto view = std::as_bytes(payload);
    return {reinterpret_cast<const char *>(view.data()), view.size()}; // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast) byte buffer viewed as chars for the I/O API; the sanctioned aliasing bridge
}

// Materialize the recorded bootstrap manifest — config.json, state.json,
// the state.arrays/* lanes and the platform's own serialized model.mjb —
// then replay the oracle's t=0 start. mj_saveModel bytes drift between
// darwin-arm64 and linux-aarch64 (~1e-13 in model fields), which the
// bitwise physics lanes observe, so the model travels as an input entry
// too; the committed fixture image stays the canonical reference.
std::unique_ptr<runtime::NativeRideRuntime> replay_ride(bt_oracle::Node &node,
                                                      const char *label) {
  bt::TemporaryDirectory const stage(std::string("ec-") + label);
  std::string manifest_text =
      input_bytes(node.f64_input(std::string(label) + ".files"));
  while (!manifest_text.empty() && manifest_text.back() == ' ')
    manifest_text.pop_back();
  const Value manifest = bike::io::parse_json(manifest_text);
  for (const Value &item : bike::io::as_array(manifest, "manifest")) {
    const auto &object = bike::io::as_object(item, "manifest");
    const auto *name_member = bike::io::find_member(object, "name");
    const auto *bytes_member = bike::io::find_member(object, "bytes");
    bt::require(name_member != nullptr && bytes_member != nullptr,
                "manifest entries carry name and bytes");
    const std::string name = bike::io::as_string(*name_member, "manifest.name");
    const auto nbytes = static_cast<std::size_t>(bike::io::number_i64(
        bike::io::as_number(*bytes_member, "manifest.bytes"),
        "manifest.bytes"));
    const auto payload = node.f64_input(std::string(label) + ".file::" + name);
    bt::require(payload.size() * 8 >= nbytes,
                "recorded file payload truncated");
    const std::filesystem::path path = stage.path() / name;
    std::filesystem::create_directories(path.parent_path());
    std::ofstream out(path, std::ios::binary);
    out.write(reinterpret_cast<const char *>(payload.data()), // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast) byte buffer viewed as chars for the I/O API; the sanctioned aliasing bridge
              static_cast<std::streamsize>(nbytes));
    bt::require(out.good(), "recorded file materialization failed");
  }
    // RuntimeBootstrap is large; heap it like the period_buffer rig does.
    auto bootstrap = std::make_unique<runtime::RuntimeBootstrap>(
        runtime::read_bootstrap(stage.path()));
    return std::make_unique<runtime::NativeRideRuntime>(std::move(*bootstrap));
}

// The driver: advance(65) with the default RideControl, flush, seal, then
// the model-joint catalog make_render_model feeds build_catalog.
struct Ride {
    std::unique_ptr<runtime::NativeRideRuntime> runtime;
    runtime::OwnedNativeEpisode episode;
    // Heap-owned: Catalog is too large for the 8 KiB frame cap.
    std::unique_ptr<rec::Catalog> catalog;
};

// advance(65) — kept in a helper so the AdvanceResult temporaries do not
// bloat replay_episode's 8 KiB frame budget.
void advance_ride(runtime::NativeRideRuntime &runtime) {
    const runtime::AdvanceResult advanced =
        runtime.advance(65, runtime::RideControl{}, 0., 0., std::nullopt);
    bt::require(advanced.step >= 0, "advance committed");
}

// The catalog build's working set and the returned Ride are heap-owned —
// build_catalog(model) constructs several vectors of JointCoords inside
// the frame otherwise.
std::unique_ptr<rec::Catalog> build_ride_catalog(const mjModel &model) {
    return std::make_unique<rec::Catalog>(rec::build_catalog(model));
}

Ride replay_episode(bt_oracle::Node &node) {
    auto ride = std::make_unique<Ride>();
    ride->runtime = replay_ride(node, "ride");
    advance_ride(*ride->runtime);
    ride->runtime->flush();
    ride->episode = ride->runtime->seal_episode();
    {
        const auto image = ride->runtime->model_image();
        const std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
            mj_loadModelBuffer(image.mjb.data(),
                               static_cast<int>(image.mjb.size())),
            &mj_deleteModel);
        bt::require(model != nullptr, "the committed model image compiles");
        ride->catalog = build_ride_catalog(*model);
    }
    return std::move(*ride);
}

// ---- piece assertions for the non-vacuous literal checks -------------------

const Piece &column(const EncodedTable &table, std::string_view name) {
    const auto found = table.columns.find(std::string(name));
    bt::require(found != table.columns.end(),
                "missing emitted column " + std::string(name));
    return found->second;
}

std::vector<int> bool_lane(const Piece &piece) {
    bt::require(piece.kind == Piece::Kind::scalar_b,
                "expected a boolean scalar lane");
    std::vector<int> out(piece.b.begin(), piece.b.end());
    return out;
}

bool lane_valid(const Piece &piece, std::size_t row) {
    return piece.valid.empty() || piece.valid[row];
}

void expect_bool_column(const EncodedTable &table, std::string_view name,
                        const std::vector<int> &expected) {
  bt::require(bool_lane(column(table, name)) == expected,
              "boolean lane mismatch for " + std::string(name));
}

// pylist() of a map<double> column: per row, nullopt or (key, value) pairs.
void expect_map_d(const Piece &piece,
                  const std::vector<std::optional<
                      std::vector<std::pair<std::string, double>>>> &expected) {
    bt::require(piece.kind == Piece::Kind::map && piece.items != nullptr,
                "expected a map<double> lane");
    bt::require(piece.offsets.size() == expected.size() + 1,
                "map row count differs");
    for (std::size_t row = 0; row < expected.size(); ++row) {
        if (!expected[row].has_value()) {
            bt::require(!lane_valid(piece, row), "map row should be NULL");
            continue;
        }
        bt::require(lane_valid(piece, row), "map row should be valid");
        const auto lo = static_cast<std::size_t>(piece.offsets[row]);
        const auto hi = static_cast<std::size_t>(piece.offsets[row + 1]);
        std::vector<std::pair<std::string, double>> entries;
        for (std::size_t i = lo; i < hi; ++i)
            entries.emplace_back(piece.s[i], piece.items->d[i]);
        bt::require(entries == (expected[row]).value(), "map row contents differ"); // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
    }
}

EncodedTable native_batch(const rec::TableSpec &spec,
                          const runtime::OwnedNativeEpisode &episode,
                          const rec::Catalog &catalog) {
  auto const plan = rec::compile_plan(catalog, spec.name);
  return runtime::encode_table_native(episode, plan, 0, episode_rows(episode));
}

// ---- the ported records, in source-file order -------------------------------

// test_make_episode_builds_retained_samples
void test_make_episode_builds_retained_samples() {
    std::vector<SamplePtr> rows;
    rows.reserve(3);
    for (std::int64_t i = 0; i < 3; ++i)
      rows.push_back(sample_row(i, static_cast<double>(i) * .001));
    const auto episode = make_episode(std::move(rows));
    bt::require(episode_rows(episode) == 3, "row_count == 3");
    bt::require(episode.generation == 1, "generation == 1");
    const auto batch = episode.sample_batch(0, 3);
    std::vector<std::int64_t> ids;
    ids.reserve(batch.samples.size());
    for (const auto &sample : batch.samples)
      ids.push_back(sample->interval_id);
    bt::require(ids == std::vector<std::int64_t>({0, 1, 2}),
                "interval_ids == (0, 1, 2)");
}

// test_make_episode_preserves_authored_value_tree — the retained sample
// keeps the authored channels/forces/powers_w value tree.
void test_make_episode_preserves_authored_value_tree() {
    WireObject channels = {
        {"drive", Wire{WireObject{
             {"watts", Wire{WireArray{Wire{150.5}, Wire{}}}},
             {"ok", Wire{true}}}}},
        {"meta", Wire{WireObject{
             {"note", Wire{"spin-up"}},
             {"tries", w_ints({1, 2})}}}},
        {"empty", Wire{WireObject{}}},
        {"nothing", Wire{}},
    };
    std::vector<SamplePtr> rows{sample_row(0, 0., .001, {}, {},
        {{"crank", {3.5, -1.25}}},
        WireObject{{"drive", Wire{250.5}}}, std::move(channels))};
    const auto episode = make_episode(std::move(rows));
    const auto &sample = *episode.sample_batch(0, 1).samples.front();
    bt::require(sample.forces.size() == 1 &&
                    sample.forces.front().first == "crank" &&
                    sample.forces.front().second ==
                        std::vector<double>({3.5, -1.25}),
                "forces keep the authored component and values");
    bt::require(sample.powers_w.size() == 1 &&
                    sample.powers_w.front().first == "drive" &&
                    std::get<double>(sample.powers_w.front().second.value) ==
                        250.5,
                "powers_w keeps the authored member");
    const auto *drive = sw::find(sample.channels, "drive");
    const auto *meta = sw::find(sample.channels, "meta");
    const auto *empty = sw::find(sample.channels, "empty");
    const auto *nothing = sw::find(sample.channels, "nothing");
    bt::require(drive && meta && empty && nothing,
                "all four authored roots survive");
    const auto &watts = sw::array(sw::required(sw::object(*drive), "watts"));
    bt::require(watts.size() == 2 &&
                    std::get<double>(watts[0].value) == 150.5 &&
                    std::holds_alternative<std::monostate>(watts[1].value),
                "watts keeps [150.5, None]");
    bt::require(sw::boolean(sw::required(sw::object(*drive), "ok")),
                "np.bool_(True) lands as bool");
    bt::require(sw::string(sw::required(sw::object(*meta), "note")) ==
                    "spin-up",
                "note survives");
    const auto &tries = sw::array(sw::required(sw::object(*meta), "tries"));
    bt::require(tries.size() == 2 &&
                    std::get<std::int64_t>(tries[0].value) == 1 &&
                    std::get<std::int64_t>(tries[1].value) == 2,
                "the authored tuple unboxes to an int list");
    bt::require(std::get<WireObject>(empty->value).empty() &&
                    std::holds_alternative<std::monostate>(nothing->value),
                "empty dict and None survive");
}

// test_make_episode_rejects_malformed_channels — dropped: the rejected
// inputs are Python-binding marshalling states (missing 'channels' key,
// non-dict payload). The C++ PhysicalSampleData carries channels as a
// typed WireObject, so the malformed states are unrepresentable; the
// canonical producers always publish a dict. See the port-map record.

// test_column_bits_detects_negative_zero_and_nan — the mirror keeps the
// same distinguishing power the oracle comparator was sanity-checked for.
void test_column_bits_detects_negative_zero_and_nan() {
    const auto make = [](double first) {
        Piece piece;
        piece.kind = Piece::Kind::scalar_d;
        piece.d = {first, std::numeric_limits<double>::quiet_NaN()};
        return piece;
    };
    const rec::FieldSpec field{.key = "x", .dtype = "double", .unit = "n/a",
                               .phase = "interval", .optional = false,
                               .measurement = "test", .source = "test",
                               .description = "x", .frame = "n/a",
                               .path = {}, .aliases = {},
                               .shape = std::nullopt, .axes = std::nullopt,
                               .except_keys = {}, .children = {}};
    bt::require(column_bits(make(0.0), field) != column_bits(make(-0.0), field),
                "column_bits distinguishes -0.0 from +0.0");
    bt::require(column_bits(make(0.0), field) == column_bits(make(0.0), field),
                "column_bits is stable for +0.0 + NaN");
}

// test_physics_and_forces_bitwise_vs_oracle
void test_physics_and_forces_bitwise_vs_oracle() {
    bt_oracle::Node node(node_id("test_physics_and_forces_bitwise_vs_oracle"));
    const auto &cat = support_catalog();
    const std::size_t n = 8;
    std::vector<SamplePtr> rows;
    for (std::int64_t i = 0; i < 6; ++i) {
        std::vector<double> qpos(n), qvel(n);
        // np.linspace(0, 1, n): k * fl(1/(n-1)) with the endpoint pinned —
        // k/(n-1) differs by ulps.
        const double delta = 1.0 / static_cast<double>(n - 1);
        for (std::size_t k = 0; k < n; ++k)
            qpos[k] = (k == n - 1 ? 1.0 : static_cast<double>(k) * delta) +
                      static_cast<double>(i);
        for (auto &v : qvel) v = i % 2 != 0 ? -0.0 : 0.5;
        rows.push_back(sample_row(i, static_cast<double>(i) * .001, .001, qpos, qvel,
            {{"gravity", std::vector<double>(n, static_cast<double>(i))},
             {"motor", std::vector<double>(n, 0.)}},
            WireObject{{"gravity", Wire{static_cast<double>(i) + 0.25}},
                       {"motor", Wire{std::numeric_limits<double>::quiet_NaN()}}},
            energy_channels()));
    }
    const auto episode = make_episode(std::move(rows));
    for (const char *name : {"physics", "forces"})
        assert_identical(node, cat.table(name), episode, cat);
    node.done();
}

// test_compile_plan_contract
void test_compile_plan_contract() {
    const auto &cat = support_catalog();
    const auto physics = rec::compile_plan(cat, "physics");
    bt::require(physics.table == "physics" && physics.kind == "physics" &&
                    !physics.has_columns && physics.leaf_columns.empty() &&
                    physics.masks.empty() && physics.roots.empty() &&
                    physics.all_roots.empty(),
                "physics compiles to the bare table/kind plan");
    const auto forces = rec::compile_plan(cat, "forces");
    bt::require(forces.table == "forces" && forces.kind == "forces" &&
                    !forces.has_columns && forces.leaf_columns.empty(),
                "forces compiles to the bare table/kind plan");
    throws<std::invalid_argument>(
        [] {
            (void)runtime::parse_encode_plan(bike::io::parse_json(
                R"({"table":"channel_energy","kind":"channels","columns":[],"masks":[],"roots":[]})"));
        },
        "all_roots", "channels plans require all_roots");
    (void)rec::compile_plan(cat, "channel_energy");
    for (const auto &spec : cat.table_order())
        (void)rec::compile_plan(cat, spec.name);
}

// test_encode_table_piece_shape_and_domain
void test_encode_table_piece_shape_and_domain() {
    std::vector<SamplePtr> rows{sample_row(0)};
    const auto episode = make_episode(std::move(rows));
    const auto plan = runtime::parse_encode_plan(
        bike::io::parse_json(R"({"table":"physics","kind":"physics"})"));
    const auto table = runtime::encode_table_native(episode, plan, 0, 1);
    bt::require(table.rows == 1, "one encoded row");
    const std::set<std::string> names{
        "interval_id", "time_s", "interval_end_s", "dt_s", "qpos", "qvel"};
    std::set<std::string> emitted;
    for (const auto &[name, piece] : table.columns) emitted.insert(name);
    bt::require(emitted == names, "physics piece set is the fixed six");
    const auto &ids = column(table, "interval_id");
    bt::require(ids.kind == Piece::Kind::scalar_i && ids.valid.empty(),
                "interval_id is an all-valid int64 scalar");
    const auto &qpos = column(table, "qpos");
    bt::require(qpos.kind == Piece::Kind::list_d && qpos.valid.empty() &&
                    !qpos.offsets.empty(),
                "qpos is an all-valid list<double> with offsets");
    const auto suffix = runtime::encode_table_native(episode, plan, 1, 1);
    bt::require(suffix.rows == 0, "start == row_count admits the empty suffix");
}

// test_encode_table_rejects_bad_plans_and_ranges
void test_encode_table_rejects_bad_plans_and_ranges() {
    std::vector<SamplePtr> rows{sample_row(0)};
    const auto episode = make_episode(std::move(rows));
    const auto good = runtime::parse_encode_plan(
        bike::io::parse_json(R"({"table":"physics","kind":"physics"})"));
    for (const char *bad_plan : {
             R"({"kind":"physics"})",
             R"({"table":"physics","kind":"forces"})",
             R"({"table":"forces","kind":"physics"})",
             R"({"table":"physics","kind":"physics","columns":["qpos"]})",
             R"({"table":"physics","kind":"physics","extra":1})"})
        throws<std::invalid_argument>(
            [&] {
                const auto bad = runtime::parse_encode_plan(
                    bike::io::parse_json(bad_plan));
                (void)runtime::encode_table_native(episode, bad, 0, 1);
            },
            "", "malformed encode plan rejects");
    for (const auto &[start, limit] :
         std::vector<std::pair<std::int64_t, std::int64_t>>{
             {-1, 1}, {0, 0}, {2, 1}})
        throws<std::invalid_argument>(
            [&] { (void)runtime::encode_table_native(episode, good, start, limit); },
            "", "out-of-domain slice rejects");
}

// test_forces_missing_power_is_value_error
void test_forces_missing_power_is_value_error() {
    std::vector<SamplePtr> rows{sample_row(
        0, 0., .001, {}, {}, {{"ghost", std::vector<double>(8, 0.)}}, {}, {})};
    const auto episode = make_episode(std::move(rows));
    const auto plan = runtime::parse_encode_plan(
        bike::io::parse_json(R"({"table":"forces","kind":"forces"})"));
    throws<std::invalid_argument>(
        [&] { (void)runtime::encode_table_native(episode, plan, 0, 1); },
        "powers_w", "a forces component without powers_w rejects");
}

// test_flat_channel_tables_bitwise
void test_flat_channel_tables_bitwise() {
    bt_oracle::Node node(node_id("test_flat_channel_tables_bitwise"));
    const auto &cat = support_catalog();
    std::vector<SamplePtr> rows;
    for (std::int64_t i = 0; i < 6; ++i) {
        auto channels = drive_channels(static_cast<double>(i));
        merge_into(channels, sensors_channels(static_cast<double>(i)));
        rows.push_back(sample_row(i, static_cast<double>(i) * .001, .001, {}, {}, {}, {},
                                  std::move(channels)));
    }
    const auto episode = make_episode(std::move(rows));
    for (const char *name : {"channel_drive", "channel_sensors"})
        assert_identical(node, cat.table(name), episode, cat);
    node.done();
}

// test_optional_leaf_absent_vs_null — the golden compare plus the
// non-vacuity literals on the native pieces.
void test_optional_leaf_absent_vs_null() {
    bt_oracle::Node node(node_id("test_optional_leaf_absent_vs_null"));
    const auto &cat = support_catalog();
    const auto &spec = cat.table("channel_drive");
    auto absent = drive_channels();
    auto &absent_drive = std::get<WireObject>(
        const_cast<Wire &>(*sw::find(absent, "drive")).value); // NOLINT(cppcoreguidelines-pro-type-const-cast) mutable write through the const lookup view mirrors the oracle's in-place mutation
    absent_drive.erase(std::ranges::find_if(absent_drive, [](const auto &kv) {
        return kv.first == "coasting_reason";
    }));
    auto nullv = drive_channels(0., {{"coasting_reason", Wire{}}});
    std::vector<SamplePtr> rows;
    rows.push_back(sample_row(0, 0., .001, {}, {}, {}, {}, std::move(absent)));
    rows.push_back(sample_row(1, 0., .001, {}, {}, {}, {}, std::move(nullv)));
    const auto episode = make_episode(std::move(rows));
    assert_identical(node, spec, episode, cat);
    // The distinction must be real in the native output: absent encodes
    // _present=False; the explicit null encodes _present=True + NULL.
    const auto table = native_batch(spec, episode, cat);
    expect_bool_column(table, "coasting_reason_present", {0, 1});
    const auto &reason = column(table, "coasting_reason");
    bt::require(reason.valid.size() == 2 && !reason.valid[0] &&
                    !reason.valid[1],
                "both coasting_reason slots are NULL");
    node.done();
}

// test_parent_mask_absent_vs_null
void test_parent_mask_absent_vs_null() {
    bt_oracle::Node node(node_id("test_parent_mask_absent_vs_null"));
    const auto &cat = support_catalog();
    const auto &spec = cat.table("channel_sensors");
    auto absent = sensors_channels();
    auto &absent_sensors = std::get<WireObject>(
        const_cast<Wire &>(*sw::find(absent, "sensors")).value); // NOLINT(cppcoreguidelines-pro-type-const-cast) mutable write through the const lookup view mirrors the oracle's in-place mutation
    absent_sensors.erase(std::ranges::find_if(
        absent_sensors, [](const auto &kv) {
            return kv.first == "encoders_rad_s";
        }));
    auto nullv = sensors_channels(0., {{"encoders_rad_s", Wire{}}});
    std::vector<SamplePtr> rows;
    rows.push_back(sample_row(0, 0., .001, {}, {}, {}, {}, std::move(absent)));
    rows.push_back(sample_row(1, 0., .001, {}, {}, {}, {}, std::move(nullv)));
    const auto episode = make_episode(std::move(rows));
    assert_identical(node, spec, episode, cat);
    const auto table = native_batch(spec, episode, cat);
    expect_bool_column(table, "__sensors.encoders_rad_s_present", {0, 1});
    expect_bool_column(table, "__sensors.encoders_rad_s_valid", {0, 0});
    node.done();
}

// test_required_leaf_missing_vs_null_rejects
void test_required_leaf_missing_vs_null_rejects() {
    const auto &cat = support_catalog();
    const auto plan = rec::compile_plan(cat, "channel_sensors");
    const std::vector<WireObject> variants = {
        {{"sensors", Wire{WireObject{}}}},
        {{"sensors", Wire{WireObject{{"motor_torque_nm", Wire{}}}}}},
        {{"sensors",
          Wire{WireObject{{"encoders_rad_s", Wire{WireObject{}}}}}}},
    };
    for (const WireObject &channels : variants) {
        std::vector<SamplePtr> rows{
            sample_row(0, 0., .001, {}, {}, {}, {}, channels)};
        const auto episode = make_episode(std::move(rows));
        throws<std::invalid_argument>(
            [&] { (void)runtime::encode_table_native(episode, plan, 0, 1); },
            "required", "missing/null required leaf rejects");
    }
}

// test_leaf_type_and_shape_gates
void test_leaf_type_and_shape_gates() {
    const auto &cat = support_catalog();
    const auto plan = rec::compile_plan(cat, "channel_sensors");
    {
        std::vector<SamplePtr> rows{sample_row(0, 0., .001, {}, {}, {}, {},
            sensors_channels(0., {{"frame_specific_force_body_mps2",
                                  w_arr({1.0, 2.0})}}))};
        const auto episode = make_episode(std::move(rows));
        throws<std::invalid_argument>(
            [&] { (void)runtime::encode_table_native(episode, plan, 0, 1); },
            "3 elements", "a short double[] leaf rejects by shape");
    }
    {
        std::vector<SamplePtr> rows{sample_row(0, 0., .001, {}, {}, {}, {},
            sensors_channels(0., {{"motor_torque_nm", Wire{"not-a-number"}}}))};
        const auto episode = make_episode(std::move(rows));
        throws<std::invalid_argument>(
            [&] { (void)runtime::encode_table_native(episode, plan, 0, 1); },
            "real number", "a nonnumeric scalar leaf rejects by type");
    }
}

// test_unknown_channel_root_rejected
void test_unknown_channel_root_rejected() {
    const auto &cat = support_catalog();
    std::vector<SamplePtr> rows{sample_row(0, 0., .001, {}, {}, {}, {},
        WireObject{{"forged_root", Wire{WireObject{}}}})};
    const auto episode = make_episode(std::move(rows));
    const auto plan = rec::compile_plan(cat, "channel_drive");
    throws<std::invalid_argument>(
        [&] { (void)runtime::encode_table_native(episode, plan, 0, 1); },
        "forged_root", "an unknown channel root rejects");
}

// test_unknown_channel_member_rejected
void test_unknown_channel_member_rejected() {
    const auto &cat = support_catalog();
    auto channels = drive_channels();
    std::get<WireObject>(const_cast<Wire &>(*sw::find(channels, "drive")).value) // NOLINT(cppcoreguidelines-pro-type-const-cast) mutable write through the const lookup view mirrors the oracle's in-place mutation
        .emplace_back("bogus_member", Wire{std::int64_t{1}});
    std::vector<SamplePtr> rows{
        sample_row(0, 0., .001, {}, {}, {}, {}, std::move(channels))};
    const auto episode = make_episode(std::move(rows));
    const auto plan = rec::compile_plan(cat, "channel_drive");
    throws<std::invalid_argument>(
        [&] { (void)runtime::encode_table_native(episode, plan, 0, 1); },
        "bogus_member", "an unknown channel member rejects");
}

// test_map_channel_tables_bitwise — golden compare plus the native-side
// empty-vs-absent literals.
void test_map_channel_tables_bitwise() {
    bt_oracle::Node node(node_id("test_map_channel_tables_bitwise"));
    const auto &cat = support_catalog();
    std::vector<SamplePtr> rows;
    {
        auto channels = energy_channels();
        merge_into(channels, suspension_channels());
        rows.push_back(sample_row(0, 0., .001, {}, {}, {}, {},
                                  std::move(channels)));
    }
    {
        auto channels = energy_channels(
            {{"elastic_energy_j", Wire{WireObject{}}}},
            {{"component_work_j", Wire{WireObject{}}}});
        merge_into(channels, suspension_channels(
            {{"generalized_force_components_n", Wire{WireObject{}}}}));
        rows.push_back(sample_row(1, .001, .001, {}, {}, {}, {},
                                  std::move(channels)));
    }
    rows.push_back(sample_row(2, .002, .001, {}, {}, {}, {}, WireObject{}));
    const auto episode = make_episode(std::move(rows));
    for (const char *name : {"channel_energy", "channel_suspension"})
        assert_identical(node, cat.table(name), episode, cat);
    const auto table =
        native_batch(cat.table("channel_energy"), episode, cat);
    expect_map_d(column(table, "elastic_energy_j"),
                 {std::vector<std::pair<std::string, double>>{{"fork_spring", 0.25}},
                  std::vector<std::pair<std::string, double>>{},
                  std::nullopt});
    expect_bool_column(table, "elastic_energy_j_present", {1, 1, 0});
    expect_map_d(column(table, "component_work_j"),
                 {std::vector<std::pair<std::string, double>>{
                      {"chain", 0.1}, {"tires", -0.2}},
                  std::nullopt, std::nullopt});
    expect_bool_column(table, "component_work_j_present", {1, 0, 0});
    const auto suspension =
        native_batch(cat.table("channel_suspension"), episode, cat);
    expect_map_d(column(suspension, "generalized_force_components_n"),
                 {std::vector<std::pair<std::string, double>>{
                      {"fork_slide", 12.5}, {"shock_slide", -3.0}},
                  std::vector<std::pair<std::string, double>>{},
                  std::nullopt});
    node.done();
}

// test_map_leaf_rejects_non_dict_and_bad_member — the native half of the
// dual-path parity: every bad member must reject from encode_table too.
void test_map_leaf_rejects_non_dict_and_bad_member() {
    const auto &cat = support_catalog();
    const auto plan = rec::compile_plan(cat, "channel_suspension");
    const Wire members[] = { // NOLINT(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) bounded label buffer for snprintf
        Wire{"a-string"},
        Wire{WireObject{{"fork_slide", Wire{"text"}}}},
        Wire{WireObject{{"fork_slide", w_arr({1.0})}}},
        Wire{std::int64_t{5}},
    };
    for (const auto &member : members) {
        std::vector<SamplePtr> rows{sample_row(0, 0., .001, {}, {}, {}, {},
            suspension_channels({{"generalized_force_components_n", member}}))};
        const auto episode = make_episode(std::move(rows));
        throws<std::invalid_argument>(
            [&] { (void)runtime::encode_table_native(episode, plan, 0, 1); },
            "", "a bad map member rejects");
    }
}

// test_map_required_leaf_missing_rejects
void test_map_required_leaf_missing_rejects() {
    const auto &cat = support_catalog();
    auto channels = suspension_channels();
    auto &suspension = std::get<WireObject>(
        const_cast<Wire &>(*sw::find(channels, "suspension")).value); // NOLINT(cppcoreguidelines-pro-type-const-cast) mutable write through the const lookup view mirrors the oracle's in-place mutation
    suspension.erase(std::ranges::find_if(suspension, [](const auto &kv) {
        return kv.first == "generalized_force_components_n";
    }));
    std::vector<SamplePtr> rows{
        sample_row(0, 0., .001, {}, {}, {}, {}, std::move(channels))};
    const auto episode = make_episode(std::move(rows));
    const auto plan = rec::compile_plan(cat, "channel_suspension");
    throws<std::invalid_argument>(
        [&] { (void)runtime::encode_table_native(episode, plan, 0, 1); },
        "required", "a missing required map<double> leaf rejects");
}

// Mini-catalog helpers — TableSpec/FieldSpec authoring mirrors the Python
// tests' inline leaf() constructors exactly.
rec::FieldSpec leaf(const std::string &key, std::string dtype,
                    std::vector<std::string> path, bool optional = false,
                    std::vector<std::string> except_keys = {}) {
  rec::FieldSpec field{.key = key,
                       .dtype = std::move(dtype),
                       .unit = "n/a",
                       .phase = "interval",
                       .optional = optional,
                       .measurement = "diagnostic",
                       .source = "test",
                       .description = key + " leaf",
                       .frame = "n/a",
                       .path = std::move(path),
                       .aliases = {},
                       .shape = std::nullopt,
                       .axes = std::nullopt,
                       .except_keys = std::move(except_keys),
                       .children = {}};
  return field;
}
rec::FieldSpec key_field(std::string key) {
    rec::FieldSpec field{.key = std::move(key), .dtype = "bigint",
                         .unit = "n/a", .phase = "n/a", .optional = false,
                         .measurement = "declared", .source = "test",
                         .description = "test identity key", .frame = "n/a",
                         .path = {}, .aliases = {}, .shape = std::nullopt,
                         .axes = std::nullopt, .except_keys = {},
                         .children = {}};
    return field;
}
rec::Catalog mini_catalog(rec::TableSpec spec) {
    return rec::Catalog({std::move(spec)}, synthetic_coordinates(), {});
}

// test_map_except_keys_routes_owned_member
void test_map_except_keys_routes_owned_member() {
    bt_oracle::Node node(node_id("test_map_except_keys_routes_owned_member"));
    rec::TableSpec spec{
        .name = "channel_owned_map", .kind = "channels",
        .keys = {key_field("episode_id"), key_field("interval_id")},
        .fields = {leaf("bag", "map<double>", {"bag"}, true, {"owned"}),
                   leaf("owned_value", "double", {"bag", "owned"}, true)},
        .description = "map remainder routing", .source = "test",
        .claims = {"bag"}, .exempt_keys = {}};
    auto const mini = mini_catalog(std::move(spec));
    std::vector<SamplePtr> rows;
    rows.push_back(sample_row(0, 0., .001, {}, {}, {}, {},
        WireObject{{"bag", Wire{WireObject{{"owned", Wire{3.5}},
            {"free_a", Wire{1.5}}, {"free_b", Wire{-0.25}}}}}}));
    rows.push_back(sample_row(1, 0., .001, {}, {}, {}, {},
        WireObject{{"bag", Wire{WireObject{{"owned", Wire{std::int64_t{1}}}}}}}));
    rows.push_back(sample_row(2, 0., .001, {}, {}, {}, {},
        WireObject{{"bag", Wire{WireObject{}}}}));
    const auto episode = make_episode(std::move(rows));
    assert_identical(node, mini.table("channel_owned_map"), episode, mini);
    const auto table = native_batch(mini.table("channel_owned_map"), episode, mini);
    expect_map_d(column(table, "bag"),
                 {std::vector<std::pair<std::string, double>>{
                      {"free_a", 1.5}, {"free_b", -0.25}},
                  std::nullopt, std::nullopt});
    expect_bool_column(table, "bag_present", {1, 0, 0});
    const auto &owned = column(table, "owned_value");
    bt::require(owned.kind == Piece::Kind::scalar_d &&
                    owned.valid.size() == 3 && owned.valid[0] &&
                    owned.valid[1] && !owned.valid[2] &&
                    owned.d[0] == 3.5 && owned.d[1] == 1.0,
                "owned_value lanes match [3.5, 1.0, None]");
    expect_bool_column(table, "owned_value_present", {1, 1, 0});
    node.done();
}

// test_all_list_and_map_dtypes_bitwise
void test_all_list_and_map_dtypes_bitwise() {
    bt_oracle::Node node(node_id("test_all_list_and_map_dtypes_bitwise"));
    rec::TableSpec spec{
        .name = "channel_dtypes", .kind = "channels",
        .keys = {key_field("episode_id"), key_field("interval_id")},
        .fields = {leaf("ints", "bigint[]", {"ints"}),
                   leaf("tags", "varchar[]", {"tags"}),
                   leaf("map_d", "map<double>", {"map_d"}, true),
                   leaf("map_b", "map<boolean>", {"map_b"}, true),
                   leaf("map_s", "map<varchar>", {"map_s"}, true),
                   leaf("map_v", "map<double[]>", {"map_v"}, true)},
        .description = "list/map dtype matrix", .source = "test",
        .claims = {"ints", "tags", "map_d", "map_b", "map_s", "map_v"},
        .exempt_keys = {}};
    auto const mini = mini_catalog(std::move(spec));
    std::vector<SamplePtr> rows;
    rows.push_back(sample_row(0, 0., .001, {}, {}, {}, {}, WireObject{
        {"ints", w_ints({1, 2, -3})}, {"tags", w_strs({"a", "b"})},
        {"map_d", Wire{WireObject{{"x", Wire{1.5}}, {"y", Wire{-0.0}}}}},
        {"map_b", Wire{WireObject{{"on", Wire{true}}}}},
        {"map_s", Wire{WireObject{{"k", Wire{"v"}}}}},
        {"map_v", Wire{WireObject{{"p", w_arr({1.0, 2.0})},
                                  {"q", Wire{WireArray{}}}}}}}));
    rows.push_back(sample_row(1, 0., .001, {}, {}, {}, {}, WireObject{
        {"ints", Wire{WireArray{}}}, {"tags", Wire{"hé"}},
        {"map_d", Wire{WireObject{}}}, {"map_b", Wire{WireObject{}}},
        {"map_s", Wire{WireObject{}}}, {"map_v", Wire{WireObject{}}}}));
    rows.push_back(sample_row(2, 0., .001, {}, {}, {}, {}, WireObject{}));
    const auto episode = make_episode(std::move(rows));
    assert_identical(node, mini.table("channel_dtypes"), episode, mini);
    const auto table = native_batch(mini.table("channel_dtypes"), episode, mini);
    {
        const auto &ints = column(table, "ints");
        bt::require(ints.kind == Piece::Kind::list_i &&
                        ints.valid.size() == 3 && ints.valid[0] &&
                        ints.valid[1] && !ints.valid[2] &&
                        ints.offsets == std::vector<std::int32_t>({0, 3, 3, 3}) &&
                        ints.i == std::vector<std::int64_t>({1, 2, -3}),
                    "ints == [[1, 2, -3], [], None]");
    }
    {
        const auto &tags = column(table, "tags");
        bt::require(tags.kind == Piece::Kind::list_s &&
                        tags.valid.size() == 3 && tags.valid[0] &&
                        tags.valid[1] && !tags.valid[2] &&
                        tags.offsets == std::vector<std::int32_t>({0, 2, 4, 4}) &&
                        tags.s == std::vector<std::string>({"a", "b", "h", "\xc3\xa9"}),
                    "tags explodes the str payload to code points");
    }
    expect_map_d(column(table, "map_d"),
                 {std::vector<std::pair<std::string, double>>{{"x", 1.5}, {"y", -0.0}},
                  std::nullopt, std::nullopt});
    {
        const auto &map_b = column(table, "map_b");
        bt::require(map_b.valid[0] && !map_b.valid[1] && !map_b.valid[2] &&
                        map_b.s == std::vector<std::string>({"on"}) &&
                        map_b.items->b == std::vector<char>({'\x01'}),
                    "map_b == [[('on', True)], None, None]");
    }
    {
        const auto &map_s = column(table, "map_s");
        bt::require(map_s.valid[0] && !map_s.valid[1] && !map_s.valid[2] &&
                        map_s.s == std::vector<std::string>({"k"}) &&
                        map_s.items->s == std::vector<std::string>({"v"}),
                    "map_s == [[('k', 'v')], None, None]");
    }
    {
        const auto &map_v = column(table, "map_v");
        bt::require(map_v.valid[0] && !map_v.valid[1] && !map_v.valid[2] &&
                        map_v.offsets == std::vector<std::int32_t>({0, 2, 2, 2}) &&
                        map_v.s == std::vector<std::string>({"p", "q"}) &&
                        map_v.items->offsets ==
                            std::vector<std::int32_t>({0, 2, 2}) &&
                        map_v.items->d == std::vector<double>({1.0, 2.0}),
                    "map_v == [[('p', [1, 2]), ('q', [])], None, None]");
    }
    node.done();
}

// test_struct_field_bitwise_channel_mass
void test_struct_field_bitwise_channel_mass() {
    bt_oracle::Node node(node_id("test_struct_field_bitwise_channel_mass"));
    const auto &cat = support_catalog();
    const auto &spec = cat.table("channel_mass");
    std::vector<SamplePtr> rows;
    rows.reserve(3);
    for (std::int64_t i = 0; i < 3; ++i)
      rows.push_back(sample_row(i, static_cast<double>(i) * .001, .001, {}, {},
                                {}, {}, mass_channels(i % 2 == 0)));
    const auto episode = make_episode(std::move(rows));
    assert_identical(node, spec, episode, cat);
    const auto table = native_batch(spec, episode, cat);
    expect_bool_column(table, "endpoint_mass_present", {1, 0, 1});
    const auto &endpoint = column(table, "endpoint_mass");
    bt::require(endpoint.kind == Piece::Kind::structure &&
                    endpoint.valid.size() == 3 && endpoint.valid[0] &&
                    !endpoint.valid[1] && endpoint.valid[2],
                "endpoint_mass is an optional struct [T, F, T]");
    const auto &first = endpoint.children;
    bt::require(first.at("mass_kg").d[0] == 95.1,
                "endpoint row 0 mass_kg == 95.1");
    const auto &momentum = first.at("angular_momentum_kg_m2_s");
    bt::require(!momentum.valid.empty() && !momentum.valid[0],
                "absent optional member encodes NULL inside the struct");
    const auto &momentum_present = first.at("angular_momentum_kg_m2_s_present");
    bt::require(momentum_present.b[0] == 0,
                "its _present companion is False");
    const auto &kinetic_present = first.at("kinetic_energy_j_present");
    bt::require(kinetic_present.b[0] == 1,
                "present member's companion is True");
    node.done();
}

// test_struct_field_optional_member_channel_control
void test_struct_field_optional_member_channel_control() {
    bt_oracle::Node node(node_id("test_struct_field_optional_member_channel_control"));
    const auto &cat = support_catalog();
    const auto &spec = cat.table("channel_control");
    std::vector<SamplePtr> rows;
    rows.push_back(sample_row(0, 0., .001, {}, {}, {}, {},
                              control_channels(true, false)));
    rows.push_back(sample_row(1, 0., .001, {}, {}, {}, {},
                              control_channels(true, true)));
    rows.push_back(sample_row(2, 0., .001, {}, {}, {}, {},
                              control_channels(false, false)));
    const auto episode = make_episode(std::move(rows));
    assert_identical(node, spec, episode, cat);
    const auto table = native_batch(spec, episode, cat);
    expect_bool_column(table, "posture_present", {1, 1, 0});
    const auto &posture = column(table, "posture");
    bt::require(posture.valid.size() == 3 && posture.valid[0] &&
                    posture.valid[1] && !posture.valid[2],
                "posture is [present, present, absent]");
    const auto &offset = posture.children.at("pelvis_offset_m");
    bt::require(!offset.valid.empty() && !offset.valid[0] &&
                    offset.valid[1],
                "pelvis_offset_m is [NULL, present]");
    const auto &offset_present = posture.children.at("pelvis_offset_m_present");
    bt::require(offset_present.b == std::vector<char>({0, 1, 0}),
                "pelvis_offset_m_present == [F, T, absent-row]");
    bt::require(offset.offsets ==
                    std::vector<std::int32_t>({0, 0, 2, 2}),
                "the present row carries [0.02, 0.01]");
    bt::require(offset.d == std::vector<double>({0.02, 0.01}),
                "pelvis_offset_m payload is the authored pair");
    node.done();
}

// test_struct_nested_and_verdict_channel_model_status
void test_struct_nested_and_verdict_channel_model_status() {
    bt_oracle::Node node(node_id("test_struct_nested_and_verdict_channel_model_status"));
    const auto &cat = support_catalog();
    const auto &spec = cat.table("channel_model_status");
    std::vector<SamplePtr> rows;
    rows.push_back(sample_row(0, 0., .001, {}, {}, {}, {},
                              model_status_channels(Wire{true})));
    rows.push_back(sample_row(1, 0., .001, {}, {}, {}, {}, [&] {
        auto channels = model_status_channels(Wire{false}, true);
        channels.emplace_back("contact_crash_cause", Wire{"tire_blowout"});
        channels.emplace_back("attachment_violations",
                              w_strs({"left pedal"}));
        return channels;
    }()));
    rows.push_back(sample_row(2, 0., .001, {}, {}, {}, {},
                              model_status_channels(Wire{"not_evaluated"})));
    const auto episode = make_episode(std::move(rows));
    assert_identical(node, spec, episode, cat);
    const auto table = native_batch(spec, episode, cat);
    const auto &status = column(table, "model_status");
    bt::require(status.kind == Piece::Kind::structure,
                "model_status is a struct column");
    const auto &verdict = status.children.at("numerically_valid");
    bt::require(verdict.s ==
                    std::vector<std::string>({"true", "false", "not_evaluated"}),
                "the verdict leaf publishes verdict text");
    const auto &violation = status.children.at("first_model_violation");
    bt::require(!violation.valid.empty() && !violation.valid[0] &&
                    violation.valid[1],
                "first_model_violation is [NULL, present, ...]");
    const auto &violation_present =
        status.children.at("first_model_violation_present");
    bt::require(violation_present.b == std::vector<char>({0, 1, 0}),
                "first_model_violation_present == [F, T, F]");
    const auto &v_id = violation.children.at("interval_id");
    const auto &v_reasons = violation.children.at("reasons");
    bt::require(v_id.i[1] == 4 && v_reasons.s ==
                    std::vector<std::string>({"energy_closure"}),
                "the nested struct keeps interval_id 4 and the reasons list");
    const auto &crash = column(table, "contact_crash_cause");
    bt::require(crash.valid.size() == 3 && !crash.valid[0] &&
                    crash.valid[1] && !crash.valid[2] &&
                    crash.s[1] == "tire_blowout",
                "contact_crash_cause == [None, 'tire_blowout', None]");
    const auto &violations = column(table, "attachment_violations");
    bt::require(violations.valid.size() == 3 && !violations.valid[0] &&
                    violations.valid[1] && !violations.valid[2] &&
                    violations.s == std::vector<std::string>({"left pedal"}),
                "attachment_violations == [None, ['left pedal'], None]");
    node.done();
}

// test_struct_list_patches_channel_tires
void test_struct_list_patches_channel_tires() {
    bt_oracle::Node node(node_id("test_struct_list_patches_channel_tires"));
    const auto &cat = support_catalog();
    const auto &spec = cat.table("channel_tires");
    std::vector<SamplePtr> rows;
    rows.push_back(sample_row(0, 0., .001, {}, {}, {}, {},
        tires_channels({tire_patch(160.0), tire_patch(95.0, false)}, {})));
    rows.push_back(sample_row(1, 0., .001, {}, {}, {}, {},
        tires_channels({tire_patch(300.0)}, {tire_patch(180.0)})));
    rows.push_back(sample_row(2, 0., .001, {}, {}, {}, {}, WireObject{}));
    const auto episode = make_episode(std::move(rows));
    assert_identical(node, spec, episode, cat);
    const auto table = native_batch(spec, episode, cat);
    const auto &front = column(table, "front");
    const auto &front_patches = front.children.at("patches");
    bt::require(front_patches.offsets ==
                    std::vector<std::int32_t>({0, 2, 3, 3}),
                "front patches keep the authored element spans");
    const auto &couple = front_patches.children.at("couple_world_nm");
    const auto &couple_present =
        front_patches.children.at("couple_world_nm_present");
    bt::require(couple.valid.size() == 3 && couple.valid[0] &&
                    !couple.valid[1] && couple.valid[2],
                "patch 1 omits couple_world_nm (NULL member)");
    bt::require(couple_present.b == std::vector<char>({1, 0, 1}),
                "couple_world_nm_present == [T, F, T]");
    bt::require(couple.d[0] == 0.0 && couple.d[1] == 0.05 &&
                    couple.d[2] == 0.0,
                "patch 0 carries [0.0, 0.05, 0.0]");
    const auto &rear = column(table, "rear");
    const auto &rear_patches = rear.children.at("patches");
    bt::require(rear_patches.offsets[0] == 0 && rear_patches.offsets[1] == 0,
                "rear row 0 has zero patches");
    bt::require(front.valid.size() == 3 && rear.valid.size() == 3 &&
                    front.valid[0] && rear.valid[0] && !front.valid[2] &&
                    !rear.valid[2],
                "the absent tires root NULLs both side structs");
    node.done();
}

// test_map_struct_except_keys_and_nesting_channel_rider
void test_map_struct_except_keys_and_nesting_channel_rider() {
    bt_oracle::Node node(node_id("test_map_struct_except_keys_and_nesting_channel_rider"));
    const auto &cat = support_catalog();
    const auto &spec = cat.table("channel_rider");
    std::vector<SamplePtr> rows;
    rows.push_back(sample_row(0, 0., .001, {}, {}, {}, {}, rider_channels()));
    {
        auto channels = rider_channels();
        // welds_owned_only replaces the whole member — only the owned
        // crank_torque_nm payload remains, so the map is all-owned.
        *const_cast<Wire *>(sw::find(channels, "rider_welds")) = // NOLINT(cppcoreguidelines-pro-type-const-cast) mutable write through the const lookup view mirrors the oracle's in-place mutation
            Wire{WireObject{{"crank_torque_nm", Wire{4.0}}}};
        rows.push_back(sample_row(1, 0., .001, {}, {}, {}, {},
                                  std::move(channels)));
    }
    rows.push_back(sample_row(2, 0., .001, {}, {}, {}, {}, WireObject{}));
    const auto episode = make_episode(std::move(rows));
    assert_identical(node, spec, episode, cat);
    const auto table = native_batch(spec, episode, cat);
    const auto &weld_torque = column(table, "weld_crank_torque_nm");
    bt::require(weld_torque.valid[0] && weld_torque.valid[1] &&
                    !weld_torque.valid[2] && weld_torque.d[0] == 6.25 &&
                    weld_torque.d[1] == 4.0,
                "weld_crank_torque_nm == [6.25, 4.0, None]");
    expect_bool_column(table, "weld_crank_torque_nm_present", {1, 1, 0});
    const auto &welds = column(table, "rider_welds");
    bt::require(welds.valid[0] && !welds.valid[1] && !welds.valid[2],
                "an all-owned remainder encodes the map absent");
    {
        const auto hi = static_cast<std::ptrdiff_t>(welds.offsets[1]);
        std::vector<std::string> keys(welds.s.begin(), welds.s.begin() + hi);
        bt::require(std::ranges::find(keys, "crank_torque_nm") == keys.end() &&
                        std::ranges::find(keys, "pedal_left") != keys.end(),
                    "the owned member never becomes a map entry");
        const auto pedal = std::distance(welds.s.begin(),
            std::ranges::find(welds.s, "pedal_left"));
        bt::require(pedal >= 0 && welds.items->children.at("normal_n").d[static_cast<std::size_t>(pedal)] == 45.0,
                    "pedal_left keeps normal_n == 45.0");
    }
    expect_bool_column(table, "rider_welds_present", {1, 0, 0});
    const auto &rider = column(table, "rider");
    {
        const auto hi = static_cast<std::ptrdiff_t>(rider.offsets[1]);
        std::set<std::string> const keys(rider.s.begin(), rider.s.begin() + hi);
        bt::require(keys == std::set<std::string>({"saddle", "bar"}),
                    "rider row 0 entries are saddle and bar");
        const auto saddle = static_cast<std::size_t>(std::distance(
            rider.s.begin(), std::ranges::find(rider.s, "saddle")));
        const auto bar = static_cast<std::size_t>(std::distance(
            rider.s.begin(), std::ranges::find(rider.s, "bar")));
        const auto &members = rider.items->children;
        bt::require(members.at("normal_load_n").d[saddle] == 310.0,
                    "saddle.normal_load_n == 310.0");
        const auto &s_patches = members.at("patches");
        bt::require(s_patches.offsets[saddle + 1] -
                        s_patches.offsets[saddle] == 1,
                    "saddle keeps one patch");
        const auto &force = members.at("force_on_rider_n");
        bt::require(!force.valid.empty() && !force.valid[saddle],
                    "saddle.force_on_rider_n is absent");
        bt::require(members.at("force_on_rider_n_present").b[saddle] == 0,
                    "its companion is False");
        bt::require(members.at("hand_gap_m").d[bar] == 0.005,
                    "bar.hand_gap_m == 0.005");
        bt::require(!members.at("normal_load_n").valid.empty() &&
                        !members.at("normal_load_n").valid[bar],
                    "bar.normal_load_n is NULL");
        bt::require(!members.at("patches").valid.empty() &&
                        !members.at("patches").valid[bar],
                    "bar.patches is NULL");
    }
    bt::require(!rider.valid.empty() && !rider.valid[2],
                "rider row 2 is NULL");
    const auto &targets = column(table, "rider_support_targets");
    {
        const auto &feet = targets.children.at("feet");
        const auto left = static_cast<std::size_t>(std::distance(
            feet.s.begin(), std::ranges::find(feet.s, "left")));
        bt::require(left < feet.s.size() && feet.offsets[1] == 1,
                    "feet keeps the 'left' entry");
        const auto &goal = feet.items->children.at("goal_geometry");
        bt::require(goal.children.at("weld").b[left] == 1 &&
                        goal.children.at("saturated").b[left] == 0,
                    "left.goal_geometry == {weld: True, saturated: False}");
        bt::require(feet.items->children.at("ik_saturated").b[left] == 0,
                    "left.ik_saturated is False");
        const auto &sole = feet.items->children.at("actual_sole_position_m");
        bt::require(!sole.valid.empty() && !sole.valid[left],
                    "left.actual_sole_position_m is absent");
        const auto &forces = targets.children.at("requested_vertical_forces_n");
        bt::require(forces.offsets[0] == 0 && forces.offsets[1] == 1 &&
                        forces.s[0] == "saddle" && forces.items->d[0] == 300.0,
                    "requested_vertical_forces_n == [('saddle', 300.0)]");
    }
    const auto &control = column(table, "rider_control");
    bt::require(control.valid.size() == 3 &&
                    std::ranges::all_of(control.valid,
                                        [](char v) { return v == 0; }),
                "rider_control encodes all-NULL");
    node.done();
}

// test_struct_member_validation_parity — native side of the dual-path
// rejections.
void test_struct_member_validation_parity() {
    const auto &cat = support_catalog();
    const auto plan = rec::compile_plan(cat, "channel_mass");
    {
        auto channels = mass_channels();
        std::get<WireObject>(const_cast<Wire &>( // NOLINT(cppcoreguidelines-pro-type-const-cast) mutable write through the const lookup view mirrors the oracle's in-place mutation
            *sw::find(channels, "endpoint_mass")).value)
            .emplace_back("bogus", Wire{std::int64_t{1}});
        std::vector<SamplePtr> rows{
            sample_row(0, 0., .001, {}, {}, {}, {}, std::move(channels))};
        const auto episode = make_episode(std::move(rows));
        throws<std::invalid_argument>(
            [&] { (void)runtime::encode_table_native(episode, plan, 0, 1); },
            "bogus", "an unknown struct member rejects");
    }
    {
        auto channels = mass_channels();
        auto &endpoint = std::get<WireObject>(const_cast<Wire &>( // NOLINT(cppcoreguidelines-pro-type-const-cast) mutable write through the const lookup view mirrors the oracle's in-place mutation
            *sw::find(channels, "endpoint_mass")).value);
        endpoint.erase(std::ranges::find_if(endpoint, [](const auto &kv) {
            return kv.first == "mass_kg";
        }));
        std::vector<SamplePtr> rows{
            sample_row(0, 0., .001, {}, {}, {}, {}, std::move(channels))};
        const auto episode = make_episode(std::move(rows));
        throws<std::invalid_argument>(
            [&] { (void)runtime::encode_table_native(episode, plan, 0, 1); },
            "required", "a missing required struct member rejects");
    }
    {
        auto channels = mass_channels();
        *const_cast<Wire *>(sw::find(channels, "endpoint_mass")) = // NOLINT(cppcoreguidelines-pro-type-const-cast) mutable write through the const lookup view mirrors the oracle's in-place mutation
            w_arr({1.0, 2.0});
        std::vector<SamplePtr> rows{
            sample_row(0, 0., .001, {}, {}, {}, {}, std::move(channels))};
        const auto episode = make_episode(std::move(rows));
        throws<std::invalid_argument>(
            [&] { (void)runtime::encode_table_native(episode, plan, 0, 1); },
            "", "a non-dict struct payload rejects");
    }
}

// test_map_struct_bad_key_and_member_parity — native side.
void test_map_struct_bad_key_and_member_parity() {
    const auto &cat = support_catalog();
    const auto plan = rec::compile_plan(cat, "channel_rider");
    {
        auto channels = rider_channels();
        *const_cast<Wire *>(sw::find(channels, "rider_welds")) = // NOLINT(cppcoreguidelines-pro-type-const-cast) mutable write through the const lookup view mirrors the oracle's in-place mutation
            Wire{WireObject{{"", Wire{WireObject{
                {"force_on_rider_n", w_arr({0.0, 0.0, -1.0})}}}}}};
        std::vector<SamplePtr> rows{
            sample_row(0, 0., .001, {}, {}, {}, {}, std::move(channels))};
        const auto episode = make_episode(std::move(rows));
        throws<std::invalid_argument>(
            [&] { (void)runtime::encode_table_native(episode, plan, 0, 1); },
            "nonempty", "an empty map key rejects");
    }
    {
        auto channels = rider_channels();
        auto const &welds = std::get<WireObject>(
            const_cast<Wire &>(*sw::find(channels, "rider_welds")).value); // NOLINT(cppcoreguidelines-pro-type-const-cast) mutable write through the const lookup view mirrors the oracle's in-place mutation
        auto &pedal = std::get<WireObject>(const_cast<Wire &>( // NOLINT(cppcoreguidelines-pro-type-const-cast) mutable write through the const lookup view mirrors the oracle's in-place mutation
            *sw::find(welds, "pedal_left")).value);
        pedal.erase(std::ranges::find_if(pedal, [](const auto &kv) {
            return kv.first == "force_on_rider_n";
        }));
        std::vector<SamplePtr> rows{
            sample_row(0, 0., .001, {}, {}, {}, {}, std::move(channels))};
        const auto episode = make_episode(std::move(rows));
        throws<std::invalid_argument>(
            [&] { (void)runtime::encode_table_native(episode, plan, 0, 1); },
            "required", "a missing required map<struct> member rejects");
    }
}

// test_struct_fragments_respect_batch_rows_bound — the production source's
// 2-row windows reassemble into exactly the frozen oracle batch.
void test_struct_fragments_respect_batch_rows_bound() {
    bt_oracle::Node node(node_id("test_struct_fragments_respect_batch_rows_bound"));
    const auto &cat = support_catalog();
    std::vector<SamplePtr> rows;
    for (std::int64_t i = 0; i < 5; ++i) {
        const double f = static_cast<double>(i);
        auto channels = tires_channels(
            i % 2 == 0 ? std::vector<WireObject>{tire_patch(f + 1.0)}
                       : std::vector<WireObject>{},
            std::vector<WireObject>(static_cast<std::size_t>(i),
                                    tire_patch(2.0 * f)));
        merge_into(channels, rider_channels());
        rows.push_back(sample_row(i, 0., .001, {}, {}, {}, {},
                                  std::move(channels)));
    }
    const auto episode = make_episode(std::move(rows));
    const auto source = rec::make_physical_source(
        std::make_shared<runtime::OwnedNativeEpisode>(episode), cat, false);
    std::map<std::string, std::vector<EncodedTable>> fragments;
    source->tables(2, [&](std::string_view name, EncodedTable &&table) {
        fragments[std::string(name)].push_back(std::move(table));
    });
    for (const char *name :
         {"channel_tires", "channel_rider", "channel_model_status"}) {
        const auto &spec = cat.table(name);
        const auto it = fragments.find(name);
        bt::require(it != fragments.end() && !it->second.empty(),
                    "fragments were emitted");
        std::int64_t total = 0;
        for (const auto &fragment : it->second) {
            bt::require(fragment.rows <= 2, "each fragment is within bound");
            total += fragment.rows;
        }
        bt::require(total == 5, "the fragments reassemble to five rows");
        const auto merged = concat_tables(it->second);
        compare_batch(node, merged, spec, 0, spec.name);
    }
    node.done();
}

// test_applied_rows_bitwise_vs_python_mirror
void test_applied_rows_bitwise_vs_python_mirror() {
    bt_oracle::Node node(node_id("test_applied_rows_bitwise_vs_python_mirror"));
    const auto &cat = support_catalog();
    rec::Catalog mini(
        {cat.table("physics"), cat.table("forces"), cat.table("channel_drive"),
         cat.table("channel_sensors"), cat.table("channel_control")},
        cat.coordinates(), {});
    std::vector<SamplePtr> rows;
    for (std::int64_t i = 0; i < 9; ++i) {
        auto channels = drive_channels(0.0);
        channels.emplace_back("control", Wire{WireObject{
            {"motor_torque_nm", Wire{static_cast<double>(i / 3)}}, // NOLINT(bugprone-integer-division) integer channel grouping into thirds is deliberate
            {"rider_enabled", Wire{true}}}});
        rows.push_back(sample_row(i, static_cast<double>(i) * .001, .001, {}, {}, {}, {},
                                  std::move(channels)));
    }
    const auto episode = make_episode(std::move(rows));
    const auto source = rec::make_physical_source(
        std::make_shared<runtime::OwnedNativeEpisode>(episode), std::move(mini),
        true);
    std::vector<EncodedTable> applied;
    source->tables(4, [&](std::string_view name, EncodedTable &&table) {
        if (name == "commands_applied") applied.push_back(std::move(table));
    });
    const auto &applied_spec = rec::table_spec("commands_applied");
    rows_compare(node, static_cast<std::int64_t>(applied.size()),
                 "applied/count");
    for (std::size_t i = 0; i < applied.size(); ++i)
        compare_batch(node, applied[i], applied_spec, 1,
                      "applied/" + std::to_string(i));
    std::vector<std::int64_t> steps, ids;
    for (const auto &fragment : applied) {
        const auto &step = column(fragment, "step");
        const auto &app = column(fragment, "application_id");
        steps.insert(steps.end(), step.i.begin(), step.i.end());
        ids.insert(ids.end(), app.i.begin(), app.i.end());
    }
    bt::require(steps == std::vector<std::int64_t>({0, 3, 6}) &&
                    ids == std::vector<std::int64_t>({0, 1, 2}),
                "chunked applied rows keep (0,3,6) steps and (0,1,2) ids");
    node.done();
}

// test_verdict_leaf_coerces_bool_to_text
void test_verdict_leaf_coerces_bool_to_text() {
    bt_oracle::Node node(node_id("test_verdict_leaf_coerces_bool_to_text"));
    rec::TableSpec spec{
        .name = "channel_verdict", .kind = "channels",
        .keys = {key_field("episode_id"), key_field("interval_id")},
        .fields = {leaf("numerically_valid", "varchar",
                        {"model_status", "numerically_valid"}),
                   leaf("model_valid", "boolean",
                        {"model_status", "model_valid"}),
                   leaf("tags", "varchar[]", {"model_status", "tags"}, true)},
        .description = "verdict coverage", .source = "test",
        .claims = {"model_status"}, .exempt_keys = {}};
    auto const mini = mini_catalog(std::move(spec));
    std::vector<SamplePtr> rows;
    rows.push_back(sample_row(0, 0., .001, {}, {}, {}, {}, WireObject{
        {"model_status", Wire{WireObject{
             {"numerically_valid", Wire{true}},
             {"model_valid", Wire{false}},
             {"tags", Wire{"héllo"}}}}}}));
    rows.push_back(sample_row(1, 0., .001, {}, {}, {}, {}, WireObject{
        {"model_status", Wire{WireObject{
             {"numerically_valid", Wire{false}},
             {"model_valid", Wire{true}}}}}}));
    rows.push_back(sample_row(2, 0., .001, {}, {}, {}, {}, WireObject{
        {"model_status", Wire{WireObject{
             {"numerically_valid", Wire{"not_evaluated"}},
             {"model_valid", Wire{true}},
             {"tags", w_strs({"explicit", "list"})}}}}}));
    const auto episode = make_episode(std::move(rows));
    assert_identical(node, mini.table("channel_verdict"), episode, mini);
    const auto table = native_batch(mini.table("channel_verdict"), episode, mini);
    const auto &verdict = column(table, "numerically_valid");
    bt::require(verdict.s ==
                    std::vector<std::string>({"true", "false", "not_evaluated"}),
                "verdict leaf publishes 'true'/'false'/'not_evaluated'");
    const auto &tags = column(table, "tags");
    bt::require(tags.valid.size() == 3 && tags.valid[0] && !tags.valid[1] &&
                    tags.valid[2] &&
                    tags.offsets == std::vector<std::int32_t>({0, 5, 5, 7}),
                "tags spans the authored code points and list");
    bt::require(tags.s == std::vector<std::string>(
                    {"h", "\xc3\xa9", "l", "l", "o", "explicit", "list"}),
                "the str payload explodes to code points");
    expect_bool_column(table, "tags_present", {1, 0, 1});
    node.done();
}

// test_applied_rows_missing_control_rejects
void test_applied_rows_missing_control_rejects() {
    std::vector<SamplePtr> rows{sample_row(0, 0., .001, {}, {}, {}, {},
                                         drive_channels())};
    const auto episode = make_episode(std::move(rows));
    throws<std::invalid_argument>(
        [&] { (void)runtime::applied_command_rows_native(episode, 0, 1); },
        "ordinary applied control",
        "a missing control channel rejects applied_rows");
}

// test_forces_fragments_respect_batch_rows_bound
void test_forces_fragments_respect_batch_rows_bound() {
    bt_oracle::Node node(node_id("test_forces_fragments_respect_batch_rows_bound"));
    const auto &cat = support_catalog();
    const std::size_t n = 8;
    std::vector<SamplePtr> rows;
    rows.reserve(5);
    for (std::int64_t i = 0; i < 5; ++i)
      rows.push_back(sample_row(
          i, static_cast<double>(i) * .001, .001, {}, {},
          {{"a", std::vector<double>(n, 0.)},
           {"b", std::vector<double>(n, 1.)},
           {"c", std::vector<double>(n, 2.)}},
          WireObject{{"a", Wire{0.0}}, {"b", Wire{1.0}}, {"c", Wire{2.0}}},
          {}));
    const auto episode = make_episode(std::move(rows));
    const auto source = rec::make_physical_source(
        std::make_shared<runtime::OwnedNativeEpisode>(episode), cat, false);
    std::vector<EncodedTable> forces;
    source->tables(4, [&](std::string_view name, EncodedTable &&table) {
        if (name == "forces") forces.push_back(std::move(table));
    });
    bt::require(!forces.empty(), "forces fragments were emitted");
    std::int64_t total = 0;
    for (const auto &fragment : forces) {
        bt::require(fragment.rows <= 4, "each fragment is within bound");
        total += fragment.rows;
    }
    bt::require(total == 15, "5 rows x 3 components reassemble to 15");
    const auto merged = concat_tables(forces);
    compare_batch(node, merged, cat.table("forces"), 0, "forces");
    node.done();
}

// test_map_fragments_respect_batch_rows_bound
void test_map_fragments_respect_batch_rows_bound() {
    bt_oracle::Node node(node_id("test_map_fragments_respect_batch_rows_bound"));
    const auto &cat = support_catalog();
    std::vector<SamplePtr> rows;
    for (std::int64_t i = 0; i < 5; ++i) {
        const double f = static_cast<double>(i);
        auto channels = energy_channels(
            {{"elastic_energy_j", Wire{WireObject{
                 {"k" + std::to_string(i), Wire{f}},
                 {"k" + std::to_string(i) + "b", Wire{-f}}}}}},
            {{"component_work_j",
              i % 2 == 0
                  ? Wire{WireObject{{"only", Wire{0.5}}}}
                  : Wire{WireObject{}}}});
        rows.push_back(sample_row(i, static_cast<double>(i) * .001, .001, {}, {}, {}, {},
                                  std::move(channels)));
    }
    const auto episode = make_episode(std::move(rows));
    const auto source = rec::make_physical_source(
        std::make_shared<runtime::OwnedNativeEpisode>(episode), cat, false);
    std::vector<EncodedTable> batches;
    source->tables(2, [&](std::string_view name, EncodedTable &&table) {
        if (name == "channel_energy") batches.push_back(std::move(table));
    });
    bt::require(!batches.empty(), "channel_energy fragments were emitted");
    std::int64_t total = 0;
    for (const auto &fragment : batches) {
        bt::require(fragment.rows <= 2, "each fragment is within bound");
        total += fragment.rows;
    }
    bt::require(total == 5, "the fragments reassemble to five rows");
    const auto merged = concat_tables(batches);
    compare_batch(node, merged, cat.table("channel_energy"), 0,
                  "channel_energy");
    node.done();
}

// test_real_episode_all_tables_bitwise — the staged bootstrap replays the
// oracle's t=0 ride; every catalog table's native encode is bitwise-pinned.
void test_real_episode_all_tables_bitwise() {
    bt_oracle::Node node(node_id("test_real_episode_all_tables_bitwise"));
    auto ride = replay_episode(node);
    for (const auto &spec : ride.catalog->table_order())
        assert_identical(node, spec, ride.episode, *ride.catalog);
    node.done();
}

// test_whole_episode_single_pass — every spec's native plan engages, so the
// production source never takes the encode_rows fallback (the Python spy's
// exact equivalent), the emitted set covers the catalog and physics row
// totals match the episode extent.
void test_whole_episode_single_pass() {
    bt_oracle::Node node(node_id("test_whole_episode_single_pass"));
    auto ride = replay_episode(node);
    for (const auto &spec : ride.catalog->table_order()) {
        // compile_plan + the optimized() gate are the two conditions under
        // which tables() takes the reviewed encode_rows fallback.
        (void)rec::compile_plan(*ride.catalog, spec.name);
        const auto optimized = [](this auto &&self, const rec::FieldSpec &f) // NOLINT(misc-no-recursion) recursion mirrors the document tree
                -> bool {
            if (f.dtype == "map<bigint>") return false;
            return std::ranges::all_of(f.children, self);
        };
        bt::require(std::ranges::all_of(spec.fields, optimized),
                    spec.name + ": every field has a native plan");
    }
    std::map<std::string, std::int64_t> emitted;
    std::set<std::string> seen;
    rec::make_physical_source(
        std::make_shared<runtime::OwnedNativeEpisode>(ride.episode),
        *ride.catalog, false)
        ->tables(8, [&](std::string_view name, EncodedTable &&table) { // NOLINT(cppcoreguidelines-rvalue-reference-param-not-moved) callback signature is fixed by the API; the argument is consumed-read
            seen.insert(std::string(name));
            emitted[std::string(name)] += table.rows;
        });
    std::set<std::string> expected;
    for (const auto &spec : ride.catalog->table_order())
        expected.insert(spec.name);
    bt::require(seen == expected,
                "the emitted table set covers the catalog");
    bt::require(emitted["physics"] == episode_rows(ride.episode),
                "physics rows cover every retained interval");
    node.done();
}

// ---- derived-transition scan ------------------------------------------------

// _derived_stage_rows: (contact, coast, brake, counts, crash) per row.
runtime::WireObject derived_stage(bool contact,
                                  std::optional<const char *> coast,
                                  double brake, const WireObject &counts,
                                  bool crash, bool model_valid) {
    WireObject status = {
        {"model_valid", Wire{model_valid}},
        {"numerically_valid", Wire{true}},
        {"model_violation_counts", Wire{counts}},
    };
    WireObject side = {
        {"raw_contact", Wire{contact}},
        {"controller_grounded", Wire{contact}},
    };
    WireObject drive = {
        {"coasting_reason",
         coast.has_value() ? Wire{*coast} : Wire{}},
        {"freehub_engaged", Wire{true}},
        {"front_brake_demand", Wire{brake}},
        {"rear_brake_demand", Wire{brake}},
    };
    WireObject channels = {
        {"tires", Wire{WireObject{{"front", Wire{side}},
                                  {"rear", Wire{side}}}}},
        {"drive", Wire{std::move(drive)}},
        {"model_status", Wire{std::move(status)}},
    };
    if (crash)
        channels.emplace_back("contact_crash_cause", Wire{WireObject{
            {"kind", Wire{"tire_blowout"}}}});
    return channels;
}

std::vector<SamplePtr> derived_stage_rows() {
    const auto nan = std::numeric_limits<double>::quiet_NaN();
    const WireObject energy_int{{"energy", Wire{std::int64_t{1}}}};
    const WireObject energy_real{{"energy", Wire{1.0}}};
    std::vector<SamplePtr> rows;
    const std::int64_t n = 12;
    for (std::int64_t i = 0; i < n; ++i) {
        const bool contact = i < 5;
        const std::optional<const char *> coast =
            i < 6 ? std::nullopt : std::optional<const char *>{"coast"};
        // rows 7-8 carry -0.0, rows 9-10 NaN, row 11 returns to +0.0.
        const double brake = (i == 9 || i == 10) ? nan
            : (i == 7 || i == 8) ? -0.0 : 0.0;
        const WireObject &counts = i < 7 ? energy_int : energy_real;
        const bool crash = i >= 8;
        rows.push_back(sample_row(i, static_cast<double>(i) * .001, .001, {}, {}, {}, {},
            derived_stage(contact, coast, brake, counts, crash, i < 8)));
    }
    return rows;
}

// The 'events' freeze consumer: one u1 blob pins the whole expected
// InputEvent sequence; then the recorded float leaves land under
// event[i].changes[...] labels in the assert_derived_events walk order.
void compare_derived_events(bt_oracle::Node &node,
                            const std::vector<WireObject> &actual) {
    const std::string actual_frozen = freeze(actual);
    bytes_compare(node, actual_frozen, "|u1", actual_frozen.size(), "events");
    // The per-leaf entries: float leaves of the captured expected changes,
    // in first-insertion order. Reproduce the walk over the native rows —
    // the byte-frozen compare above already proved the trees identical, so
    // driving the labels off the native tree consumes in the same order.
    for (std::size_t i = 0; i < actual.size(); ++i) {
        const auto &row = actual[i];
        const auto *changes =
            sw::find(sw::object(sw::required(row, "values")), "changes");
        bt::require(changes != nullptr, "event row missing changes");
        plain_bitwise(&node, *changes, *changes,
                      "event[" + std::to_string(i) + "].changes");
    }
}

// test_derived_events_bitwise_vs_oracle
void test_derived_events_bitwise_vs_oracle() {
    bt_oracle::Node node(node_id("test_derived_events_bitwise_vs_oracle"));
    const auto episode = make_episode(derived_stage_rows());
    const auto actual = derived_events(episode, 7, 3, 1.5);
    compare_derived_events(node, actual);
    std::vector<std::string> kinds;
    std::vector<std::int64_t> intervals, ids;
    for (const auto &event : actual) {
        kinds.push_back(sw::string(sw::required(
            sw::object(sw::required(event, "values")), "derived_kind")));
        intervals.push_back(std::get<std::int64_t>(
            sw::required(sw::object(sw::required(event, "values")),
                         "interval_id")
                .value));
        ids.push_back(std::get<std::int64_t>(
            sw::required(event, "event_id").value));
    }
    bt::require(kinds == std::vector<std::string>(
                    {"tire_contact", "freehub_coast", "model_validity",
                     "contact_crash", "brake_gate", "brake_gate",
                     "brake_gate"}),
                "the staged kind sequence is emitted");
    bt::require(intervals ==
                    std::vector<std::int64_t>({5, 6, 8, 8, 9, 10, 11}),
                "the staged interval sequence is emitted");
    bt::require(ids == std::vector<std::int64_t>({3, 4, 5, 6, 7, 8, 9}),
                "event ids run consecutively from 3");
    node.done();
}

// test_derived_rows_window_seeding — chunked derived_rows concatenate to
// the full pass, seeding each window's compare from row start - 1.
void test_derived_rows_window_seeding() {
    bt_oracle::Node node(node_id("test_derived_rows_window_seeding"));
    const auto episode = make_episode(derived_stage_rows());
    const auto plan = runtime::parse_derived_event_plan(derived_plan_value());
    const auto whole = derived_rows(episode, plan, 0, episode_rows(episode));
    std::vector<WireObject> chunked;
    for (std::int64_t start = 0; start < episode_rows(episode); start += 4)
        for (auto &row :
             derived_rows(episode, plan, start,
                          std::min<std::int64_t>(4, episode_rows(episode) - start)))
            chunked.push_back(std::move(row));
    bt::require(chunked.size() == whole.size(),
                "the chunked pass emits the same row count");
    {
        WireArray a, e;
        for (const auto &row : chunked)
          a.emplace_back(row);
        for (const auto &row : whole)
          e.emplace_back(row);
        plain_bitwise(&node, Wire{std::move(a)}, Wire{std::move(e)},
                      "chunked");
    }
    const auto seeded = derived_rows(episode, plan, 5, 1);
    bt::require(seeded.size() == 1,
                "the single-row window emits the flip exactly once");
    {
        const auto &row = seeded.front();
        bt::require(std::get<std::int64_t>(
                        sw::required(row, "interval_id").value) == 5 &&
                        sw::string(sw::required(row, "kind")) ==
                            "derived_transition" &&
                        sw::string(sw::required(row, "derived_kind")) ==
                            "tire_contact",
                    "the seeded row is the row-5 tire_contact flip");
        std::set<std::string> labels;
        for (const auto &[label, value] : sw::object(
                 sw::required(row, "changes")))
            labels.insert(label);
        bt::require(labels == std::set<std::string>(
                        {"front.raw_contact", "rear.raw_contact",
                         "front.controller_grounded",
                         "rear.controller_grounded"}),
                    "the seeded diff covers all four contact labels");
    }
    bt::require(derived_rows(episode, plan, episode_rows(episode), 4).empty(),
                "start == row_count admits the empty suffix");
    node.done();
}

// test_derived_rows_rejects_bad_plans_and_ranges
void test_derived_rows_rejects_bad_plans_and_ranges() {
    std::vector<SamplePtr> one{derived_stage_rows().front()};
    const auto episode = make_episode(std::move(one));
    const auto plan = runtime::parse_derived_event_plan(derived_plan_value());
    for (const char *bad : {
             R"({})",
             R"({"fields":{}})",
             R"({"fields":[{"path":["a"]}]})",
             R"({"fields":[{"kind":"k","path":[]}]})",
             R"({"fields":[{"kind":"k","path":[1]}]})",
             R"({"fields":[{"kind":1,"path":["a"]}]})",
             R"({"fields":[],"extra":1})"})
        throws<std::invalid_argument>(
            [&] {
                const auto parsed = runtime::parse_derived_event_plan(
                    bike::io::parse_json(bad));
                (void)runtime::derived_event_rows_native(episode, parsed, 0, 1);
            },
            "", "a malformed derived plan rejects");
    for (const auto &[start, limit] :
         std::vector<std::pair<std::int64_t, std::int64_t>>{
             {-1, 1}, {0, 0}, {2, 1}})
        throws<std::invalid_argument>(
            [&] {
                (void)runtime::derived_event_rows_native(episode, plan, start,
                                                       limit);
            },
            "", "an out-of-domain derived slice rejects");
}

// test_derived_events_real_episode_bitwise — the sealed ride's event stream
// is frozen including dict-valued changes.
void test_derived_events_real_episode_bitwise() {
    bt_oracle::Node node(node_id("test_derived_events_real_episode_bitwise"));
    auto const ride = replay_episode(node);
    const auto actual = derived_events(ride.episode, 2, 5, 3.25);
    compare_derived_events(node, actual);
    // F1 merge-back: the completed weld-press servo holds the freehub
    // engaged through this ride — the oracle measures zero transitions
    // (the fix round had measured three freehub_engaged toggles at
    // intervals 1/10/48 on darwin-arm64, before the servo port landed).
    // The frozen-stream compare above is the assertion; populated scans
    // stay covered by test_derived_events_bitwise_vs_oracle's staged
    // episode.
    node.done();
}

} // namespace

int main() {
    // cppcheck-suppress throwInEntryPoint // test failures propagate as exceptions into bt::run's FAIL catch — the harness entry-point contract, not an unhandled throw
    return bt::run([] {
        test_make_episode_builds_retained_samples();
        test_make_episode_preserves_authored_value_tree();
        test_column_bits_detects_negative_zero_and_nan();
        test_physics_and_forces_bitwise_vs_oracle();
        test_compile_plan_contract();
        test_encode_table_piece_shape_and_domain();
        test_encode_table_rejects_bad_plans_and_ranges();
        test_forces_missing_power_is_value_error();
        test_flat_channel_tables_bitwise();
        test_optional_leaf_absent_vs_null();
        test_parent_mask_absent_vs_null();
        test_required_leaf_missing_vs_null_rejects();
        test_leaf_type_and_shape_gates();
        test_unknown_channel_root_rejected();
        test_unknown_channel_member_rejected();
        test_map_channel_tables_bitwise();
        test_map_leaf_rejects_non_dict_and_bad_member();
        test_map_required_leaf_missing_rejects();
        test_map_except_keys_routes_owned_member();
        test_all_list_and_map_dtypes_bitwise();
        test_struct_field_bitwise_channel_mass();
        test_struct_field_optional_member_channel_control();
        test_struct_nested_and_verdict_channel_model_status();
        test_struct_list_patches_channel_tires();
        test_map_struct_except_keys_and_nesting_channel_rider();
        test_struct_member_validation_parity();
        test_map_struct_bad_key_and_member_parity();
        test_struct_fragments_respect_batch_rows_bound();
        test_applied_rows_bitwise_vs_python_mirror();
        test_verdict_leaf_coerces_bool_to_text();
        test_applied_rows_missing_control_rejects();
        test_forces_fragments_respect_batch_rows_bound();
        test_map_fragments_respect_batch_rows_bound();
        test_real_episode_all_tables_bitwise();
        test_whole_episode_single_pass();
        test_derived_events_bitwise_vs_oracle();
        test_derived_rows_window_seeding();
        test_derived_rows_rejects_bad_plans_and_ranges();
        test_derived_events_real_episode_bitwise();
    });
}
