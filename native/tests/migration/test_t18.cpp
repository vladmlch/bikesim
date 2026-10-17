// T18: independent captured projection, source-bound model compatibility,
// backend-specific road/material data and world-frame virtual work.
#include "support/migration_test.hpp"
#include "configuration/load.hpp"
#include "engine_call.hpp"
#include "io/json.hpp"
#include "model/builder.hpp"
#include "model/mapping.hpp"
#include "runtime/bootstrap.hpp"
#include "runtime/project_config.hpp"
#include "runtime/setup_values.hpp"
#include "terrain/trackfile.hpp"
#include "tyre/profile.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <ranges>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {
namespace mdl = bike::model;
namespace cfg = bike::configuration;
namespace trn = bike::terrain;
namespace io = bike::io;
using namespace runtime::setupvalue;

struct DataDeleter { void operator()(mjData* data) const noexcept { if (data) mj_deleteData(data); } };
using DataOwner = std::unique_ptr<mjData, DataDeleter>;

std::unique_ptr<mdl::BuildRequest> physical(std::string_view profile = "viewer_physics_welded.toml") {
    auto request = std::make_unique<mdl::BuildRequest>();
    request->mode = "ride";
    request->rider = cfg::RiderSpecs{};
    request->rider->variant = "articulated_planar";
    request->physics = cfg::load_physics(bt::fixture("config/" + std::string(profile)));
    return request;
}

mdl::ModelOwner compiled(const mdl::BuildRequest& request, const trn::TrackSpec& track) {
    auto result = mdl::compile_mjcf(mdl::generate_mjcf(request));
    const auto id = static_cast<std::size_t>(mdl::required_id(*result, mjOBJ_HFIELD, "road"));
    const auto offset = std::views::counted(result->hfield_adr, result->nhfield)[id];
    const auto samples = trn::build_field_data(track, request.field);
    bt::require(offset >= 0 && std::cmp_less_equal(offset, result->nhfielddata), "road address is in range");
    bt::require(samples.size() <= static_cast<std::size_t>(result->nhfielddata - offset), "road samples fit engine storage");
    auto target = std::views::counted(result->hfield_data, result->nhfielddata)
        .subspan(static_cast<std::size_t>(offset), samples.size());
    for (std::size_t i = 0; i < samples.size(); ++i) target[i] = static_cast<float>(samples[i]);
    return result;
}

bool same_bits(double a, double b) {
    return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b);
}
double number(const Value& value) {
    bool valid = false;
    const double result = io::detail::float_from_lexeme(io::as_number(value, "projection").lexeme, valid);
    bt::require(valid, "projected numeric value is finite binary64");
    return result;
}
const Value::Array& array(const Value& value) {
    const auto* result = std::get_if<Value::Array>(&value.data);
    bt::require(result != nullptr, "projected field is an array");
    return *result;
}
void equal_value(const Value& actual, const Value& expected, std::string_view label) {
    bt::require(io::canonical_json(actual) == io::canonical_json(expected), label);
}
template<class Action>
void invalid_message(std::string_view expected, Action&& action) {
    try { std::forward<Action>(action)(); }
    catch (const std::invalid_argument& error) {
        bt::require(expected == error.what(), "source invalid-state diagnostic");
        return;
    }
    bt::require(false, "expected source invalid-state exception");
}

Value captured_config() {
    // The packed snapshot contains the captured JSON but need not contain
    // the bootstrap's binary arrays. Verify exactly the artifact consumed by
    // this regression against its original manifest declaration and digest.
    auto manifest = bt::manifest_entry("bootstrap/native_start_welded");
    std::erase_if(manifest.files, [](const auto& item) {
        return item.path != "bootstrap/native_start_welded/config.json";
    });
    bt::require(manifest.files.size() == 1, "captured projection has one manifest-declared JSON artifact");
    bt::manifest_verify_files(manifest);
    return io::parse_json(bt::text(bt::fixture(manifest.files.front().path)));
}

void relocate_declared_assets(Value& controller, const cfg::ArticulatedConfig& config) {
    for (const auto& [name, local] : std::array{
             std::pair{"joint_envelope_path", config.joint_envelope_path},
             std::pair{"joint_strength_path", config.joint_strength_path}}) {
        bt::require(local.has_value(), "captured profile uses the declared rider asset");
        const auto& captured = std::get<std::string>(field(controller, name).data);
        bt::require(std::filesystem::path(captured).filename() == std::filesystem::path(*local).filename(),
                    "relocation retains the same captured asset filename");
        set(controller, name, text(*local));
    }
}

void independent_projection() {
    auto request = physical();
    const auto track = trn::load_track(bt::fixture("tracks/flat.toml"));
    request->field = trn::HeightFieldSpec::for_track(track);
    const auto model = compiled(*request, track);
    runtime::ProjectionOptions options;
    options.strict = false;
    options.record_retention = runtime::RetentionMode::legacy;
    const auto actual = runtime::project_runtime(*request, *model, track, options);
    auto captured = captured_config();
    auto controller = field(captured, "rider_controller");
    relocate_declared_assets(controller, request->physics->articulated);
    set(captured, "rider_controller", std::move(controller));
    const auto expected = runtime::decode_runtime_config(captured);
    equal_value(actual.writer_config, expected.writer_config, "complete writer projection matches captured Python setup");
    equal_value(actual.rider_controller, expected.rider_controller, "complete strength/envelope controller projection");
    equal_value(actual.rider_intent, expected.rider_intent, "complete rider intent projection");
    equal_value(actual.geometry.pose, expected.geometry.pose, "complete resolved geometry pose");
    bt::require(same_bits(actual.timestep_s, expected.timestep_s) &&
                same_bits(actual.control_period_s, expected.control_period_s) &&
                actual.control_period_steps == expected.control_period_steps, "captured runtime clock");
    bt::require(actual.strict == expected.strict && actual.measurement_decimation == expected.measurement_decimation &&
                actual.record_retention == expected.record_retention, "captured runtime publication settings");
    bt::require(same_bits(actual.monitors.balance_floor_mps, expected.monitors.balance_floor_mps) &&
                same_bits(actual.monitors.balance_dwell_s, expected.monitors.balance_dwell_s) &&
                same_bits(actual.monitors.balance_grace_s, expected.monitors.balance_grace_s) &&
                same_bits(actual.monitors.grounded_hold_s, expected.monitors.grounded_hold_s), "captured monitor configuration");
    bt::require(actual.geometry.front_brake.joint == expected.geometry.front_brake.joint &&
                actual.geometry.rear_brake.joint == expected.geometry.rear_brake.joint &&
                actual.geometry.front_brake.dof_index == expected.geometry.front_brake.dof_index &&
                actual.geometry.rear_brake.dof_index == expected.geometry.rear_brake.dof_index &&
                actual.geometry.front_wheel_body == expected.geometry.front_wheel_body &&
                actual.geometry.rear_wheel_body == expected.geometry.rear_wheel_body &&
                actual.geometry.com_marker_site == expected.geometry.com_marker_site &&
                same_bits(actual.geometry.crank_length_m, expected.geometry.crank_length_m), "captured named geometry references");
}

void model_identity_and_heightfield() {
    auto request = physical();
    const auto track = trn::flat();
    const auto model = compiled(*request, track);
    const auto projected = runtime::project_runtime(*request, *model, track);
    bt::require(projected.geometry.front_brake.joint == "front_wheel_spin", "correct typed model is accepted");
    // This foreign model has the same dimensions, names and addresses.
    auto foreign_request = std::make_unique<mdl::BuildRequest>(*request);
    foreign_request->bike.reach += 7.;
    const auto foreign = compiled(*foreign_request, track);
    bt::require(model->nq == foreign->nq && model->nv == foreign->nv && model->ngeom == foreign->ngeom,
                "wrong-geometry regression holds dimensions constant");
    bt::require(mdl::joint_address(*model, "fork_travel") == mdl::joint_address(*foreign, "fork_travel"),
                "wrong-geometry regression holds names and addresses constant");
    bt::invalid([&] { (void)runtime::project_runtime(*request, *foreign, track); });
    auto masses = std::views::counted(model->body_mass, model->nbody);
    const auto frame = static_cast<std::size_t>(mdl::required_id(*model, mjOBJ_BODY, "frame"));
    const double old_mass = masses[frame]; masses[frame] += .01;
    bt::invalid([&] { (void)runtime::project_runtime(*request, *model, track); });
    masses[frame] = old_mass;
    auto samples = std::views::counted(model->hfield_data, model->nhfielddata);
    const float old_sample = samples[0]; samples[0] = std::nextafter(old_sample, 1.F);
    bt::invalid([&] { (void)runtime::project_runtime(*request, *model, track); });
    samples[0] = old_sample;
    const auto restored = runtime::project_runtime(*request, *model, track);
    equal_value(restored.writer_config, projected.writer_config, "failed identity checks do not poison projection");
    const auto empty_road = mdl::compile_mjcf(mdl::generate_mjcf(*request));
    bt::invalid([&] { (void)runtime::project_runtime(*request, *empty_road, track); });
    DataOwner data(engine::make_data(model.get()));
    engine::forward(model.get(), data.get());
    const auto vertices = biketyre::compiled_profile_vertices(model.get(), data.get(), "terrain");
    const auto x = request->field.track_x();
    const auto row = trn::build_field_data(track, request->field);
    bt::require(vertices.size() == x.size(), "installed field has the full longitudinal profile");
    const double base = std::stod(bike::geometry::detail::py_fixed(
        request->field.geom_z_m(bike::geometry::compute_ground_z(request->bike) / 1000.), 6));
    // The engine samples a centered local raster then translates it. Its X
    // rounding is intentionally distinct from the uncentered pneumatic grid.
    const auto local_x = bike::geometry::detail::np_linspace(
        -request->field.radius_x_m, request->field.radius_x_m, x.size());
    for (std::size_t i = 0; i < vertices.size(); ++i) {
        const double z = static_cast<double>(static_cast<float>(row[i])) * request->field.elevation_m + base;
        bt::require(same_bits(vertices[i][0], local_x[i] + request->field.geom_x_m()) &&
                    same_bits(vertices[i][1], z), "compiled road placement and binary32 sample installation");
    }
}

void backend_projection() {
    auto track = trn::flat();
    track.surface = "asphalt";
    track.surface_sections = {{10., 12., "hardpack"}};
    for (const auto* profile : {"viewer_physics_welded.toml", "viewer_physics_distributed.toml", "viewer_physics_pneumatic.toml"}) {
        auto request = physical(profile);
        auto model = compiled(*request, track);
        const auto runtime_config = runtime::project_runtime(*request, *model, track);
        const auto& tire = field(runtime_config.writer_config, "tire");
        const auto& map = field(tire, "surface_map");
        bt::require(std::get<std::string>(field(field(map, "surface"), "name").data) == "asphalt", "track default surface is resolved");
        bt::require(array(field(map, "sections")).size() == 1, "track surface sections retain their boundaries");
        const auto& section = array(field(map, "sections")).front();
        bt::require(number(field(section, "start_m")) == 10. && number(field(section, "end_m")) == 12., "surface section extent");
        bt::require(std::get<std::string>(field(field(section, "surface"), "name").data) == "hardpack", "section surface is resolved");
        const auto& backend = request->physics->tires.backend;
        if (backend == "compliant_2d") {
            bt::require(find(field(tire, "front"), "material") != nullptr, "compliant material belongs to each wheel");
        } else {
            bt::require(find(field(tire, "front"), "material") == nullptr, "distributed and pneumatic sides use their own material laws");
        }
        if (backend == "distributed_2d_reference") {
            const auto& distributed = field(tire, "distributed");
            bt::require(number(field(distributed, "station_count")) == static_cast<double>(request->physics->tires.distributed.station_count), "distributed station count");
            bt::require(find(field(distributed, "front"), "calibration_status") != nullptr &&
                        find(field(distributed, "rear"), "valid_load_range_n") != nullptr, "distributed wheel material metadata");
        }
        if (backend == "pneumatic") {
            const auto& road = field(field(tire, "pneumatic"), "profile");
            const auto x = request->field.track_x();
            const auto z = trn::build_profile(track, x);
            const auto& actual = array(field(road, "z_m"));
            bt::require(same_bits(number(field(road, "dx_m")), x[1] - x[0]), "RoadProfile spacing uses the source first difference");
            bt::require(same_bits(number(field(road, "x0_m")), x.front()) && actual.size() == z.size(), "pneumatic profile extent");
            const double ground = bike::geometry::compute_ground_z(request->bike) / 1000.;
            for (std::size_t i = 0; i < z.size(); ++i)
                bt::require(same_bits(number(actual[i]), ground + z[i]), "pneumatic road preserves unquantized source binary64 heights");
            request->physics->tires.surface_mode = "configured";
            request->physics->tires.pneumatic.surface = "hardpack";
            model = compiled(*request, track);
            const auto uniform = runtime::project_runtime(*request, *model, track);
            const auto& uniform_tire = field(uniform.writer_config, "tire");
            bt::require(find(uniform_tire, "surface_map") == nullptr, "configured pneumatic surface omits a track map");
            bt::require(std::get<std::string>(field(field(field(uniform_tire, "pneumatic"), "surface"), "name").data) == "hardpack", "configured pneumatic surface is resolved");
            request->physics->tires.pneumatic.surface.reset();
            bt::invalid([&] { runtime::validate_native_supported(*request->physics, *request->rider); });
        } else {
            request->physics->tires.surface_mode = "configured";
            const auto configured = runtime::project_writer_config(*request, *model, track);
            bt::require(find(field(configured, "tire"), "surface_map") == nullptr, "configured material projection omits track data");
            bt::invalid([&] { runtime::validate_native_supported(*request->physics, *request->rider); });
        }
    }
}

void mapping_and_virtual_work() {
    const auto model = mdl::compile_mjcf(
        "<mujoco><option gravity='0 0 0'/><worldbody>"
        "<body name='slider' pos='1 0 0'><joint name='translation' type='slide' axis='1 0 0'/>"
        "<geom type='sphere' size='.1' mass='2'/></body>"
        "<body name='rotor' pos='0 0 1'><joint name='rotation' type='hinge' axis='0 1 0'/>"
        "<geom type='sphere' size='.1' mass='3'/></body></worldbody></mujoco>");
    DataOwner data(engine::make_data(model.get()));
    auto qvel = std::views::counted(data->qvel, model->nv);
    qvel[0] = 2.; qvel[1] = 3.;
    engine::forward(model.get(), data.get());
    const int slider = mdl::required_id(*model, mjOBJ_BODY, "slider");
    const int rotor = mdl::required_id(*model, mjOBJ_BODY, "rotor");
    const mdl::WorldVector point{.3, 0., 1.2}, force{5., 0., 7.}, torque{0., 11., 0.};
    const double rx = point[0], rz = point[2] - 1.;
    const auto jac = mdl::point_jacobian(*model, *data, rotor, point);
    bt::require(jac.translation.size() == 6 && jac.rotation.size() == 6, "Jacobian matrices have shape (3,nv)");
    bt::require(same_bits(jac.translation[1], rz) && same_bits(jac.translation[5], -rx) &&
                jac.rotation[3] == 1., "hinge Jacobian obeys world-frame cross-product convention");
    std::vector<double> scratch(6, 123.), second(6);
    const auto* storage = scratch.data();
    mdl::point_jacobian_into(*model, *data, rotor, point, scratch);
    bt::require(scratch == jac.translation && scratch.data() == storage, "scratch Jacobian uses the caller's complete storage");
    const auto velocity = mdl::point_velocity(*model, *data, rotor, point);
    const auto angular = mdl::body_angular_velocity(*model, *data, rotor);
    bt::require(velocity[0] == rz * qvel[1] && velocity[2] == -rx * qvel[1] && angular[1] == qvel[1], "world point and angular velocities");
    const auto wrench = mdl::map_wrench(*model, *data, rotor, point, force, torque);
    const double moment = rz * force[0] - rx * force[2] + torque[1];
    bt::require(std::abs(wrench[1] - moment) < 1e-14 && wrench[0] == 0., "world wrench maps to hinge moment");
    const double spatial_power = force[0] * velocity[0] + force[2] * velocity[2] + torque[1] * angular[1];
    bt::require(std::abs(wrench[1] * qvel[1] - spatial_power) < 1e-13, "virtual work agrees in spatial and generalized coordinates");
    mdl::relative_point_jacobian(*model, *data, slider, rotor, point, scratch, second);
    const auto on_a = mdl::map_wrench(*model, *data, slider, point, force);
    const auto on_b = mdl::map_wrench(*model, *data, rotor, point, {-force[0], 0., -force[2]});
    for (std::size_t i = 0; i < 2; ++i) {
        const double generalized = scratch[i] * force[0] + scratch[4 + i] * force[2];
        bt::require(std::abs(generalized - (on_a[i] + on_b[i])) < 1e-14, "relative Jacobian equals an equal-and-opposite force pair");
    }
    bt::invalid([&] { (void)mdl::required_id(*model, mjOBJ_JOINT, "not_a_joint"); });
    bt::invalid([&] { (void)mdl::required_id(*model, mjOBJ_SITE, "rotation"); });
    bt::invalid([&] { (void)mdl::required_id(*model, mjOBJ_JOINT, std::string_view("rotation\0suffix", 15)); });
    invalid_message("relative Jacobian requires distinct physical bodies", [&] {
        mdl::relative_point_jacobian(*model, *data, rotor, rotor, point, scratch, second);
    });
    bt::invalid([&] { mdl::relative_point_jacobian(*model, *data, slider, rotor, point, scratch, scratch); });
    bt::invalid([&] { mdl::point_jacobian_into(*model, *data, rotor, point, std::span<double>(scratch).first(5)); });
    invalid_message("point Jacobian requires a physical body", [&] {
        (void)mdl::point_jacobian(*model, *data, 0, point);
    });
    const mdl::WorldVector invalid{std::numeric_limits<double>::quiet_NaN(), 0., 0.};
    invalid_message("invalid shape or non-finite world point", [&] {
        (void)mdl::point_jacobian(*model, *data, rotor, invalid);
    });
    invalid_message("invalid world point", [&] {
        mdl::point_jacobian_into(*model, *data, rotor, invalid, scratch);
    });
    invalid_message("invalid shape or non-finite world force", [&] {
        (void)mdl::map_wrench(*model, *data, rotor, point, invalid);
    });
    invalid_message("invalid shape or non-finite world torque", [&] {
        (void)mdl::map_wrench(*model, *data, rotor, point, force, invalid);
    });
}

void coordinate_addresses() {
    const auto model = mdl::compile_mjcf(
        "<mujoco><worldbody><body name='free'><freejoint name='root'/>"
        "<geom type='sphere' size='.1' mass='1'/><body name='child'>"
        "<joint name='hinge'/><geom type='sphere' size='.1' mass='1'/>"
        "</body></body></worldbody></mujoco>");
    bt::require(mdl::joint_address(*model, "hinge") == std::pair{7, 6}, "qpos and DOF addresses remain distinct after a free joint");
    bt::require(mdl::joint_name_at_qpos(*model, 7) == "hinge", "joint name recovered from its actual qpos address");
    bt::invalid([&] { (void)mdl::joint_name_at_qpos(*model, 6); });
    const auto anonymous = mdl::compile_mjcf(
        "<mujoco><worldbody><body><joint/><geom type='sphere' size='.1' mass='1'/></body></worldbody></mujoco>");
    invalid_message("joint at qposadr 0 has no name", [&] { (void)mdl::joint_name_at_qpos(*anonymous, 0); });
}
} // namespace

int main() {
    return bt::run([] {
        independent_projection(); model_identity_and_heightfield(); backend_projection();
        mapping_and_virtual_work(); coordinate_addresses();
    });
}
