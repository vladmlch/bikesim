// T18: source-equivalent native/config.py and native/setup.py projection.
#include "project_config.hpp"
#include "capabilities.hpp"
#include "value_readers.hpp"
#include "../diag.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <ranges>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "setup_values.hpp"
#include "../cblas_abi.hpp"
#include "../configuration/load.hpp"
#include "../configuration/suspension_spec.hpp"
#include "../engine_call.hpp"
#include "../model/mapping.hpp"
#include "../io/json.hpp"
#include "../spatial/config.hpp"
#include "../spatial/topology.hpp"
#include "../rider/state_codec.hpp"
#include "../terrain/profile.hpp"
#include "../writer_config.hpp"
#include "bootstrap.hpp"

namespace runtime {
namespace {
namespace cfg = bike::configuration;
namespace mdl = bike::model;
namespace trn = bike::terrain;
namespace bio = bike::io;
using namespace setupvalue;

cfg::SimulationPhysicsConfig physics(const mdl::BuildRequest& request) {
    auto out = request.physics.value_or(cfg::default_physics());
    cfg::post_init(out);
    return out;
}
cfg::RiderSpecs rider(const mdl::BuildRequest& request) {
    return request.rider.value_or(mdl::rider_from_args(
        request.include_rider ? "lumped" : "none"));
}

void require_connected_spatial(const cfg::SimulationPhysicsConfig& config) {
    if (!config.spatial || config.spatial->mode == bike::spatial::Mode::planar) return;
    // W17 exposes bind_spatial_runtime for fully resolved writer3 input.
    // Automatic pneumatic3d material projection belongs to W25. Keep this
    // production gate until that producer and later runtime owners exist.
    const auto report = bike::spatial::check_spatial_capability(*config.spatial, config);
    if (!report.implemented)
        throw std::invalid_argument("config.spatial: " +
            (report.rejections.empty() ? std::string("spatial runtime not connected")
                                      : report.rejections.front()));
}

std::vector<std::byte> immutable_model_image(const mjModel& model) {
    struct SizeCall { const mjModel* model{}; mjtSize bytes = 0; } size{.model=&model};
    engine::ErrorBuffer error;
    if (!engine::invoke([](void* raw) noexcept {
        auto& call = *static_cast<SizeCall*>(raw);
        call.bytes = mj_sizeModel(call.model);
    }, &size, error)) engine::throw_failure(error);
    if (size.bytes <= 0 || std::cmp_greater(size.bytes, 512U * 1024U * 1024U))
        throw std::invalid_argument("project_runtime: compiled model image exceeds 512 MiB");
    std::vector<std::byte> image(static_cast<std::size_t>(size.bytes));
    struct SaveCall { const mjModel* model; void* bytes; int size; }
        save{.model=&model, .bytes=image.data(), .size=static_cast<int>(size.bytes)};
    if (!engine::invoke([](void* raw) noexcept {
        auto const& call = *static_cast<SaveCall*>(raw);
        mj_saveModel(call.model, nullptr, call.bytes, call.size);
    }, &save, error)) engine::throw_failure(error);
    return image;
}

void require_request_model(const mdl::BuildRequest& request, const mjModel& model,
                           const trn::TrackSpec& track) {
    if(request.spatial && request.spatial->mode==bike::spatial::Mode::spatial3d) {
        if(!request.terrain3d)
            throw std::invalid_argument("project_runtime.spatial.terrain: physical source missing");
        auto expected=mdl::compile_mjcf(mdl::generate_mjcf(request));
        mdl::populate_render_heightfield(*expected,trn::make_render_grid(*request.terrain3d,
            {static_cast<std::uint32_t>(request.field.nrow),static_cast<std::uint32_t>(request.field.ncol)}));
        if(immutable_model_image(model)!=immutable_model_image(*expected))
            throw std::invalid_argument("project_runtime.spatial.model: differs from typed model and terrain");
        return;
    }
    request.field.validate();
    track.validate();
    trn::assert_track_fits(track, request.field);
    // Names and dimensions are insufficient: two bikes can have identical
    // addresses but different geometry, masses or road samples. Rebuild from
    // the actual typed inputs, then compare the engine's complete immutable
    // serialization. This is a cold setup boundary, before runtime mutation;
    // it never consults fixtures or accepts an unverified caller fingerprint.
    auto expected = mdl::compile_mjcf(mdl::generate_mjcf(request));
    const auto field_id = static_cast<std::size_t>(mdl::required_id(*expected, mjOBJ_HFIELD, "road"));
    const auto rows = std::views::counted(expected->hfield_nrow, expected->nhfield);
    const auto columns = std::views::counted(expected->hfield_ncol, expected->nhfield);
    const auto addresses = std::views::counted(expected->hfield_adr, expected->nhfield);
    if (rows[field_id] != request.field.nrow || columns[field_id] != request.field.ncol)
        throw std::invalid_argument("project_runtime: generated road dimensions differ from BuildRequest");
    const auto samples = trn::build_field_data(track, request.field);
    const auto offset = addresses[field_id];
    if (offset < 0 || std::cmp_greater(offset, expected->nhfielddata) ||
        samples.size() > static_cast<std::size_t>(expected->nhfielddata - offset))
        throw std::invalid_argument("project_runtime: generated road address is outside heightfield storage");
    auto const target = std::views::counted(expected->hfield_data, expected->nhfielddata)
        .subspan(static_cast<std::size_t>(offset), samples.size());
    for (std::size_t i = 0; i < samples.size(); ++i) {
        if (!std::isfinite(samples[i]) || samples[i] < 0. || samples[i] > 1.)
            throw std::invalid_argument("project_runtime: normalized road elevation is outside [0, 1]");
        target[i] = static_cast<float>(samples[i]);
    }
    if (immutable_model_image(model) != immutable_model_image(*expected))
        throw std::invalid_argument("project_runtime: compiled model differs from BuildRequest and track");
}

Value damper_core(const nativecfg::DamperCore& d) {
    return Value{Object{
        {"max_hsc", integer(d.max_hsc)}, {"max_lsc", integer(d.max_lsc)},
        {"max_reb", integer(d.max_reb)}, {"hsc_clicks", integer(d.hsc_clicks)},
        {"lsc_clicks", integer(d.lsc_clicks)}, {"rebound_clicks", integer(d.rebound_clicks)},
        {"c_lsc_min", real(d.c_lsc_min)}, {"c_lsc_max", real(d.c_lsc_max)},
        {"c_hsc_min", real(d.c_hsc_min)}, {"c_hsc_max", real(d.c_hsc_max)},
        {"c_reb_min", real(d.c_reb_min)}, {"c_reb_max", real(d.c_reb_max)},
        {"v_knee_comp", real(d.v_knee_comp)}, {"v_knee_reb", real(d.v_knee_reb)}}};
}

Value suspension(const mdl::BuildRequest& request,
                 const cfg::SimulationPhysicsConfig& physics_config,
                 const mjModel& model) {
    const auto& b = request.bike;
    cfg::SuspensionInputs const inputs{.fork_hsc=b.fork_hsc, .fork_lsc=b.fork_lsc, .fork_rebound=b.fork_rebound,
        .shock_hsc=b.shock_hsc, .shock_lsc=b.shock_lsc, .shock_rebound=b.shock_rebound, .shock_hbo=b.shock_hbo, .shock_lockout=b.shock_lockout,
        .fork_travel=b.fork_travel, .shock_stroke=b.shock_stroke, .fork_initial_psi=b.fork_initial_psi, .fork_air_tokens=b.fork_air_tokens,
        .shock_stiffness=b.shock_stiffness};
    auto resolved = cfg::resolve_suspension(inputs);
    if (physics_config.physics_mode != "physical") {
        // RideSimulation._default_controller uses the default legacy damper
        // geometry/clicks; custom physical inputs use resolve_suspension.
        const auto legacy = cfg::resolve_suspension(cfg::SuspensionInputs{});
        resolved.fork_damper = legacy.fork_damper;
        resolved.shock_damper = legacy.shock_damper;
        resolved.fork_damper.legacy_behavior = true;
        resolved.shock_damper.legacy_behavior = true;
        resolved.fork_damper.hbo_start_mm = 160.;
        resolved.shock_damper.hbo_start_mm = 52.;
    }
    const auto& a = resolved.air;
    Value fork = damper_core(resolved.fork_damper.core);
    const auto& f = resolved.fork_damper;
    set(fork, "total_travel_mm", real(f.total_travel_mm));
    set(fork, "hbo_start_mm", real(f.hbo_start_mm));
    set(fork, "c_hbo_base", real(f.c_hbo_base));
    set(fork, "legacy_behavior", Value{f.legacy_behavior});
    Value shock = damper_core(resolved.shock_damper.core);
    const auto& s = resolved.shock_damper;
    set(shock, "total_stroke_mm", real(s.total_stroke_mm));
    set(shock, "max_hbo", integer(s.max_hbo));
    set(shock, "hbo_clicks", integer(s.hbo_clicks));
    set(shock, "lockout_firm", Value{s.lockout_firm});
    set(shock, "legacy_behavior", Value{s.legacy_behavior});
    set(shock, "hbo_start_mm", real(s.hbo_start_mm));
    set(shock, "c_hbo_min", real(s.c_hbo_min));
    set(shock, "c_hbo_max", real(s.c_hbo_max));
    set(shock, "lockout_preload_n", real(s.lockout_preload_n));
    set(shock, "lockout_stiffness", real(s.lockout_stiffness));
    const auto& c = resolved.coil;
    for (auto name : {"fork_travel", "shock_stroke"})
        (void)mdl::required_id(model, mjOBJ_JOINT, name);
    return Value{Object{
        {"physics_mode", text(physics_config.physics_mode)},
        {"joints", Value{Object{{"fork", text("fork_travel")}, {"shock", text("shock_stroke")}}}},
        {"air_spring", Value{Object{
            {"stanchion_inner_diam_mm", real(a.stanchion_inner_diam_mm)},
            {"total_travel_mm", real(a.total_travel_mm)},
            {"pos_chamber_length_mm", real(a.pos_chamber_length_mm)},
            {"neg_chamber_length_mm", real(a.neg_chamber_length_mm)},
            {"token_volume_cm3", real(a.token_volume_cm3)},
            {"max_tokens", integer(a.max_tokens)}, {"gamma", real(a.gamma)},
            {"atm_pressure_pa", real(a.atm_pressure_pa)},
            {"num_tokens", integer(resolved.num_tokens)},
            {"gauge_pressure_psi", real(resolved.gauge_pressure_psi)}}}},
        {"fork_damper", std::move(fork)}, {"shock_damper", std::move(shock)},
        {"coil", Value{Object{{"rate_n_m", real(c.rate_n_m)},
            {"preload_mm", real(c.preload_mm)}, {"stroke_mm", real(c.stroke_mm)},
            {"bumper_length_mm", real(c.bumper_length_mm)}, {"bumper_peak_n", real(c.bumper_peak_n)},
            {"legacy_behavior", Value{physics_config.physics_mode != "physical"}}}}},
        {"end_stops", Value{Object{
            {"stiffness_n_m", real(physics_config.end_stops.stiffness_n_m())},
            {"damping_n_s_m", real(physics_config.end_stops.damping_n_s_m)}}}}}};
}

Value surface(const trn::SurfaceSpec& s) {
    return Value{Object{{"name", text(s.name)}, {"mu_peak", real(s.mu_peak)},
        {"mu_slide", real(s.mu_slide)},
        {"slip_stiffness_per_load", real(s.slip_stiffness_per_load)},
        {"stribeck_speed_mps", real(s.stribeck_speed_mps)}}};
}
Value density(const cfg::HingeDensity& d) {
    return Value{Object{{"knots_m", reals(d.knots_m)},
        {"stiffness_n_m", reals(d.stiffness_n_m)},
        {"damping_ns_m", real(d.damping_ns_m)}, {"provenance", text(d.provenance)}}};
}
Value wheel_material(const cfg::DistributedWheelMaterial& m) {
    return Value{Object{{"density", density(m.density)},
        {"fitting_dataset_id", text(m.fitting_dataset_id)},
        {"valid_load_range_n", reals(m.valid_load_range_n)},
        {"valid_deflection_range_m", reals(m.valid_deflection_range_m)},
        {"calibration_status", text(m.calibration_status)}}};
}

Value tire(const mdl::BuildRequest& request,
           const cfg::SimulationPhysicsConfig& physics_config,
           const trn::TrackSpec& track, const Value& document) {
    const auto& tires = physics_config.tires;
    const auto& input = field(document, "tires");
    Value out = select(input, {"backend", "surface_mode", "significant_delta_m",
        "significance_fraction", "distinct_normal_deg"});
    for (auto side : {"front", "rear"}) {
        auto params = select(field(input, side),
            {"tangent_k_n_m", "mu", "relaxation_length_m"});
        if (tires.backend == "compliant_2d") {
            // Preserve source order: material precedes the tangential fields.
            auto with_material = Value{Object{{"material", field(field(input, side), "material")}}};
            for (const auto& entry : bio::as_object(params, "tire parameters"))
                std::get<Object>(with_material.data).push_back(entry);
            params = std::move(with_material);
        }
        set(out, side, std::move(params));
    }
    if (tires.surface_mode == "track") {
        Array sections;
        for (const auto& s : track.surface_sections)
            sections.push_back(Value{Object{{"start_m", real(s.start_m)},
                {"end_m", real(s.end_m)}, {"surface", surface(trn::get_surface(s.surface))}}});
        set(out, "surface_map", Value{Object{
            {"surface", surface(trn::get_surface(track.surface))},
            {"sections", Value{std::move(sections)}}}});
    }
    if (tires.backend == "distributed_2d_reference") {
        const auto& d = tires.distributed;
        set(out, "distributed", Value{Object{{"station_count", integer(d.station_count)},
            {"density", density(d.density)}, {"front", wheel_material(d.for_wheel("front"))},
            {"rear", wheel_material(d.for_wheel("rear"))}}});
    }
    if (tires.backend == "pneumatic") {
        const auto& p = tires.pneumatic;
        const auto* tier = cfg::find_tire_tier(p.tier);
        if (tier == nullptr) throw std::invalid_argument("unknown pneumatic tire tier '" + p.tier + "'");
        Value pneumatic{Object{{"tier", text(p.tier)}, {"n_rays", integer(tier->n_rays)},
            {"half_angle_deg", real(tier->half_angle_deg)},
            {"discretised_brush", Value{tier->discretised_brush}},
            {"timestep_s", real(tier->timestep_s)}}};
        if (p.surface) set(pneumatic, "surface", surface(trn::get_surface(*p.surface)));
        set(pneumatic, "front", field(field(input, "pneumatic"), "front"));
        set(pneumatic, "rear", field(field(input, "pneumatic"), "rear"));
        const auto x = request.field.track_x();
        auto z = trn::build_profile(track, x);
        const double ground = bike::geometry::compute_ground_z(request.bike) / 1000.;
        for (double& h : z) h = ground + h;
        if (x.size() < 2) throw std::invalid_argument("pneumatic road needs two samples");
        set(pneumatic, "profile", Value{Object{{"x0_m", real(x.front())},
            {"dx_m", real(x[1] - x[0])},
            {"z_m", reals(z)}}});
        set(out, "pneumatic", std::move(pneumatic));
    }
    return out;
}

Value drive(const cfg::SimulationPhysicsConfig& physics_config, const Value& document) {
    const auto& input = field(document, "drive");
    Value out = select(input, {"gearing", "pedaling", "shifting", "assist", "battery"});
    auto assist = field(out, "assist");
    const auto& a = physics_config.drive.assist;
    if (a.profile) {
        const auto* p = cfg::find_motor_profile(*a.profile);
        if (p == nullptr) throw std::invalid_argument("unknown motor profile '" + *a.profile + "'");
        set(assist, "max_torque", real(p->peak_torque_nm));
        set(assist, "max_power", real(p->peak_power_w));
        set(assist, "tau", real(p->torque_tau_s));
        set(assist, "slew", real(std::max(a.slew, p->peak_torque_nm / p->torque_tau_s)));
        set(assist, "cutoff_mps", real(p->cutoff_mps));
        set(assist, "taper_width_mps", real(p->taper_width_mps));
        set(assist, "gate_min_crank_rad_s", real(p->gate_min_crank_rad_s));
        set(assist, "profile", Value{Object{{"mode_gains", Value{Object{
            {"eco", real(p->mode_gains.eco)}, {"tour", real(p->mode_gains.tour)},
            {"emtb", reals(p->mode_gains.emtb)}, {"turbo", real(p->mode_gains.turbo)}}}},
            {"emtb_full_gain_at_nm", real(p->emtb_full_gain_at_nm)}}});
    }
    set(out, "assist", std::move(assist));
    set(out, "hub_stiffness_nm_rad", real(physics_config.drive.freehub_k_nm_rad));
    set(out, "hub_damping_nm_s", real(physics_config.drive.freehub_c_nms_rad));
    // NOLINTNEXTLINE(bugprone-unchecked-optional-access) physics schema requires drive_mode
    set(out, "drive_mode", text(physics_config.drive_mode.value()));
    for (auto key : {"transmission_model", "motor_clutch", "human_torque_nm", "torque_ripple",
         "crank_phase_rad", "chain_k_n_m", "chain_c_ns_m", "bearing_c_nms_rad",
         "rotor_inertia_kgm2", "brake_ceiling_nm", "brake_hold_torque_nm",
         "brake_hold_stiffness_nm_rad", "brake_hold_damping_nms_rad",
         "brake_hold_engage_radps", "brake_hold_release_radps"})
        set(out, key, field(input, key));
    return out;
}

Value curve(const cfg::TorqueCurve& c) {
    return Value{Object{{"angles_rad", reals(c.angles_rad)}, {"torques_nm", reals(c.torques_nm)},
        {"vmax_rad_s", real(c.vmax_rad_s)}, {"hill_c", real(c.hill_c)},
        {"eccentric_ratio", real(c.eccentric_ratio)}, {"source", text(c.source)}}};
}
Value envelopes(const cfg::JointEnvelopeMap& entries) {
    Object out;
    for (const auto& [name, e] : entries) out.emplace_back(name, Value{Object{
        {"neutral_anatomical_rad", real(e.neutral_anatomical_rad)},
        {"direction", integer(e.direction)}, {"minimum_anatomical_rad", real(e.minimum_anatomical_rad)},
        {"maximum_anatomical_rad", real(e.maximum_anatomical_rad)}, {"provenance", text(e.provenance)}}});
    return Value{std::move(out)};
}

} // namespace

void validate_native_supported(const cfg::SimulationPhysicsConfig& config,
                               const cfg::RiderSpecs& rider_spec) {
    validate_physical_capabilities(config, rider_spec);
}

bio::Value project_articulated_pose(const mdl::ArticulatedPose& p) {
    // Preserve the complete source ArticulatedPose document, including the
    // visual-only members the current spindle decoder does not consume.
    const auto& s = p.saddle;
    return Value{Object{{"saddle", Value{Object{
        {"seatpost_top", reals(s.seatpost_top)}, {"saddle_pos", reals(s.saddle_pos)},
        {"top_center", reals(s.top_center)}, {"top_z_m", real(s.top_z_m)},
        {"height_m", real(s.height_m)}, {"exposed_post_m", real(s.exposed_post_m)}}}},
        {"hip", reals(p.hip)}, {"shoulder", reals(p.shoulder)}, {"elbow", reals(p.elbow)},
        {"grip", reals(p.grip)}, {"head_center", reals(p.head_center)},
        {"knee_front", reals(p.knee_front)}, {"knee_rear", reals(p.knee_rear)},
        {"ankle_front", reals(p.ankle_front)}, {"ankle_rear", reals(p.ankle_rear)},
        {"pedal_front", reals(p.pedal_front)}, {"pedal_rear", reals(p.pedal_rear)}}};
}

bio::Value project_rider_controller(const cfg::SimulationPhysicsConfig& config,
                                    const mjModel& model) {
    auto const document = cfg::detail::plain_document(cfg::to_value(config));
    auto out = field(document, "articulated");
    // weld_servo_gain/weld_servo_limit_nm/leg_track_damping_nms_rad are
    // declared Python-only rider-controller knobs the native controller has
    // not ported: they belong to the resolved config surface but must not
    // cross this closed wire — decode_spindle_config validates exact keys.
    // native/setup.py rider_controller_config drops the same three.
    static constexpr std::array<std::string_view, 3> kUnportedKeys{
        "weld_servo_gain", "weld_servo_limit_nm", "leg_track_damping_nms_rad"};
    auto* members = std::get_if<Object>(&out.data);
    if (members == nullptr)
        throw std::invalid_argument(
            "rider_controller: articulated field is not an object");
    std::erase_if(*members, [](const Object::value_type& member) {
        return std::ranges::find(kUnportedKeys, member.first) !=
               kUnportedKeys.end() || member.first=="shoulder_profile" || member.first=="shoulder_profile_sha256";
    });
    set(out, "strength", Value{});
    set(out, "strength_coordinates", Value{Object{}});
    set(out, "envelopes", Value{Object{}});
    if(config.spatial && config.spatial->mode==bike::spatial::Mode::spatial3d &&
       mj_name2id(&model,mjOBJ_BODY,"rider_pelvis")>=0) {
        set(out,"rider.spatial",integer(1));
        set(out,"shoulder_profile",mdl::shoulder_profile_document(config.articulated));
        set(out,"steer_torque_owner",text("actuator"));
    }
    std::vector<std::string_view> joints;
    for (auto const name : cfg::kRiderJoints) {
        const std::string owned(name);
        if (mj_name2id(&model, mjOBJ_JOINT, owned.c_str()) >= 0) joints.push_back(name);
    }
    if (joints.empty()) return out;
    const auto& a = config.articulated;
    if (a.joint_strength_path) {
        const auto profile = cfg::load_strength_profile(*a.joint_strength_path,
            joints, false);
        Object strength;
        for (const auto& [name, curves] : profile) strength.emplace_back(name,
            Value{Object{{"positive", curve(curves.at(1))}, {"negative", curve(curves.at(-1))}}});
        set(out, "strength", Value{std::move(strength)});
        set(out, "strength_coordinates", envelopes(cfg::load_strength_coordinates(*a.joint_strength_path, joints)));
    }
    if (a.joint_envelope_path)
        set(out, "envelopes", envelopes(cfg::load_joint_envelopes(*a.joint_envelope_path,
            std::span<const std::string_view>(joints))));
    return out;
}

namespace {
bio::Value finalize_writer_config(const Value& out) {
    cfg::wire::validate_config(out);
    (void)nativecfg::decode(out);
    return cfg::wire::plain(out);
}
} // namespace

bio::Value project_drive_config(const cfg::SimulationPhysicsConfig& source) {
    auto resolved = std::make_unique<cfg::SimulationPhysicsConfig>(source);
    cfg::post_init(*resolved);
    const auto document = cfg::detail::plain_document(cfg::to_value(*resolved));
    const auto projected = finalize_writer_config(Value{Object{
        {"schema", integer(2)}, {"drive", drive(*resolved, document)}}});
    return field(projected, "drive");
}

bio::Value project_spatial_runtime_envelope(const cfg::SimulationPhysicsConfig& config,
                                           const bio::Value& resolved_fields) {
    cfg::validate(config);
    if (!config.spatial || config.spatial->mode != bike::spatial::Mode::spatial3d)
        throw std::invalid_argument("config.spatial: runtime2 requires spatial='3d'");
    bike::spatial::validate_spatial_config(*config.spatial, config);
    // This also rejects duplicate keys in caller-constructed owned Values.
    (void)bio::canonical_json(resolved_fields);
    const vread::Dict root{.value = &vread::mapping({&resolved_fields}, "config"), .path = "config"};
    vread::exact(root, vread::keys("timestep_s", "control_period_s", "control_period_steps",
        "physics_mode", "drive_mode", "strict", "measurement_decimation", "record_retention",
        "writer_config", "rider_controller", "rider_intent", "monitors", "geometry"),
        vread::keys("async_telemetry"));
    const auto positive = [](const vread::Dict& d, const char* name) {
        const double value = vread::finite_real(vread::field(d, name), d.child(name));
        if (!(value > 0.)) vread::invalid(d.child(name), "expected positive value");
        return value;
    };
    const auto nonnegative = [](const vread::Dict& d, const char* name) {
        const double value = vread::finite_real(vread::field(d, name), d.child(name));
        if (value < 0.) vread::invalid(d.child(name), "expected nonnegative value");
    };
    const auto timestep = positive(root, "timestep_s");
    const auto control_period = positive(root, "control_period_s");
    const auto steps = vread::integer32(vread::field(root, "control_period_steps"), root.child("control_period_steps"));
    if (steps <= 0) vread::invalid(root.child("control_period_steps"), "expected positive integer");
    if (timestep != config.timestep_s || control_period != config.control_period_s)
        vread::invalid(root.child("timestep_s"), "resolved periods differ from source configuration");
    if (std::abs(timestep * steps - control_period) > 1e-9 * control_period)
        vread::invalid(root.child("control_period_s"), "control_period_steps * timestep_s mismatch");
    if (vread::string(vread::field(root, "physics_mode"), root.child("physics_mode")) != "physical")
        vread::invalid(root.child("physics_mode"), "runtime2 requires physical mode");
    const auto& drive_mode = vread::string(vread::field(root, "drive_mode"), root.child("drive_mode"));
    if (!config.drive_mode || drive_mode != *config.drive_mode)
        vread::invalid(root.child("drive_mode"), "differs from source configuration");
    (void)vread::boolean(vread::field(root, "strict"), root.child("strict"));
    if (root.contains("async_telemetry"))
        (void)vread::boolean(vread::field(root, "async_telemetry"), root.child("async_telemetry"));
    if (vread::integer32(vread::field(root, "measurement_decimation"), root.child("measurement_decimation")) <= 0)
        vread::invalid(root.child("measurement_decimation"), "expected positive integer");
    const auto& retention = vread::string(vread::field(root, "record_retention"), root.child("record_retention"));
    if (retention != "off" && retention != "full" && retention != "legacy")
        vread::invalid(root.child("record_retention"), "expected 'off', 'full' or 'legacy'");

    const auto& writer = field(resolved_fields, "writer_config");
    nativecfg::validate_spatial_writer_envelope(writer);
    const auto writer_dict = vread::section(root, "writer_config");
    const auto& writer_spatial = field(writer, "spatial");
    if (bio::as_string(field(writer_spatial, "root_kind"), "config.writer_config.spatial.root_kind") !=
        bike::spatial::to_string(config.spatial->root))
        vread::invalid("config.writer_config.spatial.root_kind", "differs from config.spatial.root");
    // The selected 3D path needs its own tyre owner. Its coefficient object is
    // staged data only until W25 supplies a complete typed decoder/constructor.
    (void)vread::section(writer_dict, "tire");
    const auto writer_drive = vread::section(writer_dict, "drive");
    if (vread::string(vread::field(writer_drive, "drive_mode"), writer_drive.child("drive_mode")) != drive_mode)
        vread::invalid(root.child("drive_mode"), "differs from writer_config.drive.drive_mode");
    (void)vread::section(root, "rider_controller");
    (void)vread::section(root, "rider_intent");
    const auto monitors = vread::section(root, "monitors");
    vread::exact(monitors, vread::keys("balance_floor_mps", "balance_dwell_s", "balance_grace_s", "grounded_hold_s"));
    (void)positive(monitors, "balance_floor_mps");
    (void)positive(monitors, "balance_dwell_s");
    nonnegative(monitors, "balance_grace_s");
    nonnegative(monitors, "grounded_hold_s");
    // C7 preserves these outer field names, not planar binding semantics.
    // This cold envelope checks names/index shapes only. W17 must decode the
    // spatial pose/rider schemas and rebind names against the supplied model;
    // these integers never establish portable topology identity or prove that
    // a named joint is a one-DoF hinge on the expected spatial wheel body.
    const auto geometry = vread::section(root, "geometry");
    vread::exact(geometry, vread::keys("crank_length_m", "brake_dofs", "wheel_bodies", "com_marker_site", "pose"));
    (void)positive(geometry, "crank_length_m");
    const auto brake = vread::section(geometry, "brake_dofs");
    vread::exact(brake, vread::keys("front", "rear"));
    for (const auto name : {"front", "rear"}) {
        const auto wheel = vread::section(brake, name);
        vread::exact(wheel, vread::keys("joint", "dof_index"));
        if (vread::string(vread::field(wheel, "joint"), wheel.child("joint")).empty())
            vread::invalid(wheel.child("joint"), "must not be empty");
        if (vread::integer32(vread::field(wheel, "dof_index"), wheel.child("dof_index")) < 0)
            vread::invalid(wheel.child("dof_index"), "expected nonnegative index");
    }
    const auto bodies = vread::section(geometry, "wheel_bodies");
    vread::exact(bodies, vread::keys("front", "rear"));
    for (const auto name : {"front", "rear"})
        if (vread::string(vread::field(bodies, name), bodies.child(name)).empty())
            vread::invalid(bodies.child(name), "must not be empty");
    const auto marker = vread::field(geometry, "com_marker_site");
    if (!vread::is_none(marker)) (void)vread::string(marker, geometry.child("com_marker_site"));
    const auto pose = vread::field(geometry, "pose");
    if (!vread::is_none(pose)) (void)vread::mapping(pose, geometry.child("pose"));

    Value out = resolved_fields;
    set(out, "runtime_schema", integer(bike::spatial::kSpatialRuntimeSchema));
    set(out, "spatial", bike::spatial::spatial_config_to_wire(*config.spatial));
    return out;
}

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
bio::Value project_spatial_writer_identity(const mdl::BuildRequest& request,const mjModel& model) {
    if(!request.spatial || request.spatial->mode!=bike::spatial::Mode::spatial3d || !request.terrain3d)
        throw std::invalid_argument("project_runtime.spatial: configured physical owners required");
    const auto topology=bike::spatial::bind_topology(model,*request.spatial);
    return Value{Object{{"spatial_contract",integer(bike::spatial::kSpatialContract)},
        {"root_kind",text(bike::spatial::to_string(topology.root.kind))},
        {"topology_fingerprint",text(topology.fingerprint_hex)},
        {"terrain_sha256",text(request.terrain3d->fingerprint_hex())},
        {"numeric_profile",text("binary64")}}};
}

RuntimeConfig bind_spatial_runtime(const mdl::BuildRequest& request,const mjModel& model,
    const trn::TrackSpec& track,const bio::Value& resolved_fields) {
    const auto config=physics(request);
    if(!config.spatial || !request.spatial ||
       bike::spatial::spatial_config_fingerprint_hex(*config.spatial)!=
       bike::spatial::spatial_config_fingerprint_hex(*request.spatial))
        throw std::invalid_argument("project_runtime.spatial: model/configuration mismatch");
    require_request_model(request,model,track);
    const auto envelope=project_spatial_runtime_envelope(config,resolved_fields);
    const auto expected=project_spatial_writer_identity(request,model);
    if(bio::canonical_json(field(field(envelope,"writer_config"),"spatial"))!=bio::canonical_json(expected))
        throw std::invalid_argument("config.writer_config.spatial: differs from rebound model/terrain identity");
    auto result=decode_runtime_config(envelope);
    result.terrain3d=request.terrain3d;
    const auto topology=bike::spatial::bind_topology(model,*request.spatial);
    const std::array<const JointReference*,2> brakes{&result.geometry.front_brake,&result.geometry.rear_brake};
    const std::array<std::optional<std::string>,2> bodies{result.geometry.front_wheel_body,result.geometry.rear_wheel_body};
    for(std::size_t side=0;side<2;++side) {
        const auto& wheel=topology.wheels[side];
        if(brakes[side]->dof_index!=0 || mj_name2id(&model,mjOBJ_JOINT,brakes[side]->joint.c_str())!=wheel.spin.joint_id)
            throw std::invalid_argument("config.geometry.brake_dofs: differs from spatial wheel binding");
        if(!bodies[side] || mj_name2id(&model,mjOBJ_BODY,bodies[side]->c_str())!=wheel.wheel_body_id)
            throw std::invalid_argument("config.geometry.wheel_bodies: differs from spatial wheel binding");
    }
    return result;
}

bio::Value project_writer_config(const mdl::BuildRequest& request, const mjModel& model,
                                const trn::TrackSpec& track, const ProjectionOptions& options) {
    const auto config_ptr = std::make_unique<cfg::SimulationPhysicsConfig>(physics(request));
    const auto& config = *config_ptr;
    require_connected_spatial(config);
    track.validate();
    const auto document = cfg::detail::plain_document(cfg::to_value(config));
    auto res = field(document, "resistance");
    for (auto name : {"frame", "front_wheel", "rear_wheel"})
        (void)mdl::required_id(model, mjOBJ_BODY, name);
    set(res, "bodies", Value{Object{{"frame", text("frame")},
        {"front_wheel", text("front_wheel")}, {"rear_wheel", text("rear_wheel")}}});
    Value out{Object{{"schema", integer(2)}, {"brake", Value{Object{
        {"torque_ceiling_nm", real(200.)}, {"taper_radps", real(1.)}}}},
        {"resistance", std::move(res)}, {"suspension", suspension(request, config, model)}}};
    const auto& backend = config.tires.backend;
    if (backend != "native_reference") {
        if (backend != "compliant_2d" && backend != "distributed_2d_reference" && backend != "pneumatic")
            throw std::invalid_argument("native tire backend '" + backend + "' is not ported");
        set(out, "tire", tire(request, config, track, document));
    }
    const auto resolved_rider = rider(request);
    if (resolved_rider.variant == "seated") {
        Array paths;
        for (const auto& body : mdl::seated_pose(resolved_rider, request.bike).bodies) {
            (void)mdl::required_id(model, mjOBJ_JOINT, body.joint);
            paths.push_back(Value{Object{{"name", text(body.name)}, {"joint", text(body.joint)},
                {"stiffness_n_m", real(body.stiffness_n_m)}, {"damping_ns_m", real(body.damping_ns_m)},
                {"preload_deflection_m", real(body.preload_deflection_m())},
                {"unilateral", Value{body.unilateral}}, {"offset_m", real(0.)}}});
        }
        if (!paths.empty()) set(out, "rider_forces", Value{Object{{"paths", Value{std::move(paths)}}}});
    }
    if (config.physics_mode == "physical") {
        set(out, "drive", drive(config, document));
        if (resolved_rider.variant == "articulated_planar") {
            const auto p = mdl::geometry_pose(resolved_rider, request.bike);
            std::array<double, 3> upper{}, lower{};
            for (std::size_t i = 0; i < 3; ++i) {
                upper[i] = p.shoulder[i] - p.elbow[i];
                lower[i] = p.elbow[i] - p.grip[i];
            }
            const double reach = std::sqrt(blas::ddot(3, upper.data(), 1, upper.data(), 1)) +
                                 std::sqrt(blas::ddot(3, lower.data(), 1, lower.data(), 1));
            Value contacts{Object{{"arm_reach_m", real(reach)}}};
            auto const fields = select(field(document, "articulated"), {
                "saddle_patch_half_length_m", "pedal_patch_half_length_m", "support_pad_radius_m",
                "support_k_n_m", "support_c_ns_m", "pedal_c_ns_m", "support_tangent_k_n_m",
                "support_mu", "support_length_m", "grip_k_n_m", "grip_c_ns_m",
                "grip_release_distance_m", "grip_pair_force_limit_n", "grip_capture_distance_m",
                "grip_capture_speed_mps", "pedal_attachment", "saddle_attachment", "grip_attachment"});
            for (const auto& entry : bio::as_object(fields, "articulated contacts"))
                std::get<Object>(contacts.data).push_back(entry);
            set(out, "rider_contacts", std::move(contacts));
        }
    }
    // RideSimulation always constructs the legacy cruise object; physical
    // stepping does not apply it, but the projection preserves its presence.
    set(out, "cruise", Value{Object{{"target_speed_kmh", real(options.speed_kph)},
        {"kp_nm_per_mps", real(180.)}, {"ki_nm_per_mps_s", real(150.)},
        {"torque_ceiling_nm", real(150.)}}});
    return finalize_writer_config(out);
}

RuntimeConfig project_runtime(const mdl::BuildRequest& request, const mjModel& model,
                              const trn::TrackSpec& track, const ProjectionOptions& options) {
    const auto config = physics(request);
    require_connected_spatial(config);
    const auto resolved_rider = rider(request);
    validate_native_supported(config, resolved_rider);
    if (model.opt.timestep != config.timestep_s)
        throw std::invalid_argument("config.timestep_s differs from the compiled model");
    require_request_model(request, model, track);
    const double periods = config.control_period_s / config.timestep_s;
    if (!std::isfinite(periods) || periods < 1. || periods > std::numeric_limits<int>::max())
        throw std::invalid_argument("control period is outside the native step-count domain");
    const int period_steps = static_cast<int>(std::nearbyint(periods));
    if (std::abs(static_cast<double>(period_steps) * config.timestep_s - config.control_period_s) > 1e-9)
        throw std::invalid_argument("control_period_s must be an integer multiple of timestep_s");
    for (auto name : {"frame", "front_wheel", "rear_wheel"})
        (void)mdl::required_id(model, mjOBJ_BODY, name);
    const bool articulated = resolved_rider.variant == "articulated_planar";
    if (articulated) (void)mdl::required_id(model, mjOBJ_BODY, "rider_pelvis");
    Value wheels{Object{}};
    for (auto side : {"front", "rear"}) {
        const std::string joint = std::string(side) + "_wheel_spin";
        const int id = mdl::required_id(model, mjOBJ_JOINT, joint);
        if (std::views::counted(model.jnt_type, model.njnt)[static_cast<std::size_t>(id)] != mjJNT_HINGE)
            throw std::invalid_argument("brake wheel joint must be a hinge: " + joint);
        set(wheels, side, Value{Object{{"joint", text(joint)}, {"dof_index", integer(0)}}});
    }
    const Value pose = articulated
        ? project_articulated_pose(mdl::geometry_pose(resolved_rider, request.bike))
        : Value{};
    const int com = mj_name2id(&model, mjOBJ_SITE, "site_CG");
    const auto physics_document = cfg::detail::plain_document(cfg::to_value(config));
    Value const envelope{Object{
        {"runtime_schema", integer(kRuntimeSchema)}, {"timestep_s", real(model.opt.timestep)},
        {"control_period_s", real(config.control_period_s)}, {"control_period_steps", integer(period_steps)},
        {"physics_mode", text(config.physics_mode)}, { // NOLINTNEXTLINE(bugprone-unchecked-optional-access) physics schema requires drive_mode
        "drive_mode", text(config.drive_mode.value())},
        {"strict", Value{options.strict}}, {"measurement_decimation", integer(options.measurement_decimation)},
        {"async_telemetry", Value{options.async_telemetry}},
        {"record_retention", text(options.record_retention == RetentionMode::off ? "off" :
            options.record_retention == RetentionMode::full ? "full" : "legacy")},
        {"writer_config", project_writer_config(request, model, track, options)},
        {"rider_controller", project_rider_controller(config, model)},
        {"rider_intent", field(physics_document, "seated_climb")},
        {"monitors", Value{Object{{"balance_floor_mps", real(config.articulated.balance_floor_kmh / 3.6)},
            {"balance_dwell_s", real(config.articulated.balance_dwell_s)},
            {"balance_grace_s", real(config.articulated.balance_grace_s)}, {"grounded_hold_s", real(.005)}}}},
        {"geometry", Value{Object{{"crank_length_m", real(request.bike.crank_length / 1000.)},
            {"brake_dofs", std::move(wheels)}, {"wheel_bodies", Value{Object{
                {"front", text("front_wheel")}, {"rear", text("rear_wheel")}}}},
            {"com_marker_site", com < 0 ? Value{} : text("site_CG")},
            {"pose", pose}}}}}};
    return decode_runtime_config(envelope);
}
NATIVE_DIAG_POP

RuntimeConfig project_runtime(const cfg::SimulationPhysicsConfig& config, const mjModel& model) {
    mdl::BuildRequest request;
    request.mode = "ride";
    request.physics = config;
    request.rider = mdl::rider_from_args("articulated_planar");
    trn::TrackSpec track;
    track.name = "flat";
    track.length_m = 112.;
    return project_runtime(request, model, track);
}

} // namespace runtime
