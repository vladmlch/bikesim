#include "evidence.hpp"
#include "../diag.hpp"
#include "../configuration/load.hpp"
#include "../configuration/suspension_spec.hpp"
#include "../engine_identity.hpp"
#include "../io/hash.hpp"
#include "../io/json.hpp"
#include "../io/npy.hpp"
#include "../model/export.hpp"
#include "../platform/files.hpp"
#include "../platform/provenance.hpp"
#include "../runtime/provenance.hpp"
#include "../runtime/setup_values.hpp"
#include "../rider/allocation.hpp"
#include "../terrain/trackfile.hpp"
#include "../terrain/asset3d.hpp"
#include "../terrain/source3d.hpp"
#include "../spatial/config.hpp"
#include "../runtime/research_io.hpp"
#include "../runtime/spatial_tire_factory.hpp"
#include "../writers/pyfloat.hpp"
#include <algorithm>
#include <bit>
#include <cctype>
#include <cmath>
#include <fstream>
#include <ranges>
#include <type_traits>

namespace bike::research {
namespace {
using namespace runtime::setupvalue;
Value damper_coefficients(const nativecfg::DamperCore& c) {
    const double lsc = std::clamp(c.lsc_clicks, 0, c.max_lsc) / static_cast<double>(c.max_lsc);
    const double hsc = std::clamp(c.hsc_clicks, 0, c.max_hsc) / static_cast<double>(c.max_hsc);
    const double reb = std::clamp(c.rebound_clicks, 0, c.max_reb) / static_cast<double>(c.max_reb);
    return Value{Object{{"c_lsc", real(c.c_lsc_min + (c.c_lsc_max - c.c_lsc_min) * pyfloat::pow(lsc, 1.25))},
        {"c_hsc", real(c.c_hsc_min + (c.c_hsc_max - c.c_hsc_min) * pyfloat::pow(hsc, 1.15))},
        {"c_reb", real(c.c_reb_min + (c.c_reb_max - c.c_reb_min) * pyfloat::pow(reb, 1.20))},
        {"frac_lsc", real(lsc)}, {"frac_hsc", real(hsc)}, {"frac_reb", real(reb)}}};
}
Value active_suspension(const geometry::BikeSpecs& b) {
    const auto s = configuration::resolve_suspension(configuration::SuspensionInputs{
        .fork_hsc=b.fork_hsc, .fork_lsc=b.fork_lsc, .fork_rebound=b.fork_rebound, .shock_hsc=b.shock_hsc, .shock_lsc=b.shock_lsc, .shock_rebound=b.shock_rebound,
        .shock_hbo=b.shock_hbo, .shock_lockout=b.shock_lockout, .fork_travel=b.fork_travel, .shock_stroke=b.shock_stroke, .fork_initial_psi=b.fork_initial_psi,
        .fork_air_tokens=b.fork_air_tokens, .shock_stiffness=b.shock_stiffness});
    auto shock = damper_coefficients(s.shock_damper.core);
    const auto& d = s.shock_damper;
    const double frac = std::clamp(d.hbo_clicks, 0, d.max_hbo) / static_cast<double>(d.max_hbo);
    set(shock, "c_hbo", real(d.c_hbo_min + (d.c_hbo_max - d.c_hbo_min) * pyfloat::pow(frac, 1.30)));
    set(shock, "frac_hbo", real(frac)); set(shock, "lockout_firm", Value{d.lockout_firm});
    const auto& c = s.coil;
    Value coil{Object{{"rate_n_m", real(c.rate_n_m)}, {"preload_mm", real(c.preload_mm)}, {"stroke_mm", real(c.stroke_mm)},
        {"bumper_length_mm", real(c.bumper_length_mm)}, {"bumper_peak_n", real(c.bumper_peak_n)}}};
    return Value{Object{{"fork_pressure_psi", real(s.gauge_pressure_psi)}, {"fork_tokens", integer(s.num_tokens)},
        {"coil", std::move(coil)}, {"fork_damper", damper_coefficients(s.fork_damper.core)}, {"shock_damper", std::move(shock)}}};
}
std::string terrain_digest(const runtime::TruthGeometry& geometry) {
    if (geometry.x.empty() || geometry.x.size() != geometry.z.size())
        throw std::invalid_argument("terrain identity requires matching nonempty x/z vertices");
    std::vector<std::byte> bytes;
    bytes.reserve(geometry.x.size() * 16);
    for (std::size_t i = 0; i < geometry.x.size(); ++i)
        for (const double value : {geometry.x[i], geometry.z[i]}) {
            if (!std::isfinite(value)) throw std::invalid_argument("terrain identity requires finite vertices");
            const auto bits = std::bit_cast<std::uint64_t>(value);
            for (unsigned shift = 0; shift < 64; shift += 8)
                bytes.push_back(static_cast<std::byte>((bits >> shift) & 255U));
        }
    return io::sha256(bytes);
}
std::string configuration_digest(const Value& resolved) {
    auto hashable = resolved;
    auto physics = field(hashable, "physics");
    auto articulated = field(physics, "articulated");
    for (auto key : {"joint_envelope", "joint_strength"}) {
        const auto* digest = find(resolved, std::string(key) + "_sha256");
        set(articulated, std::string(key) + "_path", digest ? *digest : Value{});
    }
    if(find(articulated,"shoulder_profile")) {
        const auto* digest=find(resolved,"shoulder_profile_asset_sha256");
        set(articulated,"shoulder_profile",digest?*digest:Value{});
    }
    if(const auto* source=find(physics,"spatial_3d")) {
        auto spatial=*source;auto steer=field(spatial,"steer");
        if(io::as_string(field(steer,"controller"),"steer.controller")!="off" &&
           !rider::builtin_steer_profile(io::as_string(field(steer,"profile_id"),"steer.profile_id"))) {
            // A file's location is provenance; its checked coefficients own
            // the physical configuration identity after relocation.
            set(steer,"profile_id",field(steer,"profile_sha256"));
            set(spatial,"steer",std::move(steer));set(physics,"spatial_3d",std::move(spatial));
        }
    }
    set(physics, "articulated", std::move(articulated));
    set(hashable, "physics", std::move(physics));
    const auto json = io::canonical_json(hashable);
    return io::sha256(std::as_bytes(std::span{json.data(), json.size()}));
}
runtime::ResearchAsset asset(std::string name, std::string media_type, std::vector<std::byte> content) {
    const auto hash = io::sha256(content);
    return {.name=std::move(name), .media_type=std::move(media_type), .sha256=hash, .content=std::move(content)};
}
runtime::ResearchAsset text_asset(std::string name, std::string media_type, std::string_view content) {
    const auto bytes = std::as_bytes(std::span{content.data(), content.size()});
    return asset(std::move(name), std::move(media_type), std::vector<std::byte>(bytes.begin(), bytes.end()));
}
} // namespace

io::Value wire_value(const runtime::Wire& wire) {
    return std::visit([](const auto& value) -> Value {
        using T = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<T, std::monostate>) return Value{};
        else if constexpr (std::is_same_v<T, bool> || std::is_same_v<T, std::string>) return Value{value};
        else if constexpr (std::is_same_v<T, std::int64_t>) return integer(value);
        else if constexpr (std::is_same_v<T, double>) {
            if (std::isfinite(value)) return real(value);
            return Value{io::Tagged{"float64", {{"value", std::isnan(value) ? "nan" : value < 0. ? "-inf" : "+inf"}}}};
        } else if constexpr (std::is_same_v<T, runtime::WireArray>) {
            Array out; out.reserve(value.size());
            for (const auto& item : value) out.push_back(wire_value(item));
            return Value{std::move(out)};
        } else {
            Object out; out.reserve(value.size());
            for (const auto& [key, item] : value) out.emplace_back(key, wire_value(item));
            return Value{std::move(out)};
        }
    }, wire.value);
}

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
io::Value physical_configuration_metadata(const initialization::SetupRequest& request,
    const initialization::ResearchStartup& startup, const runtime::RuntimeBootstrap& bootstrap,
    const io::Value& setup_metadata, const std::optional<std::string>& seed_decimal) {
    if (!request.model.physics) throw std::invalid_argument("physical metadata requires resolved physics");
    const auto& physics = *request.model.physics;
    const auto& inputs = field(setup_metadata, "inputs");
    if (bootstrap.config.spatial && !bootstrap.config.terrain3d) throw std::invalid_argument("spatial metadata requires immutable terrain owner");
    const auto hash = bootstrap.config.spatial ? bootstrap.config.terrain3d->fingerprint_hex() : terrain_digest(startup.geometry);
    const auto seed = seed_decimal ? Value{io::Number{validate_seed(*seed_decimal)}} : Value{};
    Value resolved{Object{{"physics", configuration::detail::plain_document(configuration::to_value(physics))},
        {"geometry_and_suspension", field(inputs, "specs")}, {"mass_budget", field(inputs, "mass_specs")},
        {"rider", field(inputs, "rider")}, {"target_speed_kmh", physics.drive_mode == "ideal_speed_control" ? real(request.runtime.speed_kph) : Value{}},
        {"terrain_sha256", text(hash)}, {"seed", seed},
        {"track", terrain::track_to_dict(request.track)}, {"active_suspension", active_suspension(request.model.bike)}}};
    for (const auto& [name, value] : io::as_object(startup.compiled_metadata, "compiled metadata")) set(resolved, name, value);
    set(resolved, "allocation_numerical_guards", Value{Object{{"support_force_n", real(1e-5)},
        {"support_moment_nm", real(1e-6)}, {"grip_radius_fraction", real(1e-6)}}});
    for (const auto& [key, path] : {std::pair{"joint_envelope", physics.articulated.joint_envelope_path},
                                 std::pair{"joint_strength", physics.articulated.joint_strength_path}}) {
        set(resolved, std::string(key) + "_status", text(path ? "declared_unvalidated" : "unspecified"));
        if (path) set(resolved, std::string(key) + "_sha256", text(platform::file_sha256(*path)));
    }
    if (bootstrap.config.spatial) {
        set(resolved,"start_x_m",real(request.start_x_m));
        if(physics.articulated.shoulder_profile)
            set(resolved,"shoulder_profile_asset_sha256",text(platform::file_sha256(*physics.articulated.shoulder_profile)));
    }
    const auto configuration_hash = configuration_digest(resolved);
    const auto identity = runtime::runtime_identity();
    const auto engine_identity = engine::identity();
    const auto build = platform::build_identity();
    Object provenance;
    for (const auto& [name, ignored] : request.model.masses.component_masses()) {
        (void)ignored; provenance.emplace_back(name, text("synthetic"));
    }
    const auto limitation = physics.tires.backend == "distributed_2d_reference"
        ? "experimental distributed tire; mesh/time/station gates required"
        : physics.tires.backend == "pneumatic" ? "experimental pneumatic tire; carcass/brush parameters not calibrated"
        : "single equivalent tire support; multi-support peaks not calibrated";
    Value execution{Object{{"backend", text("native")}, {"native_source_sha256", text(identity.native_source_sha256)},
        {"execution_identity", text(platform::execution_identity())}, {"mujoco_library_sha256", text(engine_identity.library_sha256)},
        {"numeric_provider", text(engine_identity.numeric_provider)}, {"executable_sha256", text(build.executable_sha256)},
        {"source_identity_scope", text("compiled first-party source manifest; running image hashed separately")}}};
    if(bootstrap.config.spatial)set(execution,"tyre_threads",integer(bootstrap.config.tyre_threads));
    if (io::as_string(field(field(inputs, "rider"), "variant"), "rider variant") == "articulated_planar" &&
        physics.articulated.pedal_attachment != "spindle") {
        set(execution, "allocation_solver", text(bike::rider::allocation_numeric_identity));
        set(execution, "allocation_solver_qualification", text("pending"));
    }
    Value result{Object{{"schema_version", integer(2)}, {"physics_revision", text("physical-2026-09-29")},
        {"model_commit", Value{}}, {"model_source_dirty", Value{}}, {"model_source_sha256", text(identity.native_source_sha256)},
        {"versions", Value{Object{{"mujoco", text(identity.mujoco_version)}, {"compiler", text(build.compiler)}}}},
        {"configuration_sha256", text(configuration_hash)}, {"terrain_sha256", text(hash)},
        {"resolved_config", std::move(resolved)}, {"seed", seed}, {"timestep_s", real(bootstrap.config.timestep_s)},
        {"controller_interval_s", real(bootstrap.config.control_period_s)}, {"tire_backend", text(physics.tires.backend)},
        { // NOLINTNEXTLINE(bugprone-unchecked-optional-access) physics schema requires drive_mode
        "drive_mode", text(physics.drive_mode.value())}, {"rider_model", field(field(inputs, "rider"), "variant")},
        {"external_speed_controller", Value{physics.drive_mode == "ideal_speed_control"}}, {"calibration_status", text("parameterized_unvalidated")},
        {"environment_contract", Value{Object{{"scope", text("native executable, provider and dependency-lock identity")},
            {"dependency_lock_sha256", text(build.dependency_lock_sha256)}, {"build_type", text(build.build_type)},
            {"platform", text(build.platform)}, {"locked_environment_verified", Value{}}}}},
        {"component_provenance", Value{std::move(provenance)}},
        {"limitations", Value{Array{text("planar X-Z dynamics"), text("synthetic material and drive parameters"),
            text(limitation), text("native contact work is a solver diagnostic, not material tire hysteresis")}}},
        {"execution", std::move(execution)}, {"setup", setup_metadata}, {"model_sha256", text(bootstrap.model.sha256)}}};
    if (bootstrap.config.spatial) {
        if (!bootstrap.state.spatial) throw std::invalid_argument("spatial metadata requires bootstrap identity");
        set(result,"recording_schema_version",integer(2));
        set(result,"sensor_schema_version",integer(2));
        set(result,"calibration_sha256",text(bootstrap.state.spatial->calibration_sha256));
        set(result,"topology_fingerprint",text(bootstrap.state.spatial->topology_fingerprint));
        set(result,"spatial",spatial::spatial_config_to_wire(*bootstrap.config.spatial));
        set(result,"limitations",Value{Array{text("spatial 3D; numerical qualification pending V07/V08"),text("synthetic tyre coefficients; calibration not measured")}});
    }
    return result;
}

io::Value configuration_metadata(const ResearchRequest& request, const initialization::ResearchStartup& startup,
    const runtime::RuntimeBootstrap& bootstrap, const io::Value& setup_metadata) {
    auto metadata = physical_configuration_metadata(request.setup, startup, bootstrap, setup_metadata,
        request.experiment.seed_decimal);
    set(metadata, "experiment", experiment_value(request.experiment));
    set(metadata, "sensor_config", bootstrap.config.spatial
        ? wire_value(runtime::Wire(runtime::sensor_config_v2_to_wire(request.sensors_v2))) : sensor_config_value(request.sensors));
    if (bootstrap.config.spatial) set(metadata,"sensor_rng_revision",text("sensor-v2-rng/1"));
    set(metadata, "rider_program", rider_program_value(request.programs));
    set(metadata, "demand_program", demand_program_value(request.programs));
    return metadata;
}
NATIVE_DIAG_POP

io::Value reseed_metadata(const io::Value& original, std::string_view seed_decimal) {
    if (io::is_null(original)) return original; // legacy captured bootstrap
    auto metadata = original;
    const auto seed = Value{io::Number{validate_seed(seed_decimal)}};
    auto resolved = field(metadata, "resolved_config");
    set(resolved, "seed", seed);
    set(metadata, "configuration_sha256", text(configuration_digest(resolved)));
    set(metadata, "resolved_config", std::move(resolved));
    set(metadata, "seed", seed);
    // experiment.seed records the original request. actual_sensor_seed and
    // resolved_config.seed identify this episode's reset, as in the source.
    return metadata;
}

io::Value identity_receipt(const io::Value& metadata) {
    Value receipt{Object{{"source_changed_during_run", Value{}},
        {"configuration_changed_during_run", Value{}}, {"execution_changed_during_run", Value{}}}};
    try {
        const auto identity = runtime::runtime_identity();
        const auto image = platform::build_identity();
        const auto provider = engine::identity();
        const auto& execution = field(metadata, "execution");
        auto resolved = field(metadata, "resolved_config");
        const auto articulated = field(field(resolved, "physics"), "articulated");
        // The model configuration is owned and immutable. Referenced profile
        // bytes can change independently, so re-hash their original paths.
        for (auto key : {"joint_envelope", "joint_strength"}) {
            const auto* path = find(articulated, std::string(key) + "_path");
            if (path && !io::is_null(*path))
                set(resolved, std::string(key) + "_sha256", text(platform::file_sha256(io::as_string(*path, key))));
        }
        if(const auto* path=find(articulated,"shoulder_profile");path&&!io::is_null(*path))
            set(resolved,"shoulder_profile_asset_sha256",text(platform::file_sha256(io::as_string(*path,"shoulder_profile"))));
        if(const auto* spatial_source=find(field(resolved,"physics"),"spatial_3d")) {
            const auto& steer=field(*spatial_source,"steer");
            if(io::as_string(field(steer,"controller"),"steer.controller")!="off" &&
               !rider::builtin_steer_profile(io::as_string(field(steer,"profile_id"),"steer.profile_id"))) {
                auto config=spatial::spatial_config_from_wire(field(metadata,"spatial"));
                // The resolver checks both the saved content hash and the
                // cap/delay contract; a changed or missing file fails closed.
                (void)runtime::resolved_steer_profile(config.steer);
            }
        }
        const bool source_changed = identity.native_source_sha256 !=
            io::as_string(field(metadata, "model_source_sha256"), "model_source_sha256");
        const bool config_changed = configuration_digest(resolved) !=
            io::as_string(field(metadata, "configuration_sha256"), "configuration_sha256");
        bool execution_changed = platform::execution_identity() !=
            io::as_string(field(execution, "execution_identity"), "execution_identity") ||
            image.executable_sha256 != io::as_string(field(execution, "executable_sha256"), "executable_sha256") ||
            provider.library_sha256 != io::as_string(field(execution, "mujoco_library_sha256"), "mujoco_library_sha256") ||
            provider.numeric_provider != io::as_string(field(execution, "numeric_provider"), "numeric_provider");
        if (const auto* allocation = find(execution, "allocation_solver"))
            execution_changed = execution_changed || io::as_string(*allocation, "allocation_solver") !=
                bike::rider::allocation_numeric_identity;
        set(receipt, "source_changed_during_run", Value{source_changed});
        set(receipt, "configuration_changed_during_run", Value{config_changed});
        set(receipt, "execution_changed_during_run", Value{execution_changed});
    } catch (const std::exception& error) {
        set(receipt, "identity_check_error", text(error.what()));
    }
    return receipt;
}

io::Value research_recipe(const ResearchRequest& request, const io::Value& metadata) {
    const auto& f = request.setup.model.field;
    Value recipe{Object{{"replay_contract_version", integer(3)}, {"start_x_m", real(request.setup.start_x_m)},
        {"heightfield", Value{Object{{"nrow", integer(f.nrow)}, {"ncol", integer(f.ncol)},
            {"radius_x_m", real(f.radius_x_m)}, {"radius_y_m", real(f.radius_y_m)},
            {"elevation_m", real(f.elevation_m)}, {"base_m", real(f.base_m)}, {"datum_z_m", real(f.datum_z_m)}}}},
        {"state_signature", text("mjSTATE_INTEGRATION")}, {"execution", field(metadata, "execution")}}};
    const auto& articulated = field(field(field(metadata, "resolved_config"), "physics"), "articulated");
    for (const auto& [key, name] : {std::pair{"joint_envelope", "rider_joint_envelope.json"},
                                  std::pair{"joint_strength", "rider_joint_strength.json"}})
        if (const auto* path = find(articulated, std::string(key) + "_path"); path && !io::is_null(*path))
            set(recipe, std::string(key) + "_file", text(name));
    return recipe;
}
runtime::ResearchAssets snapshot_research_assets(const ResearchRequest& request,
    const initialization::ResearchStartup& startup, const io::Value& metadata, const runtime::RuntimeBootstrap& bootstrap) {
    if (bootstrap.config.spatial) {
        auto result = snapshot_spatial_assets(request.setup,bootstrap,metadata);
        result.push_back(asset("model.mjb","application/x-mujoco",bootstrap.model.mjb));
        result.push_back(text_asset("resolved_config.json","application/json",io::recording_json(field(metadata,"resolved_config"))+"\n"));
        const auto& articulated = field(field(field(metadata,"resolved_config"),"physics"),"articulated");
        for (const auto& [key,name] : {std::pair{"joint_envelope","rider_joint_envelope.json"},std::pair{"joint_strength","rider_joint_strength.json"}}) {
            const auto* path=find(articulated,std::string(key)+"_path");
            if (path && !io::is_null(*path)) {
                auto profile=asset(name,"application/json",io::read_numpy_bytes(io::as_string(*path,key)));
                if (profile.sha256!=io::as_string(field(field(metadata,"resolved_config"),std::string(key)+"_sha256"),key))
                    throw std::invalid_argument(std::string(key)+": profile changed during capture");
                result.push_back(std::move(profile));
            }
        }
        return std::make_shared<const std::vector<runtime::ResearchAsset>>(std::move(result));
    }
    (void)terrain_digest(startup.geometry); // validate before indexed interleave
    std::vector<runtime::ResearchAsset> result;
    result.push_back(text_asset("track.toml", "application/toml", terrain::dump_track(request.setup.track)));
    std::vector<double> vertices;
    vertices.reserve(startup.geometry.x.size() * 2);
    for (std::size_t i = 0; i < startup.geometry.x.size(); ++i) {
        vertices.push_back(startup.geometry.x[i]); vertices.push_back(startup.geometry.z[i]);
    }
    result.push_back(asset("terrain_vertices.npy", "application/x-npy",
        io::encode_npy(io::double_array(vertices, {startup.geometry.x.size(), 2}))));
    const auto& resolved = field(metadata, "resolved_config");
    result.push_back(text_asset("resolved_config.json", "application/json", io::recording_json(resolved) + "\n"));
    const auto& articulated = field(field(resolved, "physics"), "articulated");
    for (const auto& [key, name] : {std::pair{"joint_envelope", "rider_joint_envelope.json"},
                                  std::pair{"joint_strength", "rider_joint_strength.json"}}) {
        const auto* path = find(articulated, std::string(key) + "_path");
        if (!path || io::is_null(*path)) continue;
        auto profile = asset(name, "application/json", io::read_numpy_bytes(io::as_string(*path, key)));
        if (profile.sha256 != io::as_string(field(resolved, std::string(key) + "_sha256"), key))
            throw std::invalid_argument(std::string(key) + " changed before recording setup could snapshot it");
        result.push_back(std::move(profile));
    }
    result.push_back(text_asset("recipe.json", "application/json", io::recording_json(research_recipe(request, metadata)) + "\n"));
    return std::make_shared<const std::vector<runtime::ResearchAsset>>(std::move(result));
}
std::vector<runtime::ResearchAsset> snapshot_spatial_assets(const initialization::SetupRequest& request,
    const runtime::RuntimeBootstrap& bootstrap,const io::Value& metadata) {
    if(!bootstrap.config.spatial || !bootstrap.config.terrain3d || !bootstrap.state.spatial)
        throw std::invalid_argument("spatial assets require complete bootstrap/terrain");
    const auto& source_owner=*bootstrap.config.terrain3d;
    const auto track=terrain::dump_track(request.track);
    Value source{Object{{"kind",text(source_owner.kind()==terrain::Terrain3dKind::grid?"grid":"profile_extrusion")},
        {"sha256",text(source_owner.fingerprint_hex())},{"corridor",real(source_owner.half_width_m())},
        {"source_track_toml",source_owner.kind()==terrain::Terrain3dKind::profile_extrusion?text(track):Value{}}}};
    std::vector<runtime::ResearchAsset> out;
    out.push_back(text_asset("track.toml","application/toml",track));
    out.push_back(text_asset("spatial/terrain_source.json","application/json",io::recording_json(source)+"\n"));
    if(const auto* grid=source_owner.grid()) out.push_back(asset("spatial/terrain.terrain3d","application/x-bike-terrain3d",terrain::write_terrain3d(*grid)));
    out.push_back(text_asset("spatial/config.json","application/json",io::recording_json(field(metadata,"resolved_config"))+"\n"));
    const Value calibration{Object{{"calibration_sha256",text(bootstrap.state.spatial->calibration_sha256)},
        {"tire",field(bootstrap.config.writer_config,"tire")},{"imu",field(spatial::spatial_config_to_wire(*bootstrap.config.spatial),"imu")}}};
    out.push_back(text_asset("spatial/calibration.json","application/json",io::recording_json(calibration)+"\n"));
    const auto& steer=bootstrap.config.spatial->steer;
    if(steer.controller!=spatial::SteerController::off && !rider::builtin_steer_profile(steer.profile_id)) {
        // Preserve the bounded source bytes: canonical reserialization can
        // expand a valid near-limit source beyond the resolver's 1 MiB cap.
        // Validate this very snapshot, not a second read of the pathname.
        std::ifstream input(steer.profile_id,std::ios::binary|std::ios::ate);
        if(!input || input.tellg()<0 || input.tellg()>1024*1024)
            throw std::invalid_argument("steer profile: unreadable or oversized recording source");
        std::string content(static_cast<std::size_t>(input.tellg()),'\0');input.seekg(0);
        input.read(content.data(),static_cast<std::streamsize>(content.size()));
        if(!input || input.peek()!=std::char_traits<char>::eof())
            throw std::invalid_argument("steer profile: source changed during asset capture");
        const auto profile=rider::read_steer_profile(io::parse_json(content));
        if(profile.sha256_hex!=steer.profile_sha256)
            throw std::invalid_argument("steer profile: content changed during asset capture");
        out.push_back(text_asset("spatial/steer_profile.json","application/json",content));
    }
    if(request.model.physics && request.model.physics->articulated.shoulder_profile) {
        auto shoulder=asset("spatial/shoulder_profile.json","application/json",io::read_numpy_bytes(*request.model.physics->articulated.shoulder_profile));
        if(shoulder.sha256!=io::as_string(field(field(metadata,"resolved_config"),"shoulder_profile_asset_sha256"),"shoulder_profile_asset_sha256"))
            throw std::invalid_argument("shoulder profile changed during asset capture");
        out.push_back(std::move(shoulder));
    }
    return out;
}
runtime::ResearchAssets reseed_assets(const runtime::ResearchAssets& assets, const io::Value& metadata) {
    if (!assets) return {};
    auto copy = *assets;
    bool replaced = false;
    for (auto& value : copy)
        if (value.name == "resolved_config.json" || value.name == "spatial/config.json") {
            value = text_asset(value.name, value.media_type, io::recording_json(field(metadata, "resolved_config")) + "\n");
            replaced = true;
        }
    if (!replaced) throw std::invalid_argument("captured research assets lack resolved_config.json");
    return std::make_shared<const std::vector<runtime::ResearchAsset>>(std::move(copy));
}

std::string physical_run_dir_name(std::string_view track_name, const io::Value& metadata) {
    std::string name;
    bool invalid_run = false;
    for (const char ch : track_name) {
        const bool allowed = (ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z') ||
            (ch >= '0' && ch <= '9') || ch == '_' || ch == '.' || ch == '-';
        if (allowed) { name.push_back(ch); invalid_run = false; }
        else if (!invalid_run) { name.push_back('_'); invalid_run = true; }
    }
    const auto first = name.find_first_not_of("._");
    if (first == std::string::npos) name = "track";
    else name = name.substr(first, name.find_last_not_of("._") - first + 1).substr(0, 80);
    return name + "_" + io::as_string(field(metadata, "drive_mode"), "drive_mode") + "_" +
        io::as_string(field(metadata, "configuration_sha256"), "configuration_sha256").substr(0, 16);
}
} // namespace bike::research
