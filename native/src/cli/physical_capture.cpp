#include "physical_capture.hpp"
#include "../configuration/load.hpp"
#include "../engine_call.hpp"
#include "../initialization/research_startup.hpp"
#include "../io/hash.hpp"
#include "../io/npy.hpp"
#include "../model/export.hpp"
#include "../platform/provenance.hpp"
#include "../research/evidence.hpp"
#include "../runtime/setup_values.hpp"
#include "../terrain/trackfile.hpp"
#include "../runtime/spatial_checkpoint_internal.hpp"
#include "../tyre/profile.hpp"
#include <algorithm>
#include <cmath>
#include <memory>

namespace bike::cli {
using namespace runtime::setupvalue;
namespace {
struct DataDeleter { void operator()(mjData* data) const noexcept { mj_deleteData(data); } };
initialization::ResearchStartup compiled_snapshot(const app::PreparedSimulation& prepared) {
    const auto& bootstrap=prepared.bootstrap;
    if(io::sha256(bootstrap.model.mjb)!=bootstrap.model.sha256 || bootstrap.state.model_digest!=bootstrap.model.sha256)
        throw std::invalid_argument("recording bootstrap model digest mismatch");
    model::ModelOwner const model{engine::load_model_image(bootstrap.model.mjb)};
    if(!model)throw std::runtime_error("cannot load recording metadata model");
    runtime::verify_model_dims(bootstrap.state.model_dims,*model);
    const auto width=engine::state_size(model.get(),mjSTATE_INTEGRATION);
    if(width<0 || static_cast<std::size_t>(width)!=bootstrap.state.integration_state.size() ||
       !std::ranges::all_of(bootstrap.state.integration_state,[](double value){return std::isfinite(value);}))
        throw std::invalid_argument("recording bootstrap integration state is invalid");
    std::unique_ptr<mjData,DataDeleter> const data{engine::make_data(model.get())};
    if(!data)throw std::runtime_error("cannot allocate recording metadata state");
    engine::set_state(model.get(),data.get(),bootstrap.state.integration_state.data(),mjSTATE_INTEGRATION);
    // This detached copy computes only transforms, so its profile includes
    // the compiled float32 raster, heightfield scale and world transform.
    // Regenerating an authored profile at an arbitrary spacing loses those.
    engine::kinematics(model.get(),data.get());
    initialization::ResearchStartup result;
    if(!bootstrap.config.spatial) for(const auto& vertex:biketyre::compiled_profile_vertices(model.get(),data.get(),"terrain")) {
        result.geometry.x.push_back(vertex[0]);result.geometry.z.push_back(vertex[1]);
    }
    result.compiled_metadata=initialization::compiled_physical_metadata(*model);
    return result;
}
io::NpyArray vertices_array(const initialization::ResearchStartup& snapshot) {
    std::vector<double> points;points.reserve(2*snapshot.geometry.x.size());
    for(std::size_t i=0;i<snapshot.geometry.x.size();++i) {
        points.push_back(snapshot.geometry.x[i]);points.push_back(snapshot.geometry.z.at(i));
    }
    return io::double_array(points,{snapshot.geometry.x.size(),2});
}
} // namespace
recording::Catalog model_catalog(const runtime::ModelImage& image) {
    if(io::sha256(image.mjb)!=image.sha256)throw std::invalid_argument("recording model digest mismatch");
    model::ModelOwner const model{engine::load_model_image(image.mjb)};
    if(!model)throw std::runtime_error("cannot load recording catalog model");
    return recording::build_catalog(*model, mj_name2id(model.get(),mjOBJ_SITE,"imu_site_spatial")>=0
        ? recording::CatalogVersion::spatial2 : recording::CatalogVersion::planar1);
}
io::Value launch_metadata(const initialization::SetupRequest& request,const app::PreparedSimulation& prepared,
    const std::optional<std::string>& seed_decimal) {
    const auto snapshot=compiled_snapshot(prepared);
    auto result=research::physical_configuration_metadata(request,snapshot,prepared.bootstrap,prepared.metadata,seed_decimal);
    set(result,"track_name",text(request.track.name));
    set(result,"track",terrain::track_to_dict(request.track));set(result,"backend",text("native"));
    set(result,"equilibrium",field(prepared.metadata,"equilibrium"));
    set(result,"async_telemetry",Value{request.runtime.async_telemetry});
    return result;
}
io::Value physical_summary(runtime::NativeRideRuntime const& environment,const io::Value& metadata,std::string_view reason) {
    const auto frame=environment.render_frame();
    auto result=metadata;
    const auto accounting=research::wire_value(runtime::Wire{environment.accounting_state()});
    const auto& history=field(accounting,"history");
    set(result,"first_failure",frame.state.first_failure?research::wire_value(frame.state.first_failure->as_wire()):Value{});
    set(result,"outcome",Value{Object{{"reason",text(reason)},{"time_s",real(frame.state.time_s)},
        {"position_m",real(frame.presentation.position_m)},{"steps",integer(frame.state.step)},
        {"crashed",Value{frame.state.outcome&&frame.state.outcome->starts_with("crash")}},
        {"balance_lost",Value{frame.presentation.balance_lost_at_m.has_value()}},
        {"balance_lost_at_m",frame.presentation.balance_lost_at_m?real(*frame.presentation.balance_lost_at_m):Value{}}}});
    set(result,"energy",field(accounting,"energy"));
    set(result,"model_status",research::wire_value(runtime::Wire{frame.state.model_status}));
    set(result,"battery_energy_j",real(frame.presentation.battery_energy_j));
    set(result,"component_work_j",field(history,"work_j"));
    set(result,"airtime_threshold_s",field(history,"airtime_s"));set(result,"duration_s",field(history,"duration_s"));
    set(result,"input_identity_at_finalization",recording::input_identity_at_finalization(metadata));
    if(frame.state.spatial) {
        const auto unavailable=environment.checkpoint_unavailable_reason();
        set(result,"spatial_checkpoint",Value{Object{
            {"checkpoint_final_available",Value{!unavailable.has_value()}},
            {"checkpoint_unavailable_reason",unavailable?text(*unavailable):Value{}}}});
    }
    return result;
}
std::vector<recording::Asset> physical_assets(const initialization::SetupRequest& request,const app::PreparedSimulation& prepared,
    const io::Value& metadata) {
    if(io::as_string(field(metadata,"model_sha256"),"model_sha256")!=prepared.bootstrap.model.sha256)
        throw std::invalid_argument("recording model differs from captured launch metadata");
    std::vector<recording::Asset> assets;
    if(prepared.bootstrap.config.spatial) {
        for(auto& a:research::snapshot_spatial_assets(request,prepared.bootstrap,metadata))
            assets.push_back(recording::Asset{.name=std::move(a.name),.media_type=std::move(a.media_type),.sha256=std::move(a.sha256),.content=std::move(a.content)});
        runtime::NativeRideRuntime initial(prepared.bootstrap);
        const auto checkpoint=runtime::spatial::capture_checkpoint(initial,runtime::spatial::inactive_research_checkpoint(),
            runtime::spatial::inactive_sensor_checkpoint(),Value{Object{{"resumable",Value{false}},{"state",Value{}}}},runtime::spatial::checkpoint_identity(metadata));
        assets.push_back(recording::json_asset("spatial/checkpoint_initial.json",runtime::spatial::encode_checkpoint(checkpoint)));
        assets.push_back(recording::asset("model.mjb","application/x-mujoco",prepared.bootstrap.model.mjb));
        assets.push_back(recording::json_asset("resolved_config.json",field(metadata,"resolved_config")));
        return assets;
    }
    const auto source=terrain::dump_track(request.track);
    assets.push_back(recording::asset("track.toml","application/toml",std::vector<std::byte>(std::as_bytes(std::span{source.data(),source.size()}).begin(),std::as_bytes(std::span{source.data(),source.size()}).end())));
    const auto vertices=vertices_array(compiled_snapshot(prepared));
    if(io::sha256(vertices.bytes)!=io::as_string(field(metadata,"terrain_sha256"),"terrain_sha256"))
        throw std::invalid_argument("compiled terrain differs from captured launch metadata");
    assets.push_back(recording::asset("terrain_vertices.npy","application/x-npy",io::encode_npy(vertices)));
    assets.push_back(recording::asset("model.mjb","application/x-mujoco",prepared.bootstrap.model.mjb));
    assets.push_back(recording::asset("initial_state.npy","application/x-npy",
        io::encode_npy(io::double_array(prepared.bootstrap.state.integration_state,{prepared.bootstrap.state.integration_state.size()}))));
    assets.push_back(recording::json_asset("setup_metadata.json",prepared.metadata));
    assets.push_back(recording::json_asset("resolved_config.json",field(metadata,"resolved_config")));
    const auto& resolved=field(metadata,"resolved_config");
    const auto& articulated=field(field(resolved,"physics"),"articulated");
    for(const auto& [key,name]:{std::pair{"joint_envelope","rider_joint_envelope.json"},
                              std::pair{"joint_strength","rider_joint_strength.json"}}) {
        const auto& path=field(articulated,std::string(key)+"_path");
        if(io::is_null(path))continue;
        auto profile=recording::asset(name,"application/json",io::read_numpy_bytes(io::as_string(path,key)));
        if(profile.sha256!=io::as_string(field(resolved,std::string(key)+"_sha256"),key))
            throw std::invalid_argument(std::string(key)+" changed before recording setup could snapshot it");
        assets.push_back(std::move(profile));
    }
    return assets;
}
std::vector<recording::Asset> physical_final_assets(std::vector<recording::Asset> assets,
    const runtime::NativeRideRuntime& environment,const io::Value& metadata) {
    if(!environment.snapshot().spatial)return assets;
    if(const auto reason=environment.checkpoint_unavailable_reason()) {
        std::erase_if(assets,[](const auto& a){return a.name=="spatial/checkpoint_final.json";});
        assets.push_back(recording::json_asset("spatial/checkpoint_final.json",Value{Object{
            {"spatial_checkpoint_unavailable",integer(1)},{"replayable",Value{false}},{"reason",text(*reason)}}}));
        return assets;
    }
    const auto checkpoint=runtime::spatial::capture_checkpoint(environment,runtime::spatial::inactive_research_checkpoint(),
        runtime::spatial::inactive_sensor_checkpoint(),Value{Object{{"resumable",Value{false}},{"state",Value{}}}},runtime::spatial::checkpoint_identity(metadata));
    std::erase_if(assets,[](const auto& a){return a.name=="spatial/checkpoint_final.json";});
    assets.push_back(recording::json_asset("spatial/checkpoint_final.json",runtime::spatial::encode_checkpoint(checkpoint)));
    return assets;
}
} // namespace bike::cli
