#include "support/migration_test.hpp"
#include "model/export.hpp"
#include "io/json.hpp"
#include <mujoco/mujoco.h>
#include <array>
#include <chrono>
#include <filesystem>
#include <ranges>
#include <string>
#include <string_view>

namespace {
namespace mdl=bike::model;
namespace cfg=bike::configuration;
namespace io=bike::io;
const io::Value& member(const io::Value& value,std::string_view name) {
    const auto* found=io::find_member(io::as_object(value,"export"),name);
    bt::require(found!=nullptr,"geometry export member "+std::string(name));return *found;
}
struct Directory {
    std::filesystem::path path=std::filesystem::temp_directory_path()/
        ("bike-t17-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    Directory() {bt::require(std::filesystem::create_directory(path),"exclusive temporary export directory");}
    ~Directory() {std::error_code ignored;std::filesystem::remove_all(path,ignored);}
};
void mode_and_rider_cases() {
    for(const char* mode:{"standard","stand","playground","ride"}) {
        mdl::BuildRequest request;request.mode=mode;request.debug_markers=true;
        const auto model=mdl::compile_mjcf(mdl::generate_mjcf(request));
        const double timestep=request.mode=="ride"?.0005:request.mode=="standard"?.002:.001;
        bt::require(model->opt.timestep==timestep,"source timestep selection for "+request.mode);
        bt::require(mj_name2id(model.get(),mjOBJ_GEOM,"marker_cg")>=0,"debug marker remains in all modes");
        bt::require((mj_name2id(model.get(),mjOBJ_EQUALITY,"stand_clamp")>=0)==(request.mode=="playground"),"playground clamp membership");
        bt::require((mj_name2id(model.get(),mjOBJ_JOINT,"root_x")>=0)==(request.mode=="ride"),"ride-only planar root");
    }
    mdl::BuildRequest request;request.mode="ride";request.include_rider=true;
    const auto lumped=mdl::compile_mjcf(mdl::generate_mjcf(request));
    request.rider=cfg::RiderSpecs{};request.rider->variant="none";
    const auto none=mdl::compile_mjcf(mdl::generate_mjcf(request));
    const int frame_id=mj_name2id(none.get(),mjOBJ_BODY,"frame");
    bt::require(frame_id>=0,"frame body exists");
    const auto frame=static_cast<std::size_t>(frame_id);
    const auto lumped_masses=std::views::counted(lumped->body_mass,lumped->nbody);
    bt::require(lumped_masses[frame]>std::views::counted(none->body_mass,none->nbody)[frame]+79.,"include_rider controls lumped mass");
    bt::require(mj_name2id(none.get(),mjOBJ_GEOM,"geom_rider_torso")>=0,"none override keeps hidden toggle geoms");
    request.include_rider=false;request.rider->variant="lumped";
    const auto override=mdl::compile_mjcf(mdl::generate_mjcf(request));
    bt::require(std::views::counted(override->body_mass,override->nbody)[frame]==lumped_masses[frame],"explicit rider overrides include_rider=false");
    request.rider=cfg::RiderSpecs{};
    const auto seated=mdl::compile_mjcf(mdl::generate_mjcf(request));
    bt::require(mj_name2id(seated.get(),mjOBJ_BODY,"rider_torso")>=0,"seated rider body tree");
    bt::require(mj_name2id(seated.get(),mjOBJ_SENSOR,"sensor_rider_pelvis_accel")>=0,"seated rider telemetry sites");
    request.rider->legs="articulated";request.crank_joint=true;
    const auto legs=mdl::compile_mjcf(mdl::generate_mjcf(request));
    bt::require(mj_name2id(legs.get(),mjOBJ_JOINT,"rider_knee_front")>=0,"legacy articulated knee");
    bt::require(mj_name2id(legs.get(),mjOBJ_EQUALITY,"weld_foot_front")>=0,"legacy articulated clipless pedal weld");
    bt::require(mj_name2id(legs.get(),mjOBJ_EQUALITY,"chain_drive")>=0,"legacy crank-wheel equality");
    request.crank_joint=false;
    const auto free_legs=mdl::compile_mjcf(mdl::generate_mjcf(request));
    bt::require(mj_name2id(free_legs.get(),mjOBJ_EQUALITY,"weld_foot_front")==-1,"no pedal body means no foot weld");
    request.rider->variant="none";request.tyre_model="pneumatic";
    const auto pneumatic=mdl::compile_mjcf(mdl::generate_mjcf(request));
    for(const char* name:{"geom_front_contact","geom_rear_contact"}) {
        const int geom_id=mj_name2id(pneumatic.get(),mjOBJ_GEOM,name);
        bt::require(geom_id>=0,"legacy contact geom exists");
        const auto geom=static_cast<std::size_t>(geom_id);
        bt::require(std::views::counted(pneumatic->geom_contype,pneumatic->ngeom)[geom]==0 && std::views::counted(pneumatic->geom_conaffinity,pneumatic->ngeom)[geom]==0,"legacy pneumatic contact disabling");
    }
    request.mode="stand";
    bt::invalid([&]{(void)mdl::generate_mjcf(request);});
    request.tyre_model="sphere";request.rider->variant="seated";
    bt::invalid([&]{(void)mdl::generate_mjcf(request);});
}
void exports_and_schema() {
    Directory directory;const auto cwd=std::filesystem::current_path();
    mdl::export_playground_models(directory.path/"models");
    constexpr std::array files{"bike_playground_stand.xml","bike_model.xml","bike_playground.xml","bike_ride.xml"};
    for(const char* name:files) {
        const auto path=directory.path/"models"/name;
        bt::require(std::filesystem::is_regular_file(path),"playground export filename "+std::string(name));
        const auto model=mdl::compile_mjcf(bt::text(path),path.parent_path());
        bt::require(model->nbody>10,"exported MJCF compiles as a complete model");
    }
    bt::require(std::filesystem::current_path()==cwd,"exports preserve process cwd");
    mdl::BuildRequest request;
    mdl::ExportOptions options{directory.path/"selected.xml",directory.path/"selected.json",true,false};
    mdl::export_model(request,options);
    bt::require(std::filesystem::is_regular_file(options.xml_file) && !std::filesystem::exists(options.json_file),"XML-only export selection");
    options.xml_only=false;options.json_only=true;options.xml_file=directory.path/"never.xml";
    mdl::export_model(request,options);
    bt::require(std::filesystem::is_regular_file(options.json_file) && !std::filesystem::exists(options.xml_file),"JSON-only export selection");
    const auto json=io::parse_json(bt::text(options.json_file));
    bt::require(io::as_object(json,"geometry").size()==8,"complete geometry export root schema");
    for(const char* key:{"uncompressed_points_mm","uncompressed_points_m","compressed_points_mm","compressed_points_m"})
        bt::require(io::as_object(member(json,key),key).size()==16,"complete fixed and moving hardpoint maps");
    bt::require(io::as_object(member(json,"link_lengths_mm"),"links").size()==15,"complete invariant metadata");
    bt::require(io::as_object(member(json,"geometry_summary"),"summary").size()==22,"complete geometry summary");
    const auto& fork=member(member(json,"damper_specifications"),"fork");
    bt::require(io::as_object(fork,"fork").size()==11,"complete fork damper metadata");
    bt::require(io::as_object(mdl::bike_specs_value(request.bike),"bike").size()==30,"complete BikeSpecs provenance");
    bt::require(io::as_object(mdl::mass_specs_value(request.masses),"masses").size()==17,"complete BikeMassSpecs provenance");
    const auto original=bt::text(options.json_file);
    request.bike.reach+=10.;mdl::export_geometry_json(request,options.json_file);
    bt::require(original!=bt::text(options.json_file),"explicit export overwrites requested destination with changed geometry");
    options.xml_only=true;
    bt::invalid([&]{mdl::export_model(request,options);});
}
} // namespace
int main() {return bt::run([]{mode_and_rider_cases();exports_and_schema();});}
