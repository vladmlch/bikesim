// Source-authored W45 regression; execution belongs to V08.
#include "support/migration_test.hpp"
#include "support/temporary_directory.hpp"
#include "fixture_paths.hpp"
#include "cli/application.hpp"
#include "cli/physical_capture.hpp"
#include "configuration/load.hpp"
#include "platform/files.hpp"
#include "runtime/setup_values.hpp"
#include "spatial/bootstrap_state.hpp"
#include "viewer/pacer.hpp"
#include <array>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace {
bike::cli::Arguments ride(std::initializer_list<std::string_view> args) {
    return bike::cli::parse_arguments("bike-ride",std::span(args.begin(),args.size()));
}
void contracts() {
    bt::invalid([]{(void)ride({"--spatial","sideways"});});
    bt::require(ride({"--spatial","3d"}).text("spatial")=="3d","canonical parser owns spatial enum");
    bt::invalid([]{(void)ride({"--spatial","3d","--spatial","planar"});});
    bt::invalid([]{(void)ride({"--spatial"});});
    bt::invalid([]{(void)ride({"--duration","NaN"});});
    bt::require(ride({"--threads","2"}).integer("threads")==2,"thread option is parsed as an integer");
    bt::invalid([]{(void)ride({"--threads","2","--threads","4"});});
    bt::invalid([]{(void)ride({"--threads","1.5"});});
    bt::invalid([]{(void)bike::cli::resolve_ride(ride({"--threads","2"}));});
    bt::invalid([]{(void)ride({"--terrain3d","a.toml","--terrain3d","b.toml"});});
    const bt::TemporaryDirectory output("spatial-cli");
    const auto destination=(output.path()/"should-not-exist").string();
    const auto terrain=(std::filesystem::path(BIKE_SPATIAL_SOURCE_ROOT)/"examples/spatial/flat.toml").string();
    bt::invalid([&]{(void)bike::cli::resolve_ride(ride({"--spatial","planar","--terrain3d",terrain,"--out",destination}));});
    bt::invalid([&]{(void)bike::cli::resolve_ride(ride({"--spatial","3d","--terrain3d",terrain,"--track","road_worn","--out",destination}));});
    bt::invalid([&]{(void)bike::cli::resolve_ride(ride({"--spatial","3d","--terrain3d","bad.csv","--out",destination}));});
    bt::invalid([&]{(void)bike::cli::resolve_ride(ride({"--spatial","3d","--terrain3d",terrain,"--preview","--out",destination}));});
    bt::require(std::filesystem::is_empty(output.path()),"invalid cold resolution creates no output files");
    const auto free=bike::cli::resolve_ride(ride({"--spatial","3d","--terrain3d",terrain,"--duration","2","--headless"}));
    bt::require(free->setup.model.spatial&&free->setup.model.spatial->root==bike::spatial::RootKind::free6,"explicit spatial mode reaches one SetupRequest");
    bt::require(free->setup.model.spatial->terrain.kind==bike::spatial::TerrainRequestKind::genspec,"TOML terrain maps to genspec");
    for(const auto invalid:{"0","-1","65","18446744073709551616"})
        bt::invalid([&]{(void)bike::cli::resolve_ride(ride({"--spatial","3d","--terrain3d",terrain,"--threads",invalid}));});
    const auto parallel=bike::cli::resolve_ride(ride({"--spatial","3d","--terrain3d",terrain,"--threads","4","--duration","2","--headless"}));
    bt::require(free->setup.runtime.tyre_threads==1 && parallel->setup.runtime.tyre_threads==4,
        "serial default and explicit thread policy reach the typed request");
    bt::require(parallel->effective_config_sha256==free->effective_config_sha256,
        "execution parallelism does not alter physical configuration identity");
    const auto interactive=bike::cli::resolve_ride(ride({"--spatial","3d","--terrain3d",terrain,"--duration","2"}));
    bt::require(free->effective_config_sha256==interactive->effective_config_sha256,"pacing and display do not change physical configuration identity");
    const auto config=output.path()/"resolved.toml";
    const auto bytes=bike::configuration::write_physics_toml(*free->setup.model.physics);
    bike::platform::write_exclusive(config,std::as_bytes(std::span(bytes)));
    const auto config_name=config.string();
    const auto file=bike::cli::resolve_ride(ride({"--physics-config",config_name,"--duration","2","--headless"}));
    bt::require(file->effective_config_sha256==free->effective_config_sha256,"equivalent resolved TOML and CLI have one canonical SHA-256");
    bt::require(file->setup.model.spatial.has_value(),"absent mode flag preserves the authored spatial TOML");
    bt::invalid([&]{(void)bike::cli::resolve_ride(ride({"--physics-config",config_name,"--preview","--out",destination}));});
    bt::require(!std::filesystem::exists(destination),"spatial preview rejection never publishes a placeholder planar image");
    bt::require(!bike::cli::resolve_ride(ride({"--preview"}))->setup.model.spatial,
        "planar track preview remains available");
    const auto rejects_terrain_conflict=[](auto&& resolve) {
        bool rejected=false;
        try {resolve();}
        catch(const bike::cli::ArgumentError& error) {
            rejected=std::string_view(error.what()).starts_with("spatial terrain conflicts with");
        }
        bt::require(rejected,"loaded spatial configuration rejects explicit planar terrain options at the resolver boundary");
    };
    for (const auto option : {"--track", "--length"}) {
        const auto value = std::string_view(option) == "--track" ? "road_worn" : "100";
        const auto conflicting=ride({"--physics-config",config_name,option,value});
        rejects_terrain_conflict([&]{(void)bike::cli::resolve_ride(conflicting);});
    }
    // A CLI asset replacement must discard generation-only settings inherited
    // from the previous terrain, without loading the asset during resolution.
    const auto generated = std::make_unique<bike::configuration::SimulationPhysicsConfig>(*free->setup.model.physics);
    generated->spatial->terrain.dx_m = .2;
    generated->spatial->terrain.dy_m = .1;
    const auto generated_name = (output.path()/"generated.toml").string();
    const auto generated_bytes = bike::configuration::write_physics_toml(*generated);
    bike::platform::write_exclusive(generated_name,std::as_bytes(std::span(generated_bytes)));
    const auto asset_name = (output.path()/"replacement.terrain3d").string();
    const auto asset = bike::cli::resolve_ride(ride({"--physics-config",generated_name,"--terrain3d",asset_name}));
    bt::require(asset->setup.model.spatial->terrain.kind==bike::spatial::TerrainRequestKind::grid_asset &&
        !asset->setup.model.spatial->terrain.dx_m && !asset->setup.model.spatial->terrain.dy_m,
        "immutable replacement asset has no inherited generator spacing");
    const auto planar=bike::cli::resolve_ride(ride({}));
    bt::require(!planar->setup.model.spatial&&planar->arguments.text("physics")=="legacy","empty CLI preserves the planar default");
    const std::array<std::string_view,8> research_args{"--spatial","3d","--terrain3d",terrain,"--duration","2","--rider","lumped"};
    const auto research=bike::cli::resolve_research(bike::cli::parse_arguments("bike-research",research_args));
    bt::require(research->setup.model.spatial&&research->sensors_v2.maximum_read_period_s==research->experiment.control_period_s,"research selects the V2 request with the actual control clock");
    for (const auto option : {"--track-file", "--gen-spec", "--scenario"}) {
        const auto value=std::string_view(option)=="--scenario"?"flat":"ignored.toml";
        const std::array<std::string_view,4> conflicting{"--physics-config",config_name,option,value};
        const auto parsed=bike::cli::parse_arguments("bike-research",conflicting);
        rejects_terrain_conflict([&]{(void)bike::cli::resolve_research(parsed);});
    }
}
void finalization_contracts() {
    using namespace runtime::setupvalue;
    const auto completion=bike::spatial::to_string(bike::spatial::SpatialReason::requested_completion);
    bt::require(completion=="completed","terminal encoder supplies the public completion spelling");
    Value summary{Object{{"model_status",Value{Object{{"model_valid",Value{true}},
        {"numerically_valid",Value{true}}}}},{"first_failure",Value{}},
        {"outcome",Value{Object{{"reason",text(completion)}}}}}};
    bike::viewer::PacingReport pacing;
    pacing.simulated_s=30.;pacing.wall_s=20.;pacing.presented_frames=1200;
    pacing.stopped=true;pacing.terminal=true;pacing.one_x_eligible=true;
    pacing.mean_achieved_rtf=1.5;
    const auto completed=bike::cli::viewer_performance_value(pacing,summary,{},true,true,false);
    bt::require(bike::io::as_bool(field(completed,"eligible_1x_measurement"),"eligible")&&
        bike::io::as_bool(field(completed,"target_met"),"target")&&
        !bike::io::as_bool(field(completed,"full_acceptance_verified"),"acceptance"),
        "canonical completion qualifies timing without claiming pending physical acceptance");
    bt::require(bike::cli::physical_ride_exit_code(completion,summary)==0,
        "valid completed physical ride exits successfully");
    auto status=field(summary,"model_status");
    set(status,"numerically_valid",Value{false});set(summary,"model_status",status);
    bt::require(bike::cli::physical_ride_exit_code(completion,summary)==2,
        "numerical invalidity is an invalid physical result even without model-limit counts");
    bt::require(bike::cli::physical_ride_exit_code("operator_stop",summary)==130,
        "explicit interruption keeps its exit classification");
    set(status,"numerically_valid",text("not_evaluated"));set(summary,"model_status",status);
    set(summary,"outcome",Value{Object{{"reason",text("viewer_closed")}}});
    auto initial=pacing;initial.simulated_s=0.;initial.wall_s=.01;initial.one_x_eligible=false;
    const auto unevaluated=bike::cli::viewer_performance_value(initial,summary,{},true,true,false);
    bt::require(!bike::io::as_bool(field(unevaluated,"eligible_1x_measurement"),"eligible")&&
        bike::io::is_null(field(unevaluated,"target_met")),
        "closing before numerical evaluation yields an ineligible report rather than a type error");
    set(status,"numerically_valid",Value{true});set(summary,"model_status",status);
    const auto fall=bike::spatial::to_string(bike::spatial::SpatialReason::capsize);
    set(summary,"outcome",Value{Object{{"reason",text(fall)}}});
    const auto failed=bike::cli::viewer_performance_value(pacing,summary,{},true,true,false);
    bt::require(!bike::io::as_bool(field(failed,"eligible_1x_measurement"),"eligible")&&
        bike::cli::physical_ride_exit_code(fall,summary)==1,
        "a physical fall is never requested completion");
}
void preview_availability() {
    auto frame=std::make_unique<runtime::RenderFrame>();
    frame->presentation.spatial.emplace();
    frame->state.view=frame->presentation.as_wire();
    const auto& source=runtime::sample_wire::object(runtime::sample_wire::required(frame->state.view,"preview_row"));
    bt::require(runtime::sample_wire::is_none(runtime::sample_wire::required(source,"grade_pct")),
        "the shared presentation row also marks unmeasured spatial grade unavailable");
    const bike::terrain::TrackSpec placeholder;
    const auto spatial=bike::cli::physical_preview_row(*frame,placeholder,{});
    bt::require(runtime::sample_wire::is_none(runtime::sample_wire::required(spatial,"grade_pct")),
        "a spatial preview cannot report flat grade from the unused planar placeholder");
    frame->presentation.spatial.reset();
    frame->state.view=frame->presentation.as_wire();
    const auto planar=bike::cli::physical_preview_row(*frame,placeholder,{});
    bt::require(runtime::sample_wire::number(runtime::sample_wire::required(planar,"grade_pct"))==0.,
        "the actual flat planar profile retains its zero grade");
}
}
int main(){return bt::run([]{contracts();finalization_contracts();preview_availability();});}
