// Source-authored W45 regression; execution belongs to V08.
#include "support/migration_test.hpp"
#include "support/temporary_directory.hpp"
#include "fixture_paths.hpp"
#include "cli/application.hpp"
#include "configuration/load.hpp"
#include "platform/files.hpp"
#include <array>
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
    const auto planar=bike::cli::resolve_ride(ride({}));
    bt::require(!planar->setup.model.spatial&&planar->arguments.text("physics")=="legacy","empty CLI preserves the planar default");
    const std::array<std::string_view,8> research_args{"--spatial","3d","--terrain3d",terrain,"--duration","2","--rider","lumped"};
    const auto research=bike::cli::resolve_research(bike::cli::parse_arguments("bike-research",research_args));
    bt::require(research->setup.model.spatial&&research->sensors_v2.maximum_read_period_s==research->experiment.control_period_s,"research selects the V2 request with the actual control clock");
}
}
int main(){return bt::run(contracts);}
