#include "support/migration_test.hpp"
#include "support/temporary_directory.hpp"
#include "spatial/support/recording_fixture.hpp"
#include "recording/publish.hpp"
#include "recording/reader.hpp"
#include "recording/value_util.hpp"
#include "io/hash.hpp"
#include "io/json.hpp"
#include "rider/steer_balance.hpp"
#include "runtime/spatial_tire_factory.hpp"
#include <filesystem>
#include <fstream>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <variant>

namespace {
// An external steering profile is bundled as its exact source bytes. The
// container hash covers those bytes, the profile's own SHA covers the decoded
// coefficients, and neither depends on the original path surviving.
void external_steer_asset(const std::filesystem::path& root) {
    const auto source=root/"external-steer-profile.json";
    const auto profile=runtime::resolved_steer_profile(bike::spatial::SteerConfig{.torque_cap_nm=40.});
    const auto text=bike::io::recording_json(bike::rider::write_steer_profile(profile));
    {
        std::ofstream file(source,std::ios::binary);
        file<<text;file.close();
        bt::require(static_cast<bool>(file),"write external steer profile fixture");
    }
    auto fixture=spatial_tests::recording_fixture(source);
    const auto published=bike::recording::finalize(*fixture.capture,root/"external-steer");
    bt::require(std::filesystem::remove(source),"remove the source profile after finalize");
    bike::recording::Recording recording(published.directory);
    const auto rows=recording.rows("SELECT sha256,content FROM assets WHERE name='spatial/steer_profile.json'");
    bt::require(rows.size()==1,"an active external steering profile is bundled exactly once");
    const auto& bytes=std::get<bike::io::Bytes>(bike::recording::value::require(rows.front(),"content").data).value;
    bt::require(bike::io::sha256(bytes)==bike::io::sha256(std::as_bytes(std::span{text}))&&
        bike::io::as_string(bike::recording::value::require(rows.front(),"sha256"),"sha256")==bike::io::sha256(bytes),
        "container hash covers the exact captured source bytes");
    std::string decoded;
    for(const auto byte:bytes)decoded.push_back(static_cast<char>(byte));
    bt::require(bike::rider::read_steer_profile(bike::io::parse_recording_json(decoded)).sha256_hex==profile.sha256_hex,
        "bundled bytes keep the profile's semantic identity after the source file is gone");
}
} // namespace

int main(){return bt::run([]{
    auto fixture=spatial_tests::recording_fixture();const auto before=fixture.environment->snapshot().step;
    const auto checkpoint=bike::io::canonical_json(fixture.environment->checkpoint());
    bt::TemporaryDirectory directory("spatial-recording-assets");
    bool injected=false;bike::recording::FinalizeOptions options;
    options.before_stage=[&](std::string_view stage){if(stage=="manifest_write"&&!injected){injected=true;throw std::runtime_error("authored publication interruption");}};
    bool failed=false;try{(void)bike::recording::finalize(*fixture.capture,directory.path(),options);}catch(const std::runtime_error&){failed=true;}
    bt::require(failed&&injected,"publication fixture reaches interrupted sealing boundary");
    const auto published=bike::recording::finalize(*fixture.capture,directory.path());
    bt::require(fixture.environment->snapshot().step==before,"publication retry never steps the plant");
    bt::require(bike::io::canonical_json(fixture.environment->checkpoint())==checkpoint,"publication retry preserves sensor queues, material state, control and accounting cursors");
    bike::recording::Recording recording(published.directory);bt::require(recording.schema_version()==2,"spatial writes container2");
    std::set<std::string> names;
    for(const auto& row:recording.rows("SELECT name,sha256,content FROM assets")) {
        const auto& bytes=std::get<bike::io::Bytes>(bike::recording::value::require(row,"content").data).value;
        bt::require(bike::io::sha256(bytes)==bike::io::as_string(bike::recording::value::require(row,"sha256"),"hash"),"all assets carry actual byte hashes");
        names.insert(bike::io::as_string(bike::recording::value::require(row,"name"),"name"));
    }
    for(auto name:{"spatial/terrain_source.json","spatial/terrain.terrain3d","spatial/config.json","spatial/calibration.json","spatial/checkpoint_initial.json","spatial/checkpoint_final.json"})
        bt::require(names.contains(name),"spatial container owns every required portable asset");
    const auto coverage=recording.rows("SELECT (SELECT count(*) FROM physics) AS physics,(SELECT count(*) FROM channel_spatial) AS truth,(SELECT count(*) FROM channel_spatial_energy) AS energy,(SELECT count(*) FROM channel_spatial_geometry) AS geometry,(SELECT count(*) FROM channel_spatial_outcome) AS outcome");
    const auto& row=coverage.at(0);const auto count=bike::io::as_int64(bike::recording::value::require(row,"physics"),"physics");
    for(auto key:{"truth","energy","geometry","outcome"})bt::require(bike::io::as_int64(bike::recording::value::require(row,key),key)==count,"every saved interval has exact spatial truth/energy/geometry/outcome coverage");
    external_steer_asset(directory.path());
});}
