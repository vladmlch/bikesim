#include "support/migration_test.hpp"
#include "support/temporary_directory.hpp"
#include "spatial/support/recording_fixture.hpp"
#include "recording/reader.hpp"
#include "recording/value_util.hpp"
#include "io/hash.hpp"
#include <set>
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
});}
