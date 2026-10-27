#include "support/migration_test.hpp"
#include "support/model_fixture.hpp"
#include "spatial/support/spatial_fixture.hpp"
#include "fixture_paths.hpp"
#include "recording/catalog.hpp"
#include "recording/contracts.hpp"
#include "recording/encoding.hpp"
#include "recording/versioned.hpp"
#include "recording/value_util.hpp"
#include "runtime/spatial_truth.hpp"
#include "runtime/value_readers.hpp"
#include <algorithm>
#include <array>
#include <filesystem>
#include <memory>
#include <string_view>

namespace {
namespace rec=bike::recording;
using V=bike::io::Value;
#include "recording/catalog_v2_data.inc"
// The production encoder consumes exactly this wire object; hand-writing the 21 members
// here would let the fixture and spatial_truth_to_wire drift apart unnoticed.
V::Object truth() {
    runtime::spatial::SpatialTruth value;
    value.root.position_world_m={1.,2.,3.};
    value.root.linear_velocity_world_mps={2.,0.,0.};
    value.lateral_displacement_m=2.;
    return std::get<V::Object>(runtime::spatial::spatial_truth_to_wire(value).data);
}
}
int main(){return bt::run([]{
    const std::array<rec::JointCoords,1> joints{{{.name="root_free",.joint_type="free",.qposadr=0,.dofadr=0}}};
    const auto base=rec::build_catalog(joints,std::span<const rec::ScalarAlias>{});
    bt::require(base.coordinates().qpos.size()==7&&base.coordinates().dof.size()==6,"freejoint positions and tangent velocities have independent widths");
    bt::ModelFixture fixture(R"(<mujoco><worldbody><body name="frame"><freejoint name="root_free"/><geom type="sphere" size=".2" mass="2"/>
        <body name="arm" pos="0 0 .5"><joint name="steer" type="hinge" axis="0 0 1"/><geom type="sphere" size=".1" mass="1"/></body></body></worldbody></mujoco>)");
    const auto catalog=rec::build_catalog(*fixture.model,rec::CatalogVersion::spatial2);
    bt::require(!catalog.resolve_alias("pitch_rad")&&catalog.resolve_alias("root_free.qw").has_value(),"free root has named quaternion coordinates without scalar pitch alias");
    // Tangent coordinates are named by derivative ("dx", "dwx"); a scalar joint has no
    // component, so its aliases are the bare joint name and "<joint>.d", never a trailing dot.
    const auto linear_rate=catalog.resolve_alias("root_free.dx"),angular_rate=catalog.resolve_alias("root_free.dwx"),
        hinge_position=catalog.resolve_alias("steer"),hinge_rate=catalog.resolve_alias("steer.d");
    bt::require(linear_rate&&linear_rate->canonical=="physics.qvel"&&linear_rate->index==0&&angular_rate&&angular_rate->index==3,"free root tangent aliases follow the dof order");
    bt::require(hinge_position&&hinge_position->canonical=="physics.qpos"&&hinge_position->index==7&&hinge_rate&&hinge_rate->canonical=="physics.qvel"&&hinge_rate->index==6,"scalar joint aliases bind its qpos and dof entries");
    bt::require(std::ranges::none_of(catalog.aliases(),[](const auto& alias){return alias.alias.ends_with('.');}),"no alias ends with a separator");
    bt::require(catalog.coordinates().dof.front().component=="vx","alias naming does not rewrite the stored coordinate identity");
    bt::require(catalog.channel_table("spatial").name=="channel_spatial","truth root routes to its versioned table");
    bt::require(catalog.channel_table("spatial_outcome").name=="channel_spatial_outcome","accepted outcome latch has an explicit schema2 root");
    for(const auto name:{"specific_force_sensor_mps2","angular_rate_sensor_rad_s"}) {
        const auto* field=catalog.table("channel_sensors").field(name);
        bt::require(field&&field->shape&&*field->shape==std::vector<std::int64_t>{3}&&!field->optional,"mounted sensor-frame vectors are required physical evidence");
    }
    bt::invalid([&]{(void)catalog.channel_table("unknown");});
    bt::invalid([]{(void)rec::schema_catalog(static_cast<rec::CatalogVersion>(999));});
    bt::require(bike::io::canonical_json(rec::describe_catalog(rec::schema_catalog()))==
        bike::io::canonical_json(rec::describe_catalog(rec::schema_catalog(rec::CatalogVersion::planar1))),"legacy overload retains planar catalog");
    std::string embedded;for(auto part:kCatalogV2DocumentParts)embedded+=part;
    bt::require(embedded==spatial_tests::text(std::filesystem::path(BIKE_SPATIAL_SOURCE_ROOT)/"native/resources/recording/catalog-v2.json"),"embedded and installed catalog2 have identical bytes");
    const auto& table=catalog.table("channel_spatial");
    std::array<V::Object,1> rows{{{{"interval_id",rec::value::integer(0)},{"spatial",V{truth()}}}}};
    const auto encoded=rec::encode_rows(table,rows,0);bt::require(encoded.rows==1,"one spatial row per interval");
    auto bad=rows;auto& object=std::get<V::Object>(bad[0][1].second.data);rec::value::set(object,"root_position_world_m",rec::value::doubles(std::array{1.,2.}));
    bt::invalid([&]{(void)rec::encode_rows(table,bad,0);});
    bad=rows;bad[0].emplace_back("interval_id",rec::value::integer(0));bt::invalid([&]{(void)rec::encode_rows(table,bad,0);});
    bad=rows;std::get<V::Object>(bad[0][1].second.data).emplace_back("roll_rad",V{});bt::invalid([&]{(void)rec::encode_rows(table,bad,0);});
    bad=rows;std::erase_if(std::get<V::Object>(bad[0][1].second.data),[](const auto& p){return p.first=="roll_rad";});
    bt::invalid([&]{(void)rec::encode_rows(table,bad,0);});
    bt::require(encoded.columns.at("roll_rad_present").b==std::vector<char>{1} && encoded.columns.at("roll_rad").valid==std::vector<char>{0},"undefined attitude is explicit null, never absent or plausible zero");

    // Exercise the production research source, whose transition truth is the
    // spatial packet rather than the legacy scalar WheelieTruth structure.
    auto episode=std::make_shared<runtime::OwnedResearchEpisode>();
    episode->configuration.sensors_v2.emplace();
    episode->observations.emplace_back(runtime::spatial::SensorObservationV2{});
    episode->observations.emplace_back(runtime::spatial::SensorObservationV2{.time_s=.01,.source_time_s=.01});
    runtime::ResearchTransition transition;
    transition.observation=episode->observations.back();
    runtime::spatial::SpatialTruth spatial_truth;
    spatial_truth.root.position_world_m={1.,2.,3.};
    transition.spatial_truth=runtime::sample_wire::object(vread::wire_from_value(
        runtime::spatial::spatial_truth_to_wire(spatial_truth),"test.spatial_truth"));
    transition.contact_state="flight";transition.physics_steps=1;
    episode->transitions.push_back(transition);
    std::size_t transition_rows=0;
    rec::make_research_source(episode)->tables(1,[&](std::string_view name,runtime::EncodedTable&& batch){
        if(name!="transitions")return;
        transition_rows+=static_cast<std::size_t>(batch.rows);
        const auto& packet=batch.columns.at("truth");
        bt::require(packet.children.at("root_position_world_m").d==std::vector<double>{1.,2.,3.},"research spatial truth retains world coordinates");
        bt::require(packet.children.at("roll_rad_present").b==std::vector<char>{1}&&packet.children.at("roll_rad").valid==std::vector<char>{0},"research undefined attitude preserves explicit null");
        bt::require(!packet.children.contains("root_x")&&!packet.children.contains("relative_pitch_rad"),"research spatial truth has no planar scalar surrogate");
    });
    bt::require(transition_rows==1,"one production spatial transition is encoded");
    const auto* legacy_truth=rec::table_spec("transitions").field("truth");
    bt::require(legacy_truth&&std::ranges::any_of(legacy_truth->children,[](const auto& child){return child.key=="relative_pitch_rad";}),"schema1 retains its scalar truth packet");

    bool nested_metadata=false;
    for(const auto& row:rec::catalog_rows(rec::CatalogVersion::spatial2)) {
        if(bike::io::as_string(row[0],"table")!="transitions"||bike::io::as_string(row[1],"field")!="truth")continue;
        const auto definition=bike::io::parse_recording_json(bike::io::as_string(row[9],"definition"));
        for(const auto& child:bike::io::as_array(rec::value::require(definition,"children"),"children")) {
            bt::require(bike::io::as_string(rec::value::require(child,"classification"),"classification")=="truth","nested spatial truth metadata is classified as truth");
            (void)rec::value::require(child,"name");(void)rec::value::require(child,"type");
            (void)rec::value::require(child,"nullable");(void)rec::value::require(child,"presence");
        }
        nested_metadata=true;
    }
    bt::require(nested_metadata,"schema2 publishes nested field metadata");
});}
