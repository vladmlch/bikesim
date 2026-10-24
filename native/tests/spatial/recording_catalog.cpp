#include "support/migration_test.hpp"
#include "support/model_fixture.hpp"
#include "spatial/support/spatial_fixture.hpp"
#include "fixture_paths.hpp"
#include "recording/catalog.hpp"
#include "recording/encoding.hpp"
#include "recording/versioned.hpp"
#include "recording/value_util.hpp"
#include <array>
#include <filesystem>
#include <string_view>

namespace {
namespace rec=bike::recording;
using V=bike::io::Value;
#include "recording/catalog_v2_data.inc"
V::Object truth() {
    using namespace rec::value;
    return {{"root_position_world_m",doubles(std::array{1.,2.,3.})},
        {"root_quat_world_from_body_wxyz",doubles(std::array{1.,0.,0.,0.})},
        {"root_linear_velocity_world_mps",doubles(std::array{2.,0.,0.})},
        {"root_angular_velocity_world_rad_s",doubles(std::array{0.,0.,0.})},
        {"attitude_valid",V{false}},{"roll_rad",V{}},{"pitch_rad",V{}},{"heading_rad",V{}},
        {"steer_angle_rad",real(0.)},{"steer_rate_rad_s",real(0.)},{"steer_torque_nm",real(0.)},{"steer_work_j",real(0.)},
        {"lateral_displacement_m",real(2.)},{"front_contact_valid",V{false}},{"rear_contact_valid",V{false}},
        {"front_lateral_slip_mps",V{}},{"rear_lateral_slip_mps",V{}},{"front_normal_load_n",V{}},{"rear_normal_load_n",V{}},
        {"front_support_normal_world",V{}},{"rear_support_normal_world",V{}}};
}
}
int main(){return bt::run([]{
    const std::array<rec::JointCoords,1> joints{{{.name="root_free",.joint_type="free",.qposadr=0,.dofadr=0}}};
    const auto base=rec::build_catalog(joints,std::span<const rec::ScalarAlias>{});
    bt::require(base.coordinates().qpos.size()==7&&base.coordinates().dof.size()==6,"freejoint positions and tangent velocities have independent widths");
    bt::ModelFixture fixture(R"(<mujoco><worldbody><body name="frame"><freejoint name="root_free"/><geom type="sphere" size=".2" mass="2"/></body></worldbody></mujoco>)");
    const auto catalog=rec::build_catalog(*fixture.model,rec::CatalogVersion::spatial2);
    bt::require(!catalog.resolve_alias("pitch_rad")&&catalog.resolve_alias("root_free.qw").has_value(),"free root has named quaternion coordinates without scalar pitch alias");
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
});}
