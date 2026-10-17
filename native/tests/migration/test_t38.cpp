// Source/fixture catalog contract. Authored only; execution is pending.
#include "support/migration_test.hpp"
#include "recording/catalog.hpp"
#include "recording/value_util.hpp"
#include <array>
#include <set>

namespace {
namespace rec = bike::recording;
namespace io = bike::io;
std::size_t descriptors(const rec::FieldSpec& field) {
    std::size_t count = 1;
    field.validate();
    bt::require(!field.duckdb_type().empty(), "each descriptor renders a native SQL type");
    for (const auto& child : field.children) count += descriptors(child);
    return count;
}
void catalog_contract() {
    const auto schema = io::parse_json(bt::text(bt::fixture("recording/schema.json")));
    const auto& expected = io::as_array(rec::value::require(schema,"tables"),"tables");
    bt::require(rec::all_tables().size() == expected.size(), "every source physical/research table is present");
    std::size_t count = 0;
    for (std::size_t i=0; i<expected.size(); ++i) {
        const auto& spec = rec::all_tables()[i];
        bt::require(spec.name == io::as_string(expected[i],"table"), "source table ordering is stable");
        std::set<std::string> names;
        for (const auto& field : spec.keys) count += descriptors(field);
        for (const auto& field : spec.fields) count += descriptors(field);
        for (const auto& [name,type] : rec::column_types(spec)) {
            bt::require(names.insert(name).second && !type.empty(), "generated columns never collide");
        }
    }
    bt::require(count == 495, "full reviewed catalog retains all 495 repeated/nested descriptor occurrences");
    const auto golden = io::parse_json(bt::text(bt::fixture("recording/catalog_welded.json")));
    const auto& coordinates = io::as_array(rec::value::require(rec::value::require(golden,"coordinates"),"qpos"),"qpos");
    std::vector<rec::JointCoords> joints;
    for (const auto& coordinate : coordinates) {
        const auto index = io::as_int64(rec::value::require(coordinate,"index"),"index");
        bt::require(io::as_string(rec::value::require(coordinate,"component"),"component").empty(),
            "the acquired welded fixture declares scalar joint input blocks");
        joints.push_back({io::as_string(rec::value::require(coordinate,"joint"),"joint"),
            io::as_string(rec::value::require(coordinate,"dof_kind"),"kind") == "translation" ? "slide" : "hinge", index,index});
    }
    const auto actual = rec::build_catalog(joints);
    bt::require(io::recording_json(rec::describe_catalog(actual)) == io::recording_json(golden),
        "model coordinate input reproduces the acquired welded catalog, aliases and root inventory");
    bt::require(actual.resolve_alias("power_chain_w")->parameter == "chain", "component templates bind exact component names");
    bt::require(actual.resolve_alias("energy.residual_j")->canonical == "channel_energy.residual_j", "flattened paths resolve by reviewed meaning");
    bt::require(!actual.resolve_alias("unreviewed.channel"), "unreviewed aliases never guess a column");
    bt::invalid([&] { (void)actual.channel_table("unreviewed"); });
}
void coordinates_contract() {
    const std::array<rec::JointCoords,3> blocks{{{"free","free",0,0},{"ball","ball",7,6},{"slide","slide",11,9}}};
    const auto catalog = rec::build_catalog(blocks, std::span<const rec::ScalarAlias>{});
    bt::require(catalog.coordinates().qpos.size()==12 && catalog.coordinates().dof.size()==10,
        "free/ball coordinate widths differ from DOF widths");
    bt::require(catalog.coordinates().qpos[3].unit=="1" && catalog.coordinates().force_unit(3)=="N·m" &&
        catalog.coordinates().force_unit(9)=="N", "quaternion and rotational/translational units stay distinct");
    auto invalid = blocks; invalid[1].qposadr = 8;
    bt::invalid([&] { (void)rec::build_catalog(invalid,std::span<const rec::ScalarAlias>{}); });
    const std::array<rec::ScalarAlias,1> nonscalar{{{"x","qpos","free"}}};
    bt::invalid([&] { (void)rec::build_catalog(blocks,nonscalar); });
    auto field = rec::table_spec("physics").fields.front(); field.phase="unspecified";
    bt::invalid([&] { field.validate(); });
    bt::require(rec::quote("a\"b") == "\"a\"\"b\"", "SQL identifiers escape embedded quotes");
}
}
int main() { return bt::run([] { catalog_contract(); coordinates_contract(); }); }
