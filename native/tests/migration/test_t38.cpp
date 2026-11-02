// Source/fixture catalog contract. Authored only; execution is pending.
#include "support/migration_test.hpp"
#include "recording/catalog.hpp"
#include "recording/value_util.hpp"
#include <array>
#include <set>

namespace {
namespace rec = bike::recording;
namespace io = bike::io;
// NOLINTNEXTLINE(misc-no-recursion) recursion mirrors the Python reference structure
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
    // 495 reviewed occurrences + the F1 closure channel pair
    // (closure_signed_j / closure_absolute_j) on channel_energy.
    // 493 = 497 minus rider_passive_power_w, rider_passive_work_step_j and
    // the passive_damping_nm / solved_passive_nm rider_control children
    // (spec Q13 removal — each was a leaf descriptor).
    bt::require(count == 493, "full reviewed catalog retains all 493 repeated/nested descriptor occurrences");
    auto golden = io::parse_json(bt::text(bt::fixture("recording/catalog_welded.json")));
    // Spec Q13: the recorded catalog still lists the removed passive-
    // damping channel leaves. Drop exactly the named entries from the
    // expected side — the golden is never regenerated.
    {
        static constexpr std::array<std::string_view, 4> removed{
            "passive_damping_nm", "solved_passive_nm",
            "rider_passive_power_w", "rider_passive_work_step_j"};
        const auto strip = [&](this auto &&self, io::Value &v) -> void {
            if (auto *arr = std::get_if<io::Value::Array>(&v.data)) {
                std::erase_if(*arr, [&](const io::Value &e) {
                    const auto *s = std::get_if<std::string>(&e.data);
                    if (!s) return false;
                    for (const std::string_view name : removed)
                        if (s->find(name) != std::string::npos) return true;
                    return false;
                });
                for (auto &e : *arr) self(e);
            } else if (auto *obj = std::get_if<io::Value::Object>(&v.data))
                for (auto &[k, c] : *obj) self(c);
        };
        strip(golden);
    }
    const auto& coordinates = io::as_array(rec::value::require(rec::value::require(golden,"coordinates"),"qpos"),"qpos");
    std::vector<rec::JointCoords> joints;
    for (const auto& coordinate : coordinates) {
        const auto index = io::as_int64(rec::value::require(coordinate,"index"),"index");
        bt::require(io::as_string(rec::value::require(coordinate,"component"),"component").empty(),
            "the acquired welded fixture declares scalar joint input blocks");
        joints.push_back({.name=io::as_string(rec::value::require(coordinate,"joint"),"joint"),
            .joint_type=io::as_string(rec::value::require(coordinate,"dof_kind"),"kind") == "translation" ? "slide" : "hinge", .qposadr=index,.dofadr=index});
    }
    const auto actual = rec::build_catalog(joints);
    bt::require(io::recording_json(rec::describe_catalog(actual)) == io::recording_json(golden),
        "model coordinate input reproduces the acquired welded catalog, aliases and root inventory");
    bt::require(actual.resolve_alias("power_chain_w").value().parameter == "chain", "component templates bind exact component names"); // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture/request contract under test; .value() reports violations loudly
    bt::require(actual.resolve_alias("energy.residual_j")->canonical == "channel_energy.residual_j", "flattened paths resolve by reviewed meaning"); // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture/request contract under test; .value() reports violations loudly
    bt::require(!actual.resolve_alias("unreviewed.channel"), "unreviewed aliases never guess a column");
    bt::invalid([&] { (void)actual.channel_table("unreviewed"); });
}
void coordinates_contract() {
    const std::array<rec::JointCoords,3> blocks{{{.name="free",.joint_type="free",.qposadr=0,.dofadr=0},{.name="ball",.joint_type="ball",.qposadr=7,.dofadr=6},{.name="slide",.joint_type="slide",.qposadr=11,.dofadr=9}}};
    const auto catalog = rec::build_catalog(blocks, std::span<const rec::ScalarAlias>{});
    bt::require(catalog.coordinates().qpos.size()==12 && catalog.coordinates().dof.size()==10,
        "free/ball coordinate widths differ from DOF widths");
    bt::require(catalog.coordinates().qpos[3].unit=="1" && catalog.coordinates().force_unit(3)=="N·m" &&
        catalog.coordinates().force_unit(9)=="N", "quaternion and rotational/translational units stay distinct");
    auto invalid = blocks; invalid[1].qposadr = 8;
    bt::invalid([&] { (void)rec::build_catalog(invalid,std::span<const rec::ScalarAlias>{}); });
    const std::array<rec::ScalarAlias,1> nonscalar{{{.alias="x",.kind="qpos",.joint="free"}}};
    bt::invalid([&] { (void)rec::build_catalog(blocks,nonscalar); });
    auto field = rec::table_spec("physics").fields.front(); field.phase="unspecified";
    bt::invalid([&] { field.validate(); });
    bt::require(rec::quote("a\"b") == R"("a""b")", "SQL identifiers escape embedded quotes");
}
}
int main() { return bt::run([] { catalog_contract(); coordinates_contract(); }); }
