// Source/fixture catalog contract. Authored only; execution is pending.
#include "support/fixture_test.hpp"
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
    // 487 = 493 minus the battery energy model's leaf descriptors:
    // channel_drive.{battery_energy_j, battery_empty, energy_limited,
    // electrical_power_w} and channel_energy.{electrical_work_j,
    // electrical_residual_j}.
    // 479 = 487 minus the removed elastic-chain telemetry keys:
    // channel_drive.{chain_extension_m, chain_extension_rate_mps,
    // chain_tension_n, chain_energy_j, chain_dissipation_power_w,
    // chain_power_w, freehub_energy_j, freehub_deflection_rad}.
    // 476 = 479 minus the removed rotor-freewheel telemetry keys:
    // channel_drive.{motor_freewheel_dissipation_power_w,
    // motor_freewheel_engaged, motor_freewheel_torque_nm}.
    // 474 = 476 minus the clutch-engagement loss columns:
    // channel_drive.{crank_clutch_dissipation_power_w,
    // freehub_dissipation_power_w} — engagement is closure work, never a
    // loss (AGENTS.md «Передача», 2026-11-05).
    // 480 = 474 plus the six unilateral rider support reaction channels:
    // channel_rider.{front_pedal_press_n, front_pedal_shear_n,
    // rear_pedal_press_n, rear_pedal_shear_n, saddle_support_n,
    // saddle_shear_n} (AGENTS.md «Райдер», 2026-11-06).
    // 488 = 480 plus the rider-senses channel's two identity keys and six
    // leaf fields (channel_rider_senses.{episode_id, interval_id,
    // hand_press_n, speed_mps, rear_teeth, surface_front, surface_ahead,
    // grade_ahead} — launch part 1 task 2, senses are transport only).
    // 505 = 488 plus launch part 1 task 4's reviewed additions:
    // channel_drive.{rider_front_brake_demand, rider_rear_brake_demand,
    // motor_braking}, the channel_rider_commands identity keys and nine
    // leaf fields ({episode_id, interval_id, front_brake, rear_brake,
    // shift_request, owns_shifting, source, legs_coast, motor_braking,
    // shift_blocked, riding}), and commands_applied's three new demand
    // columns ({rider_front_brake_demand, rider_rear_brake_demand,
    // motor_braking}).
    // 506 = 505 plus channel_drive.shift_rider_response (launch part 1
    // task 5's rider shift request outcome label).
    // 512 = 506 plus channel_rider_reaction's two identity keys and four
    // leaf fields (phase, predicate, effort_factor, lean_add_rad).
    // 520 added channel_rider_launch (2 identity keys + 6 leaves).
    // 521 added channel_energy.rider_end_stop_loss_step_j (launch part 4,
    // task 3's anatomical end-stop loss line).
    // 522 added channel_energy.ground_plastic_loss_step_j (Stage B
    // deformable ground plastic compaction loss line).
    bt::require(count == 522, "full reviewed catalog retains all 522 repeated/nested descriptor occurrences");
    const auto golden = io::parse_json(bt::text(bt::fixture("recording/catalog_welded.json")));
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
