// Completion 3.31 (T53). Source: tests/reference/test_recording_catalog.py.
//
// Reviewed field/coordinate catalog, presence and typed-batch encoding.
// The oracle's encode_rows/decode_rows return Python row trees; the native
// port compares the typed EncodedTable pieces directly (scalar lanes,
// validity masks, presence companions, list offsets) and — where the oracle
// rebuilt trees through decode_rows — round-trips the batch through a real
// DuckDB container (create_streams + append_pieces + SELECT), which is the
// production decode path the manifest-validated reader uses.
//
// Comparisons are self-contained contracts on literal structure — no
// platform goldens.
#include "recording/analysis_detail.hpp"
#include "recording/duckdb.hpp"
#include "recording/encoding.hpp"
#include "recording_test_support.hpp"
#include "support/temporary_directory.hpp"
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <utility>
#include <vector>

namespace {

using namespace recording_test;
namespace detail = rec::detail;
using runtime::Piece;
using runtime::EncodedTable;
using Object = io::Value::Object;
using rec::value::integer;
using rec::value::real;
using rec::value::text;

// _recording_support.energy_channels() as a canonical io::Value row for
// rec::encode_rows (the Value-tree encoding seam, not the Wire episode one).
Object energy_object() {
    return Object{
        {"energy", io::Value{Object{
            {"mechanical_energy_j", real(12.5)},
            {"elastic_energy_j", io::Value{Object{{"fork_spring", real(.25)}}}},
            {"active_work_j", real(1.5)},
            {"external_work_j", real(-.5)},
            {"loss_j", real(.25)},
            {"loss_step_j", real(.001)},
            {"solver_constraint_work_j", real(0.)},
            {"energy_scale_j", real(10.)},
            {"muscle_signed_j", real(1.)},
            {"muscle_positive_j", real(1.25)},
            {"motor_signed_j", real(.5)},
            {"motor_positive_j", real(.5)},
            {"source_positive_work_j", real(1.75)},
            {"constraint_signed_j", real(0.)},
            {"constraint_absolute_j", real(0.)},
            {"closure_signed_j", real(0.)},
            {"closure_absolute_j", real(0.)},
            {"constraint_work_ratio", real(0.)},
            {"constraint_work_ok", io::Value{true}},
            {"residual_j", real(-.001)},
            {"measured", io::Value{true}},
            {"source_interval_id", integer(0)},
            {"source_time_s", real(.001)}}}},
        {"component_work_j", io::Value{Object{{"chain", real(.1)},
                                              {"tires", real(-.2)}}}}};
}

// _energy_row(**energy): the energy/component roots with per-leaf overrides.
Object energy_row(std::vector<std::pair<std::string, io::Value>> energy = {}) {
    auto row = energy_object();
    for (auto &entry : row) if (entry.first == "energy")
        for (auto &[key, value] : energy)
            detail::set(std::get<Object>(entry.second.data), key, std::move(value));
    return row;
}

// Mutable root lookup for staged removal/overrides on canonical rows.
Object &members_of(Object &row, std::string_view root) {
    for (auto &entry : row) if (entry.first == root)
        return std::get<Object>(entry.second.data);
    throw std::invalid_argument("row root missing: " + std::string(root));
}

// The production decode path for a bare encode: build the reviewed table
// set in a scratch database, append the encoded batch and SELECT it back —
// the same engine the sealed reader uses.
std::vector<Object> round_trip(const rec::TableSpec &spec,
                               std::vector<Object> rows,
                               bt::TemporaryDirectory const &dir) {
  rec::Database const database(dir.path() / "round-trip.duckdb");
  rec::create_streams(database.connection());
  auto const batch = rec::encode_rows(spec, rows, 0);
  rec::append_pieces(database.connection(), spec, batch, 0);
  return database.connection().query("SELECT * FROM " + rec::quote(spec.name) +
                                     " ORDER BY interval_id");
}

const Piece &piece(const EncodedTable &table, std::string_view name) {
    const auto it = table.columns.find(std::string(name));
    bt::require(it != table.columns.end(),
                "encoded table owns column " + std::string(name));
    return it->second;
}
bool valid(const Piece &p, std::size_t row) {
    return p.valid.empty() || p.valid[row] != 0;
}

void test_catalog_tables_are_finite_and_fully_described() {
    const auto &catalog = synthetic_catalog();
    const std::set<std::string> expected{"physics", "forces", "channel_mass",
        "channel_energy", "channel_sensors", "channel_drive", "channel_tires",
        "channel_suspension", "channel_control", "channel_rider",
        "channel_model_status"};
    std::set<std::string> actual;
    for (const auto &spec : catalog.table_order()) {
        actual.insert(spec.name);
        bt::require(!spec.keys.empty(), spec.name + " declares identity keys");
        std::set<std::string> seen;
        for (const auto &field : spec.fields) {
            bt::require(seen.insert(field.key).second,
                        spec.name + ": unique field key " + field.key);
            bt::require(!field.description.empty() && !field.unit.empty() &&
                        !field.measurement.empty() && !field.source.empty() &&
                        !field.phase.empty(),
                        spec.name + ": fully described " + field.key);
        }
    }
    bt::require(actual == expected, "finite reviewed table inventory");
}

void test_coordinate_catalog_maps_qpos_and_dof_addresses() {
    const auto &coordinates = synthetic_catalog().coordinates();
    bt::require(coordinates.qpos.size() == 8 && coordinates.dof.size() == 8,
                "8-coordinate qpos/DOF maps");
    bt::require(coordinates.qpos[0].joint == "root_x" &&
                coordinates.qpos[0].unit == "m" &&
                coordinates.qpos[2].joint == "root_pitch" &&
                coordinates.qpos[2].unit == "rad" &&
                coordinates.dof[0].unit == "m/s" &&
                coordinates.dof[2].unit == "rad/s",
                "joint names and coordinate units");
    bt::require(coordinates.force_unit(0) == "N" &&
                coordinates.force_unit(2) == "N·m" &&
                coordinates.force_unit(5) == "N·m",
                "mixed generalized-force units follow the DOF map");
}

void test_legacy_flat_aliases_resolve_to_canonical_fields() {
    const auto &catalog = synthetic_catalog();
    struct Case { std::string_view alias, canonical; std::optional<std::int64_t> index;
                  std::optional<std::string> parameter; };
    const std::vector<Case> cases = {
        {.alias = "interval_start_s",
         .canonical = "physics.time_s",
         .index = {},
         .parameter = {}},
        {.alias = "interval_dt_s",
         .canonical = "physics.dt_s",
         .index = {},
         .parameter = {}},
        {.alias = "x_m",
         .canonical = "physics.qpos",
         .index = 0,
         .parameter = {}},
        {.alias = "pitch_rad",
         .canonical = "physics.qpos",
         .index = 2,
         .parameter = {}},
        {.alias = "speed_mps",
         .canonical = "physics.qvel",
         .index = 0,
         .parameter = {}},
        {.alias = "prestep_qpos_5",
         .canonical = "physics.qpos",
         .index = 5,
         .parameter = {}},
        {.alias = "prestep_qvel_3",
         .canonical = "physics.qvel",
         .index = 3,
         .parameter = {}},
        {.alias = "qpos.4",
         .canonical = "physics.qpos",
         .index = 4,
         .parameter = {}},
        {.alias = "qvel.7",
         .canonical = "physics.qvel",
         .index = 7,
         .parameter = {}},
        {.alias = "compiled_mass_kg",
         .canonical = "channel_mass.mass_kg",
         .index = {},
         .parameter = {}},
        {.alias = "compiled_com_y_m",
         .canonical = "channel_mass.com_m",
         .index = 1,
         .parameter = {}},
        {.alias = "kinetic_energy_j",
         .canonical = "channel_mass.kinetic_energy_j",
         .index = {},
         .parameter = {}},
        {.alias = "gravitational_energy_j",
         .canonical = "channel_mass.gravitational_energy_j",
         .index = {},
         .parameter = {}},
        {.alias = "mechanical_energy_j",
         .canonical = "channel_energy.mechanical_energy_j",
         .index = {},
         .parameter = {}},
        {.alias = "energy_residual_j",
         .canonical = "channel_energy.residual_j",
         .index = {},
         .parameter = {}},
        {.alias = "powers_w.chain",
         .canonical = "forces.power_w",
         .index = {},
         .parameter = {"chain"}},
        {.alias = "power_chain_w",
         .canonical = "forces.power_w",
         .index = {},
         .parameter = {"chain"}},
        {.alias = "chain_power_w",
         .canonical = "forces.power_w",
         .index = {},
         .parameter = {"chain"}},
        {.alias = "chain_work_j",
         .canonical = "channel_energy.component_work_j",
         .index = {},
         .parameter = {"chain"}},
        {.alias = "component_work_j.tires",
         .canonical = "channel_energy.component_work_j",
         .index = {},
         .parameter = {"tires"}},
        {.alias = "drive.cadence_rpm",
         .canonical = "channel_drive.cadence_rpm",
         .index = {},
         .parameter = {}},
        {.alias = "tires.front.normal_load_n",
         .canonical = "channel_tires.front.normal_load_n",
         .index = {},
         .parameter = {}},
        {.alias = "sensors.encoders_rad_s.crank",
         .canonical = "channel_sensors.encoder_crank_rad_s",
         .index = {},
         .parameter = {}}};
    for (const auto &expected : cases) {
        const auto resolved = catalog.resolve_alias(expected.alias);
        bt::require(resolved.has_value(),
                    "alias resolves: " + std::string(expected.alias));
        bt::require(resolved.value().canonical == expected.canonical && // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                    resolved.value().index == expected.index && // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                    resolved.value().parameter == expected.parameter, // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                    "alias target: " + std::string(expected.alias));
    }
    bt::require(!catalog.resolve_alias("not_a_field").has_value(),
                "unknown alias reports no resolution");
}

void test_r10_presence_and_exceptional_values_round_trip() {
    const bt::TemporaryDirectory dir("catalog-r10");
    const auto &spec = synthetic_catalog().table("channel_energy");
    auto absent = energy_row();
    detail::pop(members_of(absent, "energy"), "residual_j");
    std::vector<Object> rows;
    auto nan_row = energy_row({{"residual_j", real(std::numeric_limits<double>::quiet_NaN())}});
    nan_row.emplace_back("interval_id", integer(0));
    auto null_row = energy_row({{"residual_j", io::Value{}}});
    null_row.emplace_back("interval_id", integer(1));
    absent.emplace_back("interval_id", integer(2));
    rows.push_back(std::move(nan_row));
    rows.push_back(std::move(null_row));
    rows.push_back(std::move(absent));
    auto const batch = rec::encode_rows(spec, rows, 0);
    bt::require(batch.rows == 3, "three encoded rows");
    const auto &value = piece(batch, "residual_j");
    const auto &present = piece(batch, "residual_j_present");
    bt::require(std::isnan(value.d[0]) && valid(value, 0) &&
                !valid(value, 1) && !valid(value, 2),
                "NaN stays a double NaN; explicit null and absent share the NULL lane");
    bt::require(present.b == std::vector<char>({1, 1, 0}),
                "presence flag separates explicit null from absent");
    const auto decoded = round_trip(spec, rows, dir);
    bt::require(decoded.size() == 3, "round trip preserves extents");
    bt::require(std::isnan(io::tagged_double(detail::field(decoded[0], "residual_j"), "residual_j")),
                "stored NaN decodes as tagged float64");
    bt::require(io::is_null(detail::field(decoded[1], "residual_j")) &&
                io::as_bool(detail::field(decoded[1], "residual_j_present"), "present") &&
                io::is_null(detail::field(decoded[2], "residual_j")) &&
                !io::as_bool(detail::field(decoded[2], "residual_j_present"), "present"),
                "explicit null and absent keep their separate masks");
}

void test_false_zero_empty_and_signed_values_survive_encoding() {
    const bt::TemporaryDirectory dir("catalog-false");
    const auto &spec = synthetic_catalog().table("channel_energy");
    std::vector<Object> rows;
    auto row = energy_row({{"constraint_work_ok", io::Value{false}},
                           {"loss_j", real(-0.)},
                           {"elastic_energy_j", io::Value{Object{}}},
                           {"residual_j", real(0.)}});
    row.emplace_back("interval_id", integer(0));
    rows.push_back(std::move(row));
    auto const batch = rec::encode_rows(spec, rows, 0);
    bt::require(piece(batch, "constraint_work_ok").b[0] == 0,
                "false survives as a boolean zero lane");
    const auto &loss = piece(batch, "loss_j");
    bt::require(loss.d[0] == 0. && std::signbit(loss.d[0]),
                "signed zero keeps its sign bit");
    const auto &elastic = piece(batch, "elastic_energy_j");
    bt::require(elastic.kind == Piece::Kind::map && elastic.offsets[1] == 0 &&
                valid(elastic, 0),
                "empty map is an authored empty object, not absence");
    const auto decoded = round_trip(spec, rows, dir);
    bt::require(io::as_bool(detail::field(decoded[0], "constraint_work_ok"), "b") == false,
                "false decodes verbatim");
    // DuckDB's on-disk float compression may normalize the -0.0 sign bit
    // (layout-dependent); the authored sign fidelity is asserted on the
    // encoded piece above, the stored lane asserts the value itself.
    bt::require(io::as_double(detail::field(decoded[0], "loss_j"), "loss") == 0.,
                "stored -0.0 stays the authored value");
    bt::require(io::as_object(detail::field(decoded[0], "elastic_energy_j"), "map").empty() &&
                io::as_double(detail::field(decoded[0], "residual_j"), "residual") == 0.,
                "empty map and +0.0 decode verbatim");
}

void test_nonfinite_doubles_are_not_null() {
    const bt::TemporaryDirectory dir("catalog-nonfinite");
    const auto &spec = synthetic_catalog().table("channel_energy");
    std::vector<Object> rows;
    auto row = energy_row({{"mechanical_energy_j", real(std::numeric_limits<double>::infinity())},
                           {"external_work_j", real(-std::numeric_limits<double>::infinity())}});
    row.emplace_back("interval_id", integer(0));
    rows.push_back(std::move(row));
    auto const batch = rec::encode_rows(spec, rows, 0);
    const auto &mechanical = piece(batch, "mechanical_energy_j");
    const auto &external = piece(batch, "external_work_j");
    bt::require(mechanical.d[0] == std::numeric_limits<double>::infinity() &&
                external.d[0] == -std::numeric_limits<double>::infinity() &&
                valid(mechanical, 0) && valid(external, 0),
                "±inf ride the double lane with full validity");
    const auto decoded = round_trip(spec, rows, dir);
    bt::require(io::tagged_double(detail::field(decoded[0], "mechanical_energy_j"), "m")
                    == std::numeric_limits<double>::infinity() &&
                io::tagged_double(detail::field(decoded[0], "external_work_j"), "e")
                    == -std::numeric_limits<double>::infinity(),
                "nonfinite values decode as tagged float64, never NULL");
}

void test_nullable_parent_objects_carry_their_own_presence() {
    const bt::TemporaryDirectory dir("catalog-parent");
    const auto &spec = synthetic_catalog().table("channel_mass");
    auto measured = [] {
        return io::Value{Object{{"mass_kg", real(80.)},
                                {"com_m", io::Value{Value::Array{real(0.), real(0.), real(.5)}}}}};
    };
    auto endpoint = io::Value{Object{{"mass_kg", real(80.)},
                                    {"com_m", io::Value{Value::Array{real(0.), real(0.), real(.5)}}},
                                    {"kinetic_energy_j", real(2.)}}};
    auto const row_at = [&](std::int64_t id, io::Value endpoint_value,
                            bool has_endpoint) {
      Object row{{"mass", measured()}, {"interval_id", integer(id)}};
      if (has_endpoint)
        row.emplace_back("endpoint_mass", std::move(endpoint_value));
      return row;
    };
    std::vector<Object> rows{row_at(0, std::move(endpoint), true),
                             row_at(1, io::Value{}, true),
                             row_at(2, io::Value{}, false)};
    auto const batch = rec::encode_rows(spec, rows, 0);
    const auto &endpoint_present = piece(batch, "endpoint_mass_present");
    bt::require(endpoint_present.b == std::vector<char>({1, 1, 0}),
                "parent presence flag separates authored/null/absent");
    const auto &structure = piece(batch, "endpoint_mass");
    bt::require(structure.kind == Piece::Kind::structure &&
                valid(structure, 0) && !valid(structure, 1) && !valid(structure, 2),
                "struct slot: authored value, explicit null, absent");
    bt::require(structure.children.at("kinetic_energy_j").d[0] == 2.,
                "struct member survives inside the authored parent");
    const auto decoded = round_trip(spec, rows, dir);
    const auto &stored = detail::field(decoded[0], "endpoint_mass");
    bt::require(io::as_double(detail::field(stored, "mass_kg"), "mass") == 80. &&
                io::as_double(detail::field(stored, "kinetic_energy_j"), "ke") == 2. &&
                io::as_bool(detail::field(decoded[0], "endpoint_mass_present"), "p"),
                "authored struct stores its members");
    bt::require(io::is_null(detail::field(decoded[1], "endpoint_mass")) &&
                io::is_null(detail::field(decoded[2], "endpoint_mass")) &&
                io::as_bool(detail::field(decoded[1], "endpoint_mass_present"), "p") &&
                !io::as_bool(detail::field(decoded[2], "endpoint_mass_present"), "p"),
                "null parent and absent parent keep distinct masks");
}

void test_empty_lists_round_trip_separately_from_absent() {
    const bt::TemporaryDirectory dir("catalog-lists");
    const auto &spec = synthetic_catalog().table("channel_model_status");
    auto const status = [] {
      return io::Value{
          Object{{"model_valid", io::Value{true}},
                 {"first_model_violation", io::Value{}},
                 {"model_violation_counts", io::Value{Object{}}},
                 {"numerically_valid", text("not_evaluated")},
                 {"calibration_status", text("parameterized_unvalidated")}}};
    };
    std::vector<Object> rows;
    Object first{{"model_status", status()}, {"interval_id", integer(0)},
                 {"attachment_violations", io::Value{Value::Array{}}}};
    Object second{{"model_status", status()}, {"interval_id", integer(1)}};
    rows.push_back(std::move(first));
    rows.push_back(std::move(second));
    auto const batch = rec::encode_rows(spec, rows, 0);
    const auto &list = piece(batch, "attachment_violations");
    const auto &present = piece(batch, "attachment_violations_present");
    bt::require(list.kind == Piece::Kind::list_s && list.offsets[1] == 0 &&
                valid(list, 0) && !valid(list, 1),
                "empty list is an authored empty value, absent is NULL");
    bt::require(present.b == std::vector<char>({1, 0}),
                "presence flag separates the two");
    const auto decoded = round_trip(spec, rows, dir);
    bt::require(io::as_array(detail::field(decoded[0], "attachment_violations"), "v").empty() &&
                io::is_null(detail::field(decoded[1], "attachment_violations")) &&
                !io::as_bool(detail::field(decoded[1], "attachment_violations_present"), "p"),
                "empty list and absent list decode distinctly");
}

void test_vector_fields_keep_shape_axes_and_units() {
    const auto &catalog = synthetic_catalog();
    const auto *accel = catalog.table("channel_sensors").field("frame_specific_force_body_mps2");
    const auto *com = catalog.table("channel_mass").field("com_m");
    const auto *qpos = catalog.table("physics").field("qpos");
    bt::require(accel && accel->shape && *accel->shape == std::vector<std::int64_t>{3} &&
                accel->axes && *accel->axes == std::vector<std::string>({"x", "y", "z"}) &&
                accel->unit == "m/s²",
                "acceleration vector keeps shape/axes/unit");
    bt::require(com && com->shape && *com->shape == std::vector<std::int64_t>{3} &&
                com->axes && *com->axes == std::vector<std::string>({"x", "y", "z"}),
                "com vector keeps shape and axes");
    bt::require(qpos && qpos->unit == "per_coordinate", "qpos is per-coordinate");
    const auto &spec = catalog.table("channel_sensors");
    Object row{{ "sensors", io::Value{Object{
        {"frame_specific_force_body_mps2", io::Value{Value::Array{real(1.), real(2.)}}},
        {"frame_gyro_body_rad_s", io::Value{Value::Array{real(0.), real(0.), real(0.)}}},
        {"encoders_rad_s", io::Value{Object{{"front_wheel", real(1.)},
                                          {"rear_wheel", real(2.)},
                                          {"crank", real(3.)}}}},
        {"motor_torque_nm", real(4.)},
        {"human_torque_nm", real(5.)}}}},
        {"interval_id", integer(0)}};
    const std::vector<Object> rows{std::move(row)};
    throws<std::invalid_argument>([&] { (void)rec::encode_rows(spec, rows, 0); },
                                  "shape", "wrong vector width is rejected");
}

void test_unknown_fields_fail_with_a_named_error() {
    const auto &catalog = synthetic_catalog();
    const auto &spec = catalog.table("channel_energy");
    auto bad = energy_row();
    members_of(bad, "energy")
        .emplace_back("definitely_not_a_channel_leaf", real(1.));
    bad.emplace_back("interval_id", integer(0));
    const std::vector<Object> rows{std::move(bad)};
    // The oracle's UnknownFieldError names the leaf member; the native
    // contract names the offending claim root ("channel_energy.energy")
    // with the same channel-members error class.
    throws<std::invalid_argument>(
        [&] { (void)rec::encode_rows(spec, rows, 0); },
        "channel_energy.energy: unknown channel members",
        "unclaimed channel member names a channel error");
    // The oracle's TreePhysicalSource walked claims and raised on the
    // unknown root. The native optimized projector defers that check to the
    // reviewed generic adapter, so the port probes encode_rows and the
    // catalog claim map — both name the root as a channel error.
    std::vector<Object> mystery{Object{
        {"mystery_domain", io::Value{Object{{"x", real(1.)}}}},
        {"interval_id", integer(0)}}};
    throws<std::invalid_argument>([&] { (void)rec::encode_rows(spec, mystery, 0); },
                                  "unknown channel root",
                                  "unknown channel root rejects the row");
    throws<std::invalid_argument>(
        [&] { (void)catalog.channel_table("mystery_domain"); },
        "unknown recording channel root", "claim lookup names the root");
    auto incomplete = energy_row();
    detail::pop(members_of(incomplete, "energy"), "mechanical_energy_j");
    incomplete.emplace_back("interval_id", integer(0));
    const std::vector<Object> missing{std::move(incomplete)};
    throws<std::invalid_argument>([&] { (void)rec::encode_rows(spec, missing, 0); },
                                  "mechanical_energy_j",
                                  "missing required leaf names the field");
}

void test_source_backed_catalog_entries_for_reviewed_semantics() {
    const auto &catalog = synthetic_catalog();
    const auto *ceiling = catalog.table("channel_drive").field("motor_limit_nm");
    const auto *setpoint = catalog.table("channel_drive").field("motor_setpoint_nm");
    const auto *request = catalog.table("channel_drive").field("motor_request_nm");
    bt::require(ceiling && setpoint && request, "drive command fields exist");
    for (const auto *f : {ceiling, setpoint, request})
        bt::require(f->unit == "N·m", std::string(f->key) + " carries torque units");
    bt::require(ceiling->description.find("ceiling") != std::string::npos,
                "ceiling description names the limit semantics");
    bt::require(setpoint->description.find("request") != std::string::npos ||
                setpoint->description.find("target") != std::string::npos,
                "setpoint description names the requested target");
    bt::require(ceiling->key != setpoint->key && setpoint->key != request->key,
                "ceiling/setpoint/request are distinct fields");
    const auto *applied = catalog.table("channel_control").field("motor_limit_nm");
    bt::require(applied && applied->description.find("ceiling") != std::string::npos,
                "applied ceiling carries the same semantics");
    for (const char *key : {"qpos", "qvel"}) {
        const auto *f = catalog.table("physics").field(key);
        bt::require(f && f->phase == "interval_start" &&
                    f->description.find("incoming") != std::string::npos,
                    std::string(key) + " is incoming interval-start state");
    }
    const auto *accel = catalog.table("channel_sensors").field("frame_specific_force_body_mps2");
    bt::require(accel && accel->unit == "m/s²" &&
                accel->description.find("specific force") != std::string::npos &&
                accel->description.find("gravity") != std::string::npos &&
                accel->phase == "interval_start",
                "specific acceleration keeps proper-acceleration semantics");
    const auto *sensed = catalog.table("channel_drive").field("human_sensor_nm");
    bt::require(sensed && sensed->unit == "N·m" &&
                sensed->description.find("crank") != std::string::npos &&
                sensed->description.find("wheel") != std::string::npos,
                "crank-side torque names wheel-side reference");
    const auto *human = catalog.table("channel_sensors").field("human_torque_nm");
    bt::require(human && human->unit == "N·m" &&
                human->description.find("crank") != std::string::npos,
                "human torque carries crank naming");
}

void test_tree_physical_source_emits_typed_batches_through_the_catalog() {
    std::vector<SamplePtr> rows;
    rows.reserve(3);
    for (int i = 0; i < 3; ++i)
      rows.push_back(sample_row(i, .001 * i));
    const auto source = make_source(
        std::make_shared<runtime::OwnedNativeEpisode>(make_episode(rows)));
    std::map<std::string, std::vector<EncodedTable>> by_table;
    source->tables(2, [&](std::string_view name, EncodedTable &&table) {
        bt::require(table.rows <= 2, "batch bound honored for " + std::string(name));
        by_table[std::string(name)].push_back(std::move(table));
    });
    auto &physics = by_table["physics"];
    bt::require(physics.size() == 2 && physics[0].rows == 2 && physics[1].rows == 1,
                "physics chunks at the bound");
    bt::require(physics[0].columns.at("interval_id").i ==
                    std::vector<std::int64_t>({0, 1}) &&
                physics[1].columns.at("interval_id").i ==
                    std::vector<std::int64_t>({2}),
                "interval ids stay typed and ordered");
    for (auto &batch : by_table["channel_energy"]) {
        const auto &ids = batch.columns.at("interval_id");
        bt::require(ids.kind == Piece::Kind::scalar_i &&
                        std::cmp_equal(ids.i.size(), batch.rows),
                    "channel interval ids are typed integers");
    }
    // Every catalog table reports a batch per window even when its channel
    // roots stay unclaimed; the truly-empty-per-table lane is the
    // zero-sample episode covered by the storage ports.
    bt::require(by_table.contains("channel_rider"),
                "an unclaimed channel table still reports its batch");
}

void test_forces_explode_per_component_with_vector_and_power() {
    const auto row = sample_row(0, 0., .001, {},
                                std::vector<double>{1., 0., 0., 0., 0., 0., 0., 0.},
                                runtime::ForceComponents{
                                    {"chain", std::vector<double>{.5, 0., 0., 0., 0., 0., 0., 0.}},
                                    {"tires", std::vector<double>{2., 0., 0., 0., 0., 0., 0., 0.}}},
                                WireObject{{"chain", Wire{.5}}, {"tires", Wire{2.}}});
    const auto source = make_source(
        std::make_shared<runtime::OwnedNativeEpisode>(make_episode({row})));
    EncodedTable forces;
    source->tables(4, [&](std::string_view name, EncodedTable &&table) {
        if (name == "forces") forces = std::move(table);
    });
    bt::require(forces.rows == 2, "one row per force component");
    const auto &component = forces.columns.at("component");
    bt::require(component.s == std::vector<std::string>({"chain", "tires"}),
                "components explode per force name");
    const auto &power = forces.columns.at("power_w");
    bt::require(power.d == std::vector<double>({.5, 2.}),
                "per-component power follows the authored map");
    const auto &qfrc = forces.columns.at("qfrc");
    bt::require(qfrc.kind == Piece::Kind::list_d &&
                qfrc.offsets == std::vector<std::int32_t>({0, 8, 16}) &&
                qfrc.d[0] == .5 && qfrc.d[8] == 2.,
                "qfrc vectors explode alongside their component");
    // The oracle's second half encodes the force_rows output through the
    // generic adapter and decodes it back — same seam here, plus the
    // storage-level decode through the DuckDB round trip.
    std::vector<Object> manual;
    for (std::size_t i = 0; i < 2; ++i) {
        const bool chain = i == 0;
        Value::Array vector;
        for (int k = 0; k < 8; ++k) vector.push_back(real(k == 0 ? (chain ? .5 : 2.) : 0.));
        manual.push_back(Object{{"interval_id", integer(0)},
                                {"component", text(chain ? "chain" : "tires")},
                                {"qfrc", io::Value{std::move(vector)}},
                                {"power_w", real(chain ? .5 : 2.)}});
    }
    auto batch = rec::encode_rows(synthetic_catalog().table("forces"), manual, 0);
    bt::require(batch.rows == 2 &&
                batch.columns.at("component").s ==
                    std::vector<std::string>({"chain", "tires"}),
                "the canonical encoder keeps component order");
    const bt::TemporaryDirectory dir("catalog-forces");
    const auto decoded = round_trip(synthetic_catalog().table("forces"),
                                    std::move(manual), dir);
    bt::require(io::as_double(detail::field(decoded[0], "power_w"), "power") == .5,
                "decoded force row keeps its power");
}

void test_tree_source_rejects_batch_arguments_and_bad_intervals() {
    const auto source = make_source(
        std::make_shared<runtime::OwnedNativeEpisode>(make_episode({sample_row(0, 0.)})));
    throws<std::invalid_argument>(
        [&] { source->tables(0, [](std::string_view, EncodedTable &&) {}); }, // NOLINT(cppcoreguidelines-rvalue-reference-param-not-moved) callback signature is fixed by the API; the argument is consumed-read
        "batch_rows", "zero batch bound is rejected");
    const auto &spec = synthetic_catalog().table("physics");
    // Physics rows are addressed by source path (end_time_s), which the
    // projection maps to the interval_end_s field key.
    Object row{{"interval_id", integer(0)},
               {"time_s", real(0.)}, {"end_time_s", real(.00125)},
               {"dt_s", real(.00125)},
               {"qpos", io::Value{Value::Array{real(1.), text("x"), real(0.), real(0.),
                                              real(0.), real(0.), real(0.), real(0.)}}},
               {"qvel", io::Value{Value::Array{real(0.), real(0.), real(0.), real(0.),
                                              real(0.), real(0.), real(0.), real(0.)}}}};
    const std::vector<Object> rows{std::move(row)};
    throws<std::invalid_argument>([&] { (void)rec::encode_rows(spec, rows, 0); },
                                  "qpos", "non-numeric interval members are rejected");
}

void test_catalog_rejects_malformed_joint_descriptors() {
  const std::vector<rec::JointCoords> nameless{
      {.name = "", .joint_type = "slide", .qposadr = 0, .dofadr = 0}};
  throws<std::invalid_argument>([&] { (void)rec::build_catalog(nameless); },
                                "joint coordinates",
                                "empty joint identity is rejected");
  const std::vector<rec::JointCoords> weird{
      {.name = "a", .joint_type = "weird", .qposadr = 0, .dofadr = 0}};
  throws<std::invalid_argument>([&] { (void)rec::build_catalog(weird); },
                                "joint type",
                                "unsupported joint type is rejected");
  const std::vector<rec::JointCoords> sparse{
      {.name = "a", .joint_type = "slide", .qposadr = 2, .dofadr = 2}};
  throws<std::invalid_argument>([&] { (void)rec::build_catalog(sparse); },
                                "dense", "non-dense addresses are rejected");
}

} // namespace

int main() {
    // cppcheck-suppress throwInEntryPoint // test failures propagate as exceptions into bt::run's FAIL catch — the harness entry-point contract, not an unhandled throw
    return bt::run([] {
        test_catalog_tables_are_finite_and_fully_described();
        test_coordinate_catalog_maps_qpos_and_dof_addresses();
        test_legacy_flat_aliases_resolve_to_canonical_fields();
        test_r10_presence_and_exceptional_values_round_trip();
        test_false_zero_empty_and_signed_values_survive_encoding();
        test_nonfinite_doubles_are_not_null();
        test_nullable_parent_objects_carry_their_own_presence();
        test_empty_lists_round_trip_separately_from_absent();
        test_vector_fields_keep_shape_axes_and_units();
        test_unknown_fields_fail_with_a_named_error();
        test_source_backed_catalog_entries_for_reviewed_semantics();
        test_tree_physical_source_emits_typed_batches_through_the_catalog();
        test_forces_explode_per_component_with_vector_and_power();
        test_tree_source_rejects_batch_arguments_and_bad_intervals();
        test_catalog_rejects_malformed_joint_descriptors();
    });
}
