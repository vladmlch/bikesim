// Completion 3.31 (T53). Source: tests/reference/test_recording_values.py.
//
// Optional/typed-presence contract:
//   * value=None   -> stored NULL + leaf present flag, parent masks stay set
//   * key absent   -> stored NULL + cleared leaf present flag
//   * parent=None  -> stored NULL + cleared parent VALID flag
//   * parent absent-> stored NULL + cleared parent PRESENT and VALID flags
//   * measured flag / source_interval_id / source_time_s ride on the parent
//     VALID mask (channel_energy::encode emits them only when the parent is
//     present and non-null).
// Native: every case is an io::Value tree staged through the production
// encoders — no Python-dict-shaped shortcuts, no re-encode of serialized
// JSON. The presence-mask columns are decoded as first-class booleans.
#include "recording_test_support.hpp"
#include "recording/publish.hpp"
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace {

using namespace recording_test;
namespace detail = rec::detail;
using Object = io::Value::Object;
using rec::Recording;

Wire energy(Wire residual) {
    auto channels = energy_channels();
    auto &object = object_ref(channels, "energy");
    member_ref(object, "residual_j") = std::move(residual);
    return member_ref(channels, "energy");
}

void test_optional_leaf_and_nullable_parent_have_separate_masks() {
  bt::TemporaryDirectory const dir("values-masks");
  const double dt = 1. / 8;
  // a: residual NaN      b: residual=None   c: energy without residual_j
  // d: {energy: None}    e: {} (no energy key at all)
  auto absent_leaf = energy_channels();
  wire_erase(object_ref(absent_leaf, "energy"), "residual_j");
  const std::array<WireObject, 5> cases{
      WireObject{
          {"energy", energy(Wire{std::numeric_limits<double>::quiet_NaN()})}},
      WireObject{{"energy", energy(Wire{})}}, std::move(absent_leaf),
      WireObject{{"energy", Wire{}}}, WireObject{}};
  std::vector<SamplePtr> rows;
  rows.reserve(5);
  for (std::int64_t i = 0; i < 5; ++i)
    rows.push_back(sample_row(i, static_cast<double>(i) * dt, dt, {}, {}, {},
                              {}, cases.at(static_cast<std::size_t>(i))));
  auto capture = capture_rows(std::move(rows));
  const auto reader = sealed(rec::finalize(*capture, dir.path()).directory);
  const auto stored = reader->rows(
      "SELECT residual_j,residual_j_present,__energy_present,__energy_valid "
      "FROM channel_energy ORDER BY interval_id");
  bt::require(stored.size() == 5, "five energy rows");
  const auto expect =
      [&](std::size_t i, bool residual_present, bool energy_present,
          bool energy_valid) {
        bt::require(io::as_bool(
                        detail::field(stored[i], "residual_j_present"), "p") ==
                        residual_present &&
                    io::as_bool(detail::field(stored[i], "__energy_present"),
                                "p") == energy_present &&
                    io::as_bool(detail::field(stored[i], "__energy_valid"), "v") ==
                        energy_valid,
                    "presence masks track the authored tree");
        if (!residual_present)
          bt::require(io::is_null(detail::field(stored[i], "residual_j")),
                      "absent leaves decode as NULL");
      };
  bt::require(
      std::isnan(
          io::tagged_double(detail::field(stored[0], "residual_j"), "r")) &&
          io::as_bool(detail::field(stored[0], "residual_j_present"), "p") &&
          io::as_bool(detail::field(stored[0], "__energy_present"), "p") &&
          io::as_bool(detail::field(stored[0], "__energy_valid"), "v"),
      "NaN stays a real typed value, not a NULL");
  expect(1, true, true, true); // explicit None: present leaf, NULL value
  bt::require(io::is_null(detail::field(stored[1], "residual_j")),
              "explicit null decodes as NULL with the present flag set");
  expect(2, false, true, true);   // leaf absent inside a live parent
  expect(3, false, true, false);  // null parent: present, not valid
  expect(4, false, false, false); // absent parent: neither mask
  const auto types = reader->rows(
      "SELECT typeof(interval_id) AS id_type, typeof(qpos) AS qpos_type "
      "FROM physics LIMIT 1");
  bt::require(io::as_string(detail::field(types[0], "id_type"), "t") ==
                      "BIGINT" &&
                  io::as_string(detail::field(types[0], "qpos_type"), "t") ==
                      "DOUBLE[]",
              "typed columns keep BIGINT/DOUBLE[] storage types");
}

void test_synthetic_value_cases_cover_the_reviewed_optional_matrix() {
  bt::TemporaryDirectory const dir("values-matrix");
  SyntheticCapture options;
  options.value_cases = true;
  auto capture = synthetic_capture({}, std::move(options));
  const auto reader = sealed(rec::finalize(*capture, dir.path()).directory);
  const auto stored = reader->rows(
      "SELECT residual_j,residual_j_present,__energy_present,__energy_valid,"
      "measured,source_interval_id,source_time_s "
      "FROM channel_energy ORDER BY interval_id");
  bt::require(stored.size() == 9, "the optional matrix has nine rows");
  const auto expect_masks =
      [&](std::size_t i, bool leaf, bool present, bool valid) {
        bt::require(io::as_bool(
                        detail::field(stored[i], "residual_j_present"), "p") ==
                        leaf &&
                    io::as_bool(detail::field(stored[i], "__energy_present"),
                                "p") == present &&
                    io::as_bool(detail::field(stored[i], "__energy_valid"), "v") ==
                        valid,
                    "matrix mask combination matches the oracle");
      };
  expect_masks(0, true, true, true);
  bt::require(std::isnan(io::tagged_double(
                  detail::field(stored[0], "residual_j"), "r")),
              "NaN case");
  expect_masks(1, true, true, true);
  bt::require(io::is_null(detail::field(stored[1], "residual_j")),
              "explicit-null case");
  expect_masks(2, false, true, true);   // absent leaf
  expect_masks(3, false, true, false);  // null parent
  expect_masks(4, false, false, false); // absent parent
  // IEEE-ordered value cases: +0., -0. (equal under ==), +inf, -inf.
  const std::array<double, 4> expected{
      0., 0., std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity()};
  for (std::size_t i = 5; i < 9; ++i) {
    expect_masks(i, true, true, true);
    bt::require(io::tagged_double(detail::field(stored[i], "residual_j"),
                                  "r") == expected.at(i - 5),
                "ordered value case round-trips");
  }
    // Signed-zero fidelity is asserted on the AUTHORED row: DuckDB may or
    // may not preserve the sign bit through storage; IEEE equality does.
    const auto &authored = value_case_rows();
    const double residual = sw::number(sw::required(
        sw::object(sw::required(authored[6]->channels, "energy")),
        "residual_j"));
    bt::require(std::signbit(residual),
                "the authored -0. keeps its sign bit before encoding");
    // Valid parents carry provenance: measured + source interval + end time.
    const double dt = 1. / 8;
    for (std::size_t i = 0; i < stored.size(); ++i) {
        if (!io::as_bool(detail::field(stored[i], "__energy_valid"), "v"))
            continue;
        bt::require(io::as_bool(detail::field(stored[i], "measured"), "m") &&
                    io::as_int64(detail::field(stored[i], "source_interval_id"),
                                 "i") == static_cast<std::int64_t>(i) &&
                    io::as_double(detail::field(stored[i], "source_time_s"),
                                  "t") == static_cast<double>(i + 1) * dt,
                    "provenance columns ride the parent valid mask");
    }
}

void test_resealed_presence_contradiction_is_rejected_before_readout() {
  bt::TemporaryDirectory const dir("values-tamper");
  auto capture = capture_rows(
      {sample_row(0, 0., .00125, {}, {}, {}, {}, energy_channels())});
  const auto path = rec::finalize(*capture, dir.path()).directory;
  mutate_database(path, "UPDATE channel_energy SET residual_j_present=false");
  throws<std::invalid_argument>(
      [&] { (void)Recording(path); }, "presence",
      "a rehashed presence-flag contradiction is rejected on open");
}

void test_export_reads_each_native_batch_once_without_legacy_columns() {
    const double dt = 1. / 8;
    auto const episode = std::make_shared<runtime::OwnedNativeEpisode>();
    episode->generation = 1;
    for (int i = 0; i < 5; ++i)
        episode->samples.push_back(sample_row(i, i * dt, dt));
    const auto source = make_source(episode);
    std::vector<std::pair<std::string, std::int64_t>> calls;
    auto const sink = [&](std::string_view table,
                          runtime::EncodedTable &&piece) { // NOLINT(cppcoreguidelines-rvalue-reference-param-not-moved) callback signature is fixed by the API; the argument is consumed-read
      calls.emplace_back(std::string(table), piece.rows);
    };
    source->tables(2, sink);
    std::size_t physics_calls = 0;
    std::int64_t physics_rows = 0;
    for (const auto &[table, count] : calls) {
      bt::require(!table.starts_with("legacy_"),
                  "the spec projection emits no legacy_* table");
      if (table == "physics") {
        ++physics_calls;
        physics_rows += count;
        bt::require(count <= 2, "each batch honours the requested bound");
      }
    }
    bt::require(physics_calls == 3 && physics_rows == 5,
                "five rows are decomposed into bounded batches, each read once");
}

void test_ordinary_commands_tolerate_explicit_null_optional_drive() {
    const auto control = [] {
        return Wire{WireObject{{"motor_torque_nm", Wire{1.}},
                               {"rider_enabled", Wire{true}}}};
    };
    const auto make_episode = [&](Wire drive) {
        auto episode = std::make_shared<runtime::OwnedNativeEpisode>();
        episode->generation = 1;
        episode->samples.push_back(sample_row(
            0, 0., .001, {}, {}, {}, {},
            WireObject{{"control", control()},
                       {"drive", std::move(drive)}}));
        return episode;
    };
    const auto first_applied = [](const auto &source) {
        std::optional<runtime::EncodedTable> applied;
        auto const sink = [&](std::string_view, runtime::EncodedTable &&piece) {
          if (!applied)
            applied = std::move(piece);
        };
        source->tables(4, sink);
        return applied;
    };
    {   // explicit null drive -> NULL demand, present flag cleared
        const auto episode = make_episode(Wire{});
        const auto source = make_source(episode, synthetic_catalog(),
                                        /*ordinary=*/true);
        const auto applied = first_applied(source);
        bt::require(applied && applied->rows == 1,
                    "commands_applied leads the first batch");
        const auto demand = applied.value().columns.find("front_brake_demand"); // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
        const auto flag =
            applied.value().columns.find("front_brake_demand_present"); // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
        bt::require(demand != applied.value().columns.end() && // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                    flag != applied.value().columns.end(), // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                    "optional drive demand exports a presence pair");
        bt::require(demand->second.valid.size() == 1 &&
                    demand->second.valid[0] == 0 &&
                    flag->second.b.size() == 1 && flag->second.b[0] == 0,
                    "explicit-null drive yields NULL + cleared flag");
    }
    {   // authored drive demand -> value + set flag; the native tables()
        // walk is not lazy like the oracle's generator, so the drive root
        // must satisfy channel_drive's required members — a complete
        // authored struct carries the same demand.
        const auto episode = make_episode(Wire{drive_channels(.5)});
        const auto source = make_source(episode, synthetic_catalog(), true);
        const auto applied = first_applied(source);
        const auto &demand = applied.value().columns.at("front_brake_demand"); // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
        bt::require(demand.d.size() == 1 && demand.d[0] == .5 &&
                    demand.valid.size() == 1 && demand.valid[0] == 1 &&
                    applied.value().columns.at("front_brake_demand_present").b[0] == 1, // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                    "authored drive demand keeps value + flag");
    }
    {   // missing control is a hard failure at table walk
      auto const episode = std::make_shared<runtime::OwnedNativeEpisode>();
      episode->generation = 1;
      episode->samples.push_back(
          sample_row(0, 0., .001, {}, {}, {}, {},
                     WireObject{{"drive", Wire{WireObject{}}}}));
      const auto source = make_source(episode, synthetic_catalog(), true);
      throws<std::invalid_argument>(
          [&] {
            source->tables(
                4, [](std::string_view, const runtime::EncodedTable &) {});
          },
          "ordinary applied control",
          "ordinary projection requires applied control");
    }
}

} // namespace

int main() {
    // cppcheck-suppress throwInEntryPoint // test failures propagate as exceptions into bt::run's FAIL catch — the harness entry-point contract, not an unhandled throw
    return bt::run([] {
        test_optional_leaf_and_nullable_parent_have_separate_masks();
        test_synthetic_value_cases_cover_the_reviewed_optional_matrix();
        test_resealed_presence_contradiction_is_rejected_before_readout();
        test_export_reads_each_native_batch_once_without_legacy_columns();
        test_ordinary_commands_tolerate_explicit_null_optional_drive();
    });
}
