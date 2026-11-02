// tyre_reference_support.hpp — shared plumbing for the batch-3.13 tyre
// and terrain reference ports:
//
//   tests/reference/test_native_tire.py
//   tests/reference/test_native_distributed_tire.py
//   tests/reference/test_native_distributed_stands.py
//   tests/reference/test_distributed_tire_reference.py
//   tests/reference/test_tyre_road_profile.py
//   tests/reference/test_savage_track_surface.py
//   tests/reference/test_terrain_placement_regression.py
//   tests/reference/test_alpine_track.py
//
// Everything here is test harness: golden-node addressing, the entry
// consumers matching the capture-side `_golden.record`/`assert_bitwise_equal`
// leaf stream the oracle emits, fixture-bundle resolution, the wheel-stand
// plant the distributed stand/reference tests build, and exception-class
// checks. No kernel behavior is duplicated — every comparison replays the
// production implementations captured by the two-platform oracle goldens.
#pragma once

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <source_location>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "support/golden.hpp"
#include "support/fixture_test.hpp"

#include "config_types.hpp"
#include "engine_call.hpp"
#include "golden/episode.hpp"
#include "io/json.hpp"
#include "io/npy.hpp"
#include "model/builder.hpp"
#include "model_access.hpp"
#include "platform/files.hpp"
#include "stepper.hpp"
#include "validation/rigs.hpp"
#include "writer_config.hpp"

namespace tyre_reference {

namespace golden_ns = bike::golden;
namespace io = bike::io;

// ---------------------------------------------------------------------
// Golden node addressing + leaf consumers
// ---------------------------------------------------------------------

[[nodiscard]] inline bt::oracle::Node node(std::string_view file,
                                           std::string_view function, // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
                                           std::string_view parameters = {}) {
    std::string name = "tests/reference/" + std::string(file) + "::" +
                       std::string(function);
    if (!parameters.empty()) name += "[" + std::string(parameters) + "]";
    return bt::oracle::Node(name);
}

// assert_bitwise_equal(scalar) / a float assert_tree leaf — 0-d <f8.
inline void scalar(bt::oracle::Node &golden, double actual,
                   std::string_view label) {
    const std::array value{actual};
    golden.f64(value, std::span<const std::size_t>{}, label);
}

// assert_bitwise_equal(row) — <f8 with the recorded C-order shape.
inline void vector(bt::oracle::Node &golden, std::span<const double> actual,
                   std::span<const std::size_t> shape,
                   std::string_view label) {
    golden.f64(actual, shape, label);
}

inline void vector(bt::oracle::Node &golden, std::span<const double> actual,
                   std::string_view label) {
    const std::array<std::size_t, 1> shape{actual.size()};
    golden.f64(actual, shape, label);
}

// np.testing.assert_allclose(rtol/atol) — the close kind.
inline void vector_close(bt::oracle::Node &golden,
                         std::span<const double> actual,
                         std::span<const std::size_t> shape,
                         std::string_view label) {
    golden.f64_close(actual, shape, label);
}

// _golden.record(np.asarray(bool)) — a 0-d |b1 entry.
inline void boolean(bt::oracle::Node &golden, bool actual,
                    std::string_view label) {
    const std::array<std::byte, 1> bytes{
            std::byte{static_cast<unsigned char>(actual ? 1 : 0)}};
    golden.bytes(bytes, "|b1", std::span<const std::size_t>{}, label);
}

// _golden.record(np.zeros(0, np.int8)) — the None/empty marker leaf.
inline void empty_marker(bt::oracle::Node &golden, std::string_view label) {
    const std::array<std::size_t, 1> shape{0};
    golden.bytes(std::span<const std::byte>{}, "|i1", shape, label);
}

// _golden.record(np.frombuffer(str.encode(), np.uint8)) — a str leaf.
inline void text(bt::oracle::Node &golden, std::string_view actual,
                 std::string_view label) {
    const std::array<std::size_t, 1> shape{actual.size()};
    golden.bytes(std::as_bytes(std::span(actual.data(), actual.size())),
                 "|u1", shape, label);
}

// _golden.record(np.asarray(<python int>)) — a 0-d <i8 entry.
inline void count(bt::oracle::Node &golden, std::int64_t actual,
                  std::string_view label) {
    std::array<std::byte, 8> bytes{};
    const auto bits = static_cast<std::uint64_t>(actual);
    for (unsigned shift = 0; shift < 64; shift += 8)
        bytes[shift / 8] =
                static_cast<std::byte>((bits >> shift) & 0xffu);
    golden.bytes(bytes, "<i8", std::span<const std::size_t>{}, label);
}

// _golden.record(np.asarray(<bool/int array>)) — a 1-d <i8 entry.
inline void i8_vector(bt::oracle::Node &golden,
                      std::span<const std::int64_t> actual,
                      std::string_view label) {
    std::vector<std::byte> bytes(actual.size() * 8);
    for (std::size_t i = 0; i < actual.size(); ++i) {
        const auto bits = static_cast<std::uint64_t>(actual[i]);
        for (unsigned shift = 0; shift < 64; shift += 8)
            bytes[i * 8 + shift / 8] =
                    static_cast<std::byte>((bits >> shift) & 0xffu);
    }
    const std::array<std::size_t, 1> shape{actual.size()};
    golden.bytes(bytes, "<i8", shape, label);
}

// _golden.record(np.asarray(<bool array>, np.uint8)?) — no; the segment /
// nonsmooth oracle rows land as <i8 (segment) or |b1 (nonsmooth).
inline void b1_vector(bt::oracle::Node &golden,
                      std::span<const bool> actual,
                      std::string_view label) {
    std::vector<std::byte> bytes(actual.size());
    for (std::size_t i = 0; i < actual.size(); ++i)
        bytes[i] = std::byte{static_cast<unsigned char>(actual[i] ? 1 : 0)};
    const std::array<std::size_t, 1> shape{actual.size()};
    golden.bytes(bytes, "|b1", shape, label);
}

// optional<double> — |i1 [0] for None else <f8 [].
inline void optional_scalar(bt::oracle::Node &golden,
                            const std::optional<double> &actual,
                            std::string_view label) {
    if (actual.has_value())
        scalar(golden, *actual, label);
    else
        empty_marker(golden, label);
}

// optional<string> — |i1 [0] for None else |u1 [n].
inline void optional_text(bt::oracle::Node &golden,
                          const std::optional<std::string> &actual,
                          std::string_view label) {
    if (actual.has_value())
        text(golden, *actual, label);
    else
        empty_marker(golden, label);
}

// f64_input — a Python-synthesized input replayed from the capture.
[[nodiscard]] inline std::vector<double>
input_vector(bt::oracle::Node &golden, std::string_view label) {
    return golden.f64_input(label);
}

// ---------------------------------------------------------------------
// Exception contract
// ---------------------------------------------------------------------

template <class Exception, class F>
void rejects(F &&action, std::string_view needle = {},
             const std::source_location &loc =
                     std::source_location::current()) {
    bool thrown = false;
    try {
        std::forward<F>(action)();
    } catch (const Exception &failure) {
        thrown = true;
        bt::require(needle.empty() ||
                            std::string_view(failure.what())
                                            .find(needle) !=
                                    std::string_view::npos,
                    (std::string("exception text lacks '") +=
                     std::string(needle)) +=
                            (std::string("':") + failure.what()));
    }
    bt::require(thrown, (std::string("expected rejection at ") +=
                         loc.file_name()) += ":" +
                                std::to_string(loc.line()));
}

// ---------------------------------------------------------------------
// Fixture bundles
// ---------------------------------------------------------------------

// tests/fixtures/golden/<bundle>/<name>/ — the platform-shared input
// bundle the C++ port reads via bt::oracle::root().
[[nodiscard]] inline std::filesystem::path fixture(std::string_view bundle,
                                                   std::string_view name) {
    return bt::oracle::root() / std::string(bundle) / std::string(name);
}

// The repository root — bt::oracle::root() is <repo>/tests/fixtures/golden.
[[nodiscard]] inline std::filesystem::path repo_root() {
    return bt::oracle::root().parent_path().parent_path().parent_path();
}

[[nodiscard]] inline std::filesystem::path repo_file(std::string_view rel) {
    return repo_root() / std::string(rel);
}

// tests/fixtures/golden/<platform>/<bundle>/<name>/ — per-platform
// runtime artifacts (episodes, settled state payloads, projected wires).
[[nodiscard]] inline std::filesystem::path
platform_fixture(std::string_view bundle, std::string_view name) {
    return bt::oracle::root() / bt::oracle::platform() /
           std::string(bundle) / std::string(name);
}

[[nodiscard]] inline std::vector<std::byte>
fixture_bytes(const std::filesystem::path &path) {
    auto bytes = io::read_numpy_bytes(path);
    bt::require(!bytes.empty(),
                "fixture artifact missing or empty: " + path.string());
    return bytes;
}

[[nodiscard]] inline std::string
fixture_text(const std::filesystem::path &path) {
    const auto bytes = fixture_bytes(path);
    return {reinterpret_cast<const char *>(bytes.data()), bytes.size()}; // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast) byte buffer viewed as chars for the I/O API; the sanctioned aliasing bridge
}

[[nodiscard]] inline nativecfg::WriterConfig
load_config(const std::filesystem::path &directory) {
    return nativecfg::decode(
            io::parse_json(fixture_text(directory / "wire.json")));
}

// ---------------------------------------------------------------------
// Golden-episode bundle — mirrors pneumatic_reference's Episode loader
// on the shared EpisodeArtifact schema.
// ---------------------------------------------------------------------

struct Episode {
    golden_ns::EpisodeArtifact artifact;
    std::vector<double> qpos, qvel, act, warmstart, time;
    std::vector<double> tire_state, forces;
    std::size_t nq = 0, nv = 0, na = 0, wire = 0, nfor = 0, fwidth = 0;
};

[[nodiscard]] inline Episode load_episode_bundle(
        const std::filesystem::path &directory) {
    Episode ep;
    ep.artifact = golden_ns::load_episode(directory);
    // Golden episodes can carry nonfinite cells; the codec reads them
    // with allow_nonfinite and the parity rows need the same.
    ep.qpos = ep.artifact.state_qpos.doubles(true);
    ep.qvel = ep.artifact.state_qvel.doubles(true);
    ep.act = ep.artifact.state_act.doubles(true);
    ep.warmstart = ep.artifact.state_warmstart.doubles(true);
    ep.time = ep.artifact.state_time.doubles(true);
    ep.tire_state = ep.artifact.tire_state.doubles(true);
    ep.forces = ep.artifact.forces.doubles(true);
    ep.nq = ep.artifact.state_qpos.shape.at(1);
    ep.nv = ep.artifact.state_qvel.shape.at(1);
    ep.na = ep.artifact.state_act.shape.size() == 2
                    ? ep.artifact.state_act.shape[1]
                    : 0;
    ep.wire = ep.artifact.tire_state.shape.at(1);
    ep.nfor = ep.artifact.forces.shape.at(1);
    ep.fwidth = ep.artifact.forces.shape.size() == 3
                        ? ep.artifact.forces.shape[2]
                        : 0;
    bt::require(ep.fwidth > 0, "episode forces is (steps,channels,nv)");
    return ep;
}

[[nodiscard]] inline std::span<const double>
row(const std::vector<double> &table, std::size_t k, std::size_t width) {
    return std::span<const double>(table).subspan(k * width, width);
}

// Owned copy of episode row k from a flattened (steps, width) NPY table.
[[nodiscard]] inline std::vector<double>
state_row(const io::NpyArray &table, std::size_t k, std::size_t width) {
    const auto cells = table.doubles(true);
    const auto span = row(cells, k, width);
    return {span.begin(), span.end()};
}

[[nodiscard]] inline std::size_t channel(
        const std::vector<std::string> &names, std::string_view leaf) {
    const auto it = std::ranges::find(names, leaf);
    bt::require(it != names.end(),
                "state wire carries " + std::string(leaf));
    return static_cast<std::size_t>(it - names.begin());
}

[[nodiscard]] inline std::size_t channel(
        std::span<const std::string> names, std::string_view leaf) {
    const auto it = std::ranges::find(names, leaf);
    bt::require(it != names.end(),
                "state wire carries " + std::string(leaf));
    return static_cast<std::size_t>(it - names.begin());
}

// ---------------------------------------------------------------------
// Snapshot leaf stream — _assert_snapshots_equal in both oracle files:
// per side 'front' then 'rear'; interval_id, backend, geometric_contact,
// time_s, effective_radius_m, patches count, then per-patch
// working_surface + the three floats.
// ---------------------------------------------------------------------
inline void snapshots(bt::oracle::Node &golden, const Stepper &st,
                      const std::string &tag) {
    const auto snaps = st.tire_writer().snapshots();
    const std::array<std::string_view, 2> sides{"front", "rear"};
    for (int side = 0; side < 2; ++side) {
        const auto &snap = snaps.at(static_cast<std::size_t>(side));
        bt::require(snap.has_value(),
                    std::string(sides.at(static_cast<std::size_t>(side))) +
                            " snapshot present");
        const auto &s = (snap).value(); // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
        const std::string base = std::string(
                sides.at(static_cast<std::size_t>(side)));
        count(golden, s.interval_id, base + ".interval_id step " + tag); // NOLINT(performance-inefficient-string-concatenation) golden label assembly mirrors the Python oracle's label building
        text(golden, s.backend, base + ".backend step " + tag); // NOLINT(performance-inefficient-string-concatenation) golden label assembly mirrors the Python oracle's label building
        boolean(golden, s.geometric_contact,
                base + ".geometric_contact step " + tag); // NOLINT(performance-inefficient-string-concatenation) golden label assembly mirrors the Python oracle's label building
        scalar(golden, s.time_s, base + ".time_s step " + tag); // NOLINT(performance-inefficient-string-concatenation) golden label assembly mirrors the Python oracle's label building
        scalar(golden, s.effective_radius_m,
               base + ".effective_radius_m step " + tag); // NOLINT(performance-inefficient-string-concatenation) golden label assembly mirrors the Python oracle's label building
        count(golden, static_cast<std::int64_t>(s.patches.size()),
              base + ".patches step " + tag); // NOLINT(performance-inefficient-string-concatenation) golden label assembly mirrors the Python oracle's label building
        for (std::size_t i = 0; i < s.patches.size(); ++i) {
            const auto &p = s.patches[i];
            const std::string pbase =
                    base + ".patches[" + std::to_string(i) + "]";
            boolean(golden, p.working_surface,
                    pbase + ".working_surface step " + tag); // NOLINT(performance-inefficient-string-concatenation) golden label assembly mirrors the Python oracle's label building
            scalar(golden, p.normal_load_n,
                   pbase + ".normal_load_n step " + tag); // NOLINT(performance-inefficient-string-concatenation) golden label assembly mirrors the Python oracle's label building
            scalar(golden, p.tangent_force_n,
                   pbase + ".tangent_force_n step " + tag); // NOLINT(performance-inefficient-string-concatenation) golden label assembly mirrors the Python oracle's label building
            scalar(golden, p.slip_mps, pbase + ".slip_mps step " + tag); // NOLINT(performance-inefficient-string-concatenation) golden label assembly mirrors the Python oracle's label building
        }
    }
}

// The distributed schema's per-side leaf order —
// distributed_tire_forces.py:204-213 / binding.cpp's is_distributed()
// branch: penetration_m, normal_load_n, multi_support,
// supports_multiple_contacts, capability_status, calibration_status,
// station_count, active_station_count, nonsmooth_station_count,
// max_patch_count, outside_material_load_range,
// outside_material_deflection_range, radial_energy_j, shear_energy_j,
// brush_loss_j.
inline void distributed_diagnostics(bt::oracle::Node &golden,
                                    const Stepper &st,
                                    const std::string &tag) {
    const auto diags = st.tire_writer().diagnostics();
    const std::array<std::string_view, 2> sides{"front", "rear"};
    for (int side = 0; side < 2; ++side) {
        const auto &slot = diags.at(static_cast<std::size_t>(side));
        bt::require(slot.has_value(),
                    std::string(sides.at(static_cast<std::size_t>(side))) +
                            " diagnostics present");
        const auto &d = (slot).value(); // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
        const auto at = [&](std::string_view key) {
            return std::string(
                           sides.at(static_cast<std::size_t>(side))) +
                   "." + std::string(key) + " step " + tag;
        };
        scalar(golden, d.penetration_m, at("penetration_m"));
        scalar(golden, d.normal_load_n, at("normal_load_n"));
        boolean(golden, d.multi_support, at("multi_support"));
        boolean(golden, d.supports_multiple_contacts,
                at("supports_multiple_contacts"));
        text(golden, d.capability_status, at("capability_status"));
        text(golden, d.calibration_status, at("calibration_status"));
        count(golden, d.station_count, at("station_count"));
        count(golden, d.active_station_count, at("active_station_count"));
        count(golden, d.nonsmooth_station_count,
              at("nonsmooth_station_count"));
        count(golden, d.max_patch_count, at("max_patch_count"));
        boolean(golden, d.outside_material_load_range,
                at("outside_material_load_range"));
        boolean(golden, d.outside_material_deflection_range,
                at("outside_material_deflection_range"));
        scalar(golden, d.radial_energy_j, at("radial_energy_j"));
        scalar(golden, d.shear_energy_j, at("shear_energy_j"));
        scalar(golden, d.brush_loss_j, at("brush_loss_j"));
    }
}

// The wheel-stand plant is shared, not duplicated: the ports call
// bike::validation::wheel_stand / distributed_config / flat_depth from
// validation/rigs.hpp (exported for this batch).

} // namespace tyre_reference
