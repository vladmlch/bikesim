// test_distributed_tire_reference.cpp — C++ port of
// tests/reference/test_distributed_tire_reference.py (batch 3.13).
//
// The portable distributed tyre reference: the pinned step-rig trace to
// reordering tolerance, scalar formulas bitwise-equal to the vectorised
// twins, the station-count ceiling, the state wire names/round-trip,
// scalar-only diagnostics, compute determinism and probe isolation.
#include "tyre_reference_support.hpp"

#include "configuration/physical_config.hpp"
#include "numerics/density_fit.hpp"
#include "numeric_numpy.hpp"
#include "tyre/station.hpp"
#include "validation/rigs.hpp"
#include "writers/distributed_tire.hpp"

#include <cmath>
#include <cstddef>
#include <vector>

namespace {

using namespace tyre_reference;
using bike::validation::get;
using bike::validation::real;
constexpr std::string_view kFile = "test_distributed_tire_reference.py";
namespace validation = bike::validation;
namespace numerics = bike::numerics;

// The flat raster the reference stands build — 801 points over -2..2 m.
[[nodiscard]] std::vector<std::array<double, 2>> flat_vertices() {
    std::vector<std::array<double, 2>> vertices;
    vertices.reserve(801);
    for (const double x : bike::geometry::detail::np_linspace(-2., 2., 801))
        vertices.push_back({x, 0.});
    return vertices;
}

// tests/reference/test_distributed_tire_reference.py::test_step_rig_matches_the_pinned_trace_to_reordering_tolerance
// Pinned from the vectorised implementation before the portable rewrite;
// rows[::8] compared to literal pins at 1e-9 relative tolerance.
[[gnu::noinline]] void test_step_rig_matches_the_pinned_trace_to_reordering_tolerance() {
    const std::array<double, 10> before_normal{
            632.674608364, 609.382103444, 613.516398494, 614.034816138,
            613.869405735, 613.817389269, 613.801320645, 613.703552998,
            613.341342092, 626.68744485};
    const std::array<double, 10> &before_vertical = before_normal;
    const std::array<int, 10> before_patches{9, 8, 8, 8, 8, 8, 8, 8, 9, 9};

    const auto [rows_v, metrics] = validation::distributed_road_trace(
            .00125, 128, .005, "step", .1);
    const auto &rows = io::as_array(rows_v, "rows");
    bt::require(rows.size() == 80, "step trace carries 80 rows");
    for (std::size_t j = 0; j < 10; ++j) {
        const auto &row = rows[j * 8];
        bt::require(io::as_int64(get(row, "patch_count"), "patch_count") ==
                            static_cast<std::int64_t>(before_patches[j]),
                    "patch_count matches the pinned trace");
        const double normal = real(row, "normal_load_n");
        const double vertical = real(row, "vertical_force_n");
        bt::require(std::abs(normal - before_normal[j]) <=
                            1e-9 + 1e-9 * std::abs(before_normal[j]),
                    "normal_load_n within reordering tolerance");
        bt::require(std::abs(vertical - before_vertical[j]) <=
                            1e-9 + 1e-9 * std::abs(before_vertical[j]),
                    "vertical_force_n within reordering tolerance");
    }
    bt::require(io::as_int64(get(metrics, "max_patch_count"),
                             "max_patch_count") == 9,
                "max patch count is the pinned 9");
    bt::require(real(metrics, "energy_residual_ratio") <= .05,
                "energy residual inside the rig bound");
    bt::require(real(metrics, "native_wheel_contact_count") == 0.,
                "no MuJoCo wheel contacts");
}

// tests/reference/test_distributed_tire_reference.py::test_scalar_station_formulas_equal_the_vectorised_ones_bitwise
// Capture: the four vectorised response rows; the C++ port replays the
// scalar twins per index and compares bitwise against them AND the
// native vectorised result.
[[gnu::noinline]] void test_scalar_station_formulas_equal_the_vectorised_ones_bitwise() {
    auto golden = node(
            kFile,
            "test_scalar_station_formulas_equal_the_vectorised_ones_bitwise");
    // HingeDensity() — the same default density law both wire-free
    // (numerics) and wire (nativecfg) spellings.
    const bike::configuration::HingeDensity material;
    const nativecfg::HingeDensity wire_material{
            .knots_m = material.knots_m,
            .stiffness_n_m = material.stiffness_n_m,
            .damping_ns_m = material.damping_ns_m,
            .provenance = material.provenance};
    const std::vector<double> deltas{-.01, 0., 1e-6, .003, .004, .0041,
                                     .011, .012, .02};
    const std::vector<double> rates{-.3, 0., .2, -1., .5, 0., .1, -.1, 2.};
    const std::vector<double> weights(
            deltas.size(), 2. * std::acos(-1.) / 128.);

    const auto [vec_force, vec_energy] = numerics::normal_station_response(
            deltas, rates, weights, material);
    const auto [vec_density, vec_density_energy] =
            numerics::density_response(deltas, material);

    std::vector<double> scalar_force(deltas.size()),
            scalar_energy(deltas.size()), scalar_density(deltas.size()),
            scalar_density_energy(deltas.size());
    for (std::size_t i = 0; i < deltas.size(); ++i) {
        const auto [f, e] = biketyre::station_response(
                deltas[i], rates[i], weights[i], wire_material);
        const auto [d, de] = biketyre::station_density(deltas[i],
                                                     wire_material);
        scalar_force[i] = f;
        scalar_energy[i] = e;
        scalar_density[i] = d;
        scalar_density_energy[i] = de;
        // Port-side: scalar == vectorised element bitwise, like the oracle.
        bt::require(f == vec_force[i] && e == vec_energy[i],
                    "scalar station_response equals vectorised bitwise");
        bt::require(d == vec_density[i] && de == vec_density_energy[i],
                    "scalar station_density equals vectorised bitwise");
    }
    vector(golden, scalar_force, "vec_force");
    vector(golden, scalar_energy, "vec_energy");
    vector(golden, scalar_density, "vec_density");
    vector(golden, scalar_density_energy, "vec_density_energy");
    golden.done();
}

// tests/reference/test_distributed_tire_reference.py::test_station_count_ceiling
[[gnu::noinline]] void test_station_count_ceiling() {
    const auto ceiling = biketyre::station_angles(512);
    bt::require(ceiling.angles.size() == 512 &&
                        ceiling.weights.size() == 512,
                "512 stations is the supported ceiling");
    rejects<std::invalid_argument>(
            [] { (void)biketyre::station_angles(513); }, "512");
    // DistributedTireConfig(station_count=1024) — the writer-ctor path
    // is where the range check lives natively.
    auto stand = validation::wheel_stand(.00125, 16, flat_vertices());
    const auto cfg = validation::distributed_config(1024);
    rejects<std::exception>([&] {
      DistributedTireWriter const tire(stand.stand->model.get(),
                                       stand.stand->data.get(), cfg);
    });
}

// tests/reference/test_distributed_tire_reference.py::test_state_wire_names_and_round_trip
[[gnu::noinline]] void test_state_wire_names_and_round_trip() {
    auto stand = validation::wheel_stand(.00125, 32, flat_vertices());
    auto &tire = *stand.tire;
    const auto &names = tire.state_names();
    bt::require(names.size() == static_cast<std::size_t>(2 * 32 * 4),
                "state wire has 2*32*4 names");
    bt::require(names[0] == "front.s000.xi" &&
                        names[1] == "front.s000.tangent.0" &&
                        names[2] == "front.s000.tangent.1" &&
                        names[3] == "front.s000.tangent.2" &&
                        names.back() == "rear.s031.tangent.2",
                "side-major station wire names");

    std::vector<double> row(names.size(),
                            std::numeric_limits<double>::quiet_NaN());
    const auto index = [&](std::string_view leaf) {
        return static_cast<std::size_t>(std::distance(
                names.begin(), std::ranges::find(names, leaf)));
    };
    row[index("front.s003.xi")] = .002;
    row[index("front.s003.tangent.0")] = 1.;
    row[index("front.s003.tangent.1")] = 0.;
    row[index("front.s003.tangent.2")] = 0.;
    row[index("rear.s007.xi")] = -.001;
    tire.set_state(names, row);
    const auto wire = tire.state();
    // NaN cells never satisfy operator== — the round-trip contract is
    // bitwise: same 0x7ff8… quiet-NaN payloads the oracle's np row carried.
    bt::require(std::ranges::equal(std::as_bytes(std::span{wire}),
                                   std::as_bytes(std::span{row})),
                "wire row round-trips bitwise");
    bt::require(std::isnan(wire[index("rear.s007.tangent.0")]),
                "rear.s007 keeps an all-NaN tangent");
    bt::require(std::isnan(wire[index("front.s000.xi")]),
                "inactive stations stay four NaNs");

    tire.set_state(names, wire);
    const auto rewired = tire.state();
    bt::require(std::ranges::equal(std::as_bytes(std::span{rewired}),
                                   std::as_bytes(std::span{row})),
                "re-round trip is stable");

    rejects<std::exception>([&] {
        tire.set_state(
                std::span<const std::string>(names).first(names.size() - 1),
                std::span<const double>(row).first(row.size() - 1));
    }, "rear.s031.tangent.2");
    rejects<std::exception>([&] {
        const std::array<std::string, 1> bogus{"front.s000.bogus"};
        const std::array<double, 1> cell{0.};
        tire.set_state(bogus, cell);
    }, "front.s000.bogus");
}

// tests/reference/test_distributed_tire_reference.py::test_distributed_state_arrays_round_trip_through_the_cache_codec
// DROPPED — the equilibrium cache's per-side state_tire_* array
// assembly rides a Python-only SimpleNamespace runtime shim
// (_state_arrays/restore_state in sim/ride/equilibrium_cache.py); the
// native equilibrium path reads the same document fields through
// initialization's material codec, and the wire round-trip is covered
// by test_state_wire_names_and_round_trip.

// tests/reference/test_distributed_tire_reference.py::test_diagnostics_are_scalar_only_and_multi_contact_is_informational
[[gnu::noinline]] void test_diagnostics_are_scalar_only_and_multi_contact_is_informational() {
    const auto [rows_v, metrics] = validation::distributed_road_trace(
            .00125, 64, .005, "two_support", .05);
    bt::require(io::as_int64(get(metrics, "max_patch_count"),
                             "max_patch_count") >= 2,
                "two_support yields multi-patch intervals");
    auto stand = validation::wheel_stand(.00125, 64, flat_vertices());
    auto &tire = *stand.tire;
    const auto force = tire.qfrc(stand.stand->data.get(), .00125);
    (void)force;
    const auto diag = tire.diagnostics()[0];
    bt::require(diag.has_value(), "front diagnostics published");
    bt::require(diag.value().supports_multiple_contacts && // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                        diag.value().station_count == 64, // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                "distributed diagnostics declare the capability");
}

// tests/reference/test_distributed_tire_reference.py::test_compute_is_deterministic_and_states_are_station_states
[[gnu::noinline]] void test_compute_is_deterministic_and_states_are_station_states() {
    auto a = validation::wheel_stand(.00125, 64, flat_vertices());
    auto b = validation::wheel_stand(.00125, 64, flat_vertices());
    for (auto *stand : {&a, &b}) {
        model_access::mutable_buffer(
                (*stand).stand->data->qpos, (*stand).stand->model->nq,
                "qpos")[1] = -.006;
        engine::forward((*stand).stand->model.get(),
                        (*stand).stand->data.get());
    }
    const auto fa = a.tire->qfrc(a.stand->data.get(), .00125);
    const auto fb = b.tire->qfrc(b.stand->data.get(), .00125);
    bt::require(fa == fb, "compute is deterministic");
    const auto row_a = a.tire->state(), row_b = b.tire->state();
    bt::require(row_a.size() == row_b.size(), "state rows share shape");
    // equal_nan: airborne stations keep all-NaN blocks.
    for (std::size_t i = 0; i < row_a.size(); ++i)
        bt::require(row_a[i] == row_b[i] ||
                            (std::isnan(row_a[i]) && std::isnan(row_b[i])),
                    "state rows equal with NaN tolerance");
}

// tests/reference/test_distributed_tire_reference.py::test_probe_leaves_state_and_clock_untouched
[[gnu::noinline]] void test_probe_leaves_state_and_clock_untouched() {
    auto stand = validation::wheel_stand(.00125, 64, flat_vertices());
    model_access::mutable_buffer(stand.stand->data->qpos,
                                 stand.stand->model->nq, "qpos")[1] = -.006;
    engine::forward(stand.stand->model.get(), stand.stand->data.get());
    auto &tire = *stand.tire;
    const auto force = tire.qfrc(stand.stand->data.get(), .00125);
    (void)force;
    const auto before = tire.state();
    const auto probed = tire.probe_qfrc(stand.stand->data.get(), .00125);
    (void)probed;
    const auto after = tire.state();
    for (std::size_t i = 0; i < before.size(); ++i)
        bt::require(before[i] == after[i] ||
                            (std::isnan(before[i]) && std::isnan(after[i])),
                    "probe leaves the state wire untouched");
    bt::require(tire.probe_snapshots()[0].has_value(),
                "probe snapshots publish the detached compute");
    rejects<std::invalid_argument>([&] {
        (void)tire.qfrc(stand.stand->data.get(), .00125);
    }, "once per increasing timestamp");
}

} // namespace

int main() {
    return bt::run([] {
        test_step_rig_matches_the_pinned_trace_to_reordering_tolerance();
        test_scalar_station_formulas_equal_the_vectorised_ones_bitwise();
        test_station_count_ceiling();
        test_state_wire_names_and_round_trip();
        test_diagnostics_are_scalar_only_and_multi_contact_is_informational();
        test_compute_is_deterministic_and_states_are_station_states();
        test_probe_leaves_state_and_clock_untouched();
    });
}
