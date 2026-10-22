// Completion 3.20: 10 interval conversion and 3 live cruise timestep tests.
// The three pinned-model tyre snapshot functions remain separately planned;
// this file does not claim their integration coverage.
#include "support/model_fixture.hpp"
#include "support/golden.hpp"
#include "interval_clock.hpp"
#include "initialization/capture.hpp"
#include "random/numpy_compat.hpp"
#include "stepper.hpp"
#include "writer_config.hpp"
#include "writers/cruise.hpp"
#include <array>
#include <bit>
#include <cfenv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace {
constexpr double bound = 0x1p63;
constexpr std::int64_t largest_id = INT64_C(9223372036854774784);
constexpr double largest = 9223372036854774784.;
constexpr double inf = std::numeric_limits<double>::infinity();
constexpr double nan = std::numeric_limits<double>::quiet_NaN();
using interval_clock::interval_id;
using Pair = std::pair<double, std::int64_t>;
bt::oracle::Node oracle(std::string_view group, std::string_view symbol) {
    return bt::oracle::Node("tests/reference/test_native_clock_contracts.py::" +
        std::string(group) + "::" + std::string(symbol));
}
void integer(bt::oracle::Node& golden, std::int64_t value, std::string_view label) {
    // Explicit little-endian representation, including on a future big-endian
    // host. Interval outputs are nonnegative signed int64 values.
    std::array<std::byte, 8> bytes{};
    const auto bits = static_cast<std::uint64_t>(value);
    for (std::size_t i = 0; i < bytes.size(); ++i)
        bytes[i] = static_cast<std::byte>((bits >> (8 * i)) & 0xffU);
    golden.bytes(bytes, "<i8", {}, label);
}
void scalar(bt::oracle::Node& golden, double value) {
    golden.f64(std::span<const double>(&value, 1), std::span<const std::size_t>{}, "");
}
template<class Action> void overflow(Action&& action) {
    bool caught = false;
    try { std::forward<Action>(action)(); } catch (const std::overflow_error&) { caught = true; }
    bt::require(caught, "interval quotient outside int64 raises overflow_error");
}
struct RoundingGuard {
    int original = std::fegetround();
    RoundingGuard() { bt::require(original != -1, "fegetround supported"); }
    ~RoundingGuard() { (void)std::fesetround(original); }
    RoundingGuard(const RoundingGuard&) = delete;
    RoundingGuard& operator=(const RoundingGuard&) = delete;
    RoundingGuard(RoundingGuard&&) = delete;
    RoundingGuard& operator=(RoundingGuard&&) = delete;
};
constexpr std::array modes{FE_TONEAREST, FE_UPWARD, FE_DOWNWARD, FE_TOWARDZERO};
void prove_rounding_controls() {
    bt::require(std::fesetround(FE_TONEAREST) == 0, "set nearest rounding mode");
    const volatile double one = 1., three = 3.;
    const auto nearest = std::bit_cast<std::uint64_t>(one / three);
    bool moved = false;
    for (int const mode : modes) {
        bt::require(std::fesetround(mode) == 0, "all required rounding modes supported");
        moved = moved || std::bit_cast<std::uint64_t>(one / three) != nearest;
    }
    bt::require(moved, "foreign rounding mode changes real binary64 division");
}

void test_half_even_ties() {
    static_assert(std::is_same_v<decltype(interval_id(0., 1.)), std::int64_t>);
    for (const auto& [q, expected] : std::array{Pair{.5, 0}, Pair{1.5, 2}, Pair{2.5, 2},
        Pair{3.5, 4}, Pair{4.5, 4}, Pair{11.5, 12}, Pair{0., 0}, Pair{-0., 0},
        Pair{1., 1}, Pair{.49999999999999994, 0}, Pair{.5000000000000001, 1}})
        bt::require(interval_id(q, 1.) == expected, "half-even and neighbouring values");
}
void test_nextafter_neighbours_of_ties() {
    for (int i = 0; i < 5; ++i) {
        const double tie = static_cast<double>(i) + .5;
        bt::require(interval_id(std::nextafter(tie, 0.), 1.) == i, "below tie");
        bt::require(interval_id(std::nextafter(tie, inf), 1.) == i + 1, "above tie");
    }
}
void test_scaled_dt_quotients() {
    struct Case { double time, dt; std::int64_t expected; };
    for (const auto& c : std::array{Case{.time=1., .dt=2., .expected=0}, Case{.time=3., .dt=2., .expected=2}, Case{.time=5., .dt=2., .expected=2},
        Case{.time=7., .dt=2., .expected=4}, Case{.time=.25, .dt=.5, .expected=0}, Case{.time=.75, .dt=.5, .expected=2}, Case{.time=1.25, .dt=.5, .expected=2},
        Case{.time=1.75, .dt=.5, .expected=4}, Case{.time=.3, .dt=.1, .expected=3}, Case{.time=1., .dt=3., .expected=0}, Case{.time=2., .dt=3., .expected=1}, Case{.time=1., .dt=.00125, .expected=800}})
        bt::require(interval_id(c.time, c.dt) == c.expected, "scaled binary64 quotient");
}
void test_matches_builtin_round_on_valid_domain() {
    auto golden = oracle("TestIntervalId", "test_matches_builtin_round_on_valid_domain");
    bike::random::NumpyRng rng(41);
    std::vector<double> cases{0., 1., 1e6, largest};
    for (int i = 0; i < 300; ++i) cases.push_back(rng.uniform(0., 1e3));
    for (int i = 0; i < 300; ++i) cases.push_back(std::floor(rng.uniform(0., 1e6)) + .5);
    for (double const dt : {1., .0005, .00125, 2.5, 37.}) for (double const time : cases) {
        const double quotient = time / dt;
        if (!std::isfinite(quotient) || quotient >= bound) continue;
        integer(golden, interval_id(time, dt), "builtin round interval");
    }
    golden.done();
}
void test_time_domain_rejects_with_valueerror() {
    for (double time : {nan, inf, -inf, -1e-300, -1., -bound})
        bt::invalid([&] { (void)interval_id(time, 1.); });
}
void test_dt_domain_rejects_with_valueerror() {
    for (double dt : {0., -0., -1., nan, inf, -inf})
        bt::invalid([&] { (void)interval_id(1., dt); });
}
void test_huge_positive_dt_is_valid() {
    bt::require(interval_id(1., 1e300) == 0, "huge finite positive timestep");
    bt::require(interval_id(1., std::numeric_limits<double>::max()) == 0, "largest timestep");
}
void test_interval_overflow_is_overflowerror() {
    for (const auto& [time, dt] : std::array{std::pair{bound, 1.},
        std::pair{static_cast<double>(INT64_MAX), 1.}, std::pair{std::nextafter(bound, inf), 1.},
        std::pair{bound, std::nextafter(1., 0.)}, std::pair{1e300, 1e-300},
        std::pair{std::numeric_limits<double>::max(), 1.}})
        overflow([&] { (void)interval_id(time, dt); });
}
void test_int64_upper_edge_accepted() {
    bt::require(interval_id(largest, 1.) == largest_id, "largest representable admitted int64");
    bt::require(interval_id(9.2e18, 1.) == INT64_C(9200000000000000000), "retired approximate bound");
    bt::require(interval_id(0x1p62, 1.) == (INT64_C(1) << 62), "2**62 admitted");
}
void test_ids_are_rounding_mode_independent() {
    const int original = std::fegetround();
    {
        RoundingGuard const guard;
        prove_rounding_controls();
        auto golden = oracle("TestIntervalId", "test_ids_are_rounding_mode_independent");
        std::vector<Pair> cases{{0., 0}, {-0., 0}, {.5, 0}, {1.5, 2}, {2.5, 2}, {3.5, 4}, {4.5, 4}, {largest, largest_id}};
        for (int i = 0; i < 4; ++i) {
            cases.emplace_back(std::nextafter(static_cast<double>(i) + .5, 0.), i);
            cases.emplace_back(std::nextafter(static_cast<double>(i) + .5, inf), i + 1);
        }
        for (int const mode : modes) {
            bt::require(std::fesetround(mode) == 0, "set foreign rounding mode");
            for (const auto& [q, expected] : cases)
                bt::require(interval_id(q, 1.) == expected, "conversion independent of current FE mode");
            for (const auto& [time, dt] : std::array{std::pair{.3, .1}, std::pair{1., 3.}, std::pair{5., 2.}})
                integer(golden, interval_id(time, dt), "rounded mode quotient");
            overflow([&] { (void)interval_id(bound, 1.); });
        }
        golden.done();
    }
    bt::require(std::fegetround() == original, "original FE mode restored");
}

constexpr std::string_view cruise_xml = R"(<mujoco><option timestep="0.0005"/><worldbody><body>
  <joint name="root_x" type="slide" axis="1 0 0"/><geom type="sphere" size=".1" mass="1"/>
</body></worldbody></mujoco>)";
constexpr nativecfg::CruiseConfig cruise_config{.target_speed_kmh=25., .kp_nm_per_mps=180., .ki_nm_per_mps_s=150., .torque_ceiling_nm=150.};
void speed(bt::ModelFixture& fixture, double value) {
    model_access::mutable_buffer(fixture.data->qvel, fixture.model->nv)[static_cast<std::size_t>(fixture.dof("root_x"))] = value;
}
void test_python_oracle_reads_timestep_live() {
    bt::ModelFixture fixture(cruise_xml); CruiseWriter cruise(fixture.model.get(), cruise_config);
    auto golden = oracle("TestCruiseTimestep", "test_python_oracle_reads_timestep_live");
    speed(fixture, cruise.state().target_speed_mps - .2);
    (void)cruise.compute(fixture.data.get(), true, false, {});
    scalar(golden, cruise.state().integral_mps_s);
    fixture.model->opt.timestep = .001;
    (void)cruise.compute(fixture.data.get(), true, false, {});
    scalar(golden, cruise.state().integral_mps_s);
    golden.done();
}
void test_python_oracle_checks_timestep_only_when_integrating() {
    bt::ModelFixture fixture(cruise_xml); CruiseWriter cruise(fixture.model.get(), cruise_config);
    speed(fixture, cruise.state().target_speed_mps - .2); fixture.model->opt.timestep = 0.;
    (void)cruise.compute(fixture.data.get(), false, false, {});
    (void)cruise.compute(fixture.data.get(), true, true, {});
    bt::invalid([&] { (void)cruise.compute(fixture.data.get(), true, false, {}); });
}
void test_native_matches_python_at_each_serialized_timestep() {
    auto golden = oracle("TestCruiseTimestep", "test_native_matches_python_at_each_serialized_timestep");
    std::array<double, 3> integrals{}; std::size_t index = 0;
    for (double const timestep : {.0005, .001, .0025}) {
        bt::ModelFixture fixture(cruise_xml); fixture.model->opt.timestep = timestep;
        // Preserve the actual MJB serialization/loading boundary. The input
        // bytes are built from the same XML fixture when this test is run;
        // no absent archived MJB is replaced by an invented artifact.
        const auto image = bike::initialization::capture_model_image(*fixture.model);
        auto config = std::make_unique<nativecfg::WriterConfig>(); config->cruise = cruise_config;
        auto native = std::make_unique<Stepper>(image.mjb, *config);
        for (double const offset : {-.2, -.05, .1, -.3, .0, -.15}) {
            speed(fixture, native->cruise().state().target_speed_mps + offset);
            native->set_state(model_access::readonly_buffer(fixture.data->qpos, fixture.model->nq),
                model_access::readonly_buffer(fixture.data->qvel, fixture.model->nv),
                model_access::readonly_buffer(fixture.data->act, fixture.model->na),
                model_access::readonly_buffer(fixture.data->qacc_warmstart, fixture.model->nv), fixture.data->time);
            scalar(golden, native->mutate([&] { return native->cruise().compute(native->data(), true, false, {}); }));
            scalar(golden, native->cruise().state().integral_mps_s);
        }
        integrals[index++] = native->cruise().state().integral_mps_s;
    }
    bt::require(integrals[0] != integrals[1] && integrals[1] != integrals[2], "each owned model timestep enters integrator");
    golden.done();
}
} // namespace
int main() { return bt::run([] {
    test_half_even_ties(); test_nextafter_neighbours_of_ties(); test_scaled_dt_quotients();
    test_matches_builtin_round_on_valid_domain(); test_time_domain_rejects_with_valueerror();
    test_dt_domain_rejects_with_valueerror(); test_huge_positive_dt_is_valid();
    test_interval_overflow_is_overflowerror(); test_int64_upper_edge_accepted();
    test_ids_are_rounding_mode_independent(); test_python_oracle_reads_timestep_live();
    test_python_oracle_checks_timestep_only_when_integrating();
    test_native_matches_python_at_each_serialized_timestep();
}); }
