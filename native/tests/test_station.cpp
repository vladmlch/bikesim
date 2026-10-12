// test_station.cpp — direct C++ contracts for biketyre::station_angles/
// station_density/station_response (tyre/station.cpp), the scalar port of
// physics/distributed_tire.py's station law. The bitwise gate against the
// Python oracle lives in tests/reference/test_native_distributed_tire.py;
// this file pins the domain contract and the oracle-verified constants
// standalone, like test_distance.cpp does for SignedProfile.
#include "../src/tyre/station.hpp"

#include <array>
#include <cmath>
#include <exception>
#include <iostream>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {
    using TestFunction = void (*)();

    struct TestCase {
        std::string_view name;
        TestFunction run;
    };

    void require(bool condition, std::string_view message) {
        if (!condition)
            throw std::runtime_error(std::string(message));
    }

    // C1: invokes a callable, records that exactly std::invalid_argument
    // escapes, and fails through require if it returns or throws another
    // type.
    template <class F>
    void require_throws_invalid_argument(const F &f,
                                         std::string_view message) {
        try {
            f();
        } catch (const std::invalid_argument &) {
            return;
        } catch (...) {
            require(false, message);
        }
        require(false, message);
    }

    // HingeDensity.__post_init__ defaults (distributed_tire.py:12-16) —
    // the density the example configs carry.
    nativecfg::HingeDensity default_density() {
        return {.knots_m = {0., .004, .012},
                .stiffness_n_m = {300000., 0., 0.},
                .damping_ns_m = 2000.,
                .provenance = "synthetic density; not fitted or measured"};
    }

    void test_station_angles_domain() {
        {   // Valid bounds: the documented 16..512 window.
            const biketyre::StationAngles lo = biketyre::station_angles(16);
            require(lo.angles.size() == 16 && lo.weights.size() == 16,
                    "station_angles(16) sizes");
            const biketyre::StationAngles hi =
                    biketyre::station_angles(512);
            require(hi.angles.size() == 512 && hi.weights.size() == 512,
                    "station_angles(512) sizes");
        }
        require_throws_invalid_argument(
            [] { static_cast<void>(biketyre::station_angles(15)); },
            "fewer than sixteen stations must be rejected");
        require_throws_invalid_argument(
            [] { static_cast<void>(biketyre::station_angles(513)); },
            "more than MAX_STATION_COUNT stations must be rejected");
        require_throws_invalid_argument(
            [] { static_cast<void>(biketyre::station_angles(0)); },
            "zero stations must be rejected");
        require_throws_invalid_argument(
            [] { static_cast<void>(biketyre::station_angles(-1)); },
            "a negative station count must be rejected");
    }

    void test_station_angles_values() {
        // Oracle literals (np.pi arithmetic, hexfloat-exact):
        //   2*np.pi           = 0x1.921fb54442d18p+2
        //   weight  = 2pi/16  = 0x1.921fb54442d18p-2
        //   angle_i = 2pi*i/16
        const biketyre::StationAngles s = biketyre::station_angles(16);
        require(s.angles[0] == 0x0.0p+0, "angle 0 is +0.0");
        require(s.angles[1] == 0x1.921fb54442d18p-2,
                "angle 1 equals the weight for count 16");
        require(s.angles[2] == 0x1.921fb54442d18p-1, "angle 2");
        require(s.angles[15] == 0x1.78fdb9effea46p+2, "angle 15");
        for (const double w: s.weights)
            require(w == 0x1.921fb54442d18p-2,
                    "np.full weight is the scalar quotient everywhere");
        const biketyre::StationAngles s512 = biketyre::station_angles(512);
        require(s512.angles[1] == 0x1.921fb54442d18p-7,
                "angle 1 for count 512");
        require(s512.angles[511] == 0x1.9156a569a0b01p+2,
                "angle 511 for count 512");
        require(s512.weights[0] == 0x1.921fb54442d18p-7 &&
                    s512.weights[511] == 0x1.921fb54442d18p-7,
                "weight for count 512");
    }

    void test_station_density_default_law() {
        const nativecfg::HingeDensity d = default_density();
        const auto [f_neg, e_neg] = biketyre::station_density(-.01, d);
        require(f_neg == 0. && !std::signbit(f_neg) && e_neg == 0. &&
                    !std::signbit(e_neg),
                "a non-contact station is +0.0 force and energy");
        // x = .005 on the k=300000 knot only: force = 300000*(.005-0)+0+0,
        // energy = .5*(300000*.005*.005 + 0 + 0).
        const auto [f, e] = biketyre::station_density(.005, d);
        require(f == 1500. && e == 3.75,
                "single-knot density at delta .005");
        const auto [f2, e2] = biketyre::station_density(.02, d);
        require(f2 == 6000. && e2 == 60.,
                "single-knot density at delta .02");
    }

    void test_station_density_knot_fold() {
        // A nonzero stiffness on every knot exercises the sequential zip
        // fold the oracle's pairwise sum reduces to for three terms.
        const nativecfg::HingeDensity d{
            .knots_m = {0., .004, .012},
            .stiffness_n_m = {300000., 50000., 80000.},
            .damping_ns_m = 0.,
            .provenance = "test density"};
        const auto [f0, e0] = biketyre::station_density(.02, d);
        require(f0 == 7440. && e0 == 0x1.13d70a3d70a3ep+6,
                "three-knot force/energy fold at delta .02");
        const auto [f1, e1] = biketyre::station_density(.005, d);
        require(f1 == 1550. && e1 == 0x1.e333333333333p+1,
                "three-knot fold past the second knot");
        const auto [f2, e2] = biketyre::station_density(.013, d);
        require(f2 == 4430. && e2 == 0x1.b6a3d70a3d709p+4,
                "three-knot fold past the third knot");
    }

    void test_station_density_domain() {
        const nativecfg::HingeDensity d = default_density();
        require_throws_invalid_argument(
            [&] {
                static_cast<void>(biketyre::station_density(
                    std::numeric_limits<double>::quiet_NaN(), d));
            },
            "NaN penetration must be rejected");
        require_throws_invalid_argument(
            [&] {
                static_cast<void>(biketyre::station_density(
                    std::numeric_limits<double>::infinity(), d));
            },
            "infinite penetration must be rejected");
    }

    void test_station_response_values() {
        const nativecfg::HingeDensity d = default_density();
        // elastic = 1500 at delta .005; force = max(0, 1500 + 2000*rate)
        // gated on delta > 0, then scaled by the quadrature weight.
        const auto [f0, e0] = biketyre::station_response(.005, -.3, 2., d);
        require(f0 == 1800. && e0 == 7.5,
                "weighted response w*max(0, 1500 + 2000*(-.3))");
        const auto [f1, e1] =
                biketyre::station_response(.005, -.3, 7.25, d);
        require(f1 == 6525. && e1 == 27.1875,
                "the weight multiplies force and energy alike");
        const auto [f2, e2] = biketyre::station_response(0., 1., 2., d);
        require(f2 == 0. && e2 == 0.,
                "delta == 0 stays outside the delta > 0 gate");
        const auto [f3, e3] = biketyre::station_response(-.01, .5, 2., d);
        require(f3 == 0. && e3 == 0.,
                "negative penetration carries no load or energy");
        // The damping clamp: elastic 300 + 2000*(-1000) < 0 clips the
        // force to 0 while the stored energy survives.
        const auto [f4, e4] =
                biketyre::station_response(.001, -1000., 2., d);
        require(f4 == 0. && e4 == 0.3,
                "force clamps at zero; energy is unaffected");
        const auto [f5, e5] = biketyre::station_response(.005, 0., 0., d);
        require(f5 == 0. && !std::signbit(f5) && e5 == 0. &&
                    !std::signbit(e5),
                "a zero weight zeroes both channels to +0.0");
    }

    void test_station_response_domain() {
        const nativecfg::HingeDensity d = default_density();
        const double nan = std::numeric_limits<double>::quiet_NaN();
        require_throws_invalid_argument(
            [&] { static_cast<void>(biketyre::station_response(nan, 0., 1., d)); },
            "NaN penetration must be rejected");
        require_throws_invalid_argument(
            [&] { static_cast<void>(biketyre::station_response(.005, nan, 1., d)); },
            "NaN rate must be rejected");
        require_throws_invalid_argument(
            [&] { static_cast<void>(biketyre::station_response(.005, 0., nan, d)); },
            "NaN weight must be rejected");
        require_throws_invalid_argument(
            [&] { static_cast<void>(biketyre::station_response(.005, 0., -1., d)); },
            "a negative quadrature weight must be rejected");
    }

    constexpr std::array<TestCase, 7> cases{{
        {.name = "station_angles_domain", .run = test_station_angles_domain},
        {.name = "station_angles_values", .run = test_station_angles_values},
        {.name = "station_density_default_law",
         .run = test_station_density_default_law},
        {.name = "station_density_knot_fold",
         .run = test_station_density_knot_fold},
        {.name = "station_density_domain", .run = test_station_density_domain},
        {.name = "station_response_values",
         .run = test_station_response_values},
        {.name = "station_response_domain",
         .run = test_station_response_domain},
    }};

    int run_case(const TestCase &test_case) {
        try {
            test_case.run();
            std::cout << "PASS " << test_case.name << '\n';
            return 0;
        } catch (const std::exception &error) {
            std::cerr << "FAIL " << test_case.name << ": " << error.what()
                      << '\n';
            return 1;
        }
    }
} // namespace

int main(int argc, char **argv) {
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunsafe-buffer-usage-in-container"
#endif
    // The C process-entry contract provides argc pointers before the
    // sentinel.
    const std::span<char *> arguments(argv, static_cast<std::size_t>(argc));
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
    if (arguments.size() == 2 &&
        std::string_view(arguments[1]) == "--list") {
        for (const TestCase &test_case: cases)
            std::cout << test_case.name << '\n';
        return 0;
    }
    if (arguments.size() > 2) {
        std::cerr << "usage: native_station_tests [case-name|--list]\n";
        return 2;
    }

    const std::string_view requested =
        arguments.size() == 1 ? "all" : std::string_view(arguments[1]);
    bool found = requested == "all";
    int status = 0;
    for (const TestCase &test_case: cases) {
        if (requested == "all" || requested == test_case.name) {
            found = true;
            if (run_case(test_case) != 0)
                status = 1;
        }
    }
    if (!found) {
        std::cerr << "unknown native station case: " << requested << '\n';
        return 2;
    }
    return status;
}
