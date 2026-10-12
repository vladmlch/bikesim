// test_distance.cpp — direct C++ contracts for biketyre::SignedProfile
// (tyre/distance.cpp), the port of terrain/profile_distance.py. The
// bitwise gate against the Python oracle lives in
// tests/reference/test_native_distributed_tire.py; this file pins the
// domain/geometry contract standalone, like test_contracts.cpp does for
// the other native units.
#include "../src/tyre/distance.hpp"

#include <array>
#include <cmath>
#include <exception>
#include <iostream>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
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

    void test_signed_profile_ctor_domain() {
        {   // Valid control: two-vertex monotone profile.
            const biketyre::SignedProfile profile({{0., 0.}, {10., 0.}});
            require(profile.height(5.) == 0., "flat two-vertex profile");
        }
        require_throws_invalid_argument(
            [] {
                [[maybe_unused]] const biketyre::SignedProfile p(
                    {{0., 0.}});
            },
            "a single vertex is not a road");
        require_throws_invalid_argument(
            [] {
                [[maybe_unused]] const biketyre::SignedProfile p(
                    {{0., 0.}, {0., 1.}});
            },
            "a repeated X coordinate is not strictly increasing");
        require_throws_invalid_argument(
            [] {
                [[maybe_unused]] const biketyre::SignedProfile p(
                    {{1., 0.}, {0., 1.}});
            },
            "decreasing X must be rejected");
        require_throws_invalid_argument(
            [] {
                [[maybe_unused]] const biketyre::SignedProfile p(
                    {{0., 0.},
                     {1., std::numeric_limits<double>::quiet_NaN()}});
            },
            "non-finite vertex Z must be rejected");
        require_throws_invalid_argument(
            [] {
                [[maybe_unused]] const biketyre::SignedProfile p(
                    {{0., std::numeric_limits<double>::quiet_NaN()},
                     {1., 1.}});
            },
            "non-finite vertex X must be rejected");
        require_throws_invalid_argument(
            [] {
                [[maybe_unused]] const biketyre::SignedProfile p(
                    {{0., 0.},
                     {1., std::numeric_limits<double>::infinity()}});
            },
            "infinite vertex must be rejected");
    }

    void test_height_domain() {
        const biketyre::SignedProfile profile({{0., 0.}, {10., 2.}});
        require(profile.height(0.) == 0. && profile.height(10.) == 2.,
                "height at the domain endpoints");
        require(profile.height(5.) == 1., "linear interior height");
        require_throws_invalid_argument(
            [&] { static_cast<void>(profile.height(-.5)); },
            "below-domain height must be rejected");
        require_throws_invalid_argument(
            [&] { static_cast<void>(profile.height(10.5)); },
            "above-domain height must be rejected");
        require_throws_invalid_argument(
            [&] {
                static_cast<void>(profile.height(
                    std::numeric_limits<double>::quiet_NaN()));
            },
            "non-finite height query must be rejected");
    }

    void test_query_domain() {
        const biketyre::SignedProfile profile({{0., 0.}, {10., 0.}});
        require_throws_invalid_argument(
            [&] { static_cast<void>(profile.query({-1., .5})); },
            "below-domain query must be rejected");
        require_throws_invalid_argument(
            [&] { static_cast<void>(profile.query({11., .5})); },
            "above-domain query must be rejected");
        require_throws_invalid_argument(
            [&] {
                static_cast<void>(profile.query(
                    {5., std::numeric_limits<double>::quiet_NaN()}));
            },
            "non-finite query must be rejected");
    }

    void test_flat_road_query() {
        const biketyre::SignedProfile profile({{0., 0.}, {10., 0.}});
        const biketyre::PointDistance d = profile.query({5., .25});
        require(d.distance == .25 && !std::signbit(d.distance),
                "distance == z above a flat road");
        require(d.normal[0] == 0. && d.normal[1] == 1.,
                "flat-road normal points straight up");
        require(d.point[0] == 5. && d.point[1] == 0.,
                "closest point is the vertical projection");
        require(d.segment == 0, "the only segment wins");
        require(!d.nonsmooth, "flat road is smooth");
        const biketyre::PointDistance below = profile.query({5., -.25});
        require(below.distance == -.25,
                "below-road distance carries the negative sign");
    }

    void test_vertex_on_corner_nonsmooth() {
        // (0,0)->(1,1)->(2,0): the mesh vertex at x=1 is a real corner.
        const biketyre::SignedProfile profile({{0., 0.}, {1., 1.}, {2., 0.}});
        const biketyre::PointDistance d = profile.query({1., 1.});
        require(d.nonsmooth,
                "a point on a genuine corner is nonsmooth");
        require(d.segment == 0, "the lowest segment id wins the tie");
        require(d.distance == 0. && !std::signbit(d.distance),
                "on-surface point has +0 distance");
        require(d.point[0] == 1. && d.point[1] == 1.,
                "closest point is the corner vertex");
    }

    void test_collinear_tie_stays_smooth() {
        // Collinear subdivisions do not introduce physical corners.
        const biketyre::SignedProfile profile({{0., 0.}, {1., 0.}, {2., 0.}});
        const biketyre::PointDistance d = profile.query({1., .5});
        require(d.distance == .5,
                "point equidistant to both subdivisions");
        require(d.segment == 0, "the lowest segment id wins the tie");
        require(!d.nonsmooth, "collinear tie normals are identical");
        require(d.normal[0] == 0. && d.normal[1] == 1.,
                "collinear tie normal points straight up");
    }

    constexpr std::array<TestCase, 6> cases{{
        {.name = "signed_profile_ctor_domain",
         .run = test_signed_profile_ctor_domain},
        {.name = "height_domain", .run = test_height_domain},
        {.name = "query_domain", .run = test_query_domain},
        {.name = "flat_road_query", .run = test_flat_road_query},
        {.name = "vertex_on_corner_nonsmooth",
         .run = test_vertex_on_corner_nonsmooth},
        {.name = "collinear_tie_stays_smooth",
         .run = test_collinear_tie_stays_smooth},
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
        std::cerr << "usage: native_distance_tests [case-name|--list]\n";
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
        std::cerr << "unknown native distance case: " << requested << '\n';
        return 2;
    }
    return status;
}
