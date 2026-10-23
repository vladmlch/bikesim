// test_native_pneumatic_geometry.cpp — C++ port of
// tests/reference/test_native_pneumatic_geometry.py (batch 3.12a, T52).
//
// Replays the RayRing / RoadProfile / intersect / patches oracle corpus
// against the frozen two-platform goldens. `_oracle_branch` is ported so
// every corpus row still pins the control-flow branch it was built to
// exercise; the verification itself is the bitwise golden comparison.
#include "pneumatic_reference_support.hpp"

namespace {

using namespace pneumatic_reference;
constexpr std::string_view kFile = "test_native_pneumatic_geometry.py";

// ---------------------------------------------------------------------
// tests/reference/test_native_pneumatic_geometry.py
//   ::test_pneumatic_ring_bitwise[n_rays]
// ---------------------------------------------------------------------
void test_pneumatic_ring_bitwise(std::int64_t n_rays) {
    auto golden = node(kFile, "test_pneumatic_ring_bitwise",
                       std::to_string(n_rays));
    const biketyre::RayRing ring(n_rays, kHalf);
    vector(golden, ring.theta_rad(), "");
    vector(golden, ring.ux(), "");
    vector(golden, ring.uz(), "");
    scalar(golden, ring.dtheta_rad(), "");
    golden.done();
}

// ::test_pneumatic_ring_narrow_angle
void test_pneumatic_ring_narrow_angle() {
    auto golden = node(kFile, "test_pneumatic_ring_narrow_angle");
    const biketyre::RayRing ring(64, 30.0 * kRad);
    vector(golden, ring.theta_rad(), "");
    vector(golden, ring.ux(), "");
    vector(golden, ring.uz(), "");
    golden.done();
}

// ---------------------------------------------------------------------
// _oracle_branch — classifies the intersect() path, mirroring the
// oracle's control flow (index_range window, inside mask, phi
// monotonicity and coverage flags). Returns the branch enum and the
// coverage flag the corpus expectation carries.
// ---------------------------------------------------------------------
enum class Branch { early_window, airborne, fast, brute };

struct BranchOutcome {
    Branch branch;
    bool coverage = false;
};

[[nodiscard]] BranchOutcome oracle_branch(const biketyre::RoadProfile &road,
                                          double cx, double cz,
                                          double radius, double half) {
    const auto [lo, hi] = road.index_range(cx - radius, cx + radius);
    if (hi - lo < 2) return {Branch::early_window, false};
    const auto zs = road.z_m();
    const auto count = static_cast<std::size_t>(hi - lo);
    std::vector<double> xs(count), pz(count), phi_scratch;
    std::vector<char> inside(count, 0);
    for (std::size_t k = 0; k < count; ++k) {
        const auto index = static_cast<std::size_t>(lo) + k;
        xs[k] = road.x0_m() +
                road.dx_m() * static_cast<double>(index);
        pz[k] = zs[index];
        const double dxs = xs[k] - cx;
        const double dzs = cz - pz[k];
        inside[k] = dxs * dxs + dzs * dzs < radius * radius ? 1 : 0;
    }
    if (std::ranges::none_of(inside, [](const char flag) {
            return flag != 0;
        }))
        return {Branch::airborne, false};
    std::size_t first = 0;
    while (!inside[first]) ++first;
    std::size_t last = count - 1;
    while (!inside[last]) --last;
    const std::size_t a = first > 0 ? first - 1 : 0;
    const std::size_t b = std::min(last + 2, count);
    const std::size_t m = b - a;
    phi_scratch.resize(m);
    bool coverage = false;
    for (std::size_t j = a; j < b; ++j) {
        phi_scratch[j - a] = std::atan2(xs[j] - cx, cz - pz[j]);
        if (inside[j] &&
            (std::fabs(phi_scratch[j - a]) > half + 1e-9 || pz[j] >= cz))
            coverage = true;
    }
    if (m >= 2) {
        bool monotone = true;
        for (std::size_t j = 1; j < m; ++j)
            if (!(phi_scratch[j] - phi_scratch[j - 1] > 0.0)) {
                monotone = false;
                break;
            }
        if (monotone) return {Branch::fast, coverage};
    }
    return {Branch::brute, coverage};
}

// ---------------------------------------------------------------------
// The corpus roads — z(x) cell functions matching
// test_native_pneumatic_geometry.py::_cases().
// ---------------------------------------------------------------------
double z_flat(double) { return 0.0; }
double z_ramp(double x) { return 0.03 * x - 0.15; }
double z_step_down(double x) { return x < 5.02 ? 0.0 : -0.12; }
double z_cliff(double x) { return x < 5.32 ? 0.0 : 0.27; }
double z_notch_left(double x) { return x < 5.0 ? 0.0 : -0.12; }
double z_spike_above(double x) {
    return numeric::np_maximum(0.0, 0.36 - std::fabs(x - 5.02) * 12.0);
}
double z_spike_below(double x) {
    return numeric::np_maximum(0.0, 0.27 - std::fabs(x - 5.02) * 8.0);
}
double z_parallel_ramp(double x) {
    return x < 4.5 ? 1.74879359546275 * (4.5 - 4.95) + 0.30
         : x < 4.95 ? 1.74879359546275 * (x - 4.95) + 0.30
                    : 0.0;
}
double z_edge_front(double x) {
    return numeric::np_maximum(0.0, 0.235 - std::fabs(x - 5.30) * 6.0);
}
double z_edge_rear(double x) {
    return numeric::np_maximum(0.0, 0.235 - std::fabs(x - 4.70) * 6.0);
}

enum class Expect { none, early_window, airborne, fast, brute };

struct CorpusCase {
    std::string_view id;
    double x0, dx;
    std::size_t samples;
    double (*z_at)(double);
    double cx, cz, radius;
    Expect expected;
    bool coverage;
};

const CorpusCase kCorpus[] = {
        // monotone fast path, no coverage
        {"flat-loaded", 0.0, 0.01, 1001, z_flat, 5.0, kRadius - 0.006,
         kRadius, Expect::fast, false},
        {"flat-grazing", 0.0, 0.01, 1001, z_flat, 5.0, kRadius - 1e-9,
         kRadius, Expect::fast, false},
        // fast path + coverage via pz >= cz
        {"flat-sunk", 0.0, 0.01, 1001, z_flat, 5.0, 0.05, kRadius,
         Expect::fast, true},
        // airborne
        {"flat-airborne", 0.0, 0.01, 1001, z_flat, 5.0, kRadius + 0.5,
         kRadius, Expect::airborne, false},
        // hi - lo < 2 early-out (both ends + zero radius)
        {"off-left", 0.0, 0.01, 1001, z_flat, -50.0, kRadius - 0.01,
         kRadius, Expect::early_window, false},
        {"off-right", 0.0, 0.01, 1001, z_flat, 50.0, kRadius - 0.01,
         kRadius, Expect::early_window, false},
        {"zero-radius", 0.0, 0.01, 1001, z_flat, 5.0, 0.0, 0.0,
         Expect::early_window, false},
        // ramp (sloped road through the wheel)
        {"ramp", 0.0, 0.01, 1001, z_ramp, 5.0, 0.19, kRadius,
         Expect::fast, false},
        // step down just ahead of the hub
        {"step-down", 0.0, 0.01, 1001, z_step_down, 5.0, 0.30, kRadius,
         Expect::fast, false},
        // step-up cliff inside the disc: angle coverage
        {"cliff", 0.0, 0.01, 1001, z_cliff, 5.0, 0.32, kRadius,
         Expect::fast, true},
        // non-monotone phi -> brute-force
        {"notch-left", 0.0, 0.01, 1001, z_notch_left, 4.95, 0.30,
         kRadius, Expect::brute, false},
        // sharp spike whose apex rises above the hub
        {"spike-above-hub", 0.0, 0.01, 1001, z_spike_above, 5.0, 0.30,
         kRadius, Expect::brute, true},
        // spike apex below the hub: non-monotone, no coverage
        {"spike-below-hub", 0.0, 0.01, 1001, z_spike_below, 5.0, 0.31,
         kRadius, Expect::brute, false},
        // sunk hub on flat road: window wraps through +-pi -> brute
        {"sunk-hub", 0.0, 0.01, 1001, z_flat, 5.0, -0.05, kRadius,
         Expect::brute, true},
        // steep ramp left of hub parallel to ray ~theta_19 (n=64)
        {"parallel-ramp", 0.0, 0.005, 1601, z_parallel_ramp, 5.0, 0.30,
         kRadius, Expect::brute, true},
        // wedge bump just inside +half angle: patch reaches ray n-1
        {"edge-front", 0.0, 0.01, 1001, z_edge_front, 5.0, 0.30,
         kRadius, Expect::none, false},
        // wedge bump just inside -half angle: patch starts at ray 0
        {"edge-rear", 0.0, 0.01, 1001, z_edge_rear, 5.0, 0.30,
         kRadius, Expect::none, false},
};

[[nodiscard]] biketyre::RoadProfile corpus_road(const CorpusCase &row) {
    std::vector<double> z(row.samples);
    for (std::size_t i = 0; i < row.samples; ++i)
        z[i] = row.z_at(static_cast<double>(i) * row.dx);
    return biketyre::RoadProfile(row.x0, row.dx, std::move(z));
}

void consume_intersect(bt::oracle::Node &golden,
                       const biketyre::RayHits &hits) {
    vector(golden, hits.r_m, "");
    vector(golden, hits.delta_m, "");
    vector(golden, hits.road_x_m, "");
    vector(golden, hits.road_z_m, "");
    boolean(golden, hits.coverage_event, "coverage_event");
    boolean(golden, hits.airborne, "airborne");
}

// ::test_pneumatic_intersect_bitwise[n_rays-<case id>]
void test_pneumatic_intersect_bitwise(std::int64_t n_rays,
                                    const CorpusCase &row) {
    auto golden = node(kFile, "test_pneumatic_intersect_bitwise",
                       std::to_string(n_rays) + "-" +
                               std::string(row.id));
    const biketyre::RoadProfile road = corpus_road(row);
    if (row.expected != Expect::none) {
        const auto outcome =
                oracle_branch(road, row.cx, row.cz, row.radius, kHalf);
        switch (row.expected) {
        case Expect::early_window:
            bt::require(outcome.branch == Branch::early_window,
                        "corpus row takes the early_window branch");
            break;
        case Expect::airborne:
            bt::require(outcome.branch == Branch::airborne,
                        "corpus row takes the airborne branch");
            break;
        case Expect::fast:
        case Expect::brute: {
            const auto wanted =
                    row.expected == Expect::fast ? Branch::fast
                                                 : Branch::brute;
            bt::require(outcome.branch == wanted &&
                            outcome.coverage == row.coverage,
                        "corpus row takes its pinned branch/coverage");
            break;
        }
        case Expect::none:
            break;
        }
    }
    const biketyre::RayRing ring(n_rays, kHalf);
    const auto hits =
            biketyre::intersect(ring, row.cx, row.cz, row.radius, road);
    consume_intersect(golden, hits);
    golden.done();
}

// ::test_pneumatic_intersect_coarse_and_fine_dx — different sampling
// densities hit different index_range windows.
void test_pneumatic_intersect_coarse_and_fine_dx() {
    auto golden =
            node(kFile, "test_pneumatic_intersect_coarse_and_fine_dx");
    for (const double dx : {0.05, 0.002, 0.017}) {
        const auto samples =
                static_cast<std::size_t>(static_cast<int>(10.0 / dx)) + 1;
        std::vector<double> z(samples);
        for (std::size_t i = 0; i < samples; ++i) {
            const double x = static_cast<double>(i) * dx;
            z[i] = 0.02 * numeric::np_sin(x * 3.1) +
                   numeric::np_maximum(0.0,
                                       0.05 - std::fabs(x - 5.0) * 4.0);
        }
        const biketyre::RoadProfile road(0.0, dx, std::move(z));
        for (const std::int64_t n : kRays) {
            const biketyre::RayRing ring(n, kHalf);
            const auto hits =
                    biketyre::intersect(ring, 5.0, 0.28, kRadius, road);
            consume_intersect(golden, hits);
        }
    }
    golden.done();
}

// ::test_pneumatic_intersect_hub_sweep — the hub sweep across a bumpy
// road mixes all monotone paths.
void test_pneumatic_intersect_hub_sweep() {
    auto golden = node(kFile, "test_pneumatic_intersect_hub_sweep");
    const auto x = arange_scaled(1201, 0.01);
    std::vector<double> z(1201);
    for (std::size_t i = 0; i < 1201; ++i)
        z[i] = 0.015 * numeric::np_sin(x[i] * 2.7) +
               numeric::np_maximum(0.0,
                                   0.05 - std::fabs(x[i] - 4.80) * 5.0) +
               numeric::np_maximum(0.0,
                                   0.04 - std::fabs(x[i] - 5.30) * 7.0);
    const biketyre::RoadProfile road(0.0, 0.01, std::move(z));
    const auto cxs = np_linspace(4.55, 5.45, 10);
    for (const double cx : cxs)
        for (const double cz : {0.26, 0.30, 0.34})
            for (const std::int64_t n : kRays) {
                const biketyre::RayRing ring(n, kHalf);
                const auto hits =
                        biketyre::intersect(ring, cx, cz, kRadius, road);
                consume_intersect(golden, hits);
            }
    golden.done();
}

// ::test_pneumatic_intersect_edge_patches [port] — the wedge cases
// must load the boundary rays themselves.
void test_pneumatic_intersect_edge_patches() {
    const auto x = arange_scaled(1001, 0.01);
    for (const std::int64_t n : kRays) {
        const biketyre::RayRing ring(n, kHalf);
        std::vector<double> front(1001), rear(1001);
        for (std::size_t i = 0; i < 1001; ++i) {
            front[i] = z_edge_front(x[i]);
            rear[i] = z_edge_rear(x[i]);
        }
        const biketyre::RoadProfile front_road(0.0, 0.01,
                                               std::move(front));
        auto hits =
                biketyre::intersect(ring, 5.0, 0.30, kRadius, front_road);
        auto runs = biketyre::patches(hits.delta_m);
        // rear-most +angle ray n-1 loaded
        bt::require(!runs.empty() &&
                        runs.back().second == static_cast<int>(n),
                    "front wedge loads ray n-1");
        const biketyre::RoadProfile rear_road(0.0, 0.01,
                                              std::move(rear));
        hits = biketyre::intersect(ring, 5.0, 0.30, kRadius, rear_road);
        runs = biketyre::patches(hits.delta_m);
        // ray 0 loaded
        bt::require(!runs.empty() && runs.front().first == 0,
                    "rear wedge loads ray 0");
    }
}

// ---------------------------------------------------------------------
// patches — _assert_patches(delta) consumes one 'patches' entry.
// ---------------------------------------------------------------------
void test_pneumatic_patches_empty() {
    auto golden = node(kFile, "test_pneumatic_patches_empty");
    consume_patches(golden, std::vector<double>(64, 0.0));
    consume_patches(golden, std::span<const double>{});
    const std::array neg{-1e-3, -0.2};
    consume_patches(golden, neg);
    golden.done();
}

void test_pneumatic_patches_single_run() {
    auto golden = node(kFile, "test_pneumatic_patches_single_run");
    std::vector<double> d(64, 0.0);
    for (std::size_t i = 10; i < 20; ++i) d[i] = 0.01;
    consume_patches(golden, d);
    golden.done();
}

void test_pneumatic_patches_multi_run() {
    auto golden = node(kFile, "test_pneumatic_patches_multi_run");
    std::vector<double> d(64, 0.0);
    for (std::size_t i = 2; i < 5; ++i) d[i] = 1e-4;
    d[10] = 0.3;
    d[11] = 0.1;
    for (std::size_t i = 40; i < 64; i += 2) d[i] = 0.02;
    consume_patches(golden, d);
    golden.done();
}

void test_pneumatic_patches_boundaries() {
    auto golden = node(kFile, "test_pneumatic_patches_boundaries");
    std::vector<double> d(64, 0.0);
    d[0] = 1.0; // run touching index 0
    for (std::size_t i = 62; i < 64; ++i) d[i] = 0.5;
    consume_patches(golden, d);
    const std::vector<double> full(64, 0.1); // single full-width run
    consume_patches(golden, full);
    std::vector<double> lone(64, 0.0);
    lone[0] = 0.1;
    consume_patches(golden, lone);
    golden.done();
}

void test_pneumatic_patches_tiny_positive() {
    auto golden = node(kFile, "test_pneumatic_patches_tiny_positive");
    std::vector<double> d(8, 0.0);
    // np.finfo(np.float64).tiny — the smallest normalized positive.
    d[3] = std::numeric_limits<double>::min();
    consume_patches(golden, d);
    d[5] = 1.0 / std::numeric_limits<double>::max();
    consume_patches(golden, d);
    golden.done();
}

void test_pneumatic_patches_nonfinite() {
    auto golden = node(kFile, "test_pneumatic_patches_nonfinite");
    const std::array d{std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::quiet_NaN(), 0.1,
                       -std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::infinity()};
    consume_patches(golden, d);
    golden.done();
}

void test_pneumatic_patches_from_intersect() {
    auto golden = node(kFile, "test_pneumatic_patches_from_intersect");
    const auto x = arange_scaled(1001, 0.01);
    std::vector<double> front(1001);
    for (std::size_t i = 0; i < 1001; ++i) front[i] = z_edge_front(x[i]);
    const biketyre::RoadProfile road(0.0, 0.01, std::move(front));
    for (const std::int64_t n : kRays) {
        const biketyre::RayRing ring(n, kHalf);
        const auto hits =
                biketyre::intersect(ring, 5.0, 0.30, kRadius, road);
        consume_patches(golden, hits.delta_m);
    }
    golden.done();
}

// ::test_road_profile_helpers_match_oracle_contract — x_m is the
// endpoint-pinned x0 + dx*k row and height_at is np.interp over the
// FULL profile (end-clamped).
void test_road_profile_helpers_match_oracle_contract() {
    auto golden =
            node(kFile, "test_road_profile_helpers_match_oracle_contract");
    std::vector<double> z(500);
    for (std::size_t i = 0; i < 500; ++i)
        z[i] = numeric::np_sin(static_cast<double>(i) * 0.013);
    const biketyre::RoadProfile road(1.5, 0.007, std::move(z));
    vector(golden, road.x_m(), "x_m");
    for (const double q : {-5.0, 0.0, 1.499, 1.5, 3.75, 4.995, 20.0})
        scalar(golden, road.height_at(q),
               "height_at(" + repr_binary64(q) + ")");
    golden.done();
}

} // namespace

// bt::run wraps the test body in try/catch(...) — failures are
// reported, nothing escapes main.
// cppcheck-suppress throwInEntryPoint
int main() {
    return bt::run([] {
        for (const std::int64_t n : {3, 63, 64, 256})
            test_pneumatic_ring_bitwise(n);
        test_pneumatic_ring_narrow_angle();
        for (const std::int64_t n : kRays)
            for (const auto &row : kCorpus)
                test_pneumatic_intersect_bitwise(n, row);
        test_pneumatic_intersect_coarse_and_fine_dx();
        test_pneumatic_intersect_hub_sweep();
        test_pneumatic_intersect_edge_patches();
        test_pneumatic_patches_empty();
        test_pneumatic_patches_single_run();
        test_pneumatic_patches_multi_run();
        test_pneumatic_patches_boundaries();
        test_pneumatic_patches_tiny_positive();
        test_pneumatic_patches_nonfinite();
        test_pneumatic_patches_from_intersect();
        test_road_profile_helpers_match_oracle_contract();
    });
}
