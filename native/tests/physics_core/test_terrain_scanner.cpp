#include "physics_core/cpu_exact_toolkit.hpp"
#include "physics_core/gpu_toolkit.hpp"
#include "physics_core/terrain_distance.hpp"
#include "physics_core/terrain_scanner.hpp"
#include "physics_core/tyre_types.hpp"
#include "cblas_abi.hpp"
#include "numeric_numpy.hpp"
#include "numeric_sincos.hpp"
#include "rtsan.hpp"
#include "support/fixture_test.hpp"
#include "terrain/profile.hpp"
#include "terrain/trackfile.hpp"
#include "tyre/distance.hpp"
#include "tyre/profile.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <numbers>
#include <optional>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

using bike::physics_core::CpuExactToolkit;
using bike::physics_core::ErrCode;
using bike::physics_core::GpuToolkit;
using bike::physics_core::NoMultiSupport;
using bike::physics_core::O1CpuMultiSupport;
using bike::physics_core::PointDistanceResult;
using bike::physics_core::PolylineContactResult;
using bike::physics_core::PolylineView;
using bike::physics_core::SignedProfileView;
using bike::physics_core::Status;
using bike::physics_core::i32;
using bike::physics_core::profile_height;
using bike::physics_core::scan_polyline_contact;
using bike::physics_core::signed_profile_query;

void require_bits_f64(double actual, double expected, std::string_view what) {
    if (std::bit_cast<std::uint64_t>(actual) != std::bit_cast<std::uint64_t>(expected)) {
        throw std::runtime_error(std::string(what) + ": 64-bit mismatch");
    }
}

struct HostPolylineStorage {
    std::vector<double> px;
    std::vector<double> pz;
    std::vector<double> seg_x;
    std::vector<double> seg_z;
    std::vector<double> seg_len_sq;
    std::vector<i32> height_changes;
    std::vector<double> left_x;
    std::vector<double> left_z;
    std::vector<double> right_x;
    std::vector<double> right_z;
    double maximum_z = 0.0;
    double significant_delta_m = 0.0;
    double significance_fraction = 0.0;
    double normal_cosine = 0.0;

    std::vector<float> px_f32;
    std::vector<float> pz_f32;
    std::vector<float> seg_x_f32;
    std::vector<float> seg_z_f32;
    std::vector<float> seg_len_sq_f32;
    std::vector<float> left_x_f32;
    std::vector<float> left_z_f32;
    std::vector<float> right_x_f32;
    std::vector<float> right_z_f32;

    [[nodiscard]] PolylineView<double> view_f64() const noexcept {
        return {
            .px = px.data(),
            .pz = pz.data(),
            .seg_x = seg_x.data(),
            .seg_z = seg_z.data(),
            .seg_len_sq = seg_len_sq.data(),
            .height_changes = height_changes.data(),
            .left_x = left_x.data(),
            .left_z = left_z.data(),
            .right_x = right_x.data(),
            .right_z = right_z.data(),
            .num_vertices = static_cast<i32>(px.size()),
            .maximum_z = maximum_z,
            .significant_delta_m = significant_delta_m,
            .significance_fraction = significance_fraction,
            .normal_cosine = normal_cosine,
        };
    }

    [[nodiscard]] PolylineView<float> view_f32() const noexcept {
        return {
            .px = px_f32.data(),
            .pz = pz_f32.data(),
            .seg_x = seg_x_f32.data(),
            .seg_z = seg_z_f32.data(),
            .seg_len_sq = seg_len_sq_f32.data(),
            .height_changes = height_changes.data(),
            .left_x = left_x_f32.data(),
            .left_z = left_z_f32.data(),
            .right_x = right_x_f32.data(),
            .right_z = right_z_f32.data(),
            .num_vertices = static_cast<i32>(px_f32.size()),
            .maximum_z = static_cast<float>(maximum_z),
            .significant_delta_m = static_cast<float>(significant_delta_m),
            .significance_fraction = static_cast<float>(significance_fraction),
            .normal_cosine = static_cast<float>(normal_cosine),
        };
    }
};

[[nodiscard]] HostPolylineStorage build_polyline_storage(
    std::vector<double> px_in,
    std::vector<double> pz_in,
    double significant_delta_m,
    double significance_fraction,
    double normal_angle_deg
) {
    HostPolylineStorage s;
    s.px = std::move(px_in);
    s.pz = std::move(pz_in);
    const std::size_t n = s.px.size();
    const std::size_t count = n - 1;
    s.seg_x.resize(count);
    s.seg_z.resize(count);
    s.seg_len_sq.resize(count);
    s.height_changes.resize(n);
    s.left_x.resize(n);
    s.left_z.resize(n);
    s.right_x.resize(n);
    s.right_z.resize(n);

    for (std::size_t i = 0; i < count; ++i) {
        s.seg_x[i] = s.px[i + 1] - s.px[i];
        s.seg_z[i] = s.pz[i + 1] - s.pz[i];
        double len_sq = 0.0;
        len_sq += s.seg_x[i] * s.seg_x[i];
        len_sq += s.seg_z[i] * s.seg_z[i];
        s.seg_len_sq[i] = len_sq;
    }
    s.height_changes[0] = 0;
    for (std::size_t i = 0; i < count; ++i) {
        s.height_changes[i + 1] = s.height_changes[i] + (s.seg_z[i] != 0.0 ? 1 : 0);
    }
    s.left_x[0] = 0.0;
    s.left_z[0] = 0.0;
    for (std::size_t v = 1; v <= count; ++v) {
        s.left_x[v] = -s.seg_x[v - 1];
        s.left_z[v] = -s.seg_z[v - 1];
    }
    for (std::size_t v = 0; v < count; ++v) {
        s.right_x[v] = s.seg_x[v];
        s.right_z[v] = s.seg_z[v];
    }
    s.right_x[count] = 0.0;
    s.right_z[count] = 0.0;
    s.maximum_z = *std::ranges::max_element(s.pz);
    s.significant_delta_m = significant_delta_m;
    s.significance_fraction = significance_fraction;
    s.normal_cosine = numeric::np_cos(normal_angle_deg * (std::numbers::pi / 180.0));

    const auto to_f32 = [](const std::vector<double> &src) {
        std::vector<float> dst(src.size());
        for (std::size_t i = 0; i < src.size(); ++i) {
            dst[i] = static_cast<float>(src[i]);
        }
        return dst;
    };
    s.px_f32 = to_f32(s.px);
    s.pz_f32 = to_f32(s.pz);
    s.seg_x_f32 = to_f32(s.seg_x);
    s.seg_z_f32 = to_f32(s.seg_z);
    s.seg_len_sq_f32 = to_f32(s.seg_len_sq);
    s.left_x_f32 = to_f32(s.left_x);
    s.left_z_f32 = to_f32(s.left_z);
    s.right_x_f32 = to_f32(s.right_x);
    s.right_z_f32 = to_f32(s.right_z);
    return s;
}

// Independent legacy O(N)-scratch reference implementation matching pre-refactor
// biketyre::ProfileQuery::contact (profile.cpp:483-656) bit-for-bit.
[[nodiscard]] biketyre::ProfileContact reference_contact_legacy(
    const HostPolylineStorage &s,
    std::array<double, 2> c,
    double radius,
    std::optional<int> previous_segment
) {
    if (!std::isfinite(c[0]) || !std::isfinite(c[1])) {
        throw std::invalid_argument("invalid shape or non-finite wheel center");
    }
    if (!std::isfinite(radius) || radius <= 0.0) {
        throw std::invalid_argument("invalid wheel radius");
    }
    const int count = static_cast<int>(s.px.size()) - 1;
    if (previous_segment && (*previous_segment < 0 || *previous_segment >= count)) {
        throw std::invalid_argument("invalid previous contact segment");
    }
    if (!(s.px.front() <= c[0] && c[0] <= s.px.back())) {
        throw std::invalid_argument("wheel center is outside profile domain");
    }
    if (c[1] <= biketyre::np_interp(c[0], s.px, s.pz)) {
        throw std::invalid_argument("wheel center reached or entered solid road");
    }

    const auto search_right = [&](double v) {
        return static_cast<int>(std::ranges::upper_bound(s.px, v) - s.px.begin());
    };
    const auto search_left = [&](double v) {
        return static_cast<int>(std::ranges::lower_bound(s.px, v) - s.px.begin());
    };
    const auto clip01 = [](double t) {
        return t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
    };

    const int lo = std::max(0, search_right(c[0] - radius) - 2);
    const int hi = std::min(count, search_right(c[0] + radius) + 1);
    const double height = c[1] - s.pz[static_cast<std::size_t>(lo)];

    if (height < radius &&
        s.height_changes[static_cast<std::size_t>(hi)] ==
            s.height_changes[static_cast<std::size_t>(lo)]) {
        int segment = std::max(0, search_left(c[0]) - 1);
        std::array<double, 2> point = {c[0], s.pz[static_cast<std::size_t>(lo)]};
        double distance = height;
        if (previous_segment && lo <= *previous_segment && *previous_segment < hi) {
            const std::size_t pv = static_cast<std::size_t>(*previous_segment);
            const double previous_x = std::min(std::max(c[0], s.px[pv]), s.px[pv + 1]);
            const double previous_distance = biketyre::py_hypot(c[0] - previous_x, height);
            const int vertex = previous_x == s.px[pv] ? *previous_segment : *previous_segment + 1;
            const bool endpoint = previous_x == s.px[pv] || previous_x == s.px[pv + 1];
            bool closer = false;
            if (endpoint) {
                const std::size_t vv = static_cast<std::size_t>(vertex);
                const std::array<double, 2> off = {c[0] - s.px[vv], c[1] - s.pz[vv]};
                const std::array<double, 2> left = {s.left_x[vv], s.left_z[vv]};
                const std::array<double, 2> right = {s.right_x[vv], s.right_z[vv]};
                closer = blas::ddot(2, off.data(), 1, left.data(), 1) > 1e-14 ||
                         blas::ddot(2, off.data(), 1, right.data(), 1) > 1e-14;
            }
            if (!closer && std::abs(previous_distance - distance) <= 1e-12) {
                segment = *previous_segment;
                point[0] = previous_x;
                distance = previous_distance;
            }
        }
        if (distance <= 1e-12) {
            throw std::invalid_argument("unsupported or degenerate contact geometry");
        }
        return {
            .point = point,
            .normal = {(c[0] - point[0]) / distance, (c[1] - point[1]) / distance},
            .delta = radius - distance,
            .segment_id = segment,
            .multi_support = false,
        };
    }

    struct Cand {
        std::vector<int> ids;
        std::vector<double> t;
        std::vector<std::array<double, 2>> points;
        std::vector<double> distances;
    };
    const auto build_cand = [&](int w_lo, int w_hi) {
        Cand out;
        for (int i = w_lo; i < w_hi; ++i) {
            const std::size_t u = static_cast<std::size_t>(i);
            out.ids.push_back(i);
            const double ox = c[0] - s.px[u];
            const double oz = c[1] - s.pz[u];
            double num = 0.0;
            num += ox * s.seg_x[u];
            num += oz * s.seg_z[u];
            const double t = clip01(num / s.seg_len_sq[u]);
            out.t.push_back(t);
            const double ptx = s.px[u] + t * s.seg_x[u];
            const double ptz = s.pz[u] + t * s.seg_z[u];
            out.points.push_back({ptx, ptz});
            const double dx = c[0] - ptx;
            const double dz = c[1] - ptz;
            double dsq = 0.0;
            dsq += dx * dx;
            dsq += dz * dz;
            out.distances.push_back(std::sqrt(dsq));
        }
        return out;
    };

    Cand cand = build_cand(lo, hi);
    if (cand.ids.empty() || *std::ranges::min_element(cand.distances) >= radius) {
        int search_lo = 0;
        int search_hi = 0;
        if (!cand.ids.empty()) {
            const double best = *std::ranges::min_element(cand.distances);
            const double vertical_lower = std::max(0.0, c[1] - s.maximum_z);
            const double reach =
                std::sqrt(std::max(0.0, best * best - vertical_lower * vertical_lower)) + 1e-8;
            search_lo = std::max(0, search_right(c[0] - reach) - 2);
            search_hi = std::min(count, search_right(c[0] + reach) + 1);
        } else {
            search_lo = 0;
            search_hi = count;
        }
        cand = build_cand(search_lo, search_hi);
    }
    if (cand.ids.empty() || *std::ranges::min_element(cand.distances) <= 1e-12) {
        throw std::invalid_argument("unsupported or degenerate contact geometry");
    }

    std::vector<bool> keep(cand.ids.size(), true);
    for (std::size_t i = 0; i < cand.ids.size(); ++i) {
        const double t = cand.t[i];
        if (t != 0.0 && t != 1.0) {
            continue;
        }
        const int vertex = cand.ids[i] + (t == 1.0 ? 1 : 0);
        const std::size_t v = static_cast<std::size_t>(vertex);
        const double ox = c[0] - s.px[v];
        const double oz = c[1] - s.pz[v];
        double dl = 0.0;
        dl += ox * s.left_x[v];
        dl += oz * s.left_z[v];
        if (dl > 1e-14) {
            keep[i] = false;
        }
        double dr = 0.0;
        dr += ox * s.right_x[v];
        dr += oz * s.right_z[v];
        if (dr > 1e-14) {
            keep[i] = false;
        }
    }

    std::size_t winner = 0;
    for (std::size_t i = 1; i < cand.distances.size(); ++i) {
        if (cand.distances[i] < cand.distances[winner]) {
            winner = i;
        }
    }
    if (previous_segment) {
        for (std::size_t i = 0; i < cand.ids.size(); ++i) {
            if (cand.ids[i] == *previous_segment && keep[i]) {
                if (std::abs(cand.distances[i] - cand.distances[winner]) <= 1e-12) {
                    winner = i;
                }
                break;
            }
        }
    }

    const double delta_winner = radius - cand.distances[winner];
    const std::array<double, 2> normal_winner = {
        (c[0] - cand.points[winner][0]) / cand.distances[winner],
        (c[1] - cand.points[winner][1]) / cand.distances[winner],
    };
    const double threshold = std::max(
        s.significant_delta_m,
        s.significance_fraction * std::max(delta_winner, 0.0));
    std::vector<int> near_ids;
    for (std::size_t i = 0; i < cand.ids.size(); ++i) {
        if (radius - cand.distances[i] >= threshold && keep[i]) {
            near_ids.push_back(static_cast<int>(i));
        }
    }
    bool multi = false;
    if (!near_ids.empty()) {
        std::vector<double> normals(2 * near_ids.size());
        for (std::size_t j = 0; j < near_ids.size(); ++j) {
            const std::size_t i = static_cast<std::size_t>(near_ids[j]);
            normals[2 * j] = (c[0] - cand.points[i][0]) / cand.distances[i];
            normals[2 * j + 1] = (c[1] - cand.points[i][1]) / cand.distances[i];
        }
        std::vector<double> dots(near_ids.size());
        blas::dgemv(
            blas::Order::row_major, blas::Transpose::no,
            static_cast<int>(near_ids.size()), 2, 1.0,
            normals.data(), 2, normal_winner.data(), 1, 0.0, dots.data(), 1);
        for (std::size_t j = 0; j < near_ids.size(); ++j) {
            const std::size_t i = static_cast<std::size_t>(near_ids[j]);
            const double ex = cand.points[i][0] - cand.points[winner][0];
            const double ez = cand.points[i][1] - cand.points[winner][1];
            const bool separated = std::sqrt(ex * ex + ez * ez) > 1e-10;
            if (dots[j] < s.normal_cosine && separated) {
                multi = true;
                break;
            }
        }
    }
    return {
        .point = cand.points[winner],
        .normal = normal_winner,
        .delta = delta_winner,
        .segment_id = cand.ids[winner],
        .multi_support = multi,
    };
}

struct HostSignedStorage {
    std::vector<double> x;
    std::vector<double> z;
    std::vector<double> edge_x;
    std::vector<double> edge_z;
    std::vector<double> length2;
    std::vector<double> normal_x;
    std::vector<double> normal_z;

    std::vector<float> x_f32;
    std::vector<float> z_f32;
    std::vector<float> edge_x_f32;
    std::vector<float> edge_z_f32;
    std::vector<float> length2_f32;
    std::vector<float> normal_x_f32;
    std::vector<float> normal_z_f32;

    [[nodiscard]] SignedProfileView<double> view_f64() const noexcept {
        return {
            .x = x.data(),
            .z = z.data(),
            .edge_x = edge_x.data(),
            .edge_z = edge_z.data(),
            .length2 = length2.data(),
            .normal_x = normal_x.data(),
            .normal_z = normal_z.data(),
            .num_vertices = static_cast<i32>(x.size()),
        };
    }

    [[nodiscard]] SignedProfileView<float> view_f32() const noexcept {
        return {
            .x = x_f32.data(),
            .z = z_f32.data(),
            .edge_x = edge_x_f32.data(),
            .edge_z = edge_z_f32.data(),
            .length2 = length2_f32.data(),
            .normal_x = normal_x_f32.data(),
            .normal_z = normal_z_f32.data(),
            .num_vertices = static_cast<i32>(x_f32.size()),
        };
    }
};

[[nodiscard]] HostSignedStorage build_signed_storage(
    const std::vector<double> &x_in,
    const std::vector<double> &z_in
) {
    HostSignedStorage s;
    s.x = x_in;
    s.z = z_in;
    const std::size_t nseg = s.x.size() - 1;
    s.edge_x.resize(nseg);
    s.edge_z.resize(nseg);
    s.length2.resize(nseg);
    s.normal_x.resize(nseg);
    s.normal_z.resize(nseg);
    for (std::size_t i = 0; i < nseg; ++i) {
        const double ex = s.x[i + 1] - s.x[i];
        const double ez = s.z[i + 1] - s.z[i];
        s.edge_x[i] = ex;
        s.edge_z[i] = ez;
        double len2 = 0.0;
        len2 += ex * ex;
        len2 += ez * ez;
        s.length2[i] = len2;
        const double len = std::sqrt(len2);
        s.normal_x[i] = -ez / len;
        s.normal_z[i] = ex / len;
    }
    const auto to_f32 = [](const std::vector<double> &src) {
        std::vector<float> dst(src.size());
        for (std::size_t i = 0; i < src.size(); ++i) {
            dst[i] = static_cast<float>(src[i]);
        }
        return dst;
    };
    s.x_f32 = to_f32(s.x);
    s.z_f32 = to_f32(s.z);
    s.edge_x_f32 = to_f32(s.edge_x);
    s.edge_z_f32 = to_f32(s.edge_z);
    s.length2_f32 = to_f32(s.length2);
    s.normal_x_f32 = to_f32(s.normal_x);
    s.normal_z_f32 = to_f32(s.normal_z);
    return s;
}

struct QueryCase {
    double cx = 0.0;
    double cz = 0.0;
    double radius = 0.36;
    std::optional<int> prev_seg;
};

[[nodiscard]] std::pair<std::vector<double>, std::vector<double>> load_track_polyline(
    const std::filesystem::path &track_path
) {
    const bike::terrain::TrackSpec track = bike::terrain::load_track(track_path);
    const bike::terrain::HeightFieldSpec hf = bike::terrain::HeightFieldSpec::for_track(track);
    std::vector<double> px = hf.track_x();
    std::vector<double> pz = bike::terrain::build_profile(track, px);
    return {std::move(px), std::move(pz)};
}

void verify_polyline_queries(
    std::string_view label,
    const HostPolylineStorage &st,
    const std::vector<QueryCase> &queries,
    std::size_t &total_checked
) {
    const biketyre::ProfileQuery pq(
        st.px, st.pz, st.significant_delta_m, st.significance_fraction,
        std::acos(st.normal_cosine) * (180.0 / std::numbers::pi));
    const PolylineView<double> view_d = st.view_f64();
    const PolylineView<float> view_f = st.view_f32();

    for (std::size_t i = 0; i < queries.size(); ++i) {
        const QueryCase &q = queries[i];
        const biketyre::ProfileContact ref =
            reference_contact_legacy(st, {q.cx, q.cz}, q.radius, q.prev_seg);
        const biketyre::ProfileContact pq_got =
            pq.contact({q.cx, q.cz}, q.radius, q.prev_seg);

        PolylineContactResult<double> scan_d{};
        ErrCode err_d = ErrCode::none;
        const Status st_d = scan_polyline_contact<double, CpuExactToolkit, O1CpuMultiSupport>(
            view_d, q.cx, q.cz, q.radius, q.prev_seg.has_value(),
            q.prev_seg.value_or(-1), scan_d, err_d);
        bt::require(st_d == Status::ok && err_d == ErrCode::none,
                    std::string(label) + " scan_d failed at query " + std::to_string(i));

        // 0-bit parity between scan_polyline_contact<double>, ProfileQuery::contact, and legacy reference
        require_bits_f64(scan_d.point[0], ref.point[0], "scan_d point[0] vs legacy");
        require_bits_f64(scan_d.point[1], ref.point[1], "scan_d point[1] vs legacy");
        require_bits_f64(scan_d.normal[0], ref.normal[0], "scan_d normal[0] vs legacy");
        require_bits_f64(scan_d.normal[1], ref.normal[1], "scan_d normal[1] vs legacy");
        require_bits_f64(scan_d.delta, ref.delta, "scan_d delta vs legacy");
        bt::require(scan_d.segment_id == ref.segment_id, "scan_d segment_id vs legacy");
        bt::require((scan_d.multi_support != 0) == ref.multi_support,
                    "scan_d multi_support vs legacy");

        require_bits_f64(pq_got.point[0], ref.point[0], "pq point[0] vs legacy");
        require_bits_f64(pq_got.point[1], ref.point[1], "pq point[1] vs legacy");
        require_bits_f64(pq_got.normal[0], ref.normal[0], "pq normal[0] vs legacy");
        require_bits_f64(pq_got.normal[1], ref.normal[1], "pq normal[1] vs legacy");
        require_bits_f64(pq_got.delta, ref.delta, "pq delta vs legacy");
        bt::require(pq_got.segment_id == ref.segment_id, "pq segment_id vs legacy");
        bt::require(pq_got.multi_support == ref.multi_support, "pq multi_support vs legacy");

        // FP32 GpuToolkit sanity + bounded tolerance check
        const float cx_f = static_cast<float>(q.cx);
        const float cz_f = static_cast<float>(q.cz);
        const float rad_f = static_cast<float>(q.radius);
        // Only run f32 check when static_cast<float> query stays inside f32 road domain & above road
        const float h_f = static_cast<float>(biketyre::np_interp(q.cx, st.px, st.pz));
        if (st.px_f32.front() <= cx_f && cx_f <= st.px_f32.back() &&
            cz_f - h_f > 1e-4f) {
            PolylineContactResult<float> scan_f{};
            ErrCode err_f = ErrCode::none;
            const Status st_f = scan_polyline_contact<float, GpuToolkit, NoMultiSupport>(
                view_f, cx_f, cz_f, rad_f, q.prev_seg.has_value(),
                q.prev_seg.value_or(-1), scan_f, err_f);
            bt::require(st_f == Status::ok && err_f == ErrCode::none,
                        std::string(label) + " scan_f failed at query " + std::to_string(i) +
                            " status=" + std::to_string(static_cast<int>(st_f)) +
                            " err=" + std::to_string(static_cast<int>(err_f)) +
                            " cx=" + std::to_string(cx_f) + " cz=" + std::to_string(cz_f) +
                            " h_f=" + std::to_string(h_f));
            bt::require(std::isfinite(scan_f.point[0]) && std::isfinite(scan_f.point[1]) &&
                            std::isfinite(scan_f.normal[0]) && std::isfinite(scan_f.normal[1]) &&
                            std::isfinite(scan_f.delta),
                        "scan_f produced non-finite output");
            bt::require(std::abs(scan_f.delta - static_cast<float>(ref.delta)) <= 5.0e-5f,
                        "scan_f delta outside 5e-5f tolerance");
        }
        ++total_checked;
    }
}

void test_polyline_track_and_adversarial_parity() {
    const std::filesystem::path flat_path = bt::fixture("tracks/flat.toml");
    std::filesystem::path repo_root = flat_path;
    for (int i = 0; i < 4; ++i) {
        repo_root = repo_root.parent_path();
    }
    const std::filesystem::path rough_path =
        repo_root / "examples" / "research" / "rough_uphill_extreme.toml";

    std::size_t total_checked = 0;

    // 1. flat.toml (~5,000 queries: flat fast-path, airborne, prev_seg tie-breaks, endpoints)
    {
        auto [px, pz] = load_track_polyline(flat_path);
        const HostPolylineStorage st = build_polyline_storage(
            std::move(px), std::move(pz), 0.005, 0.25, 25.0);
        std::vector<QueryCase> queries;
        queries.reserve(5200);
        const double xmin = st.px.front();
        const double xmax = st.px.back();
        const int nseg = static_cast<int>(st.px.size()) - 1;
        for (int k = 0; k < 5000; ++k) {
            const double frac = static_cast<double>(k) / 4999.0;
            const double cx = xmin + frac * (xmax - xmin);
            const double h = biketyre::np_interp(cx, st.px, st.pz);
            const double dz = (k % 5 == 0) ? 0.52 : (0.05 + 0.30 * ((k * 37) % 100) / 100.0);
            std::optional<int> prev;
            if (k % 3 == 0) {
                const int seg = std::clamp(
                    static_cast<int>(std::ranges::upper_bound(st.px, cx) - st.px.begin()) - 1,
                    0, nseg - 1);
                prev = (k % 6 == 0) ? std::max(0, seg - 1) : seg;
            }
            queries.push_back({.cx = cx, .cz = h + dz, .radius = 0.36, .prev_seg = prev});
        }
        // Exact vertices & boundary queries
        for (int idx : {0, 1, nseg / 2, nseg - 1, nseg}) {
            const double vx = st.px[static_cast<std::size_t>(idx)];
            const double vz = st.pz[static_cast<std::size_t>(idx)];
            queries.push_back({.cx = vx, .cz = vz + 0.35, .radius = 0.36, .prev_seg = std::min(idx, nseg - 1)});
            queries.push_back({.cx = vx, .cz = vz + 0.60, .radius = 0.36, .prev_seg = std::max(0, idx - 1)});
        }
        verify_polyline_queries("flat.toml", st, queries, total_checked);
    }

    // 2. rough_uphill_extreme.toml (~12,000 queries: rough terrain, airborne expansion, prev_seg)
    {
        auto [px, pz] = load_track_polyline(rough_path);
        const HostPolylineStorage st = build_polyline_storage(
            std::move(px), std::move(pz), 0.005, 0.25, 25.0);
        std::vector<QueryCase> queries;
        queries.reserve(12200);
        const double xmin = st.px.front();
        const double xmax = st.px.back();
        const int nseg = static_cast<int>(st.px.size()) - 1;
        for (int k = 0; k < 12000; ++k) {
            const double frac = static_cast<double>(k) / 11999.0;
            const double cx = xmin + frac * (xmax - xmin);
            const double h = biketyre::np_interp(cx, st.px, st.pz);
            const double dz = (k % 7 == 0) ? 0.55 : (0.04 + 0.31 * ((k * 73) % 100) / 100.0);
            std::optional<int> prev;
            if (k % 2 == 0) {
                const int seg = std::clamp(
                    static_cast<int>(std::ranges::upper_bound(st.px, cx) - st.px.begin()) - 1,
                    0, nseg - 1);
                prev = std::clamp(seg + ((k % 4 == 0) ? -1 : 1), 0, nseg - 1);
            }
            queries.push_back({.cx = cx, .cz = h + dz, .radius = 0.36, .prev_seg = prev});
        }
        verify_polyline_queries("rough_uphill_extreme.toml", st, queries, total_checked);
    }

    // 3. Adversarial synthetic polylines (~4,000+ queries):
    //    - V-notch and ridge for unkept endpoint_keep == false winning argmin
    //    - Exact/near 1e-12 ties with previous_segment (kept and unkept)
    //    - Multi-support steep corners
    //    - Empty Pass 1 window & airborne expansion
    //    - Flat-window fast path with previous_segment tie-break
    {
        std::vector<double> px = {
            0.0, 0.5, 1.0, 1.4, 1.6, 2.0, 2.5, 2.501, 3.0, 3.5, 4.0, 4.5, 5.0, 6.0, 7.0
        };
        std::vector<double> pz = {
            0.0, 0.0, 0.0, 0.25, 0.0, 0.35, 0.10, 0.30, 0.05, 0.45, 0.15, 0.15, 0.15, 0.0, 0.40
        };
        const HostPolylineStorage st = build_polyline_storage(
            std::move(px), std::move(pz), 0.001, 0.1, 15.0);
        std::vector<QueryCase> queries;
        queries.reserve(4500);

        // Sweep densely across corners, ridges, V-notches, and flat plateaus
        for (int k = 0; k < 4000; ++k) {
            const double cx = 0.01 + (6.98 * static_cast<double>(k)) / 3999.0;
            const double h = biketyre::np_interp(cx, st.px, st.pz);
            const double dz = 0.02 + 0.55 * static_cast<double>((k * 29) % 100) / 100.0;
            const std::optional<int> prev =
                (k % 3 == 0) ? std::optional<int>(k % (static_cast<int>(st.px.size()) - 1))
                             : std::nullopt;
            queries.push_back({.cx = cx, .cz = h + dz, .radius = 0.36, .prev_seg = prev});
        }

        // Exact 1e-12 & near-1e-12 tie-break queries around ridge vertex x=2.0 (segments 4 & 5)
        // and flat vertex x=4.5 (segments 10 & 11)
        const std::array<double, 7> epsilons = {
            -2.0e-12, -1.0e-12, -5.0e-13, 0.0, 5.0e-13, 1.0e-12, 2.0e-12
        };
        for (const double eps : epsilons) {
            queries.push_back({.cx = 4.5 + eps, .cz = 0.35, .radius = 0.36, .prev_seg = 10});
            queries.push_back({.cx = 4.5 + eps, .cz = 0.35, .radius = 0.36, .prev_seg = 11});
            queries.push_back({.cx = 2.0 + eps, .cz = 0.60, .radius = 0.36, .prev_seg = 4});
            queries.push_back({.cx = 2.0 + eps, .cz = 0.60, .radius = 0.36, .prev_seg = 5});
            // Unkept interior vertex x=1.4 (peak between seg 2 and seg 3):
            queries.push_back({.cx = 1.4 + eps, .cz = 0.50, .radius = 0.36, .prev_seg = 2});
            queries.push_back({.cx = 1.4 + eps, .cz = 0.50, .radius = 0.36, .prev_seg = 3});
        }
        // Explicitly test unkept interior vertex winning argmin in a symmetric V-ridge
        // and airborne expansion where Pass 1 window has min_dist >= radius
        queries.push_back({.cx = 1.6, .cz = 0.30, .radius = 0.36, .prev_seg = std::nullopt});
        queries.push_back({.cx = 1.6, .cz = 0.30, .radius = 0.36, .prev_seg = 3});
        queries.push_back({.cx = 1.6, .cz = 0.30, .radius = 0.36, .prev_seg = 4});
        queries.push_back({.cx = 3.5, .cz = 2.50, .radius = 0.36, .prev_seg = std::nullopt});

        verify_polyline_queries("adversarial", st, queries, total_checked);
    }

    // 4. Dedicated unkept-vertex argmin test: two segments meet at a sharp crest (0,0)->(1,1)->(2,0)
    //    When center is directly above (1,1), both segments project to t=1 (seg 0) and t=0 (seg 1).
    //    Wait: at (1, 1.2), off=(0, 0.2), left=(-1,-1)->dl=-0.2<0, right=(1,-1)->dr=-0.2<0 (kept).
    //    At (1.1, 1.15), off=(0.1, 0.15): for seg 0 endpoint (1,1), t=1, right=(1,-1):
    //    dr = 0.1*1 + 0.15*(-1) = -0.05 <= 0 (kept).
    //    At (1.2, 1.05): for seg 0 endpoint (1,1), t=1, right=(1,-1):
    //    dr = 0.2*1 + 0.05*(-1) = 0.15 > 1e-14 -> endpoint_keep == false!
    //    And if seg 0 is tested with prev_seg = 0, prev_keep is false so tie-break must NOT switch;
    //    meanwhile on a steeper right slope where vertex still wins argmin (or unkept seg wins argmin),
    //    both must match bit-for-bit!
    {
        const HostPolylineStorage st = build_polyline_storage(
            {0.0, 1.0, 2.0, 3.0}, {0.0, 0.0, 0.8, 0.8}, 0.001, 0.1, 15.0);
        std::vector<QueryCase> queries;
        for (int k = 0; k < 200; ++k) {
            const double cx = 0.8 + 0.4 * static_cast<double>(k) / 199.0;
            const double h = biketyre::np_interp(cx, st.px, st.pz);
            queries.push_back({.cx = cx, .cz = h + 0.10, .radius = 0.36, .prev_seg = 0});
            queries.push_back({.cx = cx, .cz = h + 0.10, .radius = 0.36, .prev_seg = 1});
        }
        verify_polyline_queries("unkept_endpoint", st, queries, total_checked);
    }

    bt::require(total_checked >= 20000u, "must check at least 20,000 polyline queries");
}

void test_polyline_error_branches() {
    const HostPolylineStorage st = build_polyline_storage(
        {0.0, 1.0, 2.0}, {0.0, 0.0, 0.0}, 0.005, 0.25, 25.0);
    const PolylineView<double> view = st.view_f64();
    const biketyre::ProfileQuery pq(st.px, st.pz, 0.005, 0.25, 25.0);
    const double qnan = std::numeric_limits<double>::quiet_NaN();
    const double inf_p = std::numeric_limits<double>::infinity();

    struct ErrCase {
        double cx;
        double cz;
        double radius;
        bool has_prev;
        int prev;
        ErrCode expected_err;
        std::string_view expected_msg;
    };
    const std::array<ErrCase, 8> cases = {{
        {.cx = qnan, .cz = 0.5, .radius = 0.36, .has_prev = false, .prev = 0,
         .expected_err = ErrCode::nonfinite_center,
         .expected_msg = "invalid shape or non-finite wheel center"},
        {.cx = 1.0, .cz = inf_p, .radius = 0.36, .has_prev = false, .prev = 0,
         .expected_err = ErrCode::nonfinite_center,
         .expected_msg = "invalid shape or non-finite wheel center"},
        {.cx = 1.0, .cz = 0.5, .radius = 0.0, .has_prev = false, .prev = 0,
         .expected_err = ErrCode::invalid_radius,
         .expected_msg = "invalid wheel radius"},
        {.cx = 1.0, .cz = 0.5, .radius = -0.2, .has_prev = false, .prev = 0,
         .expected_err = ErrCode::invalid_radius,
         .expected_msg = "invalid wheel radius"},
        {.cx = 1.0, .cz = 0.5, .radius = 0.36, .has_prev = true, .prev = -1,
         .expected_err = ErrCode::invalid_prev_seg,
         .expected_msg = "invalid previous contact segment"},
        {.cx = 1.0, .cz = 0.5, .radius = 0.36, .has_prev = true, .prev = 2,
         .expected_err = ErrCode::invalid_prev_seg,
         .expected_msg = "invalid previous contact segment"},
        {.cx = -0.1, .cz = 0.5, .radius = 0.36, .has_prev = false, .prev = 0,
         .expected_err = ErrCode::outside_domain,
         .expected_msg = "wheel center is outside profile domain"},
        {.cx = 1.0, .cz = 0.0, .radius = 0.36, .has_prev = false, .prev = 0,
         .expected_err = ErrCode::solid_road,
         .expected_msg = "wheel center reached or entered solid road"},
    }};

    for (const ErrCase &c : cases) {
        PolylineContactResult<double> out{};
        ErrCode err = ErrCode::none;
        const Status status = scan_polyline_contact<double, CpuExactToolkit, O1CpuMultiSupport>(
            view, c.cx, c.cz, c.radius, c.has_prev, c.prev, out, err);
        bt::require(status == Status::invalid_argument && err == c.expected_err,
                    "unexpected ErrCode for " + std::string(c.expected_msg));
        bool caught = false;
        try {
            const std::optional<int> p = c.has_prev ? std::optional<int>(c.prev) : std::nullopt;
            static_cast<void>(pq.contact({c.cx, c.cz}, c.radius, p));
        } catch (const std::invalid_argument &ex) {
            caught = (std::string_view(ex.what()) == c.expected_msg);
        }
        bt::require(caught, "ProfileQuery::contact message mismatch for " + std::string(c.expected_msg));
    }

    // Degenerate contact (distance <= 1e-12) on both flat fast-path and candidate path
    {
        PolylineContactResult<double> out{};
        ErrCode err = ErrCode::none;
        const Status status = scan_polyline_contact<double, CpuExactToolkit, O1CpuMultiSupport>(
            view, 1.0, 5.0e-13, 0.36, false, -1, out, err);
        bt::require(status == Status::invalid_argument && err == ErrCode::degenerate_contact,
                    "flat degenerate_contact expected");
        bool caught = false;
        try {
            static_cast<void>(pq.contact({1.0, 5.0e-13}, 0.36, std::nullopt));
        } catch (const std::invalid_argument &ex) {
            caught = (std::string_view(ex.what()) == "unsupported or degenerate contact geometry");
        }
        bt::require(caught, "ProfileQuery::contact flat degenerate message mismatch");
    }
    {
        const HostPolylineStorage slope_st = build_polyline_storage(
            {0.0, 1.0, 2.0}, {0.0, 0.1, 0.2}, 0.005, 0.25, 25.0);
        const biketyre::ProfileQuery slope_pq(slope_st.px, slope_st.pz, 0.005, 0.25, 25.0);
        PolylineContactResult<double> out{};
        ErrCode err = ErrCode::none;
        const Status status = scan_polyline_contact<double, CpuExactToolkit, O1CpuMultiSupport>(
            slope_st.view_f64(), 1.0, 0.1 + 5.0e-13, 0.36, false, -1, out, err);
        bt::require(status == Status::invalid_argument && err == ErrCode::degenerate_contact,
                    "candidate degenerate_contact expected");
        bool caught = false;
        try {
            static_cast<void>(slope_pq.contact({1.0, 0.1 + 5.0e-13}, 0.36, std::nullopt));
        } catch (const std::invalid_argument &ex) {
            caught = (std::string_view(ex.what()) == "unsupported or degenerate contact geometry");
        }
        bt::require(caught, "ProfileQuery::contact candidate degenerate message mismatch");
    }
}

void test_signed_profile_parity() {
    const std::filesystem::path flat_path = bt::fixture("tracks/flat.toml");
    std::filesystem::path repo_root = flat_path;
    for (int i = 0; i < 4; ++i) {
        repo_root = repo_root.parent_path();
    }
    const std::filesystem::path rough_path =
        repo_root / "examples" / "research" / "rough_uphill_extreme.toml";

    auto check_profile = [](const std::vector<double> &px, const std::vector<double> &pz,
                            std::string_view label) {
        std::vector<std::array<double, 2>> verts(px.size());
        for (std::size_t i = 0; i < px.size(); ++i) {
            verts[i] = {px[i], pz[i]};
        }
        const biketyre::SignedProfile sp(verts);
        const HostSignedStorage st = build_signed_storage(px, pz);
        const SignedProfileView<double> view_d = st.view_f64();
        const SignedProfileView<float> view_f = st.view_f32();

        const double xmin = px.front();
        const double xmax = px.back();
        for (int k = 0; k < 2500; ++k) {
            const double qx = xmin + (xmax - xmin) * static_cast<double>(k) / 2499.0;
            const double h_ref = sp.height(qx);
            double h_got = 0.0;
            ErrCode herr = ErrCode::none;
            const Status hst = profile_height<double, CpuExactToolkit>(view_d, qx, h_got, herr);
            bt::require(hst == Status::ok && herr == ErrCode::none,
                        std::string(label) + " profile_height failed");
            require_bits_f64(h_got, h_ref, "profile_height vs SignedProfile::height");

            const double dz = -0.4 + 0.9 * static_cast<double>((k * 43) % 100) / 99.0;
            const double qz = h_ref + dz;
            const biketyre::PointDistance d_ref = sp.query({qx, qz});
            PointDistanceResult<double> d_got{};
            ErrCode derr = ErrCode::none;
            const Status dst = signed_profile_query<double, CpuExactToolkit>(
                view_d, qx, qz, d_got, derr);
            bt::require(dst == Status::ok && derr == ErrCode::none,
                        std::string(label) + " signed_profile_query failed");

            require_bits_f64(d_got.distance, d_ref.distance, "signed_distance");
            require_bits_f64(d_got.normal[0], d_ref.normal[0], "signed_normal[0]");
            require_bits_f64(d_got.normal[1], d_ref.normal[1], "signed_normal[1]");
            require_bits_f64(d_got.point[0], d_ref.point[0], "signed_point[0]");
            require_bits_f64(d_got.point[1], d_ref.point[1], "signed_point[1]");
            bt::require(d_got.segment == d_ref.segment, "signed_segment");
            bt::require((d_got.nonsmooth != 0) == d_ref.nonsmooth, "signed_nonsmooth");

            const float qx_f = static_cast<float>(qx);
            const float qz_f = static_cast<float>(qz);
            if (st.x_f32.front() <= qx_f && qx_f <= st.x_f32.back()) {
                float hf = 0.0f;
                ErrCode herr_f = ErrCode::none;
                const Status hst_f = profile_height<float, GpuToolkit>(view_f, qx_f, hf, herr_f);
                bt::require(hst_f == Status::ok && herr_f == ErrCode::none && std::isfinite(hf),
                            "profile_height<float> failed");
                PointDistanceResult<float> df{};
                ErrCode derr_f = ErrCode::none;
                const Status dst_f = signed_profile_query<float, GpuToolkit>(
                    view_f, qx_f, qz_f, df, derr_f);
                bt::require(dst_f == Status::ok && derr_f == ErrCode::none &&
                                std::isfinite(df.distance) && std::isfinite(df.normal[0]) &&
                                std::isfinite(df.normal[1]),
                            "signed_profile_query<float> failed");
                bt::require(std::abs(df.distance - static_cast<float>(d_ref.distance)) <= 5.0e-5f,
                            "signed_profile_query<float> distance outside 5e-5f");
            }
        }
    };

    {
        auto [px, pz] = load_track_polyline(flat_path);
        check_profile(px, pz, "flat.toml");
    }
    {
        auto [px, pz] = load_track_polyline(rough_path);
        check_profile(px, pz, "rough_uphill_extreme.toml");
    }

    // Corner, collinear tie, length <= 1e-12 floor, and error gates
    {
        const std::vector<double> px = {0.0, 1.0, 2.0, 3.0};
        const std::vector<double> pz = {0.0, 1.0, 0.0, 0.0};
        const biketyre::SignedProfile sp({{0.0, 0.0}, {1.0, 1.0}, {2.0, 0.0}, {3.0, 0.0}});
        const HostSignedStorage st = build_signed_storage(px, pz);
        const SignedProfileView<double> view = st.view_f64();

        const std::array<std::array<double, 2>, 5> probes = {{
            {1.0, 1.0},             // corner exact (length == 0 <= 1e-12)
            {1.0, 1.0 + 5.0e-13},   // corner within 1e-12 floor
            {2.0, 0.0},             // second corner exact
            {2.5, 0.0},             // flat interior exact
            {1.0, 0.5},             // below corner
        }};
        for (const auto &p : probes) {
            const biketyre::PointDistance ref = sp.query(p);
            PointDistanceResult<double> got{};
            ErrCode err = ErrCode::none;
            const Status status = signed_profile_query<double, CpuExactToolkit>(
                view, p[0], p[1], got, err);
            bt::require(status == Status::ok && err == ErrCode::none, "corner probe failed");
            require_bits_f64(got.distance, ref.distance, "corner probe distance");
            require_bits_f64(got.normal[0], ref.normal[0], "corner probe normal[0]");
            require_bits_f64(got.normal[1], ref.normal[1], "corner probe normal[1]");
            require_bits_f64(got.point[0], ref.point[0], "corner probe point[0]");
            require_bits_f64(got.point[1], ref.point[1], "corner probe point[1]");
            bt::require(got.segment == ref.segment, "corner probe segment");
            bt::require((got.nonsmooth != 0) == ref.nonsmooth, "corner probe nonsmooth");
        }

        double h = 0.0;
        ErrCode err = ErrCode::none;
        bt::require(profile_height<double, CpuExactToolkit>(view, -0.1, h, err) ==
                        Status::invalid_argument &&
                    err == ErrCode::outside_domain,
                    "profile_height outside_domain");
        PointDistanceResult<double> out{};
        bt::require(signed_profile_query<double, CpuExactToolkit>(
                        view, std::numeric_limits<double>::quiet_NaN(), 0.0, out, err) ==
                        Status::invalid_argument &&
                    err == ErrCode::nonfinite_center,
                    "signed_profile_query nonfinite_center");
        bt::require(signed_profile_query<double, CpuExactToolkit>(
                        view, 3.1, 0.0, out, err) ==
                        Status::invalid_argument &&
                    err == ErrCode::outside_domain,
                    "signed_profile_query outside_domain");
    }
}

Status run_warm_nonblocking_step(
    const PolylineView<double> &pl,
    const SignedProfileView<double> &sp,
    PolylineContactResult<double> &contact_out,
    PointDistanceResult<double> &dist_out,
    double &height_out,
    ErrCode &err_out
) noexcept BIKE_NONBLOCKING {
    Status s = scan_polyline_contact<double, CpuExactToolkit, O1CpuMultiSupport>(
        pl, 1.25, 0.30, 0.36, true, 0, contact_out, err_out);
    if (s != Status::ok) {
        return s;
    }
    s = profile_height<double, CpuExactToolkit>(sp, 1.25, height_out, err_out);
    if (s != Status::ok) {
        return s;
    }
    return signed_profile_query<double, CpuExactToolkit>(sp, 1.25, 0.25, dist_out, err_out);
}

void test_nonblocking_warm_path() {
    const HostPolylineStorage pl_st = build_polyline_storage(
        {0.0, 1.0, 2.0, 3.0}, {0.0, 0.1, 0.0, 0.2}, 0.005, 0.25, 25.0);
    const HostSignedStorage sp_st = build_signed_storage(
        {0.0, 1.0, 2.0, 3.0}, {0.0, 0.1, 0.0, 0.2});
    PolylineContactResult<double> contact_out{};
    PointDistanceResult<double> dist_out{};
    double height_out = 0.0;
    ErrCode err_out = ErrCode::none;
    const Status s = run_warm_nonblocking_step(
        pl_st.view_f64(), sp_st.view_f64(), contact_out, dist_out, height_out, err_out);
    bt::require(s == Status::ok && err_out == ErrCode::none, "BIKE_NONBLOCKING warm step failed");
    bt::require(contact_out.delta > 0.0 && dist_out.distance > 0.0, "warm outputs valid");
}

} // namespace

int main() {
    return bt::run([] {
        test_polyline_track_and_adversarial_parity();
        test_polyline_error_branches();
        test_signed_profile_parity();
        test_nonblocking_warm_path();
    });
}
