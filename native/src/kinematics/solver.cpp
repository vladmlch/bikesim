// kinematics/solver.cpp — solver.py's HorstLinkageSolver, statement
// for statement. The pivot solve keeps the source's op order: circle
// intersection pick (`sol1 if sol1[1] >= sol2[1] else sol2`), the
// xz-plane 2-vector arithmetic, scalar*vector then vector+vector
// decomposition, and the transmission-angle dot/norm clip acos chain.
// The wheel-travel solve brackets and calls the same brentq the
// oracle does; the stroke solve keeps the cached stroke table and
// np.interp semantics (detail::np_interp).
#include "solver.hpp"

#include "../diag.hpp"
#include "../numeric_sincos.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace bike::kinematics {

namespace {
    using geometry::Vec2;
    using geometry::Vec3;
    namespace gd = geometry::detail;

    [[nodiscard]] inline Vec3 sub3(const Vec3 &a, const Vec3 &b) noexcept {
        return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }
    [[nodiscard]] inline Vec3 lift(const Vec2 &v) noexcept {
        return {v[0], 0.0, v[1]};
    }
} // namespace

// ---------------------------------------------------------------------------
// __init__ — the source's attribute order.
// ---------------------------------------------------------------------------

HorstLinkageSolver::HorstLinkageSolver(
        std::optional<geometry::BikeSpecs> specs,
        const ReferencePoints &reference)
        : specs_(specs.value_or(geometry::BikeSpecs{})) {
    const geometry::FixedFramePoints fixed =
            geometry::get_fixed_frame_points(specs_);

    // Fixed Frame Pivots (mm) — np.array(x if x is not None else fixed[..]).
    p0_ = reference.p0.value_or(fixed.P0);
    p5_ = reference.p5.value_or(fixed.P5);
    p7_ = reference.p7.value_or(fixed.P7);
    bb_ = fixed.BB;
    p_fa_ = geometry::compute_front_axle(specs_);
    p11_ = geometry::compute_headtube_points(specs_).top;

    // Uncompressed Reference Coordinates (mm).
    p1_0_ = reference.p1_0.value_or(geometry::compute_rear_axle(specs_));
    p2_0_ = reference.p2_0.value_or(
            Vec3{-379.118647, 0.0, -8.442119});
    p3_0_ = reference.p3_0.value_or(
            Vec3{-71.715477, 0.0, 206.598467});
    p4_0_ = reference.p4_0.value_or(
            Vec3{-38.545800, 0.0, 191.462565});
    p6_0_ = reference.p6_0.value_or(
            Vec3{28.408048, 0.0, 258.549831});
    p12_0_ = reference.p12_0.value_or(
            Vec3{-414.000688, 0.0, 59.986864});

    // Rigid Link Length Invariants (mm) — float(np.linalg.norm(a - b)).
    l_cs_ = gd::np_norm3(sub3(p2_0_, p0_));
    l_ss_ = gd::np_norm3(sub3(p3_0_, p2_0_));
    r_53_ = gd::np_norm3(sub3(p3_0_, p5_));
    r_54_ = gd::np_norm3(sub3(p4_0_, p5_));
    r_21_ = gd::np_norm3(sub3(p1_0_, p2_0_));
    r_2_12_ = gd::np_norm3(sub3(p12_0_, p2_0_));
    l_dropout_height_ = gd::np_norm3(sub3(p1_0_, p12_0_));
    l_ss_tube_ = gd::np_norm3(sub3(p3_0_, p12_0_));
    l_yoke_ = gd::np_norm3(sub3(p6_0_, p4_0_));
    l_shock_0_ = gd::np_norm3(sub3(p7_, p6_0_));

    // Reference Angles (radians).
    // NOLINTNEXTLINE(cppcoreguidelines-prefer-member-initializer) theta_cs_0_ depends on the body-assigned p2_0_/p0_
    theta_cs_0_ = std::atan2(p2_0_[2] - p0_[2], p2_0_[0] - p0_[0]);

    // Rocker link rigid angular offset beta = angle(P4-P5) - angle(P3-P5).
    const Vec2 v53_0{p3_0_[0] - p5_[0], p3_0_[2] - p5_[2]};
    const Vec2 v54_0{p4_0_[0] - p5_[0], p4_0_[2] - p5_[2]};
    beta_rocker_ = std::atan2(v54_0[1], v54_0[0]) -
                   std::atan2(v53_0[1], v53_0[0]);

    // Seatstay link rigid angular offsets.
    const Vec2 v23_0{p3_0_[0] - p2_0_[0], p3_0_[2] - p2_0_[2]};
    const Vec2 v21_0{p1_0_[0] - p2_0_[0], p1_0_[2] - p2_0_[2]};
    const Vec2 v2_12_0{p12_0_[0] - p2_0_[0], p12_0_[2] - p2_0_[2]};
    gamma_ra_ = std::atan2(v21_0[1], v21_0[0]) -
                std::atan2(v23_0[1], v23_0[0]);
    gamma_12_ = std::atan2(v2_12_0[1], v2_12_0[0]) -
                std::atan2(v23_0[1], v23_0[0]);
}

// ---------------------------------------------------------------------------
// _solve_pivots_from_theta_cs — the 9-tuple solve.
// ---------------------------------------------------------------------------

// Returned by value deliberately (the source returns tuples/dicts and
// the records carry the same fields); -Wlarge-by-value-copy cannot
// split by-value parameters (its useful half) from return values —
// same suppression precedent as writers/tire.cpp's make_patch. One
// push/pop region covers the pivot tuple and the state/trajectory
// builders below, all >64-byte returns.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wlarge-by-value-copy")
HorstLinkageSolver::SolvedPivots
HorstLinkageSolver::solve_pivots_(double theta_cs) const {
    // 1. Horst Pivot P2 on Circle(P0, L_cs) — np.cos/np.sin of one
    // operand: two independent libm calls (numeric_sincos anti-fusion).
    const Vec3 p2{p0_[0] + l_cs_ * numeric::np_cos(theta_cs), 0.0,
                  p0_[2] + l_cs_ * numeric::np_sin(theta_cs)};
    const Vec2 p2_xz{p2[0], p2[2]};

    // 2. Circle-Circle Intersection for P3:
    //    Circle(P2, L_ss) & Circle(P5, R_53).
    const auto [sol1, sol2] =
            circle_circle_intersection_2d(p2, l_ss_, p5_, r_53_);
    const Vec2 p3_xz = sol1[1] >= sol2[1] ? sol1 : sol2;

    // 3. Rocker-Yoke Pivot P4 on Rocker Body (rotating about P5).
    const Vec2 v53{p3_xz[0] - p5_[0], p3_xz[1] - p5_[2]};
    const double n53 = gd::np_norm2(v53);
    const Vec2 u53{v53[0] / n53, v53[1] / n53};
    const double cos_b = numeric::np_cos(beta_rocker_);
    const double sin_b = numeric::np_sin(beta_rocker_);
    const Vec2 u54{cos_b * u53[0] - sin_b * u53[1],
                   sin_b * u53[0] + cos_b * u53[1]};
    const Vec2 p4_xz{p5_[0] + r_54_ * u54[0], p5_[2] + r_54_ * u54[1]};

    // 4. Rear Axle P1 and Dropout Upper Kink P12 on Seatstay Body.
    const Vec2 v23{p3_xz[0] - p2_xz[0], p3_xz[1] - p2_xz[1]};
    const double n23 = gd::np_norm2(v23);
    const Vec2 u23{v23[0] / n23, v23[1] / n23};
    const double cos_g = numeric::np_cos(gamma_ra_);
    const double sin_g = numeric::np_sin(gamma_ra_);
    const Vec2 u21{cos_g * u23[0] - sin_g * u23[1],
                   sin_g * u23[0] + cos_g * u23[1]};
    const Vec2 p1_xz{p2_xz[0] + r_21_ * u21[0],
                     p2_xz[1] + r_21_ * u21[1]};

    const double cos_g12 = numeric::np_cos(gamma_12_);
    const double sin_g12 = numeric::np_sin(gamma_12_);
    const Vec2 u2_12{cos_g12 * u23[0] - sin_g12 * u23[1],
                     sin_g12 * u23[0] + cos_g12 * u23[1]};
    const Vec2 p12_xz{p2_xz[0] + r_2_12_ * u2_12[0],
                      p2_xz[1] + r_2_12_ * u2_12[1]};

    // 5. Shock Lower Eyelet P6 along Line of Action (P4 -> P7).
    const Vec2 v47{p7_[0] - p4_xz[0], p7_[2] - p4_xz[1]};
    const double d47 = gd::np_norm2(v47);
    const Vec2 u47{v47[0] / d47, v47[1] / d47};
    const Vec2 p6_xz{p4_xz[0] + l_yoke_ * u47[0],
                     p4_xz[1] + l_yoke_ * u47[1]};

    // 7. Transmission angle — np.dot(v23, v53) / (|v23| * |v53|), the
    // norms recomputed exactly like the source re-evaluates them.
    const double cos_mu =
            gd::np_dot2(v23, v53) / (n23 * gd::np_norm2(v53));
    const double transmission_angle_deg = gd::np_degrees(
            std::acos(gd::np_clip_scalar(cos_mu, -1.0, 1.0)));

    return SolvedPivots{.p1 = lift(p1_xz), .p2 = p2,
                        .p3 = lift(p3_xz), .p4 = lift(p4_xz),
                        .p6 = lift(p6_xz), .p12 = lift(p12_xz),
                        .u47 = u47, .d47 = d47,
                        .transmission_angle_deg = transmission_angle_deg};
}

// ---------------------------------------------------------------------------
// solve_state_from_chainstay_angle — the full state dict.
// ---------------------------------------------------------------------------

KinematicState HorstLinkageSolver::solve_state_from_chainstay_angle(
        double theta_cs) const {
    const SolvedPivots s = solve_pivots_(theta_cs);

    const double shock_length = s.d47 - l_yoke_;
    const double shock_stroke = l_shock_0_ - shock_length;
    const double wheel_travel = s.p1[2] - p1_0_[2];

    const double lr = instantaneous_lr_(theta_cs, s.p1, s.p2, s.p3,
                                        s.p4, s.u47);

    // link_errors dict — abs(np.linalg.norm(a - b) - l) per link.
    const LinkErrors errors{
            .chainstay = std::fabs(
                    gd::np_norm3(sub3(s.p2, p0_)) - l_cs_),
            .seatstay = std::fabs(
                    gd::np_norm3(sub3(s.p3, s.p2)) - l_ss_),
            .rocker_p5_p3 = std::fabs(
                    gd::np_norm3(sub3(s.p3, p5_)) - r_53_),
            .rocker_p5_p4 = std::fabs(
                    gd::np_norm3(sub3(s.p4, p5_)) - r_54_),
            .axle_offset = std::fabs(
                    gd::np_norm3(sub3(s.p1, s.p2)) - r_21_),
            .dropout_upper = std::fabs(
                    gd::np_norm3(sub3(s.p12, s.p2)) - r_2_12_),
            .dropout_height = std::fabs(
                    gd::np_norm3(sub3(s.p1, s.p12)) - l_dropout_height_),
            .yoke = std::fabs(
                    gd::np_norm3(sub3(s.p6, s.p4)) - l_yoke_)};

    return KinematicState{
            .P0 = p0_,
            .P1 = s.p1,
            .P2 = s.p2,
            .P3 = s.p3,
            .P4 = s.p4,
            .P5 = p5_,
            .P6 = s.p6,
            .P7 = p7_,
            .P12 = s.p12,
            .P_FA = p_fa_,
            .P11 = p11_,
            .BB = bb_,
            .theta_cs = theta_cs,
            .theta_cs_deg = gd::np_degrees(theta_cs),
            .wheel_travel = wheel_travel,
            .shock_stroke = shock_stroke,
            .shock_length = shock_length,
            .transmission_angle_deg = s.transmission_angle_deg,
            .leverage_ratio = lr,
            .link_errors = errors,
            .max_link_error = errors.max()};
}

// ---------------------------------------------------------------------------
// _compute_instantaneous_lr — the Jacobian virtual-work LR.
// ---------------------------------------------------------------------------

double HorstLinkageSolver::instantaneous_lr_(
        double theta_cs, const Vec3 &p1, const Vec3 &p2, const Vec3 &p3,
        const Vec3 &p4, const Vec2 &u47) const {
    // v2 = [-l_cs * sin(th), l_cs * cos(th)] — same-operand pair.
    const Vec2 v2{-l_cs_ * numeric::np_sin(theta_cs),
                  l_cs_ * numeric::np_cos(theta_cs)};

    const Vec2 r23{p3[0] - p2[0], p3[2] - p2[2]};
    const Vec2 r53{p3[0] - p5_[0], p3[2] - p5_[2]};
    const Vec2 r54{p4[0] - p5_[0], p4[2] - p5_[2]};
    const Vec2 r21{p1[0] - p2[0], p1[2] - p2[2]};

    const double cross_53_23 =
            -r53[1] * r23[0] + r53[0] * r23[1];
    if (std::fabs(cross_53_23) < 1e-12) return 0.0;

    const double omega_roc = gd::np_dot2(v2, r23) / cross_53_23;
    const Vec2 w53{-r53[1], r53[0]};
    const Vec2 v3{omega_roc * w53[0], omega_roc * w53[1]};

    const double omega_ss =
            (-(v3[0] - v2[0]) * r23[1] +
             (v3[1] - v2[1]) * r23[0]) /
            (l_ss_ * l_ss_);

    const Vec2 w21{-r21[1], r21[0]};
    const Vec2 v1{v2[0] + omega_ss * w21[0],
                  v2[1] + omega_ss * w21[1]};
    const Vec2 w54{-r54[1], r54[0]};
    const Vec2 v4{omega_roc * w54[0], omega_roc * w54[1]};

    const double d_stroke = gd::np_dot2(u47, v4);
    const double d_travel_z = v1[1];

    if (std::fabs(d_stroke) < 1e-12) return 0.0;

    return d_travel_z / d_stroke;
}

// ---------------------------------------------------------------------------
// solve_state_from_wheel_travel — brentq on the pivot-solve objective.
// ---------------------------------------------------------------------------

KinematicState HorstLinkageSolver::solve_state_from_wheel_travel(
        double target_travel_z) const {
    // objective(th) — only P1[2] is needed to bracket the root, same
    // as the source's "build nothing else" comment path.
    const auto objective = [this, target_travel_z](double th) {
        const SolvedPivots s = solve_pivots_(th);
        return (s.p1[2] - p1_0_[2]) - target_travel_z;
    };

    const double bracket_a = theta_cs_0_ - 0.70; // ~40 deg upward
    const double bracket_b = theta_cs_0_ + 0.10; // ~5.7 deg extension

    const double theta_opt =
            brentq(objective, bracket_a, bracket_b, 1e-12);
    return solve_state_from_chainstay_angle(theta_opt);
}

// ---------------------------------------------------------------------------
// _stroke_table + solve_state_from_shock_stroke — the lazy interp map.
// ---------------------------------------------------------------------------

const std::pair<std::vector<double>, std::vector<double>> &
HorstLinkageSolver::stroke_table_() const {
    if (!stroke_table_cache_.has_value()) {
        std::vector<double> travel =
                gd::np_linspace(0.0, specs_.rear_wheel_travel, 101);
        std::vector<double> stroke;
        stroke.reserve(travel.size());
        for (const double tr: travel)
            stroke.push_back(
                    solve_state_from_wheel_travel(tr).shock_stroke);
        stroke_table_cache_.emplace(std::move(travel),
                                    std::move(stroke));
    }
    return *stroke_table_cache_;
}

KinematicState HorstLinkageSolver::solve_state_from_shock_stroke(
        double shock_stroke_mm) const {
    const auto &[sample_travel, sample_stroke] = stroke_table_();
    const double stroke_clamped = gd::np_clip_scalar(
            shock_stroke_mm, 0.0, specs_.shock_stroke);
    const double wheel_travel_est =
            gd::np_interp(stroke_clamped, sample_stroke, sample_travel);
    return solve_state_from_wheel_travel(wheel_travel_est);
}

// ---------------------------------------------------------------------------
// solve_trajectory — the linspace sweep + per-point arrays.
// ---------------------------------------------------------------------------

const std::vector<geometry::Vec3> &
KinematicTrajectory::point_array(std::string_view name) const {
    const std::array<const std::vector<geometry::Vec3> *,
                     kPointCount>
            refs{&P0, &P1, &P2, &P3, &P4, &P5, &P6, &P7, &P12, &P_FA,
                 &P11};
    for (std::size_t i = 0; i < kPointCount; ++i)
        if (kPointNames.at(i) == name) return *refs.at(i);
    throw std::out_of_range(
            "KinematicTrajectory: unknown point name");
}

KinematicTrajectory HorstLinkageSolver::solve_trajectory(
        int n_points, double max_travel) const {
    if (n_points < 0)
        throw std::invalid_argument(
                "Number of samples, " + std::to_string(n_points) +
                ", must be non-negative.");
    const std::vector<double> travel = gd::np_linspace(
            0.0, max_travel, static_cast<std::size_t>(n_points));
    std::vector<KinematicState> states;
    states.reserve(travel.size());
    for (const double z: travel)
        states.push_back(solve_state_from_wheel_travel(z));

    KinematicTrajectory t;
    const std::size_t n = states.size();
    t.wheel_travel = travel; // np.array(travel_array) copy
    t.shock_stroke.reserve(n);
    t.shock_length.reserve(n);
    t.leverage_ratio.reserve(n);
    t.transmission_angle_deg.reserve(n);
    t.axle_path_x.reserve(n);
    t.axle_path_z.reserve(n);
    t.chainstay_angle_deg.reserve(n);
    t.link_length_errors.reserve(n);
    t.P0.reserve(n); t.P1.reserve(n); t.P2.reserve(n);
    t.P3.reserve(n); t.P4.reserve(n); t.P5.reserve(n);
    t.P6.reserve(n); t.P7.reserve(n); t.P12.reserve(n);
    t.P_FA.reserve(n); t.P11.reserve(n);
    for (const KinematicState &st: states) {
        t.shock_stroke.push_back(st.shock_stroke);
        t.shock_length.push_back(st.shock_length);
        t.leverage_ratio.push_back(st.leverage_ratio);
        t.transmission_angle_deg.push_back(
                st.transmission_angle_deg);
        t.axle_path_x.push_back(st.P1[0]);
        t.axle_path_z.push_back(st.P1[2]);
        t.chainstay_angle_deg.push_back(st.theta_cs_deg);
        t.link_length_errors.push_back(st.max_link_error);
        t.P0.push_back(st.P0);   t.P1.push_back(st.P1);
        t.P2.push_back(st.P2);   t.P3.push_back(st.P3);
        t.P4.push_back(st.P4);   t.P5.push_back(st.P5);
        t.P6.push_back(st.P6);   t.P7.push_back(st.P7);
        t.P12.push_back(st.P12); t.P_FA.push_back(st.P_FA);
        t.P11.push_back(st.P11);
    }

    // float(np.max(link_length_errors)) — empty raises ValueError in
    // source order before the leverage reads.
    if (t.link_length_errors.empty())
        throw std::invalid_argument(
                "zero-size array to reduction operation maximum "
                "which has no identity");
    t.max_link_error = gd::np_max(t.link_length_errors);
    t.initial_leverage_ratio = t.leverage_ratio[0];
    t.final_leverage_ratio = t.leverage_ratio[n - 1];
    t.progressivity_pct = (t.initial_leverage_ratio -
                           t.final_leverage_ratio) /
                          t.initial_leverage_ratio * 100.0;
    t.max_stroke = t.shock_stroke[n - 1];
    t.states = std::move(states);
    return t;
}
NATIVE_DIAG_POP

} // namespace bike::kinematics
