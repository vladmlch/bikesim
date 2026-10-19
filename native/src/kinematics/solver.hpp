// kinematics/solver.hpp — solver.py's HorstLinkageSolver: the exact
// closed-form Horst-Link + yoke rear-suspension kinematics, plus
// curves.py's compute_instant_centers. Public data products keep the
// Python dict key order as record member order (P0..P11/BB points,
// then the scalar block), so consumers read the same fields in the
// same order without dict plumbing.
//
// Bitwise contract: every expression is the oracle's own op sequence —
// np.linalg.norm/np.dot via detail::np_norm/np_dot (Accelerate ILP64
// ddot, verified bitwise), np.linspace via detail::np_linspace,
// np.clip scalar-bounds via detail::np_clip_scalar, np.interp via
// detail::np_interp (arr_interp semantics incl. the lenxp==1 branch),
// np.linalg.solve via the _dgesv$NEWLAPACK$ILP64 entry point the
// oracle's _umath_linalg imports (verified bitwise on 150,000 random
// 2x2 systems), and same-operand sin/cos pairs via numeric::np_sin/np_cos to
// keep them two separate libm calls. brentq keeps the source's
// convergence expression and bracket constants (theta_cs_0 - 0.70,
// theta_cs_0 + 0.10, xtol=1e-12) verbatim.
#pragma once

#include "../geometry/specs.hpp"
#include "linkage_math.hpp"

#include <array>
#include <cstddef>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

namespace bike::kinematics {

// Optional constructor overrides mirroring solver.py's keyword
// arguments (p0, p5, p7, p1_0, p2_0, p3_0, p4_0, p6_0, p12_0);
// std::nullopt is the source's None branch (defaults from
// get_fixed_frame_points / compute_rear_axle / the reference literals).
struct ReferencePoints {
    std::optional<geometry::Vec3> p0;
    std::optional<geometry::Vec3> p5;
    std::optional<geometry::Vec3> p7;
    std::optional<geometry::Vec3> p1_0;
    std::optional<geometry::Vec3> p2_0;
    std::optional<geometry::Vec3> p3_0;
    std::optional<geometry::Vec3> p4_0;
    std::optional<geometry::Vec3> p6_0;
    std::optional<geometry::Vec3> p12_0;
};

// The link_errors dict, source key order. max() is the builtin
// max(values) fold — strict `v > cur` keeps the first element on ties
// and skips NaN later elements.
struct LinkErrors {
    double chainstay;
    double seatstay;
    double rocker_p5_p3;
    double rocker_p5_p4;
    double axle_offset;
    double dropout_upper;
    double dropout_height;
    double yoke;

    // Not noexcept: std::array::at() may throw on the (impossible)
    // out-of-range index — the honest signature avoids
    // bugprone-exception-escape.
    [[nodiscard]] double max() const {
        // Field order mirrors the source's
        // max(err.chainstay, err.seatstay, ...) argument list.
        const std::array<double, 8> vals{chainstay,      seatstay,
                                         rocker_p5_p3,   rocker_p5_p4,
                                         axle_offset,    dropout_upper,
                                         dropout_height, yoke};
        double cur = vals.at(0);
        for (std::size_t i = 1; i < vals.size(); ++i)
            if (vals.at(i) > cur) cur = vals.at(i);
        return cur;
    }
};

// solve_state_from_chainstay_angle's dict — key order verbatim:
// the twelve point arrays then the scalar block.
struct KinematicState {
    geometry::Vec3 P0;
    geometry::Vec3 P1;
    geometry::Vec3 P2;
    geometry::Vec3 P3;
    geometry::Vec3 P4;
    geometry::Vec3 P5;
    geometry::Vec3 P6;
    geometry::Vec3 P7;
    geometry::Vec3 P12;
    geometry::Vec3 P_FA;
    geometry::Vec3 P11;
    geometry::Vec3 BB;
    double theta_cs;
    double theta_cs_deg;
    double wheel_travel;
    double shock_stroke;
    double shock_length;
    double transmission_angle_deg;
    double leverage_ratio;
    LinkErrors link_errors;
    double max_link_error;
};

// solve_trajectory's dict — key order verbatim. The eleven point
// trajectories mirror the source's point_names list (P0, P1, P2, P3,
// P4, P5, P6, P7, P12, P_FA, P11 — no BB); states holds the per-sample
// state dicts.
struct KinematicTrajectory {
    std::vector<double> wheel_travel;
    std::vector<double> shock_stroke;
    std::vector<double> shock_length;
    std::vector<double> leverage_ratio;
    std::vector<double> transmission_angle_deg;
    std::vector<double> axle_path_x;
    std::vector<double> axle_path_z;
    std::vector<double> chainstay_angle_deg;
    std::vector<double> link_length_errors;
    double max_link_error{};
    double initial_leverage_ratio{};
    double final_leverage_ratio{};
    double progressivity_pct{};
    double max_stroke{};
    std::vector<geometry::Vec3> P0;
    std::vector<geometry::Vec3> P1;
    std::vector<geometry::Vec3> P2;
    std::vector<geometry::Vec3> P3;
    std::vector<geometry::Vec3> P4;
    std::vector<geometry::Vec3> P5;
    std::vector<geometry::Vec3> P6;
    std::vector<geometry::Vec3> P7;
    std::vector<geometry::Vec3> P12;
    std::vector<geometry::Vec3> P_FA;
    std::vector<geometry::Vec3> P11;
    std::vector<KinematicState> states;

    static constexpr std::size_t kPointCount = 11;
    static constexpr std::array<std::string_view, kPointCount>
            kPointNames{"P0", "P1", "P2", "P3", "P4", "P5",
                        "P6", "P7", "P12", "P_FA", "P11"};
    // The point trajectory array for a point_names entry — the
    // dict-subscript equivalent for the golden-compare loop.
    [[nodiscard]] const std::vector<geometry::Vec3> &
    point_array(std::string_view name) const;
};

// compute_instant_centers result dict — 'travel', 'ic_x', 'ic_z'.
struct InstantCenters {
    std::vector<double> travel;
    std::vector<double> ic_x;
    std::vector<double> ic_z;
};

class HorstLinkageSolver {
public:
    // solver.py __init__: `specs or BikeSpecs()` — a present specs
    // object is always truthy, so value_or preserves the default
    // construction semantics.
    explicit HorstLinkageSolver(
            std::optional<geometry::BikeSpecs> specs = std::nullopt,
            const ReferencePoints &reference = {});

    // The ctor's public attributes, exposed verbatim.
    [[nodiscard]] const geometry::BikeSpecs &specs() const noexcept {
        return specs_;
    }
    [[nodiscard]] const geometry::Vec3 &p0() const noexcept {
        return p0_;
    }
    [[nodiscard]] const geometry::Vec3 &p5() const noexcept {
        return p5_;
    }
    [[nodiscard]] const geometry::Vec3 &p7() const noexcept {
        return p7_;
    }
    [[nodiscard]] const geometry::Vec3 &bb() const noexcept {
        return bb_;
    }
    [[nodiscard]] const geometry::Vec3 &p_fa() const noexcept {
        return p_fa_;
    }
    [[nodiscard]] const geometry::Vec3 &p11() const noexcept {
        return p11_;
    }
    [[nodiscard]] const geometry::Vec3 &p1_0() const noexcept {
        return p1_0_;
    }
    [[nodiscard]] const geometry::Vec3 &p2_0() const noexcept {
        return p2_0_;
    }
    [[nodiscard]] const geometry::Vec3 &p3_0() const noexcept {
        return p3_0_;
    }
    [[nodiscard]] const geometry::Vec3 &p4_0() const noexcept {
        return p4_0_;
    }
    [[nodiscard]] const geometry::Vec3 &p6_0() const noexcept {
        return p6_0_;
    }
    [[nodiscard]] const geometry::Vec3 &p12_0() const noexcept {
        return p12_0_;
    }

    // Rigid-link invariants + reference angles (public attrs in source).
    [[nodiscard]] double l_cs() const noexcept { return l_cs_; }
    [[nodiscard]] double l_ss() const noexcept { return l_ss_; }
    [[nodiscard]] double r_53() const noexcept { return r_53_; }
    [[nodiscard]] double r_54() const noexcept { return r_54_; }
    [[nodiscard]] double r_21() const noexcept { return r_21_; }
    [[nodiscard]] double r_2_12() const noexcept { return r_2_12_; }
    [[nodiscard]] double l_dropout_height() const noexcept {
        return l_dropout_height_;
    }
    [[nodiscard]] double l_ss_tube() const noexcept { return l_ss_tube_; }
    [[nodiscard]] double l_yoke() const noexcept { return l_yoke_; }
    [[nodiscard]] double l_shock_0() const noexcept {
        return l_shock_0_;
    }
    [[nodiscard]] double theta_cs_0() const noexcept {
        return theta_cs_0_;
    }
    [[nodiscard]] double beta_rocker() const noexcept {
        return beta_rocker_;
    }
    [[nodiscard]] double gamma_ra() const noexcept { return gamma_ra_; }
    [[nodiscard]] double gamma_12() const noexcept { return gamma_12_; }

    // The named public solves, mirroring solver.py.
    [[nodiscard]] KinematicState
    solve_state_from_chainstay_angle(double theta_cs) const;
    [[nodiscard]] KinematicState
    solve_state_from_wheel_travel(double target_travel_z) const;
    [[nodiscard]] KinematicState
    solve_state_from_shock_stroke(double shock_stroke_mm) const;
    [[nodiscard]] KinematicTrajectory
    solve_trajectory(int n_points = 101, double max_travel = 180.0) const;

private:
    // _solve_pivots_from_theta_cs's 9-tuple, named.
    struct SolvedPivots {
        geometry::Vec3 p1;
        geometry::Vec3 p2;
        geometry::Vec3 p3;
        geometry::Vec3 p4;
        geometry::Vec3 p6;
        geometry::Vec3 p12;
        geometry::Vec2 u47;
        double d47;
        double transmission_angle_deg;
    };

    [[nodiscard]] SolvedPivots solve_pivots_(double theta_cs) const;
    [[nodiscard]] double
    instantaneous_lr_(double theta_cs, const geometry::Vec3 &p1,
                      const geometry::Vec3 &p2, const geometry::Vec3 &p3,
                      const geometry::Vec3 &p4,
                      const geometry::Vec2 &u47) const;
    // functools.cached_property `_stroke_table` — built lazily on the
    // first solve_state_from_shock_stroke call (101 root finds), then
    // memoized for the life of the solver, exactly like the source.
    [[nodiscard]] const std::pair<std::vector<double>,
                                  std::vector<double>> &
    stroke_table_() const;

    geometry::BikeSpecs specs_;
    // In-class zero-init satisfies pro-type-member-init; every member
    // is overwritten by the constructor body's source-order
    // assignments (the values depend on the resolved specs, so they
    // cannot move into the init list).
    geometry::Vec3 p0_{}, p5_{}, p7_{}, bb_{}, p_fa_{}, p11_{};
    geometry::Vec3 p1_0_{}, p2_0_{}, p3_0_{}, p4_0_{}, p6_0_{}, p12_0_{};
    double l_cs_{}, l_ss_{}, r_53_{}, r_54_{}, r_21_{}, r_2_12_{};
    double l_dropout_height_{}, l_ss_tube_{}, l_yoke_{}, l_shock_0_{};
    double theta_cs_0_{}, beta_rocker_{}, gamma_ra_{}, gamma_12_{};
    mutable std::optional<std::pair<std::vector<double>,
                                    std::vector<double>>>
            stroke_table_cache_;
};

// curves.py — instant-center curve over the solver's trajectory.
[[nodiscard]] InstantCenters
compute_instant_centers(const HorstLinkageSolver &solver,
                        int n_points = 101);

} // namespace bike::kinematics
