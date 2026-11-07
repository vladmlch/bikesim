// rider/spindle_math.hpp — pure math the spindle rider port shares.
//
// Every function is a float-for-float port of the Python helpers it names
// (rider_control.py, crank_split.py, leg_loop.py, rider_activation.py,
// rider_envelope.py, joint_strength.py, seated_climb.py). The Python source
// fixes the expression order — reordering changes the bitwise contract.
//
// All functions validate like their Python counterparts and throw
// std::invalid_argument with the same field labels; they never touch
// mjModel/mjData and allocate only what they return.
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>
#include "../validation.hpp"

namespace spindle {
    using Vec2 = std::array<double, 2>;

    // ---- seated_climb.py ---------------------------------------------------
    // crank_effort_ceiling: bounded torque request for a power target.
    [[nodiscard]] double crank_effort_ceiling(double power_w,
                                              double torque_limit_nm,
                                              double crank_rate_rad_s);

    // ---- rider_control.py --------------------------------------------------
    // pedal_torque_waveform: mean*(1+ripple*cos(2*phase)), ripple < 1.
    [[nodiscard]] double spindle_torque_waveform(double mean_nm,
                                                double phase_rad,
                                                double ripple);

    // _two_link_ik core: CCW geometric angles + saturation flag.
    struct IkResult {
        Vec2 angles; // (upper/proximal, lower/distal)
        bool saturated;
    };
    [[nodiscard]] IkResult two_link_ik(Vec2 target_xz, double upper_m,
                                       double lower_m, int elbow_sign);

    // ---- leg_loop.py -------------------------------------------------------
    // LegLoopGeometry: planar two-link leg with rigid shank-foot.
    struct LegLoopGeometry {
        double thigh_m;
        double link_m;
        double knee_sign;
        double thigh_angle0_rad;
        double link_angle0_rad;
        double side_sign;
    };

    // Angle from -z toward -x: positive is rotation about native +y.
    [[nodiscard]] double planar_angle(Vec2 direction);

    // leg_loop_geometry over the (x,z) pose coordinates.
    [[nodiscard]] LegLoopGeometry leg_loop_geometry(Vec2 hip_xz, Vec2 knee_xz,
                                                    Vec2 pedal_xz,
                                                    bool front_side);

    // Spindle position in the caller's xz coordinates.
    [[nodiscard]] Vec2 spindle_xz(Vec2 crank_xz, double crank_m,
                                  double phase_rad, double side_sign,
                                  double frame_pitch_rad = 0.,
                                  double phase_offset_rad = 0.);

    [[nodiscard]] double wrap_angle(double angle_rad);

    // (hip_q, knee_q) — wrapped into the joint convention.
    [[nodiscard]] Vec2 leg_joint_q(const LegLoopGeometry &geometry,
                                   Vec2 hip_xz, Vec2 spindle_pos_xz,
                                   double pelvis_pitch_rad);

    // Centered finite-difference Jacobian d(hip_q,knee_q)/d(phase).
    [[nodiscard]] Vec2 leg_loop_jacobian(const LegLoopGeometry &geometry,
                                         Vec2 hip_xz, Vec2 crank_xz,
                                         double crank_m, double phase_rad,
                                         double pelvis_pitch_rad,
                                         double delta_rad = 1e-4,
                                         double frame_pitch_rad = 0.,
                                         double phase_offset_rad = 0.);

    // Inverse foot-vertical Jacobian d(hip_q,knee_q)/d(spindle_z) in the
    // bike frame (+x forward, +z up): maps joint torques (tau_hip, tau_knee)
    // to downward foot press delta_F_press = -(gamma[0]*tau_hip + gamma[1]*tau_knee).
    [[nodiscard]] Vec2 leg_vertical_gamma(const LegLoopGeometry &geometry,
                                          Vec2 hip_xz, Vec2 spindle_pos_xz);

    // ---- crank_split.py ----------------------------------------------------
    // leg_shares: downstroke-owning fractions (front, rear).
    [[nodiscard]] Vec2 leg_shares(double phase_rad);

    // preload_torque_nm: return-foot preload's signed crank moment.
    [[nodiscard]] double preload_torque_nm(double preload_n, Vec2 spindle_pos,
                                           Vec2 crank_xz);

    // leg_crank_targets: (front, rear) crank-torque targets after recovery.
    [[nodiscard]] Vec2 leg_crank_targets(double total_nm, double phase_rad,
                                         Vec2 spindle_front_xz,
                                         Vec2 spindle_rear_xz,
                                         Vec2 crank_xz, double preload_n);

    // split_joint_torques: weighted split against directional capacities.
    // `capacity(joint_index, sign)` mirrors capacity(name, sign); returns
    // (hip_nm, knee_nm). Header-inline so the callback stays generic.
    template<typename Capacity>
    [[nodiscard]] Vec2 split_joint_torques(double tau_leg_nm, Vec2 jac,
                                           Capacity &&capacity);

    template<typename Capacity>
    [[nodiscard]] Vec2 downstroke_joint_torques(double tau_leg_nm, Vec2 jac,
                                                Capacity &&capacity);

    // scale_to_power_budget on parallel arrays (dict order preserved by the
    // caller's iteration order).
    [[nodiscard]] std::vector<double>
    scale_to_power_budget(std::span<const double> torques,
                          std::span<const double> velocities,
                          double per_joint_w, double total_w);

    // ---- rider_activation.py ------------------------------------------------
    [[nodiscard]] std::vector<double>
    activation_step(std::span<const double> previous,
                    std::span<const double> target, double dt_s,
                    double tau_s);

    [[nodiscard]] std::vector<double>
    limit_positive_power(std::span<const double> torque,
                         std::span<const double> velocity, double limit_w);

    // ---- rider_control.py effort bound --------------------------------------
    [[nodiscard]] std::vector<double>
    bounded_effort(std::span<const double> qd,
                   std::span<const double> requested, double torque_limit,
                   double speed_limit, double power_limit);

    // ---- rider_envelope.py ---------------------------------------------------
    // soft_edge_response, elementwise: returns (torque, stored_energy) rows.
    struct SoftEdgeRow {
        double torque;
        double stored_j;
    };
    [[nodiscard]] SoftEdgeRow soft_edge_scalar(double q, double lower,
                                               double upper,
                                               double stiffness_nm_rad,
                                               double margin_rad);
    [[nodiscard]] std::vector<SoftEdgeRow>
    soft_edge_response(std::span<const double> q, std::span<const double> lower,
                       std::span<const double> upper, double stiffness_nm_rad,
                       double margin_rad);

    // ---- joint_strength.py / rider_envelope.py -------------------------------
    struct TorqueCurve {
        std::vector<double> angles_rad;
        std::vector<double> torques_nm;
        double vmax_rad_s;
        double hill_c;
        double eccentric_ratio;
        std::string source;
    };
    // TorqueCurve.__post_init__ validation.
    void validate_torque_curve(const TorqueCurve &curve, const char *label);

    [[nodiscard]] double directional_capacity(const TorqueCurve &curve,
                                              double angle_rad,
                                              double velocity_rad_s,
                                              int direction);

    struct JointEnvelope {
        double neutral_anatomical_rad = std::numeric_limits<double>::quiet_NaN();
        int direction = 0;
        double minimum_anatomical_rad = std::numeric_limits<double>::quiet_NaN();
        double maximum_anatomical_rad = std::numeric_limits<double>::quiet_NaN();
        std::string provenance;
    };
    void validate_joint_envelope(const JointEnvelope &envelope,
                                 const char *label);

    [[nodiscard]] Vec2 joint_q_range(const JointEnvelope &envelope);
    [[nodiscard]] double anatomical_angle(const JointEnvelope &envelope,
                                          double joint_q);

    // crank_split.py:36-51 verbatim — two weight iterations, then a final
    // clip against the direction-resolved capacities.
    template<typename Capacity>
    inline Vec2 split_joint_torques(double tau_leg_nm, Vec2 jac,
                                    const Capacity &capacity) {
        const double torque =
            validation::finite(tau_leg_nm, "leg crank torque");
        for (const double derivative : jac)
            validation::finite(derivative, "leg virtual-work Jacobian");
        const auto cap = [&capacity](int joint, double sign) {
            const double value = capacity(joint, sign);
            if (!std::isfinite(value) || value < 0.)
                throw std::invalid_argument(
                    (joint == 0 ? std::string("hip") : std::string("knee")) +
                    " directional capacity");
            return value;
        };
        std::array<double, 2> limits = {
            std::max(cap(0, 1.), cap(0, -1.)),
            std::max(cap(1, 1.), cap(1, -1.)),
        };
        std::array<double, 2> torques{0., 0.};
        for (int iteration = 0; iteration < 2; ++iteration) {
            const std::array<double, 2> weights{
                limits[0] * limits[0] * jac[0],
                limits[1] * limits[1] * jac[1]};
            const double denominator = jac[0] * weights[0] + jac[1] * weights[1];
            if (denominator < 1e-9)
                return Vec2{0., 0.};
            torques[0] = torque * weights[0] / denominator;
            torques[1] = torque * weights[1] / denominator;
            limits[0] = cap(0, torques[0] >= 0. ? 1. : -1.);
            limits[1] = cap(1, torques[1] >= 0. ? 1. : -1.);
        }
        for (std::size_t i = 0; i < 2; ++i) {
            if (std::abs(torques[i]) > limits[i]) {
                torques[i] =
                    std::max(-limits[i], std::min(limits[i], torques[i]));
                const std::size_t other = 1U - i;
                if (std::abs(jac[other]) > 1e-9) {
                    const double rem = torque - jac[i] * torques[i];
                    torques[other] = rem / jac[other];
                    limits[other] = cap(static_cast<int>(other),
                                        torques[other] >= 0. ? 1. : -1.);
                }
            }
        }
        return Vec2{
            std::max(-limits[0], std::min(limits[0], torques[0])),
            std::max(-limits[1], std::min(limits[1], torques[1]))};
    }

    template<typename Capacity>
    inline Vec2 downstroke_joint_torques(double tau_leg_nm, Vec2 jac,
                                         const Capacity &capacity) {
        if (tau_leg_nm <= 0.)
            return split_joint_torques(tau_leg_nm, jac, capacity);
        const double torque =
            validation::finite(tau_leg_nm, "leg crank torque");
        for (const double derivative : jac)
            validation::finite(derivative, "leg virtual-work Jacobian");
        const double cap_hip = capacity(0, 1.);
        const double cap_knee = capacity(1, -1.);
        if (!std::isfinite(cap_hip) || cap_hip < 0.)
            throw std::invalid_argument("hip directional capacity");
        if (!std::isfinite(cap_knee) || cap_knee < 0.)
            throw std::invalid_argument("knee directional capacity");
        const double w_hip = cap_hip * cap_hip * jac[0];
        const double w_knee = cap_knee * cap_knee * jac[1];
        const double denominator = jac[0] * w_hip + jac[1] * w_knee;
        if (denominator < 1e-9)
            return Vec2{0., 0.};
        const double hip =
            jac[0] > 0. ? std::min(cap_hip, torque * w_hip / denominator) : 0.;
        const double knee =
            jac[1] < 0. ? std::max(-cap_knee, torque * w_knee / denominator)
                        : 0.;
        return Vec2{hip, knee};
    }

} // namespace spindle

