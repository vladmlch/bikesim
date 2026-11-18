// rider/spindle_math.cpp — expression-for-expression ports of the pure
// helpers the spindle rider branch reaches. See the header for the
// source-file map; the Python statements fix the FP order.
#include "spindle_math.hpp"

#include "../numeric_norm.hpp"
#include "../numeric_numpy.hpp"
#include "../numeric_sincos.hpp"
#include "../numeric_sum.hpp"
#include "spindle_linalg.hpp"

#include <numbers>


namespace spindle {
    namespace {
        void require2(Vec2 v, const char *name) {
            validation::finite(v[0], name);
            validation::finite(v[1], name);
        }

        // np.linalg.norm on a float64 vector — the shared production
        // 1-D mirror (sqrt(v.dot(v))), not std::hypot's scaled algorithm.
        [[nodiscard]] double norm2(Vec2 v) {
            return rider::np_norm(v);
        }

        // np.interp over a strictly increasing knot table: clamped at the
        // ends, slope*(x-xp0)+fp0 inside. The multiply/add follows the
        // wheel's measured lowering — contracted on the macOS wheel,
        // separate on the Linux wheel — through the shared
        // numeric::np_interp_segment (same compiled_base.c mirror as
        // tyre/profile.cpp and drivetrain/motor.cpp).
        [[nodiscard]] double interp(std::span<const double> xp,
                                    std::span<const double> fp, double x) {
            if (x <= xp[0])
                return fp[0];
            if (x >= xp[xp.size() - 1])
                return fp[xp.size() - 1];
            // Upper-bound search matches np.searchsorted(..., 'right')-1.
            std::size_t hi = 1;
            while (hi + 1 < xp.size() && x >= xp[hi])
                ++hi;
            const double slope =
                (fp[hi] - fp[hi - 1]) / (xp[hi] - xp[hi - 1]);
            return numeric::np_interp_segment(slope, x, xp[hi - 1],
                                              fp[hi - 1], xp[hi],
                                              fp[hi]);
        }
    } // namespace

    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional params mirror seated_climb.py
    double crank_effort_ceiling(double power_w, double torque_limit_nm,
                                double crank_rate_rad_s) {
        const double power =
            validation::nonnegative(power_w, "crank power request");
        const double limit =
            validation::nonnegative(torque_limit_nm, "crank torque ceiling");
        const double rate = validation::finite(crank_rate_rad_s, "crank rate");
        constexpr double speed_floor = 20. * 2. * std::numbers::pi / 60.;
        return std::min(limit,
                        power / std::max({0., rate, speed_floor}));
    }

    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional params mirror the Python API
    double spindle_torque_waveform(double mean_nm, double phase_rad,
                                  double ripple) {
        const double mean =
            validation::nonnegative(mean_nm, "mean pedaling torque");
        const double phase = validation::finite(phase_rad, "pedal phase");
        const double depth =
            validation::nonnegative(ripple, "pedal torque ripple");
        if (depth >= 1.)
            throw std::invalid_argument(
                "pedal torque ripple must be below one");
        return mean * (1. + depth * numeric::math_cos(2. * phase));
    }

    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional params mirror the Python API
    IkResult two_link_ik(Vec2 target_xz, double upper_m, double lower_m,
                         int elbow_sign) {
        require2(target_xz, "IK target");
        const double a = validation::positive(upper_m, "upper link");
        const double b = validation::positive(lower_m, "lower link");
        if (elbow_sign != -1 && elbow_sign != 1)
            throw std::invalid_argument("IK branch must be -1 or +1");
        // two_link_ik's oracle is math.hypot — CPython's vector_norm, not
        // libm hypot (numeric_norm.hpp documents the verified divergence).
        const std::array<double, 2> target_abs{target_xz[0], target_xz[1]};
        const double distance = numeric::python_vector_norm(target_abs);
        const double lo = std::abs(a - b) + 1e-10, hi = a + b - 1e-10;
        if (hi <= lo)
            throw std::invalid_argument("IK links are too small");
        const bool saturated = !(lo <= distance && distance <= hi);
        const double d = std::min(std::max(distance, lo), hi);
        const double direction = distance > 1e-15
                                     ? std::atan2(target_xz[1], target_xz[0])
                                     : -std::numbers::pi / 2;
        const double cosine =
            std::min(std::max((d * d - a * a - b * b) / (2 * a * b), -1.), 1.);
        const double knee = elbow_sign * std::acos(cosine);
        const double hip =
            direction - std::atan2(b * numeric::math_sin(knee),
                                   a + b * numeric::math_cos(knee));
        return {.angles = {hip, knee}, .saturated = saturated};
    }

    ArmIkResult3d solve_arm_ik_3d_local(const ArmGeometry3d &geom,
                                        const Vec3 &d_torso) {
        const double l_ua =
            validation::positive(geom.upper_arm_m, "3D upper arm length");
        const double l_fg =
            validation::positive(geom.forearm_grip_m, "3D forearm-grip length");
        const double psi0 = validation::finite(geom.psi0_rad, "3D arm azimuth");
        const double beta_ua0 =
            validation::finite(geom.beta_ua0_rad, "3D upper arm pitch");
        const double el0 =
            validation::finite(geom.elbow_flexion0_rad, "3D elbow flexion");
        for (const double v : d_torso)
            validation::finite(v, "3D arm IK target");

        const double rho = std::hypot(d_torso[0], d_torso[1]);
        const double r = std::sqrt(rho * rho + d_torso[2] * d_torso[2]);
        const double psi =
            rho > 1e-12 ? std::atan2(d_torso[1], d_torso[0]) : psi0;
        const double q_abd = wrap_angle(psi - psi0);

        const double r_min = std::abs(l_ua - l_fg) + 1e-6;
        const double r_max = l_ua + l_fg - 1e-6;
        if (r_max <= r_min)
            throw std::invalid_argument("3D arm IK links are too small");
        const double r_eff = std::clamp(r, r_min, r_max);
        const bool saturated = (r != r_eff) || (rho <= 1e-12);

        const double gamma = std::atan2(-d_torso[2], rho);
        const double cos_el = std::clamp(
            (r_eff * r_eff - l_ua * l_ua - l_fg * l_fg) / (2.0 * l_ua * l_fg),
            -1.0, 1.0);
        const double theta_el = std::acos(cos_el);
        const double alpha =
            std::atan2(l_fg * numeric::math_sin(theta_el),
                       l_ua + l_fg * numeric::math_cos(theta_el));
        const double beta_ua = gamma + alpha;
        const double q_sh = wrap_angle(beta_ua - beta_ua0);
        const double q_el = wrap_angle(el0 - theta_el);

        return {.shoulder_abduction_q = q_abd,
                .shoulder_q = q_sh,
                .elbow_q = q_el,
                .elbow_flexion_rad = theta_el,
                .saturated = saturated};
    }

    ArmIkResult3d solve_arm_ik_3d(const ArmGeometry3d &geom,
                                  const Mat3 &torso_r,
                                  const Vec3 &shoulder_world,
                                  const Vec3 &grip_world) {
        for (const double v : torso_r)
            validation::finite(v, "3D torso rotation");
        for (const double v : shoulder_world)
            validation::finite(v, "3D shoulder world position");
        for (const double v : grip_world)
            validation::finite(v, "3D grip world position");
        const Vec3 diff{grip_world[0] - shoulder_world[0],
                        grip_world[1] - shoulder_world[1],
                        grip_world[2] - shoulder_world[2]};
        const Vec3 d_torso{
            torso_r[0] * diff[0] + torso_r[3] * diff[1] + torso_r[6] * diff[2],
            torso_r[1] * diff[0] + torso_r[4] * diff[1] + torso_r[7] * diff[2],
            torso_r[2] * diff[0] + torso_r[5] * diff[1] + torso_r[8] * diff[2]};
        return solve_arm_ik_3d_local(geom, d_torso);
    }

    double planar_angle(Vec2 direction) {
        require2(direction, "planar direction");
        return std::atan2(-direction[0], -direction[1]);
    }

    LegLoopGeometry leg_loop_geometry(Vec2 hip_xz, Vec2 knee_xz,
                                      Vec2 pedal_xz, bool front_side) {
        const Vec2 thigh{knee_xz[0] - hip_xz[0], knee_xz[1] - hip_xz[1]};
        const Vec2 link{pedal_xz[0] - knee_xz[0], pedal_xz[1] - knee_xz[1]};
        const Vec2 chord{pedal_xz[0] - hip_xz[0], pedal_xz[1] - hip_xz[1]};
        const double ahead = chord[0] * thigh[1] - chord[1] * thigh[0];
        return {.thigh_m = norm2(thigh),
                .link_m = norm2(link),
                .knee_sign = ahead >= 0. ? 1. : -1.,
                .thigh_angle0_rad = planar_angle(thigh),
                .link_angle0_rad = planar_angle(link),
                .side_sign = front_side ? 1. : -1.};
    }

    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional params mirror leg_loop.py
    Vec2 spindle_xz(Vec2 crank_xz, double crank_m, double phase_rad,
                    double side_sign, double frame_pitch_rad,
                    double phase_offset_rad) {
        require2(crank_xz, "crank position");
        const double radius =
            validation::positive(crank_m, "crank length");
        const double phase =
            validation::finite(phase_rad, "crank phase") +
            validation::finite(frame_pitch_rad, "frame pitch") +
            validation::finite(phase_offset_rad, "crank phase offset");
        const double sign = validation::finite(side_sign, "leg side sign");
        if (sign != -1. && sign != 1.)
            throw std::invalid_argument(
                "leg side sign must be plus or minus one");
        return {crank_xz[0] + sign * radius * numeric::math_cos(phase),
                crank_xz[1] - sign * radius * numeric::math_sin(phase)};
    }

    double wrap_angle(double angle_rad) {
        return std::atan2(numeric::math_sin(angle_rad),
                          numeric::math_cos(angle_rad));
    }

    Vec2 leg_joint_q(const LegLoopGeometry &geometry, Vec2 hip_xz,
                     Vec2 spindle_pos_xz, double pelvis_pitch_rad) {
        require2(hip_xz, "hip position");
        require2(spindle_pos_xz, "spindle position");
        const double thigh =
            validation::positive(geometry.thigh_m, "thigh length");
        const double link =
            validation::positive(geometry.link_m, "rigid shank-foot length");
        const double pitch =
            validation::finite(pelvis_pitch_rad, "pelvis pitch");
        const Vec2 direction{spindle_pos_xz[0] - hip_xz[0],
                             spindle_pos_xz[1] - hip_xz[1]};
        const double length = norm2(direction);
        if (length <= 1e-12 || length > thigh + link + 1e-9 ||
            length < std::abs(thigh - link) - 1e-9)
            throw std::invalid_argument("leg loop unreachable");
        const double knee_cos =
            (thigh * thigh + link * link - length * length) /
            (2. * thigh * link);
        const double hip_cos =
            (thigh * thigh + length * length - link * link) /
            (2. * thigh * length);
        if (!(-1. - 1e-9 <= knee_cos && knee_cos <= 1. + 1e-9 &&
              -1. - 1e-9 <= hip_cos && hip_cos <= 1. + 1e-9))
            throw std::invalid_argument("leg loop unreachable");
        const double thigh_angle =
            planar_angle(direction) -
            geometry.knee_sign *
                std::acos(std::min(std::max(hip_cos, -1.), 1.));
        const double bend =
            geometry.knee_sign *
            (std::numbers::pi - std::acos(std::min(std::max(knee_cos, -1.), 1.)));
        return {wrap_angle(thigh_angle - pitch - geometry.thigh_angle0_rad),
                wrap_angle(bend - (geometry.link_angle0_rad -
                                   geometry.thigh_angle0_rad))};
    }

    // NOLINTBEGIN(bugprone-easily-swappable-parameters) positional params mirror leg_loop.py
    Vec2 leg_loop_jacobian(const LegLoopGeometry &geometry, Vec2 hip_xz,
                           Vec2 crank_xz, double crank_m, double phase_rad,
                           double pelvis_pitch_rad, double delta_rad,
                           double frame_pitch_rad, double phase_offset_rad) {
        const double delta =
            validation::positive(delta_rad, "leg derivative interval");
        const auto sample = [&](double phase) {
            return leg_joint_q(geometry, hip_xz,
                               spindle_xz(crank_xz, crank_m, phase,
                                          geometry.side_sign,
                                          frame_pitch_rad, phase_offset_rad),
                               pelvis_pitch_rad);
        };
        const Vec2 plus = sample(phase_rad + delta);
        const Vec2 minus = sample(phase_rad - delta);
        return {wrap_angle(plus[0] - minus[0]) / (2. * delta),
                wrap_angle(plus[1] - minus[1]) / (2. * delta)};
    }
    // NOLINTEND(bugprone-easily-swappable-parameters)

    Vec2 leg_vertical_gamma(const LegLoopGeometry &geometry, Vec2 hip_xz,
                            Vec2 spindle_pos_xz) {
        require2(hip_xz, "hip position");
        require2(spindle_pos_xz, "spindle position");
        const double thigh =
            validation::positive(geometry.thigh_m, "thigh length");
        const double link =
            validation::positive(geometry.link_m, "rigid shank-foot length");
        const Vec2 direction{spindle_pos_xz[0] - hip_xz[0],
                             spindle_pos_xz[1] - hip_xz[1]};
        const double length = norm2(direction);
        if (length <= 1e-12 || length > thigh + link + 1e-9 ||
            length < std::abs(thigh - link) - 1e-9)
            throw std::invalid_argument("leg loop unreachable");
        const double hip_cos =
            (thigh * thigh + length * length - link * link) /
            (2. * thigh * length);
        if (!(-1. - 1e-9 <= hip_cos && hip_cos <= 1. + 1e-9))
            throw std::invalid_argument("leg loop unreachable");
        const double thigh_angle =
            planar_angle(direction) -
            geometry.knee_sign *
                std::acos(std::min(std::max(hip_cos, -1.), 1.));
        const double knee_x =
            hip_xz[0] - thigh * numeric::math_sin(thigh_angle);
        const double knee_z =
            hip_xz[1] - thigh * numeric::math_cos(thigh_angle);
        const double det =
            -(spindle_pos_xz[1] - hip_xz[1]) * (spindle_pos_xz[0] - knee_x) +
            (spindle_pos_xz[1] - knee_z) * (spindle_pos_xz[0] - hip_xz[0]);
        if (std::abs(det) <= 1e-12)
            return {0., 0.};
        return {-(spindle_pos_xz[1] - knee_z) / det,
                (spindle_pos_xz[1] - hip_xz[1]) / det};
    }

    Vec2 leg_horizontal_gamma(const LegLoopGeometry &geometry, Vec2 hip_xz,
                              Vec2 spindle_pos_xz) {
        require2(hip_xz, "hip position");
        require2(spindle_pos_xz, "spindle position");
        const double thigh =
            validation::positive(geometry.thigh_m, "thigh length");
        const double link =
            validation::positive(geometry.link_m, "rigid shank-foot length");
        const Vec2 direction{spindle_pos_xz[0] - hip_xz[0],
                             spindle_pos_xz[1] - hip_xz[1]};
        const double length = norm2(direction);
        if (length <= 1e-12 || length > thigh + link + 1e-9 ||
            length < std::abs(thigh - link) - 1e-9)
            throw std::invalid_argument("leg loop unreachable");
        const double hip_cos =
            (thigh * thigh + length * length - link * link) /
            (2. * thigh * length);
        if (!(-1. - 1e-9 <= hip_cos && hip_cos <= 1. + 1e-9))
            throw std::invalid_argument("leg loop unreachable");
        const double thigh_angle =
            planar_angle(direction) -
            geometry.knee_sign *
                std::acos(std::min(std::max(hip_cos, -1.), 1.));
        const double knee_x =
            hip_xz[0] - thigh * numeric::math_sin(thigh_angle);
        const double knee_z =
            hip_xz[1] - thigh * numeric::math_cos(thigh_angle);
        const double det =
            -(spindle_pos_xz[1] - hip_xz[1]) * (spindle_pos_xz[0] - knee_x) +
            (spindle_pos_xz[1] - knee_z) * (spindle_pos_xz[0] - hip_xz[0]);
        if (std::abs(det) <= 1e-12)
            return {0., 0.};
        return {-(spindle_pos_xz[0] - knee_x) / det,
                (spindle_pos_xz[0] - hip_xz[0]) / det};
    }

    Vec2 leg_shares(double phase_rad) {
        constexpr double kHandoverCosine = .25;
        const double downward =
            numeric::math_cos(validation::finite(phase_rad, "crank phase"));
        if (std::abs(downward) <= 1e-12)
            return {.5, .5};
        if (downward >= kHandoverCosine)
            return {1., 0.};
        if (downward <= -kHandoverCosine)
            return {0., 1.};
        const double u = downward / kHandoverCosine;
        const double front = .5 + .25 * u * (3. - u * u);
        return {front, 1. - front};
    }

    double preload_torque_nm(double preload_n, Vec2 spindle_pos,
                             Vec2 crank_xz) {
        const double preload =
            validation::nonnegative(preload_n, "return foot preload");
        require2(spindle_pos, "spindle position");
        require2(crank_xz, "crank position");
        return preload * (spindle_pos[0] - crank_xz[0]);
    }

    // NOLINTBEGIN(bugprone-easily-swappable-parameters) positional params mirror crank_split.py
    Vec2 leg_crank_targets(double total_nm, double phase_rad,
                           Vec2 spindle_front_xz, Vec2 spindle_rear_xz,
                           Vec2 crank_xz, double preload_n) {
        const double total =
            validation::finite(total_nm, "total crank torque");
        const Vec2 shares = leg_shares(phase_rad);
        const double recovery_front =
            preload_torque_nm(preload_n, spindle_front_xz, crank_xz);
        const double recovery_rear =
            preload_torque_nm(preload_n, spindle_rear_xz, crank_xz);
        if (shares[0] == shares[1])
            return {total / 2., total / 2.};
        return {shares[0] * (total - recovery_rear) +
                    shares[1] * recovery_front,
                shares[1] * (total - recovery_front) +
                    shares[0] * recovery_rear};
    }
    // NOLINTEND(bugprone-easily-swappable-parameters)

    std::vector<double>
    scale_to_power_budget(std::span<const double> torques,
                          std::span<const double> velocities,
                          double per_joint_w, double total_w) {
        const double per_joint = validation::nonnegative(
            per_joint_w, "joint positive-power budget");
        const double total = validation::nonnegative(
            total_w, "whole-body positive-power budget");
        if (torques.size() != velocities.size())
            throw std::invalid_argument("power budget vector mismatch");
        std::vector<double> result(torques.size());
        for (std::size_t i = 0; i < torques.size(); ++i) {
            const double torque =
                validation::finite(torques[i], "joint torque");
            const double speed =
                validation::finite(velocities[i], "joint speed");
            result[i] =
                torque * speed > per_joint ? per_joint / speed : torque;
        }
        // crank_split.py:63 — builtin sum() over the clipped dict values:
        // CPython's Neumaier loop, not a sequential fold. The scale factor
        // inherits the reduction rounding, so a naive += lands the bound
        // torques one ulp off whenever the budget binds. Every item is
        // max(float_product, 0.) — a Python float — so the compensated
        // float branch is the oracle's whole path.
        numeric::PythonSum positive;
        for (std::size_t i = 0; i < result.size(); ++i)
            positive.add(std::max(result[i] * velocities[i], 0.));
        if (positive.total() > total) {
            const double scale = total / positive.total();
            for (std::size_t i = 0; i < result.size(); ++i)
                if (result[i] * velocities[i] > 0.)
                    result[i] *= scale;
        }
        return result;
    }

    std::vector<double> limit_positive_power(std::span<const double> torque,
                                             std::span<const double> velocity,
                                             double limit_w) {
        if (torque.size() != velocity.size())
            throw std::invalid_argument("power budget vector mismatch");
        for (const double v : torque)
            validation::finite(v, "invalid power budget");
        for (const double v : velocity)
            validation::finite(v, "invalid power budget");
        validation::finite(limit_w, "invalid power budget");
        if (limit_w < 0)
            throw std::invalid_argument("invalid power budget");
        std::vector<double> result(torque.begin(), torque.end());
        // rider_activation.py:24-25 — result[positive]*velocity[positive]
        // materializes the masked products in index order, then np.sum
        // reduces them with numpy's pairwise block order.
        std::vector<double> products;
        products.reserve(result.size());
        for (std::size_t i = 0; i < result.size(); ++i)
            if (result[i] * velocity[i] > 0)
                products.push_back(result[i] * velocity[i]);
        const double power = numeric::numpy_sum(products);
        if (power > limit_w) {
            const double scale = limit_w / power;
            for (std::size_t i = 0; i < result.size(); ++i)
                if (result[i] * velocity[i] > 0)
                    result[i] *= scale;
        }
        return result;
    }

    // NOLINTBEGIN(bugprone-easily-swappable-parameters) positional params mirror rider_control.py
    std::vector<double> bounded_effort(std::span<const double> qd,
                                       std::span<const double> requested,
                                       double torque_limit,
                                       double speed_limit,
                                       double power_limit) {
        if (qd.size() != requested.size())
            throw std::invalid_argument("effort state shape mismatch");
        for (const double v : qd)
            validation::finite(v, "joint speed");
        for (const double v : requested)
            validation::finite(v, "requested joint torque");
        const double limit =
            validation::nonnegative(torque_limit, "joint torque ceiling");
        const double speed_ceiling =
            validation::positive(speed_limit, "joint speed ceiling");
        const double power_ceiling =
            validation::nonnegative(power_limit, "joint power ceiling");
        std::vector<double> result(requested.size());
        for (std::size_t i = 0; i < result.size(); ++i)
            result[i] =
                std::min(std::max(requested[i], -limit), limit);
        for (std::size_t i = 0; i < result.size(); ++i)
            if (result[i] * qd[i] > 0 && std::abs(qd[i]) >= speed_ceiling)
                result[i] = 0.;
        for (std::size_t i = 0; i < result.size(); ++i) {
            if (!(result[i] * qd[i] > 0))
                continue;
            const double cap = qd[i] != 0.
                                   ? std::min(limit,
                                              power_ceiling / std::abs(qd[i]))
                                   : limit;
            result[i] = std::min(std::max(result[i], -cap), cap);
        }
        return result;
    }
    // NOLINTEND(bugprone-easily-swappable-parameters)

    SoftEdgeRow soft_edge_scalar(double q, double lower, double upper,
                                 double stiffness_nm_rad, double margin_rad) {
        const double value = validation::finite(q, "joint edge state");
        const double lo = validation::finite(lower, "joint edge state");
        const double hi = validation::finite(upper, "joint edge state");
        const double k = validation::finite(stiffness_nm_rad,
                                            "joint edge state");
        const double m = validation::finite(margin_rad, "joint edge state");
        if (hi <= lo || k < 0 || m < 0)
            throw std::invalid_argument("invalid soft edge parameters");
        const double margin = std::min(m, (hi - lo) / 2.);
        const double left = std::max(lo + margin - value, 0.);
        const double right = std::max(value - hi + margin, 0.);
        return {.torque = k * (left - right),
                .stored_j = .5 * k * (left * left + right * right)};
    }

    std::vector<SoftEdgeRow>
    soft_edge_response(std::span<const double> q,
                       std::span<const double> lower,
                       std::span<const double> upper,
                       double stiffness_nm_rad, double margin_rad) {
        if (q.size() != lower.size() || q.size() != upper.size())
            throw std::invalid_argument("joint edge state must broadcast");
        std::vector<SoftEdgeRow> rows;
        rows.reserve(q.size());
        for (std::size_t i = 0; i < q.size(); ++i)
            rows.push_back(soft_edge_scalar(q[i], lower[i], upper[i],
                                            stiffness_nm_rad, margin_rad));
        return rows;
    }

    void validate_torque_curve(const TorqueCurve &curve, const char *label) {
        const std::string name = label == nullptr ? "strength curve" : label;
        if (curve.angles_rad.size() < 2 ||
            curve.angles_rad.size() != curve.torques_nm.size())
            throw std::invalid_argument(
                name + ": strength curve requires paired angle/torque knots");
        for (const double v : curve.angles_rad)
            if (!std::isfinite(v))
                throw std::invalid_argument(name + ": invalid strength data");
        for (const double v : curve.torques_nm)
            if (!std::isfinite(v) || v < 0)
                throw std::invalid_argument(name + ": invalid strength data");
        for (std::size_t i = 1; i < curve.angles_rad.size(); ++i)
            if (curve.angles_rad[i] <= curve.angles_rad[i - 1])
                throw std::invalid_argument(name + ": angles must increase");
        if (!std::isfinite(curve.vmax_rad_s) ||
            !std::isfinite(curve.hill_c) ||
            !std::isfinite(curve.eccentric_ratio))
            throw std::invalid_argument(name + ": invalid strength data");
        if (curve.vmax_rad_s <= 0 || curve.hill_c <= 0 ||
            curve.eccentric_ratio < 1)
            throw std::invalid_argument(
                name + ": invalid force-velocity parameters");
        if (curve.source.empty())
            throw std::invalid_argument(name + ": missing strength provenance");
    }

    double directional_capacity(const TorqueCurve &curve, double angle_rad,
                                double velocity_rad_s, int direction) {
        if ((direction != -1 && direction != 1) ||
            !std::isfinite(angle_rad) || !std::isfinite(velocity_rad_s))
            throw std::invalid_argument("invalid joint state or direction");
        if (!(curve.angles_rad.front() <= angle_rad &&
              angle_rad <= curve.angles_rad.back()))
            throw std::invalid_argument(
                "strength evaluation outside documented angle range");
        const double iso = interp(curve.angles_rad, curve.torques_nm, angle_rad);
        const double speed = direction * velocity_rad_s / curve.vmax_rad_s;
        double factor = 0.;
        if (speed >= 0)
            factor = std::max(0., (1. - speed) / (1. + speed / curve.hill_c));
        else
            factor = 1. + (curve.eccentric_ratio - 1.) * (-speed) /
                              (1. - speed);
        return iso * factor;
    }

    void validate_joint_envelope(const JointEnvelope &envelope,
                                 const char *label) {
        const std::string name = label == nullptr ? "joint" : label;
        if (!std::isfinite(envelope.neutral_anatomical_rad) ||
            !std::isfinite(envelope.minimum_anatomical_rad) ||
            !std::isfinite(envelope.maximum_anatomical_rad) ||
            (envelope.direction != -1 && envelope.direction != 1))
            throw std::invalid_argument(
                name + ": invalid joint coordinate convention");
        if (!(envelope.minimum_anatomical_rad <
              envelope.maximum_anatomical_rad))
            throw std::invalid_argument(name + ": empty joint range");
        if (envelope.provenance.empty())
            throw std::invalid_argument(name + ": joint range provenance is required");
    }

    Vec2 joint_q_range(const JointEnvelope &envelope) {
        validate_joint_envelope(envelope, "joint");
        const double a = (envelope.minimum_anatomical_rad -
                          envelope.neutral_anatomical_rad) /
                         envelope.direction;
        const double b = (envelope.maximum_anatomical_rad -
                          envelope.neutral_anatomical_rad) /
                         envelope.direction;
        return {std::min(a, b), std::max(a, b)};
    }

    double anatomical_angle(const JointEnvelope &envelope, double joint_q) {
        if (!std::isfinite(envelope.neutral_anatomical_rad) ||
            !std::isfinite(joint_q))
            throw std::invalid_argument(
                "anatomical angle needs an envelope and a finite joint "
                "coordinate");
        validate_joint_envelope(envelope, "joint");
        const double value =
            envelope.neutral_anatomical_rad + envelope.direction * joint_q;
        return std::atan2(numeric::math_sin(value), numeric::math_cos(value));
    }

} // namespace spindle
