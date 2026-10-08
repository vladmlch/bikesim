// rider/spindle_math.cpp — expression-for-expression ports of the pure
// helpers the spindle rider branch reaches. See the header for the
// source-file map; the Python statements fix the FP order.
#include "spindle_math.hpp"

#include "../numeric_norm.hpp"

#include <numbers>


namespace spindle {
    namespace {
        void require2(Vec2 v, const char *name) {
            validation::finite(v[0], name);
            validation::finite(v[1], name);
        }

        // np.linalg.norm on a length-2 float64 vector is sqrt(x0*x0+x1*x1)
        // evaluated as a plain multiply-add — not std::hypot's scaled
        // algorithm, which can differ in the last ulp.
        [[nodiscard]] double norm2(Vec2 v) {
            return std::sqrt(v[0] * v[0] + v[1] * v[1]);
        }

        // np.interp over a strictly increasing knot table: clamped at the
        // ends, slope*(x-xp0)+fp0 inside (numpy's evaluation order).
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
            return fp[hi - 1] + slope * (x - xp[hi - 1]);
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
        return mean * (1. + depth * std::cos(2. * phase));
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
            direction - std::atan2(b * std::sin(knee), a + b * std::cos(knee));
        return {.angles = {hip, knee}, .saturated = saturated};
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
        return {crank_xz[0] + sign * radius * std::cos(phase),
                crank_xz[1] - sign * radius * std::sin(phase)};
    }

    double wrap_angle(double angle_rad) {
        return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
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

    Vec2 leg_shares(double phase_rad) {
        const double downward =
            std::cos(validation::finite(phase_rad, "crank phase"));
        if (std::abs(downward) <= 1e-12)
            return {.5, .5};
        return downward > 0. ? Vec2{1., 0.} : Vec2{0., 1.};
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
        if (shares[0] == shares[1])
            return {total / 2., total / 2.};
        // Front is index 0, rear index 1; `rising` is the smaller share.
        const bool rising_is_front = shares[0] < shares[1];
        const Vec2 rising_spindle =
            rising_is_front ? spindle_front_xz : spindle_rear_xz;
        const double recovery =
            preload_torque_nm(preload_n, rising_spindle, crank_xz);
        return rising_is_front ? Vec2{recovery, total - recovery}
                               : Vec2{total - recovery, recovery};
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
        double positive = 0.;
        for (std::size_t i = 0; i < result.size(); ++i)
            positive += std::max(result[i] * velocities[i], 0.);
        if (positive > total) {
            const double scale = total / positive;
            for (std::size_t i = 0; i < result.size(); ++i)
                if (result[i] * velocities[i] > 0.)
                    result[i] *= scale;
        }
        return result;
    }

    std::vector<double> activation_step(std::span<const double> previous,
                                        std::span<const double> target,
                                        double dt_s, double tau_s) {
        if (previous.size() != target.size())
            throw std::invalid_argument("invalid activation state");
        for (const double v : previous)
            validation::finite(v, "invalid activation state");
        for (const double v : target)
            validation::finite(v, "invalid activation state");
        validation::finite(dt_s, "invalid activation state");
        validation::finite(tau_s, "invalid activation state");
        if (dt_s <= 0 || tau_s < 0)
            throw std::invalid_argument("invalid activation time constants");
        std::vector<double> out(target.size());
        if (tau_s == 0) {
            std::ranges::copy(target, out.begin());
            return out;
        }
        const double decay = std::exp(-dt_s / tau_s);
        for (std::size_t i = 0; i < out.size(); ++i)
            out[i] = target[i] + (previous[i] - target[i]) * decay;
        return out;
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
        double power = 0.;
        for (std::size_t i = 0; i < result.size(); ++i)
            if (result[i] * velocity[i] > 0)
                power += result[i] * velocity[i];
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
        const double a = (envelope.minimum_anatomical_rad -
                          envelope.neutral_anatomical_rad) /
                         envelope.direction;
        const double b = (envelope.maximum_anatomical_rad -
                          envelope.neutral_anatomical_rad) /
                         envelope.direction;
        return {std::min(a, b), std::max(a, b)};
    }

    double anatomical_angle(const JointEnvelope &envelope, double joint_q) {
        if (!std::isfinite(joint_q))
            throw std::invalid_argument(
                "anatomical angle needs an envelope and a finite joint "
                "coordinate");
        const double value =
            envelope.neutral_anatomical_rad + envelope.direction * joint_q;
        return std::atan2(std::sin(value), std::cos(value));
    }

} // namespace spindle
