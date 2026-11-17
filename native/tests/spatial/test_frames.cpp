#include "spatial/frames.hpp"
#include "support/fixture_test.hpp"

#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>

int main() {
    return bt::run([] {
        using namespace bike::spatial;
        constexpr double pi = std::numbers::pi;

        // 1. Right-handed MuJoCo convention: +roll tilts body +Z toward world -Y (right).
        const Quat q_roll_right = quat_from_euler_zyx(pi / 6., 0., 0.);
        const Vec3 up_world = rotate_body_to_world(q_roll_right, {0., 0., 1.});
        bt::require(std::abs(up_world[0]) <= 1e-15, "pure roll keeps world X zero");
        bt::require(std::abs(up_world[1] - (-0.5)) <= 1e-15, "+30 deg roll tilts top right (-Y)");
        bt::require(std::abs(up_world[2] - std::cos(pi / 6.)) <= 1e-15, "roll Z cosine");

        // 2. Antipodal invariance: q and -q produce identical rotated vectors and geodesic 0.
        const Quat q = quat_from_euler_zyx(0.25, -0.18, 1.35);
        const Quat neg_q{-q[0], -q[1], -q[2], -q[3]};
        const Vec3 v{1.2, -0.4, 0.7};
        const Vec3 r_pos = rotate_body_to_world(q, v);
        const Vec3 r_neg = rotate_body_to_world(neg_q, v);
        for (std::size_t i = 0; i < 3; ++i) {
            bt::require(r_pos[i] == r_neg[i], "q and -q rotate identically bitwise");
        }
        bt::require(geodesic_angle_rad(q, neg_q) <= 1e-15, "geodesic angle between q and -q is zero");
        const Vec3 back = rotate_world_to_body(q, r_pos);
        for (std::size_t i = 0; i < 3; ++i) {
            bt::require(std::abs(back[i] - v[i]) <= 1e-14, "body->world->body roundtrip");
        }

        // 3. ZYX Euler roundtrip and 80-deg gimbal guard.
        const EulerZyx e = euler_zyx(q);
        bt::require(e.valid, "moderate pitch attitude is valid");
        bt::require(std::abs(e.roll_rad - 0.25) <= 1e-14, "roll recovered");
        bt::require(std::abs(e.pitch_rad - (-0.18)) <= 1e-14, "pitch recovered");
        bt::require(std::abs(e.yaw_rad - 1.35) <= 1e-14, "yaw recovered");

        const EulerZyx near_pole = euler_zyx(quat_from_euler_zyx(0.1, 82. * pi / 180., 0.3));
        bt::require(!near_pole.valid, "pitch >= 80 deg marked invalid before tan blow-up");

        // 4. Body gyro -> Euler rates finite-difference consistency.
        const Vec3 omega_b{0.7, -0.35, 0.45};
        const EulerRatesZyx rates = body_gyro_to_euler_zyx_rates(e, omega_b);
        bt::require(rates.valid, "Euler rates valid on valid attitude");
        constexpr double dt = 1e-7;
        const Quat dq{1., 0.5 * omega_b[0] * dt, 0.5 * omega_b[1] * dt, 0.5 * omega_b[2] * dt};
        const EulerZyx e_next = euler_zyx(quat_normalized(quat_multiply(q, dq)));
        bt::require(std::abs((e_next.roll_rad - e.roll_rad) / dt - rates.roll_rate_rad_s) <= 1e-6,
                    "roll rate matches quaternion kinematic derivative");
        bt::require(std::abs((e_next.pitch_rad - e.pitch_rad) / dt - rates.pitch_rate_rad_s) <= 1e-6,
                    "pitch rate matches quaternion kinematic derivative");
        bt::require(std::abs((e_next.yaw_rad - e.yaw_rad) / dt - rates.yaw_rate_rad_s) <= 1e-6,
                    "yaw rate matches quaternion kinematic derivative");

        // 5. Horizontal heading at arbitrary yaw (+90 deg, +180 deg) and point velocity.
        const HorizontalHeading h90 = horizontal_heading_unit(quat_from_euler_zyx(0.2, -0.1, pi / 2.));
        bt::require(h90.valid, "heading valid at 90 deg yaw");
        bt::require(std::abs(h90.hx) <= 1e-14 && std::abs(h90.hy - 1.) <= 1e-14, "yaw +90 deg points +Y");

        const HorizontalHeading h180 = horizontal_heading_unit(quat_from_euler_zyx(-0.15, 0.12, pi));
        bt::require(h180.valid, "heading valid at 180 deg yaw");
        bt::require(std::abs(h180.hx - (-1.)) <= 1e-14 && std::abs(h180.hy) <= 1e-14,
                    "yaw +180 deg points -X");

        const BodyPose pose{
            .position_world_m = {1., 2., 0.5},
            .quat_world_from_body_wxyz = {1., 0., 0., 0.},
            .linear_velocity_world_mps = {3., 0., 0.},
            .angular_velocity_world_rad_s = {0., 0., 2.},
        };
        const Vec3 vp = point_velocity_world(pose, {1., 2.5, 0.5});
        bt::require(std::abs(vp[0] - 2.) <= 1e-15 && std::abs(vp[1] - 0.) <= 1e-15,
                    "lever-arm velocity v + w x r");

        // 6. Nonfinite and zero inputs fail closed.
        const double nan = std::numeric_limits<double>::quiet_NaN();
        bt::require(!euler_zyx({nan, 0., 0., 0.}).valid, "NaN quaternion rejected");
        bt::require(!euler_zyx({0., 0., 0., 0.}).valid, "zero quaternion rejected");
        bt::require(!horizontal_heading_unit({0., 0., 0., 0.}).valid, "zero heading rejected");
    });
}
