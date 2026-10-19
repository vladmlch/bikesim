// geometry/hardpoints.cpp — hardpoints.py's hardpoint/trail/frame-angle
// calculators, lowered to the same binary64 op sequence the Python
// oracle performs: elementwise NumPy array ops become per-component
// ops in source order, np.linalg.norm/np.dot dispatch to the same
// Accelerate ILP64 ddot the oracle BLAS-resolves (verified bitwise on
// 600,000 random norms), np.radians/np.degrees multiply by NumPy's
// NPY_*_PI constants, np.clip runs the const-minmax scalar loop, and
// sin/cos on the same operand go through numeric::np_cos/np_sin — the C++
// optimizer would otherwise fuse the pair into one __sincos_stret
// libcall whose components can differ from the standalone libm calls
// the oracle makes by one ulp (see numeric_sincos.hpp).
#include "specs.hpp"

#include "../diag.hpp"
#include "../numeric_sincos.hpp"

#include <stdexcept>

namespace bike::geometry {

// ---------------------------------------------------------------------------
// specs.py @property bodies.
// ---------------------------------------------------------------------------

FrameGeometrySpecs BikeSpecs::geometry() const {
    return FrameGeometrySpecs{
            .reach = reach,
            .stack = stack,
            .head_angle_deg = head_angle_deg,
            .effective_seat_angle_deg = effective_seat_angle_deg,
            .bb_drop = bb_drop,
            .wheelbase = wheelbase,
            .headtube_length = headtube_length,
            .front_wheel_radius = front_wheel_radius,
            .rear_wheel_radius = rear_wheel_radius,
            .wheel_radius = wheel_radius};
}

SuspensionHardwareSpecs BikeSpecs::suspension() const {
    return SuspensionHardwareSpecs{
            .fork_travel = fork_travel,
            .fork_offset = fork_offset,
            .rear_wheel_travel = rear_wheel_travel,
            .shock_eye_to_eye = shock_eye_to_eye,
            .shock_stroke = shock_stroke,
            .fork_stiffness = fork_stiffness,
            .fork_damping = fork_damping,
            .shock_stiffness = shock_stiffness,
            .shock_damping = shock_damping};
}

// ---------------------------------------------------------------------------
// hardpoints.py
// ---------------------------------------------------------------------------

HeadtubePoints compute_headtube_points(const BikeSpecs &specs) {
    const double theta = detail::np_radians(specs.head_angle_deg);
    const Vec3 u_steer{numeric::np_cos(theta), 0.0, -numeric::np_sin(theta)};

    const Vec3 p11_top{specs.reach, 0.0, specs.stack};
    // p_ht_bot = p11_top + headtube_length * u_steer (elementwise).
    const Vec3 p_ht_bot{
            p11_top[0] + specs.headtube_length * u_steer[0],
            p11_top[1] + specs.headtube_length * u_steer[1],
            p11_top[2] + specs.headtube_length * u_steer[2]};
    return HeadtubePoints{.top = p11_top, .bottom = p_ht_bot};
}

Vec3 compute_front_axle(const BikeSpecs &specs) {
    const double theta = detail::np_radians(specs.head_angle_deg);
    const double tan_theta = std::tan(theta);
    const double sin_theta = numeric::np_sin(theta);

    const double x_axis =
            specs.reach + (specs.stack - specs.bb_drop) / tan_theta;
    const double x_fa = x_axis + specs.fork_offset / sin_theta;

    return {x_fa, 0.0, specs.bb_drop};
}

double compute_ground_z(const BikeSpecs &specs) {
    return specs.bb_drop - specs.front_wheel_radius;
}

Vec3 compute_rear_axle(const BikeSpecs &specs) {
    return compute_rear_axle(specs, compute_front_axle(specs));
}

Vec3 compute_rear_axle(const BikeSpecs &specs, const Vec3 &front_axle) {
    const double x_ra = front_axle[0] - specs.wheelbase;
    const double z_ground = compute_ground_z(specs);
    const double z_ra = z_ground + specs.rear_wheel_radius;
    return {x_ra, 0.0, z_ra};
}

TrailResult compute_trail(const BikeSpecs &specs) {
    const double theta = detail::np_radians(specs.head_angle_deg);
    const double tan_theta = std::tan(theta);
    const double sin_theta = numeric::np_sin(theta);
    const double cos_theta = numeric::np_cos(theta);

    const double ground_z = compute_ground_z(specs);
    const double x_g =
            specs.reach + (specs.stack - ground_z) / tan_theta;

    const Vec3 fa = compute_front_axle(specs);
    const double ground_trail = x_g - fa[0];
    const double mechanical_trail = ground_trail * sin_theta;

    const HeadtubePoints h = compute_headtube_points(specs);
    const Vec3 u_steer{cos_theta, 0.0, -sin_theta};
    const Vec3 v_fork{fa[0] - h.bottom[0], fa[1] - h.bottom[1],
                      fa[2] - h.bottom[2]};
    // axle_to_crown = float(np.dot(v_fork, u_steer)) — BLAS ddot.
    const double axle_to_crown = detail::np_dot3(v_fork, u_steer);

    return TrailResult{.ground_trail = ground_trail,
                       .mechanical_trail = mechanical_trail,
                       .axle_to_crown = axle_to_crown,
                       .steer_ground_intersection_x = x_g,
                       .ground_z = ground_z};
}

// Returned by value deliberately (216-byte record mirrors the source's
// dict); -Wlarge-by-value-copy cannot split by-value parameters (its
// useful half) from return values — same suppression precedent as
// writers/tire.cpp's make_patch.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wlarge-by-value-copy")
FixedFramePoints get_fixed_frame_points(const BikeSpecs &specs) {
    const HeadtubePoints h = compute_headtube_points(specs);

    const Vec3 p10_st_junc{-60.0, 0.0, 400.0};
    const Vec3 p8_kink{410.0, 0.0, 635.0};
    const Vec3 p9_clamp{-85.0, 0.0, 490.0};

    // Dict insertion order: P0, P5, P7, P8, P9, P10, P11, P_HT_bot, BB.
    return FixedFramePoints{
            .P0 = Vec3{-44.902156, 0.0, 46.392796},
            .P5 = Vec3{41.321097, 0.0, 184.456762},
            .P7 = Vec3{173.220585, 0.0, 403.650931},
            .P8 = p8_kink,
            .P9 = p9_clamp,
            .P10 = p10_st_junc,
            .P11 = h.top,
            .P_HT_bot = h.bottom,
            .BB = Vec3{0.0, 0.0, 0.0}};
}
NATIVE_DIAG_POP

const Vec3 &FixedFramePoints::at(std::size_t index) const {
    const std::array<const Vec3 *, kCount> refs{
            &P0, &P5, &P7, &P8, &P9, &P10, &P11, &P_HT_bot, &BB};
    if (index >= refs.size())
        throw std::out_of_range("FixedFramePoints index out of range");
    return *refs.at(index);
}

std::array<std::string_view, FixedFramePoints::kCount>
FixedFramePoints::names() const {
    return {"P0",  "P5",  "P7",      "P8", "P9",
            "P10", "P11", "P_HT_bot", "BB"};
}

const Vec3 &FixedFramePoints::operator[](std::string_view name) const {
    const auto n = names();
    for (std::size_t i = 0; i < n.size(); ++i)
        if (n[i] == name) return at(i);
    throw std::out_of_range("FixedFramePoints: unknown point name");
}

FrameAngles compute_frame_angles(const BikeSpecs &specs) {
    const FixedFramePoints pts = get_fixed_frame_points(specs);
    const Vec3 &p10 = pts.P10;
    const Vec3 &p8 = pts.P8;
    const Vec3 &p9 = pts.P9;
    const Vec3 &p_ht_bot = pts.P_HT_bot;
    const Vec3 &bb = pts.BB;

    // Tube 1: P10 -> P8
    const Vec3 v_tube1{p8[0] - p10[0], p8[1] - p10[1], p8[2] - p10[2]};
    const double angle_x_tube1 =
            detail::np_degrees(std::atan2(v_tube1[2], v_tube1[0]));

    // Tube 2: BB -> P_HT_bot
    const Vec3 v_tube2{p_ht_bot[0] - bb[0], p_ht_bot[1] - bb[1],
                       p_ht_bot[2] - bb[2]};
    const double angle_x_tube2 =
            detail::np_degrees(std::atan2(v_tube2[2], v_tube2[0]));

    // Seat tube: P10 -> P9
    const Vec3 v_st{p9[0] - p10[0], p9[1] - p10[1], p9[2] - p10[2]};
    const double angle_seat_tube_x =
            detail::np_degrees(std::atan2(v_st[2], v_st[0]));
    const double sta = 180.0 - angle_seat_tube_x;

    // Angle 1: interior angle between seat tube and Tube 1 — the source
    // takes the [0, 2] components (x, z) of each 3-vector.
    const Vec2 v_st_xz{v_st[0], v_st[2]};
    const Vec2 v_t1_xz{v_tube1[0], v_tube1[2]};
    const double cos_a1 = detail::np_dot2(v_st_xz, v_t1_xz) /
                          (detail::np_norm2(v_st_xz) *
                           detail::np_norm2(v_t1_xz));
    const double angle_1 = detail::np_degrees(
            std::acos(detail::np_clip_scalar(cos_a1, -1.0, 1.0)));

    return FrameAngles{.angle_x_tube1_deg = angle_x_tube1,
                       .angle_x_tube2_deg = angle_x_tube2,
                       .angle_seat_tube_x_deg = angle_seat_tube_x,
                       .effective_seat_tube_angle_deg = sta,
                       .angle_1_deg = angle_1,
                       .head_angle_deg = specs.head_angle_deg};
}

} // namespace bike::geometry
