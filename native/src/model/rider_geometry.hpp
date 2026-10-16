// model/rider_geometry.hpp — T15 rider boundary: physics/rider.py's
// seated pose solver, leg chains and mass-path bookkeeping, plus
// physics/rider_segments.py's De Leva mass/fitting helpers. The
// RiderSpecs record itself is the T12 configuration record
// (bike::configuration::RiderSpecs, including its present /
// total_rider_mass / inseam / saddle_height_m / segment_length /
// torso_helmet_mass / legs_mass / arms_mass member ports); this header
// carries the geometry-consuming declarations T15 owns.
//
// Bitwise contract: every expression keeps the oracle's operation order —
// elementwise NumPy array ops become per-component binary64 ops,
// np.linalg.norm / np.dot route through geometry::detail's Accelerate
// ILP64 ddot entry points, np.hypot/np.arctan2/np.abs reach libm
// hypot/atan2, `x ** y` reaches libm pow() via pyfloat::pow, builtin
// sum() goes through numeric::PythonSum (CPython's Neumaier float
// accumulator) for float items and a sequential fold for ndarray items,
// and same-operand sin/cos pairs route through numeric::sin /
// numeric::cos so the optimizer cannot fuse them into a __sincos_stret
// the oracle never calls. saddle_path_apparent_mass's complex arithmetic
// reproduces CPython/NumPy complex multiply, divide (Smith's scaled
// branch) and hypot-based absolute bitwise.
#pragma once

#include <array>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include "../configuration/rider_spec.hpp" // configuration::RiderSpecs
#include "../geometry/specs.hpp"           // geometry::BikeSpecs, Vec3,
                                           // SaddleGeometry

namespace bike::model {

using configuration::RiderSpecs;
using geometry::BikeSpecs;
using geometry::SaddleGeometry;
using geometry::Vec3;
using Mat3 = std::array<double, 9>;

// ---------------------------------------------------------------------------
// Module constants — rider.py + rider_segments.py, verbatim.
// ---------------------------------------------------------------------------

// rider.py module scalars (the shared constants — GRAVITY_MPS2,
// RIDER_VARIANTS, DEFAULT_RIDER_VARIANT, DE_LEVA_LENGTH_FRACTIONS,
// DE_LEVA_REFERENCE_STATURE_M, INSEAM_STATURE_RATIO,
// LEMOND_SADDLE_HEIGHT_RATIO, LUMPED_*_FRACTION — are the T12
// configuration constants this layer reuses).
inline constexpr double HIP_ABOVE_SADDLE_M = 0.060;
inline constexpr double ANKLE_ABOVE_PEDAL_M = 0.115;
inline constexpr double ELBOW_FLEXION_DEG = 15.0;
inline constexpr double GRIP_FROM_WRIST_FRACTION = 0.60;
inline constexpr double HEAD_RADIUS_M = 0.110;
inline constexpr double NECK_LENGTH_M = 0.050;
inline constexpr double KNEE_FLEXION_BDC_MIN_DEG = 20.0;
inline constexpr double KNEE_FLEXION_BDC_MAX_DEG = 60.0;

// rider_segments.py ARM_LATERAL_OFFSET_M (each hand grasps its own bar
// half-width). PEDAL_LATERAL_OFFSET_M is geometry's cockpit constant
// (geometry::PEDAL_LATERAL_OFFSET_M, 0.115 — reused verbatim).
inline constexpr double ARM_LATERAL_OFFSET_M = 0.10;

// rider.py DE_LEVA_MASS_FRACTIONS — body-mass fractions; a nullopt return
// mirrors the dict KeyError (out_of_range with repr(name)).
[[nodiscard]] double de_leva_mass_fraction(std::string_view name);
[[nodiscard]] bool has_de_leva_mass_fraction(std::string_view name) noexcept;

// rider.py SEGMENT_RADII_M — visual capsule radii by segment key.
[[nodiscard]] double segment_radius_m(std::string_view name); // dict []
[[nodiscard]] bool has_segment_radius(std::string_view name) noexcept;

// rider.py LUMPED_COM_M — keyed capsule centres (BB frame, metres).
[[nodiscard]] Vec3 lumped_com_m(std::string_view name); // dict []

// ---------------------------------------------------------------------------
// Frozen rider records — field order verbatim.
// ---------------------------------------------------------------------------

// RiderGeom: a visual/mass capsule (or sphere) between p1..p2.
struct RiderGeom {
    std::string name;
    std::string kind; // "capsule" | "sphere"
    Vec3 p1{};
    Vec3 p2{};
    double radius = 0.0;
    double mass = 0.0;

    // RiderGeom.center — 0.5 * (p1 + p2) elementwise.
    [[nodiscard]] Vec3 center() const {
        return {0.5 * (p1[0] + p2[0]), 0.5 * (p1[1] + p2[1]),
                0.5 * (p1[2] + p2[2])};
    }
};

// RiderBody — stiffness/damping/supported_mass/unilateral validation runs
// in validate_rider_body (the __post_init__), invoked by the pose solver
// on construction just like the frozen dataclass.
struct RiderBody {
    std::string name;
    std::string parent;
    Vec3 attach{};
    std::string joint;
    std::vector<RiderGeom> geoms;
    double stiffness_n_m = 0.0;
    double damping_ns_m = 0.0;
    double supported_mass_kg = 0.0;
    bool unilateral = false;
    std::string interface; // "saddle" | "pedals" | "bar"

    // RiderBody.mass — builtin sum() over float geom masses (Neumaier
    // accumulation via numeric::PythonSum).
    [[nodiscard]] double mass() const;
    // RiderBody.center_of_mass — builtin sum() over ndarray items is the
    // generic sequential fold: acc = acc + (g.mass * g.center), then
    // / mass. Empty geoms -> ZeroDivisionError ("float division by
    // zero") via std::runtime_error.
    [[nodiscard]] Vec3 center_of_mass() const;
    // preload_n property — supported_mass_kg * GRAVITY_MPS2.
    [[nodiscard]] double preload_n() const;
    // preload_deflection_m property — preload_n / stiffness_n_m; the
    // zero-stiffness branch and its ValueError are verbatim.
    [[nodiscard]] double preload_deflection_m() const;
};

// RiderBody.__post_init__ — checks scalar/boolean domains. Invoked by
// solve_seated_pose's construction calls.
void validate_rider_body(const RiderBody &body);

// LegChain — one articulated leg (field order verbatim).
struct LegChain {
    std::string side; // "front" | "rear"
    double lateral_y_m = 0.0;
    double crank_len_m = 0.0;
    Vec3 hip{};
    Vec3 knee{};
    Vec3 ankle{};
    Vec3 pedal{};
    double thigh_len_m = 0.0;
    double shank_len_m = 0.0;
    double thigh_mass_kg = 0.0;
    double shank_mass_kg = 0.0;
    double foot_mass_kg = 0.0;
    double pitch0_thigh_rad = 0.0;
    double pitch0_shank_rad = 0.0;
    double pitch0_foot_rad = 0.0;
};

// SeatedPose — field order verbatim.
struct SeatedPose {
    SaddleGeometry saddle;
    Vec3 hip{};
    Vec3 shoulder{};
    Vec3 elbow{};
    Vec3 wrist{};
    Vec3 grip{};
    Vec3 head_center{};
    Vec3 knee_front{};
    Vec3 knee_rear{};
    Vec3 ankle_front{};
    Vec3 ankle_rear{};
    Vec3 pedal_front{};
    Vec3 pedal_rear{};
    double torso_lean_deg = 0.0;
    double knee_flexion_front_deg = 0.0;
    double knee_flexion_rear_deg = 0.0;
    double knee_flexion_bdc_deg = 0.0;
    std::vector<RiderBody> bodies;
    // path_masses_kg — {"torso","pelvis","arms","leg"} in key order.
    std::vector<std::pair<std::string, double>> path_masses_kg;
    std::vector<LegChain> leg_chains; // empty for rigid legs

    // SeatedPose.body(name) — std::out_of_range on a miss (KeyError).
    [[nodiscard]] const RiderBody &body(std::string_view name) const;
    // SeatedPose.total_mass_kg — builtin sum() over bodies plus the
    // per-chain thigh+shank+foot sums.
    [[nodiscard]] double total_mass_kg() const;
    // SeatedPose.interface_loads_n() — {"saddle","pedals","bar"} order;
    // only bodies parented to "frame" contribute.
    [[nodiscard]] std::vector<std::pair<std::string, double>>
    interface_loads_n() const;
    // SeatedPose.interface_shares() — loads / (total_mass * g);
    // ZeroDivisionError ("float division by zero") when the denominator
    // is zero.
    [[nodiscard]] std::vector<std::pair<std::string, double>>
    interface_shares() const;
};

// ---------------------------------------------------------------------------
// rider.py module-level helpers — same names/signatures.
// ---------------------------------------------------------------------------

// _two_link_ik(a, b, len_a, len_b, prefer) — prefer is "+x" | "-x" |
// "+z" | "-z" (no default in the oracle).
[[nodiscard]] Vec3 two_link_ik(const Vec3 &a, const Vec3 &b, double len_a,
                               double len_b, std::string_view prefer);

// _flexion_deg(proximal, joint, distal).
[[nodiscard]] double flexion_deg(const Vec3 &proximal, const Vec3 &joint,
                                 const Vec3 &distal);

// _spring(mass_kg, resonance_hz, damping_ratio) -> (k, c).
[[nodiscard]] std::pair<double, double> spring_params(double mass_kg,
                                                    double resonance_hz,
                                                    double damping_ratio);

// seated_path_masses(rider) — nine keys in dict order:
// torso, pelvis, arms, leg, head, trunk_to_arms, thigh_to_legs,
// shank_foot, upper_arm_fraction.
[[nodiscard]] std::vector<std::pair<std::string, double>>
seated_path_masses(const RiderSpecs &rider);

// _pitch_xz(vec).
[[nodiscard]] double pitch_xz(const Vec3 &vec);

// pedal_spindle_pos(phase_rad, lateral_y_m, crank_len_m) -> ndarray (3,).
[[nodiscard]] Vec3 pedal_spindle_pos(double phase_rad, double lateral_y_m,
                                     double crank_len_m);

// solve_leg_joints(chain, phase_rad, pelvis_z_m=0.0) -> (knee, ankle).
[[nodiscard]] std::pair<Vec3, Vec3>
solve_leg_joints(const LegChain &chain, double phase_rad,
                 double pelvis_z_m = 0.0);

// solve_leg_qpos(chain, phase_rad, pelvis_z_m=0.0) -> (3,) ndarray.
[[nodiscard]] Vec3 solve_leg_qpos(const LegChain &chain, double phase_rad,
                                  double pelvis_z_m = 0.0);

// fk_leg(chain, qpos, pelvis_z_m=0.0) -> (knee, ankle, pedal_pt).
struct LegFkResult {
    Vec3 knee;
    Vec3 ankle;
    Vec3 pedal_pt;
};
[[nodiscard]] LegFkResult fk_leg(const LegChain &chain, const Vec3 &qpos,
                                 double pelvis_z_m = 0.0);

// leg_jacobian(chain, qpos, pelvis_z_m=0.0) -> (3, 2) ndarray, row-major.
using Mat32 = std::array<std::array<double, 2>, 3>;
[[nodiscard]] Mat32 leg_jacobian(const LegChain &chain, const Vec3 &qpos,
                                 double pelvis_z_m = 0.0);

// crank_torque_share(phase_rad, depth).
[[nodiscard]] double crank_torque_share(double phase_rad, double depth);

// solve_seated_pose(specs, rider) — Python argument order (specs first).
[[nodiscard]] SeatedPose solve_seated_pose(const BikeSpecs &specs,
                                           const RiderSpecs &rider);

// RiderSpecs.seated_pose(specs=None) — the member-method port; variant
// must be "seated" and specs defaults to BikeSpecs().
[[nodiscard]] SeatedPose seated_pose(
        const RiderSpecs &rider, const BikeSpecs &specs);
[[nodiscard]] SeatedPose seated_pose(const RiderSpecs &rider);

// RiderSpecs.compute_rider_centers_of_mass(specs=None) — ordered
// (name, (com, mass)) entries in the dict's insertion order.
struct RiderCenter {
    std::string name;
    Vec3 pos_m;
    double mass_kg;
};
[[nodiscard]] std::vector<RiderCenter>
compute_rider_centers_of_mass(const RiderSpecs &rider,
                              const BikeSpecs &specs);
[[nodiscard]] std::vector<RiderCenter>
compute_rider_centers_of_mass(const RiderSpecs &rider);

// saddle_path_apparent_mass(pose, frequencies_hz) -> per-frequency
// |seat_force| / |s|^2 / (m1+m2).
[[nodiscard]] std::vector<double>
saddle_path_apparent_mass(const SeatedPose &pose,
                          std::span<const double> frequencies_hz);

// rider_from_args(variant, mass_kg=80, height_m=1.80, inseam_m=None).
[[nodiscard]] RiderSpecs
rider_from_args(std::string_view variant, double mass_kg = 80.0,
                double height_m = 1.80,
                std::optional<double> inseam_m = std::nullopt);

// resolve_rider(rider, include_rider=None, default_variant="seated").
// rider is None / RiderSpecs / str in Python; the variant encodes the
// same domain. Always returns a validated RiderSpecs.
using RiderArg = std::variant<std::monostate, RiderSpecs, std::string>;
[[nodiscard]] RiderSpecs resolve_rider(
        const RiderArg &rider,
        std::optional<bool> include_rider = std::nullopt,
        std::string_view default_variant = "seated");

// ---------------------------------------------------------------------------
// physics/rider_segments.py
// ---------------------------------------------------------------------------

// segment_masses(total_kg, helmet_kg) — eleven keys in dict order:
// pelvis, torso, head, upper_arm_pair, forearm_pair, then per side
// (front, rear) x segment (thigh, shank, foot).
using SegmentMasses = std::vector<std::pair<std::string, double>>;
[[nodiscard]] SegmentMasses segment_masses(double total_kg,
                                           double helmet_kg);

// split_paired_arm_masses(masses) — pops the two *_pair keys and appends
// halved {part}_{left,right} entries at the tail (dict order preserved).
[[nodiscard]] SegmentMasses split_paired_arm_masses(SegmentMasses masses);

// segment_inertia(mass_kg, axis_vector_m, radius_m) -> (3, 3) ndarray.
[[nodiscard]] Mat3 segment_inertia(double mass_kg, const Vec3 &axis_vector_m,
                                   double radius_m);

// ArticulatedPose — frozen; __post_init__ wraps every non-saddle field in
// checks.array(readonly=True), i.e. a 3-vector finite check.
struct ArticulatedPose {
    SaddleGeometry saddle{};
    Vec3 hip{};
    Vec3 shoulder{};
    Vec3 elbow{};
    Vec3 grip{};
    Vec3 head_center{};
    Vec3 knee_front{};
    Vec3 knee_rear{};
    Vec3 ankle_front{};
    Vec3 ankle_rear{};
    Vec3 pedal_front{};
    Vec3 pedal_rear{};
};
// ArticulatedPose.__post_init__ — "invalid shape or non-finite <name>".
void validate_articulated_pose(const ArticulatedPose &pose);

// geometry_pose(rider, specs).
[[nodiscard]] ArticulatedPose geometry_pose(const RiderSpecs &rider,
                                            const BikeSpecs &specs);

} // namespace bike::model
