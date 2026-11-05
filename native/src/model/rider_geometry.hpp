// model/rider_geometry.hpp — T15 rider boundary: physics/rider.py's
// seated pose solver, plus physics/rider_segments.py's De Leva
// mass/fitting helpers. The
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
// and each sin/cos call takes the numeric::np_* or math_* family of its
// Python mirror (np.* or math.*), so the optimizer cannot fuse a same-angle
// pair into a __sincos_stret the oracle never calls.
#pragma once

#include <array>
#include <optional>
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

// rider.py LUMPED_COM_M — keyed capsule centres (BB frame, metres).
[[nodiscard]] Vec3 lumped_com_m(std::string_view name); // dict []

// ---------------------------------------------------------------------------
// Frozen rider records — field order verbatim.
// ---------------------------------------------------------------------------

// SeatedPose — field order verbatim.
struct SeatedPose {
    SaddleGeometry saddle{};
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

// solve_seated_pose(specs, rider) — Python argument order (specs first).
[[nodiscard]] SeatedPose solve_seated_pose(const BikeSpecs &specs,
                                           const RiderSpecs &rider);

// RiderSpecs.seated_pose(specs=None) — the member-method port; the
// solver takes the record as geometry input; only positions and angles
// are produced, and specs defaults to BikeSpecs().
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

// rider_from_args(variant, mass_kg=80, height_m=1.80, inseam_m=None).
[[nodiscard]] RiderSpecs
rider_from_args(std::string_view variant, double mass_kg = 80.0,
                double height_m = 1.80,
                std::optional<double> inseam_m = std::nullopt);

// resolve_rider(rider, include_rider=None, default_variant="lumped").
// rider is None / RiderSpecs / str in Python; the variant encodes the
// same domain. Always returns a validated RiderSpecs.
using RiderArg = std::variant<std::monostate, RiderSpecs, std::string>;
[[nodiscard]] RiderSpecs resolve_rider(
        const RiderArg &rider,
        std::optional<bool> include_rider = std::nullopt,
        std::string_view default_variant = "lumped");

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
