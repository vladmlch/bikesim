// configuration/rider_spec.hpp — the T12 rider-side input records:
// physics/rider.py RiderSpecs (the member RiderSpecs of BikeConfig), the
// rider_posture.py / rider_program.py / rider_envelope.py / joint_strength.py
// declarations the configuration boundary carries, and the module constants
// its derived helpers consume.
//
// Semantically identical native records are reused, never redeclared:
//   rider::RiderPosture     <- physics/rider_posture.py RiderPosture
//   spindle::JointEnvelope  <- physics/rider_envelope.py JointEnvelope
//   spindle::TorqueCurve    <- physics/joint_strength.py TorqueCurve
// and the geometry solvers behind RiderSpecs.seated_pose /
// compute_rider_centers_of_mass stay with their T15 owner; only the pure
// scalar/ratio helpers are declared here.
#pragma once

#include <array>
#include <cstddef>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "fields.hpp"
#include "../rider/intent.hpp"        // rider::RiderPosture (via spindle_math.hpp)
#include "../rider/rider_posture.hpp"
#include "../rider/spindle_math.hpp"  // spindle::JointEnvelope, spindle::TorqueCurve

namespace bike::configuration {

// ---------------------------------------------------------------------------
// rider.py module constants the input-side helpers consume.
// ---------------------------------------------------------------------------
inline constexpr double kGravityMps2 = 9.81;  // GRAVITY_MPS2

// rider.py RIDER_VARIANTS / DEFAULT_RIDER_VARIANT.
inline constexpr std::string_view kRiderVariants[] = {
    "none", "lumped", "seated", "articulated_planar"};
inline constexpr std::string_view kDefaultRiderVariant = "seated";

// de Leva (1996) Table 4 segment length fractions over reference stature.
inline constexpr double kDeLevaReferenceStatureM = 1.741;
[[nodiscard]] double de_leva_segment_length_fraction(std::string_view name);

inline constexpr double kInseamStatureRatio = 0.47;      // INSEAM_STATURE_RATIO
inline constexpr double kLemondSaddleHeightRatio = 0.883; // LEMOND_SADDLE_HEIGHT_RATIO

// Lumped-variant capsule mass fractions (rider.py authored values).
inline constexpr double kLumpedTorsoHelmetFraction = 55.0 / 80.0;
inline constexpr double kLumpedLegsFraction = 18.0 / 80.0;
inline constexpr double kLumpedArmsFraction = 7.0 / 80.0;

// ---------------------------------------------------------------------------
// rider.py RiderSpecs (lines 139-339) — a plain (mutable) dataclass. The
// geometry-consuming methods (seated_pose, compute_rider_centers_of_mass)
// belong to the T15 pose solver; the input-side record keeps every declared
// field and every pure derived helper.
// ---------------------------------------------------------------------------
struct RiderSpecs {
    std::string variant = "seated";
    std::string legs = "rigid";
    double mass_kg = 80.0;
    double height_m = 1.80;
    std::optional<double> inseam_m;
    double helmet_mass_kg = 0.40;

    double saddle_share = 0.55;
    double pedal_share = 0.33;
    double bar_share = 0.12;

    double torso_resonance_hz = 7.0;
    double leg_resonance_hz = 5.0;
    double arm_resonance_hz = 4.0;
    double damping_ratio = 0.40;
    double saddle_interface_k_n_m = 101'000.0;
    double saddle_interface_c_ns_m = 2'762.0;

    // RiderSpecs.present / total_rider_mass.
    [[nodiscard]] bool present() const { return variant != "none"; }
    [[nodiscard]] double total_rider_mass() const {
        return present() ? mass_kg : 0.0;
    }

    // RiderSpecs.inseam — supplied crotch height or the ANSUR II estimate.
    [[nodiscard]] double inseam() const {
        return inseam_m.value_or(kInseamStatureRatio * height_m);
    }
    // RiderSpecs.saddle_height_m — LeMond BB-center to saddle-top height.
    [[nodiscard]] double saddle_height_m() const {
        return kLemondSaddleHeightRatio * inseam();
    }
    // RiderSpecs.segment_length(name) — de Leva fraction x stature.
    [[nodiscard]] double segment_length(std::string_view name) const;

    // Lumped-variant capsule masses (55/80, 18/80, 7/80 of mass_kg).
    [[nodiscard]] double torso_helmet_mass() const {
        return kLumpedTorsoHelmetFraction * mass_kg;
    }
    [[nodiscard]] double legs_mass() const {
        return kLumpedLegsFraction * mass_kg;
    }
    [[nodiscard]] double arms_mass() const {
        return kLumpedArmsFraction * mass_kg;
    }
};

// ---------------------------------------------------------------------------
// rider_posture.py — RiderPosture is the shared rider::RiderPosture record
// (same four fields/defaults). `post_init` (validate.cpp) applies the
// Python __post_init__ domain; `standing()` is the classmethod factory.
// ---------------------------------------------------------------------------
using RiderPosture = ::rider::RiderPosture;

// RiderPosture.standing(): the standing attack pose command.
[[nodiscard]] RiderPosture standing_posture();

// ---------------------------------------------------------------------------
// rider_envelope.py — JointEnvelope is spindle::JointEnvelope (identical
// five-field record). The q-range/anatomical mapping and soft-edge math
// already live in spindle_math.hpp; this header re-exports the names the
// configuration boundary uses and ports the loaders/state checks.
// ---------------------------------------------------------------------------
using JointEnvelope = ::spindle::JointEnvelope;
using ::spindle::anatomical_angle;
using ::spindle::joint_q_range;
using ::spindle::soft_edge_response;
using ::spindle::soft_edge_scalar;

// rider_envelope.py RIDER_JOINTS — the 11 internal joints of the
// two-arm reference topology, in reference_joint_names() order.
inline constexpr std::string_view kRiderJoints[] = {
    "rider_torso_hinge",
    "rider_shoulder_left", "rider_elbow_left",
    "rider_shoulder_right", "rider_elbow_right",
    "rider_hip_front", "rider_knee_front", "rider_ankle_front",
    "rider_hip_rear", "rider_knee_rear", "rider_ankle_rear"};

// rider_envelope.py LEGACY_RIDER_JOINTS — the nine-joint legacy map.
inline constexpr std::string_view kLegacyRiderJoints[] = {
    "rider_torso_hinge", "rider_shoulder", "rider_elbow",
    "rider_hip_front", "rider_knee_front", "rider_ankle_front",
    "rider_hip_rear", "rider_knee_rear", "rider_ankle_rear"};

// Ordered joint->envelope table (the Python dict's insertion order is kept).
using JointEnvelopeMap =
    std::vector<std::pair<std::string, JointEnvelope>>;

// rider_envelope.py reference_rom_check — joint coordinates in q
// convention keyed by joint name; returns the violating joint names in
// envelope order.
[[nodiscard]] std::vector<std::string> reference_rom_check(
    const std::vector<std::pair<std::string, double>> &joint_angles_rad,
    const JointEnvelopeMap &envelopes);

// rider_envelope.py load_joint_envelopes(path, present=None) — strict JSON
// profile decode: schema_version 1 (legacy joint names) or 2
// (articulated_planar_two_arm topology), or a bare joint map. `present`
// defaults to Python's None (all topology joints); an empty span is the
// distinct empty-set request — both coverage checks still run.
[[nodiscard]] JointEnvelopeMap load_joint_envelopes(
    const std::filesystem::path &path,
    std::optional<std::span<const std::string_view>> present =
        std::nullopt);

// ---------------------------------------------------------------------------
// joint_strength.py — TorqueCurve is spindle::TorqueCurve (identical record)
// and directional_capacity is spindle::directional_capacity; the loaders and
// the per-curve `verified` decoding live here.
// ---------------------------------------------------------------------------
using TorqueCurve = ::spindle::TorqueCurve;
using ::spindle::directional_capacity;

// joint_strength.py _torque_curve: decode one {angles_rad, torques_nm,
// vmax_rad_s, hill_c, eccentric_ratio, source, verified, label} member.
// Returns the validated curve and its verified flag.
[[nodiscard]] std::pair<TorqueCurve, bool>
strength_curve_value(const bike::io::Value &payload, std::string_view joint,
                     std::string_view direction_label);

// joint_strength.py load_strength_coordinate(entry, name) — the
// q->anatomical convention one strength entry declares.
[[nodiscard]] JointEnvelope
strength_coordinate(const bike::io::Value &entry, std::string_view name);

// {joint: {sign: curve}} keyed like the Python dicts — the inner map's keys
// are exactly {-1, +1}; the outer vector preserves `expected_joints` order.
using StrengthProfile = std::vector<
    std::pair<std::string, std::map<int, TorqueCurve>>>;

// joint_strength.py load_strength_profile(path, expected_joints,
// require_verified=...).
[[nodiscard]] StrengthProfile
load_strength_profile(const std::filesystem::path &path,
                      std::span<const std::string_view> expected_joints,
                      bool require_verified);

// joint_strength.py load_strength_coordinates(path, expected_joints).
[[nodiscard]] JointEnvelopeMap
load_strength_coordinates(const std::filesystem::path &path,
                          std::span<const std::string_view> expected_joints);

// ---------------------------------------------------------------------------
// rider_program.py PedalIntent + pedal_intent — the bounded per-cycle wish.
// body_pulse and SeatedPostureProgram already exist as rider::body_pulse /
// rider::SeatedPostureProgram (rider/intent.hpp); they are not redeclared.
// ---------------------------------------------------------------------------
struct PedalIntent {
    double cadence_rad_s;
    double ankle_offset_rad;
    double scrape_fraction;
};

// rider_program.py pedal_intent(phase_rad, cadence_rad_s, *,
//                               ankle_amplitude_rad, scrape_fraction).
[[nodiscard]] PedalIntent pedal_intent(double phase_rad, double cadence_rad_s,
                                       double ankle_amplitude_rad,
                                       double scrape_fraction);

// ---------------------------------------------------------------------------
// Construction-time validators (validate.cpp). Each is the owning record's
// __post_init__: normalize in place and throw std::invalid_argument carrying
// the Python message.
// ---------------------------------------------------------------------------
namespace detail {
void post_init(RiderSpecs &);
void post_init(RiderPosture &);
void post_init(const JointEnvelope &);
void post_init(TorqueCurve &);
} // namespace detail

// ---------------------------------------------------------------------------
// Field table (single declaration shared by schema/provenance/loading).
// ---------------------------------------------------------------------------
namespace detail {

inline const RiderSpecs kDeclaredRiderSpecs{};
inline constexpr FieldDecl kRiderSpecsFields[] = {
    {"variant", "<class 'str'>", FieldKind::Text,
     offsetof(RiderSpecs, variant), false, "", nullptr, nullptr},
    {"legs", "<class 'str'>", FieldKind::Text,
     offsetof(RiderSpecs, legs), false, "", nullptr, nullptr},
    {"mass_kg", "<class 'float'>", FieldKind::Real,
     offsetof(RiderSpecs, mass_kg), false, "", nullptr, nullptr},
    {"height_m", "<class 'float'>", FieldKind::Real,
     offsetof(RiderSpecs, height_m), false, "", nullptr, nullptr},
    {"inseam_m", "typing.Optional[float]", FieldKind::OptionalReal,
     offsetof(RiderSpecs, inseam_m), false, "", nullptr, nullptr},
    {"helmet_mass_kg", "<class 'float'>", FieldKind::Real,
     offsetof(RiderSpecs, helmet_mass_kg), false, "", nullptr, nullptr},
    {"saddle_share", "<class 'float'>", FieldKind::Real,
     offsetof(RiderSpecs, saddle_share), false, "", nullptr, nullptr},
    {"pedal_share", "<class 'float'>", FieldKind::Real,
     offsetof(RiderSpecs, pedal_share), false, "", nullptr, nullptr},
    {"bar_share", "<class 'float'>", FieldKind::Real,
     offsetof(RiderSpecs, bar_share), false, "", nullptr, nullptr},
    {"torso_resonance_hz", "<class 'float'>", FieldKind::Real,
     offsetof(RiderSpecs, torso_resonance_hz), false, "", nullptr, nullptr},
    {"leg_resonance_hz", "<class 'float'>", FieldKind::Real,
     offsetof(RiderSpecs, leg_resonance_hz), false, "", nullptr, nullptr},
    {"arm_resonance_hz", "<class 'float'>", FieldKind::Real,
     offsetof(RiderSpecs, arm_resonance_hz), false, "", nullptr, nullptr},
    {"damping_ratio", "<class 'float'>", FieldKind::Real,
     offsetof(RiderSpecs, damping_ratio), false, "", nullptr, nullptr},
    {"saddle_interface_k_n_m", "<class 'float'>", FieldKind::Real,
     offsetof(RiderSpecs, saddle_interface_k_n_m), false, "", nullptr,
     nullptr},
    {"saddle_interface_c_ns_m", "<class 'float'>", FieldKind::Real,
     offsetof(RiderSpecs, saddle_interface_c_ns_m), false, "", nullptr,
     nullptr},
};
inline const RecordSchema kRiderSpecsSchema{
    "bike_sim.physics.rider.RiderSpecs", kRiderSpecsFields,
    &kDeclaredRiderSpecs};

inline const RiderPosture kDeclaredRiderPosture{};
inline constexpr FieldDecl kRiderPostureFields[] = {
    {"torso_lean_rad", "<class 'float'>", FieldKind::Real,
     offsetof(RiderPosture, torso_lean_rad), false, "", nullptr, nullptr},
    {"pelvis_pitch_rad", "<class 'float'>", FieldKind::Real,
     offsetof(RiderPosture, pelvis_pitch_rad), false, "", nullptr, nullptr},
    {"pelvis_offset_m", "tuple[float, float] | None",
     FieldKind::OptionalReal2Tuple,
     offsetof(RiderPosture, pelvis_offset_m), false, "", nullptr, nullptr},
    {"use_saddle", "<class 'bool'>", FieldKind::Boolean,
     offsetof(RiderPosture, use_saddle), false, "", nullptr, nullptr},
};
inline const RecordSchema kRiderPostureSchema{
    "bike_sim.physics.rider_posture.RiderPosture", kRiderPostureFields,
    &kDeclaredRiderPosture};

} // namespace detail
} // namespace bike::configuration
