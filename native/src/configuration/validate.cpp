// configuration/validate.cpp — the T12 typed-domain validation and
// construction bodies. Every `detail::post_init(T&)` is the owning
// dataclass's __post_init__ run in the Python statement order — it
// normalizes in place (cassette sort, 'weld'->'connect' alias) and throws
// std::invalid_argument carrying the source message. checks.py `derived`
// maps to std::overflow_error, preserving the OverflowError/ValueError
// distinction. Type checks Python performs dynamically (isinstance on
// members) are vacuous on the typed records; the wire layer in schema.cpp
// owns the JSON-type distinctions.
//
// checks.py scalar/integer port (the typed-residue form): a typed member
// can only violate finiteness/domain, so the "expected"/"must be a finite
// real scalar" branches collapse to 'invalid <name>'; integer members keep
// the 'invalid integer domain' wording. The function-parameter checks the
// loaders need (direction bool, verified flag, dict/key decode) live beside
// them, decoding bike::io::Value the way json.loads hands Python objects.
#include "configuration/model_config.hpp"
#include "configuration/rider_spec.hpp"
#include "configuration/suspension_spec.hpp"
#include "config_validation.hpp"
#include "configuration/wire_decode.hpp"
#include "io/json.hpp"
#include "platform/files.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <numbers>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <unistd.h>
#include <utility>
#include <vector>

namespace bike::configuration {
namespace {

// ---- error idioms ----------------------------------------------------------
[[noreturn]] void fail(std::string_view message) {
    throw std::invalid_argument(std::string(message));
}
[[noreturn]] void fail_of(std::string_view message) {
    throw std::overflow_error(std::string(message));
}

// checks.py scalar() typed residue: finite + bounds -> 'invalid <name>'.
double scalar(double value, std::string_view name,
              std::optional<double> minimum = std::nullopt,
              bool positive = false) {
    if (!std::isfinite(value) || (minimum && value < *minimum) ||
        (positive && value <= 0.))
        fail("invalid " + std::string(name));
    return value;
}
// scalar() with an attached label string literal.
double scalar(double value, const char *name,
              std::optional<double> minimum = std::nullopt,
              bool positive = false) {
    return scalar(value, std::string_view(name), minimum, positive);
}

// checks.py integer() typed residue: the member is already an integer, so
// only the domain check can fire.
long long integer(long long value, std::string_view name,
                  std::optional<long long> minimum = std::nullopt,
                  bool positive = false) {
    if ((minimum && value < *minimum) || (positive && value <= 0))
        fail(std::string(name) + ": invalid integer domain");
    return value;
}

// checks.py derived(): a computed value must stay finite — OverflowError
// in Python, std::overflow_error here.
double derived(double value, std::string_view name) {
    if (!std::isfinite(value))
        fail_of(std::string(name) + ": non-finite derived value");
    return value;
}

// Python f"{x:.6f}" — fixed notation, six decimals; nonfinite spells
// 'inf'/'nan' like printf %.6f. The worst-case fixed spelling (DBL_MAX)
// needs ~316 chars.
std::string format_fixed6(double value) {
    std::array<char, 384> buf{};
    const auto out =
        std::to_chars(buf.data(), buf.end(), value,
                      std::chars_format::fixed, 6);
    return {buf.data(), out.ptr};
}

// str.strip() boundary: the nativecfg whitespace table is the Python
// strip set at the UTF-8 level.
bool nonblank(std::string_view text) { return nativecfg::nonblank(text); }

// Bounded whole-file read through the T06 boundary (regular file, no-follow,
// 512 MiB cap — same bound as the fixture/JSON input domains).
std::string read_regular(const std::filesystem::path &path) {
    const platform::FileHandle handle = platform::open_regular(path);
    std::string content;
    // 4 KiB read chunks: inside the 8 KiB -Wframe-larger-than bound while
    // still bounded-read semantics over the same 512 MiB cap.
    std::array<char, 4096> chunk{};
    std::size_t total = 0;
    for (;;) {
        const ssize_t got =
            ::read(handle.get(), chunk.data(), chunk.size());
        if (got == 0) break;
        if (got < 0) {
            if (errno == EINTR) continue;
            throw std::system_error(std::error_code(errno, std::generic_category()),
                                    "read " + path.string());
        }
        total += static_cast<std::size_t>(got);
        if (total > bike::io::kMaxJsonBytes)
            fail("profile file exceeds the bounded input domain");
        content.append(chunk.data(), static_cast<std::size_t>(got));
    }
    return content;
}

bike::io::Value read_json(const std::filesystem::path &path) {
    return bike::io::parse_json(read_regular(path));
}

// str.join over a set of allowed names in registry/declaration order.
std::string joined(std::span<const std::string_view> names) {
    std::string out;
    for (std::size_t i = 0; i < names.size(); ++i) {
        if (i) out += ", ";
        out += names[i];
    }
    return out;
}

// terrain/surface.py SURFACES registry order (PneumaticTireConfig surface
// membership check + its 'available:' list).
inline constexpr std::array<std::string_view, 5> kSurfaces{
    "asphalt", "hardpack", "loose", "wet", "scree"};

bool in_set(std::string_view value, std::span<const std::string_view> set) {
    return std::ranges::find(set, value) != set.end();
}
bool in_set(std::string_view value,
            std::initializer_list<std::string_view> set) {
    return std::ranges::find(set, value) != set.end();
}

// Ordered range check used by DistributedWheelMaterial/DistributedTireConfig
// and the wire _check_distributed_material.
bool ordered_domain(const std::array<double, 2> &range) {
    return std::isfinite(range[0]) && std::isfinite(range[1]) &&
           0. <= range[0] && range[0] < range[1];
}

} // namespace

// ===========================================================================
// physical_config.py + nested-record __post_init__ bodies
// ===========================================================================
namespace detail {

void post_init(TireSpec &spec) {
    // tire.py TireSpec.__post_init__ — bare field names are the labels.
    scalar(spec.radial_k_n_m, "radial_k_n_m");
    scalar(spec.radial_c_ns_m, "radial_c_ns_m");
    scalar(spec.pressure_pa_gauge, "pressure_pa_gauge");
    if (spec.radial_k_n_m <= 0.) fail("radial_k_n_m must be positive");
    if (spec.radial_c_ns_m < 0. || spec.pressure_pa_gauge < 0.)
        fail("radial damping and gauge pressure must be nonnegative");
    if (!nonblank(spec.provenance))
        fail("provenance must be a nonempty description, e.g. synthetic");
    // `lower, upper = valid_load_range_n` — the two-element shape is part of
    // the member type, so only the scalar/bounds checks can fire.
    const double lower =
        scalar(spec.valid_load_range_n[0], "minimum valid load");
    const double upper =
        scalar(spec.valid_load_range_n[1], "maximum valid load");
    if (!(0. <= lower && lower < upper))
        fail("valid load bounds must satisfy 0 <= minimum < maximum");
}

namespace {

// tire.py TireSpec.validate() — the labels carry the 'TireSpec.' prefix;
// TireParameters.__post_init__ calls this on a TireSpec material.
void validate_tire_material(const TireSpec &spec) {
    scalar(spec.radial_k_n_m, "TireSpec.radial_k_n_m",
           std::nullopt, true);
    scalar(spec.radial_c_ns_m, "TireSpec.radial_c_ns_m", 0.);
    scalar(spec.pressure_pa_gauge, "TireSpec.pressure_pa_gauge", 0.);
    if (!nonblank(spec.provenance))
        fail("TireSpec.provenance: nonblank description required");
    for (const double bound : spec.valid_load_range_n)
        if (!std::isfinite(bound))
            fail("invalid shape or non-finite TireSpec.valid_load_range_n");
    if (!(0. <= spec.valid_load_range_n[0] &&
          spec.valid_load_range_n[0] < spec.valid_load_range_n[1]))
        fail("TireSpec.valid_load_range_n: expected 0 <= lower < upper");
}

} // namespace

void post_init(TabulatedTireSpec &spec) {
    for (const double v : spec.deflection_m) scalar(v, "deflection");
    for (const double v : spec.force_n) scalar(v, "elastic force");
    const auto &x = spec.deflection_m;
    const auto &y = spec.force_n;
    if (x.size() != y.size() || x.size() < 2)
        fail("tire curve requires at least two paired samples");
    if (x.front() != 0. || y.front() != 0.)
        fail("tire curve must start at zero deflection and force");
    for (std::size_t i = 1; i < x.size(); ++i)
        if (x[i] <= x[i - 1]) fail("tire deflection must strictly increase");
    for (std::size_t i = 1; i < y.size(); ++i)
        if (y[i] < y[i - 1])
            fail("elastic tire force must be nondecreasing and positive at "
                 "the end");
    if (y.back() <= 0.)
        fail("elastic tire force must be nondecreasing and positive at the "
             "end");
    for (const auto& [member, value] :
         {std::pair{"radial_c_ns_m", spec.radial_c_ns_m},
          {"pressure_pa_gauge", spec.pressure_pa_gauge}}) {
        scalar(value, member);
        if (value < 0.)
            fail(std::string(member) + " must be nonnegative");
    }
    if (!nonblank(spec.provenance))
        fail("tire curve requires explicit provenance");
    // tire_curve.py wraps both _scalar calls: any TypeError/ValueError
    // inside re-raises as the range-diagnostic, not 'invalid valid load'.
    double lo = 0., hi = 0.;
    try {
        lo = scalar(spec.valid_load_range_n[0], "valid load");
        hi = scalar(spec.valid_load_range_n[1], "valid load");
    } catch (const std::invalid_argument &) {
        fail("valid load range requires two finite bounds");
    }
    if (!(0. <= lo && lo < hi && hi <= y.back()))
        fail("valid load range must fit within the supplied force table");
    // The slopes/integral derived sweep is part of construction.
    std::vector<double> slopes, integral{0.};
    slopes.reserve(x.size() - 1);
    integral.reserve(x.size());
    for (std::size_t i = 1; i < x.size(); ++i) {
        slopes.push_back((y[i] - y[i - 1]) / (x[i] - x[i - 1]));
        integral.push_back(integral.back() +
                           .5 * (y[i - 1] + y[i]) * (x[i] - x[i - 1]));
    }
    for (const double v : slopes)
        if (!std::isfinite(v))
            fail_of("tire calculation exceeds finite floating-point range");
    for (const double v : integral)
        if (!std::isfinite(v))
            fail_of("tire calculation exceeds finite floating-point range");
}

void post_init(const HingeDensity &density) {
    // distributed_tire.py HingeDensity.__post_init__.
    const auto &a = density.knots_m;
    const auto &k = density.stiffness_n_m;
    if (a.empty() || a.size() != k.size()) fail("invalid density table shape");
    const bool finite = std::ranges::all_of(a, [](double v) {
        return std::isfinite(v);
    }) && std::ranges::all_of(k, [](double v) {
        return std::isfinite(v);
    }) && std::isfinite(density.damping_ns_m);
    if (!finite) fail("density parameters must be finite");
    bool monotone = a.front() == 0.;
    for (std::size_t i = 1; i < a.size(); ++i)
        if (a[i] <= a[i - 1]) monotone = false;
    const bool nonnegative = std::ranges::all_of(k, [](double v) {
        return v >= 0.;
    }) && density.damping_ns_m >= 0.;
    if (!monotone || !nonnegative)
        fail("density law must be monotone and dissipative");
    if (!nonblank(density.provenance))
        fail("material provenance is required");
}

void post_init(DistributedWheelMaterial &material) {
    // Construction-time member validation precedes the body, like the
    // Python dataclass field(default_factory=HingeDensity) / argument
    // construction.
    post_init(material.density);
    if (material.calibration_status != "experimental_unvalidated")
        fail("a material record cannot certify the plant");
    if (!nonblank(material.fitting_dataset_id))
        fail("fitting dataset ID is required");
    for (const auto *range : {&material.valid_load_range_n,
                              &material.valid_deflection_range_m})
        if (!ordered_domain(*range))
            fail("invalid per-wheel material domain");
}

void post_init(DistributedTireConfig &config) {
    post_init(config.density);
    // distributed_tire.py station_angles() domain on the int member.
    if (config.station_count < 16)
        fail("at least sixteen tread stations required");
    if (config.station_count > 512)
        fail("at most 512 tread stations supported");
    for (const auto *range : {&config.valid_load_range_n,
                              &config.valid_deflection_range_m})
        if (!ordered_domain(*range))
            fail("invalid distributed tire applicability range");
    if (!nonblank(config.fitting_dataset_id))
        fail("a fitting dataset ID or explicit synthetic ID is required");
    if (config.calibration_status != "experimental_unvalidated")
        fail("distributed tire acceptance is evaluated in the validation "
             "report");
    static_cast<void>(config.for_wheel("front"));
    static_cast<void>(config.for_wheel("rear"));
}

void post_init(const TyreSpecs &spec) {
    // tyre.py TyreSpecs.__post_init__ — comparisons, not scalar(): labels
    // embed the Python str() of the offending value.
    static constexpr std::array kPositive{
        std::pair{"outer_radius_mm", &TyreSpecs::outer_radius_mm},
        std::pair{"rim_radius_mm", &TyreSpecs::rim_radius_mm},
        std::pair{"section_height_mm", &TyreSpecs::section_height_mm},
        std::pair{"pressure_bar", &TyreSpecs::pressure_bar},
        std::pair{"crr_reference", &TyreSpecs::crr_reference},
        std::pair{"casing_width_mm", &TyreSpecs::casing_width_mm},
        std::pair{"compressed_casing_mm", &TyreSpecs::compressed_casing_mm},
        std::pair{"area_factor", &TyreSpecs::area_factor},
        std::pair{"contact_length_factor", &TyreSpecs::contact_length_factor},
        std::pair{"rate_relaxation_s", &TyreSpecs::rate_relaxation_s},
        std::pair{"hysteresis_rate_eps_mps",
                  &TyreSpecs::hysteresis_rate_eps_mps},
        std::pair{"relaxation_length_mm", &TyreSpecs::relaxation_length_mm},
        std::pair{"rim_stiffness_n_mm2", &TyreSpecs::rim_stiffness_n_mm2},
    };
    for (const auto &[field, member] : kPositive)
        if (!(spec.*member > 0.))
            fail("tyre '" + spec.name + "': " + field +
                 " must be positive, got " + repr_double(spec.*member));
    static constexpr std::array kNonnegative{
        std::pair{"carcass_stiffness_n_mm2",
                  &TyreSpecs::carcass_stiffness_n_mm2},
        std::pair{"rate_stiffening", &TyreSpecs::rate_stiffening},
        std::pair{"loss_factor", &TyreSpecs::loss_factor},
        std::pair{"tread_loss_crr", &TyreSpecs::tread_loss_crr},
    };
    for (const auto &[field, member] : kNonnegative)
        if (spec.*member < 0.)
            fail("tyre '" + spec.name + "': " + field +
                 " must not be negative, got " + repr_double(spec.*member));
    if (spec.rim_radius_mm >= spec.outer_radius_mm)
        fail("tyre '" + spec.name + "': rim radius " +
             repr_double(spec.rim_radius_mm) +
             " mm is not inside the outer radius " +
             repr_double(spec.outer_radius_mm) + " mm");
    if (spec.rim_strike_deflection_mm() <= 0.)
        fail("tyre '" + spec.name + "': compressed casing " +
             repr_double(spec.compressed_casing_mm) +
             " mm leaves no travel above the rim");
    if (spec.area_factor > 1. || spec.contact_length_factor > 1.)
        fail("tyre '" + spec.name +
             "': area and contact-length factors are at most 1");
    if (spec.loss_factor >= 1.)
        fail("tyre '" + spec.name +
             "': loss factor must be below 1, got " +
             repr_double(spec.loss_factor));
}

void post_init(TireParameters &params) {
    // Member construction-time validation: the active material arm was
    // "constructed" before TireParameters.__post_init__ runs.
    if (auto *spec = std::get_if<TireSpec>(&params.material))
        post_init(*spec);
    else
        post_init(std::get<TabulatedTireSpec>(params.material));
    if (const auto *spec = std::get_if<TireSpec>(&params.material))
        validate_tire_material(*spec);
    scalar(params.tangent_k_n_m, "tangent_k_n_m", std::nullopt, true);
    scalar(params.relaxation_length_m, "relaxation_length_m", std::nullopt,
           true);
    scalar(params.mu, "tire friction", 0.);
}

void post_init(PneumaticTireConfig &config) {
    post_init(config.front);
    post_init(config.rear);
    if (find_tire_tier(config.tier) == nullptr) {
        // ', '.join(TIERS) over the registry keys in declaration order.
        std::string names;
        bool first = true;
        for (const TierSpec &tier : kTireTiers) {
            if (!first) names += ", ";
            first = false;
            names += tier.name;
        }
        fail("unknown pneumatic tyre tier '" + config.tier +
             "'; available: " + names);
    }
    // PneumaticTireConfig.__post_init__ re-runs each side explicitly.
    post_init(config.front);
    post_init(config.rear);
    if (config.surface && !in_set(*config.surface, kSurfaces))
        fail("unknown surface '" + *config.surface +
             "'; available: " + joined(kSurfaces));
}

void post_init(TireBackendConfig &config) {
    // Member construction order: front, rear, distributed, pneumatic.
    post_init(config.front);
    post_init(config.rear);
    post_init(config.distributed);
    post_init(config.pneumatic);
    if (!in_set(config.backend,
                std::initializer_list<std::string_view>{
                    "native_reference", "compliant_2d",
                    "distributed_2d_reference", "pneumatic"}))
        fail("unknown physical tire backend");
    if (config.surface_mode != "configured" && config.surface_mode != "track")
        fail("surface_mode must be configured or track");
    if (config.surface_mode == "track" &&
        !in_set(config.backend,
                std::initializer_list<std::string_view>{
                    "compliant_2d", "distributed_2d_reference", "pneumatic"}))
        fail("track material resolution requires a compliant_2d, "
             "distributed_2d_reference or pneumatic tire backend");
    post_init(config.pneumatic);  // the body's explicit re-run
    if (config.backend == "pneumatic" && config.surface_mode != "track" &&
        !config.pneumatic.surface)
        fail("pneumatic tires require surface_mode track or a named surface");
    post_init(config.front);
    post_init(config.rear);
    scalar(config.significant_delta_m, "significant penetration", 0.);
    const double fraction =
        scalar(config.significance_fraction, "significance fraction", 0.);
    const double angle =
        scalar(config.distinct_normal_deg, "distinct normal angle",
               std::nullopt, true);
    if (fraction > 1. || angle >= 180.)
        fail("invalid multi-support thresholds");
}

void post_init(const DrivetrainSpecs &specs) {
    integer(specs.front_teeth, "DrivetrainSpecs.front_teeth", 3);
    integer(specs.rear_teeth, "DrivetrainSpecs.rear_teeth", 3);
    scalar(specs.chain_pitch_m, "chain pitch", std::nullopt, true);
}

void post_init(const MotorProfile &profile) {
    // motor_profile.py MotorProfile.validate().
    for (const auto& [name, member] :
         {std::pair{"peak_torque_nm", &MotorProfile::peak_torque_nm},
          {"rated_power_w", &MotorProfile::rated_power_w},
          {"peak_power_w", &MotorProfile::peak_power_w},
          {"torque_tau_s", &MotorProfile::torque_tau_s},
          {"emtb_full_gain_at_nm", &MotorProfile::emtb_full_gain_at_nm},
          {"cadence_support_max_rpm",
           &MotorProfile::cadence_support_max_rpm},
          {"cutoff_mps", &MotorProfile::cutoff_mps},
          {"taper_width_mps", &MotorProfile::taper_width_mps},
          {"gate_min_crank_rad_s", &MotorProfile::gate_min_crank_rad_s}})
        scalar(profile.*member, name, std::nullopt, true);
    if (!nonblank(profile.name)) fail("motor profile requires a name");
    if (profile.rated_power_w > profile.peak_power_w)
        fail("rated motor power exceeds peak power");
    if (profile.taper_width_mps > profile.cutoff_mps)
        fail("speed taper exceeds the cutoff speed");
    // mode_gains is the fixed (eco, tour, emtb, turbo) struct — the
    // set-equality over its keys is satisfied by construction. Python
    // iterates the mapping in insertion order: eco, tour, emtb, turbo.
    scalar(profile.mode_gains.eco, "eco gain", 0.);
    scalar(profile.mode_gains.tour, "tour gain", 0.);
    // The 'emtb' arm is the ordered (low, high) pair.
    scalar(profile.mode_gains.emtb[0], "eMTB gain", 0.);
    scalar(profile.mode_gains.emtb[1], "eMTB gain", 0.);
    if (profile.mode_gains.emtb[0] > profile.mode_gains.emtb[1])
        fail("eMTB gain bounds must increase");
    scalar(profile.mode_gains.turbo, "turbo gain", 0.);
    if (!nonblank(profile.provenance))
        fail("motor profile requires provenance");
}

void post_init(AssistConfig &config) {
    // AssistConfig.__post_init__ -> AssistController(**asdict) ctor domain.
    // The controller substitutes resolved profile values locally; the
    // record's own members are never rewritten.
    MotorProfile resolved;
    const MotorProfile *profile = nullptr;
    if (config.profile) {
        profile = find_motor_profile(*config.profile);
        if (profile == nullptr)
            fail("unknown motor profile " + detail::py_repr_str(*config.profile));
        resolved = *profile;
        post_init(resolved);
    }
    // (torque_curve normalization `tuple(tuple(p) ...)` is a typed no-op.)
    double max_torque = config.max_torque;
    double max_power = config.max_power;
    double tau = config.tau;
    double cutoff_mps = config.cutoff_mps;
    double taper_width_mps = config.taper_width_mps;
    double gate_min_crank_rad_s = config.gate_min_crank_rad_s;
    if (profile != nullptr) {
        max_torque = profile->peak_torque_nm;
        max_power = profile->peak_power_w;
        tau = profile->torque_tau_s;
        cutoff_mps = profile->cutoff_mps;
        taper_width_mps = profile->taper_width_mps;
        gate_min_crank_rad_s = profile->gate_min_crank_rad_s;
        // `mode in p.mode_gains` — the four fixed keys.
        if (!in_set(config.mode,
                    std::initializer_list<std::string_view>{"eco", "tour",
                                                            "emtb",
                                                            "turbo"}))
            fail("unknown assist mode " + detail::py_repr_str(config.mode) +
                 " for profile " + profile->name);
    }
    scalar(config.gain, "gain", 0.);
    scalar(max_torque, "max_torque", 0.);
    scalar(max_power, "max_power", 0.);
    scalar(config.engage_torque_nm, "engage_torque_nm", 0.);
    scalar(gate_min_crank_rad_s, "gate_min_crank_rad_s", 0.);
    scalar(cutoff_mps, "cutoff_mps", 0.);
    scalar(tau, "assist lag", std::nullopt, true);
    scalar(config.slew, "assist slew", std::nullopt, true);
    scalar(taper_width_mps, "assist taper width", std::nullopt, true);
    if (taper_width_mps > cutoff_mps)
        fail("speed taper exceeds the cutoff speed");
    if (profile != nullptr) {
        const double slew_floor =
            derived(profile->peak_torque_nm / profile->torque_tau_s,
                    "AssistController.profile_slew");
        (void)slew_floor;  // effective slew = max(slew, floor) is controller
                           // state, not config state.
    }
    if (config.torque_curve) {
        const auto &curve = *config.torque_curve;
        for (const auto &row : curve)
            for (const double v : row)
                if (!std::isfinite(v))
                    fail("invalid shape or non-finite torque curve");
        if (curve.size() < 2)
            fail("torque curve needs at least two rpm/torque points");
        for (const auto &row : curve)
            if (row[0] < 0. || row[1] < 0.)
                fail("torque curve rpm must increase strictly; limits must "
                     "be nonnegative");
        for (std::size_t i = 1; i < curve.size(); ++i)
            if (curve[i][0] <= curve[i - 1][0])
                fail("torque curve rpm must increase strictly; limits must "
                     "be nonnegative");
    }
}

void post_init(const BatteryConfig &config) {
    // 'enabled' is a typed bool — the isinstance check is vacuous.
    scalar(config.energy_j, "energy_j", 0.);
    scalar(config.copper_w_per_nm2, "copper_w_per_nm2", 0.);
    scalar(config.speed_w_per_rad_s2, "speed_w_per_rad_s2", 0.);
    scalar(config.idle_w, "idle_w", 0.);
}

void post_init(const PedalingConfig &config) {
    scalar(config.coast_above_rpm, "coasting cadence", std::nullopt, true);
    scalar(config.resume_below_rpm, "resume cadence", 0.);
    scalar(config.stop_time_s, "coasting stop time", std::nullopt, true);
    scalar(config.coast_cadence_tau_s, "coast cadence tau", 0.);
    if (config.resume_below_rpm >= config.coast_above_rpm)
        fail("resume cadence must be below coasting cadence");
    scalar(config.mash_torque_nm, "mash torque", 0.);
    scalar(config.mash_cadence_rpm, "mash cadence", 0.);
    scalar(config.effort_slew_nm_s, "effort slew", 0.);
    if (config.mash_torque_nm > 0. && config.mash_cadence_rpm <= 0.)
        fail("a nonzero mash torque needs a positive mash cadence");
    scalar(config.rollback_engage_mps, "rollback engage speed",
           std::nullopt, true);
    const double release =
        scalar(config.rollback_release_mps, "rollback release speed", 0.);
    if (release >= config.rollback_engage_mps)
        fail("rollback release must be slower than engage");
    const double demand =
        scalar(config.rollback_demand, "rollback brake demand", 0.);
    if (demand > 1.) fail("rollback brake demand must not exceed one");
}

void post_init(ShiftingConfig &config) {
    if (config.upshift_slip_mode != "legacy_signed" &&
        config.upshift_slip_mode != "magnitude")
        fail("unknown upshift slip mode");
    for (const int tooth : config.cassette)
        integer(tooth, "ShiftingConfig.cassette", 3);
    if (config.cassette.empty())
        fail("cassette needs integer tooth counts of at least three");
    {
        const std::set<int> distinct(config.cassette.begin(),
                                     config.cassette.end());
        if (distinct.size() != config.cassette.size())
            fail("cassette tooth counts must be distinct");
    }
    std::ranges::sort(config.cassette);
    scalar(config.target_cadence_min_rpm, "minimum shift cadence",
           std::nullopt, true);
    scalar(config.target_cadence_max_rpm, "maximum shift cadence",
           std::nullopt, true);
    if (config.target_cadence_max_rpm <= config.target_cadence_min_rpm)
        fail("maximum shift cadence must exceed minimum shift cadence");
    scalar(config.shift_cooldown_s, "shift cooldown", 0.);
    scalar(config.shift_cut_duration_s, "shift torque cut duration", 0.);
    scalar(config.cadence_smoothing_tau_s, "shift cadence smoothing", 0.);
    scalar(config.upshift_slip_limit_mps, "upshift rear slip limit", 0.);
    if (config.shift_cut_duration_s > config.shift_cooldown_s)
        fail("shift torque cut must not outlast the cooldown");
    if (scalar(config.torque_factor, "shift torque factor", 0.) > 1.)
        fail("shift torque factor must not exceed one");
}

void post_init(PhysicalDriveConfig &config) {
    // Python order: gearing/assist/battery then pedaling, the transmission
    // and rotor/clutch gates, THEN shifting.__post_init__ (line order in
    // physical_config.py) — a bad cassette cannot mask an unknown
    // transmission model.
    post_init(config.gearing);
    post_init(config.assist);
    post_init(config.battery);
    post_init(config.pedaling);
    if (config.transmission_model != "elastic_chain" &&
        config.transmission_model != "ideal_mid_drive" &&
        config.transmission_model != "geometric_ideal_mid_drive")
        fail("unknown transmission model");
    scalar(config.rotor_inertia_kgm2, "rotor_inertia_kgm2", 0.);
    if (config.rotor_inertia_kgm2 > 0. && config.motor_clutch)
        fail("rotor freewheel and the legacy crank clutch are exclusive");
    const bool mid_drive = config.transmission_model == "ideal_mid_drive" ||
        config.transmission_model == "geometric_ideal_mid_drive";
    if (config.rotor_inertia_kgm2 > 0. && !mid_drive)
        fail("a motor rotor requires an ideal mid-drive transmission");
    if (config.motor_clutch && !mid_drive)
        fail("motor clutch requires an ideal mid-drive transmission");
    post_init(config.shifting);
    if (config.shifting.enabled) {
        if (!mid_drive)
            fail("automatic physical shifting requires ideal_mid_drive");
        if (std::ranges::find(config.shifting.cassette,
                              config.gearing.rear_teeth) ==
            config.shifting.cassette.end())
            fail("initial rear sprocket must belong to the cassette");
    }
    scalar(config.chain_k_n_m, "chain_k_n_m", std::nullopt, true);
    scalar(config.freehub_k_nm_rad, "freehub_k_nm_rad", std::nullopt, true);
    for (const auto& [name, member] :
         {std::pair{"human_torque_nm", &PhysicalDriveConfig::human_torque_nm},
          {"torque_ripple", &PhysicalDriveConfig::torque_ripple},
          {"chain_c_ns_m", &PhysicalDriveConfig::chain_c_ns_m},
          {"freehub_c_nms_rad", &PhysicalDriveConfig::freehub_c_nms_rad},
          {"bearing_c_nms_rad", &PhysicalDriveConfig::bearing_c_nms_rad},
          {"brake_ceiling_nm", &PhysicalDriveConfig::brake_ceiling_nm}})
        scalar(config.*member, name, 0.);
    scalar(config.crank_phase_rad, "initial crank phase");
    if (config.torque_ripple >= 1.) fail("torque ripple must be less than one");
}

void post_init(ResistanceConfig &config) {
    scalar(config.crr, "crr", 0.);
    scalar(config.rho_kg_m3, "rho_kg_m3", 0.);
    scalar(config.cda_m2, "cda_m2", 0.);
    scalar(config.rolling_taper_rad_s, "rolling taper", std::nullopt, true);
    for (const auto &[name, member] :
         {std::pair{"wind_world_mps", &ResistanceConfig::wind_world_mps},
          {"point_body_m", &ResistanceConfig::point_body_m}}) {
        for (const double v : config.*member)
            if (!std::isfinite(v))
                fail("invalid shape or non-finite " + std::string(name));
        if ((config.*member)[1] != 0.)
            fail("resistance configuration must be planar");
    }
}

void post_init(ArticulatedConfig &config) {
    scalar(config.arm_reach_fraction, "arm reach fraction", std::nullopt,
           true);
    if (!(0. < config.arm_reach_fraction && config.arm_reach_fraction < 1.))
        fail("arm reach fraction must lie strictly between zero and one");
    if (config.pedal_support_fraction + config.bar_support_fraction >= 1.)
        fail("postural support fractions must leave a saddle share");
    if (config.joint_envelope_path &&
        !nonblank(*config.joint_envelope_path))
        fail("joint_envelope_path must be a nonempty path or None");
    if (config.joint_strength_path &&
        !nonblank(*config.joint_strength_path))
        fail("joint_strength_path must be a nonempty path or None");
    // The dataclass-fields loop in declaration order: every non-structural
    // scalar carries minimum=0 with the bare field name as label.
    static const std::set<std::string_view> kSkipped{
        "joint_envelope_path", "joint_strength_path", "pedal_attachment",
        "saddle_attachment", "grip_attachment", "coupled_task_control"};
    for (const FieldDecl &field : kArticulatedConfigFields) {
        if (kSkipped.contains(field.name)) continue;
        // Compile-time offsetof bound against the owning record — the
        // uintptr round-trip keeps the arithmetic out of the pointer
        // domain (-Wunsafe-buffer-usage).
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast,performance-no-int-to-ptr) see above
        const void *address = reinterpret_cast<const void *>(
            // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast) see above
            reinterpret_cast<std::uintptr_t>(&config) + field.offset);
        if (field.kind == FieldKind::OptionalReal) {
            const auto &opt =
                *static_cast<const std::optional<double> *>(address);
            if (!opt) continue;
            scalar(*opt, field.name, 0.);
        } else {
            scalar(*static_cast<const double *>(address), field.name, 0.);
        }
    }
    if (config.grip_pair_force_limit_n)
        scalar(*config.grip_pair_force_limit_n, "pair grip force limit",
               std::nullopt, true);
    for (const auto& [name, member] :
         {std::pair{"support_pad_radius_m",
                    &ArticulatedConfig::support_pad_radius_m},
          {"stance_blend_load_n", &ArticulatedConfig::stance_blend_load_n},
          {"support_k_n_m", &ArticulatedConfig::support_k_n_m},
          {"support_tangent_k_n_m",
           &ArticulatedConfig::support_tangent_k_n_m},
          {"support_length_m", &ArticulatedConfig::support_length_m},
          {"grip_k_n_m", &ArticulatedConfig::grip_k_n_m},
          {"grip_release_distance_m",
           &ArticulatedConfig::grip_release_distance_m},
          {"joint_speed_limit_rad_s",
           &ArticulatedConfig::joint_speed_limit_rad_s},
          {"pedal_min_normal_n", &ArticulatedConfig::pedal_min_normal_n},
          {"grip_pull_per_hand_n",
           &ArticulatedConfig::grip_pull_per_hand_n},
          {"link_max_gap_m", &ArticulatedConfig::link_max_gap_m},
          {"balance_floor_kmh", &ArticulatedConfig::balance_floor_kmh},
          {"balance_dwell_s", &ArticulatedConfig::balance_dwell_s}})
        scalar(config.*member, name, std::nullopt, true);
    if (config.saddle_reserve_weight_fraction > 1.)
        fail("saddle reserve fraction must not exceed one");
    if (config.joint_passive_damping_nms_rad)
        scalar(*config.joint_passive_damping_nms_rad,
               "joint_passive_damping_nms_rad", 0.);
    if (config.pedal_torque_ripple >= 1.)
        fail("pedal torque ripple must be below one");
    if (config.pedal_scrape_fraction > 1.)
        fail("pedal scrape fraction must not exceed one");
    if (!in_set(config.pedal_attachment,
                std::initializer_list<std::string_view>{"flat", "weld",
                                                        "spindle"}))
        fail("pedal_attachment must be 'flat', 'weld' or 'spindle', got " +
             detail::py_repr_str(config.pedal_attachment));
    if (!in_set(config.saddle_attachment,
                std::initializer_list<std::string_view>{"flat", "weld",
                                                        "pin"}))
        fail("saddle_attachment must be 'flat', 'weld' or 'pin', got " +
             detail::py_repr_str(config.saddle_attachment));
    if (config.grip_attachment == "weld")
        config.grip_attachment = "connect";
    if (config.grip_attachment != "spring" &&
        config.grip_attachment != "connect")
        fail("grip_attachment must be 'spring' or 'connect', got " +
             detail::py_repr_str(config.grip_attachment));
}

// ---- model_config.py -------------------------------------------------------
void post_init(const EndStopConfig &config) {
    scalar(config.reference_force_n, "EndStopConfig.reference_force_n", 0.);
    scalar(config.reference_deflection_m,
           "EndStopConfig.reference_deflection_m", std::nullopt, true);
    scalar(config.damping_n_s_m, "EndStopConfig.damping_n_s_m", 0.);
    scalar(config.overtravel_m, "EndStopConfig.overtravel_m", std::nullopt,
           true);
    if (config.overtravel_m < config.reference_deflection_m)
        fail("EndStopConfig.overtravel_m: must reach reference deflection");
}

void post_init(const SeatedClimbConfig &config) {
    // seated_climb.py SeatedClimbConfig.__post_init__ on the shared
    // rider::IntentConfig record.
    static constexpr std::array kPositive{
        std::pair{"period_s", &SeatedClimbConfig::period_s},
        std::pair{"target_crank_power_w",
                  &SeatedClimbConfig::target_crank_power_w},
        std::pair{"max_crank_torque_nm",
                  &SeatedClimbConfig::max_crank_torque_nm},
        std::pair{"torque_slew_nm_s", &SeatedClimbConfig::torque_slew_nm_s},
        std::pair{"lean_rate_rad_s", &SeatedClimbConfig::lean_rate_rad_s},
        std::pair{"orientation_tau_s", &SeatedClimbConfig::orientation_tau_s},
        std::pair{"surge_power_w", &SeatedClimbConfig::surge_power_w},
        std::pair{"surge_budget_s", &SeatedClimbConfig::surge_budget_s},
    };
    for (const auto &[name, member] : kPositive)
        scalar(config.*member, name, std::nullopt, true);
    static constexpr std::array kMinimum{
        std::pair{"reaction_delay_s", &SeatedClimbConfig::reaction_delay_s},
        std::pair{"lean_gain", &SeatedClimbConfig::lean_gain},
        std::pair{"max_forward_lean_rad",
                  &SeatedClimbConfig::max_forward_lean_rad},
        std::pair{"max_backward_lean_rad",
                  &SeatedClimbConfig::max_backward_lean_rad},
        std::pair{"surge_grade", &SeatedClimbConfig::surge_grade},
        std::pair{"surge_recovery_rate",
                  &SeatedClimbConfig::surge_recovery_rate},
        std::pair{"front_load_share_target",
                  &SeatedClimbConfig::front_load_share_target},
        std::pair{"lean_trim_gain_rad_s",
                  &SeatedClimbConfig::lean_trim_gain_rad_s},
        std::pair{"lean_trim_limit_rad",
                  &SeatedClimbConfig::lean_trim_limit_rad},
        std::pair{"trim_dead_time_s", &SeatedClimbConfig::trim_dead_time_s},
    };
    for (const auto &[name, member] : kMinimum)
        scalar(config.*member, name, 0.);
    if (config.front_load_share_target > 1.)
        fail("front load share target must not exceed one");
    if (std::max(config.max_forward_lean_rad,
                 config.max_backward_lean_rad) > .8)
        fail("seated lean exceeds the posture envelope");
}

// ---- rider.py / rider_posture.py / rider_envelope.py / joint_strength.py ---
void post_init(RiderSpecs &specs) {
    if (!in_set(specs.variant,
                std::initializer_list<std::string_view>{
                    "none", "lumped", "seated", "articulated_planar"}))
        fail("rider variant must be one of ('none', 'lumped', 'seated', "
             "'articulated_planar'), got " +
             detail::py_repr_str(specs.variant));
    if (specs.legs != "rigid" && specs.legs != "articulated")
        fail("legs must be 'rigid' or 'articulated', got " +
             detail::py_repr_str(specs.legs));
    if (specs.mass_kg <= 0. && specs.variant != "none")
        fail("rider mass must be positive, got " +
             repr_double(specs.mass_kg));
    if (specs.height_m <= 0.)
        fail("rider height must be positive, got " +
             repr_double(specs.height_m));
    if (specs.inseam_m &&
        !(0. < *specs.inseam_m && *specs.inseam_m < specs.height_m))
        fail("inseam " + repr_double(*specs.inseam_m) +
             " m must lie between 0 and the height " +
             repr_double(specs.height_m) + " m");
    const double shares =
        specs.saddle_share + specs.pedal_share + specs.bar_share;
    if (std::abs(shares - 1.) > 1e-9)
        fail("saddle, pedal and bar shares must sum to 1, got " +
             format_fixed6(shares));
    if (specs.helmet_mass_kg < 0. || specs.helmet_mass_kg >= specs.mass_kg)
        fail("helmet mass " + repr_double(specs.helmet_mass_kg) +
             " kg must be non-negative and below the rider mass");
}

void post_init(RiderPosture &posture) {
    for (const auto& [name, limit, member] :
         {std::tuple{"torso_lean_rad", .8,
                     &RiderPosture::torso_lean_rad},
          {"pelvis_pitch_rad", .5, &RiderPosture::pelvis_pitch_rad}}) {
        const double value = scalar(posture.*member, name);
        if (std::abs(value) > limit)
            fail(std::string(name) + " must lie in [-" +
                 repr_double(limit) + ", " + repr_double(limit) + "]");
    }
    if (posture.pelvis_offset_m) {
        const auto &offset = *posture.pelvis_offset_m;
        for (const double v : offset)
            if (!std::isfinite(v))
                fail("invalid shape or non-finite hip offset");
        if (std::abs(offset[0]) > .25 || !(-.15 <= offset[1] && offset[1] <= .25))
            fail("hip offset exceeds the posture command envelope");
    }
}

void post_init(const JointEnvelope &envelope) {
    if (!std::isfinite(envelope.neutral_anatomical_rad) ||
        !std::isfinite(envelope.minimum_anatomical_rad) ||
        !std::isfinite(envelope.maximum_anatomical_rad) ||
        (envelope.direction != -1 && envelope.direction != 1))
        fail("invalid joint coordinate convention");
    if (!(envelope.minimum_anatomical_rad < envelope.maximum_anatomical_rad))
        fail("empty joint range");
    if (!nonblank(envelope.provenance))
        fail("joint range provenance is required");
}

void post_init(TorqueCurve &curve) {
    if (curve.angles_rad.size() < 2 ||
        curve.angles_rad.size() != curve.torques_nm.size())
        fail("strength curve requires paired angle/torque knots");
    const bool finite =
        std::ranges::all_of(curve.angles_rad, [](double v) {
            return std::isfinite(v);
        }) &&
        std::ranges::all_of(curve.torques_nm, [](double v) {
            return std::isfinite(v);
        }) &&
        std::isfinite(curve.vmax_rad_s) && std::isfinite(curve.hill_c) &&
        std::isfinite(curve.eccentric_ratio);
    const bool negative = std::ranges::any_of(curve.torques_nm, [](double v) {
        return v < 0.;
    });
    if (!finite || negative) fail("invalid strength data");
    for (std::size_t i = 1; i < curve.angles_rad.size(); ++i)
        if (curve.angles_rad[i] <= curve.angles_rad[i - 1])
            fail("angles must increase");
    if (curve.vmax_rad_s <= 0. || curve.hill_c <= 0. ||
        curve.eccentric_ratio < 1.)
        fail("invalid force-velocity parameters");
    if (!nonblank(curve.source)) fail("missing strength provenance");
}

// ---- suspension spec records ------------------------------------------------
void post_init(const AirSpringSpecs &specs) { domain::air_specs(specs); }
void post_init(const CoilShockSpecs &specs) { domain::coil_specs(specs); }

} // namespace detail

// ===========================================================================
// physical_config.py / member helpers
// ===========================================================================
double EndStopConfig::stiffness_n_m() const {
    const double stiffness = reference_force_n / reference_deflection_m;
    return derived(stiffness, "EndStopConfig.stiffness_n_m");
}

TyreSpecs TyreSpecs::with_pressure(double pressure_bar_) const {
    TyreSpecs copy = *this;
    copy.pressure_bar = static_cast<double>(pressure_bar_);
    return copy;
}

const TierSpec *find_tire_tier(std::string_view name) {
    for (const TierSpec &tier : kTireTiers)
        if (tier.name == name) return &tier;
    return nullptr;
}

const MotorProfile *find_motor_profile(std::string_view name) {
    if (name == "bosch_cx_gen4") {
        // The registry entry is a validated record, like MotorProfile(...) at
        // import — run its post_init once per lookup (idempotent).
        detail::post_init(kBoschCxGen4);
        return &kBoschCxGen4;
    }
    return nullptr;
}

double AssistConfig::effective_max_torque() const {
    if (!profile) return max_torque;
    const MotorProfile *resolved = find_motor_profile(*profile);
    if (resolved == nullptr)
        throw std::out_of_range("PROFILES: " + *profile);
    return resolved->peak_torque_nm;
}

DistributedWheelMaterial
DistributedTireConfig::for_wheel(std::string_view side) const {
    if (side != "front" && side != "rear")
        fail("wheel side must be front or rear");
    const auto &selected =
        (side == "front") ? front_material : rear_material;
    if (selected) {
        DistributedWheelMaterial result = *selected;
        detail::post_init(result);
        return result;
    }
    DistributedWheelMaterial result{
        .density = density,
        .fitting_dataset_id = fitting_dataset_id,
        .valid_load_range_n = valid_load_range_n,
        .valid_deflection_range_m = valid_deflection_range_m,
        .calibration_status = "experimental_unvalidated"};
    detail::post_init(result);
    return result;
}

double DrivetrainSpecs::front_radius_m() const {
    return derived(chain_pitch_m * front_teeth / (2 * std::numbers::pi_v<double>),
                   "DrivetrainSpecs.front_radius_m");
}
double DrivetrainSpecs::rear_radius_m() const {
    return derived(chain_pitch_m * rear_teeth / (2 * std::numbers::pi_v<double>),
                   "DrivetrainSpecs.rear_radius_m");
}

// ===========================================================================
// model_config.py SimulationPhysicsConfig
// ===========================================================================
void post_init(SimulationPhysicsConfig &config) {
    // Member construction precedes __post_init__ in the Python dataclass —
    // the six nested records validate first, in declaration order.
    detail::post_init(config.end_stops);
    detail::post_init(config.tires);
    detail::post_init(config.drive);
    detail::post_init(config.resistance);
    detail::post_init(config.articulated);
    detail::post_init(config.seated_climb);

    if (config.physics_mode != "legacy" && config.physics_mode != "physical")
        fail("unknown physics_mode");
    if (!std::isfinite(config.timestep_s) || config.timestep_s <= 0.)
        fail("timestep_s must be finite and positive");
    scalar(config.control_period_s, "control period", std::nullopt, true);
    // Python round() is round-half-even — rint honours the default FP mode.
    const double steps =
        std::rint(config.control_period_s / config.timestep_s);
    if (steps < 1. ||
        std::abs(steps * config.timestep_s - config.control_period_s) > 1e-9)
        fail("control_period_s must be an integer multiple of timestep_s");

    scalar(config.equilibrium_refine_after_s,
           "equilibrium_refine_after_s", std::nullopt, true);
    scalar(config.equilibrium_refine_period_s,
           "equilibrium_refine_period_s", std::nullopt, true);
    const bool legacy = config.physics_mode == "legacy";
    scalar(config.closure_time_constant_s, "closure time constant",
           std::nullopt, true);
    if (!legacy &&
        config.closure_time_constant_s < 2.0 * config.timestep_s)
        fail("closure_time_constant_s=" +
             detail::repr_double(config.closure_time_constant_s) +
             " must be at least 2*timestep_s=" +
             detail::repr_double(2.0 * config.timestep_s) +
             "; otherwise MuJoCo silently changes closure stiffness");
    if (!config.drive_mode)
        config.drive_mode = legacy ? "ideal_speed_control" : "coast";
    if (!config.pitch_assist) config.pitch_assist = legacy;
    if (!in_set(*config.drive_mode,
                std::initializer_list<std::string_view>{
                    "coast", "ideal_speed_control", "crank_effort",
                    "articulated_effort"}))
        fail("unknown drive_mode");
    if (legacy &&
        (*config.drive_mode == "crank_effort" ||
         *config.drive_mode == "articulated_effort"))
        fail("physical effort drive modes require physics_mode=physical");
    if (!legacy && *config.pitch_assist)
        fail("external pitch assist is forbidden in physical mode");

    scalar(config.initial_speed_mps, "initial speed");
    for (const auto& [name, member] :
         {std::pair{"initial_front_brake",
                    &SimulationPhysicsConfig::initial_front_brake},
          {"initial_rear_brake",
           &SimulationPhysicsConfig::initial_rear_brake}}) {
        const double value = scalar(config.*member, name, 0.);
        if (value > 1.)
            fail(std::string(name) + " must lie in [0,1]");
    }
    scalar(config.timestep_s, "timestep", std::nullopt, true);
    // isinstance(pitch_assist, bool) / isinstance(cache_enabled, bool) are
    // typed members — the checks are vacuous.
    if (config.equilibrium_state_file &&
        !nonblank(*config.equilibrium_state_file))
        fail("equilibrium_state_file must be a non-empty path or None");
    // The six isinstance(...immutable...) checks are type-enforced.
    const auto *detailed = find_tire_tier("detailed");
    if (config.tires.backend == "pneumatic" &&
        config.tires.pneumatic.tier == "detailed" &&
        config.timestep_s != detailed->timestep_s)
        fail("detailed pneumatic tyre requires timestep_s=" +
             detail::repr_double(detailed->timestep_s) +
             "; got " + detail::repr_double(config.timestep_s));
    if (config.seated_climb.enabled &&
        (legacy || *config.drive_mode != "articulated_effort"))
        fail("seated climb requires the articulated physical effort drive");
    if (config.drive.motor_clutch &&
        *config.drive_mode != "crank_effort" &&
        *config.drive_mode != "articulated_effort")
        fail("drive.motor_clutch requires an effort drive mode");
    if (config.drive.rotor_inertia_kgm2 > 0. &&
        *config.drive_mode != "crank_effort" &&
        *config.drive_mode != "articulated_effort")
        fail("drive.rotor_inertia_kgm2 requires an effort drive mode");
}

SimulationPhysicsConfig default_physics() {
    SimulationPhysicsConfig config{};
    post_init(config);
    return config;
}

void validate(const SimulationPhysicsConfig &config) {
    SimulationPhysicsConfig copy = config;
    post_init(copy);
}

// ===========================================================================
// domain_validation.py — the six free validators
// ===========================================================================
namespace domain {

void air_specs(const AirSpringSpecs &specs) {
    for (const auto& [name, member] :
         {std::pair{"AirSpringSpecs.stanchion_inner_diam_mm",
                    &AirSpringSpecs::stanchion_inner_diam_mm},
          {"AirSpringSpecs.total_travel_mm",
           &AirSpringSpecs::total_travel_mm},
          {"AirSpringSpecs.pos_chamber_length_mm",
           &AirSpringSpecs::pos_chamber_length_mm},
          {"AirSpringSpecs.neg_chamber_length_mm",
           &AirSpringSpecs::neg_chamber_length_mm},
          {"AirSpringSpecs.gamma", &AirSpringSpecs::gamma},
          {"AirSpringSpecs.atm_pressure_pa",
           &AirSpringSpecs::atm_pressure_pa}})
        scalar(specs.*member, name, std::nullopt, true);
    scalar(specs.token_volume_cm3, "AirSpringSpecs.token_volume_cm3", 0.);
    integer(specs.max_tokens, "AirSpringSpecs.max_tokens", 0);
    integer(specs.default_tokens, "AirSpringSpecs.default_tokens");
}

void damper_core(const nativecfg::DamperCore &core, double total_travel_mm,
                 std::string_view name) {
    const std::string label(name);
    for (const auto& [field, value] :
         {std::pair{"max_hsc", core.max_hsc}, {"max_lsc", core.max_lsc},
          {"max_reb", core.max_reb}})
        integer(value, label + "." + field, std::nullopt, true);
    for (const auto& [field, value] :
         {std::pair{"hsc_clicks", core.hsc_clicks},
          {"lsc_clicks", core.lsc_clicks},
          {"rebound_clicks", core.rebound_clicks}})
        integer(value, label + "." + field);
    for (const auto& [band, lo, hi] :
         {std::tuple{"lsc", core.c_lsc_min, core.c_lsc_max},
          {"hsc", core.c_hsc_min, core.c_hsc_max},
          {"reb", core.c_reb_min, core.c_reb_max}}) {
        const double low =
            scalar(lo, label + ".c_" + band + "_min", 0.);
        const double high =
            scalar(hi, label + ".c_" + band + "_max", 0.);
        if (low > high)
            fail(label + ".c_" + band +
                 "_min: damping bounds must be ordered");
    }
    scalar(core.v_knee_comp, label + ".v_knee_comp", std::nullopt, true);
    scalar(core.v_knee_reb, label + ".v_knee_reb", std::nullopt, true);
    scalar(total_travel_mm, label + ".total_travel_mm", std::nullopt, true);
}

void damper(const nativecfg::Charger3Config &damper_,
            std::string_view name) {
    const std::string label(name);
    damper_core(damper_.core, damper_.total_travel_mm, label);
    // legacy_behavior is a typed bool.
    const double start =
        scalar(damper_.hbo_start_mm, label + ".hbo_start_mm", 0.);
    scalar(damper_.total_travel_mm, label + ".travel", std::nullopt, true);
    if (!damper_.legacy_behavior && !(start < damper_.total_travel_mm))
        fail(label + ".hbo_start_mm: modern HBO must start within travel");
    scalar(damper_.c_hbo_base, label + ".c_hbo_base", 0.);
}

void damper(const nativecfg::SuperDeluxeConfig &damper_,
            std::string_view name) {
    const std::string label(name);
    damper_core(damper_.core, damper_.total_stroke_mm, label);
    const double start =
        scalar(damper_.hbo_start_mm, label + ".hbo_start_mm", 0.);
    scalar(damper_.total_stroke_mm, label + ".travel", std::nullopt, true);
    if (!damper_.legacy_behavior && !(start < damper_.total_stroke_mm))
        fail(label + ".hbo_start_mm: modern HBO must start within travel");
    integer(damper_.max_hbo, label + ".max_hbo", std::nullopt, true);
    integer(damper_.hbo_clicks, label + ".hbo_clicks");
    for (const auto& [field, value] :
         {std::pair{"c_hbo_min", damper_.c_hbo_min},
          {"c_hbo_max", damper_.c_hbo_max},
          {"lockout_preload_n", damper_.lockout_preload_n},
          {"lockout_stiffness", damper_.lockout_stiffness}})
        scalar(value, label + "." + field, 0.);
    if (damper_.c_hbo_min > damper_.c_hbo_max)
        fail(label + ".c_hbo_min: HBO bounds must be ordered");
}

void coil_specs(const CoilShockSpecs &specs) {
    for (const auto& [name, member] :
         {std::pair{"CoilShockSpecs.rate_n_m", &CoilShockSpecs::rate_n_m},
          {"CoilShockSpecs.stroke_mm", &CoilShockSpecs::stroke_mm},
          {"CoilShockSpecs.bumper_length_mm",
           &CoilShockSpecs::bumper_length_mm},
          {"CoilShockSpecs.bumper_peak_n",
           &CoilShockSpecs::bumper_peak_n}})
        scalar(specs.*member, name, std::nullopt, true);
    scalar(specs.preload_mm, "CoilShockSpecs.preload_mm", 0.);
    if (specs.bumper_length_mm > specs.stroke_mm)
        fail("CoilShockSpecs.bumper_length_mm: exceeds stroke");
}

void pedaling_config(const PedalingConfig &config) {
    for (const auto& [name, member] :
         {std::pair{"PedalingConfig.coast_above_rpm",
                    &PedalingConfig::coast_above_rpm},
          {"PedalingConfig.stop_time_s", &PedalingConfig::stop_time_s}})
        scalar(config.*member, name, std::nullopt, true);
    for (const auto& [name, member] :
         {std::pair{"PedalingConfig.resume_below_rpm",
                    &PedalingConfig::resume_below_rpm},
          {"PedalingConfig.coast_cadence_tau_s",
           &PedalingConfig::coast_cadence_tau_s},
          {"PedalingConfig.mash_cadence_rpm",
           &PedalingConfig::mash_cadence_rpm},
          {"PedalingConfig.mash_torque_nm",
           &PedalingConfig::mash_torque_nm},
          {"PedalingConfig.effort_slew_nm_s",
           &PedalingConfig::effort_slew_nm_s}})
        scalar(config.*member, name, 0.);
    if (config.resume_below_rpm >= config.coast_above_rpm)
        fail("PedalingConfig.resume_below_rpm: invalid cadence band");
    if (config.mash_torque_nm > 0. && config.mash_cadence_rpm <= 0.)
        fail("PedalingConfig.mash_cadence_rpm: nonzero mash torque requires "
             "positive cadence");
}

void shifting_config(const ShiftingConfig &config,
                     const DrivetrainSpecs &gearing) {
    integer(gearing.front_teeth, "DrivetrainSpecs.front_teeth", 3);
    integer(gearing.rear_teeth, "DrivetrainSpecs.rear_teeth", 3);
    scalar(gearing.chain_pitch_m, "DrivetrainSpecs.chain_pitch_m",
           std::nullopt, true);
    for (const int tooth : config.cassette)
        integer(tooth, "ShiftingConfig.cassette", 3);
    if (config.cassette.empty()) {
        fail("ShiftingConfig.cassette: nonempty unique teeth required");
    }
    const std::set<int> distinct(config.cassette.begin(),
                                 config.cassette.end());
    if (distinct.size() != config.cassette.size())
        fail("ShiftingConfig.cassette: nonempty unique teeth required");
    for (const auto& [name, member] :
         {std::pair{"ShiftingConfig.target_cadence_min_rpm",
                    &ShiftingConfig::target_cadence_min_rpm},
          {"ShiftingConfig.target_cadence_max_rpm",
           &ShiftingConfig::target_cadence_max_rpm}})
        scalar(config.*member, name, std::nullopt, true);
    for (const auto& [name, member] :
         {std::pair{"ShiftingConfig.shift_cooldown_s",
                    &ShiftingConfig::shift_cooldown_s},
          {"ShiftingConfig.shift_cut_duration_s",
           &ShiftingConfig::shift_cut_duration_s},
          {"ShiftingConfig.torque_factor", &ShiftingConfig::torque_factor},
          {"ShiftingConfig.cadence_smoothing_tau_s",
           &ShiftingConfig::cadence_smoothing_tau_s},
          {"ShiftingConfig.upshift_slip_limit_mps",
           &ShiftingConfig::upshift_slip_limit_mps}})
        scalar(config.*member, name, 0.);
    if (config.target_cadence_max_rpm <= config.target_cadence_min_rpm ||
        config.shift_cut_duration_s > config.shift_cooldown_s ||
        config.torque_factor > 1.)
        fail("ShiftingConfig: invalid timing/cadence/torque band");
    if (config.upshift_slip_mode != "legacy_signed" &&
        config.upshift_slip_mode != "magnitude")
        fail("ShiftingConfig.upshift_slip_mode: unsupported mode");
    if (config.enabled &&
        std::ranges::find(config.cassette, gearing.rear_teeth) ==
            config.cassette.end())
        fail("ShiftingConfig.cassette: enabled rear gear must belong to "
             "cassette");
}

} // namespace domain

// ===========================================================================
// suspension_config.py / damper.py ctor domains
// ===========================================================================
std::string_view damper_preset_label(DamperPreset preset) {
    switch (preset) {
    case DamperPreset::kBase: return "Base Tune (Factory Neutral)";
    case DamperPreset::kPlush: return "Plush / Wet Traction";
    case DamperPreset::kEnduro: return "Enduro Race (Supportive)";
    case DamperPreset::kPark: return "Bike Park / Jumps (Firm)";
    case DamperPreset::kCustom: return "Custom Rider Tune";
    }
    return "";
}

bool damper_preset_registered(DamperPreset preset) {
    return preset != DamperPreset::kCustom;
}

const DamperClickConfig &damper_preset_row(DamperPreset preset) {
    static const DamperClickConfig kBase{};
    static const DamperClickConfig kPlush{
        .fork_hsc = 0, .fork_lsc = 3, .fork_rebound = 5,
        .shock_hsc = 0, .shock_lsc = 3, .shock_rebound = 4,
        .shock_hbo = 1, .shock_lockout = false,
        .preset_name = "Plush / Wet Traction"};
    static const DamperClickConfig kEnduro{
        .fork_hsc = 3, .fork_lsc = 10, .fork_rebound = 11,
        .shock_hsc = 3, .shock_lsc = 10, .shock_rebound = 9,
        .shock_hbo = 3, .shock_lockout = false,
        .preset_name = "Enduro Race (Supportive)"};
    static const DamperClickConfig kPark{
        .fork_hsc = 4, .fork_lsc = 13, .fork_rebound = 14,
        .shock_hsc = 4, .shock_lsc = 13, .shock_rebound = 12,
        .shock_hbo = 4, .shock_lockout = false,
        .preset_name = "Bike Park / Jumps (Firm)"};
    static const DamperClickConfig kFallback{};
    switch (preset) {
    case DamperPreset::kBase: return kBase;
    case DamperPreset::kPlush: return kPlush;
    case DamperPreset::kEnduro: return kEnduro;
    case DamperPreset::kPark: return kPark;
    case DamperPreset::kCustom: return kFallback;  // never registered
    }
    return kFallback;
}

double AirSpringSpecs::piston_area_m2() const {
    const double r_m = (stanchion_inner_diam_mm / 2.0) / 1000.0;
    return derived(std::numbers::pi_v<double> * (r_m * r_m),
                   "AirSpringSpecs.piston_area_m2");
}
double AirSpringSpecs::token_volume_m3() const {
    return derived(token_volume_cm3 * 1e-6, "AirSpringSpecs.token_volume_m3");
}
double AirSpringSpecs::base_pos_volume_m3() const {
    const double l_pos_m = pos_chamber_length_mm / 1000.0;
    return derived(piston_area_m2() * l_pos_m,
                   "AirSpringSpecs.base_pos_volume_m3");
}
double AirSpringSpecs::base_neg_volume_m3() const {
    const double l_neg_m = neg_chamber_length_mm / 1000.0;
    return derived(piston_area_m2() * l_neg_m,
                   "AirSpringSpecs.base_neg_volume_m3");
}

namespace {

// BaseDamper.__init__ shared body: bound tables + the click clamps; the
// constants are the subclass literals. Out-parameter construction: the
// resolved records exceed the -Wlarge-by-value-copy return threshold.
// NOLINTBEGIN(bugprone-easily-swappable-parameters) positional args mirror BaseDamper.__init__'s keyword order
void damper_core_ctor(nativecfg::DamperCore &core, int max_hsc, int max_lsc,
                      int max_reb, int hsc_clicks, int lsc_clicks,
                      int rebound_clicks, double total_travel_mm,
                      double c_lsc_min, double c_lsc_max, double c_hsc_min,
                      double c_hsc_max, double c_reb_min, double c_reb_max,
                      double v_knee_comp, double v_knee_reb,
                      std::string_view subclass) {
    // NOLINTEND(bugprone-easily-swappable-parameters)
    core.max_hsc = static_cast<int>(
        integer(max_hsc, "BaseDamper.max_hsc", std::nullopt, true));
    core.max_lsc = static_cast<int>(
        integer(max_lsc, "BaseDamper.max_lsc", std::nullopt, true));
    core.max_reb = static_cast<int>(
        integer(max_reb, "BaseDamper.max_reb", std::nullopt, true));
    core.hsc_clicks = 0;
    core.lsc_clicks = 0;
    core.rebound_clicks = 0;
    core.c_lsc_min = c_lsc_min;
    core.c_lsc_max = c_lsc_max;
    core.c_hsc_min = c_hsc_min;
    core.c_hsc_max = c_hsc_max;
    core.c_reb_min = c_reb_min;
    core.c_reb_max = c_reb_max;
    core.v_knee_comp = v_knee_comp;
    core.v_knee_reb = v_knee_reb;
    scalar(total_travel_mm, "BaseDamper.total_travel_mm", std::nullopt,
           true);
    const auto clamp = [](long long value, long long ceiling) {
        return static_cast<int>(
            std::max(0LL, std::min(ceiling, value)));
    };
    core.hsc_clicks = clamp(integer(hsc_clicks, "BaseDamper.hsc_clicks"),
                            core.max_hsc);
    core.lsc_clicks = clamp(integer(lsc_clicks, "BaseDamper.lsc_clicks"),
                            core.max_lsc);
    core.rebound_clicks =
        clamp(integer(rebound_clicks, "BaseDamper.rebound_clicks"),
              core.max_reb);
    // domain_validation.damper_core labels with
    // type(damper).__name__ — the subclass, not BaseDamper.
    domain::damper_core(core, total_travel_mm, subclass);
}

// Charger3Damper.__init__ domain.
void charger3_ctor(nativecfg::Charger3Config &damper, int hsc_clicks,
                   int lsc_clicks, int rebound_clicks, double total_travel_mm,
                   bool legacy_behavior) {
    scalar(total_travel_mm, "Charger3Damper.total_travel_mm", std::nullopt,
           true);
    if (!std::isfinite(total_travel_mm) || total_travel_mm <= 0.)
        fail("fork travel must be finite and positive");
    if (!legacy_behavior && total_travel_mm < 20.)
        fail("fork travel must contain the 20 mm HBO zone");
    damper_core_ctor(damper.core, 4, 14, 17, hsc_clicks, lsc_clicks,
                     rebound_clicks, total_travel_mm, 260., 1150., 180.,
                     520., 350., 1550., .18, .22, "Charger3Damper");
    damper.total_travel_mm = total_travel_mm;
    damper.legacy_behavior = legacy_behavior;
    damper.hbo_start_mm = legacy_behavior ? 160. : total_travel_mm - 20.;
    if (!legacy_behavior &&
        !(0. < total_travel_mm - damper.hbo_start_mm &&
          total_travel_mm - damper.hbo_start_mm <= total_travel_mm))
        fail("fork HBO zone must be positive and no longer than travel");
    damper.c_hbo_base = 3500.;
}

// SuperDeluxeDamper.__init__ domain.
// NOLINTBEGIN(bugprone-easily-swappable-parameters) positional args mirror SuperDeluxeDamper.__init__'s keyword order
void super_deluxe_ctor(nativecfg::SuperDeluxeConfig &damper, int hsc_clicks,
                       int lsc_clicks, int rebound_clicks, int hbo_clicks,
                       bool lockout_firm, double total_stroke_mm,
                       bool legacy_behavior) {
    // NOLINTEND(bugprone-easily-swappable-parameters)
    scalar(total_stroke_mm, "SuperDeluxeDamper.total_stroke_mm",
           std::nullopt, true);
    if (!std::isfinite(total_stroke_mm) || total_stroke_mm <= 0.)
        fail("shock stroke must be finite and positive");
    damper_core_ctor(damper.core, 4, 14, 14, hsc_clicks, lsc_clicks,
                     rebound_clicks, total_stroke_mm, 650., 2900., 450.,
                     1450., 900., 8000., .16, .20, "SuperDeluxeDamper");
    damper.max_hbo = 4;
    damper.total_stroke_mm = total_stroke_mm;
    damper.hbo_clicks = static_cast<int>(std::max(
        0LL, std::min(4LL,
                      integer(hbo_clicks, "BaseDamper.hbo_clicks"))));
    damper.lockout_firm = lockout_firm;
    damper.legacy_behavior = legacy_behavior;
    damper.hbo_start_mm = legacy_behavior ? 52. : .8 * total_stroke_mm;
    if (!legacy_behavior &&
        !(0. < total_stroke_mm - damper.hbo_start_mm &&
          total_stroke_mm - damper.hbo_start_mm <= total_stroke_mm))
        fail("shock HBO zone must be positive and no longer than stroke");
    damper.c_hbo_min = 4000.;
    damper.c_hbo_max = 18000.;
    damper.lockout_preload_n = 480.;
    damper.lockout_stiffness = 15000.;
}

} // namespace

SuspensionSpec resolve_suspension(const SuspensionInputs &inputs,
                                  double preload_mm) {
    // build_suspension_components: the BikeSpecs field mapping plus every
    // constructor-domain check the Python chain runs. SuspensionController's
    // __init__ is pure storage.
    SuspensionSpec spec;
    spec.clicks = DamperClickConfig{
        .fork_hsc = inputs.fork_hsc, .fork_lsc = inputs.fork_lsc,
        .fork_rebound = inputs.fork_rebound, .shock_hsc = inputs.shock_hsc,
        .shock_lsc = inputs.shock_lsc,
        .shock_rebound = inputs.shock_rebound,
        .shock_hbo = inputs.shock_hbo,
        .shock_lockout = inputs.shock_lockout,
        .preset_name = "Base Tune (Factory Neutral)"};
    charger3_ctor(spec.fork_damper, spec.clicks.fork_hsc,
                  spec.clicks.fork_lsc, spec.clicks.fork_rebound,
                  inputs.fork_travel, false);
    super_deluxe_ctor(spec.shock_damper, spec.clicks.shock_hsc,
                      spec.clicks.shock_lsc, spec.clicks.shock_rebound,
                      spec.clicks.shock_hbo, spec.clicks.shock_lockout,
                      inputs.shock_stroke, false);
    AirSpringSpecs air;
    air.total_travel_mm = inputs.fork_travel;
    domain::air_specs(air);
    spec.air = air;
    spec.num_tokens = static_cast<int>(std::max(
        0LL,
        std::min(static_cast<long long>(air.max_tokens),
                 integer(inputs.fork_air_tokens, "ForkAirSpring.num_tokens"))));
    spec.gauge_pressure_psi = scalar(inputs.fork_initial_psi,
                                     "ForkAirSpring.gauge_pressure_psi", 0.);
    const CoilShockSpecs coil{
        .rate_n_m = inputs.shock_stiffness, .preload_mm = preload_mm,
        .stroke_mm = inputs.shock_stroke, .bumper_length_mm = 10.,
        .bumper_peak_n = 7000.};
    domain::coil_specs(coil);
    // CoilShock.__init__'s second gate (post-coil_specs domain).
    const bool coil_valid =
        std::isfinite(coil.rate_n_m) && std::isfinite(coil.preload_mm) &&
        std::isfinite(coil.stroke_mm) &&
        std::isfinite(coil.bumper_length_mm) &&
        std::isfinite(coil.bumper_peak_n) && coil.preload_mm >= 0. &&
        coil.rate_n_m > 0. && coil.stroke_mm > 0. &&
        0. < coil.bumper_length_mm && coil.bumper_length_mm <= coil.stroke_mm &&
        coil.bumper_peak_n > 0.;
    if (!coil_valid) fail("invalid coil shock specifications");
    spec.coil = coil;
    return spec;
}

// ===========================================================================
// rider.py / rider_posture.py / rider_program.py helpers
// ===========================================================================
double de_leva_segment_length_fraction(std::string_view name) {
    static constexpr std::array<std::pair<std::string_view, double>, 7>
        kFractions{
            std::pair{"trunk", 0.5155 / kDeLevaReferenceStatureM},
            std::pair{"upper_arm", 0.2817 / kDeLevaReferenceStatureM},
            std::pair{"forearm", 0.2689 / kDeLevaReferenceStatureM},
            std::pair{"hand", 0.0862 / kDeLevaReferenceStatureM},
            std::pair{"thigh", 0.4222 / kDeLevaReferenceStatureM},
            // NOLINTNEXTLINE(modernize-use-std-numbers) 0.4340 is the de Leva table literal in rider.py, not log10(e)
            std::pair{"shank", 0.4340 / kDeLevaReferenceStatureM},
            std::pair{"head", 0.2033 / kDeLevaReferenceStatureM},
        };
    for (const auto &[key, fraction] : kFractions)
        if (key == name) return fraction;
    throw std::out_of_range(std::string(name));  // dict KeyError equivalent
}

double RiderSpecs::segment_length(std::string_view name) const {
    return de_leva_segment_length_fraction(name) * height_m;
}

RiderPosture standing_posture() {
    RiderPosture posture;
    posture.torso_lean_rad = .08;
    posture.pelvis_pitch_rad = 0.;
    posture.pelvis_offset_m = spindle::Vec2{0., .16};
    posture.use_saddle = false;
    detail::post_init(posture);  // cls(...) runs __post_init__ at construction
    return posture;
}

PedalIntent pedal_intent(double phase_rad, double cadence_rad_s,
                         double ankle_amplitude_rad,
                         double scrape_fraction) {
    if (!std::isfinite(phase_rad) || !std::isfinite(cadence_rad_s) ||
        !std::isfinite(ankle_amplitude_rad) ||
        !std::isfinite(scrape_fraction))
        fail("nonfinite pedaling intent");
    if (cadence_rad_s < 0. || ankle_amplitude_rad < 0. ||
        !(0. <= scrape_fraction && scrape_fraction <= 1.))
        fail("invalid pedaling intent");
    return PedalIntent{.cadence_rad_s = cadence_rad_s,
                       .ankle_offset_rad =
                           ankle_amplitude_rad * std::sin(phase_rad),
                       .scrape_fraction = scrape_fraction};
}

std::vector<std::string> reference_rom_check(
    const std::vector<std::pair<std::string, double>> &joint_angles_rad,
    const JointEnvelopeMap &envelopes) {
    // set(angles) == set(envelopes) name check.
    std::set<std::string_view> angle_names, envelope_names;
    for (const auto &[name, q] : joint_angles_rad) angle_names.insert(name);
    for (const auto &[name, env] : envelopes) envelope_names.insert(name);
    if (angle_names.size() != joint_angles_rad.size() ||
        envelope_names.size() != envelopes.size() ||
        angle_names != envelope_names)
        fail("ROM check needs one angle per declared joint");
    std::vector<std::string> violations;
    for (const auto &[name, envelope] : envelopes) {
        double q = 0.;
        for (const auto &[key, value] : joint_angles_rad)
            if (key == name) q = value;
        if (!std::isfinite(q)) fail("ROM check needs finite joint angles");
        const double angle = anatomical_angle(envelope, q);
        if (!(envelope.minimum_anatomical_rad - 1e-12 <= angle &&
              angle <= envelope.maximum_anatomical_rad + 1e-12))
            violations.push_back(name);
    }
    return violations;
}

// ===========================================================================
// rider_envelope.py / joint_strength.py JSON profile loaders
// ===========================================================================
namespace {

// JSON member decode for JointEnvelope(**payload): member TYPE failures map
// the way Python's construction sees them. Angle members land in isfinite()
// — a non-numeric JSON member raises TypeError there, which the outer
// comprehension converts to 'invalid joint envelope fields'. `direction`
// fails 'invalid joint coordinate convention' whenever it is not an int in
// Python terms — a bool (type(x) is not int) or a non-integral number.
// `provenance` stays a raw str slot: a non-string member fails the
// 'joint range provenance is required' check.
JointEnvelope envelope_from_object(const bike::io::Value &entry) {
    using bike::io::Value;
    const auto *object = std::get_if<Value::Object>(&entry.data);
    static const std::set<std::string_view> kKeys{
        "direction", "maximum_anatomical_rad", "minimum_anatomical_rad",
        "neutral_anatomical_rad", "provenance"};
    {
        std::set<std::string_view> keys;
        if (object != nullptr)
            for (const auto &[key, value] : *object) keys.insert(key);
        if (object == nullptr || keys != kKeys)
            fail("invalid joint envelope fields");
    }
    JointEnvelope envelope{};
    envelope.direction = 0;  // invalid sentinel — see the comment above
    for (const auto &[key, member] : *object) {
        if (key == "direction") {
            // JointEnvelope(direction=v) assigns the raw decoded value —
            // `type(v) is not int` then fails 'invalid joint coordinate
            // convention'. Only an integer-spelled lexeme is an int;
            // bools ('type(True) is not int'), float spellings, and
            // every other kind leave the invalid sentinel.
            if (const auto *number =
                    std::get_if<io::Number>(&member.data)) {
                if (bike::io::is_integer_lexeme(number->lexeme)) {
                    long long parsed = 0;
                    const auto parsed_result = std::from_chars(
                        number->lexeme.data(),
                        std::to_address(number->lexeme.cend()), parsed);
                    if (parsed_result.ec == std::errc{} &&
                        parsed >= -2147483648LL &&
                        parsed <= 2147483647LL)
                        envelope.direction = static_cast<int>(parsed);
                    // An out-of-int32 magnitude stays invalid — the Python
                    // int is unbounded but `in (-1, 1)` rejects it anyway.
                }
            }
        } else if (key == "provenance") {
            if (const auto *text =
                    std::get_if<std::string>(&member.data))
                envelope.provenance = *text;
        } else {
            const auto *number =
                std::get_if<io::Number>(&member.data);
            const auto *flag = std::get_if<bool>(&member.data);
            if (number == nullptr && flag == nullptr)
                fail("invalid joint envelope fields");
            double *slot = nullptr;
            if (key == "neutral_anatomical_rad")
                slot = &envelope.neutral_anatomical_rad;
            else if (key == "minimum_anatomical_rad")
                slot = &envelope.minimum_anatomical_rad;
            else
                slot = &envelope.maximum_anatomical_rad;
            bool ok = false;
            *slot = flag != nullptr
                ? (*flag ? 1. : 0.)
                : bike::io::detail::float_from_lexeme(number->lexeme, ok);
        }
    }
    detail::post_init(envelope);
    return envelope;
}

JointEnvelopeMap envelope_map(const bike::io::Value &joints,
                              std::span<const std::string_view> names,
                              const std::set<std::string_view> &requested) {
    const auto *object =
        std::get_if<bike::io::Value::Object>(&joints.data);
    JointEnvelopeMap result;
    for (const std::string_view name : names) {
        if (!requested.contains(name)) continue;
        result.emplace_back(
            std::string(name),
            envelope_from_object(*bike::io::find_member(*object, name)));
    }
    return result;
}

// `payload.get('schema_version', default)` with Python == semantics: an
// integer or integral float/bool compares equal to 1/2; anything else —
// strings, lists, fractional numbers — is "neither".
int schema_version_value(const bike::io::Value::Object *object,
                         int missing_default) {
    using bike::io::Value;
    if (object == nullptr) return missing_default;
    const Value *member = bike::io::find_member(*object, "schema_version");
    if (member == nullptr) return missing_default;
    if (const auto *flag = std::get_if<bool>(&member->data))
        return *flag ? 1 : 0;
    if (const auto *number = std::get_if<io::Number>(&member->data)) {
        bool ok = false;
        const double value =
            bike::io::detail::float_from_lexeme(number->lexeme, ok);
        if (std::isfinite(value) && value == std::floor(value) &&
            std::abs(value) < 9007199254740992.)
            return static_cast<int>(value);
    }
    return -1;  // neither 1 nor 2 (nor any supported version)
}

} // namespace

JointEnvelopeMap
load_joint_envelopes(const std::filesystem::path &path,
                     std::optional<std::span<const std::string_view>>
                         present) {
    using bike::io::Value;
    Value payload = read_json(path);
    const auto *root = std::get_if<Value::Object>(&payload.data);
    if (root == nullptr) fail("joint profile must be an object");
    std::span<const std::string_view> names = kRiderJoints;
    const Value *joints = &payload;
    if (const Value *nested = bike::io::find_member(*root, "joints")) {
        const int version = schema_version_value(root, 1);
        std::set<std::string_view> keys;
        for (const auto &[key, value] : *root) keys.insert(key);
        if (version == 2) {
            if (keys != std::set<std::string_view>{"joints",
                                                   "schema_version",
                                                   "provenance",
                                                   "topology"})
                fail("unknown joint profile schema");
            if (detail::py_str(*bike::io::find_member(*root, "topology")) !=
                "articulated_planar_two_arm")
                fail("joint profile names a different rider topology");
        } else if (version == 1) {
            if (keys != std::set<std::string_view>{
                            "joints", "schema_version", "provenance"})
                fail("unknown joint profile schema");
            names = kLegacyRiderJoints;
        } else {
            fail("unknown joint profile schema");
        }
        joints = nested;
    }
    // `present is None` defaults to all topology joints; an engaged empty
    // span is Python's `set(())` — empty coverage, not the default.
    const std::set<std::string_view> requested =
        !present.has_value()
            ? std::set<std::string_view>(names.begin(), names.end())
            : std::set<std::string_view>(present->begin(), present->end());
    const auto *joint_map =
        std::get_if<Value::Object>(&joints->data);
    const auto subset_of = [](const std::set<std::string_view> &a,
                              const std::set<std::string_view> &b) {
        return std::ranges::all_of(
            a, [&](std::string_view key) { return b.contains(key); });
    };
    {
        std::set<std::string_view> payload_keys;
        if (joint_map != nullptr)
            for (const auto &[key, value] : *joint_map)
                payload_keys.insert(key);
        const std::set<std::string_view> names_set(names.begin(),
                                                   names.end());
        if (joint_map == nullptr ||
            !subset_of(requested, payload_keys) ||
            !subset_of(payload_keys, names_set) ||
            !subset_of(requested, names_set))
            fail("profile must cover present internal rider joints, never "
                 "root joints");
    }
    return envelope_map(*joints, names, requested);
}

std::pair<TorqueCurve, bool>
strength_curve_value(const bike::io::Value &payload, std::string_view joint,
                     std::string_view direction_label) {
    using bike::io::Value;
    const std::string prefix =
        std::string(joint) + ":" + std::string(direction_label);
    static const std::set<std::string_view> kKeys{
        "angles_rad", "torques_nm", "vmax_rad_s", "hill_c",
        "eccentric_ratio", "source", "verified", "label"};
    if (!std::holds_alternative<Value::Object>(payload.data))
        fail(prefix + " strength curve must be an object");
    const auto &object = std::get<Value::Object>(payload.data);
    {
        std::set<std::string_view> keys;
        for (const auto &[key, value] : object) keys.insert(key);
        if (keys != kKeys)
            fail(prefix +
                 " strength curve has unknown or missing fields");
    }
    const auto member = [&](std::string_view key) -> const Value & {
        return *bike::io::find_member(object, key);
    };
    const auto *label = std::get_if<std::string>(&member("label").data);
    if (label == nullptr || !nonblank(*label))
        fail(prefix + " anatomical direction label is required");
    if (!std::holds_alternative<bool>(member("verified").data))
        fail(prefix + " verified flag must be a boolean");
    const bool verified = std::get<bool>(member("verified").data);
    TorqueCurve curve;
    try {
        // `(float(v) for v in payload['angles_rad'])` — Python iterates the
        // member: arrays element-wise, strings per character, dicts per
        // key; a scalar member is not iterable. iterate_items reproduces
        // each arm, py_float each element.
        for (const Value &v : detail::iterate_items(member("angles_rad")))
            curve.angles_rad.push_back(detail::py_float(v));
        for (const Value &v : detail::iterate_items(member("torques_nm")))
            curve.torques_nm.push_back(detail::py_float(v));
        curve.vmax_rad_s = detail::py_float(member("vmax_rad_s"));
        curve.hill_c = detail::py_float(member("hill_c"));
        curve.eccentric_ratio = detail::py_float(member("eccentric_ratio"));
        curve.source = detail::py_str(member("source"));
        detail::post_init(curve);
    } catch (const std::invalid_argument &) {
        fail(prefix + " invalid strength curve");
    }
    return {curve, verified};
}

JointEnvelope strength_coordinate(const bike::io::Value &entry,
                                  std::string_view name) {
    using bike::io::Value;
    const std::string prefix(name);
    // load_strength_coordinate: coordinate = entry['coordinate'] — a
    // missing member propagates KeyError; a non-mapping entry raises the
    // subscript TypeError; the field-set check is the labeled ValueError.
    const auto *object = std::get_if<Value::Object>(&entry.data);
    if (object == nullptr)
        throw std::invalid_argument(
            "'" + std::string(detail::python_type_name(entry)) +
            "' object is not subscriptable");
    const Value *coordinate =
        bike::io::find_member(*object, "coordinate");
    if (coordinate == nullptr)
        throw std::out_of_range("coordinate");
    const auto *coord_object =
        std::get_if<Value::Object>(&coordinate->data);
    if (coord_object == nullptr || coord_object->size() != 2 ||
        bike::io::find_member(*coord_object, "neutral_anatomical_rad") ==
            nullptr ||
        bike::io::find_member(*coord_object, "direction") == nullptr)
        fail(prefix + ": invalid coordinate convention");
    // `low, high = entry['used_anatomical_range_rad']` — raw unpack; the
    // float() conversions happen inside the JointEnvelope(...) call below,
    // in its argument order (neutral, direction, min, max, provenance).
    const Value *range_member =
        bike::io::find_member(*object, "used_anatomical_range_rad");
    if (range_member == nullptr)
        throw std::out_of_range("used_anatomical_range_rad");
    const auto [low, high] = detail::unpack2(*range_member);
    JointEnvelope envelope;
    envelope.neutral_anatomical_rad = detail::py_float(
        *bike::io::find_member(*coord_object, "neutral_anatomical_rad"));
    envelope.direction =
        detail::py_int(*bike::io::find_member(*coord_object, "direction"));
    envelope.minimum_anatomical_rad = detail::py_float(low);
    envelope.maximum_anatomical_rad = detail::py_float(high);
    const Value *population =
        bike::io::find_member(*object, "population");
    if (population == nullptr) throw std::out_of_range("population");
    envelope.provenance = detail::py_str(*population);
    detail::post_init(envelope);
    return envelope;
}

StrengthProfile
load_strength_profile(const std::filesystem::path &path,
                      std::span<const std::string_view> expected_joints,
                      bool require_verified) {
    using bike::io::Value;
    Value payload = read_json(path);
    const auto *root = std::get_if<Value::Object>(&payload.data);
    if (root == nullptr || schema_version_value(root, -1) != 1)
        fail("unknown strength profile schema");
    const Value *topology = bike::io::find_member(*root, "topology");
    if (topology == nullptr ||
        detail::py_str(*topology) != "articulated_planar_two_arm")
        fail("strength profile names a different rider topology");
    {
        std::set<std::string_view> keys;
        for (const auto &[key, value] : *root) keys.insert(key);
        if (keys != std::set<std::string_view>{"schema_version", "topology",
                                               "provenance", "joints"})
            fail("strength profile has unknown or missing fields");
    }
    const Value &joints =
        *bike::io::find_member(*root, "joints");
    const auto *joint_map = std::get_if<Value::Object>(&joints.data);
    {
        std::set<std::string_view> present_keys;
        if (joint_map != nullptr)
            for (const auto &[key, value] : *joint_map)
                present_keys.insert(key);
        const auto covered = std::ranges::all_of(
            expected_joints,
            [&](std::string_view name) {
                return present_keys.contains(name);
            });
        if (joint_map == nullptr || !covered)
            fail("strength profile must cover the present internal rider "
                 "joints");
    }
    StrengthProfile result;
    for (const std::string_view name : expected_joints) {
        const Value &entry = *bike::io::find_member(*joint_map, name);
        const std::string label(name);
        if (!std::holds_alternative<Value::Object>(entry.data)) {
            fail(label + ": unknown or missing strength entry fields");
        }
        const auto &entry_object = std::get<Value::Object>(entry.data);
        {
            std::set<std::string_view> keys;
            for (const auto &[key, value] : entry_object)
                keys.insert(key);
            if (keys != std::set<std::string_view>{
                            "coordinate", "used_anatomical_range_rad",
                            "population", "scaling", "directions"})
                fail(label +
                     ": unknown or missing strength entry fields");
        }
        const JointEnvelope coordinate = strength_coordinate(entry, name);
        // `low, high = (float(v) for v in entry['used_anatomical_range_rad'])`
        // — the generator iterates the raw member; float() runs on each
        // element as it is pulled, and the unpack pulls at most three, so
        // conversion failures and cardinality errors interleave exactly as
        // in Python. The entry keys are already exact, so the member is
        // guaranteed present.
        const Value *range_member = bike::io::find_member(
            entry_object, "used_anatomical_range_rad");
        const auto items = detail::iterate_items(*range_member);
        if (items.empty())
            fail("not enough values to unpack (expected 2, got 0)");
        const double low = detail::py_float(items[0]);
        if (items.size() == 1)
            fail("not enough values to unpack (expected 2, got 1)");
        const double high = detail::py_float(items[1]);
        if (items.size() > 2) {
            // The third pull applies float() before the unpack detects the
            // extra item — a bad element raises first.
            static_cast<void>(detail::py_float(items[2]));
            fail("too many values to unpack (expected 2)");
        }
        if (!(std::isfinite(low) && std::isfinite(high) && low < high))
            fail(label + ": invalid used anatomical range");
        if (!(coordinate.minimum_anatomical_rad <=
                  coordinate.neutral_anatomical_rad &&
              coordinate.neutral_anatomical_rad <=
                  coordinate.maximum_anatomical_rad))
            fail(label + ": neutral angle outside the used range");
        const Value &directions =
            *bike::io::find_member(entry_object, "directions");
        // `set(directions) != {'positive','negative'}` — set() iterates the
        // member: dict keys, array elements, or string characters.
        {
            const auto direction_items = detail::iterate_items(directions);
            // set(x) hashes every member: a nested list/dict raises
            // TypeError 'unhashable type' before the comparison runs.
            for (const auto &item : direction_items)
                if (std::holds_alternative<Value::Array>(item.data))
                    throw std::invalid_argument("unhashable type: 'list'");
                else if (std::holds_alternative<Value::Object>(item.data))
                    throw std::invalid_argument("unhashable type: 'dict'");
            std::set<std::string_view> labels;
            for (const auto &item : direction_items)
                if (const auto *text =
                        std::get_if<std::string>(&item.data))
                    labels.insert(*text);
            // Non-string members are hashable too but can never equal the
            // direction names, so they only matter through set difference.
            std::size_t non_string = 0;
            for (const auto &item : direction_items)
                if (!std::holds_alternative<std::string>(item.data))
                    ++non_string;
            if (labels != std::set<std::string_view>{"positive",
                                                   "negative"} ||
                non_string != 0)
                fail(label +
                     ": exactly two torque directions are required");
        }
        // `directions[label]` — an array that passed the set check still
        // fails the subscript, as Python's TypeError does.
        const auto *direction_object =
            std::get_if<Value::Object>(&directions.data);
        if (direction_object == nullptr)
            throw std::invalid_argument(
                "list indices must be integers or slices, not str");
        std::map<int, TorqueCurve> curves;
        for (const auto &[direction_label, sign] :
             {std::pair{"positive", 1}, {"negative", -1}}) {
            const Value &curve_payload = *bike::io::find_member(
                *direction_object, direction_label);
            auto [curve, verified] = strength_curve_value(
                curve_payload, name, direction_label);
            if (curve.angles_rad.front() > low ||
                curve.angles_rad.back() < high)
                fail(label + ":" + direction_label +
                     " curve does not cover the used ROM");
            if (require_verified && !verified)
                fail(label + ":" + direction_label +
                     " strength data is not verified");
            curves.emplace(sign, std::move(curve));
        }
        result.emplace_back(std::string(name), std::move(curves));
    }
    return result;
}

JointEnvelopeMap
load_strength_coordinates(const std::filesystem::path &path,
                          std::span<const std::string_view> expected_joints) {
    using bike::io::Value;
    Value payload = read_json(path);
    const auto *root = std::get_if<Value::Object>(&payload.data);
    if (root == nullptr) {
        // payload['joints'] on a non-mapping — the Python subscript
        // TypeError text depends on the receiver kind.
        if (std::holds_alternative<Value::Array>(payload.data))
            throw std::invalid_argument(
                "list indices must be integers or slices, not str");
        if (std::holds_alternative<std::string>(payload.data))
            throw std::invalid_argument("string indices must be integers");
        throw std::invalid_argument(
            "'" + std::string(detail::python_type_name(payload)) +
            "' object is not subscriptable");
    }
    const Value *joints = bike::io::find_member(*root, "joints");
    if (joints == nullptr)
        throw std::out_of_range("joints");  // payload['joints'] KeyError
    // `set(expected_joints) <= set(joints)` — set() iterates the member:
    // dict keys, array elements or string characters; unhashable elements
    // and non-iterables raise like Python. The strings own their storage:
    // iterate_items copies keys out of the tree, so a string_view set would
    // dangle once the temporary vector dies.
    std::set<std::string, std::less<>> keys;
    {
        const auto items = detail::iterate_items(*joints);
        for (const auto &item : items) {
            if (std::holds_alternative<Value::Array>(item.data))
                throw std::invalid_argument("unhashable type: 'list'");
            if (std::holds_alternative<Value::Object>(item.data))
                throw std::invalid_argument("unhashable type: 'dict'");
            if (const auto *text = std::get_if<std::string>(&item.data))
                keys.insert(*text);
        }
    }
    const auto covered = std::ranges::all_of(
        expected_joints,
        [&](std::string_view name) { return keys.contains(name); });
    if (!covered)
        fail("strength profile must cover the present internal rider "
             "joints");
    const auto *joint_map = std::get_if<Value::Object>(&joints->data);
    JointEnvelopeMap result;
    for (const std::string_view name : expected_joints) {
        if (joint_map == nullptr) {
            // joints[name] — coverage passed on an iterable non-mapping,
            // so the subscript itself is the Python TypeError.
            if (std::holds_alternative<Value::Array>(joints->data))
                throw std::invalid_argument(
                    "list indices must be integers or slices, not str");
            if (std::holds_alternative<std::string>(joints->data))
                throw std::invalid_argument(
                    "string indices must be integers");
            throw std::invalid_argument(
                "'" + std::string(detail::python_type_name(*joints)) +
                "' object is not subscriptable");
        }
        result.emplace_back(
            std::string(name),
            strength_coordinate(*bike::io::find_member(*joint_map, name),
                                name));
    }
    return result;
}

} // namespace bike::configuration
