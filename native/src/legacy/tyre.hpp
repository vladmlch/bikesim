#pragma once

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include "../config_types.hpp"
#include "../geometry/specs.hpp"
#include "../pneumatic/model.hpp"
#include "../terrain/types.hpp"

namespace bike::legacy {

using TyreSpecs = nativecfg::PneumaticTyreSpec;
inline constexpr double BAR_TO_PA=100000.,PRESSURE_MIN_BAR=.8,PRESSURE_MAX_BAR=3.,PRESSURE_STEP_BAR=.05;
inline constexpr std::array<std::string_view,2> TYRE_MODELS{"sphere","pneumatic"},TYRE_TIERS{"fast","detailed"};
inline constexpr std::string_view DEFAULT_TYRE_MODEL="sphere",DEFAULT_TYRE_TIER="fast";
inline constexpr double STATIC_STIFFNESS_INTERCEPT_N_MM=22.,STATIC_STIFFNESS_SLOPE_N_MM_PER_BAR=24.,STIFFNESS_TOLERANCE=.15,REFERENCE_LOAD_N=418.;
inline constexpr std::array<double,2> STIFFNESS_PRESSURE_RANGE_BAR{1.,2.},PRESSURE_SHARE_BAND{.70,1.},DYNAMIC_STIFFNESS_RATIO_BAND{1.16,1.35},DAMPING_RATIO_BAND{.02,.055},RIM_STRIKE_FRACTION_BAND{.80,.85};
inline constexpr std::array<std::array<double,2>,2> CONTACT_LENGTH_TARGETS_MM{{{1.38,133.},{1.72,122.}}};
inline constexpr double CONTACT_LENGTH_TOLERANCE=.15,DROP_SLED_MASS_KG=435./9.81,CRR_REFERENCE_PRESSURE_BAR=1.5,CRR_REFERENCE_LOAD_N=490.5,CRR_REFERENCE_SPEED_KMH=20.,CRR_PRESSURE_EXPONENT=-.3,CRR_TOLERANCE=.15;
// Source constructor defaults, while the common wire payload remains the
// existing nativecfg::PneumaticTyreSpec used by the material kernels.
[[nodiscard]] TyreSpecs make_tyre_specs(std::string name,double outer_radius_mm,
    double rim_radius_mm,double section_height_mm,double pressure_bar,double crr_reference,
    double casing_width_mm=60.,double compressed_casing_mm=6.,double area_factor=.46,
    double carcass_stiffness_n_mm2=.10,double contact_length_factor=.86,double rate_stiffening=.25,
    double rate_relaxation_s=.2,double loss_factor=.07,double hysteresis_rate_eps_mps=.01,
    double relaxation_length_mm=60.,double rim_stiffness_n_mm2=28.,double tread_loss_crr=0.);
[[nodiscard]] TyreSpecs front_tyre();
[[nodiscard]] TyreSpecs rear_tyre();
[[nodiscard]] TyreSpecs with_pressure(TyreSpecs, double pressure_bar);
[[nodiscard]] double static_stiffness_target_n_mm(double pressure_bar);
[[nodiscard]] double crr_target(const TyreSpecs&, std::optional<double> pressure_bar = {});
using biketyre::clamp_pressure_bar;

struct TierSpec {
    std::string name;
    std::int64_t n_rays;
    double half_angle_deg, timestep_s;
    bool discretised_brush;
};
[[nodiscard]] TierSpec tier_spec(std::string_view name);
struct TyreConfig {
    std::string model = "sphere", tier = "fast";
    TyreSpecs front = front_tyre(), rear = rear_tyre();
    std::optional<std::string> surface;
    void validate() const;
    [[nodiscard]] bool pneumatic() const noexcept { return model == "pneumatic"; }
    [[nodiscard]] TierSpec resolved_tier() const { return tier_spec(tier); }
    [[nodiscard]] TyreConfig with_pressures(double front_bar, double rear_bar) const;
};

// The source's world-grid profile: ground plane + build_profile, before
// MuJoCo's binary32 heightfield quantization. Physical backends use their
// separate compiled-raster contact profile.
[[nodiscard]] biketyre::RoadProfile build_road_profile(const terrain::TrackSpec&,
    const terrain::HeightFieldSpec&, const geometry::BikeSpecs&);
[[nodiscard]] std::optional<double> first_crossing_reference(double theta_rad,double cx,double cz,double radius,
    const biketyre::RoadProfile&,std::size_t samples=20000);

// Legacy xfrc_applied owner. Uses the source pneumatic equations directly;
// it is deliberately distinct from PhysicalStep's generalized-force writer.
class TyreForceApplier {
public:
    TyreForceApplier(mjModel&, TyreConfig, biketyre::RoadProfile,
                     const terrain::SurfaceMap&);
    ~TyreForceApplier();
    TyreForceApplier(const TyreForceApplier&) = delete;
    TyreForceApplier& operator=(const TyreForceApplier&) = delete;
    TyreForceApplier(TyreForceApplier&&) = delete;
    TyreForceApplier& operator=(TyreForceApplier&&) = delete;
    void apply(mjData&);
    void reset(mjData* = nullptr);
    void set_pressures(double front_bar, double rear_bar);
    [[nodiscard]] const biketyre::WheelOutputs& front_outputs() const;
    [[nodiscard]] const biketyre::WheelOutputs& rear_outputs() const;
    [[nodiscard]] const TyreConfig& config() const noexcept;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace bike::legacy
