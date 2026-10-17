#pragma once
#include "../configuration/physical_config.hpp"
#include <span>
#include <string>

namespace bike::numerics {
inline constexpr std::array<double,3> default_density_knots{0.,.004,.012};
struct DensityResponse { std::vector<double> force,energy; };
[[nodiscard]] DensityResponse density_response(std::span<const double>,const bike::configuration::HingeDensity&);
[[nodiscard]] DensityResponse normal_station_response(std::span<const double> delta,
    std::span<const double> rate,std::span<const double> weights,const bike::configuration::HingeDensity&);
[[nodiscard]] DensityResponse flat_load(std::span<const double> deflection,double radius,
    const bike::configuration::HingeDensity&,int station_count=512,double phase=0.);
struct DensityFit {
    bike::configuration::HingeDensity density;
    std::string dataset_id,calibration_status;
    double radius_m=0.,maximum_relative_error=0.,rms_relative_error=0.,error_gate=.05;
    int station_count=512;
    bool accepted=false;
    std::vector<double> target_load_n,predicted_load_n,deflection_m;
};
[[nodiscard]] DensityFit fit_density(std::span<const double> deflections,std::span<const double> loads,
    double radius,std::string dataset_id,std::span<const double> knots=default_density_knots,
    int station_count=512,double damping=0.,double max_relative_error=.05);
} // namespace bike::numerics
