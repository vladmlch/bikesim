#pragma once
#include "common.hpp"
#include "../config_types.hpp"
#include "../contact/tabulated_radial.hpp"

namespace bike::validation {
struct TireCalibration {
    std::string dataset_id, source_kind, tyre_label;
    double radius_m{}, inflated_width_m{}, rim_internal_width_m{};
    nativecfg::TireMaterial material;
    [[nodiscard]] contactlaw::TabulatedRadial curve() const;
    [[nodiscard]] bool is_load_in_valid_range(double load_n) const;
};
[[nodiscard]] TireCalibration calibration_from_record(const Value& record);
[[nodiscard]] TireCalibration load_calibration(const std::filesystem::path& path);
[[nodiscard]] Value steady_brush_force(const nativecfg::TireParams& tire,
    const nativecfg::SurfaceSpec& surface, double slip_ratio, double load_n,
    double speed_mps = 2., double dt_s = .000625, double duration_s = 1.);

void validate_split(const Value& fit, const Value& holdout);
[[nodiscard]] bool is_synthetic(const Value& payload);
[[nodiscard]] std::string_view accepted_calibration_status(bool synthetic, bool holdout_passed,
                                                     bool converged, bool within_scope);
[[nodiscard]] std::string dataset_hash(const Value& payload);
[[nodiscard]] Value validate_dataset(const Value& payload, bool allow_synthetic = false);
[[nodiscard]] Value holdout_metrics(std::span<const double> measured, std::span<const double> predicted);
[[nodiscard]] Value calibration_report(const Value& payload, const Value& predictions,
    std::string_view output, const Value& parameters, const Value& bounds,
    const Value& error_budget, bool allow_synthetic = false);
} // namespace bike::validation
