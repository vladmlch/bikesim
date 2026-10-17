#pragma once
#include "support_dispatch.hpp"
#include "support_geometry.hpp"
#include "rider_posture.hpp"
#include "../io/value.hpp"

namespace bike::rider {
struct FootTargetDiagnostics {
    std::array<double,3> actual{},target{};
    bool ik_saturated=false,weld=false,recovery=false;
    std::string recovery_stage="none";
    std::optional<::rider::SoleGoalDiagnostic> geometry;
};
struct LegacySupportDiagnostics {
    SupportTargetDiagnostics target;
    std::array<bool,2> stance{};
    std::array<std::array<double,3>,2> pedal_forces{};
    ::rider::RiderPosture posture;
    bool coasting=false;
    double coasting_requested_crank_torque_nm=0.,actual_phase_rad=0.,actual_rate_rad_s=0.,target_rate_rad_s=0.;
    std::optional<double> target_phase_rad,phase_error_rad;
    std::array<FootTargetDiagnostics,2> feet;
};
[[nodiscard]] LegacySupportDiagnostics decode_support_diagnostics(const io::Value&,std::string_view path);
[[nodiscard]] io::Value encode_support_diagnostics(const LegacySupportDiagnostics&);
} // namespace bike::rider
