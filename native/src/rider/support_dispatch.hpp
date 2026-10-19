#pragma once
#include "contact_config.hpp"
#include <mujoco/mujoco.h>
#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace writers {class RiderContactWriter; struct RiderSettleOutcome;}
namespace bike::rider {
struct SupportTargetDiagnostics {
    double requested_pitch_moment_nm=0.,vertical_force_error_n=0.,gravity_moment_error_nm=0.,
        coasting_crank_torque_nm=0.,horizontal_force_error_n=0.,total_requested_pitch_moment_nm=0.;
    bool feasible=false;
    std::array<double,4> vertical_forces_n{};
    std::array<double,3> requested_balance_force{},total_requested_force{};
};
struct GravitySupportTargets {std::array<double,4> forces{};SupportTargetDiagnostics diagnostics;};
struct PedalingSupportTargets {std::array<std::array<double,3>,4> forces_on_bike{};SupportTargetDiagnostics diagnostics;};
[[nodiscard]] GravitySupportTargets gravity_support_targets(double weight,double com_x,
    const std::array<double,4>& points_x,double crank_x,double pedal_fraction,double bar_fraction,
    const std::array<bool,4>& enabled,double pitch_moment=0.,std::optional<double> vertical_target=std::nullopt);
[[nodiscard]] PedalingSupportTargets pedaling_support_targets(double weight,const std::array<double,3>& com,
    const std::array<std::array<double,3>,4>& points,double crank_x,double pedal_fraction,double bar_fraction,
    const std::array<bool,4>& enabled,const std::array<std::array<double,3>,2>& pedal_forces,
    double pitch_moment=0.,const std::array<double,3>& balance={0.,0.,0.});
struct SupportDispatch {
    ::rider::PedalAttachment pedals{};
    ::rider::SaddleAttachment saddle{};
    ::rider::GripAttachment grips{};
    bool closed_form_spindle=false;
};
[[nodiscard]] SupportDispatch support_dispatch(const ::rider::RiderContactsConfig&);
void validate_support_topology(const mjModel&,const ::rider::RiderContactsConfig&);
[[nodiscard]] std::unique_ptr<writers::RiderContactWriter> make_support_writer(
    const mjModel&,const ::rider::RiderContactsConfig&);
// Owned writer operations preserve its release loss/energy and solved
// reaction lifecycle; no parallel contact model is introduced.
// The bool is the resulting enabled state, not an operation-success flag:
// a connected grip remains enabled when a release is requested.
[[nodiscard]] bool set_spring_grip_enabled(writers::RiderContactWriter&,bool enabled);
[[nodiscard]] writers::RiderSettleOutcome settle_supports(writers::RiderContactWriter&,mjData&,bool raw=false);
} // namespace bike::rider
