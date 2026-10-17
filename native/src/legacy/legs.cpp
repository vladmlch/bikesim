#include "legs.hpp"
#include "controls.hpp"
#include "../cblas_abi.hpp"
#include "../model/mapping.hpp"
#include "../numeric_sincos.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>
#include <ranges>
#include <stdexcept>

namespace bike::legacy {
namespace {
model::Vec3 solve_or_current(const model::LegChain& chain, double phase,
        double pelvis_z, const model::Vec3& measured) {
    try { return model::solve_leg_qpos(chain,phase,pelvis_z); }
    catch (const std::invalid_argument&) { return measured; }
}
}
LegDrive::LegDrive(const mjModel& m, const std::optional<model::SeatedPose>& pose,
        double crank_length, double depth) : model_(m), crank_length_(crank_length), ripple_depth_(depth) {
    if (!std::isfinite(crank_length) || crank_length <= 0. || !std::isfinite(depth) || depth < 0. || depth > 1.)
        throw std::invalid_argument("legacy leg crank length/ripple domain");
    if (!pose || pose->leg_chains.empty()) return;
    if(pose->leg_chains.size()!=2)throw std::invalid_argument("legacy articulated pose requires two leg chains");
    for (const auto& chain : pose->leg_chains) {
        Leg leg; leg.chain = chain;
        const std::array<std::string,3> names{"hip","knee","ankle"};
        for (std::size_t i = 0; i < 3; ++i) {
            const auto address = model::joint_address(m,"rider_" + names[i] + "_" + chain.side);
            leg.q[i] = static_cast<std::size_t>(address.first); leg.v[i] = static_cast<std::size_t>(address.second);
        }
        const auto pedal = model::joint_address(m,"pedal_spin_" + chain.side);
        leg.pedal_q = static_cast<std::size_t>(pedal.first); leg.pedal_v = static_cast<std::size_t>(pedal.second);
        legs_.push_back(std::move(leg));
    }
    pelvis_q_ = static_cast<std::size_t>(model::joint_address(m,"rider_pelvis_z").first);
    crank_q_ = static_cast<std::size_t>(model::joint_address(m,"crank_spin").first);
}
void LegDrive::initialize(mjData& data, double phase) {
    if (!std::isfinite(phase)) throw std::invalid_argument("legacy leg phase must be finite");
    auto q = std::views::counted(data.qpos,model_.nq);
    auto v = std::views::counted(data.qvel,model_.nv);
    const double pelvis = active() ? q[pelvis_q_] : 0.;
    for (auto& leg : legs_) {
        model::Vec3 actual{};
        for (std::size_t i = 0; i < 3; ++i) actual[i] = q[leg.q[i]];
        const auto ref = solve_or_current(leg.chain,phase,pelvis,actual);
        for (std::size_t i = 0; i < 3; ++i) { q[leg.q[i]] = ref[i]; v[leg.v[i]] = 0.; }
        q[leg.pedal_q] = -phase; v[leg.pedal_v] = 0.; leg.previous = ref;
    }
    pedal_force_ = {};
}
void LegDrive::compute_into(const mjData& data, double torque, std::span<double> force) {
    if (force.size() != static_cast<std::size_t>(model_.nv)) throw std::invalid_argument("legacy leg force width");
    if (!std::isfinite(torque)) throw std::invalid_argument("legacy leg torque must be finite");
    std::ranges::fill(force,0.);
    if (!active()) return;
    const auto q = std::views::counted(data.qpos,model_.nq);
    const auto v = std::views::counted(data.qvel,model_.nv);
    const auto bias = std::views::counted(data.qfrc_bias,model_.nv);
    const double phase = q[crank_q_], pelvis = q[pelvis_q_], dt = model_.opt.timestep;
    const double mean = torque / std::max(ripple_shape(phase,ripple_depth_),1e-6);
    constexpr std::array<double,3> kp{50.,50.,20.}, kd{4.,4.,1.5};
    std::array<model::Vec3,2> references{};
    std::array<double,2> magnitudes{};
    for (std::size_t side = 0; side < legs_.size(); ++side) {
        const auto& leg = legs_[side];
        model::Vec3 actual{}, rates{};
        for (std::size_t i = 0; i < 3; ++i) { actual[i] = q[leg.q[i]]; rates[i] = v[leg.v[i]]; }
        const auto ref = solve_or_current(leg.chain,phase,pelvis,actual);
        const auto previous = leg.previous.value_or(ref);
        const double own_phase = phase + (leg.chain.side == "front" ? 0. : std::numbers::pi);
        const double ft = model::crank_torque_share(own_phase,ripple_depth_) * mean / crank_length_;
        const std::array<double,2> f{ft * -numeric::sin(own_phase),ft * -numeric::cos(own_phase)};
        const auto jacobian = model::leg_jacobian(leg.chain,actual,pelvis);
        model::Vec3 tau{};
        std::array<double,6> matrix{};
        for (std::size_t i = 0; i < 3; ++i) { matrix[2*i] = jacobian[i][0]; matrix[2*i+1] = jacobian[i][1]; }
        blas::cblas_dgemv(static_cast<int>(blas::Order::row_major),static_cast<int>(blas::Transpose::no),
            3,2,1.,matrix.data(),2,f.data(),1,0.,tau.data(),1);
        for (std::size_t i = 0; i < 3; ++i) {
            tau[i] += kp[i] * (ref[i]-actual[i]) + kd[i] * ((ref[i]-previous[i])/dt-rates[i]);
            tau[i] += bias[leg.v[i]];
            if (!std::isfinite(tau[i])) throw std::invalid_argument("legacy leg derived torque is nonfinite");
            force[leg.v[i]] = tau[i];
        }
        references[side] = ref;
        magnitudes[side] = std::sqrt(blas::cblas_ddot(2,f.data(),1,f.data(),1));
    }
    for (std::size_t side = 0; side < legs_.size(); ++side) legs_[side].previous = references[side];
    pedal_force_ = magnitudes;
}
} // namespace bike::legacy
