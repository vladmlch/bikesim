#include "steer_balance.hpp"
#include "steer_balance_detail.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace bike::rider {
namespace {
bool finite_state(const spatial::SteerState& s) noexcept {
    return std::isfinite(s.last_command_nm) && std::isfinite(s.last_applied_nm) &&
        std::isfinite(s.work_j) && std::ranges::all_of(s.delay_fifo_nm, [](double x) { return std::isfinite(x); });
}
} // namespace

SteerBalance::SteerBalance(SteerProfile p, double control_period_s)
    : profile_(std::move(p)), period_s_(control_period_s) {
    detail::validate_steer_profile(profile_);
    if (!std::isfinite(period_s_) || period_s_ <= 0. || profile_.delay_s / period_s_ > 1000000.)
        throw std::invalid_argument("steer.control_period_s: delay FIFO must fit one million ticks");
    const auto identity = detail::steer_profile_fingerprint(profile_);
    if (!profile_.sha256_hex.empty() && profile_.sha256_hex != identity)
        throw std::invalid_argument("steer.profile.sha256_hex: differs from canonical profile payload");
    profile_.sha256_hex = identity;
    const auto ticks = static_cast<std::size_t>(std::llround(profile_.delay_s / period_s_));
    state_.delay_fifo_nm.assign(ticks + 1, 0.);
    state_.profile_sha256 = profile_.sha256_hex;
    state_.controller = spatial::SteerController::roll_heading;
    reset();
}
double SteerBalance::tick(const SteerInput& in) noexcept {
    // There is one consumer per integer clock tick. A force probe or repeated
    // poll may observe, but may not advance, this causal queue.
    if (in.control_tick != state_.next_tick || state_.next_tick == std::numeric_limits<std::int64_t>::max())
        return state_.last_applied_nm;
    if(!in.enabled) {
        state_.delay_fifo_nm[state_.fifo_head]=0.;
        state_.fifo_head=(state_.fifo_head+1)%state_.delay_fifo_nm.size();
        state_.fifo_count=std::min(state_.fifo_count+1,state_.delay_fifo_nm.size());
        state_.last_command_nm=state_.last_applied_nm=0.;
        state_.valid_mask=1;
        ++state_.next_tick;
        return 0.;
    }
    const std::array<double, 6> x{in.roll_rad, in.roll_rate_rad_s, in.heading_error_rad,
        in.lateral_error_m, in.steer_angle_rad, in.steer_rate_rad_s};
    if (!std::isfinite(in.speed_signed_mps) ||
        !std::ranges::all_of(x, [](double a) { return std::isfinite(a); })) return state_.last_applied_nm;
    const double speed = std::abs(in.speed_signed_mps);
    const auto upper = std::ranges::upper_bound(profile_.speed_nodes_mps, speed);
    const std::size_t high = upper == profile_.speed_nodes_mps.end() ? profile_.speed_nodes_mps.size() - 1 :
        static_cast<std::size_t>(upper - profile_.speed_nodes_mps.begin());
    const std::size_t low = high == 0 ? 0 : high - 1;
    double t = high == low ? 0. : (speed-profile_.speed_nodes_mps[low]) /
        (profile_.speed_nodes_mps[high]-profile_.speed_nodes_mps[low]);
    t = std::clamp(t, 0., 1.);
    double raw = profile_.reference_torque_nm;
    for (std::size_t i=0; i<x.size(); ++i)
        raw -= (profile_.gains[low][i] + t*(profile_.gains[high][i]-profile_.gains[low][i])) * x[i];
    if (!std::isfinite(raw)) return state_.last_applied_nm;
    const double command = std::clamp(raw, -profile_.cap_nm, profile_.cap_nm);
    const std::size_t next = (state_.fifo_head+1) % state_.delay_fifo_nm.size();
    // The extra slot makes a zero-delay controller apply the new command in
    // this tick; a twenty-tick queue first emits tick zero at tick twenty.
    const double applied = state_.delay_fifo_nm.size() == 1 ? command : state_.delay_fifo_nm[next];
    const double work = state_.work_j + applied * in.steer_rate_rad_s * period_s_;
    if (!std::isfinite(work)) return state_.last_applied_nm;
    state_.delay_fifo_nm[state_.fifo_head] = command;
    state_.fifo_head = next;
    state_.fifo_count = std::min(state_.fifo_count+1, state_.delay_fifo_nm.size());
    state_.last_command_nm = command;
    state_.last_applied_nm = applied;
    state_.work_j = work;
    state_.valid_mask=1;
    if (raw != command && state_.saturated_ticks < std::numeric_limits<std::int64_t>::max())
        ++state_.saturated_ticks;
    ++state_.next_tick;
    return applied;
}
const spatial::SteerState& SteerBalance::state() const noexcept { return state_; }
CoreStatus SteerBalance::restore(const spatial::SteerState& s) noexcept {
    if (s.schema!=1 || s.controller!=spatial::SteerController::roll_heading ||
        s.profile_sha256!=profile_.sha256_hex || s.next_tick<0 || s.saturated_ticks<0 ||
        s.delay_fifo_nm.size()!=state_.delay_fifo_nm.size() || s.fifo_count>s.delay_fifo_nm.size() ||
        s.fifo_head>=s.delay_fifo_nm.size() || !finite_state(s) ||
        std::abs(s.last_applied_nm)>profile_.cap_nm || std::abs(s.last_command_nm)>profile_.cap_nm ||
        !std::ranges::all_of(s.delay_fifo_nm,[&](double x){return std::abs(x)<=profile_.cap_nm;}))
        return CoreStatus::invalid_input;
    // Equal capacities and immutable identity: restore needs no allocation.
    std::ranges::copy(s.delay_fifo_nm, state_.delay_fifo_nm.begin());
    state_.next_tick=s.next_tick;state_.fifo_head=s.fifo_head;state_.fifo_count=s.fifo_count;
    state_.last_command_nm=s.last_command_nm;state_.last_applied_nm=s.last_applied_nm;
    state_.saturated_ticks=s.saturated_ticks;state_.work_j=s.work_j;state_.valid_mask=s.valid_mask;
    return CoreStatus::ok;
}
void SteerBalance::reset() noexcept {
    std::ranges::fill(state_.delay_fifo_nm,0.);
    state_.next_tick=0;state_.fifo_head=0;state_.fifo_count=0;
    state_.last_command_nm=0.;state_.last_applied_nm=0.;state_.saturated_ticks=0;state_.work_j=0.;
    state_.valid_mask=1;
}
double SteerBalance::resolved_delay_s() const noexcept { return static_cast<double>(delay_ticks())*period_s_; }
std::size_t SteerBalance::delay_ticks() const noexcept { return state_.delay_fifo_nm.size()-1; }
} // namespace bike::rider
