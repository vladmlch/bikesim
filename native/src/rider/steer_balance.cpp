#include "steer_balance.hpp"

#include "../io/hash.hpp"
#include "../io/json.hpp"
#include "../runtime/setup_values.hpp"
#include "../runtime/value_readers.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <span>
#include <stdexcept>
#include <utility>

namespace bike::rider {
namespace {
using namespace runtime::setupvalue;

Value profile_value(const SteerProfile& p, bool identity) {
    Array gains;
    for (const auto& row : p.gains) gains.push_back(reals(row));
    return Value{Object{{"schema", integer(p.schema)},
        {"speed_nodes_mps", reals(p.speed_nodes_mps)}, {"gains", Value{std::move(gains)}},
        {"reference_torque_nm", real(p.reference_torque_nm)}, {"cap_nm", real(p.cap_nm)},
        {"delay_s", real(p.delay_s)}, {"sha256_hex", identity ? text(p.sha256_hex) : Value{}},
        {"provenance_status", text(p.provenance_status)}}};
}
void validate(const SteerProfile& p) {
    if (p.schema != 1) throw std::invalid_argument("steer.profile.schema: expected 1");
    if (p.speed_nodes_mps.empty() || p.gains.size() != p.speed_nodes_mps.size())
        throw std::invalid_argument("steer.profile.gains: expected one six-component row per speed node");
    for (std::size_t i = 0; i < p.speed_nodes_mps.size(); ++i) {
        if (!std::isfinite(p.speed_nodes_mps[i]) || p.speed_nodes_mps[i] <= 0. ||
            (i != 0 && p.speed_nodes_mps[i] <= p.speed_nodes_mps[i - 1]))
            throw std::invalid_argument("steer.profile.speed_nodes_mps: expected increasing positive finite nodes");
        for (double k : p.gains[i]) if (!std::isfinite(k))
            throw std::invalid_argument("steer.profile.gains: nonfinite coefficient");
    }
    if (!std::isfinite(p.reference_torque_nm) || !std::isfinite(p.cap_nm) || p.cap_nm <= 0. ||
        !std::isfinite(p.delay_s) || p.delay_s < 0.)
        throw std::invalid_argument("steer.profile: invalid torque cap, reference, or reaction delay");
    if (p.provenance_status != "provisional" && p.provenance_status != "calibrated-on-fixture" &&
        p.provenance_status != "measured" && p.provenance_status != "synthetic")
        throw std::invalid_argument("steer.profile.provenance_status: unsupported status");
}
std::string fingerprint(const SteerProfile& p) {
    const auto bytes = io::canonical_json(profile_value(p, false));
    return io::sha256(std::as_bytes(std::span{bytes}));
}
bool finite_state(const spatial::SteerState& s) noexcept {
    return std::isfinite(s.last_command_nm) && std::isfinite(s.last_applied_nm) &&
        std::isfinite(s.work_j) && std::ranges::all_of(s.delay_fifo_nm, [](double x) { return std::isfinite(x); });
}
} // namespace

SteerProfile read_steer_profile(const io::Value& value) {
    const auto& obj = vread::mapping(vread::Node{&value}, "steer.profile");
    const vread::Dict d{.value=&obj, .path="steer.profile"};
    // Parsed JSON already rejects duplicates, but the public Value boundary
    // also accepts documents assembled by native callers. Do not silently
    // fingerprint only the first of two conflicting safety-critical fields.
    for (std::size_t i = 0; i < obj.size(); ++i)
        for (std::size_t j = 0; j < i; ++j)
            if (obj[i].first == obj[j].first)
                vread::invalid(d.child(obj[i].first), "duplicate key");
    vread::exact(d, vread::keys("schema", "speed_nodes_mps", "gains", "reference_torque_nm",
        "cap_nm", "delay_s", "sha256_hex", "provenance_status"));
    if (vread::integer(vread::field(d, "schema"), d.child("schema")) != 1)
        vread::invalid(d.child("schema"), "expected 1");
    SteerProfile p;
    p.speed_nodes_mps = vread::vector(vread::field(d, "speed_nodes_mps"), d.child("speed_nodes_mps"));
    const auto& rows = vread::sequence(vread::field(d, "gains"), d.child("gains"));
    for (std::size_t i=0; i<rows.size(); ++i)
        p.gains.push_back(vread::fixed<6>(vread::Node{&rows[i]}, d.child("gains")+"["+std::to_string(i)+"]"));
    p.reference_torque_nm = vread::finite_real(vread::field(d, "reference_torque_nm"), d.child("reference_torque_nm"));
    p.cap_nm = vread::finite_real(vread::field(d, "cap_nm"), d.child("cap_nm"));
    p.delay_s = vread::finite_real(vread::field(d, "delay_s"), d.child("delay_s"));
    p.provenance_status = vread::string(vread::field(d, "provenance_status"), d.child("provenance_status"));
    validate(p);
    p.sha256_hex = fingerprint(p);
    const auto* supplied = vread::field(d, "sha256_hex").value;
    if (!io::is_null(*supplied) && vread::string(vread::Node{supplied}, d.child("sha256_hex")) != p.sha256_hex)
        vread::invalid(d.child("sha256_hex"), "differs from canonical profile payload");
    return p;
}
io::Value write_steer_profile(const SteerProfile& p) {
    validate(p);
    if (p.sha256_hex != fingerprint(p))
        throw std::invalid_argument("steer.profile.sha256_hex: differs from canonical profile payload");
    return profile_value(p, true);
}
SteerBalance::SteerBalance(SteerProfile p, double control_period_s)
    : profile_(std::move(p)), period_s_(control_period_s) {
    validate(profile_);
    if (!std::isfinite(period_s_) || period_s_ <= 0. || profile_.delay_s / period_s_ > 1000000.)
        throw std::invalid_argument("steer.control_period_s: delay FIFO must fit one million ticks");
    const auto identity = fingerprint(profile_);
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
    const auto upper = std::upper_bound(profile_.speed_nodes_mps.begin(), profile_.speed_nodes_mps.end(), speed);
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
