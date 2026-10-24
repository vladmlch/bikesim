#include "bootstrap_state.hpp"
#include "brake_state.hpp"
#include "../runtime/setup_values.hpp"
#include "../runtime/value_readers.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace bike::spatial {
namespace {
using namespace runtime::setupvalue;

vread::Dict object(const Value& value, std::string path) {
    const auto& fields = vread::mapping(vread::Node{&value}, path);
    for (std::size_t i = 0; i < fields.size(); ++i)
        for (std::size_t j = 0; j < i; ++j)
            if (fields[i].first == fields[j].first)
                vread::invalid(path + "." + fields[i].first, "duplicate key");
    return {.value = &fields, .path = std::move(path)};
}
double number(const vread::Dict& d, std::string_view key) {
    return vread::finite_real(vread::field(d, key), d.child(key));
}
std::int64_t count(const vread::Dict& d, std::string_view key,
                   std::int64_t minimum = 0) {
    const auto value = vread::integer(vread::field(d, key), d.child(key));
    if (value < minimum) vread::invalid(d.child(key), "integer below accepted range");
    return value;
}
std::uint8_t mask(const vread::Dict& d) {
    const auto value = count(d, "valid_mask");
    if (value > 255) vread::invalid(d.child("valid_mask"), "expected uint8");
    return static_cast<std::uint8_t>(value);
}
void schema(const vread::Dict& d) {
    if (count(d, "schema") != 1)
        vread::invalid(d.child("schema"), "unsupported state schema");
}
void nonnegative(double value, const std::string& path) {
    if (value < 0.) vread::invalid(path, "expected nonnegative value");
}
void unit_quaternion(const Quat& q, const std::string& path) {
    const double norm = std::hypot(std::hypot(q[0], q[1]), std::hypot(q[2], q[3]));
    if (std::abs(norm - 1.) > 1e-9)
        vread::invalid(path, "expected unit quaternion within 1e-9");
}
void digest_or_empty(std::string_view value, const std::string& path) {
    if (!value.empty() && (value.size() != 64 ||
        !std::ranges::all_of(value, [](char c) {
            return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
        }))) vread::invalid(path, "expected empty or lowercase SHA-256");
}
Value hold_value(const HoldState& s) {
    return Value{Object{{"schema", integer(s.schema)}, {"active", Value{s.active}},
        {"reference_position_world_m", reals(s.reference_position_world_m)},
        {"reference_quat_wxyz", reals(s.reference_quat_wxyz)},
        {"last_force_world_n", reals(s.last_force_world_n)},
        {"last_couple_world_nm", reals(s.last_couple_world_nm)},
        {"stored_energy_j", real(s.stored_energy_j)}, {"work_j", real(s.work_j)},
        {"phase", text(to_string(s.phase))}, {"valid_mask", integer(s.valid_mask)}}};
}
Value steer_value(const SteerState& s) {
    if (s.fifo_head > static_cast<std::size_t>(std::numeric_limits<std::int64_t>::max()) ||
        s.fifo_count > static_cast<std::size_t>(std::numeric_limits<std::int64_t>::max()))
        vread::invalid("steer.delay_fifo_nm", "index exceeds wire integer domain");
    return Value{Object{{"schema", integer(s.schema)},
        {"controller", text(s.controller == SteerController::off ? "off" :
                            s.controller == SteerController::roll_heading ? "roll_heading" : "invalid")},
        {"next_tick", integer(s.next_tick)}, {"delay_fifo_nm", reals(s.delay_fifo_nm)},
        {"fifo_head", integer(static_cast<std::int64_t>(s.fifo_head))},
        {"fifo_count", integer(static_cast<std::int64_t>(s.fifo_count))},
        {"profile_sha256", text(s.profile_sha256)},
        {"last_command_nm", real(s.last_command_nm)}, {"last_applied_nm", real(s.last_applied_nm)},
        {"saturated_ticks", integer(s.saturated_ticks)}, {"work_j", real(s.work_j)},
        {"valid_mask", integer(s.valid_mask)}}};
}
Value outcome_value(const OutcomeState& s) {
    const auto& l = s.latched;
    return Value{Object{{"schema", integer(s.schema)}, {"latched", Value{Object{
        {"reason", text(to_string(l.reason))}, {"terminal", Value{l.terminal}},
        {"valid", Value{l.valid}}, {"tick", integer(l.tick)}, {"time_s", real(l.time_s)},
        {"position_world_m", reals(l.position_world_m)}, {"diagnostic", text(l.diagnostic)}}},
        {"capsize_dwell_ticks", integer(s.capsize_dwell_ticks)},
        {"corridor_dwell_ticks", integer(s.corridor_dwell_ticks)},
        {"balance_dwell_ticks", integer(s.balance_dwell_ticks)},
        {"valid_mask", integer(s.valid_mask)}}};
}

} // namespace

std::string_view to_string(BootstrapPhase phase) noexcept {
    switch (phase) {
    case BootstrapPhase::settling: return "settling";
    case BootstrapPhase::release_check: return "release_check";
    case BootstrapPhase::armed: return "armed";
    case BootstrapPhase::scored: return "scored";
    }
    return "invalid";
}
std::optional<BootstrapPhase> parse_phase(std::string_view name) noexcept {
    for (auto phase : {BootstrapPhase::settling, BootstrapPhase::release_check,
                       BootstrapPhase::armed, BootstrapPhase::scored})
        if (name == to_string(phase)) return phase;
    return std::nullopt;
}
std::string_view to_string(SpatialReason reason) noexcept {
    switch (reason) {
    case SpatialReason::none: return "none";
    case SpatialReason::capsize: return "crash:capsize";
    case SpatialReason::off_corridor: return "crash:off_corridor";
    case SpatialReason::balance_lost: return "balance_lost";
    case SpatialReason::terrain_invalid: return "invalid:terrain";
    case SpatialReason::terrain_outside: return "invalid:outside";
    case SpatialReason::numerical_failure: return "invalid:numerical";
    case SpatialReason::requested_completion: return "completed";
    }
    return "invalid";
}
HoldState canonical_inactive_hold() noexcept { return {}; }
SteerState canonical_inactive_steer() { return {}; }
OutcomeState canonical_unlatched_outcome() noexcept { return {}; }

HoldState decode_hold(const Value& value) {
    const auto d = object(value, "hold");
    vread::exact(d, vread::keys("schema", "active", "reference_position_world_m",
        "reference_quat_wxyz", "last_force_world_n", "last_couple_world_nm",
        "stored_energy_j", "work_j", "phase", "valid_mask"));
    schema(d);
    HoldState s;
    s.active = vread::boolean(vread::field(d, "active"), d.child("active"));
    s.reference_position_world_m = vread::fixed<3>(vread::field(d, "reference_position_world_m"), d.child("reference_position_world_m"));
    s.reference_quat_wxyz = vread::fixed<4>(vread::field(d, "reference_quat_wxyz"), d.child("reference_quat_wxyz"));
    unit_quaternion(s.reference_quat_wxyz, d.child("reference_quat_wxyz"));
    s.last_force_world_n = vread::fixed<3>(vread::field(d, "last_force_world_n"), d.child("last_force_world_n"));
    s.last_couple_world_nm = vread::fixed<3>(vread::field(d, "last_couple_world_nm"), d.child("last_couple_world_nm"));
    s.stored_energy_j = number(d, "stored_energy_j");
    nonnegative(s.stored_energy_j, d.child("stored_energy_j"));
    s.work_j = number(d, "work_j");
    const auto phase = parse_phase(vread::string(vread::field(d, "phase"), d.child("phase")));
    if (!phase) vread::invalid(d.child("phase"), "unknown bootstrap phase");
    s.phase = *phase;
    if (s.phase >= BootstrapPhase::armed && s.active)
        vread::invalid(d.child("active"), "hold is active only during settling or release_check");
    if (s.phase >= BootstrapPhase::armed && (s.stored_energy_j != 0. ||
        std::ranges::any_of(s.last_force_world_n,[](double x){return x!=0.;}) ||
        std::ranges::any_of(s.last_couple_world_nm,[](double x){return x!=0.;})))
        vread::invalid(d.child("last_force_world_n"), "released hold requires exactly zero wrench and stored energy");
    s.valid_mask = mask(d);
    return s;
}
SteerState decode_steer(const Value& value) {
    const auto d = object(value, "steer");
    vread::exact(d, vread::keys("schema", "controller", "next_tick", "delay_fifo_nm",
        "fifo_head", "fifo_count", "profile_sha256", "last_command_nm", "last_applied_nm",
        "saturated_ticks", "work_j", "valid_mask"));
    schema(d);
    SteerState s;
    const auto& controller = vread::string(vread::field(d, "controller"), d.child("controller"));
    if (controller == "off") s.controller = SteerController::off;
    else if (controller == "roll_heading") s.controller = SteerController::roll_heading;
    else vread::invalid(d.child("controller"), "unknown controller");
    s.next_tick = count(d, "next_tick");
    s.delay_fifo_nm = vread::vector(vread::field(d, "delay_fifo_nm"), d.child("delay_fifo_nm"));
    const auto head = count(d, "fifo_head"), size = count(d, "fifo_count");
    if (std::cmp_greater(size, s.delay_fifo_nm.size()))
        vread::invalid(d.child("fifo_count"), "exceeds FIFO capacity");
    if ((s.delay_fifo_nm.empty() && head != 0) ||
        (!s.delay_fifo_nm.empty() && std::cmp_greater_equal(head, s.delay_fifo_nm.size())))
        vread::invalid(d.child("fifo_head"), "outside FIFO capacity");
    s.fifo_head = static_cast<std::size_t>(head);
    s.fifo_count = static_cast<std::size_t>(size);
    s.profile_sha256 = vread::string(vread::field(d, "profile_sha256"), d.child("profile_sha256"));
    digest_or_empty(s.profile_sha256, d.child("profile_sha256"));
    if(s.controller!=SteerController::off && (s.profile_sha256.empty() || s.delay_fifo_nm.empty()))
        vread::invalid(d.child("profile_sha256"), "active controller requires profile identity and a sized FIFO");
    s.last_command_nm = number(d, "last_command_nm");
    s.last_applied_nm = number(d, "last_applied_nm");
    s.saturated_ticks = count(d, "saturated_ticks");
    s.work_j = number(d, "work_j");
    s.valid_mask = mask(d);
    return s;
}
OutcomeState decode_outcome(const Value& value) {
    const auto d = object(value, "outcome");
    vread::exact(d, vread::keys("schema", "latched", "capsize_dwell_ticks",
        "corridor_dwell_ticks", "balance_dwell_ticks", "valid_mask"));
    schema(d);
    OutcomeState s;
    const auto l = object(*vread::field(d, "latched").value, d.child("latched"));
    vread::exact(l, vread::keys("reason", "terminal", "valid", "tick", "time_s",
        "position_world_m", "diagnostic"));
    const auto& name = vread::string(vread::field(l, "reason"), l.child("reason"));
    bool found = false;
    for (auto reason : {SpatialReason::none, SpatialReason::capsize, SpatialReason::off_corridor,
                        SpatialReason::balance_lost, SpatialReason::terrain_invalid,
                        SpatialReason::terrain_outside, SpatialReason::numerical_failure,
                        SpatialReason::requested_completion})
        if (name == to_string(reason)) { s.latched.reason = reason; found = true; break; }
    if (!found) vread::invalid(l.child("reason"), "unknown outcome reason");
    s.latched.terminal = vread::boolean(vread::field(l, "terminal"), l.child("terminal"));
    s.latched.valid = vread::boolean(vread::field(l, "valid"), l.child("valid"));
    s.latched.tick = count(l, "tick", -1);
    s.latched.time_s = number(l, "time_s");
    nonnegative(s.latched.time_s, l.child("time_s"));
    s.latched.position_world_m = vread::fixed<3>(vread::field(l, "position_world_m"), l.child("position_world_m"));
    s.latched.diagnostic = vread::string(vread::field(l, "diagnostic"), l.child("diagnostic"));
    if (s.latched.diagnostic.find('\0') != std::string::npos)
        vread::invalid(l.child("diagnostic"), "embedded NUL");
    if (s.latched.reason == SpatialReason::none && (s.latched.terminal || s.latched.tick != -1))
        vread::invalid(l.child("tick"), "unlatched outcome requires tick=-1 and terminal=false");
    if (s.latched.reason != SpatialReason::none && s.latched.tick < 0)
        vread::invalid(l.child("tick"), "latched outcome requires nonnegative tick");
    if (s.latched.reason != SpatialReason::none && !s.latched.terminal)
        vread::invalid(l.child("terminal"), "a reason is present only in a terminal latch");
    if (s.latched.terminal && s.latched.valid &&
        (s.latched.reason==SpatialReason::terrain_invalid || s.latched.reason==SpatialReason::terrain_outside ||
         s.latched.reason==SpatialReason::numerical_failure))
        vread::invalid(l.child("valid"), "numerical and terrain failures are invalid episodes");
    s.capsize_dwell_ticks = count(d, "capsize_dwell_ticks");
    s.corridor_dwell_ticks = count(d, "corridor_dwell_ticks");
    s.balance_dwell_ticks = count(d, "balance_dwell_ticks");
    s.valid_mask = mask(d);
    return s;
}

Value encode_hold(const HoldState& s) { auto v = hold_value(s); (void)decode_hold(v); return v; }
Value encode_steer(const SteerState& s) { auto v = steer_value(s); (void)decode_steer(v); return v; }
Value encode_outcome(const OutcomeState& s) { auto v = outcome_value(s); (void)decode_outcome(v); return v; }
} // namespace bike::spatial
