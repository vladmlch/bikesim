// rider/launch_plan.cpp — see launch_plan.hpp and plan 3b decisions 4-7, 11.
#include "launch_plan.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace rider {
namespace {

[[noreturn]] void bad_state(std::string_view field, std::string_view why) {
    throw std::invalid_argument("launch plan state " + std::string(field) + ": " + std::string(why));
}

std::int64_t ticks(double seconds, double period_s) {
    return std::llround(seconds / period_s);
}

// The table, period and duration checks run inside the constructor's first
// member initializer, so every ticks() below converts a validated value —
// the ordering the plan's constructor body spelled out.
LaunchConfig valid(LaunchConfig config, double period_s) {
    validate_launch_plan_context(config, period_s);
    return config;
}

} // namespace

const char *launch_phase_name(LaunchPhase phase) {
    switch (phase) {
    case LaunchPhase::idle: return "idle";
    case LaunchPhase::hold: return "hold";
    case LaunchPhase::release: return "release";
    case LaunchPhase::accelerate: return "accelerate";
    case LaunchPhase::handover: return "handover";
    case LaunchPhase::blend: return "blend";
    case LaunchPhase::done: return "done";
    }
    throw std::invalid_argument("launch phase outside the enum");
}

LaunchPhase launch_phase(std::string_view name) {
    for (const LaunchPhase phase : {LaunchPhase::idle, LaunchPhase::hold, LaunchPhase::release,
                                    LaunchPhase::accelerate, LaunchPhase::handover, LaunchPhase::blend,
                                    LaunchPhase::done})
        if (name == launch_phase_name(phase)) return phase;
    throw std::invalid_argument("unknown launch phase " + std::string(name));
}

ShiftResponse shift_response(std::string_view label) {
    if (label == "none") return ShiftResponse::none;
    if (label == "accepted") return ShiftResponse::accepted;
    if (label == "deferred") return ShiftResponse::deferred;
    if (label == "rejected") return ShiftResponse::rejected;
    throw std::invalid_argument("unknown shifter rider_response " + std::string(label));
}

LaunchPlan::LaunchPlan(LaunchConfig config, double period_s)
    : config_(valid(std::move(config), period_s)),
      period_s_(period_s),
      release_ticks_(ticks(config_.brake_release_s, period_s_)),
      max_hold_ticks_(ticks(config_.max_hold_s, period_s_)),
      end_hold_ticks_(ticks(config_.end_hold_s, period_s_)),
      max_launch_ticks_(ticks(config_.max_launch_s, period_s_)),
      handover_ticks_(ticks(config_.handover_s, period_s_)),
      retry_ticks_(ticks(config_.shift_retry_s, period_s_)) {}

void LaunchPlan::reset() { s_ = State{}; }

bool LaunchPlan::arm_start(double lean_rad) {
    if (!config_.plan_enabled) return false;
    if (!std::isfinite(lean_rad)) throw std::invalid_argument("launch plan start lean must be finite");
    State n;
    arm(n, lean_rad);
    s_ = n;
    return true;
}

void LaunchPlan::validate(const LaunchInputs &in) {
    if (in.percepts) {
        const Percepts &p = *in.percepts;
        if (!std::isfinite(p.senses.speed_mps) || !std::isfinite(p.senses.grade_ahead) ||
            !std::isfinite(p.crank_rpm) || !std::isfinite(p.human_crank_torque_nm))
            throw std::invalid_argument("launch plan percepts must be finite");
    }
    if (in.shift.shift_count < 0) throw std::invalid_argument("launch plan shift count must be nonnegative");
}

double LaunchPlan::cap(SurfaceLabel surface) const {
    switch (surface) {
    case SurfaceLabel::asphalt: return config_.effort_cap_asphalt_nm;
    case SurfaceLabel::hardpack: return config_.effort_cap_hardpack_nm;
    case SurfaceLabel::loose: return config_.effort_cap_loose_nm;
    case SurfaceLabel::wet: return config_.effort_cap_wet_nm;
    case SurfaceLabel::scree: return config_.effort_cap_scree_nm;
    }
    throw std::invalid_argument("launch plan surface outside the enum");
}

double LaunchPlan::ramp(double effort, const Percepts &p, State &n) const {
    // Launch part 4, decisions 1-2: the lower of the surface cap and the
    // grade cap; from the first tick at the rise speed the grade cap rises at
    // the effort rate (the calibration probe's arithmetic, operand order kept).
    const double surface = cap(p.senses.surface_front);
    const double grade = detail::grade_cap_unchecked(config_, p.senses.grade_ahead);
    double ceiling = std::min(surface, grade);
    if (p.senses.speed_mps >= config_.effort_rise_speed_mps) {
        if (n.rise_ticks < 0) n.rise_ticks = 0;
        ceiling = std::min(surface, grade + config_.effort_rate_nm_s * static_cast<double>(n.rise_ticks) * period_s_);
    }
    return std::min(effort + config_.effort_rate_nm_s * period_s_, ceiling);
}

bool LaunchPlan::standstill(const Percepts &p) const {
    return std::abs(p.senses.speed_mps) <= config_.standstill_speed_mps &&
           p.crank_rpm <= config_.standstill_crank_rpm;
}

void LaunchPlan::arm(State &n, double lean_rad) const {
    n = State{};
    n.phase = LaunchPhase::hold;
    n.brake = 1.;
    n.lean_target_rad = lean_rad;
}

void LaunchPlan::to_release(State &n) const {
    n.phase = LaunchPhase::release;
    n.phase_ticks = 0;
    if (release_ticks_ == 0) {
        n.brake = 0.;
        n.phase = LaunchPhase::accelerate;
    }
}

void LaunchPlan::shifting(State &n, const Percepts &p, const ShiftFeedback &fb) const {
    if (n.shift_up && (fb.shift_count > n.request_count || fb.response == ShiftResponse::rejected)) {
        n.shift_up = false;
        n.blocked_teeth = n.request_teeth;
        // Launch part 4, decision 3: a rejected cog may be asked again after
        // shift_retry_s; a committed shift keeps it blocked until the felt cog changes.
        n.retry_tick = fb.response == ShiftResponse::rejected ? n.launch_ticks + retry_ticks_ : -1;
    }
    if (!n.shift_up && !fb.cut_active && p.crank_rpm >= config_.upshift_crank_rpm &&
        (p.senses.rear_teeth != n.blocked_teeth || (n.retry_tick >= 0 && n.launch_ticks >= n.retry_tick))) {
        n.shift_up = true;
        n.request_count = fb.shift_count;
        n.request_teeth = p.senses.rear_teeth;
        n.retry_tick = -1;
    }
}

LaunchCommand LaunchPlan::update(const LaunchInputs &in) {
    validate(in);
    if (!config_.plan_enabled) return command();
    if (!in.eligible) {
        s_ = State{};
        return command();
    }
    State n = s_;
    // The rise clock counts every eligible tick once started (probe: Rise.now).
    if (n.rise_ticks >= 0) ++n.rise_ticks;
    const std::optional<Percepts> &p = in.percepts;
    switch (n.phase) {
    case LaunchPhase::idle:
    case LaunchPhase::done:
        if (p && standstill(*p)) arm(n, prepare_launch(config_, p->senses.grade_ahead).lean_rad);
        break;
    case LaunchPhase::hold:
        if (!p || in.external_brake) break;   // the rider waits: no press, no clock
        ++n.phase_ticks;
        n.effort_nm = ramp(n.effort_nm, *p, n);
        if (p->human_crank_torque_nm >= release_torque_nm(config_, p->senses.grade_ahead)) {
            to_release(n);
        } else if (n.phase_ticks >= max_hold_ticks_) {
            n.preload_missed = true;
            to_release(n);
        }
        break;
    case LaunchPhase::release:
        ++n.phase_ticks;
        ++n.launch_ticks;
        if (p) n.effort_nm = ramp(n.effort_nm, *p, n);
        n.brake = std::max(0., 1. - static_cast<double>(n.phase_ticks) / static_cast<double>(release_ticks_));
        if (n.brake == 0.) {
            n.phase = LaunchPhase::accelerate;
            n.phase_ticks = 0;
        }
        break;
    case LaunchPhase::accelerate:
        ++n.launch_ticks;
        if (p) {
            n.effort_nm = ramp(n.effort_nm, *p, n);
            shifting(n, *p, in.shift);
            n.end_ok_ticks = p->senses.speed_mps >= config_.end_speed_mps && p->crank_rpm >= config_.end_crank_rpm
                                 ? std::min(n.end_ok_ticks + 1, end_hold_ticks_ + 1)
                                 : 0;
        }
        if (n.end_ok_ticks > end_hold_ticks_ || n.launch_ticks >= max_launch_ticks_) {
            n.phase = LaunchPhase::handover;
            n.shift_up = false;   // decision 5: the request is retired at the handover
        }
        break;
    case LaunchPhase::handover:
        if (p) n.effort_nm = ramp(n.effort_nm, *p, n);
        if (!in.shift.cut_active) {
            n.phase = LaunchPhase::blend;
            n.blend_ticks = 0;
            if (handover_ticks_ == 0) n = State{.phase = LaunchPhase::done};
        }
        break;
    case LaunchPhase::blend:
        if (p) n.effort_nm = ramp(n.effort_nm, *p, n);
        ++n.blend_ticks;
        if (n.blend_ticks >= handover_ticks_) n = State{.phase = LaunchPhase::done};
        break;
    }
    s_ = n;
    return command();
}

LaunchCommand LaunchPlan::command() const {
    LaunchCommand c{.phase = s_.phase, .brake = s_.brake, .effort_nm = s_.effort_nm,
                    .lean_target_rad = s_.lean_target_rad, .shift_up = s_.shift_up,
                    .preload_missed = s_.preload_missed};
    switch (s_.phase) {
    case LaunchPhase::idle:
    case LaunchPhase::done:
        c.weight = 0.;
        break;
    case LaunchPhase::blend:
        c.weight = 1. - static_cast<double>(s_.blend_ticks) / static_cast<double>(handover_ticks_);
        break;
    case LaunchPhase::hold:
    case LaunchPhase::release:
    case LaunchPhase::accelerate:
    case LaunchPhase::handover:
        c.weight = 1.;
        c.owns_shifting = true;
        break;
    }
    return c;
}

LaunchPlan::State LaunchPlan::state() const { return s_; }

void LaunchPlan::restore(const State &s) {
    const auto max_cap = std::max({config_.effort_cap_asphalt_nm, config_.effort_cap_hardpack_nm,
                                   config_.effort_cap_loose_nm, config_.effort_cap_wet_nm,
                                   config_.effort_cap_scree_nm});
    (void)launch_phase_name(s.phase);   // throws on a phase outside the enum
    if (s.phase_ticks < 0 || s.launch_ticks < 0 || s.end_ok_ticks < 0 || s.blend_ticks < 0 ||
        s.request_count < 0)
        bad_state("phase_ticks", "negative counter");
    if (!std::isfinite(s.effort_nm) || s.effort_nm < 0. || s.effort_nm > max_cap)
        bad_state("effort_nm", "outside [0, the largest cap]");
    if (!std::isfinite(s.lean_target_rad)) bad_state("lean_target_rad", "must be finite");
    if (s.request_teeth < 0 || s.blocked_teeth < 0) bad_state("request_teeth", "negative cog");
    if (s.shift_up && s.phase != LaunchPhase::accelerate) bad_state("shift_up", "a request outside accelerate");
    if (s.shift_up && s.request_teeth < 3) bad_state("request_teeth", "a request without its felt cog");
    if (s.rise_ticks < -1) bad_state("rise_ticks", "below -1");
    if (s.retry_tick < -1) bad_state("retry_tick", "below -1");
    if (s.retry_tick >= 0 &&
        (s.shift_up || s.blocked_teeth < 3 || s.retry_tick < retry_ticks_ || s.retry_tick > s.launch_ticks + retry_ticks_))
        bad_state("retry_tick", "a retry without the rejected request it follows");
    switch (s.phase) {
    case LaunchPhase::idle:
    case LaunchPhase::done:
        if (s != State{.phase = s.phase}) bad_state(launch_phase_name(s.phase), "an idle or done plan is reset");
        break;
    case LaunchPhase::hold:
        if (s.brake != 1.) bad_state("brake", "the hold brakes fully");
        if (s.phase_ticks >= max_hold_ticks_) bad_state("phase_ticks", "the hold is past its limit");
        if (s.launch_ticks != 0 || s.end_ok_ticks != 0 || s.blend_ticks != 0 || s.preload_missed ||
            s.request_count != 0 || s.request_teeth != 0 || s.blocked_teeth != 0)
            bad_state("hold", "a hold with launch counters, a request memory or an event");
        if (s.retry_tick != -1) bad_state("retry_tick", "a retry before accelerate");
        break;
    case LaunchPhase::release:
        if (release_ticks_ == 0 || s.phase_ticks >= release_ticks_ || s.launch_ticks != s.phase_ticks)
            bad_state("phase_ticks", "outside the release window");
        if (s.brake != std::max(0., 1. - static_cast<double>(s.phase_ticks) / static_cast<double>(release_ticks_)))
            bad_state("brake", "off the release ramp");
        if (s.end_ok_ticks != 0 || s.blend_ticks != 0) bad_state("release", "end or blend counters");
        if (s.retry_tick != -1) bad_state("retry_tick", "a retry before accelerate");
        break;
    case LaunchPhase::accelerate:
    case LaunchPhase::handover:
    case LaunchPhase::blend:
        if (s.brake != 0.) bad_state("brake", "released after the release");
        if (s.launch_ticks < release_ticks_) bad_state("launch_ticks", "shorter than the release");
        if (s.phase == LaunchPhase::accelerate &&
            (s.end_ok_ticks > end_hold_ticks_ || s.launch_ticks >= max_launch_ticks_))
            bad_state("end_ok_ticks", "accelerate past its end");
        if (s.phase == LaunchPhase::blend && (handover_ticks_ == 0 || s.blend_ticks >= handover_ticks_))
            bad_state("blend_ticks", "outside the handover window");
        if (s.phase != LaunchPhase::blend && s.blend_ticks != 0) bad_state("blend_ticks", "outside blend");
        break;
    }
    s_ = s;
}

} // namespace rider
