// launch_ride_metrics.hpp — launch part 4, task 6: C++ port of the
// calibration metrics helper (gradecap/harness/cal_metrics.py summarize with
// --window-handover --rb-from-release) and the decision-10 / P10-P13 verdict
// predicates shared by the three slow rider launch-ride test executables.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <functional>
#include <limits>
#include <numbers>
#include <optional>
#include <print>
#include <set>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include "launch_ride_support.hpp"
#include "runtime/control.hpp"
#include "runtime/samples.hpp"
#include "support/fixture_test.hpp"

namespace launch_ride {

struct LaunchCase {
    std::string name;
    double grade = 0.;
    std::string surface; // asphalt | wet | loose
};

struct LaunchMetrics {
    std::string outcome;  // runtime snapshot outcome or "none"
    bool crashed = false; // outcome != "none" (crash:loop_out, crash:* and the rest) — verdicts2.py:60-61
    std::optional<std::string> failure; // first model violation / invalid step, strict mode
    // Inside the launch window (hold entry .. handover entry, plan decision 10):
    std::vector<std::pair<double, double>> lifts; // nf <= 50 N longer than 0.05 s: start, length
    std::vector<std::pair<double, double>> slips; // util > 0.9 longer than 0.05 s
    double slip_peak = 0.;
    double rollback_m = 0.; // min x from the release entry - x at the release entry (<= 0)
    double leg_ema_peak_rpm = 0.;
    double saddle_min_n = 0., front_foot_min_n = 0., rear_foot_min_n = 0.; // rider_welds.*.normal_n minima
    bool budget_exceeded = false; // rider_effort_budget_exceeded > 0.5 on a window row (cal_metrics.py:378)
    bool balance_lost = false;    // rider_balance.balance_lost > 0.5 on a window row (:380)
    // Whole ride, not windowed (cal_metrics.py:359-362): the labels of the
    // string-array channels attachment_violations, rider_joint_power_violations,
    // rider_joint_speed_violations, rider_strength_violations.
    std::vector<std::string> violations;
    // Controller window, cal_metrics.py:341-353 literally: the lift start is the
    // first lift episode longer than 0.05 s, else the first nf <= 50 N episode
    // at all; the reaction's first acting tick is searched from the hold entry
    // over the whole ride; window = acting - lift when acting >= lift, else
    // crash time - lift when the ride crashed, else open (nullopt).
    std::optional<double> lift_start_s;
    std::optional<double> controller_window_s;
    // Whole ride from t = 0, not clipped (3.031 s after part 4b is full-run):
    std::optional<double> t20_s;
    std::optional<double> t20_from_release_s; // printed only (spec: second basis)
    std::optional<double> hold_s, release_s, accelerate_s, handover_s;
    // Post-handover & whole-ride 20 s uphill climbing checks (Queue Item 2):
    std::vector<std::pair<double, double>> post_lifts;
    double post_end_speed_mps = 0.;
    double whole_ride_saddle_min_n = 0.;
    double whole_ride_front_foot_min_n = 0.;
    double whole_ride_rear_foot_min_n = 0.;
    bool whole_ride_balance_lost = false;
};

struct RestartMetrics {
    LaunchMetrics launch;
    double pre_stop_cruise_min_speed_mps = 0.;
    int pre_stop_rear_teeth = 0;
    int hold_rear_teeth_at_release = 0;
    double min_abs_speed_during_stop_mps = std::numeric_limits<double>::infinity();
    double post_restart_max_speed_mps = 0.;
    double stop_min_front_normal_n = std::numeric_limits<double>::quiet_NaN();
};

namespace detail {

struct IntervalMetricsRow {
    double time_s = 0.;
    double x_m = 0.;
    double speed_mps = 0.;
    double front_normal_n = std::numeric_limits<double>::quiet_NaN();
    double rear_normal_n = std::numeric_limits<double>::quiet_NaN();
    double rear_tangent_n = std::numeric_limits<double>::quiet_NaN();
    double rear_mu = std::numeric_limits<double>::quiet_NaN();
    double crank_rad_s = 0.;
    double saddle_normal_n = std::numeric_limits<double>::quiet_NaN();
    double front_pedal_normal_n = std::numeric_limits<double>::quiet_NaN();
    double rear_pedal_normal_n = std::numeric_limits<double>::quiet_NaN();
    double rear_teeth = std::numeric_limits<double>::quiet_NaN();
    double budget_exceeded = 0.;
    double balance_lost = 0.;
    std::string launch_phase;
    std::string reaction_phase;
};

struct Episode {
    double start_s = 0.;
    double end_s = 0.;
    std::size_t start_idx = 0;
    std::size_t end_idx = 0;
};

struct PhaseTransition {
    double time_s = 0.;
    std::string to;
};

[[nodiscard]] inline double lookup_col(const runtime::FlatColumns &cols,
                                       std::string_view key,
                                       double fallback = std::numeric_limits<double>::quiet_NaN()) {
    const auto it = std::ranges::lower_bound(
        cols, key, std::less<>{},
        [](const auto &entry) -> std::string_view { return entry.first; });
    if (it != cols.end() && it->first == key) return it->second;
    return fallback;
}

[[nodiscard]] inline std::string channel_phase(
    const runtime::WireObject &channels, std::string_view section_key) {
    const runtime::Wire *sec = runtime::sample_wire::find(channels, section_key);
    if (sec == nullptr) return {};
    const auto *obj = std::get_if<runtime::WireObject>(&sec->value);
    if (obj == nullptr) return {};
    const runtime::Wire *field = runtime::sample_wire::find(*obj, "phase");
    if (field == nullptr) return {};
    const auto *text = std::get_if<std::string>(&field->value);
    return text != nullptr ? *text : std::string{};
}

inline void collect_violations(const runtime::WireObject &channels,
                               std::set<std::string> &out) {
    constexpr std::array<std::string_view, 4> kChannels{
        "attachment_violations",
        "rider_joint_power_violations",
        "rider_joint_speed_violations",
        "rider_strength_violations",
    };
    const auto add_tokens = [&](std::string_view text) {
        std::size_t start = 0;
        while (start <= text.size()) {
            const std::size_t bar = text.find('|', start);
            const std::size_t len =
                bar == std::string_view::npos ? text.size() - start : bar - start;
            if (len > 0) out.emplace(text.substr(start, len));
            if (bar == std::string_view::npos) break;
            start = bar + 1;
        }
    };
    for (const std::string_view key : kChannels) {
        const runtime::Wire *w = runtime::sample_wire::find(channels, key);
        if (w == nullptr) continue;
        if (const auto *s = std::get_if<std::string>(&w->value)) {
            add_tokens(*s);
        } else if (const auto *arr = std::get_if<runtime::WireArray>(&w->value)) {
            for (const auto &item : *arr) {
                if (const auto *elem = std::get_if<std::string>(&item.value))
                    add_tokens(*elem);
            }
        }
    }
}

template <typename Accessor>
[[nodiscard]] inline std::vector<PhaseTransition> phase_transitions(
    std::span<const IntervalMetricsRow> rows, const Accessor &field) {
    std::vector<PhaseTransition> out;
    std::optional<std::string_view> prev;
    for (const auto &row : rows) {
        const std::string_view current = field(row);
        if (prev.has_value() && current != *prev)
            out.push_back({.time_s = row.time_s, .to = std::string(current)});
        prev = current;
    }
    return out;
}

template <typename Pred>
[[nodiscard]] inline std::vector<Episode> find_episodes(
    std::span<const IntervalMetricsRow> rows, const Pred &pred) {
    std::vector<Episode> out;
    std::optional<std::size_t> cur;
    for (std::size_t i = 0; i < rows.size(); ++i) {
        const bool m = pred(rows[i]);
        if (m && !cur.has_value()) {
            cur = i;
        } else if (!m && cur.has_value()) {
            out.push_back({.start_s = rows[*cur].time_s,
                           .end_s = rows[i - 1].time_s,
                           .start_idx = *cur,
                           .end_idx = i - 1});
            cur.reset();
        }
    }
    if (cur.has_value() && !rows.empty()) {
        const std::size_t last = rows.size() - 1;
        out.push_back({.start_s = rows[*cur].time_s,
                       .end_s = rows[last].time_s,
                       .start_idx = *cur,
                       .end_idx = last});
    }
    return out;
}

[[nodiscard]] inline std::optional<double> find_hold_entry(
    std::span<const IntervalMetricsRow> rows,
    std::span<const PhaseTransition> transitions) {
    for (const auto &tr : transitions) {
        if (tr.to == "hold" && tr.time_s >= -1e-9) return tr.time_s;
    }
    for (const auto &tr : transitions) {
        if (tr.to == "hold") return tr.time_s;
    }
    for (const auto &row : rows) {
        if (row.launch_phase == "hold") return row.time_s;
    }
    return std::nullopt;
}

[[nodiscard]] inline double window_col_min(
    std::span<const IntervalMetricsRow> rows, std::size_t i0,
    auto &&getter) {
    double best = std::numeric_limits<double>::infinity();
    bool any = false;
    for (std::size_t i = i0; i < rows.size(); ++i) {
        const double v = getter(rows[i]);
        if (!std::isnan(v)) {
            best = std::min(best, v);
            any = true;
        }
    }
    return any ? best : std::numeric_limits<double>::quiet_NaN();
}

struct RideSpec {
    double track_length_m = 150.;
    double duration_s = 20.;
};

struct RunOutput {
    std::vector<IntervalMetricsRow> rows;
    double snap_time_s = 0.;
};

template <typename BrakeFn>
[[nodiscard]] inline RunOutput run_ride(
    std::string_view profile, const LaunchCase &c, const RideSpec &spec,
    LaunchMetrics &m, const BrakeFn &brake_at) {
    const std::string knots = std::format(
        "[[0.0, {:.6f}], [{:.1f}, {:.6f}]]", c.grade, spec.track_length_m, c.grade);
    auto env = launch_environment(
        profile, track(knots, spec.track_length_m, c.surface), spec.duration_s);

    const runtime::RideControl control;
    std::string abort_msg;
    try {
        while (!env.runtime->snapshot().outcome.has_value() &&
               env.runtime->snapshot().step < env.max_steps) {
            const auto [front_brake, rear_brake] =
                brake_at(env.runtime->snapshot().time_s);
            const auto result = env.runtime->advance(
                env.runtime->snapshot().step + env.control_steps,
                control, front_brake, rear_brake, std::nullopt);
            if (result.reason == "outcome") break;
        }
        env.runtime->flush();
    } catch (const std::exception &e) {
        abort_msg = e.what();
    }

    const auto snap = env.runtime->snapshot();
    m.outcome = snap.outcome.value_or("none");
    m.crashed = m.outcome != "none";
    if (snap.first_failure.has_value()) {
        m.failure = snap.first_failure->message();
    } else if (!abort_msg.empty()) {
        m.failure = abort_msg;
    }

    runtime::ColumnLayout layout;
    layout.root_x_qpos = static_cast<std::size_t>(env.root_x_qpos);
    layout.root_x_dof = static_cast<std::size_t>(env.root_x_dof);
    layout.root_pitch_qpos = static_cast<std::size_t>(env.root_pitch_qpos);

    const auto recorded = env.runtime->recorded_intervals();
    RunOutput out;
    out.snap_time_s = snap.time_s;
    out.rows.reserve(recorded.size());
    std::set<std::string> violation_set;

    for (const auto &sample : recorded) {
        collect_violations(sample->channels, violation_set);
        const auto flat = sample->flat_columns(layout);
        IntervalMetricsRow row;
        row.time_s = lookup_col(flat, "time_s", sample->time_s);
        row.x_m = lookup_col(flat, "x_m", 0.);
        row.speed_mps = lookup_col(flat, "speed_mps", 0.);
        row.front_normal_n = lookup_col(flat, "tires.front.normal_load_n");
        row.rear_normal_n = lookup_col(flat, "tires.rear.normal_load_n");
        row.rear_tangent_n = lookup_col(flat, "tires.rear.tangent_force_n");
        row.rear_mu = lookup_col(flat, "tires.rear.friction_coefficient");
        row.crank_rad_s = lookup_col(flat, "sensors.encoders_rad_s.crank", 0.);
        row.saddle_normal_n = lookup_col(flat, "rider_welds.saddle.normal_n");
        row.front_pedal_normal_n = lookup_col(flat, "rider_welds.front_pedal.normal_n");
        row.rear_pedal_normal_n = lookup_col(flat, "rider_welds.rear_pedal.normal_n");
        row.rear_teeth = lookup_col(flat, "drive.gear_rear_teeth");
        row.budget_exceeded = lookup_col(flat, "rider_effort_budget_exceeded", 0.);
        row.balance_lost = lookup_col(flat, "rider_balance.balance_lost", 0.);
        row.launch_phase = channel_phase(sample->channels, "rider_launch");
        row.reaction_phase = channel_phase(sample->channels, "rider_reaction");
        out.rows.push_back(std::move(row));
    }
    m.violations.assign(violation_set.begin(), violation_set.end());
    env.runtime->close();
    return out;
}

[[nodiscard]] inline double rear_utilization(const IntervalMetricsRow &r) {
    if (!std::isnan(r.rear_normal_n) && !std::isnan(r.rear_mu) &&
        !std::isnan(r.rear_tangent_n) && r.rear_normal_n > 1. && r.rear_mu > 0.) {
        return std::abs(r.rear_tangent_n) / (r.rear_mu * r.rear_normal_n);
    }
    return std::numeric_limits<double>::quiet_NaN();
}

[[nodiscard]] inline std::size_t index_after_time(
    std::span<const IntervalMetricsRow> rows, double time_s) {
    for (std::size_t i = 0; i < rows.size(); ++i) {
        if (rows[i].time_s > time_s) return i;
    }
    return rows.size();
}

[[nodiscard]] inline std::size_t index_at_or_after(
    std::span<const IntervalMetricsRow> rows, double time_s) {
    for (std::size_t i = 0; i < rows.size(); ++i) {
        if (rows[i].time_s >= time_s) return i;
    }
    return rows.size();
}

struct WindowInput {
    std::span<const IntervalMetricsRow> win_rows;
    std::span<const IntervalMetricsRow> episode_rows;
    std::size_t i_start = 0;
    std::size_t ep_min_end_idx = 0;
    std::size_t i_rb = 0;
};

[[nodiscard]] inline std::vector<Episode> populate_window_metrics(
    const WindowInput &in, LaunchMetrics &m) {
    const auto win_rows = in.win_rows;
    const std::size_t i_start = in.i_start;
    const std::size_t i_rb = in.i_rb;

    // Wheel lift: front normal load <= 50 N longer than 0.05 s.
    const auto eps50_all = find_episodes(
        in.episode_rows, [](const auto &r) { return r.front_normal_n <= 50.; });
    std::vector<Episode> eps50;
    for (const auto &ep : eps50_all) {
        if (ep.end_idx < in.ep_min_end_idx) continue;
        eps50.push_back(ep);
        if (ep.end_s - ep.start_s > 0.05) {
            m.lifts.emplace_back(ep.start_s, ep.end_s - ep.start_s);
        }
    }

    // Rear slip: |tangent| / (mu * N) > 0.9 longer than 0.05 s when N > 1 and mu > 0.
    const auto slip_all = find_episodes(in.episode_rows, [](const auto &r) {
        const double u = rear_utilization(r);
        return !std::isnan(u) && u > 0.9;
    });
    for (const auto &ep : slip_all) {
        if (ep.end_idx >= in.ep_min_end_idx && ep.end_s - ep.start_s > 0.05) {
            m.slips.emplace_back(ep.start_s, ep.end_s - ep.start_s);
        }
    }
    for (std::size_t i = i_start; i < win_rows.size(); ++i) {
        const double u = rear_utilization(win_rows[i]);
        if (!std::isnan(u)) m.slip_peak = std::max(m.slip_peak, u);
    }

    // Rollback from release entry (or fallback index).
    double min_x = win_rows[i_rb].x_m;
    for (std::size_t i = i_rb; i < win_rows.size(); ++i) {
        min_x = std::min(min_x, win_rows[i].x_m);
    }
    m.rollback_m = min_x - win_rows[i_rb].x_m;

    // Leg EMA cadence (tau = 0.35 s) over clipped rows from t = 0; peak from i_start.
    double ema = 0.;
    for (std::size_t i = 0; i < win_rows.size(); ++i) {
        const double rpm =
            std::abs(win_rows[i].crank_rad_s * (60. / (2. * std::numbers::pi)));
        if (i == 0) {
            ema = rpm;
        } else {
            const double dt = win_rows[i].time_s - win_rows[i - 1].time_s;
            ema += std::min(1., dt / .35) * (rpm - ema);
        }
        if (i >= i_start) m.leg_ema_peak_rpm = std::max(m.leg_ema_peak_rpm, ema);
    }

    // Supports, effort budget, balance loss inside [i_start, win_rows.size()).
    m.saddle_min_n = window_col_min(
        win_rows, i_start, [](const auto &r) { return r.saddle_normal_n; });
    m.front_foot_min_n = window_col_min(
        win_rows, i_start, [](const auto &r) { return r.front_pedal_normal_n; });
    m.rear_foot_min_n = window_col_min(
        win_rows, i_start, [](const auto &r) { return r.rear_pedal_normal_n; });
    for (std::size_t i = i_start; i < win_rows.size(); ++i) {
        if (win_rows[i].budget_exceeded > .5) m.budget_exceeded = true;
        if (win_rows[i].balance_lost > .5) m.balance_lost = true;
    }
    return eps50;
}

struct ControllerWindowInput {
    std::span<const IntervalMetricsRow> all_rows;
    std::span<const Episode> eps50;
    double t_search_from = 0.;
    double snap_time_s = 0.;
};

inline void populate_controller_window(
    const ControllerWindowInput &in, LaunchMetrics &m) {
    if (!m.lifts.empty()) {
        m.lift_start_s = m.lifts.front().first;
    } else if (!in.eps50.empty()) {
        m.lift_start_s = in.eps50.front().start_s;
    }
    if (m.lift_start_s) {
        const auto react_tr = phase_transitions(
            in.all_rows, [](const auto &r) -> std::string_view { return r.reaction_phase; });
        std::optional<double> t_act;
        for (const auto &tr : react_tr) {
            if (tr.time_s >= in.t_search_from - 1e-9 && tr.to == "acting") {
                t_act = tr.time_s;
                break;
            }
        }
        if (t_act && *t_act >= *m.lift_start_s) {
            m.controller_window_s = *t_act - *m.lift_start_s;
        } else if (m.crashed) {
            m.controller_window_s = in.snap_time_s - *m.lift_start_s;
        }
    }
}

inline void populate_post_handover_metrics(
    std::span<const IntervalMetricsRow> all_rows,
    std::size_t i_start,
    std::size_t clip_end,
    LaunchMetrics &m) {
    if (all_rows.empty()) return;
    m.post_end_speed_mps = all_rows.back().speed_mps;
    m.whole_ride_saddle_min_n = window_col_min(
        all_rows, i_start, [](const auto &r) { return r.saddle_normal_n; });
    m.whole_ride_front_foot_min_n = window_col_min(
        all_rows, i_start, [](const auto &r) { return r.front_pedal_normal_n; });
    m.whole_ride_rear_foot_min_n = window_col_min(
        all_rows, i_start, [](const auto &r) { return r.rear_pedal_normal_n; });
    for (std::size_t i = i_start; i < all_rows.size(); ++i) {
        if (all_rows[i].balance_lost > .5) {
            m.whole_ride_balance_lost = true;
            break;
        }
    }
    if (m.handover_s.has_value() && clip_end < all_rows.size()) {
        const auto post_eps = find_episodes(
            all_rows.subspan(clip_end),
            [](const auto &r) { return r.front_normal_n <= 50.; });
        for (const auto &ep : post_eps) {
            if (ep.end_s - ep.start_s > 0.05) {
                m.post_lifts.emplace_back(ep.start_s, ep.end_s - ep.start_s);
            }
        }
    }
}

} // namespace detail

// Rides `profile` on a 150 m constant-grade track of `c` for 20 s, the
// calibration probe's drive (caller control empty, no caller brake).
[[nodiscard]] inline LaunchMetrics ride(std::string_view profile,
                                          const LaunchCase &c) {
    constexpr double kTrackLengthM = 150.;
    constexpr double kDurationS = 20.;
    constexpr double kTwentyKmhMps = 20. / 3.6;

    LaunchMetrics m;
    const auto run = detail::run_ride(
        profile, c, {.track_length_m = kTrackLengthM, .duration_s = kDurationS}, m,
        [](double) { return std::pair{0., 0.}; });
    const auto &all_rows = run.rows;
    if (all_rows.empty()) return m;

    const auto full_lp = detail::phase_transitions(
        all_rows, [](const auto &r) -> std::string_view { return r.launch_phase; });
    const auto full_hold = detail::find_hold_entry(all_rows, full_lp);
    const double th0 = full_hold.value_or(0.);
    std::optional<double> dn0;
    for (const auto &tr : full_lp) {
        if (tr.to == "handover" && tr.time_s >= th0 - 1e-9) {
            dn0 = tr.time_s;
            break;
        }
    }
    const std::size_t clip_end =
        dn0.has_value() ? detail::index_after_time(all_rows, *dn0) : all_rows.size();
    const auto win_rows = std::span<const detail::IntervalMetricsRow>(all_rows).first(clip_end);
    if (win_rows.empty()) return m;

    const auto win_lp = detail::phase_transitions(
        win_rows, [](const auto &r) -> std::string_view { return r.launch_phase; });
    m.hold_s = detail::find_hold_entry(win_rows, win_lp);
    const double t_hold = m.hold_s.value_or(0.);

    std::size_t i0 = detail::index_at_or_after(win_rows, t_hold);
    if (i0 >= win_rows.size()) i0 = 0;

    for (const auto &tr : win_lp) {
        if (tr.time_s < t_hold - 1e-9) continue;
        if (tr.to == "release" && !m.release_s) m.release_s = tr.time_s;
        else if (tr.to == "accelerate" && !m.accelerate_s) m.accelerate_s = tr.time_s;
        else if (tr.to == "handover" && !m.handover_s) m.handover_s = tr.time_s;
    }

    std::size_t i_rb = i0;
    if (m.release_s) {
        const std::size_t found = detail::index_at_or_after(win_rows, *m.release_s);
        if (found < win_rows.size()) i_rb = found;
    }

    const auto eps50 = detail::populate_window_metrics(
        {.win_rows = win_rows,
         .episode_rows = win_rows,
         .i_start = i0,
         .ep_min_end_idx = i0,
         .i_rb = i_rb},
        m);
    detail::populate_controller_window(
        {.all_rows = all_rows,
         .eps50 = eps50,
         .t_search_from = t_hold,
         .snap_time_s = run.snap_time_s},
        m);
    detail::populate_post_handover_metrics(all_rows, i0, clip_end, m);

    // Whole-ride t20 from t = 0 (unclipped) and second printed basis from accelerate entry.
    for (std::size_t i = i0; i < all_rows.size(); ++i) {
        if (all_rows[i].speed_mps >= kTwentyKmhMps) {
            m.t20_s = all_rows[i].time_s;
            break;
        }
    }
    if (m.t20_s && m.accelerate_s) m.t20_from_release_s = *m.t20_s - *m.accelerate_s;

    return m;
}

// Rides `profile` on a 200 m constant-grade track for 25 s, cruises until 10 s,
// smoothly ramps both brakes 0 -> 0.5 over 0.5 s, holds until 12.5 s, releases,
// and measures the restart cleanly inside [t_second_release, t_second_handover].
[[nodiscard]] inline RestartMetrics ride_restart(
    std::string_view profile, double grade, std::string_view surface = "asphalt") {
    static constexpr double kTrackLengthM = 200.;
    static constexpr double kDurationS = 25.;
    static constexpr double kBrakeStartS = 10.0;
    static constexpr double kBrakeRampEndS = 10.5;
    static constexpr double kBrakeReleaseS = 12.5;
    static constexpr double kBrakeLevel = 0.5;

    RestartMetrics rm;
    const LaunchCase c{
        .name = std::format("restart_{:02d}", static_cast<int>(std::llround(grade * 100.))),
        .grade = grade,
        .surface = std::string(surface),
    };
    const auto run = detail::run_ride(
        profile, c, {.track_length_m = kTrackLengthM, .duration_s = kDurationS},
        rm.launch,
        [](double t_s) -> std::pair<double, double> {
            if (t_s < kBrakeStartS || t_s >= kBrakeReleaseS) return {0., 0.};
            if (t_s < kBrakeRampEndS) {
                const double b =
                    kBrakeLevel * ((t_s - kBrakeStartS) / (kBrakeRampEndS - kBrakeStartS));
                return {b, b};
            }
            return {kBrakeLevel, kBrakeLevel};
        });
    const auto &all_rows = run.rows;
    if (all_rows.empty()) return rm;

    bool have_cruise = false;
    double cruise_min = std::numeric_limits<double>::infinity();
    for (const auto &row : all_rows) {
        if (row.time_s >= 9.0 && row.time_s <= kBrakeStartS) {
            have_cruise = true;
            cruise_min = std::min(cruise_min, row.speed_mps);
            if (!std::isnan(row.rear_teeth)) {
                rm.pre_stop_rear_teeth = static_cast<int>(std::llround(row.rear_teeth));
            }
        }
    }
    if (have_cruise) rm.pre_stop_cruise_min_speed_mps = cruise_min;

    const auto full_lp = detail::phase_transitions(
        all_rows, [](const auto &r) -> std::string_view { return r.launch_phase; });
    for (const auto &tr : full_lp) {
        if (tr.time_s < kBrakeStartS - 1e-9) continue;
        if (tr.to == "hold" && !rm.launch.hold_s) {
            rm.launch.hold_s = tr.time_s;
        } else if (rm.launch.hold_s && tr.to == "release" && !rm.launch.release_s) {
            rm.launch.release_s = tr.time_s;
        } else if (rm.launch.hold_s && tr.to == "accelerate" && !rm.launch.accelerate_s) {
            rm.launch.accelerate_s = tr.time_s;
        } else if ((rm.launch.release_s || rm.launch.accelerate_s) &&
                   tr.to == "handover" && !rm.launch.handover_s) {
            rm.launch.handover_s = tr.time_s;
        }
    }

    // Profiles with brake_release_s == 0 transition straight from hold to
    // accelerate without a release phase; normal uses brake_release_s > 0.
    const std::optional<double> release_or_accel_s =
        rm.launch.release_s ? rm.launch.release_s : rm.launch.accelerate_s;
    const double stop_end_s = release_or_accel_s.value_or(kDurationS);
    for (const auto &row : all_rows) {
        if (row.time_s >= kBrakeStartS && row.time_s <= stop_end_s) {
            rm.min_abs_speed_during_stop_mps =
                std::min(rm.min_abs_speed_during_stop_mps, std::abs(row.speed_mps));
            if (!std::isnan(row.front_normal_n)) {
                rm.stop_min_front_normal_n = std::isnan(rm.stop_min_front_normal_n)
                                                 ? row.front_normal_n
                                                 : std::min(rm.stop_min_front_normal_n,
                                                            row.front_normal_n);
            }
        }
    }

    if (!release_or_accel_s) return rm;

    const std::size_t clip_end =
        rm.launch.handover_s.has_value()
            ? detail::index_after_time(all_rows, *rm.launch.handover_s)
            : all_rows.size();
    const auto win_rows =
        std::span<const detail::IntervalMetricsRow>(all_rows).first(clip_end);
    const std::size_t i_release = detail::index_at_or_after(win_rows, *release_or_accel_s);
    if (i_release >= win_rows.size()) return rm;

    if (!std::isnan(win_rows[i_release].rear_teeth)) {
        rm.hold_rear_teeth_at_release =
            static_cast<int>(std::llround(win_rows[i_release].rear_teeth));
    }

    for (std::size_t i = i_release; i < all_rows.size(); ++i) {
        rm.post_restart_max_speed_mps =
            std::max(rm.post_restart_max_speed_mps, all_rows[i].speed_mps);
    }

    const auto eps50 = detail::populate_window_metrics(
        {.win_rows = win_rows,
         .episode_rows = win_rows.subspan(i_release),
         .i_start = i_release,
         .ep_min_end_idx = 0,
         .i_rb = i_release},
        rm.launch);
    detail::populate_controller_window(
        {.all_rows = all_rows,
         .eps50 = eps50,
         .t_search_from = *release_or_accel_s,
         .snap_time_s = run.snap_time_s},
        rm.launch);
    detail::populate_post_handover_metrics(all_rows, i_release, clip_end, rm.launch);

    return rm;
}

// The seated limits every launch keeps (spec «Проверки», roadmap task 13,
// verdicts2.py:31-61): over the ride no crash, no strict-mode failure and no
// violation label; inside the window no support pull beyond -6 N (saddle and
// both feet), no effort-budget excess, no balance loss.
[[nodiscard]] inline std::vector<std::string> seated_limit_failures(
    const LaunchCase &c, const LaunchMetrics &m) {
    std::vector<std::string> out;
    if (m.crashed || m.failure) {
        out.push_back(c.name + ": " + m.outcome +
                      (m.failure ? " " + *m.failure : ""));
    }
    if (std::isnan(m.saddle_min_n) || std::isnan(m.front_foot_min_n) ||
        std::isnan(m.rear_foot_min_n) || m.saddle_min_n < -6. ||
        m.front_foot_min_n < -6. || m.rear_foot_min_n < -6.) {
        out.push_back(c.name + ": a saddle or pedal support below -6 N");
    }
    if (m.budget_exceeded) out.push_back(c.name + ": rider effort budget exceeded");
    if (m.balance_lost) out.push_back(c.name + ": balance lost");
    for (const auto &v : m.violations) out.push_back(c.name + ": violation " + v);
    return out;
}

// The approved thresholds (spec «Проверки», plan decision 10): the seated
// limits, and inside the launch window no lift, no wheelspin, rollback from
// release at most 10 mm, leg cadence at most 110 rpm.
[[nodiscard]] inline std::vector<std::string> clean_start_failures(
    const LaunchCase &c, const LaunchMetrics &m) {
    std::vector<std::string> out = seated_limit_failures(c, m);
    if (!m.lifts.empty()) out.push_back(c.name + ": front lift in the launch window");
    if (!m.slips.empty()) out.push_back(c.name + ": rear wheelspin in the launch window");
    if (m.rollback_m < -.010) out.push_back(c.name + ": rollback beyond 10 mm");
    if (m.leg_ema_peak_rpm > 110.) out.push_back(c.name + ": leg cadence above 110 rpm");
    return out;
}

// 20 % (and careful 20 % under the normal rule, P12): the seated limits,
// every in-window lift at most 0.3 s.
[[nodiscard]] inline std::vector<std::string> short_lift_failures(
    const LaunchCase &c, const LaunchMetrics &m) {
    std::vector<std::string> out = seated_limit_failures(c, m);
    for (const auto &[start, length] : m.lifts) {
        if (length > .3) {
            out.push_back(c.name + ": a lift longer than 0.3 s at " +
                          std::to_string(start));
        }
    }
    return out;
}

// The aggressive rider (P10, spec «Проверки»): the seated limits, and when
// the front lifts the controller window (cal_metrics.py:341-353) is at least
// 0.4 s; an open window passes.
[[nodiscard]] inline std::vector<std::string> controller_window_failures(
    const LaunchCase &c, const LaunchMetrics &m) {
    std::vector<std::string> out = seated_limit_failures(c, m);
    if (m.lift_start_s && m.controller_window_s && *m.controller_window_s < .4) {
        out.push_back(c.name + ": controller window " +
                      std::to_string(*m.controller_window_s) + " s");
    }
    return out;
}

// 22% / 25% 20 s uphill acceptance (Queue Item 2): seated limits over the
// full 20 s horizon (no crash, no violation, whole-ride supports >= -6 N, no
// whole-ride balance loss), zero post-handover lifts, handover within 7 s,
// and steady 20 s climb speed >= 2.8 m/s.
[[nodiscard]] inline std::vector<std::string> steep_20s_failures(
    const LaunchCase &c, const LaunchMetrics &m) {
    std::vector<std::string> out = seated_limit_failures(c, m);
    if (std::isnan(m.whole_ride_saddle_min_n) ||
        std::isnan(m.whole_ride_front_foot_min_n) ||
        std::isnan(m.whole_ride_rear_foot_min_n) ||
        m.whole_ride_saddle_min_n < -6. ||
        m.whole_ride_front_foot_min_n < -6. ||
        m.whole_ride_rear_foot_min_n < -6.) {
        out.push_back(c.name + ": whole-ride saddle or pedal support below -6 N");
    }
    if (m.whole_ride_balance_lost) {
        out.push_back(c.name + ": whole-ride balance lost");
    }
    if (!m.post_lifts.empty()) {
        out.push_back(c.name + ": post-handover front lift");
    }
    if (!m.handover_s.has_value() || *m.handover_s > 7.0) {
        out.push_back(c.name + ": handover missing or > 7.0 s");
    }
    if (!(m.post_end_speed_mps >= 2.8)) {
        out.push_back(std::format("{}: 20 s end speed {:.3f} < 2.8 m/s",
                                  c.name, m.post_end_speed_mps));
    }
    return out;
}

enum class LaunchRule : std::uint8_t {
    clean,
    clean_with_t20,
    short_lift,
    controller_window,
    seated_only,
    steep_20s,
    info,
};

[[nodiscard]] inline std::string_view rule_label(LaunchRule rule) {
    switch (rule) {
    case LaunchRule::clean: return "clean";
    case LaunchRule::clean_with_t20: return "clean+t20";
    case LaunchRule::short_lift: return "short_lift";
    case LaunchRule::controller_window: return "ctrl_win";
    case LaunchRule::seated_only: return "seated";
    case LaunchRule::steep_20s: return "steep_20s";
    case LaunchRule::info: return "info";
    }
    return "?";
}

[[nodiscard]] inline std::vector<std::string> evaluate_rule(
    LaunchRule rule, const LaunchCase &c, const LaunchMetrics &m) {
    switch (rule) {
    case LaunchRule::clean:
        return clean_start_failures(c, m);
    case LaunchRule::clean_with_t20: {
        auto out = clean_start_failures(c, m);
        if (!m.t20_s || *m.t20_s > 3.25) {
            out.push_back(c.name + ": t20_s " +
                          (m.t20_s ? std::format("{:.5f}", *m.t20_s) : "n/a") +
                          " > 3.25 s");
        }
        return out;
    }
    case LaunchRule::short_lift:
        return short_lift_failures(c, m);
    case LaunchRule::controller_window:
        return controller_window_failures(c, m);
    case LaunchRule::seated_only:
        return seated_limit_failures(c, m);
    case LaunchRule::steep_20s:
        return steep_20s_failures(c, m);
    case LaunchRule::info:
        return {};
    }
    return {};
}

[[nodiscard]] inline std::vector<LaunchCase> standard_launch_cases() {
    std::vector<LaunchCase> cases{
        {.name = "g00", .grade = .00, .surface = "asphalt"},
        {.name = "g05", .grade = .05, .surface = "asphalt"},
        {.name = "g08", .grade = .08, .surface = "asphalt"},
        {.name = "g10", .grade = .10, .surface = "asphalt"},
        {.name = "g12", .grade = .12, .surface = "asphalt"},
        {.name = "g15", .grade = .15, .surface = "asphalt"},
        {.name = "g20", .grade = .20, .surface = "asphalt"},
        {.name = "wet10", .grade = .10, .surface = "wet"},
        {.name = "loose10", .grade = .10, .surface = "loose"},
    };
    for (const auto &[tag, grade] : {std::pair{"07", .07}, std::pair{"09", .09},
                                     std::pair{"11", .11}, std::pair{"14", .14},
                                     std::pair{"17", .17}}) {
        cases.push_back({.name = std::string("g") + tag, .grade = grade, .surface = "asphalt"});
        cases.push_back({.name = std::string("wet") + tag, .grade = grade, .surface = "wet"});
        cases.push_back({.name = std::string("loose") + tag, .grade = grade, .surface = "loose"});
    }
    cases.push_back({.name = "g22", .grade = .22, .surface = "asphalt"});
    cases.push_back({.name = "g25", .grade = .25, .surface = "asphalt"});
    cases.push_back({.name = "loose25", .grade = .25, .surface = "loose"});
    return cases;
}

namespace detail {

[[nodiscard]] inline std::string format_episodes(
    std::span<const std::pair<double, double>> eps) {
    if (eps.empty()) return "0";
    std::string out = std::format("{}(", eps.size());
    for (std::size_t i = 0; i < eps.size(); ++i) {
        if (i) out += ';';
        out += std::format("{:.3f}@{:.3f}", eps[i].second, eps[i].first);
    }
    out += ')';
    return out;
}

[[nodiscard]] inline std::string format_opt(std::optional<double> v,
                                            int decimals = 3) {
    if (!v.has_value()) return "n/a";
    if (decimals == 5) return std::format("{:.5f}", *v);
    return std::format("{:.3f}", *v);
}

[[nodiscard]] inline std::string format_window(const LaunchMetrics &m) {
    if (!m.lift_start_s.has_value()) return "no_lift";
    if (!m.controller_window_s.has_value())
        return std::format("open@{:.3f}", *m.lift_start_s);
    return std::format("{:.3f}@{:.3f}", *m.controller_window_s, *m.lift_start_s);
}

[[nodiscard]] inline std::string join_failures(std::span<const std::string> items) {
    std::string joined;
    for (std::size_t i = 0; i < items.size(); ++i) {
        if (i) joined += "; ";
        joined += items[i];
    }
    return joined;
}

} // namespace detail

template <typename RuleFn>
inline void run_launch_suite(std::string_view rider_label,
                             std::string_view profile,
                             const RuleFn &rule_for) {
    const auto cases = standard_launch_cases();
    std::vector<std::string> all_failures;
    std::println(stderr,
                 "=== launch rides: {} ({}) ===", rider_label, profile);
    std::println(
        stderr,
        "{:<8} {:<10} {:<14} {:<16} {:<10} {:>7} {:>8} {:>7} {:>20} {:>16} {:>17} {:<24} {}",
        "case", "rule", "outcome", "lifts", "slips", "slip_pk", "rb_mm",
        "cad_rpm", "sad/fF/fR_N", "ctrl_win", "t20(t0/rel)",
        "phases(h/r/a/ho)", "verdict");
    for (const auto &c : cases) {
        const LaunchMetrics m = ride(profile, c);
        const LaunchRule rule = rule_for(c);
        auto failures = evaluate_rule(rule, c, m);
        const std::string verdict =
            rule == LaunchRule::info
                ? "INFO"
                : (failures.empty() ? "PASS" : "FAIL: " + detail::join_failures(failures));
        for (auto &f : failures) all_failures.push_back(std::move(f));

        const std::string outcome_str =
            m.outcome + (m.failure ? "!" : "") +
            (m.budget_exceeded ? "+bud" : "") + (m.balance_lost ? "+bal" : "") +
            (!m.violations.empty() ? "+viol" : "");
        const std::string lifts_str = detail::format_episodes(m.lifts);
        const std::string slips_str = detail::format_episodes(m.slips);
        const std::string supports_str = std::format(
            "{:+.1f}/{:+.1f}/{:+.1f}", m.saddle_min_n, m.front_foot_min_n,
            m.rear_foot_min_n);
        const std::string win_str = detail::format_window(m);
        const std::string t20_str = std::format(
            "{}/{}", detail::format_opt(m.t20_s, 5),
            detail::format_opt(m.t20_from_release_s, 5));
        const std::string phases_str = std::format(
            "{}/{}/{}/{}", detail::format_opt(m.hold_s),
            detail::format_opt(m.release_s),
            detail::format_opt(m.accelerate_s),
            detail::format_opt(m.handover_s));
        std::println(
            stderr,
            "{:<8} {:<10} {:<14} {:<16} {:<10} {:>7.3f} {:>+8.2f} {:>7.1f} {:>20} {:>16} {:>17} {:<24} {}",
            c.name, rule_label(rule), outcome_str, lifts_str, slips_str,
            m.slip_peak, m.rollback_m * 1000., m.leg_ema_peak_rpm,
            supports_str, win_str, t20_str, phases_str, verdict);
    }
    bt::require(all_failures.empty(), detail::join_failures(all_failures));
}

} // namespace launch_ride
