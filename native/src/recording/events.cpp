#include "capture.hpp"
#include "../runtime/encode.hpp"
#include "value_util.hpp"
#include "../research/evidence.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace bike::recording {
std::vector<InputEvent> derived_events(const runtime::OwnedNativeEpisode& episode, std::int64_t id, std::int64_t first_id, double wall_s) {
    if (id < 0 || first_id < 0 || !std::isfinite(wall_s)) throw std::invalid_argument("invalid derived event identity/clock");
    static const auto plan = runtime::parse_derived_event_plan(io::parse_json(R"JSON({"fields":[
        {"kind":"tire_contact","path":["tires","front","raw_contact"]},
        {"kind":"tire_contact","path":["tires","rear","raw_contact"]},
        {"kind":"tire_contact","path":["tires","front","controller_grounded"]},
        {"kind":"tire_contact","path":["tires","rear","controller_grounded"]},
        {"kind":"shift","path":["drive","shift_active"]}, {"kind":"shift","path":["drive","shift_direction"]},
        {"kind":"freehub_coast","path":["drive","coasting_reason"]}, {"kind":"freehub_coast","path":["drive","freehub_engaged"]},
        {"kind":"drive_limits","path":["drive","safety_limited"]}, {"kind":"drive_limits","path":["drive","energy_limited"]},
        {"kind":"drive_limits","path":["drive","motor_enabled"]}, {"kind":"drive_limits","path":["drive","battery_empty"]},
        {"kind":"brake_gate","path":["drive","front_brake_demand"]}, {"kind":"brake_gate","path":["drive","rear_brake_demand"]},
        {"kind":"brake_gate","path":["drive","rollback_brake_demand"]},
        {"kind":"model_validity","path":["model_status","model_valid"]}, {"kind":"model_validity","path":["model_status","numerically_valid"]},
        {"kind":"model_validity","path":["model_status","model_violation_counts"]},
        {"kind":"model_first_failure","path":["model_status","first_model_violation"]},
        {"kind":"contact_crash","path":["contact_crash_cause"]}]} )JSON"));
    std::vector<InputEvent> out;
    for (std::size_t first = 0; first < episode.samples.size(); first += std::min<std::size_t>(1024, episode.samples.size()-first)) {
        for (const auto& row : runtime::derived_event_rows_native(episode, plan, static_cast<std::int64_t>(first), 1024)) {
            if (out.size() >= static_cast<std::size_t>(std::numeric_limits<std::int64_t>::max()-first_id)) throw std::overflow_error("derived event identity overflow");
            InputEvent event{.episode_id=id, .event_id=first_id + static_cast<std::int64_t>(out.size()), .kind="derived_transition", .received_wall_s=wall_s, .processed_step=row.interval_id, .applied_step=row.interval_id,
                .values=io::Value{io::Value::Object{{"classification", value::text("derived_transition")}, {"derived_kind", value::text(row.derived_kind)},
                    {"derivation_rule", value::text("channel_value_change.v1")}, {"interval_id", value::integer(row.interval_id)}, {"changes", research::wire_value(runtime::Wire{row.changes})}}}};
            event.validate(); out.push_back(std::move(event));
        }
    }
    return out;
}
std::int64_t episode_interval_for_time(const runtime::OwnedNativeEpisode& episode, double time_s) {
    if (!std::isfinite(time_s)) throw std::invalid_argument("event time must be finite");
    std::int64_t last = 0;
    for (const auto& sample : episode.samples) {
        if (!sample) throw std::invalid_argument("null retained sample");
        last = sample->interval_id;
        if (sample->end_time_s >= time_s - 1e-12) return last;
    }
    return last;
}
} // namespace bike::recording
