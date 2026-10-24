#include "evidence.hpp"
#include "../runtime/setup_values.hpp"

namespace bike::research {
io::Value episode_metrics(const runtime::OwnedResearchEpisode& episode) {
    using namespace runtime::sample_wire;
    using namespace runtime::setupvalue;
    const auto& s = episode.evaluation;
    const double duration = number(required(s, "time_s"));
    const double progress = number(required(s, "position_m")) - episode.configuration.start_position_m;
    const double requested = number(required(s, "torque_requested_nms"));
    const double delivered = number(required(s, "torque_delivered_nms"));
    const auto& reason = required(s, "reason");
    const std::string outcome = is_none(reason) ? "" : string(reason);
    const auto& balance = required(s, "balance_event");
    bool intervention = false;
    if (const auto* metadata = std::get_if<Object>(&episode.run_metadata.data))
        if (const auto* value = io::find_member(*metadata, "operator_intervention"))
            intervention = io::as_bool(*value, "operator_intervention");
    auto result=Value{Object{{"outcome", wire_value(reason)}, {"duration_s", real(duration)}, {"progress_m", real(progress)},
        {"mean_speed_mps", real(duration > 0. ? progress / duration : 0.)},
        {"finish_time_s", outcome == "finish" ? real(duration) : Value{}},
        {"torque_delivered_nms", real(delivered)}, {"torque_requested_nms", real(requested)},
        {"demand_integral_nms", wire_value(required(s, "demand_integral_nms"))},
        {"operator_intervention", Value{intervention}}, {"motor_pass_fraction", requested > 0. ? real(delivered / requested) : Value{}},
        {"balance_lost", Value{!is_none(balance)}},
        {"balance_lost_at_m", is_none(balance) ? Value{} : wire_value(required(object(balance), "position_m"))},
        {"loop_out", Value{outcome == "crash:loop_out"}}, {"endo", Value{outcome == "crash:endo"}},
        {"max_shock_stroke_m", wire_value(required(s, "max_shock_stroke_m"))},
        {"max_fork_travel_m", wire_value(required(s, "max_fork_travel_m"))},
        {"numerically_valid", wire_value(required(s, "numerically_valid"))},
        {"max_energy_residual_ratio", wire_value(required(s, "max_energy_residual_ratio"))},
        {"model_status", wire_value(runtime::Wire{episode.final_snapshot.model_status})},
        {"wheelie", wire_value(required(s, "metrics"))}}};
    if(const auto* spatial=find(s,"spatial"))set(result,"spatial",wire_value(*spatial));
    return result;
}
} // namespace bike::research
