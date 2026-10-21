#pragma once
#include "common.hpp"
#include "../runtime/samples.hpp"

namespace bike::validation {
// A completed accounting period may publish several single-step samples.
// Validate the complete drained interval sequence, not the batch size.
class IntervalCoverage {
public:
    IntervalCoverage(std::int64_t first_interval,double start_time_s);
    void observe(const runtime::PhysicalSampleData& sample);
    void finish(std::int64_t final_step,double end_time_s) const;
private:
    std::int64_t next_interval_;
    double previous_end_;
};
struct ValidPrefix {
    std::size_t seen=0,accepted=0;
    double valid_until_s=0.,front_normal_impulse_ns=0.,rear_normal_impulse_ns=0.;
    Value first_bad;
    bool observe(const runtime::PhysicalSampleData& sample);
};
[[nodiscard]] double trace_integral(std::span<const double> times,std::span<const double> values);
[[nodiscard]] double interval_integral(const Value& rows,std::string_view key);
[[nodiscard]] double relative_change(double value,double reference,double scale_floor);
[[nodiscard]] Value rider_work_ledger(const Value& work);
[[nodiscard]] Value support_evidence(const Value& rows,std::optional<double> required_duration_s={});
[[nodiscard]] Value coasting_evidence(const Value& rows);
[[nodiscard]] Value resume_evidence(const Value& rows,double end_s=8.);
[[nodiscard]] Value timing_metrics(std::span<const double> step_ns,double dt_s);
[[nodiscard]] Value persistent_events(const Value& rows,double persistence_s=.02,bool normal_load_only=false);
[[nodiscard]] Value contact_comparison(const Value& coarse,const Value& reference);
[[nodiscard]] Value compare_metrics(const Value& coarse,const Value& reference);
[[nodiscard]] Value metrics_from_rows(const Value& rows,double expected_duration_s,std::string_view initial_state_sha256);
[[nodiscard]] Value resolution_variants(const Value& time_steps,const Value& road_steps,const Value& station_counts,bool include_stations=true);
} // namespace bike::validation
