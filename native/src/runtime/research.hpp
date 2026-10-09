// Owns one physical runtime and all physics-rate research state. No callbacks.
#pragma once
#include <atomic>
#include <deque>
#include <memory>
#include "programs.hpp"
#include "recording.hpp"
#include "research_metrics.hpp"
#include "runtime_binding.hpp"
#include "sensors.hpp"

namespace runtime {
struct ResearchConfig {
    double timestep_s = 0., track_length_m = 0., start_position_m = 0.;
    double maximum_energy_residual_ratio = .05, wheelie_persistence_s = .02;
    std::int64_t control_steps = 0, delay_steps = 0, max_steps = 0, sensor_steps = 0;
    int record_decimation = 1;
    bool stop_on_model_violation = true;
    SensorConfig sensors;
    TruthGeometry geometry;
    ResearchPrograms programs;
    RideControl startup_control;
};
struct ResearchCommand {
    double time_s = 0.;
    std::int64_t step = 0;
    RideControl control;
    double front = 0., rear = 0.;
    [[nodiscard]] WireObject as_wire(bool requested) const;
};
struct ResearchTransition {
    SensorObservation observation;
    std::optional<WheelieTruth> truth;
    std::string contact_state;
    bool terminated = false, truncated = false, numerically_valid = true, model_valid = true;
    std::optional<std::string> reason;
    std::int64_t physics_steps = 0;
    std::optional<double> demand_nm;
    [[nodiscard]] WireObject as_wire() const;
};
struct ResearchWindow {
    RideControl control;
    double front = 0., rear = 0.;
    std::int64_t start_step = 0, target_step = 0;
    std::optional<double> seen_demand;
};
struct ResearchEpisode {
    ResearchEpisode(const ResearchConfig &config, NoiseTape tape, SensorState startup);
    SensorPipeline sensors;
    WheelieTracker tracker;
    ResearchRecorder recorder;
    std::int64_t next_sensor_step = 0, sample_cursor = -1;
    std::deque<std::pair<std::int64_t, RideControl>> commands;
    RideControl applied, motor_applied;
    std::optional<ResearchWindow> window;
    bool delivery_pending = false, stop_requested = false;
    std::vector<ResearchCommand> requested, applied_records;
    std::vector<SensorObservation> observations;
    std::vector<ResearchTransition> transitions;
    SensorObservation observation;
    std::optional<WheelieTruth> truth;
    bool terminated = false, truncated = false, numerically_valid = true;
    std::optional<std::string> reason, error;
    std::vector<std::string> cleanup_notes;
    double max_energy_residual_ratio = 0., torque_delivered_nms = 0., torque_requested_nms = 0.;
    std::optional<double> demand_integral_nms, demand_nm;
    double max_shock_stroke_m = 0., max_fork_travel_m = 0.;
    [[nodiscard]] bool done() const noexcept { return terminated || truncated; }
};
class NativeResearchRuntime {
public:
    NativeResearchRuntime(std::unique_ptr<NativeRideRuntime> runtime, ResearchConfig config,
                          NoiseTape tape, SensorState startup);
    ~NativeResearchRuntime();
    NativeResearchRuntime(const NativeResearchRuntime &) = delete;
    NativeResearchRuntime &operator=(const NativeResearchRuntime &) = delete;
    void begin_control(const RideControl &control, double front, double rear);
    // A completed result stays available until ack_control after Python boxing.
    [[nodiscard]] std::optional<ResearchTransition> advance_control(
        std::optional<double> wall_budget_s, std::optional<std::int64_t> target_step = std::nullopt);
    void acknowledge_control();
    [[nodiscard]] RuntimeSnapshot snapshot() const;
    [[nodiscard]] WireObject status() const;
    [[nodiscard]] WireArray commands_requested() const;
    [[nodiscard]] WireArray commands_applied() const;
    [[nodiscard]] WireArray observations() const;
    [[nodiscard]] WireArray transitions() const;
    [[nodiscard]] SampleColumns recorded_columns() const;
    [[nodiscard]] WireArray recorded_intervals() const;
    void reset(NoiseTape tape, SensorState startup);
    void stop();
    void fail_policy(std::string message);
    void set_error_text(std::string message);
    void close();
    [[nodiscard]] bool closed() const noexcept { return closed_.load(std::memory_order_acquire); }
    void test_fail_at_step(std::int64_t step);
    void emit_warning(); // held-GIL boundary only
private:
    void require_open() const;
    void consume_samples(bool stop_at_outcome = true);
    void consume_sample(const SamplePtr &sample, bool stop_at_outcome);
    void finish_window();
    void handle_failure(const std::exception &failure);
    [[nodiscard]] bool model_valid() const;
    std::unique_ptr<NativeRideRuntime> runtime_;
    ResearchConfig config_;
    std::unique_ptr<ResearchEpisode> episode_;
    mutable std::atomic<bool> busy_{false};
    std::atomic<bool> closed_{false};
};
} // namespace runtime
