// Owns one physical runtime and all physics-rate research state. No callbacks.
// Pure C++ (task T10): the nanobind adapter in research_binding.cpp parses
// and boxes; nothing here reaches Python. The owned bootstrap/composition
// face lives in research_io.hpp.
#pragma once
#include <atomic>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include "programs.hpp"
#include "recording.hpp"
#include "research_metrics.hpp"
#include "runtime.hpp"
#include "sensors.hpp"

namespace runtime {
enum class ResearchCaptureMode : std::uint8_t { off, full, legacy, verify };
struct ResearchAsset {
    std::string name, media_type, sha256;
    std::vector<std::byte> content;
};
using ResearchAssets = std::shared_ptr<const std::vector<ResearchAsset>>;
struct ResearchConfig {
    double timestep_s = 0., track_length_m = 0., start_position_m = 0.;
    double maximum_energy_residual_ratio = .05, wheelie_persistence_s = .02;
    std::int64_t control_steps = 0, delay_steps = 0, max_steps = 0, sensor_steps = 0;
    int record_decimation = 1;
    ResearchCaptureMode capture_mode = ResearchCaptureMode::legacy;
    bool stop_on_model_violation = true;
    SensorConfig sensors;
    TruthGeometry geometry;
    ResearchPrograms programs;
    RideControl startup_control;
    // Cold construction/reset provenance. Frozen T10 captures omit these
    // optional additions and retain their original supplied tape/state.
    std::string seed_decimal = "0";
    std::array<double, 4> noise_std{.05, .002, .01, .05};
    std::optional<SensorObservation> raw_initial;
    bike::io::Value metadata;
    ResearchAssets assets;
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
    [[nodiscard]] bool valid_for_learning() const noexcept {
        return numerically_valid && model_valid && reason != "simulation_error" && reason != "policy_error";
    }
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
    std::optional<ResearchTransition> pending_transition;
    std::optional<ResearchCommand> last_requested;
    std::int64_t requested_count = 0, completed_count = 0, applied_base = 0;
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
// A sealed episode owns its index and control-clock records. It never borrows
// a runtime, model, recorder or mutable history vector.
struct OwnedResearchEpisode {
    OwnedNativeEpisode physical;
    std::vector<ResearchCommand> requested, applied;
    std::vector<SensorObservation> observations;
    std::vector<ResearchTransition> transitions;
    WireObject evaluation;
    RuntimeSnapshot final_snapshot;
    ResearchConfig configuration;
    std::vector<double> initial_integration_state;
    bike::io::Value run_metadata;
    [[nodiscard]] std::size_t row_count(std::string_view stream) const;
    [[nodiscard]] WireArray batch(std::string_view stream, std::int64_t start,
                                  std::int64_t limit) const;
};
class NativeResearchRuntime {
public:
    NativeResearchRuntime(std::unique_ptr<NativeRideRuntime> runtime, ResearchConfig config,
                          NoiseTape tape, SensorState startup);
    ~NativeResearchRuntime();
    NativeResearchRuntime(const NativeResearchRuntime &) = delete;
    NativeResearchRuntime &operator=(const NativeResearchRuntime &) = delete;
    NativeResearchRuntime(NativeResearchRuntime &&) = delete;
    NativeResearchRuntime &operator=(NativeResearchRuntime &&) = delete;
    void begin_control(const RideControl &control, double front, double rear);
    // A completed result stays available until ack_control after Python boxing.
    [[nodiscard]] std::optional<ResearchTransition> advance_control(
        std::optional<double> wall_budget_s, std::optional<std::int64_t> target_step = std::nullopt);
    void acknowledge_control();
    [[nodiscard]] RuntimeSnapshot snapshot() const;
    [[nodiscard]] RenderFrame render_frame() const;
    [[nodiscard]] ModelImage model_image() const;
    [[nodiscard]] WireObject status() const;
    [[nodiscard]] ResearchConfig configuration() const;
    // No optional trajectory retention is required to freeze live metrics.
    [[nodiscard]] OwnedResearchEpisode evidence_snapshot() const;
    [[nodiscard]] OwnedResearchEpisode seal_episode();
    [[nodiscard]] OwnedResearchEpisode legacy_snapshot() const;
    [[nodiscard]] WireObject prepare_applied_since(std::int64_t cursor) const;
    void acknowledge_applied(std::int64_t cursor);
    [[nodiscard]] Wire last_requested() const;
    [[nodiscard]] WireArray commands_requested() const;
    [[nodiscard]] WireArray commands_applied() const;
    [[nodiscard]] WireArray observations() const;
    [[nodiscard]] WireArray transitions() const;
    [[nodiscard]] SampleColumns recorded_columns() const;
    [[nodiscard]] WireArray recorded_intervals() const;
    void reset(NoiseTape tape, SensorState startup,
               std::optional<std::string> seed_decimal = std::nullopt,
               std::optional<bike::io::Value> metadata = std::nullopt,
               ResearchAssets assets = {});
    void reset();
    void stop();
    void fail_policy(std::string message);
    void set_error_text(std::string message);
    void close();
    [[nodiscard]] bool closed() const noexcept { return closed_.load(std::memory_order_acquire); }
    void test_fail_at_step(std::int64_t step);
    // The pending non-strict reference rejection — the RuntimeWarning the
    // adapter emits at the held-GIL boundary after advance_control. The
    // pure layer holds it as state; emission is the adapter's.
    [[nodiscard]] std::optional<std::string> take_pending_warning();
private:
    void require_open() const;
    void consume_samples(bool stop_at_outcome = true);
    void consume_sample(const SamplePtr &sample, bool stop_at_outcome);
    void finish_window();
    void handle_failure(const std::exception &failure);
    [[nodiscard]] bool model_valid() const;
    [[nodiscard]] WireObject status_unlocked() const;
    [[nodiscard]] OwnedResearchEpisode evidence_unlocked(OwnedNativeEpisode physical) const;
    void reset_unlocked(NoiseTape tape, SensorState startup,
                        std::optional<std::string> seed_decimal,
                        std::optional<bike::io::Value> metadata, ResearchAssets assets);
    std::unique_ptr<NativeRideRuntime> runtime_;
    ResearchConfig config_;
    NoiseTape reset_tape_;
    SensorState reset_startup_;
    std::vector<double> initial_integration_state_;
    std::unique_ptr<ResearchEpisode> episode_;
    mutable std::atomic<bool> busy_{false};
    std::atomic<bool> closed_{false};
};
} // namespace runtime
