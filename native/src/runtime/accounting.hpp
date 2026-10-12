// Every physical interval is evaluated, regardless of measurement decimation.
#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>
#include "config.hpp"
#include "period_buffer.hpp"
#include "samples.hpp"
#include "status.hpp"

namespace runtime {
struct StepWork {
    double muscle_signed_j = 0., muscle_positive_j = 0.;
    double motor_signed_j = 0., motor_positive_j = 0.;
    double constraint_signed_j = 0., constraint_absolute_j = 0.;
    [[nodiscard]] WireObject as_wire() const;
};
[[nodiscard]] StepWork step_work(std::span<const double> muscle_power_w,
                                 double motor_power_w,
                                 std::span<const double> constraint_power_w,
                                 double dt_s);
[[nodiscard]] bool constraint_work_ok(double absolute_j, double source_positive_j,
                                      double roundoff_j = 1e-8);

struct AccountingConfig {
    double timestep_s = 0.;
    std::size_t period_steps = 0;
    // The energy/momentum measurement cadence — what legacy
    // `record_decimation` scheduled before the measurement/retention split.
    int measurement_decimation = 1;
    RetentionMode retention = RetentionMode::legacy;
    bool battery_enabled = false;
    std::size_t max_dofs = 0;
    AttachmentBudget attachment_budget;
    ColumnLayout columns;
};

// Identity of the interval that produced the last committed energy
// measurement — physical_runtime.py's _last_measurement_source. A row that
// measured its own state cites itself; a row carrying the period's closing
// measurement cites the interval that measured it.
struct MeasurementSource {
    std::int64_t interval_id = 0;
    double time_s = 0.;
};

struct AccountingState {
    EnergyState energy;
    WorkHistory history;
    ModelStatus model_status;
    ReferenceMonitor monitor;
    WireObject energy_channels;
    std::vector<ReferenceFailure> period_violations;
    // The last measured endpoint_mass datum — _last_endpoint_mass in
    // physical_runtime.py. Memory only: the oracle keeps it as a plain
    // attribute, so it is deliberately absent from state_wire() and the
    // bootstrap decode (a fresh runtime re-derives it from the first
    // measured interval; interval 0 is always measured).
    std::optional<WireObject> last_endpoint_mass;
    // Same memory-only contract as last_endpoint_mass: the interval that
    // supplied the last committed energy measurement, for forwarding rows
    // in a partial tail that holds no measured interval at all.
    std::optional<MeasurementSource> last_measurement_source;
};

struct EffortObservation {
    WireObject channels;
    rider::EffortDiagnostics diagnostics;
    rider::NamedEntries<rider::JointTerms> terms;
    std::vector<std::string> violations;
};
[[nodiscard]] EffortObservation observe_effort(const RawStep &raw,
                                               const rider::SpindleController &controller,
                                               double dt_s);

class PeriodAccounting {
public:
    PeriodAccounting(AccountingConfig config, AccountingState initial, int generation,
                     rider::SpindleController *controller,
                     writers::RiderContactWriter *contacts);
    void push(RawStep raw);
    [[nodiscard]] bool full() const noexcept { return buffer_.full(); }
    [[nodiscard]] bool has_raws() const noexcept { return !buffer_.empty(); }
    void flush();
    [[nodiscard]] RuntimeSampleBatch prepare_samples() const;
    void acknowledge_samples(const RuntimeSampleBatch &batch);
    [[nodiscard]] std::span<const SamplePtr> pending_samples() const noexcept { return pending_; }
    void acknowledge_through(std::int64_t interval_id);
    [[nodiscard]] const SamplePtr &latest_sample() const noexcept { return latest_; }
    [[nodiscard]] const AccountingState &state() const noexcept { return state_; }
    [[nodiscard]] WireObject state_wire() const;
    [[nodiscard]] SampleColumns recorded_columns() const;
    [[nodiscard]] std::span<const SamplePtr> recorded_samples() const noexcept { return retained_; }
    // The owned immutable prefix design A2 publishes: the shared_ptr
    // index is copied, never moved, so a sealed episode stays valid for
    // the lifetime of its samples — across acknowledge, reset and close.
    [[nodiscard]] OwnedNativeEpisode seal_episode() const;
    [[nodiscard]] std::optional<std::string> take_warning() { return state_.monitor.take_warning(); }
private:
    AccountingConfig config_;
    AccountingState state_;
    int generation_;
    PeriodBuffer buffer_;
    rider::LeastSquaresWorkspace workspace_;
    rider::SpindleController *controller_;
    writers::RiderContactWriter *contacts_;
    std::vector<SamplePtr> pending_;
    // The retained committed index — independent of pending
    // acknowledgement/drain. `retention` selects which committed rows it
    // keeps; recorded_columns() materializes its flat projection lazily.
    std::vector<SamplePtr> retained_;
    SamplePtr latest_;
};
} // namespace runtime
