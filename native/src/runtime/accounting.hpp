// Every physical interval is evaluated, regardless of recording decimation.
#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>
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
    int record_decimation = 1;
    bool battery_enabled = false;
    std::size_t max_dofs = 0;
    AttachmentBudget attachment_budget;
    ColumnLayout columns;
};

struct AccountingState {
    EnergyState energy;
    WorkHistory history;
    ModelStatus model_status;
    ReferenceMonitor monitor;
    WireObject energy_channels;
    std::vector<ReferenceFailure> period_violations;
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
    [[nodiscard]] const SamplePtr &latest_sample() const noexcept { return latest_; }
    [[nodiscard]] const AccountingState &state() const noexcept { return state_; }
    [[nodiscard]] WireObject state_wire() const;
    [[nodiscard]] SampleColumns recorded_columns() const;
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
    std::vector<RecordedBlock> recorded_;
    SamplePtr latest_;
};
} // namespace runtime
