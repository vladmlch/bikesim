// Owned schema-2 values and native recorder columns. No Python objects.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "control.hpp"

namespace runtime {

namespace sample_wire {
[[nodiscard]] const Wire *find(const WireObject &object, std::string_view key);
[[nodiscard]] const Wire &required(const WireObject &object, std::string_view key);
[[nodiscard]] const WireObject &object(const Wire &value);
[[nodiscard]] const WireArray &array(const Wire &value);
[[nodiscard]] double number(const Wire &value);
[[nodiscard]] std::int64_t integer(const Wire &value);
[[nodiscard]] bool boolean(const Wire &value);
[[nodiscard]] const std::string &string(const Wire &value);
[[nodiscard]] bool is_none(const Wire &value) noexcept;
[[nodiscard]] Wire strings(std::span<const std::string> values);
void set(WireObject &object, std::string key, Wire value);
void merge(WireObject &destination, const WireObject &source);
void exact(const WireObject &object, std::span<const std::string_view> keys,
           std::string_view path);
[[nodiscard]] double dot(std::span<const double> a, std::span<const double> b);
[[nodiscard]] double numpy_sum(std::span<const double> values);
} // namespace sample_wire

using ForceComponents =
    std::vector<std::pair<std::string, std::vector<double>>>;
using SampleColumns = std::map<std::string, std::vector<double>>;

struct ColumnLayout {
    std::size_t root_x_qpos = 0;
    std::size_t root_x_dof = 0;
    std::size_t root_pitch_qpos = 0;
};

struct ConstraintSnapshot;

struct PhysicalSampleData {
    std::int64_t interval_id = 0;
    double time_s = 0., end_time_s = 0.;
    std::vector<double> qpos, qvel;
    ForceComponents forces;
    WireObject powers_w, channels;
    std::shared_ptr<const ConstraintSnapshot> constraints;

    [[nodiscard]] double dt_s() const noexcept { return end_time_s - time_s; }
    [[nodiscard]] WireObject as_wire() const;
    [[nodiscard]] std::map<std::string, double>
    flat_columns(const ColumnLayout &layout) const;
};

using SamplePtr = std::shared_ptr<const PhysicalSampleData>;

// These are value wrappers, not views into a runtime's pending vector.
struct RuntimeSample {
    SamplePtr value;
};
struct RuntimeSampleBatch {
    int generation = 0;
    std::vector<SamplePtr> samples;
    SampleColumns columns;
};

[[nodiscard]] SampleColumns make_columns(std::span<const SamplePtr> samples,
                                        const ColumnLayout &layout);
// Blocks avoid copying the entire episode whenever a period is published.
struct RecordedBlock {
    std::size_t rows = 0;
    SampleColumns columns;
};
[[nodiscard]] SampleColumns join_columns(std::span<const RecordedBlock> blocks);

struct WorkHistory {
    WireObject work_j;
    std::optional<std::int64_t> last_id;
    std::optional<double> last_end;
    std::array<std::array<double, 3>, 2> airtime_s{};
    double duration_s = 0.;

    void add(const PhysicalSampleData &sample);
    [[nodiscard]] WireObject as_wire() const;
    [[nodiscard]] static WorkHistory from_wire(const WireObject &value);
};

} // namespace runtime
