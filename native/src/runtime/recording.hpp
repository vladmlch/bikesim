// Research recording retains only consumed/decimated intervals. Export is lazy.
#pragma once
#include "samples.hpp"
namespace runtime {
class ResearchRecorder {
public:
    ResearchRecorder(int decimation, ColumnLayout layout);
    void append(const SamplePtr &sample);
    [[nodiscard]] SampleColumns columns() const;
    [[nodiscard]] WireArray intervals() const;
    [[nodiscard]] std::size_t rows() const noexcept { return samples_.size(); }
private:
    int decimation_;
    ColumnLayout layout_;
    std::optional<std::int64_t> last_id_;
    std::vector<SamplePtr> samples_;
    SampleColumns columns_;
};
} // namespace runtime
