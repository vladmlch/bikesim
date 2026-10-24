// Research recording retains only consumed/decimated intervals. Export is lazy.
#pragma once
#include "samples.hpp"
namespace runtime {
class ResearchRecorder {
public:
    ResearchRecorder(int decimation, ColumnLayout layout);
    void append(const SamplePtr &sample);
    [[nodiscard]] SampleColumns columns() const;
    [[nodiscard]] OwnedNativeEpisode seal() const;
    [[nodiscard]] WireArray intervals() const;
    [[nodiscard]] std::size_t rows() const noexcept { return samples_.size(); }
    // Cold continuation inventory. Samples are rebound to the staged plant's
    // immutable retained index; the recorder does not own a second encoding.
    [[nodiscard]] WireObject continuation_state() const;
    void restore_continuation(const WireObject&, std::span<const SamplePtr>,
                              std::int64_t sample_cursor);
private:
    int decimation_;
    ColumnLayout layout_;
    std::optional<std::int64_t> last_id_;
    std::vector<SamplePtr> samples_;
};
} // namespace runtime
