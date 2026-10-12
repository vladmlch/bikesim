#include "recording.hpp"
#include <algorithm>
#include <limits>
#include <stdexcept>
namespace runtime {
namespace {
template<class T>
void reserve_row(std::vector<T> &values, std::size_t required) {
    if (values.capacity() >= required) return;
    const auto doubled = values.capacity() > values.max_size() / 2 ? required : values.capacity() * 2;
    values.reserve(std::max(required, doubled));
}
}
ResearchRecorder::ResearchRecorder(int decimation, ColumnLayout layout)
    : decimation_(decimation), layout_(layout) {
    if (decimation_ < 1) throw std::invalid_argument("record decimation must be positive");
}
void ResearchRecorder::append(const SamplePtr &sample) {
    if (!sample) throw std::invalid_argument("cannot record a null interval");
    if (last_id_ && sample->interval_id < *last_id_)
        throw std::invalid_argument("reset requires a new recorder");
    if (last_id_ && sample->interval_id == *last_id_) return;
    if (sample->interval_id % decimation_ == 0) {
        const auto count = samples_.size();
        if (count == samples_.max_size()) throw std::length_error("research recorder row limit reached");
        reserve_row(samples_, count + 1);
        samples_.push_back(sample);
    }
    last_id_ = sample->interval_id;
}
OwnedNativeEpisode ResearchRecorder::seal() const {
    return OwnedNativeEpisode::seal(0, samples_, layout_);
}
SampleColumns ResearchRecorder::columns() const { return make_columns(samples_, layout_); }
WireArray ResearchRecorder::intervals() const {
    WireArray result;
    result.reserve(samples_.size());
    for (const auto &sample : samples_) result.emplace_back(sample->as_wire());
    return result;
}
} // namespace runtime
