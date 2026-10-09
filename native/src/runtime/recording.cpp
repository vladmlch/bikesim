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
        const auto flat = sample->flat_columns(layout_);
        const auto count = samples_.size();
        if (count == samples_.max_size()) throw std::length_error("research recorder row limit reached");
        const double missing = std::numeric_limits<double>::quiet_NaN();
        // Stage new column nodes and all growth before publishing any row.
        // map::merge transfers allocated nodes, and the reserved vector pushes
        // below cannot allocate. Existing records survive allocation failures.
        SampleColumns additions;
        for (const auto &[key, value] : flat) {
            (void)value;
            if (!columns_.contains(key)) additions.emplace(key, std::vector<double>(count, missing));
        }
        for (auto &[key, column] : additions) { (void)key; reserve_row(column, count + 1); }
        for (auto &[key, column] : columns_) { (void)key; reserve_row(column, count + 1); }
        reserve_row(samples_, count + 1);
        columns_.merge(additions);
        for (auto &[key, column] : columns_) {
            const auto value = flat.find(key);
            column.push_back(value == flat.end() ? missing : value->second);
        }
        samples_.push_back(sample);
    }
    last_id_ = sample->interval_id;
}
SampleColumns ResearchRecorder::columns() const { return columns_; }
WireArray ResearchRecorder::intervals() const {
    WireArray result;
    result.reserve(samples_.size());
    for (const auto &sample : samples_) result.emplace_back(sample->as_wire());
    return result;
}
} // namespace runtime
