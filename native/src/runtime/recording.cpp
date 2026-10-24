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
WireObject ResearchRecorder::continuation_state() const {
    WireArray retained;
    retained.reserve(samples_.size());
    for(const auto& sample:samples_)retained.emplace_back(sample->interval_id);
    return {{"last_id",last_id_?Wire(*last_id_):Wire(nullptr)},
            {"retained_ids",std::move(retained)}};
}
void ResearchRecorder::restore_continuation(const WireObject& state,
    std::span<const SamplePtr> available,std::int64_t sample_cursor) {
    using namespace sample_wire;
    constexpr std::array<std::string_view,2> keys{"last_id","retained_ids"};
    exact(state,keys,"checkpoint.research.recorder");
    std::optional<std::int64_t> last;
    if(!is_none(required(state,"last_id")))last=integer(required(state,"last_id"));
    if(last&&(*last<0||*last>sample_cursor))
        throw std::invalid_argument("checkpoint.research.recorder.last_id: outside consumed prefix");
    std::vector<SamplePtr> retained;
    const auto& ids=array(required(state,"retained_ids"));
    retained.reserve(ids.size());
    std::int64_t previous=-1;
    std::size_t cursor=0;
    for(const auto& value:ids) {
        const auto id=integer(value);
        if(id<0||id<=previous||!last||id>*last||id%decimation_!=0)
            throw std::invalid_argument("checkpoint.research.recorder.retained_ids: invalid decimated prefix");
        while(cursor<available.size()&&available[cursor]&&available[cursor]->interval_id<id)++cursor;
        if(cursor==available.size()||!available[cursor]||available[cursor]->interval_id!=id)
            throw std::invalid_argument("checkpoint.research.recorder.retained_ids: sample is absent from plant checkpoint");
        retained.push_back(available[cursor]);previous=id;++cursor;
    }
    const auto expected=last?static_cast<std::size_t>(*last/decimation_+1):0U;
    if(retained.size()!=expected)
        throw std::invalid_argument("checkpoint.research.recorder.retained_ids: incomplete consumed prefix");
    samples_.swap(retained);last_id_=last;
}
} // namespace runtime
