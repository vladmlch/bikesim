#include "spatial_sensor_pipeline.hpp"
#include "samples.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <span>
#include <stdexcept>
#include <string_view>

namespace runtime::spatial {
namespace {
void nonnegative(double value, std::string_view path) {
    if (!std::isfinite(value) || value < 0.) throw std::invalid_argument(std::string(path)+": expected finite nonnegative value");
}
std::size_t capacity(const SensorConfigV2& c) {
    const double count=std::ceil((c.latency_s+c.maximum_read_period_s)/c.sample_period_s)+3.;
    const auto maximum=std::vector<SensorObservationV2>{}.max_size();
    if (!std::isfinite(count) || count < 3. || count >= static_cast<double>(maximum))
        throw std::invalid_argument("sensors_v2.capacity: ring capacity is not representable");
    return static_cast<std::size_t>(count);
}
constexpr std::array<std::string_view,5> scalar_names{"front_wheel_rad_s","rear_wheel_rad_s","crank_rad_s","motor_torque_nm","human_torque_nm"};
SensorObservationV2 invalid(SensorObservationV2 sample) { sample.valid=false; sample.valid_mask=0; return sample; }
}
void SensorObservationV2::validate() const {
    nonnegative(time_s,"observation_v2.time_s"); nonnegative(source_time_s,"observation_v2.source_time_s");
    if (source_time_s > time_s || (valid_mask & ~0x7ffU) != 0 || (!valid && valid_mask != 0))
        throw std::invalid_argument("observation_v2: future source time or invalid validity mask");
    for (double x : values) if (!std::isfinite(x)) throw std::invalid_argument("observation_v2.values: nonfinite channel");
}
WireObject SensorObservationV2::as_wire() const {
    validate();
    WireObject out{{"sensor_schema_version",std::int64_t{2}}, {"time_s",time_s}, {"source_time_s",source_time_s},
        {"valid",valid}, {"valid_mask",static_cast<std::int64_t>(valid_mask)},
        {"specific_force_sensor_mps2",wire_array(std::span<const double>(values).first<3>())},
        {"angular_rate_sensor_rad_s",wire_array(std::span<const double>(values).subspan<3,3>())}};
    for (std::size_t i=0;i<scalar_names.size();++i) out.emplace_back(std::string(scalar_names[i]),values[i+6]);
    return out;
}
SensorObservationV2 SensorObservationV2::from_wire(const WireObject& wire) {
    using namespace sample_wire;
    constexpr std::array<std::string_view,12> keys{"sensor_schema_version","time_s","source_time_s","valid","valid_mask",
        "specific_force_sensor_mps2","angular_rate_sensor_rad_s","front_wheel_rad_s","rear_wheel_rad_s","crank_rad_s","motor_torque_nm","human_torque_nm"};
    exact(wire,keys,"observation_v2");
    if (integer(required(wire,"sensor_schema_version")) != 2) throw std::invalid_argument("observation_v2: unsupported sensor schema");
    const auto mask=integer(required(wire,"valid_mask"));
    if (mask<0 || mask>0x7ff) throw std::invalid_argument("observation_v2.valid_mask: unknown channel bit");
    SensorObservationV2 out;
    out.time_s=number(required(wire,"time_s")); out.source_time_s=number(required(wire,"source_time_s"));
    out.valid=boolean(required(wire,"valid")); out.valid_mask=static_cast<std::uint16_t>(mask);
    const auto& acc=array(required(wire,"specific_force_sensor_mps2"));
    const auto& gyro=array(required(wire,"angular_rate_sensor_rad_s"));
    if (acc.size()!=3 || gyro.size()!=3) throw std::invalid_argument("observation_v2: IMU arrays require width three");
    for(std::size_t i=0;i<3;++i){out.values[i]=number(acc[i]);out.values[i+3]=number(gyro[i]);}
    for(std::size_t i=0;i<scalar_names.size();++i)out.values[i+6]=number(required(wire,scalar_names[i]));
    out.validate(); return out;
}
void NoiseTapeV2::validate() const {
    if(noise.size()!=dropout_uniform.size())throw std::invalid_argument("noise_v2: shapes must be (count,11) and (count,)");
    for(const auto& row:noise)for(double x:row)if(!std::isfinite(x))throw std::invalid_argument("noise_v2: nonfinite noise");
    for(double x:dropout_uniform)if(!std::isfinite(x)||x<0.||x>=1.)throw std::invalid_argument("noise_v2.dropout_uniform: expected [0,1)");
}
void SensorConfigV2::validate() const {
    nonnegative(latency_s,"sensors_v2.latency_s"); nonnegative(maximum_age_s,"sensors_v2.maximum_age_s");
    nonnegative(dropout_probability,"sensors_v2.dropout_probability");
    if(dropout_probability>1.)throw std::invalid_argument("sensors_v2.dropout_probability: exceeds one");
    for(double x:{sample_period_s,maximum_read_period_s})if(!std::isfinite(x)||x<=0.)throw std::invalid_argument("sensors_v2.period: expected positive finite period");
    for(const auto& bias:{acceleration_bias,gyro_bias})for(double x:bias)if(!std::isfinite(x))throw std::invalid_argument("sensors_v2.bias: nonfinite value");
    for(const auto& stddev:{acceleration_std,gyro_std})for(double x:stddev)nonnegative(x,"sensors_v2.std");
    nonnegative(encoder_std,"sensors_v2.encoder_std");nonnegative(torque_std,"sensors_v2.torque_std");
    (void)capacity(*this);
}
SensorPipelineV2::SensorPipelineV2(SensorConfigV2 config,NoiseTapeV2 tape):config_(config),tape_(std::move(tape)) {
    config_.validate();tape_.validate();queue_storage_.resize(capacity(config_));
}
void SensorPipelineV2::reset(const SensorObservationV2& initial) {
    // Cold reset stages every field so failed validation retains the old run.
    SensorPipelineV2 next(config_,tape_);next.push(initial);
    queue_storage_.swap(next.queue_storage_);queue_head_=next.queue_head_;queue_count_=next.queue_count_;
    last_time_=next.last_time_;delivery_time_.reset();startup_=next.startup_;
    cursor_=next.cursor_;samples_attempted_=next.samples_attempted_;samples_dropped_=next.samples_dropped_;
}
void SensorPipelineV2::push(const SensorObservationV2& raw) {
    raw.validate();
    if(last_time_&&raw.source_time_s<=*last_time_)throw std::invalid_argument("sensors_v2.source_time: must strictly increase");
    if(cursor_>=tape_.noise.size())throw std::runtime_error("sensors_v2: noise tape exhausted; run invalid");
    if(queue_count_==queue_storage_.size())throw std::runtime_error("sensors_v2: delivery ring overflow; run invalid");
    auto sample=raw;const auto& noise=tape_.noise[cursor_];
    for(std::size_t i=0;i<3;++i){sample.values[i]=(raw.values[i]+config_.acceleration_bias[i])+config_.acceleration_std[i]*noise[i];
        sample.values[i+3]=(raw.values[i+3]+config_.gyro_bias[i])+config_.gyro_std[i]*noise[i+3];}
    for(std::size_t i=6;i<11;++i)sample.values[i]+=noise[i]*(i<9?config_.encoder_std:config_.torque_std);
    if(!config_.imu_enabled){std::fill_n(sample.values.begin(),6,0.);sample.valid_mask=static_cast<std::uint16_t>(sample.valid_mask&0x7c0U);}
    const bool dropped=tape_.dropout_uniform[cursor_]<config_.dropout_probability;
    if(dropped)sample=invalid(sample);
    sample.validate();
    // Dropped attempts are retained as invalid slots: a dropout cannot make an
    // earlier successful reading look like a new valid delivery.
    queue_storage_[(queue_head_+queue_count_)%queue_storage_.size()]=sample;++queue_count_;
    if(!startup_)startup_=(sample.source_time_s+config_.latency_s<=sample.time_s&&
        sample.time_s-sample.source_time_s<=config_.maximum_age_s)?sample:invalid(sample);
    last_time_=raw.source_time_s;++cursor_;++samples_attempted_;if(dropped)++samples_dropped_;
}
SensorObservationV2 SensorPipelineV2::read(double now) {
    nonnegative(now,"sensors_v2.delivery_time");
    if(delivery_time_&&now<*delivery_time_)throw std::invalid_argument("sensors_v2.delivery_time: clock moved backwards");
    if(!startup_)throw std::runtime_error("sensors_v2: reset before read");
    std::size_t selected=0;
    while(selected+1<queue_count_&&queue_storage_[(queue_head_+selected+1)%queue_storage_.size()].source_time_s+config_.latency_s<=now)++selected;
    auto sample=queue_count_&&queue_storage_[(queue_head_+selected)%queue_storage_.size()].source_time_s+config_.latency_s<=now?
        queue_storage_[(queue_head_+selected)%queue_storage_.size()]:*startup_;
    if(sample.source_time_s>now)throw std::invalid_argument("sensors_v2.delivery_time: predates startup");
    sample.time_s=now;
    if(now-sample.source_time_s>config_.maximum_age_s||sample.source_time_s+config_.latency_s>now)sample=invalid(sample);
    queue_head_=(queue_head_+selected)%queue_storage_.size();queue_count_-=selected;delivery_time_=now;
    return sample;
}
SensorStateV2 SensorPipelineV2::state() const {
    SensorStateV2 out;
    for(std::size_t i=0;i<queue_count_;++i)out.queue.push_back(queue_storage_[(queue_head_+i)%queue_storage_.size()]);
    out.last_time=last_time_;out.delivery_time=delivery_time_;out.startup=startup_;
    out.cursor=cursor_;out.samples_attempted=samples_attempted_;out.samples_dropped=samples_dropped_;return out;
}
void SensorPipelineV2::import_state(SensorStateV2 next) {
    if(next.cursor!=next.samples_attempted||next.cursor>tape_.noise.size()||next.samples_dropped>next.cursor||
        next.queue.size()>queue_storage_.size()||next.queue.size()>next.cursor)
        throw std::invalid_argument("sensors_v2.state: inconsistent cursor, counters or ring size");
    if(next.cursor==0){if(next.startup||next.last_time||next.delivery_time||!next.queue.empty())throw std::invalid_argument("sensors_v2.state: uninitialized state has samples");}
    else if(!next.startup||!next.last_time||next.queue.empty())throw std::invalid_argument("sensors_v2.state: initialized state lacks clocks/startup/latest sample");
    const auto dropped=[&](double uniform){return uniform<config_.dropout_probability;};
    const auto consumed=std::span<const double>(tape_.dropout_uniform).first(next.cursor);
    if(next.samples_dropped!=static_cast<std::size_t>(std::ranges::count_if(consumed,dropped)))
        throw std::invalid_argument("sensors_v2.state.samples_dropped: differs from consumed noise tape");
    if(next.last_time)nonnegative(*next.last_time,"sensors_v2.last_time");
    if(next.delivery_time)nonnegative(*next.delivery_time,"sensors_v2.delivery_time");
    const auto configured_sample=[&](const SensorObservationV2& sample) {
        sample.validate();
        if(!config_.imu_enabled&&((sample.valid_mask&0x3fU)!=0||
            std::ranges::any_of(std::span<const double>(sample.values).first<6>(),[](double x){return x!=0.;})))
            throw std::invalid_argument("sensors_v2.state: disabled IMU channels must be canonical zero and invalid");
    };
    if(next.startup){configured_sample(*next.startup);if(next.startup->source_time_s>*next.last_time||
        (dropped(tape_.dropout_uniform.front())&&next.startup->valid)||
        (next.startup->valid&&(next.startup->source_time_s+config_.latency_s>next.startup->time_s||
         next.startup->time_s-next.startup->source_time_s>config_.maximum_age_s)))
        throw std::invalid_argument("sensors_v2.state.startup: invalid startup sentinel");
        if(next.delivery_time&&*next.delivery_time<next.startup->source_time_s)throw std::invalid_argument("sensors_v2.state.delivery_time: precedes startup");}
    std::optional<double> previous;
    // read() discards only a prefix; retained slots map to the last queue.size()
    // attempts, including dropped attempts. A nondrop may still be invalid for
    // another reason, but a dropped attempt can never be a usable delivery.
    std::size_t attempt=next.cursor-next.queue.size();
    for(const auto& sample:next.queue){configured_sample(sample);
        if(dropped(tape_.dropout_uniform[attempt++])&&sample.valid)
            throw std::invalid_argument("sensors_v2.state.queue: dropped attempt cannot be valid");
        if((previous&&sample.source_time_s<=*previous)||!next.last_time||
        sample.source_time_s>*next.last_time||sample.source_time_s<next.startup->source_time_s)
        throw std::invalid_argument("sensors_v2.state.queue: timestamps are not unique and ordered");previous=sample.source_time_s;}
    if(next.last_time&&(!previous||*previous!=*next.last_time))
        throw std::invalid_argument("sensors_v2.state.queue: latest attempt is absent");
    // All validation and snapshot allocation happened before the live commit.
    std::copy(next.queue.begin(),next.queue.end(),queue_storage_.begin());queue_head_=0;queue_count_=next.queue.size();
    last_time_=next.last_time;delivery_time_=next.delivery_time;startup_=next.startup;cursor_=next.cursor;
    samples_attempted_=next.samples_attempted;samples_dropped_=next.samples_dropped;
}
WireObject SensorPipelineV2::state_wire() const {
    WireArray queue;queue.reserve(queue_count_);
    for(std::size_t i=0;i<queue_count_;++i)queue.emplace_back(queue_storage_[(queue_head_+i)%queue_storage_.size()].as_wire());
    const auto optional=[](std::optional<double>x){return x?Wire(*x):Wire(nullptr);};
    return {{"schema",std::int64_t{2}},{"queue",std::move(queue)},{"last_time",optional(last_time_)},{"delivery_time",optional(delivery_time_)},
        {"startup",startup_?Wire(startup_->as_wire()):Wire(nullptr)},{"cursor",static_cast<std::int64_t>(cursor_)},
        {"samples_attempted",static_cast<std::int64_t>(samples_attempted_)},{"samples_dropped",static_cast<std::int64_t>(samples_dropped_)}};
}
} // namespace runtime::spatial
