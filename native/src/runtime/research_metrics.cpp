#include "research_metrics.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include "../numeric_sincos.hpp"
#include "../numeric_sum.hpp"
#include "../tyre/profile.hpp"

namespace runtime {
WireObject SpatialEpisodeMetrics::as_wire() const {
    return {{"spatial_schema",Wire(std::int64_t{1})},{"requested_duration_s",Wire(requested_duration_s)},
        {"valid_duration_s",Wire(valid_duration_s)},{"completed_distance_m",Wire(completed_distance_m)},
        {"terminal_reason",Wire(std::string(bike::spatial::to_string(terminal_reason)))},{"valid",Wire(valid)}};
}
void update_spatial_metrics(SpatialEpisodeMetrics& metrics,std::int64_t interval,
        double end,double dt,double position,const bike::spatial::Outcome& outcome) {
    if(interval<0 || !std::isfinite(end) || !std::isfinite(dt) || dt<=0. || end<dt ||
       !std::isfinite(position) || !std::isfinite(metrics.requested_duration_s) || metrics.requested_duration_s<0.)
        throw std::invalid_argument("spatial.metrics: invalid accepted interval");
    if(interval<=metrics.last_interval)return;
    if(metrics.terminal_reason!=bike::spatial::SpatialReason::none)return;
    if(metrics.last_interval>=0 && interval!=metrics.last_interval+1)
        throw std::invalid_argument("spatial.metrics: accepted interval gap");
    const double time_tolerance=32.*std::numeric_limits<double>::epsilon()*std::max({1.,std::abs(end),dt});
    if(metrics.last_interval<0)metrics.accepted_origin_time_s=end-dt;
    else if(std::abs(end-metrics.last_end_time_s-dt)>time_tolerance)
        throw std::invalid_argument("spatial.metrics: noncontiguous accepted times");
    const bool terminal=outcome.terminal && outcome.time_s<=end;
    const bool valid_interval=!terminal || outcome.valid;
    if(valid_interval && metrics.valid) {
        const double accepted_duration=end-metrics.accepted_origin_time_s;
        metrics.valid_duration_s=accepted_duration>=metrics.requested_duration_s-time_tolerance?
            metrics.requested_duration_s:accepted_duration;
        metrics.completed_distance_m=std::max(metrics.completed_distance_m,position-metrics.start_position_m);
    }
    if(terminal) {
        metrics.valid=metrics.valid && outcome.valid;
        if(metrics.terminal_reason==bike::spatial::SpatialReason::none)metrics.terminal_reason=outcome.reason;
    }
    metrics.last_interval=interval;
    metrics.last_end_time_s=end;
}
namespace {
using namespace sample_wire;
double n(const WireObject &o, std::string_view key) { return number(required(o, key)); }
const WireObject &o(const WireObject &v, std::string_view key) { return object(required(v, key)); }
double optional_number(const WireObject &v, std::string_view key, double fallback) {
    const auto *p = find(v, key);
    return p ? number(*p) : fallback;
}
Wire opt(std::optional<double> v) { return v ? Wire(*v) : Wire(nullptr); }
// wheelie.py's load() is builtin sum() over the terrain patches — CPython
// 3.13 Neumaier compensated summation, not a naive fold. The items are
// explicit float(p['normal_load_n']) conversions, so the compensated
// float branch is the oracle's whole path.
double load(const WireObject &wheel) {
    numeric::PythonSum result;
    for (const auto &patch : array(required(wheel, "patches"))) {
        const auto &p = object(patch);
        if (string(required(p, "source_geom")) == "terrain") result.add(n(p, "normal_load_n"));
    }
    return result.total();
}
}
std::string_view classify_contact(double front_load,double rear_load,double front_clearance,
        // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
        double rear_clearance,double relative_pitch,double load_threshold,double clearance_threshold,double pitch_threshold) {
    for(double const value:{front_load,rear_load,front_clearance,rear_clearance,load_threshold,clearance_threshold,pitch_threshold})
        if(!std::isfinite(value)||value<0.)throw std::invalid_argument("contact classification inputs must be finite and nonnegative");
    if(!std::isfinite(relative_pitch))throw std::invalid_argument("road-relative pitch must be finite");
    const bool front = front_load > load_threshold, rear = rear_load > load_threshold;
    if (front && rear) return "two_wheels";
    if (!front && !rear) return std::min(front_clearance,rear_clearance) > clearance_threshold ? "flight" : "unsupported";
    if (!rear) return rear_clearance > clearance_threshold ? "rear_lift" : "rear_unloaded";
    if (front_clearance <= clearance_threshold) return "front_unloaded";
    return relative_pitch > pitch_threshold ? "wheelie_candidate" : "front_lift";
}
double quasistatic_front_load(double mass,double wheelbase,double forward,double height,double road_pitch,double acceleration,double gravity) {
    for(double const value:{mass,wheelbase,gravity})if(!std::isfinite(value)||value<=0.)throw std::invalid_argument("quasistatic mass, wheelbase and gravity must be positive");
    if(!std::isfinite(height)||height<0.)throw std::invalid_argument("quasistatic CoM height must be nonnegative");
    for(double const value:{forward,road_pitch,acceleration})if(!std::isfinite(value))throw std::invalid_argument("quasistatic input must be finite");
    // Observation only. A negative estimate remains signed; it never enters
    // applied forces or controller observations.
    return mass*(gravity*numeric::math_cos(road_pitch)*forward-height*(acceleration+gravity*numeric::math_sin(road_pitch)))/wheelbase;
}
void TruthGeometry::validate(std::size_t nq, std::size_t nv) const {
    if (x.size() < 2 || x.size() != z.size()) throw std::invalid_argument("research terrain requires equal x/z arrays");
    for (std::size_t i = 0; i < x.size(); ++i)
        if (!std::isfinite(x[i]) || !std::isfinite(z[i]) || (i > 0 && x[i] <= x[i-1]))
            throw std::invalid_argument("research terrain must be finite with increasing x");
    if (!std::isfinite(front_radius) || !std::isfinite(rear_radius) || front_radius <= 0. || rear_radius <= 0.)
        throw std::invalid_argument("research wheel radii must be positive");
    if (root_x_qpos >= nq || root_pitch_qpos >= nq || root_x_dof >= nv || root_pitch_dof >= nv)
        throw std::invalid_argument("research truth addresses exceed the model dimensions");
}
WireObject WheelieTruth::as_wire() const {
    return {{"time_s", time_s}, {"front_load_n", front_load_n}, {"rear_load_n", rear_load_n},
        {"front_clearance_m", front_clearance_m}, {"rear_clearance_m", rear_clearance_m},
        {"relative_pitch_rad", relative_pitch_rad}, {"position_m", position_m}, {"speed_mps", speed_mps},
        {"pitch_up_rad", pitch_up_rad}, {"pitch_rate_up_rad_s", pitch_rate_up_rad_s},
        {"axle_pitch_rad", axle_pitch_rad}, {"road_pitch_rad", road_pitch_rad},
        {"front_slip_mps", front_slip_mps}, {"rear_slip_mps", rear_slip_mps},
        {"com_x_m", com_x_m}, {"com_z_m", com_z_m}};
}
void WheelieTruth::validate()const {
    for(double const value:{time_s,front_load_n,rear_load_n,front_clearance_m,rear_clearance_m,relative_pitch_rad,position_m,speed_mps,
        pitch_up_rad,pitch_rate_up_rad_s,axle_pitch_rad,road_pitch_rad,front_slip_mps,rear_slip_mps,com_x_m,com_z_m})
        if(!std::isfinite(value))throw std::invalid_argument("nonfinite research truth");
    for(double const value:{time_s,front_load_n,rear_load_n,front_clearance_m,rear_clearance_m})
        if(value<0.)throw std::invalid_argument("negative truth time, contact load or clearance");
}
std::string_view WheelieTruth::contact_state()const {return classify_contact(front_load_n,rear_load_n,front_clearance_m,rear_clearance_m,relative_pitch_rad);}
WheelieTruth research_truth(const PhysicalSampleData &s, const TruthGeometry &g) {
    g.validate(s.qpos.size(),s.qvel.size());
    const auto &tires = o(s.channels, "tires");
    const auto &front = o(tires, "front"), &rear = o(tires, "rear");
    const auto &f = array(required(front, "wheel_axis_m"));
    const auto &r = array(required(rear, "wheel_axis_m"));
    const auto &com = array(required(o(s.channels, "mass"), "com_m"));
    if (f.size() != 3 || r.size() != 3 || com.size() != 3)
        throw std::invalid_argument("research truth vector width mismatch");
    const double zf = biketyre::np_interp(number(f[0]), g.x, g.z);
    const double zr = biketyre::np_interp(number(r[0]), g.x, g.z);
    const double dx = number(f[0]) - number(r[0]);
    WheelieTruth t;
    t.time_s = s.time_s;
    t.axle_pitch_rad = std::atan2(number(f[2]) - number(r[2]), dx);
    t.road_pitch_rad = std::atan2(zf - zr, dx);
    const double normal_z = std::max(std::abs(numeric::math_cos(t.road_pitch_rad)), 1e-6);
    const double supported = std::atan2((zf - zr) + (g.front_radius - g.rear_radius) / normal_z, dx);
    const double difference = t.axle_pitch_rad - supported;
    t.relative_pitch_rad = std::atan2(numeric::math_sin(difference), numeric::math_cos(difference));
    std::optional<biketyre::ProfileQuery> profile;
    const auto clearance=[&](const WireObject& wheel,const WireArray& center,double radius){
        const auto* penetration=find(wheel,"penetration_m");
        if(penetration&&!is_none(*penetration))return std::max(0.,-number(*penetration));
        if(!profile)profile.emplace(g.x,g.z,.0001,.05,20.);
        return std::max(0.,-profile->contact({number(center[0]),number(center[2])},radius,std::nullopt).delta);
    };
    t.front_clearance_m = clearance(front,f,g.front_radius);
    t.rear_clearance_m = clearance(rear,r,g.rear_radius);
    t.front_load_n = load(front);
    t.rear_load_n = load(rear);
    t.position_m = s.qpos.at(g.root_x_qpos);
    t.speed_mps = s.qvel.at(g.root_x_dof);
    t.pitch_up_rad = -s.qpos.at(g.root_pitch_qpos);
    t.pitch_rate_up_rad_s = -s.qvel.at(g.root_pitch_dof);
    t.front_slip_mps = n(front, "slip_mps");
    t.rear_slip_mps = n(rear, "slip_mps");
    t.com_x_m = number(com[0]);
    t.com_z_m = number(com[2]);
    t.validate();
    return t;
}
WireObject WheelieEpisode::as_wire() const {
    return {{"start_s", start_s}, {"end_s", end_s}, {"confirmed", confirmed},
        {"max_relative_pitch_rad", max_relative_pitch_rad}, {"max_front_clearance_m", max_front_clearance_m},
        {"min_front_load_n", min_front_load_n}, {"onset", onset}};
}
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
WheelieTracker::WheelieTracker(double persistence_s,double load_threshold,double clearance_threshold,double pitch_threshold)
    : persistence_(persistence_s),load_threshold_(load_threshold),clearance_threshold_(clearance_threshold),pitch_threshold_(pitch_threshold) {
    if (!std::isfinite(persistence_) || persistence_ < 0.)
        throw std::invalid_argument("wheelie persistence must be finite and nonnegative");
    for(double const value:{load_threshold_,clearance_threshold_,pitch_threshold_})
        if(!std::isfinite(value)||value<=0.)throw std::invalid_argument("wheelie thresholds must be positive and finite");
}
void WheelieTracker::reset() {
    last_end_.reset();candidate_s_=0.;active_=false;state_="uninitialized";state_times_.clear();episode_.reset();episodes_.clear();
    duration_=0.;candidate_time_=0.;wheelie_time_=0.;wheelie_episodes_=0;fraction_count_=0;front_unloaded_=0.;front_lift_=0.;flight_=0.;
    max_clearance_=0.;max_pitch_=0.;rear_slip_=0.;min_front_load_.reset();min_fraction_.reset();fraction_sum_=0.;max_pitch_rate_=0.;
}
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) argument order mirrors WheelieTracker.update
void WheelieTracker::update(const WheelieTruth &t, double dt, double delivered, std::optional<double> applied) {
    update(t,dt,WireObject{{"delivered_motor_nm",delivered},{"applied_motor_nm",opt(applied)},
        {"road_pitch_rad",t.road_pitch_rad},{"pitch_rate_up_rad_s",t.pitch_rate_up_rad_s},{"speed_mps",t.speed_mps}});
}
void WheelieTracker::update(const WheelieTruth &t,double dt,WireObject context) {
    t.validate();
    if (!std::isfinite(dt) || dt <= 0.) throw std::invalid_argument("metric interval must be positive");
    if (last_end_ && std::abs(t.time_s - *last_end_) > 1e-9)
        throw std::invalid_argument("metric intervals must be contiguous and unrepeated");
    const bool hysteresis = candidate_s_ > 0.;
    const auto raw = classify_contact(t.front_load_n,t.rear_load_n,t.front_clearance_m,t.rear_clearance_m,t.relative_pitch_rad,
        load_threshold_,clearance_threshold_,pitch_threshold_);
    const bool candidate = t.rear_load_n > load_threshold_ && t.front_load_n <= load_threshold_ * (hysteresis ? 2. : 1.) &&
        t.front_clearance_m > clearance_threshold_ * (hysteresis ? .5 : 1.) &&
        t.relative_pitch_rad > pitch_threshold_ * (hysteresis ? .5 : 1.);
    const bool onset = candidate && candidate_s_ == 0.;
    candidate_s_ = candidate ? candidate_s_ + dt : 0.;
    const bool active = candidate && candidate_s_ + 1e-12 >= persistence_;
    if (onset) {
        episode_ = WheelieEpisode{.start_s = t.time_s, .end_s = t.time_s, .confirmed = false,
            .max_relative_pitch_rad = t.relative_pitch_rad, .max_front_clearance_m = t.front_clearance_m,
            .min_front_load_n = t.front_load_n,
            .onset = std::move(context)};
    }
    if (candidate) {
        if (!episode_) throw std::logic_error("missing wheelie candidate episode");
        auto &e = *episode_;
        e.end_s = t.time_s;
        e.confirmed = e.confirmed || active;
        e.max_relative_pitch_rad = std::max(e.max_relative_pitch_rad, t.relative_pitch_rad);
        e.max_front_clearance_m = std::max(e.max_front_clearance_m, t.front_clearance_m);
        e.min_front_load_n = std::min(e.min_front_load_n, t.front_load_n);
    } else if (episode_) {
        episodes_.push_back(*episode_);
        episode_.reset();
    }
    if (active && !active_) ++wheelie_episodes_;
    state_ = active ? "wheelie" : raw;
    active_ = active;
    duration_ += dt;
    candidate_time_ += candidate ? dt : 0.;
    wheelie_time_ += active ? dt : 0.;
    front_unloaded_ += t.front_load_n <= load_threshold_ ? dt : 0.;
    front_lift_ += raw == "front_lift" || raw == "wheelie_candidate" ? dt : 0.;
    flight_ += raw == "flight" ? dt : 0.;
    max_clearance_ = std::max(max_clearance_, t.front_clearance_m);
    max_pitch_ = std::max(max_pitch_, t.relative_pitch_rad);
    if (t.rear_load_n > load_threshold_) rear_slip_ += std::abs(t.rear_slip_mps) * dt;
    min_front_load_ = min_front_load_ ? std::min(*min_front_load_, t.front_load_n) : t.front_load_n;
    if (t.rear_load_n > load_threshold_) {
        const double fraction = t.front_load_n / (t.front_load_n + t.rear_load_n);
        min_fraction_ = min_fraction_ ? std::min(*min_fraction_, fraction) : fraction;
        fraction_sum_ += fraction;
        ++fraction_count_;
        max_pitch_rate_ = std::max(max_pitch_rate_, t.pitch_rate_up_rad_s);
    }
    state_times_[std::string(raw)] += dt;
    last_end_ = t.time_s + dt;
}
WireObject WheelieTracker::metrics() const {
    WireArray records;
    records.reserve(episodes_.size() + (episode_ ? 1U : 0U));
    for (const auto &e : episodes_) records.emplace_back(e.as_wire());
    if (episode_) records.emplace_back(episode_->as_wire());
    WireObject states;
    for (const auto &[key, value] : state_times_) states.emplace_back(key, value);
    return {{"duration_s", duration_}, {"wheelie_candidate_time_s", candidate_time_},
        {"wheelie_time_s", wheelie_time_}, {"wheelie_episodes", wheelie_episodes_},
        {"front_unloaded_time_s", front_unloaded_}, {"front_lift_time_s", front_lift_},
        {"flight_time_s", flight_}, {"max_front_clearance_m", max_clearance_},
        {"max_relative_pitch_rad", max_pitch_}, {"rear_slip_distance_m", rear_slip_},
        {"min_front_load_n", opt(min_front_load_)}, {"front_load_fraction_min", opt(min_fraction_)},
        {"max_pitch_rate_up_rad_s", max_pitch_rate_},
        {"front_load_fraction_mean", fraction_count_ ? Wire(fraction_sum_ / static_cast<double>(fraction_count_)) : Wire(nullptr)},
        {"wheelie_episode_records", std::move(records)}, {"contact_state_time_s", std::move(states)}};
}
ResearchQuality research_quality(const WireObject &energy, double maximum_ratio) {
    if (!std::isfinite(maximum_ratio) || maximum_ratio <= 0.)
        throw std::invalid_argument("maximum energy residual ratio must be positive and finite");
    double scale = optional_number(energy, "energy_scale_j", 1.);
    if (scale < 0.) throw std::invalid_argument("initial energy scale must be nonnegative");
    scale += std::abs(optional_number(energy, "external_work_j", 0.));
    const double source = find(energy, "source_positive_work_j")
        ? n(energy, "source_positive_work_j") : optional_number(energy, "active_work_j", 0.);
    scale += std::abs(source);
    if (scale == 0.) throw std::runtime_error("energy quality has a zero energy scale");
    const double ratio = std::abs(optional_number(energy, "residual_j", 0.)) / scale;
    const double electrical = optional_number(energy, "electrical_residual_j", 0.);
    const double budget = std::max(1., std::abs(optional_number(energy, "electrical_work_j", 0.)));
    return {.residual_ratio = ratio,
        .acceptable = ratio <= maximum_ratio && std::abs(electrical) <= 1e-7 * budget};
}
} // namespace runtime
