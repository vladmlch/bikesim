#include "research_metrics.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include "../numeric_sincos.hpp"
#include "../numeric_sum.hpp"
#include "../spatial/frames.hpp"
#include "../tyre/profile.hpp"

namespace runtime {
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
WheelieTruth research_truth_spatial(const PhysicalSampleData &s, const TruthGeometry &g,
                                    const SpatialTruthGeometry &sp,
                                    const WireObject &tires, const WireArray &com) {
    const auto &front = o(tires, "front"), &rear = o(tires, "rear");
    const auto &f = array(required(front, "wheel_axis_m"));
    const auto &r = array(required(rear, "wheel_axis_m"));
    const bike::spatial::Quat q_raw{
        s.qpos.at(sp.root_free_qpos + 3U), s.qpos.at(sp.root_free_qpos + 4U),
        s.qpos.at(sp.root_free_qpos + 5U), s.qpos.at(sp.root_free_qpos + 6U)};
    const bike::spatial::Quat q_F = bike::spatial::quat_normalized(q_raw);
    if (!std::isfinite(q_F[0]))
        throw std::invalid_argument("spatial research truth requires finite root_free quaternion");
    const std::array<double, 3> v_F{
        s.qvel.at(sp.root_free_dof + 0U), s.qvel.at(sp.root_free_dof + 1U), s.qvel.at(sp.root_free_dof + 2U)};
    const std::array<double, 3> omega_body{
        s.qvel.at(sp.root_free_dof + 3U), s.qvel.at(sp.root_free_dof + 4U), s.qvel.at(sp.root_free_dof + 5U)};
    const double fx = number(f[0]), fy = number(f[1]), fz = number(f[2]);
    const double rx = number(r[0]), ry = number(r[1]), rz = number(r[2]);
    const auto y_F = bike::spatial::rotate_body_to_world(q_F, {0., 1., 0.});
    // y_F x Z = (y_F[1], -y_F[0], 0): remains nonsingular across +-90 deg pitch loop-outs.
    const double sx = y_F[1], sy = -y_F[0];
    const double s_norm = std::hypot(sx, sy);
    const double ux = s_norm > 1e-8 ? sx / s_norm : 1.0;
    const double uy = s_norm > 1e-8 ? sy / s_norm : 0.0;
    const double lat_x = -uy, lat_y = ux;
    const double dx = fx - rx, dy = fy - ry, dz = fz - rz;
    const double d_s = dx * ux + dy * uy;
    const double d_lat = dx * lat_x + dy * lat_y;
    const auto sf = sp.terrain.sample_fn(sp.terrain.ctx, fx, fy);
    const auto sr = sp.terrain.sample_fn(sp.terrain.ctx, rx, ry);
    if (sf.status != bike::terrain::QueryStatus3d::ok || sr.status != bike::terrain::QueryStatus3d::ok ||
        !std::isfinite(sf.height_m) || !std::isfinite(sr.height_m))
        throw std::runtime_error("spatial truth terrain sample out of bounds");
    const double z_f = sf.height_m, z_r = sr.height_m;
    constexpr double kBaselineHalf = 0.25;
    const double mx = 0.5 * (fx + rx), my = 0.5 * (fy + ry);
    const auto s_plus = sp.terrain.sample_fn(sp.terrain.ctx, mx + kBaselineHalf * lat_x, my + kBaselineHalf * lat_y);
    const auto s_minus = sp.terrain.sample_fn(sp.terrain.ctx, mx - kBaselineHalf * lat_x, my - kBaselineHalf * lat_y);
    double s_lat = 0.0;
    if (s_plus.status == bike::terrain::QueryStatus3d::ok &&
        s_minus.status == bike::terrain::QueryStatus3d::ok) {
        s_lat = (s_plus.height_m - s_minus.height_m) / (2.0 * kBaselineHalf);
    } else {
        const auto s_mid = sp.terrain.sample_fn(sp.terrain.ctx, mx, my);
        if (s_mid.status == bike::terrain::QueryStatus3d::ok) {
            if (s_plus.status == bike::terrain::QueryStatus3d::ok)
                s_lat = (s_plus.height_m - s_mid.height_m) / kBaselineHalf;
            else if (s_minus.status == bike::terrain::QueryStatus3d::ok)
                s_lat = (s_mid.height_m - s_minus.height_m) / kBaselineHalf;
        }
    }
    double s_long = 0.0;
    if (std::abs(d_s) >= 1e-3) {
        s_long = ((z_f - z_r) - s_lat * d_lat) / d_s;
    } else {
        const auto s_fwd_p = sp.terrain.sample_fn(
            sp.terrain.ctx, mx + 0.5 * ux, my + 0.5 * uy);
        const auto s_fwd_m = sp.terrain.sample_fn(
            sp.terrain.ctx, mx - 0.5 * ux, my - 0.5 * uy);
        if (s_fwd_p.status == bike::terrain::QueryStatus3d::ok &&
            s_fwd_m.status == bike::terrain::QueryStatus3d::ok) {
            s_long = s_fwd_p.height_m - s_fwd_m.height_m;
        }
    }
    const double n_norm = std::sqrt(1.0 + s_long * s_long + s_lat * s_lat);
    const double nx = (-s_long * ux - s_lat * lat_x) / n_norm;
    const double ny = (-s_long * uy - s_lat * lat_y) / n_norm;
    const double nz = 1.0 / n_norm;
    const double n_z_sec = std::max(nz, 1e-6);
    const double sin_gamma = y_F[0] * nx + y_F[1] * ny + y_F[2] * nz;
    const double c_gamma = std::sqrt(std::max(0.0, 1.0 - sin_gamma * sin_gamma));
    const double h_perp_f = sp.front_crown_radius_m + (g.front_radius - sp.front_crown_radius_m) * c_gamma;
    const double h_perp_r = sp.rear_crown_radius_m + (g.rear_radius - sp.rear_crown_radius_m) * c_gamma;
    WheelieTruth t;
    t.time_s = s.time_s;
    t.axle_pitch_rad = std::atan2(dz, d_s);
    double theta_supported = 0.0;
    if (d_s > 1e-6) {
        t.road_pitch_rad = std::atan2((z_f - z_r) - s_lat * d_lat, d_s);
        theta_supported = std::atan2((z_f - z_r) + (h_perp_f - h_perp_r) / n_z_sec, d_s);
    } else {
        const double span = std::max(std::hypot(d_s, dz), 1e-6);
        t.road_pitch_rad = std::atan2(s_long, 1.0);
        theta_supported = std::atan2(s_long * span + (h_perp_f - h_perp_r) / n_z_sec, span);
    }
    const double diff = t.axle_pitch_rad - theta_supported;
    t.relative_pitch_rad = std::atan2(std::sin(diff), std::cos(diff));
    const auto clearance = [&](const WireObject &wheel, double hx, double hy, double hz, double radius) {
        const auto *penetration = find(wheel, "penetration_m");
        if (penetration && !is_none(*penetration))
            return std::max(0., -number(*penetration));
        const auto hit = sp.terrain.closest_point_fn(
            sp.terrain.ctx, bike::terrain::Vec3d{hx, hy, hz}, radius + 0.15);
        if (hit.status != bike::terrain::QueryStatus3d::ok || !std::isfinite(hit.signed_distance_m))
            throw std::runtime_error("spatial truth closest_point query failed");
        return std::max(0., hit.signed_distance_m - radius);
    };
    t.front_clearance_m = clearance(front, fx, fy, fz, g.front_radius);
    t.rear_clearance_m = clearance(rear, rx, ry, rz, g.rear_radius);
    t.front_load_n = load(front);
    t.rear_load_n = load(rear);
    t.position_m = s.qpos.at(sp.root_free_qpos);
    const auto x_F = bike::spatial::rotate_body_to_world(q_F, {1., 0., 0.});
    t.speed_mps = v_F[0] * x_F[0] + v_F[1] * x_F[1] + v_F[2] * x_F[2];
    t.pitch_up_rad = std::asin(std::clamp(x_F[2], -1.0, 1.0));
    const double phi = std::atan2(2.0 * (q_F[0] * q_F[1] + q_F[2] * q_F[3]),
                                  1.0 - 2.0 * (q_F[1] * q_F[1] + q_F[2] * q_F[2]));
    t.pitch_rate_up_rad_s = -(std::cos(phi) * omega_body[1] - std::sin(phi) * omega_body[2]);
    t.front_slip_mps = n(front, "slip_mps");
    t.rear_slip_mps = n(rear, "slip_mps");
    t.com_x_m = number(com[0]);
    t.com_z_m = number(com[2]);
    t.validate();
    return t;
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
    if (spatial != nullptr) {
        if (x.size() != z.size())
            throw std::invalid_argument("research terrain requires equal x/z arrays");
        for (std::size_t i = 0; i < x.size(); ++i)
            if (!std::isfinite(x[i]) || !std::isfinite(z[i]) || (i > 0 && x[i] <= x[i - 1]))
                throw std::invalid_argument("research terrain must be finite with increasing x");
        if (!std::isfinite(front_radius) || !std::isfinite(rear_radius) || front_radius <= 0. || rear_radius <= 0.)
            throw std::invalid_argument("research wheel radii must be positive");
        const auto &sp = *spatial;
        if (!std::isfinite(sp.front_crown_radius_m) || !std::isfinite(sp.rear_crown_radius_m) ||
            sp.front_crown_radius_m < 0. || sp.rear_crown_radius_m < 0. ||
            sp.front_crown_radius_m >= front_radius || sp.rear_crown_radius_m >= rear_radius)
            throw std::invalid_argument("spatial wheel crown radii must lie in [0, outer_radius)");
        if (!sp.terrain.valid())
            throw std::invalid_argument("spatial research truth requires valid 3D terrain callbacks");
        if (sp.root_free_qpos + 7U > nq || sp.root_free_dof + 6U > nv)
            throw std::invalid_argument("spatial research truth addresses exceed the model dimensions");
        return;
    }
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
    if (g.spatial != nullptr)
        return research_truth_spatial(s, g, *g.spatial, tires, com);
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
    return {.residual_ratio = ratio, .acceptable = ratio <= maximum_ratio};
}
} // namespace runtime
