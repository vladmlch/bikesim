// runtime/control.cpp — implementations of the owned step's helper layer.
// Every function mirrors the Python statement it names; ordering is the
// FP contract. No nanobind types in this TU.
#include "control.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <ranges>
#include <stdexcept>

#include "../model_access.hpp"
#include "../engine_call.hpp"
#include "../spatial/frames.hpp"
#include "../spatial/topology.hpp"
#include "../terrain/source3d.hpp"
#include "../tyre/profile.hpp"
#include "../validation.hpp"

namespace runtime {

void control_validate_for(const RideControl &control,
                          bool articulated_planar, std::string_view drive_mode) {
    // RideControl.__post_init__ (control.py:25-32): each present torque or
    // rate command is a finite nonnegative scalar and a supplied posture is
    // a valid RiderPosture. The POD cannot run construction invariants, so
    // the admission gate re-checks the domain for every control source
    // (wire decode already gates; the policy ABI does not).
    for (const std::pair<std::optional<double>, const char *> &entry : {
             std::pair{control.motor_torque_nm, "motor_torque_nm"},
             std::pair{control.motor_limit_nm, "motor_limit_nm"},
             std::pair{control.human_torque_nm, "human_torque_nm"},
             std::pair{control.crank_target_rate_rad_s, "crank_target_rate_rad_s"}})
        if (entry.first && (!std::isfinite(*entry.first) || *entry.first < 0.))
            throw std::invalid_argument(std::string("invalid ") + entry.second);
    if (control.posture) rider::validate_posture(*control.posture);
    // control.py:36-41: a physical plant admits effort commands only
    // in either effort drive mode; coast/speed commands remain empty.
    if ((control.motor_torque_nm || control.motor_limit_nm || control.human_torque_nm) &&
        drive_mode != "articulated_effort" && drive_mode != "crank_effort")
        throw std::invalid_argument("torque commands require a physical effort drive mode");
    if ((control.posture.has_value() || !control.rider_enabled ||
         control.crank_target_rate_rad_s.has_value()) &&
        !articulated_planar)
        throw std::invalid_argument("rider commands require "
                                    "articulated_planar");
}

// ---- ControlClock -----------------------------------------------------
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed (timestep, period) Python signature
ControlClock::ControlClock(double timestep, double period)
    : timestep_s(timestep), period_s(period) {
    if (!std::isfinite(timestep_s) || !std::isfinite(period_s) ||
        std::min(timestep_s, period_s) <= 0.)
        throw std::invalid_argument(
            "control clock requires positive finite durations");
    // Python round() is half-to-even; the value is a ratio of
    // well-separated step durations so llround's half-away behaviour is
    // only distinguishable exactly on a .5 boundary, which the 1e-9 check
    // below rejects anyway.
    const double ratio = period_s / timestep_s;
    // Reject before conversion: finite input durations can still overflow
    // their quotient, and converting infinity/out-of-range to int64 is UB.
    if (!std::isfinite(ratio) || ratio >= 0x1p63)
        throw std::invalid_argument(
            "control period must be a representable integer multiple of the timestep");
    const auto steps = static_cast<std::int64_t>(std::nearbyint(ratio));
    if (steps < 1 ||
        std::abs(static_cast<double>(steps) * timestep_s - period_s) > 1e-9)
        throw std::invalid_argument(
            "control period must be an integer multiple of the timestep");
    steps_per_period = steps;
}

bool ControlClock::is_tick(std::int64_t step) const {
    if (step < 0)
        throw std::invalid_argument(
            "control clock requires a nonnegative physics step");
    return step % steps_per_period == 0;
}

void ControlClock::hold(rider::NamedEntries<double> torques) {
    held_ = std::move(torques);
}

const rider::NamedEntries<double> &ControlClock::held() const {
    if (!held_.has_value())
        throw std::runtime_error("control clock holds no command yet");
    return *held_;
}

// ---- GroundedFilter ----------------------------------------------------
GroundedFilter::GroundedFilter(double hold) : hold_s(hold) {
    if (!std::isfinite(hold_s) || hold_s < 0.)
        throw std::invalid_argument("hold_s must be finite and nonnegative");
    reset();
}

void GroundedFilter::reset() noexcept {
    last_time_s.reset();
    last_loaded_s.reset();
    value = false;
}

bool GroundedFilter::update(bool raw_grounded, double time_s) {
    if (!std::isfinite(time_s))
        throw std::invalid_argument("non-finite timestamp");
    if (last_time_s.has_value() && time_s < *last_time_s)
        throw std::invalid_argument(
            "timestamp moved backwards; reset required");
    if (last_time_s.has_value() && time_s == *last_time_s) return value;
    last_time_s = time_s;
    if (raw_grounded) last_loaded_s = time_s;
    value = raw_grounded ||
            (last_loaded_s.has_value() &&
             time_s - *last_loaded_s < hold_s);
    return value;
}

// ---- BridgedLoad -------------------------------------------------------
BridgedLoad::BridgedLoad(int steps) : dropout_steps(steps) {}

double BridgedLoad::update(double raw_load_n) noexcept {
    if (raw_load_n > kContactLoadThresholdN) {
        held_load_n = raw_load_n;
        unloaded_steps = 0;
    } else {
        ++unloaded_steps;
        if (unloaded_steps > dropout_steps) held_load_n = 0.;
    }
    return held_load_n;
}

void BridgedLoad::reset() noexcept {
    held_load_n = 0.;
    unloaded_steps = 0;
}

// ---- RuntimeContactQuery -----------------------------------------------
RuntimeContactQuery::RuntimeContactQuery(const mjModel *model,
                                         int dropout_steps)
    : front_load(dropout_steps), rear_load(dropout_steps), model_(model) {
    if (model == nullptr)
        throw std::invalid_argument("contact query requires a model");
    if (dropout_steps < 0)
        throw std::invalid_argument("dropout_steps must be non-negative, got " +
                                    std::to_string(dropout_steps));
    for (const char *name : {"terrain", "catch_plane"})
        terrain_ids.push_back(
            model_access::resolve_id(model, mjOBJ_GEOM, name));
    front_id = model_access::resolve_id(model, mjOBJ_GEOM,
                                        "geom_front_contact");
    rear_id = model_access::resolve_id(model, mjOBJ_GEOM,
                                       "geom_rear_contact");
    handlebar_id = model_access::resolve_id(model, mjOBJ_GEOM,
                                            "geom_handlebar");
}

double RuntimeContactQuery::handlebar_load(const mjModel *model,
                                           const mjData *data) const {
    // contacts.py:289-300 — single pass over data.contact, mj_contactForce
    // normal magnitude per handlebar-terrain row.
    if(model!=model_||data==nullptr)
        throw std::invalid_argument("handlebar query model/data owner mismatch");
    // ncon is an int, so a row index can never leave int range; only a
    // corrupted negative count needs rejecting before it reaches counted().
    if(data->ncon<0)
        throw std::invalid_argument("handlebar contact row count is negative");
    double load_n = 0.;
    const auto contacts = std::views::counted(data->contact, data->ncon);
    for (int i = 0; i < data->ncon; ++i) {
        const mjContact &contact = contacts[static_cast<std::size_t>(i)];
        const int geom1 = contact.geom1, geom2 = contact.geom2;
        const bool g1_terrain = std::ranges::find(terrain_ids, geom1) !=
                                terrain_ids.end();
        const bool g2_terrain = std::ranges::find(terrain_ids, geom2) !=
                                terrain_ids.end();
        if (!((geom1 == handlebar_id && g2_terrain) ||
              (geom2 == handlebar_id && g1_terrain)))
            continue;
        struct Call{const mjModel* model;const mjData* data;int row;double* result;} call{.model=model,.data=data,.row=i,.result=force_.data()};
        engine::ErrorBuffer error;
        if(!engine::invoke([](void* raw)noexcept{auto const& c=*static_cast<Call*>(raw);mj_contactForce(c.model,c.data,c.row,c.result);},&call,error))
            engine::throw_failure(error);
        validation::derived(force_[0],"native handlebar contact load");
        load_n += std::abs(force_[0]);
    }
    return validation::derived(load_n,"native handlebar summed contact load");
}

void RuntimeContactQuery::reset() noexcept {
    front_load.reset();
    rear_load.reset();
    front_controller.reset();
    rear_controller.reset();
}

// ---- physical_crash.py ---------------------------------------------------
CrashGeomIds crash_geom_ids(const mjModel *model) {
    CrashGeomIds out;
    for (int i = 0; i < model->ngeom; ++i) {
        const char *name = mj_id2name(model, mjOBJ_GEOM, i);
        const std::string_view text = name == nullptr ? "" : name;
        if (text == "catch_plane")
            out.catch_ids.push_back(i);
        else if (text == "terrain")
            out.terrain_ids.push_back(i);
        if (text.starts_with("geom_rider_")) out.rider_ids.push_back(i);
    }
    return out;
}

std::optional<std::string> physical_contact_crash(
    const mjModel *model, const mjData *data,
    const std::vector<int> &catch_ids, const std::vector<int> &terrain_ids,
    const std::vector<int> &rider_ids, double load_threshold_n) {
    if (!std::isfinite(load_threshold_n) || load_threshold_n < 0.)
        throw std::invalid_argument(
            "contact threshold must be finite and nonnegative");
    std::array<mjtNum, 6> wrench{};
    const auto contacts = std::views::counted(data->contact, data->ncon);
    for (int i = 0; i < data->ncon; ++i) {
        const mjContact &contact = contacts[static_cast<std::size_t>(i)];
        const int g1 = contact.geom1, g2 = contact.geom2;
        const bool catch_hit = std::ranges::find(catch_ids, g1) !=
                                   catch_ids.end() ||
                               std::ranges::find(catch_ids, g2) !=
                                   catch_ids.end();
        const bool rider_ground =
            !catch_hit &&
            (std::ranges::find(terrain_ids, g1) != terrain_ids.end() ||
             std::ranges::find(terrain_ids, g2) != terrain_ids.end()) &&
            (std::ranges::find(rider_ids, g1) != rider_ids.end() ||
             std::ranges::find(rider_ids, g2) != rider_ids.end());
        if (!catch_hit && !rider_ground) continue;
        mj_contactForce(model, data, i, wrench.data());
        if (wrench[0] <= load_threshold_n) continue;
        return std::string(catch_hit ? "catch_plane_contact"
                                     : "rider_ground_contact");
    }
    return std::nullopt;
}

// ---- BalanceMonitor -----------------------------------------------------
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed (floor, dwell, grace) Python signature
BalanceMonitor::BalanceMonitor(double floor, double dwell,
                               double grace)
    : floor_mps(floor), dwell_s(dwell), grace_s(grace) {
    if (!std::isfinite(floor_mps) || floor_mps <= 0.)
        throw std::invalid_argument("balance floor");
    if (!std::isfinite(dwell_s) || dwell_s <= 0.)
        throw std::invalid_argument("balance dwell");
    if (!std::isfinite(grace_s) || grace_s < 0.)
        throw std::invalid_argument("balance grace");
    reset();
}

void BalanceMonitor::reset() noexcept {
    event.reset();
    current_event.reset();
    low_speed_s = 0.;
    grace_left_s = grace_s;
    started_s.reset();
    low_started_s.reset();
    last_time_s.reset();
}

std::optional<BalanceLostEvent> BalanceMonitor::update(
    double time_s, double position_m, double speed_mps, bool riding) {
    if (!std::isfinite(time_s) || time_s < 0.)
        throw std::invalid_argument("balance time");
    if (!std::isfinite(position_m))
        throw std::invalid_argument("balance position");
    if (!std::isfinite(speed_mps))
        throw std::invalid_argument("balance speed");
    if (last_time_s.has_value() && time_s < *last_time_s)
        throw std::invalid_argument(
            "balance monitor time must not rewind");
    last_time_s = time_s;
    if (riding && !started_s.has_value()) started_s = time_s;
    grace_left_s = !started_s.has_value()
                       ? grace_s
                       : std::max(0., *started_s + grace_s - time_s);
    if (!riding || grace_left_s > 1e-12 || speed_mps >= floor_mps) {
        low_started_s.reset();
        low_speed_s = 0.;
        current_event.reset();
        return std::nullopt;
    }
    if (!low_started_s.has_value()) low_started_s = time_s;
    low_speed_s = time_s - *low_started_s;
    if (low_speed_s >= dwell_s - 1e-12 && !current_event.has_value()) {
        current_event = BalanceLostEvent{.time_s = time_s,
                                         .position_m = position_m,
                                         .speed_mps = speed_mps};
        if (!event.has_value()) event = current_event;
    }
    return current_event;
}

WireObject BalanceMonitor::observation() const {
    WireObject out;
    out.emplace_back("balance_lost", Wire(current_event.has_value()));
    out.emplace_back("low_speed_s", Wire(low_speed_s));
    out.emplace_back("grace_left_s", Wire(grace_left_s));
    out.emplace_back("balance_lost_at_m",
                     current_event.has_value()
                         ? Wire(current_event->position_m)
                         : Wire());
    out.emplace_back("balance_lost_time_s",
                     current_event.has_value()
                         ? Wire(current_event->time_s)
                         : Wire());
    return out;
}

// ---- CrashDetector -------------------------------------------------------
CrashDetector::CrashDetector(const mjModel *model,
                             double pitch_limit_deg) {
    if (pitch_limit_deg <= 0.)
        throw std::invalid_argument(
            "pitch_limit_deg must be positive, got " +
            std::to_string(pitch_limit_deg));
    const int pitch = mj_name2id(model, mjOBJ_JOINT, "root_pitch");
    if(pitch<0) {
        const int root=mj_name2id(model,mjOBJ_JOINT,"root_free");
        // counted views keep the bounds-checked idiom of this file; root is
        // a valid joint index whenever mj_name2id returned it.
        const auto joint_types=std::views::counted(model->jnt_type,model->njnt);
        const auto joint_bodies=std::views::counted(model->jnt_bodyid,model->njnt);
        if(root>=0 && joint_types[static_cast<std::size_t>(root)]==mjJNT_FREE) {
            spatial_body_=joint_bodies[static_cast<std::size_t>(root)];
            nq_=model->nq;
            pitch_qposadr=root_x_qposadr=-1;
            pitch_limit_rad=pitch_limit_deg*(std::numbers::pi/180.);
            return;
        }
    }
    if (pitch < 0)
        throw std::invalid_argument(
            "model has no joint 'root_pitch'; the virtual rider acts on "
            "chassis pitch");
    const int root_x = mj_name2id(model, mjOBJ_JOINT, "root_x");
    if (root_x < 0)
        throw std::invalid_argument(
            "model has no joint 'root_x'; the crash detector reports "
            "track position");
    const auto qposadr =
        std::views::counted(model->jnt_qposadr, model->njnt);
    pitch_qposadr = static_cast<int>(qposadr[static_cast<std::size_t>(pitch)]);
    root_x_qposadr =
        static_cast<int>(qposadr[static_cast<std::size_t>(root_x)]);
    nq_ = model->nq;
    pitch_limit_rad = pitch_limit_deg * (std::numbers::pi / 180.0);
}

std::optional<CrashEvent>
CrashDetector::check(const mjData *data, const RuntimeContacts &contacts) {
    if (event_.has_value()) return event_;
    if(spatial_body_>=0) {
        const auto body=static_cast<std::ptrdiff_t>(spatial_body_);
        const auto xquat=std::views::counted(data->xquat,4*body+4);
        const auto xpos=std::views::counted(data->xpos,3*body+3);
        const auto offset=static_cast<std::size_t>(4*body);
        const bike::spatial::Quat q{xquat[offset],xquat[offset+1],xquat[offset+2],xquat[offset+3]};
        const auto attitude=bike::spatial::euler_zyx(q);
        const bool pitch_over=attitude.valid && std::abs(attitude.pitch_rad)>pitch_limit_rad;
        if(pitch_over || contacts.handlebar_in_contact())
            event_=CrashEvent{.cause=pitch_over?"pitch_over":"handlebar_contact", .time_s=data->time,
                .position_m=xpos[static_cast<std::size_t>(3*body)], .pitch_rad=attitude.pitch_rad};
        return event_;
    }
    const auto qpos = std::views::counted(data->qpos, nq_);
    const double pitch_rad = qpos[static_cast<std::size_t>(pitch_qposadr)];
    if (std::abs(pitch_rad) > pitch_limit_rad)
        event_ = CrashEvent{
            .cause = "pitch_over",
            .time_s = data->time,
            .position_m = qpos[static_cast<std::size_t>(root_x_qposadr)],
            .pitch_rad = pitch_rad};
    else if (contacts.handlebar_in_contact())
        event_ = CrashEvent{
            .cause = "handlebar_contact",
            .time_s = data->time,
            .position_m = qpos[static_cast<std::size_t>(root_x_qposadr)],
            .pitch_rad = pitch_rad};
    return event_;
}

// ---- rider_state.py -------------------------------------------------------
RoadProfile::RoadProfile(std::span<const std::array<double, 2>> vertices) {
    if (vertices.size() < 2) {
        error_ = "road vertices must be an (N,2) X-Z profile";
        return;
    }
    // Strictly increasing x — np.diff(verts[:,0]) <= 0 check, once.
    for (std::size_t i = 1; i < vertices.size(); ++i)
        if (!(vertices[i][0] > vertices[i - 1][0])) {
            error_ = "road profile x must strictly increase";
            return;
        }
    x_.reserve(vertices.size());
    z_.reserve(vertices.size());
    for (const auto &v : vertices) {
        x_.push_back(v[0]);
        z_.push_back(v[1]);
    }
}

std::vector<RoadSample>
RoadProfile::samples(std::span<const double> wheel_x_m, double lookahead_m,
                     double spacing_m) const {
    if (error_.has_value())
        throw std::invalid_argument(*error_);
    if (!std::isfinite(lookahead_m) || lookahead_m < 0.)
        throw std::invalid_argument("road lookahead");
    if (!std::isfinite(spacing_m) || spacing_m <= 0.)
        throw std::invalid_argument("road sample spacing");
    if (wheel_x_m.empty())
        throw std::invalid_argument(
            "rider road preview needs wheel positions");
    for (const double x : wheel_x_m)
        if (!std::isfinite(x))
            throw std::invalid_argument("wheel x");
    const double front = *std::ranges::max_element(wheel_x_m);
    // xs = wheels ∪ {front + (k+1)*spacing, k<count} ∪ {front+lookahead}
    std::vector<double> xs(wheel_x_m.begin(), wheel_x_m.end());
    const int count = static_cast<int>(
        std::floor(lookahead_m / spacing_m + 1e-12));
    xs.reserve(wheel_x_m.size() + static_cast<std::size_t>(count) + 1);
    for (int k = 0; k < count; ++k)
        xs.push_back(front + static_cast<double>(k + 1) * spacing_m);
    xs.push_back(front + lookahead_m);
    std::ranges::sort(xs);
    // np.interp on contiguous columns + per-segment grade (the oracle's
    // exact NumPy lowerings); the columns are the cached x_/z_.
    const std::vector<double> &profile_x = x_;
    const std::vector<double> &profile_z = z_;
    std::vector<RoadSample> out;
    out.reserve(xs.size());
    for (const double x : xs) {
        const double height =
            biketyre::np_interp(x, profile_x, profile_z);
        // np.searchsorted(xs, x, side='right') - 1 clipped to [0, n-2]:
        // the first segment whose right endpoint strictly exceeds x.
        const auto it = std::ranges::upper_bound(profile_x, x);
        auto seg = std::distance(profile_x.begin(), it) - 1;
        seg = std::clamp<std::ptrdiff_t>(
            seg, 0, static_cast<std::ptrdiff_t>(profile_x.size()) - 2);
        const double dx = profile_x[static_cast<std::size_t>(seg) + 1] -
                          profile_x[static_cast<std::size_t>(seg)];
        const double grade =
            dx <= 0. ? 0.
                     : (profile_z[static_cast<std::size_t>(seg) + 1] -
                        profile_z[static_cast<std::size_t>(seg)]) /
                           dx;
        out.push_back(RoadSample{.x_m = x,
                                 .height_m = height,
                                 .grade = grade});
        for (const double v : {x, height, grade})
            if (!std::isfinite(v))
                throw std::invalid_argument("road sample field");
    }
    return out;
}

std::vector<RoadSample>
road_samples(std::span<const std::array<double, 2>> vertices,
             std::span<const double> wheel_x_m, double lookahead_m,
             double spacing_m) {
    return RoadProfile(vertices).samples(wheel_x_m, lookahead_m, spacing_m);
}

std::vector<RoadSample> spatial_road_samples(
    const bike::terrain::Terrain3dQueryView& terrain,
    std::span<const std::array<double, 3>> wheels,
    std::array<double, 3> forward, double lookahead, double spacing) {
    constexpr std::size_t maximum_samples = 65536;
    if (wheels.empty() || wheels.size() > maximum_samples - 1 ||
        !std::isfinite(lookahead) || lookahead < 0. ||
        !std::isfinite(spacing) || spacing <= 0. ||
        !std::ranges::all_of(forward, [](double x) { return std::isfinite(x); }))
        throw std::invalid_argument("spatial.road_preview: invalid wheels, heading or sample spacing");
    const double heading_length = std::hypot(forward[0], forward[1]);
    if (!std::isfinite(heading_length) || heading_length <= 1e-12)
        throw std::invalid_argument("spatial.road_preview.heading: horizontal heading is undefined");
    forward[0] /= heading_length;
    forward[1] /= heading_length;
    const double count_real = std::floor(lookahead / spacing);
    if (!std::isfinite(count_real) ||
        count_real > static_cast<double>(maximum_samples - wheels.size() - 1))
        throw std::invalid_argument("spatial.road_preview: sample budget exceeded");
    const auto count = static_cast<std::size_t>(count_real);
    std::vector<std::pair<double, std::size_t>> order;
    order.reserve(wheels.size());
    for (std::size_t i = 0; i < wheels.size(); ++i) {
        if (!std::ranges::all_of(wheels[i], [](double x) { return std::isfinite(x); }))
            throw std::invalid_argument("spatial.road_preview.wheels: nonfinite position");
        const double along = (wheels[i][0] - wheels.front()[0]) * forward[0] +
                             (wheels[i][1] - wheels.front()[1]) * forward[1];
        if (!std::isfinite(along))
            throw std::invalid_argument("spatial.road_preview.wheels: unrepresentable separation");
        order.emplace_back(along, i);
    }
    std::ranges::sort(order);
    std::vector<RoadSample> result;
    result.reserve(wheels.size() + count + 1);
    const auto append = [&](double x, double y, bool required) {
        const auto sample = terrain.sample(x, y);
        if (!required && sample.status == bike::terrain::QueryStatus3d::outside) return false;
        if (sample.status != bike::terrain::QueryStatus3d::ok ||
            !std::isfinite(sample.height_m) || !std::isfinite(sample.normal_world[0]) ||
            !std::isfinite(sample.normal_world[1]) || !std::isfinite(sample.normal_world[2]) ||
            sample.normal_world[2] <= 0.)
            throw std::invalid_argument("spatial.road_preview.terrain: invalid physical sample");
        const double grade = -(sample.normal_world[0] * forward[0] +
                               sample.normal_world[1] * forward[1]) / sample.normal_world[2];
        if (!std::isfinite(grade))
            throw std::invalid_argument("spatial.road_preview.grade: nonfinite directional slope");
        result.push_back({.x_m = x, .height_m = sample.height_m, .grade = grade});
        return true;
    };
    for (const auto& [along, index] : order) {
        (void)along;
        (void)append(wheels[index][0], wheels[index][1], true);
    }
    const auto& front = wheels[order.back().second];
    const auto preview = [&](double distance) {
        return append(front[0] + distance * forward[0],
                      front[1] + distance * forward[1], false);
    };
    for (std::size_t i = 1; i <= count; ++i)
        if (!preview(static_cast<double>(i) * spacing)) return result;
    if (lookahead > 0. && (count == 0 || static_cast<double>(count) * spacing < lookahead))
        (void)preview(lookahead);
    return result;
}

double road_grade_for_posture(std::span<const RoadSample> road) {
    if (road.empty())
        throw std::invalid_argument(
            "posture program needs at least one road sample");
    double sum = 0.;
    for (const RoadSample &s : road) sum += s.grade;
    return sum / static_cast<double>(road.size());
}

double road_grade_preview(std::span<const RoadSample> road) {
    if (road.empty())
        throw std::invalid_argument(
            "posture program needs at least one road sample");
    return road.back().grade;
}

RiderKinematicState rider_kinematic_state(
    const mjModel *model, const mjData *data,
    std::span<const std::array<double, 2>> vertices,
    std::span<const double> wheel_x_m, double lookahead_m,
    double spacing_m) {
    return rider_kinematic_state(model, data, RoadProfile(vertices),
                                 wheel_x_m, lookahead_m, spacing_m);
}

RiderKinematicState rider_kinematic_state(
    const mjModel *model, const mjData *data, const RoadProfile &road,
    std::span<const double> wheel_x_m, double lookahead_m,
    double spacing_m) {
    if (model == nullptr || data == nullptr)
        throw std::invalid_argument("kinematic state needs a model and data");
    const std::span<const mjtNum> qpos =
        model_access::readonly_buffer(data->qpos, model->nq, "qpos");
    const std::span<const mjtNum> qvel =
        model_access::readonly_buffer(data->qvel, model->nv, "qvel");
    RiderKinematicState out{
        .qpos = std::vector<double>(qpos.begin(), qpos.end()),
        .qvel = std::vector<double>(qvel.begin(), qvel.end()),
        .rider_time_s = data->time,
        .road = road.samples(wheel_x_m, lookahead_m, spacing_m)};
    for (const double v : out.qpos)
        if (!std::isfinite(v))
            throw std::invalid_argument("rider state qpos");
    for (const double v : out.qvel)
        if (!std::isfinite(v))
            throw std::invalid_argument("rider state qvel");
    if (!std::isfinite(out.rider_time_s) || out.rider_time_s < 0.)
        throw std::invalid_argument("rider state time");
    return out;
}

RiderKinematicState spatial_rider_kinematic_state(
    const mjModel* model, const mjData* data, const bike::spatial::Topology& topology,
    const bike::terrain::Terrain3dQueryView& terrain, double lookahead, double spacing) {
    if (model == nullptr || data == nullptr)
        throw std::invalid_argument("spatial rider state needs a model and data");
    const auto positions = model_access::readonly_buffer(data->xpos, 3 * model->nbody, "xpos");
    std::array<std::array<double, 3>, 2> wheels{};
    for (std::size_t i = 0; i < wheels.size(); ++i) {
        const auto body = topology.wheels[i].wheel_body_id;
        model_access::require_id(body, model->nbody, "spatial.road_preview.wheel");
        const auto offset = 3 * static_cast<std::size_t>(body);
        for (std::size_t axis = 0; axis < 3; ++axis) wheels[i][axis] = positions[offset + axis];
    }
    model_access::require_id(topology.root.body_id, model->nbody, "spatial.road_preview.root");
    const auto rotations = model_access::readonly_buffer(data->xquat, 4 * model->nbody, "xquat");
    const auto q = 4 * static_cast<std::size_t>(topology.root.body_id);
    const auto heading = bike::spatial::rotate_body_to_world(
        {rotations[q], rotations[q + 1], rotations[q + 2], rotations[q + 3]}, {1., 0., 0.});
    const auto qpos = model_access::readonly_buffer(data->qpos, model->nq, "qpos");
    const auto qvel = model_access::readonly_buffer(data->qvel, model->nv, "qvel");
    RiderKinematicState out{
        .qpos = std::vector<double>(qpos.begin(), qpos.end()),
        .qvel = std::vector<double>(qvel.begin(), qvel.end()),
        .rider_time_s = data->time,
        .road = spatial_road_samples(terrain, wheels, heading, lookahead, spacing)};
    if (!std::ranges::all_of(out.qpos, [](double x) { return std::isfinite(x); }) ||
        !std::ranges::all_of(out.qvel, [](double x) { return std::isfinite(x); }) ||
        !std::isfinite(out.rider_time_s) || out.rider_time_s < 0.)
        throw std::invalid_argument("spatial rider state: nonfinite coordinates or invalid time");
    return out;
}

// ---- ForceAccumulator -----------------------------------------------------
void ForceAccumulator::add(std::string_view name,
                           std::span<const double> qfrc) {
    if (qfrc.size() != static_cast<std::size_t>(nv_) ||
        !std::ranges::all_of(
            qfrc, [](double v) { return std::isfinite(v); }))
        throw std::invalid_argument("invalid generalized force");
    if (component(name) != nullptr)
        throw std::invalid_argument("duplicate force component");
    components_.emplace_back(std::string(name),
                             std::vector<double>(qfrc.begin(), qfrc.end()));
}

std::vector<double> ForceAccumulator::total() const {
    std::vector<double> out(static_cast<std::size_t>(nv_), 0.);
    for (const auto &[name, values] : components_)
        for (std::size_t i = 0; i < values.size(); ++i) out[i] += values[i];
    return out;
}

} // namespace runtime
