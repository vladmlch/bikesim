#include "controls.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <ranges>
#include <stdexcept>
#include "tyre.hpp"
#include "../model/mapping.hpp"
#include "../model_topology.hpp"
#include "../numeric_sum.hpp"
#include "../numeric_sincos.hpp"

namespace bike::legacy {
namespace {
constexpr double rpm_per_radps = 60. / (2. * std::numbers::pi);
void finite(double x, std::string_view name) {
    if (!std::isfinite(x)) throw std::invalid_argument(std::string(name) + " must be finite");
}
void positive(double x, std::string_view name) {
    if (!std::isfinite(x) || x <= 0.) throw std::invalid_argument(std::string(name) + " must be finite and positive");
}
constexpr std::array<std::string_view,5> assist_names{"off","eco","tour","sport","turbo"};
}
double WheelSpin::omega_radps(std::span<const double> qvel)const{
    if(dofadr<0||static_cast<std::size_t>(dofadr)>=qvel.size())throw std::invalid_argument("wheel spin address outside velocity state");
    return qvel[static_cast<std::size_t>(dofadr)];
}
std::pair<int,int> resolve_scalar_joint(const mjModel& m,std::string_view name){
    const auto id=static_cast<std::size_t>(model::required_id(m,mjOBJ_JOINT,name));
    const auto type=std::views::counted(m.jnt_type,m.njnt)[id];
    if(type!=mjJNT_HINGE&&type!=mjJNT_SLIDE)throw std::invalid_argument(std::string(name)+": requires scalar hinge or slide joint");
    const auto address=model::joint_address(m,name);
    if(address.first<0||address.first>=m.nq||address.second<0||address.second>=m.nv)throw std::invalid_argument(std::string(name)+": joint address outside model");
    return address;
}
std::pair<int,int> resolve_hinge(const mjModel& m,std::string_view name){
    const int id=model::required_id(m,mjOBJ_JOINT,name);
    topology::joint(&m,id,std::string(name).c_str(),mjJNT_HINGE,{0.,1.,0.});return resolve_scalar_joint(m,name);
}
int validate_actuator_target(const mjModel& m,std::string_view name,std::string_view joint_name){
    const int actuator=model::required_id(m,mjOBJ_ACTUATOR,name),joint=model::required_id(m,mjOBJ_JOINT,joint_name);
    topology::actuator(&m,actuator,std::string(name).c_str(),joint);return actuator;
}
WheelSpin resolve_wheel_spin(const mjModel& m,std::string_view joint_name,std::string_view contact_geom){
    const auto address=resolve_hinge(m,joint_name);const auto geom=static_cast<std::size_t>(model::required_id(m,mjOBJ_GEOM,contact_geom));
    if(std::views::counted(m.geom_type,m.ngeom)[geom]!=mjGEOM_SPHERE)throw std::invalid_argument(std::string(contact_geom)+": wheel contact must be a sphere");
    return {std::string(joint_name),address.second,std::views::counted(m.geom_size,3*m.ngeom)[3*geom]};
}
double peak_shape(double depth) { return (1. - depth) + depth * (std::numbers::pi / 2.); }
double ripple_shape(double phase, double depth) {
    return (1. - depth) + depth * (std::numbers::pi / 2.) * std::abs(numeric::cos(phase));
}
void DrivetrainSpecs::validate() const {
    if (chainring_teeth < 20 || cog_teeth < 9) throw std::invalid_argument("legacy drivetrain implausible gearing");
    if (auto_shift) {
        if (cassette.empty() || std::ranges::any_of(cassette, [](int c){return c < 9;}))
            throw std::invalid_argument("legacy drivetrain cassette requires cogs >= 9");
        if (!(target_cadence_min_rpm > 0.) || !(target_cadence_max_rpm > target_cadence_min_rpm))
            throw std::invalid_argument("legacy drivetrain cadence band");
        if (shift_cooldown_s < 0. || shift_cut_duration_s < 0.)
            throw std::invalid_argument("legacy drivetrain shift timings must be nonnegative");
    }
    for (double x : {target_cadence_min_rpm,target_cadence_max_rpm,shift_cooldown_s,
        shift_cut_duration_s,crank_phase_deg,crank_hold_stiffness_nm_rad,crank_hold_damping_nms_rad})
        finite(x, "legacy drivetrain setting");
    if (!std::isfinite(ripple_depth) || ripple_depth < 0. || ripple_depth > 1.)
        throw std::invalid_argument("legacy drivetrain ripple_depth must be in [0,1]");
    for (double x : {rider_torque_ceiling_nm,rider_power_ceiling_w,assist_torque_ceiling_nm,
        assist_power_ceiling_w,assist_response_s,cutoff_speed_kmh,cutoff_taper_kmh})
        positive(x, "legacy drivetrain ceiling/time");
    if (rider_torque_ceiling_nm * peak_shape(ripple_depth) + assist_torque_ceiling_nm > 180.)
        throw std::invalid_argument("peak legacy crank torque exceeds 180 N.m actuator ceiling");
}
double DrivetrainSpecs::gear_ratio() const { return static_cast<double>(chainring_teeth) / static_cast<double>(cog_teeth); }
double DrivetrainSpecs::cadence_rpm_at(double speed, double radius) const {
    positive(radius, "rear wheel radius");
    return speed / radius / gear_ratio() * rpm_per_radps;
}
double DrivetrainSpecs::support_factor(std::string_view mode) const {
    constexpr std::array<double,5> factors{0.,.60,1.40,2.40,3.40};
    const auto found = std::ranges::find(assist_names, mode);
    if (found == assist_names.end()) throw std::invalid_argument("unknown legacy assist mode '" + std::string(mode) + "'");
    return factors[static_cast<std::size_t>(found - assist_names.begin())];
}
std::pair<double,std::string> limited_rider_torque(double demand, double rate, const DrivetrainSpecs& specs) {
    if (demand <= 0.) return {0., "none"};
    double torque = demand; std::string limit = "none";
    if (torque > specs.rider_torque_ceiling_nm) { torque = specs.rider_torque_ceiling_nm; limit = "torque"; }
    if (rate > 1e-3 && specs.rider_power_ceiling_w / rate < torque) {
        torque = specs.rider_power_ceiling_w / rate; limit = "power";
    }
    return {torque, std::move(limit)};
}
double cutoff_factor(double speed, const DrivetrainSpecs& specs) {
    const double kmh = speed * 3.6, start = specs.cutoff_speed_kmh - specs.cutoff_taper_kmh;
    if (kmh <= start) return 1.;
    if (kmh >= specs.cutoff_speed_kmh) return 0.;
    return (specs.cutoff_speed_kmh - kmh) / specs.cutoff_taper_kmh;
}
double assist_target_torque(double rider, double support, double rate, const DrivetrainSpecs& specs) {
    if (rider <= 0. || support <= 0.) return 0.;
    double target = std::min(support * rider, specs.assist_torque_ceiling_nm);
    if (rate > 1e-3) target = std::min(target, specs.assist_power_ceiling_w / rate);
    return target;
}
double first_order_step(double current, double target, double dt, double tau) {
    if (tau <= 0.) return target;
    const double alpha = dt / (tau + dt);
    return current + alpha * (target - current);
}
PedalDrivetrain::PedalDrivetrain(mjModel& model, DrivetrainSpecs specs,
        std::string mode, std::string assist, bool legs)
    : model_(model), specs_(std::move(specs)), mode_(std::move(mode)), assist_(std::move(assist)),
      legs_drive_(legs), cog_(specs_.cog_teeth) {
    specs_.validate();
    if (mode_ != "pedal" && mode_ != "pedelec") throw std::invalid_argument("legacy pedalled drive mode must be pedal or pedelec");
    (void)specs_.support_factor(assist_);
    if (mode_ == "pedal") assist_ = "off";
    const auto crank = resolve_hinge(model_, "crank_spin");
    crank_q_ = static_cast<std::size_t>(crank.first); crank_v_ = static_cast<std::size_t>(crank.second);
    const auto wheel = resolve_hinge(model_, "rear_wheel_spin");
    wheel_q_ = static_cast<std::size_t>(wheel.first); wheel_v_ = static_cast<std::size_t>(wheel.second);
    actuator_ = static_cast<std::size_t>(validate_actuator_target(model_, "crank_drive","crank_spin"));
    equality_ = static_cast<std::size_t>(model::required_id(model_, mjOBJ_EQUALITY, "chain_drive"));
    if (std::views::counted(model_.eq_type, model_.neq)[equality_] != mjEQ_JOINT)
        throw std::invalid_argument("legacy chain_drive must be a joint equality");
    const auto q0 = std::views::counted(model_.qpos0, model_.nq);
    crank_zero_ = q0[crank_q_]; wheel_zero_ = q0[wheel_q_];
    start_phase_ = specs_.crank_phase_deg * (std::numbers::pi / 180.);
    hold_phase_ = start_phase_;
    std::views::counted(model_.eq_data, mjNEQDATA * model_.neq)[mjNEQDATA * equality_ + 1] = 1. / current_gear_ratio();
    command_.phase_rad = start_phase_; command_.support_factor = nominal_support_factor(); command_.gear_teeth = cog_;
}
double PedalDrivetrain::current_gear_ratio() const { return static_cast<double>(specs_.chainring_teeth) / static_cast<double>(cog_); }
double PedalDrivetrain::nominal_support_factor() const { return specs_.support_factor(assist_); }
void PedalDrivetrain::set_assist_mode(std::string_view mode) {
    (void)specs_.support_factor(mode);
    if (mode_ == "pedal" && mode != "off") throw std::invalid_argument("assist modes require legacy pedelec drive");
    assist_ = mode;
}
std::string PedalDrivetrain::cycle_assist_mode() {
    if (mode_ == "pedal") return assist_;
    const auto found = std::ranges::find(assist_names, assist_);
    set_assist_mode(assist_names[(static_cast<std::size_t>(found - assist_names.begin()) + 1) % assist_names.size()]);
    return assist_;
}
void PedalDrivetrain::redatum(const mjData& data) {
    const auto q = std::views::counted(data.qpos, model_.nq);
    const double crank = q[crank_q_] - crank_zero_, wheel = q[wheel_q_] - wheel_zero_;
    std::views::counted(model_.eq_data, mjNEQDATA * model_.neq)[mjNEQDATA * equality_] = crank - wheel / current_gear_ratio();
}
double PedalDrivetrain::chain_residual(const mjData& data) const {
    const auto q = std::views::counted(data.qpos, model_.nq);
    return (q[crank_q_] - crank_zero_) - (q[wheel_q_] - wheel_zero_) / current_gear_ratio()
        - std::views::counted(model_.eq_data, mjNEQDATA * model_.neq)[mjNEQDATA * equality_];
}
void PedalDrivetrain::engage(mjData& data) {
    if (engaged_) return;
    auto v = std::views::counted(data.qvel, model_.nv);
    v[crank_v_] = v[wheel_v_] / current_gear_ratio();
    redatum(data);
    std::views::counted(data.eq_active, model_.neq)[equality_] = 1;
    engaged_ = true;
}
void PedalDrivetrain::shift_to(mjData& data, int cog) {
    cog_ = cog; cooldown_ = specs_.shift_cooldown_s; cut_ = specs_.shift_cut_duration_s;
    std::views::counted(model_.eq_data, mjNEQDATA * model_.neq)[mjNEQDATA * equality_ + 1] = 1. / current_gear_ratio();
    auto v = std::views::counted(data.qvel, model_.nv);
    v[crank_v_] = v[wheel_v_] / current_gear_ratio(); redatum(data);
}
const CrankCommand& PedalDrivetrain::compute(mjData& data, double demand,
        double speed, bool grounded, bool traction_limited) {
    finite(demand, "legacy wheel demand"); finite(speed, "legacy speed");
    const double dt = model_.opt.timestep;
    const auto q = std::views::counted(data.qpos, model_.nq);
    auto v = std::views::counted(data.qvel, model_.nv);
    finite(q[crank_q_], "legacy crank position"); finite(v[crank_v_], "legacy crank rate");
    cooldown_ = std::max(0., cooldown_ - dt); cut_ = std::max(0., cut_ - dt);
    const bool pedalling = demand > 0. && grounded;
    if (pedalling) engage(data);
    const double phase = q[crank_q_];
    double rate = v[crank_v_], cadence = rate * rpm_per_radps;
    if (specs_.auto_shift && cooldown_ <= 0. && pedalling) {
        std::optional<int> chosen;
        for (int cog : specs_.cassette) {
            if (cadence < specs_.target_cadence_min_rpm && cog > cog_ && (!chosen || cog < *chosen)) chosen = cog;
            else if (cadence > specs_.target_cadence_max_rpm && cog < cog_ && (!chosen || cog > *chosen)) chosen = cog;
        }
        if (chosen) { shift_to(data, *chosen); rate = v[crank_v_]; cadence = rate * rpm_per_radps; }
    }
    double rider_nm = 0., support = 0., total = 0., brake = 0., delivered_assist = 0.;
    std::string limit = "none";
    const double taper = cutoff_factor(speed, specs_);
    if (pedalling) {
        auto limited = limited_rider_torque(demand * current_gear_ratio(), rate, specs_);
        limit = std::move(limited.second);
        rider_nm = limited.first * ripple_shape(phase, specs_.ripple_depth);
        support = nominal_support_factor() * taper;
        const double target = assist_target_torque(rider_nm, support, rate, specs_);
        assist_nm_ = first_order_step(assist_nm_, target, dt, specs_.assist_response_s);
        delivered_assist = assist_nm_;
        if (cut_ > 0.) { rider_nm *= .30; delivered_assist *= .30; }
        total = delivered_assist + (legs_drive_ ? 0. : rider_nm);
    } else {
        if (engaged_) {
            std::views::counted(data.eq_active, model_.neq)[equality_] = 0;
            hold_phase_ = phase; engaged_ = false;
        }
        assist_nm_ = first_order_step(assist_nm_, 0., dt, specs_.assist_response_s);
        delivered_assist = assist_nm_;
        total = assist_nm_ + (specs_.crank_hold_stiffness_nm_rad * (hold_phase_ - phase)
            - specs_.crank_hold_damping_nms_rad * rate);
        if (demand < 0.) brake = std::min(1., -demand / (2. * 200.));
    }
    traction_limited_ = traction_limited;
    command_ = {std::clamp(total, -180., 180.), rider_nm, assist_nm_, phase, cadence,
        rider_nm * rate, engaged_ ? delivered_assist * rate : 0., support,
        !engaged_, nominal_support_factor() > 0. && taper < 1., std::move(limit), brake, cog_};
    return command_;
}
void PedalDrivetrain::write(mjData& data) const { std::views::counted(data.ctrl, model_.nu)[actuator_] = command_.crank_torque_nm; }
void PedalDrivetrain::reset(mjData& data, bool initialize_pose) {
    cog_ = specs_.cog_teeth; cooldown_ = 0.; cut_ = 0.;
    std::views::counted(model_.eq_data, mjNEQDATA * model_.neq)[mjNEQDATA * equality_ + 1] = 1. / current_gear_ratio();
    auto q = std::views::counted(data.qpos, model_.nq);
    if (initialize_pose) { q[crank_q_] = start_phase_; std::views::counted(data.qvel, model_.nv)[crank_v_] = 0.; }
    std::views::counted(data.ctrl, model_.nu)[actuator_] = 0.;
    assist_nm_ = 0.; hold_phase_ = q[crank_q_]; engaged_ = true; traction_limited_ = false;
    if (initialize_pose) redatum(data);
    std::views::counted(data.eq_active, model_.neq)[equality_] = 1;
    command_ = {}; command_.phase_rad = hold_phase_; command_.support_factor = nominal_support_factor(); command_.gear_teeth = cog_;
}

TerrainContactQuery::TerrainContactQuery(const mjModel& m, int dropout)
    : model_(m),query_(&m,dropout) {}
namespace {
TireSnapshot legacy_tyre_snapshot(const mjModel& m,const mjData& d,int geom,const biketyre::WheelOutputs& outputs,std::int64_t interval){
    TireSnapshot out;out.time_s=d.time;out.interval_id=interval;out.backend="legacy_pneumatic";out.geometric_contact=!outputs.airborne;
    const auto id=static_cast<std::size_t>(geom);const int body=std::views::counted(m.geom_bodyid,m.ngeom)[id];
    std::ranges::copy(std::views::counted(d.geom_xpos,3*m.ngeom).subspan(3*id,3),out.wheel_axis_m.begin());
    numeric::PythonSum weighted_drop,weights;
    for(const auto& p:outputs.patches()){
        if(!(p.normal_load_n>0.))throw std::invalid_argument("legacy tyre patch requires positive normal load");
        TirePatch patch{};patch.point_m=p.centroid_world_m;patch.normal_load_n=p.normal_load_n;patch.tangent_force_n=p.tangential_force_n;patch.working_surface=true;
        for(std::size_t j=0;j<3;++j)patch.normal[j]=p.normal_force_world_n[j]/p.normal_load_n;
        const std::array tangent{patch.normal[2],0.,-patch.normal[0]};const auto velocity=model::point_velocity(m,d,body,patch.point_m);
        patch.slip_mps=geometry::detail::np_dot3(velocity,tangent);
        const std::array delta{patch.point_m[0]-out.wheel_axis_m[0],patch.point_m[1]-out.wheel_axis_m[1],patch.point_m[2]-out.wheel_axis_m[2]};
        weighted_drop.add(p.normal_load_n * -geometry::detail::np_dot3(delta,patch.normal));weights.add(p.normal_load_n);out.patches.push_back(patch);
    }
    out.effective_radius_m=out.patches.empty()?0.:std::max(0.,weighted_drop.total()/weights.total());return out;
}
}
TerrainContacts TerrainContactQuery::query(const mjData& data,const TyreForceApplier* tyre,std::optional<double> time,std::span<const double> qvel){
    TerrainContacts out;
    if(!tyre){auto result=query_.query(&model_,&data,time,qvel);const auto& c=result.contacts;
        out.front_load_n=c.front_load_n;out.rear_load_n=c.rear_load_n;out.front_support_n=c.front_support_n;out.rear_support_n=c.rear_support_n;out.handlebar_load_n=c.handlebar_load_n;
        out.front_snapshot=std::move(result.snapshots[0]);out.rear_snapshot=std::move(result.snapshots[1]);out.front_controller_grounded=c.front_controller_grounded;out.rear_controller_grounded=c.rear_controller_grounded;return out;}
    const double interval=std::nearbyint(time.value_or(data.time)/model_.opt.timestep);
    if(!std::isfinite(interval)||interval<-0x1p63||interval>=0x1p63)throw std::invalid_argument("legacy contact interval outside int64");
    const auto& front=tyre->front_outputs();const auto& rear=tyre->rear_outputs();
    out.front_load_n=std::max(0.,front.normal_load_n);out.rear_load_n=std::max(0.,rear.normal_load_n);out.front_support_n=std::max(0.,front.support_n);out.rear_support_n=std::max(0.,rear.support_n);out.handlebar_load_n=handlebar_load(data);
    out.front_snapshot=legacy_tyre_snapshot(model_,data,query_.front_id,front,static_cast<std::int64_t>(interval));out.rear_snapshot=legacy_tyre_snapshot(model_,data,query_.rear_id,rear,static_cast<std::int64_t>(interval));return out;
}
double TerrainContactQuery::handlebar_load(const mjData& data)const{return query_.handlebar_load(&model_,&data);}
void TerrainContactQuery::reset() noexcept {query_.reset();}
double opposing_torque(double magnitude, double omega, double taper) {
    finite(magnitude,"resistive torque magnitude"); finite(omega,"wheel rate"); positive(taper,"resistance taper");
    if (magnitude < 0.) throw std::invalid_argument("resistive torque magnitude must be nonnegative");
    if(omega==0.)return 0.;
    const double scale=std::min(std::abs(omega)/taper,1.);
    const double result=-std::abs(magnitude)*scale*(omega>0.?1.:-1.);
    finite(result,"resistive torque");return result;
}
double rolling_torque(double support, double omega, double radius, double weight,
        double crr, double taper, double ceiling) {
    finite(support,"wheel support"); positive(radius,"wheel radius"); positive(weight,"system weight");
    positive(ceiling,"rolling load ceiling");
    if (!std::isfinite(crr) || crr < 0.) throw std::invalid_argument("crr must be finite and nonnegative");
    const double load = std::min(std::max(support,0.),ceiling * weight);
    return opposing_torque(crr * load * radius,omega,taper);
}
std::optional<CrashEvent> detect_crash(double time, double x, double pitch,
        const TerrainContacts& contacts, std::optional<CrashEvent> previous, double limit_deg) {
    if (previous) return previous;
    positive(limit_deg,"crash pitch limit");
    std::string cause;
    if (std::abs(pitch) > limit_deg * (std::numbers::pi / 180.)) cause = "pitch_over";
    else if (contacts.handlebar_load_n > 1.) cause = "handlebar_contact";
    else return {};
    return CrashEvent{std::move(cause),time,x,pitch};
}
PitchState pitch_step(PitchState state, bool enabled, double pitch, double rate,
        const TerrainContacts& contacts, double dt) {
    finite(pitch,"legacy pitch"); finite(rate,"legacy pitch rate"); positive(dt,"legacy timestep");
    state.active = enabled && contacts.airborne();
    state.moment_nm = state.active ? std::clamp(-500. * pitch - 150. * rate,-80.,80.) : 0.;
    state.angular_impulse_nms += state.moment_nm * dt;
    state.work_j += state.moment_nm * rate * dt;
    return state;
}
void RunLimits::validate() const {
    positive(finish_x_m,"finish_x_m"); positive(max_wall_clock_s,"max_wall_clock_s");
    if (max_steps <= 0) throw std::invalid_argument("max_steps must be positive");
}
RunLimits RunLimits::for_track(double length, double start, double dt, double wall, double speed) {
    positive(dt,"timestep_s"); positive(speed,"min_speed_mps");
    const double distance = length - start;
    positive(distance,"remaining track length");
    const double count = 2. * distance / (speed * dt);
    if (!std::isfinite(count) || count >= static_cast<double>(std::numeric_limits<std::int64_t>::max()))
        throw std::invalid_argument("legacy step cap exceeds int64 range");
    RunLimits result{length,static_cast<std::int64_t>(count),wall}; result.validate(); return result;
}
std::optional<std::string> termination_reason(const RunLimits& limits, double position,
        std::int64_t steps, double wall, const std::optional<CrashEvent>& crash) {
    limits.validate();
    if (crash) return "crash";
    if (position >= limits.finish_x_m) return "end of track";
    if (steps >= limits.max_steps) return "step cap";
    if (wall >= limits.max_wall_clock_s) return "wall clock cap";
    return {};
}
} // namespace bike::legacy
