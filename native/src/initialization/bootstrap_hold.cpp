#include "bootstrap_hold.hpp"

#include "../engine_call.hpp"
#include "../spatial/frames.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace bike::initialization {
namespace {
double norm(const spatial::Vec3& x) noexcept { return std::hypot(x[0],x[1],x[2]); }
template<class Range> bool finite(const Range& x) noexcept {
    return std::ranges::all_of(x,[](double a){return std::isfinite(a);});
}
bool valid_hold(const spatial::HoldState& s) noexcept {
    if(s.schema!=1 || !finite(s.reference_position_world_m) || !finite(s.reference_quat_wxyz) ||
       !finite(s.last_force_world_n) || !finite(s.last_couple_world_nm) ||
       !std::isfinite(s.stored_energy_j) || s.stored_energy_j<0. || !std::isfinite(s.work_j))return false;
    double q2=0.;for(const double q:s.reference_quat_wxyz)q2+=q*q;
    if(std::abs(q2-1.)>2e-9)return false;
    if(s.phase>=spatial::BootstrapPhase::armed && (s.active || norm(s.last_force_world_n)!=0. ||
        norm(s.last_couple_world_nm)!=0. || s.stored_energy_j!=0.))return false;
    return s.phase==spatial::BootstrapPhase::settling || s.phase==spatial::BootstrapPhase::release_check ||
        s.phase==spatial::BootstrapPhase::armed || s.phase==spatial::BootstrapPhase::scored;
}
}
BootstrapHold::BootstrapHold(const spatial::BootstrapConfig& cfg,
                            const spatial::RootBinding& root,const mjModel& model)
    :cfg_(cfg),root_(root) {
    if(root.body_id<=0 || root.body_id>=model.nbody || model.nv<=0)
        throw std::invalid_argument("bootstrap.hold.root: invalid body or velocity space");
    const std::array values{cfg.settle_time_s,cfg.release_force_limit_n,cfg.release_moment_limit_nm,
        cfg.hold_stiffness_n_per_m,cfg.hold_damping_ns_per_m,cfg.hold_rot_stiffness_nm_per_rad,
        cfg.hold_rot_damping_nms_per_rad};
    if(!finite(values) || std::ranges::any_of(values,[](double x){return x<0.;}) || cfg.settle_time_s<=0.)
        throw std::invalid_argument("bootstrap.hold: finite nonnegative gains and positive timeout required");
    jacp_.resize(3*static_cast<std::size_t>(model.nv));jacr_.resize(jacp_.size());
}
CoreStatus BootstrapHold::try_compute_into(const mjModel& model,const mjData& data,
        spatial::BootstrapPhase phase,std::span<double> qfrc) noexcept {
    if(qfrc.size()!=static_cast<std::size_t>(model.nv) || jacp_.size()!=3*qfrc.size())
        return CoreStatus::invalid_input;
    if(phase!=spatial::BootstrapPhase::settling && phase!=spatial::BootstrapPhase::release_check &&
       phase!=spatial::BootstrapPhase::armed && phase!=spatial::BootstrapPhase::scored)
        return CoreStatus::invalid_input;
    if(phase>=spatial::BootstrapPhase::armed) {
        state_.phase=phase;state_.active=false;
        state_.last_force_world_n={};state_.last_couple_world_nm={};state_.stored_energy_j=0.;
        return CoreStatus::ok;
    }
    const auto pose=spatial::body_pose(model,data,root_);
    if(!finite(pose.position_world_m) || !finite(pose.quat_world_from_body_wxyz) ||
       !finite(pose.linear_velocity_world_mps) || !finite(pose.angular_velocity_world_rad_s))
        return CoreStatus::invalid_geometry;
    auto stage=state_;
    if(stage.valid_mask==0) {
        stage.reference_position_world_m=pose.position_world_m;
        stage.reference_quat_wxyz=pose.quat_world_from_body_wxyz;
        stage.valid_mask=1;
    }
    // World-frame rotation from the reference to the actual pose. Its shortest
    // logarithm has the same sign as the displacement, so -K*e restores it.
    auto delta=spatial::quat_normalized(spatial::quat_multiply(pose.quat_world_from_body_wxyz,
        spatial::quat_conjugate(stage.reference_quat_wxyz)));
    if(delta[0]<0.)for(double& x:delta)x=-x;
    const double sine=std::hypot(delta[1],delta[2],delta[3]);
    const double factor=sine>1e-12?2.*std::atan2(sine,delta[0])/sine:2.;
    const spatial::Vec3 error{factor*delta[1],0.,factor*delta[3]};
    const double lateral=pose.position_world_m[1]-stage.reference_position_world_m[1];
    stage.last_force_world_n={0.,-cfg_.hold_stiffness_n_per_m*lateral-
        cfg_.hold_damping_ns_per_m*pose.linear_velocity_world_mps[1],0.};
    stage.last_couple_world_nm={-cfg_.hold_rot_stiffness_nm_per_rad*error[0]-
        cfg_.hold_rot_damping_nms_per_rad*pose.angular_velocity_world_rad_s[0],0.,
        -cfg_.hold_rot_stiffness_nm_per_rad*error[2]-
        cfg_.hold_rot_damping_nms_per_rad*pose.angular_velocity_world_rad_s[2]};
    stage.stored_energy_j=.5*cfg_.hold_stiffness_n_per_m*lateral*lateral+
        .5*cfg_.hold_rot_stiffness_nm_per_rad*(error[0]*error[0]+error[2]*error[2]);
    stage.active=true;stage.phase=phase;
    if(!valid_hold(stage))return CoreStatus::invalid_geometry;
    struct Call{const mjModel* m;const mjData* d;double* jp;double* jr;const double* p;int body;};
    Call call{.m=&model,.d=&data,.jp=jacp_.data(),.jr=jacr_.data(),.p=pose.position_world_m.data(),.body=root_.body_id};
    engine::ErrorBuffer error_buffer;
    if(!engine::invoke([](void* raw) noexcept {
        const auto& c=*static_cast<Call*>(raw);mj_jac(c.m,c.d,c.jp,c.jr,c.p,c.body);
    },&call,error_buffer))return CoreStatus::engine_failure;
    if(!finite(jacp_) || !finite(jacr_))return CoreStatus::invalid_geometry;
    for(std::size_t j=0;j<qfrc.size();++j) {
        double force=0.;
        for(std::size_t axis=0;axis<3;++axis)
            force+=jacp_[axis*qfrc.size()+j]*stage.last_force_world_n[axis]+
                jacr_[axis*qfrc.size()+j]*stage.last_couple_world_nm[axis];
        if(!std::isfinite(qfrc[j]+force))return CoreStatus::invalid_geometry;
    }
    for(std::size_t j=0;j<qfrc.size();++j) {
        double force=0.;
        for(std::size_t axis=0;axis<3;++axis)
            force+=jacp_[axis*qfrc.size()+j]*stage.last_force_world_n[axis]+
                jacr_[axis*qfrc.size()+j]*stage.last_couple_world_nm[axis];
        // Commit the same expression validated above. Adding the three axes
        // separately can overflow an intermediate even when their sum and
        // the checked final value are finite.
        qfrc[j]+=force;
    }
    state_=stage;
    return CoreStatus::ok;
}
const spatial::HoldState& BootstrapHold::state() const noexcept {return state_;}
CoreStatus BootstrapHold::restore(const spatial::HoldState& s) noexcept {
    if(!valid_hold(s))return CoreStatus::invalid_input;
    state_=s;return CoreStatus::ok;
}
bool BootstrapHold::release_check(const SpatialEquilibriumReport& r,double force_limit,double moment_limit,
    double acceleration_tolerance) const noexcept {
    return std::isfinite(acceleration_tolerance) && acceleration_tolerance>0. &&
        std::isfinite(force_limit) && force_limit>0. && std::isfinite(moment_limit) && moment_limit>0. &&
        r.errors.empty() && r.contacts_valid>0 && r.contacts_valid==r.contacts_total &&
        std::isfinite(r.max_linear_accel_mps2) && r.max_linear_accel_mps2>=0. && r.max_linear_accel_mps2<=acceleration_tolerance &&
        std::isfinite(r.max_angular_accel_radps2) && r.max_angular_accel_radps2>=0. && r.max_angular_accel_radps2<=acceleration_tolerance &&
        std::isfinite(r.weight_residual_fraction) && r.weight_residual_fraction>=0. && r.weight_residual_fraction<=.005 &&
        std::isfinite(r.roll_moment_residual_nm) && std::abs(r.roll_moment_residual_nm)<=1. &&
        std::isfinite(r.pitch_moment_residual_nm) && std::abs(r.pitch_moment_residual_nm)<=1. &&
        norm(state_.last_force_world_n)<force_limit && norm(state_.last_couple_world_nm)<moment_limit;
}
} // namespace bike::initialization
