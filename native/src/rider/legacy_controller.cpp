// Source: sim/ride/rider_control.py non-spindle branch. The existing
// SpindleController is the transaction/state owner; only this branch uses
// constrained allocation. Contact reactions remain in RiderContactWriter.
#include "legacy_controller.hpp"
#include "force_requests.hpp"
#include "activation_allocation.hpp"
#include "support_geometry.hpp"
#include "../contact/laws.hpp"
#include "../diag.hpp"
#include "../engine_call.hpp"
#include "../model_access.hpp"
#include "../numerics/dense.hpp"
#include "../numeric_sincos.hpp"
#include "../numeric_sum.hpp"
#include "../cblas_abi.hpp"
#include "../model/rider_geometry.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace bike::rider {
namespace {
namespace d=bike::numerics::dense;
using Vec=std::array<double,3>;using Rot=std::array<double,9>;using Vector=std::vector<double>;
constexpr double pi=std::numbers::pi;
constexpr double pressure_blend_cosine=.15;
double pressure_mean_scale() {
    const double boundary=std::acos(pressure_blend_cosine),s=std::sin(boundary);
    const double third=2./3.-s+std::pow(s,3.)/3.;
    const double fourth=3.*(pi/2.-boundary)/8.-std::sin(2.*boundary)/4.-std::sin(4.*boundary)/32.;
    const double integral=s+3.*third/std::pow(pressure_blend_cosine,2.)
        -2.*fourth/std::pow(pressure_blend_cosine,3.);
    return pi/(2.*integral);
}
Vec vec(const double* p,mjtSize count,int i){
    const auto span=model_access::readonly_buffer(p,count*3,"legacy_controller vec");
    model_access::require_id(i,count,"legacy_controller vec");
    const auto sub=span.subspan(static_cast<std::size_t>(i)*3,3);
    return {sub[0],sub[1],sub[2]};
}
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wlarge-by-value-copy")
Rot rot(const double* p,mjtSize count,int i){
    const auto span=model_access::readonly_buffer(p,count*9,"legacy_controller rot");
    model_access::require_id(i,count,"legacy_controller rot");
    Rot r{};std::ranges::copy(span.subspan(static_cast<std::size_t>(i)*9,9),r.begin());return r;
}
NATIVE_DIAG_POP
Vec add(const Vec&a,const Vec&b){return ::rider::add(a,b);}
Vec sub(const Vec&a,const Vec&b){return ::rider::subtract(a,b);}
Vec mul(const Vec&a,double b){return ::rider::multiply(a,b);}
Vec mv(const Rot&r,const Vec&v,bool trans=false){return ::rider::matvec(r,v,trans);}
template<class T>void set(::rider::NamedEntries<T>& entries,const std::string& name,T value){for(auto& [key,item]:entries)if(key==name){item=std::move(value);return;}entries.emplace_back(name,std::move(value));}
template<class T>T& get(::rider::NamedEntries<T>& entries,const std::string& name){for(auto& [key,item]:entries)if(key==name)return item;throw std::out_of_range(name);}
Matrix jacobian(const mjModel& m,const mjData& data,int body,const Vec& point,bool rotational=false) {
    auto out=d::zeros(3,static_cast<std::size_t>(m.nv));
    struct Context {const mjModel* m;const mjData*d;double* jp;double* jr;const double* point;int body;};
    Context c{.m=&m,.d=&data,.jp=rotational?nullptr:out.values.data(),.jr=rotational?out.values.data():nullptr,.point=point.data(),.body=body};engine::ErrorBuffer error;
    if(!engine::invoke([](void* raw) noexcept{const auto&x=*static_cast<Context*>(raw);mj_jac(x.m,x.d,x.jp,x.jr,x.point,x.body);},&c,error))engine::throw_failure(error);
    return out;
}
Vec velocity(const mjModel& m,const mjData& data,int body,const Vec& point){const auto v=d::multiply(jacobian(m,data,body,point),model_access::readonly_buffer(data.qvel,m.nv,"qvel"));return {v[0],v[1],v[2]};}
Vec feasible_pedal_force(const Vec& f,const Vec& n,double friction,double load) {
    const Vec tangent{n[2],0.,-n[0]};const double compression=std::max(-d::dot(f,n),0.);const double bound=friction*std::min(compression,load);
    return add(mul(n,-compression),mul(tangent,std::clamp(d::dot(f,tangent),-bound,bound)));
}
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
PedalingRequests pedal_requests(double phase,double mean,double radius,const std::array<Vec,2>& normals,
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
    const std::array<double,2>& loads,double mu,double ripple,double preload,const std::array<bool,2>& preload_enabled) {
    PedalingRequests out;const double fraction=std::clamp(.5+.5*numeric::cos(phase)/pressure_blend_cosine,0.,1.);
    out.weights={fraction*fraction*(3.-2.*fraction),0.};out.weights[1]=1.-out.weights[0];
    const double total=mean*(1.+ripple*numeric::cos(2.*phase));
    struct Geometry{double normal_moment,tangent_moment,ratio,capacity,compression_limit;};std::array<Geometry,2> geometry{};
    std::array<double,2> capacities{},targets{},active{};double recovery_torque=0.;
    for(std::size_t s=0;s<2;++s) {
        const double p=phase+(s?pi:0.),lx=radius*numeric::cos(p),lz=-radius*numeric::sin(p);const auto& n=normals[s];
        const double nm=lx*n[2]-lz*n[0],tm=lz*n[2]+lx*n[0],ratio=.8*mu*std::min(std::abs(tm)/radius,1.),capacity=nm+ratio*std::abs(tm);
        const double limit=nm<0.&&ratio>0.?mu*loads[s]/ratio:std::numeric_limits<double>::infinity();
        const double torque_limit=std::isinf(limit)?(nm>0.?std::numeric_limits<double>::infinity():std::abs(tm)*mu*loads[s]):nm*limit+std::abs(tm)*std::min(ratio*limit,mu*std::min(limit,loads[s]));
        capacities[s]=capacity>1e-10&&out.weights[s]>1e-6?std::max(0.,torque_limit):0.;geometry[s]={.normal_moment=nm,.tangent_moment=tm,.ratio=ratio,.capacity=capacity,.compression_limit=limit};
        if(preload>0.&&preload_enabled[s]&&out.weights[s]<=1e-6){out.force[s]=feasible_pedal_force(mul(n,-preload),n,mu,loads[s]);recovery_torque+=lz*out.force[s][0]-lx*out.force[s][2];}
        active[s]=capacities[s]>0.?out.weights[s]:0.;
    }
    const double sum=active[0]+active[1],driving=std::max(0.,total-recovery_torque);
    for(std::size_t s=0;s<2;++s)targets[s]=sum!=0.?std::min(capacities[s],driving*active[s]/sum):0.;
    double remaining=std::max(0.,driving-targets[0]-targets[1]);const std::size_t first=out.weights[0]>=out.weights[1]?0:1;
    for(std::size_t const s:{first,1-first}){const double a=std::min(remaining,capacities[s]-targets[s]);targets[s]+=a;remaining-=a;}
    for(std::size_t s=0;s<2;++s)if(targets[s]!=0.) {
        const auto g=geometry[s];const double threshold=g.ratio>0.?std::min(g.compression_limit,mu*loads[s]/g.ratio):g.compression_limit;
        double compression=targets[s]<=g.capacity*threshold+1e-12?targets[s]/g.capacity:(targets[s]-std::abs(g.tangent_moment)*mu*loads[s])/g.normal_moment;
        compression=std::min(std::max(0.,compression),g.compression_limit);const auto& n=normals[s];const Vec t{n[2],0.,-n[0]};
        out.force[s]=feasible_pedal_force(mul(add(mul(n,-1.),mul(t,g.tangent_moment<0.?-g.ratio:g.ratio)),compression),n,mu,loads[s]);
    }
    return out;
}
Vec extent(const Rot&r,const Vec&half){Vec out{};for(std::size_t i=0;i<3;++i)for(std::size_t j=0;j<3;++j)out[i]+=std::abs(r[3*i+j])*half[j];return out;}
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
void observe_recovery(::rider::PedalRecoveryState& state,const Vec& origin,const Rot&r,const Vec&half,
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
    const Vec& sole,double force_z,double pad_half,double radius) {
    const auto ext=extent(r,half);constexpr double clearance=.01;
    if(state.stage=="none") {if(force_z<-1.&&sole[2]+radius<origin[2]){state.stage="release";state.release_offset_x_m=sole[0]-origin[0];state.direction=state.release_offset_x_m<0.?-1.:1.;}}
    else if(state.stage=="release") {if(sole[2]+2.*radius<origin[2]-ext[2]-.5*clearance)state.stage="escape";}
    else if(state.stage=="escape") {if(state.direction*(sole[0]-origin[0])>ext[0]+pad_half+radius)state.stage="raise";}
    else if(state.stage=="raise") {if(sole[2]>origin[2]+ext[2]+.5*clearance)state.stage="return";}
    else if(state.stage=="return" && std::abs(sole[0]-origin[0])<clearance && sole[2]>origin[2]+ext[2])state={};
}
std::optional<Vec> recovery_goal(const ::rider::PedalRecoveryState& state,const Vec& origin,const Rot&r,const Vec&half,double pad_half,double radius) {
    if(state.stage=="none")return std::nullopt;
    const auto ext=extent(r,half);
    constexpr double clearance=.01;
    const double escape=state.direction*(ext[0]+pad_half+radius+clearance),offset=state.stage=="release"?state.release_offset_x_m:state.stage=="return"?0.:escape;
    return Vec{origin[0]+offset,origin[1],state.stage=="release"||state.stage=="escape"?origin[2]-ext[2]-2.*radius-clearance:origin[2]+ext[2]+clearance};
}
}

Force3 stance_force(double phase,double mean,double crank) {
    if(!std::isfinite(phase)||!std::isfinite(mean)||mean<0.||!std::isfinite(crank)||crank<=0.)throw std::invalid_argument("invalid stance force inputs");
    const double fraction=std::clamp(numeric::cos(phase)/pressure_blend_cosine,0.,1.);
    const double weight=fraction*fraction*(3.-2.*fraction)*pressure_mean_scale();
    return {0.,0.,-(mean/crank)*weight};
}
Force3 feasible_pedal_force_request(const Force3& force,const Force3& normal,double mu,double load) {
    if(!d::finite(force)||!d::finite(normal)||!std::isfinite(mu)||mu<0.||!std::isfinite(load)||load<0.||std::abs(d::norm(normal)-1.)>1e-9||std::abs(normal[1])>1e-9||std::abs(force[1])>1e-9)throw std::invalid_argument("pedal force request must use a planar unit normal and nonnegative friction/load");
    return feasible_pedal_force(force,normal,mu,load);
}
double pedal_torque_waveform(double mean,double phase,double ripple) {
    if(!std::isfinite(mean)||mean<0.||!std::isfinite(phase)||!std::isfinite(ripple)||ripple<0.||ripple>=1.)throw std::invalid_argument("invalid pedal waveform inputs");
    return mean*(1.+ripple*numeric::cos(2.*phase));
}
PedalingRequests pedaling_force_requests(double phase,double mean,double crank,const std::array<Force3,2>& normals,
    const std::array<double,2>& loads,double mu,double ripple,double preload,const std::array<bool,2>& sides) {
    (void)pedal_torque_waveform(mean,phase,ripple);
    if(!std::isfinite(crank)||crank<=0.||!std::isfinite(preload)||preload<0.)throw std::invalid_argument("invalid pedal radius/preload");
    for(std::size_t s=0;s<2;++s)(void)feasible_pedal_force_request({},normals[s],mu,loads[s]);
    return pedal_requests(phase,mean,crank,normals,loads,mu,ripple,preload,sides);
}

LegacyController::LegacyController(::rider::SpindleController& owner):c_(owner) {
    const auto& m=*c_.model_;
    const auto jnt_dofadr=model_access::readonly_buffer(m.jnt_dofadr,m.njnt,"jnt_dofadr");
    const auto eq_obj1id=model_access::readonly_buffer(m.eq_obj1id,m.neq,"eq_obj1id");
    const auto eq_obj2id=model_access::readonly_buffer(m.eq_obj2id,m.neq,"eq_obj2id");
    const auto eq_type=model_access::readonly_buffer(m.eq_type,m.neq,"eq_type");
    const auto eq_data=model_access::readonly_buffer(m.eq_data,m.neq*mjNEQDATA,"eq_data");
    const auto geom_bodyid=model_access::readonly_buffer(m.geom_bodyid,m.ngeom,"geom_bodyid");
    for(auto name:{"rider_root_x","rider_root_z","rider_root_pitch"}) {
        const int j=mj_name2id(&m,mjOBJ_JOINT,name);if(j<0)throw std::invalid_argument("legacy rider requires floating planar pelvis");rider_dofs_.push_back(jnt_dofadr[static_cast<std::size_t>(j)]);
    }
    for(const auto& j:c_.joints_)rider_dofs_.push_back(j.dof_adr);
    for(int i=0;i<m.nv;++i)if(std::ranges::find(rider_dofs_,i)==rider_dofs_.end())bike_dofs_.push_back(i);
    const std::array<std::string,5> names{"saddle","front","rear","grip_left","grip_right"};
    const std::array<std::string,5> eqs{"weld_saddle","weld_foot_front","weld_foot_rear","connect_grip_left","connect_grip_right"};
    for(std::size_t k=0;k<names.size();++k) {
        AllocationAttachment a;a.name=names[k];a.equality_id=mj_name2id(&m,mjOBJ_EQUALITY,eqs[k].c_str());
        if(k==0&&a.equality_id<0)a.equality_id=mj_name2id(&m,mjOBJ_EQUALITY,"connect_saddle");
        if(k<3){a.site_id=mj_name2id(&m,mjOBJ_SITE,k==0?"site_rider_saddle":k==1?"site_rider_sole_front":"site_rider_sole_rear");a.geom_id=mj_name2id(&m,mjOBJ_GEOM,k==0?"geom_saddle":k==1?"geom_pedal_front":"geom_pedal_rear");}
        if(a.equality_id>=0){const auto eq=static_cast<std::size_t>(a.equality_id);a.rider_body=eq_obj1id[eq];a.other_body=eq_obj2id[eq];a.weld=eq_type[eq]==mjEQ_WELD;std::ranges::copy(eq_data.subspan(eq*mjNEQDATA,3),a.anchor_local.begin());}
        else if(k<3){if(a.geom_id<0||a.site_id<0)throw std::invalid_argument("missing rider support geometry");a.rider_body=k==0?c_.pelvis_body_:c_.feet_bodies_[k-1];a.other_body=geom_bodyid[static_cast<std::size_t>(a.geom_id)];}
        else continue;
        attachments_.push_back(std::move(a));
    }
    for(std::size_t s=0;s<2;++s) {
        const auto knee=s?c_.pose_.knee_rear:c_.pose_.knee_front,ankle=s?c_.pose_.ankle_rear:c_.pose_.ankle_front;
        const auto upper=sub(knee,c_.pose_.hip),lower=sub(ankle,knee);const double a0=std::atan2(upper[2],upper[0]),b0=std::atan2(lower[2],lower[0]);
        legs_[s]={.upper=d::norm(upper),.lower=d::norm(lower),.a0=a0,.b0=b0,.branch=numeric::sin(b0-a0)>=0.?1:-1};
        if(legs_[s].upper<=0.||legs_[s].lower<=0.)throw std::invalid_argument("legacy rider requires nondegenerate ankle geometry");
        sole_offsets_[s]=mv(rot(c_.target_data_->xmat,m.nbody,c_.pedal_bodies_[s]),sub(vec(c_.target_data_->site_xpos,m.nsite,c_.sole_sites_[s]),vec(c_.target_data_->xpos,m.nbody,c_.pedal_bodies_[s])),true);
    }
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
Vector LegacyController::targets(const mjData& data,std::size_t s,double depth,double shear,const ::rider::RiderPosture* posture) {
    // NOLINTNEXTLINE(bugprone-unchecked-optional-access) LegacyController is only constructed with an engaged legacy config
    const auto& m=*c_.model_;const auto& cfg=*c_.config_.legacy;const std::string side=s?"rear":"front";
    const auto qpos=model_access::readonly_buffer(data.qpos,m.nq,"qpos");
    Vec hip=vec(data.xpos,m.nbody,c_.pelvis_body_);
    if(posture&&posture->pelvis_offset_m)hip=add(vec(data.xpos,m.nbody,c_.frame_body_),mv(rot(data.xmat,m.nbody,c_.frame_body_),add(c_.pose_.hip,{(*posture->pelvis_offset_m)[0],0.,(*posture->pelvis_offset_m)[1]})));
    const auto r=rot(data.xmat,m.nbody,c_.pelvis_body_);const auto geom=c_.pedal_geoms_[s];const auto origin=vec(data.geom_xpos,m.ngeom,geom),half=vec(m.geom_size,m.ngeom,geom);const auto rotation=rot(data.geom_xmat,m.ngeom,geom);
    Vec goal;bool saturated=false;auto& diagnostic=target_diagnostics_[s];diagnostic={};
    if(c_.welded()){goal=add(vec(data.xpos,m.nbody,c_.pedal_bodies_[s]),mv(rot(data.xmat,m.nbody,c_.pedal_bodies_[s]),sole_offsets_[s]));diagnostic.weld=true;}
    else if(const auto recovery=recovery_goal(get(c_.pedal_recovery_,side),origin,rotation,half,cfg.pedal_patch_half_length_m,cfg.support_pad_radius_m)){goal=*recovery;diagnostic.recovery=true;}
    else {const auto result=::rider::project_sole_goal(origin,rotation,half,cfg.pedal_patch_half_length_m,cfg.support_pad_radius_m,depth,shear);goal=result.goal;saturated=result.diagnostic.saturated;diagnostic.geometry=result.diagnostic;}
    set(c_.sole_targets_,side,goal);
    ::rider::SoleGoalDiagnostics goal_diagnostic;
    goal_diagnostic.spindle=false;goal_diagnostic.saturated=saturated;
    goal_diagnostic.geometry=diagnostic.geometry;goal_diagnostic.weld=diagnostic.weld;goal_diagnostic.recovery=diagnostic.recovery;
    if(diagnostic.geometry)goal_diagnostic.limiting_reasons=diagnostic.geometry->limiting_reasons;
    set(c_.sole_goal_diagnostics_,side,std::move(goal_diagnostic));
    const auto offset=sub(s?c_.pose_.ankle_rear:c_.pose_.ankle_front,s?c_.pose_.pedal_rear:c_.pose_.pedal_front);
    const auto target=mv(r,sub(add(goal,sub(offset,{0.,0.,.008})),hip),true);const auto leg=legs_[s];
    const auto ik=::spindle::two_link_ik({target[0],target[2]},leg.upper,leg.lower,leg.branch);
    set(c_.ik_reach_limited_,side,ik.saturated);set(c_.saturated_ik_,side,ik.saturated);
    const double h=leg.a0-ik.angles[0],k=(leg.b0-leg.a0)-ik.angles[1],pitch=std::atan2(-r[6],r[0]);
    const auto crank=rot(data.xmat,m.nbody,c_.crank_body_);const double phase=std::atan2(-crank[6],crank[0])+(s?pi:0.);
    // rider_program.py ankle rock: sin(phase), applied before ROM clipping.
    Vector result{h,k,-pitch-h-k+cfg.pedal_ankle_amplitude_rad*numeric::sin(phase)};result.resize(c_.leg_joint_index_[s].size());
    for(std::size_t j=0;j<result.size();++j) {
        const auto& joint=c_.joints_[c_.leg_joint_index_[s][j]];const double current=qpos[static_cast<std::size_t>(joint.qpos_adr)];result[j]=current+::spindle::wrap_angle(result[j]-current);
        if(const auto range=c_.joint_ranges_.find(joint.name);range!=c_.joint_ranges_.end()) {const double clipped=std::clamp(result[j],range->second[0],range->second[1]);if(clipped!=result[j])set(c_.saturated_ik_,side,true);result[j]=clipped;}
    }
    diagnostic.target=goal;diagnostic.actual=vec(data.site_xpos,m.nsite,c_.sole_sites_[s]);diagnostic.recovery_stage=get(c_.pedal_recovery_,side).stage;diagnostic.ik_saturated=get(c_.saturated_ik_,side);return result;
}

Vector LegacyController::upper_targets(const mjData& data,const ::rider::RiderPosture* posture,const Vec* grip_force) {
    // NOLINTNEXTLINE(bugprone-unchecked-optional-access) LegacyController is only constructed with an engaged legacy config
    const auto& m=*c_.model_;const auto& cfg=*c_.config_.legacy;const auto frame=rot(data.xmat,m.nbody,c_.frame_body_);const auto torso=rot(data.xmat,m.nbody,c_.torso_body_);
    const auto qpos=model_access::readonly_buffer(data.qpos,m.nq,"qpos");
    const double torso_q=c_.neutral_torso_q_+(posture?posture->torso_lean_rad:0.);Vector result(c_.joints_.size(),0.);
    result[c_.joint_index("rider_torso_hinge")]=torso_q;bool saturated=false;
    for(std::size_t s=0;s<2;++s) {
        Vec grip;
        if(cfg.grip_attachment=="connect")grip=add(vec(data.xpos,m.nbody,c_.steer_body_),mv(rot(data.xmat,m.nbody,c_.steer_body_),c_.weld_grip_offset_[s]));
        else {grip=add(vec(data.xpos,m.nbody,c_.frame_body_),mv(frame,add(c_.pose_.grip,{0.,(s?1.:-1.)*bike::model::ARM_LATERAL_OFFSET_M,0.})));if(grip_force)grip=add(grip,mul(*grip_force,1./(2.*cfg.grip_k_n_m)));}
        Vec shoulder=vec(data.xpos,m.nbody,c_.upper_arm_bodies_[s]);Rot target_rotation=torso;
        if(cfg.grip_attachment=="connect"&&cfg.coupled_task_control) {
            const double delta=::spindle::wrap_angle(torso_q-qpos[static_cast<std::size_t>(c_.joints_[c_.joint_index("rider_torso_hinge")].qpos_adr)]),co=numeric::cos(delta),si=numeric::sin(delta);const Rot turn{co,0.,si,0.,1.,0.,-si,0.,co};const auto hinge=vec(data.xanchor,m.njnt,c_.torso_joint_);shoulder=add(hinge,mv(turn,sub(shoulder,hinge)));
            for(std::size_t i=0;i<3;++i)for(std::size_t j=0;j<3;++j){target_rotation[3*i+j]=0.;for(std::size_t k=0;k<3;++k)target_rotation[3*i+j]+=turn[3*i+k]*torso[3*k+j];}
        }
        const auto goal=mv(target_rotation,sub(grip,shoulder),true);const auto ik=::spindle::two_link_ik({goal[0],goal[2]},c_.arm_upper_m_,c_.arm_lower_m_,c_.arm_branch_);saturated=saturated||ik.saturated;
        const std::string side=s?"right":"left";result[c_.joint_index("rider_shoulder_"+side)]=c_.arm_upper_angle0_-ik.angles[0];result[c_.joint_index("rider_elbow_"+side)]=(c_.arm_lower_angle0_-c_.arm_upper_angle0_)-ik.angles[1];
    }
    set(c_.saturated_ik_,"torso",c_.neutral_torso_saturated_);set(c_.saturated_ik_,"arms",saturated);
    for(std::size_t i=0;i<c_.joints_.size();++i) {
        const auto& j=c_.joints_[i];if(j.name.starts_with("rider_hip_")||j.name.starts_with("rider_knee_")||j.name.starts_with("rider_ankle_"))continue;
        const double current=qpos[static_cast<std::size_t>(j.qpos_adr)];result[i]=current+::spindle::wrap_angle(result[i]-current);
        if(const auto range=c_.joint_ranges_.find(j.name);range!=c_.joint_ranges_.end()){const double clip=std::clamp(result[i],range->second[0],range->second[1]);if(clip!=result[i])set(c_.saturated_ik_,j.name=="rider_torso_hinge"?"torso":"arms",true);result[i]=clip;}
    }
    return result;
}

std::array<double,2> LegacyController::estimate(const mjData& data,std::size_t s) const {
    // NOLINTNEXTLINE(bugprone-unchecked-optional-access) LegacyController is only constructed with an engaged legacy config
    const auto& m=*c_.model_;const auto& cfg=*c_.config_.legacy;const auto geom=c_.pedal_geoms_[s];const auto origin=vec(data.geom_xpos,m.ngeom,geom),half=vec(m.geom_size,m.ngeom,geom),sole=vec(data.site_xpos,m.nsite,c_.sole_sites_[s]);const auto r=rot(data.geom_xmat,m.ngeom,geom),foot=rot(data.xmat,m.nbody,c_.feet_bodies_[s]);std::array<double,2> result{};
    const auto geom_bodyid=model_access::readonly_buffer(m.geom_bodyid,m.ngeom,"geom_bodyid");
    for(double const sign:{-1.,1.}) {
        const auto center=add(sole,mv(foot,{sign*cfg.pedal_patch_half_length_m,0.,cfg.support_pad_radius_m}));const auto contact=::rider::box_pad_contact(center,cfg.support_pad_radius_m,origin,r,half);
        if(!contact.within_footprint||contact.gap_m>=0.)continue;
        const auto relative=sub(velocity(m,data,c_.feet_bodies_[s],contact.point_m),velocity(m,data,geom_bodyid[static_cast<std::size_t>(geom)],contact.point_m));
        const auto normal=contactlaw::normal_contact(-contact.gap_m,-d::dot(relative,contact.normal),cfg.support_k_n_m/2.,cfg.pedal_c_ns_m/2.).first;result[0]+=normal;result[1]+=normal*contact.normal[2];
    }
    return result;
}

ResponseRequest LegacyController::allocation_request(const mjData& data,std::span<const double> intent,double dt,bool steady,std::optional<double> crank) const {
    // NOLINTNEXTLINE(bugprone-unchecked-optional-access) LegacyController is only constructed with an engaged legacy config
    const auto& m=*c_.model_;const auto& cfg=*c_.config_.legacy;ResponseRequest out;out.config=cfg;out.torque_intent.assign(intent.begin(),intent.end());out.activation_state=c_.active_state_;out.attachments=attachments_;out.rider_dofs=rider_dofs_;out.bike_dofs=bike_dofs_;out.pelvis_body=c_.pelvis_body_;out.crank_dof=c_.crank_spin_dof_;out.rider_mass_kg=c_.rider_mass_;out.dt_s=dt;out.steady_state=steady;out.crank_torque_nm=crank;
    const auto qvel=model_access::readonly_buffer(data.qvel,m.nv,"qvel");
    const auto act_forcelimited=model_access::readonly_buffer(m.actuator_forcelimited,m.nu,"actuator_forcelimited");
    const auto act_forcerange=model_access::readonly_buffer(m.actuator_forcerange,m.nu*2,"actuator_forcerange");
    const auto act_ctrllimited=model_access::readonly_buffer(m.actuator_ctrllimited,m.nu,"actuator_ctrllimited");
    const auto act_ctrlrange=model_access::readonly_buffer(m.actuator_ctrlrange,m.nu*2,"actuator_ctrlrange");
    for(const auto& j:c_.joints_) {
        out.actuator_dofs.push_back(j.dof_adr);const double speed=qvel[static_cast<std::size_t>(j.dof_adr)];double positive=c_.directional_limit(j.name,&data,1.),negative=c_.directional_limit(j.name,&data,-1.);
        if(speed>0.){positive=std::min(positive,cfg.joint_power_limit_w/speed);if(speed>=cfg.joint_speed_limit_rad_s)positive=0.;}
        else if(speed<0.){negative=std::min(negative,-cfg.joint_power_limit_w/speed);if(-speed>=cfg.joint_speed_limit_rad_s)negative=0.;}
        double lo=-negative,hi=positive;
        const auto act_id=static_cast<std::size_t>(j.actuator_id);
        if(act_forcelimited[act_id]){lo=std::max(lo,act_forcerange[2*act_id]);hi=std::min(hi,act_forcerange[2*act_id+1]);}
        if(act_ctrllimited[act_id]){lo=std::max(lo,act_ctrlrange[2*act_id]);hi=std::min(hi,act_ctrlrange[2*act_id+1]);}
        out.lower_torque_nm.push_back(lo);out.upper_torque_nm.push_back(hi);
    }
    return out;
}

void LegacyController::initialize_velocity(mjData& data) {
    const auto& m=*c_.model_;
    auto const qvel=model_access::mutable_buffer(data.qvel,m.nv,"qvel");
    c_.predict_target_state(&data,c_.target_data_.get(),false);c_.predict_target_state(&data,c_.previous_target_data_.get(),true);
    for(std::size_t s=0;s<2;++s) {
        // NOLINTNEXTLINE(bugprone-unchecked-optional-access) LegacyController is only constructed with an engaged legacy config
        const auto next=targets(*c_.target_data_,s,c_.config_.legacy->posture_sole_depth_m,0.,nullptr),previous=targets(*c_.previous_target_data_,s,c_.config_.legacy->posture_sole_depth_m,0.,nullptr);
        for(std::size_t j=0;j<next.size();++j)qvel[static_cast<std::size_t>(c_.joints_[c_.leg_joint_index_[s][j]].dof_adr)]=::spindle::wrap_angle(next[j]-previous[j])/(2.*c_.target_difference_s_);
    }
}
void LegacyController::initialize(mjData& data) {
    const auto& m=*c_.model_;
    auto const qpos=model_access::mutable_buffer(data.qpos,m.nq,"qpos");
    engine::forward(c_.model_,&data);c_.reset_activation();auto upper=upper_targets(data,nullptr,nullptr);
    qpos[static_cast<std::size_t>(c_.joints_[c_.joint_index("rider_torso_hinge")].qpos_adr)]=upper[c_.joint_index("rider_torso_hinge")];engine::forward(c_.model_,&data);upper=upper_targets(data,nullptr,nullptr);
    for(std::size_t i=0;i<c_.joints_.size();++i){const auto& j=c_.joints_[i];if(!j.name.starts_with("rider_hip_")&&!j.name.starts_with("rider_knee_")&&!j.name.starts_with("rider_ankle_"))qpos[static_cast<std::size_t>(j.qpos_adr)]=upper[i];}
    engine::forward(c_.model_,&data);
    for(std::size_t s=0;s<2;++s){// NOLINTNEXTLINE(bugprone-unchecked-optional-access) LegacyController is only constructed with an engaged legacy config
        const auto goal=targets(data,s,c_.welded()?0.:c_.config_.legacy->posture_sole_depth_m,0.,nullptr);if(get(c_.ik_reach_limited_,s?"rear":"front"))throw std::invalid_argument(std::string("initial ")+(s?"rear":"front")+" pedal is unreachable");for(std::size_t j=0;j<goal.size();++j)qpos[static_cast<std::size_t>(c_.joints_[c_.leg_joint_index_[s][j]].qpos_adr)]=goal[j];}
    engine::forward(c_.model_,&data);
}

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
::rider::NamedEntries<double> LegacyController::compute(mjData& data,const ::rider::RiderCommand& command,
    bool advance,double dt,bool steady,const std::array<bool,4>& available,bool recovery) {
    const auto& m=*c_.model_;const auto& cfg=*c_.config_.legacy;const auto nt=c_.joints_.size();
    const auto qpos=model_access::readonly_buffer(data.qpos,m.nq,"qpos");
    const auto qvel=model_access::readonly_buffer(data.qvel,m.nv,"qvel");
    const auto qfrc_bias=model_access::readonly_buffer(data.qfrc_bias,m.nv,"qfrc_bias");
    const auto body_mass=model_access::readonly_buffer(m.body_mass,m.nbody,"body_mass");
    const auto body_subtreemass=model_access::readonly_buffer(m.body_subtreemass,m.nbody,"body_subtreemass");
    if(!std::isfinite(dt)||dt<0.||!std::isfinite(command.mean_crank_torque_nm)||command.mean_crank_torque_nm<0.||!std::isfinite(command.crank_target_rate_rad_s))throw std::invalid_argument("invalid articulated command");
    c_.command_enabled_=command.enabled&&c_.enabled_;::rider::NamedEntries<double> output;
    if(!c_.command_enabled_) {
        if(advance)c_.reset_activation();
        c_.last_terms_.clear();
        c_.effort_diagnostics_={};
        c_.effort_diagnostics_.present=true;
        c_.effort_diagnostics_.budget_present=false;c_.effort_diagnostics_.power_limit_present=false;c_.effort_diagnostics_.observation_present=false;c_.effort_diagnostics_.strength_violations=std::vector<std::string>{};
        for(const auto& j:c_.joints_){output.emplace_back(j.name,0.);::rider::JointTerms term;term.requested_present=false;c_.last_terms_.emplace_back(j.name,term);c_.effort_diagnostics_.active_request_nm.emplace_back(j.name,0.);c_.effort_diagnostics_.active_delivered_nm.emplace_back(j.name,0.);}
        c_.allocation_diagnostics_={};c_.allocation_diagnostics_.present=true;
        ResponseAllocation disabled;disabled.disabled=true;disabled.allocation.feasible=true;disabled.allocation.violation=0.;c_.allocation_diagnostics_.legacy=std::move(disabled);return output;
    }
    const auto original_recovery=c_.pedal_recovery_;std::array<std::array<double,2>,2> estimates{estimate(data,0),estimate(data,1)};
    for(std::size_t s=0;s<2;++s)if(recovery&&!steady&&!c_.welded()){const auto g=c_.pedal_geoms_[s];observe_recovery(get(c_.pedal_recovery_,s?"rear":"front"),vec(data.geom_xpos,m.ngeom,g),rot(data.geom_xmat,m.ngeom,g),vec(m.geom_size,m.ngeom,g),vec(data.site_xpos,m.nsite,c_.sole_sites_[s]),estimates[s][1],cfg.pedal_patch_half_length_m,cfg.support_pad_radius_m);}
    const Vec gravity{m.opt.gravity[0],m.opt.gravity[1],m.opt.gravity[2]};const double weight=c_.rider_mass_*d::norm(gravity);Vec com{};
    for(int const body:c_.rider_bodies_)com=add(com,mul(vec(data.xipos,m.nbody,body),body_mass[static_cast<std::size_t>(body)]/c_.rider_mass_));
    const auto frame=rot(data.xmat,m.nbody,c_.frame_body_),pelvis=rot(data.xmat,m.nbody,c_.pelvis_body_);const Vec saddle=add(vec(data.xpos,m.nbody,c_.pelvis_body_),mv(pelvis,{0.,0.,-.060}));const Vec grip=mul(add(vec(data.site_xpos,m.nsite,c_.grip_sites_[0]),vec(data.site_xpos,m.nsite,c_.grip_sites_[1])),.5);
    const std::array<Vec,4> points{saddle,vec(data.site_xpos,m.nsite,c_.pedal_sites_[0]),vec(data.site_xpos,m.nsite,c_.pedal_sites_[1]),grip};auto enabled=available;enabled[0]=enabled[0]&&command.posture.use_saddle;
    const double pitch_error=::spindle::wrap_angle(std::atan2(-frame[6],frame[0])-std::atan2(-pelvis[6],pelvis[0])+command.posture.pelvis_pitch_rad);
    const double pitch=std::clamp(cfg.posture_pitch_k_nm_rad*pitch_error+cfg.posture_pitch_d_nms_rad*(qvel[static_cast<std::size_t>(c_.frame_pitch_dof_)]-qvel[static_cast<std::size_t>(c_.rider_pitch_dof_)]),-cfg.posture_pitch_limit_nm,cfg.posture_pitch_limit_nm);
    const auto crank_rotation=rot(data.xmat,m.nbody,c_.crank_body_);const double phase=std::atan2(-crank_rotation[6],crank_rotation[0]);
    const bool coupled=cfg.coupled_task_control&&c_.welded()&&cfg.grip_attachment=="connect"&&cfg.saddle_attachment=="weld";
    std::optional<double> crank_task;if(coupled&&command.mean_crank_torque_nm>0.)crank_task=command.mean_crank_torque_nm*(1.+cfg.pedal_torque_ripple*numeric::cos(2.*phase));
    double coast=0.;if(command.crank_target_phase_rad&&!steady){coast=std::clamp(cfg.coasting_brake_d_nm_s_rad*(command.crank_target_rate_rad_s-qvel[static_cast<std::size_t>(c_.crank_spin_dof_)]),-cfg.coasting_brake_limit_nm,cfg.coasting_brake_limit_nm);if(coupled&&!crank_task)crank_task=coast;}
    std::array<Vec,2> normals{};std::array<double,2> loads{};std::array<bool,2> recovered{};
    for(std::size_t s=0;s<2;++s){const auto g=c_.pedal_geoms_[s];normals[s]=::rider::upper_box_face(vec(data.geom_xpos,m.ngeom,g),rot(data.geom_xmat,m.ngeom,g),vec(m.geom_size,m.ngeom,g)).normal;loads[s]=c_.welded()?weight:std::max(estimates[s][0],0.);recovered[s]=get(c_.pedal_recovery_,s?"rear":"front").stage=="none";}
    PedalingRequests requests;if(command.mean_crank_torque_nm>0.)requests=pedal_requests(phase,command.mean_crank_torque_nm,c_.crank_length_m_,normals,loads,cfg.support_mu,cfg.pedal_torque_ripple,cfg.return_foot_preload_n,recovered);
    std::array<double,2> blends{};
    for(std::size_t s=0;s<2;++s) {
        const double fraction=std::min(loads[s]/cfg.stance_blend_load_n,1.);blends[s]=fraction*fraction*(3.-2.*fraction);
        if(!recovered[s]){blends[s]=0.;requests.force[s]={};}
        else if(command.mean_crank_torque_nm==0.){const double bp=phase+(s?pi:0.)+(coast<0.?pi:0.),f=std::clamp(numeric::cos(bp)/pressure_blend_cosine,0.,1.);requests.force[s]=feasible_pedal_force({0.,0.,-std::abs(coast)/c_.crank_length_m_*f*f*(3.-2.*f)},normals[s],cfg.support_mu,loads[s]);}
        else if(requests.weights[s]<.5){const Vec t{normals[s][2],0.,-normals[s][0]};const double cap=cfg.pedal_scrape_fraction*cfg.foot_mu*loads[s],part=d::dot(requests.force[s],t);requests.force[s]=add(requests.force[s],mul(t,std::clamp(part,-cap,cap)-part));}
    }
    const auto offset=command.posture.pelvis_offset_m.value_or(std::array<double,2>{});const Vec wanted=add(vec(data.xpos,m.nbody,c_.frame_body_),mv(frame,add(c_.pose_.hip,{offset[0],0.,offset[1]}))),actual=vec(data.xpos,m.nbody,c_.pelvis_body_);
    Vec balance=add(mul(sub(wanted,actual),cfg.posture_translation_k_n_m),mul(sub(velocity(m,data,c_.frame_body_,wanted),velocity(m,data,c_.pelvis_body_,actual)),cfg.posture_translation_d_ns_m));balance[1]=0.;const double balance_norm=d::norm(balance);if(balance_norm>cfg.posture_translation_limit_n)balance=mul(balance,cfg.posture_translation_limit_n/balance_norm);
    const auto support=pedaling_support_targets(weight,com,points,vec(data.xpos,m.nbody,c_.crank_body_)[0],cfg.pedal_support_fraction,cfg.bar_support_fraction,enabled,requests.force,pitch,balance);
    c_.support_diagnostics_={.present=true,.stance_front=recovered[0]&&(command.mean_crank_torque_nm==0.||requests.weights[0]>1e-6),.stance_rear=recovered[1]&&(command.mean_crank_torque_nm==0.||requests.weights[1]>1e-6)};
    LegacySupportDiagnostics support_diagnostics;support_diagnostics.target=support.diagnostics;support_diagnostics.stance={c_.support_diagnostics_.stance_front,c_.support_diagnostics_.stance_rear};support_diagnostics.pedal_forces=requests.force;support_diagnostics.posture=command.posture;support_diagnostics.coasting=command.crank_target_phase_rad.has_value();support_diagnostics.coasting_requested_crank_torque_nm=coast;support_diagnostics.actual_phase_rad=qpos[static_cast<std::size_t>(c_.crank_spin_qpos_)];support_diagnostics.target_phase_rad=command.crank_target_phase_rad;support_diagnostics.actual_rate_rad_s=qvel[static_cast<std::size_t>(c_.crank_spin_dof_)];support_diagnostics.target_rate_rad_s=command.crank_target_rate_rad_s;if(command.crank_target_phase_rad)support_diagnostics.phase_error_rad=::spindle::wrap_angle(*command.crank_target_phase_rad-support_diagnostics.actual_phase_rad);
    auto upper=upper_targets(data,&command.posture,&support.forces_on_bike[3]);const mjData* target=&data;
    if(command.crank_target_phase_rad || (command.mean_crank_torque_nm>0.&&command.crank_target_rate_rad_s>0.)) {
        auto* scratch=c_.coasting_target_data_.get();
        std::ranges::copy(qpos,model_access::mutable_buffer(scratch->qpos,m.nq,"scratch.qpos").begin());
        auto const scratch_qvel=model_access::mutable_buffer(scratch->qvel,m.nv,"scratch.qvel");
        std::ranges::fill(scratch_qvel,0.);
        const bool follow=command.crank_target_phase_rad.has_value()&&!steady;const double rate=follow?qvel[static_cast<std::size_t>(c_.crank_spin_dof_)]:command.crank_target_rate_rad_s;
        scratch_qvel[static_cast<std::size_t>(c_.crank_spin_dof_)]=rate;for(int const i:c_.pedal_spin_dofs_)scratch_qvel[static_cast<std::size_t>(i)]=-rate;engine::kinematics(&m,scratch);target=scratch;
    }
    const mjData* future=target;const mjData* previous=target;
    if(model_access::readonly_buffer(target->qvel,m.nv,"target.qvel")[static_cast<std::size_t>(c_.crank_spin_dof_)]!=0.){c_.predict_target_state(target,c_.target_data_.get(),false);c_.predict_target_state(target,c_.previous_target_data_.get(),true);future=c_.target_data_.get();previous=c_.previous_target_data_.get();}
    const auto saturation=c_.saturated_ik_,reach=c_.ik_reach_limited_;auto future_upper=future!=&data?upper_targets(*future,&command.posture,&support.forces_on_bike[3]):upper;c_.saturated_ik_=saturation;c_.ik_reach_limited_=reach;
    Vector posture(nt,0.),pedaling(nt,0.),desired_accel(static_cast<std::size_t>(m.nv),0.),support_force(static_cast<std::size_t>(m.nv),0.);
    for(std::size_t i=0;i<nt;++i){const auto&j=c_.joints_[i];if(j.name.starts_with("rider_hip_")||j.name.starts_with("rider_knee_")||j.name.starts_with("rider_ankle_"))continue;const double speed=::spindle::wrap_angle(future_upper[i]-upper[i])/c_.target_difference_s_;const double p=cfg.joint_kp_nm_rad*(upper[i]-qpos[static_cast<std::size_t>(j.qpos_adr)]),derivative=cfg.joint_kd_nms_rad*(speed-qvel[static_cast<std::size_t>(j.dof_adr)]);posture[i]=p+derivative;set(c_.pd_split_,j.name,std::array<double,2>{p,derivative});}
    for(std::size_t s=0;s<2;++s) {
        const Vec tangent{normals[s][2],0.,-normals[s][0]};const double depth=(1.-blends[s])*cfg.posture_sole_depth_m+blends[s]*std::max(0.,-d::dot(support.forces_on_bike[s+1],normals[s]))/cfg.support_k_n_m,shear=blends[s]*d::dot(support.forces_on_bike[s+1],tangent)/cfg.support_tangent_k_n_m;
        const auto goal=targets(*target,s,depth,shear,&command.posture);const auto sat=c_.saturated_ik_,lim=c_.ik_reach_limited_;
        support_diagnostics.feet[s]=target_diagnostics_[s];support_diagnostics.feet[s].actual=vec(data.site_xpos,m.nsite,c_.sole_sites_[s]);
        const auto next=future!=target?targets(*future,s,depth,shear,&command.posture):goal,prev=previous!=target?targets(*previous,s,depth,shear,&command.posture):goal;c_.saturated_ik_=sat;c_.ik_reach_limited_=lim;
        const auto jp=jacobian(m,data,c_.feet_bodies_[s],vec(data.site_xpos,m.nsite,c_.sole_sites_[s]));const auto feed=d::multiply(d::transpose(jp),requests.force[s]);
        for(std::size_t k=0;k<goal.size();++k){const auto i=c_.leg_joint_index_[s][k];const auto&j=c_.joints_[i];const double ahead=::spindle::wrap_angle(next[k]-goal[k]),behind=::spindle::wrap_angle(prev[k]-goal[k]);const double p=cfg.joint_kp_nm_rad*(goal[k]-qpos[static_cast<std::size_t>(j.qpos_adr)]),derivative=cfg.joint_kd_nms_rad*((ahead-behind)/(2.*c_.target_difference_s_)-qvel[static_cast<std::size_t>(j.dof_adr)]);posture[i]=p+derivative;set(c_.pd_split_,j.name,std::array<double,2>{p,derivative});desired_accel[static_cast<std::size_t>(j.dof_adr)]=(ahead+behind)/(c_.target_difference_s_*c_.target_difference_s_);pedaling[i]=coupled?0.:feed[static_cast<std::size_t>(j.dof_adr)];}
        if(enabled[s+1]&&!coupled)support_force=d::add(std::move(support_force),d::multiply(d::transpose(jp),Vec{0.,0.,-support.diagnostics.vertical_forces_n[s+1]}));
    }
    if(enabled[3]) {
        Vec grip_force=support.forces_on_bike[3];
        if(coupled){const auto hinge=vec(data.xanchor,m.njnt,c_.torso_joint_),trunk=vec(data.subtree_com,m.nbody,c_.torso_body_);const double lever=grip[0]-hinge[0],floor=cfg.bar_support_fraction*weight,need=lever>.05?body_subtreemass[static_cast<std::size_t>(c_.torso_body_)]*d::norm(gravity)*(trunk[0]-hinge[0])/lever:floor;grip_force={0.,0.,-std::min(std::max(need,floor),2.*cfg.grip_pull_per_hand_n)};}
        for(std::size_t s=0;s<2;++s)support_force=d::add(std::move(support_force),d::multiply(d::transpose(jacobian(m,data,c_.forearm_bodies_[s],vec(data.site_xpos,m.nsite,c_.grip_sites_[s]))),mul(grip_force,.5)));
    }
    c_.support_diagnostics_.legacy=std::move(support_diagnostics);
    Vector inertia(static_cast<std::size_t>(m.nv));engine::mul_m(&m,&data,inertia.data(),desired_accel.data());Vector requested(nt);
    for(std::size_t i=0;i<nt;++i){const auto&j=c_.joints_[i];posture[i]+=qfrc_bias[static_cast<std::size_t>(j.dof_adr)]+support_force[static_cast<std::size_t>(j.dof_adr)]+inertia[static_cast<std::size_t>(j.dof_adr)];if(j.name.starts_with("rider_hip_"))posture[i]-=.5*pitch;requested[i]=posture[i]+pedaling[i];}
    const double gain=steady||cfg.activation_tau_s==0.?1.:-std::expm1(-dt/cfg.activation_tau_s);const auto target_torque=steady ? requested : ::spindle::activation_step(c_.active_state_,requested,dt,cfg.activation_tau_s);
    ResponseAllocation allocated;double used_scale=1.;
    const Vector effort_ladder=crank_task ? Vector{1.,.5,.25,0.} : Vector{1.,.5,.25};
    for(double const scale:effort_ladder) {
        auto wish=target_torque;for(std::size_t i=0;i<nt;++i)wish[i]-=(1.-scale)*gain*pedaling[i];auto request=allocation_request(data,wish,dt,steady,crank_task&&scale>0.?std::optional<double>(*crank_task*scale):std::nullopt);request.effort_scale=scale;
        allocated=std::ranges::all_of(attachments_,[](const auto&a){return a.equality_id>=0;})?allocate_response(m,data,request,c_.allocation_memory_):allocate_uncoupled(data,request,support,estimates);used_scale=scale;if(allocated.allocation.feasible)break;
    }
    allocated.effort_scale=used_scale;allocated.qpos.assign(qpos.begin(),qpos.end());allocated.qvel.assign(qvel.begin(),qvel.end());allocated.time_s=data.time;
    c_.allocation_diagnostics_={};c_.allocation_diagnostics_.present=true;c_.allocation_diagnostics_.invalid_controller=!allocated.allocation.feasible;c_.allocation_diagnostics_.crank_task_nm=crank_task.value_or(0.);c_.allocation_diagnostics_.crank_task_shortfall_nm=std::max(0.,crank_task.value_or(0.)-allocated.predicted_crank_torque_nm.value_or(0.));c_.allocation_diagnostics_.solution_excitation_nm=allocated.excitation;c_.allocation_diagnostics_.legacy=allocated;
    // Final actuator check uses exactly the solved excitation and preserves
    // latent (pre-clamp) state. An infeasible support fallback stays flagged.
    if(advance&&!steady&&cfg.activation_tau_s>0.&&c_.activation_time_s_&&data.time<=*c_.activation_time_s_)throw std::invalid_argument("rider activation advances once per timestamp");
    const bool effort_model=cfg.activation_tau_s>0.||cfg.active_positive_power_limit_w.has_value()||c_.config_.has_strength;
    const auto latent=effort_model ? (steady ? allocated.excitation : ::spindle::activation_step(c_.active_state_,allocated.excitation,dt,cfg.activation_tau_s)) : allocated.torque;auto active=latent;
    if(effort_model)for(std::size_t i=0;i<nt;++i)active[i]=std::clamp(active[i],allocated.lower_torque_nm[i],allocated.upper_torque_nm[i]);
    Vector speeds(nt);for(std::size_t i=0;i<nt;++i)speeds[i]=qvel[static_cast<std::size_t>(c_.joints_[i].dof_adr)];if(effort_model&&cfg.active_positive_power_limit_w)active=::spindle::limit_positive_power(active,speeds,*cfg.active_positive_power_limit_w);
    for(std::size_t i=0;i<nt;++i)if(std::abs(active[i]-allocated.torque[i])>1e-7+1e-9*std::abs(allocated.torque[i]))throw std::runtime_error("allocated rider torque is outside final actuator limits");
    if(advance&&effort_model){c_.active_state_=latent;c_.activation_time_s_=steady||cfg.activation_tau_s==0.?std::nullopt:std::optional<double>(data.time);}else if(!advance)c_.pedal_recovery_=original_recovery;
    c_.last_terms_.clear();c_.effort_diagnostics_={};auto& effort=c_.effort_diagnostics_;effort.present=true;effort.positive_power_limit_w=cfg.active_positive_power_limit_w;effort.observation="incoming_request";
    Vector positive(nt),passive(nt);
    for(std::size_t i=0;i<nt;++i) {
        const auto&j=c_.joints_[i];const double scale=requested[i]!=0.?active[i]/requested[i]:0.;::rider::JointTerms term;term.requested_nm=requested[i];term.command_nm=active[i];term.posture_nm=(requested[i]-pedaling[i])*scale;term.pedaling_nm=pedaling[i]*scale;term.support_nm=support_force[static_cast<std::size_t>(j.dof_adr)]*scale;term.tracking_nm=inertia[static_cast<std::size_t>(j.dof_adr)]*scale;term.hip_posture_nm=(j.name.starts_with("rider_hip_")?-.5*pitch:0.)*scale;term.saturated=scale!=1.;term.active_request_nm=allocated.excitation[i];term.active_delivered_nm=active[i];passive[i]=-cfg.passive_damping_nms_rad()*speeds[i];term.passive_damping_nm=passive[i];c_.last_terms_.emplace_back(j.name,term);output.emplace_back(j.name,active[i]);set(c_.joint_torques_nm_,j.name,active[i]);set(c_.joint_capacity_nm_,j.name,std::max(std::abs(allocated.lower_torque_nm[i]),allocated.upper_torque_nm[i]));effort.active_request_nm.emplace_back(j.name,allocated.excitation[i]);effort.active_delivered_nm.emplace_back(j.name,active[i]);positive[i]=std::max(active[i]*speeds[i],0.);effort.activation_saturated=effort.activation_saturated||std::abs(active[i]-allocated.excitation[i])>1e-10+1e-10*std::abs(allocated.excitation[i]);
    }
    effort.positive_power_w=numeric::numpy_pairwise_sum(positive);effort.passive_power_w=blas::ddot(static_cast<int>(nt),passive.data(),1,speeds.data(),1);
    if(effort_model)effort.strength_limited=c_.strength_limited(latent,qpos,qvel).second;
    return output;
}
NATIVE_DIAG_POP
} // namespace bike::rider
