// rider_control.py allocate(): independent rider acceleration, measured
// base acceleration and unilateral flat-pad support. MuJoCo calls are framed;
// all callback and solver storage is owned by this invocation.
#include "legacy_controller.hpp"
#include "activation_allocation.hpp"
#include "support_geometry.hpp"
#include "../engine_call.hpp"
#include "../numerics/dense.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace bike::rider {
namespace {
namespace d=bike::numerics::dense;
using Vector=std::vector<double>;
using Vec=std::array<double,3>;
constexpr double inf=std::numeric_limits<double>::infinity();
Vec row3(const double* p,int i){return {p[3*i],p[3*i+1],p[3*i+2]};}
std::array<double,9> rotation(const double*p,int i){std::array<double,9> r{};std::copy_n(p+9*i,9,r.begin());return r;}
struct Frame {Matrix linear,angular,linear_rate,angular_rate;};
Frame frame(const mjModel& m,const mjData& data,const Vec& point,int body,bool rate) {
    Frame f{d::zeros(3,static_cast<std::size_t>(m.nv)),d::zeros(3,static_cast<std::size_t>(m.nv)),
            d::zeros(3,static_cast<std::size_t>(m.nv)),d::zeros(3,static_cast<std::size_t>(m.nv))};
    struct Call {const mjModel*m;const mjData*d;const double*p;int body;double*jp;double*jr;double*jdp;double*jdr;bool rate;};
    Call call{&m,&data,point.data(),body,f.linear.values.data(),f.angular.values.data(),f.linear_rate.values.data(),f.angular_rate.values.data(),rate};
    engine::ErrorBuffer error;
    if(!engine::invoke([](void*raw) noexcept{auto&c=*static_cast<Call*>(raw);mj_jac(c.m,c.d,c.jp,c.jr,c.p,c.body);if(c.rate)mj_jacDot(c.m,c.d,c.jdp,c.jdr,c.p,c.body);},&call,error))engine::throw_failure(error);
    return f;
}
}

ResponseAllocation LegacyController::allocate_uncoupled(const mjData& data,
    const ResponseRequest& request,const PedalingSupportTargets& support,
    const std::array<std::array<double,2>,2>& estimates) {
    const auto& m=*c_.model_;const auto& cfg=request.config;
    const std::size_t nr=rider_dofs_.size(),nt=c_.joints_.size(),nf=2*attachments_.size(),nv=static_cast<std::size_t>(m.nv);
    const std::size_t it=nr,iforce=it+nt,ip=iforce+nf,base=ip+nt;
    const bool activated=cfg.activation_tau_s>0.&&!request.steady_state;
    const std::size_t nz=base+(activated?nt:0);
    auto mass=d::zeros(nv,nv);Vector unit(nv),column(nv);
    for(std::size_t j=0;j<nv;++j){std::fill(unit.begin(),unit.end(),0.);unit[j]=1.;engine::mul_m(&m,&data,column.data(),unit.data());for(std::size_t i=0;i<nv;++i)mass.values[i*nv+j]=column[i];}
    auto mr=d::zeros(nr,nr),act=d::zeros(nr,nt),jac=d::zeros(nf,nr);
    Vector bias(nr),known(nr),speed(nt),qbase(bike_dofs_.size());
    for(std::size_t j=0;j<bike_dofs_.size();++j)qbase[j]=data.qacc[bike_dofs_[j]];
    auto passive=c_.envelope_forces(&data).first;
    for(const auto& joint:c_.joints_)passive[static_cast<std::size_t>(joint.dof_adr)]-=cfg.passive_damping_nms_rad()*data.qvel[joint.dof_adr];
    for(std::size_t i=0;i<nr;++i){const auto dof=static_cast<std::size_t>(rider_dofs_[i]);bias[i]=data.qfrc_bias[dof];known[i]=passive[dof];for(std::size_t j=0;j<nr;++j)mr.values[i*nr+j]=mass.values[dof*nv+static_cast<std::size_t>(rider_dofs_[j])];for(std::size_t j=0;j<bike_dofs_.size();++j)bias[i]+=mass.values[dof*nv+static_cast<std::size_t>(bike_dofs_[j])]*qbase[j];}
    for(std::size_t i=0;i<nt;++i){const int dof=c_.joints_[i].dof_adr;const auto found=std::ranges::find(rider_dofs_,dof);if(found==rider_dofs_.end())throw std::invalid_argument("rider actuator outside rider DOFs");act.values[static_cast<std::size_t>(found-rider_dofs_.begin())*nt+i]=1.;speed[i]=data.qvel[dof];if(request.lower_torque_nm[i]>request.upper_torque_nm[i])throw std::invalid_argument("empty rider actuator bounds");}
    auto inequalities=d::zeros(0,nz);Vector rhs;
    const auto append=[&](const Vector& row,double bound){d::append_row(inequalities,row);rhs.push_back(bound);};
    for(std::size_t k=0;k<attachments_.size();++k) {
        const auto& a=attachments_[k];Vec point;
        if(a.equality_id<0)point=row3(data.site_xpos,a.site_id);
        else point=::rider::add(row3(data.xpos,a.rider_body),::rider::matvec(rotation(data.xmat,a.rider_body),a.anchor_local));
        const auto one=frame(m,data,point,a.rider_body,a.equality_id>=0);
        for(std::size_t row=0;row<2;++row)for(std::size_t j=0;j<nr;++j)jac.values[(2*k+row)*nr+j]=one.linear.values[(row?2:0)*nv+static_cast<std::size_t>(rider_dofs_[j])];
        if(a.equality_id<0)continue;
        const auto two=frame(m,data,point,a.other_body,true);
        for(std::size_t kind=0;kind<(a.weld?2U:1U);++kind) {
            const auto jrel=d::add(kind?one.angular:one.linear,kind?two.angular:two.linear,-1.);
            const auto jdot=d::add(kind?one.angular_rate:one.linear_rate,kind?two.angular_rate:two.linear_rate,-1.);
            for(std::size_t row=0;row<3;++row) {
                double b=0.;for(std::size_t j=0;j<bike_dofs_.size();++j)b-=jrel.values[row*nv+static_cast<std::size_t>(bike_dofs_[j])]*qbase[j];
                for(std::size_t j=0;j<nv;++j)b-=jdot.values[row*nv+j]*data.qvel[j];
                const double tolerance=50.+.5*std::abs(b);Vector g(nz);
                for(std::size_t j=0;j<nr;++j)g[j]=jrel.values[row*nv+static_cast<std::size_t>(rider_dofs_[j])];append(g,b+tolerance);for(double&v:g)v=-v;append(g,tolerance-b);
            }
        }
    }
    const auto dynamics=inverse_dynamics_rows(mr,act,jac,bias,known);
    const double weight=request.rider_mass_kg*std::hypot(m.opt.gravity[0],m.opt.gravity[1],m.opt.gravity[2]);
    for(std::size_t k=0;k<attachments_.size();++k) {
        const auto& a=attachments_[k];if(a.name.starts_with("grip"))continue;
        std::array<double,2> normal{0.,1.};double predicted=0.;bool recovering=false;
        if(a.name=="front"||a.name=="rear") {
            const std::size_t side=a.name=="front"?0:1;const auto g=c_.pedal_geoms_[side];
            const auto face=::rider::upper_box_face(row3(data.geom_xpos,g),rotation(data.geom_xmat,g),row3(m.geom_size,g));normal={face.normal[0],face.normal[2]};predicted=estimates[side][0];
            for(const auto&[name,state]:c_.pedal_recovery_)if(name==a.name)recovering=state.stage!="none";
        }
        const std::array<double,2> tangent{-normal[1],normal[0]};const double mu=a.name=="saddle"?cfg.saddle_mu:cfg.foot_mu;const auto f=iforce+2*k;
        for(double sign:{1.,-1.}){Vector g(nz);for(std::size_t j=0;j<2;++j)g[f+j]=sign*tangent[j]-mu*normal[j];append(g,0.);}
        double floor=a.name=="saddle"?cfg.saddle_reserve_weight_fraction*weight:0.;
        if(a.name!="saddle"&&(a.equality_id>=0||(predicted>0.&&!recovering)))floor=cfg.pedal_min_normal_n;
        Vector g(nz);for(std::size_t j=0;j<2;++j)g[f+j]=-normal[j];append(g,-floor);
        if(a.equality_id<0){for(double&v:g)v=-v;append(g,predicted+.005*cfg.support_k_n_m);}
    }
    for(std::size_t i=0;i<nt;++i){Vector g(nz);g[it+i]=speed[i];g[ip+i]=-1.;append(g,0.);}
    if(cfg.active_positive_power_limit_w){Vector g(nz);std::fill(g.begin()+static_cast<std::ptrdiff_t>(ip),g.begin()+static_cast<std::ptrdiff_t>(ip+nt),1.);append(g,*cfg.active_positive_power_limit_w);}
    AllocationProblem problem;problem.equality=d::zeros(nr,nz);problem.equality_rhs=dynamics.second;problem.inequality=std::move(inequalities);problem.inequality_rhs=std::move(rhs);
    problem.lower.assign(nz,-inf);problem.upper.assign(nz,inf);problem.target.assign(nz,0.);Vector scales(nz,50.);
    for(std::size_t j=0;j<nr;++j)scales[j]=500.;for(std::size_t j=iforce;j<ip;++j)scales[j]=300.;for(std::size_t j=ip;j<base;++j)scales[j]=450.;
    for(std::size_t i=0;i<nr;++i)for(std::size_t j=0;j<dynamics.first.cols;++j)problem.equality.values[i*nz+j]=dynamics.first.values[i*dynamics.first.cols+j];
    for(std::size_t i=0;i<nt;++i){problem.lower[it+i]=request.lower_torque_nm[i];problem.upper[it+i]=request.upper_torque_nm[i];problem.lower[ip+i]=0.;problem.upper[ip+i]=cfg.active_positive_power_limit_w.value_or(inf);problem.target[it+i]=request.torque_intent[i];}
    for(std::size_t k=0;k<attachments_.size();++k){const auto& name=attachments_[k].name;const std::size_t index=name=="saddle"?0:name=="front"?1:name=="rear"?2:3;const double fraction=index==3?.5:1.;problem.target[iforce+2*k]=-fraction*support.forces_on_bike[index][0];problem.target[iforce+2*k+1]=-fraction*support.forces_on_bike[index][2];}
    std::vector<std::size_t> torque_indices(nt),excitation_indices;std::iota(torque_indices.begin(),torque_indices.end(),it);std::optional<ActivationLaw> activation;
    if(activated) {
        excitation_indices.resize(nt);std::iota(excitation_indices.begin(),excitation_indices.end(),base);
        activation.emplace(request.activation_state,speed,request.lower_torque_nm,request.upper_torque_nm,request.dt_s,cfg.activation_tau_s,cfg.active_positive_power_limit_w);Vector excitation(nt);
        for(std::size_t i=0;i<nt;++i){problem.lower[base+i]=request.lower_torque_nm[i];problem.upper[base+i]=request.upper_torque_nm[i];const double gain=activation->gain();excitation[i]=std::clamp(gain>0.?(request.torque_intent[i]-(1.-gain)*request.activation_state[i])/gain:request.activation_state[i],request.lower_torque_nm[i],request.upper_torque_nm[i]);problem.target[base+i]=excitation[i];}
        const auto delivered=activation->delivered(excitation);for(std::size_t i=0;i<nt;++i)problem.target[it+i]=delivered[i];
    }
    for(std::size_t j=0;j<nz;++j){problem.lower[j]/=scales[j];problem.upper[j]/=scales[j];problem.target[j]/=scales[j];for(std::size_t i=0;i<nr;++i)problem.equality.values[i*nz+j]*=scales[j];for(std::size_t i=0;i<problem.inequality.rows;++i)problem.inequality.values[i*nz+j]*=scales[j];}
    for(std::size_t i=0;i<nr;++i){auto row=std::span<double>(problem.equality.values).subspan(i*nz,nz);const double norm=std::max(d::norm(row),1e-9);for(double&v:row)v/=norm;problem.equality_rhs[i]/=norm;}
    auto staged_memory=c_.allocation_memory_;
    const auto solved=select_allocation_branch(problem.target,[&](const std::array<bool,2>& branch,const std::optional<Vector>& initial) {
        auto candidate=problem;candidate.initial=initial;
        if(activation)candidate.extra.push_back(activation->constraint(torque_indices,excitation_indices,scales));
        for(std::size_t k=0;k<attachments_.size();++k)if(attachments_[k].name.starts_with("grip")) {
            const auto& a=attachments_[k];const std::size_t side=a.name=="grip_left"?0:1;
            const auto origin=allocation_support_point(m,data,a),pelvis=row3(data.xpos,c_.pelvis_body_);std::array<double,2> direction{pelvis[0]-origin[0],pelvis[2]-origin[2]};const double length=std::max(d::norm(direction),1e-9);for(double&v:direction)v/=length;
            auto force=d::zeros(2,nz);force.values[iforce+2*k]=300.;force.values[nz+iforce+2*k+1]=300.;auto extras=allocation_grip_constraints(force,direction,branch[side],cfg.grip_pull_per_hand_n);candidate.extra.insert(candidate.extra.end(),extras.begin(),extras.end());
        }
        auto result=allocate(candidate);if(activation){result.solution=activation->project(result.solution,torque_indices,excitation_indices,scales);result.violation=allocation_residual(candidate,result.solution);result.feasible=result.violation<=1e-7;}return result;
    },staged_memory);
    ResponseAllocation out;out.allocation=solved.allocation;out.branch=solved.branch;out.saddle_normal_lower_bound_n=cfg.saddle_reserve_weight_fraction*weight;out.lower_torque_nm=request.lower_torque_nm;out.upper_torque_nm=request.upper_torque_nm;
    auto z=solved.allocation.solution;for(std::size_t i=0;i<nz;++i)z[i]*=scales[i];
    out.torque.assign(z.begin()+static_cast<std::ptrdiff_t>(it),z.begin()+static_cast<std::ptrdiff_t>(it+nt));out.wrenches.assign(z.begin()+static_cast<std::ptrdiff_t>(iforce),z.begin()+static_cast<std::ptrdiff_t>(ip));out.power.assign(z.begin()+static_cast<std::ptrdiff_t>(ip),z.begin()+static_cast<std::ptrdiff_t>(base));
    out.excitation=activated?Vector(z.begin()+static_cast<std::ptrdiff_t>(base),z.end()):out.torque;out.activation_state=activation?activation->latent(out.excitation):out.torque;out.acceleration.assign(nv,0.);
    for(std::size_t i=0;i<nr;++i)out.acceleration[static_cast<std::size_t>(rider_dofs_[i])]=z[i];for(std::size_t i=0;i<bike_dofs_.size();++i)out.acceleration[static_cast<std::size_t>(bike_dofs_[i])]=qbase[i];
    out.effort_scale=request.effort_scale;out.rider_acceleration.assign(z.begin(),z.begin()+static_cast<std::ptrdiff_t>(nr));out.base_acceleration=qbase;
    c_.allocation_memory_=std::move(staged_memory);return out;
}
} // namespace bike::rider
