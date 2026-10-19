// Source: sim/ride/rider_response_allocation.py, including guards, task
// coordinate, activation projection and complete/incomplete mode evidence.
#include "response_allocation.hpp"
#include "activation_allocation.hpp"
#include "support_geometry.hpp"
#include "spindle_math.hpp"
#include "../diag.hpp"
#include "../model_access.hpp"
#include "../numerics/dense.hpp"
#include "../numerics/linear_feasibility.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace bike::rider {
namespace {
namespace d=bike::numerics::dense;
constexpr double inf=std::numeric_limits<double>::infinity();
using Vector=std::vector<double>;
std::array<double,3> row3(const double* values,int id,int maximum) {
    if(id<0 || id>=maximum)throw std::invalid_argument("allocation model row id");
    const auto span=model_access::readonly_buffer(values,static_cast<mjtSize>(maximum)*3,"allocation row3");
    const auto sub=span.subspan(static_cast<std::size_t>(id)*3,3);
    return {sub[0],sub[1],sub[2]};
}
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wlarge-by-value-copy")
std::array<double,9> row9(const double* values,int id,int maximum) {
    if(id<0 || id>=maximum)throw std::invalid_argument("allocation model rotation id");
    const auto span=model_access::readonly_buffer(values,static_cast<mjtSize>(maximum)*9,"allocation row9");
    std::array<double,9> out{};std::ranges::copy(span.subspan(static_cast<std::size_t>(id)*9,9),out.begin());return out;
}
NATIVE_DIAG_POP
std::array<double,3> rotated(const std::array<double,9>& r,const std::array<double,3>& x) {
    return {r[0]*x[0]+r[1]*x[1]+r[2]*x[2],r[3]*x[0]+r[4]*x[1]+r[5]*x[2],r[6]*x[0]+r[7]*x[1]+r[8]*x[2]};
}
}

BranchResult select_allocation_branch(std::span<const double> target,const BranchSolver& solve,AllocationMemory& memory) {
    if(!solve || !d::finite(target))throw std::invalid_argument("allocation branch solver/target");
    std::vector<std::array<bool,2>> branches{{true,true},{true,false},{false,true},{false,false}};
    if(memory.branch) {
        std::erase(branches,*memory.branch);branches.insert(branches.begin(),*memory.branch);
    }
    auto start=memory.solution;
    if(start && start->size()!=target.size())start.reset();
    BranchResult best;bool have=false;double best_error=inf;
    for(const auto& branch:branches) {
        auto result=solve(branch,start);
        if(result.solution.size()!=target.size())throw std::invalid_argument("allocation branch result shape");
        const auto delta=d::add(result.solution,target,-1.);const double error=d::dot(delta,delta);
        if(!have || (result.feasible && !best.allocation.feasible) || (result.feasible && best.allocation.feasible && error<best_error) || (!result.feasible && !best.allocation.feasible && result.violation<best.allocation.violation)) {
            best={.allocation=std::move(result),.branch=branch};best_error=error;have=true;
        }
        if(best.allocation.feasible && memory.branch && branch==*memory.branch)break;
    }
    if(best.allocation.feasible){memory.branch=best.branch;memory.solution=best.allocation.solution;}
    return best;
}

std::array<double,3> allocation_support_point(const mjModel& model,const mjData& data,const AllocationAttachment& a) {
    if(a.name.starts_with("grip")) {
        if(a.equality_id<0 || a.equality_id>=model.neq)throw std::invalid_argument("allocation grip equality id");
        auto point=row3(data.xpos,a.other_body,static_cast<int>(model.nbody));
        const auto rotation=row9(data.xmat,a.other_body,static_cast<int>(model.nbody));
        const auto eq_data=model_access::readonly_buffer(model.eq_data,model.neq*mjNEQDATA,"eq_data");
        const auto base=static_cast<std::size_t>(a.equality_id)*mjNEQDATA+3;
        const auto offset=rotated(rotation,{eq_data[base],eq_data[base+1],eq_data[base+2]});
        for(std::size_t j=0;j<3;++j)point[j]+=offset[j];
        return point;
    }
    return row3(data.site_xpos,a.site_id,static_cast<int>(model.nsite));
}
std::vector<ExtraConstraint> allocation_grip_constraints(const Matrix& f,const std::array<double,2>& direction,bool pulling,double limit) {
    auto out=grip_constraints(f,direction,pulling,pulling?limit*(1.-grip_radius_guard_fraction):limit);
    if(!pulling)out[0].lower={support_force_guard_n};
    return out;
}
std::pair<Matrix,Vector> support_rows(const Matrix& force,const std::array<double,2>& normal,
    bool saddle,const bike::configuration::ArticulatedConfig& config,double weight,double half) {
    d::validate(force);if(force.rows!=3 || !d::finite(normal) || !std::isfinite(weight) || weight<0. || !std::isfinite(half) || half<0.)throw std::invalid_argument("support wrench map/domain");
    const auto n=force.cols;Vector fn(n),ft(n);
    for(std::size_t j=0;j<n;++j) {
        fn[j]=normal[0]*force.values[j]+normal[1]*force.values[n+j];
        ft[j]=-normal[1]*force.values[j]+normal[0]*force.values[n+j];
    }
    const double mu=saddle?config.saddle_mu:config.foot_mu;
    const double floor=saddle?config.saddle_reserve_weight_fraction*weight:config.pedal_min_normal_n;
    auto g=d::zeros(0,n);Vector h;
    for(double const sign:{1.,-1.}) {
        Vector row(n);for(std::size_t j=0;j<n;++j)row[j]=sign*ft[j]-mu*fn[j];
        d::append_row(g,row);h.push_back(-support_force_guard_n);
    }
    Vector row(n);for(std::size_t j=0;j<n;++j)row[j]=-fn[j];d::append_row(g,row);h.push_back(-floor-support_force_guard_n);
    if(half>0.)for(double const sign:{1.,-1.}) {
        for(std::size_t j=0;j<n;++j)row[j]=sign*force.values[2*n+j]-half*fn[j];
        d::append_row(g,row);h.push_back(-support_moment_guard_nm);
    }
    return {std::move(g),std::move(h)};
}

ResponseAllocation allocate_response(const mjModel& model,const mjData& data,const ResponseRequest& request,AllocationMemory& memory) {
    const auto& cfg=request.config;const auto nt=request.actuator_dofs.size();
    if(nt==0 || request.torque_intent.size()!=nt || request.lower_torque_nm.size()!=nt || request.upper_torque_nm.size()!=nt || request.activation_state.size()!=nt || !d::finite(request.torque_intent) || !d::finite(request.lower_torque_nm) || !d::finite(request.upper_torque_nm) || !d::finite(request.activation_state) || !std::isfinite(request.rider_mass_kg) || request.rider_mass_kg<0. || !std::isfinite(request.dt_s) || request.dt_s<0. || (request.crank_torque_nm && !std::isfinite(*request.crank_torque_nm)))throw std::invalid_argument("response allocation request");
    const bool activated=cfg.activation_tau_s>0. && !request.steady_state;
    for(std::size_t i=0;i<nt;++i)
        if(request.lower_torque_nm[i]>request.upper_torque_nm[i])
            throw std::invalid_argument("empty rider actuator bounds");
    const auto constant=2*nt,base=constant+1;
    std::size_t nz=base+(activated?nt:0);std::optional<std::size_t> task;
    if(request.crank_torque_nm){task=nz;++nz;}
    const auto qvel=model_access::readonly_buffer(data.qvel,model.nv,"qvel");
    Vector speed(nt),lower(nz,-inf),upper(nz,inf),target(nz,0.),scales(nz,1.);
    for(std::size_t i=0;i<nt;++i) {
        const auto dof=request.actuator_dofs[i];if(dof<0 || dof>=model.nv)throw std::invalid_argument("response allocation actuator dof");
        speed[i]=qvel[static_cast<std::size_t>(dof)];lower[i]=request.lower_torque_nm[i]/50.;upper[i]=request.upper_torque_nm[i]/50.;
        lower[nt+i]=0.;upper[nt+i]=cfg.active_positive_power_limit_w?*cfg.active_positive_power_limit_w/450.:inf;
        target[i]=request.torque_intent[i]/50.;scales[i]=50.;
    }
    lower[constant]=upper[constant]=target[constant]=1.;
    // NOLINTNEXTLINE(bugprone-unchecked-optional-access) a crank task is only emitted when crank torque was requested
    if(task)target[task.value()]=request.crank_torque_nm.value()/crank_task_scale_nm;
    std::optional<ActivationLaw> activation;std::vector<std::size_t> ti(nt),ei;
    std::ranges::iota(ti,0U);
    if(activated) {
        ei.resize(nt);std::ranges::iota(ei,base);
        activation.emplace(request.activation_state,speed,request.lower_torque_nm,request.upper_torque_nm,request.dt_s,cfg.activation_tau_s,cfg.active_positive_power_limit_w);
        Vector excitation(nt);
        for(std::size_t i=0;i<nt;++i) {
            lower[ei[i]]=lower[i];upper[ei[i]]=upper[i];scales[ei[i]]=50.;
            const double gain=activation->gain();
            // dt=0 is a constant activation law; excitation is immaterial
            // but remains finite/in-envelope rather than dividing by zero.
            const double desired=gain==0.?request.activation_state[i]:(request.torque_intent[i]-(1.-gain)*request.activation_state[i])/gain;
            excitation[i]=std::clamp(desired,request.lower_torque_nm[i],request.upper_torque_nm[i]);target[ei[i]]=excitation[i]/50.;
        }
        const auto delivered=activation->delivered(excitation);for(std::size_t i=0;i<nt;++i)target[i]=delivered[i]/50.;
    }
    for(std::size_t i=0;i<nz;++i)target[i]=std::clamp(target[i],lower[i],upper[i]);
    ConstraintDynamics kernel(model,data,request.actuator_dofs);
    Vector fallback=target;
    if(!activation && cfg.active_positive_power_limit_w) {
        Vector torque(nt);for(std::size_t i=0;i<nt;++i)torque[i]=fallback[i]*50.;
        torque=::spindle::limit_positive_power(torque,speed,*cfg.active_positive_power_limit_w);
        for(std::size_t i=0;i<nt;++i)fallback[i]=torque[i]/50.;
    }
    for(std::size_t i=0;i<nt;++i)fallback[nt+i]=std::max(fallback[i]*50.*speed[i],0.)/450.;
    Vector crank_weights(kernel.jacobian.rows,0.);
    if(task) {
        if(request.crank_dof<0 || request.crank_dof>=model.nv)throw std::invalid_argument("crank torque allocation requires crank dof");
        for(auto name:{"front","rear"}) {
            const auto attachment=std::ranges::find_if(request.attachments,[&](const auto& a){return a.name==name;});
            if(attachment==request.attachments.end() || attachment->equality_id<0)throw std::invalid_argument("crank torque allocation requires welded pedals");
            for(std::size_t i=0;i<kernel.types.size();++i)if(kernel.types[i]==mjCNSTR_EQUALITY&&kernel.ids[i]==attachment->equality_id)
                crank_weights[i]=kernel.jacobian.values[i*kernel.jacobian.cols+static_cast<std::size_t>(request.crank_dof)];
        }
    }
    const std::array<double,3> gravity{model.opt.gravity[0],model.opt.gravity[1],model.opt.gravity[2]};
    const double weight=request.rider_mass_kg*d::norm(gravity);
    std::vector<Matrix> projected;std::vector<std::array<double,3>> points;
    std::vector<std::array<double,2>> normals;Vector half_lengths;
    for(const auto& a:request.attachments) {
        const auto point=allocation_support_point(model,data,a);points.push_back(point);
        projected.push_back(attachment_force_map(model,data,kernel,a.equality_id,a.rider_body,point));
        std::array<double,2> normal{};double half=0.;
        if(!a.name.starts_with("grip")) {
            const auto geom_size=row3(model.geom_size,a.geom_id,static_cast<int>(model.ngeom));
            const auto face=::rider::upper_box_face(row3(data.geom_xpos,a.geom_id,static_cast<int>(model.ngeom)),row9(data.geom_xmat,a.geom_id,static_cast<int>(model.ngeom)),geom_size);
            normal={face.normal[0],face.normal[2]};half=a.name=="saddle"?cfg.saddle_patch_half_length_m:geom_size[0];
        }
        normals.push_back(normal);half_lengths.push_back(half);
    }
    struct Payload {std::array<bool,2> branch{};std::vector<Matrix> forces;};std::vector<Payload> payloads;
    std::size_t pruned=0;auto staged_memory=memory;
    const auto mode_solver=[&](const ConstraintResponse& response)->ModeCandidate {
        Payload payload;
        for(const auto& projection:projected) {
            const auto force_matrix=d::multiply(projection,response.force_matrix);const auto offset=d::multiply(projection,response.force_offset);
            auto force=d::zeros(3,nz);
            for(std::size_t i=0;i<3;++i) {
                for(std::size_t j=0;j<nt;++j)force.values[i*nz+j]=force_matrix.values[i*nt+j]*50.;
                force.values[i*nz+constant]=offset[i];
            }
            payload.forces.push_back(std::move(force));
        }
        AllocationProblem p;p.target=target;p.lower=lower;p.upper=upper;p.equality=d::zeros(0,nz);p.inequality=d::zeros(0,nz);
        for(std::size_t i=0;i<response.domain_matrix.rows;++i) {
            Vector row(nz,0.);for(std::size_t j=0;j<nt;++j)row[j]=response.domain_matrix.values[i*nt+j]*50.;
            d::append_row(p.inequality,row);p.inequality_rhs.push_back(response.domain_bound[i]);
        }
        for(std::size_t i=0;i<request.attachments.size();++i)if(!request.attachments[i].name.starts_with("grip")) {
            auto [g,h]=support_rows(payload.forces[i],normals[i],request.attachments[i].name=="saddle",cfg,weight,half_lengths[i]);
            p.inequality.values.insert(p.inequality.values.end(),g.values.begin(),g.values.end());p.inequality.rows+=g.rows;p.inequality_rhs.insert(p.inequality_rhs.end(),h.begin(),h.end());
        }
        for(std::size_t i=0;i<nt;++i) {
            Vector row(nz,0.);row[i]=speed[i]*50.;row[nt+i]=-450.;d::append_row(p.inequality,row);p.inequality_rhs.push_back(0.);
        }
        if(cfg.active_positive_power_limit_w) {
            Vector row(nz,0.);for(std::size_t i=0;i<nt;++i)row[nt+i]=450.;d::append_row(p.inequality,row);p.inequality_rhs.push_back(*cfg.active_positive_power_limit_w);
        }
        if(task) {
            const auto crank=d::multiply(d::transpose(response.force_matrix),crank_weights);
            Vector row(nz,0.);for(std::size_t i=0;i<nt;++i)row[i]=crank[i]*50.;row[constant]=d::dot(crank_weights,response.force_offset);row[*task]=-crank_task_scale_nm;
            d::append_row(p.equality,row);p.equality_rhs.push_back(0.);
            fallback[*task]=d::dot(std::span<const double>(row).first(*task),std::span<const double>(fallback).first(*task))/crank_task_scale_nm;
        }
        const bool base_empty=allocation_residual(p,fallback)>1e-7 && bike::numerics::linear_region_infeasible(p.inequality,p.inequality_rhs,lower,upper,p.equality,p.equality_rhs);
        if(base_empty)++pruned;
        auto branch_result=select_allocation_branch(target,[&](const std::array<bool,2>& branch,const std::optional<Vector>& x0) {
            auto problem=p;problem.initial=x0;
            if(activation)problem.extra.push_back(activation->constraint(ti,ei,scales));
            for(std::size_t k=0;k<request.attachments.size();++k)if(request.attachments[k].name.starts_with("grip")) {
                const auto pelvis=row3(data.xpos,request.pelvis_body,static_cast<int>(model.nbody));std::array<double,2> direction{pelvis[0]-points[k][0],pelvis[2]-points[k][2]};
                const double length=d::norm(direction);if(length==0.)throw std::invalid_argument("allocation grip direction is undefined");for(double&v:direction)v/=length;
                Matrix const force{.rows=2,.cols=nz,.values=Vector(payload.forces[k].values.begin(),payload.forces[k].values.begin()+static_cast<std::ptrdiff_t>(2*nz))};
                auto extras=allocation_grip_constraints(force,direction,branch[request.attachments[k].name=="grip_left"?0:1],cfg.grip_pull_per_hand_n);
                problem.extra.insert(problem.extra.end(),extras.begin(),extras.end());
            }
            if(base_empty)return Allocation{.solution=fallback,.feasible=false,.violation=allocation_residual(problem,fallback)};
            auto branch_g=problem.inequality;auto branch_h=problem.inequality_rhs;bool has_linear=false;
            for(const auto& e:problem.extra)if(e.linear) {
                has_linear=true;for(std::size_t i=0;i<e.linear->rows;++i) {
                    Vector row(nz);for(std::size_t j=0;j<nz;++j)row[j]=-e.linear->values[i*nz+j];
                    d::append_row(branch_g,row);branch_h.push_back(-e.lower[e.lower.size()==1?0:i]);
                }
            }
            if(has_linear) {
                auto relaxation=problem;relaxation.extra.clear();relaxation.inequality=branch_g;relaxation.inequality_rhs=branch_h;
                if(allocation_residual(relaxation,fallback)>1e-7 && bike::numerics::linear_region_infeasible(branch_g,branch_h,lower,upper,problem.equality,problem.equality_rhs)) {
                    ++pruned;return Allocation{.solution=fallback,.feasible=false,.violation=allocation_residual(problem,fallback)};
                }
            }
            auto result=allocate(problem);
            if(activation){result.solution=activation->project(result.solution,ti,ei,scales);result.violation=allocation_residual(problem,result.solution);result.feasible=result.violation<=1e-7;}
            return result;
        },staged_memory);
        payload.branch=branch_result.branch;payloads.push_back(std::move(payload));
        return {.allocation=std::move(branch_result.allocation),.payload_index=payloads.size()-1};
    };
    Vector wish(nt);for(std::size_t i=0;i<nt;++i)wish[i]=target[i]*50.;
    auto result=search_constraint_modes(kernel,wish,mode_solver,[nt](const Allocation& a) {
        Vector torque(nt);for(std::size_t i=0;i<nt;++i)torque[i]=a.solution[i]*50.;return torque;
    });
    const auto& payload=payloads.at(result.payload_index);
    ResponseAllocation out;out.allocation=result.allocation;out.branch=payload.branch;out.torque.resize(nt);out.power.resize(nt);
    for(std::size_t i=0;i<nt;++i){out.torque[i]=result.allocation.solution[i]*50.;out.power[i]=result.allocation.solution[nt+i]*450.;}
    out.acceleration=d::add(result.response.acceleration_offset,d::multiply(result.response.acceleration_matrix,out.torque));
    out.constraint_forces=d::add(result.response.force_offset,d::multiply(result.response.force_matrix,out.torque));
    for(const auto& force:payload.forces){const auto f=d::multiply(force,result.allocation.solution);out.wrenches.insert(out.wrenches.end(),f.begin(),f.begin()+2);out.moments.push_back(f[2]);}
    out.excitation=out.torque;if(activation)for(std::size_t i=0;i<nt;++i)out.excitation[i]=result.allocation.solution[ei[i]]*50.;
    out.activation_state=activation?activation->latent(out.excitation):out.torque;
    if(task)out.predicted_crank_torque_nm=result.allocation.solution[*task]*crank_task_scale_nm;
    out.modes_tried=result.modes_tried;out.mode_search_complete=result.complete;out.linear_mode_rejections=pruned;
    out.lower_torque_nm=request.lower_torque_nm;out.upper_torque_nm=request.upper_torque_nm;out.saddle_normal_lower_bound_n=cfg.saddle_reserve_weight_fraction*weight;
    out.effort_scale=request.effort_scale;out.crank_torque_target_nm=request.crank_torque_nm;
    for(int const i:request.rider_dofs) {if(i<0||i>=model.nv)throw std::invalid_argument("rider acceleration dof");out.rider_acceleration.push_back(out.acceleration[static_cast<std::size_t>(i)]);}
    for(int const i:request.bike_dofs) {if(i<0||i>=model.nv)throw std::invalid_argument("base acceleration dof");out.base_acceleration.push_back(out.acceleration[static_cast<std::size_t>(i)]);}
    memory=std::move(staged_memory);return out;
}
} // namespace bike::rider
