// Source: sim/ride/rider_dynamics.py. The dual active-set law and all
// domain signs are transcribed; LU arithmetic has a new numeric identity.
#include "constraint_dynamics.hpp"
#include "../numerics/dense.hpp"
#include "../engine_call.hpp"
#include "../model_access.hpp"
#include "least_squares.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace bike::rider {
namespace {
namespace d=bike::numerics::dense;
ConstraintSystem snapshot(const mjModel& model,const mjData& data,std::span<const int> dofs) {
    if(model.nv<=0 || data.nefc<0) throw std::invalid_argument("constraint dynamics: invalid dimensions");
    const auto nv=static_cast<std::size_t>(model.nv),n=static_cast<std::size_t>(data.nefc);
    ConstraintSystem s;
    s.mass=d::zeros(nv,nv);s.jacobian=d::zeros(n,nv);s.actuation=d::zeros(nv,dofs.size());
    std::vector<double> basis(nv,0.),column(nv),jac_column(n);
    for(std::size_t j=0;j<nv;++j) {
        basis[j]=1.;engine::mul_m(&model,&data,column.data(),basis.data());
        for(std::size_t i=0;i<nv;++i) s.mass.values[i*nv+j]=column[i];
        if(n) {
            engine::mul_jac_vec(&model,&data,jac_column.data(),basis.data());
            for(std::size_t i=0;i<n;++i) s.jacobian.values[i*nv+j]=jac_column[i];
        }
        basis[j]=0.;
    }
    const auto qfrc_applied = model_access::readonly_buffer(data.qfrc_applied, model.nv, "qfrc_applied");
    const auto qfrc_passive = model_access::readonly_buffer(data.qfrc_passive, model.nv, "qfrc_passive");
    const auto qfrc_actuator = model_access::readonly_buffer(data.qfrc_actuator, model.nv, "qfrc_actuator");
    const auto qfrc_bias = model_access::readonly_buffer(data.qfrc_bias, model.nv, "qfrc_bias");
    s.known_force.resize(nv);
    for(std::size_t i=0;i<nv;++i)
        s.known_force[i]=qfrc_applied[i]+qfrc_passive[i]+qfrc_actuator[i]-qfrc_bias[i];
    std::vector<bool> seen(nv,false);
    for(std::size_t j=0;j<dofs.size();++j) {
        if(dofs[j]<0 || std::cmp_greater_equal(dofs[j],nv) || seen[static_cast<std::size_t>(dofs[j])])
            throw std::invalid_argument("constraint dynamics: invalid/duplicate actuator dof");
        const auto i=static_cast<std::size_t>(dofs[j]);seen[i]=true;
        s.known_force[i]-=qfrc_actuator[i];s.actuation.values[i*dofs.size()+j]=1.;
    }
    const auto efc_type = model_access::readonly_buffer(data.efc_type, data.nefc, "efc_type");
    const auto efc_id = model_access::readonly_buffer(data.efc_id, data.nefc, "efc_id");
    const auto efc_r = model_access::readonly_buffer(data.efc_R, data.nefc, "efc_R");
    const auto efc_aref = model_access::readonly_buffer(data.efc_aref, data.nefc, "efc_aref");
    const auto efc_frictionloss = model_access::readonly_buffer(data.efc_frictionloss, data.nefc, "efc_frictionloss");
    for(std::size_t i=0;i<n;++i) {
        s.types.push_back(efc_type[i]);s.ids.push_back(efc_id[i]);
        s.resistance.push_back(efc_r[i]);s.reference_acceleration.push_back(efc_aref[i]);s.frictionloss.push_back(efc_frictionloss[i]);
    }
    return s;
}
void validate_mode(const ConstraintDynamics& self,const ConstraintMode& mode) {
    std::size_t previous=0;bool first=true;
    for(const auto& [i,value]:mode) {
        if(i>=self.lower.size() || !std::isfinite(value) || (!first && i<=previous) || (value!=self.lower[i] && value!=self.upper[i]))
            throw std::invalid_argument("constraint mode: fixed rows must be unique sorted finite bounds");
        previous=i;first=false;
    }
}
std::vector<std::size_t> free_rows(std::size_t count,const ConstraintMode& mode) {
    std::vector<std::size_t> out;
    for(std::size_t i=0;i<count;++i)
        if(std::ranges::none_of(mode,[&](const auto& pair){return pair.first==i;})) out.push_back(i);
    return out;
}
Matrix selected(const Matrix& a,std::span<const std::size_t> rows,std::span<const std::size_t> cols) {
    auto out=d::zeros(rows.size(),cols.size());
    for(std::size_t i=0;i<rows.size();++i) for(std::size_t j=0;j<cols.size();++j)
        out.values[i*cols.size()+j]=a.values[rows[i]*a.cols+cols[j]];
    return out;
}
}

ConstraintDynamics::ConstraintDynamics(const mjModel& model,const mjData& data,std::span<const int> dofs)
    :ConstraintDynamics(snapshot(model,data,dofs)) {}

ConstraintDynamics::ConstraintDynamics(ConstraintSystem s) {
    d::validate(s.mass);d::validate(s.jacobian);d::validate(s.actuation);
    const auto nv=s.mass.rows,n=s.jacobian.rows,nt=s.actuation.cols;
    if(s.mass.cols!=nv || s.jacobian.cols!=nv || s.actuation.rows!=nv || s.known_force.size()!=nv || s.resistance.size()!=n || s.reference_acceleration.size()!=n || s.frictionloss.size()!=n || s.types.size()!=n || s.ids.size()!=n || !d::finite(s.known_force) || !d::finite(s.resistance) || !d::finite(s.reference_acceleration) || !d::finite(s.frictionloss))
        throw std::invalid_argument("constraint dynamics: inconsistent model snapshot");
    auto rhs=d::zeros(nv,1+nt+n);
    for(std::size_t i=0;i<nv;++i) {
        rhs.values[i*rhs.cols]=s.known_force[i];
        for(std::size_t j=0;j<nt;++j) rhs.values[i*rhs.cols+1+j]=s.actuation.values[i*nt+j];
        for(std::size_t j=0;j<n;++j) rhs.values[i*rhs.cols+1+nt+j]=s.jacobian.values[j*nv+i];
    }
    const auto inverse=d::solve(s.mass,rhs);
    smooth_offset.resize(nv);smooth_matrix=d::zeros(nv,nt);inverse_jacobian=d::zeros(nv,n);
    for(std::size_t i=0;i<nv;++i) {
        smooth_offset[i]=inverse.values[i*rhs.cols];
        for(std::size_t j=0;j<nt;++j) smooth_matrix.values[i*nt+j]=inverse.values[i*rhs.cols+1+j];
        for(std::size_t j=0;j<n;++j) inverse_jacobian.values[i*n+j]=inverse.values[i*rhs.cols+1+nt+j];
    }
    jacobian=std::move(s.jacobian);types=std::move(s.types);ids=std::move(s.ids);
    matrix=d::multiply(jacobian,inverse_jacobian);
    for(std::size_t i=0;i<n;++i) {
        if(s.resistance[i]<0. || s.frictionloss[i]<0.) throw std::invalid_argument("constraint dynamics: negative impedance/friction loss");
        matrix.values[i*n+i]+=s.resistance[i];
    }
    rhs_offset=d::add(std::move(s.reference_acceleration),d::multiply(jacobian,smooth_offset),-1.);
    rhs_matrix=d::multiply(jacobian,smooth_matrix);for(double& v:rhs_matrix.values)v=-v;
    lower.assign(n,-std::numeric_limits<double>::infinity());upper.assign(n,std::numeric_limits<double>::infinity());
    for(std::size_t i=0;i<n;++i) {
        if(types[i]==mjCNSTR_EQUALITY) continue;
        if(types[i]==mjCNSTR_LIMIT_JOINT || types[i]==mjCNSTR_LIMIT_TENDON || types[i]==mjCNSTR_CONTACT_FRICTIONLESS || types[i]==mjCNSTR_CONTACT_PYRAMIDAL) lower[i]=0.;
        else if(types[i]==mjCNSTR_FRICTION_DOF || types[i]==mjCNSTR_FRICTION_TENDON) {lower[i]=-s.frictionloss[i];upper[i]=s.frictionloss[i];}
        else throw std::invalid_argument("model response requires scalar constraint rows; elliptic cones need a coupled model");
    }
}

ConstraintResponse ConstraintDynamics::linearize(std::span<const double> torque) const {
    if(torque.size()!=rhs_matrix.cols || !d::finite(torque)) throw std::invalid_argument("constraint dynamics torque shape/nonfinite value");
    const auto rhs=d::add(rhs_offset,d::multiply(rhs_matrix,torque));
    const auto n=rhs.size();ConstraintMode fixed;
    for(std::size_t iteration=0;iteration<4*n+1;++iteration) {
        const auto free=free_rows(n,fixed);
        std::vector<double> force(n,0.);for(const auto& [i,value]:fixed)force[i]=value;
        if(!free.empty()) {
            auto target=d::add(rhs,d::multiply(matrix,force),-1.);std::vector<double> b;
            b.reserve(free.size());
for(auto const i:free)b.push_back(target[i]);
            const auto solved=d::solve(selected(matrix,free,free),b);
            for(std::size_t j=0;j<free.size();++j)force[free[j]]=solved[j];
        }
        std::size_t worst=n;double violation=1e-9;
        for(auto const i:free) {
            const double error=std::max(lower[i]-force[i],force[i]-upper[i]);
            if(error>violation){violation=error;worst=i;}
        }
        if(worst<n) {
            fixed.emplace_back(worst,force[worst]<lower[worst]?lower[worst]:upper[worst]);
            std::ranges::sort(fixed);continue;
        }
        const auto gradient=d::add(d::multiply(matrix,force),rhs,-1.);
        std::size_t release=fixed.size();double maximum=-1.;
        for(std::size_t j=0;j<fixed.size();++j) {
            const auto [i,value]=fixed[j];
            if(((value==lower[i] && gradient[i]<-1e-9)||(value==upper[i]&&gradient[i]>1e-9)) && std::abs(gradient[i])>maximum) {
                maximum=std::abs(gradient[i]);release=j;
            }
        }
        if(release<fixed.size()){fixed.erase(fixed.begin()+static_cast<std::ptrdiff_t>(release));continue;}
        return response_for_mode(fixed);
    }
    throw std::runtime_error("constraint model active set did not converge");
}

ConstraintResponse ConstraintDynamics::response_for_mode(const ConstraintMode& mode) const {
    validate_mode(*this,mode);
    const auto n=rhs_offset.size(),nt=rhs_matrix.cols;
    const auto free=free_rows(n,mode);
    std::vector<double> offset(n,0.);auto derivative=d::zeros(n,nt);
    for(const auto& [i,value]:mode)offset[i]=value;
    if(!free.empty()) {
        const auto remaining=d::add(rhs_offset,d::multiply(matrix,offset),-1.);
        auto b=d::zeros(free.size(),1+nt);
        for(std::size_t i=0;i<free.size();++i) {
            b.values[i*b.cols]=remaining[free[i]];
            for(std::size_t j=0;j<nt;++j)b.values[i*b.cols+1+j]=rhs_matrix.values[free[i]*nt+j];
        }
        const auto solved=d::solve(selected(matrix,free,free),b);
        for(std::size_t i=0;i<free.size();++i) {
            offset[free[i]]=solved.values[i*b.cols];
            for(std::size_t j=0;j<nt;++j)derivative.values[free[i]*nt+j]=solved.values[i*b.cols+1+j];
        }
    }
    auto rows=d::zeros(0,nt);std::vector<double> bounds;
    for(auto const i:free) {
        std::vector<double> row(derivative.values.begin()+static_cast<std::ptrdiff_t>(i*nt),derivative.values.begin()+static_cast<std::ptrdiff_t>((i+1)*nt));
        if(std::isfinite(lower[i])) {for(double&v:row)v=-v;d::append_row(rows,row);bounds.push_back(offset[i]-lower[i]);for(double&v:row)v=-v;}
        if(std::isfinite(upper[i])) {d::append_row(rows,row);bounds.push_back(upper[i]-offset[i]);}
    }
    const auto gradient_offset=d::add(d::multiply(matrix,offset),rhs_offset,-1.);
    const auto gradient_matrix=d::add(d::multiply(matrix,derivative),rhs_matrix,-1.);
    for(const auto& [i,value]:mode) {
        const double sign=value==lower[i]?-1.:1.;std::vector<double> row(nt);
        for(std::size_t j=0;j<nt;++j)row[j]=sign*gradient_matrix.values[i*nt+j];
        d::append_row(rows,row);bounds.push_back(-sign*gradient_offset[i]);
    }
    return {.acceleration_offset=d::add(smooth_offset,d::multiply(inverse_jacobian,offset)),
        .acceleration_matrix=d::add(smooth_matrix,d::multiply(inverse_jacobian,derivative)),.force_offset=std::move(offset),.force_matrix=std::move(derivative),.domain_matrix=std::move(rows),.domain_bound=std::move(bounds),.mode=mode};
}

std::vector<ConstraintMode> ConstraintDynamics::neighboring_modes(const ConstraintMode& mode) const {
    validate_mode(*this,mode);std::vector<ConstraintMode> out;
    for(std::size_t i=0;i<lower.size();++i) {
        const auto previous=std::ranges::find_if(mode,[&](const auto& entry){return entry.first==i;});
        if(previous!=mode.end()) {auto next=mode;std::erase_if(next,[&](const auto& item){return item.first==i;});out.push_back(std::move(next));}
        for(double value:{lower[i],upper[i]}) if(std::isfinite(value) && (previous==mode.end() || previous->second!=value)) {
            auto next=mode;std::erase_if(next,[&](const auto& item){return item.first==i;});next.emplace_back(i,value);std::ranges::sort(next);
            if(std::ranges::find(out,next)==out.end())out.push_back(std::move(next));
        }
    }
    return out;
}

Matrix attachment_force_map(const mjModel& model,const mjData& data,const ConstraintDynamics& dynamics,
    int equality_id,int body,const std::array<double,3>& point) {
    if(body<0 || body>=model.nbody || equality_id<0 || equality_id>=model.neq || !d::finite(point))
        throw std::invalid_argument("attachment force map: invalid body/equality/point");
    const auto nv=static_cast<std::size_t>(model.nv),nc=dynamics.jacobian.rows;
    if(dynamics.jacobian.cols!=nv || dynamics.types.size()!=nc || dynamics.ids.size()!=nc)
        throw std::invalid_argument("attachment force map: incompatible constraint snapshot");
    auto spatial=d::zeros(6,nv);
    struct Context {const mjModel* m;const mjData* data;double* p;double* r;const double* point;int body;};
    Context context{.m=&model,.data=&data,.p=spatial.values.data(),.r=std::span{spatial.values}.subspan(3*nv).data(),.point=point.data(),.body=body};
    engine::ErrorBuffer error;
    if(!engine::invoke([](void* raw) noexcept {auto const& c=*static_cast<Context*>(raw);mj_jac(c.m,c.data,c.p,c.r,c.point,c.body);},&context,error))engine::throw_failure(error);
    std::vector<std::size_t> columns;
    for(std::size_t j=0;j<nv;++j) for(std::size_t i=0;i<6;++i) if(spatial.values[i*nv+j]!=0.) {columns.push_back(j);break;}
    const std::array<std::size_t,3> planar{0,2,4};
    const auto planar_jac=selected(spatial,planar,columns);
    const auto singular=d::least_squares(planar_jac,d::zeros(3,1),1e-12).singular_values;
    // numpy.matrix_rank(..., tol=1e-12) has an absolute singular-value
    // cutoff; it is different from lstsq's relative rcond below.
    if(std::ranges::count_if(singular,[](double value){return value>1e-12;})!=3)
        throw std::invalid_argument("model attachment planar wrench is not observable");
    auto a=d::zeros(columns.size(),6),b=d::zeros(columns.size(),nc);
    for(std::size_t i=0;i<columns.size();++i) {
        for(std::size_t j=0;j<6;++j)a.values[i*6+j]=spatial.values[j*nv+columns[i]];
        for(std::size_t j=0;j<nc;++j) if(dynamics.types[j]==mjCNSTR_EQUALITY && dynamics.ids[j]==equality_id)
            b.values[i*nc+j]=dynamics.jacobian.values[j*nv+columns[i]];
    }
    const auto mapping=d::least_squares(a,b,1e-12).solution;
    const auto explained=d::multiply(a,mapping);
    for(std::size_t i=0;i<b.values.size();++i)
        if(std::abs(explained.values[i]-b.values[i])>1e-8+1e-8*std::abs(b.values[i]))
            throw std::invalid_argument("model attachment wrench does not explain its constraint rows");
    auto out=d::zeros(3,nc);
    for(std::size_t i=0;i<3;++i)for(std::size_t j=0;j<nc;++j)out.values[i*nc+j]=mapping.values[planar[i]*nc+j];
    return out;
}
} // namespace bike::rider
