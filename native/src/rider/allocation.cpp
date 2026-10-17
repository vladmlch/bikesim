// Source: physics/rider_allocation.py. Solver control/identity is separate
// from this source-order projection/residual and physical-constraint seam.
#include "allocation.hpp"
#include "../numerics/dense.hpp"
#include "../numerics/slsqp.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>

namespace bike::rider {
namespace {
namespace d = bike::numerics::dense;
double bound(const std::vector<double>& a, std::size_t i, std::size_t n) {
    if(a.size()!=1 && a.size()!=n) throw std::invalid_argument("allocation constraint bound shape");
    return a[a.size()==1?0:i];
}
void bounds(std::span<const double> lo, std::span<const double> hi) {
    if(lo.size()!=hi.size()) throw std::invalid_argument("allocation bounds shape");
    for(std::size_t i=0;i<lo.size();++i) {
        if(std::isnan(lo[i]) || std::isnan(hi[i])) throw std::invalid_argument("allocation bounds must not be NaN");
        if(lo[i]>hi[i]) throw std::invalid_argument("reversed allocation bounds");
    }
}
std::vector<double> clipped(std::span<const double> x,const AllocationProblem& p) {
    std::vector<double> out(x.begin(),x.end());
    for(std::size_t i=0;i<out.size();++i) out[i]=std::clamp(out[i],p.lower[i],p.upper[i]);
    return out;
}
}

void validate(const AllocationProblem& p) {
    const auto n=p.target.size();
    if(n==0 || !d::finite(p.target) || !d::finite(p.equality_rhs) || !d::finite(p.inequality_rhs))
        throw std::invalid_argument("allocation matrices must be finite and target nonempty");
    for(const auto* a:{&p.equality,&p.inequality}) {
        d::validate(*a,"allocation matrices");
        if(a->cols!=n && !(a->rows==0 && a->cols==0))
            throw std::invalid_argument("allocation matrix column count");
    }
    if(p.equality.rows!=p.equality_rhs.size() || p.inequality.rows!=p.inequality_rhs.size() || p.lower.size()!=n || p.upper.size()!=n)
        throw std::invalid_argument("allocation matrix/bounds shape");
    bounds(p.lower,p.upper);
    if(p.initial && (p.initial->size()!=n || !d::finite(*p.initial)))
        throw std::invalid_argument("warm start must have the target shape and finite values");
    for(const auto& c:p.extra) {
        if(!c.evaluate || c.lower.empty() || c.upper.empty())
            throw std::invalid_argument("unsupported allocation constraint");
        const auto width=std::max(c.lower.size(),c.upper.size());
        for(std::size_t i=0;i<width;++i) {
            const double lo=bound(c.lower,i,width),hi=bound(c.upper,i,width);
            if(std::isnan(lo) || std::isnan(hi) || lo>hi)
                throw std::invalid_argument("invalid allocation constraint bounds");
        }
        if(c.linear) {
            d::validate(*c.linear,"allocation extra matrix");
            if(c.linear->cols!=n) throw std::invalid_argument("allocation extra matrix column count");
        }
    }
}

double allocation_residual(const AllocationProblem& p,std::span<const double> x) {
    validate(p);
    if(x.size()!=p.target.size()) throw std::invalid_argument("allocation residual point shape");
    if(!d::finite(x)) return std::numeric_limits<double>::infinity();
    double error=0.;
    for(std::size_t i=0;i<x.size();++i) error=std::max({error,p.lower[i]-x[i],x[i]-p.upper[i]});
    if(p.equality.rows) {
        const auto value=d::multiply(p.equality,x);
        for(std::size_t i=0;i<value.size();++i) {
            if(!std::isfinite(value[i])) return std::numeric_limits<double>::infinity();
            error=std::max(error,std::abs(value[i]-p.equality_rhs[i]));
        }
    }
    if(p.inequality.rows) {
        const auto value=d::multiply(p.inequality,x);
        for(std::size_t i=0;i<value.size();++i) {
            if(!std::isfinite(value[i])) return std::numeric_limits<double>::infinity();
            error=std::max(error,value[i]-p.inequality_rhs[i]);
        }
    }
    for(const auto& c:p.extra) {
        const auto value=c.evaluate(x);
        if(value.empty() || !d::finite(value)) return std::numeric_limits<double>::infinity();
        for(std::size_t i=0;i<value.size();++i)
            error=std::max({error,bound(c.lower,i,value.size())-value[i],value[i]-bound(c.upper,i,value.size())});
    }
    return error;
}

Allocation allocate(const AllocationProblem& p) {
    validate(p);
    auto bounded=clipped(p.target,p);
    const double residual=allocation_residual(p,bounded);
    if(residual<=1e-7) return {std::move(bounded),true,residual};
    auto x=clipped(p.initial?std::span<const double>(*p.initial):std::span<const double>(p.target),p);
    for(int restart=0;restart<4;++restart) {
        x=bike::numerics::slsqp(p,x).x;
        if(!d::finite(x)) break;
        x=clipped(x,p);
        if(allocation_residual(p,x)<=1e-7) break;
    }
    if(!d::finite(x)) {
        for(double& v:x) {
            if(std::isnan(v)) v=0.;
            else if(std::isinf(v)) v=std::copysign(std::numeric_limits<double>::max(),v);
        }
        return {clipped(x,p),false,std::numeric_limits<double>::infinity()};
    }
    const double violation=allocation_residual(p,x);
    return {std::move(x),violation<=1e-7,violation};
}

std::pair<Matrix,std::vector<double>> inverse_dynamics_rows(
    const Matrix& mass,const Matrix& actuation,const Matrix& support,
    std::span<const double> bias,std::span<const double> known) {
    d::validate(mass);d::validate(actuation);d::validate(support);
    const auto n=mass.rows;
    if(mass.cols!=n || actuation.rows!=n || support.cols!=n || bias.size()!=n || known.size()!=n || !d::finite(bias) || !d::finite(known))
        throw std::invalid_argument("inverse dynamics rows: incompatible dimensions/nonfinite force");
    auto out=d::zeros(n,n+actuation.cols+support.rows);
    for(std::size_t i=0;i<n;++i) {
        for(std::size_t j=0;j<n;++j) out.values[i*out.cols+j]=mass.values[i*n+j];
        for(std::size_t j=0;j<actuation.cols;++j) out.values[i*out.cols+n+j]=-actuation.values[i*actuation.cols+j];
        for(std::size_t j=0;j<support.rows;++j) out.values[i*out.cols+n+actuation.cols+j]=-support.values[j*n+i];
    }
    return {std::move(out),d::add(std::vector<double>(known.begin(),known.end()),bias,-1.)};
}

ExtraConstraint linear_constraint(Matrix matrix,std::vector<double> lower,std::vector<double> upper) {
    d::validate(matrix);
    const auto shared=std::make_shared<const Matrix>(std::move(matrix));
    return {[shared](std::span<const double> x){return d::multiply(*shared,x);},
        [shared](std::span<const double>){return *shared;},std::move(lower),std::move(upper),*shared};
}

std::vector<ExtraConstraint> grip_constraints(const Matrix& force_map,
    const std::array<double,2>& direction,bool pulling,double limit_n) {
    d::validate(force_map);
    if(force_map.rows!=2 || !d::finite(direction)) throw std::invalid_argument("planar grip map and direction required");
    const double length2=d::dot(direction,direction);
    if(std::abs(length2-1.)>1e-8+1e-5 || !std::isfinite(limit_n) || limit_n<=0.)
        throw std::invalid_argument("unit direction and positive grip limit required");
    auto row=d::zeros(1,force_map.cols);
    for(std::size_t j=0;j<row.cols;++j) {
        const double projection=-(direction[0]*force_map.values[j]+direction[1]*force_map.values[force_map.cols+j]);
        row.values[j]=pulling?projection:-projection;
    }
    std::vector<ExtraConstraint> result;
    result.push_back(linear_constraint(std::move(row),{0.},{std::numeric_limits<double>::infinity()}));
    if(pulling) {
        result.push_back({[force_map,limit_n](std::span<const double> x){
            const auto force=d::multiply(force_map,x);
            return std::vector<double>{d::dot(force,force)/(limit_n*limit_n)};
        },[force_map,limit_n](std::span<const double> x){
            const auto force=d::multiply(force_map,x);
            auto out=d::zeros(1,force_map.cols);
            for(std::size_t j=0;j<out.cols;++j)
                out.values[j]=2.*(force_map.values[j]*force[0]+force_map.values[out.cols+j]*force[1])/(limit_n*limit_n);
            return out;
        },{-std::numeric_limits<double>::infinity()},{1.},std::nullopt});
    }
    return result;
}
} // namespace bike::rider
