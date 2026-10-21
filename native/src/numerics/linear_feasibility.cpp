#include "linear_feasibility.hpp"
#include "dense.hpp"
#include "slsqp.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace bike::numerics {
namespace {
namespace d=dense;
// Binary64 interval arithmetic with outward rounding (completion plan D8;
// macOS arm64 long double was already binary64).
constexpr double inf=std::numeric_limits<double>::infinity();
double down(double x) {return std::nextafter(x,-inf);}
double up(double x) {return std::nextafter(x,inf);}
// Interval lower bound on a linear row. Every product/addition is rounded
// outward. An unbounded or indeterminate interval gives no certificate.
double minimum(std::span<const double> row,std::span<const double> lo,
                    std::span<const double> hi,std::size_t excluded) {
    double sum=0.;
    for(std::size_t j=0;j<row.size();++j) if(j!=excluded && row[j]!=0.) {
        const double value=row[j]>0.?lo[j]:hi[j];
        if(!std::isfinite(value)) return -inf;
        sum=down(sum+down(row[j]*value));
    }
    return sum;
}
}

FeasibilityResult linear_feasibility(const bike::rider::Matrix& g,
    std::span<const double> h,std::span<const double> lower,std::span<const double> upper,
    const bike::rider::Matrix& a,std::span<const double> b,const FeasibilityOptions& options) {
    if(!std::isfinite(options.inequality_slack) || options.inequality_slack<0. || !std::isfinite(options.primal_tolerance) || options.primal_tolerance<0.)
        throw std::invalid_argument("linear feasibility tolerances");
    const auto n=lower.size();
    bike::rider::AllocationProblem p;
    p.target.assign(n,0.);p.lower.assign(lower.begin(),lower.end());p.upper.assign(upper.begin(),upper.end());
    p.equality=a;p.equality_rhs.assign(b.begin(),b.end());p.inequality=g;p.inequality_rhs.assign(h.begin(),h.end());
    for(double& v:p.inequality_rhs) v+=options.inequality_slack;
    bike::rider::validate(p);
    if(options.max_iterations==0) return {.status=FeasibilityStatus::unknown,.witness={},.reason="linear feasibility budget exhausted"};
    auto rows=g; if(rows.rows==0) rows.cols=n;
    std::vector<double> bounds;bounds.reserve(h.size()+2*b.size());
    for(double const value:h)bounds.push_back(up(value+options.inequality_slack));
    for(std::size_t i=0;i<a.rows;++i) {
        const auto row=std::span<const double>(a.values).subspan(i*n,n);
        d::append_row(rows,row);bounds.push_back(up(b[i]+options.primal_tolerance));
        std::vector<double> neg(row.begin(),row.end());for(double& v:neg) v=-v;
        d::append_row(rows,neg);bounds.push_back(up(-b[i]+options.primal_tolerance));
    }
    auto lo=p.lower,hi=p.upper;
    for(std::size_t pass=0;pass<std::min<std::size_t>(options.max_iterations,n+rows.rows+1);++pass) {
        bool changed=false;
        for(std::size_t i=0;i<rows.rows;++i) {
            const auto row=std::span<const double>(rows.values).subspan(i*n,n);
            const double least=minimum(row,lo,hi,n);
            if(std::isfinite(least) && least>up(bounds[i]))
                return {.status=FeasibilityStatus::proven_infeasible,.witness={},.reason="outward-rounded linear row contradicts propagated box"};
            for(std::size_t j=0;j<n;++j) if(row[j]!=0.) {
                const double rest=minimum(row,lo,hi,j);
                if(!std::isfinite(rest)) continue;
                const double numerator=up(bounds[i]-rest);
                const double raw=numerator/row[j];
                if(!std::isfinite(raw)) continue;
                if(row[j]>0.) {
                    const double new_hi=std::nextafter(up(raw),std::numeric_limits<double>::infinity());
                    if(new_hi<hi[j]) {hi[j]=new_hi;changed=true;}
                } else {
                    const double new_lo=std::nextafter(down(raw),-std::numeric_limits<double>::infinity());
                    if(new_lo>lo[j]) {lo[j]=new_lo;changed=true;}
                }
                if(lo[j]>hi[j]) return {.status=FeasibilityStatus::proven_infeasible,.witness={},.reason="outward-rounded propagated bounds are disjoint"};
            }
        }
        if(!changed) break;
    }
    for(std::size_t j=0;j<n;++j) p.target[j]=std::clamp(0.,lo[j],hi[j]);
    const auto valid=[&](std::span<const double> x) {
        return bike::rider::allocation_residual(p,x)<=options.primal_tolerance;
    };
    if(valid(p.target)) return {.status=FeasibilityStatus::feasible,.witness=p.target,.reason="verified linear witness"};
    try {
        SlsqpOptions opt;opt.max_iterations=options.max_iterations;
        const auto result=slsqp(p,p.target,opt);
        if(valid(result.x)) return {.status=FeasibilityStatus::feasible,.witness=result.x,.reason="verified linear witness"};
        return {.status=FeasibilityStatus::unknown,.witness=result.x,.reason="no witness and no verified infeasibility certificate"};
    } catch(const std::runtime_error&) {
        return {.status=FeasibilityStatus::unknown,.witness={},.reason="numerical failure has no infeasibility certificate"};
    }
}
bool linear_region_infeasible(const bike::rider::Matrix& g,std::span<const double> h,
    std::span<const double> lo,std::span<const double> hi,const bike::rider::Matrix& a,std::span<const double> b) {
    return linear_feasibility(g,h,lo,hi,a,b).status==FeasibilityStatus::proven_infeasible;
}
} // namespace bike::numerics
