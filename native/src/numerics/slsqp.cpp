// Dense sequential quadratic programming, owning all work and callbacks.
// Native algorithm identity: bike-dense-sqp-bfgs-v1. This is not a claim
// of bitwise equivalence with the historical SciPy SLSQP implementation.
#include "slsqp.hpp"
#include "dense.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace bike::numerics {
namespace {
namespace d=dense;
using bike::rider::AllocationProblem;
using bike::rider::Matrix;
using Vector=std::vector<double>;
struct Linearization { Matrix jacobian; Vector residual; std::size_t equalities=0; };
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
double component(const Vector& x,std::size_t i,std::size_t n) {
    if(x.size()!=1 && x.size()!=n) throw std::invalid_argument("SQP constraint bound shape");
    return x[x.size()==1?0:i];
}
Linearization constraints(const AllocationProblem& p,const Vector& x,double eps) {
    const std::size_t n=x.size();
    Matrix eq=d::zeros(0,n),ineq=d::zeros(0,n);
    Vector ce,ci;
    const auto append=[&](bool equality,std::span<const double> row,double value) {
        d::append_row(equality?eq:ineq,row);
        (equality?ce:ci).push_back(value);
    };
    for(std::size_t i=0;i<p.equality.rows;++i) {
        const auto row=std::span<const double>(p.equality.values).subspan(i*n,n);
        append(true,row,d::dot(row,x)-p.equality_rhs[i]);
    }
    for(std::size_t i=0;i<p.inequality.rows;++i) {
        Vector row(n);
        for(std::size_t j=0;j<n;++j) row[j]=-p.inequality.values[i*n+j];
        append(false,row,p.inequality_rhs[i]+d::dot(row,x));
    }
    for(const auto& extra:p.extra) {
        const auto value=extra.evaluate(x);
        if(value.empty() || !d::finite(value)) throw std::runtime_error("SQP nonfinite constraint value");
        Matrix jac;
        if(extra.jacobian) jac=extra.jacobian(x);
        else {
            jac=d::zeros(value.size(),n);
            Vector trial=x;
            for(std::size_t j=0;j<n;++j) {
                double step=eps;
                if(x[j]+step>p.upper[j]) step=-step;
                if(x[j]+step<p.lower[j]) step=std::min(p.upper[j]-x[j],x[j]-p.lower[j]);
                if(p.lower[j]==p.upper[j]) continue;
                if(step==0. || x[j]+step==x[j]) step=std::nextafter(x[j],p.upper[j]>x[j]?p.upper[j]:p.lower[j])-x[j];
                trial[j]=std::clamp(x[j]+step,p.lower[j],p.upper[j]);
                const double dx=trial[j]-x[j];
                if(dx==0.) continue;
                const auto sampled=extra.evaluate(trial);
                if(sampled.size()!=value.size() || !d::finite(sampled)) throw std::runtime_error("SQP invalid finite-difference constraint");
                for(std::size_t i=0;i<value.size();++i) jac.values[i*n+j]=(sampled[i]-value[i])/dx;
                trial[j]=x[j];
            }
        }
        d::validate(jac,"SQP constraint Jacobian");
        if(jac.rows!=value.size() || jac.cols!=n) throw std::invalid_argument("SQP constraint Jacobian shape");
        // _as_old_style: equalities, lower bounds, upper bounds for each
        // extra; SQP collects equality rows first without reordering any
        // row within its source family.
        for(int family=0;family<3;++family) for(std::size_t i=0;i<value.size();++i) {
            const double lo=component(extra.lower,i,value.size()),hi=component(extra.upper,i,value.size());
            const bool equal=std::isfinite(lo)&&lo==hi;
            if((family==0&&!equal)||(family==1&&(equal||!std::isfinite(lo)))||(family==2&&(equal||!std::isfinite(hi)))) continue;
            Vector row(jac.values.begin()+static_cast<std::ptrdiff_t>(i*n),jac.values.begin()+static_cast<std::ptrdiff_t>((i+1)*n));
            const double sign=family==2?-1.:1.;
            for(double& a:row) a*=sign;
            append(family==0,row,sign*(value[i]-(family==2?hi:lo)));
        }
    }
    for(std::size_t i=0;i<n;++i) {
        Vector row(n,0.);row[i]=1.;
        if(p.lower[i]==p.upper[i]) append(true,row,x[i]-p.lower[i]);
        else {
            if(std::isfinite(p.lower[i])) append(false,row,x[i]-p.lower[i]);
            row[i]=-1.;
            if(std::isfinite(p.upper[i])) append(false,row,p.upper[i]-x[i]);
        }
    }
    const auto neq=eq.rows;
    eq.values.insert(eq.values.end(),ineq.values.begin(),ineq.values.end());eq.rows+=ineq.rows;
    ce.insert(ce.end(),ci.begin(),ci.end());
    return {.jacobian=std::move(eq),.residual=std::move(ce),.equalities=neq};
}
double violation(const Linearization& c) {
    double result=0.;
    for(std::size_t i=0;i<c.residual.size();++i)
        result+=i<c.equalities?std::abs(c.residual[i]):std::max(0.,-c.residual[i]);
    return result;
}
struct QpResult { Vector step, multipliers; bool solved=false; };
QpResult quadratic(const Matrix& hessian,const Vector& gradient,const Linearization& c) {
    const auto n=gradient.size(),m=c.residual.size();
    std::vector<std::size_t> active(c.equalities);
    std::ranges::iota(active,0U);
    QpResult out{.step=Vector(n,0.),.multipliers=Vector(m,0.),.solved=false};
    for(std::size_t iteration=0;iteration<8*(m+n+1);++iteration) {
        const auto k=active.size();
        Matrix system=d::zeros(n+k,n+k),rhs=d::zeros(n+k,1);
        for(std::size_t i=0;i<n;++i) {
            rhs.values[i]=-gradient[i];
            for(std::size_t j=0;j<n;++j) system.values[i*(n+k)+j]=hessian.values[i*n+j];
        }
        for(std::size_t row=0;row<k;++row) {
            const auto r=active[row];rhs.values[n+row]=-c.residual[r];
            for(std::size_t j=0;j<n;++j) {
                const auto value=c.jacobian.values[r*n+j];
                system.values[j*(n+k)+n+row]=-value;
                system.values[(n+row)*(n+k)+j]=value;
            }
        }
        const auto solved=d::least_squares(system,rhs).solution.values;
        const auto residual=d::add(d::multiply(system,solved),rhs.values,-1.);
        if(d::norm(residual)>1e-8*(1.+d::norm(rhs.values))) return out;
        std::copy_n(solved.begin(),n,out.step.begin());
        auto linear=d::add(d::multiply(c.jacobian,out.step),c.residual);
        std::size_t worst=m;double minimum=-1e-10;
        for(std::size_t i=c.equalities;i<m;++i)
            if(std::ranges::find(active,i)==active.end() && linear[i]<minimum) {minimum=linear[i];worst=i;}
        if(worst<m) {active.push_back(worst);continue;}
        std::size_t remove=k; minimum=-1e-10;
        for(std::size_t i=c.equalities;i<k;++i)
            if(solved[n+i]<minimum) {minimum=solved[n+i];remove=i;}
        if(remove<k) {active.erase(active.begin()+static_cast<std::ptrdiff_t>(remove));continue;}
        for(std::size_t i=0;i<k;++i) out.multipliers[active[i]]=solved[n+i];
        out.solved=true;return out;
    }
    return out;
}
}

SlsqpResult slsqp(const AllocationProblem& p,std::span<const double> initial,const SlsqpOptions& options) {
    bike::rider::validate(p);
    if(initial.size()!=p.target.size() || !d::finite(initial) || options.max_iterations==0 || !std::isfinite(options.ftol) || options.ftol<=0. || !std::isfinite(options.finite_difference_step) || options.finite_difference_step<=0.)
        throw std::invalid_argument("SQP initial point/options");
    SlsqpResult result{.x=Vector(initial.begin(),initial.end()),.iterations=0,.converged=false,.status=9};
    const auto n=result.x.size();
    for(std::size_t j=0;j<n;++j)result.x[j]=std::clamp(result.x[j],p.lower[j],p.upper[j]);
    Matrix hessian=d::identity(n);
    double penalty=1.;
    for(std::size_t iteration=0;iteration<options.max_iterations;++iteration) {
        result.iterations=iteration+1;
        const Vector gradient=d::add(result.x,p.target,-1.);
        const double objective=.5*d::dot(gradient,gradient);
        Linearization current;
        try {current=constraints(p,result.x,options.finite_difference_step);}
        catch(const std::runtime_error&) {result.status=6;return result;}
        QpResult qp;
        try {qp=quadratic(hessian,gradient,current);}
        catch(const std::runtime_error&) {result.status=6;return result;}
        if(!qp.solved) {
            // Feasibility restoration for an inconsistent local SQP
            // linearization. It can yield a diagnostic iterate, never an
            // infeasibility certificate or an optimizer success flag.
            Matrix a=d::zeros(0,n),b=d::zeros(0,1);
            for(std::size_t i=0;i<current.residual.size();++i) if(i<current.equalities || current.residual[i]<0.) {
                d::append_row(a,std::span<const double>(current.jacobian.values).subspan(i*n,n));
                b.values.push_back(-current.residual[i]);++b.rows;
            }
            if(a.rows==0) {result.status=6;return result;}
            try {qp.step=d::least_squares(a,b).solution.values;}
            catch(const std::runtime_error&) {result.status=6;return result;}
            hessian=d::identity(n);
        }
        double multipliers=0.;
        for(double const value:qp.multipliers) multipliers=std::max(multipliers,std::abs(value));
        penalty=std::max(penalty,multipliers+1.);
        const double current_violation=violation(current);
        const double directional=d::dot(gradient,qp.step)-penalty*current_violation;
        if(d::norm(qp.step)<=options.ftol && current_violation<=options.ftol) {
            result.converged=true;result.status=0;return result;
        }
        Vector candidate; Linearization next; bool accepted=false;double new_objective=objective;
        double alpha=1.;
        for(int line=0;line<40;++line) {
            candidate=d::add(result.x,qp.step,alpha);
            for(std::size_t j=0;j<n;++j) candidate[j]=std::clamp(candidate[j],p.lower[j],p.upper[j]);
            bool evaluable=true;
            try {next=constraints(p,candidate,options.finite_difference_step);}
            catch(const std::runtime_error&) {evaluable=false;}
            if(evaluable) {
                const auto delta=d::add(candidate,p.target,-1.);
                new_objective=.5*d::dot(delta,delta);
                const double next_violation=violation(next);
                if(new_objective+penalty*next_violation<=objective+penalty*current_violation+1e-4*alpha*std::min(directional,-options.ftol)
                   || (next_violation<.5*current_violation && current_violation>1e-7)) {accepted=true;break;}
            }
            alpha*=.5;
        }
        if(!accepted) {result.status=8;return result;}
        const auto step=d::add(candidate,result.x,-1.);
        Vector change=step;
        if(qp.solved && next.jacobian.rows==current.jacobian.rows)
            for(std::size_t i=0;i<current.jacobian.rows;++i)
                for(std::size_t j=0;j<n;++j)
                    change[j]-=(next.jacobian.values[i*n+j]-current.jacobian.values[i*n+j])*qp.multipliers[i];
        auto hs=d::multiply(hessian,step);
        const double shs=d::dot(step,hs);
        double sy=d::dot(step,change);
        if(shs>0. && sy<.2*shs) {
            const double theta=.8*shs/(shs-sy);
            for(std::size_t j=0;j<n;++j) change[j]=theta*change[j]+(1.-theta)*hs[j];
            sy=d::dot(step,change);
        }
        if(sy>1e-20 && shs>1e-20)
            for(std::size_t i=0;i<n;++i) for(std::size_t j=0;j<n;++j)
                hessian.values[i*n+j]+=change[i]*change[j]/sy-hs[i]*hs[j]/shs;
        result.x=std::move(candidate);
        if(std::abs(new_objective-objective)<=options.ftol && violation(next)<=options.ftol) {
            result.converged=true;result.status=0;return result;
        }
    }
    return result;
}
} // namespace bike::numerics
