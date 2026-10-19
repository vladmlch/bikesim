// Dense trust-region reflective least squares: Coleman-Li scaling,
// More's SVD trust-region solve, reflected/Cauchy step selection and the
// source finite-difference/termination contracts. Control-flow authority:
// scipy.optimize._lsq.trf/common and optimize._numdiff (BSD-3-Clause).
// Redistribution notice: SCIPY-LICENSE.txt in this directory.
// The small dense SVD is an owned one-sided Jacobi provider, not a claim
// of bitwise identity with SciPy's LAPACK provider. Numerical qualification
// belongs to the solver fixtures and actual acceleration acceptance gate.
#include "least_squares.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>

namespace bike::numerics {
namespace {
using Vector = std::vector<double>;
constexpr double eps = std::numeric_limits<double>::epsilon();
constexpr double infinity = std::numeric_limits<double>::infinity();

double dot(std::span<const double> a, std::span<const double> b) {
    double value = 0.;
    for (std::size_t i = 0; i < a.size(); ++i) value += a[i] * b[i];
    return value;
}
double norm(std::span<const double> a) { return std::sqrt(dot(a, a)); }
bool finite(std::span<const double> a) {
    return std::ranges::all_of(a, [](double x) { return std::isfinite(x); });
}
Vector product(std::span<const double> a, std::span<const double> b) {
    Vector out(a.size());
    for (std::size_t i = 0; i < a.size(); ++i) out[i] = a[i] * b[i];
    return out;
}
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
Vector matvec(std::span<const double> a, std::size_t rows,
              std::size_t cols, std::span<const double> x) {
    Vector out(rows, 0.);
    for (std::size_t i = 0; i < rows; ++i)
        for (std::size_t j = 0; j < cols; ++j) out[i] += a[i * cols + j] * x[j];
    return out;
}
Vector gradient(std::span<const double> j, std::span<const double> f,
                std::size_t n) {
    Vector g(n, 0.);
    for (std::size_t row = 0; row < f.size(); ++row)
        for (std::size_t col = 0; col < n; ++col) g[col] += j[row*n+col]*f[row];
    return g;
}
void interior(Vector& x, std::span<const double> lo,
              std::span<const double> hi, double relative_step) {
    for (std::size_t i = 0; i < x.size(); ++i) {
        const double ld = x[i] - lo[i], ud = hi[i] - x[i];
        if (std::isfinite(lo[i]) && ld <= std::min(ud, relative_step*std::max(1., std::abs(lo[i]))))
            x[i] = relative_step == 0. ? std::nextafter(lo[i], hi[i])
                : lo[i] + relative_step*std::max(1., std::abs(lo[i]));
        else if (std::isfinite(hi[i]) && ud <= std::min(ld, relative_step*std::max(1., std::abs(hi[i]))))
            x[i] = relative_step == 0. ? std::nextafter(hi[i], lo[i])
                : hi[i] - relative_step*std::max(1., std::abs(hi[i]));
        if (x[i] < lo[i] || x[i] > hi[i]) x[i] = .5*lo[i] + .5*hi[i];
    }
}
struct Evaluator {
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members) aggregate mirrors the Python reference structure
    const Residual& call;
    std::size_t width = 0;
    std::int64_t calls = 0;
    Vector operator()(std::span<const double> x) {
        ++calls;
        Vector f = call(x);
        if (width == 0) width = f.size();
        if (f.empty() || f.size() != width)
            throw std::invalid_argument("least_squares: residual width changed or is zero");
        return f;
    }
};

Vector jacobian(Evaluator& fun, const Vector& x, const Vector& f,
                std::span<const double> lo, std::span<const double> hi,
                const LeastSquaresOptions& options) {
    const std::size_t n = x.size(), m = f.size();
    Vector j(m*n), trial = x;
    const bool central = options.difference == DifferenceScheme::three_point;
    const double automatic = central ? std::cbrt(eps) : std::sqrt(eps);
    for (std::size_t col = 0; col < n; ++col) {
        const double sign = x[col] < 0. ? -1. : 1.;
        double h = options.relative_difference_step > 0.
            ? options.relative_difference_step * sign * std::abs(x[col])
            : automatic * sign * std::max(1., std::abs(x[col]));
        if (x[col] + h == x[col]) h = automatic * sign * std::max(1., std::abs(x[col]));
        const double ld = x[col] - lo[col], ud = hi[col] - x[col];
        bool one_sided = !central;
        if (central) {
            h = std::abs(h);
            if (ld < h || ud < h) {
                one_sided = true;
                h = ud >= ld ? std::min(h, .5*ud) : -std::min(h, .5*ld);
                if (std::abs(h) <= std::min(ld,ud)) {
                    h = std::min(ld,ud);
                    one_sided = false;
                }
            }
        } else if (x[col] + h < lo[col] || x[col] + h > hi[col]) {
            if (std::abs(h) <= std::max(ld,ud)) h = -h;
            else h = ud >= ld ? ud : -ld;
        }
        if (h == 0. || !std::isfinite(h))
            throw std::invalid_argument("least_squares: bounds have no representable difference interval");
        trial[col] = x[col] + (one_sided ? h : -h);
        const double first_x = trial[col];
        const Vector a = fun(trial);
        if (!finite(a)) throw std::invalid_argument("least_squares: nonfinite finite-difference residual");
        if (central) {
            trial[col] = x[col] + (one_sided ? 2.*h : h);
            const double second_x = trial[col];
            const Vector b = fun(trial);
            if (!finite(b)) throw std::invalid_argument("least_squares: nonfinite finite-difference residual");
            const double dx = one_sided ? second_x-x[col] : second_x-first_x;
            for (std::size_t row=0; row<m; ++row)
                j[row*n+col] = one_sided ? (-3.*f[row]+4.*a[row]-b[row])/dx : (b[row]-a[row])/dx;
        } else {
            const double dx = first_x-x[col];
            for (std::size_t row=0; row<m; ++row) j[row*n+col]=(a[row]-f[row])/dx;
        }
        trial[col] = x[col];
    }
    if (!finite(j)) throw std::invalid_argument("least_squares: nonfinite Jacobian");
    return j;
}

struct ThinSvd { Vector singular, right, uf; };
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
ThinSvd svd(Vector a, std::size_t rows, std::size_t n,
            std::span<const double> f) {
    Vector v(n*n, 0.);
    for (std::size_t i=0; i<n; ++i) v[i*n+i] = 1.;
    bool converged = false;
    for (std::size_t sweep=0; sweep<100; ++sweep) {
        bool changed = false;
        for (std::size_t p=0; p<n; ++p) for (std::size_t q=p+1; q<n; ++q) {
            double aa=0., bb=0., ab=0.;
            for (std::size_t r=0; r<rows; ++r) {
                aa+=a[r*n+p]*a[r*n+p]; bb+=a[r*n+q]*a[r*n+q]; ab+=a[r*n+p]*a[r*n+q];
            }
            if (aa==0. || bb==0. || std::abs(ab)<=eps*std::sqrt(aa)*std::sqrt(bb)*static_cast<double>(rows)) continue;
            const double tau=(bb-aa)/(2.*ab);
            const double t=std::copysign(1.,tau)/(std::abs(tau)+std::hypot(1.,tau));
            const double c=1./std::sqrt(1.+t*t), s=c*t;
            for (std::size_t r=0; r<rows; ++r) {
                const double u=a[r*n+p], w=a[r*n+q];
                a[r*n+p]=c*u-s*w; a[r*n+q]=s*u+c*w;
            }
            for (std::size_t r=0; r<n; ++r) {
                const double u=v[r*n+p], w=v[r*n+q];
                v[r*n+p]=c*u-s*w; v[r*n+q]=s*u+c*w;
            }
            changed = true;
        }
        if (!changed) { converged = true; break; }
    }
    if (!converged) throw std::runtime_error("least_squares: dense Jacobi SVD did not converge");
    Vector s(n,0.), uf(n,0.);
    for (std::size_t col=0; col<n; ++col) {
        for (std::size_t r=0; r<rows; ++r) s[col]=std::hypot(s[col],a[r*n+col]);
        if (s[col]>0.) for (std::size_t r=0; r<f.size(); ++r) uf[col]+=a[r*n+col]/s[col]*f[r];
    }
    std::vector<std::size_t> order(n); std::ranges::iota(order,0);
    std::ranges::stable_sort(order,[&](std::size_t aidx,std::size_t bidx){return s[aidx]>s[bidx];});
    ThinSvd out{.singular=Vector(n),.right=Vector(n*n),.uf=Vector(n)};
    for (std::size_t col=0; col<n; ++col) {
        out.singular[col]=s[order[col]]; out.uf[col]=uf[order[col]];
        for (std::size_t row=0; row<n; ++row) out.right[row*n+col]=v[row*n+order[col]];
    }
    return out;
}
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
Vector trust_step(const ThinSvd& s, std::size_t m, double radius, double& alpha) {
    const std::size_t n=s.singular.size();
    const bool full=m>=n && s.singular.back()>eps*static_cast<double>(m)*s.singular.front();
    Vector coefficients(n), suf=product(s.singular,s.uf);
    if (full) {
        for (std::size_t i=0;i<n;++i) coefficients[i]=-s.uf[i]/s.singular[i];
        Vector p=matvec(s.right,n,n,coefficients);
        if (norm(p)<=radius) { alpha=0.; return p; }
    }
    auto const phi = [&](double a) {
        double sum=0., derivative=0.;
        for (std::size_t i=0;i<n;++i) {
            const double denom=s.singular[i]*s.singular[i]+a;
            const double z=denom==0. ? 0. : suf[i]/denom;
            sum+=z*z; if(denom>0.) derivative+=z*z/denom;
        }
        const double length=std::sqrt(sum);
        return std::pair{length-radius,length>0. ? -derivative/length : -infinity};
    };
    double upper=norm(suf)/radius, lower=0.;
    if (upper==0.) { alpha=0.; return Vector(n,0.); } // NOLINT(modernize-return-braced-init-list) braced form would bind initializer_list, changing semantics
    if (full) { const auto [value,derivative]=phi(0.); lower=-value/derivative; }
    if (!full && alpha==0.) alpha=std::max(.001*upper,std::sqrt(lower*upper));
    for (int iteration=0;iteration<10;++iteration) {
        if (alpha<lower || alpha>upper) alpha=std::max(.001*upper,std::sqrt(lower*upper));
        const auto [value,derivative]=phi(alpha);
        if (value<0.) upper=alpha;
        const double ratio=value/derivative;
        lower=std::max(lower,alpha-ratio);
        alpha-=(value+radius)*ratio/radius;
        if (std::abs(value)<.01*radius) break;
    }
    for(std::size_t i=0;i<n;++i) {
        const double denom=s.singular[i]*s.singular[i]+alpha;
        coefficients[i]=denom>0. ? -suf[i]/denom : 0.;
    }
    Vector p=matvec(s.right,n,n,coefficients);
    const double length=norm(p);
    if(length>0.) for(double& x:p) x*=radius/length;
    return p;
}
std::pair<double,std::vector<bool>> bound_stride(std::span<const double> x,
    std::span<const double> p,std::span<const double> lo,std::span<const double> hi) {
    Vector strides(x.size(),infinity);
    for(std::size_t i=0;i<x.size();++i) if(p[i]!=0.) strides[i]=std::max((lo[i]-x[i])/p[i],(hi[i]-x[i])/p[i]);
    const double stride=*std::ranges::min_element(strides);
    std::vector<bool> hit(x.size());
    for(std::size_t i=0;i<x.size();++i) hit[i]=strides[i]==stride;
    return {stride,std::move(hit)};
}
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
double quadratic(std::span<const double> j,std::size_t m,std::span<const double> g,
                 std::span<const double> diag,std::span<const double> x) {
    const Vector jx=matvec(j,m,x.size(),x);
    double out=.5*dot(jx,jx)+dot(g,x);
    for(std::size_t i=0;i<x.size();++i) out+=.5*diag[i]*x[i]*x[i];
    return out;
}
std::pair<double,double> line_min(std::span<const double> j,std::size_t m,
    std::span<const double> g,std::span<const double> diag,
    std::span<const double> p,std::span<const double> origin,double lower,double upper) {
    const Vector jp=matvec(j,m,p.size(),p), jo=matvec(j,m,p.size(),origin);
    double a=.5*dot(jp,jp),b=dot(g,p)+dot(jo,jp);
    for(std::size_t i=0;i<p.size();++i){a+=.5*diag[i]*p[i]*p[i];b+=diag[i]*origin[i]*p[i];}
    const double c=quadratic(j,m,g,diag,origin);
    double t=lower,value=(a*t+b)*t+c;
    const double end=(a*upper+b)*upper+c;
    if(end<value){t=upper;value=end;}
    if(a>0.){const double mid=-.5*b/a;if(mid>lower&&mid<upper){const double v=(a*mid+b)*mid+c;if(v<value){t=mid;value=v;}}}
    return {t,value};
}
struct Step {Vector physical,hat;double predicted;};
Step select(const Vector& x,const Vector& j,std::size_t m,const Vector& diag,
            const Vector& g,Vector p_hat,const Vector& d,double radius,
            std::span<const double> lo,std::span<const double> hi,double theta) {
    Vector p=product(p_hat,d);
    bool bounded=true;for(std::size_t i=0;i<x.size();++i)bounded=bounded&&x[i]+p[i]>=lo[i]&&x[i]+p[i]<=hi[i];
    if(bounded)return {.physical=std::move(p),.hat=p_hat,.predicted=-quadratic(j,m,g,diag,p_hat)};
    // `p` is moved only inside the early return above; on this path it is live.
    // cppcheck-suppress accessMoved
    const auto [stride,hits]=bound_stride(x,p,lo,hi);
    Vector r_hat=p_hat;
    for(std::size_t i=0;i<x.size();++i){if(hits[i])r_hat[i]*=-1.;p[i]*=stride;p_hat[i]*=stride;}
    Vector r=product(r_hat,d),on_bound=x;for(std::size_t i=0;i<x.size();++i)on_bound[i]+=p[i];
    const double aa=dot(r_hat,r_hat),bb=dot(p_hat,r_hat),cc=std::min(0.,dot(p_hat,p_hat)-radius*radius);
    const double root=std::sqrt(std::max(0.,bb*bb-aa*cc));
    const double q=-(bb+std::copysign(root,bb));
    const double to_tr=aa>0. ? std::max(q/aa,q==0.?0.:cc/q):0.;
    const double to_bound=bound_stride(on_bound,r,lo,hi).first;
    const double rs=std::min(to_bound,to_tr);
    double rv=infinity;
    if(rs>0.) {
        const double low=(1.-theta)*stride/rs, high=rs==to_bound ? theta*to_bound:to_tr;
        if(low<=high){const auto [t,v]=line_min(j,m,g,diag,r_hat,p_hat,low,high);rv=v;for(std::size_t i=0;i<x.size();++i)r_hat[i]=p_hat[i]+t*r_hat[i];r=product(r_hat,d);}
    }
    for(std::size_t i=0;i<x.size();++i){p[i]*=theta;p_hat[i]*=theta;}
    const double pv=quadratic(j,m,g,diag,p_hat);
    Vector ag_hat=g;for(double& z:ag_hat)z=-z;
    Vector ag=product(ag_hat,d);
    const double ag_tr=radius/norm(ag_hat),ag_bound=bound_stride(x,ag,lo,hi).first;
    const auto [ag_stride,ag_value]=line_min(j,m,g,diag,ag_hat,Vector(x.size(),0.),0.,ag_bound<ag_tr ? theta*ag_bound:ag_tr);
    for(std::size_t i=0;i<x.size();++i){ag[i]*=ag_stride;ag_hat[i]*=ag_stride;}
    if(pv<rv&&pv<ag_value)return{.physical=std::move(p),.hat=std::move(p_hat),.predicted=-pv};
    if(rv<pv&&rv<ag_value)return{.physical=std::move(r),.hat=std::move(r_hat),.predicted=-rv};
    return{.physical=std::move(ag),.hat=std::move(ag_hat),.predicted=-ag_value};
}
} // namespace

LeastSquaresResult bounded_least_squares(const Residual& residual,
    std::span<const double> initial,std::span<const double> lower,
    std::span<const double> upper,const LeastSquaresOptions& o) {
    const std::size_t n=initial.size();
    if(!residual||n==0||n>4096||lower.size()!=n||upper.size()!=n||!finite(initial))
        throw std::invalid_argument("least_squares: invalid dimensions or initial point");
    if(o.max_evaluations<=0||!std::isfinite(o.relative_difference_step)||o.relative_difference_step<0.||
       !std::isfinite(o.ftol)||!std::isfinite(o.xtol)||!std::isfinite(o.gtol)||o.ftol<0.||o.xtol<0.||o.gtol<0.||
       std::max({o.ftol,o.xtol,o.gtol})<=eps)
        throw std::invalid_argument("least_squares: invalid evaluation budget or tolerances");
    for(std::size_t i=0;i<n;++i)if(std::isnan(lower[i])||std::isnan(upper[i])||lower[i]>=upper[i]||initial[i]<lower[i]||initial[i]>upper[i])
        throw std::invalid_argument("least_squares: bounds or infeasible initial point");
    LeastSquaresResult out;
    Vector x(initial.begin(),initial.end());interior(x,lower,upper,1e-10);
    Evaluator fun{.call=residual};Vector f=fun(x);
    if(!finite(f))throw std::invalid_argument("least_squares: residuals are not finite in the initial point");
    if(n>1048576/n||f.size()>1048576/n-n)throw std::invalid_argument("least_squares: augmented Jacobian exceeds bounded dense workspace");
    Vector j=jacobian(fun,x,f,lower,upper,o),g=gradient(j,f,n),scale(n,1.),inverse(n,1.);
    out.evaluations=1;out.jacobian_evaluations=1;
    auto const rescale=[&](bool first){if(!o.jacobian_scaling)return;for(std::size_t col=0;col<n;++col){double sum=0.;for(std::size_t row=0;row<f.size();++row)sum+=j[row*n+col]*j[row*n+col];double s=std::sqrt(sum);if(first&&s==0.)s=1.;if(!first)s=std::max(s,inverse[col]);inverse[col]=s;scale[col]=1./s;}};
    rescale(true);
    if(!finite(j)||!finite(g)||!finite(scale)||!finite(inverse))
        throw std::invalid_argument("least_squares: nonfinite Jacobian scaling or gradient");
    auto const scaling=[&](){std::pair<Vector,Vector> value{Vector(n,1.),Vector(n,0.)};for(std::size_t i=0;i<n;++i){if(g[i]<0.&&std::isfinite(upper[i])){value.first[i]=upper[i]-x[i];value.second[i]=-1.;}else if(g[i]>0.&&std::isfinite(lower[i])){value.first[i]=x[i]-lower[i];value.second[i]=1.;}}return value;};
    auto [initial_v,initial_dv]=scaling();Vector rx(n);
    for(std::size_t i=0;i<n;++i){if(initial_dv[i]!=0.)initial_v[i]*=inverse[i];rx[i]=x[i]*inverse[i]/std::sqrt(initial_v[i]);}
    double radius=norm(rx);if(radius==0.)radius=1.;
    double alpha=0.,cost=.5*dot(f,f);
    if(!std::isfinite(radius)||!std::isfinite(cost))
        throw std::invalid_argument("least_squares: nonfinite initial objective or trust radius");
    for(;;) {
        auto [v,dv]=scaling();out.optimality=0.;for(std::size_t i=0;i<n;++i)out.optimality=std::max(out.optimality,std::abs(g[i]*v[i]));
        if(out.optimality<o.gtol){out.status=1;break;}
        if(out.evaluations>=o.max_evaluations)break;
        Vector d(n),diag(n),gh(n),jh(j.size()),aug((f.size()+n)*n,0.);
        for(std::size_t i=0;i<n;++i){if(dv[i]!=0.)v[i]*=inverse[i];d[i]=std::sqrt(v[i])*scale[i];diag[i]=g[i]*dv[i]*scale[i];gh[i]=d[i]*g[i];aug[(f.size()+i)*n+i]=std::sqrt(std::max(0.,diag[i]));}
        for(std::size_t row=0;row<f.size();++row)for(std::size_t col=0;col<n;++col){jh[row*n+col]=j[row*n+col]*d[col];aug[row*n+col]=jh[row*n+col];}
        const ThinSvd factor=svd(std::move(aug),f.size()+n,n,f);
        const double theta=std::max(.995,1.-out.optimality);
        double actual=-1.,new_cost=cost;Vector next=x,next_f=f;
        while(actual<=0.&&out.evaluations<o.max_evaluations) {
            const auto step=select(x,jh,f.size(),diag,gh,trust_step(factor,f.size(),radius,alpha),d,radius,lower,upper,theta);
            for(std::size_t i=0;i<n;++i)next[i]=x[i]+step.physical[i];
            interior(next,lower,upper,0.);
            next_f=fun(next);++out.evaluations;
            const double hnorm=norm(step.hat);
            if(!finite(next_f)){radius=.25*hnorm;if(radius==0.)break;continue;}
            new_cost=.5*dot(next_f,next_f);actual=cost-new_cost;
            const double ratio=step.predicted>0. ? actual/step.predicted : (actual==0.&&step.predicted==0. ? 1.:0.);
            double new_radius=radius;if(ratio<.25)new_radius=.25*hnorm;else if(ratio>.75&&hnorm>.95*radius)new_radius=2.*radius;
            const bool ftol=actual<o.ftol*cost&&ratio>.25;
            const bool xtol=norm(step.physical)<o.xtol*(o.xtol+norm(x));
            if(ftol||xtol){out.status=ftol&&xtol?4:(ftol?2:3);break;}
            if(new_radius<=0.||!std::isfinite(new_radius)){out.status=0;break;}
            alpha*=radius/new_radius;radius=new_radius;
        }
        if(actual>0.){x=std::move(next);f=std::move(next_f);cost=new_cost;j=jacobian(fun,x,f,lower,upper,o);++out.jacobian_evaluations;g=gradient(j,f,n);rescale(false);if(!finite(g)||!finite(scale)||!finite(inverse))throw std::invalid_argument("least_squares: nonfinite accepted gradient or scaling");}
        if(o.stop&&o.stop(x)){out.status=-2;break;}
        if(out.status!=0||radius<=0.)break;
    }
    out.x=std::move(x);out.cost=cost;out.residual_calls=fun.calls;out.converged=out.status>0;
    return out;
}
LeastSquaresResult bounded_least_squares(const Residual& residual,
    std::span<const double> initial,std::span<const double> lower,
    std::span<const double> upper,int max_evaluations) {
    LeastSquaresOptions options;options.max_evaluations=max_evaluations;
    return bounded_least_squares(residual,initial,lower,upper,options);
}
} // namespace bike::numerics
