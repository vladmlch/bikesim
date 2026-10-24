#include "benchmark_linalg.hpp"
#include "../../../src/numerics/dense.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace spatial_tests {
std::vector<std::complex<double>> quadratic_eigenvalues(const WhippleMatrices& q,double speed,double gravity) {
    using C=std::complex<long double>;
    if(!std::isfinite(speed)||!std::isfinite(gravity))throw std::invalid_argument("Whipple eigenvalues: finite speed/gravity required");
    // Convolution of the two determinant products gives independent quartic
    // coefficients; no matrix solver or eigenvalues from the paper are used.
    std::array<std::array<long double,3>,4> entry{};
    for(std::size_t i=0;i<4;++i)entry[i]={static_cast<long double>(gravity)*q.K0[i]+static_cast<long double>(speed)*speed*q.K2[i],
        static_cast<long double>(speed)*q.C1[i],q.M[i]};
    std::array<long double,5> p{};
    for(std::size_t i=0;i<3;++i)for(std::size_t j=0;j<3;++j)p[i+j]+=entry[0][i]*entry[3][j]-entry[1][i]*entry[2][j];
    if(!std::isfinite(p[4])||p[4]==0.)throw std::invalid_argument("Whipple eigenvalues: singular mass determinant");
    const long double lead=p[4];for(auto& x:p)x/=lead;
    // Scale lambda to keep the Cauchy bound and simultaneous iterations well
    // conditioned over the benchmark speed range, including a near-zero root.
    long double scale=1.;for(std::size_t i=0;i<4;++i)scale=std::max(scale,std::pow(std::abs(p[i]),1.L/static_cast<long double>(4-i)));
    for(std::size_t i=0;i<4;++i)p[i]/=std::pow(scale,static_cast<int>(4-i));
    const auto value=[&](C z){C y=p[4];for(int i=3;i>=0;--i)y=y*z+p[static_cast<std::size_t>(i)];return y;};
    const auto derivative=[&](C z){return ((4.L*z+3.L*p[3])*z+2.L*p[2])*z+p[1];};
    std::array<C,4> roots;
    for(std::size_t i=0;i<4;++i)roots[i]=std::polar(1.2L,(static_cast<long double>(i)+.25L)*std::numbers::pi_v<long double>/2.L);
    bool converged=false;
    for(int step=0;step<2000;++step) {
        const auto previous=roots;long double largest=0.;
        for(std::size_t i=0;i<4;++i) {
            C denom{1.,0.};for(std::size_t j=0;j<4;++j)if(i!=j)denom*=previous[i]-previous[j];
            if(std::abs(denom)<1e-30L)denom+=C{1e-28L,1e-28L};
            const C update=value(previous[i])/denom;roots[i]=previous[i]-update;
            largest=std::max(largest,std::abs(update));
        }
        if(largest<1e-18L){converged=true;break;}
    }
    long double residual=0.;
    for(auto& z:roots) {
        for(int k=0;k<4;++k){const auto d=derivative(z);if(std::abs(d)>1e-25L)z-=value(z)/d;}
        residual=std::max(residual,std::abs(value(z)));
    }
    if(!converged&&residual>1e-13L)throw std::runtime_error("Whipple quartic root iteration did not converge");
    std::vector<std::complex<double>> result;result.reserve(4);
    for(const auto z:roots)result.emplace_back(static_cast<double>(z.real()*scale),static_cast<double>(z.imag()*scale));
    std::ranges::sort(result,[](const auto& a,const auto& b){return a.real()==b.real()?a.imag()<b.imag():a.real()<b.real();});
    return result;
}
Kkt solve_kkt(std::span<const double> mass,std::span<const double> bias,const RollingConstraints& constraints,int nv) {
    using namespace bike::numerics::dense;
    if(nv<=0||constraints.rows<=0)throw std::invalid_argument("Whipple KKT: positive dimensions required");
    const auto n=static_cast<std::size_t>(nv),r=static_cast<std::size_t>(constraints.rows),all=n+r;
    if(mass.size()!=n*n||bias.size()!=n||constraints.A.size()!=r*n||constraints.Adot_v.size()!=r||
        !finite(mass)||!finite(bias)||!finite(constraints.A)||!finite(constraints.Adot_v))
        throw std::invalid_argument("Whipple KKT: finite row-major arrays with matching dimensions required");
    auto a=zeros(all,all);Vector rhs(all,0.);
    for(std::size_t i=0;i<n;++i) {
        rhs[i]=-bias[i];for(std::size_t j=0;j<n;++j)a.values[i*all+j]=mass[i*n+j];
        for(std::size_t j=0;j<r;++j)a.values[i*all+n+j]=-constraints.A[j*n+i];
    }
    for(std::size_t i=0;i<r;++i) {
        rhs[n+i]=-constraints.Adot_v[i];for(std::size_t j=0;j<n;++j)a.values[(n+i)*all+j]=constraints.A[i*n+j];
    }
    const auto solution=solve(a,rhs);const auto product=multiply(a,solution);
    double residual=0.,matrix_norm=0.;
    for(std::size_t i=0;i<all;++i) {
        residual=std::max(residual,std::abs(product[i]-rhs[i]));double sum=0.;
        for(std::size_t j=0;j<all;++j)sum+=std::abs(a.values[i*all+j]);matrix_norm=std::max(matrix_norm,sum);
    }
    double sol_norm=0.,rhs_norm=0.;for(double v:solution)sol_norm=std::max(sol_norm,std::abs(v));
    for(double v:rhs)rhs_norm=std::max(rhs_norm,std::abs(v));
    // Successful partial-pivot square LU has no rank-hiding pseudoinverse.
    // A singular constraint inventory throws; no epsilon mass regularization.
    Kkt out;out.vdot.assign(solution.begin(),solution.begin()+nv);out.lambda.assign(solution.begin()+nv,solution.end());
    out.rank=static_cast<int>(all);out.relative_residual=residual/std::max(1.,matrix_norm*sol_norm+rhs_norm);return out;
}
} // namespace spatial_tests
