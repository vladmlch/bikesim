#include "filter.hpp"
#include <algorithm>
#include <cmath>
#include <complex>
#include <numbers>
#include <stdexcept>
#include <cstdint>
#include <limits>
#include "../writers/pyfloat.hpp"

namespace bike::numerics {
namespace {
using Complex = std::complex<double>;
// lfilter_zi uses NumPy's solve, the same ILP64 provider as the existing
// kinematics/curves.cpp seam. Do not substitute a handwritten elimination.
extern "C" void filter_dgesv_ilp64(const long* n,const long* nrhs,double* a,const long* lda,
    long* ipiv,double* b,const long* ldb,long* info) __asm__("_dgesv$NEWLAPACK$ILP64");
static_assert(sizeof(long)==8);
std::vector<Complex> polynomial(const std::vector<Complex>& roots) {
    std::vector<Complex> values(1,1.);
    for (const auto root : roots) {
        std::vector<Complex> next(values.size()+1,0.);
        for (std::size_t i=0;i<values.size();++i) { next[i]+=values[i]; next[i+1]-=root*values[i]; }
        values=std::move(next);
    }
    return values;
}
std::pair<std::vector<double>,std::vector<double>> butter(int order,double normalized) {
    // scipy.signal.buttap -> lp2lp_zpk -> bilinear_zpk -> zpk2tf.
    const double warped = 4. * std::tan(std::numbers::pi * normalized / 2.);
    std::vector<Complex> poles, zeros(static_cast<std::size_t>(order),-1.);
    Complex denominator=1.;
    for (int i=0;i<order;++i) {
        const auto m=-static_cast<std::int64_t>(order)+1+2*static_cast<std::int64_t>(i);
        const Complex pole=-std::exp(Complex(0.,std::numbers::pi*static_cast<double>(m)/(2.*static_cast<double>(order))))*warped;
        denominator *= 4.-pole;
        poles.push_back((4.+pole)/(4.-pole));
    }
    const double gain = pyfloat::float_pow(warped,static_cast<double>(order))*(1./denominator).real();
    const auto numerator=polynomial(zeros), denom=polynomial(poles);
    std::vector<double> b(numerator.size()),a(denom.size());
    for (std::size_t i=0;i<b.size();++i) b[i]=numerator[i].real()*gain;
    for (std::size_t i=0;i<a.size();++i) a[i]=denom[i].real();
    return {std::move(b),std::move(a)};
}
std::vector<double> initial_state(const std::vector<double>& b,const std::vector<double>& a) {
    // lfilter_zi: (I - companion(a).T) zi = b[1:] - a[1:]*b[0].
    const std::size_t n=a.size()-1;
    std::vector<double> matrix(n*n),rhs(n);
    for (std::size_t i=0;i<n;++i) {
        // Column-major layout for the Fortran solve, same I-companion.T.
        matrix[i*n+i]=1.; matrix[i]+=a[i+1];
        if (i+1<n) matrix[(i+1)*n+i]-=1.;
        rhs[i]=b[i+1]-a[i+1]*b[0];
    }
    std::vector<long> pivots(n);const auto width=static_cast<long>(n);const long nrhs=1;long info=0;
    filter_dgesv_ilp64(&width,&nrhs,matrix.data(),&width,pivots.data(),rhs.data(),&width,&info);
    if(info!=0)throw std::invalid_argument("legacy Butterworth initial state is singular");
    return rhs;
}
void lfilter(std::vector<double>& samples,const std::vector<double>& b,
        const std::vector<double>& a,const std::vector<double>& zi) {
    auto state=zi;
    for (auto& x:state) x*=samples.front();
    for (auto& sample:samples) {
        const double input=sample, output=b[0]*input+state[0];
        for (std::size_t j=0;j+1<state.size();++j) state[j]=state[j+1]+b[j+1]*input-a[j+1]*output;
        state.back()=b.back()*input-a.back()*output; sample=output;
    }
}
}
std::vector<double> legacy_lowpass(std::span<const double> samples,double dt,double cutoff,int order) {
    if(dt==0.)throw std::invalid_argument("legacy lowpass divides by a zero sample interval");
    const double nyquist=.5/dt;
    const auto requested_padding=3*(static_cast<std::int64_t>(order)+1);
    if ((requested_padding>=0&&samples.size()<=static_cast<std::uint64_t>(requested_padding))||cutoff>=nyquist)
        return {samples.begin(),samples.end()};
    // The source's short/Nyquist fallback happens before scipy designs a
    // filter, including for a nonpositive cutoff or order on a short input.
    if(!std::isfinite(dt)||dt<=0.||!std::isfinite(cutoff)||cutoff<=0.||order<=0)
        throw std::invalid_argument("legacy Butterworth requires positive finite interval/cutoff and order");
    const auto padding=static_cast<std::size_t>(requested_padding);
    if(padding>(std::numeric_limits<std::size_t>::max()-samples.size())/2)
        throw std::length_error("legacy filter padding exceeds storage range");
    const auto [b,a]=butter(order,cutoff/nyquist);
    const auto zi=initial_state(b,a);
    std::vector<double> extended;
    extended.reserve(samples.size()+2*padding);
    for (std::size_t i=padding;i>0;--i) extended.push_back(2.*samples.front()-samples[i]);
    extended.insert(extended.end(),samples.begin(),samples.end());
    for (std::size_t i=1;i<=padding;++i) extended.push_back(2.*samples.back()-samples[samples.size()-1-i]);
    lfilter(extended,b,a,zi);
    std::reverse(extended.begin(),extended.end());
    lfilter(extended,b,a,zi);
    std::reverse(extended.begin(),extended.end());
    return {extended.begin()+static_cast<std::ptrdiff_t>(padding),extended.end()-static_cast<std::ptrdiff_t>(padding)};
}
std::vector<double> legacy_ride_filter(std::span<const double> samples,double fs) {
    if (!std::isfinite(fs)||fs<=0.) throw std::invalid_argument("legacy sample rate must be finite and positive");
    return legacy_lowpass(samples,1./fs);
}
} // namespace bike::numerics
