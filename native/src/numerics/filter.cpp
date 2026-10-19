#include "filter.hpp"
#include <algorithm>
#include <cmath>
#include <complex>
#include <numbers>
#include <stdexcept>
#include <cstdint>
#include <limits>
#include <utility>
#include "../writers/pyfloat.hpp"
#include <array>

namespace bike::numerics {
namespace {
using Complex = std::complex<double>;
// lfilter_zi uses NumPy's solve, the same ILP64 provider as the existing
// kinematics/curves.cpp seam. Do not substitute a handwritten elimination.
extern "C" void filter_dgesv_ilp64(const long* n,const long* nrhs,double* a,const long* lda,
    long* ipiv,double* b,const long* ldb,long* info) __asm__("_dgesv$NEWLAPACK$ILP64");
static_assert(sizeof(long)==8);
// np.convolve's complex inner loop is CDOUBLE_dot, which forwards contiguous
// windows to cblas_zdotu_sub$NEWLAPACK$ILP64 — the same Accelerate entry point
// NumPy binds (nm -u on _multiarray_umath confirms the ILP64 spelling).
extern "C" void filter_zdotu(long n,const void* x,long incx,
    const void* y,long incy,void* out) __asm__("_cblas_zdotu_sub$NEWLAPACK$ILP64");
// npy complex multiply: the oracle binary contracts `ar*br - ai*bi` into
// fma(ar,br,-(ai*bi)) and `ar*bi + ai*br` into fma(ar,bi,ai*br). Spelling
// the fmas is required because the project build pins -ffp-contract=off.
[[nodiscard]] Complex cmul(Complex a, Complex b) {
    return {std::fma(a.real(), b.real(), -(a.imag()*b.imag())),
            std::fma(a.real(), b.imag(), a.imag()*b.real())};
}
// npy_cdivide (npy_math_complex.c.src): Smith's guarded division with the
// same contractions the oracle's object code performs.
[[nodiscard]] Complex cdiv(Complex a, Complex b) {
    const double ar=a.real(), ai=a.imag(), br=b.real(), bi=b.imag();
    if (std::abs(br) >= std::abs(bi)) {
        if (br == 0. && bi == 0.) return {ar/0., ai/0.};
        const double rat = bi/br, scl = 1./std::fma(bi,rat,br);
        return {std::fma(ai,rat,ar)*scl, std::fma(-ar,rat,ai)*scl};
    }
    const double rat = br/bi, scl = 1./std::fma(br,rat,bi);
    return {std::fma(ar,rat,ai)*scl, std::fma(-ai,rat,ar)*scl};
}
// np.poly: values = convolve(values, [1,-root]) per root. convolve forwards to
// _pyarray_correlate with kernel [-root,1] (the reversed taps); the correlation
// dot for each output element is cblas_zdotu_sub over (window, kernel).
std::vector<Complex> polynomial(const std::vector<Complex>& roots) {
    std::vector<Complex> values(1,1.);
    for (const auto root : roots) {
        const std::array kernel{Complex{-root},Complex{1.}};
        std::vector<Complex> next(values.size()+1);
        const auto tail_kernel = std::span(kernel).subspan(1);
        filter_zdotu(1,values.data(),1,tail_kernel.data(),1,next.data());
        for (std::size_t i=0;i+2<next.size();++i) {
            const auto window = std::span(values).subspan(i,2);
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay) decay is the designed wire idiom here
            filter_zdotu(2,window.data(),1,kernel.data(),1,
                std::span(next).subspan(i+1,1).data());
        }
        filter_zdotu(1,std::span(values).subspan(values.size()-1,1).data(),1,
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay) decay is the designed wire idiom here
            kernel.data(),1,std::span(next).subspan(next.size()-1,1).data());
        values=std::move(next);
    }
    return values;
}
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
std::pair<std::vector<double>,std::vector<double>> butter(int order,double normalized) {
    // scipy.signal.buttap -> lp2lp_zpk -> bilinear_zpk -> zpk2tf.
    const double warped = 4. * std::tan(std::numbers::pi * normalized / 2.);
    std::vector<Complex> poles, zeros(static_cast<std::size_t>(order),-1.);
    Complex denominator=1.;
    for (int i=0;i<order;++i) {
        const auto m=-static_cast<std::int64_t>(order)+1+2*static_cast<std::int64_t>(i);
        const Complex pole=-std::exp(Complex(0.,std::numbers::pi*static_cast<double>(m)/(2.*static_cast<double>(order))))*warped;
        denominator = cmul(denominator,4.-pole);
        poles.push_back(cdiv(4.+pole,4.-pole));
    }
    const double gain = pyfloat::float_pow(warped,static_cast<double>(order))*cdiv(1.,denominator).real();
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
        // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
        const std::vector<double>& a,const std::vector<double>& zi) {
    auto state=zi;
    for (auto& x:state) x*=samples.front();
    for (auto& sample:samples) {
        const double input=sample, output=std::fma(b[0],input,state[0]);
        for (std::size_t j=0;j+1<state.size();++j)
            state[j]=std::fma(-a[j+1],output,std::fma(b[j+1],input,state[j+1]));
        state.back()=std::fma(b.back(),input,-(a.back()*output)); sample=output;
    }
}
}
std::vector<double> legacy_lowpass(std::span<const double> samples,double dt,double cutoff,int order) {
    if(dt==0.)throw std::invalid_argument("legacy lowpass divides by a zero sample interval");
    const double nyquist=.5/dt;
    const auto requested_padding=3*(static_cast<std::int64_t>(order)+1);
    if ((requested_padding>=0&&std::cmp_less_equal(samples.size(),requested_padding))||cutoff>=nyquist)
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
    std::ranges::reverse(extended);
    lfilter(extended,b,a,zi);
    std::ranges::reverse(extended);
    return {extended.begin()+static_cast<std::ptrdiff_t>(padding),extended.end()-static_cast<std::ptrdiff_t>(padding)};
}
std::vector<double> legacy_ride_filter(std::span<const double> samples,double fs) {
    if (!std::isfinite(fs)||fs<=0.) throw std::invalid_argument("legacy sample rate must be finite and positive");
    return legacy_lowpass(samples,1./fs);
}
} // namespace bike::numerics
