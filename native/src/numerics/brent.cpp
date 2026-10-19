// Brent's safeguarded interpolation/bisection with scipy.optimize.brentq
// tolerance defaults. Every accepted bracket still encloses opposite signs.
#include "brent.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace bike::numerics {
BrentResult brentq(const std::function<double(double)>& f,double xa,double xb,
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
    double xtol,double rtol,std::size_t maximum) {
    if(!f || !std::isfinite(xa) || !std::isfinite(xb) || !std::isfinite(xtol) || xtol<=0. || !std::isfinite(rtol) || rtol<4.*std::numeric_limits<double>::epsilon())
        throw std::invalid_argument("brentq: finite bracket and positive tolerances required");
    BrentResult result;
    const auto evaluate=[&](double x) {
        ++result.function_calls;
        const double value=f(x);
        if(!std::isfinite(value)) throw std::invalid_argument("brentq: function must be finite");
        return value;
    };
    double xpre=xa,xcur=xb,fpre=evaluate(xa),fcur=evaluate(xb),xblk=0.,fblk=0.,spre=0.,scur=0.;
    if(fpre==0.) {result.root=xa;return result;}
    if(fcur==0.) {result.root=xb;return result;}
    if(std::signbit(fpre)==std::signbit(fcur)) throw std::invalid_argument("brentq: f(a) and f(b) must have different signs");
    for(std::size_t i=0;i<maximum;++i) {
        result.iterations=i+1;
        if(fpre!=0. && fcur!=0. && std::signbit(fpre)!=std::signbit(fcur)) {
            xblk=xpre;fblk=fpre;spre=scur=xcur-xpre;
        }
        if(std::abs(fblk)<std::abs(fcur)) {
            xpre=xcur;xcur=xblk;xblk=xpre;
            fpre=fcur;fcur=fblk;fblk=fpre;
        }
        const double delta=(xtol+rtol*std::abs(xcur))/2.;
        const double sbis=(xblk-xcur)/2.;
        if(fcur==0. || std::abs(sbis)<delta) {result.root=xcur;return result;}
        if(std::abs(spre)>delta && std::abs(fcur)<std::abs(fpre)) {
            double trial = std::numeric_limits<double>::quiet_NaN();
            if(xpre==xblk) trial=-fcur*(xcur-xpre)/(fcur-fpre);
            else {
                const double dpre=(fpre-fcur)/(xpre-xcur),dblk=(fblk-fcur)/(xblk-xcur);
                trial=-fcur*(fblk*dblk-fpre*dpre)/(dblk*dpre*(fblk-fpre));
            }
            if(2.*std::abs(trial)<std::min(std::abs(spre),3.*std::abs(sbis)-delta)) {spre=scur;scur=trial;}
            else spre=scur=sbis;
        } else spre=scur=sbis;
        xpre=xcur;fpre=fcur;
        xcur+=std::abs(scur)>delta?scur:std::copysign(delta,sbis);
        fcur=evaluate(xcur);
    }
    throw std::runtime_error("brentq: failed to converge within iteration limit");
}
} // namespace bike::numerics
