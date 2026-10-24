#include "meijaard_reference.hpp"

#include <cmath>
#include <stdexcept>

namespace spatial_tests {
WhippleMatrices reference_matrices(const bike::model::WhippleParameters& p) {
    // Independent arithmetic transcription of Appendix A, equations A1-A27,
    // Meijaard et al. (2007), DOI 10.1098/rspa.2007.1857. Paper coordinates
    // are retained here; this code shares no topology/inertia computation
    // with the native four-body builder.
    if(!(p.w_m>0.)||!(p.rR_m>0.)||!(p.rF_m>0.))
        throw std::invalid_argument("Meijaard reference requires positive radii/wheelbase");
    const double mt=p.mR_kg+p.mB_kg+p.mH_kg+p.mF_kg;
    const double xt=(p.mB_kg*p.xB_m+p.mH_kg*p.xH_m+p.mF_kg*p.w_m)/mt;
    const double zt=(-p.mR_kg*p.rR_m+p.mB_kg*p.zB_m+p.mH_kg*p.zH_m-p.mF_kg*p.rF_m)/mt;
    const double itxx=p.IRxx+p.IBxx+p.IHxx+p.IFxx+p.mR_kg*p.rR_m*p.rR_m+
        p.mB_kg*p.zB_m*p.zB_m+p.mH_kg*p.zH_m*p.zH_m+p.mF_kg*p.rF_m*p.rF_m;
    const double itxz=p.IBxz+p.IHxz-p.mB_kg*p.xB_m*p.zB_m-p.mH_kg*p.xH_m*p.zH_m+p.mF_kg*p.w_m*p.rF_m;
    const double itzz=p.IRxx+p.IBzz+p.IHzz+p.IFxx+p.mB_kg*p.xB_m*p.xB_m+
        p.mH_kg*p.xH_m*p.xH_m+p.mF_kg*p.w_m*p.w_m;
    const double ma=p.mH_kg+p.mF_kg;
    const double xa=(p.mH_kg*p.xH_m+p.mF_kg*p.w_m)/ma;
    const double za=(p.mH_kg*p.zH_m-p.mF_kg*p.rF_m)/ma;
    const double iaxx=p.IHxx+p.IFxx+p.mH_kg*(p.zH_m-za)*(p.zH_m-za)+p.mF_kg*(p.rF_m+za)*(p.rF_m+za);
    const double iaxz=p.IHxz-p.mH_kg*(p.xH_m-xa)*(p.zH_m-za)+p.mF_kg*(p.w_m-xa)*(p.rF_m+za);
    const double iazz=p.IHzz+p.IFxx+p.mH_kg*(p.xH_m-xa)*(p.xH_m-xa)+p.mF_kg*(p.w_m-xa)*(p.w_m-xa);
    const double sl=std::sin(p.lambda_rad),cl=std::cos(p.lambda_rad);
    const double ua=(xa-p.w_m-p.c_m)*cl-za*sl;
    const double iall=ma*ua*ua+iaxx*sl*sl+2.*iaxz*sl*cl+iazz*cl*cl;
    const double ialx=-ma*ua*za+iaxx*sl+iaxz*cl;
    const double ialz=ma*ua*xa+iaxz*sl+iazz*cl;
    const double mu=p.c_m/p.w_m*cl;
    const double sr=p.IRyy/p.rR_m,sf=p.IFyy/p.rF_m,st=sr+sf;
    const double sa=ma*ua+mu*mt*xt;
    WhippleMatrices r;
    r.M={itxx,ialx+mu*itxz,ialx+mu*itxz,iall+2.*mu*ialz+mu*mu*itzz};
    r.K0={mt*zt,-sa,-sa,-sa*sl};
    r.K2={0.,(st-mt*zt)/p.w_m*cl,0.,(sa+sf*sl)/p.w_m*cl};
    r.C1={0.,mu*st+sf*cl+itxz/p.w_m*cl-mu*mt*zt,-(mu*st+sf*cl),
        ialz/p.w_m*cl+mu*(sa+itzz/p.w_m*cl)};
    for(const auto& matrix:{r.M,r.C1,r.K0,r.K2})for(double value:matrix)
        if(!std::isfinite(value))throw std::invalid_argument("Meijaard reference: nonfinite matrix");
    return r;
}
} // namespace spatial_tests
