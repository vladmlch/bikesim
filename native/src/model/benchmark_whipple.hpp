#pragma once

#include "builder.hpp"
#include <numbers>

namespace bike::model {

// Meijaard et al. (2007), Table 1. Source coordinates use z down;
// make_whipple_request is responsible for the conversion to the z-up model.
struct WhippleParameters {
    double w_m = 1.02, c_m = .08, lambda_rad = std::numbers::pi / 10.;
    double g_mps2 = 9.81, v_mps = 0.;
    double rR_m = .3, mR_kg = 2., IRxx = .0603, IRyy = .12;
    double xB_m = .3, zB_m = -.9, mB_kg = 85.;
    double IBxx = 9.2, IBxz = 2.4, IByy = 11., IBzz = 2.8;
    double xH_m = .9, zH_m = -.7, mH_kg = 4.;
    double IHxx = .05892, IHxz = -.00756, IHyy = .06, IHzz = .00708;
    double rF_m = .35, mF_kg = 3., IFxx = .1405, IFyy = .28;
};
BuildRequest make_whipple_request(const WhippleParameters&);

} // namespace bike::model
