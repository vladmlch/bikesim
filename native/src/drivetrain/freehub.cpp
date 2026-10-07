#include "freehub.hpp"
#include "../writers/pyfloat.hpp"
#include <algorithm>

namespace drivetrain {
    void Freehub::set_state(FreehubSnapshot s) {
        finite_optional(s.boundary, "boundary");
        nonnegative(s.energy_j, "energy_j");
        nonnegative(s.torque_nm, "torque_nm");
        state_ = s;
    }

    double Freehub::update(double pc, double pw, double wc, double ww) {
        finite(pc, "cassette angle");
        finite(pw, "wheel angle");
        finite(wc, "cassette speed");
        finite(ww, "wheel speed");
        const double relative = validation::derived(pc - pw, "freehub relative angle");
        const double boundary = state_.boundary ? std::min(*state_.boundary, relative) : relative;
        const double deflection = std::max(0., relative - boundary);
        const double energy = validation::derived(.5 * k_ * pyfloat::pow(deflection, 2.), "freehub energy");
        const double torque = std::max(0., validation::derived(k_ * deflection + c_ * (wc - ww), "Freehub.torque"));
        state_ = {.boundary = boundary, .energy_j = energy, .torque_nm = torque};
        return torque;
    }
}
