#include "freehub.hpp"
#include "../writers/pyfloat.hpp"
#include <algorithm>

namespace drivetrain {
    void Freehub::set_state(const FreehubSnapshot &s) {
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
        const double relative = finite(pc - pw, "freehub relative angle");
        const double boundary = state_.boundary ? std::min(*state_.boundary, relative) : relative;
        const double deflection = std::max(0., relative - boundary);
        const double energy = finite(.5 * k_ * pyfloat::pow(deflection, 2.), "freehub energy");
        const double torque = finite(std::max(0., k_ * deflection + c_ * (wc - ww)), "freehub torque");
        state_ = {.boundary = boundary, .energy_j = energy, .torque_nm = torque};
        return torque;
    }
}
