#include "support/migration_test.hpp"
#include "support/tyre_test_helpers.hpp"
#include "pneumatic3d/carcass.hpp"

#include <array>
#include <cmath>
#include <limits>

int main() { return bt::run([] {
    using namespace bike::pneumatic3d;
    using namespace spatial_tests::tyre;
    CarcassSpec3d spec{.pressure_pa=0.,.k_area_n_per_m3=1e6,
        .maxwell_k_n_per_m3=0.,.maxwell_tau_s=.02,.rim_clearance_m=.04,
        .rim_stiffness_n_per_m3=1e8,.initial_volume_m3=.01,.constant_pressure=true,
        .provenance_status="synthetic"};
    std::array<ContactElement3d,1> elements{element(0,.001)};
    std::array<NormalElement3d,1> out{};
    auto s=carcass_state(1);
    bt::require(update_carcass(spec,elements,.001,true,s,out)==CoreStatus::ok,"synthetic finite-volume cell");
    bt::require(std::abs(out[0].normal_load_n-10.)<=1e-10,"area-weighted normal force is 10 N");
    bt::require(std::abs(out[0].stored_energy_j-.005)<=1e-12,"cell stores 0.005 J");
    spec.pressure_pa=150000.;
    for (bool constant:{false,true}) {
        spec.constant_pressure=constant;
        const auto potential=[&](double d) {
            const double volume=d*elements[0].geometric_area_m2;
            const double pressure=constant ? spec.pressure_pa*volume :
                -spec.pressure_pa*spec.initial_volume_m3*std::log1p(-volume/spec.initial_volume_m3);
            return pressure+.5*spec.k_area_n_per_m3*d*d*elements[0].geometric_area_m2;
        };
        bt::require(update_carcass(spec,elements,0.,false,s,out)==CoreStatus::ok,"constant/isothermal pressure probe");
        const double h=1e-6,fd=(potential(.001+h)-potential(.001-h))/(2.*h);
        bt::require(std::abs(fd-out[0].normal_load_n)<=1e-5*std::max(1.,std::abs(fd)),"normal force differentiates pressure plus carcass potential");
    }
    const auto before=bytes(s);
    bt::require(update_carcass(spec,elements,.01,false,s,out)==CoreStatus::ok && bytes(s)==before,"probe leaves every committed state bit unchanged");
    bt::require(update_carcass(spec,elements,0.,true,s,out)==CoreStatus::ok && bytes(s)==before,"zero interval never advances memory");
    elements[0].material_cell=4;
    bt::require(update_carcass(spec,elements,.01,true,s,out)==CoreStatus::invalid_input && bytes(s)==before,"bad material index fails before mutation");
    elements[0].material_cell=0;
    spec.pressure_pa=0.;spec.constant_pressure=true;spec.maxwell_k_n_per_m3=2e5;
    const auto ramp=[&](double dt) {
        auto state=carcass_state(1); std::array<NormalElement3d,1> force{};
        const unsigned steps=static_cast<unsigned>(std::llround(.1/dt));
        for(unsigned i=1;i<=steps;++i) {
            elements[0].penetration_m=.01*static_cast<double>(i)*dt;
            bt::require(update_carcass(spec,elements,dt,true,state,force)==CoreStatus::ok,"Maxwell ramp accepted");
            bt::require(force[0].dissipated_work_j>=0.,"Maxwell relaxation has nonnegative exact loss");
        }
        const double q=spec.maxwell_k_n_per_m3*spec.maxwell_tau_s*.01*(-std::expm1(-.1/spec.maxwell_tau_s));
        bt::require(std::abs(state.maxwell_q_n_per_m2[0]-q)<=1e-9,"linear ramp has exact Maxwell primitive");
        return state.maxwell_q_n_per_m2[0];
    };
    bt::require(std::abs(ramp(.001)-ramp(.0005))<=1e-9,"dt-halving constant-rate Maxwell agrees");
    elements[0].valid=false;
    bt::require(update_carcass(spec,elements,.001,true,s,out)==CoreStatus::ok && !out[0].active && out[0].normal_load_n==0.,"invalid geometry cannot create a maximal force");

    auto overflowing=carcass_state(1);
    overflowing.maxwell_q_n_per_m2[0]=1e154;
    overflowing.dissipated_j=std::numeric_limits<double>::max();
    elements[0]=element(0,.001,1.);
    spec.maxwell_k_n_per_m3=1.;spec.k_area_n_per_m3=0.;
    const auto overflow_before=bytes(overflowing);
    bt::require(update_carcass(spec,elements,.001,true,overflowing,out)==CoreStatus::invalid_geometry &&
                bytes(overflowing)==overflow_before,"finite branch losses cannot overflow the cumulative ledger during commit");
}); }
