#include "pneumatic3d/carcass.hpp"
#include "pneumatic3d/element_work.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace bike::pneumatic3d {
namespace {
bool nonnegative(double x) noexcept { return std::isfinite(x) && x>=0.; }
struct MaxwellStep { double q=0., loss_density=0.; };
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
MaxwellStep maxwell(double q0, double rate, double k, double tau, double dt) noexcept {
    if (!(k>0.)) return {};
    if (!(dt>0.)) return {.q=q0,.loss_density=0.};
    const double x=dt/tau, e=std::exp(-x);
    const double one_minus_e=-std::expm1(-x);
    const double one_minus_e2=-std::expm1(-2.*x);
    double q=0., loss=0.;
    if (x<.01) {
        // Integrate in q0 and the elastic strain increment, not q_ss. The
        // three q_ss terms of the usual closed form cancel to O(x^3) when
        // dt << tau, erasing (or inventing) dissipation in an ordinary ramp.
        const double increment=k*(rate*dt);
        const double f=x>0. ? one_minus_e/x : 1.;
        const double a=x>0. ? .5*one_minus_e2/x : 1.;
        // [x-2(1-exp(-x))+(1-exp(-2x))/2]/x^3. The remainder
        // after x^7 is below 3e-21 throughout this branch.
        const double c=1./3.+x*(-1./4.+x*(7./60.+x*(-1./24.+
            x*(31./2520.+x*(-1./320.+x*(127./181440.-x*17./120960.))))));
        q=q0*e+increment*f;
        loss=(x/k)*(q0*q0*a+q0*increment*f*f+increment*increment*c);
    } else {
        const double qss=k*tau*rate;
        // Retain the small steady stress after complete relaxation: the
        // equivalent q0+(qss-q0)*(1-e) subtracts it away when e rounds to 0.
        q=q0*e+qss*one_minus_e;
        const double a=.5*one_minus_e2;
        const double b=one_minus_e*one_minus_e;
        const double c=x-one_minus_e-.5*one_minus_e*one_minus_e;
        loss=(q0*q0*a+q0*qss*b+qss*qss*c)/k;
    }
    // Never turn a nonfinite quadratic into zero through std::max(0, NaN).
    // The caller rejects it before changing any material state.
    return {.q=q,.loss_density=std::isfinite(loss) ? std::max(0.,loss) : loss};
}
}

CoreStatus detail::prepare_carcass(const CarcassSpec3d& spec,
    std::span<const ContactElement3d> elements, double dt,
    const CarcassState3d& state, std::span<NormalElement3d> out, double& pressure) noexcept {
    const auto cells=state.deflection_m.size();
    if (out.size()!=elements.size() || state.maxwell_q_n_per_m2.size()!=cells ||
        state.previous_penetration_m.size()!=cells || state.active.size()!=cells ||
        !std::isfinite(state.material_phase_rad) || !nonnegative(state.stored_energy_j) ||
        !nonnegative(state.dissipated_j) ||
        !nonnegative(dt) || !nonnegative(spec.pressure_pa) ||
        !nonnegative(spec.k_area_n_per_m3) || !nonnegative(spec.maxwell_k_n_per_m3) ||
        !nonnegative(spec.rim_clearance_m) || !nonnegative(spec.rim_stiffness_n_per_m3) ||
        (spec.maxwell_k_n_per_m3>0. && !(std::isfinite(spec.maxwell_tau_s)&&spec.maxwell_tau_s>0.)) ||
        (!spec.constant_pressure && !(std::isfinite(spec.initial_volume_m3)&&spec.initial_volume_m3>0.)))
        return CoreStatus::invalid_input;
    for (std::size_t j=0;j<cells;++j)
        if (!nonnegative(state.deflection_m[j]) || !std::isfinite(state.maxwell_q_n_per_m2[j]) ||
            !nonnegative(state.previous_penetration_m[j]) || state.active[j]>1)
            return CoreStatus::invalid_input;
    double displaced_volume=0.;
    for (const auto& e:elements) {
        if (e.material_cell>=cells || !nonnegative(e.geometric_area_m2) ||
            !std::isfinite(e.penetration_m)) return CoreStatus::invalid_input;
        if (e.valid && e.penetration_m>0.) displaced_volume+=e.penetration_m*e.geometric_area_m2;
    }
    if (!std::isfinite(displaced_volume) ||
        (!spec.constant_pressure && displaced_volume>=spec.initial_volume_m3))
        return CoreStatus::invalid_geometry;
    pressure=spec.constant_pressure ? spec.pressure_pa :
        spec.pressure_pa*spec.initial_volume_m3/(spec.initial_volume_m3-displaced_volume);
    if (!std::isfinite(pressure)) return CoreStatus::invalid_geometry;
    return CoreStatus::ok;
}

CoreStatus detail::evaluate_carcass_range(const CarcassSpec3d& spec,
    std::span<const ContactElement3d> elements, double dt, const CarcassState3d& state,
    std::span<NormalElement3d> out, double pressure, std::size_t begin, std::size_t end) noexcept {
    if(begin>end || end>elements.size() || out.size()!=elements.size())return CoreStatus::invalid_input;
    for (std::size_t i=begin;i<end;++i) {
        const auto& e=elements[i]; auto& n=out[i]; n={};
        n.element=static_cast<std::uint32_t>(i);
        if (!e.valid || !(e.penetration_m>0.) || !(e.geometric_area_m2>0.)) continue;
        const auto j=e.material_cell;
        const double area=e.geometric_area_m2, d=e.penetration_m;
        const double rate=dt>0. ? (d-state.previous_penetration_m[j])/dt : 0.;
        const auto branch=maxwell(state.maxwell_q_n_per_m2[j],rate,
            spec.maxwell_k_n_per_m3,spec.maxwell_tau_s,dt);
        if (!std::isfinite(branch.q) || !std::isfinite(branch.loss_density))
            return CoreStatus::invalid_geometry;
        n.pressure_force_n=pressure*area;
        n.elastic_force_n=spec.k_area_n_per_m3*d*area;
        n.rate_force_n=branch.q*area;
        const double rim=std::max(0.,d-spec.rim_clearance_m);
        n.rim_force_n=spec.rim_stiffness_n_per_m3*rim*area;
        n.normal_load_n=std::max(0.,n.pressure_force_n+n.elastic_force_n+n.rate_force_n+n.rim_force_n);
        n.stored_energy_j=.5*(spec.k_area_n_per_m3*d*d+
            spec.rim_stiffness_n_per_m3*rim*rim)*area;
        if (spec.maxwell_k_n_per_m3>0.)
            n.stored_energy_j+=.5*branch.q*branch.q/spec.maxwell_k_n_per_m3*area;
        n.dissipated_work_j=branch.loss_density*area;
        n.active=true;
        if (!std::isfinite(n.pressure_force_n) || !std::isfinite(n.elastic_force_n) ||
            !std::isfinite(n.rate_force_n) || !std::isfinite(n.rim_force_n) ||
            !std::isfinite(n.normal_load_n) || !std::isfinite(n.stored_energy_j) ||
            !std::isfinite(n.dissipated_work_j)) return CoreStatus::invalid_geometry;
    }
    return CoreStatus::ok;
}

CoreStatus detail::finish_carcass(const CarcassSpec3d& spec,
    std::span<const ContactElement3d> elements, double dt, bool advance,
    CarcassState3d& state, std::span<const NormalElement3d> out) noexcept {
    const auto cells=state.deflection_m.size();
    double stored=0., lost=0.;
    for (std::size_t i=0;i<elements.size();++i)if(out[i].active) {
        stored+=out[i].stored_energy_j; lost+=out[i].dissipated_work_j;
        if (!std::isfinite(stored) || !std::isfinite(lost))return CoreStatus::invalid_geometry;
    }
    if (!std::isfinite(state.dissipated_j+lost)) return CoreStatus::invalid_geometry;
    if (advance && dt>0.) {
        // Inputs carry absolute material-cell identities. Do not rotate this
        // array again: doing so would advect the same material twice. The
        // model owns the exact continuous phase (not available in this API).
        std::ranges::fill(state.active,std::uint8_t{0});
        const double decay=spec.maxwell_k_n_per_m3>0. ? std::exp(-dt/spec.maxwell_tau_s) : 0.;
        for (std::size_t j=0;j<cells;++j) {
            state.deflection_m[j]=0.; state.previous_penetration_m[j]=0.;
            state.maxwell_q_n_per_m2[j]*=decay;
        }
        for (std::size_t i=0;i<elements.size();++i) if (out[i].active) {
            const auto j=elements[i].material_cell;
            state.active[j]=1;
            state.deflection_m[j]=elements[i].penetration_m;
            state.previous_penetration_m[j]=elements[i].penetration_m;
            state.maxwell_q_n_per_m2[j]=out[i].rate_force_n/elements[i].geometric_area_m2;
        }
        state.stored_energy_j=stored; state.dissipated_j+=lost;
    }
    return CoreStatus::ok;
}
CoreStatus update_carcass(const CarcassSpec3d& spec,
    std::span<const ContactElement3d> elements, double dt, bool advance,
    CarcassState3d& state, std::span<NormalElement3d> out) noexcept {
    double pressure=0.;
    auto result=detail::prepare_carcass(spec,elements,dt,state,out,pressure);
    if(result!=CoreStatus::ok)return result;
    result=detail::evaluate_carcass_range(spec,elements,dt,state,out,pressure,0,elements.size());
    if(result!=CoreStatus::ok)return result;
    return detail::finish_carcass(spec,elements,dt,advance,state,out);
}
} // namespace bike::pneumatic3d
