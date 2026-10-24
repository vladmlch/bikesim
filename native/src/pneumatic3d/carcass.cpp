#include "pneumatic3d/carcass.hpp"
#include "pneumatic3d/element_work.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace bike::pneumatic3d {
namespace {
bool nonnegative(double x) noexcept { return std::isfinite(x) && x>=0.; }
struct MaxwellStep { double q=0., loss_density=0.; };
MaxwellStep maxwell(double q0, double rate, double k, double tau, double dt) noexcept {
    if (!(k>0.)) return {};
    if (!(dt>0.)) return {q0,0.};
    const double one_minus_e=-std::expm1(-dt/tau);
    const double one_minus_e2=-std::expm1(-2.*dt/tau);
    const double qss=k*tau*rate, a=q0-qss;
    const double q=q0+(qss-q0)*one_minus_e;
    // Exact integral int(q(t)^2/(k*tau),0,dt). A tiny roundoff-negative
    // integral is canonicalized; no negative dissipation is exported.
    const double integral=qss*qss*dt+2.*qss*a*tau*one_minus_e+
                          .5*a*a*tau*one_minus_e2;
    return {q,std::max(0.,integral/(k*tau))};
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
        std::fill(state.active.begin(),state.active.end(),std::uint8_t{0});
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
