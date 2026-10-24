#include "pneumatic3d/bristles.hpp"
#include "pneumatic3d/brush_detail.hpp"
#include "pneumatic3d/kernel_math.hpp"

#include <array>
#include <cmath>

namespace bike::pneumatic3d {
namespace {
constexpr std::size_t max_cells=1024U*15U;
struct BristleScratch {
    std::array<std::uint8_t,max_cells> touched{};
    std::array<double,detail::max_fan_elements> resolved_mu{};
    bool busy=false;
};
// Scratch only: no history survives entry, no heap allocation, no shared
// cross-thread workspace. Reentry is an explicit invalid-input status.
constinit thread_local BristleScratch scratch;
struct ScratchUse {bool& busy;explicit ScratchUse(bool& b) noexcept:busy(b){busy=true;}~ScratchUse(){busy=false;}};
CoreStatus detailed_impl(const BrushSpec3d& spec,
    std::span<const ContactElement3d> elements,std::span<const NormalElement3d> normals,
    const WheelKinematics3d& kin,double dt,bool advance,BrushState3d& state,
    std::span<PatchWrench> out,const bike::terrain::Terrain3dQueryView* terrain) noexcept {
    const std::size_t cells=state.bristle_ux_m.size();
    if (elements.size()!=normals.size() || out.size()!=elements.size() || cells==0 ||
        cells>max_cells || state.bristle_uy_m.size()!=cells ||
        state.bristle_energy_j.size()!=cells || !std::isfinite(dt) || dt<0. ||
        !std::isfinite(state.elastic_energy_j) || state.elastic_energy_j<0. ||
        !std::isfinite(state.sliding_loss_j) || state.sliding_loss_j<0. ||
        !std::isfinite(state.convective_release_j) || state.convective_release_j<0.)
        return CoreStatus::invalid_input;
    if (terrain && (!terrain->context || terrain->surface_table.empty()))
        return CoreStatus::invalid_input;
    if(scratch.busy)return CoreStatus::invalid_input;
    const ScratchUse use(scratch.busy);
    auto& touched=scratch.touched;auto& resolved_mu=scratch.resolved_mu;
    touched.fill(0);resolved_mu.fill(0.);
    if (elements.size()>resolved_mu.size()) return CoreStatus::invalid_input;
    for (std::size_t i=0;i<elements.size();++i) {
        // Unit friction belongs only to the standalone synthetic kernel.
        // Production contacts must resolve their declared material ID.
        resolved_mu[i]=terrain ? 0. : 1.;
        if (terrain && elements[i].valid && normals[i].active) {
            const auto* material=terrain->surface(elements[i].surface_id);
            if (!material) return CoreStatus::invalid_geometry;
            const auto omega=detail::add(kin.hub.angular_velocity_world_rad_s,
                detail::scale(kin.axle_world,kin.relative_spin_rad_s));
            const auto velocity=detail::add(kin.hub.linear_velocity_world_mps,
                detail::cross(omega,detail::sub(elements[i].point_world_m,kin.hub.position_world_m)));
            const double speed=std::hypot(detail::dot(velocity,elements[i].longitudinal_world),
                                          detail::dot(velocity,elements[i].lateral_world));
            const double ratio=material->stribeck_speed_mps>0. ? speed/material->stribeck_speed_mps : 0.;
            resolved_mu[i]=material->stribeck_speed_mps>0. ? material->mu_slide+
                (material->mu_peak-material->mu_slide)*std::exp(-ratio*ratio) :
                (speed==0. ? material->mu_peak : material->mu_slide);
        }
    }
    for (std::size_t j=0;j<cells;++j)
        if (!std::isfinite(state.bristle_ux_m[j]) || !std::isfinite(state.bristle_uy_m[j]) ||
            !std::isfinite(state.bristle_energy_j[j]) || state.bristle_energy_j[j]<0.)
            return CoreStatus::invalid_input;
    double stored=0.,sliding=0.,release=0.;
    for (std::size_t i=0;i<elements.size();++i) {
        const auto j=elements[i].material_cell;
        if (j>=cells || touched[j]) return CoreStatus::invalid_input;
        touched[j]=1;
        const auto result=detail::evaluate_bristle(spec,elements[i],normals[i],kin,
            resolved_mu[i]*spec.mu_x_scale,resolved_mu[i]*spec.mu_y_scale,
            state.bristle_ux_m[j],state.bristle_uy_m[j],dt);
        if (result.status!=CoreStatus::ok) return result.status;
        out[i]=result.wrench;
        stored+=result.stored_j; sliding+=result.sliding_j;
        release+=result.convective_j;
        if (!result.wrench.valid) release+=state.bristle_energy_j[j];
    }
    for (std::size_t j=0;j<cells;++j) if (!touched[j]) release+=state.bristle_energy_j[j];
    if (!std::isfinite(stored) || !std::isfinite(sliding) || !std::isfinite(release) ||
        !std::isfinite(state.sliding_loss_j+sliding) ||
        !std::isfinite(state.convective_release_j+release)) return CoreStatus::invalid_geometry;
    if (advance && dt>0.) {
        // Geometry already identifies the material under each stationary fan
        // node using signed wheel phase. Re-advecting here would count spin
        // twice. The frozen state has no former tangent bases; see C10.
        for (std::size_t j=0;j<cells;++j) if (!touched[j]) {
            state.bristle_ux_m[j]=state.bristle_uy_m[j]=state.bristle_energy_j[j]=0.;
        }
        for (std::size_t i=0;i<elements.size();++i) {
            const auto j=elements[i].material_cell;
            const auto result=detail::evaluate_bristle(spec,elements[i],normals[i],kin,
                resolved_mu[i]*spec.mu_x_scale,resolved_mu[i]*spec.mu_y_scale,
                state.bristle_ux_m[j],state.bristle_uy_m[j],dt);
            state.bristle_ux_m[j]=result.ux_m; state.bristle_uy_m[j]=result.uy_m;
            state.bristle_energy_j[j]=result.stored_j;
        }
        state.elastic_energy_j=stored; state.sliding_loss_j+=sliding;
        state.convective_release_j+=release;
    }
    return CoreStatus::ok;
}
} // namespace
CoreStatus update_detailed_brush(const BrushSpec3d& spec,
    std::span<const ContactElement3d> elements,std::span<const NormalElement3d> normals,
    const WheelKinematics3d& kin,double dt,bool advance,BrushState3d& state,
    std::span<PatchWrench> out) noexcept {
    return detailed_impl(spec,elements,normals,kin,dt,advance,state,out,nullptr);
}
namespace detail {
CoreStatus update_detailed_resolved(const BrushSpec3d& spec,
    std::span<const ContactElement3d> elements,std::span<const NormalElement3d> normals,
    const WheelKinematics3d& kin,double dt,bool advance,BrushState3d& state,
    std::span<PatchWrench> out,const bike::terrain::Terrain3dQueryView& terrain) noexcept {
    return detailed_impl(spec,elements,normals,kin,dt,advance,state,out,&terrain);
}
}
} // namespace bike::pneumatic3d
