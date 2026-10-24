#include "pneumatic3d/geometry.hpp"
#include "pneumatic3d/element_work.hpp"
#include "pneumatic3d/kernel_math.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace bike::pneumatic3d {
using namespace detail;

TorusDims tier_dims(bike::spatial::TyreTier tier) noexcept {
    return tier == bike::spatial::TyreTier::detailed ? TorusDims{256,15,1024}
                                                  : TorusDims{64,7,256};
}
std::size_t TorusGrid3d::element_count() const noexcept {
    return static_cast<std::size_t>(dims.n_theta_fan)*dims.n_phi;
}
std::size_t TorusGrid3d::material_cell_count() const noexcept {
    return static_cast<std::size_t>(dims.n_theta_material)*dims.n_phi;
}

TorusGrid3d make_torus_grid(TorusDims dims, double outer, double width) {
    const double r = .5*width, R = outer-r;
    if (!std::isfinite(R) || !std::isfinite(r) || !(R > r && r > 0.) ||
        dims.n_theta_fan == 0 || dims.n_phi == 0 || dims.n_theta_material == 0 ||
        dims.n_theta_fan > 4096 || dims.n_phi > 255 || dims.n_theta_material > 65536)
        throw std::invalid_argument("make_torus_grid: invalid dimensions or R > r > 0 domain");
    TorusGrid3d grid;
    grid.dims=dims; grid.R_m=R; grid.r_m=r;
    const double dt=(grid.theta_max_rad-grid.theta_min_rad)/dims.n_theta_fan;
    const double dp=(grid.phi_max_rad-grid.phi_min_rad)/dims.n_phi;
    grid.theta_fan.resize(dims.n_theta_fan);
    grid.phi.resize(dims.n_phi);
    grid.w_theta.assign(dims.n_theta_fan,dt);
    grid.w_phi.assign(dims.n_phi,dp);
    grid.local_position_fan.resize(grid.element_count());
    grid.geometric_area_m2.resize(grid.element_count());
    for (std::size_t it=0;it<dims.n_theta_fan;++it)
        grid.theta_fan[it]=.5*(grid.theta_min_rad+grid.theta_max_rad)+
            .5*(2.*static_cast<double>(it)+1.-dims.n_theta_fan)*dt;
    for (std::size_t ip=0;ip<dims.n_phi;++ip)
        grid.phi[ip]=.5*(grid.phi_min_rad+grid.phi_max_rad)+
            .5*(2.*static_cast<double>(ip)+1.-dims.n_phi)*dp;
    for (std::size_t it=0;it<dims.n_theta_fan;++it) {
        const double theta=grid.theta_fan[it];
        for (std::size_t ip=0;ip<dims.n_phi;++ip) {
            const double phi=grid.phi[ip], rho=R+r*std::cos(phi);
            const std::size_t i=it*dims.n_phi+ip;
            grid.local_position_fan[i]={rho*std::sin(theta),r*std::sin(phi),-rho*std::cos(theta)};
            // Exact finite-volume cell area. Midpoint-only area cannot meet
            // W20's 1e-10 integral criterion on the seven-node lateral grid.
            grid.geometric_area_m2[i]=r*dt*(R*dp+2.*r*std::cos(phi)*std::sin(.5*dp));
        }
    }
    return grid;
}

FanFrame fan_frame(const WheelKinematics3d& kin) noexcept {
    FanFrame frame;
    if (!finite(kin.axle_world) || std::abs(norm(kin.axle_world)-1.) > 1e-8) return frame;
    frame.axle=unit(kin.axle_world);
    const Vec3 gravity{0.,0.,-1.};
    const Vec3 projected=sub(gravity,scale(frame.axle,dot(gravity,frame.axle)));
    if (norm(projected)<1e-6) return frame;
    frame.down=unit(projected);
    frame.forward=unit(cross(frame.down,frame.axle));
    frame.valid=true;
    return frame;
}

CoreStatus detail::query_geometry_range(const TorusGrid3d& grid, const WheelKinematics3d& kin,
                         const bike::terrain::Terrain3dQueryView& view,
                         std::span<ContactElement3d> out,
                         std::size_t begin, std::size_t end) noexcept {
    using QS=bike::terrain::QueryStatus3d;
    if (begin>end || end>out.size() || out.size()!=grid.element_count() || grid.phi.size()!=grid.dims.n_phi ||
        grid.theta_fan.size()!=grid.dims.n_theta_fan ||
        grid.local_position_fan.size()!=out.size() || grid.geometric_area_m2.size()!=out.size() ||
        grid.dims.n_theta_material==0 || !finite(kin.hub.position_world_m) ||
        !std::isfinite(kin.wheel_phase_rad) || !view.context || !view.ray_fn)
        return CoreStatus::invalid_input;
    const auto frame=fan_frame(kin);
    if (!frame.valid) {
        for (std::size_t i=begin;i<end;++i) { auto& e=out[i]; e={}; e.reason=ReasonCode3d::degenerate_down_projection; }
        return CoreStatus::invalid_geometry;
    }
    const auto world=[&](const Vec3& p) noexcept {
        return add(add(scale(frame.forward,p[0]),scale(frame.axle,p[1])),scale(frame.down,-p[2]));
    };
    const double clearance=2.*grid.r_m;
    if (!(clearance>0.) || !std::isfinite(clearance)) return CoreStatus::invalid_input;
    for (std::size_t i=begin;i<end;++i) {
        const std::size_t it=i/grid.dims.n_phi, ip=i%grid.dims.n_phi;
        const double theta=grid.theta_fan[it];
        const double material_angle=theta+kin.wheel_phase_rad;
        if (!std::isfinite(material_angle) || !std::isfinite(grid.phi[ip]) ||
            !finite(grid.local_position_fan[i]) || !std::isfinite(grid.geometric_area_m2[i]) ||
            !(grid.geometric_area_m2[i]>0.)) return CoreStatus::invalid_input;
        // TorusGrid3d is public mutable data. In particular, do not convert
        // a NaN material coordinate to an integer cell index (undefined).
        const double material_theta=wrapped(material_angle);
        const auto material_row=std::min(grid.dims.n_theta_material-1,
            static_cast<std::uint32_t>(material_theta/two_pi*grid.dims.n_theta_material));
        const Vec3 previous_longitudinal=out[i].longitudinal_world;
        const bool previous_valid=out[i].valid;
        auto& e=out[i]; e={};
        e.material_cell=material_row*grid.dims.n_phi+static_cast<std::uint32_t>(ip);
        e.geometric_area_m2=grid.geometric_area_m2[i];
        const double phi=grid.phi[ip];
        const Vec3 nout=unit(world({std::cos(phi)*std::sin(theta),std::sin(phi),-std::cos(phi)*std::cos(theta)}));
        const Vec3 point=add(kin.hub.position_world_m,world(grid.local_position_fan[i]));
        // Start inside the casing, above a lower contact, and cast outwards.
        // The opposite sign in the W prose starts below the road.
        const auto hit=view.ray_cast(sub(point,scale(nout,clearance)),nout,2.*clearance);
        e.ray_status=hit.status;
        if (hit.status!=QS::ok) {
            if (hit.status==QS::origin_below_surface) e.reason=ReasonCode3d::origin_below_surface;
            else if (hit.status==QS::budget_exhausted) e.reason=ReasonCode3d::terrain_budget_exhausted;
            else if (hit.status!=QS::miss && hit.status!=QS::outside) e.reason=ReasonCode3d::terrain_query_failed;
            continue;
        }
        if (!finite(hit.point_world_m) || !finite(hit.normal_world) ||
            !std::isfinite(hit.distance_m) || std::abs(norm(hit.normal_world)-1.)>1e-6) {
            e.reason=ReasonCode3d::nonfinite_geometry; continue;
        }
        e.point_world_m=hit.point_world_m; e.normal_world=hit.normal_world;
        e.surface_id=hit.surface_id;
        const double projection=std::max(0.,-dot(nout,e.normal_world));
        // Normal indentation is conjugate to a normal load. Using ray
        // indentation directly loses the virtual-work projection.
        e.penetration_m=(clearance-hit.distance_m)*projection;
        e.projected_area_m2=e.geometric_area_m2*projection;
        Vec3 longitudinal=sub(frame.forward,scale(e.normal_world,dot(frame.forward,e.normal_world)));
        if (norm(longitudinal)<1e-9) {
            longitudinal=sub(previous_longitudinal,scale(e.normal_world,dot(previous_longitudinal,e.normal_world)));
            if (!previous_valid || norm(longitudinal)<=1e-12) {
                e.reason=ReasonCode3d::nonfinite_geometry; continue;
            }
        }
        e.longitudinal_world=unit(longitudinal);
        e.lateral_world=unit(cross(e.normal_world,e.longitudinal_world));
        e.valid=projection>0.;
    }
    return CoreStatus::ok;
}
CoreStatus query_geometry(const TorusGrid3d& grid, const WheelKinematics3d& kin,
                         const bike::terrain::Terrain3dQueryView& view,
                         std::span<ContactElement3d> out) noexcept {
    return detail::query_geometry_range(grid,kin,view,out,0,out.size());
}
} // namespace bike::pneumatic3d
