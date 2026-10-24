#include "tyre_fixture.hpp"
#include "../../../src/pneumatic3d/kernel_math.hpp"
#include "../../../src/spatial/frames.hpp"

#include <cmath>
#include <numbers>
#include <stdexcept>

namespace spatial_tests {
bike::pneumatic3d::Pneumatic3dSpec synthetic_spec(bike::spatial::TyreTier tier) {
    using namespace bike::pneumatic3d;
    Pneumatic3dSpec spec;
    spec.tier=tier; spec.outer_radius_m={.3675,.3493}; spec.casing_width_m={.061,.061};
    for (std::size_t w=0;w<2;++w) {
        const double r=.5*spec.casing_width_m[w],R=spec.outer_radius_m[w]-r;
        spec.carcass[w]={.pressure_pa=150000.,.k_area_n_per_m3=1e6,
            .maxwell_k_n_per_m3=2e5,.maxwell_tau_s=.02,.rim_clearance_m=.04,
            .rim_stiffness_n_per_m3=1e8,.initial_volume_m3=2.*std::numbers::pi*std::numbers::pi*R*r*r,
            .constant_pressure=true,.provenance_status="synthetic"};
        spec.brush[w]={.kx_n_per_m3=5e6,.ky_n_per_m3=5e6,
            .relaxation_x_m=.3,.relaxation_y_m=.3,.mu_x_scale=1.,.mu_y_scale=1.,
            .camber_coefficient_per_rad=.8,.pneumatic_trail_m=.02,
            .r_eff_m=spec.outer_radius_m[w],.provenance_status="synthetic"};
    }
    return spec;
}

bike::pneumatic3d::WheelKinematics3d wheel_at(const bike::spatial::Vec3& hub,
    double yaw,double camber,double speed,double spin) {
    using namespace bike::spatial;
    bike::pneumatic3d::WheelKinematics3d kin;
    const Quat qz{std::cos(.5*yaw),0.,0.,std::sin(.5*yaw)};
    const Quat qx{std::cos(.5*camber),std::sin(.5*camber),0.,0.};
    const auto q=quat_multiply(qz,qx);
    kin.hub.position_world_m=hub; kin.hub.quat_world_from_body_wxyz=q;
    kin.hub.linear_velocity_world_mps=rotate_body_to_world(q,{speed,0.,0.});
    kin.axle_world=rotate_body_to_world(q,{0.,1.,0.});
    kin.relative_spin_rad_s=spin;
    return kin;
}

double full_torus_area(const bike::pneumatic3d::TorusGrid3d& g) {
    const long double dt=2.*std::numbers::pi_v<long double>/1024.;
    const long double dp=2.*std::numbers::pi_v<long double>/256.;
    long double sum=0.;
    for (unsigned i=0;i<1024;++i) for (unsigned j=0;j<256;++j) {
        const long double p=(static_cast<long double>(j)+.5L)*dp;
        sum+=g.r_m*(g.R_m+g.r_m*std::cos(p))*dt*dp;
    }
    return static_cast<double>(sum);
}
double full_torus_volume(const bike::pneumatic3d::TorusGrid3d& g) {
    // Divergence theorem, V = 1/3 int(x.n)dA, full torus quadrature.
    const long double dt=2.*std::numbers::pi_v<long double>/1024.;
    const long double dp=2.*std::numbers::pi_v<long double>/256.;
    long double sum=0.;
    for (unsigned i=0;i<1024;++i) for (unsigned j=0;j<256;++j) {
        const long double p=(static_cast<long double>(j)+.5L)*dp;
        sum+=(g.R_m*std::cos(p)+g.r_m)*g.r_m*(g.R_m+g.r_m*std::cos(p))*dt*dp/3.;
    }
    return static_cast<double>(sum);
}

double potential_energy_fd(const bike::pneumatic3d::Pneumatic3dModel& model,
    const std::array<bike::pneumatic3d::WheelKinematics3d,2>& kin,
    const bike::spatial::Vec3& displacement,double h) {
    using namespace bike::pneumatic3d;
    using namespace bike::pneumatic3d::detail;
    if (!(h>0.) || !std::isfinite(h) || !finite(displacement))
        throw std::invalid_argument("potential_energy_fd: finite direction and h > 0 required");
    // Independent local potential on the committed contact topology. Frozen
    // const model has no query view accessor: do not const_cast and mutate it.
    // Carcass state contains normal indentation at material cells; the caller
    // must first commit the configuration represented by kin.
    (void)kin;
    const auto energy=[&](double shift) {
        double total=0.;
        for (std::size_t w=0;w<2;++w) {
            const auto& spec=model.spec().carcass[w];
            const auto grid=make_torus_grid(tier_dims(model.spec().tier),
                model.spec().outer_radius_m[w],model.spec().casing_width_m[w]);
            const auto& state=model.committed_state().carcass[w];
            double volume=0.;
            for (const auto& patch:model.patches(w)) for (const auto i:patch.elements) {
                const auto ip=i%grid.dims.n_phi, it=i/grid.dims.n_phi;
                const double theta=wrapped(grid.theta_fan[it]+state.material_phase_rad);
                const auto row=std::min(grid.dims.n_theta_material-1,
                    static_cast<std::uint32_t>(theta/two_pi*grid.dims.n_theta_material));
                const auto cell=static_cast<std::size_t>(row)*grid.dims.n_phi+ip;
                const double d=std::max(0.,state.deflection_m[cell]-shift*dot(displacement,patch.normal_world));
                const double a=grid.geometric_area_m2[i],rim=std::max(0.,d-spec.rim_clearance_m);
                volume+=d*a;
                total+=.5*(spec.k_area_n_per_m3*d*d+spec.rim_stiffness_n_per_m3*rim*rim)*a;
            }
            if (spec.constant_pressure) total+=spec.pressure_pa*volume;
            else {
                if (!(volume<spec.initial_volume_m3))
                    throw std::invalid_argument("potential_energy_fd: displaced volume exceeds initial volume");
                total-=spec.pressure_pa*spec.initial_volume_m3*std::log1p(-volume/spec.initial_volume_m3);
            }
        }
        return total;
    };
    return -(energy(h)-energy(-h))/(2.*h);
}
} // namespace spatial_tests
