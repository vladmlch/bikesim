#include "legacy_equilibrium.hpp"
#include "../engine_call.hpp"
#include "../stepper.hpp"
#include "../writers/suspension.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <limits>
#include <ranges>
#include <stdexcept>

namespace bike::initialization {
namespace {
std::size_t coordinate(const mjModel& model,std::string_view name) {
    const int id=mj_name2id(&model,mjOBJ_JOINT,std::string(name).c_str());
    if(id<0)throw std::invalid_argument("model has no joint '"+std::string(name)+"'; the equilibrium solve needs a planar root");
    const auto index=static_cast<std::size_t>(id);
    const auto type=std::views::counted(model.jnt_type,model.njnt)[index];
    if(type!=mjJNT_SLIDE&&type!=mjJNT_HINGE)throw std::invalid_argument("static equilibrium requires scalar root coordinates");
    return static_cast<std::size_t>(std::views::counted(model.jnt_qposadr,model.njnt)[index]);
}
}

StaticEquilibriumResult solve_static_equilibrium(StaticEquilibriumContext& context,
                                                const StaticEquilibriumOptions& options) {
    if(!context.initializing)throw std::invalid_argument("static equilibrium is confined to initialization");
    const auto cycle=options.relax_steps_per_cycle.value_or(40);
    if(cycle<=0||options.max_steps<=0||!std::isfinite(options.start_x_m)||
       !std::isfinite(options.tolerance)||options.tolerance<=0.)
        throw std::invalid_argument("invalid static-equilibrium cycle, budget, position or tolerance");
    if(options.crank_phase_rad&&!std::isfinite(*options.crank_phase_rad))
        throw std::invalid_argument("initial crank phase must be finite");
    if(!context.suspension_forces||!context.rear_travel_from_shock_mm)
        throw std::invalid_argument("static equilibrium requires suspension and analytical linkage owners");
    if(static_cast<bool>(context.tire_forces)!=static_cast<bool>(context.tire_reset))
        throw std::invalid_argument("static tire force and material reset must be supplied together");
    if(context.rider_loads_n&&(!context.rider_mass_kg||!std::isfinite(*context.rider_mass_kg)||*context.rider_mass_kg<=0.))
        throw std::invalid_argument("static rider load shares require positive finite rider mass");
    auto& stepper=context.stepper;
    mjModel* model=stepper.model();mjData* data=stepper.data();
    const auto x=coordinate(*model,"root_x"),z=coordinate(*model,"root_z"),pitch=coordinate(*model,"root_pitch");
    const auto fork=static_cast<std::size_t>(stepper.suspension().fork_qposadr());
    const auto shock=static_cast<std::size_t>(stepper.suspension().shock_qposadr());
    const auto crank=(options.crank_phase_rad||context.leg_initialize)?std::optional<std::size_t>(coordinate(*model,"crank_spin")):std::nullopt;
    auto forces=[&] {
        stepper.mutate([&] {
            if(context.before_forces)context.before_forces();
            if(context.tire_forces){engine::forward(model,data);context.tire_forces();}
            context.suspension_forces();
            if(context.rider_forces)context.rider_forces();
            if(context.leg_zero_torque_forces)context.leg_zero_torque_forces();
        });
    };
    stepper.mutate([&] {
        engine::reset_data(model,data);
        if(context.tire_reset)context.tire_reset();
        auto q=std::views::counted(data->qpos,model->nq);
        q[x]=options.start_x_m;q[z]=.005;
        if(options.crank_phase_rad)q[*crank]=*options.crank_phase_rad;
        if(context.drive_initialize)context.drive_initialize();
        if(context.leg_initialize)context.leg_initialize(q[*crank]);
    });
    stepper.note_posvel_stale();stepper.forward();
    std::int64_t steps=0;double residual=std::numeric_limits<double>::infinity();
    while(steps<options.max_steps) {
        const auto count=std::min(cycle,options.max_steps-steps);
        for(std::int64_t i=0;i<count;++i){forces();stepper.step();++steps;}
        std::ranges::fill(std::views::counted(data->qvel,model->nv),0.);
        stepper.note_posvel_stale();
        if(context.tire_reset){stepper.mutate([&]{context.tire_reset();});stepper.forward();}
        forces();stepper.forward();residual=0.;
        for(double value:stepper.qacc()) {
            if(!std::isfinite(value))throw std::runtime_error("nonfinite static-equilibrium acceleration");
            residual=std::max(residual,std::abs(value));
        }
        if(residual<=options.tolerance) {
            StaticEquilibriumResult result;
            auto& pose=result.pose;const auto q=stepper.qpos();
            pose.steps=steps;pose.residual_qacc=residual;
            pose.fork_travel_mm=q[fork]*1000.;pose.shock_stroke_mm=q[shock]*1000.;
            pose.rear_travel_mm=context.rear_travel_from_shock_mm(pose.shock_stroke_mm);
            pose.root_z_m=q[z];pose.pitch_rad=q[pitch];
            if(context.rider_loads_n) {
                result.rider_loads_n=context.rider_loads_n();
                result.rider_weight_shares=*result.rider_loads_n;
                for(auto& load:*result.rider_weight_shares) {
                    if(!std::isfinite(load))throw std::runtime_error("nonfinite static rider interface load");
                    load/=*context.rider_mass_kg*9.81;
                }
            }
            return result;
        }
    }
    const auto q=stepper.qpos();
    throw std::runtime_error(std::format("static equilibrium did not converge in {} steps: residual {:.4f} > {:.4f}, root_z {:.1f} mm, fork {:.1f} mm, shock {:.1f} mm",steps,residual,options.tolerance,q[z]*1000.,q[fork]*1000.,q[shock]*1000.));
}
} // namespace bike::initialization
