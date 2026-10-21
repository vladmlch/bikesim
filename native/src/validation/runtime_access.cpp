#include "runtime_access.hpp"
#include "mechanics.hpp"
#include "../model_access.hpp"
#include "../initialization/context.hpp"
#include "../research/evidence.hpp"
#include <atomic>

namespace bike::validation {
namespace {
class Guard {
    std::atomic<bool>& flag_;
public:
    explicit Guard(std::atomic<bool>& flag):flag_(flag){bool expected=false;if(!flag_.compare_exchange_strong(expected,true,std::memory_order_acquire))throw std::runtime_error("validation runtime operation already in progress");}
    ~Guard(){flag_.store(false,std::memory_order_release);}Guard(const Guard&)=delete;Guard& operator=(const Guard&)=delete;
};
std::array<double,3> cross(const std::array<double,3>& a,const std::array<double,3>& b){return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};}
}
void RuntimeRigAccess::inspect(runtime::NativeRideRuntime& ride,const std::function<void(const Stepper&,const runtime::PhysicalStep&)>& callback) {
    const Guard guard(ride.advancing_);ride.require_open();callback(*ride.stepper_,*ride.physical_);
}
void RuntimeRigAccess::release_rider(Stepper& stepper,runtime::PhysicalStep& physical){
    if(stepper.has_rider_contacts())stepper.rider_contacts_release_all();
    if(physical.rider_present()){auto state=physical.rider_control().state();state.enabled=false;physical.rider_control().restore(state);}
}
void RuntimeRigAccess::reset_rider_contacts(Stepper& stepper){
    auto state=stepper.rider_contacts().state();state.pending_release_loss_j=0.;auto probe=stepper.rider_contacts().probe();stepper.rider_contacts().assign_state(std::move(state),std::move(probe));
}
Value RuntimeRigAccess::rebase_energy(runtime::PhysicalStep& physical){
    auto [mass,elastic,total]=physical.energy_state();numeric::PythonSum stored;for(const auto& [name,x]:elastic){(void)name;stored.add(runtime::sample_wire::number(x));}
    physical.energy_.initial_energy_j=total;physical.energy_.energy_scale_j=std::max(1.,mass.kinetic_energy_j+stored.total());return research::wire_value(runtime::Wire{mass.wire});
}
Value RuntimeRigAccess::rebase_airborne(runtime::NativeRideRuntime& ride,double height,double vertical_speed,bool release,bool zero,bool spin,bool reset_tire) {
    const Guard guard(ride.advancing_);ride.require_open();if(ride.committed_step_!=0||ride.stepper_->time()!=0.)throw std::invalid_argument("a benchmark cannot rebase an already running energy ledger");scalar(height,"airborne offset");scalar(vertical_speed,"vertical velocity");
    auto& stepper=*ride.stepper_;auto& physical=*ride.physical_;auto* m=stepper.model();auto* d=stepper.data();Value result;
    const std::span<mjtNum> qpos=model_access::mutable_buffer(d->qpos,m->nq,"qpos"),qvel=model_access::mutable_buffer(d->qvel,m->nv,"qvel");
    stepper.mutate([&]{
        if(release)release_rider(stepper,physical);
        for(const char* name:{"root_z","rider_root_z"}){qpos[static_cast<std::size_t>(mechanics::qpos(m,name))]+=height;}
        if(zero)std::ranges::fill(qvel,0.);for(const char* name:{"root_z","rider_root_z"})qvel[static_cast<std::size_t>(mechanics::dof(m,name))]=vertical_speed;
        if(spin){qvel[static_cast<std::size_t>(mechanics::dof(m,"front_wheel_spin"))]=10.;qvel[static_cast<std::size_t>(mechanics::dof(m,"rear_wheel_spin"))]=20.;}
        if(reset_tire&&stepper.has_tire())stepper.tire_writer().reset();stepper.note_posvel_stale();stepper.forward();
        if(stepper.has_rider_contacts())reset_rider_contacts(stepper);
        result=rebase_energy(physical);
        engine::get_state(m,d,ride.bootstrap_.integration_state.data(),static_cast<int>(mjSTATE_INTEGRATION));ride.bootstrap_step_state_=physical.state();
    });
    ride.initialize_accounting();return result;
}
Value RuntimeRigAccess::static_metrics(runtime::NativeRideRuntime& ride,const Value& metadata,double expected_mass) {
    const Guard guard(ride.advancing_);ride.require_open();auto& stepper=*ride.stepper_;auto& physical=*ride.physical_;Value result;
    stepper.mutate([&]{auto* m=stepper.model();auto* d=stepper.data();const auto gravity=std::span{m->opt.gravity};const auto mass=physical.mass_observations(m,d);const auto channels=research::wire_value(runtime::Wire{mass.wire});const auto com=numbers(get(channels,"com_m"),"com");std::array<double,3> force{},moment{};for(std::size_t a=0;a<3;++a)force[a]=mass.mass_kg*gravity[a];
        if(!physical.snapshots_[0]||!physical.snapshots_[1])(void)physical.refresh_contacts(false,std::nullopt,false);
        const auto snapshots=physical.snapshots_;if(!snapshots[0]||!snapshots[1])throw std::runtime_error("static fixture lacks wheel snapshots");
        for(const auto& snapshot:snapshots)for(const auto& p:snapshot->patches){const std::array<double,3> applied=p.native_world_force_n.value_or(std::array<double,3>{p.normal_load_n*p.normal[0]+p.tangent_force_n*p.normal[2],0.,p.normal_load_n*p.normal[2]-p.tangent_force_n*p.normal[0]});std::array<double,3> arm{};for(std::size_t a=0;a<3;++a){force[a]+=applied[a];arm[a]=p.point_m[a]-com[a];}const auto torque=cross(arm,applied);for(std::size_t a=0;a<3;++a)moment[a]+=torque[a]+p.couple_world_nm[a];}
        std::array<double,3> axis{};for(std::size_t a=0;a<3;++a)axis[a]=snapshots[0]->wheel_axis_m[a]-snapshots[1]->wheel_axis_m[a];const double weight=mass.mass_kg*vector_norm(gravity);const auto& eq=get(metadata,"equilibrium");
        result=mechanics::metrics({{"mass_kg",mass.mass_kg},{"mass_error_kg",std::abs(mass.mass_kg-expected_mass)},{"force_relative_error",vector_norm(force)/weight},{"moment_relative_error",vector_norm(moment)/(weight*vector_norm(axis))},{"residual_qacc",real(eq,"residual_qacc")},{"fork_travel_m",real(eq,"fork_travel_mm")/1000.},{"rear_travel_m",real(eq,"rear_travel_mm")/1000.}});
    });return result;
}
runtime::TruthGeometry RuntimeRigAccess::truth_geometry(runtime::NativeRideRuntime& ride) {
    const Guard guard(ride.advancing_);ride.require_open();const auto* m=ride.stepper_->model();runtime::TruthGeometry out;for(const auto& point:ride.physical_->terrain_vertices()){out.x.push_back(point[0]);out.z.push_back(point[1]);}
    const auto geom_size=model_access::readonly_buffer(m->geom_size,3*m->ngeom,"geom_size");
    out.front_radius=geom_size[3*static_cast<std::size_t>(object_id(m,mjOBJ_GEOM,"geom_front_contact"))];out.rear_radius=geom_size[3*static_cast<std::size_t>(object_id(m,mjOBJ_GEOM,"geom_rear_contact"))];
    out.root_x_qpos=static_cast<std::size_t>(mechanics::qpos(m,"root_x"));out.root_x_dof=static_cast<std::size_t>(mechanics::dof(m,"root_x"));out.root_pitch_qpos=static_cast<std::size_t>(mechanics::qpos(m,"root_pitch"));out.root_pitch_dof=static_cast<std::size_t>(mechanics::dof(m,"root_pitch"));return out;
}
initialization::PhysicalInitialState RuntimeRigAccess::initial_state(runtime::NativeRideRuntime& ride,const initialization::SetupRequest& request,const initialization::EquilibriumResult& eq,const Value& metadata,std::optional<int> stations) {
    const Guard guard(ride.advancing_);ride.require_open();if(ride.committed_step_!=0||ride.stepper_->time()!=0.)throw std::invalid_argument("initial-state capture requires t=0");auto* m=ride.stepper_->model();
    initialization::SetupContext context{.stepper=*ride.stepper_,.physical=*ride.physical_,.physics=*request.model.physics,.start_x_m=request.start_x_m,
        .ground_z_m=geometry::compute_ground_z(request.model.bike)/1000.,.frame_body_id=object_id(m,mjOBJ_BODY,"frame"),.front_geom_id=object_id(m,mjOBJ_GEOM,"geom_front_contact"),.rear_geom_id=object_id(m,mjOBJ_GEOM,"geom_rear_contact"),
        .identity=get(metadata,"inputs"),.source_identity=text(metadata,"source_identity"),.provider_identity=text(metadata,"provider_identity"),.model_sha256=text(metadata,"model_sha256"),.initializing=true,.completed=true,.equilibrium=eq};
    return stations?initialization::PhysicalInitialState::project_stations(context,*stations):initialization::PhysicalInitialState::capture(context);
}
void RuntimeRigAccess::external_interval(runtime::NativeRideRuntime& ride,std::span<const double> values) {
    const Guard guard(ride.advancing_);ride.require_open();if(ride.committed_step_!=0)throw std::invalid_argument("external impulse fixture requires the initial interval");std::vector<double> force(values.begin(),values.end());
    auto& physical=*ride.physical_;auto& stepper=*ride.stepper_;ride.accounting_->collect(false);physical.capture_accounted_presentation(ride.committed_presentation_);ride.accounting_->push(physical.advance_physics(0.,0.,force,{}));
    engine::get_state(stepper.model(),stepper.data(),ride.next_integration_state_.data(),static_cast<int>(mjSTATE_INTEGRATION));physical.capture_presentation(ride.next_presentation_);ride.committed_integration_state_.swap(ride.next_integration_state_);std::swap(ride.committed_presentation_,ride.next_presentation_);ride.committed_step_=physical.step();ride.committed_time_s_=stepper.time();
    if(physical.crash())ride.committed_outcome_="crash:"+physical.crash()->cause;ride.cache_research_boundary();if(ride.committed_outcome_)ride.flush_accounting();else if(ride.accounting_->full()){ride.accounting_->close_period();physical.capture_accounted_presentation(ride.committed_presentation_);}
}
} // namespace bike::validation
