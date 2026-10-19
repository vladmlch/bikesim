#include "ride.hpp"
#include "legs.hpp"
#include "../diag.hpp"
#include "../writer_config.hpp"
#include "../runtime/project_config.hpp"
#include "../configuration/model_config.hpp"
#include "../initialization/capture.hpp"
#include "../initialization/legacy_equilibrium.hpp"
#include "../io/value.hpp"
#include "../model/mapping.hpp"
#include "../model/suspension_setup.hpp"
#include "../runtime/setup_values.hpp"
#include "../stepper.hpp"
#include "../terrain/profile.hpp"
#include "../tyre/profile.hpp"
#include "../numeric_sincos.hpp"
#include "../writers/cruise.hpp"
#include "../writers/suspension.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numbers>
#include <ranges>
#include <stdexcept>

namespace bike::legacy {
namespace {
void populate_road(mjModel const& m,const model::BuildRequest& request,const terrain::TrackSpec& track) {
    const int id=model::required_id(m,mjOBJ_HFIELD,"road");
    const auto field_index=static_cast<std::size_t>(id);
    const auto rows=std::views::counted(m.hfield_nrow,m.nhfield),cols=std::views::counted(m.hfield_ncol,m.nhfield);
    if(rows[field_index]!=request.field.nrow||cols[field_index]!=request.field.ncol) throw std::invalid_argument("legacy road dimensions differ from request");
    const auto heights=terrain::build_field_data(track,request.field);
    const auto start=std::views::counted(m.hfield_adr,m.nhfield)[field_index];
    if(start<0||static_cast<std::size_t>(start)>static_cast<std::size_t>(m.nhfielddata)||heights.size()>static_cast<std::size_t>(m.nhfielddata-start))
        throw std::invalid_argument("legacy road storage extent");
    auto const storage=std::views::counted(m.hfield_data,m.nhfielddata).subspan(static_cast<std::size_t>(start),heights.size());
    for(std::size_t i=0;i<heights.size();++i) storage[i]=static_cast<float>(heights[i]);
}
struct Accelerometer {
    int address=-1,site=-1;
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
    Accelerometer(const mjModel& m,std::string_view sensor,std::string_view name,bool required=true) {
        const int id=mj_name2id(&m,mjOBJ_SENSOR,std::string(sensor).c_str());
        site=mj_name2id(&m,mjOBJ_SITE,std::string(name).c_str());
        if(id<0||site<0) { if(required) throw std::invalid_argument("legacy accelerometer missing: "+std::string(sensor)); return; }
        if(std::views::counted(m.sensor_dim,m.nsensor)[static_cast<std::size_t>(id)]!=3) throw std::invalid_argument("legacy accelerometer must be three-dimensional");
        address=std::views::counted(m.sensor_adr,m.nsensor)[static_cast<std::size_t>(id)];
    }
    std::array<double,2> world(const mjModel& m,const mjData& d)const {
        if(address<0||site<0)return {};
        const auto r=std::views::counted(d.site_xmat,9*m.nsite).subspan(9*static_cast<std::size_t>(site),9);
        const auto a=std::views::counted(d.sensordata,m.nsensordata).subspan(static_cast<std::size_t>(address),3);
        return {r[6]*a[0]+r[7]*a[1]+r[8]*a[2],r[0]*a[0]+r[1]*a[1]+r[2]*a[2]};
    }
};
}
struct RideSimulation::Impl {
    RideOptions options;
    kinematics::HorstLinkageSolver solver;
    std::optional<model::SeatedPose> pose;
    std::unique_ptr<Stepper> stepper;
    std::unique_ptr<TyreForceApplier> tyre;
    std::unique_ptr<RiderForces> rider;
    std::unique_ptr<LegDrive> legs;
    std::unique_ptr<PedalDrivetrain> drive;
    std::unique_ptr<TerrainContactQuery> contact_query;
    std::unique_ptr<Accelerometer> bar,saddle,torso,pelvis;
    TerrainContacts contacts;
    PitchState pitch;
    std::optional<CrashEvent> crash;
    initialization::StaticEquilibriumResult equilibrium;
    std::array<ForceComponentView,SuspensionWriter::kLegacyComponentCount> suspension_views;
    std::vector<double> scratch,forces;
    std::vector<double> stroke_table_mm,travel_table_mm;
    std::array<double,7> suspension_scalars{};
    std::array<std::size_t,2> wheel_dof{};
    std::array<double,2> radius{};
    std::size_t x_q=0,x_v=0,pitch_q=0,pitch_v=0,rear_drive=0;
    double weight=0.;
    std::int64_t steps=0;
    std::uint64_t generation=0;
    bool brake_source=false,traction_limited=false;
    model::DamperPreset viewer_preset=model::DamperPreset::base;

    explicit Impl(RideOptions o):options(std::move(o)),solver(options.setup.model.solver
        ?*options.setup.model.solver:kinematics::HorstLinkageSolver(options.setup.model.bike)) {}
    void follow_cranks() {
        if(!drive||!rider->active())return;
        const double phase=stepper->qpos()[drive->crank_qposadr()];
        const double drop=options.setup.model.bike.crank_length/1000.*numeric::sin(phase);
        rider->set_pedal_offsets(-drop,drop);
    }
    void suspension_forces() {
        auto const* m=stepper->model();auto const* d=stepper->data();
        stepper->suspension().components_into(d,suspension_views);
        std::ranges::fill(forces,0.);
        for(const auto & suspension_view : suspension_views)
            for(std::size_t i=0;i<forces.size();++i)forces[i]+=suspension_view.values[i];
        const auto& suspension=stepper->suspension();
        const auto fork_dof=static_cast<std::size_t>(suspension.fork_dofadr()),shock_dof=static_cast<std::size_t>(suspension.shock_dofadr());
        for(std::size_t j=0;j<suspension_views.size();++j) {
            const auto row=suspension_views[j].values;
            suspension_scalars[j]=-row[fork_dof]-row[shock_dof];
        }
        // Each historical assigning writer owns disjoint qfrc coordinates.
        auto const applied=std::views::counted(d->qfrc_applied,m->nv);
        applied[fork_dof]=forces[fork_dof];
        applied[shock_dof]=forces[shock_dof];
    }
    void rider_forces() {
        rider->compute_into(*stepper->data(),scratch);
        auto const applied=std::views::counted(stepper->data()->qfrc_applied,stepper->model()->nv);
        // The sparse sum preserves other assigning writers, including zero
        // values because the run clears generalized force once per tick.
        for(std::size_t i=0;i<scratch.size();++i)applied[i]+=scratch[i];
    }
    void leg_forces(double torque) {
        legs->compute_into(*stepper->data(),torque,scratch);
        auto const applied=std::views::counted(stepper->data()->qfrc_applied,stepper->model()->nv);
        for(std::size_t i=0;i<scratch.size();++i)applied[i]+=scratch[i];
    }
};
RideSimulation::RideSimulation(std::unique_ptr<Impl> p):impl_(std::move(p)){}
RideSimulation::~RideSimulation()=default;
namespace {
[[gnu::noinline]] std::unique_ptr<Stepper> make_legacy_stepper(const model::BuildRequest& build, const mjModel& compiled,
                                             const terrain::TrackSpec& track, const runtime::ProjectionOptions& runtime_opts) {
    auto projection = std::make_unique<model::BuildRequest>(build);
    if (!projection->physics) throw std::invalid_argument("reference request requires model.physics");
    projection->physics->tires.backend = "native_reference";
    auto writers = std::make_unique<nativecfg::WriterConfig>(
        nativecfg::decode(runtime::project_writer_config(*projection, compiled, track, runtime_opts)));
    writers->tire.reset();
    writers->resistance.reset();
    writers->rider_forces.reset();
    const auto image = initialization::capture_model_image(compiled);
    return std::make_unique<Stepper>(image.mjb, *writers);
}
[[gnu::noinline]] std::unique_ptr<RideOptions> load_legacy_options(const std::filesystem::path& request) {
    auto options = std::make_unique<RideOptions>();
    options->setup=initialization::read_setup_request(request);
    // The file's speed_kph lands on model.physics->initial_speed_mps; the
    // runtime cruise target keeps its ProjectionOptions default.
    return options;
}
} // namespace
std::unique_ptr<RideSimulation> make_ride(const std::filesystem::path& request) {
    auto options = load_legacy_options(request);
    return make_ride(std::move(*options));
}
[[gnu::noinline]] std::unique_ptr<RideSimulation> make_ride(std::unique_ptr<RideOptions> options) {
    return make_ride(std::move(*options));
}
[[gnu::noinline]] std::unique_ptr<RideSimulation> make_ride(RideOptions options) {
    options.tyre.validate();options.drivetrain.validate();
    if(options.drive_mode!="motor"&&options.drive_mode!="pedal"&&options.drive_mode!="pedelec")throw std::invalid_argument("unknown legacy drive mode");
    auto& request=options.setup.model;
    if(!request.physics)request.physics=configuration::default_physics();
    configuration::post_init(*request.physics);
    if(request.physics->physics_mode!="legacy")throw std::invalid_argument("legacy RideSimulation requires physics_mode='legacy'");
    if(options.setup.initial_state)throw std::invalid_argument("physical initial state cannot initialize a legacy ride");
    if(options.setup.runtime.record_retention==runtime::RetentionMode::full)
        throw std::invalid_argument("full --record is unavailable for legacy rides; use legacy CSV capture");
    if(request.mode!="ride")throw std::invalid_argument("legacy RideSimulation requires model_mode='ride'");
    if(!std::isfinite(options.setup.start_x_m))throw std::invalid_argument("legacy start position must be finite");
    options.setup.track.validate();request.field.validate();terrain::assert_track_fits(options.setup.track,request.field);
    const bool crank=options.drive_mode!="motor"||options.visual_pedalling;
    auto resolved=model::resolve_rider(request.rider?model::RiderArg{*request.rider}:model::RiderArg{std::monostate{}},request.include_rider,"seated");
    resolved.legs=options.legs.value_or(crank?"articulated":"rigid");
    request.rider=resolved;request.crank_joint=crank;request.tyre_model=options.tyre.model;request.gear_ratio=options.drivetrain.gear_ratio();
    auto p=std::make_unique<RideSimulation::Impl>(std::move(options));
    auto const& build=p->options.setup.model;
    if(resolved.variant=="seated")p->pose=model::seated_pose(resolved,build.bike);
    const auto compiled=model::compile_mjcf(model::generate_mjcf(build),p->options.setup.asset_root);
    populate_road(*compiled,build,p->options.setup.track);
    p->stepper=make_legacy_stepper(build,*compiled,p->options.setup.track,p->options.setup.runtime);
    auto& m=*p->stepper->model();
    if(p->options.tyre.pneumatic())p->tyre=std::make_unique<TyreForceApplier>(m,p->options.tyre,
        build_road_profile(p->options.setup.track,build.field,build.bike),p->options.setup.track.surface_map());
    p->rider=std::make_unique<RiderForces>(m,p->pose);
    p->legs=std::make_unique<LegDrive>(m,p->pose,build.bike.crank_length/1000.,p->options.drivetrain.ripple_depth);
    if(p->options.drive_mode!="motor")p->drive=std::make_unique<PedalDrivetrain>(m,p->options.drivetrain,
        p->options.drive_mode,p->options.assist_mode,p->legs->active());
    p->contact_query=std::make_unique<TerrainContactQuery>(m);
    const auto x=resolve_scalar_joint(m,"root_x"),pitch=resolve_scalar_joint(m,"root_pitch");
    p->x_q=static_cast<std::size_t>(x.first);p->x_v=static_cast<std::size_t>(x.second);p->pitch_q=static_cast<std::size_t>(pitch.first);p->pitch_v=static_cast<std::size_t>(pitch.second);
    p->rear_drive=static_cast<std::size_t>(validate_actuator_target(m,"rear_drive","rear_wheel_spin"));
    const auto front_wheel=resolve_wheel_spin(m,"front_wheel_spin","geom_front_contact"),rear_wheel=resolve_wheel_spin(m,"rear_wheel_spin","geom_rear_contact");
    p->wheel_dof={static_cast<std::size_t>(front_wheel.dofadr),static_cast<std::size_t>(rear_wheel.dofadr)};
    p->radius={front_wheel.radius_m,rear_wheel.radius_m};
    for(double const mass:std::views::counted(m.body_mass,m.nbody))p->weight+=mass;
    p->weight*=std::abs(std::span(m.opt.gravity)[2]);
    p->scratch.resize(static_cast<std::size_t>(m.nv));p->forces.resize(static_cast<std::size_t>(m.nv));
    const double requested_intervals=std::nearbyint(build.bike.shock_stroke/.1);
    if(!std::isfinite(requested_intervals)||requested_intervals<1.||requested_intervals>=static_cast<double>(p->stroke_table_mm.max_size()-1))
        throw std::invalid_argument("legacy recorder shock interpolation table extent");
    const auto intervals=static_cast<std::size_t>(requested_intervals);
    if(intervals<1)throw std::invalid_argument("legacy recorder requires a nonempty shock interpolation interval");
    p->stroke_table_mm.resize(intervals+1);p->travel_table_mm.resize(intervals+1);
    const double spacing=build.bike.shock_stroke/static_cast<double>(intervals);
    for(std::size_t i=0;i<=intervals;++i){p->stroke_table_mm[i]=i==intervals?build.bike.shock_stroke:static_cast<double>(i)*spacing;
        p->travel_table_mm[i]=p->solver.solve_state_from_shock_stroke(p->stroke_table_mm[i]).wheel_travel;}
    p->bar=std::make_unique<Accelerometer>(m,"sensor_bar_accel","site_handlebar");
    p->saddle=std::make_unique<Accelerometer>(m,"sensor_saddle_accel","site_seatpost_top");
    p->torso=std::make_unique<Accelerometer>(m,"sensor_rider_torso_accel","site_rider_torso",p->rider->active());
    p->pelvis=std::make_unique<Accelerometer>(m,"sensor_rider_pelvis_accel","site_rider_pelvis",p->rider->active());
    auto result=std::unique_ptr<RideSimulation>(new RideSimulation(std::move(p)));
    result->reset();return result;
}
void RideSimulation::reset() {
    auto& p=*impl_;auto* m=p.stepper->model();auto* d=p.stepper->data();
    initialization::StaticEquilibriumContext context{.stepper=*p.stepper};
    context.before_forces=[&]{std::ranges::fill(std::views::counted(d->qfrc_applied,m->nv),0.);p.follow_cranks();};
    context.suspension_forces=[&]{p.suspension_forces();};
    context.rear_travel_from_shock_mm=[&](double stroke){return p.solver.solve_state_from_shock_stroke(stroke).wheel_travel;};
    context.rider_forces=[&]{p.rider_forces();};
    if(p.legs->active()) {
        context.leg_zero_torque_forces=[&]{p.leg_forces(0.);};
        context.leg_initialize=[&](double phase){p.legs->initialize(*d,phase);};
    }
    if(p.drive)context.drive_initialize=[&]{p.drive->reset(*d);};
    if(p.tyre) {context.tire_forces=[&]{p.tyre->apply(*d);};context.tire_reset=[&]{p.tyre->reset(d);};}
    if(p.rider->active()) {
        // NOLINTNEXTLINE(bugprone-unchecked-optional-access) pose is engaged whenever the rider is active
        context.rider_mass_kg=p.pose->total_mass_kg();
        context.rider_loads_n=[&]{const auto loads=p.rider->loads();return std::array{loads.saddle_n,loads.front_pedal_n+loads.rear_pedal_n,loads.bar_n};};
    }
    initialization::StaticEquilibriumOptions options;
    options.start_x_m=p.options.setup.start_x_m;
    if(p.drive)options.crank_phase_rad=p.options.drivetrain.crank_phase_deg*(std::numbers::pi/180.);
    if(p.tyre)options.relax_steps_per_cycle=std::max<std::int64_t>(1,static_cast<std::int64_t>(std::nearbyint(.020/m->opt.timestep)));
    p.equilibrium=initialization::solve_static_equilibrium(context,options);
    p.stepper->cruise_reset();p.contact_query->reset();
    if(p.drive)p.drive->reset(*d,false);
    if(p.drive||p.legs->active()){p.stepper->note_posvel_stale();p.stepper->forward();}
    d->time=0.;p.steps=0;++p.generation;p.pitch={};p.crash.reset();p.brake_source=false;p.traction_limited=false;
    p.contacts=p.contact_query->query(*d,p.tyre.get());
}
void RideSimulation::step(double front_brake,double rear_brake) {
    if(!std::isfinite(front_brake)||!std::isfinite(rear_brake)||front_brake<0.||front_brake>1.||rear_brake<0.||rear_brake>1.)
        throw std::invalid_argument("legacy brake demand must be in [0,1]");
    auto& p=*impl_;auto* m=p.stepper->model();auto* d=p.stepper->data();
    p.stepper->mutate([&] {
        std::ranges::fill(std::views::counted(d->qfrc_applied,m->nv),0.);
        if(p.tyre){p.stepper->forward();p.tyre->apply(*d);p.contacts=p.contact_query->query(*d,p.tyre.get());}
        p.suspension_forces();p.follow_cranks();p.rider_forces();
        auto const applied=std::views::counted(d->qfrc_applied,m->nv);
        const auto velocity=std::views::counted(d->qvel,m->nv);
        if(!p.tyre) {
            applied[p.wheel_dof[0]]=rolling_torque(p.contacts.front_support_n,velocity[p.wheel_dof[0]],p.radius[0],p.weight);
            applied[p.wheel_dof[1]]=rolling_torque(p.contacts.rear_support_n,velocity[p.wheel_dof[1]],p.radius[1],p.weight);
        }
        double torque=0.;p.traction_limited=false;
        if(p.options.setup.model.physics->drive_mode=="ideal_speed_control") {
            (void)p.stepper->cruise_set_assist_compensation(p.drive?p.drive->nominal_support_factor()*cutoff_factor(velocity[p.x_v],p.drive->specs()):0.);
            if(p.tyre) {
                const double error=p.stepper->cruise_state().target_speed_mps-velocity[p.x_v];
                for(const auto& patch:p.tyre->rear_outputs().patches())
                    if(patch.fully_sliding&&patch.slip_ratio*error>0.)p.traction_limited=true;
            }
            torque=p.stepper->cruise_compute(p.contacts.rear_in_contact(),p.traction_limited,std::nullopt);
        }
        auto const ctrl=std::views::counted(d->ctrl,m->nu);
        if(!p.drive){ctrl[p.rear_drive]=torque;p.leg_forces(0.);}
        else {
            ctrl[p.rear_drive]=0.;
            const auto& command=p.drive->compute(*d,torque,velocity[p.x_v],p.contacts.rear_in_contact(),p.traction_limited);
            p.drive->write(*d);p.leg_forces(command.rider_torque_nm);
            p.brake_source=command.brake_demand>std::max(front_brake,rear_brake);
            front_brake=std::max(front_brake,command.brake_demand);rear_brake=std::max(rear_brake,command.brake_demand);
        }
        p.stepper->apply_brake(front_brake,rear_brake);
        p.pitch=pitch_step(p.pitch,p.options.setup.model.physics->pitch_assist.value_or(true),
            p.stepper->qpos()[p.pitch_q],velocity[p.pitch_v],p.contacts,m->opt.timestep);
        applied[p.pitch_v]=p.pitch.moment_nm;
        p.crash=detect_crash(d->time,p.stepper->qpos()[p.x_q],p.stepper->qpos()[p.pitch_q],p.contacts,p.crash);
    });
    p.stepper->note_posvel_stale();p.stepper->step();++p.steps;
    if(!p.tyre)p.contacts=p.contact_query->query(*d);
}
void RideSimulation::set_target_speed_kph(double speed){impl_->stepper->cruise_set_target_speed(speed);}
void RideSimulation::set_assist_mode(std::string_view mode){if(!impl_->drive)throw std::invalid_argument("assist requires legacy pedal drivetrain");impl_->drive->set_assist_mode(mode);}
void RideSimulation::set_tyre_pressures(double front,double rear){if(!impl_->tyre)throw std::invalid_argument("pressure controls require legacy pneumatic tyre");impl_->tyre->set_pressures(front,rear);}
std::optional<std::string> RideSimulation::handle_tuning_key(int key) {
    auto& p=*impl_;
    if(key>='a'&&key<='z')key-='a'-'A';
    if(key=='W'||key=='S'){
        if(!p.stepper->has_cruise())return "W/S requires a cruise controller.";
        const double target=std::clamp(p.stepper->cruise_state().target_speed_mps*3.6+(key=='W'?1.:-1.),15.,45.);
        set_target_speed_kph(target);return "Cruise target: "+configuration::detail::repr_double(target)+" km/h";
    }
    if(key=='E')return p.drive?"Assist: "+p.drive->cycle_assist_mode():"No mid-drive on this run (--drive-mode pedelec adds one).";
    if(key=='N'||key=='M'||key==';'||key=='\''){
        if(!p.tyre)return "Tyre pressure keys require --tyre-model pneumatic.";
        const bool front=key=='N'||key=='M';const double delta=(key=='N'||key==';')?-PRESSURE_STEP_BAR:PRESSURE_STEP_BAR;
        double front_bar=p.tyre->config().front.pressure_bar,rear_bar=p.tyre->config().rear.pressure_bar;
        if(front)front_bar=std::clamp(front_bar+delta,PRESSURE_MIN_BAR,PRESSURE_MAX_BAR);
        else rear_bar=std::clamp(rear_bar+delta,PRESSURE_MIN_BAR,PRESSURE_MAX_BAR);
        set_tyre_pressures(front_bar,rear_bar);
        return std::string(front?"Front":"Rear")+" tyre: "+configuration::detail::repr_double(front?front_bar:rear_bar)+" bar";
    }
    auto config=p.stepper->suspension().config();
    if(config.physics_mode!="legacy")throw std::logic_error("legacy tuning requires the legacy suspension owner");
    auto& fork=config.fork_damper.core;auto& shock=config.shock_damper.core;
    const auto adjust=[](int& value,int delta,int maximum){value=std::clamp(value+delta,0,maximum);};
    std::string message;
    auto next_preset=p.viewer_preset;
    switch(key){
        case 'L':adjust(fork.lsc_clicks,1,fork.max_lsc);break;case 'K':adjust(fork.lsc_clicks,-1,fork.max_lsc);break;
        case 'J':adjust(fork.hsc_clicks,1,fork.max_hsc);break;case 'H':adjust(fork.hsc_clicks,-1,fork.max_hsc);break;
        case 'U':adjust(fork.rebound_clicks,1,fork.max_reb);break;case 'Y':adjust(fork.rebound_clicks,-1,fork.max_reb);break;
        case '0':adjust(shock.lsc_clicks,1,shock.max_lsc);break;case '9':adjust(shock.lsc_clicks,-1,shock.max_lsc);break;
        case '8':adjust(shock.hsc_clicks,1,shock.max_hsc);break;case '7':adjust(shock.hsc_clicks,-1,shock.max_hsc);break;
        case '6':adjust(shock.rebound_clicks,1,shock.max_reb);break;case '5':adjust(shock.rebound_clicks,-1,shock.max_reb);break;
        case '4':adjust(config.shock_damper.hbo_clicks,1,config.shock_damper.max_hbo);break;
        case '3':adjust(config.shock_damper.hbo_clicks,-1,config.shock_damper.max_hbo);break;
        case 'X':config.shock_damper.lockout_firm=!config.shock_damper.lockout_firm;break;
        case '[':adjust(config.air_spring.num_tokens,-1,config.air_spring.specs.max_tokens);break;
        case ']':adjust(config.air_spring.num_tokens,1,config.air_spring.specs.max_tokens);break;
        case '-':config.air_spring.gauge_pressure_psi=std::clamp(config.air_spring.gauge_pressure_psi-2.,10.,200.);break;
        case '=':config.air_spring.gauge_pressure_psi=std::clamp(config.air_spring.gauge_pressure_psi+2.,10.,200.);break;
        case 'P':{
            constexpr std::array presets{model::DamperPreset::base,model::DamperPreset::plush,model::DamperPreset::enduro,model::DamperPreset::park};
            std::size_t current=presets.size();
            for(std::size_t i=0;i<presets.size();++i)if(presets[i]==p.viewer_preset){current=i;break;}
            next_preset=current==presets.size()||current+1==presets.size()?presets.front():presets[current+1];
            const auto clicks=model::damper_preset_config(next_preset);
            fork.hsc_clicks=clicks.fork_hsc;fork.lsc_clicks=clicks.fork_lsc;fork.rebound_clicks=clicks.fork_rebound;
            shock.hsc_clicks=clicks.shock_hsc;shock.lsc_clicks=clicks.shock_lsc;shock.rebound_clicks=clicks.shock_rebound;
            config.shock_damper.hbo_clicks=clicks.shock_hbo;config.shock_damper.lockout_firm=clicks.shock_lockout;
            message=std::string(model::damper_preset_label(next_preset))+" | ";break;
        }
        default:return {};
    }
    message+="Fork H"+std::to_string(fork.hsc_clicks)+" L"+std::to_string(fork.lsc_clicks)+" R"+std::to_string(fork.rebound_clicks)+
        " | Shock H"+std::to_string(shock.hsc_clicks)+" L"+std::to_string(shock.lsc_clicks)+" R"+std::to_string(shock.rebound_clicks)+" B"+std::to_string(config.shock_damper.hbo_clicks)+
        (config.shock_damper.lockout_firm?" FIRM":" OPEN")+" | Air "+configuration::detail::repr_double(config.air_spring.gauge_pressure_psi)+" PSI, "+std::to_string(config.air_spring.num_tokens)+" tokens";
    // Constructor validation is performed before replacing the active writer.
    // This changes no model/data and never performs a simulation step.
    SuspensionWriter replacement(p.stepper->model(),std::move(config));
    p.stepper->suspension()=std::move(replacement);p.viewer_preset=next_preset;
    return message;
}
double RideSimulation::time_s()const{return impl_->stepper->time();}
double RideSimulation::start_x_m()const{return impl_->options.setup.start_x_m;}
double RideSimulation::shock_bumper_engage_mm()const{return impl_->stepper->suspension().config().coil.bumper_engage_mm();}
std::optional<CrashEvent> RideSimulation::crash_event()const{return impl_->crash;}
std::optional<std::array<biketyre::WheelOutputs,2>> RideSimulation::tyre_outputs()const {
    if(!impl_->tyre)return std::nullopt;
    return std::array{impl_->tyre->front_outputs(),impl_->tyre->rear_outputs()};
}
double RideSimulation::position_m()const{return impl_->stepper->qpos()[impl_->x_q];}
double RideSimulation::timestep_s()const{return impl_->stepper->model()->opt.timestep;}
std::int64_t RideSimulation::steps()const{return impl_->steps;}
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wlarge-by-value-copy")
TelemetryRow RideSimulation::telemetry()const {
    const auto& p=*impl_;const auto& m=*p.stepper->model();const auto& d=*p.stepper->data();
    const auto q=p.stepper->qpos(),v=p.stepper->qvel();const auto& s=p.stepper->suspension();
    const auto fork_q=static_cast<std::size_t>(s.fork_qposadr()),shock_q=static_cast<std::size_t>(s.shock_qposadr());
    const auto fork_v=static_cast<std::size_t>(s.fork_dofadr()),shock_v=static_cast<std::size_t>(s.shock_dofadr());
    TelemetryRow row{};
    auto const set=[&](std::string_view name,double value){const auto i=std::ranges::find(kLegacyChannels,name);if(i==kLegacyChannels.end())throw std::logic_error("unknown legacy channel");row[static_cast<std::size_t>(i-kLegacyChannels.begin())]=value;};
    set("time_s",d.time);set("x_m",q[p.x_q]);set("speed_mps",v[p.x_v]);set("pitch_rad",q[p.pitch_q]);
    set("fork_travel_mm",q[fork_q]*1000.);set("fork_shaft_mps",v[fork_v]);
    set("shock_stroke_mm",q[shock_q]*1000.);set("shock_shaft_mps",v[shock_v]);
    const double stroke=q[shock_q]*1000.;
    const auto upper=std::ranges::upper_bound(p.stroke_table_mm,stroke);
    const auto segment=std::clamp(upper-p.stroke_table_mm.begin()-1,
        std::ptrdiff_t{0},static_cast<std::ptrdiff_t>(p.stroke_table_mm.size())-2);
    const auto index=static_cast<std::size_t>(segment);
    const double slope=(p.travel_table_mm[index+1]-p.travel_table_mm[index])/(p.stroke_table_mm[index+1]-p.stroke_table_mm[index]);
    set("rear_wheel_mm",stroke>=p.stroke_table_mm.front()&&stroke<=p.stroke_table_mm.back()
        ?biketyre::np_interp(stroke,p.stroke_table_mm,p.travel_table_mm)
        :p.travel_table_mm[index]+slope*(stroke-p.stroke_table_mm[index]));
    const auto names=s.component_names();
    for(std::size_t i=0;i<names.size();++i) {
        const auto name=names[i];
        if(name=="fork_spring")set("fork_spring_n",p.suspension_scalars[i]);
        else if(name=="fork_damper")set("fork_damper_n",p.suspension_scalars[i]);
        else if(name=="shock_coil")set("shock_spring_n",p.suspension_scalars[i]);
        else if(name=="shock_bumper")set("shock_bumper_n",p.suspension_scalars[i]);
        else if(name=="shock_damper")set("shock_damper_n",p.suspension_scalars[i]);
    }
    set("front_contact",p.contacts.front_in_contact()?1.:0.);set("rear_contact",p.contacts.rear_in_contact()?1.:0.);
    set("front_load_n",p.contacts.front_load_n);set("rear_load_n",p.contacts.rear_load_n);
    double wheel_torque=p.stepper->cruise_state().torque_nm;
    if(p.drive){const auto& c=p.drive->command();wheel_torque=c.freewheel?0.:(c.rider_torque_nm+c.assist_torque_nm)/p.options.drivetrain.gear_ratio();}
    set("drive_torque_nm",wheel_torque);set("wheel_power_w",wheel_torque*v[p.wheel_dof[1]]);
    const auto ba=p.bar->world(m,d),sa=p.saddle->world(m,d),ta=p.torso->world(m,d),pa=p.pelvis->world(m,d);
    set("bar_acc_vert_mps2",ba[0]);set("bar_acc_long_mps2",ba[1]);set("saddle_acc_vert_mps2",sa[0]);set("saddle_acc_long_mps2",sa[1]);
    set("rider_torso_acc_vert_mps2",ta[0]);set("rider_torso_acc_long_mps2",ta[1]);set("rider_pelvis_acc_vert_mps2",pa[0]);set("rider_pelvis_acc_long_mps2",pa[1]);
    set("rider_moment_nm",p.pitch.moment_nm);set("rider_impulse_nms",p.pitch.angular_impulse_nms);set("rider_work_j",p.pitch.work_j);
    const auto loads=p.rider->loads();const auto legs=p.legs->pedal_forces_n();
    set("saddle_load_n",loads.saddle_n);set("saddle_gap_m",loads.gap_m);set("bar_hand_load_n",loads.bar_n);
    set("pedal_load_front_n",p.legs->active()?legs[0]:loads.front_pedal_n);set("pedal_load_rear_n",p.legs->active()?legs[1]:loads.rear_pedal_n);
    const auto tyre_start=static_cast<std::size_t>(std::ranges::find(kLegacyChannels,"front_tyre_fz_n")-kLegacyChannels.begin());
    const auto drive_start=static_cast<std::size_t>(std::ranges::find(kLegacyChannels,"crank_phase_rad")-kLegacyChannels.begin());
    std::ranges::fill(std::span{row}.subspan(tyre_start,drive_start-tyre_start),0.);
    std::ranges::fill(std::span{row}.subspan(drive_start),std::numeric_limits<double>::quiet_NaN());
    if(p.tyre) for(std::size_t i=0;i<2;++i) {
        const auto& t=i==0?p.tyre->front_outputs():p.tyre->rear_outputs();const std::string side=i==0?"front":"rear";
        set(side+"_tyre_fz_n",t.support_n);set(side+"_tyre_fx_n",t.force_world_n[0]);set(side+"_tyre_deflection_mm",t.mean_deflection_m*1000.);
        set(side+"_patch_length_mm",t.contact_length_m*1000.);set(side+"_slip_ratio",t.slip_ratio);set(side+"_tyre_full_sliding",t.fully_sliding?1.:0.);
        set(side+"_rim_strike",(t.rim_strike_active||t.rim_event.has_value())?1.:0.);set(side+"_tyre_pressure_bar",t.pressure_bar);set(side+"_tyre_loss_w",t.dissipated_power_w);
    }
    if(p.drive) {
        const auto& c=p.drive->command();
        const std::array values{c.phase_rad,c.cadence_rpm,c.rider_torque_nm,c.assist_torque_nm,c.motor_power_w,c.support_factor,
            c.freewheel?1.:0.,c.cutoff_active?1.:0.,p.brake_source?1.:0.,p.traction_limited?1.:0.,static_cast<double>(c.gear_teeth)};
        std::ranges::copy(values,std::span{row}.subspan(drive_start).begin());
    }
    return row;
}
NATIVE_DIAG_POP
RideSnapshot RideSimulation::snapshot()const {
    const auto& p=*impl_;const auto& m=*p.stepper->model();const auto& d=*p.stepper->data();
    RideSnapshot result;result.time_s=d.time;result.position_m=position_m();result.speed_mps=p.stepper->qvel()[p.x_v];result.pitch_rad=p.stepper->qpos()[p.pitch_q];
    result.steps=p.steps;result.generation=p.generation;result.contacts=p.contacts;result.crash=p.crash;result.telemetry=telemetry();
    auto const copy=[](auto span){return std::vector<double>(span.begin(),span.end());};
    result.qpos=copy(p.stepper->qpos());result.qvel=copy(p.stepper->qvel());result.ctrl=copy(p.stepper->ctrl());result.qfrc_applied=copy(p.stepper->qfrc_applied());
    result.xfrc_applied=copy(std::views::counted(d.xfrc_applied,6*m.nbody));result.eq_data=copy(std::views::counted(m.eq_data,mjNEQDATA*m.neq));
    result.integration_state.resize(static_cast<std::size_t>(engine::state_size(&m,mjSTATE_INTEGRATION)));
    engine::get_state(&m,&d,result.integration_state.data(),mjSTATE_INTEGRATION);
    const int frame_id=mj_name2id(&m,mjOBJ_BODY,"frame");
    if(frame_id>=0)result.z_m=std::views::counted(d.xpos,3*m.nbody)[3U*static_cast<std::size_t>(frame_id)+2U];
    result.model_fields={copy(std::views::counted(m.dof_frictionloss,m.nv)),copy(std::views::counted(m.site_pos,3*m.nsite)),
        copy(std::views::counted(m.tendon_stiffness,m.ntendon)),copy(std::views::counted(m.tendon_damping,m.ntendon)),
        copy(std::views::counted(m.tendon_lengthspring,2*m.ntendon)),copy(std::views::counted(m.tendon_range,2*m.ntendon))};
    const auto eq=std::views::counted(d.eq_active,m.neq);result.eq_active.assign(eq.begin(),eq.end());return result;
}
RunLimits RideSimulation::default_limits(double wall)const{return RunLimits::for_track(impl_->options.setup.track.length_m,impl_->options.setup.start_x_m,timestep_s(),wall);}
RunOutcome RideSimulation::run(const RunLimits& limits,const std::function<void(const RideSimulation&)>& on_step,double front,double rear) {
    limits.validate();const auto started=std::chrono::steady_clock::now();
    while(true){const double wall=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
        if(const auto reason=termination_reason(limits,position_m(),steps(),wall,impl_->crash))return {.reason=*reason,.steps=steps(),.sim_time_s=time_s(),.wall_clock_s=wall,.position_m=position_m(),.crash=impl_->crash};
        step(front,rear);if(on_step)on_step(*this);
    }
}
runtime::ModelImage RideSimulation::model_image()const{return initialization::capture_model_image(*impl_->stepper->model());}
io::Value RideSimulation::metadata()const {
    using namespace runtime::setupvalue;const auto& p=*impl_;
    return Value{Object{{"physics_revision",text("legacy-v1")},{"physics_mode",text("legacy")},
        { // NOLINTNEXTLINE(bugprone-unchecked-optional-access) setup schema guarantees rider engagement
        "rider_variant",text(p.options.setup.model.rider.value().variant)},{"drive_mode",text(p.options.drive_mode)},
        {"tire_model",text(p.options.tyre.model)},{"timestep_s",real(timestep_s())},
        {"equilibrium_steps",integer(p.equilibrium.pose.steps)},{"equilibrium_residual_qacc",real(p.equilibrium.pose.residual_qacc)},
        { // NOLINTNEXTLINE(bugprone-unchecked-optional-access) setup schema guarantees physics engagement
        "physics",configuration::to_value(p.options.setup.model.physics.value())}}};
}
} // namespace bike::legacy
