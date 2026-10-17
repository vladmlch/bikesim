#include "playground.hpp"
#include "../engine_call.hpp"
#include "../initialization/capture.hpp"
#include "../model/mapping.hpp"
#include "../numeric_sincos.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <mutex>
#include <numbers>
#include <ranges>
#include <stdexcept>
#include <utility>

namespace bike::legacy {
namespace {
double wall_time(){return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();}
model::AirSpringSpecs air_specs(const geometry::BikeSpecs& s){model::AirSpringSpecs result;result.total_travel_mm=s.fork_travel;return result;}
model::DamperClickConfig damper_clicks(const geometry::BikeSpecs& s){return {s.fork_hsc,s.fork_lsc,s.fork_rebound,s.shock_hsc,s.shock_lsc,s.shock_rebound,s.shock_hbo,s.shock_lockout,std::string(model::damper_preset_label(model::DamperPreset::base))};}
struct DataDeleter {void operator()(mjData* data)const noexcept{if(data)mj_deleteData(data);}};
struct FrameProperties {double mass=0.;std::array<double,3> position{},inertia{};};
FrameProperties frame_properties(const mjModel& m){
    const auto id=static_cast<std::size_t>(model::required_id(m,mjOBJ_BODY,"frame"));FrameProperties out;
    out.mass=std::views::counted(m.body_mass,m.nbody)[id];
    std::ranges::copy(std::views::counted(m.body_ipos,3*m.nbody).subspan(3*id,3),out.position.begin());
    std::ranges::copy(std::views::counted(m.body_inertia,3*m.nbody).subspan(3*id,3),out.inertia.begin());return out;
}
}
struct Playground::Impl {
    struct State {
        model::ModelOwner model;
        std::unique_ptr<mjData,DataDeleter> data;
        SimModelHandles handles;
        int rear_actuator=-1,fork_actuator=-1;
        std::array<FrameProperties,2> frame;
        std::vector<std::pair<int,std::array<double,3>>> markers;
        bool poisoned=false;
    };
    PlaygroundOptions options;
    kinematics::HorstLinkageSolver solver;
    model::ForkAirSpring air;
    model::BikeSuspensionSystem suspension;
    std::unique_ptr<State> state;
    mutable std::recursive_mutex mutex;
    std::deque<int> pending_keys;
    double rear_mm=0.,fork_mm=0.,sweep_frequency=.4,sweep_start=0.;
    bool auto_sweep=false,debug_markers=false,show_telemetry=true;
    std::string camera_mode="2d";
    std::uint64_t generation=0;
    explicit Impl(PlaygroundOptions o):options(std::move(o)),solver(options.specs),
        air(air_specs(options.specs),options.specs.fork_air_tokens,options.specs.fork_initial_psi),
        suspension(damper_clicks(options.specs),180.,65.,true){}
    void live()const{if(!state||state->poisoned)throw engine::EngineFailure("playground owner is unavailable after an engine failure");}
    void apply_markers(State& current){
        auto& m=*current.model;auto sizes=std::views::counted(m.geom_size,3*m.ngeom);auto colors=std::views::counted(m.geom_rgba,4*m.ngeom);
        for(const auto& [raw_id,size]:current.markers){const auto id=static_cast<std::size_t>(raw_id);colors[4*id+3]=debug_markers?1.f:0.f;
            for(std::size_t j=0;j<3;++j)sizes[3*id+j]=debug_markers?size[j]:.0001;}
    }
    void apply_rider(State& current){
        auto& m=*current.model;auto& d=*current.data;const auto frame=static_cast<std::size_t>(model::required_id(m,mjOBJ_BODY,"frame"));
        const auto& properties=current.frame[options.include_rider?1:0];
        std::views::counted(m.body_mass,m.nbody)[frame]=properties.mass;
        std::ranges::copy(properties.position,std::views::counted(m.body_ipos,3*m.nbody).begin()+static_cast<std::ptrdiff_t>(3*frame));
        std::ranges::copy(properties.inertia,std::views::counted(m.body_inertia,3*m.nbody).begin()+static_cast<std::ptrdiff_t>(3*frame));
        constexpr std::array names{"geom_rider_torso","geom_rider_legs","geom_rider_arms"};
        constexpr std::array<std::array<double,3>,3> dimensions{{{.13,.18027756,0.},{.075,.24515301,0.},{.055,.12971122,0.}}};
        auto sizes=std::views::counted(m.geom_size,3*m.ngeom);auto colors=std::views::counted(m.geom_rgba,4*m.ngeom);
        for(std::size_t k=0;k<names.size();++k){const int raw=mj_name2id(&m,mjOBJ_GEOM,names[k]);if(raw<0)continue;const auto id=static_cast<std::size_t>(raw);
            if(options.include_rider){for(std::size_t j=0;j<3;++j){sizes[3*id+j]=dimensions[k][j];colors[4*id+j]=.5f;}colors[4*id+3]=1.f;}
            else {colors[4*id+3]=0.f;for(std::size_t j=0;j<3;++j)sizes[3*id+j]=.0001;std::views::counted(m.geom_contype,m.ngeom)[id]=0;std::views::counted(m.geom_conaffinity,m.ngeom)[id]=0;}}
        engine::set_const(&m,&d);engine::forward(&m,&d);
    }
    void load(std::string mode){
        if(mode!="standard"&&mode!="stand"&&mode!="playground")throw std::invalid_argument("playground model must be standard, stand or playground");
        model::BuildRequest request;request.bike=options.specs;request.solver=solver;request.mode=mode;request.debug_markers=true;
        std::array<std::string,2> xml;auto next=std::make_unique<State>();
        for(std::size_t i=0;i<2;++i){request.include_rider=i!=0;xml[i]=model::generate_mjcf(request);const auto compiled=model::compile_mjcf(xml[i],options.asset_root);next->frame[i]=frame_properties(*compiled);}
        next->model=model::compile_mjcf(xml[options.include_rider?1:0],options.asset_root);next->data.reset(engine::make_data(next->model.get()));
        auto& m=*next->model;engine::forward(&m,next->data.get());next->handles=SimModelHandles::from_model(m);
        if(next->handles.pra<0)throw std::invalid_argument("model has no site_PRA; rear-axle telemetry cannot be referenced");
        next->rear_actuator=mj_name2id(&m,mjOBJ_ACTUATOR,"rear_travel_actuator");next->fork_actuator=mj_name2id(&m,mjOBJ_ACTUATOR,"fork_travel_actuator");
        for(int raw=0;raw<m.ngeom;++raw){const char* name=mj_id2name(&m,mjOBJ_GEOM,raw);if(name&&std::string_view(name).starts_with("marker_")){
            std::array<double,3> size{};std::ranges::copy(std::views::counted(m.geom_size,3*m.ngeom).subspan(static_cast<std::size_t>(3*raw),3),size.begin());next->markers.emplace_back(raw,size);}}
        apply_markers(*next);apply_rider(*next);reset_stand(m,*next->data,solver,next->rear_actuator,next->fork_actuator);
        state=std::move(next);options.model_mode=std::move(mode);rear_mm=0.;fork_mm=0.;auto_sweep=false;++generation;
    }
};
Playground::Playground(std::unique_ptr<Impl> impl):impl_(std::move(impl)){}
Playground::~Playground()=default;
std::unique_ptr<Playground> make_playground(PlaygroundOptions options){auto impl=std::make_unique<Playground::Impl>(std::move(options));impl->load(impl->options.model_mode);return std::unique_ptr<Playground>(new Playground(std::move(impl)));}
void Playground::set_model(std::string mode){auto& p=*impl_;std::lock_guard lock(p.mutex);p.load(std::move(mode));}
void Playground::reset(){auto& p=*impl_;std::lock_guard lock(p.mutex);p.live();try{reset_stand(*p.state->model,*p.state->data,p.solver,p.state->rear_actuator,p.state->fork_actuator);}catch(const engine::EngineFailure&){p.state->poisoned=true;throw;}p.rear_mm=0.;p.fork_mm=0.;p.auto_sweep=false;++p.generation;}
void Playground::step(){step_at(wall_time());}
void Playground::step_at(double now){
    if(!std::isfinite(now))throw std::invalid_argument("sweep clock must be finite");auto& p=*impl_;std::lock_guard lock(p.mutex);p.live();
    while(!p.pending_keys.empty()){const int key=p.pending_keys.front();p.pending_keys.pop_front();(void)handle_key(key,now);}
    double rear=p.rear_mm,fork=p.fork_mm;if(p.auto_sweep){const double phase=(numeric::sin(2.*std::numbers::pi*p.sweep_frequency*(now-p.sweep_start)-std::numbers::pi/2.)+1.)/2.;rear=phase*180.;fork=phase*180.;}
    try{apply_stand_targets(*p.state->model,*p.state->data,p.solver,p.state->rear_actuator,p.state->fork_actuator,rear,fork);p.rear_mm=rear;p.fork_mm=fork;engine::step(p.state->model.get(),p.state->data.get());}
    catch(const engine::EngineFailure&){p.state->poisoned=true;throw;}
}
void Playground::set_travel(double rear,double fork){if(!std::isfinite(rear)||!std::isfinite(fork))throw std::invalid_argument("stand targets must be finite");auto& p=*impl_;std::lock_guard lock(p.mutex);p.rear_mm=std::clamp(rear,0.,180.);p.fork_mm=std::clamp(fork,0.,180.);p.auto_sweep=false;}
void Playground::set_auto_sweep(bool enabled,double now,double frequency){if(!std::isfinite(now)||!std::isfinite(frequency)||frequency<=0.)throw std::invalid_argument("sweep requires finite clock and positive frequency");auto& p=*impl_;std::lock_guard lock(p.mutex);p.auto_sweep=enabled;p.sweep_start=now;p.sweep_frequency=frequency;}
bool Playground::toggle_rider(){auto& p=*impl_;std::lock_guard lock(p.mutex);p.live();p.options.include_rider=!p.options.include_rider;try{p.apply_rider(*p.state);}catch(const engine::EngineFailure&){p.state->poisoned=true;throw;}++p.generation;return p.options.include_rider;}
bool Playground::toggle_debug_markers(){auto& p=*impl_;std::lock_guard lock(p.mutex);p.live();p.debug_markers=!p.debug_markers;p.apply_markers(*p.state);try{engine::forward(p.state->model.get(),p.state->data.get());}catch(const engine::EngineFailure&){p.state->poisoned=true;throw;}return p.debug_markers;}
void Playground::set_fork_pressure(double psi){auto& p=*impl_;std::lock_guard lock(p.mutex);p.air.set_pressure_psi(psi);}
void Playground::set_fork_tokens(int tokens){auto& p=*impl_;std::lock_guard lock(p.mutex);p.air.set_tokens(tokens);}
void Playground::set_fork_clicks(std::optional<int> hsc,std::optional<int> lsc,std::optional<int> rebound){auto& p=*impl_;std::lock_guard lock(p.mutex);p.suspension.fork_damper.set_clicks(hsc,lsc,rebound);}
void Playground::set_shock_clicks(std::optional<int> hsc,std::optional<int> lsc,std::optional<int> rebound,std::optional<int> hbo,std::optional<bool> lockout){auto& p=*impl_;std::lock_guard lock(p.mutex);p.suspension.shock_damper.set_clicks(hsc,lsc,rebound,hbo,lockout);}
void Playground::apply_preset(model::DamperPreset preset){
    if(preset!=model::DamperPreset::base&&preset!=model::DamperPreset::plush&&preset!=model::DamperPreset::enduro&&preset!=model::DamperPreset::park&&preset!=model::DamperPreset::custom)
        throw std::invalid_argument("unknown suspension preset");
    auto& p=*impl_;std::lock_guard lock(p.mutex);p.suspension.apply_preset(preset);
}
model::DamperPreset Playground::cycle_preset(bool forward){auto& p=*impl_;std::lock_guard lock(p.mutex);return p.suspension.cycle_preset(forward);}
void Playground::set_camera_mode(std::string_view mode){auto& p=*impl_;std::lock_guard lock(p.mutex);if(mode=="2d"||mode=="3d")p.camera_mode=mode;}
void Playground::enqueue_key(int key){auto& p=*impl_;std::lock_guard lock(p.mutex);p.pending_keys.push_back(key);}
bool Playground::handle_key(int key,double now){
    auto& p=*impl_;std::lock_guard lock(p.mutex);if(key>='a'&&key<='z')key-=32;
    const auto& fd=p.suspension.fork_damper;const auto& sd=p.suspension.shock_damper;
    switch(key){
    case 'B':(void)toggle_rider();break;case 'C':set_camera_mode(p.camera_mode=="2d"?"3d":"2d");break;
    case '1':case 321:set_camera_mode("2d");break;case '2':case 322:set_camera_mode("3d");break;case 'P':(void)cycle_preset();break;
    case 'L':set_fork_clicks({},fd.lsc_clicks+1,{});break;case 'K':set_fork_clicks({},fd.lsc_clicks-1,{});break;
    case 'J':set_fork_clicks(fd.hsc_clicks+1,{},{});break;case 'H':set_fork_clicks(fd.hsc_clicks-1,{},{});break;
    case 'U':set_fork_clicks({},{},fd.rebound_clicks+1);break;case 'Y':set_fork_clicks({},{},fd.rebound_clicks-1);break;
    case 'X':set_shock_clicks({},{},{},{},!sd.lockout_firm);break;
    case '0':set_shock_clicks({},sd.lsc_clicks+1,{});break;case '9':set_shock_clicks({},sd.lsc_clicks-1,{});break;
    case '8':set_shock_clicks(sd.hsc_clicks+1,{},{});break;case '7':set_shock_clicks(sd.hsc_clicks-1,{},{});break;
    case '6':set_shock_clicks({},{},sd.rebound_clicks+1);break;case '5':set_shock_clicks({},{},sd.rebound_clicks-1);break;
    case '4':set_shock_clicks({},{},{},sd.hbo_clicks+1);break;case '3':set_shock_clicks({},{},{},sd.hbo_clicks-1);break;
    case 'W':set_travel(p.rear_mm+5.,p.fork_mm);break;case 'S':set_travel(p.rear_mm-5.,p.fork_mm);break;
    case 265:set_travel(p.rear_mm,p.fork_mm+5.);break;case 264:set_travel(p.rear_mm,p.fork_mm-5.);break;
    case '[':set_fork_tokens(std::clamp(p.air.num_tokens()-1,0,p.air.specs().max_tokens));break;
    case ']':set_fork_tokens(std::clamp(p.air.num_tokens()+1,0,p.air.specs().max_tokens));break;
    case '-':set_fork_pressure(std::clamp(p.air.gauge_pressure_psi()-2.,10.,200.));break;
    case '=':set_fork_pressure(std::clamp(p.air.gauge_pressure_psi()+2.,10.,200.));break;
    case ' ':set_auto_sweep(!p.auto_sweep,now,p.sweep_frequency);break;case 'R':reset();break;
    case 'T':p.show_telemetry=!p.show_telemetry;break;case 'G':(void)toggle_debug_markers();break;
    default:return false;
    }return true;
}
io::Value Playground::telemetry()const{const auto& p=*impl_;std::lock_guard lock(p.mutex);p.live();TelemetryProvider provider(p.options.specs,p.solver,p.air,p.suspension);return provider.compute(*p.state->model,*p.state->data,p.rear_mm,p.options.include_rider,p.state->handles,p.camera_mode);}
PlaygroundSnapshot Playground::snapshot()const{
    const auto& p=*impl_;std::lock_guard lock(p.mutex);p.live();const auto& m=*p.state->model;const auto& d=*p.state->data;
    PlaygroundSnapshot out;out.time_s=d.time;out.target_rear_travel_mm=p.rear_mm;out.target_fork_travel_mm=p.fork_mm;out.include_rider=p.options.include_rider;out.auto_sweep=p.auto_sweep;out.debug_markers=p.debug_markers;out.show_telemetry=p.show_telemetry;out.generation=p.generation;out.model_mode=p.options.model_mode;out.camera_mode=p.camera_mode;
    const auto copy=[](auto span){return std::vector<double>(span.begin(),span.end());};out.qpos=copy(std::views::counted(d.qpos,m.nq));out.qvel=copy(std::views::counted(d.qvel,m.nv));out.ctrl=copy(std::views::counted(d.ctrl,m.nu));
    out.timestep_s=m.opt.timestep;
    out.integration_state.resize(static_cast<std::size_t>(engine::state_size(&m,mjSTATE_INTEGRATION)));
    engine::get_state(&m,&d,out.integration_state.data(),mjSTATE_INTEGRATION);
    out.model_fields={copy(std::views::counted(m.dof_frictionloss,m.nv)),copy(std::views::counted(m.site_pos,3*m.nsite)),
        copy(std::views::counted(m.tendon_stiffness,m.ntendon)),copy(std::views::counted(m.tendon_damping,m.ntendon)),
        copy(std::views::counted(m.tendon_lengthspring,2*m.ntendon)),copy(std::views::counted(m.tendon_range,2*m.ntendon))};
    out.telemetry=telemetry();return out;
}
runtime::ModelImage Playground::model_image()const{const auto& p=*impl_;std::lock_guard lock(p.mutex);p.live();return initialization::capture_model_image(*p.state->model);}
void step(Playground& playground){playground.step();}
void reset(Playground& playground){playground.reset();}
} // namespace bike::legacy
