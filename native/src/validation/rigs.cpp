#include "rigs.hpp"
#include "acceptance.hpp"
#include "evidence.hpp"
#include "../contact/laws.hpp"
#include "../cblas_abi.hpp"
#include "../drivetrain/chain.hpp"
#include "../drivetrain/freehub.hpp"
#include "../drivetrain/transmission.hpp"
#include "../geometry/specs.hpp"
#include "../numerics/brent.hpp"
#include "../numerics/density_fit.hpp"
#include "../numeric_numpy.hpp"
#include "../writers/distributed_tire.hpp"
#include <charconv>
#include <numbers>

namespace bike::validation {
std::string xml_number(double x) {
    scalar(x,"XML number");std::array<char,64> buffer{};
    const auto [end,error]=std::to_chars(buffer.data(),buffer.data()+buffer.size(),x,std::chars_format::general,17);
    if(error!=std::errc{})throw std::runtime_error("XML number formatting failed");return {buffer.data(),end};
}
double vector_dot(std::span<const double> a,std::span<const double> b) {
    if(a.size()!=b.size()||a.size()>static_cast<std::size_t>(std::numeric_limits<int>::max()))throw std::invalid_argument("rig dot shape mismatch");
    return 0.+blas::ddot(static_cast<int>(a.size()),a.data(),1,b.data(),1);
}
double vector_norm(std::span<const double> a){return std::sqrt(vector_dot(a,a));}
int object_id(const mjModel* model,mjtObj kind,const char* name){const int id=mj_name2id(model,kind,name);if(id<0)throw std::invalid_argument(std::string("rig missing object ")+name);return id;}
namespace {
double normal_load(const TireSnapshot& snapshot){numeric::PythonSum total;for(const auto& p:snapshot.patches)total.add(p.normal_load_n);return total.total();}
double vertical_force(const TireSnapshot& snapshot){double total=0.;for(const auto& p:snapshot.patches)total+=p.native_world_force_n?(*p.native_world_force_n)[2]:p.normal_load_n*p.normal[2]-p.tangent_force_n*p.normal[0];return total;}
Value metrics(std::initializer_list<std::pair<std::string,double>> values){Object result;for(const auto& [name,x]:values)result.emplace_back(name,value::real(x));return Value{std::move(result)};}
Value bounds(std::initializer_list<std::tuple<std::string,double,double>> values){Object result;for(const auto& [name,low,high]:values)result.emplace_back(name,pair(low,high));return Value{std::move(result)};}
std::string option(double dt,std::string_view gravity="0 0 0"){positive(dt,"dt");return "<option timestep=\""+xml_number(dt)+"\" gravity=\""+std::string(gravity)+"\"/>";}
std::string slider(double dt,std::string_view axis="0 0 1",double mass=2.4,std::string_view gravity="0 0 0"){
    return "<mujoco>"+option(dt,gravity)+"<worldbody><body><joint type=\"slide\" axis=\""+std::string(axis)+"\"/><inertial mass=\""+xml_number(mass)+"\" pos=\"0 0 0\" diaginertia=\".1 .2 .1\"/></body></worldbody></mujoco>";
}
std::array<double,3> cross(const std::array<double,3>& a,const std::array<double,3>& b){return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};}
std::array<double,3> matvec(const std::vector<double>& matrix,std::span<const double> vector){
    std::array<double,3> out{};const int n=static_cast<int>(vector.size());blas::dgemv(blas::Order::row_major,blas::Transpose::no,3,n,1.,matrix.data(),n,vector.data(),1,0.,out.data(),1);return out;
}
void add_transpose(std::span<double> output,const std::vector<double>& j,const std::array<double,3>& force){
    std::vector<double> terms(output.size());const int n=static_cast<int>(output.size());blas::dgemv(blas::Order::row_major,blas::Transpose::yes,3,n,1.,j.data(),n,force.data(),1,0.,terms.data(),1);for(std::size_t i=0;i<output.size();++i)output[i]+=terms[i];
}
} // namespace
Value radial_rig(double load,double dt,double duration,bool native_reference){
    positive(load,"radial load");const auto count=steps(duration,dt);
    const auto xml=native_reference?"<mujoco>"+option(dt)+"<worldbody><geom type=\"plane\" size=\"1 1 .1\"/><body pos=\"0 0 .353\"><joint type=\"slide\" axis=\"0 0 1\"/><inertial mass=\"2.4\" pos=\"0 0 0\" diaginertia=\".1 .2 .1\"/><geom type=\"sphere\" size=\".352\" mass=\"0\" solref=\"-130000 -800\" priority=\"1\"/></body></worldbody></mujoco>":slider(dt);
    Stand s(xml);auto* m=s.model.get();auto* d=s.data.get();
    if(native_reference)d->qfrc_applied[0]=-load;
    for(std::size_t i=0;i<count;++i){if(!native_reference)d->qfrc_applied[0]=contactlaw::normal_contact(-d->qpos[0],-d->qvel[0],130000.,800.).first-load;engine::step(m,d);}
    if(native_reference)return metrics({{"load_n",load},{"dt_s",dt},{"effective_deflection_m",-d->qpos[0]-.001},{"speed_mps",d->qvel[0]}});
    return metrics({{"load_n",load},{"dt_s",dt},{"deflection_m",-d->qpos[0]},{"speed_mps",d->qvel[0]},{"expected_deflection_m",load/130000.}});
}
Value wheel_inertia_rig(double dt,double duration){
    const auto count=steps(duration,dt);Stand s("<mujoco>"+option(dt)+"<worldbody><body><joint type=\"hinge\" axis=\"0 1 0\"/><inertial mass=\"2.4\" pos=\"0 0 0\" diaginertia=\".12 .24 .12\"/></body></worldbody></mujoco>");
    auto* m=s.model.get();auto* d=s.data.get();d->qfrc_applied[0]=3.;for(std::size_t i=0;i<count;++i)engine::step(m,d);
    const double expected=3./.24*duration;return metrics({{"omega_rad_s",d->qvel[0]},{"expected_omega_rad_s",expected},
        {"acceleration_relative_error",std::abs(d->qvel[0]/duration-3./.24)/(3./.24)},{"rotational_energy_j",.5*.24*d->qvel[0]*d->qvel[0]},{"expected_energy_j",.5*.24*expected*expected}});
}
Value incline_rig(double dt,double duration,double degrees){
    const auto count=steps(duration,dt);scalar(degrees,"incline angle");const double angle=degrees*std::numbers::pi/180.;
    const double sn=std::sin(angle),cs=std::cos(angle);Stand s(slider(dt,xml_number(-sn)+" 0 "+xml_number(cs),20.,"0 0 -9.81"));
    auto* m=s.model.get();auto* d=s.data.get();double force=0.;for(std::size_t i=0;i<count;++i){force=contactlaw::normal_contact(-d->qpos[0],-d->qvel[0],130000.,800.).first;d->qfrc_applied[0]=force;engine::step(m,d);}
    const double tangent=20.*9.81*sn;return metrics({{"normal_load_n",force},{"expected_normal_load_n",20.*9.81*cs},{"normal_vertical_n",force*cs},
        {"vertical_with_stand_reaction_n",force*cs+tangent*sn},{"expected_weight_n",20.*9.81},{"deflection_m",-d->qpos[0]}});
}
Value brake_rig(double dt,double duration){
    const auto count=steps(duration,dt);Stand s("<mujoco>"+option(dt)+"<worldbody><body><joint type=\"hinge\" axis=\"0 1 0\" frictionloss=\"20\" solreffriction=\".005 1\" solimpfriction=\".9999 .9999 .001 .5 2\"/><inertial mass=\"1\" pos=\"0 0 0\" diaginertia=\"1 1 1\"/></body></worldbody></mujoco>");
    auto* m=s.model.get();auto* d=s.data.get();d->qfrc_applied[0]=10.;for(std::size_t i=0;i<count;++i)engine::step(m,d);const double held=d->qpos[0];double work=0.;d->qfrc_applied[0]=30.;
    for(std::size_t i=0,n=steps(.1,dt);i<n;++i){const double speed=d->qvel[0];engine::step(m,d);work+=d->qfrc_constraint[0]*speed*dt;}
    return metrics({{"held_angle_rad",held},{"overload_speed_rad_s",d->qvel[0]},{"brake_work_j",work}});
}
Value chain_locked_rig(double dt,double duration){
    const auto count=steps(duration,dt);const configuration::DrivetrainSpecs gear;
    const double rf=gear.front_radius_m(),rr=gear.rear_radius_m(),ratio=rf/rr,load=10.,tension=load/rr,k=200000.;
    std::string xml="<mujoco>"+option(dt)+"<worldbody>";const std::array inertias{.02,.005,.2};
    for(std::size_t i=0;i<inertias.size();++i){const double inertia=inertias[i];xml+="<body pos=\""+std::to_string(i)+" 0 0\"><joint type=\"hinge\" axis=\"0 1 0\"/><inertial mass=\"1\" pos=\"0 0 0\" diaginertia=\""+xml_number(inertia/2.)+" "+xml_number(inertia)+" "+xml_number(inertia/2.)+"\"/></body>";}
    xml+="</worldbody></mujoco>";Stand s(xml);auto* m=s.model.get();auto* d=s.data.get();drivetrain::Freehub hub(1000.,0.);
    d->qpos[1]=load/1000.;d->qpos[0]=(tension/k+rr*d->qpos[1])/rf;d->qvel[0]=10.;d->qvel[1]=10.*ratio;d->qvel[2]=10.*ratio;
    hub.set_state(drivetrain::FreehubSnapshot{.boundary=0.,.energy_j=0.,.torque_nm=0.});double input=0.,output=0.,last_t=0.,last_h=0.;
    for(std::size_t i=0;i<count;++i){const double e=rf*d->qpos[0]-rr*d->qpos[1];last_t=k*std::max(e,0.);last_h=hub.update(d->qpos[1],d->qpos[2],d->qvel[1],d->qvel[2]);
        d->qfrc_applied[0]=load*ratio-last_t*rf;d->qfrc_applied[1]=last_t*rr-last_h;d->qfrc_applied[2]=last_h-load;
        input+=load*ratio*d->qvel[0]*dt;output+=load*d->qvel[2]*dt;engine::step(m,d);}
    return metrics({{"speed_ratio",d->qvel[2]/d->qvel[0]},{"expected_ratio",ratio},{"torque_ratio",last_t*rf/last_h},
        {"power_relative_error",std::abs(input-output)/std::abs(input)},{"cassette_wheel_speed_difference",d->qvel[1]-d->qvel[2]}});
}
Value chain_geometry_rig(){
    const drivetrain::Vec2 front{0.,0.},rear{-.45,.02};const double rf=.07,rr=.04,angle=.41;
    const double base=drivetrain::chain_extension(front,rear,rf,rr,0.,0.,0.);const double cs=std::cos(angle),sn=std::sin(angle);
    const double rotated=drivetrain::chain_extension({3.,3.},{cs*rear[0]-sn*rear[1]+3.,sn*rear[0]+cs*rear[1]+3.},rf,rr,-angle,-angle,base,{-sn,cs});
    const auto evaluate=[&](std::span<const double> q){return drivetrain::chain_extension(front,{q[0],q[1]},rf,rr,q[2],q[3],base);};
    const std::array q{rear[0],rear[1],.1,.2},direction{.7,-.3,.2,-.1};const auto j=drivetrain::chain_jacobian(q,evaluate);const double h=2e-6;
    auto low=q,high=q;for(std::size_t i=0;i<q.size();++i){low[i]-=h*direction[i];high[i]+=h*direction[i];}
    const double independent=(evaluate(high)-evaluate(low))/(2.*h);
    return metrics({{"rigid_motion_extension_m",rotated},{"directional_relative_error",std::abs(vector_dot(j,direction)-independent)/std::max(std::abs(independent),1e-8)},
        {"suspension_chain_gradient_x",j[0]},{"suspension_chain_gradient_z",j[1]}});
}
Value contact_event_rig(double dt){
    Stand s(slider(dt));auto* m=s.model.get();auto* d=s.data.get();d->qpos[0]=.01;d->qvel[0]=-1.;std::optional<double> event;
    for(std::size_t i=0,n=steps(.03,dt);i<n;++i){const auto force=contactlaw::normal_contact(-d->qpos[0],-d->qvel[0],130000.,800.).first;if(force>0.&&!event)event=d->time;d->qfrc_applied[0]=force;engine::step(m,d);}
    if(!event)throw std::runtime_error("wheel never contacted the stand");return metrics({{"contact_time_s",*event},{"analytic_contact_time_s",.01},{"event_error_s",std::abs(*event-.01)}});
}
double quasistatic_front_load(double mass,double wheelbase,double x,double h,double slope,double accel,double gravity){
    for(double v:{mass,wheelbase,x,h,slope,accel,gravity})scalar(v,"load transfer input");
    if(mass<=0.||wheelbase<=0.||h<0.||gravity<=0.)throw std::invalid_argument("invalid rigid-equivalent geometry");
    return mass/wheelbase*(gravity*(x*std::cos(slope)-h*std::sin(slope))-accel*h);
}
double necessary_traction_margin(double mu,double rear,double grade){nonnegative(mu,"friction coefficient");nonnegative(rear,"rear normal-load fraction");nonnegative(grade,"uphill grade");if(rear>1.)throw std::invalid_argument("rear normal-load fraction must not exceed one");return mu*rear-grade;}
RigResult load_transfer_rig(double dt,double slope,double x,double h){
    constexpr double mass=100.,length=1.2,g=9.81,k=200000.,c=3000.,kt=100000.,ct=1500.;const auto count=steps(.5,dt);
    const double front=quasistatic_front_load(mass,length,x,h,slope),rear=mass*g*std::cos(slope)-front;
    if(front<=0.||rear<=0.)throw std::invalid_argument("two loaded supports do not exist for this rigid-equivalent fixture");
    Stand s("<mujoco><compiler angle=\"radian\"/>"+option(dt,"0 0 -9.81")+"<worldbody><body name=\"rigid_bike\"><joint name=\"x\" type=\"slide\" axis=\"1 0 0\"/><joint name=\"z\" type=\"slide\" axis=\"0 0 1\"/><joint name=\"pitch\" type=\"hinge\" axis=\"0 1 0\"/><inertial mass=\"100\" pos=\""+xml_number(x)+" 0 "+xml_number(h)+"\" diaginertia=\"10 20 10\"/><site name=\"rear\" pos=\"0 0 0\"/><site name=\"front\" pos=\"1.2 0 0\"/></body></worldbody></mujoco>");
    auto* m=s.model.get();auto* d=s.data.get();const auto nv=static_cast<std::size_t>(m->nv);const std::array normal{-std::sin(slope),0.,std::cos(slope)},tangent{std::cos(slope),0.,std::sin(slope)};
    d->qpos[0]=-rear/k*normal[0];d->qpos[1]=-rear/k*normal[2];d->qpos[2]=-slope+std::asin((front-rear)/(k*length));engine::forward(m,d);
    const int body=object_id(m,mjOBJ_BODY,"rigid_bike");const std::array sites{object_id(m,mjOBJ_SITE,"front"),object_id(m,mjOBJ_SITE,"rear")};const std::array loads{front,rear};std::array<std::array<double,3>,2> anchors{};
    for(std::size_t side=0;side<2;++side){const double share=loads[side]/(mass*g*std::cos(slope)),needed=mass*g*std::sin(slope)*share;for(std::size_t a=0;a<3;++a)anchors[side][a]=d->site_xpos[3*sites[side]+static_cast<int>(a)]+tangent[a]*(needed/kt);}
    std::vector<double> jacobian(3*nv);std::array<double,2> last{};std::array<std::array<double,3>,2> forces{};std::array<double,3> moment{};
    for(std::size_t i=0;i<count;++i){std::fill_n(d->qfrc_applied,nv,0.);moment={};
        for(std::size_t side=0;side<2;++side){std::array<double,3> p{},displacement{},arm{};for(std::size_t a=0;a<3;++a)p[a]=d->site_xpos[3*sites[side]+static_cast<int>(a)];
            mj_jac(m,d,jacobian.data(),nullptr,p.data(),body);const auto velocity=matvec(jacobian,{d->qvel,nv});
            const double force=contactlaw::normal_contact(-vector_dot(p,normal),-vector_dot(velocity,normal),k,c).first;
            for(std::size_t a=0;a<3;++a)displacement[a]=p[a]-anchors[side][a];const double shear=vector_dot(displacement,tangent);
            const double tangential=std::clamp(-kt*shear-ct*vector_dot(velocity,tangent),-force,force);
            for(std::size_t a=0;a<3;++a){forces[side][a]=force*normal[a]+tangential*tangent[a];arm[a]=p[a]-d->xipos[3*body+static_cast<int>(a)];}
            add_transpose({d->qfrc_applied,nv},jacobian,forces[side]);last[side]=force;const auto torque=cross(arm,forces[side]);for(std::size_t a=0;a<3;++a)moment[a]+=torque[a];}
        engine::step(m,d);engine::forward(m,d);
    }
    std::array<double,3> total{};for(std::size_t a=0;a<3;++a)total[a]=forces[0][a]+forces[1][a]+mass*m->opt.gravity[a];
    return {metrics({{"front_load_n",last[0]},{"rear_load_n",last[1]},{"expected_front_load_n",front},{"expected_rear_load_n",rear},
        {"load_error_over_weight",std::max(std::abs(last[0]-front),std::abs(last[1]-rear))/(mass*g)},{"force_balance_over_weight",vector_norm(total)/(mass*g)},
        {"moment_balance_over_weight_wheelbase",vector_norm(moment)/(mass*g*length)},{"free_pitch_coordinate_count",1.},{"hidden_stand_pitch_constraints",static_cast<double>(m->neq)},{"final_pitch_rate_rad_s",d->qvel[2]}}),
        bounds({{"load_error_over_weight",0.,.01},{"force_balance_over_weight",0.,.01},{"moment_balance_over_weight_wheelbase",0.,.01},{"hidden_stand_pitch_constraints",0.,0.}})};
}
RigResult shaft_ratio_rig(double dt,double ratio,bool overrun){
    positive(ratio,"gear ratio");const auto count=steps(.1,dt);constexpr double jc=.04,jw=.24;
    Stand s("<mujoco>"+option(dt)+"<worldbody><body><joint name=\"crank_spin\" axis=\"0 1 0\"/><inertial mass=\"1\" pos=\"0 0 0\" diaginertia=\".04 .04 .04\"/></body><body><joint name=\"rear_wheel_spin\" axis=\"0 1 0\"/><inertial mass=\"1\" pos=\"0 0 0\" diaginertia=\".24 .24 .24\"/></body></worldbody><tendon><fixed name=\"ideal_mid_drive_freehub\" limited=\"true\" range=\"-1e12 0\" solreflimit=\".0025 1\" solimplimit=\".9999 .9999 .001 .5 2\"><joint joint=\"crank_spin\" coef=\""+xml_number(ratio)+"\"/><joint joint=\"rear_wheel_spin\" coef=\"-1\"/></fixed></tendon><actuator><motor name=\"mid_drive\" joint=\"crank_spin\" gear=\"1\"/></actuator></mujoco>");
    auto* m=s.model.get();auto* d=s.data.get();
    const double torque=overrun?0.:20.;if(overrun)d->qvel[1]=20.;double boundary=ratio*d->qpos[0]-d->qpos[1];m->tendon_range[1]=boundary;double power_error=0.,reaction_peak=0.;std::array<double,3> tail{};std::size_t tail_count=0;
    for(std::size_t i=0;i<count;++i){boundary=std::min(boundary,ratio*d->qpos[0]-d->qpos[1]);scalar(boundary,"transmission boundary");m->tendon_range[1]=boundary;const std::array velocity{d->qvel[0],d->qvel[1]};d->ctrl[0]=torque;engine::step(m,d);std::array<double,2> reaction{};std::vector<double> multipliers(static_cast<std::size_t>(d->nefc));bool selected=false;
        for(mjtSize row=0;row<d->nefc;++row)if(d->efc_type[row]==mjCNSTR_LIMIT_TENDON&&d->efc_id[row]==0){multipliers[static_cast<std::size_t>(row)]=d->efc_force[row];selected=true;}if(selected)engine::mul_jac_t_vec(m,d,reaction.data(),multipliers.data());
        power_error=std::max(power_error,std::abs(d->actuator_force[0]*velocity[0]-vector_dot({d->qfrc_actuator,2},velocity)));reaction_peak=std::max(reaction_peak,vector_norm(reaction));
        if(i>=count/2){tail[0]+=d->qacc[0];tail[1]+=d->qacc[1];tail[2]+=d->qfrc_actuator[0];++tail_count;}}
    for(double& x:tail)x/=static_cast<double>(tail_count);
    const double expected=torque/(jc+ratio*ratio*jw);auto b=bounds({{"acceleration_relative_error",0.,.001},{"speed_ratio_relative_error",0.,.001},{"shaft_power_identity_error_w",0.,1e-10}});
    if(overrun)value::set(std::get<Object>(b.data),"max_transmission_reaction_norm",pair(0.,1e-10));
    return {metrics({{"gear_ratio",ratio},{"delivered_crank_torque_nm",tail[2]},{"crank_acceleration_rad_s2",tail[0]},{"expected_crank_acceleration_rad_s2",expected},
        {"acceleration_relative_error",std::abs(tail[0]-expected)/std::max(1.,std::abs(expected))},{"speed_ratio_relative_error",overrun?0.:std::abs(d->qvel[1]/d->qvel[0]-ratio)/ratio},
        {"shaft_power_identity_error_w",power_error},{"max_transmission_reaction_norm",reaction_peak},{"overrun",overrun?1.:0.}}),std::move(b)};
}

namespace {
nativecfg::TireConfig distributed_config(int n,double mu=.8){
    nativecfg::TireConfig c;c.backend="distributed_2d_reference";c.surface_mode="configured";c.front={std::nullopt,20000.,mu,.2};c.rear=c.front;
    c.significant_delta_m=.0001;c.significance_fraction=.05;c.distinct_normal_deg=20.;
    nativecfg::HingeDensity density{{0.,.004,.012},{300000.,0.,0.},2000.,"synthetic density; not fitted or measured"};
    nativecfg::DistributedMaterial material{density,"none-synthetic",{0.,2000.},{0.,.04},"experimental_unvalidated"};
    c.distributed=nativecfg::DistributedConfig{n,density,material,material};return c;
}
struct WheelStand {std::unique_ptr<Stand> stand;std::unique_ptr<DistributedTireWriter> tire;};
WheelStand wheel_stand(double dt,int n,const std::vector<std::array<double,2>>& vertices){
    positive(dt,"stand step");if(n<16||n>512||vertices.size()<2)throw std::invalid_argument("invalid stand resolution");
    const double dx=vertices[1][0]-vertices[0][0];positive(dx,"heightfield spacing");double low=vertices.front()[1],high=low;
    for(std::size_t i=0;i<vertices.size();++i){scalar(vertices[i][0],"road x");scalar(vertices[i][1],"road z");if(i&&std::abs((vertices[i][0]-vertices[i-1][0])-dx)>1e-12+1e-9*std::abs(dx))throw std::invalid_argument("stand heightfield must be uniformly rasterized");low=std::min(low,vertices[i][1]);high=std::max(high,vertices[i][1]);}
    const double scale=std::max(.01,high-low),center=(vertices.front()[0]+vertices.back()[0])/2.,radius=(vertices.back()[0]-vertices.front()[0])/2.;
    auto s=std::make_unique<Stand>("<mujoco><compiler angle=\"radian\"/><option timestep=\""+xml_number(dt)+"\" gravity=\"0 0 0\" integrator=\"Euler\"/><asset><hfield name=\"road\" nrow=\"2\" ncol=\""+std::to_string(vertices.size())+"\" size=\""+xml_number(radius)+" 1 "+xml_number(scale)+" .2\"/></asset><worldbody><geom name=\"terrain\" type=\"hfield\" hfield=\"road\" pos=\""+xml_number(center)+" 0 "+xml_number(low)+"\"/><body name=\"front_wheel\" pos=\"0 0 .37\"><joint name=\"root_x\" type=\"slide\" axis=\"1 0 0\"/><joint name=\"root_z\" type=\"slide\" axis=\"0 0 1\"/><joint name=\"front_wheel_spin\" type=\"hinge\" axis=\"0 1 0\"/><inertial mass=\"2.4\" pos=\"0 0 0\" diaginertia=\".15 .30 .15\"/><geom name=\"geom_front_contact\" type=\"cylinder\" size=\".37 .025\" euler=\"1.5707963267948966 0 0\" contype=\"0\" conaffinity=\"0\" mass=\"0\"/></body><body name=\"rear_wheel\" pos=\"0 0 2\"><geom name=\"geom_rear_contact\" type=\"sphere\" size=\".37\" contype=\"0\" conaffinity=\"0\"/></body></worldbody></mujoco>");
    for(std::size_t row=0;row<2;++row)for(std::size_t i=0;i<vertices.size();++i)s->model->hfield_data[row*vertices.size()+i]=static_cast<float>((vertices[i][1]-low)/scale);
    engine::forward(s->model.get(),s->data.get());auto tire=std::make_unique<DistributedTireWriter>(s->model.get(),s->data.get(),distributed_config(n));return {std::move(s),std::move(tire)};
}
double kinetic(const mjModel* m,const mjData* d){std::vector<double> momentum(static_cast<std::size_t>(m->nv));mj_mulM(m,d,momentum.data(),d->qvel);return .5*vector_dot({d->qvel,momentum.size()},momentum);}
std::pair<Value,Value> run_stand(WheelStand& s,double load,double duration){
    auto* m=s.stand->model.get();auto* d=s.stand->data.get();auto& tire=*s.tire;const double dt=m->opt.timestep;const auto count=steps(duration,dt);
    const double initial=kinetic(m,d)+tire.stored_energy(d)+load*d->qpos[1],scale=std::max(1.,kinetic(m,d)+tire.stored_energy(d));
    double loss=0.,maximum=0.,residual=0.;mjtSize native=0;Array rows;rows.reserve(count);
    for(std::size_t i=0;i<count;++i){const auto force=tire.qfrc(d,dt);const auto snapshot=tire.snapshots()[0];const auto diagnostic=tire.diagnostics()[0];
        if(!snapshot||!diagnostic)throw std::runtime_error("distributed stand published no contact snapshot");
        rows.push_back(Value{Object{{"time_s",value::real(static_cast<double>(i)*dt)},{"end_time_s",value::real(static_cast<double>(i+1)*dt)},
            {"normal_load_n",value::real(normal_load(*snapshot))},{"vertical_force_n",value::real(vertical_force(*snapshot))},
            {"position_m",value::real(d->qpos[0])},{"pitch_rate_rad_s",value::real(d->qvel[2])},{"patch_count",value::integer(static_cast<std::int64_t>(snapshot->patches.size()))},{"multi_support",Value{diagnostic->multi_support}}}});
        loss+=tire.brush_loss_step_j()+tire.radial_dissipation_power_w()*dt;std::ranges::copy(force,d->qfrc_applied);d->qfrc_applied[1]-=load;
        engine::step(m,d);engine::forward(m,d);native=std::max(native,d->ncon);
        const double current=kinetic(m,d)+tire.stored_energy(d)+load*d->qpos[1];residual=current-initial+loss;maximum=std::max(maximum,std::abs(residual));
        for(mjtSize q=0;q<m->nq;++q)scalar(d->qpos[q],"stand coordinates");for(mjtSize q=0;q<m->nv;++q)scalar(d->qvel[q],"stand velocities");}
    return {Value{std::move(rows)},metrics({{"energy_residual_j",residual},{"energy_residual_ratio",std::abs(residual)/(scale+loss)},
        {"max_energy_residual_j",maximum},{"dissipation_j",loss},{"native_wheel_contact_count",static_cast<double>(native)},{"final_speed_mps",d->qvel[0]}})};
}
double flat_depth(double load,int n,double tolerance=2e-12){const configuration::HingeDensity density;
    return numerics::brentq([&](double x){const std::array depths{x};return numerics::flat_load(depths,.37,density,n).force[0]-load;},1e-9,.1,tolerance).root;
}
}
RigResult distributed_flat_rig(double dt,int n,double load){
    positive(load,"axle load");std::vector<std::array<double,2>> vertices;for(double x:geometry::detail::np_linspace(-2.,2.,801))vertices.push_back({x,0.});
    auto s=wheel_stand(dt,n,vertices);const double depth=flat_depth(load,n,1e-14);s.stand->data->qpos[1]=-depth;engine::forward(s.stand->model.get(),s.stand->data.get());
    auto [rows,m]=run_stand(s,load,.05);std::vector<double> loads;for(const auto& row:io::as_array(rows,"rows"))loads.push_back(real(row,"normal_load_n"));
    auto& o=std::get<Object>(m.data);value::set(o,"normal_force_n",value::real(mean(loads)));value::set(o,"deflection_m",value::real(depth));value::set(o,"force_relative_error",value::real(std::abs(loads.back()-load)/load));value::set(o,"station_count",value::integer(n));
    return {std::move(m),bounds({{"force_relative_error",0.,.02},{"native_wheel_contact_count",0.,0.},{"energy_residual_ratio",0.,.02}})};
}
std::vector<std::array<double,2>> contact_case_vertices(std::string_view name,double dx){
    positive(dx,"road mesh spacing");const double intervals=std::nearbyint(4./dx);if(intervals<1.||intervals>10000000.)throw std::invalid_argument("stand mesh extent outside supported range");
    std::vector<std::array<double,2>> vertices;for(double x:geometry::detail::np_linspace(-2.,2.,static_cast<std::size_t>(intervals)+1)){
        double y=0.;if(name=="step")y=x>=0.?.04:0.;else if(name=="two_support")y=.03*std::exp(-std::pow((x+.13)/.025,2.))+.03*std::exp(-std::pow((x-.13)/.025,2.));
        else if(name=="edge_unload")y=x<0.?.04:0.;else if(name=="recontact")y=x>=0.&&x<.2?-.05:0.;else if(name=="bump")y=.02*std::exp(-std::pow(x/.08,2.));else if(name=="incline")y=.1*x;else throw std::invalid_argument("unknown distributed contact stand");vertices.push_back({x,y});}
    return vertices;
}
std::pair<Value,Value> distributed_road_trace(double dt,int n,double dx,std::string_view name,double duration){
    const auto vertices=contact_case_vertices(name,dx);auto s=wheel_stand(dt,n,vertices);const double depth=flat_depth(600.,n);auto* d=s.stand->data.get();
    const auto high=std::ranges::upper_bound(vertices,-.45,{},[](const auto& point){return point[0];});const auto j=static_cast<std::size_t>(high-vertices.begin()-1);
    const double z=numeric::np_interp_segment((vertices[j+1][1]-vertices[j][1])/(vertices[j+1][0]-vertices[j][0]),-.45,vertices[j][0],vertices[j][1],vertices[j+1][0],vertices[j+1][1]);
    d->qpos[0]=-.45;d->qpos[1]=z-depth;d->qvel[0]=2.;d->qvel[2]=2./.37;engine::forward(s.stand->model.get(),d);
    auto [rows,m]=run_stand(s,600.,duration);numeric::PythonSum normal,vertical;std::int64_t multi=0,max_patch=0;
    const auto& data=io::as_array(rows,"rows");for(const auto& row:data){normal.add(real(row,"normal_load_n")*dt);vertical.add(real(row,"vertical_force_n")*dt);max_patch=std::max(max_patch,io::as_int64(get(row,"patch_count"),"patch count"));if(flag(row,"multi_support"))++multi;}
    auto& o=std::get<Object>(m.data);value::set(o,"normal_impulse_ns",value::real(normal.total()));value::set(o,"vertical_impulse_ns",value::real(vertical.total()));value::set(o,"max_patch_count",value::integer(max_patch));value::set(o,"multi_support_fraction",value::real(static_cast<double>(multi)/static_cast<double>(data.size())));value::set(o,"station_count",value::integer(n));value::set(o,"terrain_dx_m",value::real(dx));return {std::move(rows),std::move(m)};
}
RigResult distributed_step_rig(double dt,int n,double dx){auto result=distributed_road_trace(dt,n,dx);return {std::move(result.second),bounds({{"native_wheel_contact_count",0.,0.},{"energy_residual_ratio",0.,.05},{"max_patch_count",2.,std::numeric_limits<double>::infinity()}})};}
RigResult distributed_incline_rig(double dt,int n,double degrees){
    if(!std::isfinite(degrees)||std::abs(degrees)>35.)throw std::invalid_argument("incline fixture angle range is +/-35 degrees");const auto count=steps(.05,dt);
    const double angle=degrees*(std::numbers::pi/180.),sn=std::sin(angle),cs=std::cos(angle),weight=20.*9.81;
    const auto xs=geometry::detail::np_linspace(-2.,2.,801);std::vector<double> y;for(double x:xs)y.push_back(x*std::tan(angle));const double low=*std::ranges::min_element(y),scale=std::max(.01,*std::ranges::max_element(y)-low);
    Stand s("<mujoco>"+option(dt,"0 0 -9.81")+"<asset><hfield name=\"road\" nrow=\"2\" ncol=\"801\" size=\"2 1 "+xml_number(scale)+" .2\"/></asset><worldbody><geom name=\"terrain\" type=\"hfield\" hfield=\"road\" pos=\"0 0 "+xml_number(low)+"\" contype=\"0\" conaffinity=\"0\"/><body name=\"front_wheel\" pos=\""+xml_number(-sn*.37)+" 0 "+xml_number(cs*.37)+"\"><joint name=\"normal_slide\" type=\"slide\" axis=\""+xml_number(-sn)+" 0 "+xml_number(cs)+"\"/><inertial mass=\"20\" pos=\"0 0 0\" diaginertia=\"1 1 1\"/><geom name=\"geom_front_contact\" type=\"sphere\" size=\".37\" mass=\"0\" contype=\"0\" conaffinity=\"0\"/></body><body name=\"rear_wheel\" pos=\"0 0 5\"><geom name=\"geom_rear_contact\" type=\"sphere\" size=\".37\" contype=\"0\" conaffinity=\"0\"/></body></worldbody></mujoco>");
    auto* m=s.model.get();auto* d=s.data.get();for(std::size_t row=0;row<2;++row)for(std::size_t i=0;i<y.size();++i)m->hfield_data[row*y.size()+i]=static_cast<float>((y[i]-low)/scale);engine::forward(m,d);
    DistributedTireWriter tire(m,d,distributed_config(n));const auto balance=[&](double depth){d->qpos[0]=-depth;engine::forward(m,d);return tire.probe_qfrc(d,dt)[0]-weight*cs;};
    const double depth=numerics::brentq(balance,1e-7,.05,1e-13).root;(void)balance(depth);
    for(std::size_t i=0;i<count;++i){const auto force=tire.qfrc(d,dt);std::ranges::copy(force,d->qfrc_applied);engine::step(m,d);engine::forward(m,d);}
    const auto snap=tire.snapshots()[0];if(!snap)throw std::runtime_error("incline stand has no snapshot");const double fn=normal_load(*snap),tangent=weight*sn,vertical=fn*cs+tangent*sn;
    return {metrics({{"normal_load_n",fn},{"expected_normal_n",weight*cs},{"normal_relative_error",std::abs(fn-weight*cs)/weight},{"total_vertical_support_n",vertical},
        {"vertical_relative_error",std::abs(vertical-weight)/weight},{"tangent_fixture_reaction_n",tangent},{"speed_mps",d->qvel[0]},{"deflection_m",depth},{"native_contact_count",static_cast<double>(d->ncon)},{"angle_deg",degrees}}),
        bounds({{"normal_relative_error",0.,.01},{"vertical_relative_error",0.,.01},{"speed_mps",-1e-5,1e-5},{"native_contact_count",0.,0.}})};
}
Value contact_matrix(const Value& config){
    const auto dts=numbers(optional(config,"dts",Value{Array{value::real(.00125),value::real(.000625),value::real(.0003125)}}),"dts");
    const auto counts=numbers(optional(config,"station_counts",Value{Array{value::integer(128),value::integer(256),value::integer(512)}}),"station counts");
    const auto loads=numbers(optional(config,"flat_loads",Value{Array{value::real(300.),value::real(600.),value::real(900.)}}),"flat loads");
    const auto dxs=numbers(optional(config,"step_dxs",Value{Array{value::real(.02),value::real(.01),value::real(.005)}}),"road steps");Array rows;
    const auto add=[&](std::string_view name,double dt,int count,std::string key,double x,RigResult result){
        auto checks=check_bounds(result.metrics,result.bounds);
        const bool passed=std::ranges::all_of(io::as_object(checks,"checks"),[](const auto& e){return true_value(e.second);});
        rows.push_back(Value{Object{
            {"case",value::text(name)}, {"dt_s",value::real(dt)},
            {"station_count",value::integer(count)}, {std::move(key),value::real(x)},
            {"metrics",std::move(result.metrics)}, {"checks",std::move(checks)},
            {"accepted",Value{passed}}
        }});
    };
    for(double dt:dts)for(double count:counts){if(count<16.||count>512.||std::trunc(count)!=count)throw std::invalid_argument("station counts must be integers in 16..512");const int n=static_cast<int>(count);
        for(double load:loads)add("flat",dt,n,"load_n",load,distributed_flat_rig(dt,n,load));for(double dx:dxs)add("step",dt,n,"terrain_dx_m",dx,distributed_step_rig(dt,n,dx));const double angle=real(config,"incline_angle_deg",15.);add("incline",dt,n,"angle_deg",angle,distributed_incline_rig(dt,n,angle));}
    return Value{std::move(rows)};
}
} // namespace bike::validation
