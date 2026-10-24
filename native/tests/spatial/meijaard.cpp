// W34. Independent ideal-constraint native model versus Appendix A. V06 only.
#include "support/migration_test.hpp"
#include "spatial/support/startup_fixture.hpp"
#include "spatial/support/spatial_fixture.hpp"
#include "spatial/support/meijaard_reference.hpp"
#include "spatial/support/meijaard_constraints.hpp"
#include "spatial/support/benchmark_linalg.hpp"
#include "model/benchmark_whipple.hpp"
#include "spatial/frames.hpp"
#include "numerics/dense.hpp"
#include "engine_call.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <iostream>
#include <limits>
#include <numbers>
#include <span>
#include <string>
#include <vector>

namespace {
namespace st=spatial_tests;namespace sp=bike::spatial;namespace dense=bike::numerics::dense;
using Parameters=bike::model::WhippleParameters;
using State=std::array<double,4>;
using Matrix=std::array<double,16>;
using Complex=std::complex<double>;

const bike::io::Value& member(const bike::io::Value& v,std::string_view key) {
    const auto* found=bike::io::find_member(bike::io::as_object(v,"meijaard"),key);
    if(!found)throw std::invalid_argument("meijaard."+std::string(key)+": missing field");return *found;
}
void reference_contract() {
    const Parameters p;
    const auto fixture=st::json(st::fixture("references/meijaard2007.json"));const auto& fields=member(fixture,"parameters");
    const std::array<std::pair<std::string_view,double>,27> parameters{{
        {"w_m",p.w_m},{"c_m",p.c_m},{"lambda_rad",p.lambda_rad},{"g_mps2",p.g_mps2},{"v_mps",p.v_mps},
        {"rR_m",p.rR_m},{"mR_kg",p.mR_kg},{"IRxx",p.IRxx},{"IRyy",p.IRyy},{"xB_m",p.xB_m},{"zB_m",p.zB_m},
        {"mB_kg",p.mB_kg},{"IBxx",p.IBxx},{"IBxz",p.IBxz},{"IByy",p.IByy},{"IBzz",p.IBzz},
        {"xH_m",p.xH_m},{"zH_m",p.zH_m},{"mH_kg",p.mH_kg},{"IHxx",p.IHxx},{"IHxz",p.IHxz},
        {"IHyy",p.IHyy},{"IHzz",p.IHzz},{"rF_m",p.rF_m},{"mF_kg",p.mF_kg},{"IFxx",p.IFxx},{"IFyy",p.IFyy}}};
    for(const auto& [key,value]:parameters)
        bt::require(std::abs(value-bike::io::as_double(member(member(fields,key),"value"),key))<=1e-12,
            "native benchmark parameter retains independent Table 1 datum");
    const auto matrices=st::reference_matrices(p);
    const std::array expected_m{80.81722,2.31941332208709,2.31941332208709,.29784188199686};
    const std::array expected_c{0.,33.86641391492494,-.85035641456978,1.68540397397560};
    const std::array expected_k0{-80.95,-2.59951685249872,-2.59951685249872,-.80329488458618};
    const std::array expected_k2{0.,76.59734589573222,0.,2.65431523794604};
    for(std::size_t i=0;i<4;++i) {
        bt::require(std::abs(matrices.M[i]-expected_m[i])<=1e-12,"Appendix A mass reproduces equation 6.1");
        bt::require(std::abs(matrices.C1[i]-expected_c[i])<=1e-12,"Appendix A gyro reproduces equation 6.4");
        bt::require(std::abs(matrices.K0[i]-expected_k0[i])<=1e-12,"Appendix A gravity reproduces equation 6.2");
        bt::require(std::abs(matrices.K2[i]-expected_k2[i])<=1e-12,"Appendix A velocity stiffness reproduces equation 6.3");
    }
    const auto has=[](const auto& roots,Complex expected) {
        return std::ranges::any_of(roots,[&](Complex root){return std::abs(root-expected)<=1e-9;});
    };
    const auto r2=st::quadratic_eigenvalues(matrices,2.,p.g_mps2),r6=st::quadratic_eigenvalues(matrices,6.,p.g_mps2);
    bt::require(has(r2,{2.68234517512745,1.68066296590675})&&has(r2,{2.68234517512745,-1.68066296590675}),
        "reference polynomial reproduces Table 2 weave roots at 2 m/s");
    bt::require(has(r6,{-1.52644486584142,5.87673060598709})&&has(r6,{-1.52644486584142,-5.87673060598709})&&
        has(r6,{-.00406690076970,0.}),"reference polynomial reproduces Table 2 roots at 6 m/s");
}
st::RigModel native_model(const Parameters& parameters) {
    const auto request=bike::model::make_whipple_request(parameters);st::RigModel rig;
    rig.model=bike::model::compile_mjcf(bike::model::generate_mjcf(request)).release();rig.data=engine::make_data(rig.model);
    bt::require(rig.data!=nullptr,"benchmark data allocation succeeds");
    rig.topology=sp::bind_topology(*rig.model,*request.spatial);engine::forward(rig.model,rig.data);
    bt::require(rig.model->nbody==5&&rig.model->njnt==4&&rig.model->nv==9,
        "benchmark has four rigid bodies, a free root, steering and two spins");
    for(int i=0;i<rig.model->ngeom;++i)bt::require(rig.model->geom_contype[i]==0&&rig.model->geom_conaffinity[i]==0,
        "ideal rolling has no hidden engine contact or huge tyre stiffness");
    for(int i=0;i<rig.model->nv;++i)bt::require(rig.model->dof_damping[i]==0.&&rig.model->dof_frictionloss[i]==0.&&rig.model->dof_armature[i]==0.,
        "benchmark has no artificial damping or rotor inertia");
    return rig;
}
std::array<double,2> contact_gaps(const st::RigModel& rig) {
    std::array<double,2> gaps{};
    for(std::size_t w=0;w<2;++w) {
        const auto& wheel=rig.topology.wheels[w];const auto* matrix=rig.data->xmat+9*wheel.hub_body_id;
        sp::Vec3 axle{};for(std::size_t i=0;i<3;++i)for(std::size_t j=0;j<3;++j)axle[i]+=matrix[3*i+j]*wheel.axle_axis_hub[j];
        const double nz=std::sqrt(std::max(0.,1.-axle[2]*axle[2]));
        const int geom=mj_name2id(rig.model,mjOBJ_GEOM,w==0?"geom_front_contact":"geom_rear_contact");
        gaps[w]=rig.data->xpos[3*wheel.wheel_body_id+2]-rig.model->geom_size[3*geom]*nz;
    }
    return gaps;
}
void project_positions(st::RigModel& rig) {
    const auto n=static_cast<std::size_t>(rig.model->nv),nq=static_cast<std::size_t>(rig.model->nq);
    const int root=rig.topology.root.free_dof_adr;std::vector<double> delta(n,0.);
    constexpr double h=1e-7;
    for(int iteration=0;iteration<8;++iteration) {
        engine::forward(rig.model,rig.data);const auto gap=contact_gaps(rig);
        if(std::max(std::abs(gap[0]),std::abs(gap[1]))<=1e-12)return;
        const std::vector<double> base(rig.data->qpos,rig.data->qpos+nq);std::array<double,4> jac{};
        for(int j=0;j<2;++j) {
            std::ranges::fill(delta,0.);delta[static_cast<std::size_t>(root+(j==0?2:4))]=1.;
            std::ranges::copy(base,rig.data->qpos);engine::integrate_pos(rig.model,rig.data->qpos,delta.data(),h);engine::forward(rig.model,rig.data);
            const auto plus=contact_gaps(rig);
            std::ranges::copy(base,rig.data->qpos);engine::integrate_pos(rig.model,rig.data->qpos,delta.data(),-h);engine::forward(rig.model,rig.data);
            const auto minus=contact_gaps(rig);
            for(std::size_t i=0;i<2;++i)jac[2*i+static_cast<std::size_t>(j)]=(plus[i]-minus[i])/(2.*h);
        }
        const double determinant=jac[0]*jac[3]-jac[1]*jac[2];
        bt::require(std::abs(determinant)>1e-10,"native holonomic contact chart has full rank");
        std::ranges::fill(delta,0.);delta[static_cast<std::size_t>(root+2)]=(-gap[0]*jac[3]+jac[1]*gap[1])/determinant;
        delta[static_cast<std::size_t>(root+4)]=(-jac[0]*gap[1]+gap[0]*jac[2])/determinant;
        std::ranges::copy(base,rig.data->qpos);engine::integrate_pos(rig.model,rig.data->qpos,delta.data(),1.);
    }
    bt::require(false,"native contact-position projection did not converge");
}
void project_velocity(st::RigModel& rig,double roll_rate,double steer_rate,double rear_spin) {
    const auto n=static_cast<std::size_t>(rig.model->nv);const auto constraints=st::knife_edge_constraints(*rig.model,*rig.data,rig.topology);
    auto square=dense::zeros(n,n);std::vector<double> rhs(n,0.);
    std::ranges::copy(constraints.A,square.values.begin());
    const std::array dofs{rig.topology.root.free_dof_adr+3,rig.topology.steer.joint.dof_adr,rig.topology.wheels[1].spin.dof_adr};
    const std::array values{roll_rate,steer_rate,rear_spin};
    for(std::size_t i=0;i<3;++i){square.values[(6+i)*n+static_cast<std::size_t>(dofs[i])]=1.;rhs[6+i]=values[i];}
    const auto velocity=dense::solve(square,rhs);std::ranges::copy(velocity,rig.data->qvel);engine::forward(rig.model,rig.data);
    for(std::size_t i=0;i<6;++i) {
        double residual=0.;for(std::size_t j=0;j<n;++j)residual+=constraints.A[i*n+j]*velocity[j];
        bt::require(std::abs(residual)<=1e-10,"admissible velocities obey all six knife-edge constraints");
    }
}
void initialize(st::RigModel& rig,const Parameters& p,double speed,const State& x) {
    std::ranges::copy(std::span(rig.model->qpos0,static_cast<std::size_t>(rig.model->nq)),rig.data->qpos);
    std::fill_n(rig.data->qvel,rig.model->nv,0.);rig.data->time=0.;
    std::vector<double> delta(static_cast<std::size_t>(rig.model->nv),0.);
    delta[static_cast<std::size_t>(rig.topology.root.free_dof_adr+3)]=x[0];
    delta[static_cast<std::size_t>(rig.topology.steer.joint.dof_adr)]=x[1];
    engine::integrate_pos(rig.model,rig.data->qpos,delta.data(),1.);project_positions(rig);
    project_velocity(rig,x[2],x[3],speed/p.rR_m);
}
st::Kkt acceleration(st::RigModel& rig) {
    engine::forward(rig.model,rig.data);const auto n=static_cast<std::size_t>(rig.model->nv);std::vector<double> mass(n*n);
    struct Call {const mjModel* model;const mjData* data;double* matrix;} call{rig.model,rig.data,mass.data()};engine::ErrorBuffer error;
    if(!engine::invoke([](void* raw) noexcept {const auto& c=*static_cast<Call*>(raw);mj_fullM(c.model,c.data,c.matrix);},&call,error))
        engine::throw_failure(error);
    auto result=st::solve_kkt(mass,std::span<const double>(rig.data->qfrc_bias,n),
        st::knife_edge_constraints(*rig.model,*rig.data,rig.topology),static_cast<int>(n));
    bt::require(result.relative_residual<=1e-10&&result.rank==static_cast<int>(n)+6,
        "native KKT enforces ideal rolling with full rank and residual <= 1e-10");return result;
}
State vector_field(st::RigModel& rig,const Parameters& p,double speed,const State& x) {
    initialize(rig,p,speed,x);const auto a=acceleration(rig);
    return {x[2],x[3],a.vdot[static_cast<std::size_t>(rig.topology.root.free_dof_adr+3)],
        a.vdot[static_cast<std::size_t>(rig.topology.steer.joint.dof_adr)]};
}
Matrix linearize(st::RigModel& rig,const Parameters& p,double speed,double h) {
    Matrix a{};
    for(std::size_t j=0;j<4;++j) {
        State plus{},minus{};plus[j]=h;minus[j]=-h;
        const auto fp=vector_field(rig,p,speed,plus),fm=vector_field(rig,p,speed,minus);
        for(std::size_t i=0;i<4;++i)a[4*i+j]=(fp[i]-fm[i])/(2.*h);
    }
    return a;
}
Matrix richardson(const Matrix& coarse,const Matrix& fine) {Matrix a{};for(std::size_t i=0;i<16;++i)a[i]=(4.*fine[i]-coarse[i])/3.;return a;}
std::vector<Complex> eig(const Matrix& a) {
    // The native reduced matrix is a second-order system with identity mass;
    // this conversion uses only its measured tangent Jacobian, never Appendix A.
    st::WhippleMatrices q{{1.,0.,0.,1.},{-a[10],-a[11],-a[14],-a[15]},
        {-a[8],-a[9],-a[12],-a[13]},{0.,0.,0.,0.}};
    return st::quadratic_eigenvalues(q,1.,1.);
}
std::vector<Complex> match(const std::vector<Complex>& actual,const std::vector<Complex>& target) {
    std::array<std::size_t,4> order{0,1,2,3},best=order;double best_error=std::numeric_limits<double>::infinity();
    do {double error=0.;for(std::size_t i=0;i<4;++i)error+=std::norm(actual[order[i]]-target[i]);
        if(error<best_error){best_error=error;best=order;}
    } while(std::next_permutation(order.begin(),order.end()));
    std::vector<Complex> result;for(auto i:best)result.push_back(actual[i]);return result;
}
std::vector<Complex> native_roots(st::RigModel& rig,const Parameters& p,double speed) {
    const auto a=linearize(rig,p,speed,1e-5),b=linearize(rig,p,speed,5e-6),c=linearize(rig,p,speed,2.5e-6);
    const auto coarse=eig(richardson(a,b)),fine=match(eig(richardson(b,c)),coarse);
    for(std::size_t i=0;i<4;++i)if(std::abs(fine[i])<.05)
        bt::require(std::abs(coarse[i]-fine[i])<=1e-4,"near-zero root is stable under tangent-step refinement");
    return fine;
}
double mode_real(const std::vector<Complex>& roots,bool weave) {
    double answer=-std::numeric_limits<double>::infinity();
    for(auto z:roots)if(weave?std::abs(z.imag())>1e-4:std::abs(z.imag())<=1e-4)answer=std::max(answer,z.real());
    bt::require(std::isfinite(answer),"requested stability branch exists in the bracket");return answer;
}
double boundary(st::RigModel& rig,const Parameters& p,double low,double high,bool weave) {
    double fl=mode_real(native_roots(rig,p,low),weave),fh=mode_real(native_roots(rig,p,high),weave);
    bt::require(fl*fh<0.,"native stability boundary is bracketed by opposite signs");
    while(high-low>.01) {
        const double mid=(low+high)/2.,fm=mode_real(native_roots(rig,p,mid),weave);
        if((fm>0.)==(fl>0.)){low=mid;fl=fm;}else{high=mid;fh=fm;}
    }
    (void)fh;return (low+high)/2.;
}
State measured_state(const st::RigModel& rig) {
    const auto q=static_cast<std::size_t>(rig.topology.root.free_qpos_adr);
    const sp::Quat quat{rig.data->qpos[q+3],rig.data->qpos[q+4],rig.data->qpos[q+5],rig.data->qpos[q+6]};
    const auto angles=sp::euler_zyx(quat);bt::require(angles.valid,"small rollout stays inside Euler chart");
    return {angles.roll_rad,rig.data->qpos[rig.topology.steer.joint.qpos_adr],
        rig.data->qvel[rig.topology.root.free_dof_adr+3],rig.data->qvel[rig.topology.steer.joint.dof_adr]};
}
void rollout(st::RigModel& rig,const Parameters& p,double speed) {
    const auto a=richardson(linearize(rig,p,speed,1e-5),linearize(rig,p,speed,5e-6));
    const auto roots=eig(a);Complex lambda{};bool found=false;
    for(auto z:roots)if(z.imag()>1e-4){lambda=z;found=true;break;}
    bt::require(found,"weave eigenmode exists for nonlinear rollout");
    const Complex delta=(lambda*lambda-a[8]-a[10]*lambda)/(a[9]+a[11]*lambda);
    const std::array<Complex,4> mode{Complex{1.,0.},delta,lambda,lambda*delta};
    auto basis=dense::zeros(4,2);State initial{};
    for(std::size_t i=0;i<4;++i){basis.values[2*i]=mode[i].real();basis.values[2*i+1]=mode[i].imag();initial[i]=1e-5*mode[i].real();}
    initialize(rig,p,speed,initial);
    const auto amplitude=[&] {
        const auto x=measured_state(rig);dense::Matrix rhs{.rows=4,.cols=1,.values=std::vector<double>(x.begin(),x.end())};
        const auto fit=dense::least_squares(basis,rhs);bt::require(fit.rank==2,"real and imaginary weave basis are independent");
        return std::hypot(fit.solution.values[0],fit.solution.values[1]);
    };
    const double start=amplitude();constexpr double dt=1e-4;const auto nv=static_cast<std::size_t>(rig.model->nv);
    for(int step=0;step<5000;++step) {
        const auto kkt=acceleration(rig);std::vector<double> midpoint(nv);
        for(std::size_t i=0;i<nv;++i){midpoint[i]=rig.data->qvel[i]+.5*dt*kkt.vdot[i];rig.data->qvel[i]+=dt*kkt.vdot[i];}
        engine::integrate_pos(rig.model,rig.data->qpos,midpoint.data(),dt);rig.data->time+=dt;project_positions(rig);
        project_velocity(rig,rig.data->qvel[rig.topology.root.free_dof_adr+3],
            rig.data->qvel[rig.topology.steer.joint.dof_adr],rig.data->qvel[rig.topology.wheels[1].spin.dof_adr]);
    }
    const double growth=std::log(amplitude()/start)/.5;
    bt::require(speed==2.?growth>0.:growth<0.,"half-second nonlinear native rollout confirms weave growth/decay sign");
}
}
int main(){return bt::run([]{
    reference_contract();const Parameters p;auto rig=native_model(p);const auto reference=st::reference_matrices(p);
    for(double speed:std::array{2.,4.,6.,8.,10.}) {
        const auto steady=vector_field(rig,p,speed,State{});
        for(double component:steady)bt::require(std::abs(component)<=1e-10,
            "upright native motion is an exact reduced steady state");
        const auto expected=st::quadratic_eigenvalues(reference,speed,p.g_mps2);
        const auto measured=match(native_roots(rig,p,speed),expected);
        for(std::size_t i=0;i<4;++i) {
            const double error=std::abs(measured[i]-expected[i]);
            bt::require(error<=std::max(.03*std::abs(expected[i]),.05),"native eigenvalues match Meijaard within 3 percent or .05/s");
            if(std::abs(expected[i])<.05)bt::require(error<=5e-4,"near-zero capsize root meets absolute tolerance");
        }
    }
    const double weave=boundary(rig,p,4.,4.6,true),capsize=boundary(rig,p,5.5,6.5,false);
    bt::require(std::abs(weave-4.29238253634111)<=.2,"native weave boundary agrees within .2 m/s");
    bt::require(std::abs(capsize-6.02426201538837)<=.2,"native capsize boundary agrees within .2 m/s");
    rollout(rig,p,2.);rollout(rig,p,6.);
    std::cout<<"{\"benchmark\":\"meijaard2007\",\"weave_boundary_mps\":"<<weave<<",\"capsize_boundary_mps\":"<<capsize<<"}\n";
});}
