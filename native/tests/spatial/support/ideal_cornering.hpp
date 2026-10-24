#pragma once

// Independent rigid, thin-contact reference. This solves a native relative
// equilibrium with ideal rolling and a workless steering-position constraint;
// no pneumatic kernel or calibrated lateral coefficient enters the solve.
#include "ride_qualification.hpp"
#include "benchmark_linalg.hpp"
#include "meijaard_constraints.hpp"
#include "model/benchmark_whipple.hpp"
#include "numerics/dense.hpp"

namespace spatial_tests::ride {
class IdealCorner {
    RigModel rig_;
    bike::model::WhippleParameters parameters_;
    static constexpr double h=1e-7;
    std::array<Vec3,2> contacts() const {
        std::array<Vec3,2> out{};
        const auto quaternion=model_access::readonly_buffer(rig_.data->xquat,4*rig_.model->nbody);
        const auto position=model_access::readonly_buffer(rig_.data->xpos,3*rig_.model->nbody);
        for(std::size_t w=0;w<2;++w) {
            const auto& b=rig_.topology.wheels[w];
            const auto base=4*static_cast<std::size_t>(b.hub_body_id);
            const bike::spatial::Quat q{quaternion[base],quaternion[base+1],quaternion[base+2],quaternion[base+3]};
            const auto axle=bike::spatial::rotate_body_to_world(q,b.axle_axis_hub);
            const auto up=bike::pneumatic3d::detail::unit(sub(Vec3{0.,0.,1.},scale(axle,axle[2])));
            const auto body_base=3*static_cast<std::size_t>(b.wheel_body_id);
            for(std::size_t k=0;k<3;++k)out[w][k]=position[body_base+k]-(w==0?parameters_.rF_m:parameters_.rR_m)*up[k];
        }
        return out;
    }
    void contact_positions() {
        auto* m=rig_.model;auto* d=rig_.data;
        const auto n=static_cast<std::size_t>(m->nv);
        auto qpos=model_access::mutable_buffer(d->qpos,m->nq);
        std::vector<double> delta(n,0.);const auto root=rig_.topology.root.free_dof_adr;
        for(unsigned iteration=0;iteration<12;++iteration) {
            engine::forward(m,d);const auto points=contacts();
            const std::array gaps{points[0][2],points[1][2]};
            if(std::max(std::abs(gaps[0]),std::abs(gaps[1]))<=1e-12)return;
            const std::vector<double> q(qpos.begin(),qpos.end());std::array<std::array<double,2>,2> jac{};
            for(unsigned j=0;j<2;++j) {
                std::ranges::fill(delta,0.);delta[static_cast<std::size_t>(root+(j==0?2:4))]=1.;
                std::ranges::copy(q,qpos.begin());engine::integrate_pos(m,qpos.data(),delta.data(),h);engine::forward(m,d);const auto plus=contacts();
                std::ranges::copy(q,qpos.begin());engine::integrate_pos(m,qpos.data(),delta.data(),-h);engine::forward(m,d);const auto minus=contacts();
                for(unsigned i=0;i<2;++i)jac[i][j]=(plus[i][2]-minus[i][2])/(2.*h);
            }
            const double determinant=jac[0][0]*jac[1][1]-jac[0][1]*jac[1][0];
            bt::require(std::abs(determinant)>1e-10,"ideal corner contact chart has rank two");
            std::ranges::fill(delta,0.);
            delta[static_cast<std::size_t>(root+2)]=(-gaps[0]*jac[1][1]+jac[0][1]*gaps[1])/determinant;
            delta[static_cast<std::size_t>(root+4)]=(-jac[0][0]*gaps[1]+gaps[0]*jac[1][0])/determinant;
            std::ranges::copy(q,qpos.begin());engine::integrate_pos(m,qpos.data(),delta.data(),1.);
        }
        throw std::runtime_error("inconclusive: ideal corner contact-position projection did not converge");
    }
    void velocity(double rear_spin) {
        namespace dense=bike::numerics::dense;
        auto* m=rig_.model;auto* d=rig_.data;const auto n=static_cast<std::size_t>(m->nv);
        const auto c=knife_edge_constraints(*m,*d,rig_.topology);
        auto matrix=dense::zeros(n,n);std::ranges::copy(c.A,matrix.values.begin());std::vector<double> rhs(n,0.);
        const auto root=rig_.topology.root.free_dof_adr;
        const auto e=bike::spatial::euler_zyx(bike::spatial::body_pose(*m,*d,rig_.topology.root).quat_world_from_body_wxyz);
        matrix.values[6*n+static_cast<std::size_t>(root+3)]=1.;
        matrix.values[6*n+static_cast<std::size_t>(root+4)]=std::sin(e.roll_rad)*std::tan(e.pitch_rad);
        matrix.values[6*n+static_cast<std::size_t>(root+5)]=std::cos(e.roll_rad)*std::tan(e.pitch_rad);
        matrix.values[7*n+static_cast<std::size_t>(rig_.topology.steer.joint.dof_adr)]=1.;
        matrix.values[8*n+static_cast<std::size_t>(rig_.topology.wheels[1].spin.dof_adr)]=1.;rhs[8]=rear_spin;
        const auto answer=dense::solve(matrix,rhs);auto qvel=model_access::mutable_buffer(d->qvel,m->nv);
        std::ranges::copy(answer,qvel.begin());engine::forward(m,d);engine::subtree_vel(m,d);
    }
    Kkt acceleration() {
        auto* m=rig_.model;auto* d=rig_.data;const auto n=static_cast<std::size_t>(m->nv);
        engine::forward(m,d);std::vector<double> mass(n*n);
        struct Call {const mjModel* m;const mjData* d;double* mass;} call{m,d,mass.data()};engine::ErrorBuffer error;
        if(!engine::invoke([](void* raw) noexcept {const auto& c=*static_cast<Call*>(raw);mj_fullM(c.m,c.d,c.mass);},&call,error))engine::throw_failure(error);
        auto constraints=knife_edge_constraints(*m,*d,rig_.topology);
        constraints.A.resize(7*n,0.);constraints.A[6*n+static_cast<std::size_t>(rig_.topology.steer.joint.dof_adr)]=1.;
        constraints.Adot_v.push_back(0.);constraints.rows=7;
        const auto bias=model_access::readonly_buffer(d->qfrc_bias,m->nv);
        auto result=solve_kkt(mass,bias,constraints,static_cast<int>(n));
        bt::require(result.relative_residual<=1e-10 && result.rank==static_cast<int>(n)+7,"ideal corner native KKT residual and full rank");
        return result;
    }
    std::array<double,2> residual(double lean,double steer,double speed,double radius) {
        auto* m=rig_.model;auto* d=rig_.data;
        const auto qpos0=model_access::readonly_buffer(m->qpos0,m->nq);
        auto qpos=model_access::mutable_buffer(d->qpos,m->nq);
        auto qvel=model_access::mutable_buffer(d->qvel,m->nv);
        const auto com_velocity=model_access::readonly_buffer(d->subtree_linvel,3*m->nbody);
        std::ranges::copy(qpos0,qpos.begin());std::ranges::fill(qvel,0.);
        std::vector<double> tangent(static_cast<std::size_t>(m->nv),0.);
        tangent[static_cast<std::size_t>(rig_.topology.root.free_dof_adr+3)]=lean;
        tangent[static_cast<std::size_t>(rig_.topology.steer.joint.dof_adr)]=steer;
        engine::integrate_pos(m,d->qpos,tangent.data(),1.);contact_positions();velocity(speed/parameters_.rR_m);
        const double first_speed=std::hypot(com_velocity[0],com_velocity[1]);
        bt::require(first_speed>0.,"ideal relative-equilibrium velocity has nonzero COM speed");
        velocity(speed*speed/(parameters_.rR_m*first_speed));
        const auto a=acceleration();const auto pose=bike::spatial::body_pose(*m,*d,rig_.topology.root);
        const double yaw=pose.angular_velocity_world_rad_s[2];
        bt::require(std::abs(yaw)>1e-9,"ideal corner has a finite turn rate");
        return {a.vdot[static_cast<std::size_t>(rig_.topology.root.free_dof_adr+3)],
            std::hypot(com_velocity[0],com_velocity[1])/yaw-radius};
    }
public:
    IdealCorner() {
        const auto request=bike::model::make_whipple_request(parameters_);
        rig_.model=bike::model::compile_mjcf(bike::model::generate_mjcf(request)).release();
        rig_.data=engine::make_data(rig_.model);if(!rig_.data)throw std::runtime_error("ideal corner data allocation");
        rig_.topology=bike::spatial::bind_topology(*rig_.model,*request.spatial);
    }
    struct Result {std::vector<Point> points;double lean_com=0.,lateral_force=0.,mass=0.,speed=0.,radius=0.;};
    Result solve(double radius,double speed) {
        double lean=-std::atan(speed*speed/(gravity*radius)),steer=std::atan(parameters_.w_m/radius);
        bool converged=false;
        for(unsigned iteration=0;iteration<24;++iteration) {
            const auto f=residual(lean,steer,speed,radius);
            if(std::abs(f[0])<1e-8 && std::abs(f[1])<1e-7){converged=true;break;}
            const double step=1e-5;
            const auto lp=residual(lean+step,steer,speed,radius),lm=residual(lean-step,steer,speed,radius);
            const auto sp=residual(lean,steer+step,speed,radius),sm=residual(lean,steer-step,speed,radius);
            const double a=(lp[0]-lm[0])/(2.*step),b=(sp[0]-sm[0])/(2.*step);
            const double c=(lp[1]-lm[1])/(2.*step),d=(sp[1]-sm[1])/(2.*step),det=a*d-b*c;
            if(std::abs(det)<1e-12)break;
            const double dl=std::clamp((-d*f[0]+b*f[1])/det,-.1,.1);
            const double ds=std::clamp((c*f[0]-a*f[1])/det,-.05,.05);
            lean+=dl;steer+=ds;
        }
        bt::require(converged,"inconclusive: thin-contact relative equilibrium was not found");
        (void)residual(lean,steer,speed,radius);
        auto* m=rig_.model;auto* d=rig_.data;const auto base=acceleration();engine::subtree_vel(m,d);
        const auto subtree_com=model_access::readonly_buffer(d->subtree_com,3*m->nbody);
        const auto subtree_velocity=model_access::readonly_buffer(d->subtree_linvel,3*m->nbody);
        const auto subtree_mass=model_access::readonly_buffer(m->body_subtreemass,m->nbody);
        auto qpos=model_access::mutable_buffer(d->qpos,m->nq);
        auto qvel=model_access::mutable_buffer(d->qvel,m->nv);
        const auto pose=bike::spatial::body_pose(*m,*d,rig_.topology.root);const double yaw_rate=pose.angular_velocity_world_rad_s[2];
        const Vec3 com{subtree_com[0],subtree_com[1],subtree_com[2]};
        const Vec3 velocity{subtree_velocity[0],subtree_velocity[1],subtree_velocity[2]};
        const Vec3 center=add(com,scale(cross(Vec3{0.,0.,1.},velocity),1./yaw_rate));
        const auto ground=contacts();const double normal=base.lambda[0]+base.lambda[3];
        const auto support=scale(add(scale(ground[0],base.lambda[0]),scale(ground[1],base.lambda[3])),1./normal);
        const auto inward=bike::pneumatic3d::detail::unit(Vec3{center[0]-com[0],center[1]-com[1],0.});
        Result result;result.mass=subtree_mass[0];result.speed=std::hypot(velocity[0],velocity[1]);
        result.lean_com=std::atan2(dot(sub(com,support),inward),com[2]-support[2]);
        const std::vector<double> q(qpos.begin(),qpos.end()),v(qvel.begin(),qvel.end());
        const auto rootq=static_cast<std::size_t>(rig_.topology.root.free_qpos_adr);
        const auto rootv=static_cast<std::size_t>(rig_.topology.root.free_dof_adr);
        for(unsigned sample=0;sample<=100;++sample) {
            const double time=.05*sample,angle=yaw_rate*time;
            const bike::spatial::Quat rotation{std::cos(.5*angle),0.,0.,std::sin(.5*angle)};
            std::ranges::copy(q,qpos.begin());std::ranges::copy(v,qvel.begin());
            const Vec3 original{q[static_cast<std::size_t>(rootq)],q[static_cast<std::size_t>(rootq+1)],q[static_cast<std::size_t>(rootq+2)]};
            const auto position=add(center,bike::spatial::rotate_body_to_world(rotation,sub(original,center)));
            const bike::spatial::Quat orientation{q[static_cast<std::size_t>(rootq+3)],q[static_cast<std::size_t>(rootq+4)],q[static_cast<std::size_t>(rootq+5)],q[static_cast<std::size_t>(rootq+6)]};
            const auto turned=bike::spatial::quat_multiply(rotation,orientation);
            const auto linear=bike::spatial::rotate_body_to_world(rotation,Vec3{v[static_cast<std::size_t>(rootv)],v[static_cast<std::size_t>(rootv+1)],v[static_cast<std::size_t>(rootv+2)]});
            for(std::size_t k=0;k<3;++k){qpos[rootq+k]=position[k];qvel[rootv+k]=linear[k];}
            for(std::size_t k=0;k<4;++k)qpos[rootq+3+k]=turned[k];
            for(const auto& wheel:rig_.topology.wheels)qpos[static_cast<std::size_t>(wheel.spin.qpos_adr)]+=time*v[static_cast<std::size_t>(wheel.spin.dof_adr)];
            d->time=time;engine::forward(m,d);engine::subtree_vel(m,d);const auto dynamic=acceleration();
            const auto required=cross(Vec3{0.,0.,yaw_rate},linear);
            for(std::size_t k=0;k<static_cast<std::size_t>(m->nv);++k) {
                const double expected=k>=rootv&&k<rootv+3?required[k-rootv]:0.;
                bt::require(std::abs(dynamic.vdot[k]-expected)<=2e-5,
                    "native acceleration verifies the prescribed thin-contact relative-equilibrium orbit");
            }
            Point p;p.time=time;p.mass=result.mass;p.speed=result.speed;p.yaw_rate=yaw_rate;
            for(std::size_t k=0;k<3;++k){p.com[k]=subtree_com[k];p.velocity[k]=subtree_velocity[k];}
            const auto c=knife_edge_constraints(*m,*d,rig_.topology);
            // Translation columns of J^T lambda are the complete external
            // reaction; include steering only as the internal seventh row.
            for(std::size_t axis=0;axis<3;++axis)for(std::size_t row=0;row<6;++row)
                p.force[axis]+=c.A[row*static_cast<std::size_t>(m->nv)+static_cast<std::size_t>(rootv)+axis]*dynamic.lambda[row];
            const auto in=bike::pneumatic3d::detail::unit(Vec3{center[0]-p.com[0],center[1]-p.com[1],0.});
            result.lateral_force+=dot(p.force,in)/101.;result.points.push_back(p);
        }
        result.radius=circle_radius(result.points);return result;
    }
};
} // namespace spatial_tests::ride
