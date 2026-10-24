#include "meijaard_constraints.hpp"
#include "../../../src/engine_call.hpp"
#include "../../../src/model_access.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <vector>

namespace spatial_tests {
namespace {
using Vec=std::array<double,3>;
double dot(const Vec& a,const Vec& b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
Vec cross(const Vec& a,const Vec& b){return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};}
Vec unit(Vec a){const double n=std::sqrt(dot(a,a));if(!(n>1e-10))throw std::invalid_argument("Whipple contact: degenerate wheel plane");for(auto& x:a)x/=n;return a;}
std::vector<double> rows(const mjModel& m,const mjData& d,const bike::spatial::Topology& t) {
    const auto nv=static_cast<std::size_t>(m.nv);
    if(m.nv!=t.nv||m.nq!=t.nq)throw std::invalid_argument("Whipple constraint: model topology dimensions differ");
    std::vector<double> a(6U*nv,0.),jp(3U*nv,0.),jr(3U*nv,0.);
    const auto sizes=model_access::readonly_buffer(m.geom_size,3*m.ngeom,"geom_size");
    const auto rotations=model_access::readonly_buffer(d.xmat,9*m.nbody,"xmat");
    const auto positions=model_access::readonly_buffer(d.xpos,3*m.nbody,"xpos");
    for(std::size_t w=0;w<2;++w) {
        const auto& wheel=t.wheels[w];
        const int geom=mj_name2id(&m,mjOBJ_GEOM,w==0?"geom_front_contact":"geom_rear_contact");
        if(geom<0||wheel.wheel_body_id<=0||wheel.wheel_body_id>=m.nbody||wheel.hub_body_id<=0||wheel.hub_body_id>=m.nbody)
            throw std::invalid_argument("Whipple constraint: missing wheel binding/radius");
        const double radius=sizes[3*static_cast<std::size_t>(geom)];
        if(!(radius>0.))throw std::invalid_argument("Whipple contact radius must be positive");
        Vec axle{};const auto rotation=rotations.subspan(9*static_cast<std::size_t>(wheel.hub_body_id),9);
        for(std::size_t i=0;i<3;++i)for(std::size_t j=0;j<3;++j)axle[i]+=rotation[3*i+j]*wheel.axle_axis_hub[j];
        axle=unit(axle);const Vec normal{0.,0.,1.};
        const auto up=unit(Vec{-axle[2]*axle[0],-axle[2]*axle[1],1.-axle[2]*axle[2]});
        Vec point{};for(std::size_t i=0;i<3;++i)point[i]=positions[3*static_cast<std::size_t>(wheel.wheel_body_id)+i]-radius*up[i];
        const auto longitudinal=unit(cross(axle,normal)),lateral=cross(normal,longitudinal);
        struct Call {const mjModel* model;const mjData* data;double* jp;double* jr;const double* point;int body;};
        Call call{&m,&d,jp.data(),jr.data(),point.data(),wheel.wheel_body_id};engine::ErrorBuffer error;
        if(!engine::invoke([](void* raw) noexcept {const auto& c=*static_cast<Call*>(raw);mj_jac(c.model,c.data,c.jp,c.jr,c.point,c.body);},&call,error))
            engine::throw_failure(error);
        const std::array axes{normal,longitudinal,lateral};
        for(std::size_t row=0;row<3;++row)for(std::size_t j=0;j<nv;++j)
            for(std::size_t axis=0;axis<3;++axis)a[(3U*w+row)*nv+j]+=axes[row][axis]*jp[axis*nv+j];
        // mj_jac at the contact point on the *wheel* already includes spin
        // omega x (P-hub). Adding a second spin correction would double it.
    }
    return a;
}
}
RollingConstraints knife_edge_constraints(const mjModel& m,const mjData& d,const bike::spatial::Topology& t) {
    RollingConstraints result;result.rows=6;result.A=rows(m,d,t);result.Adot_v.assign(6,0.);
    using Data=std::unique_ptr<mjData,decltype(&mj_deleteData)>;
    Data plus(engine::make_data(&m),mj_deleteData),minus(engine::make_data(&m),mj_deleteData);
    if(!plus||!minus)throw std::runtime_error("Whipple constraint: detached derivative data allocation failed");
    struct Copy {mjData* target;const mjModel* model;const mjData* source;};engine::ErrorBuffer error;
    for(auto* target:{plus.get(),minus.get()}) {
        Copy call{target,&m,&d};if(!engine::invoke([](void* raw) noexcept {
            const auto& c=*static_cast<Copy*>(raw);mj_copyData(c.target,c.model,c.source);
        },&call,error))engine::throw_failure(error);
    }
    constexpr double h=1e-7;
    const auto velocity=model_access::readonly_buffer(d.qvel,m.nv,"qvel");
    engine::integrate_pos(&m,plus->qpos,velocity.data(),h);engine::integrate_pos(&m,minus->qpos,velocity.data(),-h);
    engine::forward(&m,plus.get());engine::forward(&m,minus.get());
    const auto ap=rows(m,*plus,t),am=rows(m,*minus,t);const auto nv=static_cast<std::size_t>(m.nv);
    for(std::size_t row=0;row<6;++row)for(std::size_t j=0;j<nv;++j)
        result.Adot_v[row]+=(ap[row*nv+j]-am[row*nv+j])/(2.*h)*velocity[j];
    return result;
}
} // namespace spatial_tests
