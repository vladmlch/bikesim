#pragma once
// Cold validation fixtures share the production model, typed projection and
// force writers. No reduced dynamics or replacement material model lives here.
#include "rigs.hpp"
#include "../model_access.hpp"
#include "../runtime/project_config.hpp"
#include "../writer_config.hpp"
#include "../writers/drivetrain.hpp"
#include "../writers/suspension.hpp"
#include "../terrain/generators.hpp"
#include "../cblas_abi.hpp"
#include <tuple>

namespace bike::validation::mechanics {
inline Value metrics(std::initializer_list<std::pair<std::string,double>> fields) {
    Object out;for(const auto& [key,x]:fields)out.emplace_back(key,value::real(x));return Value{std::move(out)};
}
inline Value bounds(std::initializer_list<std::tuple<std::string,double,double>> fields) {
    Object out;for(const auto& [key,low,high]:fields)out.emplace_back(key,pair(low,high));return Value{std::move(out)};
}
inline model::BuildRequest request(configuration::SimulationPhysicsConfig physics,bool articulated=false) {
    configuration::post_init(physics);model::BuildRequest out;out.mode="ride";out.physics=std::move(physics);
    configuration::RiderSpecs rider;rider.variant=articulated?"articulated_planar":"none";out.rider=rider;return out;
}
struct BikeStand {
    model::BuildRequest request;
    Stand stand;
    nativecfg::WriterConfig writers;
    explicit BikeStand(model::BuildRequest input):request(std::move(input)),stand(model::generate_mjcf(request)),
        writers(nativecfg::decode(runtime::project_writer_config(request,*stand.model,terrain::get_preset("flat")))){}
    mjModel* model()const{return stand.model.get();}mjData* data()const{return stand.data.get();}
};
inline int qpos(const mjModel* m,const char* name){
    const auto adr=model_access::readonly_buffer(m->jnt_qposadr,m->njnt,"jnt_qposadr");
    return adr[static_cast<std::size_t>(object_id(m,mjOBJ_JOINT,name))];
}
inline int dof(const mjModel* m,const char* name){
    const auto adr=model_access::readonly_buffer(m->jnt_dofadr,m->njnt,"jnt_dofadr");
    return adr[static_cast<std::size_t>(object_id(m,mjOBJ_JOINT,name))];
}
inline void finite_state(const mjModel* m,const mjData* d) {
    for(double const v:model_access::readonly_buffer(d->qpos,m->nq,"qpos"))scalar(v,"fixture coordinates");
    for(double const v:model_access::readonly_buffer(d->qvel,m->nv,"qvel"))scalar(v,"fixture velocities");
}
inline std::array<double,3> matvec(std::span<const double> matrix,std::span<const double> x) {
    if(matrix.size()!=3*x.size()||x.size()>static_cast<std::size_t>(std::numeric_limits<int>::max()))throw std::invalid_argument("fixture Jacobian dimensions");
    std::array<double,3> out{};const int n=static_cast<int>(x.size());
    blas::dgemv(blas::Order::row_major,blas::Transpose::no,3,n,1.,matrix.data(),n,x.data(),1,0.,out.data(),1);return out;
}
inline void add_transpose(std::span<double> out,std::span<const double> jac,const std::array<double,3>& f) {
    if(jac.size()!=3*out.size()||out.size()>static_cast<std::size_t>(std::numeric_limits<int>::max()))throw std::invalid_argument("fixture force dimensions");
    std::vector<double> term(out.size());const int n=static_cast<int>(out.size());
    blas::dgemv(blas::Order::row_major,blas::Transpose::yes,3,n,1.,jac.data(),n,f.data(),1,0.,term.data(),1);
    for(std::size_t i=0;i<out.size();++i)out[i]+=term[i];
}
template<class Components> std::vector<double> sum_components(const Components& components,std::size_t n) {
    std::vector<double> out(n);for(const auto& [name,force]:components){(void)name;if(force.size()!=n)throw std::invalid_argument("force component width");for(std::size_t j=0;j<n;++j)out[j]+=force[j];}return out;
}
inline double telemetry(const drivetrain::DrivetrainWriter& drive,std::string_view name,double fallback=0.) {
    const auto& last=drive.diagnostics(false);if(!last.present(name))return fallback;return std::get<double>(last.at(name));
}
} // namespace bike::validation::mechanics
