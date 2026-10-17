#include "rider.hpp"
#include "../cblas_abi.hpp"
#include "../engine_call.hpp"
#include "../model/mapping.hpp"
#include "../numeric_sum.hpp"
#include <algorithm>
#include <cmath>
#include <ranges>
#include <limits>
#include <stdexcept>

namespace bike::legacy {
RiderForces::RiderForces(const mjModel& m, const std::optional<model::SeatedPose>& pose) : model_(m) {
    if (!pose) return;
    nativecfg::RiderForcesConfig config;
    for (const auto& b : pose->bodies)
        config.paths.push_back({b.name,b.joint,b.stiffness_n_m,b.damping_ns_m,b.preload_deflection_m(),0.,b.unilateral});
    // The existing writer owns the named/unlimited-slide validation.
    const RiderForcesWriter checked(&m,config);
    paths_.assign(checked.paths().begin(),checked.paths().end());
    telemetry_.resize(paths_.size()); staging_.resize(paths_.size());
}
void RiderForces::set_pedal_offsets(double front, double rear) {
    if (!std::isfinite(front) || !std::isfinite(rear)) throw std::invalid_argument("legacy rider pedal offsets must be finite");
    for (auto& path : paths_) {
        if (path.name == "rider_leg_front") path.offset_m = front;
        if (path.name == "rider_leg_rear") path.offset_m = rear;
    }
}
void RiderForces::compute_into(const mjData& data, std::span<double> force) {
    if (force.size() != static_cast<std::size_t>(model_.nv)) throw std::invalid_argument("legacy rider force width");
    const auto q = std::views::counted(data.qpos,model_.nq), v = std::views::counted(data.qvel,model_.nv);
    std::ranges::fill(force,0.);
    for (std::size_t i = 0; i < paths_.size(); ++i) {
        const auto& p = paths_[i]; auto& t = staging_[i];
        const auto qi=static_cast<std::size_t>(p.qposadr),vi=static_cast<std::size_t>(p.dofadr);
        const double deflection = p.preload_deflection_m + p.offset_m - q[qi];
        if (!std::isfinite(deflection)) throw std::invalid_argument("legacy rider deflection is nonfinite");
        t.gap_m = p.unilateral && deflection <= 0. ? -deflection : 0.;
        t.force_n = p.unilateral && deflection <= 0. ? 0. : p.stiffness_n_m * deflection - p.damping_ns_m * v[vi];
        if (!std::isfinite(t.force_n)) throw std::invalid_argument("legacy rider force is nonfinite");
        if (p.unilateral) t.force_n = std::max(0.,t.force_n);
        force[vi] = t.force_n;
    }
    telemetry_.swap(staging_);
}
RiderLoads RiderForces::loads() const {
    RiderLoads result;
    for (std::size_t i = 0; i < paths_.size(); ++i) {
        const auto& name = paths_[i].name; const auto& t = telemetry_[i];
        if (name == "rider_pelvis") { result.saddle_n = t.force_n; result.gap_m = t.gap_m; }
        else if (name == "rider_torso") result.torso_n = t.force_n;
        else if (name == "rider_arms") result.bar_n = t.force_n;
        else if (name == "rider_leg_front") result.front_pedal_n = t.force_n;
        else if (name == "rider_leg_rear") result.rear_pedal_n = t.force_n;
    }
    return result;
}
double RiderForces::stored_energy(const mjData& data) const {
    const auto q = std::views::counted(data.qpos,model_.nq);
    double result = 0.;
    for (const auto& p : paths_) {
        double delta = p.preload_deflection_m + p.offset_m - q[static_cast<std::size_t>(p.qposadr)];
        if (p.unilateral) delta = std::max(0.,delta);
        result += .5 * p.stiffness_n_m * delta * delta;
    }
    return result;
}
EnergyLedger::EnergyLedger(double initial) : initial_(initial) {
    if (!std::isfinite(initial)) throw std::invalid_argument("invalid initial mechanical energy");
}
double EnergyLedger::residual(double energy, double active, double external, double loss) const {
    for (double x : {energy,active,external,loss}) if (!std::isfinite(x)) throw std::invalid_argument("non-finite energy ledger");
    if (loss < 0.) throw std::invalid_argument("dissipation cannot be negative");
    return energy - initial_ - active - external + loss;
}
MechanicalEnergy mechanical_energy_terms(const mjModel& model, mjData& data, double elastic) {
    if (!std::isfinite(elastic) || elastic < 0.) throw std::invalid_argument("invalid elastic energy");
    if(model.nv<0||model.nv>std::numeric_limits<int>::max())throw std::invalid_argument("energy matrix width exceeds BLAS range");
    engine::forward(&model,&data);
    const auto n = static_cast<std::size_t>(model.nv);
    std::vector<double> mass(n*n), left(n);
    struct Call { const mjModel* m; const mjData* d; double* result; } call{&model,&data,mass.data()};
    engine::ErrorBuffer error;
    if (!engine::invoke([](void* raw) noexcept { auto& c = *static_cast<Call*>(raw); mj_fullM(c.m,c.d,c.result); },&call,error)) engine::throw_failure(error);
    for (std::size_t i = 0; i < n; ++i) left[i] = .5 * std::views::counted(data.qvel,model.nv)[i];
    std::vector<double> mv(n);
    if (n) blas::cblas_dgemv(static_cast<int>(blas::Order::row_major),static_cast<int>(blas::Transpose::yes),
        static_cast<int>(n),static_cast<int>(n),1.,mass.data(),static_cast<int>(n),left.data(),1,0.,mv.data(),1);
    const double kinetic = n ? blas::cblas_ddot(static_cast<int>(n),mv.data(),1,data.qvel,1) : 0.;
    const auto masses = std::views::counted(model.body_mass,model.nbody), positions = std::views::counted(data.xipos,3*model.nbody);
    std::vector<double> gravitational_terms(positions.size());
    for (std::size_t b = 0; b < masses.size(); ++b)
        for (std::size_t axis = 0; axis < 3; ++axis)
            gravitational_terms[3*b+axis]=masses[b]*positions[3*b+axis]*std::span(model.opt.gravity)[axis];
    const double gravitational=-numeric::numpy_pairwise_sum(gravitational_terms);
    return {kinetic,gravitational,elastic};
}
SystemMomentum system_momentum(const mjModel& model, mjData& data) {
    if(model.nbody<0||model.nbody>std::numeric_limits<int>::max())throw std::invalid_argument("momentum body count exceeds MuJoCo ID range");
    const auto masses = std::views::counted(model.body_mass,model.nbody);
    const double total=numeric::numpy_pairwise_sum(masses);
    if(total<=0.)throw std::invalid_argument("no physical system mass");
    if(model.nv<0||model.nv>std::numeric_limits<int>::max())throw std::invalid_argument("momentum Jacobian width exceeds BLAS range");
    engine::forward(&model,&data);
    const auto positions = std::views::counted(data.xipos,3*model.nbody);
    const auto rotations = std::views::counted(data.ximat,9*model.nbody), inertia = std::views::counted(model.body_inertia,3*model.nbody);
    std::array<double,3> com{};
    for (std::size_t b = 0; b < masses.size(); ++b)for (std::size_t a = 0; a < 3; ++a)com[a]+=masses[b]*positions[3*b+a];
    for (auto& x : com) x /= total;
    SystemMomentum result{};
    const auto width=static_cast<int>(model.nv);
    std::vector<double> jp(static_cast<std::size_t>(width)*3),jr(jp.size());
    for (std::size_t b = 0; b < masses.size(); ++b) {
        if (masses[b] == 0.) continue;
        const bike::model::WorldVector point{positions[3*b],positions[3*b+1],positions[3*b+2]};
        struct JacobianCall{const mjModel* model;const mjData* data;double* jp;double* jr;int body;} call{&model,&data,jp.data(),jr.data(),static_cast<int>(b)};
        engine::ErrorBuffer error;
        if(!engine::invoke([](void* raw)noexcept{auto& c=*static_cast<JacobianCall*>(raw);mj_jacBodyCom(c.model,c.data,c.jp,c.jr,c.body);},&call,error))engine::throw_failure(error);
        std::array<double,3> velocity{},omega{},linear{},spin{},arm{};
        if(width){blas::dgemv(blas::Order::row_major,blas::Transpose::no,3,width,1.,jp.data(),width,data.qvel,1,0.,velocity.data(),1);
            blas::dgemv(blas::Order::row_major,blas::Transpose::no,3,width,1.,jr.data(),width,data.qvel,1,0.,omega.data(),1);}
        std::array<double,9> diagonal{},scaled{},tensor{};
        for(std::size_t i=0;i<3;++i)diagonal[3*i+i]=inertia[3*b+i];
        const auto rotation=rotations.subspan(9*b,9);
        blas::dgemm(blas::Order::row_major,blas::Transpose::no,blas::Transpose::no,3,3,3,1.,rotation.data(),3,diagonal.data(),3,0.,scaled.data(),3);
        blas::dgemm(blas::Order::row_major,blas::Transpose::no,blas::Transpose::yes,3,3,3,1.,scaled.data(),3,rotation.data(),3,0.,tensor.data(),3);
        blas::dgemv(blas::Order::row_major,blas::Transpose::no,3,3,1.,tensor.data(),3,omega.data(),1,0.,spin.data(),1);
        for (std::size_t a = 0; a < 3; ++a) {
            linear[a] = masses[b]*velocity[a]; result.linear[a] += linear[a]; arm[a] = point[a]-com[a];
        }
        result.angular[0] += spin[0] + (arm[1]*linear[2]-arm[2]*linear[1]);
        result.angular[1] += spin[1] + (arm[2]*linear[0]-arm[0]*linear[2]);
        result.angular[2] += spin[2] + (arm[0]*linear[1]-arm[1]*linear[0]);
    }
    return result;
}
} // namespace bike::legacy
