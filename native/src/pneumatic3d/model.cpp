#include "pneumatic3d/model.hpp"
#include "pneumatic3d/element_work.hpp"
#include "runtime/spatial_worker_pool.hpp"
#include "runtime/spatial_timing.hpp"
#include "pneumatic3d/bristles.hpp"
#include "pneumatic3d/brush_detail.hpp"
#include "pneumatic3d/kernel_math.hpp"
#include "pneumatic3d/state_detail.hpp"
#include "spatial/frames.hpp"
#include "terrain/grid3d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace bike::pneumatic3d {
namespace {
using spatial::Vec3;
double dot(const Vec3& a,const Vec3& b) noexcept { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
Vec3 point_velocity(const WheelKinematics3d& k,const Vec3& point) noexcept {
    auto pose=k.hub;
    for(std::size_t a=0;a<3;++a)pose.angular_velocity_world_rad_s[a]+=k.axle_world[a]*k.relative_spin_rad_s;
    return spatial::point_velocity_world(pose,point);
}
template<class T> void copy_vector(const std::vector<T>& from,std::vector<T>& to) noexcept {
    std::copy(from.begin(),from.end(),to.begin());
}
// Identity and capacities never change on the warm path. In particular this
// does not copy strings, resize vectors, or rely on vector::operator= capacity.
void copy_material(const TyreState3d& from,TyreState3d& to) noexcept {
    for(std::size_t w=0;w<2;++w) {
        const auto& c=from.carcass[w];auto& d=to.carcass[w];
        copy_vector(c.deflection_m,d.deflection_m);copy_vector(c.maxwell_q_n_per_m2,d.maxwell_q_n_per_m2);
        copy_vector(c.previous_penetration_m,d.previous_penetration_m);copy_vector(c.active,d.active);
        d.material_phase_rad=c.material_phase_rad;d.stored_energy_j=c.stored_energy_j;d.dissipated_j=c.dissipated_j;
        const auto& p=from.patches[w];auto& q=to.patches[w];
        copy_vector(p.ids,q.ids);copy_vector(p.element_to_patch_slot,q.element_to_patch_slot);copy_vector(p.slot_used,q.slot_used);
        q.next_id=p.next_id;q.capacity=p.capacity;
        const auto& b=from.brush[w];auto& e=to.brush[w];
        copy_vector(b.ux_m,e.ux_m);copy_vector(b.uy_m,e.uy_m);copy_vector(b.camber_state,e.camber_state);
        copy_vector(b.bristle_ux_m,e.bristle_ux_m);copy_vector(b.bristle_uy_m,e.bristle_uy_m);
        copy_vector(b.bristle_energy_j,e.bristle_energy_j);
        e.elastic_energy_j=b.elastic_energy_j;e.sliding_loss_j=b.sliding_loss_j;e.convective_release_j=b.convective_release_j;
    }
    to.accepted_interval=from.accepted_interval;
}
void size_state(TyreState3d& s,const TyreStateIdentity3d& id) {
    s.identity=id;
    const auto n=static_cast<std::size_t>(id.dims.n_theta_fan)*id.dims.n_phi;
    const auto m=static_cast<std::size_t>(id.dims.n_theta_material)*id.dims.n_phi;
    for(std::size_t w=0;w<2;++w) {
        auto& c=s.carcass[w];c.deflection_m.assign(m,0.);c.maxwell_q_n_per_m2.assign(m,0.);
        c.previous_penetration_m.assign(m,0.);c.active.assign(m,0);
        auto& p=s.patches[w];p.capacity=id.patch_capacity;p.ids.assign(p.capacity,0);
        p.slot_used.assign(p.capacity,0);p.element_to_patch_slot.assign(n,std::numeric_limits<std::uint32_t>::max());
        auto& b=s.brush[w];b.ux_m.assign(p.capacity,0.);b.uy_m.assign(p.capacity,0.);b.camber_state.assign(p.capacity,0.);
        b.bristle_ux_m.assign(m,0.);b.bristle_uy_m.assign(m,0.);b.bristle_energy_j.assign(m,0.);
    }
}
double surface_mu(const terrain::Terrain3dQueryView& view,std::uint32_t id,double speed) noexcept {
    const auto* material=view.surface(id);
    if(!material)return -1.;
    if(material->stribeck_speed_mps<=0.)return speed==0.?material->mu_peak:material->mu_slide;
    const double ratio=std::abs(speed)/material->stribeck_speed_mps;
    return material->mu_slide+(material->mu_peak-material->mu_slide)*std::exp(-ratio*ratio);
}
}

struct Pneumatic3dModel::ElementWorkers {
    ElementWorkers(unsigned threads,std::size_t elements)
        :pool(threads),ranges(::runtime::spatial::partition_ranges(elements,threads)),statuses(elements,0) {}
    struct Work {
        const TorusGrid3d* grid=nullptr;
        const WheelKinematics3d* kin=nullptr;
        const terrain::Terrain3dQueryView* view=nullptr;
        const CarcassSpec3d* carcass=nullptr;
        const CarcassState3d* state=nullptr;
        std::span<ContactElement3d> elements;
        std::span<NormalElement3d> normals;
        std::span<std::uint8_t> statuses;
        double dt=0.,pressure=0.;
    };
    static void geometry_work(void* raw,std::size_t begin,std::size_t end) noexcept {
        auto& w=*static_cast<Work*>(raw);
        const auto status=detail::query_geometry_range(*w.grid,*w.kin,*w.view,w.elements,begin,end);
        std::fill(w.statuses.begin()+static_cast<std::ptrdiff_t>(begin),
                  w.statuses.begin()+static_cast<std::ptrdiff_t>(end),static_cast<std::uint8_t>(status));
    }
    static void normal_work(void* raw,std::size_t begin,std::size_t end) noexcept {
        auto& w=*static_cast<Work*>(raw);
        const auto status=detail::evaluate_carcass_range(*w.carcass,w.elements,w.dt,*w.state,
            w.normals,w.pressure,begin,end);
        std::fill(w.statuses.begin()+static_cast<std::ptrdiff_t>(begin),
                  w.statuses.begin()+static_cast<std::ptrdiff_t>(end),static_cast<std::uint8_t>(status));
    }
    CoreStatus run(Work& work,::runtime::spatial::ElementWork callback) noexcept {
        work.statuses=statuses;
        try { pool.run(ranges,callback,&work); }
        catch (...) { return CoreStatus::invalid_input; }
        for(const auto status:statuses)
            if(status!=static_cast<std::uint8_t>(CoreStatus::ok))return static_cast<CoreStatus>(status);
        return CoreStatus::ok;
    }
    ::runtime::spatial::WorkerPool pool;
    std::vector<::runtime::spatial::IndexRange> ranges;
    std::vector<std::uint8_t> statuses;
};

Pneumatic3dModel::~Pneumatic3dModel() = default;
void Pneumatic3dModel::set_worker_threads(unsigned threads) {
    if(threads==0 || threads>64)throw std::invalid_argument("pneumatic3d.tyre_threads: expected [1,64]");
    if(threads==worker_threads())return;
    if(threads==1){workers_.reset();return;}
    auto candidate=std::make_unique<ElementWorkers>(threads,grid_[0].element_count());
    workers_=std::move(candidate);
}
unsigned Pneumatic3dModel::worker_threads() const noexcept {
    return workers_?workers_->pool.worker_count()+1U:1U;
}

Pneumatic3dModel::Pneumatic3dModel(Pneumatic3dSpec spec,terrain::Terrain3dQueryView view,TyreStateIdentity3d identity)
    :spec_(std::move(spec)),view_(view) {
    if(!view.context || !view.sample_fn || !view.ray_fn)
        throw std::invalid_argument("pneumatic3d.terrain: bound immutable query view required");
    if(view.surface_table.empty())
        throw std::invalid_argument("pneumatic3d.terrain: immutable surface registry required");
    const auto dims=tier_dims(spec_.tier);
    const auto n=static_cast<std::size_t>(dims.n_theta_fan)*dims.n_phi;
    // Material/normal discontinuities can disconnect every single element.
    // ceil(N/2) bounds a checkerboard only on a homogeneous smooth surface.
    if(!spec_.patch_capacity)spec_.patch_capacity=static_cast<std::uint32_t>(n);
    if(spec_.patch_capacity>n || !spec_.patch_capacity)
        throw std::invalid_argument("pneumatic3d.patch_capacity: outside [1,element_count]");
    if(identity.tier!=spec_.tier || identity.dims.n_theta_fan!=dims.n_theta_fan ||
       identity.dims.n_phi!=dims.n_phi || identity.dims.n_theta_material!=dims.n_theta_material ||
       identity.patch_capacity!=spec_.patch_capacity)
        throw std::invalid_argument("pneumatic3d.identity: tier/dimensions/capacity differ from specification");
    for(std::size_t w=0;w<2;++w) {
        grid_[w]=make_torus_grid(dims,spec_.outer_radius_m[w],spec_.casing_width_m[w]);
        elements_[w].resize(n);normals_[w].resize(n);patches_[w].resize(spec_.patch_capacity);
        wrench_[w].resize(std::max(n,static_cast<std::size_t>(spec_.patch_capacity)));
        patch_element_index_[w].resize(n);
    }
    size_state(committed_,identity);size_state(staging_,identity);
    if(!detail::validate_state_layout(committed_,identity))
        throw std::invalid_argument("pneumatic3d.identity: noncanonical state identity");
    // Exercise only the non-advancing, empty-input validation paths during
    // cold construction. No engine, terrain traversal, or simulation runs.
    for(std::size_t w=0;w<2;++w) {
        if(update_carcass(spec_.carcass[w],elements_[w],0.,false,staging_.carcass[w],normals_[w])!=CoreStatus::ok)
            throw std::invalid_argument("pneumatic3d.carcass: invalid coefficients");
        const auto& b=spec_.brush[w];
        if(!(std::isfinite(b.kx_n_per_m3)&&b.kx_n_per_m3>0.&&std::isfinite(b.ky_n_per_m3)&&b.ky_n_per_m3>0.&&
             std::isfinite(b.relaxation_x_m)&&b.relaxation_x_m>0.&&std::isfinite(b.relaxation_y_m)&&b.relaxation_y_m>0.&&
             std::isfinite(b.mu_x_scale)&&b.mu_x_scale>=0.&&std::isfinite(b.mu_y_scale)&&b.mu_y_scale>=0.&&
             std::isfinite(b.camber_coefficient_per_rad)&&std::isfinite(b.pneumatic_trail_m)&&b.pneumatic_trail_m>=0.&&
             std::isfinite(b.r_eff_m)&&b.r_eff_m>0.))
            throw std::invalid_argument("pneumatic3d.brush: invalid coefficients");
    }
}

CoreStatus Pneumatic3dModel::evaluate(const std::array<WheelKinematics3d,2>& kin,
    std::span<const Vec3> supplied_velocity,double dt,bool advance) noexcept {
    reason_=ReasonCode3d::none;
    if(!std::isfinite(dt)||dt<0. || (!supplied_velocity.empty() &&
       supplied_velocity.size()!=2*(static_cast<std::size_t>(spec_.patch_capacity)+1))) {
        reason_=ReasonCode3d::invalid_spec;return CoreStatus::invalid_input;
    }
    copy_material(committed_,staging_);
    // An interval is an accepted owner call. Clock-based duplicate protection
    // lives in the runtime adapter; this engine-independent signature has no
    // time/interval token (C10). A probe never changes accepted_interval.
    for(std::size_t w=0;w<2;++w) {
        CoreStatus result=CoreStatus::ok;
        {
            ::runtime::spatial::timing::Scope rays(::runtime::spatial::timing::Region::terrain_rays);
            if(workers_) {
                ElementWorkers::Work work;work.grid=&grid_[w];work.kin=&kin[w];work.view=&view_;
                work.elements=elements_[w];
                result=workers_->run(work,ElementWorkers::geometry_work);
            } else result=query_geometry(grid_[w],kin[w],view_,elements_[w]);
        }
        if(result!=CoreStatus::ok){reason_=result==CoreStatus::invalid_geometry?
            ReasonCode3d::degenerate_down_projection:ReasonCode3d::invalid_spec;return result;}
        for(const auto& e:elements_[w])if(e.reason!=ReasonCode3d::none) {
            reason_=e.reason;return CoreStatus::invalid_geometry;
        }
        ::runtime::spatial::timing::Scope tyre(::runtime::spatial::timing::Region::tyre_elements);
        if(workers_) {
            double pressure=0.;
            result=detail::prepare_carcass(spec_.carcass[w],elements_[w],dt,staging_.carcass[w],normals_[w],pressure);
            if(result==CoreStatus::ok) {
                ElementWorkers::Work work;work.carcass=&spec_.carcass[w];work.state=&staging_.carcass[w];
                work.elements=elements_[w];work.normals=normals_[w];work.dt=dt;work.pressure=pressure;
                result=workers_->run(work,ElementWorkers::normal_work);
            }
            if(result==CoreStatus::ok)
                result=detail::finish_carcass(spec_.carcass[w],elements_[w],dt,advance,staging_.carcass[w],normals_[w]);
        } else result=update_carcass(spec_.carcass[w],elements_[w],dt,advance,staging_.carcass[w],normals_[w]);
        if(result!=CoreStatus::ok){reason_=ReasonCode3d::invalid_spec;return result;}
        if(advance && dt>0.) {
            // The low-level carcass kernel sees active quadrature only. Its
            // inactive material cells still retain and dissipate Maxwell
            // memory. A cell's lateral row determines its fixed area, so the
            // owning grid can account for that memory without hidden state.
            const auto& c=spec_.carcass[w];
            auto& radial=staging_.carcass[w];
            if(c.maxwell_k_n_per_m3>0.) {
                for(std::size_t j=0;j<radial.active.size();++j)if(!radial.active[j]) {
                    const double area=grid_[w].geometric_area_m2[j%grid_[w].dims.n_phi];
                    const double q0=committed_.carcass[w].maxwell_q_n_per_m2[j];
                    const double q1=radial.maxwell_q_n_per_m2[j];
                    const double old_energy=.5*area*q0*q0/c.maxwell_k_n_per_m3;
                    const double new_energy=.5*area*q1*q1/c.maxwell_k_n_per_m3;
                    radial.stored_energy_j+=new_energy;
                    radial.dissipated_j+=std::max(0.,old_energy-new_energy);
                }
            }
            // Pressure is reversible work as well as support. The frozen
            // state has one aggregate radial energy lane, so add its
            // potential to the carcass/Maxwell contribution in that lane.
            double volume=0.;
            for(const auto& e:elements_[w])if(e.valid && e.penetration_m>0.)
                volume+=e.penetration_m*e.geometric_area_m2;
            const double potential=c.constant_pressure?c.pressure_pa*volume:
                -c.pressure_pa*c.initial_volume_m3*std::log1p(-volume/c.initial_volume_m3);
            if(!std::isfinite(potential)||potential<0.){reason_=ReasonCode3d::invalid_spec;return CoreStatus::invalid_geometry;}
            radial.stored_energy_j+=potential;
            if(!std::isfinite(radial.stored_energy_j)||!std::isfinite(radial.dissipated_j)) {
                reason_=ReasonCode3d::invalid_spec;return CoreStatus::invalid_geometry;
            }
        }
        for(auto& p:patches_[w])p.elements=patch_element_index_[w];
        result=assign_patches(elements_[w],normals_[w],true,staging_.patches[w],patches_[w],patch_count_[w]);
        if(result!=CoreStatus::ok){reason_=ReasonCode3d::patch_overflow;return result;}
        auto& brush=staging_.brush[w];
        const auto& previous=committed_.brush[w];
        const auto& previous_ids=committed_.patches[w].ids;
        brush.elastic_energy_j=0.;
        // Fast relaxation belongs to patch identity, never to an incidental
        // output index. Release vanished patches once, without borrowing a
        // surviving patch's memory at a split.
        for(std::size_t s=0;s<brush.ux_m.size();++s) {
            brush.ux_m[s]=brush.uy_m[s]=brush.camber_state[s]=0.;
            if(!staging_.patches[w].slot_used[s])continue;
            const auto id=staging_.patches[w].ids[s];
            for(std::size_t old=0;old<previous_ids.size();++old)if(previous_ids[old]==id) {
                brush.ux_m[s]=previous.ux_m[old];brush.uy_m[s]=previous.uy_m[old];break;
            }
        }
        if(spec_.tier==spatial::TyreTier::fast) {
            // This branch is the fixed 64 x 7 fast grid (3.5 KiB scratch).
            std::array<double,64U*7U> old_area{};
            const auto& old_map=committed_.patches[w].element_to_patch_slot;
            for(std::size_t i=0;i<old_map.size();++i)if(old_map[i]<previous_ids.size())
                old_area[old_map[i]]+=grid_[w].geometric_area_m2[i];
            if(advance && dt>0.)for(std::size_t old=0;old<previous_ids.size();++old) {
                if(!committed_.patches[w].slot_used[old])continue;
                const bool survives=std::find(staging_.patches[w].ids.begin(),staging_.patches[w].ids.end(),previous_ids[old])!=staging_.patches[w].ids.end();
                if(!survives)brush.convective_release_j+=.5*old_area[old]*(
                    spec_.brush[w].kx_n_per_m3*previous.ux_m[old]*previous.ux_m[old]+
                    spec_.brush[w].ky_n_per_m3*previous.uy_m[old]*previous.uy_m[old]);
            }
            for(std::size_t p=0;p<patch_count_[w];++p) {
                const auto& patch=patches_[w][p];
                std::size_t slot=0;
                while(slot<staging_.patches[w].ids.size() && staging_.patches[w].ids[slot]!=patch.id)++slot;
                if(slot==staging_.patches[w].ids.size()){reason_=ReasonCode3d::patch_overflow;return CoreStatus::invalid_geometry;}
                for(std::size_t old=0;old<previous_ids.size();++old)if(previous_ids[old]==patch.id) {
                    // The serialized old fan membership and immutable cell
                    // areas reconstruct the old patch energy after restore.
                    // Shrinking releases removed material; growing dilutes
                    // extension so a topology change cannot create energy.
                    if(patch.area_m2>old_area[old]) {
                        const double scale=std::sqrt(old_area[old]/patch.area_m2);
                        brush.ux_m[slot]*=scale;brush.uy_m[slot]*=scale;
                    } else if(advance && dt>0.)brush.convective_release_j+=.5*(old_area[old]-patch.area_m2)*(
                        spec_.brush[w].kx_n_per_m3*brush.ux_m[slot]*brush.ux_m[slot]+
                        spec_.brush[w].ky_n_per_m3*brush.uy_m[slot]*brush.uy_m[slot]);
                    break;
                }
                const auto v=supplied_velocity.empty()?point_velocity(kin[w],patch.centroid_world_m):
                    supplied_velocity[w*(spec_.patch_capacity+1)+p];
                const auto vh=supplied_velocity.empty()?kin[w].hub.linear_velocity_world_mps:
                    supplied_velocity[w*(spec_.patch_capacity+1)+spec_.patch_capacity];
                const auto mu=surface_mu(view_,patch.surface_id,std::hypot(dot(v,patch.longitudinal_world),dot(v,patch.lateral_world)));
                if(mu<0.){reason_=ReasonCode3d::terrain_query_failed;return CoreStatus::invalid_geometry;}
                const auto gamma=std::asin(std::clamp(dot(kin[w].axle_world,patch.normal_world),-1.,1.));
                auto angular=kin[w].hub.angular_velocity_world_rad_s;
                for(std::size_t a=0;a<3;++a)angular[a]+=kin[w].axle_world[a]*kin[w].relative_spin_rad_s;
                const auto output=detail::evaluate_fast_patch(spec_.brush[w],patch,v,vh,kin[w].axle_world,
                    dot(angular,kin[w].axle_world),gamma,mu*spec_.brush[w].mu_x_scale,mu*spec_.brush[w].mu_y_scale,
                    brush.ux_m[slot],brush.uy_m[slot],dt,angular);
                if(output.status!=CoreStatus::ok){reason_=ReasonCode3d::invalid_spec;return output.status;}
                wrench_[w][p]=output.wrench;
                brush.ux_m[slot]=output.ux_m;brush.uy_m[slot]=output.uy_m;brush.camber_state[slot]=gamma;
                brush.elastic_energy_j+=output.stored_j;
                if(advance && dt>0.){brush.sliding_loss_j+=output.sliding_j;brush.convective_release_j+=output.convective_j;}
            }
        } else {
            // The detailed kernel receives local road coefficients through
            // its private resolver overload; public W00 remains unchanged.
            result=detail::update_detailed_resolved(spec_.brush[w],elements_[w],normals_[w],kin[w],
                dt,advance,brush,wrench_[w],view_);
            if(result!=CoreStatus::ok){reason_=ReasonCode3d::invalid_spec;return result;}
            for(std::size_t i=0;i<elements_[w].size();++i)if(wrench_[w][i].valid) {
                const auto slot=staging_.patches[w].element_to_patch_slot[i];
                if(slot<staging_.patches[w].ids.size())wrench_[w][i].patch_id=staging_.patches[w].ids[slot];
            }
        }
        if(advance && dt>0.)staging_.carcass[w].material_phase_rad=std::remainder(kin[w].wheel_phase_rad,2.*std::numbers::pi);
    }
    if(advance && dt>0.) {
        double stored=0.,lost=0.;
        for(std::size_t w=0;w<2;++w) {
            stored+=staging_.carcass[w].stored_energy_j+staging_.brush[w].elastic_energy_j;
            lost+=staging_.carcass[w].dissipated_j+staging_.brush[w].sliding_loss_j+
                  staging_.brush[w].convective_release_j;
        }
        if(!std::isfinite(stored)||!std::isfinite(lost)) {
            reason_=ReasonCode3d::invalid_spec;return CoreStatus::invalid_geometry;
        }
        if(committed_.accepted_interval==std::numeric_limits<std::int64_t>::max()) {
            reason_=ReasonCode3d::duplicate_advance;return CoreStatus::invalid_input;
        }
        staging_.accepted_interval=committed_.accepted_interval+1;
        copy_material(staging_,committed_);last_advanced_interval_=committed_.accepted_interval;
    }
    return CoreStatus::ok;
}
std::span<const PatchWrench> Pneumatic3dModel::wrenches(std::size_t w) const noexcept {
    if(w>=2)return {};
    return std::span<const PatchWrench>(wrench_[w]).first(spec_.tier==spatial::TyreTier::fast?patch_count_[w]:elements_[w].size());
}
std::span<const Patch3d> Pneumatic3dModel::patches(std::size_t w) const noexcept {
    return w<2?std::span<const Patch3d>(patches_[w]).first(patch_count_[w]):std::span<const Patch3d>{};
}
const TyreState3d& Pneumatic3dModel::committed_state() const noexcept {return committed_;}
CoreStatus Pneumatic3dModel::restore(const TyreState3d& state) noexcept {
    if(!detail::validate_state_layout(state,committed_.identity)) {
        reason_=ReasonCode3d::state_identity_mismatch;return CoreStatus::invalid_input;
    }
    copy_material(state,committed_);copy_material(state,staging_);last_advanced_interval_=state.accepted_interval;
    patch_count_.fill(0);reason_=ReasonCode3d::none;
    for(auto& lane:wrench_)std::fill(lane.begin(),lane.end(),PatchWrench{});
    return CoreStatus::ok;
}
ReasonCode3d Pneumatic3dModel::last_reason() const noexcept {return reason_;}
double Pneumatic3dModel::stored_energy_j() const noexcept {
    double e=0.;for(std::size_t w=0;w<2;++w)e+=committed_.carcass[w].stored_energy_j+committed_.brush[w].elastic_energy_j;return e;
}
double Pneumatic3dModel::dissipated_j() const noexcept {
    double e=0.;for(std::size_t w=0;w<2;++w)e+=committed_.carcass[w].dissipated_j+committed_.brush[w].sliding_loss_j+
        committed_.brush[w].convective_release_j;return e;
}
const Pneumatic3dSpec& Pneumatic3dModel::spec() const noexcept {return spec_;}
} // namespace bike::pneumatic3d
