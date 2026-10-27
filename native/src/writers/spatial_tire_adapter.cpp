#include "writers/spatial_tire_adapter.hpp"
#include "engine_call.hpp"
#include "model_access.hpp"
#include "spatial/frames.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace bike::pneumatic3d {
SpatialTireEvaluationError::~SpatialTireEvaluationError() = default;
}

SpatialTireAdapter::SpatialTireAdapter(const mjModel& m,bike::spatial::Topology topology,
    bike::pneumatic3d::Pneumatic3dTireWriter& writer,bike::terrain::Terrain3dQueryView terrain)
    :model_(&m),topology_(std::move(topology)),writer_(&writer),terrain_(terrain),initial_(writer.state()),radii_(writer.model().spec().outer_radius_m) {
    const auto& dims=writer.state().identity.dims;
    const auto bound=std::max(static_cast<std::size_t>(writer.state().identity.patch_capacity),
        static_cast<std::size_t>(dims.n_theta_fan)*dims.n_phi);
    for(std::size_t w=0;w<2;++w) {
        model_access::require_id(topology_.wheels[w].wheel_body_id,m.nbody,"pneumatic_3d.wheel_body");
        model_access::require_id(topology_.wheels[w].hub_body_id,m.nbody,"pneumatic_3d.hub_body");
        for(auto* lane:{&snapshots_[w],&probes_[w],&projection_stage_[w]}) {
            lane->backend="pneumatic_3d";lane->patches.reserve(bound);
        }
        diagnostics_[w].surface="terrain3d";diagnostics_[w].calibration_status="provisional";
        diagnostics_[w].capability_status="qualification_pending";
        diagnostics_[w].supports_multiple_contacts=true;
        diagnostics_[w].station_count=static_cast<int>(dims.n_theta_fan*dims.n_phi);
    }
}
CoreStatus SpatialTireAdapter::capture_motion(const mjData& data) noexcept {
    struct Call {
        SpatialTireAdapter* self;
        const mjData* data;
        std::span<const mjtNum> positions,rotations,mass_centers;
        bool valid=true;
    } call{.self=this,.data=&data,.positions={},.rotations={},.mass_centers={}};
    try {
        call.positions=model_access::readonly_buffer(data.xpos,3*model_->nbody,"pneumatic_3d.xpos");
        call.rotations=model_access::readonly_buffer(data.xquat,4*model_->nbody,"pneumatic_3d.xquat");
        call.mass_centers=model_access::readonly_buffer(data.xipos,3*model_->nbody,"pneumatic_3d.xipos");
    } catch(...) {return CoreStatus::invalid_input;}
    engine::ErrorBuffer error;
    if(!engine::invoke([](void* raw) noexcept {
        auto& c=*static_cast<Call*>(raw);auto& self=*c.self;
        for(std::size_t w=0;w<2;++w) {
            const auto& binding=self.topology_.wheels[w];auto& pose=self.wheel_motion_[w];
            const auto wheel=static_cast<std::size_t>(binding.wheel_body_id);
            const auto hub=static_cast<std::size_t>(binding.hub_body_id);
            std::array<mjtNum,6> velocity{};
            mj_objectVelocity(self.model_,c.data,mjOBJ_BODY,binding.wheel_body_id,velocity.data(),0);
            for(std::size_t a=0;a<3;++a) {
                pose.position_world_m[a]=c.positions[3*wheel+a];
                pose.angular_velocity_world_rad_s[a]=velocity[a];
                pose.linear_velocity_world_mps[a]=velocity[3+a];
            }
            for(std::size_t a=0;a<4;++a)pose.quat_world_from_body_wxyz[a]=c.rotations[4*hub+a];
            const bike::spatial::Vec3 lever{
                pose.position_world_m[0]-c.mass_centers[3*wheel],
                pose.position_world_m[1]-c.mass_centers[3*wheel+1],
                pose.position_world_m[2]-c.mass_centers[3*wheel+2]};
            const auto& omega=pose.angular_velocity_world_rad_s;
            pose.linear_velocity_world_mps[0]+=omega[1]*lever[2]-omega[2]*lever[1];
            pose.linear_velocity_world_mps[1]+=omega[2]*lever[0]-omega[0]*lever[2];
            pose.linear_velocity_world_mps[2]+=omega[0]*lever[1]-omega[1]*lever[0];
            bike::pneumatic3d::WheelKinematics3d kin;
            kin.hub=pose;kin.axle_world=bike::spatial::rotate_body_to_world(pose.quat_world_from_body_wxyz,binding.axle_axis_hub);
            const auto frame=bike::pneumatic3d::fan_frame(kin);
            self.forward_[w]=frame.forward;c.valid=c.valid&&frame.valid;
        }
    },&call,error))return CoreStatus::engine_failure;
    return call.valid?CoreStatus::ok:CoreStatus::invalid_geometry;
}
// NOLINTNEXTLINE(bugprone-exception-escape) patch lanes are cold-reserved to capacity in the ctor and every fallible call is a noexcept query
CoreStatus SpatialTireAdapter::project(const mjData& data) noexcept {
    std::span<const mjtNum> positions;
    try {positions=model_access::readonly_buffer(data.xpos,3*model_->nbody,"pneumatic_3d.xpos");}
    catch(...) {return CoreStatus::invalid_input;}
    for(std::size_t w=0;w<2;++w) {
        auto& s=projection_stage_[w];s.time_s=data.time;s.interval_id=writer_->state().accepted_interval;
        s.patches.clear();s.geometric_contact=false;
        const auto wheel=static_cast<std::size_t>(topology_.wheels[w].wheel_body_id);
        for(std::size_t a=0;a<3;++a)s.wheel_axis_m[a]=positions[3*wheel+a];
        double weight=0.,radius=0.;
        for(const auto& p:writer_->wrenches(w))if(p.valid) {
            std::array<double,3> n{};
            auto longitudinal=forward_[w];
            // A fast patch uses its weighted frame; detailed outputs retain
            // each element's local terrain normal, including banks/slopes.
            bool found=false;
            if(writer_->model().spec().tier==bike::spatial::TyreTier::fast)
                for(const auto& patch:writer_->model().patches(w))if(patch.id==p.patch_id) {
                    n=patch.normal_world;longitudinal=patch.longitudinal_world;found=true;break;
                }
            if(!found) {
                const auto sample=terrain_.sample(p.point_world_m[0],p.point_world_m[1]);
                if(sample.status!=bike::terrain::QueryStatus3d::ok)return CoreStatus::invalid_geometry;
                n=sample.normal_world;
                double projection=0.;for(std::size_t a=0;a<3;++a)projection+=longitudinal[a]*n[a];
                double norm=0.;for(std::size_t a=0;a<3;++a){longitudinal[a]-=projection*n[a];norm+=longitudinal[a]*longitudinal[a];}
                if(!(norm>1e-24))return CoreStatus::invalid_geometry;
                for(auto& value:longitudinal)value/=std::sqrt(norm);
            }
            const auto velocity=bike::spatial::point_velocity_world(wheel_motion_[w],p.point_world_m);
            double slip_mps=0.;for(std::size_t a=0;a<3;++a)slip_mps+=velocity[a]*longitudinal[a];
            s.patches.push_back(TirePatch{.point_m=p.point_world_m,.normal=n,.normal_load_n=p.normal_load_n,
                .tangent_force_n=p.fx_n,.slip_mps=slip_mps,.working_surface=true,
                .native_world_force_n=p.force_world_n,.couple_world_nm=p.couple_world_nm});
            s.geometric_contact=true;
            double drop=0.;for(std::size_t a=0;a<3;++a)drop+=(s.wheel_axis_m[a]-p.point_world_m[a])*n[a];
            weight+=p.normal_load_n;radius+=p.normal_load_n*drop;
        }
        s.effective_radius_m=weight>0.?radius/weight:radii_[w];
    }
    // Terrain projection can fail on either wheel after the previous pair
    // was published. Keep that pair intact until the whole replacement is
    // ready; both vector sets have the same cold-reserved capacity.
    probes_.swap(projection_stage_);probed_=true;
    return CoreStatus::ok;
}
// NOLINTNEXTLINE(bugprone-exception-escape) snapshot lanes are cold-reserved to capacity in the ctor and every fallible call is a noexcept query
void SpatialTireAdapter::publish_probe(const mjData& data) noexcept {
    for(std::size_t w=0;w<2;++w) {
        auto& s=snapshots_[w];const auto& probe=probes_[w];
        s.time_s=data.time;s.interval_id=writer_->state().accepted_interval;
        s.geometric_contact=probe.geometric_contact;s.wheel_axis_m=probe.wheel_axis_m;
        s.effective_radius_m=probe.effective_radius_m;
        s.patches.clear();for(const auto& patch:probe.patches)s.patches.push_back(patch);
        double normal=0.,fx=0.,fy=0.,slip=0.,gamma=0.,mz=0.;
        for(const auto& p:writer_->wrenches(w))if(p.valid) {
            normal+=p.normal_load_n;fx+=p.fx_n;fy+=p.fy_n;mz+=p.mz_nm;
            slip+=p.slip_ratio*p.normal_load_n;gamma+=p.camber_rad*p.normal_load_n;
        }
        auto& d=diagnostics_[w];d.normal_load_n=normal;d.support_n=0.;
        for(const auto& p:s.patches)d.support_n+=p.normal_load_n*p.normal[2];
        d.tangent_force_n=fx;d.slip_ratio=normal>0.?slip/normal:0.;
        d.patch_count=static_cast<int>(writer_->model().patches(w).size());
        d.active_station_count=static_cast<int>(s.patches.size());d.airborne=normal<=0.;
        d.pressure_bar=writer_->model().spec().carcass[w].pressure_pa/100000.;
        d.radial_energy_j=writer_->state().carcass[w].stored_energy_j;
        d.shear_energy_j=writer_->state().brush[w].elastic_energy_j;
        d.spatial_lateral_force_n=fy;d.spatial_aligning_moment_nm=mz;
        d.spatial_camber_rad=normal>0.?gamma/normal:0.;
        d.spatial_generation=writer_->state().accepted_interval;
    }
    committed_=true;
}
CoreStatus SpatialTireAdapter::try_compute_into(const mjData* data,double dt,std::span<double> out) noexcept {
    if(!data || out.size()!=static_cast<std::size_t>(model_->nv))return CoreStatus::invalid_input;
    const auto motion=capture_motion(*data);if(motion!=CoreStatus::ok)return motion;
    double old_brush=0.,old_radial=0.;
    for(std::size_t w=0;w<2;++w){old_brush+=writer_->state().brush[w].sliding_loss_j+writer_->state().brush[w].convective_release_j;
        old_radial+=writer_->state().carcass[w].dissipated_j;}
    std::ranges::fill(out,0.);
    // Every fallible geometric/engine projection is a probe. Only after it
    // succeeds may the material owner commit; publication then copies into
    // pre-reserved fixed-capacity snapshot storage without another query.
    const auto probe=writer_->try_compute_into(data,dt,false,out);
    if(probe!=CoreStatus::ok)return probe;
    const auto projected=project(*data);
    if(projected!=CoreStatus::ok)return projected;
    std::ranges::fill(out,0.);
    const auto status=writer_->try_compute_into(data,dt,true,out);
    if(status!=CoreStatus::ok)return status;
    double brush=0.,radial=0.;
    for(std::size_t w=0;w<2;++w){brush+=writer_->state().brush[w].sliding_loss_j+writer_->state().brush[w].convective_release_j;
        radial+=writer_->state().carcass[w].dissipated_j;}
    brush_loss_=brush-old_brush;radial_power_=dt>0.?(radial-old_radial)/dt:0.;
    publish_probe(*data);return CoreStatus::ok;
}
void SpatialTireAdapter::compute_into(const mjData* d,double dt,std::span<double> out) {
    const auto status=try_compute_into(d,dt,out);
    if(status==CoreStatus::engine_failure)throw engine::EngineFailure("pneumatic_3d: engine force evaluation failed");
    if(status!=CoreStatus::ok)throw bike::pneumatic3d::SpatialTireEvaluationError(status,writer_->last_reason());
}
void SpatialTireAdapter::probe_compute_into(const mjData* d,double dt,std::span<double> out) {
    const auto motion=d?capture_motion(*d):CoreStatus::invalid_input;
    if(motion==CoreStatus::engine_failure)throw engine::EngineFailure("pneumatic_3d: wheel motion query failed");
    if(motion!=CoreStatus::ok)throw bike::pneumatic3d::SpatialTireEvaluationError(motion,bike::pneumatic3d::ReasonCode3d::nonfinite_geometry);
    std::ranges::fill(out,0.);
    const auto status=writer_->try_compute_into(d,dt,false,out);
    if(status==CoreStatus::engine_failure)throw engine::EngineFailure("pneumatic_3d: engine force probe failed");
    if(status!=CoreStatus::ok)throw bike::pneumatic3d::SpatialTireEvaluationError(status,writer_->last_reason());
    if(project(*d)!=CoreStatus::ok)throw bike::pneumatic3d::SpatialTireEvaluationError(CoreStatus::invalid_geometry,bike::pneumatic3d::ReasonCode3d::terrain_query_failed);
}
std::vector<double> SpatialTireAdapter::qfrc(const mjData* d,double dt) {
    std::vector<double> out(static_cast<std::size_t>(model_->nv));compute_into(d,dt,out);return out;
}
std::vector<double> SpatialTireAdapter::probe_qfrc(const mjData* d,double dt) {
    std::vector<double> out(static_cast<std::size_t>(model_->nv));probe_compute_into(d,dt,out);return out;
}
std::array<std::optional<TireSnapshot>,2> SpatialTireAdapter::snapshots() const {
    std::array<std::optional<TireSnapshot>,2> out{};if(committed_)for(std::size_t w=0;w<2;++w)out[w]=snapshots_[w];return out;
}
std::array<std::optional<TireSnapshot>,2> SpatialTireAdapter::probe_snapshots() const {
    std::array<std::optional<TireSnapshot>,2> out{};if(probed_)for(std::size_t w=0;w<2;++w)out[w]=probes_[w];return out;
}
std::array<std::optional<TireDiagnostics>,2> SpatialTireAdapter::diagnostics() const {
    std::array<std::optional<TireDiagnostics>,2> out{};if(committed_)for(std::size_t w=0;w<2;++w)out[w]=diagnostics_[w];return out;
}
[[noreturn]] void SpatialTireAdapter::set_state(std::span<const std::string>,std::span<const double>) {
    throw std::invalid_argument("pneumatic_3d.state: use the versioned BKS3 codec");
}
[[noreturn]] void SpatialTireAdapter::prepare_state(std::span<const std::string>,std::span<const double>) {
    throw std::invalid_argument("pneumatic_3d.state: planar column rows cannot restore spatial material");
}
[[noreturn]] std::vector<double> SpatialTireAdapter::state() const {
    throw std::invalid_argument("pneumatic_3d.state: use state_bytes, not planar columns");
}
void SpatialTireAdapter::invalidate_projection() noexcept {
    committed_=probed_=false;for(auto& s:snapshots_)s.patches.clear();for(auto& s:probes_)s.patches.clear();
    clear_settled_loss();
}
void SpatialTireAdapter::reset() {
    if(writer_->restore(initial_)!=CoreStatus::ok)throw std::logic_error("pneumatic_3d: invalid initial material state");
    invalidate_projection();
}
void SpatialTireAdapter::restart_clock() {
    // Cold transition from settled material to scored t=0. Keep the contact
    // snapshots needed by initialization while rebasing accepted intervals.
    auto material=writer_->state();material.accepted_interval=-1;
    for(auto& carcass:material.carcass)carcass.dissipated_j=0.;
    for(auto& brush:material.brush){brush.sliding_loss_j=0.;brush.convective_release_j=0.;}
    if(writer_->restore(material)!=CoreStatus::ok)throw std::logic_error("pneumatic_3d: clock rebase rejected");
    for(auto* slots:{&snapshots_,&probes_})for(auto& s:*slots){s.time_s=0.;s.interval_id=-1;}
    for(auto& diagnostic:diagnostics_)diagnostic.spatial_generation=-1;
    clear_settled_loss();
}
