#include "context.hpp"
#include "refine.hpp"
#include "../engine_call.hpp"
#include "../runtime/step.hpp"
#include "../spatial/frames.hpp"
#include "../stepper.hpp"
#include "../tyre/profile.hpp"
#include "../writers/drivetrain.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <optional>
#include <ranges>
#include <stdexcept>
#include <utility>

namespace bike::initialization {
namespace {

struct SurfacePoint3d {
    double height_m = 0.;
    std::array<double, 3> normal{0., 0., 1.};
};

struct XY {
    double x = 0.;
    double y = 0.;
};

class Heightfield3dSampler {
public:
    Heightfield3dSampler(const mjModel* m, double ground_z_m) {
        const int hid = mj_name2id(m, mjOBJ_HFIELD, "road");
        const int tid = mj_name2id(m, mjOBJ_GEOM, "terrain");
        if (hid < 0 || tid < 0) return;
        const auto h_idx = static_cast<std::size_t>(hid);
        const auto t_idx = static_cast<std::size_t>(tid);
        const auto nrows = std::views::counted(m->hfield_nrow, m->nhfield);
        const auto ncols = std::views::counted(m->hfield_ncol, m->nhfield);
        const auto adrs = std::views::counted(m->hfield_adr, m->nhfield);
        const int nr = nrows[h_idx];
        const int nc = ncols[h_idx];
        const int adr = adrs[h_idx];
        if (nr <= 2 || nc <= 2 || adr < 0) return;
        const auto total = static_cast<std::size_t>(nr) * static_cast<std::size_t>(nc);
        if (static_cast<std::size_t>(adr) + total > static_cast<std::size_t>(m->nhfielddata)) return;
        const auto hsize = std::views::counted(m->hfield_size, 4 * m->nhfield);
        const auto gpos = std::views::counted(m->geom_pos, 3 * m->ngeom);
        const double rx = hsize[4U * h_idx + 0U];
        const double ry = hsize[4U * h_idx + 1U];
        const double elev = hsize[4U * h_idx + 2U];
        if (!(rx > 0.) || !(ry > 0.) || !(elev > 0.)) return;
        valid_ = true;
        nrow_ = nr;
        ncol_ = nc;
        adr_ = static_cast<std::size_t>(adr);
        hfield_data_ = std::views::counted(m->hfield_data, m->nhfielddata);
        x_min_ = gpos[3U * t_idx + 0U] - rx;
        y_min_ = gpos[3U * t_idx + 1U] - ry;
        dx_ = (2. * rx) / static_cast<double>(nc - 1);
        dy_ = (2. * ry) / static_cast<double>(nr - 1);
        elev_ = elev;
        datum_z_ = ground_z_m - gpos[3U * t_idx + 2U];
    }

    [[nodiscard]] bool valid() const noexcept { return valid_; }

    [[nodiscard]] SurfacePoint3d sample(XY pt) const noexcept {
        const double fx = std::clamp((pt.x - x_min_) / dx_, 0., static_cast<double>(ncol_ - 1));
        const double fy = std::clamp((pt.y - y_min_) / dy_, 0., static_cast<double>(nrow_ - 1));
        const int ix0 = std::min(static_cast<int>(std::floor(fx)), ncol_ - 2);
        const int iy0 = std::min(static_cast<int>(std::floor(fy)), nrow_ - 2);
        const double tx = fx - static_cast<double>(ix0);
        const double ty = fy - static_cast<double>(iy0);
        const auto z_at = [&](int ix, int iy) noexcept {
            const std::size_t idx =
                adr_ + static_cast<std::size_t>(iy) * static_cast<std::size_t>(ncol_) +
                static_cast<std::size_t>(ix);
            return static_cast<double>(hfield_data_[idx]) * elev_ - datum_z_;
        };
        const double z00 = z_at(ix0, iy0);
        const double z10 = z_at(ix0 + 1, iy0);
        const double z01 = z_at(ix0, iy0 + 1);
        const double z11 = z_at(ix0 + 1, iy0 + 1);
        const double h = (1. - tx) * (1. - ty) * z00 + tx * (1. - ty) * z10 +
                         (1. - tx) * ty * z01 + tx * ty * z11;
        const double dz_dx = ((1. - ty) * (z10 - z00) + ty * (z11 - z01)) / dx_;
        const double dz_dy = ((1. - tx) * (z01 - z00) + tx * (z11 - z10)) / dy_;
        const double inv = 1. / std::sqrt(1. + dz_dx * dz_dx + dz_dy * dz_dy);
        return {.height_m = h, .normal = {-dz_dx * inv, -dz_dy * inv, inv}};
    }

private:
    bool valid_ = false;
    int nrow_ = 0;
    int ncol_ = 0;
    std::size_t adr_ = 0;
    std::span<const float> hfield_data_{};
    double x_min_ = 0.;
    double y_min_ = 0.;
    double dx_ = 1.;
    double dy_ = 1.;
    double elev_ = 1.;
    double datum_z_ = 0.;
};

void initialize_pose_spatial(SetupContext& context, const FreeJointAddress& free_root) {
    auto& stepper = context.stepper;
    mjModel* m = stepper.model();
    mjData* d = stepper.data();
    if (!std::isfinite(context.start_x_m) || !std::isfinite(context.ground_z_m))
        throw std::invalid_argument("initial pose requires finite source geometry");
    for (int const id : {context.front_geom_id, context.rear_geom_id})
        if (id < 0 || id >= m->ngeom)
            throw std::invalid_argument("initial pose wheel geometry id is invalid");
    const int rider_free_id = mj_name2id(m, mjOBJ_JOINT, "rider_root_free");
    if (context.physical.rider_present()) {
        if (rider_free_id < 0 ||
            std::views::counted(m->jnt_type, m->njnt)[static_cast<std::size_t>(rider_free_id)] != mjJNT_FREE)
            throw std::invalid_argument(
                "spatial 3D initial_pose requires rider_root_free freejoint for articulated_3d");
        if (!context.rider_pose || context.frame_body_id < 0 ||
            context.frame_body_id >= m->nbody)
            throw std::invalid_argument(
                "initial rider pose and frame identity are required");
    }

    const auto vertices = context.physical.terrain_vertices();
    const bool has_vertices = vertices.size() >= 2;
    if (has_vertices) {
        for (std::size_t i = 0; i < vertices.size(); ++i)
            if (!std::isfinite(vertices[i][0]) || !std::isfinite(vertices[i][1]) ||
                (i > 0 && vertices[i][0] <= vertices[i - 1][0]))
                throw std::invalid_argument(
                    "initial pose road vertices must be finite and strictly increasing");
    }

    const Heightfield3dSampler hfield(m, context.ground_z_m);
    if (!has_vertices && !hfield.valid())
        throw std::invalid_argument("spatial initial pose requires a compiled 3D heightfield or road vertices");

    const auto locate_vertex = [&](double x) -> std::size_t {
        const auto it = std::ranges::lower_bound(vertices, x, {}, [](const auto& a) { return a[0]; });
        const auto dist = std::ranges::distance(vertices.begin(), it);
        return static_cast<std::size_t>(
            std::clamp<std::ptrdiff_t>(dist - 1, 0, static_cast<std::ptrdiff_t>(vertices.size()) - 2));
    };
    const auto vertex_height = [&](double x) -> double {
        if (x <= vertices.front()[0]) return vertices.front()[1];
        if (x >= vertices.back()[0]) return vertices.back()[1];
        const auto at = locate_vertex(x);
        return biketyre::np_interp(
            x, std::array{vertices[at][0], vertices[at + 1][0]},
            std::array{vertices[at][1], vertices[at + 1][1]});
    };
    const auto vertex_slope = [&](double x) -> double {
        const auto at = locate_vertex(x);
        return (vertices[at + 1][1] - vertices[at][1]) / (vertices[at + 1][0] - vertices[at][0]);
    };

    const double z0 = has_vertices
        ? vertex_height(context.start_x_m)
        : hfield.sample({.x = context.start_x_m, .y = 0.}).height_m;
    const double sx0 = [&] {
        if (has_vertices) return vertex_slope(context.start_x_m);
        const auto s = hfield.sample({.x = context.start_x_m, .y = 0.});
        return -s.normal[0] / std::max(0.2, s.normal[2]);
    }();
    const double theta0 = -std::atan(sx0);
    const bike::spatial::Quat q_init = bike::spatial::quat_from_euler_zyx(0., theta0, 0.);

    const JointAddress steer = joint_address(context, "steer_joint");
    const JointAddress crank = joint_address(context, "crank_spin");
    const JointAddress pedal_front = joint_address(context, "pedal_front_spin");
    const JointAddress pedal_rear = joint_address(context, "pedal_rear_spin");

    auto stage = std::make_unique<runtime::StepState>(context.physical.state());
    stage->initializing = true;
    stage->contact_query = {};
    stage->probe_query = {};
    context.physical.restore(*stage);
    stepper.mutate([&] { engine::reset_data(m, d); });

    const auto fq = static_cast<std::size_t>(free_root.qpos);
    auto const q = std::views::counted(d->qpos, m->nq);
    q[fq + 0U] = context.start_x_m;
    q[fq + 1U] = 0.;
    q[fq + 2U] = z0 - context.ground_z_m + .005;
    for (std::size_t k = 0; k < 4; ++k) q[fq + 3U + k] = q_init[k];
    q[static_cast<std::size_t>(steer.qpos)] = 0.;
    const double phase = context.physics.drive.crank_phase_rad;
    q[static_cast<std::size_t>(crank.qpos)] = phase;
    q[static_cast<std::size_t>(pedal_front.qpos)] = -phase;
    q[static_cast<std::size_t>(pedal_rear.qpos)] = -phase;

    stepper.note_posvel_stale();
    stepper.forward();

    double clearance = 0.;
    const auto centers = std::views::counted(d->geom_xpos, 3 * m->ngeom);
    const auto sizes = std::views::counted(m->geom_size, 3 * m->ngeom);
    const auto bodies = std::views::counted(m->geom_bodyid, m->ngeom);
    const auto xmat = std::views::counted(d->xmat, 9 * m->nbody);
    for (int const gid : {context.front_geom_id, context.rear_geom_id}) {
        const auto g_idx = static_cast<std::size_t>(gid);
        const auto b_idx = static_cast<std::size_t>(bodies[g_idx]);
        const double cx = centers[3U * g_idx + 0U];
        const double cy = centers[3U * g_idx + 1U];
        const double cz = centers[3U * g_idx + 2U];
        const std::array<double, 3> ax_raw{
            xmat[9U * b_idx + 1U], xmat[9U * b_idx + 4U], xmat[9U * b_idx + 7U]};
        const double ax_len = std::sqrt(ax_raw[0] * ax_raw[0] + ax_raw[1] * ax_raw[1] + ax_raw[2] * ax_raw[2]);
        const std::array<double, 3> a_s = ax_len > 1e-12
            ? std::array<double, 3>{ax_raw[0] / ax_len, ax_raw[1] / ax_len, ax_raw[2] / ax_len}
            : std::array<double, 3>{0., 1., 0.};

        SurfacePoint3d surf{};
        if (hfield.valid()) {
            surf = hfield.sample({.x = cx, .y = cy});
            if (has_vertices && std::abs(cy) < 1e-9) surf.height_m = vertex_height(cx);
        } else {
            const double slope = vertex_slope(cx);
            const double inv = 1. / std::sqrt(1. + slope * slope);
            surf.height_m = vertex_height(cx);
            surf.normal = {-slope * inv, 0., inv};
        }

        const double r_outer = sizes[3U * g_idx];
        const double r_crown =
            (std::abs(r_outer - 0.379) < 0.004 || std::abs(r_outer - 0.360) < 0.004) ? 0.0355 : 0.0300;
        const double r_major = r_outer - r_crown;
        const double a_dot_n = a_s[0] * surf.normal[0] + a_s[1] * surf.normal[1] + a_s[2] * surf.normal[2];
        const double cos_gamma = std::sqrt(std::max(0., 1. - a_dot_n * a_dot_n));
        const double dz = surf.height_m +
                          (r_major * cos_gamma + r_crown) / std::max(0.2, surf.normal[2]) -
                          cz + .005;
        clearance = std::max(clearance, dz);
    }
    q[fq + 2U] += clearance;
    stepper.note_posvel_stale();
    stepper.forward();
    if (context.physical.rider_present()) {
        if (!context.rider_pose)
            throw std::invalid_argument("initial rider pose is required");
        const auto xp = std::views::counted(d->xpos, 3 * m->nbody);
        const auto xm = std::views::counted(d->xmat, 9 * m->nbody);
        const auto& hip = context.rider_pose->hip;
        const auto frame = static_cast<std::size_t>(context.frame_body_id);
        std::array<double, 3> wanted{};
        for (std::size_t axis = 0; axis < 3U; ++axis) {
            wanted[axis] = xp[3U * frame + axis];
            for (std::size_t a = 0; a < 3U; ++a)
                wanted[axis] += xm[9U * frame + 3U * axis + a] * hip[a];
        }
        const auto rfq = static_cast<std::size_t>(
            std::views::counted(m->jnt_qposadr, m->njnt)[static_cast<std::size_t>(rider_free_id)]);
        for (std::size_t axis = 0; axis < 3U; ++axis) q[rfq + axis] = wanted[axis];
        for (std::size_t k = 0; k < 4U; ++k) q[rfq + 3U + k] = q_init[k];
        stepper.note_posvel_stale();
        stepper.forward();
    }
    stepper.mutate([&] {
        if (context.physical.rider_present()) {
            auto controller = context.physical.rider_control().state();
            controller.enabled = true;
            context.physical.rider_control().restore(controller);
            context.physical.rider_control().initialize(d);
            if (stepper.has_rider_contacts()) stepper.rider_contacts_reset();
        }
        stepper.drive().reset();
        if (stepper.has_tire()) stepper.tire_writer().reset();
    });
    stepper.note_posvel_stale();
    stepper.forward();
}

} // namespace

void require_initializing(const SetupContext& context) {
    if (!context.initializing || context.physical.step() != 0)
        throw std::invalid_argument("equilibrium operations are confined to initialization");
}
JointAddress joint_address(const SetupContext& context, std::string_view name) {
    const mjModel* m = context.stepper.model();
    const int id = mj_name2id(m, mjOBJ_JOINT, std::string(name).c_str());
    if (id < 0)
        throw std::invalid_argument("model has no joint '" + std::string(name) + "'; equilibrium requires a planar root");
    const auto types=std::views::counted(m->jnt_type,m->njnt);
    const auto index=static_cast<std::size_t>(id);
    if(types[index]!=mjJNT_SLIDE && types[index]!=mjJNT_HINGE)
        throw std::invalid_argument("equilibrium requires scalar planar joint '"+std::string(name)+"'");
    return {.joint=id,.qpos=std::views::counted(m->jnt_qposadr,m->njnt)[index],
               .dof=std::views::counted(m->jnt_dofadr,m->njnt)[index]};
}

void initialize_pose(SetupContext& context) {
    require_initializing(context);
    auto& stepper=context.stepper;
    mjModel* m=stepper.model();mjData* d=stepper.data();
    if (const auto free_root = find_free_root(m)) {
        initialize_pose_spatial(context, *free_root);
        return;
    }
    const auto vertices=context.physical.terrain_vertices();
    if(vertices.size()<2 || !std::isfinite(context.start_x_m) || !std::isfinite(context.ground_z_m))
        throw std::invalid_argument("initial pose requires finite source geometry and a compiled road");
    for(std::size_t i=0;i<vertices.size();++i)
        if(!std::isfinite(vertices[i][0])||!std::isfinite(vertices[i][1])||(i>0&&vertices[i][0]<=vertices[i-1][0]))
            throw std::invalid_argument("initial pose road vertices must be finite and strictly increasing");
    for(int const id:{context.front_geom_id,context.rear_geom_id})
        if(id<0||id>=m->ngeom)throw std::invalid_argument("initial pose wheel geometry id is invalid");
    const JointAddress x=joint_address(context,"root_x"),z=joint_address(context,"root_z"),pitch=joint_address(context,"root_pitch");
    const JointAddress crank=joint_address(context,"crank_spin"),front=joint_address(context,"pedal_front_spin"),rear=joint_address(context,"pedal_rear_spin");
    auto locate=[&](double position){
        const auto it=std::ranges::lower_bound(vertices,position,{},[](const auto& a){return a[0];});
        const auto distance=std::ranges::distance(vertices.begin(),it);
        return static_cast<std::size_t>(std::clamp<std::ptrdiff_t>(distance-1,0,static_cast<std::ptrdiff_t>(vertices.size())-2));
    };
    auto const height=[&](double position){
        if(position<=vertices.front()[0])return vertices.front()[1];
        if(position>=vertices.back()[0])return vertices.back()[1];
        const auto at=locate(position);
        return biketyre::np_interp(position,std::array{vertices[at][0],vertices[at+1][0]},std::array{vertices[at][1],vertices[at+1][1]});
    };
    const auto i=locate(context.start_x_m);
    const double slope=(vertices[i+1][1]-vertices[i][1])/(vertices[i+1][0]-vertices[i][0]);
    auto stage=std::make_unique<runtime::StepState>(context.physical.state());
    stage->initializing=true;stage->contact_query={};stage->probe_query={};
    context.physical.restore(*stage);
    stepper.mutate([&]{engine::reset_data(m,d);});
    auto const q=std::views::counted(d->qpos,m->nq);
    q[static_cast<std::size_t>(x.qpos)]=context.start_x_m;
    q[static_cast<std::size_t>(z.qpos)]=height(context.start_x_m)-context.ground_z_m+.005;
    q[static_cast<std::size_t>(pitch.qpos)]=-std::atan(slope);
    stepper.note_posvel_stale();stepper.forward();
    double clearance=0.;
    const auto centers=std::views::counted(d->geom_xpos,3*m->ngeom);
    const auto sizes=std::views::counted(m->geom_size,3*m->ngeom);
    for(int const id:{context.front_geom_id,context.rear_geom_id}) {
        const auto offset=3*static_cast<std::size_t>(id);
        clearance=std::max(clearance,height(centers[offset])+sizes[offset]*std::sqrt(1.+slope*slope)-centers[offset+2]+.005);
    }
    q[static_cast<std::size_t>(z.qpos)]+=clearance;
    const double phase=context.physics.drive.crank_phase_rad;
    q[static_cast<std::size_t>(crank.qpos)]=phase;
    q[static_cast<std::size_t>(front.qpos)]=-phase;
    q[static_cast<std::size_t>(rear.qpos)]=-phase;
    if(context.physical.rider_present()) {
        if(!context.rider_pose||context.frame_body_id<0||context.frame_body_id>=m->nbody)
            throw std::invalid_argument("initial rider pose and frame identity are required");
        stepper.note_posvel_stale();stepper.forward();
        const auto xp=std::views::counted(d->xpos,3*m->nbody);
        const auto xm=std::views::counted(d->xmat,9*m->nbody);
        const auto& hip=context.rider_pose->hip;
        const auto frame=static_cast<std::size_t>(context.frame_body_id);
        std::array<double,3> wanted{};
        for(std::size_t axis=0;axis<3;++axis){wanted[axis]=xp[3*frame+axis];for(std::size_t a=0;a<3;++a)wanted[axis]+=xm[9*frame+3*axis+a]*hip[a];}
        q[static_cast<std::size_t>(joint_address(context,"rider_root_x").qpos)]=wanted[0]-hip[0];
        q[static_cast<std::size_t>(joint_address(context,"rider_root_z").qpos)]=wanted[2]-hip[2];
        q[static_cast<std::size_t>(joint_address(context,"rider_root_pitch").qpos)]=q[static_cast<std::size_t>(pitch.qpos)];
    }
    stepper.note_posvel_stale();stepper.forward();
    stepper.mutate([&]{
        if(context.physical.rider_present()) {
            auto controller=context.physical.rider_control().state();controller.enabled=true;
            context.physical.rider_control().restore(controller);
            context.physical.rider_control().initialize(d);
            if(stepper.has_rider_contacts())stepper.rider_contacts_reset();
        }
        stepper.drive().reset();
        if(stepper.has_tire())stepper.tire_writer().reset();
    });
    stepper.note_posvel_stale();
}
} // namespace bike::initialization
