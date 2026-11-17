#include "context.hpp"
#include "state_file.hpp"
#include "../model/mapping.hpp"
#include "../numeric_sum.hpp"
#include "../runtime/config.hpp"
#include "../runtime/step.hpp"
#include "../spatial/types.hpp"
#include "../stepper.hpp"
#include "../writers/drivetrain.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <ranges>
#include <stdexcept>

namespace bike::initialization {
namespace {

double dot3(const std::array<double, 3>& a, const std::array<double, 3>& b) noexcept {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

std::array<double, 3> cross3(const std::array<double, 3>& a,
                             const std::array<double, 3>& b) noexcept {
    return {a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]};
}

struct FallbackDir {
    std::array<double, 3> value{};
};

std::array<double, 3> normalize3(const std::array<double, 3>& v,
                                 FallbackDir fallback) noexcept {
    const double len = std::sqrt(dot3(v, v));
    if (len > 1e-12) return {v[0] / len, v[1] / len, v[2] / len};
    return fallback.value;
}

struct BodyColumn {
    int body_id{};
    std::size_t col{};
};

std::array<double, 3> body_xmat_col(std::span<const double> xmat, BodyColumn index) noexcept {
    const std::size_t off = static_cast<std::size_t>(index.body_id) * 9U;
    return {xmat[off + index.col], xmat[off + 3U + index.col], xmat[off + 6U + index.col]};
}

struct InitialCogQuery {
    double rear_omega_rad_s{};
    double speed_mps{};
};

int select_initial_cog(const drivetrain::DriveConfig& drive, InitialCogQuery query) {
    if (!drive.policies.shifting.enabled || drive.policies.shifting.cassette.empty() || !(query.speed_mps > 0.))
        return drive.policies.gearing.rear_teeth;
    const double front = static_cast<double>(drive.policies.gearing.front_teeth);
    const double min_rpm = drive.policies.shifting.target_cadence_min_rpm;
    const double max_rpm = drive.policies.shifting.target_cadence_max_rpm;
    const double target_rpm = 0.5 * (min_rpm + max_rpm);
    int best_teeth = drive.policies.gearing.rear_teeth;
    bool best_in_band = false;
    double best_dist = std::numeric_limits<double>::infinity();
    for (const int teeth : drive.policies.shifting.cassette) {
        const double ratio = front / static_cast<double>(teeth);
        const double cadence_rpm = std::abs(query.rear_omega_rad_s / ratio) * 60. / (2. * std::numbers::pi);
        const bool in_band = cadence_rpm >= min_rpm && cadence_rpm <= max_rpm;
        const double dist = std::abs(cadence_rpm - target_rpm);
        if ((in_band && !best_in_band) || (in_band == best_in_band && dist < best_dist)) {
            best_in_band = in_band;
            best_dist = dist;
            best_teeth = teeth;
        }
    }
    return best_teeth;
}

void initial_speed_spatial(SetupContext& context,
                           const runtime::WheelSnapshots& snapshots,
                           const std::optional<bike::spatial::SpatialConfig>& spatial,
                           int root_free_id) {
    mjModel const* m = context.stepper.model();
    mjData* d = context.stepper.data();
    const auto jtypes = std::views::counted(m->jnt_type, m->njnt);
    if (jtypes[static_cast<std::size_t>(root_free_id)] != mjJNT_FREE)
        throw std::invalid_argument("root_free must be a freejoint");
    if (spatial.has_value() && spatial->start_mode == bike::spatial::SpatialStartMode::moving &&
        (!std::isfinite(context.physics.initial_speed_mps) || context.physics.initial_speed_mps < 2.5)) {
        throw std::invalid_argument("spatial moving start requires initial_speed_mps >= 2.5");
    }
    if (context.frame_body_id < 0 || context.frame_body_id >= m->nbody)
        throw std::invalid_argument("initial speed requires a valid frame body");

    auto const velocity = std::views::counted(d->qvel, m->nv);
    const auto free_dof = static_cast<std::size_t>(
        std::views::counted(m->jnt_dofadr, m->njnt)[static_cast<std::size_t>(root_free_id)]);

    std::array<double, 3> normal_sum{};
    for (const auto& snapshot : snapshots) {
        if (!snapshot) continue;
        for (const auto& p : snapshot->patches) {
            if (p.working_surface) {
                normal_sum[0] += p.normal_load_n * p.normal[0];
                normal_sum[1] += p.normal_load_n * p.normal[1];
                normal_sum[2] += p.normal_load_n * p.normal[2];
            }
        }
    }
    const std::array<double, 3> n_hat = normalize3(normal_sum, {{0., 0., 1.}});
    const auto xmat = std::views::counted(d->xmat, 9 * m->nbody);
    const std::array<double, 3> fwd = normalize3(
        body_xmat_col(xmat, {.body_id = context.frame_body_id, .col = 0}), {{1., 0., 0.}});
    const double fn = dot3(fwd, n_hat);
    const std::array<double, 3> fwd_tangent = normalize3(
        {fwd[0] - fn * n_hat[0], fwd[1] - fn * n_hat[1], fwd[2] - fn * n_hat[2]},
        {{1., 0., 0.}});
    const std::array<double, 3> y_frame_axis = normalize3(
        body_xmat_col(xmat, {.body_id = context.frame_body_id, .col = 1}), {{0., 1., 0.}});
    const std::array<double, 3> roll_tan = cross3(y_frame_axis, n_hat);
    const std::array<double, 3> t_tangent = dot3(roll_tan, fwd) > 1e-8
        ? normalize3(roll_tan, {fwd_tangent})
        : fwd_tangent;

    const double v0 = context.physics.initial_speed_mps;
    for (std::size_t k = 0; k < 3; ++k) {
        velocity[free_dof + k] = v0 * t_tangent[k];
        velocity[free_dof + 3U + k] = 0.;
    }
    const int rider_free_id = mj_name2id(m, mjOBJ_JOINT, "rider_root_free");
    if (rider_free_id >= 0 && jtypes[static_cast<std::size_t>(rider_free_id)] == mjJNT_FREE) {
        const auto r_dof = static_cast<std::size_t>(
            std::views::counted(m->jnt_dofadr, m->njnt)[static_cast<std::size_t>(rider_free_id)]);
        for (std::size_t k = 0; k < 3; ++k) {
            velocity[r_dof + k] = v0 * t_tangent[k];
            velocity[r_dof + 3U + k] = 0.;
        }
    }
    context.stepper.note_posvel_stale();
    context.stepper.forward();

    const auto bodies = std::views::counted(m->geom_bodyid, m->ngeom);
    const auto centers = std::views::counted(d->geom_xpos, 3 * m->ngeom);
    const auto sizes = std::views::counted(m->geom_size, 3 * m->ngeom);
    const auto xmat_after = std::views::counted(d->xmat, 9 * m->nbody);
    for (std::size_t side = 0; side < 2; ++side) {
        const auto& snap = snapshots[side];
        if (!snap) throw std::invalid_argument("initial speed requires both physical tire snapshots");
        const auto geom = static_cast<std::size_t>(side == 0 ? context.front_geom_id : context.rear_geom_id);
        const int body = bodies[geom];
        const std::array<double, 3> p_hub{centers[3 * geom], centers[3 * geom + 1], centers[3 * geom + 2]};
        const std::array<double, 3> a_w = normalize3(
            body_xmat_col(xmat_after, {.body_id = body, .col = 1}), {{0., 1., 0.}});

        std::array<double, 3> nw_sum{};
        std::array<double, 3> pc_sum{};
        double weight_sum = 0.;
        for (const auto& p : snap->patches) {
            const double w = p.normal_load_n > 0. ? p.normal_load_n : 1.;
            for (std::size_t k = 0; k < 3; ++k) {
                nw_sum[k] += w * p.normal[k];
                pc_sum[k] += w * p.point_m[k];
            }
            weight_sum += w;
        }
        const std::array<double, 3> n_w = weight_sum > 0.
            ? normalize3(nw_sum, {n_hat}) : n_hat;
        const std::array<double, 3> d_w = normalize3(cross3(a_w, n_w), {t_tangent});
        const double radius = snap->effective_radius_m > 0. ? snap->effective_radius_m : sizes[3 * geom];
        const std::array<double, 3> p_c = weight_sum > 0.
            ? std::array<double, 3>{pc_sum[0] / weight_sum, pc_sum[1] / weight_sum, pc_sum[2] / weight_sum}
            : std::array<double, 3>{p_hub[0] - radius * n_w[0], p_hub[1] - radius * n_w[1], p_hub[2] - radius * n_w[2]};
        const std::array<double, 3> r_vec{p_c[0] - p_hub[0], p_c[1] - p_hub[1], p_c[2] - p_hub[2]};
        const auto v_c = model::point_velocity(*m, *d, body, p_c);
        const double lever = dot3(cross3(a_w, r_vec), d_w);
        const auto dof = static_cast<std::size_t>(
            joint_address(context, side == 0 ? "front_wheel_spin" : "rear_wheel_spin").dof);
        if (std::abs(lever) > 1e-9) {
            velocity[dof] -= dot3(v_c, d_w) / lever;
        } else {
            const auto v_hub = model::point_velocity(*m, *d, body, p_hub);
            const auto omega = model::body_angular_velocity(*m, *d, body);
            const double roll = dot3(v_hub, d_w);
            const double parent_omega = dot3(omega, a_w) - velocity[dof];
            velocity[dof] = roll / radius - parent_omega;
        }
    }

    const auto& drive_cfg = context.physics.drive;
    const double rear_omega = velocity[static_cast<std::size_t>(joint_address(context, "rear_wheel_spin").dof)];
    int rear_teeth = drive_cfg.gearing.rear_teeth;
    if (context.stepper.has_drive()) {
        const auto& writer_cfg = context.stepper.drive().config();
        rear_teeth = select_initial_cog(writer_cfg, {.rear_omega_rad_s = rear_omega, .speed_mps = v0});
        if (rear_teeth != writer_cfg.policies.gearing.rear_teeth) {
            auto snap = context.stepper.drive().state();
            const double new_ratio = static_cast<double>(writer_cfg.policies.gearing.front_teeth) /
                                     static_cast<double>(rear_teeth);
            snap.shifting.rear_teeth = rear_teeth;
            snap.shifting.from_teeth = rear_teeth;
            snap.shifting.shift_count = 0;
            snap.shifting.direction = "none";
            snap.shifting.cooldown_s = 0.;
            snap.shifting.cut_remaining_s = 0.;
            if (snap.ideal_hub.has_value()) {
                snap.ideal_hub->ratio = new_ratio;
                if (!snap.ideal_hub->coefficients.empty()) {
                    snap.ideal_hub->coefficients[0] = new_ratio;
                }
                if (snap.ideal_hub->boundary.has_value()) {
                    const char* driver_name = writer_cfg.motor_clutch ? "drive_shaft_spin" : "crank_spin";
                    const auto driver_qpos = static_cast<std::size_t>(joint_address(context, driver_name).qpos);
                    const auto driven_qpos = static_cast<std::size_t>(joint_address(context, "rear_wheel_spin").qpos);
                    const auto q = std::views::counted(d->qpos, m->nq);
                    const double b = new_ratio * q[driver_qpos] - q[driven_qpos];
                    snap.ideal_hub->boundary = b;
                    snap.ideal_hub->range = {snap.ideal_hub->range[0], b};
                }
            }
            context.stepper.mutate([&] { context.stepper.drive().restore(snap); });
            if (context.stepper.drive().take_model_dirty()) {
                context.stepper.note_posvel_stale();
                context.stepper.forward();
            }
        }
    }

    const double ratio = static_cast<double>(drive_cfg.gearing.front_teeth) / static_cast<double>(rear_teeth);
    const double rate = rear_omega / ratio;
    bool pedaling = (context.physics.drive_mode == "crank_effort" ||
                     context.physics.drive_mode == "articulated_effort") &&
                    drive_cfg.human_torque_nm > 0.;
    if (drive_cfg.pedaling.enabled &&
        std::abs(rate) * 60. / (2. * std::numbers::pi) >= drive_cfg.pedaling.coast_above_rpm) {
        pedaling = false;
    }
    if (pedaling) {
        velocity[static_cast<std::size_t>(joint_address(context, "crank_spin").dof)] = rate;
        if (mj_name2id(m, mjOBJ_JOINT, "drive_shaft_spin") >= 0) {
            velocity[static_cast<std::size_t>(joint_address(context, "drive_shaft_spin").dof)] = rate;
        }
        const auto xmat_frame = std::views::counted(d->xmat, 9 * m->nbody);
        const std::array<double, 3> y_frame = normalize3(
            body_xmat_col(xmat_frame, {.body_id = context.frame_body_id, .col = 1}), {{0., 1., 0.}});
        const double frame_rate = dot3(model::body_angular_velocity(*m, *d, context.frame_body_id), y_frame);
        velocity[static_cast<std::size_t>(joint_address(context, "pedal_front_spin").dof)] = -rate - frame_rate;
        velocity[static_cast<std::size_t>(joint_address(context, "pedal_rear_spin").dof)] = -rate - frame_rate;
        if (context.physical.rider_present()) {
            context.stepper.mutate([&] { context.physical.rider_control().initialize_velocity(d); });
        }
    }
    context.stepper.note_posvel_stale();
    context.stepper.forward();
}

void initial_speed(SetupContext& context, const runtime::WheelSnapshots& snapshots,
                   const std::optional<bike::spatial::SpatialConfig>& spatial) {
    mjModel const* m = context.stepper.model();
    const int root_free_id = mj_name2id(m, mjOBJ_JOINT, "root_free");
    if (root_free_id >= 0) {
        initial_speed_spatial(context, snapshots, spatial, root_free_id);
        return;
    }
    mjData* d = context.stepper.data();auto const velocity=std::views::counted(d->qvel,m->nv);
    std::array<double,3> tangent{};
    for(const auto& snapshot:snapshots)if(snapshot)for(const auto& p:snapshot->patches)if(p.working_surface){tangent[0]+=p.normal_load_n*p.normal[2];tangent[2]-=p.normal_load_n*p.normal[0];}
    const double length=std::sqrt(tangent[0]*tangent[0]+tangent[2]*tangent[2]);
    if(length>0.){tangent[0]/=length;tangent[2]/=length;}else tangent={1.,0.,0.};
    for(std::string_view const root:{"root","rider_root"}){
        if(root=="rider_root"&&!context.physical.rider_present())continue;
        velocity[static_cast<std::size_t>(joint_address(context,std::string(root)+"_x").dof)]=context.physics.initial_speed_mps*tangent[0];
        velocity[static_cast<std::size_t>(joint_address(context,std::string(root)+"_z").dof)]=context.physics.initial_speed_mps*tangent[2];
    }
    context.stepper.note_posvel_stale();context.stepper.forward();
    const auto bodies=std::views::counted(m->geom_bodyid,m->ngeom);
    const auto centers=std::views::counted(d->geom_xpos,3*m->ngeom);
    const auto sizes=std::views::counted(m->geom_size,3*m->ngeom);
    for(std::size_t side=0;side<2;++side){
        const auto& snap=snapshots[side];if(!snap)throw std::invalid_argument("initial speed requires both physical tire snapshots");
        const auto geom=static_cast<std::size_t>(side==0?context.front_geom_id:context.rear_geom_id);
        const int body=bodies[geom];
        const std::array<double,3> point{centers[3*geom],centers[3*geom+1],centers[3*geom+2]};
        const std::array<double,3> direction=snap->patches.empty()?std::array<double,3>{1.,0.,0.}:std::array<double,3>{snap->patches.front().normal[2],0.,-snap->patches.front().normal[0]};
        const double radius=snap->effective_radius_m>0.?snap->effective_radius_m:sizes[3*geom];
        const auto v=model::point_velocity(*m,*d,body,point);const auto omega=model::body_angular_velocity(*m,*d,body);
        const auto dof=static_cast<std::size_t>(joint_address(context,side==0?"front_wheel_spin":"rear_wheel_spin").dof);
        const double roll=v[0]*direction[0]+v[1]*direction[1]+v[2]*direction[2],parent_omega=omega[1]-velocity[dof];
        velocity[dof]=roll/radius-parent_omega;
    }
    const auto& drive=context.physics.drive;
    const double ratio=static_cast<double>(drive.gearing.front_teeth)/static_cast<double>(drive.gearing.rear_teeth);
    const double rate=velocity[static_cast<std::size_t>(joint_address(context,"rear_wheel_spin").dof)]/ratio;
    bool pedaling=(context.physics.drive_mode=="crank_effort"||context.physics.drive_mode=="articulated_effort")&&drive.human_torque_nm>0.;
    if(drive.pedaling.enabled&&std::abs(rate)*60./(2.*std::numbers::pi)>=drive.pedaling.coast_above_rpm)pedaling=false;
    if(pedaling){
        velocity[static_cast<std::size_t>(joint_address(context,"crank_spin").dof)]=rate;
        if(mj_name2id(m,mjOBJ_JOINT,"drive_shaft_spin")>=0)
            velocity[static_cast<std::size_t>(joint_address(context,"drive_shaft_spin").dof)]=rate;
        const double frame_rate=model::body_angular_velocity(*m,*d,context.frame_body_id)[1];
        velocity[static_cast<std::size_t>(joint_address(context,"pedal_front_spin").dof)]=-rate-frame_rate;
        velocity[static_cast<std::size_t>(joint_address(context,"pedal_rear_spin").dof)]=-rate-frame_rate;
        if(context.physical.rider_present())context.stepper.mutate([&]{context.physical.rider_control().initialize_velocity(d);});
    }
    context.stepper.note_posvel_stale();context.stepper.forward();
}
}
void finalize_initial_state(SetupContext& context,const runtime::RuntimeConfig& config){
    require_initializing(context);
    if(!context.equilibrium)throw std::invalid_argument("cannot finalize an incomplete equilibrium");
    const auto& spatial = context.physical.config_.spatial.has_value()
        ? context.physical.config_.spatial
        : config.spatial;
    if(mj_name2id(context.stepper.model(), mjOBJ_JOINT, "root_free") >= 0 &&
       spatial.has_value() &&
       spatial->start_mode == bike::spatial::SpatialStartMode::moving &&
       (!std::isfinite(context.physics.initial_speed_mps) ||
        context.physics.initial_speed_mps < 2.5)) {
        throw std::invalid_argument("spatial moving start requires initial_speed_mps >= 2.5");
    }
    auto& physical=context.physical;auto& stepper=context.stepper;mjData* d=stepper.data();mjModel* m=stepper.model();
    d->time=0.;physical.step_=0;physical.generation_=1;physical.initializing_=false;
    physical.control_clock_.reset();physical.contact_query_.restore({});physical.probe_query_.restore({});
    for(auto& filter:physical.filters_)filter.restore({});
    physical.balance_monitor_.restore({});physical.crash_detector_.set_event(std::nullopt);
    physical.rider_intent_.reset();
    // Launch part 2: the first running posture continues from the prepared lean.
    physical.rider_intent_.seed_lean(context.preparation.lean_rad);
    // Launch part 3b, decision 7: a standing start in an effort drive mode
    // holds the brakes from step 0; a moving start leaves the plan idle.
    if(context.physics.initial_speed_mps==0.&&(context.physics.drive_mode=="articulated_effort"||context.physics.drive_mode=="crank_effort"))
        (void)physical.rider_intent_.arm_standstill_start(context.preparation.lean_rad);
    physical.intent_signals_={};physical.applied_control_={};physical.held_rider_terms_.reset();
    physical.energy_={};physical.rollback_hold_=false;physical.research_accounting_valid_=false;physical.last_force_sample_.reset();physical.last_constraint_snapshot_={};physical.accumulator_.clear();
    physical.measurement_decimation_=config.measurement_decimation;
    if(stepper.has_cruise())stepper.cruise_reset();
    stepper.drive().restart_clock();
    if(stepper.has_tire())stepper.tire_writer().restart_clock();
    if(stepper.has_rider_contacts())stepper.rider_contacts_restart_clock();
    std::ranges::fill(std::views::counted(d->qfrc_applied,m->nv),0.);std::ranges::fill(std::views::counted(d->ctrl,m->nu),0.);
    stepper.note_posvel_stale();stepper.forward();apply_initial_forces(context,false);stepper.forward();
    if(stepper.has_tire()){
        std::vector<double> forces(static_cast<std::size_t>(m->nv));
        stepper.mutate([&]{stepper.tire_writer().compute_into(d,m->opt.timestep,forces);});
        stepper.tire_writer().clear_settled_loss();stepper.tire_writer().restart_clock();
    }
    if(stepper.has_rider_contacts())stepper.mutate([&]{stepper.rider_contacts_initialize_settled_state();});
    physical.contacts_=physical.refresh_contacts(false,std::nullopt,false);
    context.equilibrium->total_vertical_force_n=0.;
    for(std::size_t side=0;side<2;++side){
        const auto& snap=physical.snapshots_[side];if(!snap)throw std::invalid_argument("settled model lacks a wheel contact snapshot");
        numeric::PythonSum normal,vertical;for(const auto& p:snap->patches){normal.add(p.normal_load_n);vertical.add(p.native_world_force_n ? (p.native_world_force_n.value())[2] : p.normal_load_n*p.normal[2]-p.tangent_force_n*p.normal[0]);}// every addend is a float patch field or float product — exact under CPython's float branch
        if(side==0)context.equilibrium->static_front_load_n=normal.total();
        else context.equilibrium->static_rear_load_n=normal.total();
        context.equilibrium->total_vertical_force_n+=vertical.total();
    }
    initial_speed(context,physical.snapshots_,spatial);stepper.forward();
    if(context.initial_state)context.initial_state->restore(context);
    if(stepper.has_rider_contacts())stepper.mutate([&]{(void)stepper.rider_contacts_settle(std::nullopt,false,nullptr);});
    stepper.mutate([&]{
        auto [mass,elastic,total]=physical.energy_state();numeric::PythonSum sum;
        // sum(elastic.values()): stored_terms entries are Python floats —
        // the compensated float branch is the oracle's whole path.
        for(const auto& [name,value]:elastic){(void)name;sum.add(std::get<double>(value.value));}
        physical.energy_.initial_energy_j=total;physical.energy_.energy_scale_j=std::max(1.,mass.kinetic_energy_j+sum.total());
        physical.energy_.mechanical_energy_j=total;physical.energy_.elastic_energy_j=std::move(elastic);physical.energy_.residual_j=0.;
        physical.update_compiled_com_marker();
        if(context.physics.seated_climb.enabled){auto sensors=physical.sensor_channels(stepper.qvel(),stepper.drive().diagnostics(true).to_map());auto tires=physical.tire_channels(physical.snapshots_,stepper.qvel());sensors.emplace_back("tires",runtime::Wire{std::move(tires)});physical.update_intent_signals(sensors);}
        // The settled startup senses record: the settle latch above
        // already holds the grip reactions on the settled pose, so this
        // refresh carries the real hand pressure into the initial
        // capture (P2's startup acquisition point).
        physical.update_rider_senses(physical.settled_grip_entries(),physical.current_wheel_x_m(),physical.current_frame_up(),stepper.qvel());
    });
    physical.research_accounting_valid_=true;context.completed=true;
}
} // namespace bike::initialization
