#include "context.hpp"
#include "refine.hpp"
#include "../engine_call.hpp"
#include "../numerics/least_squares.hpp"
#include "../runtime/step.hpp"
#include "../spatial/frames.hpp"
#include "../stepper.hpp"
#include "../writers/compliant_3d_tire.hpp"
#include "../writers/drivetrain.hpp"
#include "../writers/rider_contacts.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <limits>
#include <memory>
#include <optional>
#include <ranges>
#include <stdexcept>
#include <utility>
#include <vector>

namespace bike::initialization {
namespace {

struct SpatialRefineArgs {
    FreeJointAddress free_root{};
    int max_evaluations = 40;
    double acceleration_tolerance = 0.05;
};

bike::spatial::Vec3 quat_log(const bike::spatial::Quat& q_in) noexcept {
    const double sign = q_in[0] < 0. ? -1. : 1.;
    const double w = sign * q_in[0];
    const double x = sign * q_in[1];
    const double y = sign * q_in[2];
    const double z = sign * q_in[3];
    const double v_norm = std::sqrt(x * x + y * y + z * z);
    const double factor = v_norm > 1e-12 ? (2. * std::atan2(v_norm, w) / v_norm) : 2.;
    return {factor * x, factor * y, factor * z};
}

double refine_equilibrium_spatial(SetupContext& context, const SpatialRefineArgs& args) {
    if (phase_a_hold_active(context))
        throw std::logic_error("Phase B refine_equilibrium requires Phase A hold to be inactive");
    auto& stepper = context.stepper;
    mjModel* m = stepper.model();
    mjData* d = stepper.data();
    const auto jnt_type = std::views::counted(m->jnt_type, m->njnt);
    const int rider_free_id = mj_name2id(m, mjOBJ_JOINT, "rider_root_free");
    const bool has_rider_free =
        rider_free_id >= 0 &&
        jnt_type[static_cast<std::size_t>(rider_free_id)] == mjJNT_FREE;
    const mjtSize expected_nq = m->nv + 1 + (has_rider_free ? 1 : 0);
    if (m->nq != expected_nq)
        throw std::invalid_argument(
            "spatial 3D equilibrium requires root_free (and optional rider_root_free)");
    const int rfd = has_rider_free
        ? std::views::counted(m->jnt_dofadr, m->njnt)[static_cast<std::size_t>(rider_free_id)]
        : -1;

    const auto nv = static_cast<std::size_t>(m->nv);
    const auto fq = static_cast<std::size_t>(args.free_root.qpos);
    const auto fd = static_cast<std::size_t>(args.free_root.dof);
    const JointAddress fw = joint_address(context, "front_wheel_spin");
    const JointAddress rw = joint_address(context, "rear_wheel_spin");
    const JointAddress steer = joint_address(context, "steer_joint");

    const std::vector<double> q0(stepper.qpos().begin(), stepper.qpos().end());
    const bike::spatial::Quat quat0{
        q0[fq + 3U], q0[fq + 4U], q0[fq + 5U], q0[fq + 6U]};
    const auto eul0 = bike::spatial::euler_zyx(quat0);
    if (!eul0.valid)
        throw std::runtime_error("spatial 3D equilibrium requires non-singular reference pitch");

    // NOLINTNEXTLINE(modernize-make-unique) make_unique's inlined forwarding path overflows the 8192-byte frame budget here
    const auto incoming_drive = std::unique_ptr<drivetrain::DriveSnapshot>{
        new drivetrain::DriveSnapshot(stepper.drive().state())};
    // NOLINTNEXTLINE(modernize-make-unique) same frame-budget constraint
    const auto incoming_step = std::unique_ptr<runtime::StepState>{
        new runtime::StepState(context.physical.state())};
    std::unique_ptr<writers::RiderContactsState> rider_state;
    std::optional<writers::RiderContactsProbe> rider_probe;
    if (stepper.has_rider_contacts()) {
        rider_state = std::make_unique<writers::RiderContactsState>(
            stepper.rider_contacts().state());
        rider_probe = stepper.rider_contacts().probe();
    }

    std::vector<std::size_t> internal_dofs;
    std::vector<double> scales{0.05, 0.10, 0.10};
    const auto dof_jnt = std::views::counted(m->dof_jntid, m->nv);
    for (std::size_t i = 0; i < nv; ++i) {
        if (i >= fd && i < fd + 6U) continue;
        if (std::cmp_equal(i, fw.dof) || std::cmp_equal(i, rw.dof)) continue;
        internal_dofs.push_back(i);
        const bool is_rider_trans =
            has_rider_free && std::cmp_greater_equal(i, rfd) && std::cmp_less(i, rfd + 3);
        const bool is_slide =
            jnt_type[static_cast<std::size_t>(dof_jnt[i])] == mjJNT_SLIDE;
        scales.push_back((is_slide || is_rider_trans) ? 0.05 : 0.20);
    }

    const bool has_compliant_3d = stepper.has_tire() && stepper.tire_writer().is_compliant_3d();
    std::array<double, 4> u_seed{};
    const std::size_t boff = scales.size();
    const double dt = m->opt.timestep;
    std::vector<double> tire_scratch(nv, 0.);
    if (has_compliant_3d) {
        std::ranges::fill(std::views::counted(d->qvel, m->nv), 0.);
        stepper.note_posvel_stale();
        stepper.forward();
        stepper.tire_writer().reset();
        stepper.tire_writer().compute_into(d, dt, tire_scratch);
        const auto init_row = stepper.tire_writer().state();
        const auto init_snaps = stepper.tire_writer().snapshots();
        const auto radii = stepper.tire_writer().radii();
        const auto preset = std::abs(radii[0] - 0.3720) < 0.003
            ? bike::spatial::SpatialTirePreset::mtb_2p4_standard
            : bike::spatial::SpatialTirePreset::mtb_2p8_1p2bar;
        const auto tire_cfg = make_compliant_3d_tire_config(preset);
        const std::array<double, 2> k_bx{tire_cfg.front.k_brush_x_n_m, tire_cfg.rear.k_brush_x_n_m};
        const std::array<double, 2> k_by{tire_cfg.front.k_brush_y_n_m, tire_cfg.rear.k_brush_y_n_m};
        for (std::size_t s = 0; s < 2U; ++s) {
            const std::size_t row_off = 14U * s;
            const auto& snap_opt = init_snaps[s];
            if (!std::isfinite(init_row[row_off]) || !snap_opt.has_value()) continue;
            const auto& patches = snap_opt->patches;
            if (patches.empty()) continue;
            const double nz = patches.front().normal[2];
            if (nz > 0.1) {
                const double fz = patches.front().normal_load_n / nz;
                u_seed[2U * s + 0U] = -(fz * init_row[row_off + 4U]) / k_bx[s];
                u_seed[2U * s + 1U] = -(fz * init_row[row_off + 7U]) / k_by[s];
            }
        }
        for (std::size_t k = 0; k < 4U; ++k) scales.push_back(0.005);
    }
    const std::size_t roff = scales.size();
    if (rider_state) {
        for (std::size_t k = 0; k < rider_state->supports.size(); ++k) scales.push_back(0.005);
        for (std::size_t s = 0; s < rider_state->grip_xi_local.size(); ++s)
            for (std::size_t axis = 0; axis < 3U; ++axis) scales.push_back(0.005);
    }

    const std::size_t n_var = scales.size();
    std::vector<double> dq(nv, 0.);
    std::vector<double> candidate_tire_row;

    auto place_spatial = [&](std::span<const double> y) {
        std::ranges::fill(dq, 0.);
        dq[fd + 2U] = scales[0] * y[0];
        const auto q_target = bike::spatial::quat_from_euler_zyx(
            eul0.roll_rad + scales[1] * y[1],
            eul0.pitch_rad + scales[2] * y[2],
            eul0.yaw_rad);
        const auto omega_step = quat_log(
            bike::spatial::quat_multiply(bike::spatial::quat_conjugate(quat0), q_target));
        dq[fd + 3U] = omega_step[0];
        dq[fd + 4U] = omega_step[1];
        dq[fd + 5U] = omega_step[2];
        for (std::size_t k = 0; k < internal_dofs.size(); ++k)
            dq[internal_dofs[k]] = scales[3U + k] * y[3U + k];

        auto next_rider = rider_state
            ? std::make_unique<writers::RiderContactsState>(*rider_state)
            : nullptr;
        if (next_rider) {
            std::size_t idx = roff;
            for (std::size_t k = 0; k < next_rider->supports.size(); ++k) {
                next_rider->supports[k].xi = rider_state->supports[k].xi + scales[idx] * y[idx];
                ++idx;
            }
            for (std::size_t s = 0; s < next_rider->grip_xi_local.size(); ++s) {
                for (std::size_t axis = 0; axis < 3U; ++axis) {
                    next_rider->grip_xi_local[s][axis] =
                        rider_state->grip_xi_local[s][axis] + scales[idx] * y[idx];
                    ++idx;
                }
            }
        }

        context.physical.restore(*incoming_step);
        stepper.mutate([&] { stepper.drive().restore(*incoming_drive); });
        if (next_rider) {
            auto probe = rider_probe;
            stepper.rider_contacts().assign_state(
                writers::RiderContactsState(*next_rider), std::move(probe));
        }

        const auto qpos = std::views::counted(d->qpos, m->nq);
        const auto qvel = std::views::counted(d->qvel, m->nv);
        std::ranges::copy(q0, qpos.begin());
        std::ranges::fill(qvel, 0.);
        stepper.mutate([&] { engine::integrate_pos(m, d->qpos, dq.data(), 1.); });

        const auto fw_q = static_cast<std::size_t>(fw.qpos);
        const auto rw_q = static_cast<std::size_t>(rw.qpos);
        const auto fw_d = static_cast<std::size_t>(fw.dof);
        const auto rw_d = static_cast<std::size_t>(rw.dof);
        const double theta_w_f = qpos[fw_q];
        const double theta_w_r = qpos[rw_q];
        stepper.note_posvel_stale();
        stepper.forward();

        if (has_compliant_3d) {
            stepper.tire_writer().reset();
            stepper.tire_writer().compute_into(d, dt, tire_scratch);
            candidate_tire_row = stepper.tire_writer().state();
            for (std::size_t s = 0; s < 2U; ++s) {
                const std::size_t row_off = 14U * s;
                if (std::isfinite(candidate_tire_row[row_off])) {
                    candidate_tire_row[row_off + 0U] =
                        u_seed[2U * s + 0U] + scales[boff + 2U * s + 0U] * y[boff + 2U * s + 0U];
                    candidate_tire_row[row_off + 1U] =
                        u_seed[2U * s + 1U] + scales[boff + 2U * s + 1U] * y[boff + 2U * s + 1U];
                }
            }
            stepper.tire_writer().set_state(Compliant3dTireWriter::state_names(), candidate_tire_row);
        }

        // First non-advancing solve with StaticBrake unlatched to read net wheel torques.
        apply_initial_forces(context, false);
        stepper.forward();

        const auto& drv = stepper.drive().config();
        const double k_hold = drv.brake_hold_stiffness_nm_rad;
        const double max_hold = context.physical.is_spatial()
            ? std::min(drv.brake_hold_torque_nm, drv.brake_ceiling_nm)
            : drv.brake_hold_torque_nm;
        const double u_f = std::clamp(context.physics.initial_front_brake, 0., 1.);
        const double u_r = std::clamp(context.physics.initial_rear_brake, 0., 1.);
        const double cap_f = max_hold * u_f;
        const double cap_r = max_hold * u_r;
        if (k_hold > 0. && (cap_f > 0. || cap_r > 0.)) {
            std::vector<double> efc_f(static_cast<std::size_t>(d->nefc), 0.);
            std::vector<double> non_fric(nv, 0.);
            const auto efc_types = std::views::counted(d->efc_type, d->nefc);
            const auto efc_forces = std::views::counted(d->efc_force, d->nefc);
            for (int e = 0; e < d->nefc; ++e) {
                const auto idx = static_cast<std::size_t>(e);
                if (efc_types[idx] != mjCNSTR_FRICTION_DOF)
                    efc_f[idx] = efc_forces[idx];
            }
            stepper.mutate([&] { engine::mul_jac_t_vec(m, d, non_fric.data(), efc_f.data()); });
            const auto bias = std::views::counted(d->qfrc_bias, m->nv);
            const auto applied = std::views::counted(d->qfrc_applied, m->nv);
            const auto passive = std::views::counted(d->qfrc_passive, m->nv);
            const double tau_req_f = cap_f > 0.
                ? std::clamp(bias[fw_d] - applied[fw_d] - passive[fw_d] - non_fric[fw_d], -cap_f, cap_f)
                : 0.;
            const double tau_req_r = cap_r > 0.
                ? std::clamp(bias[rw_d] - applied[rw_d] - passive[rw_d] - non_fric[rw_d], -cap_r, cap_r)
                : 0.;
            const double theta_latch_f = theta_w_f + tau_req_f / k_hold;
            const double theta_latch_r = theta_w_r + tau_req_r / k_hold;

            // Latch pad-bite anchors via an advancing apply_forces pass at theta_latch,
            // then restore candidate wheel angles, drivetrain, and tire bristles cleanly.
            qpos[fw_q] = theta_latch_f;
            qpos[rw_q] = theta_latch_r;
            stepper.note_posvel_stale();
            apply_initial_forces(context, true);

            qpos[fw_q] = theta_w_f;
            qpos[rw_q] = theta_w_r;
            std::ranges::fill(qvel, 0.);
            stepper.mutate([&] { stepper.drive().restore(*incoming_drive); });
            if (has_compliant_3d)
                stepper.tire_writer().set_state(Compliant3dTireWriter::state_names(), candidate_tire_row);
            if (next_rider) {
                auto probe = rider_probe;
                stepper.rider_contacts().assign_state(std::move(*next_rider), std::move(probe));
            }
            stepper.note_posvel_stale();
            apply_initial_forces(context, false);
            stepper.forward();
        }
    };

    std::vector<double> y_seed(n_var, 0.);
    std::vector<double> best_y(n_var, 0.);
    std::pair<int, double> best_rank{1, std::numeric_limits<double>::infinity()};
    std::vector<double> force_scale(nv), basis(nv, 0.), column(nv), f_gen(nv);
    std::exception_ptr error;
    const auto steer_d = static_cast<std::size_t>(steer.dof);

    for (int round = 0; round < 4; ++round) {
        y_seed = best_y;
        place_spatial(y_seed);
        const bool polish = acceleration_residual(context) < 25.;

        for (std::size_t i = 0; i < nv; ++i) {
            basis[i] = 1.;
            stepper.mutate([&] { engine::mul_m(m, d, column.data(), basis.data()); });
            force_scale[i] = std::sqrt(std::max(column[i], 1e-12));
            basis[i] = 0.;
        }

        auto eval_y = [&](std::span<const double> y_total, bool already_placed) -> std::vector<double> {
            if (!already_placed) place_spatial(y_total);
            stepper.mutate([&] { engine::mul_m(m, d, f_gen.data(), d->qacc); });
            std::vector<double> result(nv + 3U + n_var, 0.);
            if (polish) {
                std::ranges::copy(stepper.qacc(), result.begin());
            } else {
                for (std::size_t i = 0; i < nv; ++i)
                    result[i] = f_gen[i] / force_scale[i];
            }
            result[nv + 0U] = f_gen[fd + 3U];
            result[nv + 1U] = f_gen[fd + 4U];
            result[nv + 2U] = f_gen[steer_d];
            const double lambda = polish ? 1e-7 : 1e-5;
            for (std::size_t k = 0; k < n_var; ++k)
                result[nv + 3U + k] = lambda * y_total[k];

            const double acc_inf = acceleration_residual(context);
            const bool gate_ok = (acc_inf <= args.acceleration_tolerance) &&
                                 (std::abs(f_gen[fd + 3U]) <= .1) &&
                                 (std::abs(f_gen[fd + 4U]) <= .1) &&
                                 (std::abs(f_gen[steer_d]) <= .1);
            double squared = 0.;
            for (double const value : result) squared += value * value;
            const std::pair<int, double> rank{gate_ok ? 0 : 1, squared};
            if (std::isfinite(squared) && rank < best_rank) {
                best_rank = rank;
                best_y.assign(y_total.begin(), y_total.end());
            }
            return result;
        };

        if (round == 0) (void)eval_y(y_seed, true);
        if (best_rank.first == 0 && best_rank.second < 1e-10) break;

        std::vector<double> y_trial(n_var, 0.);
        const auto residual_fn = [&](std::span<const double> dy) {
            for (std::size_t k = 0; k < n_var; ++k) y_trial[k] = y_seed[k] + dy[k];
            return eval_y(y_trial, false);
        };

        numerics::LeastSquaresOptions options;
        options.max_evaluations = args.max_evaluations;
        options.difference = numerics::DifferenceScheme::three_point;
        options.ftol = options.xtol = options.gtol = 1e-12;
        options.stop = [&](std::span<const double>) {
            return best_rank.first == 0 && best_rank.second < 1e-10;
        };
        try {
            (void)numerics::bounded_least_squares(
                residual_fn, std::vector<double>(n_var, 0.),
                std::vector<double>(n_var, -1.), std::vector<double>(n_var, 1.), options);
        } catch (const engine::EngineFailure&) {
            throw;
        } catch (...) {
            error = std::current_exception();
        }
        if (best_rank.first == 0 && best_rank.second < 1e-10) break;
    }

    place_spatial(best_y);
    if (error) std::rethrow_exception(error);
    return acceleration_residual(context);
}

} // namespace

std::optional<FreeJointAddress> find_free_root(const mjModel* m) {
    if (m == nullptr) throw std::invalid_argument("null model in equilibrium");
    const int id = mj_name2id(m, mjOBJ_JOINT, "root_free");
    if (id < 0) return std::nullopt;
    const auto idx = static_cast<std::size_t>(id);
    const auto types = std::views::counted(m->jnt_type, m->njnt);
    if (types[idx] != mjJNT_FREE)
        throw std::invalid_argument("root_free must be a freejoint");
    return FreeJointAddress{
        .joint = id,
        .qpos = std::views::counted(m->jnt_qposadr, m->njnt)[idx],
        .dof = std::views::counted(m->jnt_dofadr, m->njnt)[idx]};
}

SpatialEquilibriumMoments spatial_equilibrium_moments(const SetupContext& context) {
    const mjModel* m = context.stepper.model();
    const mjData* d = context.stepper.data();
    const auto free_root = find_free_root(m);
    if (!free_root)
        throw std::invalid_argument("spatial_equilibrium_moments requires root_free");
    const JointAddress steer = joint_address(context, "steer_joint");
    const auto nv = static_cast<std::size_t>(m->nv);
    std::vector<double> f_gen(nv, 0.);
    context.stepper.mutate([&] { engine::mul_m(m, d, f_gen.data(), d->qacc); });
    const auto fd = static_cast<std::size_t>(free_root->dof);
    return {
        .roll_nm = f_gen[fd + 3U],
        .pitch_nm = f_gen[fd + 4U],
        .steer_nm = f_gen[static_cast<std::size_t>(steer.dof)],
    };
}

double refine_equilibrium(SetupContext& context,int max_evaluations,double acceleration_tolerance) {
    require_initializing(context);
    if(max_evaluations<=0||!std::isfinite(acceleration_tolerance)||acceleration_tolerance<=0.)
        throw std::invalid_argument("invalid equilibrium refinement budget or tolerance");
    auto& stepper=context.stepper;
    mjModel* m=stepper.model();mjData* d=stepper.data();
    if (const auto free_root = find_free_root(m)) {
        return refine_equilibrium_spatial(context, {.free_root = *free_root,
                                                    .max_evaluations = max_evaluations,
                                                    .acceleration_tolerance = acceleration_tolerance});
    }
    if(m->nq!=m->nv)throw std::invalid_argument("physical equilibrium requires scalar planar coordinates");
    const auto n=static_cast<std::size_t>(m->nv);
    const std::vector<double> q0(stepper.qpos().begin(),stepper.qpos().end());
    const auto root_x=static_cast<std::size_t>(joint_address(context,"root_x").qpos);
    // NOLINTNEXTLINE(modernize-make-unique) make_unique's inlined forwarding path overflows the 8192-byte frame budget here
    const auto incoming_drive=std::unique_ptr<drivetrain::DriveSnapshot>{new drivetrain::DriveSnapshot(stepper.drive().state())};
    // NOLINTNEXTLINE(modernize-make-unique) same frame-budget constraint
    const auto incoming_step=std::unique_ptr<runtime::StepState>{new runtime::StepState(context.physical.state())};
    const auto tire_names=stepper.has_tire()?stepper.tire_state_names():std::vector<std::string>{};
    const auto tire_state=stepper.has_tire()?stepper.tire_state():std::vector<double>{};
    std::unique_ptr<writers::RiderContactsState> rider_state;
    std::optional<writers::RiderContactsProbe> rider_probe;
    if(stepper.has_rider_contacts()){rider_state=std::make_unique<writers::RiderContactsState>(stepper.rider_contacts().state());rider_probe=stepper.rider_contacts().probe();}
    std::vector<std::size_t> qids,tire_ids;
    std::vector<double> x0,scales;
    const auto dof_jnt=std::views::counted(m->dof_jntid,m->nv);
    const auto joint_type=std::views::counted(m->jnt_type,m->njnt);
    for(std::size_t i=0;i<n;++i)if(i!=root_x){
        qids.push_back(i);x0.push_back(q0[i]);
        scales.push_back(joint_type[static_cast<std::size_t>(dof_jnt[i])]==mjJNT_SLIDE?.05:.2);
    }
    if(stepper.has_tire()&&!stepper.tire_writer().is_pneumatic())
        for(std::size_t i=0;i<tire_names.size();++i)
            if(tire_names[i].ends_with(".xi")&&std::isfinite(tire_state[i])){
                tire_ids.push_back(i);x0.push_back(tire_state[i]);scales.push_back(.005);
            }
    if(rider_state){
        for(const auto& support:rider_state->supports){x0.push_back(support.xi);scales.push_back(.005);}
        for(const auto& hand:rider_state->grip_xi_local)for(std::size_t const axis:{0U,2U}){x0.push_back(hand[axis]);scales.push_back(.005);}
    }
    std::vector<double> force_scale(n),basis(n,0.),column(n);
    for(std::size_t i=0;i<n;++i){
        basis[i]=1.;stepper.mutate([&]{engine::mul_m(m,d,column.data(),basis.data());});
        force_scale[i]=std::sqrt(std::max(column[i],1e-12));basis[i]=0.;
    }
    const bool polish=acceleration_residual(context)<1.;
    std::pair<int,double> best_rank{1,std::numeric_limits<double>::infinity()};
    std::vector<double> best=x0;
    auto place=[&](std::span<const double> x){
        // Every unordered candidate starts from the incoming material and
        // freehub datum. restore() stages its model writes; the ordinary
        // shared drive.prepare then linearizes at this trial's pose. No
        // ratchet state from an earlier rejected point reaches this trial.
        auto next_rider=rider_state?std::make_unique<writers::RiderContactsState>(*rider_state):nullptr;
        std::vector<double> next_tire=tire_state;
        std::size_t offset=qids.size();
        for(std::size_t const id:tire_ids)next_tire[id]=x[offset++];
        if(next_rider){
            for(auto& support:next_rider->supports)support.xi=x[offset++];
            for(auto& hand:next_rider->grip_xi_local)for(std::size_t const axis:{0U,2U})hand[axis]=x[offset++];
        }
        context.physical.restore(*incoming_step);
        stepper.mutate([&]{stepper.drive().restore(*incoming_drive);});
        if(stepper.has_tire())stepper.set_tire_state(tire_names,next_tire);
        if(next_rider){auto probe=rider_probe;stepper.rider_contacts().assign_state(std::move(*next_rider),std::move(probe));}
        auto const q=std::views::counted(d->qpos,m->nq);
        std::ranges::copy(q0,q.begin());
        for(std::size_t i=0;i<qids.size();++i)q[qids[i]]=x[i];
        std::ranges::fill(std::views::counted(d->qvel,m->nv),0.);
        stepper.note_posvel_stale();
    };
    auto const residual=[&](std::span<const double> y){
        std::vector<double> x(x0.size());
        for(std::size_t i=0;i<x.size();++i)x[i]=x0[i]+scales[i]*y[i];
        place(x);apply_initial_forces(context,false);stepper.forward();
        std::vector<double> result(n+x.size());
        if(polish)std::ranges::copy(stepper.qacc(),result.begin());
        else {
            stepper.mutate([&]{engine::mul_m(m,d,result.data(),d->qacc);});
            for(std::size_t i=0;i<n;++i)result[i]/=force_scale[i];
        }
        for(std::size_t i=0;i<x.size();++i)result[n+i]=(polish?1e-7:1e-5)*y[i];
        const double acceleration=acceleration_residual(context);
        double squared=0.;for(double const value:result)squared+=value*value;
        const std::pair<int,double> rank{acceleration>acceleration_tolerance?1:0,squared};
        if(std::isfinite(squared)&&rank<best_rank){best_rank=rank;best=std::move(x);}
        return result;
    };
    numerics::LeastSquaresOptions options;
    options.max_evaluations=max_evaluations;options.difference=numerics::DifferenceScheme::three_point;
    options.ftol=options.xtol=options.gtol=1e-10;
    options.stop=[&](std::span<const double>){return best_rank.first==0;};
    std::exception_ptr error;
    try {
        (void)numerics::bounded_least_squares(residual,std::vector<double>(x0.size(),0.),
            std::vector<double>(x0.size(),-1.),std::vector<double>(x0.size(),1.),options);
    } catch(const engine::EngineFailure&) { throw; }
      catch(...) { error=std::current_exception(); }
    // The best physically evaluated candidate wins, including candidates
    // discovered during differences; an optimizer status is not acceptance.
    place(best);apply_initial_forces(context,false);stepper.forward();
    if(error)std::rethrow_exception(error);
    return acceleration_residual(context);
}
} // namespace bike::initialization
