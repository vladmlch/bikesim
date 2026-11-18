#include "context.hpp"
#include "refine.hpp"
#include "../engine_call.hpp"
#include "../runtime/step.hpp"
#include "../spatial/frames.hpp"
#include "../stepper.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <ranges>
#include <stdexcept>
#include <utility>
#include <vector>

namespace bike::initialization {
namespace {
constexpr double kPhaseAKRoll = 4000.;
constexpr double kPhaseAKY = 15000.;
constexpr double kPhaseAKPsi = 1500.;
constexpr double kPhaseAKSteer = 500.;

struct PhaseAHoldState {
    const SetupContext* owner = nullptr;
    int free_qpos = -1;
    int free_dof = -1;
    int rider_free_qpos = -1;
    int rider_free_dof = -1;
    int steer_qpos = -1;
    int steer_dof = -1;
    double d_roll = 0.;
    double d_y = 0.;
    double d_psi = 0.;
    double d_rider_roll = 0.;
    double d_rider_y = 0.;
    double d_rider_psi = 0.;
    double d_steer = 0.;
    double psi0 = 0.;
};

PhaseAHoldState& phase_a_hold_storage() noexcept {
    thread_local PhaseAHoldState state{};
    return state;
}

struct DampingSpec {
    double inertia = 0.;
    double stiffness = 0.;
    double dt = 0.;
};

[[nodiscard]] double capped_damping(DampingSpec spec) noexcept {
    const double m_eff = std::max(0., spec.inertia);
    return std::min(2. * std::sqrt(m_eff * spec.stiffness), 0.5 * m_eff / spec.dt);
}

// Launch part 4b (user ruling 2026-11-13): the settle holds crank_spin at
// the prepared pedal angle, the way it holds the prepared lean — spec
// 2026-11-11-rider-launch-plan-design.md:38. A stiff PD restoring torque
// through the step's external generalized-force channel: the constraint
// solver balances it like any applied torque, so the crank reaches
// qacc ~ 0 at the held pose (a kinematic clamp leaves the reaction on the
// pedal hinges and does not converge — experiment Follow-up 8). Setup
// only: require_initializing guards the caller, and the ride's steps pass
// no external force, so nothing holds the crank in the ride.
double crank_hold_torque(SetupContext const& context) {
    const auto crank=joint_address(context,"crank_spin");
    const mjModel* m=context.stepper.model();const mjData* d=context.stepper.data();
    const auto qpos=std::views::counted(d->qpos,m->nq);
    const auto qvel=std::views::counted(d->qvel,m->nv);
    const double seed=context.physics.drive.crank_phase_rad;
    return -context.physics.equilibrium_crank_hold_kp_nm_rad*(qpos[static_cast<std::size_t>(crank.qpos)]-seed)
           -context.physics.equilibrium_crank_hold_kd_nms_rad*qvel[static_cast<std::size_t>(crank.dof)];
}

struct FreeRyyHoldSpec {
    int free_qpos = -1;
    int free_dof = -1;
    double d_roll = 0.;
    double d_y = 0.;
    double d_psi = 0.;
    double psi0 = 0.;
};

void add_free_ryy_hold(std::span<const double> qpos,
                       std::span<const double> qvel,
                       const FreeRyyHoldSpec& spec,
                       std::vector<double>& hold) {
    const auto fq = static_cast<std::size_t>(spec.free_qpos);
    const auto fd = static_cast<std::size_t>(spec.free_dof);
    const bike::spatial::Quat q_curr{
        qpos[fq + 3U], qpos[fq + 4U], qpos[fq + 5U], qpos[fq + 6U]};
    const auto eul = bike::spatial::euler_zyx(q_curr);
    const bike::spatial::Vec3 omega_b{qvel[fd + 3U], qvel[fd + 4U], qvel[fd + 5U]};
    const auto rates = bike::spatial::body_gyro_to_euler_zyx_rates(eul, omega_b);
    if (!eul.valid || !rates.valid)
        throw std::runtime_error("spatial Phase A hold encountered gimbal lock");

    // Euler roll axis (phi) is intrinsic body +X -> maps 1-to-1 to free_dof + 3.
    hold[fd + 3U] += -kPhaseAKRoll * eul.roll_rad - spec.d_roll * rates.roll_rate_rad_s;
    // Euler yaw axis (psi) is world +Z -> covector R^T * (0, 0, tau_psi_w)^T.
    const double tau_psi_w =
        -kPhaseAKPsi * (eul.yaw_rad - spec.psi0) -
        spec.d_psi * rates.yaw_rate_rad_s;
    const auto tau_psi_b = bike::spatial::rotate_world_to_body(q_curr, {0., 0., tau_psi_w});
    hold[fd + 3U] += tau_psi_b[0];
    hold[fd + 4U] += tau_psi_b[1];
    hold[fd + 5U] += tau_psi_b[2];

    // Lateral corridor hold (y -> 0).
    hold[fd + 1U] += -kPhaseAKY * qpos[fq + 1U] - spec.d_y * qvel[fd + 1U];
}

void add_phase_a_hold(const SetupContext& context, std::vector<double>& hold) {
    const auto& state = phase_a_hold_storage();
    const mjModel* m = context.stepper.model();
    const mjData* d = context.stepper.data();
    const auto qpos = std::views::counted(d->qpos, m->nq);
    const auto qvel = std::views::counted(d->qvel, m->nv);
    add_free_ryy_hold(qpos, qvel, {
        .free_qpos = state.free_qpos,
        .free_dof = state.free_dof,
        .d_roll = state.d_roll,
        .d_y = state.d_y,
        .d_psi = state.d_psi,
        .psi0 = state.psi0,
    }, hold);
    if (state.rider_free_dof >= 0) {
        add_free_ryy_hold(qpos, qvel, {
            .free_qpos = state.rider_free_qpos,
            .free_dof = state.rider_free_dof,
            .d_roll = state.d_rider_roll,
            .d_y = state.d_rider_y,
            .d_psi = state.d_rider_psi,
            .psi0 = state.psi0,
        }, hold);
    }
    // Temporary steer_joint lock (delta -> 0).
    if (state.steer_dof >= 0) {
        const auto sq = static_cast<std::size_t>(state.steer_qpos);
        const auto sd = static_cast<std::size_t>(state.steer_dof);
        hold[sd] += -kPhaseAKSteer * qpos[sq] - state.d_steer * qvel[sd];
    }
}
} // namespace

PhaseAHoldScope::PhaseAHoldScope(SetupContext& context) {
    require_initializing(context);
    const mjModel* m = context.stepper.model();
    const auto free_root = find_free_root(m);
    if (!free_root) return;
    auto& state = phase_a_hold_storage();
    if (state.owner != nullptr)
        throw std::logic_error("nested spatial PhaseAHoldScope is not supported");

    if (!context.stepper.posvel_fresh()) context.stepper.forward();
    const mjData* d = context.stepper.data();
    const double dt = m->opt.timestep;
    if (!std::isfinite(dt) || !(dt > 0.))
        throw std::invalid_argument("Phase A hold requires positive finite timestep");

    const JointAddress steer = joint_address(context, "steer_joint");
    const auto nv = static_cast<std::size_t>(m->nv);
    std::vector<double> basis(nv, 0.);
    std::vector<double> col(nv, 0.);
    const auto diag_inertia = [&](int dof) -> double {
        const auto idx = static_cast<std::size_t>(dof);
        basis[idx] = 1.;
        context.stepper.mutate([&] { engine::mul_m(m, d, col.data(), basis.data()); });
        basis[idx] = 0.;
        return col[idx];
    };

    const double m_yy = diag_inertia(free_root->dof + 1);
    const double m_phi = diag_inertia(free_root->dof + 3);
    const double m_psi = diag_inertia(free_root->dof + 5);
    const double m_delta = diag_inertia(steer.dof);

    int rider_free_qpos = -1;
    int rider_free_dof = -1;
    double d_rider_roll = 0.;
    double d_rider_y = 0.;
    double d_rider_psi = 0.;
    const int rider_free_id = mj_name2id(m, mjOBJ_JOINT, "rider_root_free");
    if (rider_free_id >= 0 &&
        std::views::counted(m->jnt_type, m->njnt)[static_cast<std::size_t>(rider_free_id)] == mjJNT_FREE) {
        const auto r_idx = static_cast<std::size_t>(rider_free_id);
        rider_free_qpos = std::views::counted(m->jnt_qposadr, m->njnt)[r_idx];
        rider_free_dof = std::views::counted(m->jnt_dofadr, m->njnt)[r_idx];
        d_rider_roll = capped_damping({.inertia = diag_inertia(rider_free_dof + 3), .stiffness = kPhaseAKRoll, .dt = dt});
        d_rider_y = capped_damping({.inertia = diag_inertia(rider_free_dof + 1), .stiffness = kPhaseAKY, .dt = dt});
        d_rider_psi = capped_damping({.inertia = diag_inertia(rider_free_dof + 5), .stiffness = kPhaseAKPsi, .dt = dt});
    }

    const auto qpos = std::views::counted(d->qpos, m->nq);
    const auto fq = static_cast<std::size_t>(free_root->qpos);
    const bike::spatial::Quat q_init{
        qpos[fq + 3U], qpos[fq + 4U], qpos[fq + 5U], qpos[fq + 6U]};
    const auto eul = bike::spatial::euler_zyx(q_init);
    if (!eul.valid)
        throw std::runtime_error("spatial Phase A hold requires non-singular initial pitch");

    state = PhaseAHoldState{
        .owner = &context,
        .free_qpos = free_root->qpos,
        .free_dof = free_root->dof,
        .rider_free_qpos = rider_free_qpos,
        .rider_free_dof = rider_free_dof,
        .steer_qpos = steer.qpos,
        .steer_dof = steer.dof,
        .d_roll = capped_damping({.inertia = m_phi, .stiffness = kPhaseAKRoll, .dt = dt}),
        .d_y = capped_damping({.inertia = m_yy, .stiffness = kPhaseAKY, .dt = dt}),
        .d_psi = capped_damping({.inertia = m_psi, .stiffness = kPhaseAKPsi, .dt = dt}),
        .d_rider_roll = d_rider_roll,
        .d_rider_y = d_rider_y,
        .d_rider_psi = d_rider_psi,
        .d_steer = capped_damping({.inertia = m_delta, .stiffness = kPhaseAKSteer, .dt = dt}),
        .psi0 = eul.yaw_rad,
    };
    active_ = true;
}

PhaseAHoldScope::~PhaseAHoldScope() {
    if (active_) phase_a_hold_storage() = PhaseAHoldState{};
}

bool phase_a_hold_active(const SetupContext& context) noexcept {
    return context.initializing && phase_a_hold_storage().owner == &context;
}

void apply_initial_forces(SetupContext& context,bool advance) {
    require_initializing(context);
    // Launch part 2: an articulated rider holds the prepared start lean
    // and the prepared pedal angle (launch part 4b) through the settle.
    // A zero lean keeps the empty command the settle
    // always sent, and a lumped or absent rider has no torso to lean — the
    // step rejects any posture for it (control.cpp:40-44).
    runtime::RideControl control{};
    if(context.preparation.lean_rad!=0.&&context.physical.rider_present()){
        auto posture=::rider::RiderPosture{};
        posture.torso_lean_rad=context.preparation.lean_rad;
        control.posture=posture;
    }
    // Launch part 4b: with the feet welded to the pedals (spindle) the same
    // settle also holds the crank at the table angle. Otherwise (no rider, a
    // lumped rider, flat pedals; weld is unmeasured) the crank is only its
    // own light assembly, about 0.013 kg·m², and the explicit hold would
    // ring it — it settles free, as before the hold.
    std::optional<std::vector<double>> external;
    if(context.physical.rider_present()&&context.physics.articulated.pedal_attachment=="spindle"){
        const mjModel* m=context.stepper.model();
        std::vector<double> hold(static_cast<std::size_t>(m->nv),0.);
        const auto crank=joint_address(context,"crank_spin");
        hold[static_cast<std::size_t>(crank.dof)]=crank_hold_torque(context);
        external=std::move(hold);
    }
    if (phase_a_hold_active(context)) {
        if (!external)
            external.emplace(static_cast<std::size_t>(context.stepper.model()->nv), 0.);
        add_phase_a_hold(context, *external);
    }
    (void)context.physical.apply_forces(false,advance,
        context.physics.initial_front_brake,context.physics.initial_rear_brake,
        external,control);
}
double acceleration_residual(SetupContext const& context) {
    double result=0.;
    for(double const a:context.stepper.qacc()) {
        if(!std::isfinite(a))throw std::runtime_error("non-finite physical equilibrium acceleration");
        result=std::max(result,std::abs(a));
    }
    return result;
}
void settle_contacts(SetupContext& context,const std::vector<double>& qpos,
                     const std::vector<double>& qvel) {
    require_initializing(context);
    if(context.stepper.has_rider_contacts())
        (void)context.stepper.mutate([&]{return context.stepper.rider_contacts_settle(
            writers::RiderIntervalState{qpos,qvel},false,nullptr);});
}
} // namespace bike::initialization
