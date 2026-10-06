// writers/brake.cpp — expression-for-expression port of braking.py's
// compute path and its wheels.py helpers; comments cite file:line at the
// time of porting. Python min()/max() map to std::min/std::max — same
// two-argument keep-first-on-tie semantics, including the NaN cases.
#include "brake.hpp"
#include "../config_validation.hpp"
#include "../model_topology.hpp"
#include "../validation.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>

namespace {
    // wheels.py:40-71 — resolve_wheel_spin, same checks and order.
    std::pair<int, double> resolve_wheel_spin(const mjModel *m,
                                              const char *joint_name,
                                              const char *contact_geom) {
        const int jid = mj_name2id(m, mjOBJ_JOINT, joint_name);
        if (jid < 0)
            throw std::invalid_argument(
                "model has no joint '" + std::string(joint_name) +
                "'; ride-mode wheel torques need it");
        topology::joint(m, jid, joint_name, mjJNT_HINGE, {0., 1., 0.});
        const int gid = mj_name2id(m, mjOBJ_GEOM, contact_geom);
        if (gid < 0)
            throw std::invalid_argument(
                "model has no geom '" + std::string(contact_geom) +
                "'; ride-mode wheel torques need its radius");
        // views::counted everywhere — -Wunsafe-buffer-usage rejects raw
        // indexing into mjModel pointer fields (same pattern as suspension.cpp).
        const std::span<const int> geom_type =
                std::views::counted(m->geom_type, m->ngeom);
        if (geom_type[static_cast<std::size_t>(gid)] != mjGEOM_SPHERE)
            throw std::invalid_argument(
                "geom '" + std::string(contact_geom) +
                "' is not a sphere, so its size is not a wheel radius");
        const std::span<const int> dofadr =
                std::views::counted(m->jnt_dofadr, m->njnt);
        const std::span<const mjtNum> geom_size =
                std::views::counted(m->geom_size, 3 * m->ngeom);
        return {
            dofadr[static_cast<std::size_t>(jid)],
            geom_size[3 * static_cast<std::size_t>(gid)]
        };
    }

    // ride_sim.py:794-807 — _actuator_id.
    int actuator_id(const mjModel *m, const char *name) {
        const int id = mj_name2id(m, mjOBJ_ACTUATOR, name);
        if (id < 0)
            throw std::invalid_argument("model has no actuator '" +
                                        std::string(name) +
                                        "'; the ride torque path needs it");
        return id;
    }

    // wheels.py:74-97 — opposing_torque, including its own taper check (the
    // ctor rejects non-positive tapers already; the per-call check stays so a
    // NaN slips through to NaN rather than a branch).
    double opposing_torque(double magnitude_nm, double omega_radps,
                           double taper_radps) {
        if (taper_radps <= 0.0)
            throw std::invalid_argument(
                std::format("taper_radps must be positive, got {}", taper_radps));
        if (omega_radps == 0.0)
            return 0.0;
        const double scale = std::min(std::abs(omega_radps) / taper_radps, 1.0);
        return -std::abs(magnitude_nm) * scale *
               (omega_radps > 0.0 ? 1.0 : -1.0);
    }
} // namespace

BrakeWriter::BrakeWriter(const mjModel *m, nativecfg::BrakeConfig config)
    : cfg_(config), nv_(m->nv), nu_(m->nu) {
    validation::nonnegative(cfg_.torque_ceiling_nm, "BrakeWriter.torque_ceiling_nm");
    validation::positive(cfg_.taper_radps, "BrakeWriter.taper_radps");
    nativecfg::validate(cfg_);
    // braking.py:54-57 — ctor validation.
    if (cfg_.torque_ceiling_nm < 0.0)
        throw std::invalid_argument(
            std::format("torque_ceiling_nm must be non-negative, got {}",
                        cfg_.torque_ceiling_nm));
    if (cfg_.taper_radps <= 0.0)
        throw std::invalid_argument(
            std::format("taper_radps must be positive, got {}",
                        cfg_.taper_radps));
    // braking.py:61-62 — the names are literals in the Python source.
    auto [fdof, frad] =
            resolve_wheel_spin(m, "front_wheel_spin", "geom_front_contact");
    auto [rdof, rrad] =
            resolve_wheel_spin(m, "rear_wheel_spin", "geom_rear_contact");
    front_ = {.dofadr = fdof, .radius_m = frad};
    rear_ = {.dofadr = rdof, .radius_m = rrad};
    // ride_sim.py:330-331.
    front_ctrl_adr_ = actuator_id(m, "front_brake");
    rear_ctrl_adr_ = actuator_id(m, "rear_brake");
    topology::actuator(m, front_ctrl_adr_, "front_brake", mj_name2id(m, mjOBJ_JOINT, "front_wheel_spin"));
    topology::actuator(m, rear_ctrl_adr_, "rear_brake", mj_name2id(m, mjOBJ_JOINT, "rear_wheel_spin"));
}

// braking.py:89-107 — _wheel_torque; WheelSpin.omega_radps is the qvel read
// (wheels.py:35-37).
double BrakeWriter::wheel_torque(const mjData *d, const WheelSpin &wheel,
                                 double demand) const {
    validation::finite(demand, "BrakeWriter.demand");
    const double clamped = std::min(std::max(demand, 0.0), 1.0);
    const std::span<const mjtNum> qvel = std::views::counted(d->qvel, nv_);
    const double omega = qvel[static_cast<std::size_t>(wheel.dofadr)];
    return opposing_torque(clamped * cfg_.torque_ceiling_nm, omega,
                           cfg_.taper_radps);
}

std::pair<double, double> BrakeWriter::torques(const mjData *d,
                                               double front_demand,
                                               double rear_demand) const {
    last_ = {
        wheel_torque(d, front_, front_demand),
        wheel_torque(d, rear_, rear_demand)
    };
    return last_;
}

void BrakeWriter::apply(mjData *d, double front_demand,
                        double rear_demand) const {
    const auto [front_torque, rear_torque] =
            torques(d, front_demand, rear_demand);
    const std::span<mjtNum> ctrl = std::views::counted(d->ctrl, nu_);
    ctrl[static_cast<std::size_t>(front_ctrl_adr_)] = front_torque;
    ctrl[static_cast<std::size_t>(rear_ctrl_adr_)] = rear_torque;
}
