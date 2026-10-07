// writers/brake.hpp — port of BrakeController (src/bike_sim/sim/ride/
// braking.py) plus the wheels.py helpers it calls: WheelSpin.omega_radps,
// resolve_wheel_spin, opposing_torque. The writer plays the compute() +
// ctrl-write pair from ride_sim's legacy step (ride_sim.py:530-534): the
// torques land in d.ctrl at the front_brake/rear_brake actuator addresses,
// not in qfrc — verified through the ctrl view, not the force matrix.
// FP operation order follows the Python source expression-for-expression —
// that order is the bitwise contract.
#pragma once

#include <mujoco/mujoco.h>

#include <utility>

#include "../config_types.hpp"

class BrakeWriter {
public:
    // Mirrors BrakeController.__init__ (braking.py:39-65): ceiling/taper
    // validation, then resolve_wheel_spin for both wheels and the two brake
    // actuator ids (ride_sim's _actuator_id). Wheel radius is resolved like
    // WheelSpin.radius_m even though compute never reads it — the sphere
    // check is part of the ctor contract. Throws std::invalid_argument on
    // any violation.
    BrakeWriter(const mjModel *m, nativecfg::BrakeConfig config);

    // BrakeController.compute (braking.py:67-87): (front, rear) torques in
    // N.m, front computed first like the source. Pure for d.
    [[nodiscard]] std::pair<double, double> torques(const mjData *d,
                                                    double front_demand,
                                                    double rear_demand) const;

    // ride_sim.py:530-534: compute() then write both torques into d->ctrl at
    // the resolved actuator addresses, front first.
    void apply(mjData *d, double front_demand, double rear_demand) const;

private:
    // WheelSpin fields; radius_m is resolved but unused by compute (see ctor
    // comment).
    struct WheelSpin {
        int dofadr = -1;
        double radius_m = 0.0;
    };

    double wheel_torque(const mjData *d, const WheelSpin &wheel,
                        double demand) const;

    nativecfg::BrakeConfig cfg_;
    mjtSize nv_ = 0, nu_ = 0;
    WheelSpin front_{}, rear_{};
    // The ctor resolves both actuators unconditionally (it throws on a
    // missing name); -1 is the declared "not resolved" state, matching
    // mj_name2id's miss sentinel.
    int front_ctrl_adr_ = -1, rear_ctrl_adr_ = -1;
    // No retained telemetry: torques() returns the computed pair directly.
    // The Python controller's self.front/rear_torque_nm snapshot exists only
    // so its own compute() can return what it just stored — this writer
    // keeps the value local, so every call is scratch-free and const in
    // observable effect.
};
