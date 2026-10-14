// Recover fatal MuJoCo C errors at a bounded, thread-local operation frame.
#pragma once

#include <mujoco/mujoco.h>

#include <array>
#include <stdexcept>

namespace engine {

enum class ErrorKind : unsigned char { none, fatal, unsupported_callback, poisoned };

struct ErrorBuffer {
    std::array<char, 2048> message{};
    ErrorKind kind = ErrorKind::none;
};

using Operation = void (*)(void *) noexcept;

// The operation must contain only trivial automatic state. A fatal MuJoCo
// message jumps over its C frames and returns false without throwing.
[[nodiscard]] bool invoke(Operation operation, void *context,
                          ErrorBuffer &error) noexcept;

class EngineFailure final : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
    EngineFailure(const EngineFailure &) = default;
    EngineFailure &operator=(const EngineFailure &) = default;
    EngineFailure(EngineFailure &&) = default;
    EngineFailure &operator=(EngineFailure &&) = default;
    ~EngineFailure() override;
};

[[noreturn]] void throw_failure(const ErrorBuffer &error);
void set_poisoned_error(ErrorBuffer &error) noexcept;
void set_unsupported_callback_error(ErrorBuffer &error, const char *name) noexcept;
void require_supported_callbacks();

[[nodiscard]] mjModel *load_model(const char *path);
[[nodiscard]] mjData *make_data(const mjModel *model);
void forward(const mjModel *model, mjData *data);
void step(const mjModel *model, mjData *data);
// mj_forwardSkip(mjSTAGE_VEL, skipsensor=0) — legal only when the data's
// position/velocity-stage products already correspond to the current
// qpos/qvel and pos/vel-stage model inputs; the caller owns that freshness
// contract. step_skip is mj_step verbatim except the forward inside skips
// the same stages (checks, FWDINV gate and integrator dispatch preserved).
void forward_skip(const mjModel *model, mjData *data);
void step_skip(const mjModel *model, mjData *data);
// step_skip without the forward: legal only when every acceleration-stage
// input (qfrc_applied, xfrc_applied, ctrl, act, time, qpos, qvel) is
// byte-identical to the state the last forward_skip saw and the pos/vel
// freshness contract holds — then the forward would reproduce the same
// bits mj_step's forward leaves (no hidden state: warm-start is written
// only by integrators). Checks, FWDINV gate, integrator dispatch and the
// STEP timer bookkeeping stay verbatim (perf-roadmap/17, R2).
void step_skip_fresh(const mjModel *model, mjData *data);
void implicit(const mjModel *model, mjData *data);
void reset_data(const mjModel *model, mjData *data);
void set_const(mjModel *model, mjData *data);

// mj_stateSize/mj_getState/mj_setState — the integration-vector boundary the
// runtime bootstrap uses. Framed like every engine call so a fatal message
// surfaces as EngineFailure instead of aborting.
[[nodiscard]] mjtSize state_size(const mjModel *model, int spec);
void get_state(const mjModel *model, const mjData *data, mjtNum *state, int spec);
void set_state(const mjModel *model, mjData *data, const mjtNum *state, int spec);

// mj_mulJacTVec — generalized force of a constraint-multiplier vector.
// Caller owns both buffers: result is nv-wide, vector is nefc-wide.
void mul_jac_t_vec(const mjModel *model, const mjData *data, mjtNum *result,
                   const mjtNum *vector);
// mj_mulJacVec — constraint-space velocity of a generalized velocity.
// Caller owns both buffers: result is nefc-wide, vector is nv-wide.
void mul_jac_vec(const mjModel *model, const mjData *data, mjtNum *result,
                 const mjtNum *vector);
// mj_mulM — mass-matrix product. result and vector are both nv-wide.
void mul_m(const mjModel *model, const mjData *data, mjtNum *result,
           const mjtNum *vector);
// mj_subtreeVel — subtree momentum observation caches (subtree_linvel /
// subtree_angmom). Observation only; it solves no forces.
void subtree_vel(const mjModel *model, mjData *data);
// mj_kinematics/mj_integratePos — detached-kinematics helpers for target
// prediction (scratch mjData only; never the live data's substitute).
void kinematics(const mjModel *model, mjData *data);
void integrate_pos(const mjModel *model, mjtNum *qpos, const mjtNum *qvel,
                   mjtNum dt);

[[nodiscard]] bool try_forward(const mjModel *model, mjData *data,
                               ErrorBuffer &error) noexcept;
[[nodiscard]] bool try_step(const mjModel *model, mjData *data,
                            ErrorBuffer &error) noexcept;
[[nodiscard]] bool try_forward_skip(const mjModel *model, mjData *data,
                                    ErrorBuffer &error) noexcept;
[[nodiscard]] bool try_step_skip(const mjModel *model, mjData *data,
                                 ErrorBuffer &error) noexcept;
[[nodiscard]] bool try_step_skip_fresh(const mjModel *model, mjData *data,
                                       ErrorBuffer &error) noexcept;
[[nodiscard]] bool try_implicit(const mjModel *model, mjData *data,
                                ErrorBuffer &error) noexcept;

} // namespace engine
