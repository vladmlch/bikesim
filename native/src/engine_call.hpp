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
void reset_data(const mjModel *model, mjData *data);
void set_const(mjModel *model, mjData *data);

// mj_stateSize/mj_getState/mj_setState — the integration-vector boundary the
// runtime bootstrap uses. Framed like every engine call so a fatal message
// surfaces as EngineFailure instead of aborting.
[[nodiscard]] mjtSize state_size(const mjModel *model, int spec);
void get_state(const mjModel *model, const mjData *data, mjtNum *state, int spec);
void set_state(const mjModel *model, mjData *data, const mjtNum *state, int spec);

[[nodiscard]] bool try_forward(const mjModel *model, mjData *data,
                               ErrorBuffer &error) noexcept;
[[nodiscard]] bool try_step(const mjModel *model, mjData *data,
                            ErrorBuffer &error) noexcept;

} // namespace engine
