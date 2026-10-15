#include "engine_call.hpp"
#include "engine_abi_312.hpp"
#include "engine_identity.hpp"
#include "cblas_abi.hpp"
#include "io/hash.hpp"

#include <mujoco/mjxmacro.h>

#include <dlfcn.h>

#include <algorithm>
#include <bit>
#include <chrono>
#include <csetjmp>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <memory>
#include <ranges>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace engine {
namespace {

constexpr auto model_size_names = std::array{
#define X(name) std::string_view{#name},
    MJMODEL_SIZES
#undef X
};
static_assert(std::size(model_size_names) == 98);
constexpr std::size_t plugin_size_index = 72;
static_assert(model_size_names[plugin_size_index] == "nplugin");

template <typename T>
T read_native_value(std::span<const char> bytes, std::size_t offset) noexcept {
    std::array<char, sizeof(T)> representation{};
    std::ranges::copy(bytes.subspan(offset, sizeof(T)), representation.begin());
    return std::bit_cast<T>(representation);
}

bool contains_plugins(std::span<const char> bytes) noexcept {
    constexpr std::size_t header_size = 5 * sizeof(int);
    constexpr std::size_t plugin_offset =
        header_size + plugin_size_index * sizeof(mjtSize);
    if (bytes.size() < plugin_offset + sizeof(mjtSize)) return false;
    std::array<int, 5> header{};
    for (std::size_t i = 0; i < header.size(); ++i)
        header[i] = read_native_value<int>(bytes, i * sizeof(int));
    // Invalid headers are rejected by MuJoCo before it can read plugin data.
    if (header[0] != 54321 || !std::cmp_equal(header[1], sizeof(mjtNum)) ||
        !std::cmp_equal(header[2], model_size_names.size()) ||
        header[3] != mj_version())
        return false;
    const mjtSize count = read_native_value<mjtSize>(bytes, plugin_offset);
    return count > 0;
}

struct Frame {
    std::jmp_buf jump;
    Frame *previous_frame;
    mjfLogHandler previous_handler;
    mjfLogHandler forward_handler;
    ErrorBuffer *error;
};

static_assert(std::is_trivially_destructible_v<Frame>);
// The active jump target is necessarily mutable and thread local. MuJoCo has
// no public nonreturning per-thread handler API in the pinned 3.12.0 release.
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
thread_local Frame *active_frame = nullptr;

void append_bounded(std::span<char> destination, std::size_t &used,
                    std::string_view source) noexcept {
    for (const char value : source) {
        if (used + 1 >= destination.size()) break;
        destination[used++] = value;
    }
    destination[used] = '\0';
}

void plugin_error(ErrorBuffer &error) noexcept {
    error.kind = ErrorKind::unsupported_callback;
    std::size_t used = 0;
    append_bounded(error.message, used, "unsupported MuJoCo plugin model");
}

void copy_error(const mjLogMessage *message, ErrorBuffer &error) noexcept {
    std::size_t used = 0;
    if (message == nullptr) {
        append_bounded(error.message, used, "MuJoCo fatal error");
        return;
    }
    if (message->func != nullptr) {
        append_bounded(error.message, used, std::string_view(message->func));
        append_bounded(error.message, used, ": ");
    }
    const std::span subject(message->subject);
    const auto end = std::ranges::find(subject, '\0');
    append_bounded(error.message, used,
                   std::string_view(subject.data(),
                                    static_cast<std::size_t>(end - subject.begin())));
}

[[noreturn]] void fatal_message(const mjLogMessage *message) noexcept {
    // jmp_buf is a mutable C array; a const pointee cannot be passed to longjmp.
    // NOLINTNEXTLINE(misc-const-correctness)
    Frame *frame = active_frame;
    if (frame == nullptr || frame->error == nullptr)
        std::abort();
    copy_error(message, *frame->error);
    // MuJoCo's error callback must not return or unwind through its C frames.
    // NOLINTNEXTLINE(cert-err52-cpp,modernize-avoid-setjmp-longjmp,cppcoreguidelines-pro-bounds-array-to-pointer-decay)
    std::longjmp(frame->jump, 1);
}

extern "C" void log_handler(const mjLogMessage *message) noexcept {
    const Frame *frame = active_frame;
    if (frame == nullptr || message == nullptr)
        std::abort();
    if (message->level == mjLOG_ERROR)
        fatal_message(message);
    if (frame->forward_handler != nullptr)
        frame->forward_handler(message);
}

const char *unsupported_callback() noexcept {
    if (mjcb_control != nullptr) return "mjcb_control";
    if (mjcb_passive != nullptr) return "mjcb_passive";
    if (mjcb_sensor != nullptr) return "mjcb_sensor";
    if (mjcb_contactfilter != nullptr) return "mjcb_contactfilter";
    // Stock MjData construction installs GetTime here (structs_wrappers.cc
    // in tag 3.12.0). Its Python trampoline also clears owned references
    // before reporting an error, so this slot is compatible with our frame.
    if (mjcb_act_dyn != nullptr) return "mjcb_act_dyn";
    if (mjcb_act_gain != nullptr) return "mjcb_act_gain";
    if (mjcb_act_bias != nullptr) return "mjcb_act_bias";
    if (mju_user_error != nullptr) return "mju_user_error";
    if (mju_user_warning != nullptr) return "mju_user_warning";
    if (mju_user_malloc != nullptr) return "mju_user_malloc";
    if (mju_user_free != nullptr) return "mju_user_free";
    return nullptr;
}

void copy_callback_error(ErrorBuffer &error, const char *name) noexcept {
    std::size_t used = 0;
    append_bounded(error.message, used,
                   "unsupported installed MuJoCo callback: ");
    append_bounded(error.message, used, std::string_view(name));
}

void invalid_solver_error(ErrorBuffer &error, int solver) noexcept {
    error.kind = ErrorKind::fatal;
    std::size_t used = 0;
    append_bounded(error.message, used,
                   "mj_fwdConstraint: unknown solver type ");
    if (solver < 0) append_bounded(error.message, used, "-");
    std::uint64_t magnitude = solver < 0
                                  ? static_cast<std::uint64_t>(-static_cast<std::int64_t>(solver))
                                  : static_cast<std::uint64_t>(solver);
    std::array<char, 11> reversed{};
    std::size_t length = 0;
    while (true) {
        reversed[length++] = static_cast<char>('0' + magnitude % 10);
        magnitude /= 10;
        if (magnitude == 0) break;
    }
    while (length != 0 && used + 1 < error.message.size())
        error.message[used++] = reversed[--length];
    error.message[used] = '\0';
}

void restore_frame(const Frame &frame) noexcept {
    active_frame = frame.previous_frame;
    static_cast<void>(_mjPRIVATE_setTlsLogHandler(frame.previous_handler));
}

struct LoadContext { const void *bytes; int size; mjModel *result; };
void load_operation(void *raw) noexcept {
    auto *context = static_cast<LoadContext *>(raw);
    context->result = mj_loadModelBuffer(context->bytes, context->size);
}

struct AllocateDataContext { mjData *result; };
void allocate_data_operation(void *raw) noexcept {
    auto *context = static_cast<AllocateDataContext *>(raw);
    context->result = static_cast<mjData *>(mju_malloc(sizeof(mjData)));
}

struct MakeRawDataContext { const mjModel *model; mjData *data; };
void make_raw_data_operation(void *raw) noexcept {
    auto *context = static_cast<MakeRawDataContext *>(raw);
    mj_makeRawData(&context->data, context->model);
}

struct ModelDataContext { const mjModel *model; mjData *data; };
void forward_operation(void *raw) noexcept {
    const auto *context = static_cast<ModelDataContext *>(raw);
    mj_forward(context->model, context->data);
}
void step_operation(void *raw) noexcept {
    const auto *context = static_cast<ModelDataContext *>(raw);
    mj_step(context->model, context->data);
}
void forward_skip_operation(void *raw) noexcept {
    const auto *context = static_cast<ModelDataContext *>(raw);
    mj_forwardSkip(context->model, context->data, mjSTAGE_VEL, 0);
}
void implicit_operation(void *raw) noexcept {
    const auto *context = static_cast<ModelDataContext *>(raw);
    mj_implicit(context->model, context->data);
}
// mj_step (engine_forward.c, tag 3.12.0) with an mjSTAGE_VEL-skipped
// forward: checks, the FWDINV comparison gate, the integrator dispatch and
// the STEP timer bookkeeping stay verbatim — only the proven-fresh
// position/velocity stages are elided. (TM_START/TM_END live in
// engine_macro.h; the hand-inlined timer lines replicate them against the
// public mjcb_time/d->timer fields.)
void step_skip_operation(void *raw) noexcept {
    const auto *context = static_cast<ModelDataContext *>(raw);
    const mjModel *m = context->model;
    mjData *d = context->data;
    const mjtNum tm = (mjcb_time ? mjcb_time() : 0);
    mj_checkPos(m, d);
    mj_checkVel(m, d);
    mj_forwardSkip(m, d, mjSTAGE_VEL, 0);
    mj_checkAcc(m, d);
    if ((static_cast<unsigned>(m->opt.enableflags) &
         static_cast<unsigned>(mjENBL_FWDINV)) != 0U)
        mj_compareFwdInv(m, d);
    // opt.integrator is an int in the ABI; switching on the raw value keeps
    // the out-of-range default reachable (mj_step's verbatim behaviour).
    switch (m->opt.integrator) {
    case mjINT_EULER:
        mj_Euler(m, d);
        break;
    case mjINT_RK4:
        mj_RungeKutta(m, d, 4);
        break;
    case mjINT_IMPLICIT:
    case mjINT_IMPLICITFAST:
        mj_implicit(m, d);
        break;
    default:
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg) mju_error is the engine's fatal-error ABI — mj_step calls it verbatim
        mju_error("invalid integrator");
    }
    d->timer[mjTIMER_STEP].duration += (mjcb_time ? mjcb_time() : 0) - tm;
    d->timer[mjTIMER_STEP].number++;
}
// step_skip_operation minus the forward (see engine_call.hpp: step_skip_fresh).
void step_skip_fresh_operation(void *raw) noexcept {
    const auto *context = static_cast<ModelDataContext *>(raw);
    const mjModel *m = context->model;
    mjData *d = context->data;
    const mjtNum tm = (mjcb_time ? mjcb_time() : 0);
    mj_checkPos(m, d);
    mj_checkVel(m, d);
    // forward elided: the caller proved the acc-stage inputs unchanged.
    mj_checkAcc(m, d);
    if ((static_cast<unsigned>(m->opt.enableflags) &
         static_cast<unsigned>(mjENBL_FWDINV)) != 0U)
        mj_compareFwdInv(m, d);
    // opt.integrator is an int in the ABI; switching on the raw value keeps
    // the out-of-range default reachable (mj_step's verbatim behaviour).
    switch (m->opt.integrator) {
    case mjINT_EULER:
        mj_Euler(m, d);
        break;
    case mjINT_RK4:
        mj_RungeKutta(m, d, 4);
        break;
    case mjINT_IMPLICIT:
    case mjINT_IMPLICITFAST:
        mj_implicit(m, d);
        break;
    default:
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg) mju_error is the engine's fatal-error ABI — mj_step calls it verbatim
        mju_error("invalid integrator");
    }
    d->timer[mjTIMER_STEP].duration += (mjcb_time ? mjcb_time() : 0) - tm;
    d->timer[mjTIMER_STEP].number++;
}
void reset_operation(void *raw) noexcept {
    const auto *context = static_cast<ModelDataContext *>(raw);
    mj_resetData(context->model, context->data);
}

struct MutableModelDataContext { mjModel *model; mjData *data; };
void set_const_operation(void *raw) noexcept {
    const auto *context = static_cast<MutableModelDataContext *>(raw);
    mj_setConst(context->model, context->data);
}

struct StateSizeContext { const mjModel *model; int spec; mjtSize result; };
void state_size_operation(void *raw) noexcept {
    auto *context = static_cast<StateSizeContext *>(raw);
    context->result = mj_stateSize(context->model, context->spec);
}
struct GetStateContext { const mjModel *model; const mjData *data; mjtNum *state; int spec; };
// NOLINTNEXTLINE(misc-const-correctness) void* param is the fixed Operation ABI
void get_state_operation(void *raw) noexcept {
    const auto *context = static_cast<const GetStateContext *>(raw);
    mj_getState(context->model, context->data, context->state, context->spec);
}
struct SetStateContext { const mjModel *model; mjData *data; const mjtNum *state; int spec; };
// NOLINTNEXTLINE(misc-const-correctness) void* param is the fixed Operation ABI
void set_state_operation(void *raw) noexcept {
    const auto *context = static_cast<const SetStateContext *>(raw);
    mj_setState(context->model, context->data, context->state, context->spec);
}

struct MulJacTVecContext { const mjModel *model; const mjData *data; mjtNum *result; const mjtNum *vector; };
// NOLINTNEXTLINE(misc-const-correctness) void* param is the fixed Operation ABI
void mul_jac_t_vec_operation(void *raw) noexcept {
    const auto *context = static_cast<const MulJacTVecContext *>(raw);
    mj_mulJacTVec(context->model, context->data, context->result,
                  context->vector);
}
// NOLINTNEXTLINE(misc-const-correctness) void* param is the fixed Operation ABI
void mul_jac_vec_operation(void *raw) noexcept {
    const auto *context = static_cast<const MulJacTVecContext *>(raw);
    mj_mulJacVec(context->model, context->data, context->result,
                 context->vector);
}
// NOLINTNEXTLINE(misc-const-correctness) void* param is the fixed Operation ABI
void mul_m_operation(void *raw) noexcept {
    const auto *context = static_cast<const MulJacTVecContext *>(raw);
    mj_mulM(context->model, context->data, context->result,
            context->vector);
}
// NOLINTNEXTLINE(misc-const-correctness) void* param is the fixed Operation ABI
void subtree_vel_operation(void *raw) noexcept {
    const auto *context = static_cast<const ModelDataContext *>(raw);
    mj_subtreeVel(context->model, context->data);
}

// NOLINTNEXTLINE(misc-const-correctness) void* param is the fixed Operation ABI
void kinematics_operation(void *raw) noexcept {
    const auto *context = static_cast<const ModelDataContext *>(raw);
    mj_kinematics(context->model, context->data);
}

struct IntegratePosContext { const mjModel *model; mjtNum *qpos; const mjtNum *qvel; mjtNum dt; };
// NOLINTNEXTLINE(misc-const-correctness) void* param is the fixed Operation ABI
void integrate_pos_operation(void *raw) noexcept {
    const auto *context = static_cast<const IntegratePosContext *>(raw);
    mj_integratePos(context->model, context->qpos, context->qvel,
                    context->dt);
}

// ---- SHA-256 for the resolved engine image (T04) ---------------------------
// The FIPS 180-4 compression lives in the owned io/hash.cpp boundary (T05):
// bike::io::Sha256 is the shared digest every config/recording/fixture and
// engine-identity consumer uses — one implementation, no private copy.
// SHA-256 over a resolved file's bytes as lowercase hex. Throwing here is
// correct: a runtime that cannot read the library it loaded cannot verify
// against the locked digest, which must fail closed rather than report "".
[[nodiscard]] std::string file_sha256_hex(const char *path) {
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open())
        throw std::runtime_error(
            "engine::identity: cannot open the resolved MuJoCo library");
    bike::io::Sha256 digest;
    std::array<char, 4096> buffer{};
    while (file.good()) {
        file.read(buffer.data(),
                  static_cast<std::streamsize>(buffer.size()));
        const std::streamsize got = file.gcount();
        if (got > 0)
            digest.update(std::as_bytes(
                std::span(buffer).first(static_cast<std::size_t>(got))));
        if (file.bad())
            throw std::runtime_error(
                "engine::identity: read failure while hashing the MuJoCo library");
    }
    static constexpr std::string_view digits = "0123456789abcdef";
    std::string hex;
    hex.reserve(64);
    for (const std::uint8_t byte : digest.finish()) {
        hex += digits[byte >> 4U];
        hex += digits[byte & 0x0fU];
    }
    return hex;
}

} // namespace

EngineFailure::~EngineFailure() = default;

bool invoke(Operation operation, void *context, ErrorBuffer &error) noexcept {
    error.message[0] = '\0';
    error.kind = ErrorKind::none;
    if (const char *callback = unsupported_callback()) {
        error.kind = ErrorKind::unsupported_callback;
        copy_callback_error(error, callback);
        return false;
    }

    Frame frame{};
    frame.previous_frame = active_frame;
    frame.error = &error;
    frame.previous_handler = _mjPRIVATE_setTlsLogHandler(&log_handler);
    // A stock handler may sit between two native frames. It forwards warnings
    // to the outer native handler, which otherwise sees this inner active
    // frame again and recurses. Continue at the outer frame's destination.
    frame.forward_handler = frame.previous_frame != nullptr
                                ? frame.previous_frame->forward_handler
                                : frame.previous_handler != nullptr
                                      ? frame.previous_handler
                                      : _mjPRIVATE_getGlobalLogHandler();
    // Both normal and longjmp exits call restore_frame before this stack
    // object dies; the nested/concurrent C++ contracts exercise each path.
    // cppcheck-suppress danglingLifetime
    active_frame = &frame;
    // Only trivial automatic objects lie between this point and the C error
    // handler. C++ exceptions would unwind through MuJoCo C frames instead.
    // NOLINTNEXTLINE(cert-err52-cpp,modernize-avoid-setjmp-longjmp,cppcoreguidelines-pro-bounds-array-to-pointer-decay)
    if (setjmp(frame.jump) == 0) {
        operation(context);
        restore_frame(frame);
        return true;
    }
    restore_frame(frame);
    error.kind = ErrorKind::fatal;
    return false;
}

void throw_failure(const ErrorBuffer &error) {
    if (error.kind == ErrorKind::unsupported_callback)
        throw std::invalid_argument(error.message.data());
    if (error.kind == ErrorKind::poisoned)
        throw std::logic_error(error.message.data());
    throw EngineFailure(error.message[0] ? error.message.data()
                                          : "MuJoCo fatal error");
}

void set_poisoned_error(ErrorBuffer &error) noexcept {
    error.kind = ErrorKind::poisoned;
    std::size_t used = 0;
    append_bounded(error.message, used,
                   "Stepper is poisoned after a fatal MuJoCo error; call reset");
}

void set_unsupported_callback_error(ErrorBuffer &error, const char *name) noexcept {
    error.kind = ErrorKind::unsupported_callback;
    copy_callback_error(error, name);
}

void require_supported_callbacks() {
    if (const char *callback = unsupported_callback()) {
        ErrorBuffer error;
        set_unsupported_callback_error(error, callback);
        throw_failure(error);
    }
}

// ---- Native callback provenance (T04) ------------------------------------

mjtNum native_time() noexcept {
    using Clock = std::chrono::steady_clock;
    // Same epoch convention as the vendored DefaultTimer: steady-clock
    // seconds since first use. The static's guard is taken here (cold path)
    // so a marked step never blocks on first-use initialization.
    static const Clock::time_point epoch = Clock::now();
    return std::chrono::duration<mjtNum>(Clock::now() - epoch).count();
}

TimeCallbackProvenance time_callback_provenance() noexcept {
    if (mjcb_time == nullptr) return TimeCallbackProvenance::none;
    if (mjcb_time == &native_time) return TimeCallbackProvenance::native;
    return TimeCallbackProvenance::foreign;
}

bool try_native_time_policy(ErrorBuffer &error) noexcept {
    error.message[0] = '\0';
    error.kind = ErrorKind::none;
    switch (time_callback_provenance()) {
    case TimeCallbackProvenance::none:
        return true;
    case TimeCallbackProvenance::native:
        // Warm the epoch's static guard outside any marked region — the same
        // warm-up the vendored timers get before their first marked call.
        static_cast<void>(native_time());
        return true;
    case TimeCallbackProvenance::foreign:
        set_unsupported_callback_error(error, "mjcb_time");
        return false;
    }
    // -Wcovered-switch-default: all enumerators handled above.
    std::unreachable();
}

void require_native_time_policy() {
    ErrorBuffer error;
    if (!try_native_time_policy(error)) throw_failure(error);
}

// ---- Resolved engine identity (T04) ---------------------------------------

Identity identity() {
    const char *version = mj_versionString();
    if (version == nullptr)
        throw std::runtime_error(
            "engine::identity: mj_versionString returned null");
    // mj_versionString's return is a static char array inside the loaded
    // library — dladdr on that DATA address identifies the image without a
    // function-to-object reinterpret_cast (same technique as
    // runtime/provenance.cpp).
    Dl_info location{};
    if (dladdr(static_cast<const void *>(version), &location) == 0 ||
        location.dli_fname == nullptr)
        throw std::runtime_error(
            "engine::identity: cannot identify the loaded MuJoCo library");
    return Identity{.mujoco_version = version,
                    .library_sha256 = file_sha256_hex(location.dli_fname),
                    .numeric_provider = std::string(blas::provider_name)};
}

void verify_identity(const Identity &required) {
    const Identity resolved = identity();
    const auto compare = [](std::string_view field,
                            const std::string &wanted,
                            const std::string &actual) {
        if (wanted != actual)
            throw std::invalid_argument(
                "engine::verify_identity: " + std::string(field) +
                " mismatch — required '" + wanted + "', resolved '" +
                actual + "'");
    };
    compare("mujoco_version", required.mujoco_version,
            resolved.mujoco_version);
    compare("library_sha256", required.library_sha256,
            resolved.library_sha256);
    compare("numeric_provider", required.numeric_provider,
            resolved.numeric_provider);
}

mjModel *load_model(const char *path) {
    // Read through C++ ownership before installing a jump frame. MuJoCo's
    // resource loader can hold C++ RAII objects and call user providers.
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) return nullptr;
    const std::streampos end = file.tellg();
    if (end <= 0 || end > std::streampos(std::numeric_limits<int>::max()))
        return nullptr;
    std::vector<char> bytes(static_cast<std::size_t>(end));
    file.seekg(0);
    if (!file.read(bytes.data(), static_cast<std::streamsize>(bytes.size())))
        return nullptr;
    if (contains_plugins(bytes))
        throw std::invalid_argument("unsupported MuJoCo plugin model");
    LoadContext context{.bytes = bytes.data(),
                        .size = static_cast<int>(bytes.size()), .result = nullptr};
    ErrorBuffer error;
    if (!invoke(load_operation, &context, error)) throw_failure(error);
    return context.result;
}

mjData *make_data(const mjModel *model) {
    if (model->nplugin != 0)
        throw std::invalid_argument("unsupported MuJoCo plugin model");
    // mj_makeData allocates mjData before reset; this known reset fatal would
    // otherwise jump past its private partial-allocation cleanup.
    if (model->nhistory > 0 && model->opt.timestep <= 0)
        throw std::invalid_argument("MuJoCo history requires positive timestep");
    AllocateDataContext allocation{.result = nullptr};
    ErrorBuffer error;
    if (!invoke(allocate_data_operation, &allocation, error)) throw_failure(error);
    if (allocation.result == nullptr) return nullptr;
    // A value-initialized mjData has empty plugin/buffer ownership, so
    // mj_deleteData also works if the private raw-data call stops midway.
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(
        std::construct_at(allocation.result), &mj_deleteData);
    MakeRawDataContext context{.model = model, .data = data.get()};
    if (!invoke(make_raw_data_operation, &context, error)) throw_failure(error);
    if (data->buffer == nullptr || (data->narena > 0 && data->arena == nullptr))
        return nullptr;
    reset_data(model, data.get());
    return data.release();
}

bool try_forward(const mjModel *model, mjData *data, ErrorBuffer &error) noexcept {
    if (model->nplugin != 0) {
        plugin_error(error);
        return false;
    }
    if (model->opt.solver < mjSOL_PGS || model->opt.solver > mjSOL_NEWTON) {
        invalid_solver_error(error, model->opt.solver);
        return false;
    }
    ModelDataContext context{.model = model, .data = data};
    return invoke(forward_operation, &context, error);
}

bool try_step(const mjModel *model, mjData *data, ErrorBuffer &error) noexcept {
    if (model->nplugin != 0) {
        plugin_error(error);
        return false;
    }
    if (model->opt.solver < mjSOL_PGS || model->opt.solver > mjSOL_NEWTON) {
        invalid_solver_error(error, model->opt.solver);
        return false;
    }
    ModelDataContext context{.model = model, .data = data};
    return invoke(step_operation, &context, error);
}

bool try_forward_skip(const mjModel *model, mjData *data,
                      ErrorBuffer &error) noexcept {
    if (model->nplugin != 0) {
        plugin_error(error);
        return false;
    }
    if (model->opt.solver < mjSOL_PGS || model->opt.solver > mjSOL_NEWTON) {
        invalid_solver_error(error, model->opt.solver);
        return false;
    }
    ModelDataContext context{.model = model, .data = data};
    return invoke(forward_skip_operation, &context, error);
}

bool try_step_skip(const mjModel *model, mjData *data,
                   ErrorBuffer &error) noexcept {
    if (model->nplugin != 0) {
        plugin_error(error);
        return false;
    }
    if (model->opt.solver < mjSOL_PGS || model->opt.solver > mjSOL_NEWTON) {
        invalid_solver_error(error, model->opt.solver);
        return false;
    }
    ModelDataContext context{.model = model, .data = data};
    return invoke(step_skip_operation, &context, error);
}

bool try_step_skip_fresh(const mjModel *model, mjData *data,
                   ErrorBuffer &error) noexcept {
    if (model->nplugin != 0) {
        plugin_error(error);
        return false;
    }
    if (model->opt.solver < mjSOL_PGS || model->opt.solver > mjSOL_NEWTON) {
        invalid_solver_error(error, model->opt.solver);
        return false;
    }
    ModelDataContext context{.model = model, .data = data};
    return invoke(step_skip_fresh_operation, &context, error);
}

bool try_implicit(const mjModel *model, mjData *data,
                  ErrorBuffer &error) noexcept {
    if (model->nplugin != 0) {
        plugin_error(error);
        return false;
    }
    ModelDataContext context{.model = model, .data = data};
    return invoke(implicit_operation, &context, error);
}

void forward(const mjModel *model, mjData *data) {
    ErrorBuffer error;
    if (!try_forward(model, data, error)) throw_failure(error);
}

void step(const mjModel *model, mjData *data) {
    ErrorBuffer error;
    if (!try_step(model, data, error)) throw_failure(error);
}

void forward_skip(const mjModel *model, mjData *data) {
    ErrorBuffer error;
    if (!try_forward_skip(model, data, error)) throw_failure(error);
}

void step_skip(const mjModel *model, mjData *data) {
    ErrorBuffer error;
    if (!try_step_skip(model, data, error)) throw_failure(error);
}

void step_skip_fresh(const mjModel *model, mjData *data) {
    ErrorBuffer error;
    if (!try_step_skip_fresh(model, data, error)) throw_failure(error);
}

void implicit(const mjModel *model, mjData *data) {
    ErrorBuffer error;
    if (!try_implicit(model, data, error)) throw_failure(error);
}

void reset_data(const mjModel *model, mjData *data) {
    if (model->nplugin != 0)
        throw std::invalid_argument("unsupported MuJoCo plugin model");
    ModelDataContext context{.model = model, .data = data};
    ErrorBuffer error;
    if (!invoke(reset_operation, &context, error)) throw_failure(error);
}

void set_const(mjModel *model, mjData *data) {
    if (model->nplugin != 0)
        throw std::invalid_argument("unsupported MuJoCo plugin model");
    MutableModelDataContext context{.model = model, .data = data};
    ErrorBuffer error;
    if (!invoke(set_const_operation, &context, error)) throw_failure(error);
}

mjtSize state_size(const mjModel *model, int spec) {
    if (model->nplugin != 0)
        throw std::invalid_argument("unsupported MuJoCo plugin model");
    StateSizeContext context{.model = model, .spec = spec, .result = 0};
    ErrorBuffer error;
    if (!invoke(state_size_operation, &context, error)) throw_failure(error);
    return context.result;
}

void get_state(const mjModel *model, const mjData *data, mjtNum *state, int spec) {
    if (model->nplugin != 0)
        throw std::invalid_argument("unsupported MuJoCo plugin model");
    GetStateContext context{.model = model, .data = data, .state = state, .spec = spec};
    ErrorBuffer error;
    if (!invoke(get_state_operation, &context, error)) throw_failure(error);
}

void set_state(const mjModel *model, mjData *data, const mjtNum *state, int spec) {
    if (model->nplugin != 0)
        throw std::invalid_argument("unsupported MuJoCo plugin model");
    SetStateContext context{.model = model, .data = data, .state = state, .spec = spec};
    ErrorBuffer error;
    if (!invoke(set_state_operation, &context, error)) throw_failure(error);
}

void mul_jac_t_vec(const mjModel *model, const mjData *data, mjtNum *result,
                   const mjtNum *vector) {
    if (model->nplugin != 0)
        throw std::invalid_argument("unsupported MuJoCo plugin model");
    MulJacTVecContext context{.model = model, .data = data,
                              .result = result, .vector = vector};
    ErrorBuffer error;
    if (!invoke(mul_jac_t_vec_operation, &context, error)) throw_failure(error);
}

void mul_jac_vec(const mjModel *model, const mjData *data, mjtNum *result,
                 const mjtNum *vector) {
    if (model->nplugin != 0)
        throw std::invalid_argument("unsupported MuJoCo plugin model");
    MulJacTVecContext context{.model = model, .data = data,
                              .result = result, .vector = vector};
    ErrorBuffer error;
    if (!invoke(mul_jac_vec_operation, &context, error)) throw_failure(error);
}

void mul_m(const mjModel *model, const mjData *data, mjtNum *result,
           const mjtNum *vector) {
    if (model->nplugin != 0)
        throw std::invalid_argument("unsupported MuJoCo plugin model");
    MulJacTVecContext context{.model = model, .data = data,
                              .result = result, .vector = vector};
    ErrorBuffer error;
    if (!invoke(mul_m_operation, &context, error)) throw_failure(error);
}

void subtree_vel(const mjModel *model, mjData *data) {
    if (model->nplugin != 0)
        throw std::invalid_argument("unsupported MuJoCo plugin model");
    ModelDataContext context{.model = model, .data = data};
    ErrorBuffer error;
    if (!invoke(subtree_vel_operation, &context, error)) throw_failure(error);
}

void kinematics(const mjModel *model, mjData *data) {
    if (model->nplugin != 0)
        throw std::invalid_argument("unsupported MuJoCo plugin model");
    ModelDataContext context{.model = model, .data = data};
    ErrorBuffer error;
    if (!invoke(kinematics_operation, &context, error)) throw_failure(error);
}

void integrate_pos(const mjModel *model, mjtNum *qpos, const mjtNum *qvel,
                   mjtNum dt) {
    if (model->nplugin != 0)
        throw std::invalid_argument("unsupported MuJoCo plugin model");
    IntegratePosContext context{.model = model, .qpos = qpos,
                                .qvel = qvel, .dt = dt};
    ErrorBuffer error;
    if (!invoke(integrate_pos_operation, &context, error)) throw_failure(error);
}

} // namespace engine
