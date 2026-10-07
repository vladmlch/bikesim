#include "stepper.hpp"
#include "engine_call.hpp"
#include "validation.hpp"
#include <bit>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <dlfcn.h>
#include <filesystem>
#include <stdexcept>
#include <nanobind/stl/string.h>
#include "config.hpp"
#include "writers/brake.hpp"
#include "writers/cruise.hpp"
#include "writers/resistance.hpp"
#include "writers/rider_forces.hpp"
#include "writers/suspension.hpp"
#include "writers/tire.hpp"
#include "writers/drivetrain.hpp"

// The stepper.hpp view scratch is literal-sized (the writer constants live
// in headers stepper.hpp cannot include without pulling writer bodies into
// its boundary) — pin the literal values here.
static_assert(SuspensionWriter::kPhysicalComponentCount == 8 &&
              "suspension_views_ must cover the physical-mode count");
static_assert(ResistanceWriter::kComponentCount == 2 &&
              "resistance_views_ must cover kComponentCount");

namespace {
    // Same check for every incoming span — one place only (the binding converts
    // ndarray→span and forwards; widths live on the model, which only the core
    // can see). Width is mjtSize (int64) in mjModel; the cast is the explicit,
    // always-safe direction (a negative width can never equal a size, so it
    // still throws). The api.field names mirror the Python call signature.
    void check_width(std::span<const double> values, std::string_view api,
                     std::string_view field, mjtSize expected) {
        if (values.size() != static_cast<std::size_t>(expected))
            throw std::invalid_argument(
                std::string(api) + "." + std::string(field) + ": has " +
                std::to_string(values.size()) + " elements; model expects " +
                std::to_string(expected));
    }

    void check_finite(std::span<const double> values, std::string_view api,
                      std::string_view field) {
        for (const double value: values)
            if (!std::isfinite(value))
                throw std::invalid_argument(
                    std::string(api) + "." + std::string(field) +
                    ": expected finite values");
    }

    // Empty spans are skipped, not memcpy'd: d_->act is nullptr when na==0.
    void copy_in(std::span<const double> src, mjtNum *dst) {
        if (!src.empty())
            std::ranges::copy(src, dst);
    }
} // namespace

Stepper::Stepper(const std::string &mjb_path, nanobind::handle config)
    : Stepper(mjb_path, wire::mapping(config, "config")) {}

Stepper::Stepper(const std::string &mjb_path) : m_(nullptr), d_(nullptr) {
    // Reject raw C timers before any engine operation can call them while a
    // fatal-error jump frame is active. Stock Python trampolines catch their
    // own errors and are allowed here, outside the marked realtime region.
    require_unmarked_time_callback();
    // Keep both allocations locally owned until the fatal-capable forward
    // succeeds. A throwing constructor has no Stepper destructor to clean up.
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
        engine::load_model(mjb_path.c_str()), &mj_deleteModel);
    if (!model) throw std::runtime_error("mj_loadModel failed: " + mjb_path);
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(
        engine::make_data(model.get()), &mj_deleteData);
    if (!data) throw std::runtime_error("mj_makeData");
    engine::forward(model.get(), data.get());
    m_ = model.release();
    d_ = data.release();
}

Stepper::Stepper(const std::string &mjb_path, const nanobind::dict &config)
    : Stepper(mjb_path) {
    // Config-parse or writer-construction failure must not leak m_/d_:
    // ~Stepper never runs for an object whose constructor throws.
    try {
        if (config.empty())
            return; // no sections → every writer stays disabled
        nativecfg::NativeConfig cfg =
                nativecfg::native_config_from_dict(config);
        if (cfg.cruise)
            cruise_ = std::make_unique<CruiseWriter>(m_, *cfg.cruise);
        if (cfg.suspension)
            suspension_ =
                    std::make_unique<SuspensionWriter>(m_, *cfg.suspension);
        if (cfg.brake)
            brake_ = std::make_unique<BrakeWriter>(m_, *cfg.brake);
        if (cfg.resistance)
            resistance_ =
                    std::make_unique<ResistanceWriter>(m_, *cfg.resistance);
        if (cfg.tire) {
            tire_ = std::make_unique<TireWriter>(m_, d_, *cfg.tire);
            // Construction-sized convenience scratch — the boxed tire_qfrc
            // reroutes through compute_into instead of copying qfrc()'s
            // member out per call.
            tire_out_.resize(static_cast<std::size_t>(m_->nv));
        }
        if (cfg.rider_forces) {
            rider_forces_ =
                    std::make_unique<RiderForcesWriter>(m_, *cfg.rider_forces);
            rider_out_.resize(static_cast<std::size_t>(m_->nv));
        }
        if (cfg.drive)
            drive_ = std::make_unique<drivetrain::DrivetrainWriter>(m_, d_, *cfg.drive);
    } catch (...) {
        drive_.reset();
        mj_deleteData(d_);
        mj_deleteModel(m_);
        d_ = nullptr;
        m_ = nullptr;
        throw;
    }
}

Stepper::~Stepper() {
    drive_.reset();
    if (d_) mj_deleteData(d_);
    if (m_) mj_deleteModel(m_);
}

void Stepper::require_healthy() const {
    if (poisoned_)
        throw std::logic_error(
            "Stepper is poisoned after a fatal MuJoCo error; call reset");
}

void Stepper::require_unmarked_time_callback() {
    refresh_time_callback_policy();
    if (!unmarked_time_callback_safe_)
        throw std::invalid_argument(
            "unsupported installed MuJoCo callback: mjcb_time");
    engine::require_supported_callbacks();
}

void Stepper::refresh_time_callback_policy() {
    const mjfTime current = mjcb_time;
    if (time_callback_policy_ready_ && current == observed_time_callback_)
        return;
    const auto record = [this, current](mjfTime allowed, bool unmarked_safe) noexcept {
        allowed_time_callback_ = allowed;
        observed_time_callback_ = current;
        unmarked_time_callback_safe_ = unmarked_safe;
        time_callback_policy_ready_ = true;
    };
    if (current == nullptr) {
        record(nullptr, true);
        return;
    }

    const nanobind::gil_scoped_acquire gil;
    const nanobind::object callback =
        nanobind::module_::import_("mujoco").attr("get_mjcb_time")();
    if (!callback.is_none()) {
        const nanobind::object cfunc_type =
            nanobind::module_::import_("ctypes").attr("_CFuncPtr");
        const bool raw_c_callback = nanobind::cast<bool>(
            nanobind::module_::import_("builtins").attr("isinstance")(
                callback, cfunc_type));
        record(nullptr, !raw_c_callback);
        return;
    }

    // structs_wrappers.cc in MuJoCo 3.12.0 installs GetTime from _structs
    // after constructing a stock MjData. Identify that exact module before
    // calling it; direct foreign C timer pointers are not part of this owner.
    static_assert(sizeof(mjfTime) == sizeof(const void *));
    Dl_info image{};
    // dladdr requires an object pointer on Apple platforms; this pinned ABI
    // uses equal-sized function and object pointers for module identification.
    // NOLINTNEXTLINE(bugprone-bitwise-pointer-cast)
    const auto address = std::bit_cast<const void *>(current);
    if (dladdr(address, &image) == 0 || image.dli_fname == nullptr) {
        record(nullptr, false);
        return;
    }
    const nanobind::object structs =
        nanobind::module_::import_("mujoco._structs");
    const std::string expected = nanobind::cast<std::string>(structs.attr("__file__"));
    if (!std::filesystem::equivalent(image.dli_fname, expected)) {
        record(nullptr, false);
        return;
    }

    // GetTime initializes a static steady-clock epoch. Do that before
    // entering BIKE_NONBLOCKING: its first call takes a C++ guard lock.
    static_cast<void>(current());
    record(current, true);
}

bool Stepper::try_step(engine::ErrorBuffer &error) noexcept {
    if (poisoned_) {
        engine::set_poisoned_error(error);
        return false;
    }
    if (mjcb_time != nullptr && mjcb_time != allowed_time_callback_) {
        engine::set_unsupported_callback_error(error, "mjcb_time");
        return false;
    }
    const bool succeeded = engine::try_step(m_, d_, error);
    if (!succeeded && error.kind == engine::ErrorKind::fatal)
        poisoned_ = true;
    return succeeded;
}

bool Stepper::try_forward(engine::ErrorBuffer &error) noexcept {
    if (poisoned_) {
        engine::set_poisoned_error(error);
        return false;
    }
    if (mjcb_time != nullptr && mjcb_time != allowed_time_callback_) {
        engine::set_unsupported_callback_error(error, "mjcb_time");
        return false;
    }
    const bool succeeded = engine::try_forward(m_, d_, error);
    if (!succeeded && error.kind == engine::ErrorKind::fatal)
        poisoned_ = true;
    return succeeded;
}

void Stepper::step() {
    if (!poisoned_) refresh_time_callback_policy();
    engine::ErrorBuffer error;
    if (!try_step(error)) engine::throw_failure(error);
}

void Stepper::forward() {
    if (!poisoned_) refresh_time_callback_policy();
    engine::ErrorBuffer error;
    if (!try_forward(error)) engine::throw_failure(error);
}

void Stepper::reset() {
    require_unmarked_time_callback();
    bool data_reset = false;
    try {
        engine::reset_data(m_, d_);
        data_reset = true;
        engine::forward(m_, d_);
        if (drive_) drive_->reset();
        if (tire_) tire_->reset();
        if (cruise_) cruise_->reset();
        poisoned_ = false;
    } catch (const engine::EngineFailure &) {
        poisoned_ = true;
        throw;
    } catch (...) {
        if (data_reset) poisoned_ = true;
        throw;
    }
}

drivetrain::DrivetrainWriter &Stepper::drive() const {
    if (!drive_)
        throw std::logic_error(
            "drive: Stepper was built without a drive config (pass the dict "
            "from tools.native_config.project)");
    return *drive_;
}

void Stepper::set_inputs(std::span<const double> ctrl,
                         std::span<const double> force) {
    require_healthy();
    check_width(ctrl, "set_inputs", "ctrl", m_->nu);
    check_width(force, "set_inputs", "qfrc_applied", m_->nv);
    check_finite(ctrl, "set_inputs", "ctrl");
    check_finite(force, "set_inputs", "qfrc_applied");
    const std::vector<double> saved_ctrl(ctrl.begin(), ctrl.end());
    const std::vector<double> saved_force(force.begin(), force.end());
    copy_in(saved_ctrl, d_->ctrl);
    copy_in(saved_force, d_->qfrc_applied);
}

CruiseWriter &Stepper::require_cruise() const {
    if (!cruise_)
        throw std::logic_error(
            "cruise: Stepper was built without a cruise config (pass the "
            "dict from tools.native_config.project)");
    return *cruise_;
}

double Stepper::cruise_compute(bool rear_in_contact, bool traction_limited,
                               std::optional<bool> controller_grounded) {
    require_healthy();
    return require_cruise().compute(d_, rear_in_contact, traction_limited,
                                    controller_grounded);
}

void Stepper::cruise_reset() {
    require_healthy();
    require_cruise().reset();
}

void Stepper::cruise_set_target_speed(double value_kmh) {
    require_healthy();
    require_cruise().set_target_speed(value_kmh);
}

double Stepper::cruise_set_assist_compensation(double support_factor) {
    require_healthy();
    return require_cruise().set_assist_compensation(support_factor);
}

CruiseState Stepper::cruise_state() const { return require_cruise().state(); }

void Stepper::set_cruise_state(const CruiseState &state) {
    require_healthy();
    require_cruise().set_state(state);
}

void Stepper::set_state(std::span<const double> qpos,
                        std::span<const double> qvel,
                        std::span<const double> act,
                        std::span<const double> warmstart,
                        double time) {
    require_unmarked_time_callback();
    // All domain checks first (strong guarantee), then the same sequence the
    // Python oracle runs: mj_resetData + buffer writes + mj_forward.
    check_width(qpos, "set_state", "qpos", m_->nq);
    check_width(qvel, "set_state", "qvel", m_->nv);
    check_width(act, "set_state", "act", m_->na);
    check_width(warmstart, "set_state", "warmstart", m_->nv);
    check_finite(qpos, "set_state", "qpos");
    check_finite(qvel, "set_state", "qvel");
    check_finite(act, "set_state", "act");
    check_finite(warmstart, "set_state", "warmstart");
    validation::nonnegative(time, "set_state.time");
    // Callers may pass our own views, including cross-buffer aliases (e.g.
    // qacc as warmstart). Snapshot EVERY input before resetting any mjData
    // buffer. Allocation failures also leave the old state intact.
    const std::vector<double> saved_qpos(qpos.begin(), qpos.end());
    const std::vector<double> saved_qvel(qvel.begin(), qvel.end());
    const std::vector<double> saved_act(act.begin(), act.end());
    const std::vector<double> saved_warmstart(warmstart.begin(), warmstart.end());
    const bool recovering = poisoned_;
    try {
        engine::reset_data(m_, d_);
    } catch (const engine::EngineFailure &) {
        poisoned_ = true;
        throw;
    }
    copy_in(saved_qpos, d_->qpos);
    copy_in(saved_qvel, d_->qvel);
    copy_in(saved_act, d_->act);
    copy_in(saved_warmstart, d_->qacc_warmstart);
    d_->time = time;
    if (recovering) {
        try {
            if (drive_) drive_->reset();
            if (tire_) tire_->reset();
            if (cruise_) cruise_->reset();
        } catch (...) {
            poisoned_ = true;
            throw;
        }
    }
    poisoned_ = false;
}

namespace {
    // Serialize a ForceComponentView prefix as the boxed (name, values)
    // list components() used to return — identical names/values/insertion
    // order, but the names come from the writer's constant table and the
    // values copy from the writer's own persistent component storage, so
    // the writer side never allocates: only this serialization copy does,
    // and it IS the boxing surface the caller asked for.
    [[nodiscard]] std::vector<std::pair<std::string, std::vector<double> > >
    box_views(std::span<const std::string_view> names,
              std::span<const ForceComponentView> views) {
        std::vector<std::pair<std::string, std::vector<double> > > out;
        out.reserve(views.size());
        for (std::size_t i = 0; i < views.size(); ++i)
            out.emplace_back(std::string(names[i]),
                             std::vector<double>(views[i].values.begin(),
                                                 views[i].values.end()));
        return out;
    }
} // namespace

std::vector<std::pair<std::string, std::vector<double> > >
Stepper::suspension_components() const {
    if (!suspension_)
        throw std::logic_error(
            "suspension_components: Stepper was built without a suspension "
            "config (pass the dict from tools.native_config.project)");
    // Route through the non-allocating writer face over member scratch —
    // the serialized copy below is the only allocation, and it is the
    // return value itself.
    const std::span<const std::string_view> names =
            suspension_->component_names();
    suspension_->components_into(
            d_, std::span{suspension_views_}.first(names.size()));
    return box_views(names, std::span{suspension_views_}.first(names.size()));
}

std::pair<double, double>
Stepper::brake_torques(double front_demand, double rear_demand) const {
    if (!brake_)
        throw std::logic_error(
            "brake_torques: Stepper was built without a brake config (pass "
            "the dict from tools.native_config.project)");
    return brake_->torques(d_, front_demand, rear_demand);
}

void Stepper::apply_brake(double front_demand, double rear_demand) {
    require_healthy();
    if (!brake_)
        throw std::logic_error(
            "apply_brake: Stepper was built without a brake config (pass "
            "the dict from tools.native_config.project)");
    brake_->apply(d_, front_demand, rear_demand);
}

std::vector<std::pair<std::string, std::vector<double> > >
Stepper::resistance_components(const TireSideInput &front,
                               const TireSideInput &rear) const {
    if (!resistance_)
        throw std::logic_error(
            "resistance_components: Stepper was built without a resistance "
            "config (pass the dict from tools.native_config.project)");
    // Same routing as suspension_components: non-allocating writer face,
    // member scratch, one serialization copy out.
    const std::span<const std::string_view> names =
            ResistanceWriter::component_names();
    resistance_->components_into(d_, front, rear, resistance_views_);
    return box_views(names, resistance_views_);
}

namespace {
    constexpr const char *kNoTire =
            "tire writer: Stepper was built without a tire config (pass the dict "
            "from tools.native_config.project)";
} // namespace

std::vector<double> Stepper::tire_qfrc(double dt) {
    require_healthy();
    if (!tire_)
        throw std::logic_error(kNoTire);
    // compute_into writes the caller-owned construction-sized scratch —
    // same advance+commit pipeline as qfrc(), no writer-side copy.
    tire_->compute_into(d_, dt, tire_out_);
    return tire_out_;
}

void Stepper::set_tire_state(std::span<const std::string> names,
                             std::span<const double> row) {
    require_healthy();
    if (!tire_)
        throw std::logic_error(kNoTire);
    tire_->set_state(names, row);
}

std::vector<double> Stepper::tire_state() const {
    if (!tire_)
        throw std::logic_error(kNoTire);
    return tire_->state();
}

const std::vector<std::string> &Stepper::tire_state_names() const {
    return TireWriter::state_names();
}

std::vector<double> Stepper::rider_forces_qfrc() const {
    require_healthy();
    if (!rider_forces_)
        throw std::logic_error(
            "rider_forces_qfrc: Stepper was built without a rider_forces "
            "config (pass the dict from tools.native_config.project)");
    rider_forces_->compute_into(d_, rider_out_);
    return rider_out_;
}

std::vector<double>
Stepper::total(std::span<const std::vector<double>> components) const {
    // force_accumulator.py: np.zeros(nv) then `result += value` per
    // component, in order — one rounded add per element per component.
    // acc.add's validation (force_accumulator.py:20-22) runs BEFORE the
    // component joins the fold; same message so pytest.raises can match.
    std::vector<double> result(static_cast<std::size_t>(m_->nv), 0.0);
    for (const std::vector<double> &c: components) {
        if (c.size() != result.size() ||
            !std::ranges::all_of(
                c, [](double v) { return std::isfinite(v); }))
            throw std::invalid_argument("invalid generalized force");
        for (std::size_t i = 0; i < result.size(); ++i)
            result[i] += c[i];
    }
    return result;
}
