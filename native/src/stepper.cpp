#include "stepper.hpp"
#include "cblas_abi.hpp"
#include "engine_call.hpp"
#include "validation.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include "writer_config.hpp"
#include "writers/brake.hpp"
#include "writers/cruise.hpp"
#include "writers/resistance.hpp"
#include "writers/rider_contacts.hpp"
#include "writers/rider_forces.hpp"
#include "writers/suspension.hpp"
#include "writers/tire.hpp"
#include "writers/distributed_tire.hpp"
#include "writers/pneumatic_tire.hpp"
#include "writers/pneumatic_3d_tire.hpp"
#include "writers/spatial_tire_adapter.hpp"
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

namespace stepper_detail {
    ForeignTimeCallbackProbe &foreign_time_callback_probe() noexcept {
        // Function-local static: the bindings' module init installs the
        // recognizer before any Stepper is built; a plain store is the
        // whole contract (no post-publish mutation).
        static ForeignTimeCallbackProbe installed = nullptr;
        return installed;
    }
} // namespace stepper_detail

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
    init_engine_data(std::move(model));
}

Stepper::Stepper(std::span<const std::byte> image) : m_(nullptr), d_(nullptr) {
    require_unmarked_time_callback();
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
        engine::load_model_image(image),
        &mj_deleteModel);
    if (!model)
        throw std::runtime_error("mj_loadModel failed on the model image");
    init_engine_data(std::move(model));
}

Stepper::Stepper(const std::string &mjb_path,
                 const nativecfg::WriterConfig &config)
    : Stepper(mjb_path) {
    install_writers(config);
}

Stepper::Stepper(std::span<const std::byte> image,
                 const nativecfg::WriterConfig &config)
    : Stepper(image) {
    install_writers(config);
}

void Stepper::init_engine_data(
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model) {
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(
        engine::make_data(model.get()), &mj_deleteData);
    if (!data) throw std::runtime_error("mj_makeData");
    engine::forward(model.get(), data.get());
    m_ = model.release();
    d_ = data.release();
    posvel_fresh_ = true;
}

void Stepper::install_writers(const nativecfg::WriterConfig &config) {
    // Decode happened at the boundary (nativecfg::decode for the owned
    // Value wire, nativecfg::writer_config_from_handle for the nanobind
    // adapter) BEFORE this constructor runs — the input here is already
    // the typed WriterConfig. An all-absent config installs nothing (the
    // Stepper(path) contract). Writer-construction failure must not leak
    // m_/d_: ~Stepper never runs for an object whose constructor throws,
    // and an early-out leaves the writers' owned state exactly as the
    // unconfigured ctor left it.
    const nativecfg::WriterConfig &cfg = config;
    try {
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
            // The backend string selects the writer; each writer's own
            // ctor re-validates the section it consumes (TireWriter
            // rejects a distributed-only config and vice versa).
            if (cfg.tire->backend == "distributed_2d_reference")
                distributed_tire_ = std::make_unique<DistributedTireWriter>(
                        m_, d_, *cfg.tire);
            else if (cfg.tire->backend == "pneumatic")
                pneumatic_tire_ = std::make_unique<PneumaticTireWriter>(
                        m_, d_, *cfg.tire);
            else
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
        if (cfg.rider_contacts) {
            rider_contacts_ = std::make_unique<writers::RiderContactWriter>(
                m_, *cfg.rider_contacts);
            // Construction-sized convenience scratch — the boxed
            // rider_contacts_qfrc reroutes through compute_qfrc_into like
            // tire_qfrc; the writer's anchors stay uninitialized until the
            // first rider_contacts_reset (the oracle's reset(model, None)).
            rider_contacts_out_.resize(static_cast<std::size_t>(m_->nv));
        }
    } catch (...) {
        rider_contacts_.reset();
        drive_.reset();
        mj_deleteData(d_);
        mj_deleteModel(m_);
        d_ = nullptr;
        m_ = nullptr;
        throw;
    }
}

void Stepper::install_spatial_tire(const bike::spatial::Config& config,
    std::shared_ptr<const bike::terrain::Terrain3dSource> terrain,
    bike::pneumatic3d::Pneumatic3dSpec spec,
    bike::pneumatic3d::TyreStateIdentity3d identity) {
    require_healthy();
    if(config.mode!=bike::spatial::Mode::spatial3d || !terrain)
        throw std::invalid_argument("stepper.spatial.tire: spatial configuration and terrain required");
    if(has_tire() || has_spatial_tire())
        throw std::invalid_argument("stepper.spatial.tire: a contact producer is already installed");
    const auto topology=bike::spatial::bind_topology(*m_,config);
    if(identity.topology_fingerprint_hex!=topology.fingerprint_hex ||
       identity.terrain_sha256_hex!=terrain->fingerprint_hex() ||
       identity.calibration_sha256_hex!=config.tyre.lateral_coefficients_sha256)
        throw std::invalid_argument("stepper.spatial.tire.identity: differs from bound owners");
    auto producer=std::make_unique<bike::pneumatic3d::Pneumatic3dTireWriter>(
        *m_,topology,std::move(spec),*terrain,std::move(identity));
    auto initial=producer->state();
    auto projection=std::make_unique<SpatialTireAdapter>(*m_,topology,*producer,terrain->query_view());
    tire_out_.resize(static_cast<std::size_t>(m_->nv));
    // All validation/allocation is complete before the ownership commit.
    initial_spatial_tire_.emplace(std::move(initial));
    spatial_terrain_=std::move(terrain);
    spatial_tire_=std::move(producer);
    spatial_tire_view_=std::move(projection);
}

bike::pneumatic3d::Pneumatic3dTireWriter& Stepper::spatial_tire_writer() const {
    require_healthy();
    if(!spatial_tire_)
        throw std::invalid_argument("bootstrap.spatial.tire: producer state missing");
    return *spatial_tire_;
}

void Stepper::restore_spatial_tire(const bike::pneumatic3d::TyreState3d& state) {
    auto& producer=spatial_tire_writer();
    if(producer.restore(state)!=CoreStatus::ok)
        throw std::invalid_argument("bootstrap.spatial.tire: producer state rejected");
    spatial_tire_view_->invalidate_projection();
}

Stepper::~Stepper() {
    spatial_tire_view_.reset();
    spatial_tire_.reset();
    rider_contacts_.reset();
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
    // The engine boundary classifies the installed timer by address alone
    // (T04 provenance): no callback or the documented native timer are
    // accepted; every other owner is foreign at this layer. Only the
    // foreign case needs the bindings adapter's vendor-Python recognition
    // below — it stays reachable until T63, inside the bindings build.
    switch (engine::time_callback_provenance()) {
    case engine::TimeCallbackProvenance::none:
        record(nullptr, true);
        return;
    case engine::TimeCallbackProvenance::native:
        // engine::native_time — warm the epoch's static guard outside the
        // marked regions, the same reason the vendored timers are called
        // once below before their first marked step.
        static_cast<void>(engine::native_time());
        record(current, true);
        return;
    case engine::TimeCallbackProvenance::foreign:
        break;
    }

    // Foreign owner: classifying it needs the vendor Python runtime
    // (mujoco.get_mjcb_time / the _structs/_simulate vendored timers), so
    // the recognizer lives on the bindings side and is reached through
    // stepper_detail::foreign_time_callback_probe(). With no interpreter
    // the slot stays null and every foreign callback is unsafe.
    if (const auto probe = stepper_detail::foreign_time_callback_probe()) {
        const stepper_detail::ForeignTimeCallbackVerdict verdict =
            probe(current);
        record(verdict.allowed, verdict.unmarked_safe);
        return;
    }
    record(nullptr, false);
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
    if (succeeded) posvel_fresh_ = false;
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
    if (succeeded) posvel_fresh_ = true;
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
        posvel_fresh_ = false;
        data_reset = true;
        engine::forward(m_, d_);
        posvel_fresh_ = true;
        if (drive_) drive_->reset();
        if (tire_) tire_->reset();
        if (distributed_tire_) distributed_tire_->reset();
        if (pneumatic_tire_) pneumatic_tire_->reset();
        if (spatial_tire_view_) spatial_tire_view_->reset();
        if (cruise_) cruise_->reset();
        // reset(model, data) on the post-forward pose — re-anchors grips and
        // revalidates the planar support topology like the oracle applier.
        if (rider_contacts_) rider_contacts_->reset(d_);
        // A writer reset may still commit live-model transmission writes.
        if (drive_ && drive_->take_model_dirty()) posvel_fresh_ = false;
        poisoned_ = false;
    } catch (const engine::EngineFailure &) {
        poisoned_ = true;
        throw;
    } catch (...) {
        if (data_reset) poisoned_ = true;
        throw;
    }
}

namespace {
    constexpr const char *kNoTire =
            "tire writer: Stepper was built without a tire config (pass the dict "
            "from tools.native_config.project)";
} // namespace

drivetrain::DrivetrainWriter &Stepper::drive() const {
    if (!drive_)
        throw std::logic_error(
            "drive: Stepper was built without a drive config (pass the dict "
            "from tools.native_config.project)");
    return *drive_;
}

// ---- A3 owned-writer access ----------------------------------------------
// Each accessor mirrors drive()/require_cruise(): std::logic_error naming
// the writer when its config section was absent at construction.

SuspensionWriter &Stepper::suspension() const {
    if (!suspension_)
        throw std::logic_error(
            "suspension: Stepper was built without a suspension config (pass "
            "the dict from tools.native_config.project)");
    return *suspension_;
}

ResistanceWriter &Stepper::resistance() const {
    if (!resistance_)
        throw std::logic_error(
            "resistance: Stepper was built without a resistance config (pass "
            "the dict from tools.native_config.project)");
    return *resistance_;
}

RiderForcesWriter &Stepper::rider_forces() const {
    if (!rider_forces_)
        throw std::logic_error(
            "rider_forces: Stepper was built without a rider_forces config "
            "(pass the dict from tools.native_config.project)");
    return *rider_forces_;
}

TireWriterView Stepper::tire_writer() const {
    if (tire_) return TireWriterView(tire_.get());
    if (distributed_tire_) return TireWriterView(distributed_tire_.get());
    if (pneumatic_tire_) return TireWriterView(pneumatic_tire_.get());
    if (spatial_tire_view_) return TireWriterView(spatial_tire_view_.get());
    throw std::logic_error(kNoTire);
}

// ---- TireWriterView ---------------------------------------------------
// One std::visit forward per method: the pointer variant selects the
// engaged backend and both writers implement this surface identically.
// The calls add one indirect dispatch, never an allocation.

void TireWriterView::compute_into(const mjData *d, double dt,
                                  std::span<double> out) const {
    std::visit([&](auto *w) { w->compute_into(d, dt, out); }, writer_);
}

CoreStatus TireWriterView::try_compute_into(
        const mjData *d, double dt, std::span<double> out) const {
    return std::visit(
            [&](auto *w) { return w->try_compute_into(d, dt, out); },
            writer_);
}

std::vector<double> TireWriterView::qfrc(const mjData *d,
                                         double dt) const {
    return std::visit([&](auto *w) { return w->qfrc(d, dt); }, writer_);
}

void TireWriterView::probe_compute_into(const mjData *d, double dt,
                                        std::span<double> out) const {
    std::visit([&](auto *w) { w->probe_compute_into(d, dt, out); },
               writer_);
}

std::vector<double> TireWriterView::probe_qfrc(const mjData *d,
                                               double dt) const {
    return std::visit([&](auto *w) { return w->probe_qfrc(d, dt); },
                      writer_);
}

std::array<std::optional<TireSnapshot>, 2>
TireWriterView::snapshots() const {
    return std::visit([&](auto *w) { return w->snapshots(); }, writer_);
}

std::array<std::optional<TireSnapshot>, 2>
TireWriterView::probe_snapshots() const {
    return std::visit([&](auto *w) { return w->probe_snapshots(); },
                      writer_);
}

std::array<std::optional<TireDiagnostics>, 2>
TireWriterView::diagnostics() const {
    return std::visit([&](auto *w) { return w->diagnostics(); }, writer_);
}

const std::array<TireSnapshot, 2> &
TireWriterView::snapshot_storage() const {
    return std::visit(
            [&](auto *w) -> const auto & { return w->snapshot_storage(); },
            writer_);
}

bool TireWriterView::snapshots_committed() const {
    return std::visit(
            [&](auto *w) { return w->snapshots_committed(); }, writer_);
}

const std::array<TireDiagnostics, 2> &
TireWriterView::diagnostic_storage() const {
    return std::visit([&](auto *w) -> const auto & {
        return w->diagnostic_storage();
    }, writer_);
}

const std::array<TireSnapshot, 2> &
TireWriterView::probe_snapshot_storage() const {
    return std::visit([&](auto *w) -> const auto & {
        return w->probe_snapshot_storage();
    }, writer_);
}

bool TireWriterView::probe_committed() const {
    return std::visit(
            [&](auto *w) { return w->probe_committed(); }, writer_);
}

const std::array<double, 2> &TireWriterView::radii() const {
    return std::visit(
            [&](auto *w) -> const auto & { return w->radii(); }, writer_);
}

double TireWriterView::stored_energy(const mjData *d) const {
    return std::visit([&](auto *w) { return w->stored_energy(d); },
                      writer_);
}

double TireWriterView::elastic_energy_j() const {
    return std::visit(
            [&](auto *w) { return w->elastic_energy_j(); }, writer_);
}

double TireWriterView::brush_loss_step_j() const {
    return std::visit(
            [&](auto *w) { return w->brush_loss_step_j(); }, writer_);
}

double TireWriterView::radial_dissipation_power_w() const {
    return std::visit([&](auto *w) {
        return w->radial_dissipation_power_w();
    }, writer_);
}

void TireWriterView::set_state(std::span<const std::string> names,
                               std::span<const double> row) const {
    std::visit([&](auto *w) { w->set_state(names, row); }, writer_);
}

void TireWriterView::prepare_state(std::span<const std::string> names,
                                  std::span<const double> row) const {
    std::visit([&](auto *w) { w->prepare_state(names, row); }, writer_);
}

// NOLINTNEXTLINE(bugprone-exception-escape) entry point lets failures surface as the reference does
void TireWriterView::commit_prepared_state() const noexcept {
    std::visit([](auto *w) { w->commit_prepared_state(); }, writer_);
}

std::vector<double> TireWriterView::state() const {
    return std::visit([&](auto *w) { return w->state(); }, writer_);
}

const std::vector<std::string> &TireWriterView::state_names() const {
    return std::visit(
            [&](auto *w) -> const auto & { return w->state_names(); },
            writer_);
}

void TireWriterView::reset() const {
    std::visit([](auto *w) { w->reset(); }, writer_);
}

void TireWriterView::restart_clock() const {
    std::visit([](auto *w) { w->restart_clock(); }, writer_);
}

void TireWriterView::clear_settled_loss() const {
    std::visit([](auto *w) { w->clear_settled_loss(); }, writer_);
}

bool TireWriterView::is_distributed() const noexcept {
    return std::holds_alternative<DistributedTireWriter *>(writer_);
}

bool TireWriterView::is_pneumatic() const noexcept {
    return std::holds_alternative<PneumaticTireWriter *>(writer_);
}
bool TireWriterView::is_spatial() const noexcept {
    return std::holds_alternative<SpatialTireAdapter *>(writer_);
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
    for (int joint=0; joint<m_->njnt; ++joint) {
        const int type=m_->jnt_type[joint];
        if(type!=mjJNT_FREE && type!=mjJNT_BALL) continue;
        const auto start=static_cast<std::size_t>(m_->jnt_qposadr[joint])+
            (type==mjJNT_FREE?3U:0U);
        if(start>qpos.size() || qpos.size()-start<4)
            throw std::invalid_argument("set_state.qpos.quaternion: span escapes model");
        const double norm=std::hypot(std::hypot(qpos[start],qpos[start+1]),
            std::hypot(qpos[start+2],qpos[start+3]));
        if(std::abs(norm-1.)>1e-9)
            throw std::invalid_argument("set_state.qpos.quaternion: expected unit quaternion");
    }
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
    // reset_data + raw buffer writes: every derived stage is stale until
    // the next full forward (the oracle's trailing mj_forward is deferred).
    posvel_fresh_ = false;
    if (recovering) {
        try {
            if (drive_) drive_->reset();
            if (tire_) tire_->reset();
            if (distributed_tire_) distributed_tire_->reset();
            if (pneumatic_tire_) pneumatic_tire_->reset();
            if (spatial_tire_view_) spatial_tire_view_->reset();
            if (cruise_) cruise_->reset();
            if (rider_contacts_) rider_contacts_->reset(d_);
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

std::vector<double> Stepper::tire_qfrc(double dt) {
    require_healthy();
    // compute_into writes the caller-owned construction-sized scratch —
    // same advance+commit pipeline as qfrc(), no writer-side copy. The
    // view forwards to whichever backend the config engaged (and throws
    // kNoTire when there is none).
    tire_writer().compute_into(d_, dt, tire_out_);
    return tire_out_;
}

void Stepper::set_tire_state(std::span<const std::string> names,
                             std::span<const double> row) {
    require_healthy();
    tire_writer().set_state(names, row);
}

std::vector<double> Stepper::tire_state() const {
    return tire_writer().state();
}

double Stepper::kinetic_energy() const {
    require_healthy();
    std::vector<mjtNum> mv(static_cast<std::size_t>(m_->nv));
    engine::mul_m(m_, d_, mv.data(), d_->qvel);
    // data.qvel @ mv — np.dot dispatches to the pinned CBLAS ABI; the
    // typed wrapper widens to the provider's integer width internally.
    return .5 * blas::ddot(static_cast<int>(m_->nv), d_->qvel, 1,
                         mv.data(), 1);
}

double Stepper::tire_stored_energy() const {
    require_healthy();
    return tire_writer().stored_energy(d_);
}

double Stepper::tire_brush_loss_step_j() const {
    require_healthy();
    return tire_writer().brush_loss_step_j();
}

double Stepper::tire_radial_dissipation_power_w() const {
    require_healthy();
    return tire_writer().radial_dissipation_power_w();
}

const std::vector<std::string> &Stepper::tire_state_names() const {
    // The no-tire case keeps the historical compliant schema — the
    // binding documents state_names as the default artifact's names.
    if (tire_) return tire_->state_names();
    if (distributed_tire_) return distributed_tire_->state_names();
    if (pneumatic_tire_) return pneumatic_tire_->state_names();
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

writers::RiderContactWriter &Stepper::rider_contacts() const {
    if (!rider_contacts_)
        throw std::logic_error(
            "rider_contacts: Stepper was built without a rider_contacts "
            "config (pass the dict from tools.native_config.project)");
    return *rider_contacts_;
}

void Stepper::rider_contacts_reset() {
    require_healthy();
    rider_contacts().reset(d_);
}

void Stepper::rider_contacts_restart_clock() {
    require_healthy();
    rider_contacts().restart_clock();
}

void Stepper::rider_contacts_initialize_settled_state() {
    require_healthy();
    rider_contacts().initialize_settled_state(d_);
}

void Stepper::rider_contacts_release_all() {
    require_healthy();
    rider_contacts().release_all();
}

bool Stepper::rider_contacts_set_enabled(std::string_view name,
                                         bool enabled) {
    require_healthy();
    return rider_contacts().set_enabled(name, enabled);
}

double Stepper::rider_contacts_stored_energy() const {
    // A read on the current arena — like suspension_components() it stays
    // available after a fatal engine error poisons the mutation paths.
    return rider_contacts().stored_energy(d_);
}

std::vector<double> Stepper::rider_contacts_qfrc(double dt, bool advance,
                                                 bool detailed) {
    require_healthy();
    rider_contacts().compute_qfrc_into(d_, dt, advance, detailed,
                                       rider_contacts_out_);
    return rider_contacts_out_;
}

writers::RiderPreparedAttachments
Stepper::rider_contacts_prepare_attachment_raw() const {
    // Pose-only read — the same non-mutating contract as
    // rider_contacts_stored_energy.
    return rider_contacts().prepare_attachment_raw(d_);
}

writers::RiderAttachmentMeasurement
Stepper::rider_contacts_attachment_samples(
    const std::optional<writers::RiderIntervalState> &interval_state,
    bool raw) {
    require_healthy();
    return rider_contacts().attachment_samples(d_, interval_state, raw);
}

writers::RiderSettleOutcome Stepper::rider_contacts_settle(
    const std::optional<writers::RiderIntervalState> &interval_state,
    bool raw,
    const writers::RiderPreparedAttachments *prepared) {
    require_healthy();
    return rider_contacts().settle_welds(d_, interval_state, raw,
                                         prepared);
}
