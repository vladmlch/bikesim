#include "stepper.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include "config.hpp"
#include "writers/brake.hpp"
#include "writers/cruise.hpp"
#include "writers/resistance.hpp"
#include "writers/rider_forces.hpp"
#include "writers/suspension.hpp"
#include "writers/tire.hpp"
#include "writers/drivetrain.hpp"

namespace {
    // Same check for every incoming span — one place only (the binding converts
    // ndarray→span and forwards; widths live on the model, which only the core
    // can see). Width is mjtSize (int64) in mjModel; the cast is the explicit,
    // always-safe direction (a negative width can never equal a size, so it
    // still throws).
    void check_width(std::span<const double> src, const char *name,
                     mjtSize width) {
        if (src.size() != static_cast<std::size_t>(width))
            throw std::invalid_argument(
                "set_state: " + std::string(name) + " has " +
                std::to_string(src.size()) + " elements; model expects " +
                std::to_string(width));
    }

    // Empty spans are skipped, not memcpy'd: d_->act is nullptr when na==0.
    void copy_in(std::span<const double> src, mjtNum *dst) {
        if (!src.empty())
            std::ranges::copy(src, dst);
    }
} // namespace

Stepper::Stepper(const std::string &mjb_path) : m_(mj_loadModel(mjb_path.c_str(), nullptr)) {
    // mjb: no VFS needed
    if (!m_) throw std::runtime_error("mj_loadModel failed: " + mjb_path);
    d_ = mj_makeData(m_);
    if (!d_) {
        mj_deleteModel(m_);
        throw std::runtime_error("mj_makeData");
    }
    mj_forward(m_, d_);
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
        if (cfg.tire)
            tire_ = std::make_unique<TireWriter>(m_, d_, *cfg.tire);
        if (cfg.rider_forces)
            rider_forces_ =
                    std::make_unique<RiderForcesWriter>(m_, *cfg.rider_forces);
        if (cfg.drive)
            drive_ = std::make_unique<DrivetrainWriter>(m_, d_, *cfg.drive);
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

drivetrain::DrivetrainWriter &Stepper::drive() const {
    if (!drive_)
        throw std::logic_error("Stepper was built without a drive config");
    return *drive_;
}

void Stepper::set_inputs(std::span<const double> ctrl,
                         std::span<const double> force) {
    check_width(ctrl, "ctrl", m_->nu);
    check_width(force, "qfrc_applied", m_->nv);
    for (double const x: ctrl)
        if (!std::isfinite(x)) throw std::invalid_argument("non-finite ctrl");
    for (double const x: force)
        if (!std::isfinite(x))
            throw std::invalid_argument("non-finite qfrc_applied");
    const std::vector<double> saved_ctrl(ctrl.begin(), ctrl.end());
    const std::vector<double> saved_force(force.begin(), force.end());
    copy_in(saved_ctrl, d_->ctrl);
    copy_in(saved_force, d_->qfrc_applied);
}

CruiseWriter &Stepper::require_cruise() const {
    if (!cruise_)
        throw std::logic_error("Stepper was built without a cruise config");
    return *cruise_;
}

double Stepper::cruise_compute(bool rear_in_contact, bool traction_limited,
                               std::optional<bool> controller_grounded) {
    return require_cruise().compute(d_, rear_in_contact, traction_limited,
                                    controller_grounded);
}

void Stepper::cruise_reset() { require_cruise().reset(); }

void Stepper::cruise_set_target_speed(double value_kmh) {
    require_cruise().set_target_speed(value_kmh);
}

double Stepper::cruise_set_assist_compensation(double support_factor) {
    return require_cruise().set_assist_compensation(support_factor);
}

CruiseState Stepper::cruise_state() const { return require_cruise().state(); }

void Stepper::set_cruise_state(const CruiseState &state) {
    require_cruise().set_state(state);
}

void Stepper::set_state(std::span<const double> qpos,
                        std::span<const double> qvel,
                        std::span<const double> act,
                        std::span<const double> warmstart,
                        double time) {
    // All width checks first (strong guarantee), then the same sequence the
    // Python oracle runs: mj_resetData + buffer writes + mj_forward.
    check_width(qpos, "qpos", m_->nq);
    check_width(qvel, "qvel", m_->nv);
    check_width(act, "act", m_->na);
    check_width(warmstart, "warmstart", m_->nv);
    // Callers may pass our own views, including cross-buffer aliases (e.g.
    // qacc as warmstart). Snapshot EVERY input before resetting any mjData
    // buffer. Allocation failures also leave the old state intact.
    const std::vector<double> saved_qpos(qpos.begin(), qpos.end());
    const std::vector<double> saved_qvel(qvel.begin(), qvel.end());
    const std::vector<double> saved_act(act.begin(), act.end());
    const std::vector<double> saved_warmstart(warmstart.begin(), warmstart.end());
    mj_resetData(m_, d_);
    copy_in(saved_qpos, d_->qpos);
    copy_in(saved_qvel, d_->qvel);
    copy_in(saved_act, d_->act);
    copy_in(saved_warmstart, d_->qacc_warmstart);
    d_->time = time;
}

std::vector<std::pair<std::string, std::vector<double> > >
Stepper::suspension_components() const {
    if (!suspension_)
        throw std::logic_error(
            "suspension_components: Stepper was built without a suspension "
            "config (pass the dict from tools.native_config.project)");
    return suspension_->components(d_);
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
    return resistance_->components(d_, front, rear);
}

namespace {
    constexpr const char *kNoTire =
            "tire writer: Stepper was built without a tire config (pass the dict "
            "from tools.native_config.project)";
} // namespace

std::vector<double> Stepper::tire_qfrc(double dt) {
    if (!tire_)
        throw std::logic_error(kNoTire);
    return tire_->qfrc(d_, dt);
}

void Stepper::set_tire_state(std::span<const std::string> names,
                             std::span<const double> row) {
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
    if (!rider_forces_)
        throw std::logic_error(
            "rider_forces_qfrc: Stepper was built without a rider_forces "
            "config (pass the dict from tools.native_config.project)");
    return rider_forces_->qfrc(d_);
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
