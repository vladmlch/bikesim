#include "stepper.hpp"
#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include "config.hpp"
#include "writers/brake.hpp"
#include "writers/resistance.hpp"
#include "writers/suspension.hpp"
#include "writers/tire.hpp"

namespace {
// Same check for every incoming span — one place only (the binding converts
// ndarray→span and forwards; widths live on the model, which only the core
// can see). Width is mjtSize (int64) in mjModel; the cast is the explicit,
// always-safe direction (a negative width can never equal a size, so it
// still throws).
void check_width(std::span<const double> src, const char* name,
                 mjtSize width) {
    if (src.size() != static_cast<std::size_t>(width))
        throw std::invalid_argument(
            "set_state: " + std::string(name) + " has " +
            std::to_string(src.size()) + " elements; model expects " +
            std::to_string(width));
}
// Empty spans are skipped, not memcpy'd: d_->act is nullptr when na==0.
void copy_in(std::span<const double> src, mjtNum* dst) {
    if (!src.empty())
        std::ranges::copy(src, dst);
}
} // namespace

Stepper::Stepper(const std::string& mjb_path) {
    m_ = mj_loadModel(mjb_path.c_str(), nullptr);   // mjb: no VFS needed
    if (!m_) throw std::runtime_error("mj_loadModel failed: " + mjb_path);
    d_ = mj_makeData(m_);
    if (!d_) { mj_deleteModel(m_); throw std::runtime_error("mj_makeData"); }
    mj_forward(m_, d_);
}

Stepper::Stepper(const std::string& mjb_path, const nanobind::dict& config)
    : Stepper(mjb_path) {
    // Config-parse or writer-construction failure must not leak m_/d_:
    // ~Stepper never runs for an object whose constructor throws.
    try {
        if (config.empty())
            return;   // no sections → every writer stays disabled
        nativecfg::NativeConfig cfg =
            nativecfg::native_config_from_dict(config);
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
    } catch (...) {
        mj_deleteData(d_);
        mj_deleteModel(m_);
        d_ = nullptr;
        m_ = nullptr;
        throw;
    }
}

Stepper::~Stepper() { if (d_) mj_deleteData(d_); if (m_) mj_deleteModel(m_); }

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
    mj_resetData(m_, d_);
    copy_in(qpos, d_->qpos);
    copy_in(qvel, d_->qvel);
    copy_in(act, d_->act);
    copy_in(warmstart, d_->qacc_warmstart);
    d_->time = time;
}

std::vector<std::pair<std::string, std::vector<double>>>
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

std::vector<std::pair<std::string, std::vector<double>>>
Stepper::resistance_components(const TireSideInput& front,
                               const TireSideInput& rear) const {
    if (!resistance_)
        throw std::logic_error(
            "resistance_components: Stepper was built without a resistance "
            "config (pass the dict from tools.native_config.project)");
    return resistance_->components(d_, front, rear);
}

namespace {
constexpr const char* kNoTire =
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

const std::vector<std::string>& Stepper::tire_state_names() const {
    return TireWriter::state_names();
}
