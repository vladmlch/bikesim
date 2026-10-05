#include "stepper.hpp"
#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include "config.hpp"
#include "writers/suspension.hpp"

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
