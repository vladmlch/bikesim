#include "stepper.hpp"
#include <stdexcept>

Stepper::Stepper(const std::string& mjb_path) {
    m_ = mj_loadModel(mjb_path.c_str(), nullptr);   // mjb: no VFS needed
    if (!m_) throw std::runtime_error("mj_loadModel failed: " + mjb_path);
    d_ = mj_makeData(m_);
    if (!d_) { mj_deleteModel(m_); throw std::runtime_error("mj_makeData"); }
    mj_forward(m_, d_);
}

Stepper::~Stepper() { if (d_) mj_deleteData(d_); if (m_) mj_deleteModel(m_); }
