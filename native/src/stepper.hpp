// stepper.hpp — owns mjModel/mjData; buffers never escape ownership.
#pragma once
#include <mujoco/mujoco.h>
#include <memory>
#include <ranges>
#include <span>
#include <string>
#include <utility>
#include <vector>
#include <nanobind/nanobind.h>

class SuspensionWriter;

class Stepper {
public:
    explicit Stepper(const std::string& mjb_path);
    // Config-bridge ctor: the dict from tools.native_config.project(env)
    // (or an empty dict — every writer stays disabled). The dict is read
    // ONCE here; nothing keeps a reference into Python objects.
    Stepper(const std::string& mjb_path, const nanobind::dict& config);
    ~Stepper();
    Stepper(const Stepper&) = delete;
    Stepper& operator=(const Stepper&) = delete;
    void step() { mj_step(m_, d_); }
    void forward() { mj_forward(m_, d_); }
    // Restore a solver-relevant snapshot (qpos/qvel/act/qacc_warmstart/time)
    // exactly as `mj_resetData` + buffer writes do in Python. Widths are
    // checked against the model BEFORE reset — a bad argument must not
    // clobber previously restored state. Empty `act` is legal (na==0).
    void set_state(std::span<const double> qpos,
                   std::span<const double> qvel,
                   std::span<const double> act,
                   std::span<const double> warmstart,
                   double time);
    // views::counted instead of the (ptr, size) span ctor: clang's
    // -Wunsafe-buffer-usage-in-container rejects the raw two-argument form.
    // nq/nv are mjtSize (signed) — counted takes iter_difference_t (signed);
    // passing size_t here trips GCC's -Wsign-conversion, so no cast.
    [[nodiscard]] std::span<const double> qpos() const {
        return std::views::counted(d_->qpos, m_->nq); }
    [[nodiscard]] std::span<const double> qvel() const {
        return std::views::counted(d_->qvel, m_->nv); }
    [[nodiscard]] std::span<const double> qacc() const {
        return std::views::counted(d_->qacc, m_->nv); }
    [[nodiscard]] std::span<const double> qfrc_constraint() const {
        return std::views::counted(d_->qfrc_constraint, m_->nv); }
    // nefc is a RUNTIME mjData field — the view's size varies per forward.
    // (mjModel has no nefc member; do not look there.)
    [[nodiscard]] std::span<const double> efc_force() const {
        return std::views::counted(d_->efc_force, d_->nefc); }
    [[nodiscard]] double time() const { return d_->time; }
    // compute_qfrc_components on the CURRENT mjData — insertion-ordered
    // (name, nv-vector) pairs, identical to the Python dict. Throws
    // std::logic_error when constructed without a suspension config.
    [[nodiscard]] std::vector<std::pair<std::string, std::vector<double>>>
        suspension_components() const;
private:
    mjModel* m_;
    mjData* d_;
    std::unique_ptr<SuspensionWriter> suspension_;
};
