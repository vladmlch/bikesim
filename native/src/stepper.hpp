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
class BrakeWriter;
class ResistanceWriter;
class TireWriter;
struct TireSideInput;

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
    // d.ctrl is an actuator INPUT buffer (nu wide) — the brake writer's
    // output surface; kept readable so tests can verify the write path.
    [[nodiscard]] std::span<const double> ctrl() const {
        return std::views::counted(d_->ctrl, m_->nu); }
    [[nodiscard]] double time() const { return d_->time; }
    // compute_qfrc_components on the CURRENT mjData — insertion-ordered
    // (name, nv-vector) pairs, identical to the Python dict. Throws
    // std::logic_error when constructed without a suspension config.
    [[nodiscard]] std::vector<std::pair<std::string, std::vector<double>>>
        suspension_components() const;
    // BrakeController.compute — (front, rear) torques at the current qvel.
    [[nodiscard]] std::pair<double, double>
        brake_torques(double front_demand, double rear_demand) const;
    // compute() + the ctrl writes (ride_sim.py:530-534).
    void apply_brake(double front_demand, double rear_demand);
    // ExternalResistanceApplier.compute_components — 'road_rolling' +
    // 'aerodynamic' for the given per-side tire snapshots.
    [[nodiscard]] std::vector<std::pair<std::string, std::vector<double>>>
        resistance_components(const TireSideInput& front,
                              const TireSideInput& rear) const;
    // TireForceApplier.compute_qfrc advance=True on the CURRENT mjData —
    // restores no brush state on its own; call set_tire_state first when
    // replaying. Throws std::logic_error without a tire config.
    [[nodiscard]] std::vector<double> tire_qfrc(double dt);
    // The artifact's flattened brush-state encoding: names follow the
    // flatten_row schema ('front.tangent.0' style; leaf 'front.tangent'
    // columns also decode, all-NaN vector -> unset field). Restoring
    // clears the once-per-timestamp clock like `last_time_s = None`.
    void set_tire_state(std::span<const std::string> names,
                        std::span<const double> row);
    [[nodiscard]] std::vector<double> tire_state() const;
    [[nodiscard]] const std::vector<std::string>&
        tire_state_names() const;
    // The writer itself — snapshots()/diagnostics() are read here by the
    // binding. nullptr without a tire config (callers gate on it).
    [[nodiscard]] const TireWriter* tire() const { return tire_.get(); }
private:
    mjModel* m_;
    mjData* d_;
    std::unique_ptr<SuspensionWriter> suspension_;
    std::unique_ptr<BrakeWriter> brake_;
    std::unique_ptr<ResistanceWriter> resistance_;
    std::unique_ptr<TireWriter> tire_;
};
