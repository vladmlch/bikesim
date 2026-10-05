// stepper.hpp — owns mjModel/mjData; buffers never escape ownership.
#pragma once
#include <mujoco/mujoco.h>
#include <ranges>
#include <span>
#include <string>

class Stepper {
public:
    explicit Stepper(const std::string& mjb_path);
    ~Stepper();
    Stepper(const Stepper&) = delete;
    Stepper& operator=(const Stepper&) = delete;
    void step() { mj_step(m_, d_); }
    // views::counted instead of the (ptr, size) span ctor: clang's
    // -Wunsafe-buffer-usage-in-container rejects the raw two-argument form.
    // nq/nv are int — counted takes iter_difference_t (signed); passing
    // size_t here trips GCC's -Wsign-conversion, so no cast.
    [[nodiscard]] std::span<const double> qpos() const {
        return std::views::counted(d_->qpos, m_->nq); }
    [[nodiscard]] std::span<const double> qvel() const {
        return std::views::counted(d_->qvel, m_->nv); }
    [[nodiscard]] double time() const { return d_->time; }
private:
    mjModel* m_;
    mjData* d_;
};
