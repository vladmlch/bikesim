#pragma once
#include <mujoco/mujoco.h>
#include <array>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>

namespace topology {
    inline void scalar_joint(const mjModel *model, int id, const char *name) {
        if (id < 0 || id >= model->njnt) throw std::invalid_argument(std::string(name) + ": invalid joint ID");
        const std::span<const int> types = std::views::counted(model->jnt_type, model->njnt);
        const int kind = types[static_cast<std::size_t>(id)];
        if (kind != mjJNT_HINGE && kind != mjJNT_SLIDE)
            throw std::invalid_argument(std::string(name) + ": requires scalar hinge or slide joint");
    }
    inline void joint(const mjModel *model, int id, const char *name,
                      mjtJoint kind, const std::array<double, 3> &axis) {
        if (id < 0 || id >= model->njnt) throw std::invalid_argument(std::string(name) + ": invalid joint ID");
        const std::span<const int> types = std::views::counted(model->jnt_type, model->njnt);
        const std::span<const mjtNum> axes = std::views::counted(model->jnt_axis, 3 * model->njnt);
        const auto index = static_cast<std::size_t>(id);
        if (types[index] != kind) throw std::invalid_argument(std::string(name) + ": unsupported joint type");
        for (std::size_t i = 0; i < 3; ++i)
            if (axes[3 * index + i] != axis[i])
                throw std::invalid_argument(std::string(name) + ": unsupported joint axis");
    }

    inline void actuator(const mjModel *model, int id, const char *name, int joint_id) {
        if (id < 0 || id >= model->nu) throw std::invalid_argument(std::string(name) + ": missing actuator");
        const std::span<const int> types = std::views::counted(model->actuator_trntype, model->nu);
        const std::span<const int> targets = std::views::counted(model->actuator_trnid, 2 * model->nu);
        const auto index = static_cast<std::size_t>(id);
        if (types[index] != mjTRN_JOINT || targets[2 * index] != joint_id)
            throw std::invalid_argument(std::string(name) + ": actuator targets the wrong joint");
    }
}
