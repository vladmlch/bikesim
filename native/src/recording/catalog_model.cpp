#include "catalog.hpp"
#include <mujoco/mujoco.h>
#include <ranges>
#include <stdexcept>

namespace bike::recording {
Catalog build_catalog(const mjModel& model) {
    std::vector<JointCoords> joints;
    joints.reserve(static_cast<std::size_t>(model.njnt));
    const auto types = std::views::counted(model.jnt_type, model.njnt);
    const auto qposadr = std::views::counted(model.jnt_qposadr, model.njnt);
    const auto dofadr = std::views::counted(model.jnt_dofadr, model.njnt);
    for (int i = 0; i < model.njnt; ++i) {
        const auto index = static_cast<std::size_t>(i);
        const char* name = mj_id2name(&model, mjOBJ_JOINT, i);
        if (!name) throw std::invalid_argument("recording model has unnamed joint " + std::to_string(i));
        std::string type;
        switch (types[index]) {
            case mjJNT_FREE: type = "free"; break;
            case mjJNT_BALL: type = "ball"; break;
            case mjJNT_SLIDE: type = "slide"; break;
            case mjJNT_HINGE: type = "hinge"; break;
            default: throw std::invalid_argument("recording model has unknown joint type");
        }
        joints.push_back(JointCoords{.name=name, .joint_type=std::move(type), .qposadr=qposadr[index], .dofadr=dofadr[index]});
    }
    return build_catalog(joints);
}
} // namespace bike::recording
