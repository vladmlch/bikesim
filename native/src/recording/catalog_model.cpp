#include "catalog.hpp"
#include <mujoco/mujoco.h>
#include <stdexcept>

namespace bike::recording {
Catalog build_catalog(const mjModel& model) {
    std::vector<JointCoords> joints;
    joints.reserve(static_cast<std::size_t>(model.njnt));
    for (int i = 0; i < model.njnt; ++i) {
        const char* name = mj_id2name(&model, mjOBJ_JOINT, i);
        if (!name) throw std::invalid_argument("recording model has unnamed joint " + std::to_string(i));
        std::string type;
        switch (model.jnt_type[i]) {
            case mjJNT_FREE: type = "free"; break;
            case mjJNT_BALL: type = "ball"; break;
            case mjJNT_SLIDE: type = "slide"; break;
            case mjJNT_HINGE: type = "hinge"; break;
            default: throw std::invalid_argument("recording model has unknown joint type");
        }
        joints.push_back(JointCoords{name, std::move(type), model.jnt_qposadr[i], model.jnt_dofadr[i]});
    }
    return build_catalog(joints);
}
} // namespace bike::recording
