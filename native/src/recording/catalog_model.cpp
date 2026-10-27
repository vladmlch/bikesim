#include "catalog.hpp"
#include <mujoco/mujoco.h>
#include <ranges>
#include <stdexcept>

namespace bike::recording {
Catalog build_catalog(const mjModel& model) { return build_catalog(model, CatalogVersion::planar1); }
Catalog build_catalog(const mjModel& model, CatalogVersion version) {
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
    if (version == CatalogVersion::planar1) return build_catalog(joints);
    if (version != CatalogVersion::spatial2) throw std::invalid_argument("unsupported recording catalog version");
    const auto base = build_catalog(joints, std::span<const ScalarAlias>{});
    auto coordinates = base.coordinates(); auto aliases = base.aliases();
    // Scalar joints (hinge/slide) have an empty component, so a plain
    // joint + "." + component would publish the trailing-dot alias "joint.".
    // The stored coordinate identity keeps its schema-1 component spelling.
    for (const auto& c : coordinates.qpos) aliases.push_back(AliasSpec{
        .alias=c.component.empty() ? c.joint : c.joint + "." + c.component,
        .canonical="physics.qpos", .index=c.index, .parameter={}, .note="joint-specific position coordinate"});
    for (const auto& c : coordinates.dof) {
        // Linear tangent components are "vx" in the coordinate catalog; the alias
        // spells the time derivative of the matching position member ("dx").
        const std::string_view member = c.component.starts_with('v') ? std::string_view(c.component).substr(1) : std::string_view(c.component);
        aliases.push_back(AliasSpec{.alias=c.joint + ".d" + std::string(member),
            .canonical="physics.qvel", .index=c.index, .parameter={}, .note="joint-specific tangent coordinate"});
    }
    return {schema_catalog(version).table_order(), std::move(coordinates), std::move(aliases)};
}
} // namespace bike::recording
