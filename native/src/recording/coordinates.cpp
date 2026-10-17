#include "catalog.hpp"
#include <array>
#include <limits>
#include <stdexcept>

namespace bike::recording {
std::string CoordinateCatalog::force_unit(std::size_t index) const {
    const auto& kind = dof.at(index).dof_kind;
    if (kind == "translation") return "N";
    if (kind == "rotation") return "N·m";
    return "n/a";
}
const std::string& CoordinateCatalog::velocity_unit(std::size_t index) const { return dof.at(index).unit; }
Catalog build_catalog(std::span<const JointCoords> joints, std::span<const ScalarAlias> scalars) {
    CoordinateCatalog coordinates;
    for (const auto& joint : joints) {
        if (joint.name.empty() || joint.qposadr < 0 || joint.dofadr < 0) throw std::invalid_argument("invalid recording joint coordinates");
        std::vector<std::string> qmembers, dmembers, qkinds, dkinds;
        if (joint.joint_type == "free") {
            qmembers = {"x", "y", "z", "qw", "qx", "qy", "qz"}; dmembers = {"vx", "vy", "vz", "wx", "wy", "wz"};
            qkinds = {"translation", "translation", "translation", "quaternion", "quaternion", "quaternion", "quaternion"};
            dkinds = {"translation", "translation", "translation", "rotation", "rotation", "rotation"};
        } else if (joint.joint_type == "ball") {
            qmembers = {"qw", "qx", "qy", "qz"}; dmembers = {"wx", "wy", "wz"};
            qkinds = {"quaternion", "quaternion", "quaternion", "quaternion"}; dkinds = {"rotation", "rotation", "rotation"};
        } else if (joint.joint_type == "hinge" || joint.joint_type == "slide") {
            qmembers = {""}; dmembers = {""}; qkinds = {joint.joint_type == "hinge" ? "rotation" : "translation"}; dkinds = qkinds;
        } else throw std::invalid_argument(joint.name + ": unknown joint type");
        if (joint.qposadr != static_cast<std::int64_t>(coordinates.qpos.size()) || joint.dofadr != static_cast<std::int64_t>(coordinates.dof.size()))
            throw std::invalid_argument("qpos/DOF addresses must be dense from zero");
        for (std::size_t i = 0; i < qmembers.size(); ++i) coordinates.qpos.push_back(Coordinate{static_cast<std::int64_t>(coordinates.qpos.size()), "qpos", joint.name, qmembers[i], qkinds[i], qkinds[i] == "translation" ? "m" : qkinds[i] == "rotation" ? "rad" : "1"});
        for (std::size_t i = 0; i < dmembers.size(); ++i) coordinates.dof.push_back(Coordinate{static_cast<std::int64_t>(coordinates.dof.size()), "dof", joint.name, dmembers[i], dkinds[i], dkinds[i] == "translation" ? "m/s" : "rad/s"});
    }
    std::vector<AliasSpec> aliases;
    for (const auto& scalar : scalars) {
        if (scalar.kind != "qpos" && scalar.kind != "dof") throw std::invalid_argument("scalar alias must bind qpos or dof");
        const auto& sequence = scalar.kind == "qpos" ? coordinates.qpos : coordinates.dof;
        std::optional<std::int64_t> index;
        for (const auto& c : sequence) if (c.joint == scalar.joint) {
            if (index) throw std::invalid_argument("scalar alias needs exactly one coordinate: " + scalar.alias);
            index = c.index;
        }
        if (!index) throw std::invalid_argument("scalar alias coordinate missing: " + scalar.alias);
        aliases.push_back(AliasSpec{scalar.alias, scalar.kind == "qpos" ? "physics.qpos" : "physics.qvel", index, {}, "legacy scalar column bound by meaning"});
    }
    for (const auto& c : coordinates.qpos) for (const auto& name : {"qpos." + std::to_string(c.index), "prestep_qpos_" + std::to_string(c.index)})
        aliases.push_back(AliasSpec{name, "physics.qpos", c.index, {}, "flattened schema-2 element name"});
    for (const auto& c : coordinates.dof) for (const auto& name : {"qvel." + std::to_string(c.index), "prestep_qvel_" + std::to_string(c.index)})
        aliases.push_back(AliasSpec{name, "physics.qvel", c.index, {}, "flattened schema-2 element name"});
    auto alias = [&](std::string name, std::string canonical, std::string note, std::optional<std::int64_t> index = {}) {
        aliases.push_back(AliasSpec{std::move(name), std::move(canonical), index, {}, std::move(note)});
    };
    alias("interval_start_s", "physics.time_s", "legacy interval-start alias");
    alias("interval_dt_s", "physics.dt_s", "legacy interval-duration alias");
    alias("compiled_mass_kg", "channel_mass.mass_kg", "PhysicalRecorder compiled-mass column");
    alias("kinetic_energy_j", "channel_mass.kinetic_energy_j", "PhysicalRecorder mass energy column");
    alias("gravitational_energy_j", "channel_mass.gravitational_energy_j", "PhysicalRecorder mass energy column");
    alias("mechanical_energy_j", "channel_energy.mechanical_energy_j", "PhysicalRecorder energy column");
    alias("energy_residual_j", "channel_energy.residual_j", "PhysicalRecorder energy residual column");
    const std::array<std::string, 3> axes{"x", "y", "z"};
    for (std::size_t i = 0; i < axes.size(); ++i) alias("compiled_com_" + axes[i] + "_m", "channel_mass.com_m", "PhysicalRecorder compiled-CoM column", static_cast<std::int64_t>(i));
    alias("powers_w.{component}", "forces.power_w", "schema-2 power object member");
    alias("power_{component}_w", "forces.power_w", "PhysicalRecorder power column");
    alias("{component}_power_w", "forces.power_w", "PhysicalRecorder power column");
    alias("{component}_work_j", "channel_energy.component_work_j", "PhysicalRecorder work column");
    alias("component_work_j.{component}", "channel_energy.component_work_j", "schema-2 work object member");
    return Catalog(schema_catalog().table_order(), std::move(coordinates), std::move(aliases));
}
Catalog build_catalog(std::span<const JointCoords> joints) {
    static const std::array<ScalarAlias, 3> aliases{{{"x_m", "qpos", "root_x"}, {"speed_mps", "dof", "root_x"}, {"pitch_rad", "qpos", "root_pitch"}}};
    return build_catalog(joints, aliases);
}
} // namespace bike::recording
