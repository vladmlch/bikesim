#pragma once

#include "pneumatic3d/brush.hpp"
#include "pneumatic3d/carcass.hpp"
#include "pneumatic3d/geometry.hpp"
#include "pneumatic3d/patches.hpp"
#include "pneumatic3d/state.hpp"
#include "spatial/types.hpp"
#include "terrain/source3d.hpp"
#include "writers/writer_types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace bike::pneumatic3d {

struct Pneumatic3dSpec {
    bike::spatial::TyreTier tier = {};
    std::array<double, 2> outer_radius_m{}, casing_width_m{};
    std::array<CarcassSpec3d, 2> carcass;
    std::array<BrushSpec3d, 2> brush;
    std::uint32_t patch_capacity = 0; // Zero selects the proved maximum.
};

// Engine-independent kernels. Cold construction sizes all hot-path buffers.
// The terrain view's immutable owner must outlive this model.
class Pneumatic3dModel {
public:
    Pneumatic3dModel(Pneumatic3dSpec, bike::terrain::Terrain3dQueryView,
                     const TyreStateIdentity3d&);
    ~Pneumatic3dModel();
    // Cold, owner-confined execution policy; not part of material identity.
    void set_worker_threads(unsigned);
    unsigned worker_threads() const noexcept;
    CoreStatus evaluate(
        const std::array<WheelKinematics3d, 2>&,
        std::span<const bike::spatial::Vec3> patch_point_velocity_world_mps,
        double dt_s, bool advance) noexcept;
    std::span<const PatchWrench> wrenches(std::size_t wheel) const noexcept;
    std::span<const Patch3d> patches(std::size_t wheel) const noexcept;
    const TyreState3d& committed_state() const noexcept;
    CoreStatus restore(const TyreState3d&) noexcept;
    ReasonCode3d last_reason() const noexcept;
    double stored_energy_j() const noexcept;
    double dissipated_j() const noexcept;
    const Pneumatic3dSpec& spec() const noexcept;

private:
    struct ElementWorkers;
    std::unique_ptr<ElementWorkers> workers_;
    Pneumatic3dSpec spec_;
    bike::terrain::Terrain3dQueryView view_;
    std::array<TorusGrid3d, 2> grid_;
    TyreState3d committed_, staging_;
    std::array<std::vector<ContactElement3d>, 2> elements_;
    std::array<std::vector<NormalElement3d>, 2> normals_;
    std::array<std::vector<Patch3d>, 2> patches_;
    std::array<std::vector<PatchWrench>, 2> wrench_;
    std::array<std::vector<std::uint32_t>, 2> patch_element_index_;
    std::array<std::size_t, 2> patch_count_{};
    ReasonCode3d reason_ = ReasonCode3d::none;
    std::int64_t last_advanced_interval_ = -1;
};

} // namespace bike::pneumatic3d
