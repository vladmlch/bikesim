#pragma once

#include "pneumatic3d/model.hpp"
#include "spatial/topology.hpp"
#include "spatial/types.hpp"
#include "terrain/source3d.hpp"
#include "writers/writer_types.hpp"

#include <mujoco/mujoco.h>

#include <array>
#include <cstddef>
#include <span>
#include <vector>

namespace bike::pneumatic3d {

class Pneumatic3dTireWriter {
public:
    // Cold bind; topology and model identities must match. Terrain lives longer
    // than this writer. No engine tyre contacts coexist with this custom tyre.
    Pneumatic3dTireWriter(const mjModel&, const bike::spatial::Topology&,
                          Pneumatic3dSpec, const bike::terrain::Terrain3dSource&,
                          TyreStateIdentity3d);
    CoreStatus try_compute_into(const mjData*, double dt_s, bool advance,
                                std::span<double> qfrc) noexcept;
    const TyreState3d& state() const noexcept;
    std::vector<std::byte> state_bytes() const;
    CoreStatus restore(const TyreState3d&) noexcept;
    std::span<const PatchWrench> wrenches(std::size_t wheel) const noexcept;
    double axle_moment_nm(std::size_t wheel) const noexcept;
    ReasonCode3d last_reason() const noexcept;
    const Pneumatic3dModel& model() const noexcept;
    // Cold execution-policy change; owner must not be evaluating the writer.
    void set_worker_threads(unsigned);

private:
    const mjModel* model_ = nullptr;
    bike::spatial::Topology topology_;
    Pneumatic3dModel kernels_;
    std::vector<double> jacp_, jacr_; // Each contains 3*nv scratch scalars.
    std::vector<bike::spatial::Vec3> point_velocity_;
    std::array<double, 2> axle_moment_{};
};

} // namespace bike::pneumatic3d
