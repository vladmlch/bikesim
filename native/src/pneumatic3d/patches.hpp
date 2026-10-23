#pragma once

#include "pneumatic3d/carcass.hpp"
#include "pneumatic3d/geometry.hpp"
#include "pneumatic3d/state.hpp"
#include "spatial/types.hpp"
#include "writers/writer_types.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

namespace bike::pneumatic3d {

struct Patch3d {
    std::uint64_t id = 0;
    std::span<const std::uint32_t> elements;
    bike::spatial::Vec3 centroid_world_m{}, normal_world{};
    bike::spatial::Vec3 longitudinal_world{}, lateral_world{};
    double normal_load_n = 0., area_m2 = 0.;
    std::uint32_t surface_id = 0;
    bike::spatial::Vec3 force_world_n{}, couple_world_nm{};
    bool valid = false;
};

CoreStatus assign_patches(std::span<const ContactElement3d>,
                          std::span<const NormalElement3d>, bool advance,
                          PatchState3d&, std::span<Patch3d>,
                          std::size_t& count) noexcept;

} // namespace bike::pneumatic3d
