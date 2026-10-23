#pragma once

#include "pneumatic3d/brush.hpp"
#include "pneumatic3d/carcass.hpp"
#include "pneumatic3d/geometry.hpp"
#include "pneumatic3d/state.hpp"
#include "writers/writer_types.hpp"

#include <span>

namespace bike::pneumatic3d {

CoreStatus update_detailed_brush(
    const BrushSpec3d&, std::span<const ContactElement3d>,
    std::span<const NormalElement3d>, const WheelKinematics3d&, double dt_s,
    bool advance, BrushState3d&,
    std::span<PatchWrench> element_wrench_world) noexcept;

} // namespace bike::pneumatic3d
