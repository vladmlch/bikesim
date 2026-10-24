#pragma once

#include "pneumatic3d/carcass.hpp"

namespace bike::pneumatic3d::detail {

// Private decomposition of the unchanged public kernels. The pressure and
// every material/energy reduction are computed by the owner in index order.
// Workers write disjoint geometry/normal slots, never shared material state.
CoreStatus query_geometry_range(const TorusGrid3d&, const WheelKinematics3d&,
    const terrain::Terrain3dQueryView&, std::span<ContactElement3d>,
    std::size_t begin, std::size_t end) noexcept;
CoreStatus prepare_carcass(const CarcassSpec3d&, std::span<const ContactElement3d>,
    double dt_s, const CarcassState3d&, std::span<NormalElement3d>, double& pressure) noexcept;
CoreStatus evaluate_carcass_range(const CarcassSpec3d&, std::span<const ContactElement3d>,
    double dt_s, const CarcassState3d&, std::span<NormalElement3d>, double pressure,
    std::size_t begin, std::size_t end) noexcept;
CoreStatus finish_carcass(const CarcassSpec3d&, std::span<const ContactElement3d>,
    double dt_s, bool advance, CarcassState3d&, std::span<const NormalElement3d>) noexcept;

} // namespace bike::pneumatic3d::detail
