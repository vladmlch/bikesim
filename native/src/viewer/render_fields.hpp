#pragma once
#include "../model_access.hpp"
#include <array>
#include <limits>

namespace bike::viewer::detail {
// MuJoCo owns these C buffers for the lifetime of its model. Centralize the
// pointer/extent conversion and check signed products before creating spans.
[[nodiscard]] inline mjtSize render_extent(mjtSize count, std::size_t width = 1) {
    if (count < 0) throw std::invalid_argument("render model has a negative field extent");
    return static_cast<mjtSize>(model_access::checked_product(static_cast<std::size_t>(count), width,
        static_cast<std::size_t>(std::numeric_limits<mjtSize>::max())));
}
[[nodiscard]] inline std::array<std::span<mjtNum>, 6> mutable_model_fields(mjModel const& model) {
    return {model_access::mutable_buffer(model.dof_frictionloss, model.nv),
        model_access::mutable_buffer(model.site_pos, render_extent(model.nsite, 3)),
        model_access::mutable_buffer(model.tendon_stiffness, model.ntendon),
        model_access::mutable_buffer(model.tendon_damping, model.ntendon),
        model_access::mutable_buffer(model.tendon_lengthspring, render_extent(model.ntendon, 2)),
        model_access::mutable_buffer(model.tendon_range, render_extent(model.ntendon, 2))};
}
[[nodiscard]] inline std::array<std::span<const mjtNum>, 6> readonly_model_fields(const mjModel& model) {
    return {model_access::readonly_buffer(model.dof_frictionloss, model.nv),
        model_access::readonly_buffer(model.site_pos, render_extent(model.nsite, 3)),
        model_access::readonly_buffer(model.tendon_stiffness, model.ntendon),
        model_access::readonly_buffer(model.tendon_damping, model.ntendon),
        model_access::readonly_buffer(model.tendon_lengthspring, render_extent(model.ntendon, 2)),
        model_access::readonly_buffer(model.tendon_range, render_extent(model.ntendon, 2))};
}
} // namespace bike::viewer::detail
