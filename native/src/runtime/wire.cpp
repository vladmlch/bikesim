// runtime/wire.cpp — the wire-tree constructors declared in control.hpp.
// Split out so the contract-core object library (which links samples.cpp
// but not control.cpp) resolves them without pulling the runtime helper
// layer. Ordering of the visitor arms mirrors the Python statement it
// names; that ordering is the FP contract.
#include "control.hpp"

namespace runtime {

// The existing contact-wire property, moved verbatim from step.cpp so
// standalone reference tests exercise the same published reconstruction.
std::array<double, 3> patch_world_force(const TirePatch &p) {
    if (p.native_world_force_n) return *p.native_world_force_n;
    return {p.normal_load_n * p.normal[0] + p.tangent_force_n * p.normal[2],
            0.,
            p.normal_load_n * p.normal[2] - p.tangent_force_n * p.normal[0]};
}

Wire wire_array(std::span<const double> values) {
    WireArray out;
    out.reserve(values.size());
    for (const double v : values) out.emplace_back(v);
    return out;
}

Wire wire_diagnostics(
    const std::map<std::string,
                   std::variant<std::monostate, double, int, bool,
                                std::string>> &values) {
    WireObject out;
    out.reserve(values.size());
    for (const auto &[name, v] : values) {
        out.emplace_back(name, std::visit(
                                   [](const auto &x) -> Wire {
                                       using T = std::decay_t<decltype(x)>;
                                       if constexpr (std::is_same_v<
                                                         T, int>)
                                           return Wire(
                                               static_cast<std::int64_t>(
                                                   x));
                                       else if constexpr (std::is_same_v<
                                                              T,
                                                              std::monostate>)
                                           return Wire{nullptr};
                                       else
                                           return Wire{x};
                                   },
                                   v));
    }
    return out;
}

// pneumatic_tire_forces.py:426-455 — the pneumatic per-side dict, its
// own key set in the oracle's insertion order. Moved verbatim from step.cpp.
WireObject wire_pneumatic_tire_diagnostics(const TireDiagnostics &d) {
    WireObject out;
    out.emplace_back("normal_load_n", Wire(d.normal_load_n));
    out.emplace_back("support_n", Wire(d.support_n));
    out.emplace_back("tangent_force_n", Wire(d.tangent_force_n));
    out.emplace_back("slip_mps", Wire(d.slip_mps));
    out.emplace_back("slip_ratio", Wire(d.slip_ratio));
    out.emplace_back("transient_slip_ratio",
                     Wire(d.transient_slip_ratio));
    out.emplace_back("friction_coefficient", Wire(d.friction_coefficient));
    out.emplace_back("penetration_m", Wire(d.penetration_m));
    out.emplace_back("contact_length_m", Wire(d.contact_length_m));
    out.emplace_back("patch_count", Wire(d.patch_count));
    out.emplace_back("multi_support", Wire(d.multi_support));
    out.emplace_back("supports_multiple_contacts",
                     Wire(d.supports_multiple_contacts));
    out.emplace_back("fully_sliding", Wire(d.fully_sliding));
    out.emplace_back("airborne", Wire(d.airborne));
    out.emplace_back("coverage_event", Wire(d.coverage_event));
    out.emplace_back("rim_strike_active", Wire(d.rim_strike_active));
    out.emplace_back("rim_event_x_m", Wire(d.rim_event_x_m));
    out.emplace_back("rim_event_speed_mps", Wire(d.rim_event_speed_mps));
    out.emplace_back("rim_event_peak_load_n",
                     Wire(d.rim_event_peak_load_n));
    out.emplace_back("rim_event_peak_force_n",
                     Wire(d.rim_event_peak_force_n));
    out.emplace_back("rim_event_energy_j", Wire(d.rim_event_energy_j));
    out.emplace_back("pressure_bar", Wire(d.pressure_bar));
    out.emplace_back("dissipated_power_w", Wire(d.dissipated_power_w));
    out.emplace_back("brush_loss_j", Wire(d.brush_loss_j));
    out.emplace_back("radial_energy_j", Wire(d.radial_energy_j));
    out.emplace_back("shear_energy_j", Wire(d.shear_energy_j));
    out.emplace_back("outside_material_load_range",
                     Wire(d.outside_material_load_range));
    out.emplace_back("surface", Wire(d.surface));
    return out;
}

} // namespace runtime
