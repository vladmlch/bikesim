// binding.cpp — nanobind FFI edge for the native Stepper.
//
// qpos/qvel return zero-copy, NON-OWNING numpy views directly onto
// mjData's buffers: the empty nb::handle() owner is deliberate — the
// view keeps no reference to the Stepper, so the Stepper must outlive
// any array taken from it (views of a dead Stepper dangle). Rebuild
// the array or keep the Stepper referenced for the view's lifetime.
// The solved-quantity views (qacc, qfrc_constraint, efc_force) follow
// the same rule; efc_force's length is d->nefc, which varies per
// forward — a stored view is not refreshed, re-read the property.
#include <algorithm>
#include <array>
#include <cstddef>
#include <ranges>
#include <span>
#include <stdexcept>
#include <vector>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include "stepper.hpp"
#include "rider/contact_binding.hpp"
#include "drivetrain/policy_binding.hpp"
#include "drivetrain/drive_binding.hpp"
#include "writers/cruise.hpp"
#include "writers/resistance.hpp"
#include "writers/tire.hpp"

namespace nb = nanobind;

namespace {
// 1-D C-contiguous float64 ndarray → span. Shape and contiguity are
// enforced by the parameter type; model-width checks live in the core
// (set_state) — one place only. size() is size_t; counted wants the
// signed iter_difference_t, hence the cast.
std::span<const double> as_span(
        const nb::ndarray<const double, nb::shape<-1>, nb::c_contig>& a) {
    return std::views::counted(a.data(),
                             static_cast<std::ptrdiff_t>(a.size()));
}
// Read-only numpy view over a Stepper span — shared by every prop_ro.
nb::ndarray<nb::numpy, const double, nb::shape<-1>>
as_view(std::span<const double> v) {
    return nb::ndarray<nb::numpy, const double, nb::shape<-1>>(
        v.data(), {v.size()}, nb::handle());
}
// Owning float64 ndarray: the buffer is freed when the array is GC'd, so
// dict values outlive both the writer's temporaries and the Stepper.
nb::ndarray<nb::numpy, double, nb::shape<-1>>
as_owned(std::vector<double> v) {
    auto* buf = new double[v.size()];
    std::ranges::copy(v, buf);
    nb::capsule owner(buf, [](void* p) noexcept {
        delete[] static_cast<double*>(p);
    });
    return nb::ndarray<nb::numpy, double, nb::shape<-1>>(
        buf, {v.size()}, owner);
}
// Owning bool ndarray — the patch_working array in the resistance schema.
nb::ndarray<nb::numpy, bool, nb::shape<-1>>
as_owned_bool(std::vector<char> v) {
    auto* buf = new bool[v.size()];
    const std::span<bool> view = std::views::counted(
        buf, static_cast<std::ptrdiff_t>(v.size()));
    for (std::size_t i = 0; i < v.size(); ++i)
        view[i] = v[i] != 0;
    nb::capsule owner(buf, [](void* p) noexcept {
        delete[] static_cast<bool*>(p);
    });
    return nb::ndarray<nb::numpy, bool, nb::shape<-1>>(
        buf, {v.size()}, owner);
}
// One TireSnapshot as a dict carrying BOTH consumers' schemas: the
// manifest's readable digest keys (what the artifact serializes in
// tools/golden_episode._tire_snapshot_row) and the flat-array keys
// resistance_components's side_input reads.
nb::dict snapshot_dict(const TireSnapshot& s) {
    nb::dict d;
    d["time_s"] = s.time_s;
    d["interval_id"] = s.interval_id;
    d["backend"] = s.backend;
    d["geometric_contact"] = s.geometric_contact;
    d["effective_radius_m"] = s.effective_radius_m;
    std::vector<double> loads;
    std::vector<char> working;
    loads.reserve(s.patches.size());
    working.reserve(s.patches.size());
    nb::list patches;
    for (const TirePatch& p : s.patches) {
        loads.push_back(p.normal_load_n);
        working.push_back(static_cast<char>(p.working_surface));
        nb::dict pd;
        pd["normal_load_n"] = p.normal_load_n;
        pd["tangent_force_n"] = p.tangent_force_n;
        pd["slip_mps"] = p.slip_mps;
        pd["working_surface"] = p.working_surface;
        patches.append(pd);
    }
    d["patches"] = std::move(patches);
    d["patch_loads"] = as_owned(std::move(loads));
    d["patch_working"] = as_owned_bool(std::move(working));
    d["eff_radius"] = s.effective_radius_m;
    return d;
}
// One tire-snapshot side, flat-array schema:
// {'patch_loads': f64[n], 'patch_working': bool[n], 'eff_radius': float}.
// Dtype and layout conversions are accepted. Keep their ndarray owners alive
// through the writer call: a span alone cannot retain a conversion temporary.
struct OwnedTireSideInput {
    nb::ndarray<const double, nb::shape<-1>, nb::c_contig> loads;
    nb::ndarray<const bool, nb::shape<-1>, nb::c_contig> working;
    double effective_radius_m;

    [[nodiscard]] TireSideInput view() const {
        return {std::views::counted(loads.data(),
                                   static_cast<std::ptrdiff_t>(loads.size())),
                std::views::counted(working.data(),
                                   static_cast<std::ptrdiff_t>(working.size())),
                effective_radius_m};
    }
};

OwnedTireSideInput side_input(const nb::dict& snaps, const char* side) {
    if (!snaps.contains(side))
        throw std::invalid_argument(
            "resistance_components: missing side '" + std::string(side) +
            "'");
    const nb::dict d = nb::cast<nb::dict>(snaps[side]);
    const auto arr = [&d, side](const char* key) -> nb::object {
        if (!d.contains(key))
            throw std::invalid_argument(
                "resistance_components: '" + std::string(side) + "." + key +
                "' is required");
        return d[key];
    };
    using F64 = nb::ndarray<const double, nb::shape<-1>, nb::c_contig>;
    using B1 = nb::ndarray<const bool, nb::shape<-1>, nb::c_contig>;
    const F64 loads = nb::cast<F64>(arr("patch_loads"));
    const B1 working = nb::cast<B1>(arr("patch_working"));
    return {loads, working, nb::cast<double>(arr("eff_radius"))};
}
} // namespace

NB_MODULE(bike_native, m) {
    bind_drive_policies(m);
    auto stepper_class = nb::class_<Stepper>(m, "Stepper");
    stepper_class
        .def(nb::init<const std::string&>())
        .def(nb::init<const std::string&, const nb::dict&>(),
             nb::arg("mjb_path"), nb::arg("config"))
        .def("step", &Stepper::step)
        .def("forward", &Stepper::forward)
        .def("set_state", [](Stepper& s,
                const nb::ndarray<const double, nb::shape<-1>,
                                  nb::c_contig>& qpos,
                const nb::ndarray<const double, nb::shape<-1>,
                                  nb::c_contig>& qvel,
                const nb::ndarray<const double, nb::shape<-1>,
                                  nb::c_contig>& act,
                const nb::ndarray<const double, nb::shape<-1>,
                                  nb::c_contig>& warmstart,
                double time) {
            s.set_state(as_span(qpos), as_span(qvel), as_span(act),
                        as_span(warmstart), time);
        })
        .def_prop_ro("qpos", [](Stepper& s) { return as_view(s.qpos()); })
        .def_prop_ro("qvel", [](Stepper& s) { return as_view(s.qvel()); })
        .def_prop_ro("qacc", [](Stepper& s) { return as_view(s.qacc()); })
        .def_prop_ro("qfrc_constraint", [](Stepper& s) {
            return as_view(s.qfrc_constraint());
        })
        .def_prop_ro("efc_force", [](Stepper& s) {
            return as_view(s.efc_force());
        })
        .def_prop_ro("ctrl", [](Stepper& s) { return as_view(s.ctrl()); })
        .def("brake_torques", &Stepper::brake_torques)
        .def("apply_brake", &Stepper::apply_brake)
        .def("cruise_compute", &Stepper::cruise_compute,
             nb::arg("rear_in_contact"), nb::arg("traction_limited") = false,
             nb::arg("controller_grounded") = nb::none())
        .def("cruise_reset", &Stepper::cruise_reset)
        .def("cruise_set_target_speed", &Stepper::cruise_set_target_speed,
             nb::arg("value_kmh"))
        .def("cruise_set_assist_compensation",
             &Stepper::cruise_set_assist_compensation, nb::arg("support_factor"))
        .def("cruise_state", [](const Stepper& s) {
            const CruiseState state = s.cruise_state();
            nb::dict out;
            out["target_speed_mps"] = state.target_speed_mps;
            out["integral_mps_s"] = state.integral_mps_s;
            out["torque_nm"] = state.torque_nm;
            out["engaged"] = state.engaged;
            out["gain_scale"] = state.gain_scale;
            return out;
        })
        .def("set_cruise_state", [](Stepper& s, const nb::dict& state) {
            // Disabled controller errors precede candidate parsing, as for
            // the other controller APIs. All reads finish before mutation.
            static_cast<void>(s.cruise_state());
            if (state.size() != 5)
                throw std::invalid_argument(
                    "cruise state requires exactly target_speed_mps, "
                    "integral_mps_s, torque_nm, engaged, gain_scale");
            const auto number = [&state](const char* key) {
                try {
                    return nativecfg::detail::req_f64(state, "cruise_state", key);
                } catch (const nb::cast_error&) {
                    throw std::invalid_argument(std::string(key) + " must be numeric");
                }
            };
            CruiseState candidate{number("target_speed_mps")};
            candidate.integral_mps_s = number("integral_mps_s");
            candidate.torque_nm = number("torque_nm");
            const nb::object engaged =
                nativecfg::detail::req(state, "cruise_state", "engaged");
            if (!nb::isinstance<nb::bool_>(engaged))
                throw std::invalid_argument("engaged must be bool");
            candidate.engaged = nb::cast<bool>(engaged);
            candidate.gain_scale = number("gain_scale");
            s.set_cruise_state(candidate);
        }, nb::arg("state"))
        .def("resistance_components", [](Stepper& s, const nb::dict& snaps) {
            // Named locals in Python's dict order — arg eval order is
            // unspecified, so on malformed input the 'front' error must win.
            const OwnedTireSideInput front = side_input(snaps, "front");
            const OwnedTireSideInput rear = side_input(snaps, "rear");
            nb::dict out;
            for (auto& [name, vec] :
                 s.resistance_components(front.view(), rear.view()))
                out[name.c_str()] = as_owned(std::move(vec));
            return out;
        }, nb::arg("snapshots"),
        "Compute resistance from snapshot arrays; 1-D loads/working arrays "
        "may be converted to contiguous float64/bool for this call.")
        .def("suspension_components", [](Stepper& s) {
            nb::dict out;
            for (auto& [name, vec] : s.suspension_components())
                out[name.c_str()] = as_owned(std::move(vec));
            return out;
        })
        .def("rider_forces_qfrc", [](Stepper& s) {
            return as_owned(s.rider_forces_qfrc());
        })
        .def("total", [](Stepper& s, const nb::dict& components) {
            // Insertion order is the contract: PyDict_Keys preserves it,
            // and the fold inside Stepper::total mirrors
            // ForceAccumulator.total() exactly. Nanobind may convert a 1-D
            // ndarray's dtype/layout to contiguous float64. Copy each array
            // into vecs while its owner is alive; reject wrong-rank or
            // non-convertible values before the fold.
            std::vector<std::vector<double>> vecs;
            const nb::list keys = components.keys();
            vecs.reserve(keys.size());
            for (nb::handle key : keys) {
                const std::string name = nb::cast<std::string>(key);
                const nb::object value = components[key];
                try {
                    const auto a = nb::cast<nb::ndarray<
                        const double, nb::shape<-1>, nb::c_contig>>(value);
                    const std::span<const double> sv =
                        std::views::counted(
                            a.data(),
                            static_cast<std::ptrdiff_t>(a.size()));
                    vecs.emplace_back(sv.begin(), sv.end());
                } catch (const nb::cast_error&) {
                    throw std::invalid_argument(
                        "total: component '" + name +
                        "' must be a 1-D float64 array");
                }
            }
            return as_owned(s.total(vecs));
        }, nb::arg("components"),
        "Sum finite nv-wide 1-D arrays in dict insertion order; ndarray "
        "dtype/layout conversion to contiguous float64 is accepted.")
        .def("set_tire_state", [](Stepper& s,
                const std::vector<std::string>& names,
                const nb::ndarray<const double, nb::shape<-1>,
                                  nb::c_contig>& row) {
            s.set_tire_state(names, as_span(row));
        })
        .def("tire_state", [](Stepper& s) {
            return as_owned(s.tire_state());
        })
        .def_prop_ro("tire_state_names", [](Stepper& s) {
            return s.tire_state_names();
        })
        .def("tire_qfrc", [](Stepper& s, double dt) {
            return as_owned(s.tire_qfrc(dt));
        })
        .def("tire_snapshots", [](Stepper& s) {
            const TireWriter* t = s.tire();
            if (!t)
                throw std::logic_error(
                    "tire writer: Stepper was built without a tire config "
                    "(pass the dict from tools.native_config.project)");
            nb::dict out;
            const auto& snaps = t->snapshots();
            const std::array<const char*, 2> sides = {"front", "rear"};
            for (std::size_t i = 0; i < 2; ++i)
                if (snaps[i])
                    out[sides[i]] = snapshot_dict(*snaps[i]);
            return out;
        })
        .def("tire_diagnostics", [](Stepper& s) {
            const TireWriter* t = s.tire();
            if (!t)
                throw std::logic_error(
                    "tire writer: Stepper was built without a tire config "
                    "(pass the dict from tools.native_config.project)");
            nb::dict out;
            const auto& diags = t->diagnostics();
            const std::array<const char*, 2> sides = {"front", "rear"};
            for (std::size_t i = 0; i < 2; ++i) {
                if (!diags[i])
                    continue;
                const TireDiagnostics& g = *diags[i];
                nb::dict d;
                d["multi_support"] = g.multi_support;
                d["penetration_m"] = g.penetration_m;
                d["normal_speed_mps"] = g.normal_speed_mps;
                d["slip_mps"] = g.slip_mps;
                d["normal_load_n"] = g.normal_load_n;
                d["tangent_force_n"] = g.tangent_force_n;
                d["friction_coefficient"] = g.friction_coefficient;
                d["surface"] = g.surface;
                d["branch_release_loss_j"] = g.branch_release_loss_j;
                d["brush_loss_j"] = g.brush_loss_j;
                d["radial_energy_j"] = g.radial_energy_j;
                d["shear_energy_j"] = g.shear_energy_j;
                d["outside_material_load_range"] =
                    g.outside_material_load_range;
                out[sides[i]] = d;
            }
            return out;
        })
        .def_prop_ro("time", &Stepper::time);
    bind_drivetrain(m, stepper_class);
    bind_rider_contact_math(m);
}
