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
#include <cstddef>
#include <ranges>
#include <span>
#include <stdexcept>
#include <vector>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/string.h>
#include "stepper.hpp"
#include "writers/resistance.hpp"

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
// One tire-snapshot side, flat-array schema:
// {'patch_loads': f64[n], 'patch_working': bool[n], 'eff_radius': float}.
// Spans alias the caller's numpy arrays — valid for the call's duration.
TireSideInput side_input(const nb::dict& snaps, const char* side) {
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
    return {std::views::counted(loads.data(),
                              static_cast<std::ptrdiff_t>(loads.size())),
            std::views::counted(working.data(),
                              static_cast<std::ptrdiff_t>(working.size())),
            nb::cast<double>(arr("eff_radius"))};
}
} // namespace

NB_MODULE(bike_native, m) {
    nb::class_<Stepper>(m, "Stepper")
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
        .def("resistance_components", [](Stepper& s, const nb::dict& snaps) {
            // Named locals in Python's dict order — arg eval order is
            // unspecified, so on malformed input the 'front' error must win.
            TireSideInput front = side_input(snaps, "front");
            TireSideInput rear = side_input(snaps, "rear");
            nb::dict out;
            for (auto& [name, vec] : s.resistance_components(front, rear))
                out[name.c_str()] = as_owned(std::move(vec));
            return out;
        })
        .def("suspension_components", [](Stepper& s) {
            nb::dict out;
            for (auto& [name, vec] : s.suspension_components())
                out[name.c_str()] = as_owned(std::move(vec));
            return out;
        })
        .def_prop_ro("time", &Stepper::time);
}
