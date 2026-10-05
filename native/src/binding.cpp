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
#include <cstddef>
#include <ranges>
#include <span>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>
#include "stepper.hpp"

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
} // namespace

NB_MODULE(bike_native, m) {
    nb::class_<Stepper>(m, "Stepper")
        .def(nb::init<const std::string&>())
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
        .def_prop_ro("time", &Stepper::time);
}
