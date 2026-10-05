// binding.cpp — nanobind FFI edge for the native Stepper.
//
// qpos/qvel return zero-copy, NON-OWNING numpy views directly onto
// mjData's buffers: the empty nb::handle() owner is deliberate — the
// view keeps no reference to the Stepper, so the Stepper must outlive
// any array taken from it (views of a dead Stepper dangle). Rebuild
// the array or keep the Stepper referenced for the view's lifetime.
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>
#include "stepper.hpp"

namespace nb = nanobind;

NB_MODULE(bike_native, m) {
    nb::class_<Stepper>(m, "Stepper")
        .def(nb::init<const std::string&>())
        .def("step", &Stepper::step)
        .def_prop_ro("qpos", [](Stepper& s) {
            auto v = s.qpos();
            return nb::ndarray<nb::numpy, const double, nb::shape<-1>>(
                v.data(), {v.size()}, nb::handle());
        })
        .def_prop_ro("qvel", [](Stepper& s) {
            auto v = s.qvel();
            return nb::ndarray<nb::numpy, const double, nb::shape<-1>>(
                v.data(), {v.size()}, nb::handle());
        })
        .def_prop_ro("time", &Stepper::time);
}
