#include "contact_binding.hpp"
#include "support_geometry.hpp"
#include "../binding_arrays.hpp"
#include "../diag.hpp"
#include "../stepper.hpp"
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <algorithm>
#include <memory>
#include <ranges>
#include <span>

namespace nb = nanobind;

namespace {
    double number(nb::handle value) {
        if (nb::isinstance<nb::bool_>(value)) throw std::invalid_argument("expected a finite real scalar");
        try { return nb::cast<double>(value); } catch (const nb::cast_error &) {
            throw std::invalid_argument("expected a finite real scalar");
        }
    }

    rider::Vec3 vector(nb::handle value) {
        try { return nb::cast<rider::Vec3>(value); } catch (const nb::cast_error &) {
            throw std::invalid_argument("expected a 3-vector");
        }
    }

    NATIVE_DIAG_PUSH
    NATIVE_DIAG_IGNORE("-Wlarge-by-value-copy")
    rider::Mat3 matrix(nb::handle value) {
        try {
            const auto rows = nb::cast<std::array<rider::Vec3, 3> >(value);
            rider::Mat3 result{};
            for (std::size_t i = 0; i < 3; ++i)
                std::ranges::copy(rows[i], std::span(result).subspan(3 * i, 3).begin());
            return result;
        } catch (const nb::cast_error &) { throw std::invalid_argument("expected a 3x3 rotation"); }
    }

    NATIVE_DIAG_POP
    nb::ndarray<nb::numpy, double, nb::shape<3> > owned(const rider::Vec3 &value) {
        // The unique_ptr guards the allocation until the capsule owner
        // exists — a throwing capsule ctor frees the Vec3, never leaks.
        auto allocation = std::make_unique<rider::Vec3>(value);
        auto [vec, owner] = wire::detail::release_with_owner(
            std::move(allocation), [](rider::Vec3 *pointer) {
                return nb::capsule(pointer, [](void *p) noexcept {
                    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory) nanobind capsule destructor is the release path
                    delete static_cast<rider::Vec3 *>(p);
                });
            });
        return {vec->data(), {3}, owner};
    }

    nb::dict diagnostic(const rider::SoleGoalDiagnostic &value) {
        nb::dict result;
        result["requested_compression_m"] = value.requested_compression_m;
        result["applied_compression_m"] = value.applied_compression_m;
        result["requested_shear_m"] = value.requested_shear_m;
        result["applied_shear_m"] = value.applied_shear_m;
        result["saturated"] = value.saturated;
        result["limiting_reasons"] = nb::cast(value.limiting_reasons);
        return result;
    }
} // namespace
void bind_rider_contact_math(nb::module_ &module) {
    // nb::exception<> registers a translator — the RAII temporary is the API,
    // not a forgotten throw or an accidental throwaway object.
    // NOLINTNEXTLINE(bugprone-throw-keyword-missing,bugprone-unused-raii,misc-const-correctness)
    nb::exception<rider::UnreachableSoleTarget>(module, "UnreachableSoleTarget", PyExc_ValueError);
    // nb::arg names mirror the Python counterparts
    // (support_geometry.py / rider_contacts.py / grip_release.py); the
    // calls stay positional-compatible.
    module.def("rider_box_pad_contact",
               [](nb::handle center, nb::handle radius, nb::handle origin, nb::handle rotation, nb::handle half) {
                   const auto value = rider::box_pad_contact(vector(center), number(radius), vector(origin),
                                                             matrix(rotation), vector(half));
                   nb::dict result;
                   result["point_m"] = owned(value.point_m);
                   result["normal"] = owned(value.normal);
                   result["tangent"] = owned(value.tangent());
                   result["gap_m"] = value.gap_m;
                   result["within_width"] = value.within_width;
                   result["within_footprint"] = value.within_footprint;
                   return result;
               },
               nb::arg("center_m"), nb::arg("radius_m"), nb::arg("box_origin_m"),
               nb::arg("box_rotation"), nb::arg("half_size_m"));
    module.def("rider_upper_box_face", [](nb::handle origin, nb::handle rotation, nb::handle half) {
        const auto value = rider::upper_box_face(vector(origin), matrix(rotation), vector(half));
        return nb::make_tuple(owned(value.point_m), owned(value.normal), owned(value.tangent));
    }, nb::arg("box_origin_m"), nb::arg("box_rotation"), nb::arg("half_size_m"));
    module.def("rider_sole_target_height", [](nb::handle origin, nb::handle rotation, nb::handle half, double sole_x,
                                              double length, double radius, double compression) {
        return rider::sole_target_height(vector(origin), matrix(rotation), vector(half), sole_x, length, radius,
                                         compression);
    }, nb::arg("origin"), nb::arg("rotation"), nb::arg("half"), nb::arg("sole_x_m"),
       nb::arg("pad_half_length_m"), nb::arg("pad_radius_m"), nb::arg("compression_m"));
    module.def("rider_project_sole_goal", [](nb::handle origin, nb::handle rotation, nb::handle half, double length,
                                             double radius, double compression, double shear) {
        const auto value = rider::project_sole_goal(vector(origin), matrix(rotation), vector(half), length, radius,
                                                    compression, shear);
        return nb::make_tuple(owned(value.goal), diagnostic(value.diagnostic));
    }, nb::arg("origin"), nb::arg("rotation"), nb::arg("half"), nb::arg("pad_half_length_m"),
       nb::arg("pad_radius_m"), nb::arg("compression_m"), nb::arg("shear_m"));
    module.def("rider_grip_step", [](nb::handle xi, nb::handle velocity, nb::handle k, nb::handle c, nb::handle dt) {
        const auto value = rider::grip_step(vector(xi), vector(velocity), number(k), number(c), number(dt));
        return nb::make_tuple(owned(value.xi), owned(value.force), value.energy_j, value.loss_j);
    }, nb::arg("xi"), nb::arg("relative_velocity"), nb::arg("k"), nb::arg("c"), nb::arg("dt"));
    module.def("rider_release_if_overloaded", [](nb::handle force, double energy, double limit) {
        const auto value = rider::release_if_overloaded(vector(force), energy, limit);
        return nb::make_tuple(owned(value.force), value.loss_j, value.overloaded);
    }, nb::arg("force_n"), nb::arg("old_energy_j"), nb::arg("limit_n"));
    module.def("_rider_validate_support_model", [](const Stepper &stepper, nb::handle values) {
        std::vector<int> ids;
        try { ids = nb::cast<std::vector<int> >(values); } catch (const nb::cast_error &) {
            throw std::invalid_argument("invalid rider support geom indices");
        }
        rider::validate_planar_support_model(stepper.model(), stepper.data(), ids);
    }, nb::arg("stepper"), nb::arg("geom_ids"));
    module.def("_rider_hypot3", [](nb::handle value) { return rider::hypot3(vector(value)); },
               nb::arg("value"));
}
