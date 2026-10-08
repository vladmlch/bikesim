// attachment_wire.hpp — the Python↔typed conversions shared by the two
// attachment FFI surfaces: T2b's rider_* measurement methods
// (attachment_binding.cpp) and T3b-2's rider_contacts_* settle/sample
// face (rider_contact_binding.cpp). Every owning array/dict emission and
// every dataclass-or-mapping parse lives behind this one header so the
// two paths can never disagree about the wire schema.
//
// Implementations live in attachment_binding.cpp — these are ordinary
// external-linkage declarations, not an inline-header duplicate.
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include "attachment_wrench.hpp"

namespace attachment_wire {

    // Owning (rows, columns) float64 ndarray — wire::owned_array's
    // policy extended to 2-D (capsule-owned storage, writeable).
    [[nodiscard]] nanobind::ndarray<nanobind::numpy, double,
                                    nanobind::shape<-1, -1>>
    owned_matrix(std::vector<double> values, std::size_t rows,
                 std::size_t columns);

    // A rider::DenseMatrix as an owning 2-D ndarray.
    [[nodiscard]] nanobind::ndarray<nanobind::numpy, double,
                                    nanobind::shape<-1, -1>>
    owned_dense_matrix(const rider::DenseMatrix &m);

    // Python truthiness — PyObject_IsTrue; an error propagates as
    // nanobind::python_error like a Python bool() failure.
    bool truthy(nanobind::handle value);

    // np.asarray(x, dtype=float) for 1-D input — lists, tuples and
    // 1-D float-convertible arrays; a bad input is std::invalid_argument.
    [[nodiscard]] std::vector<double> float_vector(nanobind::handle value,
                                                   std::string_view label);

    // None -> nullopt; anything else is a 1-D float64 sequence.
    [[nodiscard]] std::optional<std::vector<double>>
    optional_vector(nanobind::handle value, std::string_view label);

    // A stored 1-D index vector — np.flatnonzero's int64 columns or any
    // integer sequence.
    [[nodiscard]] std::vector<std::int64_t> index_vector(
        nanobind::handle value, std::string_view label);

    // A stored (rows, columns) float64 block — accepts ndarray and
    // nested sequences like the lstsq cast.
    [[nodiscard]] rider::DenseMatrix dense_matrix_arg(
        nanobind::handle value, std::string_view label);

    // Field access accepting both a dataclass (attributes) and a
    // mapping (dict-style []): attributes first so the oracle's
    // dataclass round-trips, [] fallback keeps the dict contract.
    [[nodiscard]] nanobind::object attachment_field(nanobind::handle owner,
                                                    const char *name);

    // The AttachmentGeometry dict — the dataclass's exact field names;
    // every array is fresh capsule-owned storage.
    [[nodiscard]] nanobind::dict
    geometry_dict(const rider::AttachmentGeometry &g);

    // The AttachmentRaw dict — the dataclass's exact field names.
    [[nodiscard]] nanobind::dict raw_dict(const rider::AttachmentRaw &r);

    // The AttachmentSample dict — the budget dataclass's exact fields.
    [[nodiscard]] nanobind::dict
    sample_dict(const rider::AttachmentSample &s);

    // Parse a Python AttachmentGeometry (dataclass or dict) into the
    // typed struct — every field is copied; nothing references the
    // Python object afterwards.
    [[nodiscard]] rider::AttachmentGeometry
    attachment_geometry_from(nanobind::handle owner);

} // namespace attachment_wire
