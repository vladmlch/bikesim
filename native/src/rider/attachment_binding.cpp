// attachment_binding.cpp — the T2a/T2b FFI seam for attachment-wrench
// measurement. Everything a rider-attachment diagnostic needs lives
// behind this one entry point: the Stepper method face
// (rider_equality_qfrc), the module-level equality-row/reaction
// diagnostics, and the DGELSD least-squares hook that T2b's wrench
// recovery reuses. A later TU implementing attachment_wrench.* extends
// bind_rider_attachment rather than binding.cpp.
#include "attachment_binding.hpp"

#include "../binding_arrays.hpp"
#include "../diag.hpp"
#include "../stepper.hpp"
#include "attachment_wrench.hpp"
#include "attachment_wire.hpp"
#include "equality_reactions.hpp"
#include "least_squares.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <ranges>
#include <stdexcept>
#include <string>
#include <vector>

#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;

// The conversion helpers below moved into namespace attachment_wire —
// shared with rider_contact_binding.cpp via attachment_wire.hpp. The
// unqualified call sites in bind_rider_attachment resolve through this
// directive exactly as they did when the helpers were anonymous.
using namespace attachment_wire;

// attachment_wire.hpp's conversions live here (declarations in the
// header, definitions in this TU — one schema for both attachment faces).
namespace attachment_wire {

    // Owning 2-D float64 array — wire::owned_array's exact policy,
    // extended to the (n, k) solution matrix np.linalg.lstsq returns for
    // a 2-D right-hand side.
    nb::ndarray<nb::numpy, double, nb::shape<-1, -1>>
    owned_matrix(std::vector<double> values, std::size_t rows,
                 std::size_t columns) {
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) ndarray requires runtime-sized contiguous primitive storage
        auto storage = std::make_unique<double[]>(values.size());
        std::ranges::copy(values, storage.get());
        auto [data, owner] = wire::detail::release_with_owner(
            std::move(storage), [](double *value) {
                return nb::capsule(value, [](void *p) noexcept {
                    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory) nanobind capsule destructor is the release path
                    delete[] static_cast<double *>(p);
                });
            });
        return {data, {rows, columns}, owner};
    }

} // namespace attachment_wire

namespace {

    // np.linalg.lstsq's input contract: A is exactly 2-D, B is 1-D or
    // 2-D, and B's row count equals A's. The casts accept dtype and
    // layout conversions (float32 -> float64, strided -> contiguous)
    // exactly as numpy's gufunc dispatch does — nonfinite VALUES are
    // never screened; they flow into DGELSD like the oracle's.
    nb::tuple least_squares_hook(nb::handle raw_a, nb::handle raw_b,
                                 double rcond) {
        using Matrix =
            nb::ndarray<const double, nb::shape<-1, -1>, nb::c_contig>;
        using Vector =
            nb::ndarray<const double, nb::shape<-1>, nb::c_contig>;
        Matrix a;
        try {
            a = nb::cast<Matrix>(raw_a);
        } catch (const nb::cast_error &) {
            throw std::invalid_argument(
                "expected a 2-dimensional array for 'a'");
        }
        const std::size_t em = a.shape(0);
        const std::size_t en = a.shape(1);

        const double *b_data = nullptr;
        std::size_t ek = 0;
        bool vector_b = true;
        try {
            const Vector b = nb::cast<Vector>(raw_b);
            if (b.shape(0) != em)
                throw std::invalid_argument("Incompatible dimensions");
            b_data = b.data();
            ek = 1;
        } catch (const nb::cast_error &) {
            Matrix b;
            try {
                b = nb::cast<Matrix>(raw_b);
            } catch (const nb::cast_error &) {
                throw std::invalid_argument(
                    "expected a 1- or 2-dimensional array for 'b'");
            }
            if (b.shape(0) != em)
                throw std::invalid_argument("Incompatible dimensions");
            b_data = b.data();
            ek = b.shape(1);
            vector_b = false;
        }

        // numpy's n_rhs == 0 quirk: a (m, 0) target still runs the SVD —
        // rank and s come back populated, x is (n, 0), resids is (0,).
        // Solve against one zero column (the SVD is rhs-independent),
        // then emit the empty outputs.
        std::vector<double> zero_rhs;
        std::span<const double> b_span;
        std::size_t solve_rhs = ek;
        if (ek == 0) {
            zero_rhs.assign(em, 0.);
            b_span = zero_rhs;
            solve_rhs = 1;
        } else {
            b_span = std::views::counted(
                b_data, static_cast<std::ptrdiff_t>(em * ek));
        }

        rider::LeastSquaresWorkspace workspace(
            static_cast<rider::lapack_int>(em),
            static_cast<rider::lapack_int>(en),
            static_cast<rider::lapack_int>(solve_rhs));
        rider::LeastSquaresResult result = workspace.solve(
            std::views::counted(a.data(),
                                static_cast<std::ptrdiff_t>(em * en)),
            static_cast<rider::lapack_int>(em),
            static_cast<rider::lapack_int>(en), b_span,
            static_cast<rider::lapack_int>(solve_rhs), rcond);

        // x: (n,) for a vector b, (n, k) for a matrix b. resids: (k,)
        // when populated — always (0,) for a zero-column target.
        if (ek == 0) {
            result.solution.clear();
            result.residuals.clear();
        }
        if (vector_b)
            return nb::make_tuple(
                wire::owned_array<double>(result.solution),
                wire::owned_array<double>(result.residuals), result.rank,
                wire::owned_array<double>(result.singular_values));
        return nb::make_tuple(
            owned_matrix(std::move(result.solution), en, ek),
            wire::owned_array<double>(result.residuals), result.rank,
            wire::owned_array<double>(result.singular_values));
    }

} // namespace

// --------------------------- T2b conversions --------------------------
// attachment_wire.hpp — shared with rider_contact_binding.cpp's settle /
// sample face. One schema, one implementation: the prepared-dict
// round-trip must byte-for-byte reproduce what T2b emits.
namespace attachment_wire {

    // Python truthiness for the oracle's `not rotational` /
    // `if validate_wrench` gates — accepts np.bool_, ints and every
    // truthy/falsy object like the oracle does.
    bool truthy(nb::handle value) {
        const int result = PyObject_IsTrue(value.ptr());
        if (result < 0) throw nb::python_error();
        return result != 0;
    }

    // np.asarray(x, dtype=float) for the 1-D ports: lists, tuples and
    // 1-D float-convertible arrays all land here; a non-1-D or
    // non-numeric input is the same invalid_argument the typed core's
    // shape gate raises downstream.
    std::vector<double> float_vector(nb::handle value,
                                     std::string_view label) {
        try {
            return nb::cast<std::vector<double>>(value);
        } catch (const nb::cast_error &) {
            throw std::invalid_argument(std::string(label) +
                                        " must be a 1-D float64 sequence");
        }
    }

    // The optional pull_direction: None stays None, anything else is a
    // 1-D float64 sequence like `normal`.
    std::optional<std::vector<double>> optional_vector(nb::handle value,
                                                       std::string_view label) {
        if (value.is_none()) return std::nullopt;
        return float_vector(value, label);
    }

    // An owning (rows, columns) float64 matrix — DenseMatrix's copy.
    nb::ndarray<nb::numpy, double, nb::shape<-1, -1>>
    owned_dense_matrix(const rider::DenseMatrix &m) {
        return owned_matrix(m.values, m.rows, m.columns);
    }

    // The AttachmentGeometry dict — the dataclass's exact field names;
    // every array is fresh capsule-owned storage.
    nb::dict geometry_dict(const rider::AttachmentGeometry &g) {
        nb::dict out;
        out["eq_id"] = g.eq_id;
        out["rider_jac"] = owned_dense_matrix(g.rider_jac);
        out["rider_columns"] =
            wire::owned_array<std::int64_t>(g.rider_columns);
        out["bike_jac"] = owned_dense_matrix(g.bike_jac);
        out["bike_columns"] =
            wire::owned_array<std::int64_t>(g.bike_columns);
        out["observable"] = g.observable;
        out["normal"] = wire::owned_array<double>(g.normal);
        out["kind"] = g.kind;
        out["rotational"] = g.rotational;
        out["half_patch_m"] = g.half_patch_m;
        if (g.pull_direction.has_value())
            out["pull_direction"] =
                wire::owned_array<double>(*g.pull_direction);
        else
            out["pull_direction"] = nb::none();
        return out;
    }

    // The AttachmentRaw dict — the dataclass's exact field names.
    nb::dict raw_dict(const rider::AttachmentRaw &r) {
        nb::dict out;
        out["rider_jac"] = owned_dense_matrix(r.rider_jac);
        out["rider_qfrc"] = wire::owned_array<double>(r.rider_qfrc);
        out["bike_jac"] = owned_dense_matrix(r.bike_jac);
        out["bike_qfrc"] = wire::owned_array<double>(r.bike_qfrc);
        out["observable"] = r.observable;
        out["normal"] = wire::owned_array<double>(r.normal);
        out["kind"] = r.kind;
        out["rotational"] = r.rotational;
        out["half_patch_m"] = r.half_patch_m;
        out["gap_m"] = r.gap_m;
        if (r.pull_direction.has_value())
            out["pull_direction"] =
                wire::owned_array<double>(*r.pull_direction);
        else
            out["pull_direction"] = nb::none();
        return out;
    }

    // The AttachmentSample dict — the budget dataclass's exact fields.
    nb::dict sample_dict(const rider::AttachmentSample &s) {
        nb::dict out;
        out["kind"] = s.kind;
        out["normal_n"] = s.normal_n;
        out["tangent_n"] = s.tangent_n;
        out["moment_nm"] = s.moment_nm;
        out["gap_m"] = s.gap_m;
        out["pull_n"] = s.pull_n;
        out["half_patch_m"] = s.half_patch_m;
        return out;
    }

    // Field access that accepts both the dataclass (attributes) and a
    // mapping (dict / vars()-style namespace): attributes first so a
    // dataclass round-trips, [] fallback keeps the dict contract.
    nb::object attachment_field(nb::handle owner, const char *name) {
        if (nb::hasattr(owner, name)) return nb::getattr(owner, name);
        return nb::object(owner[name]);
    }

    // A stored 1-D index vector — np.flatnonzero's int64 columns or any
    // integer sequence.
    std::vector<std::int64_t> index_vector(nb::handle value,
                                           std::string_view label) {
        try {
            return nb::cast<std::vector<std::int64_t>>(value);
        } catch (const nb::cast_error &) {
            throw std::invalid_argument(
                std::string(label) + " must be an integer sequence");
        }
    }

    // A stored (rows, columns) float64 block — the geometry's jacobian
    // fields; accepts ndarray and nested sequences like the lstsq cast.
    rider::DenseMatrix dense_matrix_arg(nb::handle value,
                                        std::string_view label) {
        using Matrix =
            nb::ndarray<const double, nb::shape<-1, -1>, nb::c_contig>;
        try {
            const Matrix a = nb::cast<Matrix>(value);
            rider::DenseMatrix m{.rows = a.shape(0),
                                 .columns = a.shape(1),
                                 .values = {}};
            const std::span<const double> flat =
                std::views::counted(a.data(), static_cast<std::ptrdiff_t>(
                                                  m.rows * m.columns));
            m.values.assign(flat.begin(), flat.end());
            return m;
        } catch (const nb::cast_error &) {
            throw std::invalid_argument(std::string(label) +
                                        " must be a 2-D float64 array");
        }
    }

    // Parse a Python AttachmentGeometry (dataclass or dict) into the
    // typed struct — every field is copied; nothing references the
    // Python object afterwards.
    rider::AttachmentGeometry attachment_geometry_from(nb::handle owner) {
        rider::AttachmentGeometry g;
        try {
            g.eq_id = nb::cast<int>(attachment_field(owner, "eq_id"));
        } catch (const nb::cast_error &) {
            throw std::invalid_argument("eq_id must be an integer");
        }
        g.rider_jac =
            dense_matrix_arg(attachment_field(owner, "rider_jac"),
                             "rider_jac");
        g.rider_columns = index_vector(attachment_field(owner, "rider_columns"),
                                       "rider_columns");
        g.bike_jac =
            dense_matrix_arg(attachment_field(owner, "bike_jac"),
                             "bike_jac");
        g.bike_columns = index_vector(attachment_field(owner, "bike_columns"),
                                      "bike_columns");
        g.observable = truthy(attachment_field(owner, "observable"));
        g.normal = float_vector(attachment_field(owner, "normal"), "normal");
        try {
            g.kind = nb::cast<std::string>(attachment_field(owner, "kind"));
        } catch (const nb::cast_error &) {
            throw std::invalid_argument("kind must be a string");
        }
        g.rotational = truthy(attachment_field(owner, "rotational"));
        try {
            g.half_patch_m =
                nb::cast<double>(attachment_field(owner, "half_patch_m"));
        } catch (const nb::cast_error &) {
            throw std::invalid_argument("half_patch_m must be a float");
        }
        g.pull_direction =
            optional_vector(attachment_field(owner, "pull_direction"),
                            "pull_direction");
        return g;
    }

} // namespace attachment_wire

namespace {

    // recover_wrench's matrix argument — np.asarray(jac, dtype=float)
    // then numpy _assert_2d's wording on a wrong rank.
    rider::DenseMatrix recover_matrix_arg(nb::handle value) {
        using Matrix =
            nb::ndarray<const double, nb::shape<-1, -1>, nb::c_contig>;
        try {
            const Matrix a = nb::cast<Matrix>(value);
            rider::DenseMatrix m{.rows = a.shape(0),
                                 .columns = a.shape(1),
                                 .values = {}};
            const std::span<const double> flat =
                std::views::counted(a.data(), static_cast<std::ptrdiff_t>(
                                                  m.rows * m.columns));
            m.values.assign(flat.begin(), flat.end());
            return m;
        } catch (const nb::cast_error &) {
            // An ndarray of the wrong rank gets numpy's _assert_2d
            // wording; a nested sequence is flattened below.
            if (nb::isinstance<nb::ndarray<>>(value)) {
                const nb::ndarray<> any = nb::cast<nb::ndarray<>>(value);
                throw std::invalid_argument(
                    std::to_string(any.ndim()) +
                    "-dimensional array given. Array must be "
                    "two-dimensional");
            }
            try {
                const auto nested =
                    nb::cast<std::vector<std::vector<double>>>(value);
                rider::DenseMatrix m{
                    .rows = nested.size(),
                    .columns = nested.empty() ? 0 : nested.front().size(),
                    .values = {}};
                m.values.reserve(m.rows * m.columns);
                for (const auto &row : nested) {
                    if (row.size() != m.columns)
                        throw std::invalid_argument(
                            "expected a 2-dimensional array for "
                            "'relative_jacobian'");
                    m.values.insert(m.values.end(), row.begin(),
                                    row.end());
                }
                return m;
            } catch (const nb::cast_error &) {
                throw std::invalid_argument(
                    "expected a 2-dimensional array for "
                    "'relative_jacobian'");
            }
        }
    }
} // namespace

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void bind_rider_attachment(nb::module_ &module, nb::class_<Stepper> &cls) {
    // The SVD non-convergence error surfaces as a ValueError — numpy's
    // LinAlgError counterpart. Registered once per interpreter like the
    // contact binder's translator.
    // NOLINTNEXTLINE(bugprone-throw-keyword-missing,bugprone-unused-raii,misc-const-correctness) RAII translator registration, see bind_rider_contact_math
    nb::exception<rider::LeastSquaresError>(module, "LeastSquaresError",
                                          PyExc_ValueError);

    cls.def("rider_equality_qfrc", [](const Stepper &s, int eq_id) {
                return wire::owned_array<double>(
                    s.rider_equality_qfrc(eq_id));
            }, nb::arg("eq_id"),
            "attachment_wrench.equality_qfrc on the solved arena: zeros(nv) "
            "plus J^T times this equality's current multipliers");

    module.def("_rider_equality_rows", [](Stepper &stepper) {
        nb::dict out;
        for (auto &[eq_id, rows]:
             stepper.rider_equalities().equality_rows(stepper.data()))
            out[nb::cast(eq_id)] = wire::owned_array<std::int64_t>(rows);
        return out;
    }, nb::arg("stepper"));

    module.def("_rider_equality_reaction",
               [](Stepper &stepper, int eq_id) {
        const rider::EqualityReactions &reactions =
            stepper.rider_equalities();
        const mjData *data = stepper.data();
        nb::dict out;
        out["rows"] =
            wire::owned_array<std::int64_t>(reactions.rows_of(data, eq_id));
        out["force_on_rider_n"] = wire::owned_array<double>(
            reactions.force_on_rider(data, eq_id));
        out["translation_residual_m"] =
            reactions.translation_residual(data, eq_id);
        out["qfrc"] =
            wire::owned_array<double>(stepper.rider_equality_qfrc(eq_id));
        return out;
    }, nb::arg("stepper"), nb::arg("eq_id"));

    module.def("_rider_equalities_qfrc_at",
               [](Stepper &stepper, nb::handle raw_ids, int dof) {
        std::vector<int> eq_ids;
        try {
            eq_ids = nb::cast<std::vector<int>>(raw_ids);
        } catch (const nb::cast_error &) {
            throw std::invalid_argument(
                "equalities_qfrc_at: eq_ids must be an integer sequence");
        }
        return stepper.rider_equalities().equalities_qfrc_at(
            stepper.data(), eq_ids, dof);
    }, nb::arg("stepper"), nb::arg("eq_ids"), nb::arg("dof"));

    module.def("_rider_least_squares", &least_squares_hook,
               nb::arg("a"), nb::arg("b"), nb::arg("rcond"));

    // ------------------------- T2b measurement face -------------------
    // The Stepper methods return owning typed blocks; these wrappers are
    // pure conversion (dict/array boxing) — no numerics live here.

    cls.def("rider_relative_planar_jacobian",
            [](const Stepper &s, int body_a, int body_b, nb::handle point,
               nb::handle rotational) {
                return owned_dense_matrix(s.rider_relative_planar_jacobian(
                    body_a, body_b,
                    float_vector(point, "world point"),
                    truthy(rotational)));
            },
            nb::arg("body_a"), nb::arg("body_b"), nb::arg("point"),
            nb::arg("rotational"),
            "attachment_wrench.relative_planar_jacobian on the owned "
            "pair: (2, nv) or (3, nv) x/z + world-y difference rows");

    cls.def("rider_prepare_attachment",
            [](const Stepper &s, int eq_id, int body_rider, int body_bike,
               nb::handle point, nb::handle normal, nb::handle kind,
               nb::handle rotational, nb::handle half_patch_m,
               nb::handle pull_direction) {
                return geometry_dict(s.rider_prepare_attachment(
                    eq_id, body_rider, body_bike,
                    float_vector(point, "world point"),
                    float_vector(normal, "normal"),
                    nb::cast<std::string>(kind), truthy(rotational),
                    nb::cast<double>(half_patch_m),
                    optional_vector(pull_direction, "pull_direction")));
            },
            nb::arg("eq_id"), nb::arg("body_rider"), nb::arg("body_bike"),
            nb::arg("point"), nb::arg("normal"), nb::arg("kind"),
            nb::arg("rotational"), nb::arg("half_patch_m") = 0.,
            nb::arg("pull_direction") = nb::none(),
            "attachment_wrench.prepare_attachment_geometry: the owning "
            "interval-start capture");

    cls.def("rider_attachment_raw",
            [](const Stepper &s, int eq_id, int body_rider, int body_bike,
               nb::handle point, nb::handle normal, nb::handle kind,
               nb::handle rotational, nb::handle half_patch_m,
               nb::handle pull_direction) {
                return raw_dict(s.rider_attachment_raw(
                    eq_id, body_rider, body_bike,
                    float_vector(point, "world point"),
                    float_vector(normal, "normal"),
                    nb::cast<std::string>(kind), truthy(rotational),
                    nb::cast<double>(half_patch_m),
                    optional_vector(pull_direction, "pull_direction")));
            },
            nb::arg("eq_id"), nb::arg("body_rider"), nb::arg("body_bike"),
            nb::arg("point"), nb::arg("normal"), nb::arg("kind"),
            nb::arg("rotational"), nb::arg("half_patch_m") = 0.,
            nb::arg("pull_direction") = nb::none(),
            "attachment_wrench.attachment_raw: geometry AND the current "
            "solve in one pass");

    cls.def("rider_attachment_raw_from_geometry",
            [](const Stepper &s, nb::handle geometry,
               nb::handle validate_wrench) {
                return raw_dict(s.rider_attachment_raw_from_geometry(
                    attachment_geometry_from(geometry),
                    truthy(validate_wrench)));
            },
            nb::arg("geometry"), nb::arg("validate_wrench") = true,
            "attachment_wrench.attachment_raw_from_geometry: frozen "
            "jacobians + the current solve's multipliers/residual");

    cls.def("rider_attachment_sample",
            [](const Stepper &s, int eq_id, int body_rider, int body_bike,
               nb::handle point, nb::handle normal, nb::handle kind,
               nb::handle rotational, nb::handle half_patch_m,
               nb::handle pull_direction) {
                return sample_dict(s.rider_attachment_sample(
                    eq_id, body_rider, body_bike,
                    float_vector(point, "world point"),
                    float_vector(normal, "normal"),
                    nb::cast<std::string>(kind), truthy(rotational),
                    nb::cast<double>(half_patch_m),
                    optional_vector(pull_direction, "pull_direction")));
            },
            nb::arg("eq_id"), nb::arg("body_rider"), nb::arg("body_bike"),
            nb::arg("point"), nb::arg("normal"), nb::arg("kind"),
            nb::arg("rotational"), nb::arg("half_patch_m") = 0.,
            nb::arg("pull_direction") = nb::none(),
            "attachment_wrench.attachment_sample: the reduced budget "
            "sample of one solved attachment");

    module.def("rider_recover_wrench",
               // NOLINTBEGIN(bugprone-easily-swappable-parameters) the
               // two handles mirror the oracle's (jac, qfrc) order.
               [](nb::handle raw_jac, nb::handle raw_qfrc) {
                   // NOLINTEND(bugprone-easily-swappable-parameters)
                   const rider::DenseMatrix jac =
                       recover_matrix_arg(raw_jac);
                   const std::vector<double> qfrc =
                       float_vector(raw_qfrc, "qfrc");
                   rider::LeastSquaresWorkspace workspace(
                       static_cast<rider::lapack_int>(jac.columns),
                       static_cast<rider::lapack_int>(jac.rows), 1);
                   return wire::owned_array<double>(rider::recover_wrench(
                       workspace, jac, qfrc));
               },
               nb::arg("relative_jacobian"), nb::arg("qfrc"),
               "attachment_wrench.recover_wrench: full-rank planar "
               "wrench solve (owning output)");

    module.def("rider_decompose_wrench",
               // NOLINTBEGIN(bugprone-easily-swappable-parameters)
               // positional handles mirror the oracle's signature.
               [](nb::handle wrench, nb::handle normal, nb::handle kind,
                  nb::handle rotational, nb::handle half_patch_m,
                  nb::handle gap_m, nb::handle pull_direction) {
                   // NOLINTEND(bugprone-easily-swappable-parameters)
                   const std::vector<double> w =
                       float_vector(wrench, "wrench");
                   return sample_dict(rider::decompose_wrench(
                       w, float_vector(normal, "normal"),
                       nb::cast<std::string>(kind), truthy(rotational),
                       nb::cast<double>(half_patch_m),
                       nb::cast<double>(gap_m),
                       optional_vector(pull_direction, "pull_direction")));
               },
               nb::arg("wrench"), nb::arg("normal"), nb::arg("kind"),
               nb::arg("rotational"), nb::arg("half_patch_m") = 0.,
               nb::arg("gap_m") = 0.,
               nb::arg("pull_direction") = nb::none(),
               "attachment_wrench.decompose_wrench: the support-frame "
               "budget projection");
}
NATIVE_DIAG_POP

// The T2b Stepper members live in this TU (not stepper.cpp) for the same
// reason as T2a's — out-of-line definitions leave stepper.cpp untouched
// while the Stepper's own (m_, d_) stay the only simulation owner.

rider::LeastSquaresWorkspace &Stepper::rider_attachment_lstsq() const {
    require_healthy();
    if (attachment_lstsq_ == nullptr) {
        const rider::lapack_int bound =
            static_cast<rider::lapack_int>(m_->nv);
        attachment_lstsq_ =
            std::make_unique<rider::LeastSquaresWorkspace>(
                bound, std::max<rider::lapack_int>(bound, 6), 1);
    }
    return *attachment_lstsq_;
}

std::span<double> Stepper::attachment_qfrc_scratch() const {
    const std::size_t nv = static_cast<std::size_t>(m_->nv);
    if (attachment_qfrc_.size() != nv) attachment_qfrc_.assign(nv, 0.);
    return attachment_qfrc_;
}

rider::DenseMatrix
Stepper::rider_relative_planar_jacobian(int body_a, int body_b,
                                      std::span<const double> point,
                                      bool rotational) const {
    require_healthy();
    return rider::relative_planar_jacobian(m_, d_, body_a, body_b, point,
                                         rotational);
}

rider::AttachmentGeometry Stepper::rider_prepare_attachment(
    int eq_id, int body_rider, int body_bike,
    std::span<const double> point, std::span<const double> normal,
    std::string kind, bool rotational, double half_patch_m,
    const std::optional<std::vector<double>> &pull_direction) const {
    require_healthy();
    return rider::prepare_attachment_geometry(
        m_, d_, eq_id, body_rider, body_bike, point, normal,
        std::move(kind), rotational, half_patch_m, pull_direction);
}

rider::AttachmentRaw Stepper::rider_attachment_raw(
    int eq_id, int body_rider, int body_bike,
    std::span<const double> point, std::span<const double> normal,
    std::string kind, bool rotational, double half_patch_m,
    const std::optional<std::vector<double>> &pull_direction) const {
    require_healthy();
    return rider::attachment_raw(m_, d_, rider_equalities(),
                                 attachment_qfrc_scratch(), eq_id,
                                 body_rider, body_bike, point, normal,
                                 std::move(kind), rotational,
                                 half_patch_m, pull_direction);
}

rider::AttachmentRaw Stepper::rider_attachment_raw_from_geometry(
    const rider::AttachmentGeometry &geometry,
    bool validate_wrench) const {
    require_healthy();
    return rider::attachment_raw_from_geometry(
        d_, rider_equalities(), rider_attachment_lstsq(),
        attachment_qfrc_scratch(), geometry, validate_wrench);
}

rider::AttachmentSample Stepper::rider_attachment_sample(
    int eq_id, int body_rider, int body_bike,
    std::span<const double> point, std::span<const double> normal,
    const std::string &kind, bool rotational, double half_patch_m,
    const std::optional<std::vector<double>> &pull_direction) const {
    require_healthy();
    return rider::attachment_sample(m_, d_, rider_equalities(),
                                    rider_attachment_lstsq(),
                                    attachment_qfrc_scratch(), eq_id,
                                    body_rider, body_bike, point, normal,
                                    kind, rotational,
                                    half_patch_m, pull_direction);
}
