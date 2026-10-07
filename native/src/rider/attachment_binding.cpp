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
#include "equality_reactions.hpp"
#include "least_squares.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <ranges>
#include <stdexcept>
#include <vector>

#include <nanobind/ndarray.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;

namespace {
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
}
NATIVE_DIAG_POP
