// test_native_platform_blas.cpp — §3.35 port of
// tests/reference/test_native_platform_blas.py (12 records → 11 ported,
// 1 drop). The Python file drove ``bike_native._debug_platform_blas`` —
// the debug surface that calls the SAME blas::ddot/dgemv/dgemm wrappers
// the production callers use. The port replays captured golden operands
// (recorded as their physical stored byte streams, so the C- and
// F-contiguous call shapes are both preserved) through the same
// blas:: wrappers under a test-local dispatch that applies the debug
// surface's exact validation and layout→entry-point rules from
// native/src/binding.cpp's debug_platform_blas: tightly C- or
// F-contiguous float64 only, an F operand reaches the row-major Trans
// entry point, degenerate sizes keep NumPy's explicit zero answers.
//
// Dispositions:
//   test_blas_debug_binding_import_order — drop: fresh-interpreter
//     subprocess import ordering is Python-extension mechanics; the
//     native port is itself the "native-first" image and there is no
//     numpy image to order against.
//   Every other record ports: dot/matvec/matmul across the DIMS grid,
//   C/F layouts, transposed views, zero sizes, and the four wire
//   rejection contracts replayed at the mirror's validation boundary.

#include <algorithm>
#include <cstddef>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "../support/golden.hpp"
#include "../support/fixture_test.hpp"
#include "cblas_abi.hpp"

namespace {

constexpr std::string_view kFile =
    "tests/reference/test_native_platform_blas.py::";

[[nodiscard]] std::string node(std::string_view name, std::string_view param) {
    std::string out{kFile};
    out += name;
    out += '[';
    out += param;
    out += ']';
    return out;
}

enum class Layout : std::uint8_t { c, f, strided, wrong_dtype };

// A validated operand descriptor: the binding accepts tightly packed
// float64 only — the same checks debug_blas_array/row_contiguous/
// column_contiguous apply before any blas:: call.
struct Operand {
    std::span<const double> stored;
    std::vector<std::size_t> shape;
    Layout layout = Layout::c;
};

[[nodiscard]] bool row_contiguous(const Operand& a) {
    return a.layout == Layout::c || a.layout == Layout::f;
}
[[nodiscard]] bool col_contiguous(const Operand& a) {
    return a.layout == Layout::c || a.layout == Layout::f;
}
[[nodiscard]] bool c_order(const Operand& a) { return a.layout == Layout::c; }

void require_f64(const Operand& a, std::string_view path) {
    if (a.layout == Layout::wrong_dtype)
        throw std::invalid_argument(
            std::string(path) + ": expected a float64 array");
}
void require_contiguous(const Operand& a, std::string_view path) {
    if (!row_contiguous(a) && !col_contiguous(a))
        throw std::invalid_argument(
            std::string(path) + ": expected a C- or F-contiguous array");
}

// debug_platform_blas — same validation order and same blas:: call
// shapes as the binding (comments cite the binding rules verbatim).
[[nodiscard]] std::vector<double> platform_blas(
    const Operand& a, const Operand& b, std::string_view operation) {
    require_f64(a, "a");
    require_f64(b, "b");
    if (operation == "dot") {
        if (a.shape.size() != 1 || b.shape.size() != 1)
            throw std::invalid_argument("a,b: dot requires two 1-D arrays");
        if (!row_contiguous(a))
            throw std::invalid_argument("a: expected a contiguous 1-D array");
        if (!row_contiguous(b))
            throw std::invalid_argument("b: expected a contiguous 1-D array");
        if (a.shape[0] != b.shape[0])
            throw std::invalid_argument("a,b: dot requires equal lengths");
        const int n = static_cast<int>(a.shape[0]);
        // numpy's (0,)@(0,) is +0.0: answer explicitly rather than
        // handing a possibly-null data pointer to a degenerate call.
        if (n == 0) return {0.};
        return {blas::ddot(n, a.stored.data(), 1, b.stored.data(), 1)};
    }
    if (operation == "matvec") {
        if (a.shape.size() != 2 || b.shape.size() != 1)
            throw std::invalid_argument(
                "a,b: matvec requires a 2-D matrix and a 1-D vector");
        require_contiguous(a, "a");
        if (!row_contiguous(b))
            throw std::invalid_argument("b: expected a contiguous 1-D array");
        if (a.shape[1] != b.shape[0])
            throw std::invalid_argument(
                "a,b: matvec inner dimensions must match");
        const int m = static_cast<int>(a.shape[0]);
        const int n = static_cast<int>(a.shape[1]);
        std::vector<double> y(a.shape[0], 0.);
        if (m == 0) return y;
        const auto lda = [](std::size_t packed) {
            return static_cast<int>(std::max<std::size_t>(packed, 1));
        };
        if (c_order(a)) {
            blas::dgemv(blas::Order::row_major, blas::Transpose::no, m, n, 1.,
                        a.stored.data(), lda(a.shape[1]),
                        b.stored.data(), 1, 0., y.data(), 1);
        } else {
            // F-contiguous memory is the C-order (n,m) transpose — the
            // row-major Trans entry computes A^T@x.
            blas::dgemv(blas::Order::row_major, blas::Transpose::yes, n, m, 1.,
                        a.stored.data(), lda(a.shape[0]),
                        b.stored.data(), 1, 0., y.data(), 1);
        }
        return y;
    }
    if (operation == "matmul") {
        if (a.shape.size() != 2 || b.shape.size() != 2)
            throw std::invalid_argument(
                "a,b: matmul requires two 2-D arrays");
        require_contiguous(a, "a");
        require_contiguous(b, "b");
        if (a.shape[1] != b.shape[0])
            throw std::invalid_argument(
                "a,b: matmul inner dimensions must match");
        const int m = static_cast<int>(a.shape[0]);
        const int k = static_cast<int>(a.shape[1]);
        const int n = static_cast<int>(b.shape[1]);
        std::vector<double> c(a.shape[0] * b.shape[1], 0.);
        if (m > 0 && n > 0)
            blas::dgemm(blas::Order::row_major,
                        c_order(a) ? blas::Transpose::no : blas::Transpose::yes,
                        c_order(b) ? blas::Transpose::no : blas::Transpose::yes,
                        m, n, k, 1., a.stored.data(),
                        static_cast<int>(std::max(
                            {c_order(a) ? a.shape[1] : a.shape[0], 1ul})),
                        b.stored.data(),
                        static_cast<int>(std::max(
                            {c_order(b) ? b.shape[1] : b.shape[0], 1ul})),
                        0., c.data(), std::max(n, 1));
        return c;
    }
    throw std::invalid_argument(
        "operation: expected 'dot', 'matvec' or 'matmul'");
}

void dot_nodes() {
    for (const char* n : {"0", "1", "2", "3", "7", "16", "17", "64", "127",
                          "128", "129"}) {
        bt::oracle::Node node_{node("test_blas_dot_matches_numpy", n)};
        const auto a = node_.f64_input("input.a");
        const auto b = node_.f64_input("input.b");
        const auto actual = platform_blas(
            Operand{.stored = a, .shape = {a.size()}, .layout = Layout::c},
            Operand{.stored = b, .shape = {b.size()}, .layout = Layout::c},
            "dot");
        node_.f64(actual, node_.peek_label());
        node_.done();
    }
    {
        bt::oracle::Node node_{std::string{kFile} + "test_blas_dot_zero_size"};
        const auto a = node_.f64_input("input.a");
        const auto b = node_.f64_input("input.b");
        const auto actual = platform_blas(
            Operand{.stored = a, .shape = {a.size()}, .layout = Layout::c},
            Operand{.stored = b, .shape = {b.size()}, .layout = Layout::c},
            "dot");
        node_.f64(actual, node_.peek_label());
        node_.done();
    }
}

void matvec_node(std::string_view param, std::size_t m, std::size_t n,
                 Layout layout) {
    bt::oracle::Node node_{
        node("test_blas_matvec_matches_numpy", param)};
    const auto a = node_.f64_input("input.a");
    const auto x = node_.f64_input("input.b");
    const auto actual = platform_blas(
        Operand{.stored = a, .shape = {m, n}, .layout = layout},
        Operand{.stored = x, .shape = {n}, .layout = Layout::c}, "matvec");
    node_.f64(actual, node_.peek_label());
    node_.done();
}

void matvec_nodes() {
    const std::pair<std::size_t, std::size_t> dims[] = { // NOLINT(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) bounded label buffer for snprintf
        {0, 0}, {0, 3}, {1, 1}, {2, 3}, {3, 2}, {3, 17}, {16, 16},
        {17, 129}, {64, 127}, {128, 64}, {129, 1}};
    for (const auto& [m, n] : dims)
        for (const char* layout : {"C", "F"}) {
          std::string const param = std::string(layout) + "-" +
                                    std::to_string(m) + "-" + std::to_string(n);
          matvec_node(param, m, n, *layout == 'C' ? Layout::c : Layout::f);
        }
}

void matmul_nodes() {
    const std::array<std::size_t, 3> dims[] = { // NOLINT(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) bounded label buffer for snprintf
        {0, 0, 0}, {0, 3, 2}, {2, 0, 3}, {1, 1, 1}, {2, 3, 4}, {3, 2, 2},
        {3, 17, 5}, {16, 17, 64}, {127, 1, 2}, {129, 64, 7}};
    for (const auto& [m, k, n] : dims)
        for (const char* layout : {"C-C", "C-F", "F-C", "F-F"}) {
            const std::string layout_str{layout};
            const std::string param = layout_str + "-" +
                std::to_string(m) + "-" + std::to_string(k) + "-" +
                std::to_string(n);
            bt::oracle::Node node_{
                node("test_blas_matmul_matches_numpy", param)};
            const auto a = node_.f64_input("input.a");
            const auto b = node_.f64_input("input.b");
            const auto actual = platform_blas(
                Operand{.stored = a,
                        .shape = {m, k},
                        .layout =
                            layout_str.at(0) == 'C' ? Layout::c : Layout::f},
                Operand{.stored = b,
                        .shape = {k, n},
                        .layout =
                            layout_str.at(2) == 'C' ? Layout::c : Layout::f},
                "matmul");
            node_.f64(actual, node_.peek_label());
            node_.done();
        }
}

void transposed_and_rectangular() {
    // test_blas_matvec_transposed_view_matches_numpy — the recorded
    // operand is `transposed` (3,17): F-view (stored as the C-order
    // (17,3) a) for [F], materialized C for [C].
    for (const char* layout : {"C", "F"}) {
        bt::oracle::Node node_{
            node("test_blas_matvec_transposed_view_matches_numpy", layout)};
        const auto a = node_.f64_input("input.a");
        const auto x = node_.f64_input("input.b");
        const auto actual = platform_blas(
            Operand{.stored = a,
                    .shape = {3, 17},
                    .layout = *layout == 'C' ? Layout::c : Layout::f},
            Operand{.stored = x, .shape = {17}, .layout = Layout::c}, "matvec");
        node_.f64(actual, node_.peek_label());
        node_.done();
    }
    {
        // test_blas_matmul_transposed_view_matches_numpy — local (4,3) C
        // times r.T (3,3) F: the row-major NT dgemm call shape.
        bt::oracle::Node node_{std::string{kFile} +
            "test_blas_matmul_transposed_view_matches_numpy"};
        const auto a = node_.f64_input("input.a");
        const auto b = node_.f64_input("input.b");
        const auto actual = platform_blas(
            Operand{.stored = a, .shape = {4, 3}, .layout = Layout::c},
            Operand{.stored = b, .shape = {3, 3}, .layout = Layout::f},
            "matmul");
        node_.f64(actual, node_.peek_label());
        node_.done();
    }
    {
        // test_blas_rectangular_matmul_matches_selected_numpy.
        bt::oracle::Node node_{std::string{kFile} +
            "test_blas_rectangular_matmul_matches_selected_numpy"};
        const auto a = node_.f64_input("input.a");
        const auto b = node_.f64_input("input.b");
        const auto actual = platform_blas(
            Operand{.stored = a, .shape = {3, 17}, .layout = Layout::c},
            Operand{.stored = b, .shape = {17, 5}, .layout = Layout::c},
            "matmul");
        node_.f64(actual, node_.peek_label());
        node_.done();
    }
}

// The wire-rejection records — same validation rules at the mirror's
// boundary: unknown operation, shape mismatch, non-contiguous storage
// (never re-interpreted through invented strides) and wrong rank/dtype.
void rejection_suites() {
    const std::vector<double> two{1., 1.}, three{1., 1., 1.},
        four{1., 1., 1., 1.};
    bt::invalid([&] {
      static_cast<void>(platform_blas(
          Operand{.stored = two, .shape = {2}, .layout = Layout::c},
          Operand{.stored = two, .shape = {2}, .layout = Layout::c}, "ger"));
    });
    // shape mismatch: unequal dot lengths and unmatched inner dims.
    bt::invalid([&] {
      static_cast<void>(platform_blas(
          Operand{.stored = three, .shape = {3}, .layout = Layout::c},
          Operand{.stored = four, .shape = {4}, .layout = Layout::c}, "dot"));
    });
    bt::invalid([&] {
      static_cast<void>(platform_blas(
          Operand{.stored = four, .shape = {2, 3}, .layout = Layout::c},
          Operand{.stored = four, .shape = {4}, .layout = Layout::c},
          "matvec"));
    });
    bt::invalid([&] {
      static_cast<void>(platform_blas(
          Operand{.stored = four, .shape = {2, 3}, .layout = Layout::c},
          Operand{.stored = four, .shape = {4, 2}, .layout = Layout::c},
          "matmul"));
    });
    // non-contiguous storage rejected, never strided through.
    bt::invalid([&] {
      static_cast<void>(platform_blas(
          Operand{.stored = four, .shape = {4}, .layout = Layout::strided},
          Operand{.stored = four, .shape = {4}, .layout = Layout::c}, "dot"));
    });
    bt::invalid([&] {
      static_cast<void>(platform_blas(
          Operand{.stored = four, .shape = {2, 2}, .layout = Layout::strided},
          Operand{.stored = two, .shape = {2}, .layout = Layout::c}, "matvec"));
    });
    bt::invalid([&] {
      static_cast<void>(platform_blas(
          Operand{.stored = four, .shape = {2, 2}, .layout = Layout::strided},
          Operand{.stored = four, .shape = {2, 2}, .layout = Layout::c},
          "matmul"));
    });
    // wrong rank and wrong dtype.
    bt::invalid([&] {
      static_cast<void>(platform_blas(
          Operand{.stored = four, .shape = {2, 2, 2}, .layout = Layout::c},
          Operand{.stored = four, .shape = {2, 2}, .layout = Layout::c},
          "matmul"));
    });
    bt::invalid([&] {
      static_cast<void>(platform_blas(
          Operand{.stored = two, .shape = {2}, .layout = Layout::wrong_dtype},
          Operand{.stored = two, .shape = {2}, .layout = Layout::c}, "dot"));
    });
}
} // namespace

int main() {
    // cppcheck-suppress throwInEntryPoint // test failures propagate as exceptions into bt::run's FAIL catch — the harness entry-point contract, not an unhandled throw
    return bt::run([] {
        dot_nodes();
        matvec_nodes();
        matmul_nodes();
        transposed_and_rectangular();
        rejection_suites();
    });
}
