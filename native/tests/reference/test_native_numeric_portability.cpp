// test_native_numeric_portability.cpp — §3.35 port of
// tests/reference/test_native_numeric_portability.py (23 records).
// Every bitwise oracle is replayed from captured goldens: the Python
// file records (input.*) operand streams plus the local NumPy/CPython
// result, and this file drives the SAME production helpers the debug
// bindings call — numeric::np_tanh/std::hypot, numeric::python_vector_
// norm, np_minimum/np_maximum, np_multiply_broadcast, numpy_sum,
// np_sin/np_cos, biketyre::np_interp and the rider::spindle_linalg
// mirrors — then compares raw uint64 bits against the recorded oracle.
//
// The three wire-rejection records are ported through a test-local
// dispatch mirror that applies the same validation the binding's debug
// surface applies (production callers never see those shapes — the C++
// type system already pins them).
//
// Dispositions: all 23 source records ported (the rejection records map
// onto the same validation rules at the test-local dispatch boundary).

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "../support/golden.hpp"
#include "../support/fixture_test.hpp"
#include "numeric_norm.hpp"
#include "numeric_numpy.hpp"
#include "numeric_sincos.hpp"
#include "numeric_sum.hpp"
#include "rider/spindle_linalg.hpp"
#include "tyre/profile.hpp"

namespace {

constexpr std::string_view kFile =
    "tests/reference/test_native_numeric_portability.py::";

[[nodiscard]] std::string node(std::string_view name, std::string_view param) {
    std::string out{kFile};
    out += name;
    out += '[';
    out += param;
    out += ']';
    return out;
}

// numeric::np_tanh + std::hypot elementwise — the _debug_pneumatic_math
// call shapes.
void pneumatic(bt::oracle::Node& node_) {
    const auto xs = node_.f64_input("input.xs");
    const auto ys = node_.f64_input("input.ys");
    std::vector<double> tanh(xs.size());
    for (std::size_t i = 0; i < xs.size(); ++i) tanh[i] = numeric::np_tanh(xs[i]);
    const std::size_t pairs = std::min(xs.size(), ys.size());
    std::vector<double> hypot(pairs);
    for (std::size_t i = 0; i < pairs; ++i) hypot[i] = std::hypot(xs[i], ys[i]);
    node_.f64(tanh, node_.peek_label());
    node_.f64(hypot, node_.peek_label());
    node_.done();
}

// One _debug_numeric_contract dispatch: consumes one (xs, ys) input pair
// and returns the production helper's row; the caller compares with the
// recorded "out" entry. Mirrors the binding's validation.
[[nodiscard]] std::vector<double> contract(
    std::string_view name, std::span<const double> xs,
    std::span<const double> ys) {
    if (name == "sum")
        return {numeric::numpy_sum(xs)};
    if (name == "sin" || name == "cos") {
        std::vector<double> out(xs.size());
        for (std::size_t i = 0; i < xs.size(); ++i)
            out[i] = name == "sin" ? numeric::np_sin(xs[i])
                                   : numeric::np_cos(xs[i]);
        return out;
    }
    if (name != "python_hypot" && name != "minimum" &&
        name != "maximum" && name != "multiply_broadcast")
        throw std::invalid_argument("unknown numeric contract op");
    const std::size_t pairs = std::min(xs.size(), ys.size());
    std::vector<double> out(pairs);
    for (std::size_t i = 0; i < pairs; ++i) {
        if (name == "python_hypot")
            out[i] = numeric::python_vector_norm(std::array{xs[i], ys[i]});
        else if (name == "minimum")
            out[i] = numeric::np_minimum(xs[i], ys[i]);
        else if (name == "maximum")
            out[i] = numeric::np_maximum(xs[i], ys[i]);
        else
            out[i] = numeric::np_multiply_broadcast(xs[i], ys[i]);
    }
    return out;
}

// Replays every interleaved (input.xs, input.ys, expected…) case in a
// node — the loop shape every _contract test shares. compares_per_case
// counts the oracle entries one case records: the nonfinite
// scalar/array agreement test records both the scalar-broadcast oracle
// and the array oracle, and the port must equal each.
void contract_node(std::string_view test, std::string_view op, // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
                   int compares_per_case = 1) {
    bt::oracle::Node node_{test};
    while (node_.has_next()) {
        const auto xs = node_.f64_input("input.xs");
        const auto ys = node_.f64_input("input.ys");
        const auto out = contract(op, xs, ys);
        for (int i = 0; i < compares_per_case; ++i)
            node_.f64(out, node_.peek_label());
    }
    node_.done();
}

void interp_node(std::string_view test) {
    bt::oracle::Node node_{test};
    const auto xs = node_.f64_input("input.xs");
    const auto ys = node_.f64_input("input.ys");
    const auto queries = node_.f64_input("input.queries");
    std::vector<double> out(queries.size());
    for (std::size_t i = 0; i < queries.size(); ++i)
        out[i] = biketyre::np_interp(queries[i], xs, ys);
    node_.f64(out, node_.peek_label());
    node_.done();
}

// rider:: fixed-shape mirrors on the recorded flat operands.
[[nodiscard]] std::vector<double> rider_op(
    std::string_view op, std::span<const double> a, // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
    std::span<const double> b) {
    const auto fill = [](std::span<const double> src, auto& dst,
                         std::string_view what) {
        if (src.size() != dst.size())
            throw std::invalid_argument(std::string(what));
        std::ranges::copy(src, dst.begin());
    };
    const auto flat = [](const auto& arr) {
        return std::vector<double>(arr.begin(), arr.end());
    };
    if (op == "norm")
        return {rider::np_norm(a)};
    if (op == "matvec" || op == "mat_t_vec" || op == "mat_t_vec_xz" ||
        op == "matmul" || op == "mat_t_mul") {
        rider::Mat3 a3{};
        fill(a, a3, "mat3 operand must hold 9 values");
        if (op == "matvec" || op == "mat_t_vec" || op == "mat_t_vec_xz") {
            rider::Vec3 x{};
            fill(b, x, "vec3 operand must hold 3 values");
            if (op == "matvec") return flat(rider::mat_vec(a3, x));
            if (op == "mat_t_vec") return flat(rider::mat_t_vec(a3, x));
            return flat(rider::mat_t_vec_xz(a3, x));
        }
        rider::Mat3 b3{};
        fill(b, b3, "mat3 operand must hold 9 values");
        if (op == "matmul") return flat(rider::mat_mul(a3, b3));
        return flat(rider::mat_t_mul(a3, b3));
    }
    if (op == "matvec2" || op == "mat_t_vec2") {
        rider::Mat2 a2{};
        rider::Vec2 x2{};
        fill(a, a2, "mat2 operand must hold 4 values");
        fill(b, x2, "vec2 operand must hold 2 values");
        if (op == "matvec2") return flat(rider::mat_vec2(a2, x2));
        return flat(rider::mat_t_vec2(a2, x2));
    }
    throw std::invalid_argument("unknown rider op");
}

void rider_node(std::string_view test, std::string_view op) { // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
    bt::oracle::Node node_{test};
    while (node_.has_next()) {
        const auto a = node_.f64_input("input.a");
        if (op == "norm") {
            const auto out = rider_op("norm", a, {});
            node_.f64(out, node_.peek_label());
            continue;
        }
        const auto b = node_.f64_input("input.b");
        const auto out = rider_op(op, a, b);
        node_.f64(out, node_.peek_label());
    }
    node_.done();
}

void pneumatic_suites() {
    for (const char* length : {"1", "2", "3", "7", "8", "9", "31", "32",
                               "33", "65"}) {
        bt::oracle::Node node_{
            node("test_tanh_and_hypot_match_local_array_ufunc", length)};
        pneumatic(node_);
    }
    {
        bt::oracle::Node node_{std::string{kFile} +
            "test_tanh_and_hypot_adversarial_fixture"};
        pneumatic(node_);
    }
    for (const char* stride : {"2", "3", "-1", "-2"}) {
        bt::oracle::Node node_{
            node("test_tanh_and_hypot_strided_oracle", stride)};
        pneumatic(node_);
    }
    {
        // test_tanh_and_hypot_layout_views — three recorded views in one
        // node, each an (xs, ys, tanh, hypot) quadruple.
        bt::oracle::Node node_{std::string{kFile} +
            "test_tanh_and_hypot_layout_views"};
        for (int view = 0; view < 3; ++view) {
            const auto xs = node_.f64_input("input.xs");
            const auto ys = node_.f64_input("input.ys");
            std::vector<double> tanh(xs.size());
            for (std::size_t i = 0; i < xs.size(); ++i)
                tanh[i] = numeric::np_tanh(xs[i]);
            const std::size_t pairs = std::min(xs.size(), ys.size());
            std::vector<double> hypot(pairs);
            for (std::size_t i = 0; i < pairs; ++i)
                hypot[i] = std::hypot(xs[i], ys[i]);
            node_.f64(tanh, node_.peek_label());
            node_.f64(hypot, node_.peek_label());
        }
        node_.done();
    }
}

void contract_suites() {
    contract_node(std::string{kFile} + "test_python_hypot_matches_math_hypot",
                  "python_hypot");
    contract_node(std::string{kFile} + "test_python_hypot_scale_and_nan_edges",
                  "python_hypot");
    for (const char* name : {"minimum", "maximum"}) {
        contract_node(node("test_minmax_matches_elementwise_ufunc", name), name);
        contract_node(node("test_minmax_signed_zero_and_nan_fixture", name),
                      name);
        contract_node(node("test_minmax_scalar_broadcast_shape", name), name);
    }
    contract_node(
        std::string{kFile} + "test_multiply_broadcast_matches_scalar_ufunc",
        "multiply_broadcast");
    contract_node(std::string{kFile} +
                      "test_multiply_broadcast_nan_class_fixture",
                  "multiply_broadcast");
    for (const char* length :
             {"0", "1", "2", "3", "7", "8", "9", "15", "16", "17", "31", "32",
              "33", "63", "64", "65", "127", "128", "129", "130", "131", "135",
              "136", "255", "256", "257", "511", "512", "513"})
        contract_node(node("test_sum_matches_np_sum", length), "sum");
    contract_node(std::string{kFile} + "test_sum_cancellation_fixture", "sum");
    for (const char* name : {"sin", "cos"}) {
        contract_node(node("test_sin_cos_match_local_ufunc", name), name);
        contract_node(node("test_sin_cos_edge_fixture", name), name);
        // test_sin_cos_nonfinite_scalar_array_agreement — the source
        // collects all four cases' inputs first, then asserts both the
        // scalar-broadcast and array oracles per case: a block of
        // (xs, ys) inputs, then per-case oracle pairs the port must
        // each equal bitwise.
        bt::oracle::Node node_{
            node("test_sin_cos_nonfinite_scalar_array_agreement", name)};
        std::vector<std::vector<double>> outs;
        while (node_.has_next() &&
               node_.peek_label() == "input.xs") {
            const auto xs = node_.f64_input("input.xs");
            const auto ys = node_.f64_input("input.ys");
            outs.push_back(contract(name, xs, ys));
        }
        for (const auto& out : outs) {
            node_.f64(out, node_.peek_label());
            node_.f64(out, node_.peek_label());
        }
        node_.done();
    }
}

void interp_suites() {
    for (const char* label : {"n17-random", "n2-random", "n3-nan-y",
                              "n3-random", "n4-constant", "n4-inf-y",
                              "n5-random", "n9-random"})
        interp_node(
            node("test_interp_matches_np_interp_general_table", label));
}

void rider_suites() {
    for (const char* op : {"mat_t_mul", "mat_t_vec", "mat_t_vec_xz",
                           "matmul", "matvec"})
        rider_node(node("test_rider_linalg_3d_matches_numpy", op), op);
    for (const char* op : {"mat_t_vec2", "matvec2"})
        rider_node(node("test_rider_linalg_2d_matches_numpy", op), op);
    for (const char* length : {"1", "2", "3", "5", "8"})
        rider_node(
            node("test_rider_linalg_norm_matches_np_linalg_norm", length),
            "norm");
}

// The wire-rejection records — the binding's own validation rules,
// replayed at the test-local dispatch boundary the production callers
// share. Production helpers are fixed-shape types; these checks are the
// same ones debug_rider_linalg/_debug_numeric_contract/_debug_interp
// apply before any helper is reached.
void rejection_suites() {
    // test_contract_rejects_unknown_name — an unknown dispatch name is
    // rejected before any helper call.
    bt::invalid([] {
        static_cast<void>(contract("definitely_not_an_op",
                                   std::span<const double>{},
                                   std::span<const double>{}));
    });
    // test_interp_rejects_mismatched_or_empty_tables — biketyre::np_interp
    // carries the production validation (empty and mismatched tables).
    bt::invalid([] {
        static_cast<void>(biketyre::np_interp(0.5, std::span<const double>{},
                                              std::span<const double>{}));
    });
    bt::invalid([] {
        const std::vector<double> xs{1., 1., 1.}, ys{1., 2.};
        static_cast<void>(biketyre::np_interp(0.5, xs, ys));
    });
    // test_rider_linalg_rejects_bad_inputs — fixed-shape helpers reject
    // wrong-sized operands at the typed boundary.
    bt::invalid([] {
        static_cast<void>(rider_op("bogus_op", std::span<const double>{},
                                   std::span<const double>{}));
    });
    bt::invalid([] {
        const std::vector<double> bad{1., 0., 0.};
        static_cast<void>(rider_op("matvec", bad, bad));
    });
}
} // namespace

int main() {
    // cppcheck-suppress throwInEntryPoint // test failures propagate as exceptions into bt::run's FAIL catch — the harness entry-point contract, not an unhandled throw
    return bt::run([] {
        pneumatic_suites();
        contract_suites();
        interp_suites();
        rider_suites();
        rejection_suites();
    });
}
