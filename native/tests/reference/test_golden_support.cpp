#include "support/golden.hpp"
#include "support/fixture_test.hpp"
#include "io/npy.hpp"

#include <array>
#include <bit>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>

namespace {
constexpr std::string_view node_name = "synthetic_reader/case/1.5";
struct Scratch {
    std::filesystem::path root = std::filesystem::current_path() / "golden-support-scratch";
    Scratch() { std::filesystem::remove_all(root); std::filesystem::create_directories(root); }
    ~Scratch() { std::error_code ignored; std::filesystem::remove_all(root, ignored); }
    Scratch(const Scratch &) = delete;
    Scratch &operator=(const Scratch &) = delete;
    Scratch(Scratch &&) = delete;
    Scratch &operator=(Scratch &&) = delete;
    std::filesystem::path base() const { return root / bt::oracle::platform() / bt::oracle::node_path(node_name); }
    void write(std::string_view meta, std::span<const double> values) const {
        const auto target = base();
        std::filesystem::create_directories(target.parent_path());
        std::ofstream sidecar(target.string() + ".json", std::ios::binary);
        sidecar << meta;
        const auto array = bike::io::double_array(values, {values.size()});
        std::ofstream payload(target.string() + ".bin", std::ios::binary);
        for (const auto byte : array.bytes) payload.put(static_cast<char>(std::to_integer<unsigned char>(byte)));
        bt::require(sidecar.good() && payload.good(), "synthetic reader fixture writes succeed");
    }
};
template<class F> void fails(F &&body, std::string_view reason) {
    bool threw = false;
    try { std::forward<F>(body)(); } catch (const std::exception &) { threw = true; }
    bt::require(threw, reason);
}
std::string entry(int index, std::string_view label, int offset, int size,
                  std::string_view kind = "bitwise", std::string_view tolerance = "null") {
    return "{\"index\":" + std::to_string(index) + R"(,"node":")" + std::string(node_name) +
        R"(","label":")" + std::string(label) + R"(","kind":")" + std::string(kind) +
        R"(","dtype":"<f8","shape":[)" + std::to_string(size) +
        "],\"offset\":" + std::to_string(offset) + ",\"nbytes\":" + std::to_string(size * 8) +
        ",\"rtol\":" + std::string(tolerance) + ",\"atol\":" + std::string(tolerance) + "}";
}
void path_contract() {
    bt::require(bt::oracle::node_path("a/b") == "a/b", "two-component key passes through");
    bt::require(bt::oracle::node_path("a/b/c d") == "a/b/c_d", "parameter component sanitizes");
    bt::require(bt::oracle::node_path("a/b/\xf0\x9f\x98\x80") == "a/b/_", "Unicode uses one underscore per code point");
    fails([] { (void)bt::oracle::node_path("a::b"); }, "pytest file::test syntax rejects");
    fails([] { (void)bt::oracle::node_path("a/b[c]"); }, "pytest parameter syntax rejects");
    fails([] { (void)bt::oracle::node_path("a"); }, "single component rejects");
    fails([] { (void)bt::oracle::node_path("a/b/c/d/e"); }, "five components reject");
    fails([] { (void)bt::oracle::node_path("a//b"); }, "empty component rejects");
    fails([] { (void)bt::oracle::node_path("a/../b"); }, "traversal component rejects");
}
void ordered_bitwise_contract() {
    Scratch scratch;
    const std::array values{1., -0.};
    scratch.write("[" + entry(0, "one", 0, 1) + ',' + entry(1, "zero", 8, 1) + ']', values);
    bt::oracle::Node node(node_name, scratch.root);
    bt::require(node.size() == 2, "all sidecar comparisons loaded");
    fails([&] { node.f64(std::span(values).first(1), "zero"); }, "out-of-order label fails before consumption");
    node.f64(std::span(values).first(1), "one");
    fails([&] { node.done(); }, "unconsumed entry fails");
    const std::array positive_zero{0.};
    fails([&] { node.f64(positive_zero, "zero"); }, "signed zero bits differ");
    node.f64(std::span(values).subspan(1), "zero");
    node.done();
    fails([&] { node.f64(positive_zero, "extra"); }, "over-consumption fails");
    fails([&] { bt::oracle::Node const missing("no/missing", scratch.root); }, "missing data fails closed");
}
void close_and_nan_contract() {
    Scratch const scratch;
    const double nan = std::bit_cast<double>(UINT64_C(0x7ff8000000001234));
    const std::array values{1., nan, std::numeric_limits<double>::infinity()};
    scratch.write('[' + entry(0, "close", 0, 3, "close", "0.01") + ']', values);
    bt::oracle::Node node(node_name, scratch.root);
    const std::array too_far{1.021, nan, std::numeric_limits<double>::infinity()};
    fails([&] { node.f64_close(too_far, "close"); }, "outside atol+rtol*expected fails");
    const std::array finite_instead_of_inf{1., nan, 1e100};
    fails([&] { node.f64_close(finite_instead_of_inf, "close"); }, "finite cannot match infinity through infinite tolerance product");
    const std::array within{1.019, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()};
    node.f64_close(within, "close");
    node.done();
    scratch.write('[' + entry(0, "payload", 0, 1) + ']', std::span(values).subspan(1, 1));
    bt::oracle::Node exact(node_name, scratch.root);
    const std::array other{std::bit_cast<double>(UINT64_C(0xfff8000000001234))};
    fails([&] { exact.f64(other, "payload"); }, "NaN sign remains significant for bitwise entries");
    exact.f64(std::span(values).subspan(1, 1), "payload");
    exact.done();
}
void malformed_contract() {
    Scratch scratch;
    const std::array values{1., 2.};
    scratch.write('[' + entry(0, "one", 0, 1) + ']', values);
    fails([&] { bt::oracle::Node const node(node_name, scratch.root); }, "unindexed payload rejects");
    scratch.write('[' + entry(0, "one", 8, 1) + ']', values);
    fails([&] { bt::oracle::Node const node(node_name, scratch.root); }, "noncontiguous offsets reject");
    scratch.write('[' + entry(1, "one", 0, 2) + ']', values);
    fails([&] { bt::oracle::Node const node(node_name, scratch.root); }, "nonzero first index rejects");
    scratch.write("[]", std::span<const double>{});
    fails([&] { bt::oracle::Node const node(node_name, scratch.root); }, "empty capture cannot pass");
}
void explicit_shape_contract() {
    Scratch const scratch;
    const std::array values{1.};
    scratch.write('[' + entry(0, "vector", 0, 1) + ']', values);
    bt::oracle::Node node(node_name, scratch.root);
    fails([&] { node.f64(values, std::span<const std::size_t>{}, "vector"); }, "scalar and length-one vector differ");
    const std::array<std::size_t, 2> matrix_shape{1, 1};
    fails([&] { node.f64(values, matrix_shape, "vector"); }, "equal element count does not erase rank");
    const std::array<std::size_t, 1> vector_shape{1};
    node.f64(values, vector_shape, "vector"); node.done();
}
} // namespace
int main() {
    return bt::run([] { path_contract(); ordered_bitwise_contract(); close_and_nan_contract(); malformed_contract(); explicit_shape_contract(); });
}
