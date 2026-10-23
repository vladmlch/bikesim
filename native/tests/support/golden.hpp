#pragma once

#include <cstddef>
#include <filesystem>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace bt::oracle {
struct Entry {
    std::size_t index = 0;
    std::string label, kind, dtype;
    std::vector<std::size_t> shape;
    std::vector<std::byte> bytes;
    double rtol = 0., atol = 0.;
};
[[nodiscard]] std::string node_path(std::string_view node);
[[nodiscard]] std::filesystem::path root();
[[nodiscard]] std::string_view platform();

class Node {
public:
    explicit Node(std::string_view node);
    // Test-owned scratch root; the platform component is still appended.
    Node(std::string_view node, const std::filesystem::path &root);
    [[nodiscard]] std::size_t size() const noexcept { return entries_.size(); }
    // Flattened convenience faces: span carries only an element count.
    // Ports with a source ndarray shape use the explicit-shape overloads
    // below so a scalar () cannot accidentally stand in for a vector (1,).
    void f64(std::span<const double> actual, std::string_view label);
    void f64_close(std::span<const double> actual, std::string_view label);
    void f64(std::span<const double> actual, std::span<const std::size_t> shape,
             std::string_view label);
    void f64_close(std::span<const double> actual, std::span<const std::size_t> shape,
                   std::string_view label);
    void bytes(std::span<const std::byte> actual, std::string_view dtype,
               std::span<const std::size_t> shape, std::string_view label);
    // Captured-input entries (label prefix "input." by convention): the
    // oracle froze the values a port must replay rather than compare.
    // Consumes the next entry, requiring a bitwise little-endian f8
    // payload, and returns its decoded doubles in C order.
    [[nodiscard]] std::vector<double> f64_input(std::string_view label);
    void done() const;
private:
    [[nodiscard]] const Entry &next(std::string_view label) const;
    [[nodiscard]] std::string context() const;
    std::string node_;
    std::vector<Entry> entries_;
    std::size_t cursor_ = 0;
};
} // namespace bt::oracle
