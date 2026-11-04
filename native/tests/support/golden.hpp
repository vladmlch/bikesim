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
    // Recorded tolerance spellings ("null" for bitwise entries), kept so a
    // record-mode rewrite reproduces the sidecar byte for byte.
    std::string rtol_text, atol_text;
};
[[nodiscard]] std::string key(std::string_view source);
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
    // Same replay contract for byte documents (recorded JSON emissions and
    // other "|u1" payloads the port parses instead of comparing): consumes
    // the next entry and returns its recorded bytes untouched.
    [[nodiscard]] std::vector<std::byte> bytes_input(std::string_view label);
    // Readback for kind="close" <f8 entries: consumes the next entry and
    // returns the recorded values with the entry's comparison tolerances —
    // for test-harness comparators that re-verify a subset of a recorded
    // close array against a normalized expected side.
    struct CloseReadback {
        std::vector<double> values;
        double rtol;
        double atol;
    };
    [[nodiscard]] CloseReadback f64_close_input(std::string_view label);
    // The next unconsumed entry's label without advancing the cursor —
    // for replay branches (output tuple vs error capture) that choose
    // their consumer by the recorded kind. Empty at end of node.
    [[nodiscard]] std::string_view peek_label() const;
    // Cursor not yet at the end: distinguishes an end-of-node empty
    // peek_label() from a legitimately ''-labeled recorded entry.
    [[nodiscard]] bool has_next() const noexcept {
        return cursor_ < entries_.size();
    }
    // The next unconsumed entry's declared shape (same lifetime rules).
    [[nodiscard]] std::span<const std::size_t> peek_shape() const;
    // Record mode (environment BIKE_GOLDEN_RECORD=1): every comparison keeps
    // the recorded label, kind, dtype and tolerances but stores the actual
    // payload instead of comparing it (a payload still within tolerance is
    // kept as recorded), and done() rewrites this node's sidecar and
    // payload. Used to re-record goldens after a reviewed model
    // change; a test that throws before done() writes nothing.
    [[nodiscard]] static bool recording();
    // Record mode only: replace the payload of the entry consumed last by
    // f64_close_input()/bytes_input() — the record path of comparators that
    // read the expected side and compare it themselves. A binary64 payload
    // the values still match (within the entry's tolerances) is kept.
    void record_consumed(std::span<const double> values);
    void record_consumed(std::span<const std::byte> bytes);
    // Record mode only: delete the entry consumed last — the actual side no
    // longer produces it (a removed channel). Later entries are renumbered.
    void drop_consumed();
    void done() const;
private:
    [[nodiscard]] const Entry &next(std::string_view label) const;
    [[nodiscard]] std::string context() const;
    [[nodiscard]] static bool matches(const Entry &entry, std::span<const double> values);
    void store(Entry &entry, std::span<const double> values);
    void write() const;
    std::string node_;
    // The golden root this node was read from; record mode writes back
    // under it. Held as text: a std::filesystem::path member next to the
    // string_view constructor trips cppcheck's dangling-view check at
    // every `return Node(name)` factory.
    std::string root_;
    std::vector<Entry> entries_;
    std::size_t cursor_ = 0;
};
} // namespace bt::oracle
