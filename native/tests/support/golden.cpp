#include "golden.hpp"
#include "migration_test.hpp"
#include "bike_golden_paths.hpp"
#include "io/hash.hpp"
#include "io/json.hpp"

#include <algorithm>
#include <bit>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace bt::oracle {
namespace {
using bike::io::Value;
[[noreturn]] void bad(std::string_view context, std::string_view reason) {
    throw std::invalid_argument(std::string(context) + ": " + std::string(reason));
}
const Value &required(const Value::Object &value, std::string_view name,
                      std::string_view context) {
    const auto *entry = bike::io::find_member(value, name);
    if (entry == nullptr) bad(context, "missing field " + std::string(name));
    return *entry;
}
std::size_t natural(const Value &value, std::string_view context) {
    const auto number = bike::io::number_i64(bike::io::as_number(value, context), context);
    if (number < 0 || !std::in_range<std::size_t>(number)) bad(context, "expected nonnegative size");
    return static_cast<std::size_t>(number);
}
std::string key(std::string_view source) {
    std::string safe;
    std::size_t cursor = 0;
    while (cursor < source.size()) {
        const auto cp = bike::io::detail::utf8_decode_one(source, cursor);
        if (cp == bike::io::detail::kInvalidCodepoint) bad("golden node", "invalid UTF-8");
        const bool allowed = (cp >= U'A' && cp <= U'Z') ||
            (cp >= U'a' && cp <= U'z') || (cp >= U'0' && cp <= U'9') ||
            cp == U'.' || cp == U'_' || cp == U'-';
        safe.push_back(allowed ? static_cast<char>(cp) : '_');
    }
    if (safe.size() > 150) {
        safe.resize(140);
        safe += '-' + bike::io::sha256(std::as_bytes(std::span(source))).substr(0, 8);
    }
    if (safe.empty() || safe == "." || safe == "..") bad("golden node", "empty or unsafe component");
    return safe;
}
std::size_t item_size(std::string_view dtype) {
    if (dtype.size() < 3 || std::string_view("<>=|").find(dtype[0]) == std::string_view::npos ||
        std::string_view("biufcSUMmV").find(dtype[1]) == std::string_view::npos)
        bad("golden dtype", "expected primitive pickle-free descriptor");
    std::size_t width = 0;
    const auto suffix = dtype.find('[');
    const auto digits = dtype.substr(2, suffix == std::string_view::npos ? suffix : suffix - 2);
    const auto *end = std::to_address(digits.end());
    const auto parsed = std::from_chars(std::to_address(digits.begin()), end, width);
    if (digits.empty() || parsed.ec != std::errc{} || parsed.ptr != end)
        bad("golden dtype", "invalid element width");
    if (suffix != std::string_view::npos &&
        ((dtype[1] != 'M' && dtype[1] != 'm') || dtype.back() != ']'))
        bad("golden dtype", "invalid temporal unit");
    if (dtype[1] == 'U') {
        if (width > std::numeric_limits<std::size_t>::max() / 4) bad("golden dtype", "width overflow");
        width *= 4;
    }
    if (width == 0 && dtype[1] != 'S' && dtype[1] != 'U' && dtype[1] != 'V')
        bad("golden dtype", "zero numeric width");
    return width;
}
std::uint64_t bits(const Entry &entry, std::size_t index) {
    std::uint64_t result = 0;
    for (std::size_t byte = 0; byte < 8; ++byte)
        result |= static_cast<std::uint64_t>(std::to_integer<unsigned int>(entry.bytes[index * 8 + byte])) << (byte * 8);
    return result;
}
std::string hex(std::uint64_t value) {
    std::ostringstream out;
    out << "0x" << std::hex << std::setw(16) << std::setfill('0') << value;
    return out.str();
}
} // namespace

std::filesystem::path root() { return BIKE_GOLDEN_ROOT; }
std::string_view platform() { return BIKE_GOLDEN_PLATFORM; }
std::string node_path(std::string_view node) {
    const auto split = node.find("::");
    if (split == std::string_view::npos) bad("golden node", "expected file::test");
    return key(node.substr(0, split)) + '/' + key(node.substr(split + 2));
}
Node::Node(std::string_view node) : Node(node, root()) {}
Node::Node(std::string_view node, const std::filesystem::path &directory) : node_(node) {
    const auto base = directory / platform() / node_path(node);
    const auto meta_path = std::filesystem::path(base.string() + ".json");
    const auto payload_path = std::filesystem::path(base.string() + ".bin");
    // Resolve symlinks before reading, even for the test-only root override.
    const auto canonical_root = std::filesystem::weakly_canonical(directory);
    for (const auto &path : {meta_path, payload_path}) {
        const auto relative = std::filesystem::weakly_canonical(path).lexically_relative(canonical_root);
        if (relative.empty() || relative.is_absolute() || *relative.begin() == "..") bad(node_, "fixture escapes golden root");
    }
    const auto document = bike::io::parse_json(bt::text(meta_path));
    const auto &metadata = bike::io::as_array(document, node_);
    const auto payload = bt::text(payload_path);
    if (metadata.empty()) bad(node_, "empty sidecar is not a captured comparison");
    std::size_t offset = 0;
    for (const auto &value : metadata) {
        const auto &object = bike::io::as_object(value, node_);
        Entry entry;
        entry.index = natural(required(object, "index", node_), node_);
        if (entry.index != entries_.size()) bad(node_, "noncontiguous comparison index");
        if (bike::io::as_string(required(object, "node", node_), node_) != node_)
            bad(node_, "sidecar belongs to another pytest node (path collision)");
        entry.label = bike::io::as_string(required(object, "label", node_), node_);
        entry.kind = bike::io::as_string(required(object, "kind", node_), node_);
        entry.dtype = bike::io::as_string(required(object, "dtype", node_), node_);
        if (entry.kind != "bitwise" && entry.kind != "close") bad(node_, "unknown comparison kind");
        std::size_t count = 1;
        for (const auto &dimension : bike::io::as_array(required(object, "shape", node_), node_)) {
            const auto size = natural(dimension, node_);
            if (size && count > std::numeric_limits<std::size_t>::max() / size) bad(node_, "shape overflow");
            count *= size;
            entry.shape.push_back(size);
        }
        const auto width = item_size(entry.dtype);
        if (width && count > std::numeric_limits<std::size_t>::max() / width) bad(node_, "byte count overflow");
        const auto nbytes = natural(required(object, "nbytes", node_), node_);
        if (nbytes != count * width || natural(required(object, "offset", node_), node_) != offset)
            bad(node_, "shape, dtype and contiguous payload offsets disagree");
        if (offset > payload.size() || nbytes > payload.size() - offset) bad(node_, "truncated binary payload");
        const auto view = std::as_bytes(std::span(payload)).subspan(offset, nbytes);
        entry.bytes.assign(view.begin(), view.end());
        offset += nbytes;
        const auto &rtol = required(object, "rtol", node_);
        const auto &atol = required(object, "atol", node_);
        if (entry.kind == "close") {
            if (entry.dtype != "<f8") bad(node_, "close requires little-endian binary64");
            entry.rtol = bike::io::number_double(bike::io::as_number(rtol, node_), node_);
            entry.atol = bike::io::number_double(bike::io::as_number(atol, node_), node_);
            if (entry.rtol < 0 || entry.atol < 0) bad(node_, "negative comparison tolerance");
        } else if (!bike::io::is_null(rtol) || !bike::io::is_null(atol)) bad(node_, "bitwise entry has tolerances");
        entries_.push_back(std::move(entry));
    }
    if (offset != payload.size()) bad(node_, "unindexed bytes remain in binary payload");
}
std::string Node::context() const { return node_ + " comparison " + std::to_string(cursor_); }
const Entry &Node::next(std::string_view label) const {
    bt::require(cursor_ < entries_.size(), context() + ": no more captured entries");
    const auto &entry = entries_[cursor_];
    bt::require(entry.label == label, context() + ": label expected '" + entry.label + "', received '" + std::string(label) + "'");
    return entry;
}
void Node::f64(std::span<const double> actual, std::string_view label) {
    const auto &entry = next(label);
    bt::require(entry.kind == "bitwise" && entry.dtype == "<f8", context() + ": expected bitwise binary64");
    bt::require(actual.size() == entry.bytes.size() / 8, context() + ": element count differs");
    for (std::size_t i = 0; i < actual.size(); ++i) {
        const auto observed = std::bit_cast<std::uint64_t>(actual[i]);
        const auto expected = bits(entry, i);
        bt::require(observed == expected, context() + " label '" + entry.label + "' element " +
            std::to_string(i) + ": actual=" + hex(observed) + " expected=" + hex(expected));
    }
    ++cursor_;
}
void Node::f64_close(std::span<const double> actual, std::string_view label) {
    const auto &entry = next(label);
    bt::require(entry.kind == "close" && entry.dtype == "<f8", context() + ": expected close binary64");
    bt::require(actual.size() == entry.bytes.size() / 8, context() + ": element count differs");
    for (std::size_t i = 0; i < actual.size(); ++i) {
        const double expected = std::bit_cast<double>(bits(entry, i));
        const bool equal = actual[i] == expected || (std::isnan(actual[i]) && std::isnan(expected)) ||
            (std::isfinite(actual[i]) && std::isfinite(expected) &&
             std::abs(actual[i] - expected) <= entry.atol + entry.rtol * std::abs(expected));
        bt::require(equal, context() + " label '" + entry.label + "' element " + std::to_string(i) +
            ": actual=" + hex(std::bit_cast<std::uint64_t>(actual[i])) +
            " expected=" + hex(std::bit_cast<std::uint64_t>(expected)) +
            ": outside recorded tolerance");
    }
    ++cursor_;
}
void Node::f64(std::span<const double> actual, std::span<const std::size_t> shape,
               std::string_view label) {
    bt::require(std::ranges::equal(next(label).shape, shape), context() + ": shape differs");
    f64(actual, label);
}
void Node::f64_close(std::span<const double> actual, std::span<const std::size_t> shape,
                     std::string_view label) {
    bt::require(std::ranges::equal(next(label).shape, shape), context() + ": shape differs");
    f64_close(actual, label);
}
void Node::bytes(std::span<const std::byte> actual, std::string_view dtype,
                 std::span<const std::size_t> shape, std::string_view label) {
    const auto &entry = next(label);
    bt::require(entry.kind == "bitwise" && entry.dtype == dtype && std::ranges::equal(entry.shape, shape), context() + ": kind, dtype or shape differs");
    bt::require(std::ranges::equal(entry.bytes, actual), context() + " label '" + entry.label + "': bytes differ");
    ++cursor_;
}
std::vector<double> Node::f64_input(std::string_view label) {
    const auto &entry = next(label);
    bt::require(entry.kind == "bitwise" && entry.dtype == "<f8", context() + ": expected bitwise binary64 input");
    std::vector<double> result(entry.bytes.size() / 8);
    for (std::size_t i = 0; i < result.size(); ++i)
        result[i] = std::bit_cast<double>(bits(entry, i));
    ++cursor_;
    return result;
}
std::string_view Node::peek_label() const {
    if (cursor_ >= entries_.size()) return {};
    return entries_[cursor_].label;
}
std::span<const std::size_t> Node::peek_shape() const {
    if (cursor_ >= entries_.size()) return {};
    return entries_[cursor_].shape;
}
void Node::done() const {
    bt::require(cursor_ == entries_.size(), context() + ": unconsumed captured comparisons remain");
}
} // namespace bt::oracle
