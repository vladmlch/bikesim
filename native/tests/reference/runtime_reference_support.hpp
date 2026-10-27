// runtime_reference_support.hpp — shared plumbing for the 3.25
// runtime-batch reference ports (test_native_runtime_{bootstrap,step,
// commands,ownership}.cpp; owner task T53).
//
// The Python sources construct NativeRideRuntime from a fresh
// capture_bootstrap() of the welded-physics + rough_uphill_savage ride
// (tests/reference/_native_runtime_support.py::make_python_ride). The C++
// ports run the identical boundary against the recorded welded capture
// tests/fixtures/migration/bootstrap/welded — the same model, config
// envelope (dt .00125, control period 4 steps, decimation 1, non-strict)
// and t=0 state document. All ported assertions are native-vs-native or
// literals: the six/four live-oracle parity reds of the source files stay
// `planned` in the port map (see task-C325-report.md) and are not
// reproduced here, so this header holds no golden nodes.
#pragma once

#include "support/migration_test.hpp"
#include "support/temporary_directory.hpp"
#include "diag.hpp"

#include "io/hash.hpp"
#include "io/json.hpp"
#include "io/value.hpp"
#include "runtime/bootstrap.hpp"
#include "runtime/bootstrap_io.hpp"
#include "runtime/runtime.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace runtime_reference {

namespace bio = bike::io;
namespace fs = std::filesystem;
namespace sw = runtime::sample_wire;

[[nodiscard]] inline fs::path welded_dir() {
    return bt::fixture("bootstrap/welded");
}

// The welded capture as a fresh owned runtime — create_native_ride()'s
// native-side shape (model image + decoded documents, t=0). The by-value
// RuntimeBootstrap materializes on this frame like the T09 decode frames.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
[[nodiscard]] inline std::unique_ptr<runtime::NativeRideRuntime> ride() {
    return std::make_unique<runtime::NativeRideRuntime>(
        runtime::read_bootstrap(welded_dir()));
}
NATIVE_DIAG_POP

// ---- document plumbing ----------------------------------------------------

[[nodiscard]] inline bio::Value parse_doc(const fs::path &path) {
    return bio::parse_json(bt::text(path));
}

[[nodiscard]] inline const bio::Value::Object &object_of(
    const bio::Value &value, std::string_view what) {
    const auto *const object = std::get_if<bio::Value::Object>(&value.data);
    if (object == nullptr)
        throw std::invalid_argument(std::string(what) + ": not an object");
    return *object;
}
[[nodiscard]] inline bio::Value::Object &object_of(bio::Value &value,
                                                   std::string_view what) {
    auto *const object = std::get_if<bio::Value::Object>(&value.data);
    if (object == nullptr)
        throw std::invalid_argument(std::string(what) + ": not an object");
    return *object;
}

[[nodiscard]] inline const bio::Value &member(const bio::Value &value,
                                              std::string_view key) {
    const bio::Value *const found =
        bio::find_member(object_of(value, "member"), key);
    if (found == nullptr)
        throw std::invalid_argument("document member missing: " +
                                    std::string(key));
    return *found;
}
[[nodiscard]] inline bio::Value &member(bio::Value &value,
                                        std::string_view key) {
    auto &object = object_of(value, "member");
    for (auto &[k, v] : object)
        if (k == key) return v;
    throw std::invalid_argument("document member missing: " +
                                std::string(key));
}
inline void erase_member(bio::Value &value, std::string_view key) {
    auto &object = object_of(value, "erase_member");
    const auto at = std::size(object);
    std::erase_if(object,
                  [&key](const std::pair<std::string, bio::Value> &entry) {
                      return entry.first == key;
                  });
    if (std::size(object) == at)
        throw std::invalid_argument("erase_member missing: " +
                                    std::string(key));
}
// Replace-or-append — the test document equivalent of `doc[key] = value`
// on an insertion-ordered mapping.
inline void set_member(bio::Value &value, std::string key,
                       bio::Value replacement) {
    auto &object = object_of(value, "set_member");
    for (auto &[k, v] : object)
        if (k == key) {
            v = std::move(replacement);
            return;
        }
    object.emplace_back(std::move(key), std::move(replacement));
}

[[nodiscard]] inline bio::Value num(std::int64_t value) {
    return bio::Value{bio::Number{std::to_string(value)}};
}
[[nodiscard]] inline bio::Value nan_value() {
    // The resolved $array sentinel for a nonfinite <f8 element.
    return bio::Value{bio::Tagged{.kind = "float64",
                                  .fields = {{"value", "nan"}}}};
}
[[nodiscard]] inline std::int64_t as_int(const bio::Value &value) {
    const auto *const number = std::get_if<bio::Number>(&value.data);
    if (number == nullptr)
        throw std::invalid_argument("expected an integer document member");
    return std::stoll(number->lexeme);
}

[[nodiscard]] inline runtime::ModelImage model_image(const fs::path &dir) {
    const std::string bytes = bt::text(dir / "model.mjb");
    runtime::ModelImage out;
    out.mjb.reserve(bytes.size());
    for (const char unit : bytes)
        out.mjb.push_back(static_cast<std::byte>(unit));
    out.sha256 = bio::sha256(out.mjb);
    return out;
}

// The resolved (tags expanded) fixture documents — the two owned inputs
// compose_bootstrap() consumes, exactly like the adapter's materialized
// dicts before NativeRideRuntime(...).
struct ResolvedDocs {
    bio::Value config;
    bio::Value state;
};
[[nodiscard]] inline ResolvedDocs resolved_docs(const fs::path &dir) {
    return {.config = runtime::resolve_bootstrap_document(
                parse_doc(dir / "config.json"), dir),
            .state = runtime::resolve_bootstrap_document(
                parse_doc(dir / "state.json"), dir)};
}

// The adapter-path composition order: config envelope decode at the call
// site, then compose_bootstrap() runs the digest/model/state validation.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
[[nodiscard]] inline std::unique_ptr<runtime::NativeRideRuntime>
compose_runtime(const bio::Value &config_doc, const bio::Value &state_doc,
                const fs::path &dir) {
    return std::make_unique<runtime::NativeRideRuntime>(
        runtime::compose_bootstrap(
            runtime::decode_runtime_config(config_doc),
            model_image(dir), state_doc));
}
NATIVE_DIAG_POP
[[nodiscard]] inline std::unique_ptr<runtime::NativeRideRuntime>
compose_runtime(const ResolvedDocs &docs) {
    return compose_runtime(docs.config, docs.state, welded_dir());
}

// ---- assertions -----------------------------------------------------------

// std::invalid_argument whose what() names the field path — the Python
// pytest.raises(ValueError, match='field.path') contract.
inline void expect_invalid(std::string_view field,
                           const std::function<void()> &action) {
    try {
        action();
    } catch (const std::invalid_argument &failure) {
        bt::require(std::string(failure.what()).find(field) !=
                        std::string::npos,
                    "rejection names the field path");
        return;
    }
    bt::require(false, "expected a rejection naming the field path");
}

// Any exception (RuntimeError/logic_error/EngineFailure — everything maps
// to Python's exception surface) whose message carries the marker.
inline void expect_error(std::string_view marker,
                         const std::function<void()> &action) {
    try {
        action();
    } catch (const std::exception &failure) {
        bt::require(std::string(failure.what()).find(marker) !=
                        std::string::npos,
                    "error message carries the marker");
        return;
    }
    bt::require(false, "expected a rejecting exception");
}

// assert_bitwise_equal — uint64 identity per lane, NaN payloads and
// signed zero significant.
inline void require_bitwise(std::span<const double> actual,
                            std::span<const double> expected,
                            std::string_view what) {
    bt::require(actual.size() == expected.size(), what);
    for (std::size_t i = 0; i < actual.size() && i < expected.size(); ++i)
        bt::require(std::bit_cast<std::uint64_t>(actual[i]) ==
                        std::bit_cast<std::uint64_t>(expected[i]),
                    what);
}
[[nodiscard]] inline bool bitwise_equal(std::span<const double> a,
                                        std::span<const double> b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i)
        if (std::bit_cast<std::uint64_t>(a[i]) !=
            std::bit_cast<std::uint64_t>(b[i]))
            return false;
    return true;
}

// assert_tree_close at zero tolerance — the native-vs-native wire tree
// identity: same variant, same order, doubles bitwise.
[[nodiscard]] inline bool wire_bitwise(const runtime::Wire &a,
                                       const runtime::Wire &b) {
    if (a.value.index() != b.value.index()) return false;
    switch (a.value.index()) {
    case 0: return true; // monostate
    case 1:
        return std::get<bool>(a.value) == std::get<bool>(b.value);
    case 2:
        return std::get<std::int64_t>(a.value) ==
               std::get<std::int64_t>(b.value);
    case 3:
        return std::bit_cast<std::uint64_t>(std::get<double>(a.value)) ==
               std::bit_cast<std::uint64_t>(std::get<double>(b.value));
    case 4:
        return std::get<std::string>(a.value) ==
               std::get<std::string>(b.value);
    case 5: {
        const auto &aa = std::get<runtime::WireArray>(a.value);
        const auto &bb = std::get<runtime::WireArray>(b.value);
        if (aa.size() != bb.size()) return false;
        for (std::size_t i = 0; i < aa.size(); ++i)
            if (!wire_bitwise(aa[i], bb[i])) return false;
        return true;
    }
    default: {
        const auto &ao = std::get<runtime::WireObject>(a.value);
        const auto &bo = std::get<runtime::WireObject>(b.value);
        if (ao.size() != bo.size()) return false;
        for (std::size_t i = 0; i < ao.size(); ++i)
            if (ao[i].first != bo[i].first ||
                !wire_bitwise(ao[i].second, bo[i].second))
                return false;
        return true;
    }
    }
}
[[nodiscard]] inline bool wire_bitwise(const runtime::WireObject &a,
                                       const runtime::WireObject &b) {
    return wire_bitwise(runtime::Wire{runtime::WireObject{a}},
                        runtime::Wire{runtime::WireObject{b}});
}
inline void require_wire_bitwise(const runtime::WireObject &a,
                                 const runtime::WireObject &b,
                                 std::string_view what) {
    bt::require(wire_bitwise(a, b), what);
}

// _native_runtime_support.assert_tree_close: unordered object members,
// positional arrays, strings/bools exact, numbers within atol/rtol 1e-9.
[[nodiscard]] inline bool tree_close(const runtime::Wire &actual,
                                     const runtime::Wire &expected) {
    constexpr double kClose = 1e-9;
    if (const auto *a = std::get_if<double>(&actual.value)) {
        const auto *e = std::get_if<double>(&expected.value);
        if (e != nullptr)
            return std::isfinite(*a) && std::isfinite(*e) &&
                   std::abs(*a - *e) <= std::max(kClose, kClose * std::abs(*e));
        if (const auto *i = std::get_if<std::int64_t>(&expected.value))
            return std::isfinite(*a) &&
                   std::abs(*a - static_cast<double>(*i)) <= kClose;
        return false;
    }
    if (const auto *a = std::get_if<std::int64_t>(&actual.value)) {
        if (const auto *e = std::get_if<std::int64_t>(&expected.value))
            return *a == *e;
        if (const auto *e = std::get_if<double>(&expected.value))
            return std::isfinite(*e) &&
                   std::abs(static_cast<double>(*a) - *e) <= kClose;
        return false;
    }
    if (actual.value.index() != expected.value.index()) return false;
    switch (actual.value.index()) {
    case 0: return true;
    case 1:
        return std::get<bool>(actual.value) == std::get<bool>(expected.value);
    case 4:
        return std::get<std::string>(actual.value) ==
               std::get<std::string>(expected.value);
    case 5: {
        const auto &a = std::get<runtime::WireArray>(actual.value);
        const auto &e = std::get<runtime::WireArray>(expected.value);
        if (a.size() != e.size()) return false;
        for (std::size_t i = 0; i < a.size(); ++i)
            if (!tree_close(a[i], e[i])) return false;
        return true;
    }
    default: {
        const auto &a = std::get<runtime::WireObject>(actual.value);
        const auto &e = std::get<runtime::WireObject>(expected.value);
        if (a.size() != e.size()) return false;
        for (const auto &[name, value] : a) {
            const runtime::Wire *other = sw::find(e, name);
            if (other == nullptr || !tree_close(value, *other)) return false;
        }
        return true;
    }
    }
}
inline void require_tree_close(const runtime::Wire &actual,
                               const runtime::Wire &expected,
                               std::string_view what) {
    bt::require(tree_close(actual, expected), what);
}
inline void require_tree_close(const runtime::WireObject &actual,
                               const runtime::WireObject &expected,
                               std::string_view what) {
    require_tree_close(runtime::Wire{runtime::WireObject{actual}},
                       runtime::Wire{runtime::WireObject{expected}}, what);
}

// ---- wire navigation for accounting/snapshot assertions -------------------

[[nodiscard]] inline const runtime::Wire &wire_member(
    const runtime::WireObject &object, std::string_view key) {
    return sw::required(object, key);
}
[[nodiscard]] inline std::int64_t wire_int(const runtime::WireObject &o,
                                           std::string_view key) {
    return sw::integer(wire_member(o, key));
}
[[nodiscard]] inline const runtime::WireObject &wire_obj(
    const runtime::WireObject &o, std::string_view key) {
    return sw::object(wire_member(o, key));
}
[[nodiscard]] inline bool wire_has(const runtime::WireObject &o,
                                   std::string_view key) {
    return sw::find(o, key) != nullptr;
}

} // namespace runtime_reference
