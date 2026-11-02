// test_native_state_fuzz.cpp — C++ port of
// tests/reference/test_native_state_fuzz.py (T53 batch 3.27).
//
// The source drives Hypothesis with @given(st.data()): mutate() draws a
// unique nonempty subset of {set, del, add} ops and applies each to a random
// dict path of a deep-copied drive_state() snapshot with a drawn
// JSON_VALUE (scalars and recursive lists/dicts, bounded fan-out).
//
// This port keeps the same mutation model and the same property as a
// DETERMINISTIC seeded property test — no hypothesis, no golden fixture:
//
//   * a mutated drive snapshot passed to DriveWriter::restore either
//     succeeds or throws a member of the sanctioned contract set — never a
//     crash, a foreign exception type, or silent corruption: a rejected
//     restore must leave state() byte-identical (canonical JSON is the
//     freeze() twin);
//   * a successful mutation restores back to the baseline (the object stays
//     restorable for the next example);
//   * Stepper::set_state fuzzed over f64 spans of arbitrary width and
//     arbitrary doubles (NaN/Inf/-Inf/subnormals/extremes) plus arbitrary
//     double times is likewise contracted — and because every width/
//     finiteness check runs before mj_resetData, a rejection additionally
//     leaves every live buffer bitwise unchanged.
//
// Exception surface: the Python CONTRACT_ERRORS are (ValueError, TypeError,
// OverflowError, RuntimeError). On this side of the typed API the sanctioned
// set is the owned validation vocabulary: std::invalid_argument (ValueError
// and every kind/cast guard — TypeError's analog), std::out_of_range /
// std::overflow_error (OverflowError), and std::runtime_error including
// engine::EngineFailure (RuntimeError). std::bad_alloc, std::bad_cast and
// other logic_error subclasses are contract defects and fail the example.
// The source's non-array JSON draws against set_state exercise the binding
// cast layer, which has no typed-API counterpart — those sub-draws are
// `python-only` in the port map.
#include <mujoco/mujoco.h>

#include "drivetrain_reference_support.hpp"
#include "support/fixture_test.hpp"
#include "engine_call.hpp"
#include "io/json.hpp"
#include "io/value.hpp"
#include "model_access.hpp"
#include "stepper.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
#include <functional>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

using bike::io::Value;
using Object = Value::Object;
using Array = Value::Array;
using namespace runtime::setupvalue;

// ---------------------------------------------------------------------------
// Deterministic generator — splitmix64, fixed seeds, no environment state.
// ---------------------------------------------------------------------------
struct Rng {
    std::uint64_t state;
    explicit Rng(std::uint64_t seed) : state(seed) {}
    std::uint64_t next() {
        std::uint64_t z = (state += 0x9E3779B97F4A7C15ULL);
        z = (z ^ (z >> 30U)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27U)) * 0x94D049BB133111EBULL;
        return z ^ (z >> 31U);
    }
    std::size_t pick(std::size_t bound) { // bound > 0
        return static_cast<std::size_t>(next() % bound);
    }
};

// ---------------------------------------------------------------------------
// JSON_VALUE mirror: none | bool | int | float | text(max 12) — recursive
// lists/dicts bounded at 5 children and 4 levels (hypothesis' max_leaves=25
// envelope keeps documents small; 4 levels × 5 children exceeds it only for
// pathological draws, which the property tolerates as generic inputs).
// ---------------------------------------------------------------------------
[[nodiscard]] Value random_text(Rng &rng) {
    static constexpr std::string_view alphabet =
        "abz09_./+-\\\"' \t<>&=eE"; // printable + structural characters
    std::string out;
    const std::size_t length = rng.pick(13);
    out.reserve(length);
    for (std::size_t i = 0; i < length; ++i)
        out.push_back(alphabet[rng.pick(alphabet.size())]);
    return text(out);
}

[[nodiscard]] Value random_scalar(Rng &rng) {
    switch (rng.pick(6)) {
    case 0:
        return Value{}; // null
    case 1:
        return Value{rng.pick(2) == 0}; // bool
    case 2: { // integers — including int64 extremes and a >int64 lexeme
        switch (rng.pick(6)) {
        case 0:
            return integer(static_cast<std::int64_t>(rng.next()));
        case 1:
            return integer(0);
        case 2:
            return integer(-1);
        case 3:
            return integer(
                static_cast<std::int64_t>(rng.pick(1'000'000)));
        case 4:
            return integer(std::numeric_limits<std::int64_t>::max());
        default:
            return Value{bike::io::Number{
                "99999999999999999999999999999999"}};
        }
    }
    case 3:
    case 4: { // floats — hypothesis' allow_nan/allow_infinity domain
        static constexpr std::array<double, 10> specials{
            0., -0., 1., -1., .125, 1e300,
            std::numeric_limits<double>::denorm_min(),
            std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::quiet_NaN()};
        if (rng.pick(4) == 0)
            return real(std::bit_cast<double>(rng.next()));
        return real(specials[rng.pick(specials.size())]);
    }
    default:
        return random_text(rng);
    }
}

[[nodiscard]] Value random_value(Rng &rng, unsigned depth) { // NOLINT(misc-no-recursion) recursion mirrors the document tree
    if (depth == 0 || rng.pick(3) == 0) return random_scalar(rng);
    if (rng.pick(2) == 0) {
        Array out;
        const std::size_t count = rng.pick(6);
        out.reserve(count);
        for (std::size_t i = 0; i < count; ++i)
            out.push_back(random_value(rng, depth - 1));
        return Value{std::move(out)};
    }
    Object out;
    const std::size_t count = rng.pick(6);
    out.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        // Dict keys: text(max 8) over a narrower alphabet.
        static constexpr std::string_view key_alphabet = "abcxyz_.09";
        std::string key;
        const std::size_t length = rng.pick(9);
        for (std::size_t c = 0; c < length; ++c)
            key.push_back(key_alphabet[rng.pick(key_alphabet.size())]);
        out.emplace_back(std::move(key), random_value(rng, depth - 1));
    }
    return Value{std::move(out)};
}

// ---------------------------------------------------------------------------
// dict_paths / apply_path — the source helpers, operating on Value objects
// (dicts) only; a walk that hits a missing key or non-object returns
// failure exactly like the source's (KeyError, TypeError) continue.
// ---------------------------------------------------------------------------
using Path = std::vector<std::string>;

void dict_paths(const Value &value, Path &prefix, std::vector<Path> &out) { // NOLINT(misc-no-recursion) recursion mirrors the document tree
    const auto *object = std::get_if<Object>(&value.data);
    if (object == nullptr) return;
    for (const auto &[key, child] : *object) {
        prefix.push_back(key);
        out.push_back(prefix);
        dict_paths(child, prefix, out);
        prefix.pop_back();
    }
}

[[nodiscard]] std::vector<Path> dict_paths(const Value &value) {
    std::vector<Path> out;
    Path prefix;
    dict_paths(value, prefix, out);
    return out;
}

[[nodiscard]] Value *walk(Value &root, std::span<const std::string> keys) {
    Value *node = &root;
    for (const auto &key : keys) {
        auto *object = std::get_if<Object>(&node->data);
        if (object == nullptr) return nullptr;
        auto const found =
            std::ranges::find(*object, key, &Object::value_type::first);
        if (found == object->end()) return nullptr;
        node = &found->second;
    }
    return node;
}

void apply_set(Value &root, const Path &path, Value value) {
    Value *parent =
        walk(root, std::span<const std::string>{path}.first(path.size() - 1));
    auto *object =
        parent == nullptr ? nullptr : std::get_if<Object>(&parent->data);
    if (object == nullptr) return;
    auto const found =
        std::ranges::find(*object, path.back(), &Object::value_type::first);
    if (found == object->end())
        object->emplace_back(path.back(), std::move(value));
    else
        found->second = std::move(value);
}

void apply_del(Value &root, const Path &path) {
    Value *parent =
        walk(root, std::span<const std::string>{path}.first(path.size() - 1));
    auto *object =
        parent == nullptr ? nullptr : std::get_if<Object>(&parent->data);
    if (object == nullptr) return;
    std::erase_if(*object, [&](const auto &entry) {
        return entry.first == path.back();
    });
}

// mutate(): a unique nonempty subset of {set, del, add} in drawn order.
[[nodiscard]] Value mutate(const Value &baseline, Rng &rng) {
    Value out = baseline;
    std::array<int, 3> ops{0, 1, 2}; // set, del, add
    const std::size_t count = 1 + rng.pick(ops.size());
    for (std::size_t i = 0; i < count; ++i)
        std::swap(ops[i], ops[i + rng.pick(ops.size() - i)]);
    for (std::size_t i = 0; i < count; ++i) {
        const std::vector<Path> paths = dict_paths(out);
        switch (ops[i]) {
        case 0: // set
            if (!paths.empty())
                apply_set(out, paths[rng.pick(paths.size())],
                          random_value(rng, 4));
            break;
        case 1: // del
            if (!paths.empty())
                apply_del(out, paths[rng.pick(paths.size())]);
            break;
        default: // add — top-level `extra_<0..99>` member
            apply_set(out, {"extra_" + std::to_string(rng.pick(100))},
                      random_value(rng, 4));
            break;
        }
    }
    return out;
}

// ---------------------------------------------------------------------------
// The sanctioned contract surface — see the header comment.
// ---------------------------------------------------------------------------
[[nodiscard]] bool sanctioned(const std::exception &failure) {
    return dynamic_cast<const std::invalid_argument *>(&failure) != nullptr ||
           dynamic_cast<const std::out_of_range *>(&failure) != nullptr ||
           dynamic_cast<const std::overflow_error *>(&failure) != nullptr ||
           dynamic_cast<const std::length_error *>(&failure) != nullptr ||
           dynamic_cast<const std::runtime_error *>(&failure) != nullptr;
}

// ---------------------------------------------------------------------------
// test_drive_state_restore_fuzz — 150 seeded examples over the
// ideal_mid_drive pair() rig (the source's module-scope drive_stepper).
// ---------------------------------------------------------------------------
void test_drive_state_restore_fuzz() {
    const auto owner = drivetrain_reference::pair();
    Rng rng{0xB1357A77EULL};
    std::size_t accepted = 0, rejected = 0;
    for (int example = 0; example < 150; ++example) {
        const Value baseline = drivetrain_reference::state(*owner);
        Value mutated = mutate(baseline, rng);
        try {
            drivetrain_reference::restore(*owner, mutated);
        } catch (const std::exception &failure) {
            bt::require(sanctioned(failure),
                        "drive fuzz rejection stays inside the contract "
                        "set (" + std::string{failure.what()} + ")");
            ++rejected;
            bt::require(
                bike::io::canonical_json(drivetrain_reference::state(*owner)) ==
                    bike::io::canonical_json(baseline),
                "a rejected fuzzed drive restore leaves the snapshot "
                "byte-identical");
            continue;
        }
        ++accepted;
        // A successful mutation still leaves a restorable object.
        drivetrain_reference::restore(*owner, baseline);
    }
    // The draw profile must reach both arms — a vacuous all-reject run would
    // pin nothing.
    bt::require(accepted > 0 && rejected > 0,
                "fuzz draws exercise accepted and rejected paths");
}

// ---------------------------------------------------------------------------
// test_set_state_fuzz — 100 seeded examples; f64 spans of arbitrary width
// (0..w+3 in the source's bounds — width-mismatch and nonfinite arms alike)
// and arbitrary double times.
// ---------------------------------------------------------------------------
struct LiveBuffers {
    std::vector<double> qpos;
    std::vector<double> qvel;
    std::vector<double> qacc;
    std::vector<double> qfrc_constraint;
    std::vector<double> efc_force;
    std::vector<double> ctrl;
    std::vector<double> qfrc_applied;
    double time = 0.;
};

[[nodiscard]] std::vector<double> copy_of(std::span<const double> values) {
    return {values.begin(), values.end()};
}

[[nodiscard]] LiveBuffers live_buffers(const Stepper &st) {
  return {.qpos = copy_of(st.qpos()),
          .qvel = copy_of(st.qvel()),
          .qacc = copy_of(st.qacc()),
          .qfrc_constraint = copy_of(st.qfrc_constraint()),
          .efc_force = copy_of(st.efc_force()),
          .ctrl = copy_of(st.ctrl()),
          .qfrc_applied = copy_of(st.qfrc_applied()),
          .time = st.time()};
}

[[nodiscard]] bool bitwise(std::span<const double> a,
                           std::span<const double> b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i)
        if (std::bit_cast<std::uint64_t>(a[i]) !=
            std::bit_cast<std::uint64_t>(b[i]))
            return false;
    return true;
}

void require_live_buffers(const Stepper &st, const LiveBuffers &before) {
    bt::require(bitwise(st.qpos(), before.qpos) &&
                    bitwise(st.qvel(), before.qvel) &&
                    bitwise(st.qacc(), before.qacc) &&
                    bitwise(st.qfrc_constraint(), before.qfrc_constraint) &&
                    bitwise(st.efc_force(), before.efc_force) &&
                    bitwise(st.ctrl(), before.ctrl) &&
                    bitwise(st.qfrc_applied(), before.qfrc_applied) &&
                    st.time() == before.time,
                "a rejected fuzzed set_state leaves live buffers untouched");
}

// vec(w): floats incl. NaN/Inf/subnormals/extremes, width 0..w+3.
[[nodiscard]] double random_double(Rng &rng) {
    static constexpr std::array<double, 12> specials{
        0., -0., 1., -1., .125, 1e300, -1e-300,
        std::numeric_limits<double>::denorm_min(),
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::min()};
    if (rng.pick(4) == 0) return std::bit_cast<double>(rng.next());
    return specials[rng.pick(specials.size())];
}

[[nodiscard]] std::vector<double> vec(Rng &rng, std::size_t bound) {
    std::vector<double> out;
    const std::size_t count = rng.pick(bound + 4);
    out.reserve(count);
    for (std::size_t i = 0; i < count; ++i) out.push_back(random_double(rng));
    return out;
}

void test_set_state_fuzz() {
    const auto owner = drivetrain_reference::pair();
    Stepper &stepper = *owner;
    const std::size_t nq = stepper.qpos().size();
    const std::size_t nv = stepper.qvel().size();
    Rng rng{0xF0225EED5ULL};
    std::size_t rejected = 0;
    for (int example = 0; example < 100; ++example) {
        // Source bounds: vec(nq), vec(nv), vec(3), vec(nv) + a JSON time.
        // The arbitrary-object arg arm has no typed-API counterpart.
        auto qpos = vec(rng, nq), qvel = vec(rng, nv), act = vec(rng, 3),
             warm = vec(rng, nv);
        const double when = random_double(rng);
        const LiveBuffers before = live_buffers(stepper);
        try {
            stepper.set_state(qpos, qvel, act, warm, when);
        } catch (const std::exception &failure) {
            bt::require(sanctioned(failure),
                        "set_state fuzz rejection stays inside the contract "
                        "set (" + std::string{failure.what()} + ")");
            ++rejected;
            require_live_buffers(stepper, before);
            continue;
        }
    }
    bt::require(rejected > 0,
                "fuzz draws reach the width/finiteness rejection arms");
}

void check() {
    test_drive_state_restore_fuzz();
    test_set_state_fuzz();
}

} // namespace

int main() { return bt::run(check); }
