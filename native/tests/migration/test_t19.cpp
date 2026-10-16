// test_t19.cpp — T19 seeded-random migration regressions. Covers the owned
// declarations behind the random/numpy_compat.hpp boundary: SeedSequence's
// entropy coercion + INIT_A/MULT_A pool + generate_state + spawn
// bookkeeping, the PCG64 XSL-RR core (raw64, the buffered next_uint32 lane,
// uint64_to_double), the Ziggurat standard normal and loc/scale transforms,
// Lemire bounded integers (32-bit buffered and 64-bit lanes, the rng==0
// no-draw and full-range direct lanes), choice()'s uniform and
// kahan/cumsum/searchsorted probability paths, sensor_noise.py's
// NoiseTape/SensorNoiseSource/build_noise_tape draw order and validation,
// and rider_random.py's RiderRandomSpec domain + sample_rider's five-draw
// order — every value comparison bitwise against the oracle fixtures
// declared in the manifest, with fixture docs driving the replay inputs.
#include "support/migration_test.hpp"

#include "io/json.hpp"
#include "random/numpy_compat.hpp"

#include <array>
#include <bit>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace {

namespace rnd = bike::random;
namespace io = bike::io;
using io::Value;

// ---- JSON helpers (same idioms as test_t14) ---------------------------------

const Value::Object &object_of(const Value &value, std::string_view field) {
    return io::as_object(value, field);
}
const Value &member(const Value &object, std::string_view key,
                    std::string_view path) {
    const Value *found = io::find_member(object_of(object, path), key);
    bt::require(found != nullptr,
                "missing member " + std::string(path) + "." +
                        std::string(key));
    return *found;
}
const Value *member_opt(const Value &object, std::string_view key,
                        std::string_view path) {
    return io::find_member(object_of(object, path), key);
}
double number_of(const Value &value, std::string_view field) {
    const std::string lexeme = io::as_number(value, field).lexeme;
    bool ok = false;
    const double result = io::detail::float_from_lexeme(lexeme, ok);
    bt::require(ok, "number lexeme did not decode: " + lexeme);
    return result;
}
std::int64_t i64_of(const Value &value, std::string_view field) {
    return io::as_int64(value, field);
}
const std::string &string_of(const Value &value,
                             std::string_view field) {
    return io::as_string(value, field);
}
bool bool_of(const Value &value, std::string_view field) {
    return io::as_bool(value, field);
}
const Value::Array &array_of(const Value &value,
                             std::string_view field) {
    return io::as_array(value, field);
}
// Unsigned 64-bit decimal string ("str(int(v))" on the capture side).
std::uint64_t u64_of(const Value &value, std::string_view field) {
    const std::string &text = string_of(value, field);
    std::uint64_t result = 0;
    const auto parsed =
            std::from_chars(text.data(), std::to_address(text.cend()), result,
                            10);
    bt::require(parsed.ec == std::errc() &&
                    parsed.ptr == std::to_address(text.cend()),
                std::string(field) + ": expected unsigned decimal string");
    return result;
}
// Signed 64-bit decimal string (the capture's "out_i64"/"out_u64" fields
// are str(int(v)) — strings, not JSON numbers).
std::int64_t i64_str_of(const Value &value, std::string_view field) {
    const std::string &text = string_of(value, field);
    std::int64_t result = 0;
    const auto parsed =
            std::from_chars(text.data(), std::to_address(text.cend()), result,
                            10);
    bt::require(parsed.ec == std::errc() &&
                    parsed.ptr == std::to_address(text.cend()),
                std::string(field) + ": expected signed decimal string");
    return result;
}
// JSON number member -> int64 (low/high bounds, sizes).
std::int64_t int_member(const Value &object, std::string_view key,
                        std::string_view path) {
    return i64_of(member(object, key, path), path);
}
// Decimal-string member -> uint64.
std::uint64_t u64_member(const Value &object, std::string_view key,
                         std::string_view path) {
    return u64_of(member(object, key, path), path);
}
std::vector<std::uint64_t> u64s_of(const Value &value,
                                   std::string_view field) {
    std::vector<std::uint64_t> out;
    for (const Value &v: array_of(value, field))
        out.push_back(u64_of(v, field));
    return out;
}
std::vector<std::int64_t> i64s_of(const Value &value,
                                  std::string_view field) {
    std::vector<std::int64_t> out;
    for (const Value &v: array_of(value, field))
        out.push_back(i64_of(v, field));
    return out;
}
std::vector<double> doubles_of(const Value &value,
                               std::string_view field) {
    std::vector<double> out;
    for (const Value &v: array_of(value, field))
        out.push_back(number_of(v, field));
    return out;
}
std::vector<std::uint32_t> spawn_key_of(const Value &value,
                                        std::string_view field) {
    std::vector<std::uint32_t> out;
    for (const Value &v: array_of(value, field))
        out.push_back(static_cast<std::uint32_t>(
                i64_of(v, field)));
    return out;
}

// A manifest-validated JSON doc: the entry's declared files verify
// (bytes + SHA-256) before the doc text is accepted. `entry` is the
// manifest fixture name; `relative` is the fixture-root-relative path
// (identical for the single-file entries, different for multi-file
// entries like random/seed1234 whose doc is one declared file).
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (entry, relative) is a (manifest name, fixture path) pair by design
Value doc_of(std::string_view entry, std::string_view relative) {
    bt::manifest_verify_files(bt::manifest_entry(entry));
    return io::parse_json(
            bt::text(bt::fixture(relative)));
}
Value doc_of(std::string_view entry_and_path) {
    return doc_of(entry_and_path, entry_and_path);
}

// ---- bit-level comparisons ----------------------------------------------------

// produced double vs the capture's "bits" (float64.view('<u8')) decimal
// string — identical object representation required.
void expect_bits(std::uint64_t expected, double actual,
                 std::string_view label) {
    bt::require(expected == std::bit_cast<std::uint64_t>(actual),
                std::string(label) + ": double bits mismatch");
}
void expect_u64(std::uint64_t expected, std::uint64_t actual,
                std::string_view label) {
    bt::require(expected == actual,
                std::string(label) + ": uint64 mismatch");
}
void expect_i64(std::int64_t expected, std::int64_t actual,
                std::string_view label) {
    bt::require(expected == actual,
                std::string(label) + ": int64 mismatch");
}

// Explicit little-endian uint64 decode — the .u64 fixture's dtype is <u8.
std::vector<std::uint64_t>
read_u64_words(const std::filesystem::path &path) {
    const std::string bytes = bt::text(path);
    bt::require(bytes.size() % 8U == 0U,
                "u64 fixture byte count must be a multiple of eight");
    std::vector<std::uint64_t> out(bytes.size() / 8U);
    for (std::size_t i = 0; i < out.size(); ++i) {
        std::uint64_t word = 0;
        for (std::size_t b = 0; b < 8U; ++b)
            word |= static_cast<std::uint64_t>(static_cast<unsigned char>(
                            bytes.at(i * 8U + b)))
                    << (8U * b);
        out.at(i) = word;
    }
    return out;
}

void expect_invalid_msg(std::string_view expected,
                        const std::function<void()> &action,
                        std::string_view label) {
    try {
        action();
    } catch (const std::invalid_argument &error) {
        bt::require(expected == error.what(),
                    std::string(label) + ": invalid_argument message "
                            "mismatch: '" +
                            std::string(error.what()) + "'");
        return;
    }
    bt::require(false,
                std::string(label) + ": expected std::invalid_argument");
}

// ---- SeedSequence + PCG64 core ------------------------------------------------

// stream-words.json "seeds": each seed's three first-8 streams are fresh
// generators (PCG64(seed).random_raw, default_rng(seed).normal(0,1,8),
// default_rng(seed).random(8)).
void test_seed_streams() {
    const Value doc = doc_of("random/stream-words.json");
    const Value seeds = member(doc, "seeds", "doc");
    for (const auto &[key, expected]: object_of(seeds, "seeds")) {
        const std::uint64_t seed = [&] {
            std::uint64_t value = 0;
            const auto parsed = std::from_chars(
                    key.data(), std::to_address(key.cend()), value, 10);
            bt::require(parsed.ec == std::errc(), "seed key must decode");
            return value;
        }();
        {
            rnd::NumpyRng rng(seed);
            const std::vector<std::uint64_t> raw =
                    u64s_of(member(expected, "raw64", key), key);
            for (const std::uint64_t word: raw)
                expect_u64(word, rng.raw64(), "raw64 " + key);
        }
        {
            rnd::NumpyRng rng(seed);
            std::array<double, 8> draws{};
            rng.normal(0., 1., draws);
            const std::vector<std::uint64_t> expected_bits =
                    u64s_of(member(expected, "normal01", key), key);
            bt::require(expected_bits.size() == draws.size(),
                        "normal01 count " + key);
            for (std::size_t i = 0; i < draws.size(); ++i)
                expect_bits(expected_bits.at(i), draws.at(i),
                            "normal01 " + key);
        }
        {
            rnd::NumpyRng rng(seed);
            std::array<double, 8> draws{};
            rng.uniform01(draws);
            const std::vector<std::uint64_t> expected_bits =
                    u64s_of(member(expected, "uniform01", key), key);
            bt::require(expected_bits.size() == draws.size(),
                        "uniform01 count " + key);
            for (std::size_t i = 0; i < draws.size(); ++i)
                expect_bits(expected_bits.at(i), draws.at(i),
                            "uniform01 " + key);
        }
    }
}

// The >u64 decimal entropy with a spawn key: SeedSequence.generate_state(4,
// np.uint64), PCG64(seq).random_raw(8), normal and random streams.
void test_decimal_entropy() {
    const Value doc = doc_of("random/stream-words.json");
    const Value entropy = member(doc, "decimal_entropy", "doc");
    const std::string decimal =
            string_of(member(entropy, "decimal", "entropy"), "entropy");
    const std::vector<std::uint32_t> key = spawn_key_of(
            member(entropy, "spawn_key", "entropy"), "entropy");
    const rnd::SeedSequence sequence(decimal, key);
    {
        const std::vector<std::uint64_t> expected = u64s_of(
                member(entropy, "generate_state_u64", "entropy"), "entropy");
        const std::vector<std::uint64_t> actual =
                sequence.generate_state_u64(4);
        bt::require(actual.size() == expected.size(),
                    "generate_state_u64 count");
        for (std::size_t i = 0; i < expected.size(); ++i)
            expect_u64(expected.at(i), actual.at(i),
                       "generate_state_u64");
    }
    {
        rnd::NumpyRng rng(sequence);
        for (const std::uint64_t word:
             u64s_of(member(entropy, "raw64", "entropy"), "entropy"))
            expect_u64(word, rng.raw64(), "decimal raw64");
    }
    {
        rnd::NumpyRng rng(sequence);
        std::array<double, 8> draws{};
        rng.normal(0., 1., draws);
        const std::vector<std::uint64_t> expected_bits = u64s_of(
                member(entropy, "normal01", "entropy"), "entropy");
        for (std::size_t i = 0; i < draws.size(); ++i)
            expect_bits(expected_bits.at(i), draws.at(i),
                        "decimal normal01");
    }
    {
        rnd::NumpyRng rng(sequence);
        std::array<double, 8> draws{};
        rng.uniform01(draws);
        const std::vector<std::uint64_t> expected_bits = u64s_of(
                member(entropy, "uniform01", "entropy"), "entropy");
        for (std::size_t i = 0; i < draws.size(); ++i)
            expect_bits(expected_bits.at(i), draws.at(i),
                        "decimal uniform01");
    }
}

// seed1234-spawn.json: parent SeedSequence(1234).spawn(2) — parent state,
// child spawn keys + generate_state, and the spawned normal16 streams; the
// same file's siblings are the seed1234 golden stream files.
void test_spawn() {
    const Value doc =
            doc_of("random/seed1234", "random/seed1234-spawn.json");
    bt::require(i64_of(member(doc, "seed", "doc"), "doc") == 1234,
                "spawn doc seed");
    rnd::SeedSequence parent(1234);
    const std::vector<rnd::SeedSequence> children = parent.spawn(2);
    bt::require(children.size() == 2, "spawn(2) child count");
    bt::require(parent.n_children_spawned() == 2U,
                "parent n_children_spawned");
    bt::require(parent.pool().size() == 4U, "pool size");
    const Value parent_state = member(doc, "parent_state", "doc");
    bt::require(i64_of(member(parent_state, "n_children_spawned",
                            "parent_state"),
                       "parent_state") == 2,
                "recorded n_children_spawned");
    bt::require(i64_of(member(parent_state, "pool_size", "parent_state"),
                       "parent_state") == 4,
                "recorded pool_size");
    const Value spawn_key =
            member(parent_state, "spawn_key", "parent_state");
    const Value tuple =
            member(spawn_key, "$tuple", "parent_state.spawn_key");
    bt::require(array_of(tuple, "spawn_key").empty(),
                "parent spawn_key must be ()");

    const Value children_doc = member(doc, "children", "doc");
    const Value::Array &child_docs =
            array_of(children_doc, "children");
    bt::require(child_docs.size() == children.size(),
                "child doc count");
    for (std::size_t index = 0; index < children.size(); ++index) {
        const Value &child_doc = child_docs.at(index);
        const std::string label = "child " + std::to_string(index);
        const std::vector<std::uint32_t> expected_key = spawn_key_of(
                member(child_doc, "spawn_key", label), label);
        bt::require(children.at(index).spawn_key() == expected_key,
                    label + ": spawn_key");
        bt::require(children.at(index).n_children_spawned() == 0U,
                    label + ": n_children_spawned");
        const std::vector<std::uint64_t> expected_state = u64s_of(
                member(child_doc, "state_u64", label), label);
        const std::vector<std::uint64_t> actual_state =
                children.at(index).generate_state_u64(4);
        bt::require(actual_state.size() == expected_state.size(),
                    label + ": state_u64 count");
        for (std::size_t i = 0; i < expected_state.size(); ++i)
            expect_u64(expected_state.at(i), actual_state.at(i),
                       label + ": state_u64");
        // normal16: default_rng(child).normal(0., 1., 16).
        const std::string normal_path = string_of(
                member(child_doc, "normal16", label), label);
        rnd::NumpyRng child_rng(children.at(index));
        std::array<double, 16> draws{};
        child_rng.normal(0., 1., draws);
        bt::golden_f64(normal_path, draws);
    }
}

// stream-words.json "spawn_chain": seed 999, sequential spawn(2) calls and
// one deep child — the n_children_spawned bookkeeping contract.
void test_spawn_chain() {
    const Value doc = doc_of("random/stream-words.json");
    const Value chain = member(doc, "spawn_chain", "doc");
    bt::require(i64_of(member(chain, "seed", "chain"), "chain") == 999,
                "spawn_chain seed");
    rnd::SeedSequence parent(999);
    const auto check_children = [](const std::vector<rnd::SeedSequence> &children,
                                   const Value &doc_array,
                                   std::string_view label) {
        const Value::Array &docs = array_of(doc_array, label);
        bt::require(docs.size() == children.size(),
                std::string(label) + ": count");
        for (std::size_t i = 0; i < children.size(); ++i) {
            const std::vector<std::uint32_t> expected_key = spawn_key_of(
                    member(docs.at(i), "spawn_key", label), label);
            bt::require(children.at(i).spawn_key() == expected_key,
                    std::string(label) + ": spawn_key");
            const std::vector<std::uint64_t> expected = u64s_of(
                    member(docs.at(i), "state_u64", label), label);
            const std::vector<std::uint64_t> actual =
                    children.at(i).generate_state_u64(4);
            bt::require(actual.size() == expected.size(),
                    std::string(label) + ": state count");
            for (std::size_t k = 0; k < expected.size(); ++k)
                expect_u64(expected.at(k), actual.at(k),
                    std::string(label) + ": state_u64");
        }
    };
    std::vector<rnd::SeedSequence> first = parent.spawn(2);
    check_children(first, member(chain, "first_spawn", "chain"),
                   "first_spawn");
    const std::vector<rnd::SeedSequence> second = parent.spawn(2);
    check_children(second, member(chain, "second_spawn", "chain"),
                   "second_spawn");
    // children_a[0].spawn(1): the child's own counter advances — never the
    // parent's; the spawned key appends at key depth 1 -> (0, 0).
    const std::vector<rnd::SeedSequence> deep = first.at(0).spawn(1);
    check_children(deep, member(chain, "deep_child", "chain"), "deep_child");
    bt::require(deep.at(0).spawn_key().size() == 2U &&
                    deep.at(0).spawn_key().at(0) == 0U &&
                    deep.at(0).spawn_key().at(1) == 0U,
                "deep child key (0, 0)");
    bt::require(parent.n_children_spawned() ==
                    static_cast<std::uint32_t>(i64_of(
                            member(chain, "n_children_spawned", "chain"),
                            "chain")),
                "spawn_chain n_children_spawned");
    bt::require(first.at(0).n_children_spawned() == 1U,
                "the child's own counter advanced");
    bt::require(first.at(1).n_children_spawned() == 0U &&
                    second.at(0).n_children_spawned() == 0U,
                "sibling counters stay independent");
    // A second spawn(1) on first[0] keeps its own counter — the next deep
    // key is (0, 1), not (0, 0) again.
    const std::vector<rnd::SeedSequence> next = first.at(0).spawn(1);
    bt::require(next.at(0).spawn_key().size() == 2U &&
                    next.at(0).spawn_key().at(1) == 1U,
                "child counter advances its own spawn keys");
    // generate_state_u32/u64 n=0 draw nothing; the pool cycles mod 4 for
    // n>4 (pool_size is the fixed 4).
    bt::require(parent.generate_state_u32(0).empty(),
                "generate_state_u32(0) is empty");
    bt::require(parent.generate_state_u64(0).empty(),
                "generate_state_u64(0) is empty");
    const std::vector<std::uint32_t> cycle = parent.generate_state_u32(6);
    bt::require(cycle.size() == 6U, "generate_state_u32(6) length");
}

// The seed1234 golden stream files: fresh generators, 64 draws each.
void test_seed1234_streams() {
    const bt::ManifestFixture &entry =
            bt::manifest_entry("random/seed1234");
    bt::manifest_verify_files(entry);
    {
        rnd::NumpyRng rng(1234);
        std::vector<double> draws(64);
        rng.normal(0., 1., draws);
        bt::golden_f64("random/seed1234-normal64.f64", draws);
    }
    {
        rnd::NumpyRng rng(1234);
        std::vector<double> draws(64);
        rng.uniform01(draws);
        bt::golden_f64("random/seed1234-uniform64.f64", draws);
    }
    {
        rnd::NumpyRng rng(1234);
        const std::vector<std::uint64_t> expected = read_u64_words(
                bt::fixture("random/seed1234-pcg64-raw64.u64"));
        bt::require(expected.size() == 64U, "raw64 fixture count");
        for (const std::uint64_t word: expected)
            expect_u64(word, rng.raw64(), "pcg64 raw64");
    }
}

// seed1234-integers.json — scripted integers() ops on one generator plus a
// raw64 tail locking the post-sequence stream position.
void test_integers() {
    const Value doc = doc_of("random/seed1234-integers.json");
    rnd::NumpyRng rng(1234);
    for (const Value &op:
         array_of(member(doc, "ops", "doc"), "ops")) {
        const std::int64_t low = int_member(op, "low", "op");
        const std::int64_t high = int_member(op, "high", "op");
        const std::int64_t size = int_member(op, "size", "op");
        const bool endpoint =
                bool_of(member(op, "endpoint", "op"), "op");
        const std::vector<std::int64_t> expected =
                i64s_of(member(op, "out", "op"), "op");
        std::vector<std::int64_t> actual(
                static_cast<std::size_t>(size));
        rng.integers(low, high, actual, endpoint);
        bt::require(actual.size() == expected.size(), "integers size");
        for (std::size_t i = 0; i < actual.size(); ++i)
            expect_i64(expected.at(i), actual.at(i),
                       "integers " + std::to_string(low) + ":" +
                               std::to_string(high));
    }
    for (const std::uint64_t word:
         u64s_of(member(doc, "tail_raw64", "doc"), "doc"))
        expect_u64(word, rng.raw64(), "integers tail");
}

// seed1234-choice.json — the uniform a path is integers(0, a); the p path
// is the kahan/cumsum/searchsorted draw.
void test_choice() {
    const Value doc = doc_of("random/seed1234-choice.json");
    rnd::NumpyRng rng(1234);
    for (const Value &op:
         array_of(member(doc, "ops", "doc"), "ops")) {
        const std::string &kind =
                string_of(member(op, "kind", "op"), "op");
        const std::int64_t a = int_member(op, "a", "op");
        const Value *size_v = member_opt(op, "size", "op");
        const bool scalar_size =
                size_v == nullptr || io::is_null(*size_v);
        const std::vector<std::int64_t> expected =
                i64s_of(member(op, "out", "op"), "op");
        if (kind == "uniform") {
            if (scalar_size) {
                bt::require(expected.size() == 1U,
                            "scalar choice out count");
                expect_i64(expected.at(0), rng.choice(a),
                           "choice uniform scalar");
            } else {
                std::vector<std::int64_t> actual(
                        static_cast<std::size_t>(
                                i64_of(*size_v, "op.size")));
                // choice(a, size) for integer a is integers(0, a, size).
                rng.integers(0, a, actual);
                for (std::size_t i = 0; i < actual.size(); ++i)
                    expect_i64(expected.at(i), actual.at(i),
                               "choice uniform");
            }
        } else {
            const std::vector<double> p = doubles_of(
                    member(op, "p", "op"), "op");
            bt::require(a >= 0 &&
                            static_cast<std::size_t>(a) == p.size(),
                        "p size must equal a");
            if (scalar_size) {
                bt::require(expected.size() == 1U,
                            "scalar choice p out count");
                expect_i64(expected.at(0), rng.choice(p),
                           "choice p scalar");
            } else {
                std::vector<std::int64_t> actual(
                        static_cast<std::size_t>(
                                i64_of(*size_v, "op.size")));
                rng.choice(p, actual);
                for (std::size_t i = 0; i < actual.size(); ++i)
                    expect_i64(expected.at(i), actual.at(i),
                               "choice p");
            }
        }
    }
    for (const std::uint64_t word:
         u64s_of(member(doc, "tail_raw64", "doc"), "doc"))
        expect_u64(word, rng.raw64(), "choice tail");
}

// seed1234-mixed.json — the interleaved sequence locking the buffered-u32
// semantics across next64 draws.
void test_mixed() {
    const Value doc = doc_of("random/seed1234-mixed.json");
    rnd::NumpyRng rng(1234);
    for (const Value &step:
         array_of(member(doc, "steps", "doc"), "steps")) {
        const std::string &op =
                string_of(member(step, "op", "step"), "step");
        if (op == "integers") {
            const std::int64_t low = int_member(step, "low", "step");
            const std::int64_t high = int_member(step, "high", "step");
            expect_i64(
                    i64_str_of(member(step, "out_i64", "step"), "step"),
                    rng.integers(low, high), "mixed integers");
        } else if (op == "random") {
            expect_bits(u64_member(step, "out_bits", "step"),
                        rng.uniform01(), "mixed random");
        } else if (op == "normal01") {
            expect_bits(u64_member(step, "out_bits", "step"),
                        rng.normal01(), "mixed normal01");
        } else if (op == "uniform") {
            expect_bits(
                    u64_member(step, "out_bits", "step"),
                    rng.uniform(number_of(member(step, "low", "step"),
                                          "step"),
                                number_of(member(step, "high", "step"),
                                          "step")),
                    "mixed uniform");
        } else if (op == "normal") {
            const std::vector<std::uint64_t> expected = u64s_of(
                    member(step, "out_bits", "step"), "step");
            std::vector<double> actual(expected.size());
            rng.normal(number_of(member(step, "loc", "step"), "step"),
                       number_of(member(step, "scale", "step"), "step"),
                       actual);
            for (std::size_t i = 0; i < actual.size(); ++i)
                expect_bits(expected.at(i), actual.at(i), "mixed normal");
        } else if (op == "choice") {
            const std::vector<double> p =
                    doubles_of(member(step, "p", "step"), "step");
            expect_i64(
                    i64_str_of(member(step, "out_i64", "step"), "step"),
                    rng.choice(p), "mixed choice");
        } else if (op == "raw64") {
            expect_u64(u64_member(step, "out_u64", "step"), rng.raw64(),
                       "mixed raw64");
        } else {
            bt::require(false, "unknown mixed op: " + op);
        }
    }
}

// random/rider-samples.json — sample_rider(RiderRandomSpec(), seed) for
// 0/42/210: the five uniform draws plus the static defaults the sampled
// records carry.
void test_rider_samples() {
    const Value doc = doc_of("random/rider-samples.json");
    for (const auto &[key, expected]: object_of(doc, "doc")) {
        const std::uint64_t seed = [&] {
            std::uint64_t value = 0;
            const auto parsed = std::from_chars(
                    key.data(), std::to_address(key.cend()), value, 10);
            bt::require(parsed.ec == std::errc(), "seed key must decode");
            return value;
        }();
        const rnd::SampledRider sampled =
                rnd::sample_rider(rnd::RiderRandomSpec{}, seed);
        const Value rider = member(expected, "rider", key);
        const auto rider_scalar = [&](std::string_view field,
                                      double actual) {
            const double expected_value = number_of(
                    member(rider, field, key), key);
            bt::require(std::bit_cast<std::uint64_t>(expected_value) ==
                            std::bit_cast<std::uint64_t>(actual),
                        key + "." + std::string(field) + " bits");
        };
        bt::require(string_of(member(rider, "variant", key), key) ==
                            "articulated_planar",
                    key + ".variant");
        rider_scalar("mass_kg", sampled.rider.mass_kg);
        rider_scalar("height_m", sampled.rider.height_m);
        rider_scalar("arm_resonance_hz", sampled.rider.arm_resonance_hz);
        rider_scalar("leg_resonance_hz", sampled.rider.leg_resonance_hz);
        rider_scalar("torso_resonance_hz",
                     sampled.rider.torso_resonance_hz);
        rider_scalar("damping_ratio", sampled.rider.damping_ratio);
        rider_scalar("bar_share", sampled.rider.bar_share);
        rider_scalar("pedal_share", sampled.rider.pedal_share);
        rider_scalar("saddle_share", sampled.rider.saddle_share);
        rider_scalar("helmet_mass_kg", sampled.rider.helmet_mass_kg);
        rider_scalar("saddle_interface_k_n_m",
                     sampled.rider.saddle_interface_k_n_m);
        rider_scalar("saddle_interface_c_ns_m",
                     sampled.rider.saddle_interface_c_ns_m);
        bt::require(io::is_null(member(rider, "inseam_m", key)) &&
                            !sampled.rider.inseam_m.has_value(),
                    key + ".inseam_m");
        bt::require(string_of(member(rider, "legs", key), key) ==
                            sampled.rider.legs,
                    key + ".legs");
        const Value program = member(expected, "program", key);
        const double delay = number_of(
                member(program, "reaction_delay_s", key), key);
        bt::require(std::bit_cast<std::uint64_t>(delay) ==
                            std::bit_cast<std::uint64_t>(
                                    sampled.program.reaction_delay_s),
                    key + ".reaction_delay_s");
        const Value keyframes = member(program, "keyframes", key);
        const Value::Array &frames = array_of(
                member(keyframes, "$tuple", key + ".keyframes"),
                key + ".keyframes");
        bt::require(frames.size() == 1U &&
                            sampled.program.keyframes.size() == 1U,
                    key + ".keyframes count");
        const Value &frame = frames.at(0);
        const rnd::RiderKeyframe &actual_frame =
                sampled.program.keyframes.at(0);
        const double expected_time =
                number_of(member(frame, "time_s", key), key);
        bt::require(std::bit_cast<std::uint64_t>(expected_time) ==
                            std::bit_cast<std::uint64_t>(
                                    actual_frame.time_s),
                    key + ".time_s");
        const Value posture = member(frame, "posture", key);
        const auto posture_scalar = [&](std::string_view field,
                                        double actual) {
            const double expected_value = number_of(
                    member(posture, field, key), key);
            bt::require(std::bit_cast<std::uint64_t>(expected_value) ==
                            std::bit_cast<std::uint64_t>(actual),
                        key + "." + std::string(field));
        };
        posture_scalar("torso_lean_rad",
                       actual_frame.posture.torso_lean_rad);
        posture_scalar("pelvis_pitch_rad",
                       actual_frame.posture.pelvis_pitch_rad);
        bt::require(io::is_null(member(posture, "pelvis_offset_m",
                                     key)) &&
                            !actual_frame.posture.pelvis_offset_m
                                     .has_value(),
                    key + ".pelvis_offset_m");
        bt::require(bool_of(member(posture, "use_saddle", key), key) ==
                            actual_frame.posture.use_saddle,
                    key + ".use_saddle");
        const double expected_torque = number_of(
                member(frame, "human_torque_nm", key), key);
        bt::require(actual_frame.human_torque_nm.has_value() &&
                            std::bit_cast<std::uint64_t>(
                                    expected_torque) ==
                                    std::bit_cast<std::uint64_t>(
                                            *actual_frame.human_torque_nm),
                    key + ".human_torque_nm");
    }
}

// random/research-noise-seed210 — build_noise_tape(SensorConfig(), 210, 128):
// the spawned noise/dropout children and the four-call draw order.
void test_noise_tape() {
    const bt::ManifestFixture &entry =
            bt::manifest_entry("random/research-noise-seed210");
    bt::manifest_verify_files(entry);
    const rnd::NoiseTape tape =
            rnd::build_noise_tape(rnd::SensorNoiseConfig{}, 210, 128);
    bt::require(tape.noise.size() == 128U &&
                    tape.dropout_uniform.size() == 128U,
                "noise tape count");
    std::vector<double> flat;
    flat.reserve(tape.noise.size() * 9U);
    for (const std::array<double, 9> &row: tape.noise)
        flat.insert(flat.end(), row.begin(), row.end());
    bt::golden_f64("random/research-noise-seed210-noise.f64", flat);
    bt::golden_f64("random/research-noise-seed210-dropout.f64",
                   tape.dropout_uniform);
}

// Same-seed determinism and the zero-scale contract: scale=0 still consumes
// one full normal draw per element — the streams of a zero-scale and a
// one-scale generator diverge only in output, never in position.
void test_determinism() {
    {
        rnd::NumpyRng a(42), b(42);
        for (int i = 0; i < 32; ++i) {
            bt::require(a.raw64() == b.raw64(), "same-seed raw64");
        }
    }
    {
        rnd::NumpyRng a(1234), b(1234);
        const double zero_draw = a.normal(0., 0.);
        b.normal01();
        bt::require(zero_draw == 0. && !std::signbit(zero_draw),
                "scale=0 produces +0.0 through loc + 0*z");
        bt::require(a.uniform01() == b.uniform01(),
                "scale=0 consumes exactly one standard_normal draw");
    }
    {
        // The Ziggurat draw produces signed zeros through the loc-scale
        // transform; a whole zero-scale tape row is ±0 bitwise.
        rnd::SensorNoiseConfig zero;
        zero.acceleration_std_mps2 = 0.;
        zero.gyro_std_rad_s = 0.;
        zero.encoder_std_rad_s = 0.;
        zero.torque_std_nm = 0.;
        rnd::SensorNoiseSource quiet(zero, 210);
        rnd::SensorNoiseSource loud(rnd::SensorNoiseConfig{}, 210);
        const auto [quiet_noise, quiet_dropout] = quiet.draw();
        const auto [loud_noise, loud_dropout] = loud.draw();
        (void)loud_noise;
        bt::require(std::bit_cast<std::uint64_t>(quiet_dropout) ==
                        std::bit_cast<std::uint64_t>(loud_dropout),
                "dropout stream is independent of noise scales");
        for (const double value: quiet_noise)
            bt::require(value == 0., "zero-scale noise element is zero");
    }
    {
        // The dropout child is spawn key (1,): it equals a manual
        // SeedSequence(210).spawn(2)[1] generator draw-for-draw.
        rnd::SeedSequence parent(210);
        const std::vector<rnd::SeedSequence> children = parent.spawn(2);
        rnd::NumpyRng dropout_stream(children.at(1));
        rnd::SensorNoiseSource source(rnd::SensorNoiseConfig{}, 210);
        const auto [noise, dropout] = source.draw();
        (void)noise;
        // noise child consumed 9 normal draws; dropout consumed 1 uniform.
        rnd::NumpyRng manual_noise(children.at(0));
        std::array<double, 9> manual{};
        for (std::size_t i = 0; i < 3; ++i)
            manual.at(i) = manual_noise.normal(0., .05);
        manual.at(3) = manual_noise.normal(0., .002);
        for (std::size_t i = 4; i < 7; ++i)
            manual.at(i) = manual_noise.normal(0., .01);
        for (std::size_t i = 7; i < 9; ++i)
            manual.at(i) = manual_noise.normal(0., .05);
        for (std::size_t i = 0; i < 9; ++i)
            bt::require(std::bit_cast<std::uint64_t>(manual.at(i)) ==
                            std::bit_cast<std::uint64_t>(noise.at(i)),
                        "noise draw order " + std::to_string(i));
        bt::require(std::bit_cast<std::uint64_t>(dropout) ==
                        std::bit_cast<std::uint64_t>(
                                dropout_stream.uniform01()),
                    "dropout child draw");
    }
}

// ---- validation branches -------------------------------------------------------

void test_seed_sequence_validation() {
    bt::invalid([] { const rnd::SeedSequence seq(""); });
    bt::invalid([] { const rnd::SeedSequence seq("abc"); });
    bt::invalid([] { const rnd::SeedSequence seq("-5"); });
    bt::invalid([] { const rnd::SeedSequence seq("0x"); });
    bt::invalid([] { const rnd::SeedSequence seq("0xzz"); });
    bt::invalid([] { const rnd::SeedSequence seq("12a3"); });
    // _coerce_to_uint32_array gate cases: DECIMAL_RE fails on a leading
    // space/non-digit ('unrecognized'), while '0X12' passes the decimal
    // gate on its leading '0' and then fails inside int() — all ValueError
    // class either way.
    bt::invalid([] { const rnd::SeedSequence seq(" 42"); });
    bt::invalid([] { const rnd::SeedSequence seq("ff"); });
    bt::invalid([] { const rnd::SeedSequence seq("0X12"); });
    bt::invalid([] { const rnd::SeedSequence seq("5__5"); });
    bt::invalid([] { const rnd::SeedSequence seq("_5"); });
    // CPython int() grammar cases that DO parse — the pooled state must be
    // identical to the equivalent integer entropy.
    const auto same_pool = [](const rnd::SeedSequence &a,
                              const rnd::SeedSequence &b) {
        bt::require(a.generate_state_u32(8) == b.generate_state_u32(8),
                    "string/int entropy pools disagree");
    };
    same_pool(rnd::SeedSequence("42"), rnd::SeedSequence(42));
    same_pool(rnd::SeedSequence("4_2"), rnd::SeedSequence(42));
    same_pool(rnd::SeedSequence("42 "), rnd::SeedSequence(42));
    same_pool(rnd::SeedSequence("0x5"), rnd::SeedSequence(5));
    same_pool(rnd::SeedSequence("0x_5"), rnd::SeedSequence(5));
    same_pool(rnd::SeedSequence("0x5_5"), rnd::SeedSequence(85));
    // Hex entropy and zero/one word decompositions.
    {
        const rnd::SeedSequence hex("0xdeadbeef");
        const std::vector<std::uint32_t> words = hex.generate_state_u32(4);
        bt::require(words.size() == 4U, "hex seed generates words");
    }
    {
        const rnd::SeedSequence zero(0);
        bt::require(zero.generate_state_u64(4).size() == 4U,
                    "seed 0 generates");
    }
    // spawn bookkeeping: spawn(0) is an empty list and leaves the counter.
    rnd::SeedSequence parent(7);
    bt::require(parent.spawn(0).empty(), "spawn(0) empty");
    bt::require(parent.n_children_spawned() == 0U,
                "spawn(0) leaves counter");
}

void test_rng_validation() {
    rnd::NumpyRng rng(0);
    // uniform(): nonfinite range is the OverflowError branch (kept out of
    // invalid_argument so the source's two error classes stay distinct),
    // then the CONS_NON_NEGATIVE signbit check on the range.
    bool overflow = false;
    try {
        (void)rng.uniform(0., std::numeric_limits<double>::infinity());
    } catch (const std::overflow_error &error) {
        overflow =
                std::string(error.what()) ==
                "high - low range exceeds valid bounds";
    }
    bt::require(overflow, "uniform nonfinite range is OverflowError");
    expect_invalid_msg("high - low < 0",
                       [&] { (void)rng.uniform(10., 5.); },
                       "uniform inverted range");
    expect_invalid_msg("high - low < 0",
                       [&] { (void)rng.uniform(0., -0.); },
                       "uniform -0.0 range is signbit-negative");
    // normal(): scale's CONS_NON_NEGATIVE is signbit (NaN passes through
    // and produces NaN draws exactly like the oracle).
    expect_invalid_msg("scale < 0", [&] { (void)rng.normal(0., -1.); },
                       "normal negative scale");
    expect_invalid_msg("scale < 0", [&] { (void)rng.normal(0., -0.0); },
                       "normal -0.0 scale");
    {
        const double nan_draw =
                rng.normal(0., std::numeric_limits<double>::quiet_NaN());
        bt::require(std::isnan(nan_draw),
                    "NaN scale propagates without error");
    }
    // integers(): format_bounds_error's low==0 special case and the
    // arbitrary-precision high-1 underflow branch.
    expect_invalid_msg("low >= high", [&] { (void)rng.integers(5, 3); },
                       "integers inverted");
    expect_invalid_msg("low >= high", [&] { (void)rng.integers(5, 5); },
                       "integers empty open interval");
    expect_invalid_msg("high <= 0", [&] { (void)rng.integers(0, -1); },
                       "integers zero-low inverted");
    expect_invalid_msg("high <= 0", [&] { (void)rng.integers(0, 0); },
                       "integers zero");
    expect_invalid_msg(
            "low >= high",
            [&] {
                (void)rng.integers(std::numeric_limits<std::int64_t>::min(),
                                   std::numeric_limits<std::int64_t>::min());
            },
            "integers int64-min underflow");
    bt::require(rng.integers(std::numeric_limits<std::int64_t>::min(),
                             std::numeric_limits<std::int64_t>::min(),
                             true) ==
                        std::numeric_limits<std::int64_t>::min(),
                "integers closed unit range consumes nothing");
    // The size==0 fill path returns before bounds work entirely.
    {
        std::vector<std::int64_t> empty;
        rng.integers(5, 3, empty);
        bt::require(empty.empty(), "integers(5,3,size=0) no error");
    }
    // choice(): the population and probability domains in source order.
    expect_invalid_msg(
            "a must be a positive integer unless no samples are taken",
            [&] { (void)rng.choice(0); }, "choice zero population");
    expect_invalid_msg(
            "a must be a positive integer unless no samples are taken",
            [&] { (void)rng.choice(-1); }, "choice negative population");
    {
        const std::array<double, 2> nan_p{
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN()};
        expect_invalid_msg("Probabilities contain NaN",
                           [&] { (void)rng.choice(nan_p); },
                           "choice NaN p");
    }
    {
        const std::array<double, 2> negative_p{1.5, -.5};
        expect_invalid_msg("Probabilities are not non-negative",
                           [&] { (void)rng.choice(negative_p); },
                           "choice negative p");
    }
    {
        const std::array<double, 3> bad_sum{.5, .5, .5};
        expect_invalid_msg(
                "Probabilities do not sum to 1. See Notes section of "
                "docstring for more information.",
                [&] { (void)rng.choice(bad_sum); }, "choice bad sum");
    }
    {
        const std::array<double, 0> empty_p{};
        std::array<std::int64_t, 1> one{0};
        expect_invalid_msg(
                "a cannot be empty unless no samples are taken",
                [&] { rng.choice(empty_p, one); }, "choice empty p");
        std::vector<std::int64_t> none;
        expect_invalid_msg(
                "Probabilities do not sum to 1. See Notes section of "
                "docstring for more information.",
                [&] { rng.choice(empty_p, none); },
                "choice empty p at size 0");
    }
    {
        // A finite-but-non-1 sum exceeding the sqrt(eps) tolerance rejects;
        // a sum inside the tolerance (float noise) draws.
        const std::array<double, 3> within{.3333333333333333,
                                           .3333333333333333,
                                           .3333333333333333};
        const std::size_t picked = static_cast<std::size_t>(rng.choice(within));
        bt::require(picked < within.size(), "tolerance-inner choice draws");
    }
}

void test_rider_validation() {
    // _range: scalar() rejects non-finite bounds first ('invalid {name}'),
    // then the enclosing chain check with the formatted limits.
    {
        rnd::RiderRandomSpec spec;
        spec.mass_kg = {std::numeric_limits<double>::quiet_NaN(), 60.};
        expect_invalid_msg("invalid mass_kg",
                           [&] { spec.validate(); }, "NaN mass bound");
        spec.mass_kg = {50., 40.};
        expect_invalid_msg(
                "mass_kg must satisfy 20.0 <= low <= high <= 200.0",
                [&] { spec.validate(); }, "mass inverted");
        spec.mass_kg = {10., 60.};
        expect_invalid_msg(
                "mass_kg must satisfy 20.0 <= low <= high <= 200.0",
                [&] { spec.validate(); }, "mass below limit");
        spec.mass_kg = {60., 300.};
        expect_invalid_msg(
                "mass_kg must satisfy 20.0 <= low <= high <= 200.0",
                [&] { spec.validate(); }, "mass above limit");
    }
    {
        rnd::RiderRandomSpec spec;
        spec.height_m = {.5, 3.};
        expect_invalid_msg(
                "height_m must satisfy 1.0 <= low <= high <= 2.3",
                [&] { spec.validate(); }, "height range");
    }
    {
        rnd::RiderRandomSpec spec;
        spec.torso_lean_rad = {-1., .2};
        expect_invalid_msg(
                "torso_lean_rad must satisfy -0.8 <= low <= high <= 0.8",
                [&] { spec.validate(); }, "torso range");
    }
    {
        rnd::RiderRandomSpec spec;
        spec.pelvis_pitch_rad = {0., .6};
        expect_invalid_msg(
                "pelvis_pitch_rad must satisfy "
                "-0.5 <= low <= high <= 0.5",
                [&] { spec.validate(); }, "pelvis range");
    }
    {
        rnd::RiderRandomSpec spec;
        spec.human_torque_nm = {-5., 10.};
        expect_invalid_msg(
                "human_torque_nm must satisfy "
                "0.0 <= low <= high <= 1000.0",
                [&] { spec.validate(); }, "torque range");
    }
    // sample_rider validates before any draw; a degenerate [x, x] range is
    // accepted and pins the draw to x.
    {
        rnd::RiderRandomSpec spec;
        spec.mass_kg = {70., 70.};
        spec.height_m = {1.7, 1.7};
        spec.torso_lean_rad = {0., 0.};
        spec.pelvis_pitch_rad = {0., 0.};
        spec.human_torque_nm = {0., 0.};
        const rnd::SampledRider sampled = rnd::sample_rider(spec, 5);
        const std::optional<double> &torque =
                sampled.program.keyframes.at(0).human_torque_nm;
        bt::require(sampled.rider.mass_kg == 70. &&
                        sampled.rider.height_m == 1.7 &&
                        sampled.program.keyframes.at(0)
                                        .posture.torso_lean_rad == 0. &&
                        torque.has_value() && *torque == 0.,
                    "degenerate ranges pin draws");
    }
    {
        rnd::RiderRandomSpec spec;
        spec.mass_kg = {300., 400.};
        expect_invalid_msg(
                "mass_kg must satisfy 20.0 <= low <= high <= 200.0",
                [&] { (void)rnd::sample_rider(spec, 5); },
                "sample_rider validates first");
    }
    // The declared spec defaults reproduce the source dataclass.
    const rnd::RiderRandomSpec defaults;
    bt::require(defaults.mass_kg.at(0) == 60. &&
                    defaults.mass_kg.at(1) == 100. &&
                    defaults.height_m.at(0) == 1.60 &&
                    defaults.height_m.at(1) == 1.85 &&
                    defaults.torso_lean_rad.at(0) == 0. &&
                    defaults.torso_lean_rad.at(1) == .25 &&
                    defaults.pelvis_pitch_rad.at(0) == 0. &&
                    defaults.pelvis_pitch_rad.at(1) == .08 &&
                    defaults.human_torque_nm.at(0) == 0. &&
                    defaults.human_torque_nm.at(1) == 25.,
                "RiderRandomSpec defaults");
}

void test_noise_validation() {
    // The count domain: negative counts reject before any draw.
    expect_invalid_msg(
            "noise tape count must be a nonnegative integer",
            [] {
                (void)rnd::build_noise_tape(rnd::SensorNoiseConfig{}, 0,
                                            -1);
            },
            "negative count");
    // SensorConfig.__post_init__'s scalar(minimum=0.) on each carried
    // scale, in field order.
    {
        rnd::SensorNoiseConfig config;
        config.acceleration_std_mps2 = -1.;
        expect_invalid_msg("invalid acceleration_std_mps2",
                           [&] { config.validate(); }, "accel scale");
        config.acceleration_std_mps2 =
                std::numeric_limits<double>::quiet_NaN();
        expect_invalid_msg("invalid acceleration_std_mps2",
                           [&] { config.validate(); }, "NaN accel");
    }
    {
        rnd::SensorNoiseConfig config;
        config.gyro_std_rad_s = -0.5;
        expect_invalid_msg("invalid gyro_std_rad_s",
                           [&] { config.validate(); }, "gyro scale");
        config.torque_std_nm =
                std::numeric_limits<double>::infinity();
        expect_invalid_msg("invalid gyro_std_rad_s",
                           [&] { config.validate(); },
                           "gyro first in order");
    }
    {
        rnd::SensorNoiseConfig config;
        config.encoder_std_rad_s = -1.;
        expect_invalid_msg("invalid encoder_std_rad_s",
                           [&] { config.validate(); }, "encoder scale");
    }
    {
        rnd::SensorNoiseConfig config;
        config.torque_std_nm = -1.;
        expect_invalid_msg("invalid torque_std_nm",
                           [&] { config.validate(); }, "torque scale");
    }
    // The SensorNoiseSource ctor reruns config validation.
    {
        rnd::SensorNoiseConfig config;
        config.acceleration_std_mps2 = -1.;
        expect_invalid_msg(
                "invalid acceleration_std_mps2",
                [&] { const rnd::SensorNoiseSource source(config, 0); },
                "source ctor validates config");
    }
    // NoiseTape.__post_init__: shape first, then finiteness, then the
    // [0, 1) dropout domain — each with the source's message.
    {
        rnd::NoiseTape tape;
        tape.noise.resize(2);
        tape.dropout_uniform.resize(3);
        expect_invalid_msg(
                "noise tape requires (count, 9) noise and (count,) dropout",
                [&] { tape.validate(); }, "tape shape");
    }
    {
        rnd::NoiseTape tape;
        tape.noise = {{0., 0., 0., 0., 0., 0., 0., 0.,
                       std::numeric_limits<double>::quiet_NaN()}};
        tape.dropout_uniform = {.5};
        expect_invalid_msg("noise tape must be finite",
                           [&] { tape.validate(); }, "NaN noise");
    }
    {
        rnd::NoiseTape tape;
        tape.noise = {{0., 0., 0., 0., 0., 0., 0., 0., 0.}};
        tape.dropout_uniform = {std::numeric_limits<double>::quiet_NaN()};
        expect_invalid_msg("noise tape must be finite",
                           [&] { tape.validate(); }, "NaN dropout");
    }
    {
        rnd::NoiseTape tape;
        tape.noise = {{0., 0., 0., 0., 0., 0., 0., 0., 0.}};
        tape.dropout_uniform = {1.};
        expect_invalid_msg("dropout uniforms must lie in [0, 1)",
                           [&] { tape.validate(); }, "dropout one");
        tape.dropout_uniform = {-.1};
        expect_invalid_msg("dropout uniforms must lie in [0, 1)",
                           [&] { tape.validate(); }, "dropout negative");
    }
    // count=0 is an empty tape — still valid.
    {
        const rnd::NoiseTape empty =
                rnd::build_noise_tape(rnd::SensorNoiseConfig{}, 0, 0);
        bt::require(empty.noise.empty() && empty.dropout_uniform.empty(),
                    "empty tape");
    }
}

} // namespace

int main() {
    return bt::run([] {
        test_seed_streams();
        test_decimal_entropy();
        test_spawn();
        test_spawn_chain();
        test_seed1234_streams();
        test_integers();
        test_choice();
        test_mixed();
        test_rider_samples();
        test_noise_tape();
        test_determinism();
        test_seed_sequence_validation();
        test_rng_validation();
        test_rider_validation();
        test_noise_validation();
    });
}
