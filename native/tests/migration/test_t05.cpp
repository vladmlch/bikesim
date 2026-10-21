// test_t05.cpp — strict lossless data/JSON boundary regressions (task T05).
// Covers the owned cold-path Value tree, lexeme-preserving parse/serialize,
// the bike_recording.json.v1 tag codec (CPython 3.13 semantics for
// Decimal/date/time/datetime/timedelta/base64), the shared SHA-256 and the
// strict oracle-manifest/provenance layer — mirroring
// tests/reference/test_recording_values.py and json_codec.py's domains.
#include "support/migration_test.hpp"
#include "io/json.hpp"
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

#include <cstddef>
#include <limits>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <variant>

#include "io/hash.hpp"
#include "io/value.hpp"

namespace {

using bike::io::Bytes;
using bike::io::Number;
using bike::io::Tagged;
using bike::io::Value;

static_assert(std::is_aggregate_v<Value>);
static_assert(std::is_nothrow_move_constructible_v<Value::Object>);
static_assert(std::is_nothrow_move_assignable_v<Value::Object>);

// Completion-plan 1.13: recursive object storage must keep the existing
// aggregate/variant interface and own every nested value after copy/move.
void checks_value_ownership() {
    using bike::io::as_object;
    using bike::io::canonical_json;

    Value child{Value::Object{{"text", Value{std::string{"seed"}}}}};
    const Value original{.data = Value::Object{
        {"z", Value{Number{"9007199254740993"}}},
        {"a", Value{Value::Array{std::move(child), Value{Number{"-0.0"}}}}},
        {"n", Value{}},
    }};
    const std::string expected =
        R"({"a":[{"text":"seed"},-0.0],"n":null,"z":9007199254740993})";
    const auto &members = as_object(original, "$");
    bt::require(members.size() == 3 && members[0].first == "z" &&
                    members[1].first == "a" && members[2].first == "n",
                "object keeps insertion order behind the variant");
    bt::require(canonical_json(original) == expected,
                "aggregate object retains lexemes and explicit null");

    Value copied = original;
    auto &copied_members = std::get<Value::Object>(copied.data);
    auto &array = std::get<Value::Array>(copied_members.at(1).second.data);
    auto &nested = std::get<Value::Object>(array.at(0).data);
    nested.at(0).second.data = std::string{"changed"};
    bt::require(canonical_json(original) == expected,
                "nested object/array copy must not alias its source");
    const std::string changed =
        R"({"a":[{"text":"changed"},-0.0],"n":null,"z":9007199254740993})";
    bt::require(canonical_json(copied) == changed,
                "nested mutation stays in the copied tree");

    Value assigned{Value::Object{{"old", Value{true}}}};
    assigned = original;
    std::get<Value::Object>(assigned.data).at(0).second.data = Number{"7"};
    bt::require(canonical_json(original) == expected,
                "object copy assignment preserves source ownership");

    const Value moved{std::move(copied)};
    bt::require(canonical_json(moved) == changed,
                "object move construction preserves its nested payload");
    // No assumption about a moved-from vector's size; it must remain
    // destructible and assignable through the same active alternative.
    copied = original;
    bt::require(canonical_json(copied) == expected,
                "moved-from object can receive a deep copy");

    Value move_assigned{Value::Object{{"old", Value{false}}}};
    move_assigned = std::move(assigned);
    bt::require(canonical_json(move_assigned) ==
                    R"({"a":[{"text":"seed"},-0.0],"n":null,"z":7})",
                "object move assignment replaces the complete old tree");
    assigned = original;
    bt::require(canonical_json(assigned) == expected,
                "move-assigned source stays assignable");

    const Value from_range{Value::Object{members.begin(), members.end()}};
    bt::require(canonical_json(from_range) == expected,
                "object preserves the inherited vector range constructor");
    Value::Object::base base_members{{"base", Value{Number{"5"}}}};
    const Value from_base{Value::Object{std::move(base_members)}};
    bt::require(canonical_json(from_base) == R"({"base":5})",
                "object accepts its previous vector representation");
    bt::require(canonical_json(Value{Value::Object{}}) == "{}" &&
                    bike::io::is_null(Value{}),
                "empty object and default null remain distinct");
}

// recording_json(parse_recording_json(text)) — the canonical tag form the
// source codec's dumps(to_json_value(loads(text))) produces.
std::string tag_canonical(std::string_view text) {
    return bike::io::recording_json(bike::io::parse_recording_json(text));
}

void tag_round_trip(std::string_view text, std::string_view expected) {
    const std::string canonical = tag_canonical(text);
    bt::require(canonical == expected,
                "tag canonical mismatch: " + std::string(text) + " -> " +
                    canonical + " expected " + std::string(expected));
    // Decoded tags must re-serialize to the identical wire bytes.
    bt::require(tag_canonical(canonical) == canonical,
                "tag round trip not stable: " + canonical);
}

void tag_invalid(std::string_view text) {
    bt::invalid([text] { (void) bike::io::parse_recording_json(text); });
}

void parse_invalid(std::string_view text) {
    bt::invalid([text] { (void) bike::io::parse_json(text); });
}

// Minimal valid oracle-manifest document; mutation cases patch substrings.
std::string manifest_doc() {
    const std::string digest(64, 'a');
    return std::string(R"({"schema":"migration-oracle-manifest/v1",)") +
           R"("created_utc":"2026-01-01T00:00:00Z",)" +
           R"("oracle":{"bike_sim_file":"src/bike_sim/x.py",)" +
           R"("commit":"0123456789abcdef0123456789abcdef01234567",)" +
           R"("describe":"d","source_sha256":")" + digest + R"(",)" +
           R"("status_porcelain":"clean","worktree":"w"},)" +
           R"("tool":{"argv":["a","b"],"path":"tool","sha256":")" + digest +
           R"("},)" + R"("environment":{"executable":"p","machine":"m",)" +
           R"("mujoco_version":"3.12.0","numpy_config":"c","packages":{},)" +
           R"("platform":"p","python":"3"},"blockers":null,)" +
           R"("negative_controls":[{"detail":"d","name":"n","result":true}],)" +
           R"("fixtures":[{"name":"g/f","group":"g","comparison":"Q",)" +
           R"("recipe":"RCP-00","produced_by":"p","notes":"n","inputs":[],)" +
           R"("expected":{"kind":"accepted"},"files":[{"path":"g/f.txt",)" +
           R"("bytes":1,"sha256":")" + digest + R"("}]}]})";
}

void manifest_invalid(std::string_view text) {
    bt::invalid([text] { (void) bt::manifest_parse(text); });
}

void replace_once(std::string &haystack, std::string_view needle,
                  std::string_view replacement) {
    const std::size_t pos = haystack.find(needle);
    bt::require(pos != std::string::npos,
                "manifest_doc patch needle missing: " +
                    std::string(needle));
    haystack.replace(pos, needle.size(), replacement);
}

// manifest_verify_files failures are require() failures (std::runtime_error)
// — the doctored-declaration path cannot ride bt::invalid.
template <class F> void require_fails(F &&action) {
    bool rejected = false;
    try {
        std::forward<F>(action)();
    } catch (const std::runtime_error &) {
        rejected = true;
    }
    bt::require(rejected, "expected a require() failure");
}

// Sections run as separate bt::run bodies: the harness reports per block and
// each function keeps its own frame under -Wframe-larger-than.
void checks_json() {
    using namespace bike::io;
    bt::invalid([] { parse_json(R"({"x":1,"x":2})"); });
    bt::invalid([] { parse_json("NaN"); });
    bt::require(canonical_json(parse_json("9007199254740993")) ==
                "9007199254740993", "integer precision");
    bt::require(canonical_json(parse_json("-0.0")) == "-0.0", "negative zero");

    // ---- syntax strictness ------------------------------------------------
    parse_invalid("Infinity");
    parse_invalid("-Infinity");
    parse_invalid("nan");
    parse_invalid("1 2");
    parse_invalid("{} {}");
    parse_invalid("[] x");
    parse_invalid("01");
    parse_invalid("-01");
    parse_invalid("1.");
    parse_invalid(".5");
    parse_invalid("+1");
    parse_invalid("-");
    parse_invalid("");
    parse_invalid("   ");
    parse_invalid("e5");
    parse_invalid("1e");
    parse_invalid("1e+");
    parse_invalid("1e-");
    parse_invalid(".0");
    parse_invalid("{");
    parse_invalid("[1,]");
    parse_invalid("{'a':1}");
    parse_invalid(R"("\x41")");
    parse_invalid(R"({"a":{"x":1,"x":2}})");
    parse_invalid(R"([{"x":1},{"x":1,"x":2}])");
    // raw NUL inside the document (never a legal JSON byte)
    parse_invalid(std::string("{\"a\":1}\x00 x", 9));
    parse_invalid(std::string("\x00", 1));
    // UTF-8 BOM is not part of the document
    parse_invalid("\xef\xbb\xbf{}");
    // malformed raw UTF-8 inside a string
    parse_invalid("\"\xc3\x28\"");
    parse_invalid("\"\xed\xa0\x80\""); // CESU-8 surrogate half
    parse_invalid("\"\xf5\x80\x80\x80\""); // beyond U+10FFFF
    // unpaired surrogates through \u escapes
    parse_invalid(R"("\ud800")");
    parse_invalid(R"("\udc00")");
    parse_invalid(R"("\ud800x")");
    parse_invalid(R"("\ud800\ud800")");
    // depth bound: 512 nested containers accepted, 600 rejected
    {
        const std::string deep(600U, '[');
        const std::string close(600U, ']');
        parse_invalid(deep + close);
        const std::string ok_text =
            std::string(512U, '[') + std::string(512U, ']');
        (void) parse_json(ok_text);
    }
    // positive: valid escapes + passthrough bytes
    bt::require(canonical_json(parse_json(R"("\ud83d\ude00")")) ==
                    "\"\xf0\x9f\x98\x80\"",
                "paired surrogate must decode");
    bt::require(
        canonical_json(parse_json("\"caf\xc3\xa9 \xe2\x98\x83\"")) ==
            "\"caf\xc3\xa9 \xe2\x98\x83\"",
        "UTF-8 passthrough");
    bt::require(canonical_json(parse_json(R"("\u0001\n\t\"")")) ==
                    R"("\u0001\n\t\"")",
                "control escapes canonicalized");
    bt::require(canonical_json(parse_json("null")) == "null", "null");
    bt::require(canonical_json(parse_json("true")) == "true", "true");
    bt::require(canonical_json(parse_json("  [ 1 , 2 ] ")) == "[1,2]",
                "whitespace trimmed");
    bt::require(canonical_json(parse_json("123456789012345678901234567890"
                                          "123456789")) ==
                    "123456789012345678901234567890123456789",
                "arbitrary-magnitude integer lexeme");
    // A float lexeme beyond binary64 range (positive exponent) is a
    // syntax error at the raw boundary (RapidJSON NumberTooBig); the
    // recording path rejects it either way, like Python's finite() check.
    parse_invalid("1e999");
    parse_invalid("5e999");
    bt::require(canonical_json(parse_json("1.5e-3")) == "1.5e-3",
                "float lexeme verbatim");
    bt::require(canonical_json(parse_json(R"({"b":1,"a":2,"A":3})")) ==
                    R"({"A":3,"a":2,"b":1})",
                "canonical serialization sorts keys");
    // an ordinary object whose key happens to be the tag marker serializes
    // literally through canonical_json (no codec semantics on this path)
    bt::require(canonical_json(parse_json(
                    R"({"$bike_recording":"x","a":1})")) ==
                    R"({"$bike_recording":"x","a":1})",
                "canonical_json emits the literal object");

    // ---- typed accessors --------------------------------------------------
    {
        const Value doc = parse_json(
            R"({"i":5,"big":9007199254740993,"n":null,"f":1.5,"z":-0.0,)"
            R"("under":-1e-999,"s":"x","a":[1],"b":true})");
        const Value::Object &object = as_object(doc, "$");
        bt::require(find_member(object, "missing") == nullptr,
                    "absent member is not explicit null");
        const Value *null_member = find_member(object, "n");
        bt::require(null_member != nullptr && is_null(*null_member),
                    "explicit null member present as null");
        bt::require(as_int64(*find_member(object, "i"), "$.i") == 5,
                    "int64 member");
        bt::require(as_int64(*find_member(object, "big"), "$.big") ==
                        9007199254740993LL,
                    "large int64 member");
        bt::require(as_double(*find_member(object, "f"), "$.f") == 1.5,
                    "double member");
        bt::require(std::bit_cast<std::uint64_t>(
                        as_double(*find_member(object, "z"), "$.z")) ==
                        0x8000000000000000ULL,
                    "signed-zero bit pattern preserved");
        bt::require(std::bit_cast<std::uint64_t>(as_double(
                        *find_member(object, "under"), "$.under")) ==
                        0x8000000000000000ULL,
                    "underflow keeps the sign bit like float()");
        bt::require(as_string(*find_member(object, "s"), "$.s") == "x",
                    "string member");
        bt::require(as_array(*find_member(object, "a"), "$.a").size() == 1U,
                    "array member");
        bt::require(as_bool(*find_member(object, "b"), "$.b"),
                    "bool member");
        bt::invalid([&] { (void) as_bool(*find_member(object, "i"), "$.i"); });
        bt::invalid(
            [&] { (void) as_object(*find_member(object, "a"), "$.a"); });
        bt::invalid(
            [&] { (void) as_string(*find_member(object, "n"), "$.n"); });
        bt::invalid(
            [&] { (void) as_int64(*find_member(object, "b"), "$.b"); });
        bt::invalid(
            [&] { (void) as_int64(*find_member(object, "f"), "$.f"); });
        bt::invalid(
            [&] { (void) as_double(*find_member(object, "s"), "$.s"); });
    }
    // integer conversion edges
    bt::require(number_i64(Number{"9223372036854775807"}, "f") ==
                    9223372036854775807LL,
                "int64 max");
    bt::require(number_i64(Number{"-9223372036854775808"}, "f") ==
                    (-9223372036854775807LL - 1LL),
                "int64 min");
    bt::invalid(
        [] { (void) number_i64(Number{"9223372036854775808"}, "f"); });
    bt::invalid(
        [] { (void) number_i64(Number{"-9223372036854775809"}, "f"); });
    bt::invalid([] { (void) number_i64(Number{"5.0"}, "f"); });
    bt::invalid([] { (void) number_i64(Number{"1e3"}, "f"); });
    bt::invalid([] { (void) number_i64(Number{""}, "f"); });
    bt::invalid([] { (void) number_i64(Number{"5x"}, "f"); });
    bt::invalid(
        [] { (void) number_double(Number{"1e999"}, "f"); });
    bt::invalid(
        [] { (void) number_double(Number{"-1e999"}, "f"); });
    bt::invalid([] { (void) number_double(Number{"abc"}, "f"); });
    bt::require(number_double(Number{"1e-999"}, "f") == 0.0,
                "underflow is finite like float()");
    // tagged_double: the only nonfinite accessor
    {
        const Value nan_tag = parse_recording_json(
            R"({"$bike_recording":"float64","value":"nan"})");
        bt::require(std::isnan(tagged_double(nan_tag, "$")),
                    "tagged nan decodes");
        const Value inf_tag = parse_recording_json(
            R"({"$bike_recording":"float64","value":"+inf"})");
        bt::require(tagged_double(inf_tag, "$") ==
                        std::numeric_limits<double>::infinity(),
                    "tagged +inf decodes");
        const Value ninf_tag = parse_recording_json(
            R"({"$bike_recording":"float64","value":"-inf"})");
        bt::require(tagged_double(ninf_tag, "$") ==
                        -std::numeric_limits<double>::infinity(),
                    "tagged -inf decodes");
        const Value finite = parse_recording_json("1.5");
        bt::require(tagged_double(finite, "$") == 1.5,
                    "finite number through tagged_double");
        const Value ordinary = parse_recording_json(R"("x")");
        bt::invalid([&] { (void) tagged_double(ordinary, "$"); });
        const Value bad_kind = Value{
            Tagged{.kind = "decimal", .fields = {{"value", "1"}}}};
        bt::invalid([&] { (void) tagged_double(bad_kind, "$"); });
    }

}

void checks_recording() {
    using namespace bike::io;
    // ---- nonfinite JSON tokens are a recording-codec error ---------------
    tag_invalid("1e999");
    tag_invalid("-1e999");
    tag_invalid(R"({"a":1e999})");
    tag_invalid("[[[1e999]]]");
    tag_invalid(R"({"$bike_recording":"decimal","value":1e999})");
    // ... while underflow-to-zero is finite (float() semantics)
    bt::require(tag_canonical("1e-999") == "1e-999",
                "underflow token is finite");
    bt::require(tag_canonical("9007199254740993") == "9007199254740993",
                "recording path keeps integer precision");

    // ---- float64 sentinels ------------------------------------------------
    tag_round_trip(R"({"$bike_recording":"float64","value":"nan"})",
                   R"({"$bike_recording":"float64","value":"nan"})");
    tag_round_trip(R"({"$bike_recording":"float64","value":"+inf"})",
                   R"({"$bike_recording":"float64","value":"+inf"})");
    tag_round_trip(R"({"$bike_recording":"float64","value":"-inf"})",
                   R"({"$bike_recording":"float64","value":"-inf"})");
    tag_invalid(R"({"$bike_recording":"float64","value":"NaN"})");
    tag_invalid(R"({"$bike_recording":"float64","value":"inf"})");
    tag_invalid(R"({"$bike_recording":"float64","value":0})");
    tag_invalid(R"({"$bike_recording":"float64","value":"nan","x":1})");
    tag_invalid(R"({"$bike_recording":"float64"})");
    tag_invalid(R"({"$bike_recording":"float64","value":null})");
    tag_invalid(R"({"$bike_recording":5,"value":"nan"})");
    tag_invalid(R"({"$bike_recording":"bogus","value":1})");
    tag_invalid(R"({"$bike_recording":"x","a":1})");

    // ---- bytes tag: strict base64 -----------------------------------------
    tag_round_trip(R"({"$bike_recording":"bytes","value":"TWFu"})",
                   R"({"$bike_recording":"bytes","value":"TWFu"})");
    tag_round_trip(R"({"$bike_recording":"bytes","value":"TQ=="})",
                   R"({"$bike_recording":"bytes","value":"TQ=="})");
    tag_round_trip(R"({"$bike_recording":"bytes","value":"TWE="})",
                   R"({"$bike_recording":"bytes","value":"TWE="})");
    tag_round_trip(R"({"$bike_recording":"bytes","value":""})",
                   R"({"$bike_recording":"bytes","value":""})");
    tag_round_trip(
        R"({"$bike_recording":"bytes","value":"V2VsY29tZQ=="})",
        R"({"$bike_recording":"bytes","value":"V2VsY29tZQ=="})");
    {
        const Value decoded = parse_recording_json(
            R"({"$bike_recording":"bytes","value":"TWFu"})");
        const auto *bytes = std::get_if<Bytes>(&decoded.data);
        bt::require(bytes != nullptr && bytes->value.size() == 3U &&
                        bytes->value[0] == std::byte{'M'} &&
                        bytes->value[1] == std::byte{'a'} &&
                        bytes->value[2] == std::byte{'n'},
                    "bytes tag decodes to raw payload");
        bt::require(recording_json(decoded) ==
                        R"({"$bike_recording":"bytes","value":"TWFu"})",
                    "bytes re-encode is canonical base64");
        Value binary{Bytes{{std::byte{0xff}, std::byte{0x00}}}};
        bt::require(recording_json(binary) ==
                        R"({"$bike_recording":"bytes","value":"/wA="})",
                    "binary payload encodes");
    }
    tag_invalid(R"({"$bike_recording":"bytes","value":"T"})");
    tag_invalid(R"({"$bike_recording":"bytes","value":"TQ"})");
    tag_invalid(R"({"$bike_recording":"bytes","value":"TWE"})");
    tag_invalid(R"({"$bike_recording":"bytes","value":"TWFu="})");
    tag_invalid(R"({"$bike_recording":"bytes","value":"TQ==TQ=="})");
    tag_invalid(R"({"$bike_recording":"bytes","value":"==="})");
    tag_invalid(R"({"$bike_recording":"bytes","value":"TQ= ="})");
    tag_invalid(R"({"$bike_recording":"bytes","value":"TW=u"})");
    tag_invalid(R"({"$bike_recording":"bytes","value":"T!Fu"})");
    tag_invalid(R"({"$bike_recording":"bytes","value":0})");
    tag_invalid(R"({"$bike_recording":"bytes","value":"TWFu","x":1})");

    // ---- decimal tag -------------------------------------------------------
    tag_round_trip(R"({"$bike_recording":"decimal","value":"1.50"})",
                   R"({"$bike_recording":"decimal","value":"1.50"})");
    tag_round_trip(R"({"$bike_recording":"decimal","value":"1E+5"})",
                   R"({"$bike_recording":"decimal","value":"1E+5"})");
    tag_round_trip(R"({"$bike_recording":"decimal","value":"0.0000001"})",
                   R"({"$bike_recording":"decimal","value":"1E-7"})");
    tag_round_trip(R"({"$bike_recording":"decimal","value":"1_0"})",
                   R"({"$bike_recording":"decimal","value":"10"})");
    tag_round_trip(R"({"$bike_recording":"decimal","value":"I_nfinity"})",
                   R"({"$bike_recording":"decimal","value":"Infinity"})");
    tag_round_trip(R"({"$bike_recording":"decimal","value":" 1.5 "})",
                   R"({"$bike_recording":"decimal","value":"1.5"})");
    tag_round_trip(R"({"$bike_recording":"decimal","value":"NaN000123"})",
                   R"({"$bike_recording":"decimal","value":"NaN123"})");
    tag_round_trip(R"({"$bike_recording":"decimal","value":"-sNaN0"})",
                   R"({"$bike_recording":"decimal","value":"-sNaN"})");
    tag_round_trip(R"({"$bike_recording":"decimal","value":"-inf"})",
                   R"({"$bike_recording":"decimal","value":"-Infinity"})");
    tag_round_trip(R"({"$bike_recording":"decimal","value":"10e5"})",
                   R"({"$bike_recording":"decimal","value":"1.0E+6"})");
    tag_round_trip(R"({"$bike_recording":"decimal","value":"0e-5"})",
                   R"({"$bike_recording":"decimal","value":"0.00000"})");
    tag_round_trip(R"({"$bike_recording":"decimal","value":"-0e5"})",
                   R"({"$bike_recording":"decimal","value":"-0E+5"})");
    tag_round_trip(
        R"({"$bike_recording":"decimal","value":"1e-1999999999999999997"})",
        R"({"$bike_recording":"decimal","value":"1E-1999999999999999997"})");
    tag_round_trip(
        R"({"$bike_recording":"decimal","value":"0.00e1000000000000000000"})",
        R"({"$bike_recording":"decimal","value":"0E+999999999999999998"})");
    // Unicode Nd digits (Arabic-Indic 4.2) canonicalize to ASCII
    tag_round_trip(
        "{\"$bike_recording\":\"decimal\",\"value\":\"\xd9\xa4.\xd9\xa2\"}",
        R"({"$bike_recording":"decimal","value":"4.2"})");
    // number members take the Decimal(int)/Decimal(float) path — the float
    // path is the exact binary64 expansion, like Python's Decimal(0.1)
    tag_round_trip(R"({"$bike_recording":"decimal","value":5})",
                   R"({"$bike_recording":"decimal","value":"5"})");
    tag_round_trip(R"({"$bike_recording":"decimal","value":5.5})",
                   R"({"$bike_recording":"decimal","value":"5.5"})");
    tag_round_trip(R"({"$bike_recording":"decimal","value":-0.0})",
                   R"({"$bike_recording":"decimal","value":"-0"})");
    tag_round_trip(R"({"$bike_recording":"decimal","value":0.0})",
                   R"({"$bike_recording":"decimal","value":"0"})");
    tag_round_trip(R"({"$bike_recording":"decimal","value":1200.0})",
                   R"({"$bike_recording":"decimal","value":"1200"})");
    tag_round_trip(
        R"({"$bike_recording":"decimal","value":1e22})",
        R"({"$bike_recording":"decimal","value":"10000000000000000000000"})");
    tag_round_trip(
        R"({"$bike_recording":"decimal","value":1e23})",
        R"({"$bike_recording":"decimal","value":"99999999999999991611392"})");
    tag_round_trip(
        R"({"$bike_recording":"decimal","value":true})",
        R"({"$bike_recording":"decimal","value":"1"})");
    {
        const std::string exact_0_1 = tag_canonical(
            R"({"$bike_recording":"decimal","value":0.1})");
        bt::require(
            exact_0_1 ==
                R"({"$bike_recording":"decimal","value":"0.1000000000000000)"
                R"(055511151231257827021181583404541015625"})",
            "Decimal(float) is the exact binary expansion");
        const std::string subnormal = tag_canonical(
            R"({"$bike_recording":"decimal","value":5e-324})");
        bt::require(
            subnormal.find("\"4.9406564584124654417656879") !=
                    std::string::npos &&
                subnormal.find(R"(E-324"})") != std::string::npos,
            "Decimal(denormal) expands exactly to E-324");
    }
    tag_invalid(R"({"$bike_recording":"decimal","value":"1e1000000000000000000"})");
    tag_invalid(R"({"$bike_recording":"decimal","value":"10e999999999999999999"})");
    tag_invalid(R"({"$bike_recording":"decimal","value":"1e-1999999999999999998"})");
    tag_invalid(R"({"$bike_recording":"decimal","value":"1.5e-1999999999999999997"})");
    tag_invalid(R"({"$bike_recording":"decimal","value":"1e-9223372036854775807"})");
    tag_invalid(R"({"$bike_recording":"decimal","value":"0e1000000000000000000"})");
    tag_invalid(R"({"$bike_recording":"decimal","value":"1.2.3"})");
    tag_invalid(R"({"$bike_recording":"decimal","value":"abc"})");
    tag_invalid(R"({"$bike_recording":"decimal","value":"e5"})");
    tag_invalid(R"({"$bike_recording":"decimal","value":"_"})");
    tag_invalid(R"({"$bike_recording":"decimal","value":{}})");
    tag_invalid(R"({"$bike_recording":"decimal","value":null})");
    tag_invalid(R"({"$bike_recording":"decimal","value":"1","x":1})");

    // ---- datetime tag -------------------------------------------------------
    tag_round_trip(
        R"({"$bike_recording":"datetime","value":"2000-01-02T03:04:05"})",
        R"({"$bike_recording":"datetime","value":"2000-01-02T03:04:05"})");
    tag_round_trip(
        R"({"$bike_recording":"datetime","value":"2000-01-02 03:04:05"})",
        R"({"$bike_recording":"datetime","value":"2000-01-02T03:04:05"})");
    tag_round_trip(
        R"({"$bike_recording":"datetime","value":"2000-01-02x03:04"})",
        R"({"$bike_recording":"datetime","value":"2000-01-02T03:04:00"})");
    tag_round_trip(
        R"({"$bike_recording":"datetime","value":"2000-01-0190304"})",
        R"({"$bike_recording":"datetime","value":"2000-01-01T03:04:00"})");
    tag_round_trip(
        R"({"$bike_recording":"datetime","value":"2000-W01-1"})",
        R"({"$bike_recording":"datetime","value":"2000-01-03T00:00:00"})");
    tag_round_trip(
        R"({"$bike_recording":"datetime","value":"2000W011T0304"})",
        R"({"$bike_recording":"datetime","value":"2000-01-03T03:04:00"})");
    tag_round_trip(
        R"({"$bike_recording":"datetime","value":"20000102T030405"})",
        R"({"$bike_recording":"datetime","value":"2000-01-02T03:04:05"})");
    tag_round_trip(
        R"({"$bike_recording":"datetime","value":"2000-01-02T03:04:05.5"})",
        R"({"$bike_recording":"datetime","value":"2000-01-02T03:04:05.500000"})");
    tag_round_trip(
        R"({"$bike_recording":"datetime","value":"2000-01-02T03:04:05,5"})",
        R"({"$bike_recording":"datetime","value":"2000-01-02T03:04:05.500000"})");
    tag_round_trip(
        R"({"$bike_recording":"datetime","value":"2000-01-02T03:04:05.123456789"})",
        R"({"$bike_recording":"datetime","value":"2000-01-02T03:04:05.123456"})");
    tag_round_trip(
        R"({"$bike_recording":"datetime","value":"2000-01-02T03:04:05Z"})",
        R"({"$bike_recording":"datetime","value":"2000-01-02T03:04:05+00:00"})");
    tag_round_trip(
        R"({"$bike_recording":"datetime","value":"2000-01-02T03:04:05+01:30"})",
        R"({"$bike_recording":"datetime","value":"2000-01-02T03:04:05+01:30"})");
    tag_round_trip(
        R"({"$bike_recording":"datetime","value":"2000-01-02T03:04:05+01:30:15"})",
        R"({"$bike_recording":"datetime","value":"2000-01-02T03:04:05+01:30:15"})");
    tag_round_trip(
        R"({"$bike_recording":"datetime","value":"2000-01-02T03:04:05+01:30:15.5"})",
        R"({"$bike_recording":"datetime","value":"2000-01-02T03:04:05+01:30:15.500000"})");
    tag_round_trip(
        R"({"$bike_recording":"datetime","value":"2000-01-02T03:04:05-05"})",
        R"({"$bike_recording":"datetime","value":"2000-01-02T03:04:05-05:00"})");
    tag_round_trip(
        R"({"$bike_recording":"datetime","value":"2000-01-01"})",
        R"({"$bike_recording":"datetime","value":"2000-01-01T00:00:00"})");
    tag_invalid(R"({"$bike_recording":"datetime","value":"2000-13-01"})");
    tag_invalid(R"({"$bike_recording":"datetime","value":"2000-02-30"})");
    tag_invalid(R"({"$bike_recording":"datetime","value":"2001-02-29T00:00"})");
    tag_invalid(R"({"$bike_recording":"datetime","value":"2000-01-01T25:00"})");
    tag_invalid(R"({"$bike_recording":"datetime","value":"2000-01-01T24:00"})");
    tag_invalid(R"({"$bike_recording":"datetime","value":"2000-01-01T24:01"})");
    tag_invalid(R"({"$bike_recording":"datetime","value":"9999-12-31T24:00"})");
    tag_invalid(R"({"$bike_recording":"datetime","value":"2000-0102"})");
    tag_invalid(R"({"$bike_recording":"datetime","value":"2000-01-01T"})");
    tag_invalid(R"({"$bike_recording":"datetime","value":"2000-01-01 "})");
    tag_invalid(R"({"$bike_recording":"datetime","value":"2000-01-01T3"})");
    tag_invalid(R"({"$bike_recording":"datetime","value":"2000-01-01T03:04:05+24:00"})");
    tag_invalid(R"({"$bike_recording":"datetime","value":"2000-01-01T03:04:05Zx"})");
    tag_invalid(R"({"$bike_recording":"datetime","value":"2000-01-01TT03"})");
    tag_invalid(R"({"$bike_recording":"datetime","value":"2000-01-01T12:34:56z"})");
    tag_invalid(R"({"$bike_recording":"datetime","value":"0000-01-01"})");
    tag_invalid(R"({"$bike_recording":"datetime","value":"2000-01-01T03:04:5"})");
    tag_invalid(R"({"$bike_recording":"datetime","value":"2000-01-01T12345"})");
    tag_invalid(R"({"$bike_recording":"datetime","value":"2000-01-01T1234:56"})");
    tag_invalid(R"({"$bike_recording":"datetime","value":"2000-01-01T12:3456"})");
    tag_invalid(R"({"$bike_recording":"datetime","value":20000101})");
    tag_invalid(R"({"$bike_recording":"datetime","value":"2000-01-02T03:04:05","x":1})");

    // ---- date tag -----------------------------------------------------------
    tag_round_trip(R"({"$bike_recording":"date","value":"2000-02-29"})",
                   R"({"$bike_recording":"date","value":"2000-02-29"})");
    tag_round_trip(R"({"$bike_recording":"date","value":"20000102"})",
                   R"({"$bike_recording":"date","value":"2000-01-02"})");
    tag_round_trip(R"({"$bike_recording":"date","value":"2000-W52-6"})",
                   R"({"$bike_recording":"date","value":"2000-12-30"})");
    tag_round_trip(R"({"$bike_recording":"date","value":"2024-W09-4"})",
                   R"({"$bike_recording":"date","value":"2024-02-29"})");
    tag_round_trip(R"({"$bike_recording":"date","value":"2000W526"})",
                   R"({"$bike_recording":"date","value":"2000-12-30"})");
    tag_round_trip(R"({"$bike_recording":"date","value":"2000-W01"})",
                   R"({"$bike_recording":"date","value":"2000-01-03"})");
    tag_invalid(R"({"$bike_recording":"date","value":"2000-0102"})");
    tag_invalid(R"({"$bike_recording":"date","value":"2000-01"})");
    tag_invalid(R"({"$bike_recording":"date","value":"2001-02-29"})");
    tag_invalid(R"({"$bike_recording":"date","value":"2000-02-30"})");
    tag_invalid(R"({"$bike_recording":"date","value":"2000-13-01"})");
    tag_invalid(R"({"$bike_recording":"date","value":"2000-00-01"})");
    tag_invalid(R"({"$bike_recording":"date","value":"2000-01-00"})");
    tag_invalid(R"({"$bike_recording":"date","value":"2000-01-32"})");
    tag_invalid(R"({"$bike_recording":"date","value":"999-01-01"})");
    tag_invalid(R"({"$bike_recording":"date","value":"2000-W54"})");
    tag_invalid(R"({"$bike_recording":"date","value":"2000-W00"})");
    tag_invalid(R"({"$bike_recording":"date","value":"0000-01-01"})");
    tag_invalid(R"({"$bike_recording":"date","value":"2000-01-01T03"})");

    // ---- time tag -----------------------------------------------------------
    tag_round_trip(R"({"$bike_recording":"time","value":"12:34:56"})",
                   R"({"$bike_recording":"time","value":"12:34:56"})");
    tag_round_trip(R"({"$bike_recording":"time","value":"12:34"})",
                   R"({"$bike_recording":"time","value":"12:34:00"})");
    tag_round_trip(R"({"$bike_recording":"time","value":"03"})",
                   R"({"$bike_recording":"time","value":"03:00:00"})");
    tag_round_trip(R"({"$bike_recording":"time","value":"T12:34"})",
                   R"({"$bike_recording":"time","value":"12:34:00"})");
    tag_round_trip(R"({"$bike_recording":"time","value":"1234"})",
                   R"({"$bike_recording":"time","value":"12:34:00"})");
    tag_round_trip(R"({"$bike_recording":"time","value":"123456"})",
                   R"({"$bike_recording":"time","value":"12:34:56"})");
    tag_round_trip(R"({"$bike_recording":"time","value":"123456.5"})",
                   R"({"$bike_recording":"time","value":"12:34:56.500000"})");
    tag_round_trip(R"({"$bike_recording":"time","value":"12:34:56,5"})",
                   R"({"$bike_recording":"time","value":"12:34:56.500000"})");
    tag_round_trip(
        R"({"$bike_recording":"time","value":"12:34:56.1234567891"})",
        R"({"$bike_recording":"time","value":"12:34:56.123456"})");
    // CPython 3.13 accepts a decimal mark after any component and a ':'
    // after the seconds (both spill into the fraction); 3.14 rejects them.
    tag_round_trip(R"({"$bike_recording":"time","value":"12.5"})",
                   R"({"$bike_recording":"time","value":"12:00:00.500000"})");
    tag_round_trip(R"({"$bike_recording":"time","value":"12:34.5"})",
                   R"({"$bike_recording":"time","value":"12:34:00.500000"})");
    tag_round_trip(R"({"$bike_recording":"time","value":"1234.5"})",
                   R"({"$bike_recording":"time","value":"12:34:00.500000"})");
    tag_round_trip(R"({"$bike_recording":"time","value":"12:34:56:78"})",
                   R"({"$bike_recording":"time","value":"12:34:56.780000"})");
    tag_round_trip(
        R"({"$bike_recording":"time","value":"12:34:56.0000005"})",
        R"({"$bike_recording":"time","value":"12:34:56"})");
    tag_round_trip(R"({"$bike_recording":"time","value":"12:34:56Z"})",
                   R"({"$bike_recording":"time","value":"12:34:56+00:00"})");
    tag_round_trip(R"({"$bike_recording":"time","value":"12:34:56+01"})",
                   R"({"$bike_recording":"time","value":"12:34:56+01:00"})");
    tag_round_trip(R"({"$bike_recording":"time","value":"12:34:56+0130"})",
                   R"({"$bike_recording":"time","value":"12:34:56+01:30"})");
    tag_round_trip(R"({"$bike_recording":"time","value":"12:34:56+01:30"})",
                   R"({"$bike_recording":"time","value":"12:34:56+01:30"})");
    tag_round_trip(
        R"({"$bike_recording":"time","value":"12:34:56+01:30:15"})",
        R"({"$bike_recording":"time","value":"12:34:56+01:30:15"})");
    tag_round_trip(
        R"({"$bike_recording":"time","value":"12:34:56+01:30:15.5"})",
        R"({"$bike_recording":"time","value":"12:34:56+01:30:15.500000"})");
    tag_round_trip(
        R"({"$bike_recording":"time","value":"12:34:56+01:30.5"})",
        R"({"$bike_recording":"time","value":"12:34:56+01:30:00.500000"})");
    tag_round_trip(
        R"({"$bike_recording":"time","value":"12:34:56+01.5"})",
        R"({"$bike_recording":"time","value":"12:34:56+01:00:00.500000"})");
    tag_round_trip(R"({"$bike_recording":"time","value":"12:34:56-01"})",
                   R"({"$bike_recording":"time","value":"12:34:56-01:00"})");
    tag_round_trip(
        R"({"$bike_recording":"time","value":"12:34:56+23:59:59.9"})",
        R"({"$bike_recording":"time","value":"12:34:56+23:59:59.900000"})");
    tag_invalid(R"({"$bike_recording":"time","value":"24:00"})");
    tag_invalid(R"({"$bike_recording":"time","value":"T24:00"})");
    tag_invalid(R"({"$bike_recording":"time","value":"24:00:00.5"})");
    tag_invalid(R"({"$bike_recording":"time","value":"x12:34"})");
    tag_invalid(R"({"$bike_recording":"time","value":" 12:34"})");
    tag_invalid(R"({"$bike_recording":"time","value":"t12:34"})");
    tag_invalid(R"({"$bike_recording":"time","value":"3"})");
    tag_invalid(R"({"$bike_recording":"time","value":"030"})");
    tag_invalid(R"({"$bike_recording":"time","value":"345"})");
    tag_invalid(R"({"$bike_recording":"time","value":"123"})");
    tag_invalid(R"({"$bike_recording":"time","value":"12345"})");
    tag_invalid(R"({"$bike_recording":"time","value":"1234567"})");
    tag_invalid(R"({"$bike_recording":"time","value":"12:3"})");
    tag_invalid(R"({"$bike_recording":"time","value":"1:34"})");
    tag_invalid(R"({"$bike_recording":"time","value":"12:34:5"})");
    tag_invalid(R"({"$bike_recording":"time","value":"12:34:567"})");
    tag_invalid(R"({"$bike_recording":"time","value":"12:34:56:"})");
    tag_invalid(R"({"$bike_recording":"time","value":"12:34:56."})");
    tag_invalid(R"({"$bike_recording":"time","value":"12:34:56.x"})");
    tag_invalid(R"({"$bike_recording":"time","value":"12:34:56.5x"})");
    tag_invalid(R"({"$bike_recording":"time","value":"12:34:56+24:00"})");
    tag_invalid(R"({"$bike_recording":"time","value":"12:34:56+013"})");
    tag_invalid(R"({"$bike_recording":"time","value":"12:34:56+01305"})");
    tag_invalid(R"({"$bike_recording":"time","value":"12:34:56+013:05"})");
    tag_invalid(R"({"$bike_recording":"time","value":"12:34:56Zx"})");
    tag_invalid(R"({"$bike_recording":"time","value":"12:34:56z"})");
    tag_invalid(R"({"$bike_recording":"time","value":"1234:56"})");
    tag_invalid(R"({"$bike_recording":"time","value":"12:3456"})");
    tag_invalid(R"({"$bike_recording":"time","value":"25:00"})");
    tag_invalid(R"({"$bike_recording":"time","value":"12:60"})");
    tag_invalid(R"({"$bike_recording":"time","value":"12:34:60"})");
    tag_invalid(R"({"$bike_recording":"time","value":1234})");

    // ---- timedelta tag ------------------------------------------------------
    tag_round_trip(
        R"({"$bike_recording":"timedelta","days":1,"seconds":2,"microseconds":3})",
        R"({"$bike_recording":"timedelta","days":1,"microseconds":3,"seconds":2})");
    tag_round_trip(
        R"({"$bike_recording":"timedelta","days":0,"seconds":90000,"microseconds":1500000})",
        R"({"$bike_recording":"timedelta","days":1,"microseconds":500000,"seconds":3601})");
    tag_round_trip(
        R"({"$bike_recording":"timedelta","days":-1,"seconds":86400,"microseconds":0})",
        R"({"$bike_recording":"timedelta","days":0,"microseconds":0,"seconds":0})");
    tag_round_trip(
        R"({"$bike_recording":"timedelta","days":0,"seconds":0,"microseconds":-1})",
        R"({"$bike_recording":"timedelta","days":-1,"microseconds":999999,"seconds":86399})");
    tag_round_trip(
        R"({"$bike_recording":"timedelta","days":-1,"seconds":-1,"microseconds":0})",
        R"({"$bike_recording":"timedelta","days":-2,"microseconds":0,"seconds":86399})");
    tag_round_trip(
        R"({"$bike_recording":"timedelta","days":0,"seconds":2147483648,"microseconds":0})",
        R"({"$bike_recording":"timedelta","days":24855,"microseconds":0,"seconds":11648})");
    tag_round_trip(
        R"({"$bike_recording":"timedelta","days":999999999,"seconds":0,"microseconds":0})",
        R"({"$bike_recording":"timedelta","days":999999999,"microseconds":0,"seconds":0})");
    tag_round_trip(
        R"({"$bike_recording":"timedelta","days":-999999999,"seconds":0,"microseconds":0})",
        R"({"$bike_recording":"timedelta","days":-999999999,"microseconds":0,"seconds":0})");
    tag_invalid(
        R"({"$bike_recording":"timedelta","days":1000000000,"seconds":0,"microseconds":0})");
    tag_invalid(
        R"({"$bike_recording":"timedelta","days":-999999999,"seconds":-1,"microseconds":0})");
    tag_invalid(
        R"({"$bike_recording":"timedelta","days":1.5,"seconds":0,"microseconds":0})");
    tag_invalid(
        R"({"$bike_recording":"timedelta","days":"1","seconds":0,"microseconds":0})");
    tag_invalid(
        R"({"$bike_recording":"timedelta","days":true,"seconds":0,"microseconds":0})");
    tag_invalid(
        R"({"$bike_recording":"timedelta","days":1,"seconds":0})");
    tag_invalid(
        R"({"$bike_recording":"timedelta","days":1,"seconds":0,"microseconds":0,"x":1})");
    tag_invalid(
        R"({"$bike_recording":"timedelta","days":2147483648,"seconds":0,"microseconds":0})");

    // ---- escaped reserved-key objects ---------------------------------------
    tag_round_trip(
        R"({"$bike_recording":"object","items":[["a",1],["$bike_recording","x"]]})",
        R"({"$bike_recording":"object","items":[["a",1],["$bike_recording","x"]]})");
    {
        const Value decoded = parse_recording_json(
            R"({"$bike_recording":"object","items":[["a",1],["$bike_recording","x"]]})");
        const Value::Object &object = as_object(decoded, "$");
        bt::require(object.size() == 2U && object[0].first == "a" &&
                        object[1].first == kRecordingTag,
                    "escaped object restores insertion order");
        bt::require(recording_json(decoded) ==
                        R"({"$bike_recording":"object","items":[["a",1],)"
                        R"(["$bike_recording","x"]]})",
                    "reserved-key object re-escapes");
        bt::require(canonical_json(decoded) ==
                        R"({"$bike_recording":"x","a":1})",
                    "canonical serialization keeps the literal object");
        // nested tags inside escaped items decode too
        const Value nested = parse_recording_json(
            R"({"$bike_recording":"object","items":[["a",)"
            R"({"$bike_recording":"float64","value":"+inf"}]]})");
        const Value::Object &inner = as_object(nested, "$");
        bt::require(std::isinf(tagged_double(inner[0].second, "$.a")),
                    "nested tag inside escaped object decodes");
    }
    tag_invalid(
        R"({"$bike_recording":"object","items":[["a",1],["a",2]]})");
    tag_invalid(R"({"$bike_recording":"object","items":[["a"]]})");
    tag_invalid(
        R"({"$bike_recording":"object","items":[["a",1,2]]})");
    tag_invalid(R"({"$bike_recording":"object","items":[[1,2]]})");
    tag_invalid(R"({"$bike_recording":"object","items":{}})");
    tag_invalid(R"({"$bike_recording":"object","items":[["a",1]],"x":1})");
    // ordinary objects without the reserved key stay ordinary
    {
        const Value ordinary = parse_recording_json(
            R"({"outer":{"$bike_recording":"float64","value":"nan"},"list":[null,false]})");
        const Value *outer =
            find_member(as_object(ordinary, "$"), "outer");
        bt::require(outer != nullptr, "outer member present");
        bt::require(std::isnan(tagged_double(*outer, "$.outer")),
                    "nested float64 tag inside ordinary object");
    }
    // non-object scalars through the recording path are unaffected
    bt::require(tag_canonical("null") == "null", "recording null");
    bt::require(tag_canonical("[null,false]") == "[null,false]",
                "recording false/null distinct");
    bt::require(tag_canonical(R"({"a":null,"b":false,"c":0,"d":""})") ==
                    R"({"a":null,"b":false,"c":0,"d":""})",
                "null/false/0/empty stay distinct");

    // ---- serializer validation on constructed trees --------------------------
    {
        Value::Object duplicate{{"k", Value{Number{"1"}}},
                                {"k", Value{Number{"2"}}}};
        bt::invalid(
            [&] { (void) canonical_json(Value{duplicate}); });
        bt::invalid(
            [&] { (void) canonical_json(Value{Number{"1.2.3"}}); });
        bt::invalid([&] { (void) canonical_json(Value{Number{""}}); });
        bt::invalid([&] { (void) canonical_json(Value{Number{"abc"}}); });
        bt::invalid([&] {
            (void) canonical_json(
                Value{std::string("\xc3\x28", 2)}); // invalid UTF-8
        });
        bt::invalid([&] {
            (void) recording_json(Value{Number{"1e999"}});
        });
        // reserved-key object escapes only under the recording serializer
        Value::Object reserved{{"$bike_recording", Value{std::string("x")}},
                               {"a", Value{Number{"1"}}}};
        bt::require(recording_json(Value{reserved}) ==
                        R"({"$bike_recording":"object","items":[)"
                        R"(["$bike_recording","x"],["a",1]]})",
                    "constructed reserved-key object escapes");
        bt::require(canonical_json(Value{reserved}) ==
                        R"({"$bike_recording":"x","a":1})",
                    "canonical serializer keeps it literal");
        // malformed constructed tags fail closed
        bt::invalid([&] {
            (void) recording_json(
                Value{Tagged{.kind = "bogus", .fields = {{"value", "1"}}}});
        });
        bt::invalid([&] {
            (void) recording_json(
                Value{Tagged{.kind = "float64", .fields = {{"value", "NaN"}}}});
        });
        bt::invalid([&] {
            (void) recording_json(
                Value{Tagged{.kind = "decimal", .fields = {{"value", "1.2.3"}}}});
        });
        bt::invalid([&] {
            (void) recording_json(
                Value{Tagged{.kind = "timedelta",
                             .fields = {{"days", "1000000000"},
                                        {"seconds", "0"},
                                        {"microseconds", "0"}}}});
        });
        bt::invalid([&] {
            (void) recording_json(
                Value{Tagged{.kind = "timedelta",
                             .fields = {{"days", "1"},
                                        {"seconds", "0"}}}});
        });
        bt::invalid([&] {
            (void) recording_json(Value{Tagged{
                .kind = "bytes", .fields = {{"value", "not!base64"}}}});
        });
        // constructed timedelta normalizes like the domain object
        bt::require(
            recording_json(Value{Tagged{.kind = "timedelta",
                                        .fields = {{"days", "0"},
                                                  {"seconds", "90000"},
                                                  {"microseconds", "0"}}}}) ==
                R"({"$bike_recording":"timedelta","days":1,"microseconds":0,)"
                R"("seconds":3600})",
            "constructed timedelta normalizes");
        bt::require(
            recording_json(
                Value{Tagged{.kind = "float64", .fields = {{"value", "-inf"}}}}) ==
                R"({"$bike_recording":"float64","value":"-inf"})",
            "constructed float64 tag");
        bt::require(canonical_json(Value{std::monostate{}}) == "null",
                    "explicit null serializes");
    }

}

void checks_sha256() {
    using namespace bike::io;
    // ---- SHA-256 -------------------------------------------------------------
    {
        const auto bytes_of = [](std::string_view text) {
            return std::as_bytes(std::span(text));
        };
        bt::require(sha256(bytes_of("")) ==
                        "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca49"
                        "5991b7852b855",
                    "sha256 empty vector");
        bt::require(sha256(bytes_of("abc")) ==
                        "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb41"
                        "0ff61f20015ad",
                    "sha256 'abc' vector");
        bt::require(sha256(bytes_of("abcdbcdecdefdefgefghfghighijhijkijkljk"
                                    "lmklmnlmnomnopnopq")) ==
                        "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ec"
                        "edd419db06c1",
                    "sha256 56-byte vector");
        {
            // one million 'a' — FIPS long-message vector through the
            // streaming update path in bounded chunks
            Sha256 stream;
            const std::string chunk(1024, 'a');
            for (int i = 0; i < 976; ++i)
                stream.update(std::as_bytes(std::span(chunk)));
            stream.update(std::as_bytes(
                std::span(chunk).first(576U)));
            std::string digest;
            for (const std::uint8_t b : stream.finish()) {
                constexpr std::string_view hex{"0123456789abcdef"};
                digest += hex[b >> 4U];
                digest += hex[b & 0x0fU];
            }
            bt::require(digest ==
                            "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e"
                            "046d39ccc7112cd0",
                        "sha256 streaming vector");
        }
        // binary fixture digest through the file path
        const std::string payload =
            bt::text(bt::fixture("harness/utf8.txt"));
        bt::require(sha256(std::as_bytes(std::span(payload))) ==
                        "08f3677f5b3c2e83dba262983949b963482ee6c248252876ff"
                        "adc398c72f62b6",
                    "utf8.txt digest matches the manifest");
    }

}

void checks_manifest() {
    using namespace bike::io;
    // ---- oracle manifest + provenance ---------------------------------------
    {
        const auto &entries = bt::manifest();
        bt::require(entries.size() >= 92U,
                    "oracle manifest parses and validates");
        const bt::ManifestFixture &utf8 = bt::manifest_entry("harness/utf8.txt");
        bt::require(utf8.comparison == 'Q' && utf8.kind == "accepted" &&
                        utf8.files.size() == 1U && utf8.files[0].bytes == 5,
                    "manifest entry fields validated");
        // every declared file on every entry: size + sha256 verified
        for (const bt::ManifestFixture &entry : entries)
            bt::manifest_verify_files(entry);
        // verified_fixture path equality with the T02 traversal
        bt::require(bt::verified_fixture("harness/utf8.txt") ==
                        bt::fixture("harness/utf8.txt"),
                    "verified fixture resolves through the harness root");
        bt::require(bt::text(bt::verified_fixture("harness/utf8.txt")) ==
                        bt::text(bt::fixture("harness/utf8.txt")),
                    "verified fixture bytes readable");
        // doctored declarations fail closed
        {
            bt::ManifestFixture tampered = utf8;
            tampered.files[0].sha256 = std::string(64U, '0');
            require_fails([&] { bt::manifest_verify_files(tampered); });
            tampered = utf8;
            tampered.files[0].bytes = utf8.files[0].bytes + 1;
            require_fails([&] { bt::manifest_verify_files(tampered); });
            tampered = utf8;
            tampered.files[0].path = "harness/does-not-exist.bin";
            require_fails([&] { bt::manifest_verify_files(tampered); });
        }
        // structural validation of a minimal manifest
        {
            const auto parsed = bt::manifest_parse(manifest_doc());
            bt::require(parsed.size() == 1U && parsed[0].name == "g/f" &&
                            parsed[0].comparison == 'Q' &&
                            parsed[0].kind == "accepted" &&
                            parsed[0].files.size() == 1U &&
                            parsed[0].files[0].path == "g/f.txt",
                        "minimal manifest parses");
        }
        manifest_invalid("{}");
        manifest_invalid(
            R"({"schema":"migration-oracle-manifest/v1","schema":"x"})");
        {
            std::string doc = manifest_doc();
            replace_once(doc, "migration-oracle-manifest/v1", "other");
            manifest_invalid(doc);
            doc = manifest_doc();
            replace_once(doc,
                         R"("commit":"0123456789abcdef0123456789abcdef)"
                         R"(01234567")",
                         R"("commit":"zzzz")");
            manifest_invalid(doc);
            doc = manifest_doc();
            replace_once(doc, R"("oracle":{)", R"("oracle_missing":{)");
            manifest_invalid(doc);
            // uppercase hex must not satisfy the lowercase digest contract
            doc = manifest_doc();
            const std::string lower(64, 'a');
            const std::string upper(64, 'A');
            replace_once(doc, R"("sha256":")" + lower + R"("})",
                         R"("sha256":")" + upper + R"("})");
            manifest_invalid(doc);
            doc = manifest_doc();
            replace_once(doc, R"("platform":"p",)", "");
            manifest_invalid(doc);
            doc = manifest_doc();
            replace_once(doc, R"("expected":{"kind":"accepted"})",
                         R"("expected":{"kind":"bogus"})");
            manifest_invalid(doc);
            doc = manifest_doc();
            replace_once(doc, R"("comparison":"Q")",
                         R"("comparison":"Z")");
            manifest_invalid(doc);
            doc = manifest_doc();
            replace_once(doc, R"("recipe":"RCP-00")",
                         R"("recipe":"R-00")");
            manifest_invalid(doc);
            doc = manifest_doc();
            replace_once(doc, R"("produced_by":"p")",
                         R"("produced_by":"p","extra":1)");
            manifest_invalid(doc);
            doc = manifest_doc();
            replace_once(doc, R"("path":"g/f.txt")",
                         R"("path":"../escape")");
            manifest_invalid(doc);
            doc = manifest_doc();
            replace_once(doc, R"("path":"g/f.txt")",
                         R"("path":"/absolute")");
            manifest_invalid(doc);
            doc = manifest_doc();
            replace_once(doc, R"("path":"g/f.txt")",
                         R"("path":"a//b")");
            manifest_invalid(doc);
            doc = manifest_doc();
            replace_once(doc, R"("path":"g/f.txt")",
                         R"("path":"a/./b")");
            manifest_invalid(doc);
            doc = manifest_doc();
            replace_once(doc, R"("path":"g/f.txt")",
                         R"("path":"a\\b")");
            manifest_invalid(doc);
            doc = manifest_doc();
            replace_once(doc, R"("bytes":1)", R"("bytes":-1)");
            manifest_invalid(doc);
            doc = manifest_doc();
            replace_once(doc, R"("sha256":")" + lower + R"("}]}]})",
                         R"("sha256":")" + lower.substr(0, 63) + R"("}]}]})");
            manifest_invalid(doc);
            doc = manifest_doc();
            // shape/dtype/order must arrive together and agree with bytes
            replace_once(doc, R"("bytes":1)",
                         R"("bytes":8,"shape":[2],"dtype":"<f8")");
            manifest_invalid(doc);
            doc = manifest_doc();
            replace_once(doc, R"("bytes":1)",
                         R"("bytes":16,"shape":[2],"dtype":"<f8",)"
                         R"("order":"C")");
            {
                const auto parsed = bt::manifest_parse(doc);
                bt::require(parsed[0].files[0].dtype == "<f8" &&
                                parsed[0].files[0].shape.size() == 1U &&
                                parsed[0].files[0].shape[0] == 2,
                            "shape/dtype block validates");
            }
            doc = manifest_doc();
            replace_once(doc, R"("bytes":1)",
                         R"("bytes":16,"shape":[3],"dtype":"<f8",)"
                         R"("order":"C")");
            manifest_invalid(doc);
            doc = manifest_doc();
            replace_once(doc, R"("bytes":1)",
                         R"("bytes":8,"shape":[],"dtype":"<f8",)"
                         R"("order":"F")");
            manifest_invalid(doc);
            doc = manifest_doc();
            replace_once(doc, R"("fixtures":[)", R"("fixtures_extra":[)");
            manifest_invalid(doc);
            // fixtures: [] — truncate the tail
            doc = manifest_doc();
            replace_once(doc, R"("fixtures":[{)", R"("fixtures":[] ,"x":[{)");
            manifest_invalid(doc);
            // files: [] — swap the whole files member value
            doc = manifest_doc();
            replace_once(doc,
                         R"("files":[{"path":"g/f.txt","bytes":1,)"
                         R"("sha256":")" + lower + R"("}])",
                         R"("files":[])");
            manifest_invalid(doc);
            // a second fixture repeating the same name is rejected
            doc = manifest_doc();
            replace_once(
                doc, "}]}]}",
                R"(}]},{"name":"g/f","group":"g","comparison":"Q",)"
                R"("recipe":"RCP-00","produced_by":"p","notes":"n",)"
                R"("inputs":[],"expected":{"kind":"accepted"},"files":)"
                R"([{"path":"g/f2.txt","bytes":1,"sha256":")" + lower +
                R"("}]}]})");
            doc = manifest_doc();
            // input member contracts: sha256 must be 64-hex or null
            replace_once(doc, R"("inputs":[])",
                         R"("inputs":[{"kind":"k","path":"p",)"
                         R"("sha256":null}])");
            {
                const auto parsed = bt::manifest_parse(doc);
                bt::require(parsed.size() == 1U, "nullable input digest ok");
            }
            doc = manifest_doc();
            replace_once(doc, R"("inputs":[])",
                         R"("inputs":[{"kind":"k","path":"p",)"
                         R"("sha256":"xyz"}])");
            manifest_invalid(doc);
            doc = manifest_doc();
            replace_once(doc, R"("blockers":null)", R"("blockers":[])");
            {
                const auto parsed = bt::manifest_parse(doc);
                bt::require(parsed.size() == 1U, "empty blockers array ok");
            }
            doc = manifest_doc();
            replace_once(doc, R"("blockers":null)", R"("blockers":5)");
            manifest_invalid(doc);
            // expected-kind required fields
            doc = manifest_doc();
            replace_once(doc, R"("expected":{"kind":"accepted"})",
                         R"("expected":{"kind":"invalid"})");
            manifest_invalid(doc); // missing error/message
            doc = manifest_doc();
            replace_once(doc, R"("expected":{"kind":"accepted"})",
                         R"("expected":{"kind":"invalid","error":"e")"
                         R"(,"message":"m"})");
            {
                const auto parsed = bt::manifest_parse(doc);
                bt::require(parsed[0].kind == "invalid", "invalid kind ok");
            }
        }
        manifest_invalid(manifest_doc() + " garbage");
    }
}

} // namespace

int main() {
    // cppcheck's throwInEntryPoint cannot see that bt::run catches every
    // test escape; the outer try covers std::function allocation failure.
    try {
        int failures = 0;
        failures += bt::run(checks_value_ownership);
        failures += bt::run(checks_json);
        failures += bt::run(checks_recording);
        failures += bt::run(checks_sha256);
        failures += bt::run(checks_manifest);
        return failures == 0 ? 0 : 1;
    } catch (...) {
        return 1;
    }
}
