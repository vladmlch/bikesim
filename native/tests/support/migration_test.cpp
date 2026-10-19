// migration_test.cpp — implementations for the migration contract-test
// harness. Explicit little-endian binary64 decoding, length checks,
// fixture-root containment and bounded payload reads per
// migration/interface-contracts.md §8. T05 layers the strict oracle-manifest
// boundary on top: provenance parses through the owned bike::io JSON tree —
// duplicate keys, trailing bytes and malformed UTF-8 are parse errors; every
// structural violation is std::invalid_argument naming the member path.
#include "migration_test.hpp"

#include "io/hash.hpp"
#include "io/json.hpp"
#include "io/value.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <initializer_list>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <system_error>
#include <unordered_set>
#include <utility>
#include <variant>

#ifndef BIKE_FIXTURE_ROOT
#error "BIKE_FIXTURE_ROOT must be supplied as a compile definition"
#endif
#ifndef BIKE_ORACLE_MANIFEST
#error "BIKE_ORACLE_MANIFEST must be supplied as a compile definition"
#endif

namespace {
    // Bound for a single fixture payload read (interface-contracts.md §8:
    // test payloads are bounded; large trajectories are chunked into
    // declared fixture pieces).
    constexpr std::uintmax_t max_fixture_bytes = 512ULL * 1024ULL * 1024ULL;

    // require()/run() share a scoreboard so a test body that ran zero checks
    // cannot pass vacuously; thread_local keeps a later threaded helper
    // honest.
    std::size_t &require_count() {
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables) deliberate harness scoreboard shared by require()/run()
        static thread_local std::size_t value = 0;
        return value;
    }

    std::string hex64(std::uint64_t bits) {
        std::ostringstream out;
        out << "0x" << std::hex << std::setfill('0') << std::setw(16) << bits;
        return out.str();
    }
} // namespace

namespace bt {

int run(const std::function<void()> &test) {
    const std::size_t before = require_count();
    try {
        test();
    } catch (const std::exception &error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    } catch (...) {
        std::cerr << "FAIL non-standard test exception\n";
        return 1;
    }
    const std::size_t executed = require_count() - before;
    if (executed == 0) {
        std::cerr << "FAIL test body ran zero checks\n";
        return 1;
    }
    std::cout << "PASS " << executed << " checks\n";
    return 0;
}

void require(bool condition, std::string_view message) {
    ++require_count();
    if (!condition)
        throw std::runtime_error(std::string(message));
}

std::filesystem::path fixture(std::string_view relative) {
    const std::filesystem::path requested{relative};
    require(!requested.empty(), "fixture path must not be empty");
    require(!requested.is_absolute(), "fixture path must be relative");
    for (const auto &part: requested)
        require(part != "..",
                "fixture path must not contain a '..' component");
    const std::filesystem::path root =
        std::filesystem::weakly_canonical(BIKE_FIXTURE_ROOT);
    const std::filesystem::path full =
        std::filesystem::weakly_canonical(root / requested);
    const std::filesystem::path inside = full.lexically_relative(root);
    require(!inside.empty() && !inside.is_absolute(),
            "fixture path must stay inside the fixture root");
    for (const auto &part: inside)
        require(part != "..",
                "fixture path must not escape the fixture root");
    require(std::filesystem::exists(full),
            std::string("fixture does not exist: ") + std::string(relative));
    return full;
}

std::string text(const std::filesystem::path &path) {
    require(std::filesystem::is_regular_file(path),
            "fixture is not a regular file");
    const std::uintmax_t size = std::filesystem::file_size(path);
    require(size <= max_fixture_bytes, "fixture payload exceeds 512 MiB");
    std::ifstream stream(path, std::ios::binary);
    require(stream.is_open(), "cannot open fixture file");
    std::string content((std::istreambuf_iterator<char>(stream)),
                        std::istreambuf_iterator<char>());
    require(!stream.bad(), "fixture read failed");
    require(content.size() == size, "fixture changed while reading");
    return content;
}

std::vector<double> read_f64(const std::filesystem::path &path) {
    const std::string bytes = text(path);
    require(bytes.size() % 8 == 0,
            "truncated little-endian binary64 fixture");
    std::vector<double> values;
    values.reserve(bytes.size() / 8);
    for (std::size_t offset = 0; offset < bytes.size(); offset += 8) {
        std::uint64_t bits = 0;
        for (unsigned shift = 0; shift < 8; ++shift)
            bits |= static_cast<std::uint64_t>(
                        static_cast<unsigned char>(bytes.at(offset + shift)))
                    << (8U * shift);
        values.push_back(std::bit_cast<double>(bits));
    }
    return values;
}

void golden_text(std::string_view relative, std::string_view actual) {
    const std::string expected = text(fixture(relative));
    const bool equal = expected == actual;
    std::string message;
    if (!equal) {
        const std::size_t shared = std::min(expected.size(), actual.size());
        std::size_t first = 0;
        while (first < shared && expected.at(first) == actual.at(first))
            ++first;
        message = std::string("golden_text mismatch: ") +
                  std::string(relative) + " first_diff_byte=" +
                  std::to_string(first) + " expected_bytes=" +
                  std::to_string(expected.size()) + " actual_bytes=" +
                  std::to_string(actual.size());
    }
    require(equal, message);
}

void golden_f64(std::string_view relative, std::span<const double> actual) {
    const std::vector<double> expected = read_f64(fixture(relative));
    bool equal = expected.size() == actual.size();
    std::string message;
    if (!equal) {
        message = std::string("golden_f64 shape mismatch: ") +
                  std::string(relative) + " expected_count=" +
                  std::to_string(expected.size()) + " actual_count=" +
                  std::to_string(actual.size());
    } else {
        for (std::size_t index = 0; index < actual.size(); ++index) {
            const std::uint64_t expected_bits =
                std::bit_cast<std::uint64_t>(expected.at(index));
            const std::uint64_t actual_bits =
                std::bit_cast<std::uint64_t>(actual[index]);
            if (expected_bits != actual_bits) {
                equal = false;
                message = std::string("golden_f64 mismatch: ") +
                          std::string(relative) + " index=" +
                          std::to_string(index) + " expected=" +
                          hex64(expected_bits) + " actual=" +
                          hex64(actual_bits);
                break;
            }
        }
    }
    require(equal, message);
}

// ---- strict oracle-manifest/provenance validation (task T05) --------------

namespace {
    using bike::io::Number;
    using bike::io::Value;

    [[noreturn]] void manifest_error(std::string_view detail) {
        throw std::invalid_argument(std::string("bt::manifest: ") +
                                    std::string(detail));
    }

    const Value::Object *manifest_object(const Value &value,
                                         const std::string &path) {
        const auto *object = std::get_if<Value::Object>(&value.data);
        if (object == nullptr) manifest_error(path + ": expected object");
        return object;
    }
    const Value::Array *manifest_array(const Value &value,
                                       const std::string &path) {
        const auto *array = std::get_if<Value::Array>(&value.data);
        if (array == nullptr) manifest_error(path + ": expected array");
        return array;
    }
    const std::string *manifest_string(const Value &value,
                                       const std::string &path) {
        const auto *text_value = std::get_if<std::string>(&value.data);
        if (text_value == nullptr)
            manifest_error(path + ": expected string");
        return text_value;
    }
    const Number *manifest_number(const Value &value,
                                  const std::string &path) {
        const auto *number = std::get_if<Number>(&value.data);
        if (number == nullptr) manifest_error(path + ": expected number");
        return number;
    }
    bool manifest_bool(const Value &value, const std::string &path) {
        const auto *flag = std::get_if<bool>(&value.data);
        // cppcheck-suppress nullPointerRedundantCheck
        if (flag == nullptr) manifest_error(path + ": expected boolean");
        // manifest_error is [[noreturn]] — flag is provably nonnull here.
        // cppcheck-suppress nullPointerRedundantCheck
        return *flag;
    }
    const Value *manifest_member(const Value::Object &object,
                                 std::string_view key,
                                 const std::string &path) {
        const Value *member = bike::io::find_member(object, key);
        if (member == nullptr)
            manifest_error(path + ": missing member '" +
                           std::string(key) + "'");
        return member;
    }

    const Value::Object *member_object(const Value::Object &object,
                                       std::string_view key,
                                       const std::string &path) {
        return manifest_object(*manifest_member(object, key, path),
                               path + "." + std::string(key));
    }
    const Value::Array *member_array(const Value::Object &object,
                                     std::string_view key,
                                     const std::string &path) {
        return manifest_array(*manifest_member(object, key, path),
                              path + "." + std::string(key));
    }
    const std::string *member_string(const Value::Object &object,
                                     std::string_view key,
                                     const std::string &path) {
        return manifest_string(*manifest_member(object, key, path),
                               path + "." + std::string(key));
    }
    bool member_bool(const Value::Object &object, std::string_view key,
                     const std::string &path) {
        return manifest_bool(*manifest_member(object, key, path),
                             path + "." + std::string(key));
    }
    std::int64_t member_i64(const Value::Object &object, std::string_view key,
                            const std::string &path) {
        const std::string member_path = path + "." + std::string(key);
        const Number &number =
            *manifest_number(*manifest_member(object, key, path), member_path);
        try {
            return bike::io::number_i64(number, member_path);
        } catch (const std::invalid_argument &) {
            manifest_error(member_path + ": expected int64 integer lexeme");
        }
    }
    std::int64_t manifest_i64(const Value &value, const std::string &path) {
        const Number &number = *manifest_number(value, path);
        try {
            return bike::io::number_i64(number, path);
        } catch (const std::invalid_argument &) {
            manifest_error(path + ": expected int64 integer lexeme");
        }
    }

    bool lower_hex(std::string_view text, std::size_t count) {
        if (text.size() != count) return false;
        for (const char ch: text)
            if (!((ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'f')))
                return false;
        return true;
    }
    void require_sha256_hex(std::string_view text, const std::string &path) {
        if (!lower_hex(text, 64))
            manifest_error(path + ": expected 64 lowercase hex characters");
    }

    // A manifest-declared path is a relative POSIX path: nonempty
    // components, no absolute prefix, no '.'/'..' components, no backslash
    // or NUL (fixture() re-verifies containment canonically at use).
    void manifest_path_check(std::string_view relative,
                             const std::string &path) {
        if (relative.empty() || relative.front() == '/' ||
            relative.find('\\') != std::string_view::npos ||
            relative.find('\0') != std::string_view::npos)
            manifest_error(path + ": invalid manifest-relative path '" +
                           std::string(relative) + "'");
        std::size_t begin = 0;
        while (begin <= relative.size()) {
            const std::size_t slash = relative.find('/', begin);
            const std::size_t end = slash == std::string_view::npos
                                        ? relative.size()
                                        : slash;
            const std::string_view component =
                relative.substr(begin, end - begin);
            if (component.empty() || component == "." || component == "..")
                manifest_error(path + ": invalid path component in '" +
                               std::string(relative) + "'");
            if (slash == std::string_view::npos) break;
            begin = slash + 1;
        }
    }

    void require_known_members(
        const Value::Object &object,
        std::initializer_list<std::string_view> allowed,
        const std::string &path) {
        for (const auto &member: object)
            if (std::ranges::find(allowed,
                                  std::string_view(member.first)) ==
                allowed.end())
                manifest_error(path + ": unknown member '" + member.first +
                               "'");
    }

    // One manifest "files" entry: closed member set, contained relative
    // path, int64 byte bound, 64-hex digest and — for ndarray payloads —
    // the all-or-none dtype/order/shape block whose element count times the
    // dtype element size must equal "bytes" exactly.
    ManifestFile manifest_file(const Value::Object &file,
                               const std::string &path) {
        require_known_members(file,
                              {"bytes", "dtype", "order", "path", "sha256",
                               "shape"},
                              path);
        ManifestFile out;
        out.path = *member_string(file, "path", path);
        manifest_path_check(out.path, path + ".path");
        out.bytes = member_i64(file, "bytes", path);
        if (out.bytes < 0 ||
            std::cmp_greater(static_cast<std::uintmax_t>(out.bytes),
                             max_fixture_bytes))
            manifest_error(path + ".bytes: declared size out of range");
        out.sha256 = *member_string(file, "sha256", path);
        require_sha256_hex(out.sha256, path + ".sha256");
        const bool has_shape =
            bike::io::find_member(file, "shape") != nullptr;
        const bool has_dtype =
            bike::io::find_member(file, "dtype") != nullptr;
        const bool has_order =
            bike::io::find_member(file, "order") != nullptr;
        if (has_shape != has_dtype || has_dtype != has_order)
            manifest_error(path +
                           ": dtype/order/shape must appear together");
        if (!has_dtype) return out;
        out.dtype = *member_string(file, "dtype", path);
        out.order = *member_string(file, "order", path);
        if (out.order != "C")
            manifest_error(path + ".order: only 'C' layout is declared");
        std::int64_t element_size = 0;
        if (out.dtype == "|u1")
            element_size = 1;
        else if (out.dtype == "<f8" || out.dtype == "<u8")
            element_size = 8;
        else
            manifest_error(path + ".dtype: unsupported dtype '" + out.dtype +
                           "'");
        const Value::Array &shape = *member_array(file, "shape", path);
        std::int64_t elements = 1;
        out.shape.reserve(shape.size());
        for (std::size_t index = 0; index < shape.size(); ++index) {
            const std::int64_t dim =
                manifest_i64(shape[index], path + ".shape[" +
                                                  std::to_string(index) +
                                                  "]");
            if (dim < 0)
                manifest_error(path + ".shape: negative dimension");
            if (dim != 0 && elements > out.bytes / element_size / dim)
                manifest_error(path + ".shape: element count exceeds "
                                      "declared bytes");
            elements *= dim;
            out.shape.push_back(dim);
        }
        if (elements * element_size != out.bytes)
            manifest_error(path + ": shape elements * dtype size != bytes");
        return out;
    }

    // The "expected" contract: kind drives which extra members the capture
    // tool declared — strict per-kind shape, tolerant of extra annotation
    // members the tool adds for accepted cases.
    std::string manifest_expected_kind(const Value::Object &expected,
                                       const std::string &path) {
        const std::string &kind = *member_string(expected, "kind", path);
        static constexpr std::array<std::string_view, 5> kinds{
            "accepted", "invalid", "diagnostic", "unexpected-acceptance",
            "rejected"};
        if (std::ranges::find(kinds, std::string_view(kind)) ==
            kinds.end())
            manifest_error(path + ".kind: unknown expected kind '" + kind +
                           "'");
        if (kind == "invalid" || kind == "rejected") {
            (void) *member_string(expected, "error", path);
            (void) *member_string(expected, "message", path);
        }
        if (kind == "diagnostic") {
            (void) *member_string(expected, "diagnostic", path);
            (void) member_bool(expected, "valid", path);
            if (bike::io::find_member(expected, "reason") != nullptr)
                (void) *member_string(expected, "reason", path);
            if (bike::io::find_member(expected, "replay") != nullptr)
                (void) *member_string(expected, "replay", path);
        }
        if (bike::io::find_member(expected, "feasible") != nullptr)
            (void) member_bool(expected, "feasible", path);
        if (bike::io::find_member(expected, "identical_digests") != nullptr)
            (void) member_bool(expected, "identical_digests", path);
        if (bike::io::find_member(expected, "coerced_grip_attachment") !=
            nullptr)
            (void) *member_string(expected, "coerced_grip_attachment", path);
        return kind;
    }
} // namespace

std::vector<ManifestFixture> manifest_parse(std::string_view source) {
    const Value root_value = bike::io::parse_json(source);
    const Value::Object &root = *manifest_object(root_value, "$");
    if (*member_string(root, "schema", "$") !=
        "migration-oracle-manifest/v1")
        manifest_error("$.schema: expected 'migration-oracle-manifest/v1'");
    if (member_string(root, "created_utc", "$")->empty())
        manifest_error("$.created_utc: expected nonempty string");

    const Value::Object &oracle = *member_object(root, "oracle", "$");
    const std::string &commit = *member_string(oracle, "commit", "$.oracle");
    if (!lower_hex(commit, 40))
        manifest_error("$.oracle.commit: expected 40 lowercase hex "
                       "characters");
    require_sha256_hex(*member_string(oracle, "source_sha256", "$.oracle"),
                       "$.oracle.source_sha256");
    for (const std::string_view key:
         {"describe", "status_porcelain", "worktree", "bike_sim_file"})
        (void) *member_string(oracle, key, "$.oracle");

    const Value::Object &tool = *member_object(root, "tool", "$");
    if (member_string(tool, "path", "$.tool")->empty())
        manifest_error("$.tool.path: expected nonempty string");
    require_sha256_hex(*member_string(tool, "sha256", "$.tool"),
                       "$.tool.sha256");
    const Value::Array &argv = *member_array(tool, "argv", "$.tool");
    for (std::size_t index = 0; index < argv.size(); ++index)
        (void) *manifest_string(argv[index],
                               "$.tool.argv[" + std::to_string(index) + "]");

    const Value::Object &environment =
        *member_object(root, "environment", "$");
    for (const std::string_view key:
         {"executable", "machine", "mujoco_version", "platform", "python"})
        if (member_string(environment, key, "$.environment")->empty())
            manifest_error("$.environment." + std::string(key) +
                           ": expected nonempty string");
    (void) *member_string(environment, "numpy_config", "$.environment");
    (void) *member_object(environment, "packages", "$.environment");

    // "blockers" is null or an array of objects (the capture tool emits
    // null when the oracle run had no blockers).
    const Value &blockers_value = *manifest_member(root, "blockers", "$");
    if (!bike::io::is_null(blockers_value)) {
        const Value::Array &blockers = *manifest_array(blockers_value,
                                                      "$.blockers");
        for (std::size_t index = 0; index < blockers.size(); ++index)
            (void) *manifest_object(blockers[index],
                                   "$.blockers[" +
                                       std::to_string(index) + "]");
    }
    const Value::Array &controls =
        *member_array(root, "negative_controls", "$");
    for (std::size_t index = 0; index < controls.size(); ++index) {
        const std::string path =
            "$.negative_controls[" + std::to_string(index) + "]";
        const Value::Object &control =
            *manifest_object(controls[index], path);
        (void) *member_string(control, "name", path);
        (void) *member_string(control, "detail", path);
        (void) member_bool(control, "result", path);
    }

    const Value::Array &fixtures = *member_array(root, "fixtures", "$");
    if (fixtures.empty())
        manifest_error("$.fixtures: manifest declares zero fixtures");
    std::vector<ManifestFixture> out;
    out.reserve(fixtures.size());
    std::unordered_set<std::string> names;
    for (std::size_t index = 0; index < fixtures.size(); ++index) {
        const std::string path =
            "$.fixtures[" + std::to_string(index) + "]";
        const Value::Object &fixture_object =
            *manifest_object(fixtures[index], path);
        require_known_members(fixture_object,
                              {"comparison", "expected", "files", "group",
                               "inputs", "name", "notes", "produced_by",
                               "recipe"},
                              path);
        ManifestFixture entry;
        entry.name = *member_string(fixture_object, "name", path);
        if (entry.name.empty() || !names.insert(entry.name).second)
            manifest_error(path + ".name: empty or duplicate fixture name");
        entry.group = *member_string(fixture_object, "group", path);
        if (entry.group.empty())
            manifest_error(path + ".group: expected nonempty string");
        const std::string &comparison =
            *member_string(fixture_object, "comparison", path);
        if (comparison.size() != 1 ||
            std::string_view("BNPQS").find(comparison.front()) ==
                std::string_view::npos)
            manifest_error(path +
                           ".comparison: expected one of B, N, P, Q, S");
        entry.comparison = comparison.front();
        const std::string &recipe =
            *member_string(fixture_object, "recipe", path);
        if (recipe.size() != 6 || !recipe.starts_with("RCP-") ||
            recipe[4] < '0' || recipe[4] > '9' || recipe[5] < '0' ||
            recipe[5] > '9')
            manifest_error(path + ".recipe: expected 'RCP-NN'");
        if (member_string(fixture_object, "produced_by", path)->empty())
            manifest_error(path + ".produced_by: expected nonempty string");
        (void) *member_string(fixture_object, "notes", path);
        const Value::Array &inputs =
            *member_array(fixture_object, "inputs", path);
        for (std::size_t input_index = 0; input_index < inputs.size();
             ++input_index) {
            const std::string input_path =
                path + ".inputs[" + std::to_string(input_index) + "]";
            const Value::Object &input =
                *manifest_object(inputs[input_index], input_path);
            for (const auto &member: input) {
                if (bike::io::is_null(member.second)) continue;
                const std::string &text_value = *manifest_string(
                    member.second,
                    input_path + "." + member.first);
                if (member.first == "sha256")
                    require_sha256_hex(text_value,
                                       input_path + ".sha256");
            }
        }
        const Value::Object &expected =
            *member_object(fixture_object, "expected", path);
        entry.kind = manifest_expected_kind(expected, path + ".expected");
        const Value::Array &files =
            *member_array(fixture_object, "files", path);
        if (files.empty())
            manifest_error(path + ".files: fixture declares zero files");
        std::unordered_set<std::string> file_paths;
        entry.files.reserve(files.size());
        for (std::size_t file_index = 0; file_index < files.size();
             ++file_index) {
            ManifestFile file = manifest_file(
                *manifest_object(files[file_index],
                                path + ".files[" +
                                    std::to_string(file_index) + "]"),
                path + ".files[" + std::to_string(file_index) + "]");
            if (!file_paths.insert(file.path).second)
                manifest_error(path + ".files: duplicate declared path '" +
                               file.path + "'");
            entry.files.push_back(std::move(file));
        }
        out.push_back(std::move(entry));
    }
    return out;
}

void manifest_verify_files(const ManifestFixture &entry) {
    for (const ManifestFile &file: entry.files) {
        const std::string payload = text(fixture(file.path));
        require(payload.size() ==
                    static_cast<std::size_t>(file.bytes),
                "manifest byte count mismatch for " + file.path);
        const std::string digest = bike::io::sha256(
            std::as_bytes(std::span(payload)));
        require(digest == file.sha256,
                "manifest sha256 mismatch for " + file.path + ": declared " +
                    file.sha256 + " actual " + digest);
    }
}

const std::vector<ManifestFixture> &manifest() {
    static const std::vector<ManifestFixture> entries = [] {
        const std::filesystem::path path = [] {
            const std::filesystem::path declared{BIKE_ORACLE_MANIFEST};
            require(std::filesystem::is_regular_file(declared),
                    "oracle manifest is not a regular file: "
                    BIKE_ORACLE_MANIFEST);
            return declared;
        }();
        return manifest_parse(text(path));
    }();
    return entries;
}

const ManifestFixture &manifest_entry(std::string_view name) {
    const ManifestFixture *found = nullptr;
    for (const ManifestFixture &entry: manifest())
        if (entry.name == name) found = &entry;
    require(found != nullptr,
            std::string("fixture not declared in the oracle manifest: ") +
                std::string(name));
    return *found;
}

std::filesystem::path verified_fixture(std::string_view name) {
    const ManifestFixture &entry = manifest_entry(name);
    manifest_verify_files(entry);
    return fixture(name);
}

} // namespace bt
