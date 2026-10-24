#include "qualification_registry.hpp"

#include <array>
#include <cstddef>
#include <initializer_list>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>

namespace spatial_tests {
namespace {
namespace io = bike::io;
using io::Value;

[[noreturn]] void fail(std::string_view path, std::string_view reason) {
    throw std::invalid_argument("spatial_tests::validate_qualification: " +
        std::string(path) + ": " + std::string(reason));
}

const Value& member(const Value::Object& object, std::string_view name,
                    const std::string& path) {
    const auto* value = io::find_member(object, name);
    if (value == nullptr) fail(path + "." + std::string(name), "missing member");
    return *value;
}

const Value::Object& object(const Value& value, const std::string& path,
                            std::initializer_list<std::string_view> keys) {
    const auto& result = io::as_object(value, path);
    std::set<std::string_view> names;
    for (const auto& [name, unused] : result) {
        (void)unused;
        if (!names.insert(name).second)
            fail(path + "." + name, "duplicate member");
        bool known = false;
        for (const auto key : keys) known = known || key == name;
        if (!known) fail(path + "." + name, "unknown member");
    }
    for (const auto key : keys) (void)member(result, key, path);
    return result;
}

const std::string& nonempty(const Value& value, const std::string& path) {
    const auto& text = io::as_string(value, path);
    if (text.empty() || text.find_first_not_of(" \t\r\n") == std::string::npos ||
        text.find('\0') != std::string::npos)
        fail(path, "expected nonempty text without NUL");
    return text;
}

bool hex(std::string_view text, std::size_t size) {
    if (text.size() != size) return false;
    for (const char c : text)
        if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f')))
            return false;
    return true;
}

void digest(const Value& value, const std::string& path) {
    if (!hex(io::as_string(value, path), 64))
        fail(path, "expected SHA-256 as 64 lowercase hexadecimal characters");
}

void evidence_path(std::string_view relative, const std::string& path) {
    if (relative.empty() || relative.front() == '/' ||
        relative.find_first_of("\\:") != std::string_view::npos ||
        relative.find('\0') != std::string_view::npos)
        fail(path, "expected repository-relative POSIX evidence path");
    std::size_t begin = 0;
    for (;;) {
        const auto slash = relative.find('/', begin);
        const auto end = slash == std::string_view::npos ? relative.size() : slash;
        const auto component = relative.substr(begin, end - begin);
        if (component.empty() || component == "." || component == "..")
            fail(path, "empty, dot or traversal path component");
        if (slash == std::string_view::npos) break;
        begin = slash + 1;
    }
}

const std::string& status(const Value& value, const std::string& path) {
    const auto& text = io::as_string(value, path);
    if (text != "pending" && text != "passed" &&
        text != "failed" && text != "blocked")
        fail(path, "expected pending, passed, failed or blocked");
    return text;
}

void identity(const Value& value, const std::string& path) {
    const auto& fields = object(value, path,
        {"source_sha", "dependency_hashes", "compiler", "provider", "config_sha256"});
    const auto& source = io::as_string(member(fields, "source_sha", path), path + ".source_sha");
    if (!hex(source, 40) && !hex(source, 64))
        fail(path + ".source_sha", "expected 40- or 64-character lowercase Git object id");
    const auto& dependencies = io::as_object(member(fields, "dependency_hashes", path),
                                             path + ".dependency_hashes");
    if (dependencies.empty()) fail(path + ".dependency_hashes", "empty dependency identity");
    std::set<std::string_view> names;
    for (const auto& [name, hash] : dependencies) {
        const auto field = path + ".dependency_hashes." + name;
        if (name.empty() || name.find_first_not_of(" \t\r\n") == std::string::npos ||
            name.find('\0') != std::string::npos || !names.insert(name).second)
            fail(field, "empty or duplicate dependency name");
        digest(hash, field);
    }
    (void)nonempty(member(fields, "compiler", path), path + ".compiler");
    (void)nonempty(member(fields, "provider", path), path + ".provider");
    digest(member(fields, "config_sha256", path), path + ".config_sha256");
}

// This checks declared evidence structure only. Reading the bytes, verifying
// their digest/candidate/platform and retaining command/exit/check counts are
// separate V10 responsibilities; a syntactically valid hash is not a result.
void evidence(const Value& value, bool passed, const std::string& path) {
    const auto& entries = io::as_array(value, path);
    if (passed && entries.empty()) fail(path, "passed entry requires evidence");
    std::set<std::string_view> paths;
    for (std::size_t i = 0; i < entries.size(); ++i) {
        const auto entry_path = path + "[" + std::to_string(i) + "]";
        const auto& fields = object(entries[i], entry_path, {"path", "sha256"});
        const auto& relative = nonempty(member(fields, "path", entry_path), entry_path + ".path");
        evidence_path(relative, entry_path + ".path");
        if (!paths.insert(relative).second) fail(entry_path + ".path", "duplicate evidence path");
        digest(member(fields, "sha256", entry_path), entry_path + ".sha256");
    }
}

void entries(const Value& value, char prefix, bool overall_passed,
             bool has_identity, const std::string& path) {
    const auto& rows = io::as_array(value, path);
    if (rows.size() != 7) fail(path, "requires exactly seven entries, numbered 0 through 6");
    std::array<bool, 7> seen{};
    for (std::size_t i = 0; i < rows.size(); ++i) {
        const auto row_path = path + "[" + std::to_string(i) + "]";
        const auto& fields = object(rows[i], row_path, {"id", "status", "evidence"});
        const auto& id = io::as_string(member(fields, "id", row_path), row_path + ".id");
        if (id.size() != 2 || id[0] != prefix || id[1] < '0' || id[1] > '6')
            fail(row_path + ".id", "unknown milestone or gate id");
        const auto index = static_cast<std::size_t>(id[1] - '0');
        if (seen[index]) fail(row_path + ".id", "duplicate milestone or gate id");
        seen[index] = true;
        const bool passed = status(member(fields, "status", row_path), row_path + ".status") == "passed";
        if (overall_passed && !passed)
            fail(row_path + ".status", "overall passed requires every milestone and gate passed");
        if (passed && !has_identity)
            fail("$.identity", "passed milestone or gate requires candidate identity");
        evidence(member(fields, "evidence", row_path), passed, row_path + ".evidence");
    }
}
} // namespace

void validate_qualification(const io::Value& value) {
    const auto& root = object(value, "$",
        {"qualification_schema", "status", "identity", "milestones", "upstream_gates"});
    if (io::as_int64(member(root, "qualification_schema", "$"), "$.qualification_schema") != 1)
        fail("$.qualification_schema", "unsupported qualification schema");
    const bool passed = status(member(root, "status", "$"), "$.status") == "passed";
    const auto& candidate = member(root, "identity", "$");
    const bool has_identity = !io::is_null(candidate);
    if (passed && !has_identity) fail("$.identity", "passed qualification requires candidate identity");
    if (has_identity) identity(candidate, "$.identity");
    entries(member(root, "milestones", "$"), 'M', passed, has_identity, "$.milestones");
    entries(member(root, "upstream_gates", "$"), 'G', passed, has_identity, "$.upstream_gates");
}
} // namespace spatial_tests
