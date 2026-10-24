// W54 source controls. Execution belongs to V10; synthetic accepted data below
// exercises the structural validator and is never persisted as qualification.
#include "support/migration_test.hpp"
#include "spatial/support/qualification_registry.hpp"
#include "spatial/support/spatial_fixture.hpp"
#include "fixture_paths.hpp"
#include "io/json.hpp"

#include <algorithm>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <variant>

namespace {
namespace io = bike::io;
using io::Value;

Value& field(Value& value, std::string_view key) {
    auto& object = std::get<Value::Object>(value.data);
    for (auto& [name, entry] : object) if (name == key) return entry;
    throw std::runtime_error("qualification test fixture is missing " + std::string(key));
}

const Value& field(const Value& value, std::string_view key) {
    const auto& object = io::as_object(value, "qualification-control");
    const auto* result = io::find_member(object, key);
    if (result == nullptr)
        throw std::runtime_error("qualification test fixture is missing " + std::string(key));
    return *result;
}

void set(Value& value, std::string_view key, Value member) {
    field(value, key) = std::move(member);
}

Value text(std::string_view value) { return Value{std::string(value)}; }
Value integer(int value) { return Value{io::Number{std::to_string(value)}}; }

Value::Array& rows(Value& value, std::string_view key) {
    return std::get<Value::Array>(field(value, key).data);
}

void reject(const Value& value, std::string_view path) {
    bool rejected = false;
    try {
        spatial_tests::validate_qualification(value);
    } catch (const std::invalid_argument& error) {
        rejected = std::string_view(error.what()).find(path) != std::string_view::npos;
    }
    bt::require(rejected, "qualification rejection must identify the offending field path");
}

Value accepted_control(const Value& pending) {
    auto result = pending;
    set(result, "status", text("passed"));
    set(result, "identity", Value{Value::Object{
        {"source_sha", text(std::string(40, 'a'))},
        {"dependency_hashes", Value{Value::Object{{"synthetic-dependency-lock", text(std::string(64, 'b'))}}}},
        {"compiler", text("synthetic-compiler-for-schema-control")},
        {"provider", text("synthetic-provider-for-schema-control")},
        {"config_sha256", text(std::string(64, 'c'))}
    }});
    for (const auto key : {"milestones", "upstream_gates"}) {
        for (auto& row : rows(result, key)) {
            set(row, "status", text("passed"));
            const auto& id = io::as_string(field(row, "id"), "control.id");
            set(row, "evidence", Value{Value::Array{Value{Value::Object{
                {"path", text("synthetic-control/" + id + ".json")},
                {"sha256", text(std::string(64, 'd'))}
            }}}});
        }
    }
    return result;
}

void schema_controls(const Value& pending) {
    // The literal false pass from the plan must not be accepted even before
    // individual missing-member diagnostics are considered.
    bt::invalid([] {
        spatial_tests::validate_qualification(io::parse_json(
            R"({"qualification_schema":1,"status":"passed","milestones":[],"upstream_gates":[]})"));
    });
    bt::invalid([] { spatial_tests::validate_qualification(Value{}); });
    auto bad = pending;
    set(bad, "qualification_schema", integer(2));
    reject(bad, "$.qualification_schema");
    set(bad, "qualification_schema", Value{true});
    reject(bad, "$.qualification_schema");
    set(bad, "qualification_schema", Value{io::Number{"1.0"}});
    reject(bad, "$.qualification_schema");
    bad = pending;
    set(bad, "status", text("verified"));
    reject(bad, "$.status");
    bad = pending;
    std::get<Value::Object>(bad.data).emplace_back("status", text("passed"));
    reject(bad, "$.status");
    bad = pending;
    std::get<Value::Object>(bad.data).emplace_back("unknown", Value{});
    reject(bad, "$.unknown");
    bad = pending;
    auto& object = std::get<Value::Object>(bad.data);
    object.erase(std::remove_if(object.begin(), object.end(),
        [](const auto& item) { return item.first == "identity"; }), object.end());
    reject(bad, "$.identity");
    for (const auto state : {"pending", "failed", "blocked"}) {
        auto valid = pending;
        set(valid, "status", text(state));
        spatial_tests::validate_qualification(valid);
        bt::require(true, "non-passed root with complete pending rows remains structurally valid");
    }
}

void collection_controls(const Value& pending, const Value& accepted) {
    for (const auto key : {"milestones", "upstream_gates"}) {
        auto bad = pending;
        rows(bad, key).clear();
        reject(bad, "$." + std::string(key));
        bad = pending;
        rows(bad, key).pop_back();
        reject(bad, "$." + std::string(key));
        bad = pending;
        rows(bad, key).push_back(rows(bad, key).front());
        reject(bad, "$." + std::string(key));
        bad = pending;
        rows(bad, key).back() = rows(bad, key).front();
        reject(bad, "$." + std::string(key) + "[6].id");
        bad = pending;
        set(rows(bad, key).back(), "id", text("M7"));
        reject(bad, "$." + std::string(key) + "[6].id");
        bad = pending;
        set(rows(bad, key).front(), "status", text("skipped"));
        reject(bad, "$." + std::string(key) + "[0].status");
        bad = accepted;
        set(rows(bad, key).front(), "status", text("pending"));
        reject(bad, "$." + std::string(key) + "[0].status");
        bad = accepted;
        set(rows(bad, key).front(), "evidence", Value{Value::Array{}});
        reject(bad, "$." + std::string(key) + "[0].evidence");
    }
}

void identity_controls(const Value& pending, const Value& accepted) {
    auto bad = accepted;
    set(bad, "identity", Value{});
    reject(bad, "$.identity");
    bad = accepted;
    set(bad, "identity", Value{Value::Object{}});
    reject(bad, "$.identity.source_sha");
    for (const auto key : {"source_sha", "compiler", "provider", "config_sha256"}) {
        bad = accepted;
        set(field(bad, "identity"), key, text(""));
        reject(bad, "$.identity." + std::string(key));
    }
    bad = accepted;
    set(field(bad, "identity"), "dependency_hashes", Value{Value::Object{}});
    reject(bad, "$.identity.dependency_hashes");
    bad = accepted;
    set(field(bad, "identity"), "dependency_hashes", Value{Value::Object{{"mujoco", text("pending")}}});
    reject(bad, "$.identity.dependency_hashes.mujoco");
    bad = accepted;
    auto& dependencies = std::get<Value::Object>(field(field(bad, "identity"), "dependency_hashes").data);
    dependencies.emplace_back(dependencies.front());
    reject(bad, "$.identity.dependency_hashes");
    bad = pending;
    set(rows(bad, "milestones").front(), "status", text("passed"));
    reject(bad, "$.identity");
}

void evidence_controls(const Value& accepted) {
    for (const auto unsafe : {"", "/tmp/log.json", "../log.json", "logs/../log.json",
                             "logs//log.json", "logs/./log.json", "logs/", "C:\\log.json"}) {
        auto bad = accepted;
        set(rows(rows(bad, "milestones").front(), "evidence").front(), "path", text(unsafe));
        reject(bad, "$.milestones[0].evidence[0].path");
    }
    auto bad = accepted;
    set(rows(rows(bad, "milestones").front(), "evidence").front(), "sha256", text(std::string(64, 'G')));
    reject(bad, "$.milestones[0].evidence[0].sha256");
    bad = accepted;
    auto& entries = rows(rows(bad, "milestones").front(), "evidence");
    entries.push_back(entries.front());
    reject(bad, "$.milestones[0].evidence[1].path");
}
} // namespace

int main() {
    return bt::run([] {
        const auto pending = spatial_tests::json(
            std::filesystem::path(BIKE_SPATIAL_SOURCE_ROOT) / "migration/spatial3d/qualification.json");
        spatial_tests::validate_qualification(pending);
        bt::require(io::as_string(field(pending, "status"), "$.status") == "pending",
                    "the authored evidence skeleton must not claim completed qualification");
        const auto accepted = accepted_control(pending);
        spatial_tests::validate_qualification(accepted);
        bt::require(true, "complete synthetic accepted schema is a positive structural control");
        schema_controls(pending);
        collection_controls(pending, accepted);
        identity_controls(pending, accepted);
        evidence_controls(accepted);
    });
}
