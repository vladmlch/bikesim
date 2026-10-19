#pragma once

#include "../io/value.hpp"
#include <cstdint>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>
struct mjModel_;
using mjModel = mjModel_;

namespace bike::recording {
inline constexpr int schema_version = 1;
inline constexpr int replay_contract_version = 3;
inline constexpr std::size_t batch_rows = 4096;
inline constexpr std::string_view database_name = "run.duckdb";
inline constexpr std::string_view manifest_name = "manifest.json";
inline constexpr std::string_view report_name = "summary.md";

// NOLINTNEXTLINE(misc-no-recursion) recursion mirrors the Python reference structure
struct FieldSpec {
    std::string key, dtype, unit, phase;
    bool optional = false;
    std::string measurement, source, description, frame = "n/a";
    std::vector<std::string> path, aliases;
    std::optional<std::vector<std::int64_t>> shape;
    std::optional<std::vector<std::string>> axes;
    std::vector<std::string> except_keys;
    std::vector<FieldSpec> children;
    void validate() const;
    [[nodiscard]] std::string duckdb_type() const;
};
struct TableSpec {
    std::string name, kind;
    std::vector<FieldSpec> keys, fields;
    std::string description, source;
    std::vector<std::string> claims, exempt_keys;
    void validate() const;
    [[nodiscard]] const FieldSpec* field(std::string_view key) const noexcept;
};
struct JointCoords { std::string name, joint_type; std::int64_t qposadr = 0, dofadr = 0; };
struct Coordinate {
    std::int64_t index = 0;
    std::string kind, joint, component, dof_kind, unit;
};
struct CoordinateCatalog {
    std::vector<Coordinate> qpos, dof;
    [[nodiscard]] std::string_view force_unit(std::size_t index) const;
    [[nodiscard]] const std::string& velocity_unit(std::size_t index) const;
};
struct AliasSpec {
    std::string alias, canonical;
    std::optional<std::int64_t> index;
    std::optional<std::string> parameter;
    std::string note;
};
struct ScalarAlias { std::string alias, kind, joint; };

class Catalog {
public:
    Catalog(std::vector<TableSpec> tables, CoordinateCatalog coordinates,
            std::vector<AliasSpec> aliases);
    [[nodiscard]] const TableSpec& table(std::string_view name) const;
    [[nodiscard]] const std::vector<TableSpec>& table_order() const noexcept { return tables_; }
    [[nodiscard]] const CoordinateCatalog& coordinates() const noexcept { return coordinates_; }
    [[nodiscard]] const std::vector<AliasSpec>& aliases() const noexcept { return aliases_; }
    [[nodiscard]] std::vector<std::string> channel_roots() const;
    [[nodiscard]] const TableSpec& channel_table(std::string_view root) const;
    [[nodiscard]] std::optional<AliasSpec> resolve_alias(std::string_view name) const;
private:
    std::vector<TableSpec> tables_;
    CoordinateCatalog coordinates_;
    std::vector<AliasSpec> aliases_;
    std::map<std::string, std::string, std::less<>> roots_;
};

[[nodiscard]] Catalog build_catalog(const mjModel& model);
[[nodiscard]] Catalog build_catalog(std::span<const JointCoords> joints,
                                   std::span<const ScalarAlias> scalar_aliases);
// The default aliases bind the planar source joints. Explicit empty aliases
// support free/ball models and schema-only construction without guessing.
[[nodiscard]] Catalog build_catalog(std::span<const JointCoords> joints);
[[nodiscard]] const Catalog& schema_catalog();
[[nodiscard]] const std::vector<TableSpec>& research_tables();
[[nodiscard]] const std::vector<TableSpec>& all_tables();
[[nodiscard]] const TableSpec& table_spec(std::string_view name);
[[nodiscard]] io::Value describe_catalog(const Catalog& catalog);
[[nodiscard]] io::Value describe_field(const FieldSpec& field);
[[nodiscard]] std::string quote(std::string_view identifier);
[[nodiscard]] std::vector<std::vector<std::string>> parent_paths(const TableSpec& table);
[[nodiscard]] std::vector<std::string> parent_masks(const TableSpec& table);
[[nodiscard]] std::vector<std::pair<std::string, std::string>> column_types(const TableSpec& table);

class Connection;
void create_metadata(Connection const& connection);
void create_streams(Connection const& connection);
void create_views(Connection const& connection);
void create_legacy_view(Connection const& connection);
[[nodiscard]] std::vector<io::Value::Array> catalog_rows();
} // namespace bike::recording
