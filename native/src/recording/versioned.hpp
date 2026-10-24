#pragma once
// Internal schema dispatch; the public W00 catalog overloads stay unchanged.
#include "catalog.hpp"
namespace bike::recording {
class RunCapture;
[[nodiscard]] CatalogVersion catalog_version(const Catalog&) noexcept;
[[nodiscard]] CatalogVersion capture_version(const RunCapture&);
[[nodiscard]] CatalogVersion database_version(const Connection&);
[[nodiscard]] const TableSpec& table_spec(std::string_view, CatalogVersion);
[[nodiscard]] std::vector<io::Value::Array> catalog_rows(CatalogVersion);
void create_streams(const Connection&, CatalogVersion);
void create_views(const Connection&, CatalogVersion);
}
