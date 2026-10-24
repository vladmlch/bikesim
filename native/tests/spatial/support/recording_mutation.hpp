#pragma once
// Authored corruption fixtures mutate a newly owned copy and recompute only
// transport hashes. Semantic identities/evidence remain independently checked
// by the production reader/replay path.
#include "recording/reader.hpp"
#include "runtime/setup_values.hpp"
#include "io/hash.hpp"
#include "platform/files.hpp"
#include <array>
#include <filesystem>
#include <functional>

namespace spatial_tests {
inline std::vector<std::byte> saved_asset(const bike::recording::Connection& connection,std::string_view name) {
    const std::array params{runtime::setupvalue::text(name)};
    const auto rows=connection.query("SELECT content FROM assets WHERE episode_id=0 AND name=?",params);
    if(rows.size()!=1)throw std::invalid_argument("corruption fixture asset missing");
    return std::get<bike::io::Bytes>(runtime::setupvalue::field(bike::io::Value{rows.front()},"content").data).value;
}
inline bike::io::Value saved_document(const bike::recording::Connection& connection,std::string_view name) {
    const auto bytes=saved_asset(connection,name);
    return bike::io::parse_recording_json(std::string(reinterpret_cast<const char*>(bytes.data()),bytes.size()));
}
inline void replace_asset(const bike::recording::Connection& connection,std::string_view name,std::vector<std::byte> bytes) {
    using namespace runtime::setupvalue;
    const auto hash=bike::io::sha256(bytes);
    const std::array params{text(hash),Value{bike::io::Bytes{std::move(bytes)}},text(name)};
    connection.execute("UPDATE assets SET sha256=?,content=? WHERE episode_id=0 AND name=?",params);
}
inline void replace_document(const bike::recording::Connection& connection,std::string_view name,const bike::io::Value& value) {
    const auto source=bike::io::recording_json(value);const auto bytes=std::as_bytes(std::span(source));
    replace_asset(connection,name,{bytes.begin(),bytes.end()});
}
inline std::filesystem::path mutate_recording(const std::filesystem::path& source,const std::filesystem::path& destination,
    const std::function<void(const bike::recording::Connection&)>& mutation) {
    using namespace runtime::setupvalue;
    Value manifest;{const bike::recording::Recording original(source);manifest=original.manifest();}
    bike::platform::create_directory(destination);
    for(const auto& [name,unused]:bike::io::as_object(field(manifest,"file_sha256"),"hashes")) {
        (void)unused;std::filesystem::copy_file(source/name,destination/name);
    }
    {bike::recording::Database database(destination/"run.duckdb");mutation(database.connection());database.connection().execute("CHECKPOINT");}
    auto hashes=field(manifest,"file_sha256");set(hashes,"run.duckdb",text(bike::platform::file_sha256(destination/"run.duckdb")));
    set(manifest,"file_sha256",std::move(hashes));const auto json=bike::io::recording_json(manifest)+"\n";
    bike::platform::write_exclusive(destination/"manifest.json",std::as_bytes(std::span(json)));
    return destination;
}
}
