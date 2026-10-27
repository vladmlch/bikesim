// V08-only fixture preparation. Never linked into an installed executable.
// Owns a newly created destination; the accepted source is opened read-only.
#include "migration/recording_fixture.hpp"
#include "recording/duckdb.hpp"
#include "recording/reader.hpp"
#include "recording/report.hpp"
#include "recording/value_util.hpp"
#include "runtime/setup_values.hpp"
#include "io/hash.hpp"
#include "platform/argv.hpp"
#include "platform/files.hpp"
#include <array>
#include <cstddef>
#include <filesystem>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <variant>

namespace {
void incompatible(const std::filesystem::path& source,const std::filesystem::path& destination) {
    using namespace runtime::setupvalue;
    namespace rec=bike::recording;namespace io=bike::io;
    Value manifest,summary,recipe;
    {
        const rec::Recording accepted(source);
        if(accepted.schema_version()!=2)throw std::invalid_argument("fixture source must be a schema2 research recording");
        manifest=accepted.manifest();
        summary=io::parse_recording_json(io::as_string(field(accepted.episode(0),"summary_json"),"summary"));
        const auto rows=accepted.rows("SELECT content FROM assets WHERE episode_id=0 AND name='recipe.json'");
        if(rows.size()!=1)throw std::invalid_argument("fixture source needs exactly one recipe");
        const auto& bytes=std::get<io::Bytes>(rec::value::require(rows[0],"content").data).value;
        std::string document;document.reserve(bytes.size());for(const auto byte:bytes)document.push_back(static_cast<char>(byte));
        recipe=io::parse_recording_json(document);
    }
    auto execution=field(summary,"execution");
    auto digest=io::as_string(field(execution,"execution_identity"),"identity");
    if(digest.size()!=64)throw std::invalid_argument("fixture source execution identity is not SHA-256");
    digest[0]=digest[0]=='0'?'1':'0';
    set(execution,"execution_identity",text(digest));set(summary,"execution",execution);
    auto research=field(summary,"research");set(research,"execution",execution);set(summary,"research",research);
    set(recipe,"execution_identity_sha256",text(digest));
    for(const auto key:{"checkpoint_initial","checkpoint_final"}) {
        auto checkpoint=field(recipe,key);auto identity=field(checkpoint,"identity");
        set(identity,"solver",text(digest));set(checkpoint,"identity",identity);set(recipe,key,checkpoint);
    }
    bike::platform::create_directory(destination);
    std::filesystem::copy_file(source/"run.duckdb",destination/"run.duckdb");
    {
        rec::Database writable(destination/"run.duckdb");
        const auto asset=[&](std::string_view name,const Value& document) {
            const auto encoded=io::recording_json(document);const auto bytes=std::as_bytes(std::span(encoded));
            const std::array parameters{text(io::sha256(bytes)),Value{io::Bytes{{bytes.begin(),bytes.end()}}},text(name)};
            writable.connection().execute("UPDATE assets SET sha256=?,content=? WHERE episode_id=0 AND name=?",parameters);
        };
        asset("recipe.json",recipe);
        asset("spatial/checkpoint_initial.json",field(recipe,"checkpoint_initial"));
        asset("spatial/checkpoint_final.json",field(recipe,"checkpoint_final"));
        const std::array summary_parameters{text(io::recording_json(summary))};
        writable.connection().execute("UPDATE episodes SET summary_json=? WHERE episode_id=0",summary_parameters);
        writable.connection().execute("CHECKPOINT");
    }
    const auto report=rec::render_report_database(destination/"run.duckdb");
    bike::platform::write_exclusive(destination/"summary.md",std::as_bytes(std::span(report)));
    auto hashes=field(manifest,"file_sha256");
    set(hashes,"run.duckdb",text(bike::platform::file_sha256(destination/"run.duckdb")));
    set(hashes,"summary.md",text(bike::platform::file_sha256(destination/"summary.md")));
    set(manifest,"file_sha256",hashes);set(manifest,"report_status",text("complete"));set(manifest,"report_error",Value{});
    const auto encoded=io::recording_json(manifest)+"\n";
    bike::platform::write_exclusive(destination/"manifest.json",std::as_bytes(std::span(encoded)));
    // A malformed/checksum-broken container would not test compatibility.
    const rec::Recording inspectable(destination);
    if(inspectable.schema_version()!=2)throw std::runtime_error("fixture preparation changed container version");
}
}
int main(int argc,char** argv) {
    try {
        // The shared wrapper keeps raw argv subscripting out of this file under
        // -Wunsafe-buffer-usage; the pair is exactly the one main received.
        const auto arguments=bike::platform::main_arguments(argc,argv);
        if(arguments.size()==3&&std::string_view(arguments[1])=="--planar") {
            std::cout<<recording_fixture::publish(arguments[2]).directory.string()<<'\n';return 0;
        }
        if(arguments.size()==4&&std::string_view(arguments[1])=="--incompatible") {incompatible(arguments[2],arguments[3]);return 0;}
        throw std::invalid_argument("usage: installed_fixture --planar OUT | --incompatible SOURCE OUT");
    } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
