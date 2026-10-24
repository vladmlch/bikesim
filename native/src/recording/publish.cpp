#include "publish.hpp"
#include "versioned.hpp"
#include "database.hpp"
#include "report.hpp"
#include "value_util.hpp"
#include "../platform/files.hpp"
#include <cerrno>
#include <cstdlib>
#include <limits>
#include <sys/stat.h>
#include <system_error>
#include <unistd.h>

namespace bike::recording {
namespace {
std::filesystem::path staging_directory(const std::filesystem::path& root) {
    auto name = (root / ".run-recording-unsealed-XXXXXX").string(); std::vector<char> bytes(name.begin(), name.end()); bytes.push_back('\0');
    if (!::mkdtemp(bytes.data())) throw std::system_error(errno, std::generic_category(), "exclusive recording staging");
    return {bytes.data()};
}
void write_text(const std::filesystem::path& path, const std::string& text) { platform::write_exclusive(path, std::as_bytes(std::span(text.data(), text.size()))); }
std::string name_base(const RunMetadata& metadata) {
    const auto& iso = metadata.started_utc;
    std::string stamp = iso.substr(0,4) + iso.substr(5,2) + iso.substr(8,2) + 'T' + iso.substr(11,2) + iso.substr(14,2) + iso.substr(17,2) + '.';
    std::string micro;
    if (iso.size() > 19 && iso[19] == '.') for (std::size_t i = 20; i < iso.size() && iso[i] >= '0' && iso[i] <= '9'; ++i) micro += iso[i];
    micro.resize(6, '0'); stamp += micro;
    return stamp + 'Z' + '-' + metadata.run_uuid;
}
void checked_link(const std::filesystem::path& source, const std::filesystem::path& destination) {
    auto const from = platform::open_regular(source); struct stat a{}, b{};
    if (::fstat(from.get(), &a) != 0) throw std::system_error(errno, std::generic_category(), "fstat source payload");
    platform::link_new(source, destination);
    auto const to = platform::open_regular(destination);
    if (::fstat(to.get(), &b) != 0) throw std::system_error(errno, std::generic_category(), "fstat destination payload");
    if (a.st_dev != b.st_dev || a.st_ino != b.st_ino) throw std::runtime_error("recording payload changed during exclusive publication");
}
} // namespace
Publication publish(const RunCapture& capture, const std::filesystem::path& output_root, const FinalizeOptions& options) {
    if (!capture.enabled()) return {.directory={}, .report_status="disabled"};
    if (!options.maximum_batch_rows) throw std::invalid_argument("batch_rows must be positive");
    if (output_root.empty()) throw std::invalid_argument("recording output root must be nonempty");
    std::filesystem::create_directories(output_root);
    const auto staging = staging_directory(output_root);
    std::filesystem::path destination;
    std::string report_status = "complete";
    try {
        write_text(staging / "UNSEALED", "A manifest is required. This directory is not a recording.\n");
        const auto counts = write_database(staging / database_name, capture, options.maximum_batch_rows, options.before_stage);
        io::Value report_error;
        try {
            if (options.before_stage) options.before_stage("report");
            const auto report = options.report ? options.report(staging / database_name) : render_report_database(staging / database_name);
            write_text(staging / report_name, report);
        } catch (const std::exception& error) {
            report_status = "failed"; report_error = value::text(std::string("report generation failed: ") + error.what());
        }
        std::vector<std::string> payloads{std::string(database_name)};
        if (report_status == "complete") payloads.emplace_back(report_name);
        io::Value::Object hashes;
        for (const auto& name : payloads) {
            if (options.before_stage) options.before_stage("sync_" + name);
            auto file = platform::open_regular(staging / name); platform::sync(file); file.close();
            if (options.before_stage) options.before_stage("hash_" + name);
            hashes.emplace_back(name, value::text(platform::file_sha256(staging / name)));
        }
        const io::Value manifest{io::Value::Object{{"schema_version", value::integer(static_cast<int>(capture_version(capture)))}, {"sealed", io::Value{true}},
            {"run_id", value::text(capture.metadata().run_uuid)}, {"database", value::text(database_name)},
            {"file_sha256", io::Value{hashes}}, {"row_counts", io::Value{counts}}, {"report_status", value::text(report_status)}, {"report_error", report_error}}};
        const auto candidate = staging / "sealed-manifest.json";
        if (options.before_stage) options.before_stage("manifest_write");
        write_text(candidate, io::recording_json(manifest) + '\n'); platform::sync_directory(staging);
        const auto base = name_base(capture.metadata());
        for (std::uint64_t attempt = 0;; ++attempt) {
            destination = output_root / (base + (attempt ? "-" + std::to_string(attempt) : ""));
            if (options.before_stage) options.before_stage("destination_create");
            try { platform::create_directory(destination); break; }
            catch (const std::system_error& error) { if (error.code() != std::errc::file_exists) throw; }
            if (attempt == std::numeric_limits<std::uint64_t>::max()) throw std::overflow_error("recording destination attempts exhausted");
        }
        for (const auto& name : payloads) {
            if (options.before_stage) options.before_stage("link_" + name);
            checked_link(staging / name, destination / name);
            if (platform::file_sha256(destination / name) != io::as_string(value::require(hashes, name), "payload hash")) throw std::runtime_error("recording payload changed before sealing");
        }
        platform::sync_directory(destination); platform::sync_directory(output_root);
        if (options.before_stage) options.before_stage("manifest_link");
        // There are no fallible validation operations after the commit link.
        // The candidate is an exclusively written regular file in our 0700
        // staging directory and has already been synced.
        auto const marker = platform::open_regular(candidate);
        (void)marker;
        platform::link_new(candidate, destination / manifest_name); // sole sealing marker, LAST
    } catch (const std::exception& error) {
        throw std::runtime_error(std::string(error.what()) + "; owned evidence retained; unsealed staging: " + staging.string() +
            (destination.empty() ? "" : "; publication destination: " + destination.string()));
    }
    // Once the manifest link exists, cleanup cannot turn a sealed success into
    // a reported failure. A killed RAM-only launch is not crash recoverable.
    try {
        platform::sync_directory(destination); platform::sync_directory(output_root);
        for (const auto& name : {std::string(database_name), std::string(report_name), std::string("sealed-manifest.json"), std::string("UNSEALED")}) {
            std::error_code ignored; std::filesystem::remove(staging / name, ignored);
        }
        std::error_code ignored; std::filesystem::remove(staging, ignored);
    // NOLINTNEXTLINE(bugprone-empty-catch) deliberate: the result is advisory and the failure path is handled separately
    } catch (const std::system_error&) { }
    return {.directory=std::move(destination), .report_status=std::move(report_status)};
}
} // namespace bike::recording
