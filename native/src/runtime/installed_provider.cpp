#include "installed_provider.hpp"
#include "../io/json.hpp"
#include "../platform/files.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace runtime {
namespace {
namespace io = bike::io;
const io::Value& member(const io::Value& object, std::string_view key) {
    if (const auto* value = io::find_member(io::as_object(object, "provider_policy"), key)) return *value;
    throw std::invalid_argument("provider_policy is missing " + std::string(key));
}
void replace(io::Value& object, std::string_view key, io::Value value) {
    auto* fields = std::get_if<io::Value::Object>(&object.data);
    if (fields == nullptr) throw std::invalid_argument("provider_policy must be an object");
    for (auto& [name, field] : *fields) {
        if (name == key) { field = std::move(value); return; }
    }
    throw std::invalid_argument("provider_policy is missing " + std::string(key));
}
bool sha256_text(std::string_view value) {
    return value.size() == 64 && std::ranges::all_of(value, [](char c) {
        return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
    });
}
} // namespace

InstalledProviderIdentity installed_provider_identity(std::string_view policy_json,
    std::span<const std::string> required_images, const std::filesystem::path& executable) {
    auto policy = io::parse_json(policy_json);
    if (io::as_string(member(policy, "backend"), "provider_policy.backend") != "NUMPY_OPENBLAS" ||
        io::as_int64(member(policy, "blas_integer_bits"), "provider_policy.blas_integer_bits") != 64 ||
        io::as_int64(member(policy, "lapack_integer_bits"), "provider_policy.lapack_integer_bits") != 64)
        throw std::invalid_argument("installed provider identity requires the NUMPY_OPENBLAS ILP64 policy");
    auto libraries = io::as_array(member(policy, "libraries"), "provider_policy.libraries");
    if (libraries.empty() || libraries.size() != required_images.size())
        throw std::invalid_argument("provider_policy.libraries must cover every required image exactly once");
    if (executable.empty() || !executable.is_absolute())
        throw std::invalid_argument("installed provider identity requires an absolute executable path");
    const auto image = std::filesystem::canonical(executable);
    if (!std::filesystem::is_regular_file(image) || image.parent_path().filename() != "bin")
        throw std::invalid_argument("installed provider identity requires a regular executable inside bin/");
    const auto library_dir = image.parent_path().parent_path() / "lib";
    if (!std::filesystem::is_directory(library_dir) || std::filesystem::is_symlink(std::filesystem::symlink_status(library_dir)))
        throw std::runtime_error("installed provider lib directory is missing or is a symlink: " + library_dir.string());
    const auto canonical_library_dir = std::filesystem::canonical(library_dir);
    InstalledProviderIdentity result;
    result.required_images.reserve(libraries.size());
    for (std::size_t index = 0; index < libraries.size(); ++index) {
        auto& library = libraries[index];
        const auto& original_name = io::as_string(member(library, "path"), "provider_policy.libraries.path");
        const std::filesystem::path original(original_name);
        const auto& digest = io::as_string(member(library, "sha256"), "provider_policy.libraries.sha256");
        if (original_name != required_images[index] || original_name.find('\0') != std::string::npos ||
            !original.is_absolute() || !original.has_filename() || original.filename() == "." || original.filename() == ".." ||
            io::as_string(member(library, "identity_kind"), "provider_policy.libraries.identity_kind") != "file" || !sha256_text(digest))
            throw std::invalid_argument("provider_policy requires an exact file identity and SHA-256 for each image");
        const auto candidate = canonical_library_dir / original.filename();
        if (std::filesystem::is_symlink(std::filesystem::symlink_status(candidate)))
            throw std::runtime_error("installed provider must not be a symlink: " + candidate.string());
        const auto resolved = std::filesystem::canonical(candidate);
        if (resolved.parent_path() != canonical_library_dir)
            throw std::runtime_error("installed provider escapes the adjacent lib directory: " + candidate.string());
        if (!std::filesystem::is_regular_file(resolved))
            throw std::runtime_error("installed provider must be a regular file: " + resolved.string());
        if (bike::platform::file_sha256(resolved) != digest)
            throw std::runtime_error("installed provider SHA-256 mismatch: " + resolved.string());
        const auto name = resolved.string();
        if (std::ranges::find(result.required_images, name) != result.required_images.end())
            throw std::invalid_argument("provider_policy resolves duplicate installed provider images");
        result.required_images.push_back(name);
        replace(library, "path", io::Value{name});
    }
    replace(policy, "libraries", io::Value{std::move(libraries)});
    result.policy_json = io::canonical_json(policy);
    return result;
}
} // namespace runtime
