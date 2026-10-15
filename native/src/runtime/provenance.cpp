#include "provenance.hpp"
#include "research_build_identity.hpp"
#include "../diag.hpp"
#include <algorithm>
#include <dlfcn.h>
#include <stdexcept>
#include <string>
#include <vector>
#include <mujoco/mujoco.h>
#include <nanobind/stl/string.h>
#if defined(__APPLE__)
#include <array>
#include <cstring>
#include <string_view>
#include <mach-o/dyld.h>
#include <mach-o/loader.h>
#elif defined(__linux__)
#include <filesystem>
#include <system_error>
#include <link.h>
#endif

namespace runtime {
namespace {

// The provider images the qualified build linked against — one loader name
// per line, baked at configure time from the dependency-probe record.
std::vector<std::string> required_provider_images() {
    const std::string joined = &build_identity::required_provider_images[0];
    std::vector<std::string> names;
    std::string::size_type start = 0;
    while (start <= joined.size()) {
        const std::string::size_type end = joined.find('\n', start);
        std::string name =
            joined.substr(start, end == std::string::npos ? end : end - start);
        if (!name.empty()) names.push_back(std::move(name));
        if (end == std::string::npos) break;
        start = end + 1;
    }
    return names;
}

#if defined(__APPLE__)

// LC_UUID of a loaded Mach-O image — a read-only walk of the load commands at
// the header address the public _dyld API reports. The header is a data
// object of the loaded image; no function-to-object conversion is involved.
// The Mach-O load-command table is a foreign binary layout whose own ncmds/
// cmdsize fields are the bounds — there is no span to traverse instead.
std::string format_uuid(const uuid_command &command) {
    constexpr std::string_view digits{"0123456789abcdef"};
    std::array<char, 36> text{};
    std::size_t out = 0;
    std::size_t index = 0;
    for (const uint8_t byte : command.uuid) {
        if (index == 4 || index == 6 || index == 8 || index == 10)
            text[out++] = '-';
        const unsigned value = byte;
        text[out++] = digits[value / 16];
        text[out++] = digits[value % 16];
        ++index;
    }
    return {text.data(), text.size()};
}

// LC_UUID of a loaded Mach-O image — a read-only walk of the load commands at
// the header address the public _dyld API reports. The header is a data
// object of the loaded image; no function-to-object conversion is involved.
// The Mach-O load-command table is a foreign binary layout whose own ncmds/
// cmdsize fields are the bounds — there is no span to traverse instead, so
// each record is memcpy'd at byte offsets relative to the mapped header.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wunsafe-buffer-usage")
// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,
//             cppcoreguidelines-pro-bounds-pointer-arithmetic)
// The dyld API yields only the base address of a foreign binary structure —
// a byte-cursor walk over the mapped header is the only correct traversal;
// every load command is memcpy'd rather than cast-dereferenced.
std::string image_uuid(const mach_header *header) {
    if (header == nullptr || header->magic != MH_MAGIC_64) return {};
    const char *cursor =
        reinterpret_cast<const char *>(header) + sizeof(mach_header_64);
    for (uint32_t index = 0; index < header->ncmds; ++index) {
        load_command command{};
        std::memcpy(&command, cursor, sizeof command);
        if (command.cmdsize < sizeof(load_command)) return {};
        if (command.cmd == LC_UUID) {
            uuid_command uuid{};
            std::memcpy(&uuid, cursor, sizeof uuid);
            return format_uuid(uuid);
        }
        cursor += command.cmdsize;
    }
    return {};
}
// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,
//           cppcoreguidelines-pro-bounds-pointer-arithmetic)
NATIVE_DIAG_POP

// Every loaded Accelerate.framework image with its LC_UUID — loader
// inspection at identity boundaries only, never inside a step loop.
nanobind::list provider_images() {
    constexpr std::string_view prefix{
        "/System/Library/Frameworks/Accelerate.framework/"};
    nanobind::list images;
    std::vector<std::string> loaded_names;
    for (uint32_t index = 0; index < _dyld_image_count(); ++index) {
        const char *name = _dyld_get_image_name(index);
        if (name == nullptr) continue;
        const std::string image{name};
        loaded_names.push_back(image);
        if (!image.starts_with(prefix)) continue;  // not a provider image
        nanobind::dict entry;
        entry["path"] = nanobind::str(image.c_str());
        const std::string uuid = image_uuid(_dyld_get_image_header(index));
        if (uuid.empty()) {
            entry["macho_uuid"] = nanobind::none();
        } else {
            entry["macho_uuid"] = nanobind::str(uuid.c_str());
        }
        images.append(entry);
    }
    for (const std::string &required : required_provider_images()) {
        if (std::ranges::find(loaded_names, required) == loaded_names.end())
            throw std::runtime_error(
                "qualified numerics provider image is not loaded: " + required);
    }
    return images;
}

#elif defined(__linux__)

#if defined(__GLIBC__)
int collect_provider_image(dl_phdr_info *info, size_t, void *out) {
    auto *loaded = static_cast<std::vector<std::filesystem::path> *>(out);
    if (info->dlpi_name == nullptr || info->dlpi_name[0] == '\0') return 0;
    std::error_code error;
    const std::filesystem::path resolved =
        std::filesystem::canonical(info->dlpi_name, error);
    if (!error) loaded->push_back(resolved);
    return 0;
}
#endif

// The qualified provider file among the actually-loaded images — the
// kernel's loader table via dl_iterate_phdr, resolved to canonical paths.
nanobind::list provider_images() {
#if defined(__GLIBC__)
    std::vector<std::filesystem::path> loaded;
    dl_iterate_phdr(collect_provider_image, &loaded);
    nanobind::list images;
    for (const std::string &required : required_provider_images()) {
        std::error_code error;
        const std::filesystem::path expected =
            std::filesystem::canonical(required, error);
        if (error || std::ranges::find(loaded, expected) == loaded.end())
            throw std::runtime_error(
                "qualified numerics provider image is not loaded: " + required);
        nanobind::dict entry;
        entry["path"] = nanobind::str(expected.string().c_str());
        entry["macho_uuid"] = nanobind::none();
        images.append(entry);
    }
    return images;
#else
    throw std::runtime_error(
        "numerics provider image inspection requires the glibc "
        "dl_iterate_phdr loader interface");
#endif
}

#else

nanobind::list provider_images() {
    throw std::runtime_error(
        "numerics provider image inspection is unavailable on this platform");
}

#endif

} // namespace

void bind_provenance(nanobind::module_ &module) {
    module.def("runtime_identity", [] {
        // In the supplied MuJoCo engine_support.c, mj_versionString returns a
        // static character array inside the loaded library. dladdr on that
        // DATA address avoids a function-to-object pointer reinterpret_cast.
        const char *version = mj_versionString();
        Dl_info location{};
        if (version == nullptr || dladdr(static_cast<const void *>(version), &location) == 0
                || location.dli_fname == nullptr)
            throw std::runtime_error("cannot identify the loaded MuJoCo library");
        nanobind::dict result;
        result["mujoco_version"] = nanobind::str(version);
        result["mujoco_library_path"] = nanobind::str(location.dli_fname);
        // &array[0] produces the char pointer without an implicit array decay.
        result["native_source_sha256"] = nanobind::str(&build_identity::native_source_sha256[0]);
        result["build_context_json"] = nanobind::str(&build_identity::context_json[0]);
        // The qualified provider policy (compile-time) and the provider
        // images actually loaded into this process (runtime). The Python
        // side cross-checks the two before native execution.
        result["provider_policy_json"] =
            nanobind::str(&build_identity::numerics_provider_policy_json[0]);
        result["numerics_provider_images"] = provider_images();
        return result;
    });
}
} // namespace runtime
