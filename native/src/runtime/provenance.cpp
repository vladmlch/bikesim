// The pure provenance producer (task T11): mj_versionString/dladdr
// resolution plus the compiled-in Track-B build identity, and the loaded
// numerics-provider inspection. The loader inspection itself lives in
// provider_images.cpp, which the configure-time identity probe also compiles.
// The extension's held-GIL dict boxing is encode_binding.cpp's
// bind_provenance().
#include "provenance.hpp"
#include "provider_images.hpp"
#include "research_build_identity.hpp"
#if defined(__linux__) && defined(BIKE_ROOT_INSTALL_NUMERICS) && BIKE_ROOT_INSTALL_NUMERICS
#include "installed_provider.hpp"
#include "../platform/provenance.hpp"
#endif
#include <dlfcn.h>
#include <stdexcept>
#include <string>
#include <vector>
#include <mujoco/mujoco.h>

namespace runtime {

namespace {

// The provider images the qualified build linked against — one loader name
// per line, baked at configure time by the identity declarations.
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

#if defined(__linux__) && defined(BIKE_ROOT_INSTALL_NUMERICS) && BIKE_ROOT_INSTALL_NUMERICS
InstalledProviderIdentity current_provider_identity() {
    InstalledProviderIdentity identity{&build_identity::numerics_provider_policy_json[0], required_provider_images()};
    try {
        // A developer build keeps its original strict image identity.
        (void)loaded_provider_images(identity.required_images);
        return identity;
    } catch (const ProviderImageMismatch&) {
        // Only a missing qualified path permits an installed-layout lookup.
        // Unavailable inspection and allocation failures must propagate.
        auto installed = installed_provider_identity(identity.policy_json, identity.required_images,
            bike::platform::executable_path());
        // Matching bytes on disk are necessary but insufficient: the exact
        // validated adjacent files must also be the images the process loaded.
        (void)loaded_provider_images(installed.required_images);
        return installed;
    }
}
#endif

} // namespace

std::string numerics_provider_policy_json() {
#if defined(__linux__) && defined(BIKE_ROOT_INSTALL_NUMERICS) && BIKE_ROOT_INSTALL_NUMERICS
    return current_provider_identity().policy_json;
#else
    return &build_identity::numerics_provider_policy_json[0];
#endif
}

std::vector<ProviderImage> provider_images() {
#if defined(__linux__) && defined(BIKE_ROOT_INSTALL_NUMERICS) && BIKE_ROOT_INSTALL_NUMERICS
    return loaded_provider_images(current_provider_identity().required_images);
#else
    return loaded_provider_images(required_provider_images());
#endif
}

RuntimeIdentity runtime_identity() {
    // In the supplied MuJoCo engine_support.c, mj_versionString returns a
    // static character array inside the loaded library. dladdr on that
    // DATA address avoids a function-to-object pointer reinterpret_cast.
    const char *version = mj_versionString();
    Dl_info location{};
    if (version == nullptr ||
        dladdr(static_cast<const void *>(version), &location) == 0 ||
        location.dli_fname == nullptr)
        throw std::runtime_error(
            "cannot identify the loaded MuJoCo library");
    // &array[0] produces the char pointer without an implicit array decay.
    return {.mujoco_version = version,
            .mujoco_library_path = location.dli_fname,
            .native_source_sha256 =
                &build_identity::native_source_sha256[0],
            .build_context_json = &build_identity::context_json[0]};
}

} // namespace runtime
