// The pure provenance producer (task T11): mj_versionString/dladdr
// resolution plus the compiled-in Track-B build identity. The extension's
// held-GIL dict boxing is encode_binding.cpp's bind_provenance().
#include "provenance.hpp"
#include "research_build_identity.hpp"
#include <dlfcn.h>
#include <stdexcept>
#include <mujoco/mujoco.h>

namespace runtime {

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
