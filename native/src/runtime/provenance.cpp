#include "provenance.hpp"
#include "research_build_identity.hpp"
#include <dlfcn.h>
#include <stdexcept>
#include <mujoco/mujoco.h>
#include <nanobind/stl/string.h>

namespace runtime {
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
        return result;
    });
}
} // namespace runtime
