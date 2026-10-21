// Loaded numerics-provider inspection, shared by the runtime identity
// (provenance.cpp) and the configure-time identity probe
// (native/cmake/provider_identity_probe.cpp). Both must report the same
// images and the same LC_UUIDs, so the discovery code exists once, in
// provider_images.cpp, and the probe is compiled from that same TU.
#pragma once

#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace runtime {

// A qualified path is absent from the actual loader image set. Distinct
// from unavailable inspection, filesystem failures and allocation errors,
// so the root install fallback can catch only this expected mismatch.
class ProviderImageMismatch : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
    ~ProviderImageMismatch() override;
    ProviderImageMismatch(const ProviderImageMismatch&) = default;
    ProviderImageMismatch& operator=(const ProviderImageMismatch&) = default;
    ProviderImageMismatch(ProviderImageMismatch&&) = default;
    ProviderImageMismatch& operator=(ProviderImageMismatch&&) = default;
};

// One loaded numerics-provider image the qualified build requires.
// macho_uuid is the LC_UUID on Mach-O hosts; ELF hosts identify an image by
// its canonical path alone and report no uuid.
struct ProviderImage {
    std::string path;
    std::optional<std::string> macho_uuid;
};

// The provider images loaded into this process, checked against `required`
// (loader names as the qualified build records them). Loader inspection:
// identity boundaries only, never inside a step loop. Throws
// std::runtime_error when a required image is not loaded or the platform
// cannot inspect loaded images.
[[nodiscard]] std::vector<ProviderImage>
loaded_provider_images(const std::vector<std::string> &required);

} // namespace runtime
