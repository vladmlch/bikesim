#pragma once

// Internal helpers shared between the hot-kernel SteerBalance TU and its
// cold io face (rider_io/steer_profile.cpp). The fingerprint needs
// io::canonical_json + io::sha256, which the kernel must not include, so it
// is defined on the io side while the pure profile invariant stays kernel
// side. This header is not part of the frozen public contract.
#include "steer_balance.hpp"

#include <string>

namespace bike::rider::detail {
void validate_steer_profile(const SteerProfile& profile);
std::string steer_profile_fingerprint(const SteerProfile& profile);
} // namespace bike::rider::detail
