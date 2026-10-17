#pragma once

#include <filesystem>
#include <memory>
#include "setup.hpp"
#include "../runtime/runtime.hpp"

namespace bike::app {
struct PreparedSimulation {
    runtime::RuntimeBootstrap bootstrap;
    io::Value metadata;
};
[[nodiscard]] PreparedSimulation prepare(const std::filesystem::path& request_toml);
[[nodiscard]] PreparedSimulation prepare(SetupRequest request);
[[nodiscard]] std::unique_ptr<runtime::NativeRideRuntime> make_ride(PreparedSimulation prepared);
} // namespace bike::app
