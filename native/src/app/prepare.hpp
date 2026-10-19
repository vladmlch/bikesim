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
// SetupRequest and PreparedSimulation are each several KB; the heap-owning
// overloads keep the value objects out of strict caller frames.
[[nodiscard]] std::unique_ptr<PreparedSimulation> prepare(std::unique_ptr<SetupRequest> request);
[[nodiscard]] std::unique_ptr<runtime::NativeRideRuntime> make_ride(PreparedSimulation prepared);
[[nodiscard]] std::unique_ptr<runtime::NativeRideRuntime> make_ride(std::unique_ptr<PreparedSimulation> prepared);
} // namespace bike::app
