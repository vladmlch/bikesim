#pragma once
#include "runtime_access.hpp"
#include "rigs.hpp"

namespace bike::validation {
inline constexpr std::string_view motor_only_unsupported="motor-only open-loop replay is unsupported by the pedelec gate; use rider-assisted ceiling commands or the declared airborne momentum stand";
struct Simulation {
    initialization::SetupRequest request;
    initialization::EquilibriumResult equilibrium;
    Value metadata;
    std::unique_ptr<runtime::NativeRideRuntime> runtime;
    explicit Simulation(initialization::SetupRequest input);
    [[nodiscard]] initialization::PhysicalInitialState initial_state(std::optional<int> stations={});
};
[[nodiscard]] initialization::SetupRequest profile_request(const std::filesystem::path& physics,const std::filesystem::path& track,std::optional<double> dt={},const Value& overrides=Value{Object{}});
[[nodiscard]] runtime::RideControl resume_schedule(double time);
[[nodiscard]] runtime::RideControl automatic_schedule(double time);
[[nodiscard]] runtime::RideControl open_loop_schedule(double time);
[[nodiscard]] Value diagnostic_row(const runtime::PhysicalSampleData&);
[[nodiscard]] Value interval_row(const runtime::PhysicalSampleData&,const runtime::TruthGeometry&,runtime::WheelieTracker&);
[[nodiscard]] Value run_replay(const Value& configuration,const std::filesystem::path& root);
[[nodiscard]] Value rider_factor_matrix(const Value& configuration,const std::filesystem::path& root);
[[nodiscard]] Value automatic_coast_case(double dt,double duration,const std::filesystem::path& root);
[[nodiscard]] RigResult ride_case(std::string_view name,double dt);
[[nodiscard]] Value fidelity_sweep(const Value& configuration,const std::filesystem::path& root);
[[nodiscard]] Value reference_matrix(const Value& configuration,const std::filesystem::path& root);
[[nodiscard]] Value climbing_benchmark(const Value& configuration,const std::filesystem::path& root);
[[nodiscard]] std::span<const std::string_view> benchmark_case_names();
[[nodiscard]] RigResult benchmark_case(std::string_view name,double dt);
[[nodiscard]] Value profile_identity(const std::filesystem::path& physics,const std::filesystem::path& root);
[[nodiscard]] bool evidence_matches(const Value& record,const Value& expected);
[[nodiscard]] Value benchmark_suite(const Value& configuration,const std::filesystem::path& root);
[[nodiscard]] Value contact_resolution(const Value& configuration,const std::filesystem::path& root);
[[nodiscard]] Value mechanics_report(const Value& configuration,const std::filesystem::path& root);
} // namespace bike::validation
