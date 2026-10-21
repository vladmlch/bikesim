#pragma once
#include "common.hpp"
#include <optional>

namespace bike::validation {
struct Verdict { bool accepted = false; Value evidence; };
struct CaseDescription {
    std::string_view id;
    std::string_view source;
    std::string_view description;
    bool slow = false;
};
[[nodiscard]] std::span<const CaseDescription> cases();
[[nodiscard]] Verdict run_case(std::string_view case_id, const std::filesystem::path& configuration);
[[nodiscard]] Verdict run_case(std::string_view case_id, const Value& configuration,
                              const std::filesystem::path& base_directory = {});
// Reporting never promotes a rig's aggregate acceptance into a physical,
// numerical-parity, or timing claim. Missing evidence stays null.
[[nodiscard]] Value report_axes(const Value& evidence);
[[nodiscard]] Value check_bounds(const Value& metrics,const Value& bounds);
[[nodiscard]] Value release_decision(const Value& evidence,bool require_completion);
[[nodiscard]] Value assess_run(const Value& summary,double required_duration_s,double minimum_progress_m=0.);
[[nodiscard]] Value paired_summary(const Value& records);
[[nodiscard]] std::pair<bool,std::vector<std::string>> release_gates(const Value& report);
[[nodiscard]] std::string release_level(const Value& report);
[[nodiscard]] Value select_verified_resolution(const Value& comparisons);
[[nodiscard]] Value write_release_manifest(const std::filesystem::path& path,const Value& report,
    const std::filesystem::path& physics_profile,const Value& measurement_evidence=Value{Array{}});
[[nodiscard]] Value classify_outcome(const Value& rows,std::optional<double> goal_x_m={},
    bool crashed=false,std::optional<std::string> numerical_error={},bool duration_reached=false);
[[nodiscard]] Value scenario_definitions();
[[nodiscard]] Value environment_contract(const std::filesystem::path& project_directory);
[[nodiscard]] std::string source_fingerprint(const std::filesystem::path& root);
} // namespace bike::validation
