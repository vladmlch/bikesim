#include "support/migration_test.hpp"
#include "../../src/runtime/realtime_measurement.hpp"
#include "../../src/runtime/setup_values.hpp"
#include "../../src/io/json.hpp"
#include <array>
#include <limits>

namespace {
using namespace runtime::setupvalue;
namespace m = runtime::measurement;
namespace io = bike::io;
Value measured_run() {
    return Value{Object{
        {"measured_ok", Value{true}}, {"valid_prefix", Value{true}},
        {"identity_unchanged", Value{true}}, {"errors", Value{Array{}}},
        {"physics_steps", integer(24000)}, {"simulated_time_s", real(30.)},
        {"wall_clock_s", real(10.)},
        {"prefix", Value{Object{
            {"steps", integer(24000)}, {"requested_steps", integer(24000)},
            {"reason", text("duration_reached")}, {"model_valid", Value{true}},
            {"numerically_valid", Value{true}}
        }}}
    }};
}
void prefix(Value& run, std::string_view name, Value value) { auto p = field(run, "prefix"); set(p, name, std::move(value)); set(run, "prefix", std::move(p)); }
void reasons(const Value& run, std::initializer_list<std::string_view> names) {
    Array expected; for (auto const name : names) expected.push_back(text(name));
    const auto result = m::steady_state_evidence(run, 30.);
    bt::require(io::canonical_json(field(result, "reasons")) == io::canonical_json(Value{expected}), "classification preserves the exact documented failure reasons and order");
    bt::require(io::as_string(field(result, "status"), "status") == (names.size() == 0 ? "eligible" : "diagnostic"), "only a complete valid measured window is eligible");
}
// tests/portability/test_performance_evidence.py::test_short_invalid_prefix_is_diagnostic_not_steady_state
void test_short_invalid_prefix_is_diagnostic_not_steady_state() {
    auto run = measured_run(); set(run, "simulated_time_s", real(.005)); set(run, "physics_steps", integer(4)); set(run, "valid_prefix", Value{false});
    prefix(run, "steps", integer(4)); prefix(run, "model_valid", Value{false}); prefix(run, "reason", text("invalid_controller"));
    reasons(run, {"invalid_prefix", "incomplete_requested_prefix", "non_duration_terminal", "insufficient_simulated_window"});
}
// tests/portability/test_performance_evidence.py::test_completed_valid_window_is_eligible
void test_completed_valid_window_is_eligible() { reasons(measured_run(), {}); }
// tests/portability/test_performance_evidence.py::test_changed_identity_is_not_a_valid_performance_claim
void test_changed_identity_is_not_a_valid_performance_claim() { auto run = measured_run(); set(run, "identity_unchanged", Value{false}); reasons(run, {"identity_changed"}); }
// tests/portability/test_performance_evidence.py::test_non_duration_terminal_reason_is_not_steady_state
void test_non_duration_terminal_reason_is_not_steady_state() { auto run = measured_run(); prefix(run, "reason", text("finished")); reasons(run, {"non_duration_terminal"}); }
// tests/portability/test_performance_evidence.py::test_invalid_wall_clock_blocks_steady_state
void test_invalid_wall_clock_blocks_steady_state() {
    for (const auto& value : std::array{real(0.), real(-1.), Value{io::Tagged{.kind="float64", .fields={{"value", "nan"}}}}, Value{io::Tagged{.kind="float64", .fields={{"value", "+inf"}}}}}) {
        auto run = measured_run(); set(run, "wall_clock_s", value); reasons(run, {"invalid_wall_clock"});
    }
}
// tests/portability/test_performance_evidence.py::test_measurement_failure_is_diagnostic
void test_measurement_failure_is_diagnostic() {
    for (int mode = 0; mode < 3; ++mode) {
        auto run = measured_run(); set(run, "measured_ok", Value{mode == 1});
        if (mode != 0) set(run, "errors", Value{Array{Value{Object{{"stage", text(mode == 1 ? "advance" : "flush")}, {"type", text("RuntimeError")}, {"message", text("x")}}}}});
        reasons(run, {"measurement_failed"});
    }
}
// tests/portability/test_performance_evidence.py::test_non_finite_or_non_positive_minimum_is_rejected
void test_non_finite_or_non_positive_minimum_is_rejected() {
    for (double minimum : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), 0., -1.})
        bt::invalid([&] { (void)m::steady_state_evidence(measured_run(), minimum); });
}
template<class T> concept MinimumAccepted = requires(const Value& run, T minimum) { m::steady_state_evidence(run, minimum); };
// tests/portability/test_performance_evidence.py::test_bool_minimum_is_rejected
void test_bool_minimum_is_rejected() { bt::require(!MinimumAccepted<bool> && MinimumAccepted<double>, "typed C++ boundary rejects booleans while accepting the binary64 minimum"); }
// tests/portability/test_performance_evidence.py::test_partial_step_prefix_is_incomplete
void test_partial_step_prefix_is_incomplete() { auto run = measured_run(); prefix(run, "steps", integer(12000)); set(run, "physics_steps", integer(12000)); reasons(run, {"incomplete_requested_prefix"}); }
// tests/portability/test_performance_evidence.py::test_committed_steps_must_match_prefix_record
void test_committed_steps_must_match_prefix_record() { auto run = measured_run(); set(run, "physics_steps", integer(23999)); reasons(run, {"incomplete_requested_prefix"}); }
// tests/portability/test_performance_evidence.py::test_zero_requested_steps_is_incomplete
void test_zero_requested_steps_is_incomplete() { auto run = measured_run(); prefix(run, "requested_steps", integer(0)); reasons(run, {"incomplete_requested_prefix"}); }
// tests/portability/test_performance_evidence.py::test_numerically_invalid_prefix_is_diagnostic
void test_numerically_invalid_prefix_is_diagnostic() { auto run = measured_run(); prefix(run, "numerically_valid", Value{false}); reasons(run, {"invalid_prefix"}); }
// tests/portability/test_performance_evidence.py::test_non_finite_simulated_window_is_insufficient
void test_non_finite_simulated_window_is_insufficient() { auto run = measured_run(); set(run, "simulated_time_s", Value{io::Tagged{.kind="float64", .fields={{"value", "nan"}}}}); reasons(run, {"insufficient_simulated_window"}); }
// tests/portability/test_performance_evidence.py::test_window_minimum_has_a_narrow_equality_tolerance
void test_window_minimum_has_a_narrow_equality_tolerance() {
    auto run = measured_run(); set(run, "simulated_time_s", real(30. - 1e-10)); reasons(run, {});
    set(run, "simulated_time_s", real(30. - 1e-6)); reasons(run, {"insufficient_simulated_window"});
}
// tests/portability/test_performance_evidence.py::test_reasons_have_stable_documented_order
void test_reasons_have_stable_documented_order() {
    auto run = measured_run(); set(run, "measured_ok", Value{false}); set(run, "valid_prefix", Value{false}); set(run, "identity_unchanged", Value{false});
    set(run, "physics_steps", integer(4)); set(run, "simulated_time_s", real(.005)); set(run, "wall_clock_s", Value{io::Tagged{.kind="float64", .fields={{"value", "nan"}}}});
    prefix(run, "steps", integer(4)); prefix(run, "reason", text("invalid_controller")); prefix(run, "model_valid", Value{false});
    reasons(run, {"measurement_failed", "invalid_prefix", "identity_changed", "incomplete_requested_prefix", "non_duration_terminal", "insufficient_simulated_window", "invalid_wall_clock"});
}
// tests/portability/test_performance_evidence.py::test_classification_does_not_mutate_the_run
void test_classification_does_not_mutate_the_run() { const auto run = measured_run(); const auto before = io::canonical_json(run); (void)m::steady_state_evidence(run, 30.); bt::require(before == io::canonical_json(run), "classification is observational"); }
} // namespace
// bt::run wraps the test body in try/catch(...) — failures are reported, nothing escapes main.
// cppcheck-suppress throwInEntryPoint
int main() { return bt::run([] {
    test_short_invalid_prefix_is_diagnostic_not_steady_state(); test_completed_valid_window_is_eligible();
    test_changed_identity_is_not_a_valid_performance_claim(); test_non_duration_terminal_reason_is_not_steady_state();
    test_invalid_wall_clock_blocks_steady_state(); test_measurement_failure_is_diagnostic();
    test_non_finite_or_non_positive_minimum_is_rejected(); test_bool_minimum_is_rejected();
    test_partial_step_prefix_is_incomplete(); test_committed_steps_must_match_prefix_record();
    test_zero_requested_steps_is_incomplete(); test_numerically_invalid_prefix_is_diagnostic();
    test_non_finite_simulated_window_is_insufficient(); test_window_minimum_has_a_narrow_equality_tolerance();
    test_reasons_have_stable_documented_order(); test_classification_does_not_mutate_the_run();
}); }
