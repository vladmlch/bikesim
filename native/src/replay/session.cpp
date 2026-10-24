#include "session.hpp"
#include "rebuild.hpp"
#include "input_detail.hpp"
#include "../research/evidence.hpp"
#include "../platform/provenance.hpp"
#include "../rider/state_codec.hpp"
#include "../runtime/spatial_checkpoint_internal.hpp"
#include <algorithm>
#include <set>
#include <utility>
#include <format>

namespace bike::replay {
IncompatibleExecution::~IncompatibleExecution() = default;
namespace {
using namespace detail;
class MismatchFailure final : public recording::RecordingError {
public:
    explicit MismatchFailure(Comparison value) : RecordingError(value.first_path + ": " + value.reason), comparison(std::move(value)) {}
    ~MismatchFailure() override;
    MismatchFailure(const MismatchFailure&) = default;
    MismatchFailure& operator=(const MismatchFailure&) = default;
    MismatchFailure(MismatchFailure&&) = default;
    MismatchFailure& operator=(MismatchFailure&&) = default;
    Comparison comparison;
};
MismatchFailure::~MismatchFailure() = default;
// NOLINTNEXTLINE(misc-no-recursion) recursion mirrors the Python reference structure
void compare_tree(const Value& actual, const Value& expected, const std::string& path, Comparison& result) {
    const auto fail = [&](std::string why) {
        if (result.matched) { result.matched = false; result.first_path = path; result.reason = std::move(why); }
    };
    if (const auto* object = std::get_if<Object>(&expected.data)) {
        const auto* current = std::get_if<Object>(&actual.data);
        if (!current || keys(actual) != keys(expected)) { fail("different fields"); return; }
        for (const auto& [key, value] : *object) compare_tree(field(*current, key), value, std::format("{}.{}", path, key), result);
    } else if (const auto* array = std::get_if<Array>(&expected.data)) {
        const auto* current = std::get_if<Array>(&actual.data);
        if (!current || current->size() != array->size()) { fail("different array length"); return; }
        for (std::size_t i = 0; i < array->size(); ++i) compare_tree((*current)[i], (*array)[i], path + '[' + std::to_string(i) + ']', result);
    } else if (std::holds_alternative<io::Number>(expected.data)) {
        if (!std::holds_alternative<io::Number>(actual.data)) { fail("numerical replay mismatch: different type"); return; }
        const auto& expected_number=std::get<io::Number>(expected.data);
        const auto& actual_number=std::get<io::Number>(actual.data);
        if(io::is_integer_lexeme(expected_number.lexeme)) {
            if(!io::is_integer_lexeme(actual_number.lexeme) || expected_number.lexeme!=actual_number.lexeme)fail("integer/schema replay mismatch");
            return;
        }
        const auto a = io::as_double(actual, path), e = io::as_double(expected, path);
        const auto error = std::abs(a - e); result.max_absolute_error = std::max(result.max_absolute_error, error);
        const auto tolerance=path.starts_with("checkpoint_final.") ? 1e-9+1e-9*std::abs(e)
            : std::max(1e-9, 1e-9 * std::max(std::abs(a), std::abs(e)));
        if (!std::isfinite(error) || error > tolerance)
            fail("numerical replay mismatch (" + io::recording_json(actual) + " vs " + io::recording_json(expected) + ')');
    } else if (std::holds_alternative<io::Tagged>(expected.data) || std::holds_alternative<io::Tagged>(actual.data)) {
        fail("nonfinite/tagged evidence cannot certify finite checked replay");
    } else if (actual.data.index() != expected.data.index() || !equal(actual, expected)) fail("replay mismatch");
}
void require_evidence(const Value& actual, const Value& expected, const std::string& path, double& maximum) {
    const auto compared = compare_evidence(actual, expected, path);
    maximum = std::max(maximum, compared.max_absolute_error);
    if (!compared.matched) throw MismatchFailure(compared);
}
double require_state(std::span<const double> actual, std::span<const double> expected, const std::string& label) {
    if (actual.size() != expected.size()) throw MismatchFailure({.matched=false, .first_path=label, .reason="incompatible state shape", .max_absolute_error=0.});
    double maximum = 0.; std::optional<std::size_t> first;
    for (std::size_t i = 0; i < actual.size(); ++i) {
        if (!std::isfinite(actual[i]) || !std::isfinite(expected[i])) throw MismatchFailure({.matched=false, .first_path=label + '[' + std::to_string(i) + ']', .reason="nonfinite state", .max_absolute_error=maximum});
        const double error = std::abs(actual[i] - expected[i]); maximum = std::max(maximum, error);
        // numpy.allclose's asymmetric state rule, kept distinct from the
        // recursive evidence comparator's math.isclose rule.
        if (error > 1e-9 + 1e-9 * std::abs(expected[i]) && !first) first = i;
    }
    if (first) throw MismatchFailure({.matched=false, .first_path=label + '[' + std::to_string(*first) + ']', .reason="integration state differs", .max_absolute_error=maximum});
    return maximum;
}
Value status(const runtime::NativeResearchRuntime& environment) { return research::wire_value(runtime::Wire{environment.status()}); }
} // namespace
Comparison compare_evidence(const io::Value& actual, const io::Value& expected, const std::string& path) {
    Comparison result; compare_tree(actual, expected, path, result); return result;
}

struct ReplaySession::Impl {
    std::filesystem::path directory;
    std::optional<std::int64_t> episode;
    ReplayInput input;
    std::unique_ptr<FixedAssets> assets;
    std::unique_ptr<runtime::NativeResearchRuntime> environment;
    std::vector<std::byte> model;
    runtime::RenderFrame frame, verified;
    std::size_t command = 0;
    std::int64_t applied = 0;
    int generation = 0;
    double max_error = 0.;
    bool completed = false, closed = false;
    std::optional<std::string> mismatch;
    std::optional<Comparison> first_mismatch;
    std::optional<std::int64_t> failure_step;
    Value result;
    std::string replay_executable_sha256;

    Impl(std::filesystem::path path, std::optional<std::int64_t> selected, int next_generation = 0)
        : directory(std::move(path)), episode(selected), input(load_replay_input(directory, episode)), verified(frame), generation(next_generation) {
        check_execution(input);
        replay_executable_sha256 = platform::build_identity().executable_sha256;
        assets = std::make_unique<FixedAssets>(input);
        start();
        (void)require_state(environment->snapshot().integration_state, input.initial_state, "initial integration state");
        require_evidence(field(status(*environment), "observation"), input.observations.at(0), "initial observation", max_error);
        frame = environment->render_frame(); frame.state.generation = generation; 
    }
    // The request and prepared environment each exceed the strict stack bound;
    // both stay heap-owned so this frame only carries a pair of unique_ptrs.
    void start() {
        auto prepared = research::prepare_environment(request_from_input(input, *assets));
        verify_configuration(input, *prepared);
        model = prepared->bootstrap.ride.model.mjb;
        environment = runtime::make_research(std::move(prepared->bootstrap));
        if(input.spatial_recipe)environment->restore_checkpoint(input.spatial_recipe->checkpoint_initial);
    }
    void require_open() const { if (closed) throw std::runtime_error("replay session is closed"); }
    void applied_prefix() {
        const auto prefix = research::wire_value(runtime::Wire{environment->prepare_applied_since(applied)});
        const auto cursor = count(field(prefix, "next_cursor")); const auto& rows = io::as_array(field(prefix, "rows"), "applied rows");
        if (cursor < applied || std::cmp_not_equal(cursor - applied, rows.size()) || std::cmp_greater(cursor, input.applied.size()))
            throw recording::RecordingError("applied commands: replay produced extra events or an invalid cursor");
        for (std::size_t i = 0; i < rows.size(); ++i)
            require_evidence(rows[i], input.applied.at(static_cast<std::size_t>(applied) + i), "applied commands[" + std::to_string(applied + static_cast<std::int64_t>(i)) + ']', max_error);
        environment->acknowledge_applied(cursor); applied = cursor;
    }
    void capture_verified() { frame = environment->render_frame(); frame.state.generation = generation; verified = frame; }
    void finish() {
        applied_prefix();
        if (std::cmp_not_equal(applied, input.applied.size())) throw recording::RecordingError("applied commands: replay omitted recorded events");
        auto expected_reason = field(field(input.summary, "outcome"), "reason");
        if (const auto* reason = std::get_if<std::string>(&expected_reason.data); reason && *reason == "not_finished") expected_reason = Value{};
        auto current = status(*environment);
        if (const auto* reason = std::get_if<std::string>(&expected_reason.data); reason && *reason == "operator_stop" &&
            !io::as_bool(field(current, "terminated"), "terminated") && !io::as_bool(field(current, "truncated"), "truncated")) {
            environment->stop(); current = status(*environment);
        }
        require_evidence(field(current, "reason"), expected_reason, "outcome.reason", max_error);
        auto evidence = environment->evidence_snapshot();
        const auto& saved_research = field(input.summary, "research");
        if (const auto* metadata = find(saved_research, "run_metadata"))
            if (const auto* intervention = find(*metadata, "operator_intervention"))
                evidence.run_metadata = Value{Object{{"operator_intervention", *intervention}}};
        const auto summary = research::physical_summary(evidence);
        auto expected_outcome = field(input.summary, "outcome"); set(expected_outcome, "reason", expected_reason);
        require_evidence(field(summary, "outcome"), expected_outcome, "outcome", max_error);
        require_evidence(field(current, "metrics"), field(saved_research, "metrics"), "event metrics", max_error);
        for (auto key : {"terminated", "truncated", "numerically_valid", "max_energy_residual_ratio"})
            if (const auto* value = find(saved_research, key)) require_evidence(field(current, key), *value, std::string("research.") + key, max_error);
        if (const auto* saved_status = find(saved_research, "model_status")) require_evidence(field(summary, "model_status"), *saved_status, "model status", max_error);
        if (!io::is_null(input.metrics)) require_evidence(research::episode_metrics(evidence), input.metrics, "episode metrics", max_error);
        const auto final_error = require_state(environment->snapshot().integration_state, input.final_state, "final integration state");
        if(input.spatial_recipe)require_evidence(runtime::spatial::checkpoint_comparison_value(evidence.checkpoint_final),
            runtime::spatial::checkpoint_comparison_value(input.spatial_recipe->checkpoint_final),"checkpoint_final",max_error);
        check_execution(input);
        if (platform::build_identity().executable_sha256 != replay_executable_sha256)
            throw IncompatibleExecution("replay execution image changed during playback");
        const bool numerical = io::as_bool(field(current, "numerically_valid"), "numerically_valid");
        const bool model_valid = io::as_bool(field(current, "model_valid"), "model_valid");
        result = Value{Object{{"passed", Value{true}}, {"backend", text("native")}, {"episode_id", integer(input.episode_id)},
            {"container_schema_version", input.container_schema_version ? integer(*input.container_schema_version) : Value{}},
            {"replay_contract_version", integer(input.replay_contract_version)}, {"replayed_control_steps", integer(static_cast<std::int64_t>(command))},
            {"physics_steps", integer(evidence.final_snapshot.step)}, {"final_state_max_abs_error", real(final_error)}, {"evidence_max_abs_error", real(max_error)},
            {"outcome", field(current, "reason")}, {"numerically_valid", Value{numerical}}, {"model_valid", Value{model_valid}},
            {"source_sha256", field(input.summary, "model_source_sha256")}, {"execution_profile", text("native_executable")},
            {"recorded_executable_sha256", field(field(input.manifest, "execution"), "executable_sha256")},
            {"replay_executable_sha256", text(replay_executable_sha256)},
            {"scope", text("Deterministic software replay under the recorded numerical execution identity, not physical calibration.")}}};
        capture_verified(); completed = true;
    }
};

ReplaySession::ReplaySession(const std::filesystem::path& path, std::optional<std::int64_t> episode) : impl_(std::make_unique<Impl>(path, episode)) {}
ReplaySession::~ReplaySession() = default;
bool ReplaySession::done() const noexcept { return impl_->completed; }
bool ReplaySession::pending() const {
    if (impl_->closed) return false;
    const auto current = status(*impl_->environment);
    return io::as_bool(field(current, "active"), "replay pending") || io::as_bool(field(current, "delivery_pending"), "replay delivery pending");
}
const runtime::RuntimeSnapshot& ReplaySession::snapshot() const { impl_->require_open(); return impl_->frame.state; }
const runtime::RenderFrame& ReplaySession::render_frame() const { impl_->require_open(); return impl_->frame; }
std::optional<std::string> ReplaySession::error() const { return impl_->mismatch; }
const std::vector<std::byte>& ReplaySession::model_image() const { impl_->require_open(); return impl_->model; }
double ReplaySession::timestep_s() const { impl_->require_open(); return impl_->environment->configuration().timestep_s; }
Value ReplaySession::report() const {
    if (!impl_->completed || io::is_null(impl_->result)) throw std::runtime_error("incomplete replay: final verification has not passed");
    return impl_->result;
}
Value ReplaySession::diagnostics() const {
    const auto& p = *impl_;
    if (p.completed) return p.result;
    return Value{Object{{"passed", Value{false}}, {"status", text(p.mismatch ? "mismatch" : p.closed ? "closed-incomplete" : "incomplete")},
        {"reason", p.mismatch ? text(*p.mismatch) : text("final verification has not passed")},
        {"episode_id", integer(p.input.episode_id)}, {"replayed_control_steps", integer(static_cast<std::int64_t>(p.command))},
        {"last_verified_step", integer(p.verified.state.step)}, {"first_mismatch_step", p.failure_step ? integer(*p.failure_step) : Value{}},
        {"first_mismatch_field", p.first_mismatch ? text(p.first_mismatch->first_path) : Value{}},
        {"maximum_absolute_error", real(p.first_mismatch ? std::max(p.max_error, p.first_mismatch->max_absolute_error) : p.max_error)}}};
}
bool ReplaySession::advance(std::optional<double> wall_budget, std::optional<std::int64_t> target) {
    auto& p = *impl_; p.require_open();
    if (wall_budget && (!std::isfinite(*wall_budget) || *wall_budget < 0.)) throw std::invalid_argument("wall_budget_s must be finite and nonnegative");
    if (target && *target < p.frame.state.step) throw std::invalid_argument("target_step must be at or after the committed step");
    if (p.mismatch) throw std::runtime_error("replay stopped after mismatch: " + *p.mismatch);
    if (p.completed) return true;
    try {
        if (p.command == p.input.requested.size()) { p.finish(); return true; }
        if (!pending()) {
            const auto current = status(*p.environment); const auto& command = p.input.requested[p.command];
            if (io::as_bool(field(current, "terminated"), "terminated") || io::as_bool(field(current, "truncated"), "truncated") ||
                count(field(command, "step")) != p.environment->snapshot().step)
                throw recording::RecordingError("command " + std::to_string(p.command) + ": episode ended early or step does not align");
            require_evidence(field(current, "time_s"), field(command, "time_s"), "command time", p.max_error);
            const auto control = ::rider::decode_intent_control(field(command, "control"), "replay.control");
            p.environment->begin_control(control, io::as_double(field(command, "front_brake_demand"), "front brake"),
                io::as_double(field(command, "rear_brake_demand"), "rear brake"));
            require_evidence(research::wire_value(p.environment->last_requested()), command, "command request", p.max_error);
        }
        const auto transition = p.environment->advance_control(wall_budget, target);
        if (!transition) { p.frame = p.environment->render_frame(); p.frame.state.generation = p.generation; return false; }
        require_evidence(research::wire_value(runtime::Wire{transition->as_wire()}), p.input.transitions[p.command], "transition " + std::to_string(p.command), p.max_error);
        require_evidence(field(status(*p.environment), "observation"), p.input.observations[p.command + 1], "observation " + std::to_string(p.command + 1), p.max_error);
        p.applied_prefix();
        p.environment->acknowledge_control(); ++p.command;
        if (p.command == p.input.requested.size()) p.finish(); else p.capture_verified();
        return p.completed;
    } catch (const std::exception& error) {
        p.mismatch = error.what();
        p.failure_step = p.frame.state.step;
        // NOLINTNEXTLINE(bugprone-empty-catch) deliberate: the result is advisory and the failure path is handled separately
        try { p.failure_step = p.environment->snapshot().step; } catch (...) { /* Keep the last safely copied boundary. */ }
        if (const auto* compared = dynamic_cast<const MismatchFailure*>(&error)) p.first_mismatch = compared->comparison;
        // A verified-boundary rollback can move time backwards after a UI
        // budget yield. Mark a new presentation generation so render owners
        // never accept a same-generation temporal rewind.
        if (p.generation < std::numeric_limits<int>::max()) ++p.generation;
        p.frame = p.verified; p.frame.state.generation = p.generation; throw;
    }
}
void ReplaySession::restart() {
    impl_->require_open();
    // Reserve one generation for a possible mismatch rollback.
    if (impl_->generation >= std::numeric_limits<int>::max() - 1) throw std::overflow_error("replay generation exhausted");
    auto next = std::make_unique<Impl>(impl_->directory, impl_->episode, impl_->generation + 1);
    if (!equal(next->input.manifest, impl_->input.manifest)) throw recording::RecordingError("recording changed since replay session construction");
    impl_->environment->close(); impl_ = std::move(next);
}
void ReplaySession::close() {
    if (!impl_->closed) { impl_->environment->close(); impl_->closed = true; }
}
ReplayResult check(const std::filesystem::path& path, std::int64_t episode) {
    try {
        ReplaySession session(path, episode);
        try { while (!session.advance()) {} }
        catch (const std::exception& error) {
            auto details = session.diagnostics();
            if (dynamic_cast<const IncompatibleExecution*>(&error)) set(details, "status", text("incompatible-execution"));
            // NOLINTNEXTLINE(bugprone-empty-catch) deliberate: the result is advisory and the failure path is handled separately
            try { session.close(); } catch (...) { /* Preserve the numerical mismatch as primary. */ }
            return {.matched=false, .physically_valid=false, .reason=error.what(), .details=std::move(details)};
        }
        auto details = session.report();
        const bool valid = io::as_bool(field(details, "numerically_valid"), "numerically_valid") && io::as_bool(field(details, "model_valid"), "model_valid");
        session.close(); return {.matched=true, .physically_valid=valid, .reason=valid ? "matched" : "matched invalid numerical/model evidence", .details=std::move(details)};
    } catch (const IncompatibleExecution& error) {
        return {.matched=false, .physically_valid=false, .reason=error.what(), .details=Value{Object{{"passed", Value{false}}, {"status", text("incompatible-execution")},
            {"reason", text(error.what())}, {"stored_state_playback_available", Value{true}}}}};
    } catch (const MismatchFailure& error) {
        return {.matched=false, .physically_valid=false, .reason=error.what(), .details=Value{Object{{"passed", Value{false}}, {"status", text("mismatch")}, {"reason", text(error.what())},
            {"first_mismatch_step", integer(0)}, {"first_mismatch_field", text(error.comparison.first_path)},
            {"maximum_absolute_error", real(error.comparison.max_absolute_error)}}}};
    } catch (const std::exception& error) {
        return {.matched=false, .physically_valid=false, .reason=error.what(), .details=Value{Object{{"passed", Value{false}}, {"status", text("mismatch-or-invalid-input")}, {"reason", text(error.what())}}}};
    }
}
} // namespace bike::replay
