// Port of tests/reference/test_native_viewer_snapshot.py — the snapshot
// frame is an owned presentation boundary: mutating a render replica or a
// retained frame can never perturb the live environment, and pacing slices
// (target_step) yield without flushing, duplicating requests or completing
// the window. The Python file's cross-oracle parity assertions
// (preview_row/endpoint/integration_state vs the live Python HUD oracle)
// stay planned-red until the viewer goldens land — they are recorded in
// migration/test-port-map.json as the two snapshot reds for this file.
#include "replay_reference_support.hpp"

namespace {
namespace rr = replay_reference;
using rr::field;

[[nodiscard]] bool finite_state(const std::vector<double>& state) {
    return !state.empty() &&
        std::ranges::all_of(state, [](double value) { return std::isfinite(value); });
}

// test_native_preview_matches_reference_and_is_owned — the owned halves:
// three snapshot/step rounds, then a retained frame survives replica
// mutation, reset and close bit-for-bit. Cross-oracle preview/endpoint
// parity is the planned red noted in the port map.
void preview_owned() {
    auto environment = rr::make_environment();
    auto& runtime = *environment->runtime;
    for (int round = 0; round < 3; ++round) {
        const auto frame = runtime.render_frame();
        bt::require(finite_state(frame.state.integration_state),
            "the preview frame carries a finite owned integration state");
        rr::step_control(runtime);
    }
    const auto frame = runtime.render_frame();
    const auto expected = frame.state.integration_state;
    // The render replica is a detached model+data assembled from the
    // published model image; scribbling on its qpos/site rows exercises
    // the same isolation the Python test asserts.
    auto replica_model = bike::model::ModelOwner{
        engine::load_model_image(runtime.model_image().mjb)};
    auto replica_data = engine::make_data(replica_model.get());
    const auto qpos = model_access::mutable_buffer(replica_data->qpos, replica_model->nq);
    for (auto& value : qpos) value += 1.;
    const auto sites = model_access::mutable_buffer(replica_model->site_pos, replica_model->nsite * 3);
    for (auto& value : sites) value = 10.;
    auto scribbled = frame.state.integration_state;
    for (auto& value : scribbled) value = 42.;
    bt::require(std::ranges::equal(runtime.snapshot().integration_state, expected),
        "replica and frame mutation leave the environment untouched");
    runtime.reset();
    runtime.close();
    bt::require(std::ranges::equal(frame.state.integration_state, expected),
        "a retained frame survives reset and close as an owned value");
}

// test_native_target_slice_does_not_flush_or_duplicate_request — a pacing
// yield at target_step=1 leaves the window latched with exactly one
// request, one observation and no delivered transition; a stale target is
// the same non-strict rejection; the completing call publishes exactly one
// transition for the one request.
void target_slice() {
    auto environment = rr::make_environment();
    auto& runtime = *environment->runtime;
    runtime::RideControl command;
    command.motor_torque_nm = 0.;
    command.human_torque_nm = 0.;
    runtime.begin_control(command, 0., 0.);
    bt::require(!runtime.advance_control(std::nullopt, std::int64_t{1}).has_value(),
        "the pacing slice yields without a transition");
    const auto status = rr::research::wire_value(runtime::Wire{runtime.status()});
    bt::require(rr::count(field(status, "step")) == 1 &&
        rr::flag(status, "active") &&
        runtime.commands_requested().size() == 1 &&
        runtime.observations().size() == 1 &&
        runtime.transitions().empty(),
        "one step lands inside the pending window with one request and one observation");
    rr::rejects([&] { (void)runtime.advance_control(std::nullopt, std::int64_t{0}); },
        "target_step is behind the committed step");
    bt::require(rr::flag(rr::research::wire_value(runtime::Wire{runtime.status()}), "active"),
        "the rejected slice leaves the window pending");
    bt::require(runtime.advance_control(std::nullopt, std::nullopt).has_value(),
        "the unbounded advance completes the pending window");
    bt::require(runtime.commands_requested().size() == 1 &&
        runtime.transitions().size() == 1,
        "no request or trace row was duplicated by the pacing yields");
    runtime.acknowledge_control();
    runtime.close();
}
} // namespace
int main() {
    return bt::run([] {
        preview_owned();
        target_slice();
    });
}
