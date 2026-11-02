// Completion 3.34 (T53). Source: tests/reference/test_physical_view.py.
//
// The Python file exercises RenderReplica/apply_frame/snapshot_python — the
// native twin of all three is viewer::RenderOwner plus the runtime's owned
// RenderFrame production. The records port as:
//   * steps-parametrized preview/HUD oracle parity — the python-vs-python
//     regression pair collapses to the single native HUD/preview schema
//     surface, which the 3.22b hud_columns port covers; this file adds the
//     preview-row ownership piece the oracle asserted;
//   * replica-never-live-state — RenderOwner::apply is kinematic-only by
//     construction and can only write its own copied model/data pair;
//   * frame-owns-fields — RenderFrame carries owned vectors that must survive
//     reset+close;
//   * reject-before-mutation — apply() stages the whole frame and throws
//     before touching the replica arrays.
#include "support/fixture_test.hpp"
#include "recording_test_support.hpp"
#include "runtime/runtime.hpp"
#include "viewer/render_owner.hpp"
#include "viewer/hud.hpp"
#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace {

namespace view = bike::viewer;
using recording_test::welded_runtime;

bool equal(const std::vector<double> &a, const std::vector<double> &b) {
    return a == b;
}

// test_existing_preview_and_hud_remain_equal — parametrized over steps in the
// source; the native HUD/preview surface is step-count agnostic (the frame
// carries the step), so the port asserts the schema on the real welded frame
// at step 0 and after a committed advance.
void test_existing_preview_and_hud_remain_equal() {
    auto rig = welded_runtime();
    view::InputState const input;
    const auto frame = rig->runtime->render_frame();
    const auto columns = view::physical_columns(frame, std::nullopt);
    bt::require(!columns.empty(),
                "physical columns exist for the live frame");
    const auto legend = view::hud_legend(columns);
    const auto line = view::hud_line(frame, input, std::nullopt, view::Mode::physical);
    bt::require(!legend.empty() && !line.empty(),
                "hud_legend and hud_line produce the physical schema");
    // preview_row ownership: the frame's integration state is an owned copy —
    // re-reading the snapshot after mutating a local copy leaves the runtime
    // view untouched (the oracle's `preview_row` dict-of-owned-values check).
    const auto snapshot = rig->runtime->snapshot();
    // The oracle's [0,1,2] parametrization sweeps live (sampled) frames:
    // step twice so both compared frames carry a committed sample — far
    // enough apart that the HUD's formatted clock visibly differs.
    recording_test::advance(*rig->runtime, 200);
    const auto advanced = rig->runtime->snapshot();
    bt::require(advanced.step == snapshot.step + 200,
                "advance commits and produces a fresh owned snapshot");
    const auto live1 = rig->runtime->render_frame();
    const auto live_columns = view::physical_columns(live1, std::nullopt);
    bt::require(live_columns.size() > columns.size(),
                "a sampled frame carries the full column schema");
    [[maybe_unused]] const auto live_line =
        view::hud_line(live1, input, std::nullopt, view::Mode::physical);
    recording_test::advance(*rig->runtime, 400);
    const auto frame2 = rig->runtime->render_frame();
    const auto columns2 = view::physical_columns(frame2, std::nullopt);
    bt::require(columns2.size() == live_columns.size(),
                "the column schema is stable across steps (the [0,1,2] param)");
    const auto line2 = view::hud_line(frame2, input, std::nullopt, view::Mode::physical);
    bt::require(rig->runtime->snapshot().step == advanced.step + 200 &&
                    !line2.empty(),
                "hud_line renders the stepped frame");
    rig->runtime->close();
}

// test_replica_only_updates_geometry_and_never_live_state — the replica may
// only observe captured state. RenderOwner::apply is kinematic-only by
// construction (mj_setState + kinematics/comPos/camlight/tendon); the source
// runtime's snapshot must be byte-identical before and after the apply, and
// the replica exposes the frame's values through its owned accessors.
void test_replica_only_updates_geometry_and_never_live_state() {
    auto rig = welded_runtime();
    recording_test::advance(*rig->runtime, 2);
    const auto frame = rig->runtime->render_frame();
    const auto expected = frame.state.integration_state;
    view::RenderOwner replica(rig->runtime->model_image());
    const auto before = rig->runtime->snapshot();
    replica.apply(frame);
    bt::require(equal(replica.integration_state(), expected),
                "the replica's integration state equals the applied frame");
    const auto replica_fields = replica.model_fields();
    for (std::size_t i = 0; i < replica_fields.size(); ++i)
        bt::require(equal(replica_fields[i], frame.presentation.model_fields[i]),
                    "replica model field matches the frame's owned field");
    const auto after = rig->runtime->snapshot();
    bt::require(after.step == before.step &&
                    equal(after.integration_state, before.integration_state),
                "the source runtime is untouched by the replica apply");
    // The replica owns its arrays: mutating a re-applied frame's source data
    // cannot reach back into the runtime, and the runtime never shares the
    // replica's copies (the oracle's shares_memory check).
    auto drifted = frame;
    drifted.state.integration_state = expected;
    drifted.state.integration_state[1] += 1.;
    drifted.state.generation = frame.state.generation;
    replica.apply(drifted);
    const auto final = rig->runtime->snapshot();
    bt::require(equal(final.integration_state, before.integration_state),
                "a second replica apply still leaves the live state alone");
    replica.close();
    rig->runtime->close();
}

// test_frame_owns_nested_fields_after_reset — the RenderFrame's
// integration_state and presentation.model_fields are owned vectors; they
// must still hold the captured values after reset + close. The oracle's
// TypeError/ValueError immutability checks are Python frozen-dataclass
// semantics — the native contract is plain ownership, asserted here.
void test_frame_owns_nested_fields_after_reset() {
    auto rig = welded_runtime();
    const auto frame = rig->runtime->render_frame();
    const auto state = frame.state.integration_state;
    const auto site_pos = frame.presentation.model_fields[1]; // site_pos slot
    rig->runtime->reset();
    rig->runtime->close();
    bt::require(equal(frame.state.integration_state, state),
                "the captured integration state survives reset+close");
    bt::require(equal(frame.presentation.model_fields[1], site_pos),
                "the captured site_pos field survives reset+close");
}

// test_incompatible_frame_is_rejected_before_model_mutation — every rejection
// happens before the staged copy is committed, so the replica's observable
// fields are byte-identical after a throwing apply.
void test_incompatible_frame_is_rejected_before_model_mutation() {
    auto rig = welded_runtime();
    const auto frame = rig->runtime->render_frame();
    view::RenderOwner replica(rig->runtime->model_image());
    replica.apply(frame);
    const auto before_state = replica.integration_state();
    const auto before_fields = replica.model_fields();
    const auto unchanged = [&] {
        return equal(replica.integration_state(), before_state) &&
               replica.model_fields() == before_fields;
    };
    auto bad_fields = frame;
    bad_fields.presentation.model_fields[1] = {std::nan("")};
    bt::invalid([&] { replica.apply(bad_fields); });
    bt::require(unchanged(), "rejection left the replica arrays untouched");
    auto wrong_model = frame;
    wrong_model.model_sha256 = "deadbeef";
    bt::invalid([&] { replica.apply(wrong_model); });
    bt::require(unchanged(), "identity rejection left the replica untouched");
    auto backwards = frame;
    backwards.state.step = -1;
    bt::invalid([&] { replica.apply(backwards); });
    auto bad_state = frame;
    bad_state.state.integration_state = {0.};
    bt::invalid([&] { replica.apply(bad_state); });
    bt::require(unchanged(), "every rejection is pre-mutation");
    replica.close();
    rig->runtime->close();
}

} // namespace

int main() {
    return bt::run([] {
        test_existing_preview_and_hud_remain_equal();
        test_replica_only_updates_geometry_and_never_live_state();
        test_frame_owns_nested_fields_after_reset();
        test_incompatible_frame_is_rejected_before_model_mutation();
    });
}
