// Completion 3.33a. Source: tests/reference/test_native_research_capture.py.
// The applied-record cursor protocol under 'verify' capture. The sibling
// off/full live-parity record stays planned (documented baseline drift).
//
// Substitution notes: the Python test injects MemoryError into the boxing
// freeze to prove an un-acked suffix is re-delivered. In the native layer
// prepare_applied_since is a pure const read — a consumer-side "boxing
// failure" cannot lose the un-acked rows by construction. The port pins
// exactly that contract: repeated prepares re-deliver the same suffix until
// acknowledge_applied commits the cursor.
#include "ft2_reference_support.hpp"
#include "research_reference_support.hpp"

namespace {
using namespace research_reference;
namespace sw = runtime::sample_wire;

// tests/reference/test_native_research_capture.py:
//   test_native_verify_boxing_failure_does_not_acknowledge_suffix
void test_native_verify_boxing_failure_does_not_acknowledge_suffix() {
    auto env = environment({.capture = runtime::ResearchCaptureMode::verify});
    // The MemoryError-injected boxing path never acknowledges: two prepares
    // must deliver the identical committed prefix.
    const auto first = env->prepare_applied_since(0);
    const auto second = env->prepare_applied_since(0);
    const auto &rows = sw::array(sw::required(first, "rows"));
    const auto cursor = sw::integer(sw::required(first, "next_cursor"));
    bt::require(rows.size() == 1 && cursor == 1,
                "the startup applied row is delivered once");
    bt::require(wire_equal(runtime::Wire{first}, runtime::Wire{second}),
                "an un-acked prepare re-delivers the same rows (box-fail safe)");
    env->acknowledge_applied(cursor);
    bt::require(env->commands_applied().empty(),
                "verify retention erases the acknowledged prefix");
    const auto empty = env->prepare_applied_since(cursor);
    bt::require(sw::array(sw::required(empty, "rows")).empty() &&
                    sw::integer(sw::required(empty, "next_cursor")) == cursor,
                "the committed cursor yields an empty suffix");
    bt::invalid([&] { (void)env->prepare_applied_since(0); });
    // env.step(RideControl(2.)): the delayed command lands inside the one
    // window (delay_steps 2 < control_steps 4) and publishes a new applied
    // record.
    (void)step(*env, motor_control(2.));
    const auto suffix = env->prepare_applied_since(cursor);
    const auto &tail = sw::array(sw::required(suffix, "rows"));
    const auto end = sw::integer(sw::required(suffix, "next_cursor"));
    bt::require(end == cursor + static_cast<std::int64_t>(tail.size()),
                "the suffix cursor advances by exactly its row count");
    bt::require(!tail.empty(),
                "the changed window publishes at least one applied record");
    env->acknowledge_applied(end);
    bt::require(env->commands_requested().empty() && env->observations().empty() &&
                    env->transitions().empty(),
                "verify retention keeps no stream history");
    const auto last = env->last_requested();
    const auto &control = sw::object(sw::required(sw::object(last), "control"));
    bt::require(sw::number(sw::required(control, "motor_torque_nm")) == 2.,
                "the last requested command carries the submitted motor torque");
}

// tests/reference/test_native_research_capture.py:
//   test_native_off_full_are_passive_and_full_records_every_interval
// The oracle's 'off' env is capture-passive while 'full' keeps every
// interval; both replay the same research_flat commands and their
// transitions/state consume the frozen rows. Two Node cursors over one
// golden node keep per-env consumption honest.
constexpr std::string_view kNode =
    "native_research_capture/";

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_native_off_full_are_passive_and_full_records_every_interval() {
    bt::oracle::Node node_off(
        std::string(kNode) +
        "native_off_full_are_passive_and_full_records_every_interval");
    bt::oracle::Node node_full(
        std::string(kNode) +
        "native_off_full_are_passive_and_full_records_every_interval");
    // The Python _noise seam injects a non-default delivery config —
    // accel bias (.1,-.2,.3), gyro bias .01, dropout .25 — before both
    // envs compose; research_variant re-runs the delivered startup state
    // under the same delivery document.
    const std::array<double, 3> accel_bias{.1, -.2, .3};
    const auto off = ft2_reference::research_variant(
        "research_flat", runtime::ResearchCaptureMode::off, 3, std::nullopt,
        std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt,
        std::nullopt, accel_bias, .01, .25);
    const auto full = ft2_reference::research_variant(
        "research_flat", runtime::ResearchCaptureMode::full, 3, std::nullopt,
        std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt,
        std::nullopt, accel_bias, .01, .25);
    const std::array<double, 3> torques{0., 2., 1.};
    char label[40]; // NOLINT(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) bounded label buffer for snprintf
    for (std::size_t index = 0; index < torques.size(); ++index) {
        const auto control = motor_control(torques[index]);
        std::snprintf(label, sizeof(label), "step.%zu.transition", index); // NOLINT(cert-err33-c,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-pro-type-vararg) snprintf truncation return intentionally unused on the sized label buffer; label buffer decays into snprintf/sizeof by design; bounded snprintf is the sized-label API
        ft2_reference::consume_tree(node_off, step(*off, control).as_wire(),
                                    label); // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay) label buffer decays into snprintf/sizeof by design
        ft2_reference::consume_tree(node_full, step(*full, control).as_wire(),
                                    label); // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay) label buffer decays into snprintf/sizeof by design
        std::snprintf(label, sizeof(label), "step.%zu.integration_state", // NOLINT(cert-err33-c,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-pro-type-vararg) snprintf truncation return intentionally unused on the sized label buffer; label buffer decays into snprintf/sizeof by design; bounded snprintf is the sized-label API
                      index);
        const std::vector<double> off_state = off->snapshot().integration_state;
        node_off.f64(off_state, label); // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay) label buffer decays into snprintf/sizeof by design
        const std::vector<double> full_state =
            full->snapshot().integration_state;
        node_full.f64(full_state, label); // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay) label buffer decays into snprintf/sizeof by design
    }
    bt::require(off->commands_requested().empty() &&
                    off->commands_applied().empty() &&
                    off->transitions().empty() && off->observations().empty(),
                "off capture keeps no stream history");
    const auto committed = full->snapshot().step;
    const auto episode = full->seal_episode();
    bt::require(episode.transitions.size() == 3,
                "the full episode completes three control intervals");
    // recorder.rows == sim.steps on the oracle: the committed extent is
    // what 'full' seals; the retained row cadence is the decimation.
    bt::require(committed % 3 == 0 &&
                    episode.physical.samples.size() ==
                        static_cast<std::size_t>(committed / 3) &&
                    episode.physical.samples.back()->interval_id ==
                        committed - 3,
                "full records every committed interval on the decimated "
                "cadence");
    bt::require(episode.physical.sample_batch(0, 2).columns.empty(),
                "sample_batch publishes no explicit flat-column view");
    const auto held =
        episode.physical.sample_batch(0, 1).samples[0]->qpos;
    node_off.f64(held, "episode.qpos_row1");
    node_full.f64(held, "episode.qpos_row1");
    full->reset();
    full->close();
    bt::require(episode.physical.sample_batch(0, 1).samples[0]->qpos == held,
                "sealed episode rows outlive reset+close of the runtime");
    bt::require(episode.row_count("observations") == 4,
                "the startup plus three step observations seal into history");
    node_off.done();
    node_full.done();
}
NATIVE_DIAG_POP

} // namespace

int main() {
    return bt::run([] {
        test_native_verify_boxing_failure_does_not_acknowledge_suffix();
        test_native_off_full_are_passive_and_full_records_every_interval();
    });
}
