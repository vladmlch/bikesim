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
#include "research_reference_support.hpp"

namespace {
using namespace research_reference;
namespace sw = runtime::sample_wire;

// tests/reference/test_native_research_capture.py::
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

} // namespace

int main() {
    return bt::run([] {
        test_native_verify_boxing_failure_does_not_acknowledge_suffix();
    });
}
