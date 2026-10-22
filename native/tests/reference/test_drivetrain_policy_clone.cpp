// Completion 3.18; source: tests/reference/test_drivetrain_policy_clone.py.
// Python vars()/deepcopy identity has no native analogue. Exercise the owned
// policy copy with independent state mutations and continuation instead.
#include "policy_reference_support.hpp"

namespace {
void test_clone_matches_deepcopy_and_shares_no_mutable_state() {
    auto physics = std::make_unique<bike::configuration::SimulationPhysicsConfig>(
        bike::configuration::load_physics(bt::fixture("config/viewer_physics_welded.toml")));
    const auto projected = drive_reference::projected_writer(physics->drive);
    auto decoded = std::make_unique<nativecfg::WriterConfig>(nativecfg::decode(projected));
    bt::require(decoded->drive.has_value(), "pinned source profile has a physical drive");
    auto original = std::make_unique<drivetrain::PolicySet>(decoded->drive->policies); // NOLINT(bugprone-unchecked-optional-access) optional engagement is guaranteed by the fixture/request contract.
    auto cloned = std::make_unique<drivetrain::PolicySet>(*original);
    const auto before = policy_reference::state_text(*cloned);

    // The source's hub parameter skips: ideal_mid_drive uses a model-owned
    // transmission, not the elastic policy. It is documented as a drop,
    // not counted as a successfully ported mutable-policy assertion.
    bt::require(decoded->drive->transmission_model == "ideal_mid_drive", // NOLINT(bugprone-unchecked-optional-access) optional engagement is guaranteed by the fixture/request contract.
        "the clone fixture retains the source profile's absent elastic hub");
    bt::require(original->config.shifting.cassette.data() != cloned->config.shifting.cassette.data(),
        "shifting configuration owns its cassette after copying");
    bt::require(original->config.assist.torque_curve.has_value() && cloned->config.assist.torque_curve.has_value(),
        "pinned assist profile contains the source's measured torque curve");
    bt::require(original->config.assist.torque_curve->data() != cloned->config.assist.torque_curve->data(), // NOLINT(bugprone-unchecked-optional-access) optional engagement is guaranteed by the fixture/request contract.
        "assist configuration owns its torque curve after copying");

    (void)original->shifting.update(150., 150., .1);
    auto pedal = original->pedaling.state();
    pedal.effort = 7.;
    original->pedaling.set_state(pedal);
    (void)original->assist.step(20., 80., 2., false, .01);
    bt::require(policy_reference::state_text(*cloned) == before,
        "changing any source policy leaves the cloned mutable state intact");

    // The copied policies still contain the original configuration after
    // their owner and every caller-side configuration document are gone.
    original.reset(); decoded.reset(); physics.reset();
    auto independent = std::make_unique<drivetrain::PolicySet>(*cloned);
    (void)cloned->shifting.update(150., 150., .1);
    (void)independent->shifting.update(150., 150., .1);
    drive_reference::near(cloned->assist.step(20., 80., 2., false, .01),
        independent->assist.step(20., 80., 2., false, .01), "copied assist continuation");
    auto changed = cloned->pedaling.state(); changed.effort = 9.;
    cloned->pedaling.set_state(changed);
    bt::require(independent->pedaling.state().effort != 9.,
        "copied pedaling state remains independent in both directions");
    bt::require(cloned->shifting.state().rear_teeth == independent->shifting.state().rear_teeth,
        "copied shifting policies preserve configuration and continuation");
}
} // namespace
int main() { return bt::run(test_clone_matches_deepcopy_and_shares_no_mutable_state); }
