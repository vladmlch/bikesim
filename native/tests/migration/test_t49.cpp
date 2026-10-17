// T49 controls, pacing, camera and HUD contracts. No window is opened by this
// test body; GUI interaction evidence remains a separate unrun acceptance gate.
#include "support/migration_test.hpp"
#include "viewer/pacer.hpp"
#include "viewer/input.hpp"
#include "viewer/render_owner.hpp"
#include "viewer/hud.hpp"
#include <chrono>
#include <cmath>
#include <limits>

namespace {
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
namespace view = bike::viewer;

void pacing() {
    const Clock::time_point zero{};
    view::Pacer pacing(.125, .25);
    const auto initial = pacing.poll(zero);
    bt::require(initial.physics_steps == 0 && initial.request_frame, "initial frame does not invent a physics step");
    const auto requested = pacing.poll(zero + 500ms);
    bt::require(requested.physics_steps == 4 && requested.request_frame, "wall debt and render cadence use separate clocks");
    pacing.completed(2);
    bt::require(pacing.debt_s() == .25 && pacing.poll(zero + 500ms).physics_steps == 2,
        "a cooperative yield debits only completed steps");
    bt::invalid([&] { pacing.completed(3); });
    pacing.completed(2);
    pacing.set_scale(2);
    bt::require(pacing.poll(zero + 1s).physics_steps == 0, "speed change rebases old wall debt");
    bt::require(pacing.poll(zero + 1250ms).physics_steps == 4, "2x pacing keeps the original fixed physics timestep");
    pacing.completed(4);
    pacing.set_paused(true);
    bt::require(pacing.poll(zero + 2s).physics_steps == 0 && pacing.poll(zero + 4s).physics_steps == 0,
        "pause accrues no physics debt even over a long interval");
    pacing.set_paused(false);
    bt::require(pacing.poll(zero + 5s).physics_steps == 0 && pacing.poll(zero + 5250ms).physics_steps == 4,
        "resume avoids a catch-up burst");
    bt::invalid([&] { (void)pacing.poll(zero); });
    bt::invalid([&] { pacing.set_scale(3); });
    bt::invalid([] { view::Pacer bad(0.); });
    bt::invalid([] { view::Pacer bad(.00125, 0.); });
    pacing.reset(zero + 6s);
    bt::require(pacing.debt_s() == 0., "reset discards previous-episode debt");

    view::AchievedRate rate(zero, 0.);
    bt::require(!rate.update(zero + 250ms, .5), "achieved rate waits for the declared half-second window");
    bt::require(rate.update(zero + 500ms, 1.) == std::optional<double>(2.), "RTF reports actual simulated advance");
    bt::invalid([&] { (void)rate.update(zero + 1s, .5); });
    rate.rebase(zero + 1s, 0.);
    bt::require(!rate.update(zero + 1250ms, .25), "generation reset clears the prior RTF sample");
}
void controls_and_camera() {
    bt::require(view::action_for_key(view::Mode::physical, ' ') == view::Action::toggle_brakes &&
        view::action_for_key(view::Mode::research, ' ') == view::Action::toggle_pause &&
        view::action_for_key(view::Mode::research, 'B') == view::Action::toggle_brakes,
        "physical braking and research pause keep their distinct source controls");
    bt::require(view::action_for_key(view::Mode::inspection, 'R') == view::Action::reset &&
        view::action_for_key(view::Mode::checked_replay, 'R') == view::Action::reset &&
        view::action_for_key(view::Mode::inspection, 262) == view::Action::none,
        "both replay modes restart while inspection seek keys reach the cursor owner");
    bt::require(view::action_for_key(view::Mode::physical, 'P') == view::Action::fixed_parameter &&
        view::action_for_key(view::Mode::legacy, 'P') == view::Action::none &&
        view::action_for_key(view::Mode::playground, ' ') == view::Action::none,
        "legacy/playground tuning routes to their owners while physical parameters stay fixed");
    view::InputState input;
    input.apply(view::Action::toggle_brakes); input.apply(view::Action::brake_less);
    bt::require(std::abs(input.brake_demand() - .9) < 1e-12, "brake demand uses the selected strength");
    for (int i = 0; i < 30; ++i) input.apply(view::Action::brake_less);
    bt::require(input.brake_demand() == 0., "brake strength clamps at zero");
    for (int i = 0; i < 30; ++i) input.apply(view::Action::brake_more);
    input.apply(view::Action::faster); input.apply(view::Action::faster); input.apply(view::Action::faster);
    input.apply(view::Action::faster); input.apply(view::Action::toggle_pause); input.apply(view::Action::reset);
    bt::require(input.scale == 8 && !input.paused && !input.braking && input.brake_strength == 1.,
        "reset clears operator pause/braking while preserving playback preferences");
    input.apply(view::Action::normal_speed);
    bt::require(input.scale == 1 && view::speed_for_action(view::Action::slower, 1) == 1,
        "time-scale keys clamp to the declared 1/2/4/8 choices");

    mjvCamera camera{};
    view::Camera tracking;
    tracking.update(camera, 10., 2.);
    bt::require(camera.azimuth == 90. && camera.elevation == -5. && camera.distance == 2.5 &&
        camera.lookat[0] == 10.15 && camera.lookat[2] == 2.25, "side tracking uses the source preset");
    tracking.update(camera, 11., 3.);
    bt::require(camera.lookat[0] == 11.15 && std::abs(camera.lookat[2] - 2.33) < 1e-12,
        "camera follows X directly and smooths vertical movement only");
    tracking.cycle_mode(); tracking.update(camera, 11., 3.);
    bt::require(camera.azimuth == 135. && camera.elevation == -15. && camera.distance == 3.2 &&
        camera.lookat[2] == 3.3, "chase preset resets rather than interpolating the old camera");
    bt::invalid([&] { tracking.update(camera, std::numeric_limits<double>::infinity(), 0.); });
    bt::invalid([] { view::Camera bad(-.1); });
    bt::require(view::help_text(view::Mode::inspection).find("Shift 0.1 s") != std::string::npos &&
        view::replay_exit_code(false, true) == 2 && view::replay_exit_code(true, false) == 1 &&
        view::replay_exit_code(true, true) == 0, "inspection help and checked replay outcomes are explicit");
    const std::vector<view::HudColumn> legend{{"F", " L→10"}, {"v", " 2°"}};
    bt::require(view::hud_legend(legend) == " F    v ", "HUD legends measure visible Unicode symbols rather than UTF-8 bytes");
}
}
int main() { return bt::run([] { pacing(); controls_and_camera(); }); }
