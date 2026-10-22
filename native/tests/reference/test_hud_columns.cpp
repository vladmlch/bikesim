// Completion 3.22b. Source: tests/reference/test_hud_columns.py.
// The Python HUD emits styled (text, color) segments; the native HudColumn
// surface is plain (legend, text) by design. The ported assertions cover the
// column text, ordering and latched balance states. The orange1
// saturation-highlight record is dispositioned drop: no styled-segment
// surface exists on the native HUD, and the directional-capacity data it
// colored still reaches the sample's rider_joint_capacity_nm channel.
#include "support/migration_test.hpp"
#include "runtime/runtime.hpp"
#include "viewer/hud.hpp"
#include <algorithm>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

namespace {

runtime::WireObject balance_state(bool lost, double low_speed_s, // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API.
                                  double lost_at_m = 0.) {
    runtime::WireObject balance{{"balance_lost", runtime::Wire(lost)},
                                {"low_speed_s", runtime::Wire(low_speed_s)}};
    if (lost) balance.emplace_back("balance_lost_at_m", runtime::Wire(lost_at_m));
    return balance;
}

runtime::RenderFrame frame_for(runtime::WireObject balance, double shoulder = 20.,
                               double capacity = 100.) {
    runtime::RenderFrame frame;
    auto owned = std::make_shared<runtime::PhysicalSampleData>();
    owned->time_s = 1.;
    owned->end_time_s = 1.0005;
    owned->qvel = {0.};
    owned->channels = {
        {"tires", runtime::WireObject{{"front", runtime::WireObject{{"normal_load_n", 300.}}},
                                      {"rear", runtime::WireObject{{"normal_load_n", 500.}}}}},
        {"drive", runtime::WireObject{}},
        {"rider", runtime::WireObject{}},
        {"rider_joint_torques", runtime::WireObject{{"rider_shoulder_left", shoulder},
                                                    {"rider_elbow_left", -20.}}},
        {"rider_joint_capacity_nm", runtime::WireObject{{"rider_shoulder_left", capacity},
                                                        {"rider_elbow_left", 100.}}},
        {"rider_lean_limit_rad", std::numbers::pi / 6.},
        {"rider_balance", std::move(balance)}};
    frame.state.latest_sample = std::move(owned);
    frame.state.view = {{"drive_mode", runtime::Wire("articulated_effort")}};
    frame.presentation.root_x_dofadr = 0;
    frame.presentation.torso_pitch_deg = 0.;
    frame.presentation.pitch_rad = 0.;
    return frame;
}

std::string joined(const std::vector<bike::viewer::HudColumn> &columns) {
    std::string out;
    for (const auto &column : columns) out += column.text;
    return out;
}

std::size_t label_at(const std::vector<bike::viewer::HudColumn> &columns,
                     const std::string &legend) {
    const auto found = std::ranges::find_if(columns, [&](const auto &column) {
        return column.legend == legend; });
    bt::require(found != columns.end(), "HUD column legend is present: " + legend);
    return static_cast<std::size_t>(std::distance(columns.begin(), found));
}

// test_arm_lean_limit_and_balance_columns — parametrized over the three
// balance states {ok, low 0.3s, LOST@x=12 m}. The green/yellow/red style
// assertion is not portable: HudColumn carries no color segment.
void test_arm_lean_limit_and_balance_columns() {
    struct Case { runtime::WireObject state; std::string want; };
    const std::array cases{
        Case{.state=balance_state(false, 0.), .want="ok"},
        Case{.state=balance_state(false, .3), .want="low 0.3s"},
        Case{.state=balance_state(true, .5, 12.), .want="LOST@x=12 m"},
    };
    for (const auto &[state, want] : cases) {
        const auto columns = bike::viewer::physical_columns(
            frame_for(runtime::WireObject(state)), 1.);
        const std::string text = joined(columns);
        bt::require(text.find("arm=") != std::string::npos, "arm column is emitted");
        bt::require(text.find("/lim30°") != std::string::npos,
                    "lean column prints the 30-degree limit for pi/6 rad");
        bt::require(text.find("BAL") != std::string::npos, "BAL column is emitted");
        bt::require(text.find(want) != std::string::npos,
                    "balance status " + want + " is rendered");
        const auto bal = label_at(columns, "BAL");
        bt::require(columns[bal].text.find(want) != std::string::npos,
                    "the balance state lands on the BAL column");
        bt::require(label_at(columns, "bar l/r") < label_at(columns, "arm") &&
                    label_at(columns, "arm") < label_at(columns, "saddle"),
                    "bar l/r < arm < saddle ordering is kept");
        bt::require(label_at(columns, "lean") < label_at(columns, "BAL"),
                    "lean < BAL ordering is kept");
    }
    // The arm column renders the published shoulder/elbow torques themselves.
    const auto columns = bike::viewer::physical_columns(frame_for(balance_state(false, 0.), 90.), 1.);
    bt::require(columns[label_at(columns, "arm")].text.find("sh  +90") != std::string::npos,
                "the shoulder torque reaches the arm column text");
}

} // namespace

int main() {
    return bt::run([] { test_arm_lean_limit_and_balance_columns(); });
}
