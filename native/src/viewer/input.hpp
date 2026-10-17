#pragma once
#include <optional>
#include <string>
#include <string_view>

namespace bike::viewer {
enum class Mode { physical, research, legacy, playground, inspection, checked_replay };
enum class Action {
    none, close, reset, toggle_pause, toggle_brakes, brake_less, brake_more,
    camera_cycle, camera_side, camera_chase, telemetry, markers, help,
    slower, faster, normal_speed, fixed_parameter
};
[[nodiscard]] Action action_for_key(Mode, int key);
[[nodiscard]] unsigned speed_for_action(Action, unsigned current);
[[nodiscard]] std::string help_text(Mode);
struct InputState {
    unsigned scale = 1;
    bool paused = false, braking = false, telemetry = true, markers = false;
    double brake_strength = 1.;
    void apply(Action);
    [[nodiscard]] double brake_demand() const noexcept;
};
} // namespace bike::viewer
