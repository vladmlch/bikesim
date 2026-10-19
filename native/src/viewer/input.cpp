#include "input.hpp"
#include <algorithm>
#include <array>
#include <stdexcept>

namespace bike::viewer {
Action action_for_key(Mode mode, int key) {
    if (key >= 'a' && key <= 'z') key -= 'a' - 'A';
    switch (key) {
        case 'Q': case 256: return Action::close;
        case 'R': return Action::reset;
        case 'C': return Action::camera_cycle;
        case '1': case 321: return Action::camera_side;
        case '2': case 322: return Action::camera_chase;
        case 'T': return Action::telemetry;
        case 'G': return Action::markers;
        case '/': case '?': return Action::help;
        case 295: return Action::slower;
        case 296: return Action::faster;
        case 297: return Action::normal_speed;
        case ' ': return mode == Mode::research || mode == Mode::inspection || mode == Mode::checked_replay ?
            Action::toggle_pause : mode == Mode::playground ? Action::none : Action::toggle_brakes;
        case 'B': return mode == Mode::research ? Action::toggle_brakes : Action::none;
        case ',': return mode == Mode::physical || mode == Mode::legacy ? Action::brake_less : Action::none;
        case '.': return mode == Mode::physical || mode == Mode::legacy ? Action::brake_more : Action::none;
        default: break;
    }
    if (mode == Mode::physical && key >= 0 && key < 128 && std::string_view("PLKJHUYX09876543[]-=NM;'E").find(static_cast<char>(key)) != std::string_view::npos)
        return Action::fixed_parameter;
    return Action::none;
}
unsigned speed_for_action(Action action, unsigned current) {
    constexpr std::array<unsigned, 4> scales{1, 2, 4, 8};
    std::size_t index = scales.size();
    for (std::size_t i = 0; i < scales.size(); ++i)
        if (scales[i] == current) { index = i; break; }
    if (index == scales.size()) throw std::invalid_argument("scale must be one of 1, 2, 4, 8");
    if (action == Action::normal_speed) return 1;
    if (action == Action::slower) return index == 0 ? 1 : scales[index - 1];
    if (action == Action::faster) return index + 1 == scales.size() ? 8 : scales[index + 1];
    return current;
}
void InputState::apply(Action action) {
    scale = speed_for_action(action, scale);
    switch (action) {
        case Action::toggle_pause: paused = !paused; break;
        case Action::toggle_brakes: braking = !braking; break;
        case Action::brake_less: brake_strength = std::clamp(brake_strength - .1, 0., 1.); break;
        case Action::brake_more: brake_strength = std::clamp(brake_strength + .1, 0., 1.); break;
        case Action::telemetry: telemetry = !telemetry; break;
        case Action::markers: markers = !markers; break;
        case Action::reset: braking = false; paused = false; break;
        case Action::none: case Action::close: case Action::camera_cycle:
        case Action::camera_side: case Action::camera_chase: case Action::help:
        case Action::slower: case Action::faster: case Action::normal_speed:
        case Action::fixed_parameter: break;
    }
}
double InputState::brake_demand() const noexcept { return braking ? brake_strength : 0.; }
std::string help_text(Mode mode) {
    std::string const controls = "C/1/2 camera | T telemetry | G markers | Q/Escape close | F6/F7/F8 speed";
    if (mode == Mode::research) return "Space pause | B brakes | R close/reset | " + controls;
    if (mode == Mode::inspection) return "Space pause | R restart | Left/Right seek 1 s (Shift 0.1 s) | Up/Down speed x1.5 | " + controls;
    if (mode == Mode::checked_replay) return "Space pause | R restart | " + controls;
    if (mode == Mode::physical) return "Space brake toggle | ,/. brake strength | R reset | " + controls +
        "\nPhysical parameters are fixed per run; edit the configuration and start a new run.";
    if (mode == Mode::legacy) return "W/S cruise | Space brakes | ,/. brake strength | E assist | R reset | " + controls +
        "\nFork L/K LSC J/H HSC U/Y rebound | Shock 0/9 LSC 8/7 HSC 6/5 rebound 4/3 HBO X lockout"
        "\n[/] air tokens -/= PSI | N/M front tyre ;/' rear tyre | P preset";
    return controls + "\nW/S rear travel | Up/Down fork travel | Space auto sweep | B rider | P preset"
        "\nFork L/K LSC J/H HSC U/Y rebound | Shock 0/9 LSC 8/7 HSC 6/5 rebound 4/3 HBO X lockout"
        "\n[/] air tokens -/= PSI";
}
} // namespace bike::viewer
