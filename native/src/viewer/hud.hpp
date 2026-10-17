#pragma once
#include "session.hpp"
#include <optional>
#include <string>
namespace bike::viewer {
struct HudColumn { std::string legend, text; };
[[nodiscard]] std::vector<HudColumn> physical_columns(const runtime::RenderFrame&, std::optional<double> achieved_rtf);
[[nodiscard]] std::string hud_legend(const std::vector<HudColumn>&);
[[nodiscard]] std::string hud_line(const runtime::RenderFrame&, const InputState&, std::optional<double> achieved_rtf, Mode);
[[nodiscard]] std::string overlay_text(const runtime::RenderFrame&, const InputState&, std::optional<double> achieved_rtf, Mode);
[[nodiscard]] std::string playground_hud(const bike::io::Value&, bool auto_sweep);
} // namespace bike::viewer
