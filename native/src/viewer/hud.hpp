#pragma once
#include "session.hpp"
#include <optional>
#include <string>
namespace bike::viewer {
struct HudColumn { std::string legend, text; };
struct SpatialHudView { std::optional<double> unwrapped_yaw_rad; };
[[nodiscard]] std::vector<HudColumn> physical_columns(const runtime::RenderFrame&, std::optional<double> achieved_rtf);
[[nodiscard]] std::vector<HudColumn> physical_columns(const runtime::RenderFrame&, std::optional<double> achieved_rtf, SpatialHudView);
[[nodiscard]] std::string hud_legend(const std::vector<HudColumn>&);
[[nodiscard]] std::string hud_line(const runtime::RenderFrame&, const InputState&, std::optional<double> achieved_rtf, Mode);
[[nodiscard]] std::string hud_line(const runtime::RenderFrame&, const InputState&, std::optional<double> achieved_rtf, Mode, SpatialHudView);
[[nodiscard]] std::string overlay_text(const runtime::RenderFrame&, const InputState&, std::optional<double> achieved_rtf, Mode);
[[nodiscard]] std::string overlay_text(const runtime::RenderFrame&, const InputState&, std::optional<double> achieved_rtf, Mode, SpatialHudView);
} // namespace bike::viewer
