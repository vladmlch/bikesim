// senses.hpp — rider-sensed channel records (launch part 1, task 2).
//
// The launch plan's human-perception channel: what the rider feels and
// reads at the cockpit each interval — the grip load at the handlebar,
// own speed, the engaged rear cog, the surface label under the front
// wheel and at the lookahead point, and the road grade preview. Senses
// are telemetry/state only in this part; no behavior consumes them yet.
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace rider {

// The track surface registry's label domain as a closed type — the five
// names terrain::get_surface knows. Labels travel as strings on the wire;
// the enum keeps the runtime record inside the registry domain.
enum class SurfaceLabel : std::uint8_t { asphalt, hardpack, loose, wet, scree };

[[nodiscard]] constexpr std::optional<SurfaceLabel>
surface_label(std::string_view name) noexcept {
    if (name == "asphalt") return SurfaceLabel::asphalt;
    if (name == "hardpack") return SurfaceLabel::hardpack;
    if (name == "loose") return SurfaceLabel::loose;
    if (name == "wet") return SurfaceLabel::wet;
    if (name == "scree") return SurfaceLabel::scree;
    return std::nullopt;
}

[[nodiscard]] constexpr std::string_view
surface_label_name(SurfaceLabel label) noexcept {
    switch (label) {
    case SurfaceLabel::asphalt: return "asphalt";
    case SurfaceLabel::hardpack: return "hardpack";
    case SurfaceLabel::loose: return "loose";
    case SurfaceLabel::wet: return "wet";
    case SurfaceLabel::scree: return "scree";
    }
    return {};
}

// What a human rider senses this interval (spec «Ощущения райдера»).
// hand_press_n: the component, along the bicycle frame's up axis, of the
// summed grip reaction ON the rider — positive when the bar pushes the hands
// up, i.e. the hands press down on the bar (rider_contacts.hpp:211-216
// convention). No tyre load, friction or COM enters here.
struct RiderSenses {
    double hand_press_n = 0.;
    double speed_mps = 0.;
    // The strict reader's cog domain is >=3 teeth, so the member default
    // keeps a not-yet-acquired record decode-valid (review P2) — the real
    // startup value always comes from the drivetrain's engaged cog.
    int rear_teeth = 3;
    SurfaceLabel surface_rear = SurfaceLabel::hardpack;
    SurfaceLabel surface_front = SurfaceLabel::hardpack;
    SurfaceLabel surface_ahead = SurfaceLabel::hardpack;
    double grade_ahead = 0.;
};

// Label-only projection of the track surface map: default label plus
// [start_m, end_m) sections (terrain/surface.cpp:55-58 semantics).
struct SurfaceLabelSection {
    double start_m = 0., end_m = 0.;
    SurfaceLabel label{};
};

struct SurfaceLabels {
    SurfaceLabel surface = SurfaceLabel::hardpack;
    std::vector<SurfaceLabelSection> sections;

    SurfaceLabels() = default;
    // SurfaceMap's constructor contract on the label twin: sections are
    // stable-sorted by start_m and must be ordered and non-overlapping.
    explicit SurfaceLabels(SurfaceLabel default_label,
                           std::vector<SurfaceLabelSection> sections = {});

    // SurfaceMap::at's label twin: the first matching section's label,
    // else the uniform default; nonfinite x rejects.
    [[nodiscard]] SurfaceLabel at(double x_m) const;
};

} // namespace rider
