#pragma once
#include "plot.hpp"
#include <array>
#include <cstdint>
#include <string_view>

namespace bike::viz::detail {
inline constexpr std::array<std::string_view, 8> palette{
    "#38bdf8", "#fbbf24", "#34d399", "#f97316", "#c084fc", "#ef4444", "#8b949e", "#a855f7"};
inline constexpr std::string_view background="#1a1e24", panel="#22272e", foreground="#e6edf3", muted="#8b949e", grid="#484f58";
struct Point { double x=0., y=0.; };
struct Primitive {
    // NOLINTNEXTLINE(performance-enum-size) enum width matches the reference ABI layout
    enum class Kind : std::uint8_t { line, rectangle, text } kind=Kind::line;
    std::vector<Point> points;
    std::string color, text;
    double thickness=1., font_size=14.;
    bool fill=false, dashed=false;
};
struct Drawing { unsigned width=0,height=0; std::vector<Primitive> primitives; };
struct Limits { double x0=0.,x1=1.,y0=0.,y1=1.; };
[[nodiscard]] Limits limits(const Plot&,bool secondary=false);
[[nodiscard]] Drawing draw(const Report&);
[[nodiscard]] std::string escape_xml(std::string_view);
[[nodiscard]] std::string json_string(std::string_view);
[[nodiscard]] std::string number(double);
[[nodiscard]] std::string color(const Series&,std::size_t index);
[[nodiscard]] std::array<unsigned char,3> rgb(std::string_view);
[[nodiscard]] std::string drawing_svg(const Drawing&);
[[nodiscard]] std::string report_json(const Report&);
void write_text(const std::filesystem::path&,std::string_view);
[[nodiscard]] std::vector<double> required(const Channels&,std::string_view);
void add_series(Plot&,const Channels&,const std::vector<double>& x,std::string_view key,
    std::string_view color={},bool secondary=false,SeriesStyle=SeriesStyle::line);
void track_markers(Plot&,const terrain::TrackSpec&);
using Hardpoints=std::vector<std::pair<std::string,geometry::Vec3>>;
[[nodiscard]] Hardpoints hardpoints(const geometry::BikeSpecs&,const kinematics::HorstLinkageSolver&,double travel_mm);
} // namespace bike::viz::detail
