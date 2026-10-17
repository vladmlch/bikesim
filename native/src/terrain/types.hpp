// T20/T21 owned terrain records. Source: bike_sim/terrain/*.py.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

namespace bike::terrain {

using Point = std::array<double, 2>;
using Interval = std::pair<double, double>;
using Range = std::array<double, 2>;
using CountRange = std::array<std::int64_t, 2>;
inline constexpr double DEFAULT_RESOLUTION_M = .005;
inline constexpr double FIELD_RUNOUT_MARGIN_M = 5.;
inline constexpr double FIELD_LENGTH_STEP_M = 10.;
inline constexpr std::string_view DEFAULT_SURFACE = "hardpack";
inline constexpr std::string_view ROAD_SURFACE = "asphalt";
inline constexpr std::string_view TRAIL_SURFACE = "hardpack";
inline constexpr double DEFAULT_STRIBECK_SPEED_MPS = 4.5;

struct SurfaceSpec {
    std::string name;
    double mu_peak = 0.;
    double mu_slide = 0.;
    double slip_stiffness_per_load = 0.;
    double stribeck_speed_mps = DEFAULT_STRIBECK_SPEED_MPS;
    std::string description;
    void validate() const;
    [[nodiscard]] double mu(double sliding_speed_mps) const;
    [[nodiscard]] std::vector<double> mu(std::span<const double> speeds) const;
};
[[nodiscard]] std::vector<std::string> available_surfaces();
[[nodiscard]] const SurfaceSpec& get_surface(std::string_view name);

struct SurfaceSection {
    double start_m = 0.;
    double end_m = 0.;
    std::string surface;
    void validate() const;
};
class SurfaceMap {
public:
    explicit SurfaceMap(SurfaceSpec surface, std::vector<SurfaceSection> sections = {});
    [[nodiscard]] static SurfaceMap uniform(std::string_view name);
    [[nodiscard]] const SurfaceSpec& at(double x_m) const;
    void validate() const;
    [[nodiscard]] std::vector<SurfaceSpec> surfaces() const;
    [[nodiscard]] std::string name() const;
    [[nodiscard]] const std::vector<SurfaceSection>& sections() const noexcept { return sections_; }
private:
    SurfaceSpec surface_;
    std::vector<SurfaceSection> sections_;
};
[[nodiscard]] double slip_stiffness_n(const SurfaceSpec& surface, double normal_load_n);

struct GradeProfile {
    std::vector<Point> knots;
    void validate() const;
    [[nodiscard]] double slope(double x_m) const;
    [[nodiscard]] double elevation(double x_m) const;
    [[nodiscard]] std::vector<double> slope(std::span<const double> x_m) const;
    [[nodiscard]] std::vector<double> elevation(std::span<const double> x_m) const;
};

// These are the full dataclass fields, in source order. Random seeds keep
// arbitrary precision in decimal rather than narrowing the NumPy seed domain.
struct SquareEdge { double start_m=0., height_m=.090, ledge_length_m=.250; };
struct Pothole { double start_m=0., depth_m=.180, hole_length_m=.600; };
struct Washboard { double start_m=0., amplitude_m=.035, wavelength_m=.900; std::int64_t n_waves=9; };
struct GOut { double start_m=0., depth_m=.350, dip_length_m=5.; };
struct Drop { double start_m=0., height_m=.600, ramp_m=0.; };
struct Kicker {
    double start_m=0., height_m=.350, ramp_m=1.400, gap_m=2.500;
    double landing_angle_deg=20., landing_m=3.;
    [[nodiscard]] double landing_drop_m() const;
    [[nodiscard]] double launch_angle_deg() const;
};
struct Roots {
    double start_m=0.; std::int64_t n_bumps=7;
    double height_m=.080, width_m=.250, section_length_m=7., jitter=.35;
    std::string seed="0";
    [[nodiscard]] std::vector<double> bump_centres_m() const;
};
struct RockGarden {
    double start_m=0., section_length_m=8., amplitude_m=.060, correlation_length_m=.250;
    std::string seed="0";
    [[nodiscard]] std::pair<std::vector<double>,std::vector<double>> nodes() const;
};
struct Bump { double start_m=0., height_m=.050, bump_length_m=.400; };
struct TrapezoidBump { double start_m=0., height_m=.050, ramp_m=.150, plateau_m=.300; };
struct SlopedPothole {
    double start_m=0., depth_m=.080, hole_length_m=.500, edge_m=.100;
    [[nodiscard]] double effective_edge_m() const;
};
struct BowlPothole { double start_m=0., depth_m=.080, hole_length_m=.500; };
struct RoadRoughness {
    double start_m=0., section_length_m=10., amplitude_m=.003, correlation_length_m=.200;
    std::string seed="0";
    [[nodiscard]] std::pair<std::vector<double>,std::vector<double>> nodes() const;
};
struct SteppedClimb {
    double start_m=0.;
    std::vector<Point> steps{{5.,15.},{10.,15.},{15.,15.},{20.,15.},{25.,15.}};
    double transition_m=3.;
};

struct Obstacle {
    using Shape = std::variant<SquareEdge,Pothole,Washboard,GOut,Drop,Kicker,Roots,
                               RockGarden,Bump,TrapezoidBump,SlopedPothole,
                               BowlPothole,RoadRoughness,SteppedClimb>;
    Shape shape;
    // Source dataclasses retain authored integer tokens in geometric fields.
    // This preserves their source-compatible TOML rendering after a typed read.
    std::vector<std::string> integer_scalar_fields{};
    [[nodiscard]] std::string_view kind() const;
    [[nodiscard]] double start_m() const;
    void set_start_m(double value);
    [[nodiscard]] double length_m() const;
    [[nodiscard]] double end_m() const;
    [[nodiscard]] double datum_shift_m() const;
    [[nodiscard]] std::string label() const;
    [[nodiscard]] bool annotate() const;
    [[nodiscard]] bool is_pothole() const;
    [[nodiscard]] bool is_bump() const;
    void validate() const;
    [[nodiscard]] std::vector<double> elevation(std::span<const double> local_x) const;
};

struct TrackSpec {
    std::string name;
    double length_m=0.;
    std::vector<Obstacle> obstacles;
    std::string description;
    std::string surface="hardpack";
    std::optional<GradeProfile> grade_profile;
    std::vector<SurfaceSection> surface_sections;
    [[nodiscard]] SurfaceMap surface_map() const;
    [[nodiscard]] std::vector<Obstacle> sorted_obstacles() const;
    [[nodiscard]] double datum_shift_m() const;
    [[nodiscard]] std::vector<std::pair<double,std::string>> markers() const;
    void validate() const;
};

struct HeightFieldSpec {
    int nrow=2, ncol=24001;
    double radius_x_m=60., radius_y_m=.5, elevation_m=3.4, base_m=.5, datum_z_m=2.8;
    [[nodiscard]] double track_length_m() const;
    [[nodiscard]] double resolution_m() const;
    [[nodiscard]] double min_profile_m() const;
    [[nodiscard]] double max_profile_m() const;
    [[nodiscard]] std::string size_attr() const;
    [[nodiscard]] std::vector<double> track_x() const;
    [[nodiscard]] double geom_x_m() const;
    [[nodiscard]] double geom_z_m(double ground_z_m) const;
    [[nodiscard]] double catch_plane_z_m(double ground_z_m,double clearance_m=1.) const;
    [[nodiscard]] bool fits(Interval extent) const;
    [[nodiscard]] std::vector<double> normalize(std::span<const double> profile_m) const;
    [[nodiscard]] HeightFieldSpec with_length(double field_length_m) const;
    [[nodiscard]] static HeightFieldSpec for_track_length(double track_length_m);
    [[nodiscard]] static HeightFieldSpec for_track(const TrackSpec& track);
    [[nodiscard]] bool is_default() const;
    void validate() const;
    bool operator==(const HeightFieldSpec&) const = default;
};
inline constexpr HeightFieldSpec FIELD{};

} // namespace bike::terrain
