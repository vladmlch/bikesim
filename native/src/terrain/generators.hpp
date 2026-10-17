// Full procedural terrain specs and deterministic seed/placement boundaries.
#pragma once
#include "terrain/types.hpp"
#include "io/value.hpp"
#include "random/numpy_compat.hpp"

namespace bike::terrain {
inline constexpr double CLEARANCE_M=1.;
inline constexpr int PLACEMENT_TRIES=200;
inline constexpr double DEFAULT_ROAD_LENGTH_M=100.;
inline constexpr std::string_view DEFAULT_PRESET="enduro_aggressive";
inline constexpr std::string_view DEFAULT_ROAD_PRESET="road_worn";
inline constexpr std::int64_t ALPINE_PRESET_SEED=41;
inline constexpr std::int64_t ROCKY_PRESET_SEED=0;
[[nodiscard]] double rounded(double value,int digits=4);
[[nodiscard]] bool is_free(double start,double end,std::span<const Interval> taken);
[[nodiscard]] std::optional<Interval> place(bike::random::NumpyRng& rng,double length,double lo,double hi,
                                           std::span<const Interval> taken,bool shrink=false);
[[nodiscard]] std::vector<Interval> free_intervals(Interval span,std::span<const Interval> occupied,double gap_m);

using Weights=std::vector<std::pair<std::string,double>>;
struct RoadGeneratorSpec {
    std::string seed="0";
    double runup_m=10.,runout_m=5.,min_gap_m=1.;
    double potholes_per_100m=4.;
    Range pothole_depth_m{.040,.120},pothole_length_m{.300,.800};
    Weights pothole_edge{{"sharp",1.}};
    double pothole_edge_m=.100;
    double bumps_per_100m=6.;
    Range bump_height_m{.030,.080},bump_length_m{.300,.600};
    Weights bump_shape{{"cosine",1.}};
    double bump_ramp_fraction=.3,roughness_m=.003,roughness_correlation_m=.200;
    void validate() const;
};
[[nodiscard]] std::vector<Obstacle> generate_road(const RoadGeneratorSpec& spec,double length_m,
                                                std::span<const Interval> reserved={});
[[nodiscard]] TrackSpec build_road(std::string name,const RoadGeneratorSpec& spec,
    double length_m=DEFAULT_ROAD_LENGTH_M,std::span<const Obstacle> hand_placed={},
    std::string description="",std::string surface="asphalt");
[[nodiscard]] RoadGeneratorSpec road_smooth_spec();
[[nodiscard]] RoadGeneratorSpec road_worn_spec();
[[nodiscard]] RoadGeneratorSpec road_broken_spec();
[[nodiscard]] TrackSpec road_smooth();
[[nodiscard]] TrackSpec road_worn();
[[nodiscard]] TrackSpec road_broken();

struct TerrainGenSpec {
    Range length_m{60.,120.};
    double grade_max=.25,grade_peak=.30,grade_peak_prob=.35;
    Range roughness_amp_m{.005,.030},roughness_len_m{3.,10.},roughness_corr_m{.25,.5};
    Range bump_height_m{.02,.10},bump_length_m{.5,1.},edge_height_m{.02,.08},drop_height_m{.02,.10},roots_len_m{3.,6.};
    std::vector<std::string> base_surfaces{"hardpack","asphalt","loose"},section_surfaces{"wet","loose"},feature_types{"bump","roots"};
    Range section_len_m{3.,15.};
    CountRange n_roughness{2,6},n_features{1,4},n_surface_sections{0,2};
    double lead_in_m=4.,lead_out_m=5.;
    // Python does not coerce these scalar input fields when validating;
    // preserve integer-vs-float input tokens solely for source spec hashing.
    std::vector<std::string> integer_scalar_fields;
    void validate() const;
    [[nodiscard]] static TerrainGenSpec from_dict(const bike::io::Value& data);
    [[nodiscard]] bike::io::Value to_dict() const;
    [[nodiscard]] std::string sha256() const;
};
struct AlpineGenSpec {
    double length_m=120.,lead_in_m=10.,lead_out_m=15.,transition_m=2.;
    Range ramp_len_m{15.,30.},ramp_grade{.15,.25},shelf_len_m{4.,8.},shelf_grade{.02,.06},wall_grade{.26,.32};
    std::int64_t n_walls=1;
    Range counter_len_m{3.,6.},counter_grade{-.08,-.03};
    double counter_prob=.3;
    std::string base_surface="hardpack";
    CountRange n_scree{2,3};
    Range scree_len_m{6.,15.};
    double scree_ramp_bias=.7;
    Range scree_amp_m{.010,.020},scree_corr_m{.15,.20};
    CountRange n_wet{0,0};
    Range wet_len_m{2.,5.},background_amp_m{.005,.010},background_corr_m{.20,.30};
    CountRange n_boulders{2,5};
    Range boulder_height_m{.05,.15},boulder_len_m{.6,1.2};
    CountRange n_rock_gardens{1,2};
    Range rock_len_m{4.,8.},rock_amp_m{.03,.06},rock_corr_m{.30,.45};
    CountRange n_water_bars{1,3};
    Range water_bar_depth_m{.08,.15},water_bar_len_m{1.,1.5};
    CountRange n_sharp_edges{0,0};
    Range sharp_edge_height_m{.03,.08};
    bool single_contact_safe=true;
    double min_contact_radius_m=.45,max_kink_deg=17.,contact_window_m=.25;
    void validate() const;
    [[nodiscard]] static AlpineGenSpec from_dict(const bike::io::Value& data);
    [[nodiscard]] bike::io::Value to_dict() const;
    [[nodiscard]] std::string sha256() const;
};
[[nodiscard]] TrackSpec generate_track(const TerrainGenSpec& spec,std::string_view seed,
                                     std::optional<std::string> name=std::nullopt);
[[nodiscard]] TrackSpec generate_track(const TerrainGenSpec& spec,std::uint64_t seed,
                                     std::optional<std::string> name=std::nullopt);
[[nodiscard]] TrackSpec generate_alpine_track(const AlpineGenSpec& spec,std::string_view seed,
                                            std::optional<std::string> name=std::nullopt);
[[nodiscard]] TrackSpec generate_alpine_track(const AlpineGenSpec& spec,std::uint64_t seed,
                                            std::optional<std::string> name=std::nullopt);
[[nodiscard]] TrackSpec generate(const bike::io::Value& specification,std::uint64_t seed);
[[nodiscard]] TrackSpec build_research_track(std::string_view name="rough_uphill",std::string_view seed="0");
[[nodiscard]] TrackSpec enduro_aggressive();
[[nodiscard]] TrackSpec single_edge();
[[nodiscard]] TrackSpec washboard_only();
[[nodiscard]] TrackSpec climb_steps();
[[nodiscard]] TrackSpec alpine_scree(std::string_view seed="41");
[[nodiscard]] TrackSpec alpine_rocky(std::string_view seed="0");
[[nodiscard]] std::vector<std::string> available_presets();
[[nodiscard]] TrackSpec get_preset(std::string_view name);
} // namespace bike::terrain
