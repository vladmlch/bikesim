#pragma once
#include "terrain/types.hpp"
#include "tyre/profile.hpp"
#include "tyre/distance.hpp"

namespace bike::terrain {

[[nodiscard]] std::vector<double> build_profile(const TrackSpec& track,std::span<const double> x);
[[nodiscard]] std::vector<Point> profile(const TrackSpec& track,double resolution_m);
[[nodiscard]] Interval profile_extent(const TrackSpec& track,double resolution_m=DEFAULT_RESOLUTION_M);
[[nodiscard]] std::vector<double> build_field_data(const TrackSpec& track,const HeightFieldSpec& spec);
[[nodiscard]] std::vector<double> build_field_data(const TrackSpec& track);
void assert_track_fits(const TrackSpec& track,const HeightFieldSpec& spec=FIELD);
[[nodiscard]] TrackSpec flat(double length_m=112.);

// Reuse the already qualified contact solver and its per-query owner.
using ProfileContact = biketyre::ProfileContact;
[[nodiscard]] ProfileContact validated_profile_contact(Point point,Point normal,double delta,
                                                       int segment_id,bool multi_support);
class ProfileQuery {
public:
    explicit ProfileQuery(std::vector<Point> vertices,double significant_delta_m=.0001,
                          double significance_fraction=.05,double normal_angle_deg=20.);
    [[nodiscard]] ProfileContact contact(Point center_xz,double radius,
                                        std::optional<int> previous_segment=std::nullopt) const;
private:
    biketyre::ProfileQuery query_;
};
[[nodiscard]] ProfileContact closest_profile_contact(std::vector<Point> vertices,Point center_xz,
                                                    double radius,std::optional<int> previous_segment=std::nullopt);
[[nodiscard]] ProfileContact closest_profile_contact(Point center_xz,double radius,std::vector<Point> vertices,
                                                    std::optional<int> previous_segment=std::nullopt);
struct ProfileDistance {
    std::vector<double> distance;
    std::vector<Point> normal;
    std::vector<Point> point;
    std::vector<std::size_t> segment;
    std::vector<bool> nonsmooth;
};
class SignedProfile {
public:
    explicit SignedProfile(std::vector<Point> vertices);
    [[nodiscard]] const std::vector<Point>& vertices() const noexcept { return vertices_; }
    [[nodiscard]] double height(double x) const;
    [[nodiscard]] std::vector<double> height(std::span<const double> x) const;
    [[nodiscard]] ProfileDistance query(std::span<const Point> points) const;
private:
    std::vector<Point> vertices_;
    biketyre::SignedProfile query_;
};
[[nodiscard]] ProfileDistance signed_profile_distance(std::vector<Point> vertices,std::span<const Point> points);

inline constexpr double DEFAULT_MIN_CONTACT_RADIUS_M=.45;
inline constexpr double DEFAULT_MAX_KINK_DEG=17.;
inline constexpr double DEFAULT_CONTACT_WINDOW_M=.25;
[[nodiscard]] double min_bump_length_m(double height_m,double min_radius_m=DEFAULT_MIN_CONTACT_RADIUS_M);
[[nodiscard]] double max_bowl_depth_m(double length_m,double min_radius_m=DEFAULT_MIN_CONTACT_RADIUS_M);
[[nodiscard]] double concave_span_deg(const RockGarden& obstacle,double window_m=DEFAULT_CONTACT_WINDOW_M);
[[nodiscard]] double concave_span_deg(const RoadRoughness& obstacle,double window_m=DEFAULT_CONTACT_WINDOW_M);
[[nodiscard]] std::vector<std::string> single_contact_hazards(const TrackSpec& track,
    double min_radius_m=DEFAULT_MIN_CONTACT_RADIUS_M,double max_kink_deg=DEFAULT_MAX_KINK_DEG,
    double window_m=DEFAULT_CONTACT_WINDOW_M);
inline constexpr double WHEELPATH_RESOLUTION_M=.0005;
inline constexpr double FRONT_WHEEL_RADIUS_M=.372;
inline constexpr double REAR_WHEEL_RADIUS_M=.352;
[[nodiscard]] std::pair<std::vector<double>,std::vector<double>> wheel_centre_path(
    const Obstacle& obstacle,double wheel_radius_m,double resolution_m=WHEELPATH_RESOLUTION_M);
[[nodiscard]] double effective_drop_m(const Obstacle&,double wheel_radius_m,double resolution_m=WHEELPATH_RESOLUTION_M);
[[nodiscard]] double effective_rise_m(const Obstacle&,double wheel_radius_m,double resolution_m=WHEELPATH_RESOLUTION_M);
[[nodiscard]] double bridged_drop_m(double hole_length_m,double wheel_radius_m);

} // namespace bike::terrain
