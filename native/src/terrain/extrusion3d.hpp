#pragma once

#include "terrain/profile.hpp"
#include "terrain/terrain3d_types.hpp"
#include "terrain/types.hpp"

#include <string>
#include <vector>

namespace bike::terrain {

class ExtrudedTrack3d {
public:
    ExtrudedTrack3d(const TrackSpec& track, double half_width_m, double datum_z_offset_m = 0.,
                    double resolution_m = DEFAULT_RESOLUTION_M);

    [[nodiscard]] const TerrainBounds3d& bounds() const noexcept { return bounds_; }
    [[nodiscard]] const std::string& sha256_hex() const noexcept { return sha256_hex_; }
    [[nodiscard]] TerrainSample3d sample(double x_m, double y_m) const noexcept;
    [[nodiscard]] TorusClosestPoint3d closest_point(const Vec3d& center_world_m,
                                                    double search_radius_m) const noexcept;

private:
    struct LabelSpan {
        double start_m = 0.;
        double end_m = 0.;
        rider::SurfaceLabel label = rider::SurfaceLabel::hardpack;
    };

    [[nodiscard]] rider::SurfaceLabel label_at(double x_m) const noexcept;

    std::vector<double> xs_;
    std::vector<double> zs_;
    SignedProfile signed_profile_;
    rider::SurfaceLabel default_label_ = rider::SurfaceLabel::hardpack;
    std::vector<LabelSpan> sections_;
    TerrainBounds3d bounds_{};
    std::string sha256_hex_{};
};

} // namespace bike::terrain
