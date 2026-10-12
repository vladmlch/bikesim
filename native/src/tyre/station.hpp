// tyre/station.hpp — the scalar per-station law of the distributed tyre
// reference: station_angles/station_density/station_response from
// physics/distributed_tire.py. The knot fold is sequential, which is what
// numpy's pairwise sum computes below eight terms, so each scalar result
// is bitwise the vectorised oracle value for the same station.
#pragma once

#include <utility>
#include <vector>

#include "../config_types.hpp"

namespace biketyre {
    // station_angles(count) (distributed_tire.py:99-104): the per-station
    // circumferential angle 2*pi*i/count and the uniform quadrature
    // weight 2*pi/count.
    struct StationAngles {
        std::vector<double> angles, weights;
    };

    // 16..512 stations (MAX_STATION_COUNT, distributed_tire.py:95), else
    // std::invalid_argument — the oracle's ValueError domain.
    [[nodiscard]] StationAngles station_angles(int count);

    // station_density (distributed_tire.py:125-139): one station's
    // elastic force and stored energy per radian of circumference.
    [[nodiscard]] std::pair<double, double>
    station_density(double delta, const nativecfg::HingeDensity &density);

    // station_response (distributed_tire.py:142-148): one station's
    // normal load and stored energy — density plus damping inside the
    // delta>0 gate, scaled by the quadrature weight.
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional params mirror the Python API
    // NOLINTBEGIN(bugprone-easily-swappable-parameters) positional params mirror the Python API
    [[nodiscard]] std::pair<double, double>
    station_response(double delta, double rate, double weight,
                     const nativecfg::HingeDensity &density);
    // NOLINTEND(bugprone-easily-swappable-parameters)
} // namespace biketyre
