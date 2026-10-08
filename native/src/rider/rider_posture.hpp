// rider/rider_posture.hpp — rider_posture.py RiderPosture, shared by the
// spindle controller command surface and the seated-climb intent stack.
#pragma once
#include <optional>

#include "spindle_math.hpp"

namespace rider {

// rider_posture.py RiderPosture. The construction-time bounds are checked
// where Python's __post_init__ runs: at wire decode (parse_rider_posture)
// and at every posture the policy constructs.
struct RiderPosture {
    double torso_lean_rad = 0.;
    double pelvis_pitch_rad = 0.;
    std::optional<spindle::Vec2> pelvis_offset_m;
    bool use_saddle = true;
};

} // namespace rider
