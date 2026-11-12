// rider/intent_signals.hpp — seated_climb.py SeatedClimbSignals (moved
// from intent.hpp so perception.hpp and intent.hpp do not include each
// other).
#pragma once

#include <array>
#include <optional>

namespace rider {

// seated_climb.py SeatedClimbSignals — validated at wire decode like the
// frozen dataclass's __post_init__.
struct IntentSignals {
    double pitch_rate_up_rad_s = 0.;
    std::array<double, 3> specific_force_body_mps2{0., 0., 0.};
    double crank_rate_rad_s = 0.;
    double human_crank_torque_nm = 0.;
    std::optional<double> front_load_share;
};

} // namespace rider
