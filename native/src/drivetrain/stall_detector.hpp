#pragma once
// Loaded drive that makes no forward progress within a window
// (c809f12a^ pedaling.py ProgressStallDetector). A low-rate test misses the
// real dead-spot stall: the crank rocks about top/bottom dead centre at
// several rad/s while going nowhere. Net forward advance of the crank or of
// the wheel (crank-equivalent) since the last anchor separates a slow grind
// from a stall; rocking and rollback both count as no progress.
#include <optional>

namespace drivetrain {
struct StallDetectorLimits { double window_s, crank_progress_rad, wheel_progress_rad; };
struct StallDetectorRates { double crank_rad_s, wheel_rad_s; };
struct StallDetectorState {
    double crank_rad = 0., wheel_rad = 0.;
    std::optional<double> elapsed_s;
    friend bool operator==(const StallDetectorState&, const StallDetectorState&) = default;
};
// True once per stall; the next stall needs a fresh full window.
[[nodiscard]] inline bool stall_update(StallDetectorState& s, const StallDetectorLimits& l,
                                       StallDetectorRates r, double dt, bool loaded) noexcept {
    if (!loaded) { s = {}; return false; }
    if (!s.elapsed_s) { s.elapsed_s = 0.; return false; }
    s.crank_rad += r.crank_rad_s * dt;
    s.wheel_rad += r.wheel_rad_s * dt;
    if (s.crank_rad > l.crank_progress_rad || s.wheel_rad > l.wheel_progress_rad) {
        s = {}; s.elapsed_s = 0.; return false;
    }
    *s.elapsed_s += dt;
    if (*s.elapsed_s >= l.window_s) { s = {}; return true; }
    return false;
}
}  // namespace drivetrain
