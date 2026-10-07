// interval_clock.hpp — the shared simulation-time -> interval-id
// conversion (spec S4 interval domain, mirrored by
// bike_sim.sim.ride.tire_forces.interval_id):
//
//   binary64 `time / dt`, then half-even rounding computed from the
//   value's bits — floor/fmod only, so the result never depends on the
//   process floating-point rounding mode — accepted in [0, INT64_MAX].
//
// Nonfinite or negative time and nonpositive or nonfinite dt are
// std::invalid_argument (Python ValueError); a finite input whose
// interval exceeds int64 is std::overflow_error (Python OverflowError).
// Domain checks run before the quotient so the error class is fixed by
// the input, not by an intermediate overflow.
#pragma once

#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace interval_clock {
    [[nodiscard]] inline std::int64_t interval_id(double time, double dt) {
        if (!std::isfinite(time) || time < 0.0)
            throw std::invalid_argument("invalid tire.interval_id.time");
        if (!std::isfinite(dt) || dt <= 0.0)
            throw std::invalid_argument("invalid tire.interval_id.dt");
        const double quotient = time / dt;
        if (!std::isfinite(quotient) || quotient >= 0x1p63)
            throw std::overflow_error("tire.interval_id");
        // Half-even without nearbyint/rint/fenv: floor splits the quotient
        // exactly (Sterbenz), and a tie lands on the even neighbour.
        double base = std::floor(quotient);
        const double fraction = quotient - base;
        if (fraction > 0.5 ||
            (fraction == 0.5 && std::fmod(base, 2.0) != 0.0))
            base += 1.0;
        if (base >= 0x1p63)
            throw std::overflow_error("tire.interval_id");
        return static_cast<std::int64_t>(base);
    }
} // namespace interval_clock
