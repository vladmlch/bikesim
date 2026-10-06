#pragma once
#include "../config_types.hpp"
#include "../validation.hpp"
#include "../config_validation.hpp"

namespace drivetrain {
    using validation::finite;
    using validation::nonnegative;
    using validation::positive;
    using validation::finite_optional;

    // Named invalid MuJoCo object ID: mj_name2id reports a missing name as
    // -1 while 0 is the world body (and a valid joint/actuator index), so
    // zero must never double as the "absent" sentinel.
    inline constexpr int absent_id = -1;
}
