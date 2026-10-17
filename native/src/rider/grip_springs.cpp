#include "support_dispatch.hpp"
#include "../writers/rider_contacts.hpp"

namespace bike::rider {
bool set_spring_grip_enabled(writers::RiderContactWriter& owner,bool enabled) {
    // Existing source-equivalent owner handles capture distance/speed,
    // cohesive pair overload and pending release-loss energy exactly once.
    return owner.set_enabled("grip",enabled);
}
} // namespace bike::rider
