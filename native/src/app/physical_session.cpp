#include "physical_session.hpp"

namespace bike::app {
std::unique_ptr<runtime::NativeRideRuntime> make_ride(PreparedSimulation prepared) {
    return std::make_unique<runtime::NativeRideRuntime>(std::move(prepared.bootstrap));
}
} // namespace bike::app
