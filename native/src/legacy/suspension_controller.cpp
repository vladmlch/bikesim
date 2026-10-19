#include "playground.hpp"
#include <cmath>
#include <stdexcept>
namespace bike::legacy {
SuspensionController::SuspensionController(const geometry::BikeSpecs& specs,const model::ForkAirSpring& air,const model::BikeSuspensionSystem& suspension)
    :specs_(specs),air_(air),suspension_(suspension){}
SuspensionForce SuspensionController::compute_fork_force(double travel,double velocity)const {
    const double spring=air_.compute_axial_force(travel);
    const double damping=suspension_.fork_damper.compute_damping_force(velocity,travel);
    const double total=spring+damping;
    if(!std::isfinite(total))throw std::invalid_argument("SuspensionController.fork_total must be finite");
    return {.total_n=total,.spring_n=spring,.damper_n=damping};
}
SuspensionForce SuspensionController::compute_shock_force(double stroke,double velocity)const {
    const double spring=(stroke/1000.)*specs_.shock_stiffness;
    const double damping=suspension_.shock_damper.compute_damping_force(velocity,stroke);
    return {.total_n=spring+damping,.spring_n=spring,.damper_n=damping};
}
} // namespace bike::legacy
