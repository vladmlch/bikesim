#include "capabilities.hpp"
#include "../rider/allocation.hpp"
#include <algorithm>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace runtime {
CapabilityReport implemented_capabilities(
    const bike::configuration::SimulationPhysicsConfig& config,
    const bike::configuration::RiderSpecs& rider) {
    namespace cfg=bike::configuration;
    CapabilityReport out;
    const auto reject=[&](std::string path,std::string reason) {
        out.rejections.push_back(std::move(path)+": "+std::move(reason));
    };
    const bool spatial_requested=config.spatial &&
        config.spatial->mode==bike::spatial::Mode::spatial3d;
    if(spatial_requested && config.drive.transmission_model!="elastic_chain" &&
       config.drive.transmission_model!="ideal_mid_drive" &&
       config.drive.transmission_model!="geometric_ideal_mid_drive") {
        reject("drive:"+config.drive.transmission_model,"unknown spatial transmission dispatch");
        out.reasons=out.rejections;
        return out;
    }
    cfg::validate(config);
    if(config.physics_mode!="physical")
        reject("physics_mode","physical runtime requires 'physical'; legacy configurations use RideSimulation");
    if(!config.drive_mode)
        reject("drive_mode","a resolved drive mode is required");
    else if(*config.drive_mode!="articulated_effort" && *config.drive_mode!="crank_effort" &&
            *config.drive_mode!="coast" && *config.drive_mode!="ideal_speed_control")
        reject("drive_mode","unknown native drive dispatch");
    if(!config.pitch_assist || *config.pitch_assist)
        reject("pitch_assist","physical mode requires resolved false");
    if(std::ranges::find(cfg::kRiderVariants,rider.variant)==cfg::kRiderVariants.end())
        reject("rider.variant","unknown rider variant");
    if(config.drive_mode==std::optional<std::string>("articulated_effort") && rider.variant!="articulated_planar")
        reject("rider.variant","articulated_effort requires articulated_planar topology");
    if(rider.variant=="seated" && rider.legs!="rigid")
        reject("rider.legs","physical seated topology supports rigid legs");
    const auto& attachments=config.articulated;
    if(rider.variant!="articulated_planar" && (attachments.pedal_attachment!="flat" ||
       attachments.saddle_attachment!="flat" || attachments.grip_attachment!="spring"))
        reject("articulated","non-flat rider attachments require articulated_planar topology");
    const auto& tires=config.tires;
    if(tires.backend!="native_reference" && tires.backend!="compliant_2d" &&
       tires.backend!="distributed_2d_reference" && tires.backend!="pneumatic" &&
       !(spatial_requested && tires.backend=="pneumatic_3d"))
        reject("tires.backend","unknown contact dispatch");
    // setup.py validate_supported:67-76 — surface_mode's supported set is
    // backend-dependent: compliant/distributed writers read the track map
    // only; pneumatic additionally accepts a configured uniform surface.
    // native_reference has no writer, so the field is unconsumed for it.
    if(tires.backend=="pneumatic" && tires.surface_mode=="configured" && !tires.pneumatic.surface)
        reject("tires.pneumatic.surface","configured pneumatic contact requires a named surface");
    if((tires.backend=="compliant_2d" || tires.backend=="distributed_2d_reference") &&
       tires.surface_mode!="track")
        reject("tires.surface_mode","compliant_2d/distributed_2d_reference resolve the track surface map");
    if(spatial_requested) {
        if(rider.variant=="articulated_planar" &&
           (attachments.pedal_attachment!="spindle" || attachments.saddle_attachment!="pin" ||
            attachments.grip_attachment!="connect" || attachments.coupled_task_control))
            reject("rider.spatial","requires spindle pedals, pin saddle, connect grips and uncoupled controller");
        out.spatial=bike::spatial::check_spatial_capability(*config.spatial,config);
        for(const auto& reason:out.spatial->rejections)
            reject("spatial",reason);
        // W25/W31/W36 connect these source dispatches. No W authoring task
        // clears qualification_pending or claims V03/V04/V06 acceptance.
        out.qualification_pending=true;
    }
    if(out.rejections.empty()) {
        out.paths={"runtime/step.cpp", "writers/drivetrain.cpp", "runtime/native_contacts.cpp"};
        if(tires.backend=="compliant_2d")out.paths.emplace_back("writers/tire.cpp");
        if(tires.backend=="distributed_2d_reference")out.paths.emplace_back("writers/distributed_tire.cpp");
        if(tires.backend=="pneumatic")out.paths.emplace_back("writers/pneumatic_tire.cpp");
        if(tires.backend=="pneumatic_3d") {
            out.paths.emplace_back("writers/pneumatic_3d_tire.cpp");
            out.paths.emplace_back("tyre:pneumatic_3d");
            if(config.spatial->root==bike::spatial::RootKind::free6) { // NOLINT(bugprone-unchecked-optional-access) a pneumatic_3d backend reaches the empty-rejections path only when spatial_requested holds
                out.paths.emplace_back("startup:spatial_free");out.paths.emplace_back("ride:spatial_free");
            }
        }
        if(rider.variant=="articulated_planar") {
            out.paths.emplace_back("rider/spindle_controller.cpp");
            out.paths.emplace_back("writers/rider_contacts.cpp");
            if(config.articulated.pedal_attachment!="spindle") {
                out.paths.emplace_back("rider/legacy_controller.cpp");
                out.paths.emplace_back("rider/response_allocation.cpp");
                out.paths.emplace_back("rider/legacy_allocation.cpp");
                out.allocation_numeric_identity=std::string(bike::rider::allocation_numeric_identity);
            }
        }
        out.implemented=true;
    }
    out.reasons=out.rejections;
    if(out.implemented)out.reasons.emplace_back("implementation is connected; numerical and runtime qualification has not been executed");
    return out;
}
CapabilityReport capabilities(const bike::configuration::SimulationPhysicsConfig& config,
                              const bike::configuration::RiderSpecs& rider) {
    return implemented_capabilities(config,rider);
}
void validate_physical_capabilities(const bike::configuration::SimulationPhysicsConfig& config,
                                    const bike::configuration::RiderSpecs& rider) {
    const auto report=implemented_capabilities(config,rider);
    if(!report.implemented)throw std::invalid_argument("unsupported native configuration: "+report.rejections.front());
}
} // namespace runtime
