#include "support_dispatch.hpp"
#include "../writers/rider_contacts.hpp"
#include <stdexcept>

namespace bike::rider {
SupportDispatch support_dispatch(const ::rider::RiderContactsConfig& config) {
    ::rider::validate(config);
    return {.pedals=config.pedal_attachment,.saddle=config.saddle_attachment,.grips=config.grip_attachment,
        .closed_form_spindle=config.pedal_attachment==::rider::PedalAttachment::spindle};
}
void validate_support_topology(const mjModel& model,const ::rider::RiderContactsConfig& config) {
    (void)support_dispatch(config);
    const auto require=[&](mjtObj type,const std::string& name) {
        if(mj_name2id(&model,type,name.c_str())<0)throw std::invalid_argument("rider support topology missing '"+name+"'");
    };
    for(auto side:{"front","rear"}) {
        const std::string suffix=side;
        require(mjOBJ_SITE,"site_rider_sole_"+suffix);require(mjOBJ_GEOM,"geom_pedal_"+suffix);
        if(config.pedal_attachment==::rider::PedalAttachment::weld)require(mjOBJ_EQUALITY,"weld_foot_"+suffix);
        if(config.pedal_attachment==::rider::PedalAttachment::spindle)require(mjOBJ_EQUALITY,"connect_foot_"+suffix);
    }
    require(mjOBJ_SITE,"site_rider_saddle");require(mjOBJ_GEOM,"geom_saddle");
    if(config.saddle_attachment==::rider::SaddleAttachment::weld)require(mjOBJ_EQUALITY,"weld_saddle");
    if(config.saddle_attachment==::rider::SaddleAttachment::pin)require(mjOBJ_EQUALITY,"connect_saddle");
    if(config.grip_attachment==::rider::GripAttachment::connect)
        for(auto side:{"left","right"})require(mjOBJ_EQUALITY,"connect_grip_"+std::string(side));
}
std::unique_ptr<writers::RiderContactWriter> make_support_writer(const mjModel& model,const ::rider::RiderContactsConfig& config) {
    validate_support_topology(model,config);return std::make_unique<writers::RiderContactWriter>(&model,config);
}
} // namespace bike::rider
