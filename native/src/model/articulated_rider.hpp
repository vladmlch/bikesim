#pragma once

#include "builder_internal.hpp"
#include "rider_geometry.hpp"

namespace bike::model::detail {

void build_articulated_rider_3d(
    Element& world,
    const ArticulatedPose3d& pose,
    const SegmentMasses& masses,
    const configuration::JointEnvelopeMap* envelopes,
    bool lock_ankles);

void finish_rider_attachments_3d(
    Element& root,
    const BuildContext& context,
    const ArticulatedPose3d& pose);

} // namespace bike::model::detail
