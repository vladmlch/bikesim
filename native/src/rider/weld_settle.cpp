#include "support_dispatch.hpp"
#include "../writers/rider_contacts.hpp"

namespace bike::rider {
writers::RiderSettleOutcome settle_supports(writers::RiderContactWriter& owner,mjData& data,bool raw) {
    // Geometry is captured from this incoming pose and handed to the same
    // owner which latches solved reactions. Runtime never rebinds eq_data
    // to hide overload or a separation gap.
    const auto prepared=raw?std::optional{owner.prepare_attachment_raw(&data)}:std::nullopt;
    return owner.settle_welds(&data,std::nullopt,raw,prepared?&*prepared:nullptr);
}
} // namespace bike::rider
