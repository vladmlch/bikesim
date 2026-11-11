// senses_codec.hpp — strict owned-Value codecs for the rider senses
// channel (launch part 1, task 2). The documents ride inside the
// bootstrap state (`state.senses`, beside `signals`) and the runtime
// config envelope (`config.surface_labels`, beside `rider_intent`).
#pragma once

#include "../io/value.hpp"
#include "senses.hpp"

namespace rider {

// state.senses — six exact keys; finite reals, integer rear_teeth >= 3,
// surface labels from the closed SurfaceLabel domain.
[[nodiscard]] bike::io::Value encode_rider_senses(const RiderSenses &s);
[[nodiscard]] RiderSenses decode_rider_senses(const bike::io::Value &v,
                                              std::string_view path);
// config.surface_labels — {"surface": <default label>, "sections":
// [{"start_m", "end_m", "surface": <label>}]}; the section list must be
// sorted and non-overlapping like SurfaceMap's constructor enforces.
[[nodiscard]] bike::io::Value
encode_surface_labels(const SurfaceLabels &s);
[[nodiscard]] SurfaceLabels
decode_surface_labels(const bike::io::Value &v, std::string_view path);

} // namespace rider
