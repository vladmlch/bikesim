// drivetrain/state_codec.hpp — the owned bike::io::Value state codec for
// the drivetrain boundary (task T08; migration/interface-contracts.md
// §1/§2).
//
// The wire document is the flattened `state.drive` object the bootstrap
// restores — the same 12-key schema set_drive_state() reads through the
// nanobind adapter (drive_binding.cpp), decoded here over the Python-free
// T05 AST. The codec is a pure function of the wire document: a malformed
// member, a foreign transmission layout or a width mismatch throws
// std::invalid_argument naming the public path BEFORE anything mutates —
// nothing here touches a live DriveWriter, so rejection is atomic by
// construction.
//
// The schema is closed: policy sections, optional scalars, the fixed
// DriveAngles width, typed DriveTelemetry lanes (67-field spec table,
// closed labels, integer floors), pending actuation and the three
// transmission snapshots. Encoding re-emits the identical document shape
// (fixed-width angles expand back to the wire's variable list; telemetry
// lanes emit in schema order with labels spelled back), so
// decode(encode(state)) is the identity on well-formed snapshots.
#pragma once

#include "../io/value.hpp"
#include "../writers/drivetrain.hpp"

namespace drivetrain {

    // The `state.drive` document -> typed DriveSnapshot. Rejections are
    // std::invalid_argument carrying the full member path under
    // "state.drive" (the path the binding's parse_drive_snapshot reports
    // for the same document).
    [[nodiscard]] DriveSnapshot
    decode_drive_snapshot(const bike::io::Value &value);

    // The inverse map — the same object a live writer's drive state
    // snapshot serializes to. Key emission order mirrors the binding's
    // state_dict(); canonical_json() normalizes order for comparison.
    [[nodiscard]] bike::io::Value
    encode_drive_snapshot(const DriveSnapshot &state);

} // namespace drivetrain
