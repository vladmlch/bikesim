#pragma once

#include "io/value.hpp"
#include "pneumatic3d/geometry.hpp"
#include "spatial/types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace bike::pneumatic3d {

inline constexpr std::uint32_t kPneumatic3dStateVersion = 1;

struct CarcassState3d {
    std::vector<double> deflection_m, maxwell_q_n_per_m2, previous_penetration_m;
    std::vector<std::uint8_t> active;
    double material_phase_rad = 0.;
    double stored_energy_j = 0., dissipated_j = 0.;
};

struct PatchState3d {
    std::vector<std::uint64_t> ids;
    std::vector<std::uint32_t> element_to_patch_slot;
    std::vector<std::uint8_t> slot_used;
    std::uint64_t next_id = 1;
    std::uint32_t capacity = 0;
};

struct BrushState3d {
    std::vector<double> ux_m, uy_m;
    std::vector<double> camber_state;
    std::vector<double> bristle_ux_m, bristle_uy_m, bristle_energy_j;
    double elastic_energy_j = 0., sliding_loss_j = 0., convective_release_j = 0.;
};

struct TyreStateIdentity3d {
    std::uint32_t schema = kPneumatic3dStateVersion;
    bike::spatial::TyreTier tier = {};
    TorusDims dims;
    std::uint32_t patch_capacity = 0;
    std::string topology_fingerprint_hex, config_fingerprint_hex;
    std::string terrain_sha256_hex, calibration_sha256_hex;
};

struct TyreState3d {
    TyreStateIdentity3d identity;
    std::array<CarcassState3d, 2> carcass;
    std::array<PatchState3d, 2> patches;
    std::array<BrushState3d, 2> brush;
    std::int64_t accepted_interval = -1;
};

// Cold codecs; decode validates identity before returning owned staged state.
std::vector<std::byte> encode_tyre_state(const TyreState3d&);
std::expected<TyreState3d, std::string> decode_tyre_state(
    std::span<const std::byte>, const TyreStateIdentity3d&);
bike::io::Value tyre_identity_to_wire(const TyreStateIdentity3d&);
TyreStateIdentity3d tyre_identity_from_wire(const bike::io::Value&);
std::string base64_canonical(std::span<const std::byte>);
std::expected<std::vector<std::byte>, std::string> base64_decode_strict(
    std::string_view);

} // namespace bike::pneumatic3d
