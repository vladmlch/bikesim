#pragma once

// Internal validation shared by the cold codec and noexcept model restore.
// This header is not part of the W00 frozen public contract.
#include "state.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <limits>
#include <string_view>

namespace bike::pneumatic3d::detail {
inline bool identity_equal(const TyreStateIdentity3d& a,const TyreStateIdentity3d& b) noexcept {
    return a.schema==b.schema && a.tier==b.tier &&
        a.dims.n_theta_fan==b.dims.n_theta_fan && a.dims.n_phi==b.dims.n_phi &&
        a.dims.n_theta_material==b.dims.n_theta_material && a.patch_capacity==b.patch_capacity &&
        a.topology_fingerprint_hex==b.topology_fingerprint_hex &&
        a.config_fingerprint_hex==b.config_fingerprint_hex &&
        a.terrain_sha256_hex==b.terrain_sha256_hex &&
        a.calibration_sha256_hex==b.calibration_sha256_hex;
}
inline bool digest_valid(std::string_view s) noexcept {
    return s.size()==64 && std::ranges::all_of(s,[](char c) {
        return (c>='0'&&c<='9') || (c>='a'&&c<='f');
    });
}
inline bool identity_valid(const TyreStateIdentity3d& id) noexcept {
    constexpr std::uint64_t limit=1U<<20;
    const auto fan=std::uint64_t{id.dims.n_theta_fan}*id.dims.n_phi;
    const auto mat=std::uint64_t{id.dims.n_theta_material}*id.dims.n_phi;
    return id.schema==kPneumatic3dStateVersion &&
        (id.tier==bike::spatial::TyreTier::fast || id.tier==bike::spatial::TyreTier::detailed) &&
        fan>0 && fan<=limit && mat>0 && mat<=limit &&
        id.patch_capacity>0 && id.patch_capacity<=fan &&
        digest_valid(id.topology_fingerprint_hex) && digest_valid(id.config_fingerprint_hex) &&
        digest_valid(id.terrain_sha256_hex) && digest_valid(id.calibration_sha256_hex);
}
inline bool finite_values(const std::vector<double>& values,bool nonnegative=false) noexcept {
    return std::ranges::all_of(values,[=](double v){return std::isfinite(v)&&(!nonnegative||v>=0.);});
}
inline bool positive_zero(double v) noexcept { return std::bit_cast<std::uint64_t>(v)==0; }
inline const char* state_layout_error(const TyreState3d& s,const TyreStateIdentity3d& expected) noexcept {
    if(!identity_valid(expected))return "identity: invalid expected identity";
    if(!identity_valid(s.identity)||!identity_equal(s.identity,expected))return "identity: mismatch";
    if(s.accepted_interval< -1)return "accepted_interval: below -1";
    const auto fan=std::size_t{expected.dims.n_theta_fan}*expected.dims.n_phi;
    const auto mat=std::size_t{expected.dims.n_theta_material}*expected.dims.n_phi;
    const auto cap=static_cast<std::size_t>(expected.patch_capacity);
    double stored=0.,lost=0.;
    for(std::size_t wheel=0;wheel<2;++wheel) {
        const auto& c=s.carcass[wheel];const auto& p=s.patches[wheel];const auto& b=s.brush[wheel];
        if(c.deflection_m.size()!=mat||c.maxwell_q_n_per_m2.size()!=mat||
           c.previous_penetration_m.size()!=mat||c.active.size()!=mat)
            return "carcass: material array dimensions mismatch";
        if(!finite_values(c.deflection_m,true)||!finite_values(c.maxwell_q_n_per_m2)||
           !finite_values(c.previous_penetration_m,true)||!std::isfinite(c.material_phase_rad)||
           !std::isfinite(c.stored_energy_j)||c.stored_energy_j<0.||
           !std::isfinite(c.dissipated_j)||c.dissipated_j<0.)
            return "carcass: nonfinite value or negative energy/deflection";
        if(!std::ranges::all_of(c.active,[](auto v){return v<=1;}))return "carcass.active: expected 0 or 1";
        for(std::size_t i=0;i<mat;++i) {
            // The accepted producer writes the same penetration into both
            // lanes, and resets both when the cell leaves contact. Maxwell
            // stress is deliberately allowed to relax outside contact.
            if(!c.active[i]) {
                if(!positive_zero(c.deflection_m[i])||!positive_zero(c.previous_penetration_m[i]))
                    return "carcass: inactive penetration history must be canonical zero";
            } else if(!(c.deflection_m[i]>0.)||c.previous_penetration_m[i]!=c.deflection_m[i])
                return "carcass: active penetration history differs from accepted deflection";
        }
        if(p.capacity!=cap||p.ids.size()!=cap||p.slot_used.size()!=cap||p.element_to_patch_slot.size()!=fan)
            return "patches: dimensions mismatch";
        if(b.ux_m.size()!=cap||b.uy_m.size()!=cap||b.camber_state.size()!=cap||
           b.bristle_ux_m.size()!=mat||b.bristle_uy_m.size()!=mat||b.bristle_energy_j.size()!=mat)
            return "brush: dimensions mismatch";
        if(!finite_values(b.ux_m)||!finite_values(b.uy_m)||!finite_values(b.camber_state)||
           !finite_values(b.bristle_ux_m)||!finite_values(b.bristle_uy_m)||!finite_values(b.bristle_energy_j,true)||
           !std::isfinite(b.elastic_energy_j)||b.elastic_energy_j<0.||
           !std::isfinite(b.sliding_loss_j)||b.sliding_loss_j<0.||
           !std::isfinite(b.convective_release_j)||b.convective_release_j<0.)
            return "brush: nonfinite value or negative energy";
        std::uint64_t max_id=0;
        for(std::size_t i=0;i<cap;++i) {
            if(p.slot_used[i]>1)return "patches.slot_used: expected 0 or 1";
            if(!p.slot_used[i]) {
                if(p.ids[i]!=0||!positive_zero(b.ux_m[i])||!positive_zero(b.uy_m[i])||!positive_zero(b.camber_state[i]))
                    return "patches: inactive slot must be canonical zero";
            } else {
                if(p.ids[i]==0)return "patches.ids: active patch ID is zero";
                for(std::size_t j=0;j<i;++j)
                    if(p.slot_used[j]&&p.ids[j]==p.ids[i])return "patches.ids: duplicate active ID";
                max_id=std::max(max_id,p.ids[i]);
            }
        }
        if(p.next_id==0||p.next_id<=max_id)return "patches.next_id: must exceed active IDs";
        for(auto slot:p.element_to_patch_slot)
            if(slot!=std::numeric_limits<std::uint32_t>::max()&&(slot>=cap||!p.slot_used[slot]))
                return "patches.element_to_patch_slot: inactive or out-of-range slot";
        stored+=c.stored_energy_j+b.elastic_energy_j;
        lost+=c.dissipated_j+b.sliding_loss_j+b.convective_release_j;
        if(!std::isfinite(stored)||!std::isfinite(lost))return "energy: combined wheel totals overflow";
    }
    return nullptr;
}
inline bool validate_state_layout(const TyreState3d& s,const TyreStateIdentity3d& expected) noexcept {
    return state_layout_error(s,expected)==nullptr;
}
} // namespace bike::pneumatic3d::detail
