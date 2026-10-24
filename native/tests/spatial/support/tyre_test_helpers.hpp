#pragma once

#include "support/migration_test.hpp"
#include "pneumatic3d/brush.hpp"
#include "pneumatic3d/carcass.hpp"
#include "pneumatic3d/kernel_math.hpp"

#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace spatial_tests::tyre {
using namespace bike::pneumatic3d;
inline CarcassState3d carcass_state(std::size_t n) {
    CarcassState3d s;
    s.deflection_m.assign(n,0.); s.maxwell_q_n_per_m2.assign(n,0.);
    s.previous_penetration_m.assign(n,0.); s.active.assign(n,0);
    return s;
}
inline BrushState3d brush_state(std::size_t patches,std::size_t cells=0) {
    BrushState3d s;
    s.ux_m.assign(patches,0.); s.uy_m.assign(patches,0.); s.camber_state.assign(patches,0.);
    s.bristle_ux_m.assign(cells,0.); s.bristle_uy_m.assign(cells,0.); s.bristle_energy_j.assign(cells,0.);
    return s;
}
inline ContactElement3d element(std::uint32_t cell,double d,double area=.01) {
    ContactElement3d e;
    e.material_cell=cell; e.valid=true; e.ray_status=bike::terrain::QueryStatus3d::ok;
    e.penetration_m=d; e.geometric_area_m2=area; e.projected_area_m2=area;
    e.normal_world={0.,0.,1.}; e.longitudinal_world={1.,0.,0.}; e.lateral_world={0.,1.,0.};
    return e;
}
inline Patch3d patch(double load=400.,double area=.01) {
    Patch3d p;
    p.id=1; p.valid=true; p.normal_load_n=load; p.area_m2=area;
    p.normal_world={0.,0.,1.}; p.longitudinal_world={1.,0.,0.}; p.lateral_world={0.,1.,0.};
    p.force_world_n={0.,0.,load}; return p;
}
inline void append(std::vector<std::uint64_t>& out,std::span<const double> v) {
    for (const double x:v) out.push_back(std::bit_cast<std::uint64_t>(x));
}
inline std::vector<std::uint64_t> bytes(const CarcassState3d& s) {
    std::vector<std::uint64_t> b;
    append(b,s.deflection_m); append(b,s.maxwell_q_n_per_m2); append(b,s.previous_penetration_m);
    for (const auto x:s.active) b.push_back(x);
    b.push_back(std::bit_cast<std::uint64_t>(s.material_phase_rad));
    b.push_back(std::bit_cast<std::uint64_t>(s.stored_energy_j));
    b.push_back(std::bit_cast<std::uint64_t>(s.dissipated_j)); return b;
}
inline std::vector<std::uint64_t> bytes(const BrushState3d& s) {
    std::vector<std::uint64_t> b;
    append(b,s.ux_m); append(b,s.uy_m); append(b,s.camber_state);
    append(b,s.bristle_ux_m); append(b,s.bristle_uy_m); append(b,s.bristle_energy_j);
    b.push_back(std::bit_cast<std::uint64_t>(s.elastic_energy_j));
    b.push_back(std::bit_cast<std::uint64_t>(s.sliding_loss_j));
    b.push_back(std::bit_cast<std::uint64_t>(s.convective_release_j)); return b;
}
inline std::vector<std::uint64_t> bytes(const PatchState3d& s) {
    std::vector<std::uint64_t> b=s.ids;
    for (const auto x:s.element_to_patch_slot) b.push_back(x);
    for (const auto x:s.slot_used) b.push_back(x);
    b.push_back(s.next_id); b.push_back(s.capacity); return b;
}
} // namespace spatial_tests::tyre
