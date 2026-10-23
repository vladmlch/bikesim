#pragma once

#include "pneumatic3d/geometry.hpp"
#include "pneumatic3d/state.hpp"
#include "writers/writer_types.hpp"

#include <cstdint>
#include <span>
#include <string>

namespace bike::pneumatic3d {

struct CarcassSpec3d {
    double pressure_pa = 0.;
    double k_area_n_per_m3 = 0.;
    double maxwell_k_n_per_m3 = 0., maxwell_tau_s = 0.;
    double rim_clearance_m = 0., rim_stiffness_n_per_m3 = 0.;
    double initial_volume_m3 = 0.;
    bool constant_pressure = true;
    std::string provenance_status; // synthetic/measured/calibrated/inferred/provisional
};

struct NormalElement3d {
    std::uint32_t element = 0;
    double normal_load_n = 0., pressure_force_n = 0., elastic_force_n = 0.;
    double rate_force_n = 0., rim_force_n = 0.;
    double stored_energy_j = 0., dissipated_work_j = 0.;
    bool active = false;
};

CoreStatus update_carcass(const CarcassSpec3d&,
                          std::span<const ContactElement3d>, double dt_s,
                          bool advance, CarcassState3d&,
                          std::span<NormalElement3d>) noexcept;

} // namespace bike::pneumatic3d
