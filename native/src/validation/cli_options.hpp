#pragma once
#include "acceptance.hpp"
#include <array>
#include <map>
#include <set>

namespace bike::validation {
// Shared by the frontend and its source regression. This only validates the
// option grammar: it neither opens input files nor starts a validation case.
class CliOptions {
public:
    explicit CliOptions(std::string_view case_id) : id_(normalize(case_id)) {
        if (std::ranges::none_of(cases(),[&](const auto& item) {
                return normalize(item.id)==id_;
            })) throw std::invalid_argument("unknown validation case: "+std::string(case_id));
    }
    void accept(std::string_view option) {
        static constexpr auto names=std::to_array<std::pair<std::string_view,std::string_view>>({
            {"--config","config"},{"--root","root"},{"--report","report"},
            {"--physics-config","physics_profile"},{"--track","track"},{"--mode","mode"},
            {"--backend","backend"},{"--transmission","transmission"},{"--output","output"},
            {"--phase","phase"},{"--dt","dt_s"},{"--timestep","dt_s"},{"--dx","dx_m"},
            {"--duration","duration_s"},{"--activation-tau","activation_tau_s"},
            {"--jobs","jobs"},{"--repeats","repeats"},{"--cases","cases"},
            {"--scenarios","names"},{"--time-steps","time_steps"},{"--road-steps","road_steps"},
            {"--station-counts","station_counts"},{"--dt-values","dt_values_s"},
            {"--stations","include_stations"}});
        const auto found=std::ranges::find_if(names,[&](const auto& pair){return pair.first==option;});
        if(found==names.end())throw std::invalid_argument("unknown argument: "+std::string(option));
        const auto field=found->second;
        // Output publication and input-location options belong to the native
        // envelope. The other fields must be consumed by this exact operation.
        bool allowed=field=="config"||field=="root"||field=="report"||field=="output";
        static const std::map<std::string_view,std::set<std::string_view>> fields{
            {"rider_replay",{"physics_profile","track","duration_s","dt_s","mode","activation_tau_s"}},
            {"rider_factor_matrix",{"physics_profile"}},
            {"drive_suspension_rig",{}},{"contact_resolution",{}},
            {"fidelity_sweep",{"cases","backend","transmission","include_stations","jobs","time_steps","road_steps","station_counts","physics_profile"}},
            {"reference_release",{"physics_profile","dt_s","dx_m","names"}},
            {"climbing_benchmark",{"physics_profile","track","mode","duration_s","repeats"}},
            {"antiwheelie_bench",{"physics_profile","phase"}},
            {"benchmarks",{"cases","dt_values_s","jobs"}},
            {"tire_characterization",{"physics_profile","dt_s","duration_s"}},
            {"radial_rig",{"dt_s","duration_s"}},{"native_radial_rig",{"dt_s","duration_s"}},
            {"wheel_inertia_rig",{"dt_s","duration_s"}},{"incline_rig",{"dt_s","duration_s"}},
            {"brake_stand",{"dt_s","duration_s"}},{"chain_locked_rig",{"dt_s","duration_s"}},
            {"load_transfer",{"dt_s"}},{"distributed_flat",{"dt_s"}},
            {"distributed_step_rig",{"dt_s","dx_m"}},{"distributed_incline_rig",{"dt_s"}},
            {"distributed_road_trace",{"dt_s","dx_m","duration_s"}},
            {"contact_case_vertices",{"dx_m"}},{"shaft_ratio",{"dt_s"}},
            {"drive_suspension",{"dt_s","duration_s","transmission"}},
            {"momentum",{"dt_s","transmission"}},{"automatic_coast",{"dt_s","duration_s"}},
            {"timing_metrics",{"dt_s"}},{"release_manifest",{"physics_profile"}},
            {"profile_identity",{"physics_profile"}}};
        if(const auto module=fields.find(id_);module!=fields.end())allowed=allowed||module->second.contains(field);
        // All discovered benchmark cases have an explicit timestep argument;
        // no other field in their configuration is consumed by that interface.
        if(field=="dt_s")for(const auto& item:cases())
            if(normalize(item.id)==id_&&item.source=="validation/benchmarks.py:89-118")allowed=true;
        if(!allowed)throw std::invalid_argument(std::string(option)+" is not an option for "+id_);
        if(!seen_.insert(std::string(field)).second)
            throw std::invalid_argument("duplicate argument for "+std::string(field)+": "+std::string(option));
    }
private:
    static std::string normalize(std::string_view name) {
        std::string result(name);std::ranges::replace(result,'-','_');return result;
    }
    std::string id_;
    std::set<std::string> seen_;
};
} // namespace bike::validation
