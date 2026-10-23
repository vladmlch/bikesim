// Internal spec reflection is an exhaustive transcription of dataclass fields.
#pragma once
#include "terrain/generators.hpp"
#include "terrain/detail.hpp"
#include "io/json.hpp"
#include "io/hash.hpp"
#include "terrain/spec3d.hpp"
#include "terrain/source3d.hpp"
#include "spatial/config.hpp"
#include <filesystem>
#include <memory>

#define BIKE_TERRAIN_SPEC_FIELDS(X) \
 X(length_m) X(grade_max) X(grade_peak) X(grade_peak_prob) \
 X(roughness_amp_m) X(roughness_len_m) X(roughness_corr_m) \
 X(bump_height_m) X(bump_length_m) X(edge_height_m) X(drop_height_m) X(roots_len_m) \
 X(base_surfaces) X(section_surfaces) X(feature_types) X(section_len_m) \
 X(n_roughness) X(n_features) X(n_surface_sections) X(lead_in_m) X(lead_out_m)
#define BIKE_ALPINE_SPEC_FIELDS(X) \
 X(length_m) X(lead_in_m) X(lead_out_m) X(transition_m) \
 X(ramp_len_m) X(ramp_grade) X(shelf_len_m) X(shelf_grade) X(wall_grade) X(n_walls) \
 X(counter_len_m) X(counter_grade) X(counter_prob) X(base_surface) X(n_scree) \
 X(scree_len_m) X(scree_ramp_bias) X(scree_amp_m) X(scree_corr_m) X(n_wet) X(wet_len_m) \
 X(background_amp_m) X(background_corr_m) X(n_boulders) X(boulder_height_m) X(boulder_len_m) \
 X(n_rock_gardens) X(rock_len_m) X(rock_amp_m) X(rock_corr_m) X(n_water_bars) \
 X(water_bar_depth_m) X(water_bar_len_m) X(n_sharp_edges) X(sharp_edge_height_m) \
 X(single_contact_safe) X(min_contact_radius_m) X(max_kink_deg) X(contact_window_m)

namespace bike::terrain::detail {
// Cold file dispatch. Relative base_track paths resolve against the declaring
// genspec, never the process cwd. The returned immutable owner is shared by
// startup, runtime and render; no terrain file is read on a warm query.
Track3dSpec load_track3d_spec(const std::filesystem::path&);
ProfileExtrusion3d load_profile_extrusion3d(const std::filesystem::path&,
    double corridor_half_width_m, double resolution_m = DEFAULT_RESOLUTION_M);
std::shared_ptr<const Terrain3dSource> load_terrain3d_source(
    const bike::spatial::TerrainRequest3d&);

inline void read_field(double& out,const bike::io::Value& value,std::string_view name){out=bike::io::as_double(value,name);}
inline void read_field(std::int64_t& out,const bike::io::Value& value,std::string_view name){out=bike::io::as_int64(value,name);}
inline void read_field(bool& out,const bike::io::Value& value,std::string_view name){out=bike::io::as_bool(value,name);}
inline void read_field(std::string& out,const bike::io::Value& value,std::string_view name){out=bike::io::as_string(value,name);}
inline void read_field(Range& out,const bike::io::Value& value,std::string_view name){
    const auto& a=bike::io::as_array(value,name);if(a.size()!=2)throw std::invalid_argument(std::string(name)+": expected a low/high pair");
    out={bike::io::as_double(a[0],name),bike::io::as_double(a[1],name)};
}
inline void read_field(CountRange& out,const bike::io::Value& value,std::string_view name){
    const auto& a=bike::io::as_array(value,name);if(a.size()!=2)throw std::invalid_argument(std::string(name)+": expected a low/high pair");
    out={bike::io::as_int64(a[0],name),bike::io::as_int64(a[1],name)};
}
inline void read_field(std::vector<std::string>& out,const bike::io::Value& value,std::string_view name){
    out.clear();for(const auto& v:bike::io::as_array(value,name))out.push_back(bike::io::as_string(v,name));
}
inline bike::io::Value field_value(double value){return number(value);}
inline bike::io::Value field_value(std::int64_t value){return integer(value);}
inline bike::io::Value field_value(bool value){return {value};}
inline bike::io::Value field_value(const std::string& value){return {value};}
inline bike::io::Value field_value(Range value){return {bike::io::Value::Array{number(value[0]),number(value[1])}};}
inline bike::io::Value field_value(CountRange value){return {bike::io::Value::Array{integer(value[0]),integer(value[1])}};}
inline bike::io::Value field_value(const std::vector<std::string>& value){
    bike::io::Value::Array out;for(const auto& v:value)out.push_back({v});return {std::move(out)};
}
inline std::string source_spec_hash(const bike::io::Value& value){
    // json.dumps(sort_keys=True) has spaces after comma/colon. All strings
    // in validated generator specs are ASCII registry names; no alternate
    // Unicode/float codec enters this source hash boundary.
    const auto compact=bike::io::canonical_json(value);std::string text;text.reserve(compact.size()*2);
    bool quoted=false,escaped=false;
    for(char c:compact){text.push_back(c);if(quoted){if(escaped)escaped=false;else if(c=='\\')escaped=true;else if(c=='"')quoted=false;}else if(c=='"')quoted=true;else if(c==','||c==':')text.push_back(' ');}
    return bike::io::sha256(std::as_bytes(std::span(text.data(),text.size())));
}
} // namespace bike::terrain::detail
