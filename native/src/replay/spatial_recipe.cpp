#include "spatial_recipe.hpp"
#include "../runtime/spatial_checkpoint.hpp"
#include "../runtime/setup_values.hpp"
#include <algorithm>
#include <set>
#include <stdexcept>

namespace bike::replay {
namespace {
using namespace runtime::setupvalue;
void digest(std::string_view value,std::string_view path) {
    if(value.size()!=64 || !std::ranges::all_of(value,[](char c){return (c>='0'&&c<='9')||(c>='a'&&c<='f');}))
        throw std::invalid_argument(std::string(path)+": expected lowercase SHA-256");
}
void validate(const SpatialRecipe4& r) {
    if(r.contract!=kSpatialReplayContract) throw std::invalid_argument("spatial replay: unsupported contract");
    if(r.terrain_kind!=terrain::Terrain3dKind::grid && r.terrain_kind!=terrain::Terrain3dKind::profile_extrusion)
        throw std::invalid_argument("spatial replay: unknown terrain kind");
    digest(r.terrain_sha256_hex,"spatial replay terrain");digest(r.execution_identity_sha256,"spatial replay execution");
    (void)io::as_object(r.config_spatial,"spatial replay config");
    const auto initial=runtime::spatial::decode_checkpoint(r.checkpoint_initial);
    const auto final=runtime::spatial::decode_checkpoint(r.checkpoint_final);
    for(const auto* c : {&initial,&final}) {
        if(io::as_string(field(c->identity,"terrain_sha256"),"terrain_sha256")!=r.terrain_sha256_hex)
            throw std::invalid_argument("spatial replay: checkpoint terrain identity differs");
        if(io::as_string(field(c->identity,"solver"),"solver")!=r.execution_identity_sha256)
            throw std::invalid_argument("spatial replay: checkpoint execution identity differs");
    }
    if(io::canonical_json(initial.identity)!=io::canonical_json(final.identity))
        throw std::invalid_argument("spatial replay: initial/final identities differ");
}
}
io::Value write_spatial_recipe(const SpatialRecipe4& r) {
    validate(r);
    return Value{Object{{"replay_contract_version",integer(r.contract)},
        {"terrain_kind",text(r.terrain_kind==terrain::Terrain3dKind::grid?"grid":"profile_extrusion")},
        {"terrain_sha256",text(r.terrain_sha256_hex)},{"config_spatial",r.config_spatial},
        {"checkpoint_initial",r.checkpoint_initial},{"checkpoint_final",r.checkpoint_final},
        {"execution_identity_sha256",text(r.execution_identity_sha256)}}};
}
SpatialRecipe4 read_spatial_recipe(const io::Value& value) {
    const auto& object=io::as_object(value,"spatial replay recipe");
    const std::set<std::string> expected{"replay_contract_version","terrain_kind","terrain_sha256","config_spatial","checkpoint_initial","checkpoint_final","execution_identity_sha256"};
    std::set<std::string> keys;for(const auto& [key,unused]:object){(void)unused;if(!keys.insert(key).second)throw std::invalid_argument("spatial replay: duplicate field");}
    if(keys!=expected)throw std::invalid_argument("spatial replay: exact recipe key set differs");
    if(io::as_int64(field(value,"replay_contract_version"),"replay contract")!=4)throw std::invalid_argument("spatial replay: unsupported contract");
    const auto& kind=io::as_string(field(value,"terrain_kind"),"terrain kind");
    if(kind!="grid"&&kind!="profile_extrusion")throw std::invalid_argument("spatial replay: unknown terrain kind");
    SpatialRecipe4 out{.contract=kSpatialReplayContract,.terrain_kind=kind=="grid"?terrain::Terrain3dKind::grid:terrain::Terrain3dKind::profile_extrusion,
        .terrain_sha256_hex=io::as_string(field(value,"terrain_sha256"),"terrain_sha256"),.config_spatial=field(value,"config_spatial"),
        .checkpoint_initial=field(value,"checkpoint_initial"),.checkpoint_final=field(value,"checkpoint_final"),
        .execution_identity_sha256=io::as_string(field(value,"execution_identity_sha256"),"execution_identity_sha256")};
    validate(out);return out;
}
} // namespace bike::replay
