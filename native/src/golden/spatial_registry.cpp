#include "spatial_registry.hpp"
#include "spatial_compare.hpp"
#include "../runtime/setup_values.hpp"
#include "../io/hash.hpp"
#include "../io/json.hpp"
#include "../platform/files.hpp"
#include <array>
#include <cctype>
#include <cmath>
#include <set>
#include <stdexcept>
#include <string>

namespace bike::golden {
namespace {
using namespace runtime::setupvalue;
[[noreturn]] void fail(std::string_view path,std::string_view reason) { throw std::invalid_argument(std::string("spatial manifest ")+std::string(path)+": "+std::string(reason)); }
void nonempty(const Value& v,std::string_view path) { if(io::as_string(v,path).empty())fail(path,"empty value"); }
void digest(const Value& v,bool pending,std::string_view path) {
    if(io::is_null(v)){if(!pending)fail(path,"incompatibility: missing accepted identity");return;}
    const auto& text=io::as_string(v,path);
    if(text.size()!=64)fail(path,"incompatibility: malformed SHA-256");
    for(const char c:text)if(!((c>='0'&&c<='9')||(c>='a'&&c<='f')))fail(path,"incompatibility: malformed SHA-256");
}
void unique(const Value& v,std::string_view path) {
    std::set<std::string> names;
    for(const auto& [name,unused]:io::as_object(v,path)){(void)unused;if(!names.insert(name).second)fail(path,"duplicate key");}
}
}
void validate_spatial_manifest(const io::Value& value) {
    unique(value,"root");
    if(io::as_int64(field(value,"spatial_manifest_version"),"spatial_manifest_version")!=kSpatialManifestVersion)
        fail("version","incompatibility: unsupported version");
    const auto& entries=io::as_array(field(value,"entries"),"entries");
    if(entries.empty())fail("entries","empty fixture selection");
    std::set<std::string> tests;
    const std::set<std::string> classes{"analytic","physical_reference","deterministic","golden","compatibility"};
    for(const auto& entry:entries) {
        unique(entry,"entry");const auto& name=io::as_string(field(entry,"test"),"test");
        if(!name.starts_with("spatial.")||name.find_first_not_of("abcdefghijklmnopqrstuvwxyz0123456789_.")!=std::string::npos||!tests.insert(name).second)fail("test","invalid or duplicate test");
        nonempty(field(entry,"executable"),"executable");
        const auto& task=io::as_string(field(entry,"source_task"),"source_task");
        if(task.size()!=3||task[0]!='W'||!std::isdigit(static_cast<unsigned char>(task[1]))||!std::isdigit(static_cast<unsigned char>(task[2])))fail("source_task","expected W-id");
        if(!classes.contains(io::as_string(field(entry,"classification"),"classification")))fail("classification","unknown validation class");
        const auto status=io::as_string(field(entry,"status"),"status");
        const bool pending=status=="pending_verification";
        if(!pending&&status!="accepted")fail("status","unknown status");
        for(auto key:{"config_sha256","model_sha256","terrain_sha256","calibration_sha256"})digest(field(entry,key),pending,key);
        if(io::is_null(field(entry,"provider"))){if(!pending)fail("provider","incompatibility: missing provider");}
        else nonempty(field(entry,"provider"),"provider");
        const auto& schemas=field(entry,"schema_versions");unique(schemas,"schema_versions");
        const std::array<std::pair<std::string_view,int>,6> versions{{{"sensor",2},{"catalog",2},{"container",2},{"replay",4},{"tyre_state",1},{"checkpoint",1}}};
        if(io::as_object(schemas,"schema_versions").size()!=versions.size())fail("schema_versions","incompatibility: unexpected schemas");
        for(const auto& [key,version]:versions)if(io::as_int64(field(schemas,key),key)!=version)fail(key,"incompatibility: schema differs");
        std::set<std::string> channels;
        for(const auto& channel:io::as_array(field(entry,"expected_channels"),"expected_channels")){
            const auto& key=io::as_string(channel,"channel");if(key.empty()||!channels.insert(key).second)fail("expected_channels","empty or duplicate channel");
        }
        const auto& platforms=field(entry,"platforms");unique(platforms,"platforms");
        if(io::as_object(platforms,"platforms").size()!=2)fail("platforms","both qualified platform entries required");
        for(auto platform:{"macos-arm64","linux-aarch64"}){
            const auto& p=field(platforms,platform);unique(p,platform);
            const auto state=io::as_string(field(p,"status"),"platform.status");
            if(state=="pending_verification") { if(!io::is_null(field(p,"hashes")))fail(platform,"pending fixture cannot advertise accepted hashes"); }
            else if(state=="accepted") {
                const auto& hashes=io::as_object(field(p,"hashes"),"platform.hashes");
                if(hashes.empty())fail(platform,"incompatibility: missing artifact");
                std::set<std::string> files;
                for(const auto& [path,hash]:hashes){if(path.empty()||path.starts_with('/')||path.find("..")!=std::string::npos||path.find('\\')!=std::string::npos||!files.insert(path).second)fail(platform,"unsafe or duplicate artifact path");digest(hash,false,path);}
                for(auto required:{"manifest.json","episode.npz","model.mjb"})if(!files.contains(required))fail(platform,"incompatibility: missing artifact hash");
                digest(field(p,"source_identity"),false,"source_identity");
                digest(field(p,"execution_identity"),false,"execution_identity");
                nonempty(field(p,"provider"),"platform.provider");
                nonempty(field(p,"executable"),"platform.executable");
            } else fail(platform,"unknown platform status");
        }
        const auto& comparison=field(entry,"comparison");unique(comparison,"comparison");
        if(io::as_string(field(comparison,"same_platform"),"same_platform")!="bitwise")fail("comparison","same-platform comparison must be bitwise");
        const auto& tolerances=field(comparison,"cross_platform_tolerances");unique(tolerances,"cross_platform_tolerances");
        for(const auto& [path,tolerance]:io::as_object(tolerances,"cross_platform_tolerances")) {
            if(path.empty()||path.find_first_of("/\\")!=std::string::npos)fail("comparison","invalid field path");
            unique(tolerance,path);
            if(io::as_object(tolerance,path).size()!=2)fail(path,"tolerance requires exactly atol and rtol");
            for(auto key:{"atol","rtol"}){const auto x=io::as_double(field(tolerance,key),key);if(!std::isfinite(x)||x<0.)fail(path,"invalid nonnegative finite tolerance");}
        }
    }
}
SpatialReferenceStatus inspect_spatial_reference(const io::Value& entry,
    const std::filesystem::path& directory,std::string_view current_platform) {
    if(io::as_string(field(entry,"status"),"status")=="pending_verification")return {.pending=true};
    if(current_platform!="macos-arm64"&&current_platform!="linux-aarch64")fail("platform","incompatibility: unqualified host");
    const auto manifest_path=directory/"manifest.json";
    if(!std::filesystem::is_regular_file(manifest_path)||std::filesystem::is_symlink(manifest_path))fail("artifact","incompatibility: missing artifact manifest.json");
    const auto bytes=io::read_numpy_bytes(manifest_path);
    std::string source;source.reserve(bytes.size());
    for(const std::byte byte:bytes)source.push_back(static_cast<char>(std::to_integer<unsigned char>(byte)));
    const auto manifest=io::parse_json(source);
    const auto reference_platform=io::as_string(field(manifest,"spatial_platform"),"spatial_platform");
    if(reference_platform!="macos-arm64"&&reference_platform!="linux-aarch64")fail("artifact","incompatibility: unsupported reference platform");
    const auto& platforms=field(entry,"platforms");
    const auto& reference=field(platforms,reference_platform);
    if(io::as_string(field(reference,"status"),"status")!="accepted" ||
       io::as_string(field(field(platforms,current_platform),"status"),"status")!="accepted")return {.pending=true,.platform=reference_platform};
    for(const auto& [name,hash]:io::as_object(field(reference,"hashes"),"hashes")) {
        const auto path=directory/name;
        if(!std::filesystem::is_regular_file(path)||std::filesystem::is_symlink(path))fail(name,"incompatibility: missing artifact");
        if(platform::file_sha256(path)!=io::as_string(hash,name))fail(name,"regression: artifact hash differs from reviewed reference");
    }
    for(auto key:{"config_sha256","model_sha256","terrain_sha256","calibration_sha256"})
        if(io::as_string(field(manifest,key),key)!=io::as_string(field(entry,key),key))fail(key,"incompatibility: reference identity differs from selected fixture");
    if(platform::file_sha256(directory/"model.mjb")!=io::as_string(field(manifest,"model_sha256"),"model_sha256"))
        fail("model_sha256","incompatibility: model identity does not describe the reviewed image bytes");
    for(auto key:{"source_identity","execution_identity","provider"})
        if(io::as_string(field(manifest,key),key)!=io::as_string(field(reference,key),key))fail(key,"incompatibility: reference provenance differs from reviewed platform");
    return {.pending=false,.platform=reference_platform};
}
} // namespace bike::golden
