#include "artifact_identity.hpp"

#include <algorithm>
#include <array>
#include <stdexcept>
#include <string>
#include <string_view>

namespace bike::initialization {
namespace {
bool nonempty(const io::Value& value) {
    const auto* text=std::get_if<std::string>(&value.data);
    return text&&text->find_first_not_of(" \t\r\n\f\v")!=std::string::npos;
}
template<std::size_t N>
bool exact(const io::Value::Object& object,const std::array<std::string_view,N>& keys) {
    return object.size()==keys.size()&&std::ranges::all_of(keys,[&](auto key){return io::find_member(object,key)!=nullptr;});
}
}
void validate_recorded_execution(const io::Value& value) {
    constexpr std::array<std::string_view,8> keys{"backend","runtime_schema","python_source_sha256","native_source_sha256","extension_sha256","mujoco_version","mujoco_library_sha256","build_context"};
    const auto* object=std::get_if<io::Value::Object>(&value.data);
    if(!object||!exact(*object,keys))throw std::invalid_argument("execution must contain the exact schema-2 identity fields");
    auto const field=[&](std::string_view key)->const io::Value&{return *io::find_member(*object,key);};
    const auto* backend=std::get_if<std::string>(&field("backend").data);
    if(!backend||(*backend!="python"&&*backend!="native"))throw std::invalid_argument("unsupported recorded execution backend");
    const auto* schema=std::get_if<io::Number>(&field("runtime_schema").data);
    if(!schema||schema->lexeme!="1")throw std::invalid_argument("unsupported recorded runtime schema");
    if(!nonempty(field("mujoco_version")))throw std::invalid_argument("execution.mujoco_version must be a nonempty string");
    for(std::string_view const name:{"python_source_sha256","native_source_sha256","extension_sha256","mujoco_library_sha256"}) {
        if(*backend=="python"&&name!="python_source_sha256") {
            if(!io::is_null(field(name)))throw std::invalid_argument("Python execution."+std::string(name)+" must be null");
        } else {
            const auto* hash=std::get_if<std::string>(&field(name).data);
            if(!hash||hash->size()!=64||hash->find_first_not_of("0123456789abcdef")!=std::string::npos)
                throw std::invalid_argument("execution."+std::string(name)+" must be a lowercase SHA-256 digest");
        }
    }
    const auto& context=field("build_context");
    if(*backend=="python") {
        if(!io::is_null(context))throw std::invalid_argument("Python execution.build_context must be null");
    } else {
        constexpr std::array<std::string_view,4> fields{"build_type","compiler_id","compiler_version","numerical_flags"};
        const auto* build=std::get_if<io::Value::Object>(&context.data);
        if(!build||!exact(*build,fields)||!std::ranges::all_of(*build,[](const auto& pair){return nonempty(pair.second);}))
            throw std::invalid_argument("native execution.build_context is incomplete");
    }
}
} // namespace bike::initialization
