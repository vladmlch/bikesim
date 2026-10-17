#include "state_file.hpp"
#include "../io/hash.hpp"
#include "../io/json.hpp"
#include "../platform/files.hpp"

#include <atomic>
#include <cerrno>
#include <cstdio>
#include <new>
#include <set>
#include <stdexcept>
#include <system_error>
#include <unistd.h>

namespace bike::initialization {
namespace {
const io::Value& field(const io::Value::Object& object,std::string_view key){const auto* found=io::find_member(object,key);if(!found)throw std::invalid_argument("state file: missing "+std::string(key));return *found;}
io::Value array_document(const io::NpyArray& array){
    array.validate();io::Value::Array shape;for(std::size_t width:array.shape)shape.emplace_back(io::Number{std::to_string(width)});
    return io::Value{io::Value::Object{{"dtype",io::Value{array.dtype}},{"shape",io::Value{std::move(shape)}},{"fortran_order",io::Value{array.fortran_order}},{"bytes",io::Value{io::Bytes{array.bytes}}}}};
}
io::Value document(const StateFile& state){
    if(state.schema_version!=1)throw std::invalid_argument("state file: native writer requires schema version 1");
    io::Value::Object arrays;std::set<std::string> names;std::size_t total=0;
    for(const auto& [name,array]:state.arrays){if(name.empty()||!names.insert(name).second)throw std::invalid_argument("state file: duplicate/empty array name");if(array.bytes.size()>io::kMaxNumpyBytes-total)throw std::invalid_argument("state file exceeds byte bound");total+=array.bytes.size();arrays.emplace_back(name,array_document(array));}
    return io::Value{io::Value::Object{{"format",io::Value{std::string("bike.equilibrium.v1")}},{"schema_version",io::Value{io::Number{"1"}}},{"metadata",state.metadata},{"arrays",io::Value{std::move(arrays)}}}};
}
StateFile decode(const io::Value& value){
    const auto& root=io::as_object(value,"state file");
    if(root.size()!=4||io::as_string(field(root,"format"),"format")!="bike.equilibrium.v1"||io::as_int64(field(root,"schema_version"),"schema_version")!=1)
        throw std::invalid_argument("state file: unsupported format/schema");
    StateFile state;state.metadata=field(root,"metadata");const auto& arrays=io::as_object(field(root,"arrays"),"arrays");
    if(arrays.size()>io::kMaxNumpyMembers)throw std::invalid_argument("state file: too many arrays");
    std::size_t total=0;
    for(const auto& [name,data]:arrays){
        const auto& item=io::as_object(data,"array");if(item.size()!=4)throw std::invalid_argument("state file: unknown array member");io::NpyArray array;
        array.dtype=io::as_string(field(item,"dtype"),"dtype");array.fortran_order=io::as_bool(field(item,"fortran_order"),"fortran_order");
        for(const auto& dimension:io::as_array(field(item,"shape"),"shape")){const auto n=io::as_int64(dimension,"shape dimension");if(n<0||static_cast<std::uint64_t>(n)>io::kMaxNumpyBytes)throw std::invalid_argument("state file: oversized dimension");array.shape.push_back(static_cast<std::size_t>(n));}
        const auto* bytes=std::get_if<io::Bytes>(&field(item,"bytes").data);if(!bytes)throw std::invalid_argument("state file: typed bytes required");
        if(bytes->value.size()>io::kMaxNumpyBytes-total)throw std::invalid_argument("state file: byte limit exceeded");
        total+=bytes->value.size();array.bytes=bytes->value;array.validate();state.arrays.emplace_back(name,std::move(array));
    }return state;
}
} // namespace
const io::NpyArray& state_array(const StateFile& state,std::string_view name){
    for(const auto& [key,value]:state.arrays)if(key==name)return value;
    throw std::invalid_argument("equilibrium state: missing "+std::string(name));
}
StateFile read_state_file(const std::filesystem::path& file){
    try{
        const auto bytes=io::read_numpy_bytes(file);
        if(bytes.size()>=4&&bytes[0]==std::byte{'P'}&&bytes[1]==std::byte{'K'})return StateFile{0,io::Value{},io::parse_npz(bytes)};
        std::string text;text.reserve(bytes.size());for(auto byte:bytes)text.push_back(static_cast<char>(std::to_integer<unsigned char>(byte)));
        return decode(io::parse_recording_json(text));
    }catch(const std::bad_alloc&){throw;}
    catch(const std::exception& error){throw std::invalid_argument("equilibrium state file "+file.string()+": "+error.what());}
}
void write_state_file(const std::filesystem::path& file,const StateFile& state){
    const std::string json=io::recording_json(document(state));
    if(json.size()>io::kMaxNumpyBytes)throw std::invalid_argument("equilibrium state file exceeds byte bound");
    const auto parent=file.has_parent_path()?file.parent_path():std::filesystem::path(".");std::filesystem::create_directories(parent);
    // Existing explicit state files can be replaced like the source; a leaf
    // symlink/device is never adopted. rename replaces the name atomically.
    std::error_code ec;const auto status=std::filesystem::symlink_status(file,ec);
    if(!ec&&std::filesystem::exists(status)){const auto check=platform::open_regular(file);(void)check;}
    static std::atomic<unsigned long long> serial{0};std::filesystem::path temporary;platform::FileHandle handle;
    for(int attempt=0;attempt<64;++attempt){temporary=parent/(".equilibrium-"+std::to_string(::getpid())+"-"+std::to_string(serial.fetch_add(1,std::memory_order_relaxed))+".tmp");
        try{handle=platform::open_exclusive(temporary);break;}
        catch(const std::system_error& error){if(error.code().value()==EEXIST&&attempt<63)continue;throw;}
    }
    try{platform::write_all(handle,std::as_bytes(std::span{json}));platform::sync(handle);handle.close();if(::rename(temporary.c_str(),file.c_str())!=0)throw std::system_error(errno,std::generic_category(),"equilibrium state publication");platform::sync_directory(parent);}
    catch(...){std::error_code ignore;std::filesystem::remove(temporary,ignore);throw;}
}
std::string fingerprint(const StateFile& state){
    for(const auto& [name,array]:state.arrays)if(name=="fingerprint_sha256"){
        const auto values=array.strings();if(values.size()!=1||values[0].size()!=64||values[0].find_first_not_of("0123456789abcdef")!=std::string::npos)throw std::invalid_argument("invalid state fingerprint");return values[0];
    }
    if(state.schema_version==1){const auto text=io::canonical_json(document(state));return io::sha256(std::as_bytes(std::span{text}));}
    io::Sha256 digest;for(const auto& [name,array]:state.arrays){digest.update(std::as_bytes(std::span{name}));const auto encoded=io::encode_npy(array);digest.update(encoded);}
    const auto value=digest.finish();static constexpr std::string_view digits="0123456789abcdef";std::string out;out.reserve(64);for(auto b:value){out.push_back(digits[b>>4]);out.push_back(digits[b&15]);}return out;
}
} // namespace bike::initialization
