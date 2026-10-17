// T24 authored controls: bounded NumPy/ZIP readers, durable native publication,
// cache reuse, strict explicit files and t=0 transfer. No test or compiler was
// executed while these sources were implemented.
#include "support/migration_test.hpp"
#include "diag.hpp"
#include "initialization/artifact_identity.hpp"
#include "initialization/setup_owner.hpp"
#include "initialization/state_file.hpp"
#include "initialization/source_json.hpp"
#include "io/json.hpp"
#include "io/npy.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {
namespace init=bike::initialization;
namespace io=bike::io;

class Directory {
public:
    Directory():path(std::filesystem::temp_directory_path()/
        ("bike-t24-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()))) {
        if(!std::filesystem::create_directory(path))throw std::runtime_error("T24 isolated directory creation failed");
    }
    ~Directory(){std::error_code ignored;std::filesystem::remove_all(path,ignored);}
    Directory(const Directory&)=delete;
    Directory& operator=(const Directory&)=delete;
    Directory(Directory&&)=delete;
    Directory& operator=(Directory&&)=delete;
    std::filesystem::path path;
};
void little(std::vector<std::byte>& out,std::uint64_t value,std::size_t width) {
    for(std::size_t i=0;i<width;++i){out.push_back(static_cast<std::byte>(value&255U));value>>=8U;}
}
void bytes(std::vector<std::byte>& out,std::string_view text) {
    for(char c:text)out.push_back(static_cast<std::byte>(static_cast<unsigned char>(c)));
}
std::uint32_t crc(std::span<const std::byte> data) {
    std::uint32_t result=0xffffffffU;
    for(auto cell:data){result^=std::to_integer<std::uint32_t>(cell);for(int i=0;i<8;++i)result=(result>>1U)^((result&1U)?0xedb88320U:0U);}
    return ~result;
}
std::vector<std::byte> fixed_deflate(std::span<const std::byte> data) {
    // Independent RFC1951 fixed-Huffman literal encoder. This exercises the
    // reader's bit order without relying on its own inverse or a Python tool.
    std::vector<std::byte> out;std::size_t bit=0;
    auto put=[&](unsigned int value,unsigned int width) {
        for(unsigned int i=0;i<width;++i){if(bit%8==0)out.push_back(std::byte{0});if(((value>>i)&1U)!=0)out[bit/8]|=static_cast<std::byte>(1U<<(bit%8));++bit;}
    };
    auto code=[&](unsigned int symbol) {
        unsigned int value=0,width=0;
        if(symbol<=143){value=0x30U+symbol;width=8;}
        else if(symbol<=255){value=0x190U+symbol-144U;width=9;}
        else {value=symbol-256U;width=7;}
        unsigned int reversed=0;for(unsigned int i=0;i<width;++i)reversed=(reversed<<1U)|((value>>i)&1U);
        put(reversed,width);
    };
    put(1,1);put(1,2);
    for(auto cell:data)code(std::to_integer<unsigned int>(cell));
    code(256);return out;
}
std::vector<std::byte> archive(std::string_view name,std::span<const std::byte> data,
                               bool compressed,bool zip64_local=false) {
    const auto payload=compressed?fixed_deflate(data):std::vector<std::byte>(data.begin(),data.end());
    std::vector<std::byte> out;
    little(out,0x04034b50,4);little(out,zip64_local?45U:20U,2);little(out,0,2);
    little(out,compressed?8U:0U,2);little(out,0,4);little(out,crc(data),4);
    little(out,zip64_local?0xffffffffU:payload.size(),4);little(out,zip64_local?0xffffffffU:data.size(),4);
    little(out,name.size(),2);little(out,zip64_local?20U:0U,2);bytes(out,name);
    if(zip64_local){little(out,1,2);little(out,16,2);little(out,data.size(),8);little(out,payload.size(),8);}
    out.insert(out.end(),payload.begin(),payload.end());
    const auto central=out.size();
    little(out,0x02014b50,4);little(out,20,2);little(out,20,2);little(out,0,2);
    little(out,compressed?8U:0U,2);little(out,0,4);little(out,crc(data),4);
    little(out,payload.size(),4);little(out,data.size(),4);little(out,name.size(),2);
    little(out,0,2);little(out,0,2);little(out,0,2);little(out,0,2);little(out,0,4);little(out,0,4);bytes(out,name);
    const auto central_size=out.size()-central;
    little(out,0x06054b50,4);little(out,0,2);little(out,0,2);little(out,1,2);little(out,1,2);
    little(out,central_size,4);little(out,central,4);little(out,0,2);return out;
}
bool same_bits(double a,double b){return std::bit_cast<std::uint64_t>(a)==std::bit_cast<std::uint64_t>(b);}

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")

[[gnu::noinline]] void numpy_and_zip() {
    const std::array<double,4> expected{1.,-0.,-2.5,std::numeric_limits<double>::quiet_NaN()};
    const auto array=io::double_array(expected,{2,2});
    const auto encoded=io::encode_npy(array);const auto decoded=io::parse_npy(encoded).doubles(true);
    for(std::size_t i=0;i<expected.size();++i)bt::require(same_bits(decoded[i],expected[i]),"NPY preserves binary64 and signed-zero/NaN payload bits");
    bt::invalid([&]{(void)io::parse_npy(encoded).doubles();});
    auto big=io::double_array(std::array{1.25,-2.5},{2});big.dtype=">f8";
    for(std::size_t i=0;i<2;++i)std::ranges::reverse(std::span{big.bytes}.subspan(i*8,8));
    bt::require(io::parse_npy(io::encode_npy(big)).doubles()==std::vector<double>{1.25,-2.5},"big-endian NPY is decoded explicitly");
    auto fortran=io::double_array(std::array{1.,4.,2.,5.,3.,6.},{2,3});fortran.fortran_order=true;
    bt::require(fortran.doubles()==std::vector<double>{1.,2.,3.,4.,5.,6.},"Fortran storage maps to logical row-major values");
    const std::array<std::string,2> strings{"front", "колесо"};
    bt::require(io::parse_npy(io::encode_npy(io::string_array(strings,{2}))).strings()==std::vector<std::string>(strings.begin(),strings.end()),"NPY UCS4 material keys round-trip through UTF-8");
    bt::require(io::parse_npy(io::encode_npy(io::bool_array({true,false},{2}))).booleans()==std::vector<bool>{true,false},"boolean state flags retain their dtype");
    auto invalid=encoded;invalid.pop_back();bt::invalid([&]{(void)io::parse_npy(invalid);});
    auto oversized=array;oversized.shape={io::kMaxNumpyBytes,2};bt::invalid([&]{oversized.validate();});
    io::NpyArray object{"|O8",{1},false,std::vector<std::byte>(8)};bt::invalid([&]{object.validate();});
    for(bool compressed:{false,true})for(bool zip64:{false,true}) {
        const auto zip=archive("qpos.npy",encoded,compressed,zip64);
        const auto restored=io::parse_npz(zip);
        bt::require(restored.size()==1 && restored[0].first=="qpos" && restored[0].second.bytes==array.bytes,
                    "stored/fixed-DEFLATE and NumPy ZIP64-local NPZ preserve array bytes");
    }
    const auto unsafe=archive("../qpos.npy",encoded,false);bt::invalid([&]{(void)io::parse_npz(unsafe);});
    auto corrupt=archive("qpos.npy",encoded,false);corrupt[40]^=std::byte{1};bt::invalid([&]{(void)io::parse_npz(corrupt);});
    auto truncated=archive("qpos.npy",encoded,true);truncated.resize(truncated.size()-3);bt::invalid([&]{(void)io::parse_npz(truncated);});
}

[[gnu::noinline]] void durable_file() {
    Directory temporary;
    init::StateFile state;state.metadata=io::Value{io::Value::Object{}};
    state.arrays.emplace_back("qpos",io::double_array(std::array{1.,-0.},{2}));
    state.arrays.emplace_back("fingerprint_sha256",io::string_array(std::array{std::string(64,'a')},{}));
    const auto path=temporary.path/"pose.npz";init::write_state_file(path,state);
    const auto restored=init::read_state_file(path);
    bt::require(restored.schema_version==1 && init::fingerprint(restored)==std::string(64,'a') &&
                init::state_array(restored,"qpos").bytes==state.arrays[0].second.bytes,
                "native typed envelope is durable and content-detected independent of suffix");
    const auto previous=bt::text(path);auto invalid=state;invalid.arrays[0].second.shape={3};
    bt::invalid([&]{init::write_state_file(path,invalid);});
    bt::require(bt::text(path)==previous,"invalid state serialization leaves the prior durable file untouched");
    const auto link=temporary.path/"symlink.npz";std::filesystem::create_symlink(path,link);
    bool rejected=false;try{init::write_state_file(link,state);}catch(const std::exception&){rejected=true;}
    bt::require(rejected && bt::text(path)==previous,"state publication never follows a leaf symlink");
    bt::invalid([&]{(void)init::PhysicalInitialState("{\"schema_version\":2}");});
}

[[gnu::noinline]] void recorded_identity() {
    bt::require(init::detail::source_json(io::parse_json(R"({"s":"é𝄞"})"))==R"({"s":"\u00e9\ud834\udd1e"})",
                "historical JSON identity preserves Python ensure_ascii surrogate escaping");
    io::Value native=io::parse_json("{\"backend\":\"native\",\"runtime_schema\":1,\"python_source_sha256\":\""+std::string(64,'a')+"\",\"native_source_sha256\":\""+std::string(64,'b')+"\",\"extension_sha256\":\""+std::string(64,'c')+"\",\"mujoco_library_sha256\":\""+std::string(64,'d')+"\",\"mujoco_version\":\"3.12.0\",\"build_context\":{\"build_type\":\"Release\",\"compiler_id\":\"Clang\",\"compiler_version\":\"21\",\"numerical_flags\":\"-ffp-contract=off\"}}");
    init::validate_recorded_execution(native);
    bt::require(true,"complete recorded native identity is accepted as data");
    auto wrong=native;auto& object=std::get<io::Value::Object>(wrong.data);object.emplace_back("recorded_path",io::Value{std::string("/untrusted/module")});
    bt::invalid([&]{init::validate_recorded_execution(wrong);});
    wrong=native;for(auto& [name,value]:std::get<io::Value::Object>(wrong.data))if(name=="runtime_schema")value=io::Value{true};
    bt::invalid([&]{init::validate_recorded_execution(wrong);});
    auto python=native;for(auto& [name,value]:std::get<io::Value::Object>(python.data)) {
        if(name=="backend")value=io::Value{std::string("python")};
        else if(name=="native_source_sha256"||name=="extension_sha256"||name=="mujoco_library_sha256"||name=="build_context")value=io::Value{};
    }
    init::validate_recorded_execution(python);
    bt::require(true,"historical Python execution record needs no Python import");
}

[[gnu::noinline]] void cache_and_transfer() {
    Directory temporary;
    auto request=init::read_setup_request(bt::fixture("requests/welded_flat.toml"));
    request.model.physics->equilibrium_cache_enabled=true;
    request.model.physics->equilibrium_state_file.reset();request.model.physics->initial_speed_mps=0.;
    request.cache_directory=temporary.path;
    auto original=init::make_setup(request);const auto first=init::settle(*original);
    bt::require(!first.cache_hit && first.residual_qacc<=.05,"first native equilibrium is physically accepted");
    const auto initial=original->capture_initial_state();
    auto reused=init::make_setup(request);const auto second=init::settle(*reused);
    bt::require(second.cache_hit && second.steps==first.steps,"the derived-name native cache restores validated metadata");
    std::vector<std::filesystem::path> paths;for(const auto& entry:std::filesystem::directory_iterator(temporary.path))if(entry.path().extension()==".npz")paths.push_back(entry.path());
    bt::require(paths.size()==1,"one resolved fingerprint names one cache entry");
    auto explicit_request=request;explicit_request.model.physics->equilibrium_cache_enabled=false;
    explicit_request.model.physics->equilibrium_state_file=paths.front().string();
    auto explicit_owner=init::make_setup(explicit_request);
    bt::require(init::settle(*explicit_owner).cache_hit,"explicit matching state is rechecked against measured acceleration");
    auto corrupt=init::read_state_file(paths.front());
    for(auto& [name,array]:corrupt.arrays)if(name=="qpos")array=io::double_array(std::array{0.},{1});
    init::write_state_file(paths.front(),corrupt);
    auto invalid=init::make_setup(explicit_request);
    bt::invalid([&]{(void)init::settle(*invalid);});
    bt::invalid([&]{(void)invalid->seal_bootstrap();});
    auto transfer=request;transfer.model.physics->equilibrium_cache_enabled=false;
    transfer.model.physics->timestep_s*=.5;transfer.initial_state=initial;
    auto target=init::make_setup(transfer);const auto transferred=init::settle(*target);
    bt::require(transferred.initial_state_sha256==initial.sha256() &&
                transferred.initial_state_target_residual_qacc.has_value(),
                "timestep transfer retains source identity and measures target residual");
    transfer.start_x_m+=1.;auto incompatible=init::make_setup(transfer);
    bt::invalid([&]{(void)init::settle(*incompatible);});
    bt::invalid([&]{(void)incompatible->seal_bootstrap();});
}
} // namespace

int main(){return bt::run([]{numpy_and_zip();durable_file();recorded_identity();cache_and_transfer();});}
NATIVE_DIAG_POP
