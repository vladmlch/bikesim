// W27. Explicit little-endian state image; never memcpy a struct or pointer.
#include "state.hpp"
#include "state_detail.hpp"
#include "runtime/setup_values.hpp"
#include "runtime/value_readers.hpp"

#include <array>
#include <bit>
#include <limits>
#include <stdexcept>
#include <utility>

namespace bike::pneumatic3d {
namespace {
using runtime::setupvalue::Value;
using runtime::setupvalue::Object;
using runtime::setupvalue::integer;
using runtime::setupvalue::text;
constexpr std::size_t max_bytes=std::size_t{512}*1024U*1024U;

std::size_t encoded_size(const TyreStateIdentity3d& id) {
    if(!detail::identity_valid(id))throw std::invalid_argument("tyre_state.identity: invalid schema, dimensions or digest");
    const auto fan=std::uint64_t{id.dims.n_theta_fan}*id.dims.n_phi;
    const auto mat=std::uint64_t{id.dims.n_theta_material}*id.dims.n_phi;
    const auto cap=std::uint64_t{id.patch_capacity};
    // 4 magic + 6 uint32 + 4 length-prefixed 64-byte identities; two
    // carcass/patch/brush records; final int64 accepted interval.
    const auto bytes=4U+6U*4U+4U*(4U+64U)+2U*(49U*mat+33U*cap+4U*fan+60U)+8U;
    if(bytes>max_bytes)throw std::invalid_argument("tyre_state.sizes: state exceeds 512 MiB");
    // uint64_t and size_t are the same type on LP64 Linux; a cast there is
    // rejected by GCC's -Wuseless-cast under -Werror.
    return bytes;
}
struct Writer {
    std::vector<std::byte> out;
    void u8(std::uint8_t v){out.push_back(static_cast<std::byte>(v));}
    void u32(std::uint32_t v){for(unsigned k=0;k<4;++k)u8(static_cast<std::uint8_t>(v>>(8U*k)));}
    void u64(std::uint64_t v){for(unsigned k=0;k<8;++k)u8(static_cast<std::uint8_t>(v>>(8U*k)));}
    void f64(double v){u64(std::bit_cast<std::uint64_t>(v));}
    void string(const std::string& s){u32(static_cast<std::uint32_t>(s.size()));for(const char c:s)u8(static_cast<std::uint8_t>(c));}
    void reals(const std::vector<double>& v){for(const double x:v)f64(x);}
};
struct Reader {
    std::span<const std::byte> bytes;std::size_t at=0;
    void require(std::size_t n,std::string_view field) const {
        if(n>bytes.size()-at)throw std::invalid_argument("tyre_state."+std::string(field)+": truncated payload");
    }
    std::uint8_t u8(){require(1,"byte");return std::to_integer<std::uint8_t>(bytes[at++]);}
    std::uint32_t u32(){std::uint32_t v=0;for(unsigned k=0;k<4;++k)v|=std::uint32_t{u8()}<<(8U*k);return v;}
    std::uint64_t u64(){std::uint64_t v=0;for(unsigned k=0;k<8;++k)v|=std::uint64_t{u8()}<<(8U*k);return v;}
    double f64(){return std::bit_cast<double>(u64());}
    std::string string(std::string_view path){
        const auto n=u32();if(n!=64)throw std::invalid_argument("tyre_state."+std::string(path)+": expected 64 hex bytes");
        require(n,path);std::string s;s.reserve(n);for(std::uint32_t i=0;i<n;++i)s.push_back(static_cast<char>(u8()));return s;
    }
    void reals(std::vector<double>& v,std::size_t n){require(n*8U,"array");v.resize(n);for(auto& x:v)x=f64();}
};
vread::Dict object(const Value& v,std::string path) {
    const auto& fields=vread::mapping(vread::Node{&v},path);
    for(std::size_t i=0;i<fields.size();++i)for(std::size_t j=0;j<i;++j)
        if(fields[i].first==fields[j].first)vread::invalid(path+"."+fields[i].first,"duplicate key");
    return {.value=&fields,.path=std::move(path)};
}
std::uint32_t uint32(const vread::Dict& d,std::string_view key) {
    const auto n=vread::integer(vread::field(d,key),d.child(key));
    if(n<0||std::cmp_greater(n,std::numeric_limits<std::uint32_t>::max()))vread::invalid(d.child(key),"expected uint32");
    return static_cast<std::uint32_t>(n);
}
}

std::vector<std::byte> encode_tyre_state(const TyreState3d& state) {
    if(const auto* why=detail::state_layout_error(state,state.identity))
        throw std::invalid_argument("tyre_state."+std::string(why));
    Writer w;w.out.reserve(encoded_size(state.identity));
    for(const auto c:std::array{'B','K','S','3'})w.u8(static_cast<std::uint8_t>(c));
    const auto& id=state.identity;
    w.u32(id.schema);w.u32(static_cast<std::uint32_t>(id.tier));
    w.u32(id.dims.n_theta_fan);w.u32(id.dims.n_phi);w.u32(id.dims.n_theta_material);w.u32(id.patch_capacity);
    w.string(id.topology_fingerprint_hex);w.string(id.config_fingerprint_hex);
    w.string(id.terrain_sha256_hex);w.string(id.calibration_sha256_hex);
    for(std::size_t k=0;k<2;++k) {
        const auto& c=state.carcass[k];const auto& p=state.patches[k];const auto& b=state.brush[k];
        w.reals(c.deflection_m);w.reals(c.maxwell_q_n_per_m2);w.reals(c.previous_penetration_m);
        for(const auto x:c.active)w.u8(x);
        w.f64(c.material_phase_rad);w.f64(c.stored_energy_j);w.f64(c.dissipated_j);
        for(const auto x:p.ids)w.u64(x);
        for(const auto x:p.element_to_patch_slot)w.u32(x);
        for(const auto x:p.slot_used)w.u8(x);
        w.u64(p.next_id);w.u32(p.capacity);
        w.reals(b.ux_m);w.reals(b.uy_m);w.reals(b.camber_state);
        w.reals(b.bristle_ux_m);w.reals(b.bristle_uy_m);w.reals(b.bristle_energy_j);
        w.f64(b.elastic_energy_j);w.f64(b.sliding_loss_j);w.f64(b.convective_release_j);
    }
    w.u64(std::bit_cast<std::uint64_t>(state.accepted_interval));
    return std::move(w.out);
}

std::expected<TyreState3d,std::string> decode_tyre_state(
    std::span<const std::byte> bytes,const TyreStateIdentity3d& expected) {
    try {
        Reader r{.bytes=bytes};
        for(const auto c:std::array{'B','K','S','3'})
            if(r.u8()!=static_cast<std::uint8_t>(c))throw std::invalid_argument("tyre_state.magic: expected BKS3");
        TyreState3d s;auto& id=s.identity;
        id.schema=r.u32();if(id.schema!=1)throw std::invalid_argument("tyre_state.schema: unsupported version");
        const auto tier=r.u32();if(tier>1)throw std::invalid_argument("tyre_state.tier: unknown enum");
        id.tier=static_cast<bike::spatial::TyreTier>(tier);
        id.dims={.n_theta_fan=r.u32(),.n_phi=r.u32(),.n_theta_material=r.u32()};id.patch_capacity=r.u32();
        id.topology_fingerprint_hex=r.string("topology_fingerprint");
        id.config_fingerprint_hex=r.string("config_fingerprint");
        id.terrain_sha256_hex=r.string("terrain_sha256");
        id.calibration_sha256_hex=r.string("calibration_sha256");
        const auto required=encoded_size(id);
        if(bytes.size()!=required)throw std::invalid_argument("tyre_state.sizes: truncated or trailing payload");
        if(!detail::identity_valid(expected)||!detail::identity_equal(id,expected))
            throw std::invalid_argument("tyre_state.identity: different runtime dimensions, tier or digests");
        const auto mat=std::size_t{id.dims.n_theta_material}*id.dims.n_phi;
        const auto fan=std::size_t{id.dims.n_theta_fan}*id.dims.n_phi;
        const auto cap=static_cast<std::size_t>(id.patch_capacity);
        for(std::size_t k=0;k<2;++k) {
            auto& c=s.carcass[k];auto& p=s.patches[k];auto& b=s.brush[k];
            r.reals(c.deflection_m,mat);r.reals(c.maxwell_q_n_per_m2,mat);r.reals(c.previous_penetration_m,mat);
            c.active.resize(mat);for(auto& x:c.active)x=r.u8();
            c.material_phase_rad=r.f64();c.stored_energy_j=r.f64();c.dissipated_j=r.f64();
            p.ids.resize(cap);for(auto& x:p.ids)x=r.u64();
            p.element_to_patch_slot.resize(fan);for(auto& x:p.element_to_patch_slot)x=r.u32();
            p.slot_used.resize(cap);for(auto& x:p.slot_used)x=r.u8();p.next_id=r.u64();p.capacity=r.u32();
            r.reals(b.ux_m,cap);r.reals(b.uy_m,cap);r.reals(b.camber_state,cap);
            r.reals(b.bristle_ux_m,mat);r.reals(b.bristle_uy_m,mat);r.reals(b.bristle_energy_j,mat);
            b.elastic_energy_j=r.f64();b.sliding_loss_j=r.f64();b.convective_release_j=r.f64();
        }
        s.accepted_interval=std::bit_cast<std::int64_t>(r.u64());
        if(r.at!=bytes.size())throw std::invalid_argument("tyre_state.payload: trailing bytes");
        if(const auto* why=detail::state_layout_error(s,expected))
            throw std::invalid_argument("tyre_state."+std::string(why));
        return s;
    } catch(const std::invalid_argument& error) { return std::unexpected(std::string(error.what())); }
}

Value tyre_identity_to_wire(const TyreStateIdentity3d& id) {
    if(!detail::identity_valid(id))throw std::invalid_argument("tyre_state.identity: invalid identity");
    return Value{Object{{"schema",integer(id.schema)}, {"tier",text(bike::spatial::to_string(id.tier))},
        {"dims",Value{Object{{"n_theta_fan",integer(id.dims.n_theta_fan)}, {"n_phi",integer(id.dims.n_phi)},
            {"n_theta_material",integer(id.dims.n_theta_material)}}}}, {"patch_capacity",integer(id.patch_capacity)},
        {"topology_fingerprint",text(id.topology_fingerprint_hex)}, {"config_fingerprint",text(id.config_fingerprint_hex)},
        {"terrain_sha256",text(id.terrain_sha256_hex)}, {"calibration_sha256",text(id.calibration_sha256_hex)}}};
}
TyreStateIdentity3d tyre_identity_from_wire(const Value& value) {
    const auto d=object(value,"tyre_state.identity");
    vread::exact(d,vread::keys("schema","tier","dims","patch_capacity","topology_fingerprint",
                              "config_fingerprint","terrain_sha256","calibration_sha256"));
    TyreStateIdentity3d id;id.schema=uint32(d,"schema");id.patch_capacity=uint32(d,"patch_capacity");
    const auto tier=bike::spatial::parse_tyre_tier(vread::string(vread::field(d,"tier"),d.child("tier")));
    if(!tier)vread::invalid(d.child("tier"),"unknown tyre tier");
    id.tier=*tier;
    const auto dims=object(*vread::field(d,"dims").value,d.child("dims"));
    vread::exact(dims,vread::keys("n_theta_fan","n_phi","n_theta_material"));
    id.dims={.n_theta_fan=uint32(dims,"n_theta_fan"),.n_phi=uint32(dims,"n_phi"),
        .n_theta_material=uint32(dims,"n_theta_material")};
    id.topology_fingerprint_hex=vread::string(vread::field(d,"topology_fingerprint"),d.child("topology_fingerprint"));
    id.config_fingerprint_hex=vread::string(vread::field(d,"config_fingerprint"),d.child("config_fingerprint"));
    id.terrain_sha256_hex=vread::string(vread::field(d,"terrain_sha256"),d.child("terrain_sha256"));
    id.calibration_sha256_hex=vread::string(vread::field(d,"calibration_sha256"),d.child("calibration_sha256"));
    if(!detail::identity_valid(id))vread::invalid(d.path,"invalid dimensions, schema or digest");
    return id;
}

std::string base64_canonical(std::span<const std::byte> bytes) {
    constexpr std::string_view alphabet="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    if(bytes.size()>max_bytes)throw std::invalid_argument("tyre_state.base64: payload exceeds 512 MiB");
    std::string out;out.reserve(4U*((bytes.size()+2U)/3U));
    for(std::size_t i=0;i<bytes.size();i+=3) {
        const auto a=std::to_integer<std::uint32_t>(bytes[i]);
        const auto b=i+1<bytes.size()?std::to_integer<std::uint32_t>(bytes[i+1]):0U;
        const auto c=i+2<bytes.size()?std::to_integer<std::uint32_t>(bytes[i+2]):0U;
        out.push_back(alphabet[a>>2U]);out.push_back(alphabet[((a&3U)<<4U)|(b>>4U)]);
        out.push_back(i+1<bytes.size()?alphabet[((b&15U)<<2U)|(c>>6U)]:'=');
        out.push_back(i+2<bytes.size()?alphabet[c&63U]:'=');
    }
    return out;
}
std::expected<std::vector<std::byte>,std::string> base64_decode_strict(std::string_view text64) {
    if(text64.size()%4U!=0||text64.size()>4U*((max_bytes+2U)/3U))
        return std::unexpected("tyre_state.base64: invalid length");
    const auto digit=[](char c)->unsigned {
        if(c>='A'&&c<='Z')return static_cast<unsigned>(c-'A');
        if(c>='a'&&c<='z')return static_cast<unsigned>(c-'a'+26);
        if(c>='0'&&c<='9')return static_cast<unsigned>(c-'0'+52);
        if(c=='+')return 62U;
        if(c=='/')return 63U;
        return ~0U;
    };
    std::vector<std::byte> out;out.reserve(text64.size()/4U*3U);
    for(std::size_t i=0;i<text64.size();i+=4U) {
        const auto a=digit(text64[i]),b=digit(text64[i+1]);
        const bool pc=text64[i+2]=='=',pd=text64[i+3]=='=';
        const auto c=pc?0U:digit(text64[i+2]),d=pd?0U:digit(text64[i+3]);
        if(a>63U||b>63U||c>63U||d>63U||(pc&&!pd)||((pc||pd)&&i+4U!=text64.size())||
           (pc&&(b&15U)!=0U)||(!pc&&pd&&(c&3U)!=0U))
            return std::unexpected("tyre_state.base64: invalid alphabet or noncanonical padding");
        out.push_back(static_cast<std::byte>((a<<2U)|(b>>4U)));
        if(!pc)out.push_back(static_cast<std::byte>(((b&15U)<<4U)|(c>>2U)));
        if(!pd)out.push_back(static_cast<std::byte>(((c&3U)<<6U)|d));
    }
    if(out.size()>max_bytes)return std::unexpected("tyre_state.base64: payload exceeds 512 MiB");
    return out;
}
} // namespace bike::pneumatic3d
