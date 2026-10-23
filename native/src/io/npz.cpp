// Bounded ZIP/DEFLATE reader for NumPy's pickle-free scalar/array NPZ
// format. No archive member is opened/extracted or executed. ZIP64 local
// headers written by numpy.savez are accepted through their central sizes.
#include "npy.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <iterator>
#include <limits>
#include <set>
#include <stdexcept>
#include <string_view>

namespace bike::io {
namespace {
[[noreturn]] void bad(std::string_view why){throw std::invalid_argument("NPZ: "+std::string(why));}
std::uint64_t integer(std::span<const std::byte> data,std::size_t offset,std::size_t length){
    if(offset>data.size()||length>data.size()-offset||length>8)bad("truncated ZIP structure");
    std::uint64_t value=0;for(std::size_t i=0;i<length;++i)value|=static_cast<std::uint64_t>(std::to_integer<unsigned int>(data[offset+i]))<<(8*i);return value;
}
std::uint32_t crc32(std::span<const std::byte> data){
    std::uint32_t crc=0xffffffffU;for(std::byte const byte:data){crc^=std::to_integer<std::uint32_t>(byte);for(int k=0;k<8;++k)crc=(crc>>1U)^(0xedb88320U&(0U-(crc&1U)));}return ~crc;
}
class Bits {
public:
    explicit Bits(std::span<const std::byte> data):bytes(data){}
    unsigned int get(unsigned int count){if(count>24)bad("invalid DEFLATE bit width");unsigned int out=0;for(unsigned int i=0;i<count;++i){if(position/8>=bytes.size())bad("truncated DEFLATE stream");out|=((std::to_integer<unsigned int>(bytes[position/8])>>(position%8))&1U)<<i;++position;}return out;}
    void align(){position=(position+7U)&~std::size_t{7};}
    [[nodiscard]] std::size_t consumed()const{return(position+7)/8;}
private:std::span<const std::byte> bytes;std::size_t position=0;
};
struct Huffman {
    std::array<unsigned int,16> counts{},first{},base{};
    std::vector<unsigned int> symbols;
    explicit Huffman(std::span<const unsigned int> lengths){
        for(unsigned int const length:lengths){if(length>15)bad("oversized Huffman code");if(length!=0)++counts[length];}
        int slots=1;unsigned int code=0,index=0;
        for(unsigned int len=1;len<=15;++len){slots=slots*2-static_cast<int>(counts[len]);if(slots<0)bad("oversubscribed Huffman tree");code=(code+counts[len-1])*2;first[len]=code;base[len]=index;index+=counts[len];}
        symbols.resize(index);auto next=base;for(std::size_t i=0;i<lengths.size();++i)if(lengths[i]!=0)symbols[next[lengths[i]]++]=static_cast<unsigned int>(i);
    }
    unsigned int read(Bits& bits)const{unsigned int code=0;for(unsigned int len=1;len<=15;++len){code=(code<<1U)|bits.get(1);if(code>=first[len]&&code-first[len]<counts[len])return symbols[base[len]+code-first[len]];}bad("invalid Huffman code");}
};
std::vector<std::byte> inflate(std::span<const std::byte> input,std::size_t expected){
    static constexpr std::array<unsigned int,29> lengths={3,4,5,6,7,8,9,10,11,13,15,17,19,23,27,31,35,43,51,59,67,83,99,115,131,163,195,227,258};
    static constexpr std::array<unsigned int,29> length_extra={0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,4,5,5,5,5,0};
    static constexpr std::array<unsigned int,30> distances={1,2,3,4,5,7,9,13,17,25,33,49,65,97,129,193,257,385,513,769,1025,1537,2049,3073,4097,6145,8193,12289,16385,24577};
    static constexpr std::array<unsigned int,30> distance_extra={0,0,0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8,9,9,10,10,11,11,12,12,13,13};
    Bits bits(input);std::vector<std::byte> out;out.reserve(expected);bool final=false;
    auto const push=[&](unsigned int value){if(out.size()>=expected)bad("DEFLATE output exceeds declared size");out.push_back(static_cast<std::byte>(value));};
    while(!final){
        final=bits.get(1)!=0;const unsigned int type=bits.get(2);
        if(type==0){bits.align();const auto len=bits.get(16),inverse=bits.get(16);if((len^0xffffU)!=inverse)bad("bad uncompressed block length");for(unsigned int i=0;i<len;++i)push(bits.get(8));continue;}
        if(type==3)bad("reserved DEFLATE block type");
        std::vector<unsigned int> literal_lengths,distance_lengths;
        if(type==1){literal_lengths.resize(288);for(std::size_t i=0;i<288;++i)literal_lengths[i]=i<144?8U:(i<256?9U:(i<280?7U:8U));distance_lengths.assign(32,5);}
        else{
            const unsigned int nl=bits.get(5)+257,nd=bits.get(5)+1,nc=bits.get(4)+4;
            if(nl>286||nd>32)bad("invalid dynamic Huffman dimensions");
            // RFC 1951 order; spelling it once prevents any dependency on
            // a platform's zlib/library or optional Python package.
            constexpr std::array<unsigned int,19> rfc_order={16,17,18,0,8,7,9,6,10,5,11,4,12,3,13,2,14,1,15};
            std::array<unsigned int,19> code_lengths{};for(unsigned int i=0;i<nc;++i)code_lengths[rfc_order[i]]=bits.get(3);
            const Huffman code_tree(code_lengths);std::vector<unsigned int> all;all.reserve(nl+nd);
            while(all.size()<nl+nd){const unsigned int symbol=code_tree.read(bits);if(symbol<16){all.push_back(symbol);continue;}
                unsigned int repeat=0,value=0;if(symbol==16){if(all.empty())bad("repeat without a previous code length");repeat=bits.get(2)+3;value=all.back();}else if(symbol==17)repeat=bits.get(3)+3;else if(symbol==18)repeat=bits.get(7)+11;else bad("invalid length repeat");
                if(repeat>nl+nd-all.size())bad("Huffman repeat exceeds declared code count");
                all.insert(all.end(),repeat,value);
            }
            const auto split=std::next(all.begin(),static_cast<std::ptrdiff_t>(nl));
            literal_lengths.assign(all.begin(),split);distance_lengths.assign(split,all.end());
        }
        // assign() copies the elements: literal_lengths/distance_lengths own
        // their data and nothing references `all` past the else-block.
        // cppcheck-suppress invalidLifetime
        if(literal_lengths[256]==0)bad("missing end-of-block code");
        // cppcheck-suppress invalidLifetime
        const Huffman literal(literal_lengths),distance(distance_lengths);
        for(;;){const unsigned int symbol=literal.read(bits);if(symbol<256){push(symbol);continue;}if(symbol==256)break;
            if(symbol<257||symbol>285)bad("reserved DEFLATE length symbol");
            const std::size_t index=symbol-257;
            const unsigned int length=lengths[index]+bits.get(length_extra[index]);const unsigned int ds=distance.read(bits);if(ds>=30)bad("reserved DEFLATE distance symbol");
            const unsigned int back=distances[ds]+bits.get(distance_extra[ds]);if(back==0||back>out.size()||length>expected-out.size())bad("invalid DEFLATE back-reference");
            for(unsigned int i=0;i<length;++i)out.push_back(out[out.size()-back]);
        }
    }
    if(out.size()!=expected||bits.consumed()!=input.size())bad("DEFLATE size mismatch or trailing data");
    return out;
}
std::string member_name(std::span<const std::byte> data){
    if(data.empty()||data.size()>512)bad("invalid member name");
    std::string name;name.reserve(data.size());
    for(std::byte const byte:data){const auto c=std::to_integer<unsigned char>(byte);if(c<32||c>=127||c=='/'||c=='\\'||c==':')bad("unsafe member path");name.push_back(static_cast<char>(c));}
    if(name.front()=='.'||name.find("..")!=std::string::npos||!name.ends_with(".npy"))bad("member must be a simple .npy filename");
    return name;
}
} // namespace

NpzArchive parse_npz(std::span<const std::byte> input){
    if(input.size()<22||input.size()>kMaxNumpyBytes)bad("truncated or oversized ZIP");
    std::size_t end=input.size()-22;const std::size_t stop=input.size()>65557?input.size()-65557:0;
    for(;;){if(integer(input,end,4)==0x06054b50U&&end+22+integer(input,end+20,2)==input.size())break;if(end==stop)bad("missing end-of-directory record");--end;}
    if(integer(input,end+4,2)!=0||integer(input,end+6,2)!=0)bad("multi-disk archive is not supported");
    std::uint64_t count=integer(input,end+10,2),central_size=integer(input,end+12,4),central_offset=integer(input,end+16,4);
    if(integer(input,end+8,2)!=count)bad("split directory record");
    if(count==65535||central_size==0xffffffffU||central_offset==0xffffffffU){
        if(end<20||integer(input,end-20,4)!=0x07064b50U||integer(input,end-16,4)!=0||integer(input,end-4,4)!=1)bad("invalid ZIP64 locator");
        const auto offset=integer(input,end-12,8);if(offset>input.size()||input.size()-static_cast<std::size_t>(offset)<56)bad("truncated ZIP64 directory");const auto z=static_cast<std::size_t>(offset);
        if(integer(input,z,4)!=0x06064b50U||integer(input,z+16,4)!=0||integer(input,z+20,4)!=0)bad("invalid ZIP64 directory");
        count=integer(input,z+32,8);if(integer(input,z+24,8)!=count)bad("split ZIP64 directory");central_size=integer(input,z+40,8);central_offset=integer(input,z+48,8);
    }
    if(count>kMaxNumpyMembers||central_offset>end||central_size>end-central_offset)bad("directory exceeds archive bounds");
    NpzArchive out;out.reserve(static_cast<std::size_t>(count));std::set<std::string> names;std::vector<std::pair<std::size_t,std::size_t>> extents;
    std::size_t pos=static_cast<std::size_t>(central_offset),total=0;
    for(std::uint64_t entry=0;entry<count;++entry){
        if(integer(input,pos,4)!=0x02014b50U)bad("invalid central member header");
        const auto flags=integer(input,pos+8,2),method=integer(input,pos+10,2),crc=integer(input,pos+16,4);
        std::uint64_t compressed=integer(input,pos+20,4),expanded=integer(input,pos+24,4),local=integer(input,pos+42,4);
        const std::size_t name_length=integer(input,pos+28,2),extra_length=integer(input,pos+30,2),comment_length=integer(input,pos+32,2);
        if(flags&~std::uint64_t{0x080e})bad("encrypted or unsupported ZIP flags");
        if(method!=0&&method!=8)bad("only stored/DEFLATE members are supported");
        if(integer(input,pos+34,2)!=0)bad("member lies on another disk");
        const auto mode=(integer(input,pos+38,4)>>16U)&0170000U;if(mode!=0&&mode!=0100000U)bad("nonregular/symlink archive member");
        if(pos>input.size()-46||name_length+extra_length+comment_length>input.size()-pos-46)bad("truncated member fields");
        const std::string name=member_name(input.subspan(pos+46,name_length));if(!names.insert(name).second)bad("duplicate archive member");
        if(expanded==0xffffffffU||compressed==0xffffffffU||local==0xffffffffU){
            bool found=false;std::size_t at=pos+46+name_length,limit=at+extra_length;
            while(at+4<=limit){const auto tag=integer(input,at,2);const std::size_t size=integer(input,at+2,2);at+=4;if(size>limit-at)bad("truncated extra field");if(tag==1){found=true;std::size_t z=at;auto const get=[&](){if(z+8>at+size)bad("truncated ZIP64 extra");const auto result=integer(input,z,8);z+=8;return result;};if(expanded==0xffffffffU)expanded=get();if(compressed==0xffffffffU)compressed=get();if(local==0xffffffffU)local=get();break;}at+=size;}if(!found)bad("missing ZIP64 member sizes");
        }
        if(expanded>kMaxNumpyBytes-total||compressed>kMaxNumpyBytes||expanded>std::max<std::uint64_t>(1024,compressed*10000U)||local>central_offset||central_offset-local<30)bad("member exceeds resource bounds");
        const std::size_t lp=static_cast<std::size_t>(local);
        if(integer(input,lp,4)!=0x04034b50U||integer(input,lp+6,2)!=flags||integer(input,lp+8,2)!=method)bad("local/central header mismatch");
        const std::size_t local_name=integer(input,lp+26,2),local_extra=integer(input,lp+28,2);
        const std::size_t payload=lp+30+local_name+local_extra;
        if(payload>central_offset||compressed>central_offset-payload||local_name!=name_length||member_name(input.subspan(lp+30,local_name))!=name)bad("local member extent/name mismatch");
        extents.emplace_back(lp,payload+static_cast<std::size_t>(compressed));
        const auto data=input.subspan(payload,static_cast<std::size_t>(compressed));
        std::vector<std::byte> bytes;if(method==0){if(compressed!=expanded)bad("stored member size mismatch");bytes.assign(data.begin(),data.end());}else bytes=inflate(data,static_cast<std::size_t>(expanded));
        if(crc32(bytes)!=crc)bad("member CRC mismatch");
        total+=bytes.size();out.emplace_back(name.substr(0,name.size()-4),parse_npy(bytes));
        pos+=46+name_length+extra_length+comment_length;
    }
    if(pos!=central_offset+central_size)bad("directory length mismatch");
    std::ranges::sort(extents);for(std::size_t i=1;i<extents.size();++i)if(extents[i].first<extents[i-1].second)bad("overlapping member extents");
    return out;
}

std::vector<std::byte> encode_npz(const NpzArchive &archive) {
    if (archive.size() > kMaxNumpyMembers) bad("too many archive members");
    struct Member {
        std::string name;
        std::vector<std::byte> payload;
        std::uint32_t checksum = 0;
        std::size_t offset = 0;
    };
    std::vector<Member> members;
    members.reserve(archive.size());
    std::set<std::string> names;
    std::size_t length = 22;
    for (const auto &[key, array] : archive) {
        Member member;
        member.name = key + ".npy";
        (void)member_name(std::as_bytes(std::span(member.name)));
        if (!names.insert(member.name).second) bad("duplicate archive member");
        member.payload = encode_npy(array);
        member.checksum = crc32(member.payload); // same routine as parse_npz
        const auto overhead = 30U + 46U + 2U * member.name.size();
        if (overhead > kMaxNumpyBytes - length || member.payload.size() > kMaxNumpyBytes - length - overhead)
            bad("encoded archive exceeds resource bound");
        length += overhead + member.payload.size();
        members.push_back(std::move(member));
    }
    std::vector<std::byte> result;
    result.reserve(length);
    const auto put = [&](std::uint64_t value, unsigned int width) {
        for (unsigned int i = 0; i < width; ++i)
            result.push_back(static_cast<std::byte>((value >> (8U * i)) & 0xffU));
    };
    const auto name_bytes = [&](const std::string &name) {
        const auto bytes = std::as_bytes(std::span(name));
        result.insert(result.end(), bytes.begin(), bytes.end());
    };
    for (auto &member : members) {
        member.offset = result.size();
        put(0x04034b50U, 4); put(20, 2); // local header and version required
        put(0, 2); put(0, 2); // flags and STORED method
        put(0, 2); put(33, 2); // 00:00:00, 1980-01-01; deterministic DOS epoch
        put(member.checksum, 4); put(member.payload.size(), 4); put(member.payload.size(), 4);
        put(member.name.size(), 2); put(0, 2);
        name_bytes(member.name);
        result.insert(result.end(), member.payload.begin(), member.payload.end());
    }
    const auto central_offset = result.size();
    for (const auto &member : members) {
        put(0x02014b50U, 4); put(20, 2); put(20, 2);
        put(0, 2); put(0, 2); put(0, 2); put(33, 2);
        put(member.checksum, 4); put(member.payload.size(), 4); put(member.payload.size(), 4);
        put(member.name.size(), 2); put(0, 2); put(0, 2);
        put(0, 2); put(0, 2); put(0, 4); put(member.offset, 4);
        name_bytes(member.name);
    }
    const auto central_size = result.size() - central_offset;
    put(0x06054b50U, 4); put(0, 2); put(0, 2);
    put(members.size(), 2); put(members.size(), 2);
    put(central_size, 4); put(central_offset, 4); put(0, 2);
    if (result.size() != length) bad("encoded ZIP length disagrees with directory");
    return result;
}
} // namespace bike::io
