#include "npy.hpp"
#include "../platform/files.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cerrno>
#include <charconv>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string_view>
#include <system_error>
#include <utility>
#include <sys/stat.h>
#include <unistd.h>

namespace bike::io {
namespace {
[[noreturn]] void bad(std::string_view why){throw std::invalid_argument("NPY: "+std::string(why));}
std::size_t decimal(std::string_view text){
    std::size_t value=0;const auto last=std::to_address(text.end());
    const auto parsed=std::from_chars(text.data(),last,value); // NOLINT(bugprone-suspicious-stringview-data-usage) std::from_chars parses the (data, data+size) pointer range, not a C string
    if(text.empty()||parsed.ec!=std::errc{}||parsed.ptr!=last)bad("invalid unsigned dimension/item size");
    return value;
}
struct Dtype{char kind;std::size_t count,width;bool little;};
Dtype dtype(std::string_view name){
    if(name.size()<3||(name[0]!='<'&&name[0]!='>'&&name[0]!='='&&name[0]!='|'))bad("unsupported dtype descriptor");
    const char kind=name[1];const std::size_t count=decimal(name.substr(2));
    if((kind=='f'&&(count!=4&&count!=8))||((kind=='i'||kind=='u')&&(count!=1&&count!=2&&count!=4&&count!=8))||
       (kind=='b'&&count!=1)||(kind!='f'&&kind!='i'&&kind!='u'&&kind!='b'&&kind!='S'&&kind!='U'))
        bad("unsupported dtype (objects/pickle/structured records are forbidden)");
    if(count>kMaxNumpyBytes/(kind=='U'?4U:1U))bad("dtype exceeds byte limit");
    const std::size_t width=count*(kind=='U'?4U:1U);
    if(name[0]=='|'&&width>1&&kind!='S')bad("multi-byte numeric dtype requires byte order");
    return{.kind=kind,.count=count,.width=width,.little=name[0]=='<'||(name[0]=='='&&std::endian::native==std::endian::little)||name[0]=='|'};
}
std::uint64_t bits(std::span<const std::byte> data,bool little){
    std::uint64_t value=0;
    for(std::size_t i=0;i<data.size();++i){const std::size_t at=little?data.size()-1-i:i;value=(value<<8U)|std::to_integer<unsigned int>(data[at]);}
    return value;
}
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
void append_bits(std::vector<std::byte>& bytes,std::uint64_t value,std::size_t count){
    for(std::size_t i=0;i<count;++i){bytes.push_back(static_cast<std::byte>(value&255U));value>>=8U;}
}
std::size_t stored_index(const NpyArray& array,std::size_t logical){
    if(!array.fortran_order||array.shape.size()<2)return logical;
    std::size_t stored=0,multiplier=1;
    std::vector<std::size_t> coordinates(array.shape.size());
    for(std::size_t dim=array.shape.size();dim-->0;){coordinates[dim]=logical%array.shape[dim];logical/=array.shape[dim];}
    for(std::size_t dim=0;dim<array.shape.size();++dim){stored+=coordinates[dim]*multiplier;multiplier*=array.shape[dim];}
    return stored;
}
std::span<const std::byte> cell(const NpyArray& array,std::size_t i){return std::span{array.bytes}.subspan(stored_index(array,i)*array.item_size(),array.item_size());}
void utf8(std::string& out,std::uint32_t c){
    if(c>0x10ffffU||(c>=0xd800U&&c<=0xdfffU))bad("invalid Unicode scalar");
    if(c<128U)out.push_back(static_cast<char>(c));
    else if(c<2048U){out.push_back(static_cast<char>(0xc0U|(c>>6U)));out.push_back(static_cast<char>(0x80U|(c&63U)));}
    else if(c<65536U){out.push_back(static_cast<char>(0xe0U|(c>>12U)));out.push_back(static_cast<char>(0x80U|((c>>6U)&63U)));out.push_back(static_cast<char>(0x80U|(c&63U)));}
    else{out.push_back(static_cast<char>(0xf0U|(c>>18U)));out.push_back(static_cast<char>(0x80U|((c>>12U)&63U)));out.push_back(static_cast<char>(0x80U|((c>>6U)&63U)));out.push_back(static_cast<char>(0x80U|(c&63U)));}
}
std::vector<std::uint32_t> codepoints(std::string_view s){
    std::vector<std::uint32_t> out;
    for(std::size_t i=0;i<s.size();){const auto first=static_cast<unsigned char>(s[i++]);std::uint32_t cp=first;unsigned int more=0;
        if(first>=0xc2U&&first<=0xdfU){cp=first&31U;more=1;}else if(first>=0xe0U&&first<=0xefU){cp=first&15U;more=2;}else if(first>=0xf0U&&first<=0xf4U){cp=first&7U;more=3;}else if(first>=128U)bad("invalid UTF-8 string");
        const unsigned int count=more;for(unsigned int next=0;next<more;++next){if(i==s.size())bad("truncated UTF-8 string");const auto c=static_cast<unsigned char>(s[i++]);if((c&0xc0U)!=0x80U)bad("invalid UTF-8 continuation");cp=(cp<<6U)|(c&63U);}
        if((count==1&&cp<128U)||(count==2&&cp<2048U)||(count==3&&cp<65536U)||cp>0x10ffffU||(cp>=0xd800U&&cp<=0xdfffU))bad("invalid UTF-8 scalar");
        out.push_back(cp);
    }return out;
}
class Header {
public:
    explicit Header(std::string_view input):source(input){}
    void space(){while(pos<source.size()&&(source[pos]==' '||source[pos]=='\n'||source[pos]=='\r'||source[pos]=='\t'))++pos;}
    bool take(char c){space();if(pos<source.size()&&source[pos]==c){++pos;return true;}return false;}
    void need(char c){if(!take(c))bad("malformed header literal");}
    std::string string(){space();if(pos==source.size()||(source[pos]!='\''&&source[pos]!='"'))bad("expected header string");const char quote=source[pos++];std::string result;while(pos<source.size()&&source[pos]!=quote){if(source[pos]=='\\'||static_cast<unsigned char>(source[pos])<32U)bad("escaped header strings are not supported");result.push_back(source[pos++]);}if(pos==source.size())bad("truncated header string");++pos;return result;}
    bool boolean(){space();for(auto const text:{std::string_view("True"),std::string_view("False")})if(source.substr(pos,text.size())==text){pos+=text.size();return text=="True";}bad("expected boolean literal");}
    std::vector<std::size_t> shape(){need('(');std::vector<std::size_t> out;if(take(')'))return out;for(;;){space();const std::size_t start=pos;while(pos<source.size()&&source[pos]>='0'&&source[pos]<='9')++pos;out.push_back(decimal(source.substr(start,pos-start)));if(out.size()>16)bad("too many array dimensions");if(take(')')){if(out.size()==1)bad("singleton shape requires a tuple comma");break;}need(',');if(take(')'))break;}return out;}
    NpyArray parse(){NpyArray out;need('{');bool has_dtype=false,has_order=false,has_shape=false;for(;;){const auto key=string();need(':');if(key=="descr"&&!has_dtype){out.dtype=string();has_dtype=true;}else if(key=="fortran_order"&&!has_order){out.fortran_order=boolean();has_order=true;}else if(key=="shape"&&!has_shape){out.shape=shape();has_shape=true;}else bad("unknown or duplicate header field");if(take('}'))break;need(',');if(take('}'))break;}space();if(pos!=source.size()||!has_dtype||!has_order||!has_shape)bad("incomplete header");return out;}
private:std::string_view source;std::size_t pos=0;
};
} // namespace

std::size_t NpyArray::elements() const{
    if(shape.size()>16)bad("too many array dimensions");
    std::size_t count=1;for(std::size_t const x:shape){if(x>kMaxNumpyBytes|| (x!=0&&count>kMaxNumpyBytes/x))bad("array dimensions exceed bound");count*=x;}return count;
}
std::size_t NpyArray::item_size() const{return io::dtype(dtype).width;}
void NpyArray::validate() const{const auto width=item_size(),count=elements();if(width!=0&&count>kMaxNumpyBytes/width)bad("array exceeds byte bound");if(bytes.size()!=width*count)bad("payload length does not match shape and dtype");}
std::vector<double> NpyArray::doubles(bool allow_nonfinite) const{
    validate();const auto type=io::dtype(dtype);if(type.kind=='U'||type.kind=='S')bad("numeric dtype required");
    std::vector<double> out(elements());for(std::size_t i=0;i<out.size();++i){const auto raw=bits(cell(*this,i),type.little);double value=0.;
        if(type.kind=='f')value=type.width==8?std::bit_cast<double>(raw):static_cast<double>(std::bit_cast<float>(static_cast<std::uint32_t>(raw)));
        else if(type.kind=='i'){std::uint64_t extended=raw;if(type.width<8&&(raw&(1ULL<<(type.width*8-1)))!=0)extended|=(~0ULL)<<(type.width*8);value=static_cast<double>(std::bit_cast<std::int64_t>(extended));}
        else value=static_cast<double>(raw);
        if(!allow_nonfinite&&!std::isfinite(value))bad("nonfinite numeric array");
        out[i]=value;
    }return out;
}
std::vector<std::int64_t> NpyArray::integers() const{
    validate();const auto type=io::dtype(dtype);if(type.kind!='i'&&type.kind!='u'&&type.kind!='b')bad("integer dtype required");
    std::vector<std::int64_t> out(elements());for(std::size_t i=0;i<out.size();++i){auto value=bits(cell(*this,i),type.little);if(type.kind=='i'&&type.width<8&&(value&(1ULL<<(type.width*8-1)))!=0)value|=(~0ULL)<<(type.width*8);if(type.kind!='i'&&value>static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))bad("integer exceeds signed 64-bit domain");out[i]=std::bit_cast<std::int64_t>(value);}return out;
}
std::vector<bool> NpyArray::booleans() const{validate();if(io::dtype(dtype).kind!='b')bad("boolean dtype required");std::vector<bool> out(elements());for(std::size_t i=0;i<out.size();++i){const auto value=bits(cell(*this,i),true);if(value>1)bad("invalid boolean representation");out[i]=value!=0;}return out;}
std::vector<std::string> NpyArray::strings() const{
    validate();const auto type=io::dtype(dtype);if(type.kind!='U'&&type.kind!='S')bad("string dtype required");std::vector<std::string> out(elements());
    for(std::size_t i=0;i<out.size();++i){const auto data=cell(*this,i);if(type.kind=='S'){for(std::byte const b:data)out[i].push_back(static_cast<char>(std::to_integer<unsigned char>(b)));while(!out[i].empty()&&out[i].back()=='\0')out[i].pop_back();(void)codepoints(out[i]);}
        else{std::vector<std::uint32_t> chars;chars.reserve(type.count);
            for(std::size_t j=0;j<type.count;++j)chars.push_back(static_cast<std::uint32_t>(bits(data.subspan(j*4,4),type.little)));
            while(!chars.empty()&&chars.back()==0)chars.pop_back();
            for(auto const c:chars)utf8(out[i],c);}}
    return out;
}
NpyArray parse_npy(std::span<const std::byte> data){
    constexpr std::array<unsigned char,6> magic{0x93U,'N','U','M','P','Y'};
    if(data.size()<10||data.size()>kMaxNumpyBytes)bad("truncated or oversized file");
    for(std::size_t i=0;i<6;++i)if(std::to_integer<unsigned char>(data[i])!=magic[i])bad("invalid magic");
    const unsigned int major=std::to_integer<unsigned int>(data[6]),minor=std::to_integer<unsigned int>(data[7]);
    if(major<1||major>3||minor!=0)bad("unsupported NPY version");
    const std::size_t prefix=major==1?10:12;if(data.size()<prefix)bad("truncated header");
    const auto length=static_cast<std::size_t>(bits(data.subspan(8,prefix-8),true));
    if(length>65536||length>data.size()-prefix)bad("truncated or oversized header");
    std::string header;header.reserve(length);for(std::byte const b:data.subspan(prefix,length))header.push_back(static_cast<char>(std::to_integer<unsigned char>(b)));
    if(header.empty()||header.back()!='\n')bad("header must end with newline");
    NpyArray array=Header(header).parse();const auto width=array.item_size(),count=array.elements();
    if(width!=0&&count>kMaxNumpyBytes/width)bad("array size exceeds bound");
    if(width*count!=data.size()-prefix-length)bad("payload size mismatch");
    array.bytes.assign(data.begin()+static_cast<std::ptrdiff_t>(prefix+length),data.end());array.validate();return array;
}
std::vector<std::byte> encode_npy(const NpyArray& array){
    array.validate();std::string header="{'descr': '"+array.dtype+"', 'fortran_order': "+(array.fortran_order?"True":"False")+", 'shape': (";
    for(std::size_t i=0;i<array.shape.size();++i){if(i!=0)header+=", ";header+=std::to_string(array.shape[i]);}if(array.shape.size()==1)header+=',';header+="), }";
    while((10+header.size()+1)%64!=0)header+=' ';
    header+='\n';if(header.size()>65535)bad("encoded header exceeds NPY1 bound");
    std::vector<std::byte> out{std::byte{0x93},std::byte{'N'},std::byte{'U'},std::byte{'M'},std::byte{'P'},std::byte{'Y'},std::byte{1},std::byte{0}};
    append_bits(out,header.size(),2);for(char c:header)out.push_back(static_cast<std::byte>(static_cast<unsigned char>(c)));out.insert(out.end(),array.bytes.begin(),array.bytes.end());return out;
}
std::vector<std::byte> read_numpy_bytes(const std::filesystem::path& file){
    const auto handle=platform::open_regular(file);struct stat info{};if(::fstat(handle.get(),&info)!=0)throw std::system_error(errno,std::generic_category(),"state fstat");
    if(info.st_size<0||std::cmp_greater(info.st_size,kMaxNumpyBytes))bad("file exceeds read bound");
    std::vector<std::byte> bytes(static_cast<std::size_t>(info.st_size));std::size_t at=0;
    while(at<bytes.size()){const auto got=::read(handle.get(),std::span{bytes}.subspan(at).data(),bytes.size()-at);if(got<0){if(errno==EINTR)continue;throw std::system_error(errno,std::generic_category(),"state read");}if(got==0)bad("file truncated while reading");at+=static_cast<std::size_t>(got);}
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-do-while) mirrors the reference control flow
    std::byte extra{};ssize_t count = 0;do{count=::read(handle.get(),&extra,1);}while(count<0&&errno==EINTR);if(count<0)throw std::system_error(errno,std::generic_category(),"state read");if(count!=0)bad("file changed while reading");return bytes;
}
NpyArray double_array(std::span<const double> values,std::vector<std::size_t> shape){NpyArray out{.dtype="<f8",.shape=std::move(shape),.fortran_order=false,.bytes={}};out.bytes.reserve(values.size()*8);for(double const value:values)append_bits(out.bytes,std::bit_cast<std::uint64_t>(value),8);out.validate();return out;}
NpyArray integer_array(std::span<const std::int64_t> values,std::vector<std::size_t> shape){NpyArray out{.dtype="<i8",.shape=std::move(shape),.fortran_order=false,.bytes={}};out.bytes.reserve(values.size()*8);for(auto const value:values)append_bits(out.bytes,std::bit_cast<std::uint64_t>(value),8);out.validate();return out;}
NpyArray bool_array(const std::vector<bool>& values,std::vector<std::size_t> shape){NpyArray out{.dtype="|b1",.shape=std::move(shape),.fortran_order=false,.bytes={}};out.bytes.reserve(values.size());for(bool const value:values)out.bytes.push_back(value?std::byte{1}:std::byte{0});out.validate();return out;}
NpyArray string_array(std::span<const std::string> values,std::vector<std::size_t> shape){std::vector<std::vector<std::uint32_t>> chars;std::size_t width=1;for(const auto& value:values){chars.push_back(codepoints(value));width=std::max(width,chars.back().size());}NpyArray out{.dtype="<U"+std::to_string(width),.shape=std::move(shape),.fortran_order=false,.bytes={}};for(const auto& value:chars)for(std::size_t i=0;i<width;++i)append_bits(out.bytes,i<value.size()?value[i]:0,4);out.validate();return out;}
} // namespace bike::io
