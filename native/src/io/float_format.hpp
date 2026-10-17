#pragma once
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <system_error>

namespace bike::io {
// The existing T12 binary64 repr spelling, shared with data-only recording
// output. Shortest round-trip digits use fixed notation for -4 <= e < 16;
// integral fixed values retain ".0". This helper has no runtime dependency.
[[nodiscard]] inline std::string repr_binary64(double value) {
    if(std::isnan(value))return "nan";
    if(std::isinf(value))return value<0.?"-inf":"inf";
    std::array<char,64> buffer{};
    const auto out=std::to_chars(buffer.begin(),buffer.end(),value,std::chars_format::general);
    if(out.ec!=std::errc{})throw std::invalid_argument("binary64 cannot be formatted");
    std::string token(buffer.begin(),out.ptr);
    const auto exponent_position=token.find_first_of("eE");
    if(exponent_position==std::string::npos) {
        if(token.find('.')==std::string::npos)token+=".0";
        return token;
    }
    std::string mantissa=token.substr(0,exponent_position);
    int exponent=0;
    auto first=token.cbegin()+static_cast<std::ptrdiff_t>(exponent_position)+1;
    const auto last=token.cend();
    if(first<last&&*first=='+')++first;
    const auto parsed=std::from_chars(std::to_address(first),std::to_address(last),exponent);
    if(parsed.ec!=std::errc{}||parsed.ptr!=std::to_address(last))throw std::invalid_argument("binary64 exponent cannot be formatted");
    if(exponent>=-4&&exponent<16) {
        const bool negative=mantissa.front()=='-';
        if(negative)mantissa.erase(0,1);
        std::string digits;for(const char c:mantissa)if(c!='.')digits+=c;
        const int point=exponent+1;
        std::string result=negative?"-":"";
        if(point<=0) {
            result+="0.";result.append(static_cast<std::size_t>(-point),'0');result+=digits;
        } else if(static_cast<std::size_t>(point)>=digits.size()) {
            result+=digits;result.append(static_cast<std::size_t>(point)-digits.size(),'0');result+=".0";
        } else {
            result+=digits.substr(0,static_cast<std::size_t>(point));result+='.';
            result+=digits.substr(static_cast<std::size_t>(point));
        }
        return result;
    }
    token[exponent_position]='e';return token;
}
} // namespace bike::io
