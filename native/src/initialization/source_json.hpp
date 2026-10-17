#pragma once

#include "../io/json.hpp"

#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>

namespace bike::initialization::detail {
// json.dumps(sort_keys=True, separators=(',', ':'), allow_nan=False)
// defaults to ensure_ascii=True. The shared native codec intentionally uses
// UTF-8, so historical fingerprints and initial-state contracts add this
// source-specific escaping layer without changing native file serialization.
[[nodiscard]] inline std::string source_json(const io::Value& value) {
    const auto utf8=io::canonical_json(value);
    constexpr std::string_view hex="0123456789abcdef";
    std::string out;out.reserve(utf8.size());
    auto unit=[&](std::uint32_t code) {
        out+="\\u";
        for(int shift:{12,8,4,0})out.push_back(hex[(code>>static_cast<unsigned int>(shift))&15U]);
    };
    for(std::size_t i=0;i<utf8.size();) {
        const auto first=static_cast<unsigned char>(utf8[i++]);
        if(first<127U){out.push_back(static_cast<char>(first));continue;}
        if(first==127U){unit(first);continue;}
        std::uint32_t code=0;unsigned int count=0;
        if(first>=0xc2U&&first<=0xdfU){code=first&31U;count=1;}
        else if(first>=0xe0U&&first<=0xefU){code=first&15U;count=2;}
        else if(first>=0xf0U&&first<=0xf4U){code=first&7U;count=3;}
        else throw std::invalid_argument("invalid UTF-8 in source JSON");
        for(unsigned int j=0;j<count;++j) {
            if(i==utf8.size())throw std::invalid_argument("truncated UTF-8 in source JSON");
            const auto next=static_cast<unsigned char>(utf8[i++]);
            if((next&0xc0U)!=0x80U)throw std::invalid_argument("invalid UTF-8 continuation in source JSON");
            code=(code<<6U)|(next&63U);
        }
        if((count==1&&code<128U)||(count==2&&code<2048U)||(count==3&&code<65536U)||
           code>0x10ffffU||(code>=0xd800U&&code<=0xdfffU))
            throw std::invalid_argument("invalid Unicode scalar in source JSON");
        if(code<=0xffffU)unit(code);
        else {code-=0x10000U;unit(0xd800U+(code>>10U));unit(0xdc00U+(code&1023U));}
    }
    return out;
}
} // namespace bike::initialization::detail
