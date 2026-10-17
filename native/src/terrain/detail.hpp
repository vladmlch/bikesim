#pragma once
#include "geometry/specs.hpp"
#include "io/value.hpp"
#include "numeric_sum.hpp"
#include "random/numpy_compat.hpp"
#include "terrain/types.hpp"
#include <algorithm>
#include <charconv>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace bike::terrain::detail {
namespace gd = bike::geometry::detail;
inline void finite(double value,std::string_view path) {
    if (!std::isfinite(value)) throw std::invalid_argument(std::string(path)+": must be finite");
}
inline void positive(double value,std::string_view path) {
    finite(value,path);
    if (!(value>0.)) throw std::invalid_argument(std::string(path)+": must be positive");
}
inline void nonnegative(double value,std::string_view path) {
    finite(value,path);
    if (value<0.) throw std::invalid_argument(std::string(path)+": must be nonnegative");
}
inline std::size_t count(double value,std::string_view path) {
    finite(value,path);
    // Every terrain buffer has a representable, bounded element count before
    // floating-to-integer conversion or allocation. 2^31-1 is also MuJoCo's
    // hfield dimension domain, rather than an implementation-defined cast.
    if (value<0. || value>static_cast<double>(std::numeric_limits<int>::max()))
        throw std::invalid_argument(std::string(path)+": sample count outside native domain");
    return static_cast<std::size_t>(value);
}
inline std::vector<double> arange(double start,double stop,double step) {
    finite(start,"arange.start"); finite(stop,"arange.stop"); positive(step,"arange.step");
    const auto n=count(std::max(0.,std::ceil((stop-start)/step)),"arange");
    std::vector<double> out(n);
    if (n==0) return out;
    out[0]=start;
    if(n==1) return out;
    out[1]=start+step;
    // NumPy's DOUBLE_fill obtains its actual step from the first two cast
    // values; this matters for the negative wheelpath grid origin.
    const double actual_step=out[1]-out[0];
    for(std::size_t i=2;i<n;++i) out[i]=start+static_cast<double>(i)*actual_step;
    return out;
}
inline double rounded(double value,int digits=4) {
    if(!std::isfinite(value)) return value;
    const auto spelling=gd::py_fixed(value,digits);
    double out=0.;
    const auto parsed=std::from_chars(spelling.data(),std::to_address(spelling.end()),out,std::chars_format::fixed);
    if(parsed.ec!=std::errc{}) throw std::invalid_argument("rounded: number outside decimal conversion domain");
    return out;
}
inline std::string seed_plus(std::string seed,std::uint64_t extra) {
    // Preserve the arbitrary-precision nonnegative integer source domain.
    if(seed.empty() || std::ranges::any_of(seed,[](char c){return c<'0'||c>'9';}))
        throw std::invalid_argument("terrain.seed: expected a nonnegative decimal integer");
    for(std::size_t i=seed.size();i>0 && extra!=0;--i) {
        const auto digit=static_cast<std::uint64_t>(seed[i-1]-'0')+extra%10U;
        extra/=10U; seed[i-1]=static_cast<char>('0'+digit%10U); extra+=digit/10U;
    }
    if(extra!=0) seed=std::to_string(extra)+seed;
    const auto first=seed.find_first_not_of('0');
    return first==std::string::npos ? "0" : seed.substr(first);
}
inline bike::io::Value number(double value) { return {bike::io::Number{gd::py_repr(value)}}; }
inline bike::io::Value integer(std::int64_t value) { return {bike::io::Number{std::to_string(value)}}; }
inline bike::io::Value seed_value(std::string_view value) { return {bike::io::Number{seed_plus(std::string(value),0)}}; }
inline const bike::io::Value* member(const bike::io::Value& value,std::string_view key) {
    return bike::io::find_member(bike::io::as_object(value,"terrain"),key);
}
inline const bike::io::Value& required(const bike::io::Value& value,std::string_view key) {
    const auto* result=member(value,key);
    if(!result) throw std::invalid_argument("terrain: missing '"+std::string(key)+"'");
    return *result;
}
inline void range(Range r,std::string_view name,double minimum=0.) {
    finite(r[0],name); finite(r[1],name);
    if(r[0]<minimum || r[0]>r[1]) throw std::invalid_argument(std::string(name)+": invalid low/high range");
}
inline void range(CountRange r,std::string_view name) {
    if(r[0]<0 || r[0]>r[1] || r[1]==std::numeric_limits<std::int64_t>::max())
        throw std::invalid_argument(std::string(name)+": invalid count range");
}
inline std::int64_t count_draw(bike::random::NumpyRng& rng,CountRange value) {
    return rng.integers(value[0],value[1]+1);
}
} // namespace bike::terrain::detail
