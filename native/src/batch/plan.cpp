#include "plan.hpp"
#include "integer_detail.hpp"
#include "../io/json.hpp"
#include "../runtime/setup_values.hpp"
#include <algorithm>
#include <array>
#include <cctype>
#include <charconv>
#include <cmath>
#include <limits>
#include <ranges>
#include <set>
#include <stdexcept>

namespace bike::batch {
namespace {
using namespace runtime::setupvalue;
constexpr std::size_t maximum_jobs = 1000000;
std::string scalar(const Value& value) {
    if (const auto* b = std::get_if<bool>(&value.data)) return *b ? "True" : "False";
    if (const auto* n = std::get_if<io::Number>(&value.data)) return n->lexeme;
    if (const auto* s = std::get_if<std::string>(&value.data)) return *s;
    throw std::invalid_argument("batch flags must be bool, int, float or str; lists belong in [grid]");
}
Object merged(Object base, const Object& overrides) {
    for (const auto& [key, value] : overrides) {
        bool found = false;
        for (auto& [name, stored] : base) if (name == key) { stored = value; found = true; break; }
        if (!found) base.emplace_back(key, value);
    }
    return base;
}
std::optional<std::string> integer_like(const Value& value) {
    if (const auto* flag = std::get_if<bool>(&value.data)) return *flag ? "1" : "0";
    if (const auto* number = std::get_if<io::Number>(&value.data);
        number && io::is_integer_lexeme(number->lexeme)) return integer_detail::normalize(number->lexeme);
    return {};
}
std::string range_integer(const Value& value, std::string_view name) {
    if (auto number = integer_like(value)) return *number;
    throw std::invalid_argument(std::string(name) + " must be an integer");
}
std::size_t bounded_count(const Value& value) {
    const auto decimal = range_integer(value,"grid.count");
    if (integer_detail::compare(decimal,"0") <= 0) return 0; // Python range(nonpositive) is empty
    if (integer_detail::compare(decimal,std::to_string(maximum_jobs)) > 0)
        throw std::invalid_argument("grid exceeds 1000000 jobs");
    std::size_t result = 0;
    for (const char c : decimal) result = result * 10 + static_cast<std::size_t>(c-'0');
    return result;
}
bool numeric_equal(const Value& a, const Value& b) {
    // Python compares an arbitrary-width int with the exact represented
    // binary64 value. Converting both sides to float would collapse nearby
    // large seeds and silently remove their distinguishing slug component.
    const auto ia=integer_like(a),ib=integer_like(b);
    if(ia&&ib)return *ia==*ib;
    const bool na=ia.has_value()||std::holds_alternative<io::Number>(a.data);
    const bool nb=ib.has_value()||std::holds_alternative<io::Number>(b.data);
    if(na!=nb)return false;
    if(!na)return io::canonical_json(a)==io::canonical_json(b);
    if(!ia&&!ib)return io::as_double(a,"batch number")==io::as_double(b,"batch number");
    const auto number=io::as_double(ia?b:a,"batch number");
    if(!std::isfinite(number)||std::trunc(number)!=number)return false;
    std::array<char,384> digits{};
    const auto result=std::to_chars(digits.begin(),digits.end(),number,std::chars_format::fixed,0);
    if(result.ec!=std::errc{})throw std::runtime_error("batch numeric comparison conversion failed");
    return *(ia?ia:ib)==integer_detail::normalize(std::string(digits.begin(),result.ptr));
}
std::string slug(const Object& params, const Object& base) {
    std::string result;
    for (const auto& [key, value] : params) {
        const auto* previous=io::find_member(base,key);
        if (previous && numeric_equal(*previous,value)) continue;
        std::string text;
        if (const auto* b=std::get_if<bool>(&value.data)) text=*b?key:"no-"+key;
        else text=key+scalar(value);
        std::string part; bool dash=false;
        for (const char raw : text) {
            const auto c=static_cast<unsigned char>(raw);
            if ((c>='a'&&c<='z') || (c>='A'&&c<='Z') || (c>='0'&&c<='9')) {
                if (dash && !part.empty()) part+='-';
                part+=static_cast<char>(c); dash=false;
            } else dash=true;
        }
        if (!part.empty()) { if (!result.empty()) result+='-'; result+=part; }
    }
    if(result.size()>48) result.resize(48);
    return result;
}
std::vector<Value> grid_values(std::string_view key,const Value& raw) {
    if(const auto* values=std::get_if<Array>(&raw.data)) {
        if(values->empty())throw std::invalid_argument("grid."+std::string(key)+": empty list");
        for(const auto& value:*values)(void)scalar(value);
        return *values;
    }
    const auto& fields=io::as_object(raw,"grid range");
    const auto* start=io::find_member(fields,"start");
    const auto* count=io::find_member(fields,"count");
    const auto* stop=io::find_member(fields,"stop");
    const auto* step=io::find_member(fields,"step");
    if(!start || (count ? fields.size()!=2 || stop || step : !stop || fields.size()>(step?3U:2U)))
        throw std::invalid_argument("grid range wants {start,count} or {start,stop[,step]}");
    std::vector<Value> result;
    if(count) {
        const auto n=bounded_count(*count);
        if (!n) return result;
        result.reserve(n);
        if (auto current=integer_like(*start)) {
            for (std::size_t i=0;i<n;++i) {
                result.push_back(Value{io::Number{*current}});
                *current=integer_detail::add(*current,"1");
            }
        } else {
            const auto first=io::as_double(*start,"grid.start");
            for(std::size_t i=0;i<n;++i)result.push_back(real(first+static_cast<double>(i)));
        }
    } else {
        auto current=range_integer(*start,"grid.start");
        const auto end=range_integer(*stop,"grid.stop");
        const auto delta=step?range_integer(*step,"grid.step"):std::string("1");
        if(delta=="0")throw std::invalid_argument("grid range step must not be zero");
        const bool negative=delta.starts_with('-');
        while(negative?integer_detail::compare(current,end)>0:integer_detail::compare(current,end)<0) {
            if(result.size()==maximum_jobs)throw std::invalid_argument("grid exceeds 1000000 jobs");
            result.push_back(Value{io::Number{current}});
            current=integer_detail::add(current,delta);
        }
    }
    return result;
}
} // namespace
std::vector<Job> expand_spec(const io::Value& document) {
    const auto& root=io::as_object(document,"batch spec");
    for(const auto& [key,value]:root) {
        (void)value;
        if(key!="base"&&key!="grid"&&key!="job")throw std::invalid_argument("unknown spec sections: "+key);
    }
    const auto* base_value=io::find_member(root,"base"), *grid_value=io::find_member(root,"grid"), *explicit_value=io::find_member(root,"job");
    const Object base=base_value?io::as_object(*base_value,"base"):Object{};
    const Object grid=grid_value?io::as_object(*grid_value,"grid"):Object{};
    const Array entries=explicit_value?io::as_array(*explicit_value,"job"):Array{};
    std::vector<Object> rows;
    if(!grid.empty()||entries.empty())rows.push_back(base);
    for(const auto& [key,raw]:grid) {
        const auto values=grid_values(key,raw);
        if(!values.empty()&&rows.size()>maximum_jobs/values.size())throw std::invalid_argument("grid exceeds 1000000 jobs");
        std::vector<Object> next;
        for(const auto& row:rows)for(const auto& value:values)next.push_back(merged(row,{{key,value}}));
        rows=std::move(next);
    }
    for(const auto& entry:entries) {
        if(rows.size()==maximum_jobs)throw std::invalid_argument("batch exceeds 1000000 jobs");
        rows.push_back(merged(base,io::as_object(entry,"job")));
    }
    std::vector<Job> jobs; jobs.reserve(rows.size());
    for(std::size_t index=0;index<rows.size();++index) {
        for(const auto& [key,value]:rows[index]) {
            try {(void)scalar(value);}
            catch(const std::invalid_argument& error){throw std::invalid_argument("flag '"+key+"': "+error.what());}
        }
        auto id=std::to_string(index); if(id.size()<4)id.insert(0,4-id.size(),'0');
        const auto suffix=slug(rows[index],base);if(!suffix.empty())id+='-'+suffix;
        Job job;job.id=std::move(id);job.index=index;job.params=Value{rows[index]};
        for(const auto& [key,value]:rows[index]) {
            if(key=="out")throw std::invalid_argument("'out' is assigned by bike-batch; remove it from the spec");
            if(key.empty()||key.starts_with('-')||key.find('\0')!=std::string::npos)throw std::invalid_argument("invalid batch flag name");
            if(const auto* b=std::get_if<bool>(&value.data)) {if(*b)job.argv.push_back("--"+key);}
            else {job.argv.push_back("--"+key);job.argv.push_back(scalar(value));}
        }
        jobs.push_back(std::move(job));
    }
    return jobs;
}
std::vector<std::string> parse_seeds(std::string_view input) {
    std::vector<std::string> seeds;
    std::set<std::string> seen;
    const auto normalize=[](std::string_view value) {
        while(!value.empty()&&std::isspace(static_cast<unsigned char>(value.front())))value.remove_prefix(1);
        while(!value.empty()&&std::isspace(static_cast<unsigned char>(value.back())))value.remove_suffix(1);
        if(value.starts_with('+'))value.remove_prefix(1);
        if(value.empty()||!std::ranges::all_of(value,[](char c){return c>='0'&&c<='9';}))throw std::invalid_argument("seeds must be nonnegative decimal integers");
        const auto first=value.find_first_not_of('0');return first==std::string_view::npos?std::string("0"):std::string(value.substr(first));
    };
    const auto increment=[](std::string value) {
        for(auto it=value.rbegin();it!=value.rend();++it){if(*it!='9'){++*it;return value;}*it='0';}
        value.insert(value.begin(),'1');return value;
    };
    std::size_t visited=0;
    while(!input.empty()) {
        const auto comma=input.find(',');auto part=input.substr(0,comma);
        while(!part.empty()&&std::isspace(static_cast<unsigned char>(part.front())))part.remove_prefix(1);
        while(!part.empty()&&std::isspace(static_cast<unsigned char>(part.back())))part.remove_suffix(1);
        if(!part.empty()) {
            const auto dash=part.find('-');
            const auto first=normalize(part.substr(0,dash));
            const auto last=dash==std::string_view::npos?first:normalize(part.substr(dash+1));
            for(auto seed=first;seed.size()<last.size()||(seed.size()==last.size()&&seed<=last);seed=increment(std::move(seed))) {
                if(++visited>maximum_jobs)throw std::invalid_argument("seed list exceeds 1000000 entries");
                if(seen.insert(seed).second)seeds.push_back(seed);
            }
        }
        if(comma==std::string_view::npos)break;input.remove_prefix(comma+1);
    }
    return seeds;
}
} // namespace bike::batch
