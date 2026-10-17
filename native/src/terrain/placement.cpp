#include "terrain/generators.hpp"
#include "terrain/detail.hpp"
#include <algorithm>

namespace bike::terrain {
double rounded(double value,int digits){return detail::rounded(value,digits);}
bool is_free(double start,double end,std::span<const Interval> taken){
    return std::ranges::all_of(taken,[&](const auto& interval){return end+CLEARANCE_M<=interval.first||start>=interval.second+CLEARANCE_M;});
}
std::optional<Interval> place(bike::random::NumpyRng& rng,double length,double lo,double hi,std::span<const Interval> taken,bool shrink){
    detail::finite(length,"placement.length");detail::finite(lo,"placement.lo");detail::finite(hi,"placement.hi");
    for(;;){
        if(length<=hi-lo)for(int attempt=0;attempt<PLACEMENT_TRIES;++attempt){
            const double start=rounded(rng.uniform(lo,hi-length),2);
            if(is_free(start,start+length,taken))return Interval{start,length};
        }
        if(!shrink||length/2.<1.)return std::nullopt;
        length=rounded(length/2.,2);
    }
}
std::vector<Interval> free_intervals(Interval span,std::span<const Interval> occupied,double gap){
    std::vector<Interval> sorted(occupied.begin(),occupied.end());std::sort(sorted.begin(),sorted.end());
    std::vector<Interval> out;double cursor=span.first;
    for(const auto& [start,end]:sorted){const double right=std::min(start-gap,span.second);if(right>cursor)out.emplace_back(cursor,right);cursor=std::max(cursor,end+gap);}
    if(span.second>cursor)out.emplace_back(cursor,span.second);return out;
}
} // namespace bike::terrain
