#include "support/migration_test.hpp"
#include "runtime/spatial_worker_pool.hpp"

#include <cstdint>
#include <limits>

namespace {
void covered(std::size_t count,unsigned workers) {
    const auto ranges=runtime::spatial::partition_ranges(count,workers);
    bt::require(ranges.size()==workers,"one range per caller/worker");
    bt::require(ranges.front().begin==0 && ranges.back().end==count,"ranges cover the complete set");
    std::size_t total=0,ones=0,empty=0;
    for(std::size_t i=0;i<ranges.size();++i) {
        bt::require(ranges[i].begin<=ranges[i].end,"range bounds are ordered");
        if(i)bt::require(ranges[i-1].end==ranges[i].begin,"no overlap or hole");
        const auto width=ranges[i].end-ranges[i].begin;
        total+=width;ones+=width==1?1U:0U;empty+=width==0?1U:0U;
    }
    bt::require(total==count,"each element has exactly one owner");
    if(count==5 && workers==8)bt::require(ones==5 && empty==3,"N greater than count admits empty ranges");
    if(count==0)bt::require(empty==workers,"zero elements yields only empty ranges");
}
void cases() {
    covered(7,3);covered(0,4);covered(5,8);covered(1,1);
    covered(static_cast<std::size_t>(std::numeric_limits<std::uint64_t>::max()/2),3);
    covered(std::numeric_limits<std::size_t>::max(),3);
    bt::invalid([]{(void)runtime::spatial::partition_ranges(7,0);});
    const auto canonical=runtime::spatial::partition_ranges(7,3);
    bt::require(canonical[0].end==2 && canonical[1].end==4,"ordinary partition keeps floor(count*i/N)");
}
}
int main(){return bt::run(cases);}
