#include "ride.hpp"
#include <algorithm>
#include <cmath>
#include <ranges>
#include <stdexcept>
namespace bike::legacy {
ForceAccumulator::ForceAccumulator(std::size_t width) : total_(width) {}
void ForceAccumulator::clear() noexcept {
    std::ranges::fill(total_,0.);
    for (auto& c:components_) std::ranges::fill(c.force,0.);
}
void ForceAccumulator::add(std::string_view name,std::span<const double> row) {
    if (row.size()!=total_.size() || name.empty()) throw std::invalid_argument("legacy force contribution name/width");
    for (std::size_t i=0;i<row.size();++i)
        if (!std::isfinite(row[i]) || !std::isfinite(total_[i]+row[i])) throw std::invalid_argument("legacy force contribution must be finite");
    auto found=std::ranges::find(components_,name,&ForceContribution::name);
    if(found!=components_.end())for(std::size_t i=0;i<row.size();++i)
        if(!std::isfinite(found->force[i]+row[i]))throw std::invalid_argument("legacy named force contribution overflow");
    if (found==components_.end()) { components_.push_back({.name=std::string(name),.force=std::vector<double>(row.size())}); found=components_.end()-1; }
    for (std::size_t i=0;i<row.size();++i) { found->force[i]+=row[i]; total_[i]+=row[i]; }
}
} // namespace bike::legacy
