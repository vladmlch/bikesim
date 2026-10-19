#include "constraint_dynamics.hpp"
#include "../numerics/dense.hpp"
#include <algorithm>
#include <deque>
#include <set>
#include <stdexcept>

namespace bike::rider {
ModeSearchResult search_constraint_modes(const ConstraintDynamics& dynamics,std::span<const double> wish,
    const ModeSolver& solve,const TorqueOf& torque_of,std::size_t budget) {
    if(budget<1)throw std::invalid_argument("constraint mode search needs a positive integer candidate budget");
    if(!solve)throw std::invalid_argument("constraint mode search needs a solver");
    std::deque<ConstraintMode> pending{dynamics.linearize(wish).mode};
    std::set<ConstraintMode> seen;
    ModeSearchResult best;bool have_best=false;
    while(!pending.empty() && seen.size()<budget) {
        auto const mode=std::move(pending.front());pending.pop_front();seen.insert(mode);
        auto response=dynamics.response_for_mode(mode);
        auto const candidate=solve(response);
        const bool feasible=candidate.allocation.feasible;
        const auto torque=torque_of?torque_of(candidate.allocation):candidate.allocation.solution;
        if(!have_best || candidate.allocation.violation<best.allocation.violation || feasible) {
            best.allocation=candidate.allocation;best.response=std::move(response);best.payload_index=candidate.payload_index;have_best=true;
        }
        if(feasible){best.complete=true;best.modes_tried=seen.size();return best;}
        const auto actual=dynamics.linearize(torque).mode;
        if(!seen.contains(actual)) {
            std::erase(pending,actual);pending.push_front(actual);
        }
        for(auto neighbor:dynamics.neighboring_modes(mode))
            if(!seen.contains(neighbor) && std::ranges::find(pending,neighbor)==pending.end())pending.push_back(std::move(neighbor));
    }
    best.complete=pending.empty();best.modes_tried=seen.size();return best;
}
} // namespace bike::rider
