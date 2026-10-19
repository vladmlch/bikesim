#include "../rider/allocation.hpp"
#include "../io/value.hpp"
#include "../numerics/dense.hpp"
#include <algorithm>
#include <limits>
#include <stdexcept>

namespace bike::rider {
namespace {
namespace io=bike::io;
const io::Value& member(const io::Value::Object& object,std::string_view name) {
    const auto* value=io::find_member(object,name);
    if(!value)throw std::invalid_argument("allocation fixture: missing "+std::string(name));
    return *value;
}
std::vector<double> vector(const io::Value& value,bool bounds=false) {
    std::vector<double> out;
    for(const auto& v:io::as_array(value,"allocation array")) {
        if(bounds && std::holds_alternative<std::string>(v.data)) {
            const auto& s=io::as_string(v,"allocation bound");
            if(s=="-inf")out.push_back(-std::numeric_limits<double>::infinity());
            else if(s=="+inf" || s=="inf")out.push_back(std::numeric_limits<double>::infinity());
            else throw std::invalid_argument("allocation bound: only named infinity allowed");
        } else out.push_back(io::as_double(v,"allocation number"));
    }
    return out;
}
Matrix matrix(const io::Value& value,std::size_t columns) {
    Matrix out{.rows=0,.cols=columns,.values={}};
    for(const auto& row:io::as_array(value,"allocation matrix")) {
        const auto data=vector(row);
        bike::numerics::dense::append_row(out,data);
    }
    return out;
}
}
AllocationProblem allocation_problem_from_json(const io::Value& value) {
    const auto& object=io::as_object(value,"allocation fixture");
    const std::array<std::string_view,10> keys{"target","equality","equality_rhs","inequality","inequality_rhs","lower","upper","extra","initial","fixture"};
    for(const auto& [key,v]:object){(void)v;if(std::ranges::find(keys,key)==keys.end())throw std::invalid_argument("allocation fixture: unknown field "+key);}
    AllocationProblem p;p.target=vector(member(object,"target"));const auto n=p.target.size();
    p.equality={.rows=0,.cols=n,.values={}};p.inequality={.rows=0,.cols=n,.values={}};
    if(const auto* a=io::find_member(object,"equality"))p.equality=matrix(*a,n);
    if(const auto* b=io::find_member(object,"equality_rhs"))p.equality_rhs=vector(*b);
    if(const auto* a=io::find_member(object,"inequality"))p.inequality=matrix(*a,n);
    if(const auto* b=io::find_member(object,"inequality_rhs"))p.inequality_rhs=vector(*b);
    p.lower=vector(member(object,"lower"),true);p.upper=vector(member(object,"upper"),true);
    if(const auto* start=io::find_member(object,"initial");start && !io::is_null(*start))p.initial=vector(*start);
    if(const auto* extras=io::find_member(object,"extra"))for(const auto& item:io::as_array(*extras,"allocation extra")) {
        const auto& e=io::as_object(item,"allocation extra");const auto& kind=io::as_string(member(e,"kind"),"allocation extra.kind");
        if(kind=="linear")p.extra.push_back(linear_constraint(matrix(member(e,"matrix"),n),vector(member(e,"lower"),true),vector(member(e,"upper"),true)));
        else if(kind=="grip") {
            const auto direction=vector(member(e,"direction"));if(direction.size()!=2)throw std::invalid_argument("grip fixture direction width");
            auto constraints=grip_constraints(matrix(member(e,"force_map"),n),{direction[0],direction[1]},io::as_bool(member(e,"pulling"),"grip pulling"),io::as_double(member(e,"limit_n"),"grip limit"));
            p.extra.insert(p.extra.end(),constraints.begin(),constraints.end());
        } else throw std::invalid_argument("allocation fixture: unknown named native constraint");
    }
    validate(p);return p;
}
} // namespace bike::rider
