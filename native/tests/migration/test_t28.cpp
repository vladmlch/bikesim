// Analytic contracts for T28. Written during the no-execution review;
// no fixture-oracle or bitwise provider qualification is claimed here.
#include "support/migration_test.hpp"
#include "rider/allocation.hpp"
#include "numerics/linear_feasibility.hpp"
#include "numerics/nnls.hpp"
#include "numerics/brent.hpp"
#include "numerics/density_fit.hpp"
#include "io/json.hpp"
#include <array>
#include <cmath>
#include <limits>

namespace {
using namespace bike::rider;
namespace n=bike::numerics;
constexpr double inf=std::numeric_limits<double>::infinity();
AllocationProblem box(std::vector<double> target) {
    AllocationProblem p;p.target=std::move(target);p.lower.assign(p.target.size(),-3.);p.upper.assign(p.target.size(),3.);return p;
}
void projection_and_residuals() {
    auto p=box({2.,-3.});p.lower={0.,0.};p.upper={1.,1.};p.initial=std::vector<double>{.2,.3};
    int calls=0;p.extra.push_back({[&](std::span<const double>x){++calls;bt::require(x[0]>=0.&&x[0]<=1.&&x[1]>=0.&&x[1]<=1.,"callback only sees projected bounds");return std::vector<double>{std::sqrt(x[0])};},{},{0.},{1.},std::nullopt});
    auto result=allocate(p);bt::require(result.feasible&&result.solution==std::vector<double>({1.,0.})&&calls==1,"bounded target exact optimum ignores warm start");
    p=box({0.,0.});p.equality={1,2,{1.,1.}};p.equality_rhs={1.};p.lower={0.,0.};p.upper={1.,1.};
    result=allocate(p);bt::require(result.feasible&&std::abs(result.solution[0]-.5)<1e-7&&std::abs(result.solution[1]-.5)<1e-7,"nearest point on equality simplex");
    p=box({0.});p.equality={1,1,{1.}};p.equality_rhs={2.};p.lower={0.};p.upper={1.};result=allocate(p);
    bt::require(!result.feasible&&result.violation>=1.&&std::isfinite(result.solution[0]),"infeasible bounded point is a diagnostic");
    p=box({2.,0.});p.extra=grip_constraints({2,2,{1.,0.,0.,1.}},{-1.,0.},true,1.);result=allocate(p);
    bt::require(result.feasible&&std::abs(result.solution[0]-1.)<1e-6&&std::abs(result.solution[1])<1e-7,"nonlinear pulling radius constrains nearest point");
    p=box({0.});p.lower={inf};p.upper={-inf};bt::invalid([&]{(void)allocate(p);});
    p=box({0.});p.lower.clear();bt::invalid([&]{(void)allocation_residual(p,p.target);});
    p=box({0.});p.extra.push_back({[](std::span<const double>){return std::vector<double>{std::numeric_limits<double>::quiet_NaN()};},{},{0.},{1.},std::nullopt});
    bt::require(std::isinf(allocation_residual(p,p.target)),"nonfinite extra is infinite violation");
    const auto parsed=allocation_problem_from_json(bike::io::parse_json(R"({"target":[2,-3],"equality":[],"equality_rhs":[],"inequality":[],"inequality_rhs":[],"lower":[0,0],"upper":[1,1],"extra":[],"initial":null})"));
    bt::require(allocate(parsed).solution==std::vector<double>({1.,0.}),"cold problem adapter reaches same bounded objective");
}
void conservative_linear_regions() {
    const Matrix g{2,1,{1.,-1.}};const std::array<double,2> h{0.,-1.};const std::array<double,1> lo{-inf},hi{inf};
    bt::require(n::linear_feasibility(g,h,lo,hi).status==n::FeasibilityStatus::proven_infeasible,"contradictory linear interval certificate");
    n::FeasibilityOptions stopped;stopped.max_iterations=0;
    bt::require(n::linear_feasibility(g,h,lo,hi,{}, {},stopped).status==n::FeasibilityStatus::unknown,"budget exhaustion cannot certify emptiness");
    const std::array<double,1> zero{0.},small{1e-10};
    bt::require(n::linear_feasibility({0,1,{}},{},zero,zero,{1,1,{1.}},small).status!=n::FeasibilityStatus::proven_infeasible,"equality tolerance is retained by interval propagation");
    const auto rows=inverse_dynamics_rows({1,1,{2.}},{1,1,{1.}},{1,1,{3.}},std::array<double,1>{4.},std::array<double,1>{5.});
    bt::require(rows.first.values==std::vector<double>({2.,-1.,-3.})&&rows.second==std::vector<double>({1.}),"inverse dynamics sign convention");
}
void calibration_and_root() {
    const auto fit=n::nnls({3,2,{1.,0.,0.,1.,1.,1.}},std::array<double,3>{-1.,2.,1.});
    bt::require(fit.x[0]==0.&&std::abs(fit.x[1]-1.5)<1e-12&&std::abs(fit.residual_norm-std::sqrt(1.5))<1e-12,"NNLS boundary optimum");
    const auto root=n::brentq([](double x){return x*x-2.;},0.,2.);
    bt::require(std::abs(root.root-std::sqrt(2.))<2e-12,"safeguarded bracket converges to analytic root");
    bt::invalid([]{(void)n::brentq([](double x){return x*x+1.;},-1.,1.);});
    bike::configuration::HingeDensity material{{0.,.01},{100.,100.},10.,"analytic test"};
    const auto response=n::density_response(std::array<double,3>{-.01,.01,.02},material);
    bt::require(response.force==std::vector<double>({0.,1.,3.})&&std::abs(response.energy[2]-.025)<1e-15,"hinge potential derivative and separated force");
    const auto detached=n::normal_station_response(std::array<double,1>{0.},std::array<double,1>{100.},std::array<double,1>{1.},material);
    bt::require(detached.force[0]==0.,"damper creates no detached contact force");
    bt::require(n::normal_station_response({},std::array<double,1>{0.},std::array<double,1>{1.},material).force.empty(),"empty quadrature broadcasts scalar rate and weight without indexing it");
    const std::array<double,5> deflection{.002,.004,.008,.016,.024};const auto load=n::flat_load(deflection,.35,material,64).force;
    const auto fitted=n::fit_density(deflection,load,.35,"synthetic analytic",material.knots_m,64,10.);
    bt::require(fitted.accepted&&fitted.maximum_relative_error<1e-7&&fitted.calibration_status=="synthetic_fit_only","fit cannot certify physical material");
}
}
int main(){return bt::run([]{projection_and_residuals();conservative_linear_regions();calibration_and_root();});}
