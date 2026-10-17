#include "activation_allocation.hpp"
#include "spindle_math.hpp"
#include "../numerics/dense.hpp"
#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>

namespace bike::rider {
namespace {
namespace d=bike::numerics::dense;
void indices(std::size_t joints,std::span<const std::size_t> ti,std::span<const std::size_t> ei,std::span<const double> scales) {
    if(ti.size()!=joints || ei.size()!=joints || !d::finite(scales))throw std::invalid_argument("activation allocation indices/scale shape");
    std::vector<bool> used(scales.size(),false);
    for(auto list:{ti,ei})for(auto i:list) {
        if(i>=scales.size() || used[i] || scales[i]<=0.)throw std::invalid_argument("activation allocation indices/scales");
        used[i]=true;
    }
}
}
ActivationLaw::ActivationLaw(std::vector<double> state,std::vector<double> speeds,
    std::vector<double> lower,std::vector<double> upper,double dt,double tau,std::optional<double> cap)
    :state_(std::move(state)),speeds_(std::move(speeds)),lower_(std::move(lower)),upper_(std::move(upper)),dt_s_(dt),tau_s_(tau),gain_(tau==0.?1.:-std::expm1(-dt/tau)),power_limit_(cap) {
    const auto n=state_.size();
    if(n==0 || speeds_.size()!=n || lower_.size()!=n || upper_.size()!=n || !d::finite(state_) || !d::finite(speeds_) || !d::finite(lower_) || !d::finite(upper_) || !std::isfinite(dt) || dt<0. || !std::isfinite(tau) || tau<0. || (cap && (!std::isfinite(*cap)||*cap<0.)))
        throw std::invalid_argument("activation allocation: finite same-shape arrays and nonnegative time/power required");
    for(std::size_t i=0;i<n;++i)if(lower_[i]>upper_[i])throw std::invalid_argument("activation allocation: reversed bounds");
}
std::vector<double> ActivationLaw::latent(std::span<const double> excitation) const {
    if(excitation.size()!=state_.size() || !d::finite(excitation))throw std::invalid_argument("activation excitation shape/nonfinite");
    return ::spindle::activation_step(state_,excitation,dt_s_,tau_s_);
}
std::vector<double> ActivationLaw::delivered(std::span<const double> excitation) const {
    auto active=latent(excitation);
    for(std::size_t i=0;i<active.size();++i)active[i]=std::clamp(active[i],lower_[i],upper_[i]);
    return power_limit_?::spindle::limit_positive_power(active,speeds_,*power_limit_):active;
}
Matrix ActivationLaw::derivative(std::span<const double> excitation) const {
    const auto z=latent(excitation);auto active=z;
    const auto n=z.size();auto jac=d::zeros(n,n);
    std::vector<std::size_t> producing;double power=0.;
    for(std::size_t i=0;i<n;++i) {
        active[i]=std::clamp(active[i],lower_[i],upper_[i]);
        jac.values[i*n+i]=(z[i]>=lower_[i]&&z[i]<=upper_[i])?gain_:0.;
        if(active[i]*speeds_[i]>0.)producing.push_back(i);
        power+=std::max(active[i]*speeds_[i],0.);
    }
    if(power_limit_ && power>*power_limit_) {
        auto projection=d::identity(n);
        for(auto i:producing)for(auto j:producing)
            projection.values[i*n+j]=(*power_limit_/power)*(i==j?1.:0.)-*power_limit_/(power*power)*active[i]*speeds_[j];
        jac=d::multiply(projection,jac);
    }
    return jac;
}
ExtraConstraint ActivationLaw::constraint(std::vector<std::size_t> ti,std::vector<std::size_t> ei,std::vector<double> scales) const {
    indices(state_.size(),ti,ei,scales);
    auto law=std::make_shared<const ActivationLaw>(*this);
    return {[law,ti,ei,scales](std::span<const double> x) {
        if(x.size()!=scales.size())throw std::invalid_argument("activation constraint point shape");
        std::vector<double> excitation(ei.size());for(std::size_t i=0;i<ei.size();++i)excitation[i]=x[ei[i]]*scales[ei[i]];
        auto delivered=law->delivered(excitation);
        for(std::size_t i=0;i<ti.size();++i)delivered[i]=x[ti[i]]*scales[ti[i]]-delivered[i];
        return delivered;
    },[law,ti,ei,scales](std::span<const double> x) {
        if(x.size()!=scales.size())throw std::invalid_argument("activation Jacobian point shape");
        std::vector<double> excitation(ei.size());for(std::size_t i=0;i<ei.size();++i)excitation[i]=x[ei[i]]*scales[ei[i]];
        const auto derivative=law->derivative(excitation);auto out=d::zeros(ti.size(),x.size());
        for(std::size_t i=0;i<ti.size();++i) {
            out.values[i*x.size()+ti[i]]=scales[ti[i]];
            for(std::size_t j=0;j<ei.size();++j)out.values[i*x.size()+ei[j]]=-derivative.values[i*ei.size()+j]*scales[ei[j]];
        }
        return out;
    },{0.},{0.},std::nullopt};
}
std::vector<double> ActivationLaw::project(std::span<const double> x,std::span<const std::size_t> ti,
    std::span<const std::size_t> ei,std::span<const double> scales) const {
    indices(state_.size(),ti,ei,scales);
    if(x.size()!=scales.size())throw std::invalid_argument("activation projection point shape");
    std::vector<double> result(x.begin(),x.end()),excitation(ei.size());
    for(std::size_t i=0;i<ei.size();++i)excitation[i]=x[ei[i]]*scales[ei[i]];
    const auto active=delivered(excitation);
    for(std::size_t i=0;i<ti.size();++i)result[ti[i]]=active[i]/scales[ti[i]];
    return result;
}
} // namespace bike::rider
