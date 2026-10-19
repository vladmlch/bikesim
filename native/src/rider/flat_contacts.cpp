// Source: sim/ride/rider_support.py. These are requested internal support
// targets, not an external root force or overwritten measured contact.
#include "support_dispatch.hpp"
#include "../numerics/dense.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace bike::rider {
GravitySupportTargets gravity_support_targets(double weight,double com,const std::array<double,4>& points,
    double crank,double pedal,double bar,const std::array<bool,4>& enabled,double moment,std::optional<double> vertical) {
    namespace d=bike::numerics::dense;
    const std::array<double,7> scalars{weight,com,crank,pedal,bar,moment,vertical.value_or(weight)};
    if(!d::finite(scalars)||!d::finite(points)||weight<0.||pedal<0.||bar<0.||pedal+bar>=1.)throw std::invalid_argument("invalid postural support distribution");
    const std::array<double,4> preferred{weight*(1.-pedal-bar),weight*pedal/2.,weight*pedal/2.,weight*bar};
    Matrix matrix{.rows=3,.cols=4,.values={1.,1.,1.,1.,points[0]-com,points[1]-com,points[2]-com,points[3]-com,0.,points[1]-crank,points[2]-crank,0.}};
    const std::vector<double> target{vertical.value_or(weight),-moment,0.};auto available=enabled;GravitySupportTargets out;
    for(int iteration=0;iteration<4;++iteration) {
        std::vector<std::size_t> columns;for(std::size_t i=0;i<4;++i)if(available[i])columns.push_back(i);
        if(columns.empty()){out.forces.fill(0.);break;}
        auto selected=d::zeros(3,columns.size());std::vector<double> desired;
        for(std::size_t i=0;i<columns.size();++i) {
            desired.push_back(preferred[columns[i]]);for(std::size_t j=0;j<3;++j)selected.values[j*columns.size()+i]=matrix.values[j*4+columns[i]];
        }
        const auto rhs=d::add(target,d::multiply(selected,desired),-1.);const auto trans=d::transpose(selected);std::vector<double> correction;
        try {
            correction=columns.size()>=3?d::multiply(trans,d::solve(d::multiply(selected,trans),rhs)):d::solve(d::multiply(trans,selected),d::multiply(trans,rhs));
        }catch(const std::runtime_error&){correction=d::least_squares(selected,{.rows=3,.cols=1,.values=rhs},1e-10).solution.values;}
        out.forces.fill(0.);for(std::size_t i=0;i<columns.size();++i)out.forces[columns[i]]=desired[i]+correction[i];
        std::size_t negative=4;double minimum=-1e-9;
        for(auto const i:columns)if(i!=3 && out.forces[i]<minimum){minimum=out.forces[i];negative=i;}
        if(negative==4){for(std::size_t i=0;i<3;++i)out.forces[i]=std::max(out.forces[i],0.);break;}
        available[negative]=false;
    }
    const auto error=d::add(d::multiply(matrix,out.forces),target,-1.);double maximum=0.;for(double const e:error)maximum=std::max(maximum,std::abs(e));
    out.diagnostics.requested_pitch_moment_nm=moment;out.diagnostics.vertical_force_error_n=error[0];out.diagnostics.gravity_moment_error_nm=error[1];out.diagnostics.coasting_crank_torque_nm=error[2];out.diagnostics.feasible=maximum<=std::max(1e-8,weight*1e-8);out.diagnostics.vertical_forces_n=out.forces;return out;
}
PedalingSupportTargets pedaling_support_targets(double weight,const std::array<double,3>& com,
    const std::array<std::array<double,3>,4>& points,double crank,double pedal,double bar,
    const std::array<bool,4>& enabled,const std::array<std::array<double,3>,2>& forces,double moment,
    const std::array<double,3>& balance) {
    namespace d=bike::numerics::dense;
    if(!d::finite(com)||!d::finite(balance)||std::abs(balance[1])>1e-12)throw std::invalid_argument("balance request must be planar");
    for(const auto& p:points)if(!d::finite(p))throw std::invalid_argument("support points must be finite");
    for(const auto& f:forces)if(!d::finite(f)||std::abs(f[1])>1e-12)throw std::invalid_argument("pedaling request must be planar");
    const std::array<double,3> target{balance[0],0.,weight+balance[2]};std::array<std::array<double,3>,4> extra{};
    for(std::size_t i=0;i<2;++i)if(enabled[i+1])for(std::size_t j=0;j<3;++j)extra[i+1][j]=-forces[i][j];
    if(enabled[3])extra[3][0]=target[0]-(extra[0][0]+extra[1][0]+extra[2][0]);
    double extra_moment=0.,extra_z=0.;std::array<double,4> xs{};
    for(std::size_t i=0;i<4;++i){extra_moment+=(points[i][2]-com[2])*extra[i][0]-(points[i][0]-com[0])*extra[i][2];extra_z+=extra[i][2];xs[i]=points[i][0];}
    const auto gravity=gravity_support_targets(weight,com[0],xs,crank,pedal,bar,enabled,moment-extra_moment,target[2]-extra_z);
    PedalingSupportTargets out;out.diagnostics=gravity.diagnostics;auto reactions=extra;
    std::array<double,3> sum{};double total_moment=0.;
    for(std::size_t i=0;i<4;++i) {
        reactions[i][2]+=gravity.forces[i];
        for(std::size_t j=0;j<3;++j){sum[j]+=reactions[i][j];out.forces_on_bike[i][j]=-reactions[i][j];}
        total_moment+=(points[i][2]-com[2])*reactions[i][0]-(points[i][0]-com[0])*reactions[i][2];
    }
    double maximum=0.;for(std::size_t j=0;j<3;++j)maximum=std::max(maximum,std::abs(sum[j]-target[j]));
    out.diagnostics.feasible=out.diagnostics.feasible&&maximum<=1e-8;out.diagnostics.requested_balance_force=balance;out.diagnostics.horizontal_force_error_n=sum[0]-target[0];out.diagnostics.requested_pitch_moment_nm=moment;out.diagnostics.total_requested_force=sum;out.diagnostics.total_requested_pitch_moment_nm=total_moment;return out;
}
} // namespace bike::rider
