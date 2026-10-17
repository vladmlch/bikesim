#include "density_fit.hpp"
#include "dense.hpp"
#include "nnls.hpp"
#include "../numeric_sum.hpp"
#include "../cblas_abi.hpp"
#include "../tyre/station.hpp"
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace bike::numerics {
namespace {
void material(const bike::configuration::HingeDensity& m) {
    if(m.knots_m.empty() || m.knots_m.size()!=m.stiffness_n_m.size() || !dense::finite(m.knots_m) || !dense::finite(m.stiffness_n_m) || !std::isfinite(m.damping_ns_m) || m.damping_ns_m<0. || m.knots_m[0]!=0. || m.provenance.find_first_not_of(" \n\t\r")==std::string::npos)throw std::invalid_argument("invalid density table/domain/provenance");
    for(std::size_t i=0;i<m.knots_m.size();++i)if(m.stiffness_n_m[i]<0. || (i && m.knots_m[i]<=m.knots_m[i-1]))throw std::invalid_argument("density law must be monotone and dissipative");
}
}
DensityResponse density_response(std::span<const double> delta,const bike::configuration::HingeDensity& m) {
    material(m);if(!dense::finite(delta))throw std::invalid_argument("contact penetration must be finite");
    DensityResponse out{std::vector<double>(delta.size(),0.),std::vector<double>(delta.size(),0.)};
    std::vector<double> force(m.knots_m.size()),energy(m.knots_m.size());
    for(std::size_t i=0;i<delta.size();++i) {
        for(std::size_t j=0;j<m.knots_m.size();++j) {
            const double x=std::max(delta[i]-m.knots_m[j],0.);force[j]=m.stiffness_n_m[j]*x;energy[j]=force[j]*x;
        }
        out.force[i]=numeric::numpy_pairwise_sum(force);out.energy[i]=.5*numeric::numpy_pairwise_sum(energy);
    }
    return out;
}
DensityResponse normal_station_response(std::span<const double> delta,std::span<const double> rate,
    std::span<const double> weights,const bike::configuration::HingeDensity& m) {
    std::size_t count=1;
    for(auto v:{delta,rate,weights}) {
        if(!dense::finite(v))throw std::invalid_argument("invalid contact quadrature");
        if(v.size()!=1) {
            if(count!=1 && count!=v.size())throw std::invalid_argument("invalid contact quadrature");
            count=v.size();
        }
    }
    for(double w:weights)if(w<0.)throw std::invalid_argument("invalid contact quadrature");
    std::vector<double> expanded(count);for(std::size_t i=0;i<count;++i)expanded[i]=delta[delta.size()==1?0:i];
    auto out=density_response(expanded,m);
    for(std::size_t i=0;i<count;++i) {
        const double w=weights[weights.size()==1?0:i],r=rate[rate.size()==1?0:i];
        out.force[i]=w*(expanded[i]>0.?std::max(0.,out.force[i]+m.damping_ns_m*r):0.);out.energy[i]*=w;
    }
    return out;
}
DensityResponse flat_load(std::span<const double> deflection,double radius,
    const bike::configuration::HingeDensity& m,int count,double phase) {
    if(!std::isfinite(radius)||radius<=0. || !std::isfinite(phase) || !dense::finite(deflection))throw std::invalid_argument("tire radius/phase/deflection domain");
    material(m);const auto stations=biketyre::station_angles(count);
    DensityResponse out{std::vector<double>(deflection.size(),0.),std::vector<double>(deflection.size(),0.)};
    for(std::size_t i=0;i<deflection.size();++i) {
        std::vector<double> delta(static_cast<std::size_t>(count));
        // This is a lone sin call, with no same-angle cosine to fuse.
        for(std::size_t j=0;j<delta.size();++j)delta[j]=deflection[i]-radius*(1.+std::sin(stations.angles[j]+phase));
        const std::array<double,1> zero_rate{0.};
        const auto response=normal_station_response(delta,zero_rate,stations.weights,m);
        out.force[i]=numeric::numpy_pairwise_sum(response.force);out.energy[i]=numeric::numpy_pairwise_sum(response.energy);
    }
    return out;
}
DensityFit fit_density(std::span<const double> deflections,std::span<const double> loads,double radius,
    std::string dataset,std::span<const double> knots,int count,double damping,double gate) {
    if(deflections.size()<3 || deflections.size()!=loads.size() || !dense::finite(deflections) || !dense::finite(loads))throw std::invalid_argument("fit needs ordered positive deflections and positive aggregate loads");
    for(std::size_t i=0;i<loads.size();++i)if(loads[i]<=0. || deflections[i]<=0. || (i && deflections[i]<=deflections[i-1]))throw std::invalid_argument("fit needs ordered positive deflections and positive aggregate loads");
    if(dataset.find_first_not_of(" \n\r\t")==std::string::npos)throw std::invalid_argument("a fitting dataset ID is required");
    if(!std::isfinite(gate)||gate<=0.||gate>=1.)throw std::invalid_argument("invalid fitting error gate");
    auto basis=dense::zeros(loads.size(),knots.size());
    for(std::size_t j=0;j<knots.size();++j) {
        bike::configuration::HingeDensity unit;unit.knots_m.assign(knots.begin(),knots.end());unit.stiffness_n_m.assign(knots.size(),0.);unit.stiffness_n_m[j]=1.;unit.damping_ns_m=0.;unit.provenance="unit basis";
        const auto column=flat_load(deflections,radius,unit,count).force;
        for(std::size_t i=0;i<loads.size();++i)basis.values[i*knots.size()+j]=column[i];
    }
    auto weighted=basis;for(std::size_t i=0;i<loads.size();++i)for(std::size_t j=0;j<knots.size();++j)weighted.values[i*knots.size()+j]/=loads[i];
    const auto fit=nnls(weighted,std::vector<double>(loads.size(),1.));
    DensityFit out;out.density.knots_m.assign(knots.begin(),knots.end());out.density.stiffness_n_m=fit.x;out.density.damping_ns_m=damping;out.density.provenance="fit: "+dataset;material(out.density);
    out.dataset_id=std::move(dataset);out.radius_m=radius;out.station_count=count;out.error_gate=gate;out.target_load_n.assign(loads.begin(),loads.end());out.deflection_m.assign(deflections.begin(),deflections.end());out.predicted_load_n.resize(loads.size());
    // np.array(columns).T is Fortran contiguous, so preserve that matvec
    // layout at the existing BLAS supplier instead of a scalar row fold.
    if(std::cmp_greater(loads.size(),std::numeric_limits<int>::max())||std::cmp_greater(knots.size(),std::numeric_limits<int>::max()))throw std::invalid_argument("density fit exceeds BLAS integer dimensions");
    const auto columns=dense::transpose(basis);
    blas::dgemv(blas::Order::column_major,blas::Transpose::no,static_cast<int>(loads.size()),static_cast<int>(knots.size()),1.,columns.values.data(),static_cast<int>(loads.size()),fit.x.data(),1,0.,out.predicted_load_n.data(),1);
    std::vector<double> squared(loads.size());for(std::size_t i=0;i<loads.size();++i){const double e=std::abs(out.predicted_load_n[i]-loads[i])/loads[i];out.maximum_relative_error=std::max(out.maximum_relative_error,e);squared[i]=e*e;}
    out.rms_relative_error=std::sqrt(numeric::numpy_pairwise_sum(squared)/static_cast<double>(loads.size()));out.accepted=out.maximum_relative_error<=gate;
    std::string lowered=out.dataset_id;for(char& c:lowered)c=static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    out.calibration_status=lowered.find("synthetic")!=std::string::npos?"synthetic_fit_only":"fit_only_not_validated";return out;
}
} // namespace bike::numerics
