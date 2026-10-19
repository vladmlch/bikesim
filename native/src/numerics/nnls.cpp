#include "nnls.hpp"
#include "dense.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace bike::numerics {
NnlsResult nnls(const bike::rider::Matrix& a,std::span<const double> b,std::size_t maximum) {
    namespace d=dense;
    d::validate(a,"NNLS matrix");
    if(a.rows!=b.size() || a.cols==0 || !d::finite(b)) throw std::invalid_argument("NNLS target shape/nonfinite input");
    const auto n=a.cols;
    if(maximum==0) maximum=3*n;
    const double tolerance=10.*static_cast<double>(std::max(a.rows,a.cols))*std::numeric_limits<double>::epsilon();
    NnlsResult result{.x=std::vector<double>(n,0.),.residual_norm=d::norm(b),.iterations=0};
    std::vector<bool> passive(n,false);
    const auto at=d::transpose(a);
    for(;;) {
        const auto residual=d::add(std::vector<double>(b.begin(),b.end()),d::multiply(a,result.x),-1.);
        const auto w=d::multiply(at,residual);
        std::size_t selected=n;double best=tolerance;
        for(std::size_t j=0;j<n;++j) if(!passive[j] && w[j]>best) {best=w[j];selected=j;}
        if(selected==n) {result.residual_norm=d::norm(residual);return result;}
        passive[selected]=true;
        for(;;) {
            if(result.iterations++>=maximum) throw std::runtime_error("Maximum number of NNLS iterations reached");
            std::vector<std::size_t> columns;
            for(std::size_t j=0;j<n;++j) if(passive[j]) columns.push_back(j);
            auto restricted=d::zeros(a.rows,columns.size());
            for(std::size_t i=0;i<a.rows;++i) for(std::size_t j=0;j<columns.size();++j)
                restricted.values[i*columns.size()+j]=a.values[i*n+columns[j]];
            const auto solved=d::least_squares(restricted,{.rows=a.rows,.cols=1,.values=std::vector<double>(b.begin(),b.end())}).solution.values;
            std::vector<double> z(n,0.);
            for(std::size_t j=0;j<columns.size();++j) z[columns[j]]=solved[j];
            bool positive=true;
            for(auto const j:columns) if(z[j]<=0.) positive=false;
            if(positive) {result.x=std::move(z);break;}
            double alpha=1.;
            for(auto const j:columns) if(z[j]<=0.) {
                const double denom=result.x[j]-z[j];
                alpha=std::min(alpha,denom>0.?result.x[j]/denom:0.);
            }
            for(std::size_t j=0;j<n;++j) result.x[j]+=alpha*(z[j]-result.x[j]);
            for(auto const j:columns) if(result.x[j]<=tolerance) {result.x[j]=0.;passive[j]=false;}
        }
    }
}
} // namespace bike::numerics
