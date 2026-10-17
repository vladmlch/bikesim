#include "terrain/profile.hpp"
#include "terrain/detail.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace bike::terrain {
void HeightFieldSpec::validate() const {
    if(nrow<2||ncol<2)throw std::invalid_argument("HeightFieldSpec: nrow/ncol must be at least two");
    detail::positive(radius_x_m,"HeightFieldSpec.radius_x_m");detail::positive(radius_y_m,"HeightFieldSpec.radius_y_m");
    detail::positive(elevation_m,"HeightFieldSpec.elevation_m");detail::positive(base_m,"HeightFieldSpec.base_m");detail::finite(datum_z_m,"HeightFieldSpec.datum_z_m");
    const auto rows=static_cast<std::size_t>(nrow),cols=static_cast<std::size_t>(ncol);
    if(cols>static_cast<std::size_t>(std::numeric_limits<int>::max())/rows)throw std::invalid_argument("HeightFieldSpec: field sample count exceeds native engine domain");
}
double HeightFieldSpec::track_length_m()const{return 2.*radius_x_m;}
double HeightFieldSpec::resolution_m()const{return track_length_m()/static_cast<double>(ncol-1);}
double HeightFieldSpec::min_profile_m()const{return -datum_z_m;}
double HeightFieldSpec::max_profile_m()const{return elevation_m-datum_z_m;}
std::string HeightFieldSpec::size_attr()const{return detail::gd::py_fixed(radius_x_m,6)+" "+detail::gd::py_fixed(radius_y_m,6)+" "+detail::gd::py_fixed(elevation_m,6)+" "+detail::gd::py_fixed(base_m,6);}
std::vector<double> HeightFieldSpec::track_x()const{validate();return detail::gd::np_linspace(0.,track_length_m(),static_cast<std::size_t>(ncol));}
double HeightFieldSpec::geom_x_m()const{return radius_x_m;}
double HeightFieldSpec::geom_z_m(double ground)const{return ground-datum_z_m;}
double HeightFieldSpec::catch_plane_z_m(double ground,double clearance)const{return geom_z_m(ground)-clearance;}
bool HeightFieldSpec::fits(Interval extent)const{return extent.first>=min_profile_m()&&extent.second<=max_profile_m();}
std::vector<double> HeightFieldSpec::normalize(std::span<const double> values)const{
    validate();if(values.size()!=static_cast<std::size_t>(ncol))throw std::invalid_argument("HeightFieldSpec.normalize: wrong sample count");
    for(double v:values)detail::finite(v,"HeightFieldSpec.profile");
    const auto [lo,hi]=std::minmax_element(values.begin(),values.end());
    if(!fits({*lo,*hi}))throw std::invalid_argument("profile extent leaves the field envelope");
    std::vector<double> out(static_cast<std::size_t>(nrow)*static_cast<std::size_t>(ncol));
    for(std::size_t i=0;i<values.size();++i)out[i]=(values[i]+datum_z_m)/elevation_m;
    for(int row=1;row<nrow;++row)std::copy_n(out.begin(),ncol,out.begin()+static_cast<std::ptrdiff_t>(row)*ncol);
    return out;
}
HeightFieldSpec HeightFieldSpec::with_length(double value)const{
    validate();detail::positive(value,"field length");const double steps=value/resolution_m(),nearest=detail::rounded(steps,0);
    if(std::abs(steps-nearest)>1e-6)throw std::invalid_argument("field length is not a multiple of the resolution");
    HeightFieldSpec out=*this;out.radius_x_m=value/2.;out.ncol=static_cast<int>(detail::count(nearest+1.,"HeightFieldSpec.ncol"));out.validate();return out;
}
HeightFieldSpec HeightFieldSpec::for_track_length(double length){
    detail::positive(length,"track length");HeightFieldSpec out;const double needed=length+FIELD_RUNOUT_MARGIN_M;
    if(needed<=out.track_length_m())return out;
    return out.with_length(std::ceil(needed/FIELD_LENGTH_STEP_M)*FIELD_LENGTH_STEP_M);
}
HeightFieldSpec HeightFieldSpec::for_track(const TrackSpec& track){
    auto out=for_track_length(track.length_m);const auto extent=profile_extent(track,out.resolution_m());
    if(extent.first<out.min_profile_m()||extent.second>out.max_profile_m()){
        out.datum_z_m=std::max(out.datum_z_m,std::ceil((-extent.first+1.)*10.)/10.);
        out.elevation_m=std::ceil((out.datum_z_m+extent.second+1.)*10.)/10.;
    }out.validate();return out;
}
bool HeightFieldSpec::is_default()const{return *this==HeightFieldSpec{};}
std::vector<double> build_field_data(const TrackSpec& track,const HeightFieldSpec& spec){return spec.normalize(build_profile(track,spec.track_x()));}
std::vector<double> build_field_data(const TrackSpec& track){return build_field_data(track,HeightFieldSpec::for_track(track));}
void assert_track_fits(const TrackSpec& track,const HeightFieldSpec& spec){
    track.validate();spec.validate();
    if(track.length_m>spec.track_length_m())throw std::invalid_argument("track '"+track.name+"' is longer than the field");
    if(!spec.fits(profile_extent(track,spec.resolution_m())))throw std::invalid_argument("track '"+track.name+"' extent leaves the field envelope");
}
} // namespace bike::terrain
