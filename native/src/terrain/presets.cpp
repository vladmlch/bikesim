#include "terrain/profile.hpp"
#include "terrain/generators.hpp"
#include <algorithm>

namespace bike::terrain {
TrackSpec enduro_aggressive(){
    TrackSpec track{.name="enduro_aggressive",.length_m=115.,.obstacles={},.description="Braking bumps, square edges, rock gardens, pothole, G-out, drop and kicker.",.surface="hardpack",.grade_profile=std::nullopt,.surface_sections={}};
    track.obstacles={{.shape=Washboard{.start_m=12.,.amplitude_m=.035,.wavelength_m=.900,.n_waves=9}},{.shape=SquareEdge{.start_m=22.,.height_m=.090,.ledge_length_m=.250}},{.shape=RockGarden{.start_m=26.,.section_length_m=8.,.amplitude_m=.060,.correlation_length_m=.250,.seed="1701"}},{.shape=Pothole{.start_m=38.,.depth_m=.180,.hole_length_m=.600}},{.shape=GOut{.start_m=42.,.depth_m=.350,.dip_length_m=5.}},{.shape=Drop{.start_m=52.,.height_m=.600,.ramp_m=0.}},{.shape=Kicker{.start_m=57.,.height_m=.350,.ramp_m=1.400,.gap_m=2.500,.landing_angle_deg=20.,.landing_m=5.}},{.shape=Roots{.start_m=69.,.n_bumps=7,.height_m=.080,.width_m=.250,.section_length_m=7.,.jitter=.35,.seed="2029"}},{.shape=SquareEdge{.start_m=80.,.height_m=.070,.ledge_length_m=.250}},{.shape=SquareEdge{.start_m=80.8,.height_m=.090,.ledge_length_m=.250}},{.shape=RockGarden{.start_m=84.,.section_length_m=11.,.amplitude_m=.090,.correlation_length_m=.200,.seed="3301"}}};return track;
}
TrackSpec flat(double length){return {.name="flat",.length_m=length,.obstacles={},.description="No obstacles; sag, rolling resistance and coast-down reference.",.surface="hardpack",.grade_profile=std::nullopt,.surface_sections={}};}
TrackSpec single_edge(){return {.name="single_edge",.length_m=40.,.obstacles={{.shape=SquareEdge{.start_m=20.,.height_m=.090,.ledge_length_m=.250}}},.description="Run-up and one 90 mm square edge at 20 m.",.surface="hardpack",.grade_profile=std::nullopt,.surface_sections={}};}
TrackSpec washboard_only(){return {.name="washboard_only",.length_m=60.,.obstacles={{.shape=Washboard{.start_m=15.,.amplitude_m=.035,.wavelength_m=.900,.n_waves=20}}},.description="Run-up and 20 waves of 35 mm braking bumps.",.surface="hardpack",.grade_profile=std::nullopt,.surface_sections={}};}
TrackSpec climb_steps(){SteppedClimb climb;climb.start_m=15.;return {.name="climb_steps",.length_m=115.,.obstacles={{.shape=std::move(climb)}},.description="Stepped climb: 5%, 10%, 15%, 20%, 25% grades with smooth transitions.",.surface="hardpack",.grade_profile=std::nullopt,.surface_sections={}};}
std::vector<std::string> available_presets(){return {"alpine_rocky","alpine_scree","climb_steps","enduro_aggressive","flat","road_broken","road_smooth","road_worn","single_edge","washboard_only"};}
TrackSpec get_preset(std::string_view name){
    TrackSpec track;
    if(name=="enduro_aggressive")track=enduro_aggressive();else if(name=="flat")track=flat();else if(name=="single_edge")track=single_edge();else if(name=="washboard_only")track=washboard_only();else if(name=="climb_steps")track=climb_steps();else if(name=="road_smooth")track=road_smooth();else if(name=="road_worn")track=road_worn();else if(name=="road_broken")track=road_broken();else if(name=="alpine_scree")track=alpine_scree();else if(name=="alpine_rocky")track=alpine_rocky();else throw std::invalid_argument("unknown track preset '"+std::string(name)+"'");
    track.validate();return track;
}
} // namespace bike::terrain
