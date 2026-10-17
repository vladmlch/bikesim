#include "terrain/profile.hpp"
#include "terrain/generators.hpp"
#include <algorithm>

namespace bike::terrain {
TrackSpec enduro_aggressive(){
    TrackSpec track{"enduro_aggressive",115.,{},"Braking bumps, square edges, rock gardens, pothole, G-out, drop and kicker.","hardpack",std::nullopt,{}};
    track.obstacles={{Washboard{12.,.035,.900,9}},{SquareEdge{22.,.090,.250}},{RockGarden{26.,8.,.060,.250,"1701"}},{Pothole{38.,.180,.600}},{GOut{42.,.350,5.}},{Drop{52.,.600,0.}},{Kicker{57.,.350,1.400,2.500,20.,5.}},{Roots{69.,7,.080,.250,7.,.35,"2029"}},{SquareEdge{80.,.070,.250}},{SquareEdge{80.8,.090,.250}},{RockGarden{84.,11.,.090,.200,"3301"}}};return track;
}
TrackSpec flat(double length){return {"flat",length,{},"No obstacles; sag, rolling resistance and coast-down reference.","hardpack",std::nullopt,{}};}
TrackSpec single_edge(){return {"single_edge",40.,{{SquareEdge{20.,.090,.250}}},"Run-up and one 90 mm square edge at 20 m.","hardpack",std::nullopt,{}};}
TrackSpec washboard_only(){return {"washboard_only",60.,{{Washboard{15.,.035,.900,20}}},"Run-up and 20 waves of 35 mm braking bumps.","hardpack",std::nullopt,{}};}
TrackSpec climb_steps(){SteppedClimb climb;climb.start_m=15.;return {"climb_steps",115.,{{std::move(climb)}},"Stepped climb: 5%, 10%, 15%, 20%, 25% grades with smooth transitions.","hardpack",std::nullopt,{}};}
std::vector<std::string> available_presets(){return {"alpine_rocky","alpine_scree","climb_steps","enduro_aggressive","flat","road_broken","road_smooth","road_worn","single_edge","washboard_only"};}
TrackSpec get_preset(std::string_view name){
    TrackSpec track;
    if(name=="enduro_aggressive")track=enduro_aggressive();else if(name=="flat")track=flat();else if(name=="single_edge")track=single_edge();else if(name=="washboard_only")track=washboard_only();else if(name=="climb_steps")track=climb_steps();else if(name=="road_smooth")track=road_smooth();else if(name=="road_worn")track=road_worn();else if(name=="road_broken")track=road_broken();else if(name=="alpine_scree")track=alpine_scree();else if(name=="alpine_rocky")track=alpine_rocky();else throw std::invalid_argument("unknown track preset '"+std::string(name)+"'");
    track.validate();return track;
}
} // namespace bike::terrain
