#include "metrics.hpp"
#include "../numeric_sum.hpp"
#include "../numeric_sincos.hpp"
#include "../numerics/filter.hpp"
#include "../rider/least_squares.hpp"
#include "../terrain/profile.hpp"
#include "../runtime/setup_values.hpp"
#include "../io/json.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <numbers>
#include <ranges>
#include <sstream>
#include <stdexcept>

namespace bike::legacy {
namespace {
using Values=std::span<const double>;
Values channel(const Channels& columns,std::string_view name,bool required=true){
    for(const auto& [key,data]:columns)if(key==name)return data;
    if(required)throw std::invalid_argument("missing legacy metric channel '"+std::string(name)+"'");return {};
}
void positive(double value,std::string_view name){if(!std::isfinite(value)||value<=0.)throw std::invalid_argument(std::string(name)+" must be positive and finite");}
void same(Values a,Values b){if(a.size()!=b.size())throw std::invalid_argument("legacy metric channel lengths differ");}
double sum(Values x){return numeric::numpy_pairwise_sum(x);}
double mean(Values x){return x.empty()?0.:sum(x)/static_cast<double>(x.size());}
double maximum(Values x){if(x.empty())return 0.;double result=x.front();for(double v:x){if(std::isnan(v))return v;result=std::max(result,v);}return result;}
double minimum(Values x){if(x.empty())return 0.;double result=x.front();for(double v:x){if(std::isnan(v))return v;result=std::min(result,v);}return result;}
template<class F>std::vector<unsigned char> flags(Values x,F predicate){std::vector<unsigned char> out;out.reserve(x.size());for(double v:x)out.push_back(predicate(v)?1:0);return out;}
double count_time(std::span<const unsigned char> x,double dt){std::size_t n=0;for(auto v:x)n+=v!=0;return static_cast<double>(n)*dt;}
double fraction(Values x){if(x.empty())return 0.;std::size_t n=0;for(double v:x)n+=v>.5;return static_cast<double>(n)/static_cast<double>(x.size());}
double percentile95(Values x){if(x.empty())return 0.;for(double v:x)if(std::isnan(v))return v;std::vector<double> sorted(x.begin(),x.end());std::ranges::sort(sorted);
    const double position=.95*static_cast<double>(sorted.size()-1);const auto index=static_cast<std::size_t>(position);const double t=position-static_cast<double>(index);
    if(index+1==sorted.size())return sorted.back();const double delta=sorted[index+1]-sorted[index];
    // NumPy _lerp uses the second endpoint for the upper half interval.
    return t>=.5?sorted[index+1]-delta*(1.-t):sorted[index]+delta*t;
}
TyreStats tyre_stats(std::string side,const Channels& w,double dt,double start,const RideRecorder::TyreHistory* history){
    TyreStats out;out.wheel=side;
    const auto fz=channel(w,side+"_tyre_fz_n",false);
    const std::vector<double> zeros(fz.size());
    const auto optional=[&](std::string_view name)->Values{for(const auto& [key,values]:w)if(key==name)return values;return zeros;};
    const auto defl=optional(side+"_tyre_deflection_mm"),loss=optional(side+"_tyre_loss_w");
    const auto slip=optional(side+"_slip_ratio"),sliding=optional(side+"_tyre_full_sliding"),rim=optional(side+"_rim_strike"),speed=optional("speed_mps");
    out.peak_fz_n=maximum(fz);out.max_deflection_mm=maximum(defl);out.mean_dissipated_power_w=mean(loss);
    if(!fz.empty())same(fz,speed);std::vector<double> load_speed;load_speed.reserve(fz.size());for(std::size_t i=0;i<fz.size();++i)load_speed.push_back(fz[i]*std::abs(speed[i]));
    const double denominator=mean(load_speed);if(denominator>1e-9)out.crr_equivalent=out.mean_dissipated_power_w/denominator;
    out.rim_strikes=count_events(flags(rim,[](double v){return v>.5;}));
    if(history){
        for(const auto& event:history->events)if(event.x_m>=start)out.rim_strike_events.push_back(event);
        out.rim_strikes=0;for(double x:history->starts_m)out.rim_strikes+=x>=start;
        out.wheelspin_time_s=history->wheelspin_time_s;out.locked_time_s=history->locked_time_s;
    }else{same(slip,sliding);std::size_t spinning=0,locked=0;for(std::size_t i=0;i<slip.size();++i)if(sliding[i]>.5){spinning+=slip[i]>0.;locked+=slip[i]<0.;}
        out.wheelspin_time_s=static_cast<double>(spinning)*dt;out.locked_time_s=static_cast<double>(locked)*dt;}
    out.completed_rim_impact_episodes=count_completed_impact_episodes(out.rim_strike_events);return out;
}
}
std::size_t count_events(std::span<const unsigned char> flags){bool previous=false;std::size_t count=0;for(auto f:flags){const bool current=f!=0;if(current&&!previous)++count;previous=current;}return count;}
TravelStats travel_stats(std::string end,Values travel,Values velocity,double limit,double threshold){
    positive(limit,"travel limit");same(travel,velocity);TravelStats out;out.end=std::move(end);out.travel_limit_mm=limit;out.bottom_out_threshold_mm=threshold;
    if(travel.empty())return out;out.max_mm=maximum(travel);out.max_pct=100.*out.max_mm/limit;out.p95_mm=percentile95(travel);out.mean_mm=mean(travel);
    out.bottom_outs=count_events(flags(travel,[=](double x){return x>=threshold;}));out.top_outs=count_events(flags(travel,[](double x){return x<=kTopOutToleranceMm;}));
    out.max_compression_mps=std::max(0.,maximum(velocity));out.max_rebound_mps=std::max(0.,-minimum(velocity));return out;
}
AccelStats accel_stats(std::string point,Values acceleration,double dt){
    positive(dt,"sample interval");AccelStats out;out.point=std::move(point);if(acceleration.empty())return out;
    std::vector<double> a;for(double v:acceleration)a.push_back(v-kGravityMps2);
    const auto filtered=numerics::legacy_lowpass(a,dt,kAccelFilterHz,4);
    std::vector<double> squared,absolute,raw;squared.reserve(a.size());absolute.reserve(a.size());raw.reserve(a.size());
    for(std::size_t i=0;i<a.size();++i){squared.push_back(filtered[i]*filtered[i]);absolute.push_back(std::abs(filtered[i]));raw.push_back(std::abs(a[i]));}
    out.rms_filtered_mps2=std::sqrt(mean(squared));out.peak_filtered_mps2=maximum(absolute);out.peak_raw_mps2=maximum(raw);return out;
}
RiderStats rider_stats(const Channels& w,double dt,double weight){
    positive(weight,"rider weight");const auto saddle=channel(w,"saddle_load_n"),front=channel(w,"pedal_load_front_n"),rear=channel(w,"pedal_load_rear_n"),bar=channel(w,"bar_hand_load_n"),gap=channel(w,"saddle_gap_m");
    same(saddle,front);same(saddle,rear);same(saddle,bar);same(saddle,gap);std::vector<double> pedals;for(std::size_t i=0;i<saddle.size();++i)pedals.push_back(front[i]+rear[i]);
    RiderStats out;out.torso=accel_stats("torso",channel(w,"rider_torso_acc_vert_mps2"),dt);out.pelvis=accel_stats("pelvis",channel(w,"rider_pelvis_acc_vert_mps2"),dt);
    const auto lifted=flags(gap,[](double x){return x>0.;});out.saddle_lift_events=count_events(lifted);out.saddle_lift_time_s=count_time(lifted,dt);out.saddle_gap_max_mm=maximum(gap)*1000.;
    const auto n=static_cast<double>(std::max<std::size_t>(1,saddle.size()));out.mean_saddle_share=sum(saddle)/n/weight;out.mean_pedal_share=sum(pedals)/n/weight;out.mean_bar_share=sum(bar)/n/weight;
    out.min_saddle_load_n=minimum(saddle);out.max_saddle_load_n=maximum(saddle);return out;
}
double sine_amplitude(Values signal,double frequency,double dt){
    positive(dt,"sample interval");if(signal.size()<8||frequency<=0.)return 0.;
    if(frequency>=.5/dt||static_cast<double>(signal.size())*dt<1./frequency)return 0.;
    if(!std::isfinite(frequency))throw std::invalid_argument("bob frequency must be finite");
    const double average=mean(signal),omega=2.*std::numbers::pi*frequency;
    if(signal.size()>static_cast<std::size_t>(std::numeric_limits<::rider::lapack_int>::max())||signal.size()>std::vector<double>().max_size()/2)
        throw std::length_error("bob fit sample count exceeds native solver range");
    std::vector<double> basis,centred;basis.reserve(signal.size()*2);centred.reserve(signal.size());
    for(std::size_t i=0;i<signal.size();++i){const double angle=omega*(static_cast<double>(i)*dt);basis.push_back(numeric::cos(angle));basis.push_back(numeric::sin(angle));centred.push_back(signal[i]-average);}
    const auto n=static_cast<::rider::lapack_int>(signal.size());::rider::LeastSquaresWorkspace solve(n,2,1);
    const auto result=solve.solve(basis,n,2,centred,1,std::numeric_limits<double>::epsilon()*static_cast<double>(std::max<::rider::lapack_int>(n,2)));
    return std::hypot(result.solution[0],result.solution[1]);
}
std::optional<PedalBobStats> pedal_bob_stats(const Channels& w,double dt,int switches){
    const auto cadence=channel(w,"cadence_rpm",false);if(cadence.empty()||std::ranges::none_of(cadence,[](double v){return std::isfinite(v);}))return std::nullopt;
    const auto coast=channel(w,"freewheel"),crank=channel(w,"crank_torque_nm"),assist=channel(w,"assist_torque_nm"),wheel=channel(w,"drive_torque_nm");
    same(cadence,coast);same(cadence,crank);same(cadence,assist);same(cadence,wheel);
    std::vector<double> ca,cr,as,wh;for(std::size_t i=0;i<cadence.size();++i)if(std::isfinite(cadence[i])&&!(coast[i]>.5)){ca.push_back(cadence[i]);cr.push_back(crank[i]);as.push_back(assist[i]);wh.push_back(wheel[i]);}
    PedalBobStats out;out.mean_cadence_rpm=mean(ca);out.bob_frequency_hz=2.*out.mean_cadence_rpm/60.;
    out.shock_bob_amplitude_mm=sine_amplitude(channel(w,"shock_stroke_mm"),out.bob_frequency_hz,dt);out.fork_bob_amplitude_mm=sine_amplitude(channel(w,"fork_travel_mm"),out.bob_frequency_hz,dt);
    out.crank_torque_mean_nm=mean(cr);out.crank_torque_peak_nm=maximum(cr);out.assist_torque_mean_nm=mean(as);out.wheel_torque_mean_nm=mean(wh);
    const double total=out.crank_torque_mean_nm+out.assist_torque_mean_nm;out.gear_ratio_check=std::abs(total)>1e-9?out.wheel_torque_mean_nm/total:0.;
    out.freewheel_fraction=fraction(coast);out.cutoff_fraction=fraction(channel(w,"assist_cutoff_active"));out.traction_limited_fraction=fraction(channel(w,"drive_traction_limited"));out.assist_switches=switches;return out;
}
std::vector<PotholeReport> pothole_reports(const terrain::TrackSpec& track){
    std::vector<PotholeReport> out;for(const auto& obstacle:track.sorted_obstacles())if(obstacle.is_pothole()){
        const double depth=std::visit([](const auto& shape){if constexpr(requires{shape.depth_m;})return shape.depth_m;else return 0.;},obstacle.shape);
        out.push_back({obstacle.label(),std::string(obstacle.kind()),obstacle.start_m(),obstacle.length_m(),depth*1000.,
            terrain::effective_drop_m(obstacle,terrain::FRONT_WHEEL_RADIUS_M)*1000.,terrain::effective_drop_m(obstacle,terrain::REAR_WHEEL_RADIUS_M)*1000.});}return out;
}
std::array<std::size_t,3> count_kinds(std::span<const terrain::Obstacle> obstacles){std::array<std::size_t,3> out{};for(const auto& o:obstacles){out[0]+=o.is_pothole();out[1]+=o.is_bump();out[2]+=o.annotate();}return out;}
std::size_t count_completed_impact_episodes(std::span<const RimStrikeStats> events,double gap,double distance){
    if(!std::isfinite(gap)||!std::isfinite(distance)||gap<0.||distance<0.)throw std::invalid_argument("rim episode thresholds must be nonnegative and finite");
    std::optional<double> previous_end,previous_x;std::size_t count=0;
    for(const auto& event:events){if(!std::isfinite(event.x_m))throw std::invalid_argument("rim event x must be finite");
        if(!event.start_time_s||!event.end_time_s){++count;previous_end.reset();previous_x.reset();continue;}
        const double start=*event.start_time_s,end=*event.end_time_s;
        if(!std::isfinite(start)||!std::isfinite(end)||start<0.||end<start)throw std::invalid_argument("invalid rim event interval");
        if(previous_end&&start<*previous_end-1e-12)throw std::invalid_argument("rim events must be ordered and nonoverlapping");
        if(!(previous_end&&start-*previous_end<=gap+1e-12&&std::abs(event.x_m-*previous_x)<=distance))++count;
        previous_end=end;previous_x=event.x_m;
    }return count;
}
RideSummary summarize_ride(const Channels& columns,double dt,const terrain::TrackSpec& track,const RunOutcome& outcome,const SummaryOptions& options){
    positive(dt,"sample interval");const auto x=channel(columns,"x_m");const double start=options.start_x_m+kRampExclusionM;
    Channels window;window.reserve(columns.size());for(const auto& [name,values]:columns){same(x,values);std::vector<double> selected;for(std::size_t i=0;i<x.size();++i)if(x[i]>=start)selected.push_back(values[i]);window.emplace_back(name,std::move(selected));}
    RideSummary out;out.track=track.name;out.track_length_m=track.length_m;const auto counts=count_kinds(track.obstacles);out.n_potholes=counts[0];out.n_bumps=counts[1];out.n_obstacles=counts[2];
    out.target_speed_kmh=options.target_speed_kmh;out.outcome=outcome.reason;out.completed=outcome.completed();out.steps=outcome.steps;out.sim_time_s=outcome.sim_time_s;out.wall_clock_s=outcome.wall_clock_s;
    if(outcome.crash){const auto& c=*outcome.crash;std::ostringstream text;text.imbue(std::locale::classic());text<<c.cause<<" at x = "<<std::fixed<<std::setprecision(2)<<c.position_m<<" m, t = "<<std::setprecision(3)<<c.time_s<<" s, pitch = "<<std::showpos<<std::setprecision(1)<<c.pitch_rad*180./std::numbers::pi<<" deg";out.crash=text.str();}
    out.sample_interval_s=dt;out.window_start_m=start;out.window_rows=channel(window,"x_m").size();out.mean_speed_kmh=mean(channel(window,"speed_mps"))*3.6;
    const auto front=channel(window,"front_contact"),rear=channel(window,"rear_contact");std::vector<unsigned char> air;for(std::size_t i=0;i<front.size();++i)air.push_back(front[i]<.5&&rear[i]<.5?1:0);
    out.airborne_events=count_events(air);out.airborne_time_s=count_time(air,dt);
    out.fork=travel_stats("fork",channel(window,"fork_travel_mm"),channel(window,"fork_shaft_mps"),options.fork_travel_mm,options.fork_travel_mm-kForkBottomMarginMm);
    out.shock=travel_stats("shock",channel(window,"shock_stroke_mm"),channel(window,"shock_shaft_mps"),options.shock_stroke_mm,options.shock_bumper_engage_mm);
    out.bar=accel_stats("bar",channel(window,"bar_acc_vert_mps2"),dt);out.saddle=accel_stats("saddle",channel(window,"saddle_acc_vert_mps2"),dt);
    out.potholes=pothole_reports(track);out.extras=options.extras;out.rider_variant=options.rider_variant;
    if(options.rider_variant=="seated"&&std::ranges::any_of(window,[](const auto& entry){return entry.first=="saddle_load_n";}))out.rider=rider_stats(window,dt,options.rider_mass_kg*kGravityMps2);
    if(out.window_rows)out.pedals=pedal_bob_stats(window,dt,options.assist_switches);
    for(std::size_t i=0;i<2;++i)out.tyres[i]=tyre_stats(i==0?"front":"rear",window,dt,start,options.tyre_history?&(*options.tyre_history)[i]:nullptr);return out;
}

namespace {
using namespace runtime::setupvalue;
Value number(double x){if(std::isnan(x))return Value{io::Number{"NaN"}};if(std::isinf(x))return Value{io::Number{x<0.?"-Infinity":"Infinity"}};return real(x);}
Value size_value(std::size_t n){return Value{io::Number{std::to_string(n)}};}
Value stats_value(const TravelStats& t){return Value{Object{{"end",text(t.end)},{"travel_limit_mm",number(t.travel_limit_mm)},{"max_mm",number(t.max_mm)},{"max_pct",number(t.max_pct)},
    {"p95_mm",number(t.p95_mm)},{"mean_mm",number(t.mean_mm)},{"bottom_out_threshold_mm",number(t.bottom_out_threshold_mm)},{"bottom_outs",size_value(t.bottom_outs)},
    {"top_outs",size_value(t.top_outs)},{"max_compression_mps",number(t.max_compression_mps)},{"max_rebound_mps",number(t.max_rebound_mps)}}};}
Value stats_value(const AccelStats& a){return Value{Object{{"point",text(a.point)},{"filter_hz",number(a.filter_hz)},{"rms_filtered_mps2",number(a.rms_filtered_mps2)},
    {"peak_filtered_mps2",number(a.peak_filtered_mps2)},{"peak_raw_mps2",number(a.peak_raw_mps2)},{"raw_peak_note",text(a.raw_peak_note)}}};}
Value stats_value(const RiderStats& r){return Value{Object{{"torso",stats_value(r.torso)},{"pelvis",stats_value(r.pelvis)},{"saddle_lift_events",size_value(r.saddle_lift_events)},
    {"saddle_lift_time_s",number(r.saddle_lift_time_s)},{"saddle_gap_max_mm",number(r.saddle_gap_max_mm)},{"mean_saddle_share",number(r.mean_saddle_share)},
    {"mean_pedal_share",number(r.mean_pedal_share)},{"mean_bar_share",number(r.mean_bar_share)},{"min_saddle_load_n",number(r.min_saddle_load_n)},{"max_saddle_load_n",number(r.max_saddle_load_n)}}};}
Value stats_value(const PotholeReport& p){return Value{Object{{"label",text(p.label)},{"kind",text(p.kind)},{"start_m",number(p.start_m)},{"length_m",number(p.length_m)},
    {"declared_depth_mm",number(p.declared_depth_mm)},{"effective_drop_front_mm",number(p.effective_drop_front_mm)},{"effective_drop_rear_mm",number(p.effective_drop_rear_mm)}}};}
Value stats_value(const RimStrikeStats& e){return Value{Object{{"x_m",number(e.x_m)},{"speed_mps",number(e.speed_mps)},{"peak_load_n",number(e.peak_load_n)},
    {"peak_rim_force_n",number(e.peak_rim_force_n)},{"absorbed_energy_j",number(e.absorbed_energy_j)},{"start_time_s",e.start_time_s?number(*e.start_time_s):Value{}},{"end_time_s",e.end_time_s?number(*e.end_time_s):Value{}}}};}
Value stats_value(const TyreStats& t){Array events;for(const auto& e:t.rim_strike_events)events.push_back(stats_value(e));return Value{Object{{"wheel",text(t.wheel)},
    {"peak_fz_n",number(t.peak_fz_n)},{"max_deflection_mm",number(t.max_deflection_mm)},{"rim_strikes",size_value(t.rim_strikes)},{"rim_strike_events",Value{std::move(events)}},
    {"wheelspin_time_s",number(t.wheelspin_time_s)},{"locked_time_s",number(t.locked_time_s)},{"mean_dissipated_power_w",number(t.mean_dissipated_power_w)},{"crr_equivalent",number(t.crr_equivalent)},
    {"completed_rim_impact_episodes",size_value(t.completed_rim_impact_episodes)},{"rim_episode_gap_s",number(t.rim_episode_gap_s)},{"rim_episode_distance_m",number(t.rim_episode_distance_m)}}};}
Value stats_value(const PedalBobStats& p){return Value{Object{{"mean_cadence_rpm",number(p.mean_cadence_rpm)},{"bob_frequency_hz",number(p.bob_frequency_hz)},
    {"shock_bob_amplitude_mm",number(p.shock_bob_amplitude_mm)},{"fork_bob_amplitude_mm",number(p.fork_bob_amplitude_mm)},{"crank_torque_mean_nm",number(p.crank_torque_mean_nm)},
    {"crank_torque_peak_nm",number(p.crank_torque_peak_nm)},{"assist_torque_mean_nm",number(p.assist_torque_mean_nm)},{"wheel_torque_mean_nm",number(p.wheel_torque_mean_nm)},
    {"gear_ratio_check",number(p.gear_ratio_check)},{"freewheel_fraction",number(p.freewheel_fraction)},{"cutoff_fraction",number(p.cutoff_fraction)},
    {"traction_limited_fraction",number(p.traction_limited_fraction)},{"assist_switches",integer(p.assist_switches)}}};}
// Source summary.json deliberately uses Python json.dumps(allow_nan=True).
// Keep that legacy export policy local; the recording codec stays strict.
std::string ascii_json_string(std::string_view value){
    // The common serializer validates UTF-8 and emits JSON control escapes.
    // The legacy report additionally uses json.dumps' default ensure_ascii.
    const auto utf8=io::canonical_json(text(value));std::string out;
    constexpr std::string_view digits="0123456789abcdef";
    const auto unit=[&](std::uint32_t cp){out+="\\u";for(int shift:{12,8,4,0})out+=digits[(cp>>shift)&0xfU];};
    for(std::size_t i=0;i<utf8.size();){const auto first=static_cast<unsigned char>(utf8[i++]);
        if(first<0x7fU){out+=static_cast<char>(first);continue;}
        if(first==0x7fU){unit(first);continue;}
        const std::size_t remaining=first<0xe0U?1:first<0xf0U?2:3;
        std::uint32_t cp=first&(remaining==1?0x1fU:remaining==2?0x0fU:0x07U);
        for(std::size_t k=0;k<remaining;++k)cp=(cp<<6)|(static_cast<unsigned char>(utf8[i++])&0x3fU);
        if(cp<=0xffffU)unit(cp);else{cp-=0x10000U;unit(0xd800U+(cp>>10));unit(0xdc00U+(cp&0x3ffU));}}
    return out;
}
void pretty_json(std::ostream& stream,const Value& value,std::size_t depth=0){
    if(depth>io::kMaxDepth)throw std::invalid_argument("legacy summary nesting limit");const auto indent=[&](std::size_t n){stream<<std::string(n*2,' ');};
    if(const auto* object=std::get_if<Object>(&value.data)){stream<<'{';for(std::size_t i=0;i<object->size();++i){stream<<(i?",\n":"\n");indent(depth+1);stream<<ascii_json_string((*object)[i].first)<<": ";pretty_json(stream,(*object)[i].second,depth+1);}if(!object->empty()){stream<<'\n';indent(depth);}stream<<'}';}
    else if(const auto* array=std::get_if<Array>(&value.data)){stream<<'[';for(std::size_t i=0;i<array->size();++i){stream<<(i?",\n":"\n");indent(depth+1);pretty_json(stream,(*array)[i],depth+1);}if(!array->empty()){stream<<'\n';indent(depth);}stream<<']';}
    else if(const auto* string=std::get_if<std::string>(&value.data))stream<<ascii_json_string(*string);
    else if(const auto* n=std::get_if<io::Number>(&value.data);n&&(n->lexeme=="NaN"||n->lexeme=="Infinity"||n->lexeme=="-Infinity"))stream<<n->lexeme;
    else stream<<io::canonical_json(value);
}
}
io::Value RideSummary::to_value()const {
    Array pits;for(const auto& p:potholes)pits.push_back(stats_value(p));
    return Value{Object{{"track",text(track)},{"track_length_m",number(track_length_m)},{"n_potholes",size_value(n_potholes)},{"n_bumps",size_value(n_bumps)},
    {"n_obstacles",size_value(n_obstacles)},{"target_speed_kmh",number(target_speed_kmh)},{"outcome",text(outcome)},{"completed",Value{completed}},{"crash",crash?text(*crash):Value{}},
    {"steps",integer(steps)},{"sim_time_s",number(sim_time_s)},{"wall_clock_s",number(wall_clock_s)},{"sample_interval_s",number(sample_interval_s)},{"window_start_m",number(window_start_m)},
    {"window_rows",size_value(window_rows)},{"mean_speed_kmh",number(mean_speed_kmh)},{"airborne_events",size_value(airborne_events)},{"airborne_time_s",number(airborne_time_s)},
    {"fork",stats_value(fork)},{"shock",stats_value(shock)},{"bar",stats_value(bar)},{"saddle",stats_value(saddle)},{"potholes",Value{std::move(pits)}},{"extras",Value{extras}},
    {"rider_variant",text(rider_variant)},{"rider",rider?stats_value(*rider):Value{}},{"tyres",Value{Object{{"front",stats_value(tyres[0])},{"rear",stats_value(tyres[1])}}}},
    {"pedals",pedals?stats_value(*pedals):Value{}},{"schema_version",integer(1)},{"physics_revision",text("legacy-v1")}}};
}
std::string RideSummary::to_json()const{std::ostringstream out;out.imbue(std::locale::classic());pretty_json(out,to_value());return out.str();}
void RideSummary::write_json(const std::filesystem::path& path)const{const auto encoded=to_json();if(!path.parent_path().empty())std::filesystem::create_directories(path.parent_path());std::ofstream out(path);if(!out)throw std::runtime_error("cannot write legacy summary: "+path.string());out<<encoded<<'\n';out.flush();if(!out)throw std::runtime_error("legacy summary write failed: "+path.string());}
std::string RideSummary::format_table()const{
    std::ostringstream s;s.imbue(std::locale::classic());s<<std::fixed<<std::setprecision(0)<<"Ride summary -- "<<track<<" ("<<track_length_m<<" m, "<<n_potholes<<" potholes, "<<n_bumps<<" bumps) at "<<target_speed_kmh<<" km/h\n  outcome: "<<outcome;if(crash)s<<" ("<<*crash<<')';
    s<<"\n  "<<steps<<" steps, "<<std::setprecision(2)<<sim_time_s<<" s sim, "<<wall_clock_s<<" s wall; window from x = "<<std::setprecision(1)<<window_start_m<<" m ("<<window_rows<<" rows)\n  mean speed "<<std::setprecision(2)<<mean_speed_kmh<<" km/h; airborne "<<airborne_events<<" x, "<<std::setprecision(0)<<airborne_time_s*1000.<<" ms total\n\n";
    s<<"  "<<std::left<<std::setw(6)<<"end"<<std::right<<' '<<std::setw(8)<<"max mm"<<' '<<std::setw(6)<<"max %"<<' '<<std::setw(8)<<"p95 mm"<<' '<<std::setw(8)<<"mean mm"<<' '<<std::setw(7)<<"bottom"<<' '<<std::setw(5)<<"top"<<' '<<std::setw(9)<<"comp m/s"<<' '<<std::setw(8)<<"reb m/s"<<'\n';
    if(pedals){const auto& p=*pedals;s<<"  pedals: "<<p.mean_cadence_rpm<<" rpm, bob at "<<std::setprecision(2)<<p.bob_frequency_hz<<" Hz -> shock "<<p.shock_bob_amplitude_mm<<" mm / fork "<<p.fork_bob_amplitude_mm<<" mm; crank "<<std::setprecision(1)<<p.crank_torque_mean_nm<<" N.m mean ("<<p.crank_torque_peak_nm<<" peak) + motor "<<p.assist_torque_mean_nm<<"; ratio check "<<std::setprecision(3)<<p.gear_ratio_check<<"; freewheel "<<std::setprecision(0)<<100.*p.freewheel_fraction<<" %";if(p.assist_switches)s<<"; "<<p.assist_switches<<" assist switches in window";s<<"\n\n";}
    for(const auto* t:{&fork,&shock})s<<"  "<<std::left<<std::setw(6)<<t->end<<std::right<<' '<<std::fixed<<std::setprecision(1)<<std::setw(8)<<t->max_mm<<' '<<std::setw(6)<<t->max_pct<<' '<<std::setw(8)<<t->p95_mm<<' '<<std::setw(8)<<t->mean_mm<<' '<<std::setw(7)<<t->bottom_outs<<' '<<std::setw(5)<<t->top_outs<<' '<<std::setprecision(3)<<std::setw(9)<<t->max_compression_mps<<' '<<std::setw(8)<<t->max_rebound_mps<<'\n';
    s<<"\n  "<<std::left<<std::setw(6)<<"point"<<std::right<<' '<<std::setw(8)<<"RMS"<<' '<<std::setw(8)<<"peak"<<' '<<std::setw(9)<<"raw peak"<<"   (m/s^2, vertical, g removed, "<<std::setprecision(0)<<bar.filter_hz<<" Hz low-pass; raw incl. solver transients)\n";
    const auto accel=[&](const AccelStats& a){s<<"  "<<std::left<<std::setw(6)<<a.point<<std::right<<' '<<std::setprecision(2)<<std::setw(8)<<a.rms_filtered_mps2<<' '<<std::setw(8)<<a.peak_filtered_mps2<<' '<<std::setprecision(1)<<std::setw(9)<<a.peak_raw_mps2<<'\n';};accel(bar);accel(saddle);
    if(rider){const auto& r=*rider;accel(r.torso);accel(r.pelvis);s<<"\n  seated rider: mean load split saddle "<<std::setprecision(1)<<100.*r.mean_saddle_share<<" % / pedals "<<100.*r.mean_pedal_share<<" % / bar "<<100.*r.mean_bar_share<<" % of rider weight; saddle load "<<std::setprecision(0)<<r.min_saddle_load_n<<'-'<<r.max_saddle_load_n<<" N; left the saddle "<<r.saddle_lift_events<<" x, "<<r.saddle_lift_time_s*1000.<<" ms total, max gap "<<std::setprecision(1)<<r.saddle_gap_max_mm<<" mm\n";}
    if(!potholes.empty()){s<<"\n  "<<std::left<<std::setw(18)<<"pothole"<<std::right<<' '<<std::setw(7)<<"x m"<<' '<<std::setw(6)<<"len m"<<' '<<std::setw(9)<<"depth mm"<<' '<<std::setw(10)<<"drop F mm"<<' '<<std::setw(10)<<"drop R mm"<<'\n';for(const auto& p:potholes)s<<"  "<<std::left<<std::setw(18)<<p.label<<std::right<<' '<<std::setprecision(2)<<std::setw(7)<<p.start_m<<' '<<std::setw(6)<<p.length_m<<' '<<std::setprecision(0)<<std::setw(9)<<p.declared_depth_mm<<' '<<std::setw(10)<<p.effective_drop_front_mm<<' '<<std::setw(10)<<p.effective_drop_rear_mm<<'\n';}
    s<<"\n  "<<std::left<<std::setw(6)<<"tyre"<<std::right<<' '<<std::setw(9)<<"peak Fz"<<' '<<std::setw(10)<<"max defl"<<' '<<std::setw(5)<<"rim"<<' '<<std::setw(10)<<"wheelspin"<<' '<<std::setw(8)<<"locked"<<' '<<std::setw(9)<<"loss W"<<' '<<std::setw(8)<<"Crr eq"<<'\n';for(const auto& t:tyres)s<<"  "<<std::left<<std::setw(6)<<t.wheel<<std::right<<' '<<std::setprecision(1)<<std::setw(9)<<t.peak_fz_n<<' '<<std::setw(10)<<t.max_deflection_mm<<' '<<std::setw(5)<<t.rim_strikes<<' '<<std::setprecision(3)<<std::setw(10)<<t.wheelspin_time_s<<' '<<std::setw(8)<<t.locked_time_s<<' '<<std::setprecision(2)<<std::setw(9)<<t.mean_dissipated_power_w<<' '<<std::setprecision(4)<<std::setw(8)<<t.crr_equivalent<<'\n';
    auto result=s.str();if(!result.empty())result.pop_back();return result;
}
} // namespace bike::legacy
