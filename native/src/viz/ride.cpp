#include "plot_internal.hpp"
#include "../legacy/metrics.hpp"
#include "../legacy/tyre.hpp"
#include "../numerics/filter.hpp"
#include "../terrain/profile.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <regex>
#include <set>
#include <stdexcept>

namespace bike::viz {
namespace {
double percentile(std::vector<double> values,double percent) {
    if(values.empty())return 0.;
    if(std::any_of(values.begin(),values.end(),[](double x){return std::isnan(x);}))return std::numeric_limits<double>::quiet_NaN();
    std::sort(values.begin(),values.end());const double index=(static_cast<double>(values.size())-1.)*(percent/100.);
    const auto lower=static_cast<std::size_t>(std::floor(index)),upper=static_cast<std::size_t>(std::ceil(index));const double fraction=index-static_cast<double>(lower);
    // NumPy linear percentile interpolates toward the nearer endpoint.
    const double delta=values[upper]-values[lower];return fraction>=.5?values[upper]-delta*(1.-fraction):values[lower]+delta*fraction;
}
void add(Plot& plot,const Channels& channels,const std::vector<double>& x,std::string_view name,std::string_view color={},bool secondary=false,SeriesStyle style=SeriesStyle::line) {
    detail::add_series(plot,channels,x,name,color,secondary,style);
}
void evidence_markers(std::vector<Report>& reports,const Channels& channels,const std::vector<double>& time,const std::vector<double>& distance) {
    // Preview/report metadata is observed, not re-simulated. The invalid prefix
    // and episode/termination boundaries remain visible even if no force
    // sample or trace was recorded after that boundary.
    std::vector<std::pair<std::size_t,std::string>> events;
    for(const auto key:{"model_valid","numerically_valid","valid"})if(const auto* valid=numeric(channels,key)){
        if(valid->size()!=time.size())throw std::invalid_argument("validity marker width differs from time_s");
        for(std::size_t i=0;i<valid->size();++i)if(std::isfinite((*valid)[i])&&(*valid)[i]==0.){events.emplace_back(i,std::string("INVALID PREFIX: ")+key);break;}}
    const auto* episode=numeric(channels,"episode_id");if(!episode)episode=numeric(channels,"generation");
    if(episode){if(episode->size()!=time.size())throw std::invalid_argument("episode marker width differs from time_s");
        for(std::size_t i=1;i<episode->size();++i)if(std::isfinite((*episode)[i])&&std::isfinite((*episode)[i-1])&&(*episode)[i]!=(*episode)[i-1])events.emplace_back(i,"RESET / episode "+detail::number((*episode)[i]));}
    if(const auto* labels=strings(channels,"event")) {
        if(labels->size()!=time.size())throw std::invalid_argument("preview event width differs from time_s");
        for(std::size_t i=0;i<labels->size();++i)if(!(*labels)[i].empty()&&!((*labels)[i]=="reset"&&episode)){
            auto boundary=i;while(boundary>0&&!std::isfinite(time[boundary]))--boundary;
            if(std::isfinite(time[boundary]))events.emplace_back(boundary,(*labels)[i]);
        }
    }
    for(const auto key:{"termination_reason","outcome"})if(const auto* values=strings(channels,key)){
        if(values->size()!=time.size())throw std::invalid_argument("termination marker width differs from time_s");
        for(std::size_t i=0;i<values->size();++i)if(!(*values)[i].empty()&&(i==0||(*values)[i]!=(*values)[i-1]))events.emplace_back(i,(*values)[i]);}
    if(const auto* brakes=numeric(channels,"braking")){if(brakes->size()!=time.size())throw std::invalid_argument("brake marker width differs from time_s");
        bool active=false;for(std::size_t i=0;i<brakes->size();++i){const bool current=(*brakes)[i]>.5;if(current!=active){events.emplace_back(i,current?"BRAKE ON":"BRAKE OFF");active=current;}}}
    for(std::size_t r=0;r<reports.size();++r)for(auto& panel:reports[r].panels)for(const auto& [i,label]:events){const double x=r==0?time[i]:distance[i];if(std::isfinite(x))panel.markers.push_back({MarkerKind::vertical,x,0.,label,"#ef4444"});}
}
}
Report travel_plot(const Channels& channels,const terrain::TrackSpec& track,double fork_limit,double shock_limit) {
    const auto x=detail::required(channels,"x_m");
    Plot fork{"Fork travel along "+track.name,"","travel (mm)",{{"fork travel",x,detail::required(channels,"fork_travel_mm"),"#38bdf8"}}};
    fork.markers.push_back({MarkerKind::horizontal,fork_limit,0.,"full travel "+detail::number(fork_limit)+" mm","#ef4444"});detail::track_markers(fork,track);
    Plot shock{"Shock stroke and rear wheel travel","track x (m)","mm",{{"shock stroke",x,detail::required(channels,"shock_stroke_mm"),"#fbbf24"},
        {"rear wheel travel",x,detail::required(channels,"rear_wheel_mm"),"#f97316"}}};
    shock.markers.push_back({MarkerKind::horizontal,shock_limit,0.,"full stroke "+detail::number(shock_limit)+" mm","#ef4444"});detail::track_markers(shock,track);
    Report out{"Suspension travel",{std::move(fork),std::move(shock)},1,2400,1200};validate(out);return out;
}
Report shaft_velocity_plot(const Channels& channels,std::span<const unsigned char> window,unsigned bins) {
    if(bins==0||bins>10000)throw std::invalid_argument("histogram bins must be in [1,10000]");
    const auto x=detail::required(channels,"x_m");if(!window.empty()&&window.size()!=x.size())throw std::invalid_argument("shaft-velocity window width differs from x_m");
    Report out{"Shaft velocity",{},2,2100,900};
    for(std::size_t end=0;end<2;++end){const std::string name=end==0?"fork_shaft_mps":"shock_shaft_mps";const auto source=detail::required(channels,name);
        if(source.size()!=x.size())throw std::invalid_argument("shaft-velocity channel width differs from x_m");
        std::vector<double> values;for(std::size_t i=0;i<source.size();++i)if((window.empty()||window[i])&&std::isfinite(source[i]))values.push_back(source[i]);
        Plot plot{end==0?"Fork shaft velocity":"Shock shaft velocity","shaft velocity (m/s), + compression / - rebound","time share",{}};plot.log_y=true;
        if(!values.empty()){auto absolute=values;for(double& value:absolute)value=std::abs(value);const double limit=std::max(.5,percentile(absolute,99.5)*1.1),width=2.*limit/bins;
            std::vector<double> centers(bins),density(bins,0.);std::size_t included=0;
            for(const double value:values)if(value>=-limit&&value<=limit){const auto index=std::min<std::size_t>(bins-1,static_cast<std::size_t>(std::floor((value+limit)/width)));density[index]+=1.;++included;}
            for(unsigned i=0;i<bins;++i){centers[i]=-limit+(static_cast<double>(i)+.5)*width;if(included)density[i]/=static_cast<double>(included)*width;}
            plot.series.push_back({end==0?"fork":"shock",std::move(centers),std::move(density),std::string(end==0?"#38bdf8":"#fbbf24"),SeriesStyle::bars});
            std::vector<double> compression,rebound;for(const double value:values)if(value>0.)compression.push_back(value);else if(value<0.)rebound.push_back(-value);
            plot.notes={"compression: p50 "+detail::number(percentile(compression,50.))+", p95 "+detail::number(percentile(compression,95.))+", max "+detail::number(compression.empty()?0.:*std::max_element(compression.begin(),compression.end()))+" m/s",
                "rebound: p50 "+detail::number(-percentile(rebound,50.))+", p95 "+detail::number(-percentile(rebound,95.))+", max "+detail::number(rebound.empty()?0.:-*std::max_element(rebound.begin(),rebound.end()))+" m/s"};
        }
        plot.markers.push_back({MarkerKind::vertical,0.,0.,{},"#8b949e"});out.panels.push_back(std::move(plot));
    }
    validate(out);return out;
}
Report acceleration_plot(const Channels& channels,double dt,const terrain::TrackSpec& track) {
    const auto x=detail::required(channels,"x_m");Report report{"Vertical acceleration",{},1,2400,1200};
    for(std::size_t end=0;end<2;++end){auto raw=detail::required(channels,end==0?"bar_acc_vert_mps2":"saddle_acc_vert_mps2");if(raw.size()!=x.size())throw std::invalid_argument("acceleration channel width differs from x_m");
        for(double& value:raw)value-=legacy::kGravityMps2;const auto filtered=numerics::legacy_lowpass(raw,dt);
        Plot plot{end==0?"Handlebar vertical acceleration":"Saddle vertical acceleration",end==0?"":"track x (m)","m/s^2 (g removed)",
            {{"100 Hz low-pass",x,filtered,end==0?"#34d399":"#f97316"}}};
        plot.markers.push_back({MarkerKind::horizontal,0.,0.,{},"#8b949e"});detail::track_markers(plot,track);
        if(!raw.empty()){std::size_t peak=0;for(std::size_t i=1;i<raw.size();++i)if(std::abs(raw[i])>std::abs(raw[peak]))peak=i;
            plot.notes.push_back("raw peak "+detail::number(raw[peak])+" m/s^2 at x = "+detail::number(x[peak])+" m (solver transient, unfiltered)");}
        if(end==1)if(const auto* torso=numeric(channels,"rider_torso_acc_vert_mps2"))if(!torso->empty()&&std::any_of(torso->begin(),torso->end(),[](double v){return v!=0.;})){
            auto values=*torso;for(double& v:values)v-=legacy::kGravityMps2;plot.series.push_back({"rider torso, 100 Hz low-pass",x,numerics::legacy_lowpass(values,dt),"#c084fc"});}
        report.panels.push_back(std::move(plot));
    }
    validate(report);return report;
}
Report tyre_plot(const Channels& channels,const terrain::TrackSpec& track) {
    const auto x=detail::required(channels,"x_m");Report report{"Pneumatic tyres",{
        {"Tyre vertical load","","load (N)",{}},{"Tyre deflection and rim strikes","","deflection (mm)",{}},
        {"Longitudinal slip","track x (m)","slip ratio κ",{}},{"Contact patch length","track x (m)","length (mm)",{}}},2,2400,1350};
    report.panels[2].markers={{MarkerKind::horizontal,0.,0.,{},"#8b949e"},{MarkerKind::horizontal,-1.,0.,"locked wheel κ = −1","#ef4444"}};
    for(std::size_t wheel=0;wheel<2;++wheel){const std::string side=wheel==0?"front":"rear",color=wheel==0?"#38bdf8":"#fbbf24";
        for(std::size_t p=0;p<4;++p){const std::array suffix{"_tyre_fz_n","_tyre_deflection_mm","_slip_ratio","_patch_length_mm"};add(report.panels[p],channels,x,side+suffix[p],color);}
        // Historical ride_plots.py uses FRONT_TYRE/REAR_TYRE for these
        // geometric thresholds. A newer recording may advertise its actual
        // geometry explicitly; pressure changes do not move the rim datum.
        double rim_threshold=(wheel==0?legacy::front_tyre():legacy::rear_tyre()).rim_strike_deflection_mm();
        if(const auto* threshold=numeric(channels,side+"_rim_strike_deflection_mm"))if(!threshold->empty()&&std::isfinite(threshold->front()))rim_threshold=threshold->front();
        report.panels[1].markers.push_back({MarkerKind::horizontal,rim_threshold,0.,side+" rim threshold",color});
        if(const auto* rim=numeric(channels,side+"_rim_strike")){if(rim->size()!=x.size())throw std::invalid_argument("rim-event width differs from x_m");bool previous=false;
            for(std::size_t i=0;i<rim->size();++i){const bool active=(*rim)[i]>.5;if(active&&!previous&&std::isfinite(x[i]))for(const auto panel:{0U,1U,3U})report.panels[panel].markers.push_back({MarkerKind::vertical,x[i],0.,side+" rim strike","#ef4444"});previous=active;}}
    }
    for(auto& panel:report.panels)detail::track_markers(panel,track);validate(report);return report;
}
Report track_profile(const terrain::TrackSpec& track,double resolution) {
    if(!std::isfinite(resolution)||resolution<=0.)throw std::invalid_argument("track plot resolution must be finite and positive");track.validate();
    const double raw_count=std::ceil((track.length_m+resolution)/resolution);if(raw_count>2000000.)throw std::invalid_argument("track plot exceeds two million points");
    const auto count=static_cast<std::size_t>(raw_count);std::vector<double>x(count);for(std::size_t i=0;i<count;++i)x[i]=static_cast<double>(i)*resolution;
    const auto z=terrain::build_profile(track,x);
    Plot profile{"Road profile: "+track.name+" ("+detail::number(track.length_m)+" m)","track x (m)","height (m)",{{"Road profile",x,z,"#e6edf3"}}};
    if(!track.description.empty())profile.notes.push_back(track.description);detail::track_markers(profile,track);
    Plot holes{"Potholes: declared depth vs effective wheel drop","track x (m)","mm",{}};
    Series declared{"declared depth",{}, {},"#8b949e",SeriesStyle::bars},front{"front wheel drop (r = 372 mm)",{}, {},"#38bdf8",SeriesStyle::bars},rear{"rear wheel drop (r = 352 mm)",{}, {},"#fbbf24",SeriesStyle::bars};
    const double width=std::max(.4,track.length_m/150.);
    for(const auto& hole:legacy::pothole_reports(track)){const double center=hole.start_m+hole.length_m/2.;declared.x.push_back(center-width);declared.y.push_back(hole.declared_depth_mm);
        front.x.push_back(center);front.y.push_back(hole.effective_drop_front_mm);rear.x.push_back(center+width);rear.y.push_back(hole.effective_drop_rear_mm);}
    if(declared.x.empty())holes.notes.push_back("no potholes on this track");else holes.series={std::move(declared),std::move(front),std::move(rear)};
    holes.x_range=std::pair{0.,track.length_m};Report report{"Track preview",{std::move(profile),std::move(holes)},1,2400,1200};validate(report);return report;
}
std::vector<Report> physical_ride_plots(const Channels& channels) {
    const auto time=detail::required(channels,"time_s");
    const std::array<std::string_view,4> titles{"speed","normal_load","power","energy_residual"},labels{"Speed (m/s)","Normal force (N)","Power (W)","Mechanical residual (J)"};
    const std::array<std::vector<std::string_view>,4> keys{{{"speed_mps"},{"tires.front.normal_load_n","tires.rear.normal_load_n"},{"drive.motor_shaft_power_w","drive.electrical_power_w"},{"energy.residual_j"}}};
    std::vector<Report> out;
    for(std::size_t i=0;i<titles.size();++i){Plot plot{std::string(titles[i]),"Force interval start (s)",std::string(labels[i]),{}};
        for(const auto key:keys[i])add(plot,channels,time,key);Report report{std::string(titles[i]),{std::move(plot)},1,1170,520};validate(report);out.push_back(std::move(report));}
    return out;
}
std::vector<std::pair<double,std::string>> shift_events(const Channels& channels) {
    const auto* count=numeric(channels,"shift_count"),*time=numeric(channels,"time_s");if(!count||!time)return {};
    if(count->size()!=time->size())throw std::invalid_argument("shift_count/time_s widths differ");
    const auto* direction=strings(channels,"last_shift_direction");const auto* from=numeric(channels,"last_shift_from_teeth"),*to=numeric(channels,"last_shift_to_teeth");
    for(const auto* values:{from,to})if(values&&values->size()!=time->size())throw std::invalid_argument("shift teeth/time_s widths differ");
    if(direction&&direction->size()!=time->size())throw std::invalid_argument("shift direction/time_s widths differ");
    std::vector<std::pair<double,std::string>> out;
    for(std::size_t i=1;i<count->size();++i)if((*count)[i]!=(*count)[i-1]&&std::isfinite((*count)[i])){
        std::string label;if(direction&&from&&to&&std::isfinite((*from)[i])&&std::isfinite((*to)[i]))label=(*direction)[i]+":"+detail::number(std::trunc((*from)[i]))+"->"+detail::number(std::trunc((*to)[i]));
        if(std::isfinite((*time)[i]))out.emplace_back((*time)[i],std::move(label));}
    return out;
}
std::vector<std::pair<double,std::string>> obstacle_positions(const Channels& channels) {
    const auto* labels=strings(channels,"obstacle");if(!labels)return {};
    const std::regex pattern(R"(@(-?\d+(?:\.\d+)?)m\b)");std::set<std::string> seen;std::vector<std::pair<double,std::string>> out;
    for(const auto& label:*labels)if(!seen.contains(label)){std::smatch match;if(std::regex_search(label,match,pattern)){const double x=std::stod(match[1].str());if(std::isfinite(x)){out.emplace_back(x,label);seen.insert(label);}}}
    std::sort(out.begin(),out.end());return out;
}
std::vector<Report> physical_ride_figures(const Channels& channels) {
    const auto time=detail::required(channels,"time_s"),x=detail::required(channels,"x_m");if(time.size()!=x.size())throw std::invalid_argument("channels time_s/x_m widths differ");
    Report effort{"Effort and drivetrain",{{"Speed and grade","","km/h",{}},{"Power: motor vs human","","W",{}},
        {"Cadence vs required","","rpm",{}},{"Rear gear and freehub torque","","teeth",{}},{"Motor request vs delivered","time (s)","Nm",{}}},1,1280,1600};
    add(effort.panels[0],channels,time,"speed_kmh","#38bdf8");add(effort.panels[0],channels,time,"grade_pct","#8b949e",true);effort.panels[0].secondary_y_label="%";
    add(effort.panels[1],channels,time,"motor_shaft_power_w","#fbbf24");
    const auto* cadence=numeric(channels,"cadence_rpm"),*human=numeric(channels,"human_sensor_nm");if(cadence&&human){if(cadence->size()!=time.size()||human->size()!=time.size())throw std::invalid_argument("human power inputs differ from time_s width");std::vector<double> power(time.size());
        for(std::size_t i=0;i<power.size();++i)power[i]=(*human)[i]*(*cadence)[i]*(2.*std::numbers::pi/60.);
        effort.panels[1].series.push_back({"human_power_w (sensor Nm x cadence)",time,std::move(power),"#c084fc"});}
    add(effort.panels[2],channels,time,"cadence_rpm","#34d399");add(effort.panels[2],channels,time,"required_cadence_rpm","#ef4444");
    if(const auto* cut=numeric(channels,"shift_torque_factor")){if(cut->size()!=time.size())throw std::invalid_argument("shift_torque_factor/time_s widths differ");std::vector<std::size_t> starts,ends;
        for(std::size_t i=1;i<cut->size();++i){const bool before=(*cut)[i-1]<.999,after=(*cut)[i]<.999;if(!before&&after)starts.push_back(i-1);if(before&&!after)ends.push_back(i-1);}
        for(std::size_t i=0;i<std::min(starts.size(),ends.size());++i)if(std::isfinite(time[starts[i]])&&std::isfinite(time[ends[i]]))effort.panels[2].markers.push_back({MarkerKind::band,time[starts[i]],time[ends[i]],"shift torque cut"});}
    add(effort.panels[3],channels,time,"gear_rear_teeth","#f97316",false,SeriesStyle::step);add(effort.panels[3],channels,time,"gear_front_teeth","#8b949e",false,SeriesStyle::step);
    add(effort.panels[3],channels,time,"freehub_torque_nm","#c084fc",true);effort.panels[3].secondary_y_label="Nm";
    for(const auto& [shift_time,label]:shift_events(channels))effort.panels[3].markers.push_back({MarkerKind::vertical,shift_time,0.,label});
    add(effort.panels[4],channels,time,"motor_request_nm","#8b949e");add(effort.panels[4],channels,time,"motor_torque_nm","#fbbf24");
    Report suspension{"Suspension and wheel load",{{"Suspension travel","","mm",{}},{"Wheel normal load","track x (m)","N",{}}},1,1280,800};
    add(suspension.panels[0],channels,x,"fork_travel_mm","#38bdf8");add(suspension.panels[0],channels,x,"shock_stroke_mm","#fbbf24");
    add(suspension.panels[1],channels,x,"front_load_n","#34d399");add(suspension.panels[1],channels,x,"rear_load_n","#f97316");
    for(const auto& [position,label]:obstacle_positions(channels))for(auto& panel:suspension.panels)panel.markers.push_back({MarkerKind::vertical,position,0.,label,"#a855f7",true});
    std::vector<Report> out{std::move(effort),std::move(suspension)};evidence_markers(out,channels,time,x);for(const auto& report:out)validate(report);return out;
}
std::vector<std::filesystem::path> plot_ride(const Channels& channels,double dt,const terrain::TrackSpec& track,double fork,double shock,const std::filesystem::path& out,std::span<const unsigned char> window) {
    const std::array reports{travel_plot(channels,track,fork,shock),shaft_velocity_plot(channels,window),acceleration_plot(channels,dt,track)};
    constexpr std::array names{"travel.png","shaft_velocity.png","acceleration.png"};std::vector<std::filesystem::path> paths;
    for(std::size_t i=0;i<reports.size();++i){paths.push_back(out/names[i]);write_png(reports[i],paths.back());}return paths;
}
std::filesystem::path plot_physical_ride_html(const Channels& channels,const std::filesystem::path& out,std::string_view filename) {
    const std::filesystem::path name(filename);if(name.empty()||name.has_parent_path()||name.extension()!=".html")throw std::invalid_argument("ride HTML filename must be a leaf .html path");
    const auto reports=physical_ride_figures(channels);const auto path=out/name;detail::write_text(path,to_html(reports));return path;
}
} // namespace bike::viz
