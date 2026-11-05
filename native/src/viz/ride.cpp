#include "plot_internal.hpp"
#include "../terrain/pothole_report.hpp"
#include "../terrain/profile.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>
#include <regex>
#include <set>
#include <stdexcept>

namespace bike::viz {
namespace {
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
    for(std::size_t r=0;r<reports.size();++r)for(auto& panel:reports[r].panels)for(const auto& [i,label]:events){const double x=r==0?time[i]:distance[i];if(std::isfinite(x))panel.markers.push_back({.kind=MarkerKind::vertical,.first=x,.second=0.,.label=label,.color="#ef4444"});}
}
}
Report track_profile(const terrain::TrackSpec& track,double resolution) {
    if(!std::isfinite(resolution)||resolution<=0.)throw std::invalid_argument("track plot resolution must be finite and positive");
    track.validate();
    const double raw_count=std::ceil((track.length_m+resolution)/resolution);if(raw_count>2000000.)throw std::invalid_argument("track plot exceeds two million points");
    const auto count=static_cast<std::size_t>(raw_count);std::vector<double>x(count);for(std::size_t i=0;i<count;++i)x[i]=static_cast<double>(i)*resolution;
    const auto z=terrain::build_profile(track,x);
    Plot profile{.title="Road profile: "+track.name+" ("+detail::number(track.length_m)+" m)",.x_label="track x (m)",.y_label="height (m)",.series={{.name="Road profile",.x=x,.y=z,.color="#e6edf3"}}};
    if(!track.description.empty())profile.notes.push_back(track.description);
    detail::track_markers(profile,track);
    Plot holes{.title="Potholes: declared depth vs effective wheel drop",.x_label="track x (m)",.y_label="mm",.series={}};
    Series declared{.name="declared depth",.x={}, .y={},.color="#8b949e",.style=SeriesStyle::bars},front{.name="front wheel drop (r = 372 mm)",.x={}, .y={},.color="#38bdf8",.style=SeriesStyle::bars},rear{.name="rear wheel drop (r = 352 mm)",.x={}, .y={},.color="#fbbf24",.style=SeriesStyle::bars};
    const double width=std::max(.4,track.length_m/150.);
    for(const auto& hole:terrain::pothole_reports(track)){const double center=hole.start_m+hole.length_m/2.;declared.x.push_back(center-width);declared.y.push_back(hole.declared_depth_mm);
        front.x.push_back(center);front.y.push_back(hole.effective_drop_front_mm);rear.x.push_back(center+width);rear.y.push_back(hole.effective_drop_rear_mm);}
    if(declared.x.empty())holes.notes.emplace_back("no potholes on this track");else holes.series={std::move(declared),std::move(front),std::move(rear)};
    holes.x_range=std::pair{0.,track.length_m};Report report{.title="Track preview",.panels={std::move(profile),std::move(holes)},.columns=1,.width=2400,.height=1200};validate(report);return report;
}
std::vector<Report> physical_ride_plots(const Channels& channels) {
    const auto time=detail::required(channels,"time_s");
    const std::array<std::string_view,4> titles{"speed","normal_load","power","energy_residual"},labels{"Speed (m/s)","Normal force (N)","Power (W)","Mechanical residual (J)"};
    const std::array<std::vector<std::string_view>,4> keys{{{"speed_mps"},{"tires.front.normal_load_n","tires.rear.normal_load_n"},{"drive.motor_shaft_power_w"},{"energy.residual_j"}}};
    std::vector<Report> out;
    for(std::size_t i=0;i<titles.size();++i){Plot plot{.title=std::string(titles[i]),.x_label="Force interval start (s)",.y_label=std::string(labels[i]),.series={}};
        for(const auto key:keys[i])add(plot,channels,time,key);
        Report report{.title=std::string(titles[i]),.panels={std::move(plot)},.columns=1,.width=1170,.height=520};
        validate(report);
        out.push_back(std::move(report));
        }
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
    std::ranges::sort(out);return out;
}
std::vector<Report> physical_ride_figures(const Channels& channels) {
    const auto time=detail::required(channels,"time_s"),x=detail::required(channels,"x_m");if(time.size()!=x.size())throw std::invalid_argument("channels time_s/x_m widths differ");
    Report effort{.title="Effort and drivetrain",.panels={{.title="Speed and grade",.x_label="",.y_label="km/h",.series={}},{.title="Power: motor vs human",.x_label="",.y_label="W",.series={}},
        {.title="Cadence vs required",.x_label="",.y_label="rpm",.series={}},{.title="Rear gear and freehub torque",.x_label="",.y_label="teeth",.series={}},{.title="Motor request vs delivered",.x_label="time (s)",.y_label="Nm",.series={}}},.columns=1,.width=1280,.height=1600};
    add(effort.panels[0],channels,time,"speed_kmh","#38bdf8");add(effort.panels[0],channels,time,"grade_pct","#8b949e",true);effort.panels[0].secondary_y_label="%";
    add(effort.panels[1],channels,time,"motor_shaft_power_w","#fbbf24");
    const auto* cadence=numeric(channels,"cadence_rpm"),*human=numeric(channels,"human_sensor_nm");if(cadence&&human){if(cadence->size()!=time.size()||human->size()!=time.size())throw std::invalid_argument("human power inputs differ from time_s width");std::vector<double> power(time.size());
        for(std::size_t i=0;i<power.size();++i)power[i]=(*human)[i]*(*cadence)[i]*(2.*std::numbers::pi/60.);
        effort.panels[1].series.push_back({.name="human_power_w (sensor Nm x cadence)",.x=time,.y=std::move(power),.color="#c084fc"});}
    add(effort.panels[2],channels,time,"cadence_rpm","#34d399");add(effort.panels[2],channels,time,"required_cadence_rpm","#ef4444");
    if(const auto* cut=numeric(channels,"shift_torque_factor")){if(cut->size()!=time.size())throw std::invalid_argument("shift_torque_factor/time_s widths differ");std::vector<std::size_t> starts,ends;
        for(std::size_t i=1;i<cut->size();++i){const bool before=(*cut)[i-1]<.999,after=(*cut)[i]<.999;if(!before&&after)starts.push_back(i-1);if(before&&!after)ends.push_back(i-1);}
        for(std::size_t i=0;i<std::min(starts.size(),ends.size());++i)if(std::isfinite(time[starts[i]])&&std::isfinite(time[ends[i]]))effort.panels[2].markers.push_back({.kind=MarkerKind::band,.first=time[starts[i]],.second=time[ends[i]],.label="shift torque cut"});}
    add(effort.panels[3],channels,time,"gear_rear_teeth","#f97316",false,SeriesStyle::step);add(effort.panels[3],channels,time,"gear_front_teeth","#8b949e",false,SeriesStyle::step);
    add(effort.panels[3],channels,time,"freehub_torque_nm","#c084fc",true);effort.panels[3].secondary_y_label="Nm";
    for(const auto& [shift_time,label]:shift_events(channels))effort.panels[3].markers.push_back({.kind=MarkerKind::vertical,.first=shift_time,.second=0.,.label=label});
    add(effort.panels[4],channels,time,"motor_request_nm","#8b949e");add(effort.panels[4],channels,time,"motor_torque_nm","#fbbf24");
    Report suspension{.title="Suspension and wheel load",.panels={{.title="Suspension travel",.x_label="",.y_label="mm",.series={}},{.title="Wheel normal load",.x_label="track x (m)",.y_label="N",.series={}}},.columns=1,.width=1280,.height=800};
    add(suspension.panels[0],channels,x,"fork_travel_mm","#38bdf8");add(suspension.panels[0],channels,x,"shock_stroke_mm","#fbbf24");
    add(suspension.panels[1],channels,x,"front_load_n","#34d399");add(suspension.panels[1],channels,x,"rear_load_n","#f97316");
    for(const auto& [position,label]:obstacle_positions(channels))for(auto& panel:suspension.panels)panel.markers.push_back({.kind=MarkerKind::vertical,.first=position,.second=0.,.label=label,.color="#a855f7",.full_height=true});
    std::vector<Report> out{std::move(effort),std::move(suspension)};evidence_markers(out,channels,time,x);for(const auto& report:out)validate(report);return out;
}
std::filesystem::path plot_physical_ride_html(const Channels& channels,const std::filesystem::path& out,std::string_view filename) {
    const std::filesystem::path name(filename);if(name.empty()||name.has_parent_path()||name.extension()!=".html")throw std::invalid_argument("ride HTML filename must be a leaf .html path");
    const auto reports=physical_ride_figures(channels);const auto path=out/name;detail::write_text(path,to_html(reports));return path;
}
} // namespace bike::viz
