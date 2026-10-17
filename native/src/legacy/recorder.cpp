#include "ride.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <locale>
#include <ranges>
#include <sstream>
#include <stdexcept>
namespace bike::legacy {
namespace {
void csv_rows(std::ostream& out,std::span<const TelemetryRow> rows){
    out.imbue(std::locale::classic());out<<std::setprecision(10)<<std::defaultfloat;
    for(std::size_t i=0;i<kLegacyChannels.size();++i){if(i)out<<',';out<<kLegacyChannels[i];}out<<'\n';
    for(const auto& row:rows){for(std::size_t i=0;i<row.size();++i){if(i)out<<',';if(std::isnan(row[i]))out<<"nan";else out<<row[i];}out<<'\n';}
}
}
RideRecorder::RideRecorder(double timestep,std::size_t decimate):timestep_(timestep),decimate_(decimate) {
    if(!std::isfinite(timestep)||timestep<=0.||decimate==0)throw std::invalid_argument("legacy recorder requires positive timestep/decimation");
}
void RideRecorder::record(const RideSimulation& sim){
    // Observe every callback before decimation. A completed impact may
    // begin and end between two CSV rows and must still be reported.
    if(const auto outputs=sim.tyre_outputs())for(std::size_t i=0;i<2;++i){
        const auto& t=(*outputs)[i];auto& history=tyres_[i];
        if(t.rim_strike_active&&!rim_active_[i]){rim_start_time_[i]=sim.time_s();history.starts_m.push_back(t.hub_position_world_m[0]);}
        rim_active_[i]=t.rim_strike_active;
        if(sim.position_m()>=sim.start_x_m()+8.){
            bool spin=false,locked=false;for(const auto& patch:t.patches()){
                spin=spin||(patch.fully_sliding&&patch.slip_ratio>0.);locked=locked||(patch.fully_sliding&&patch.slip_ratio<0.);}
            if(spin)history.wheelspin_time_s+=sim.timestep_s();if(locked)history.locked_time_s+=sim.timestep_s();
        }
        if(t.rim_event){const auto& e=*t.rim_event;history.events.push_back({e.x_m,e.speed_mps,e.peak_load_n,e.peak_rim_force_n,e.absorbed_energy_j,rim_start_time_[i],sim.time_s()});}
    }
    record(sim.telemetry());
}
void RideRecorder::record(const TelemetryRow& row){const bool keep=calls_%decimate_==0;if(keep)rows_.push_back(row);++calls_;}
double RideRecorder::sample_interval_s()const noexcept{return timestep_*static_cast<double>(decimate_);}
std::vector<double> RideRecorder::column(std::string_view name)const {
    const auto found=std::ranges::find(kLegacyChannels,name);
    if(found==kLegacyChannels.end())throw std::invalid_argument("unknown legacy recorder column '"+std::string(name)+"'");
    const auto column=static_cast<std::size_t>(found-kLegacyChannels.begin());
    std::vector<double> result;result.reserve(rows_.size());for(const auto& row:rows_)result.push_back(row[column]);return result;
}
Channels RideRecorder::columns()const {
    Channels result;result.reserve(kLegacyChannels.size());
    for(auto name:kLegacyChannels)result.emplace_back(std::string(name),column(name));return result;
}
std::string RideRecorder::to_csv()const{std::ostringstream out;csv_rows(out,rows_);return out.str();}
void RideRecorder::write_csv(const std::filesystem::path& path)const {
    if(!path.parent_path().empty())std::filesystem::create_directories(path.parent_path());
    std::ofstream out(path);if(!out)throw std::runtime_error("cannot write legacy telemetry CSV: "+path.string());
    csv_rows(out,rows_);
    out.flush();if(!out)throw std::runtime_error("legacy telemetry CSV write failed: "+path.string());
}
} // namespace bike::legacy
