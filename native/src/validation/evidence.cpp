#include "evidence.hpp"
#include "acceptance.hpp"
#include "../cblas_abi.hpp"
#include "../engine_identity.hpp"
#include "../platform/files.hpp"
#include "../platform/provenance.hpp"
#include "../research/evidence.hpp"
#include "../runtime/value_readers.hpp"
#include <array>
#include <atomic>
#include <bit>
#include <cstdint>
#include <map>
#include <numbers>
#include <system_error>

namespace bike::validation {
namespace {
double dot(std::span<const double> a,std::span<const double> b) {
    if(a.size()!=b.size()||a.size()>static_cast<std::size_t>(std::numeric_limits<int>::max()))throw std::invalid_argument("diagnostic dot size mismatch");
    return 0.+blas::ddot(static_cast<int>(a.size()),a.data(),1,b.data(),1);
}
Value criterion(double actual,double maximum) {
    return Value{Object{{"actual",value::real(actual)},{"maximum",value::real(maximum)},{"passed",Value{std::isfinite(actual)&&actual<=maximum}}}};
}
bool all_checks(const Object& criteria) {
    return std::ranges::all_of(criteria,[](const auto& item){return flag(item.second,"passed");});
}
research::EnergyQuality quality(const Value& energy) {
    auto wire=vread::wire_from_value(energy,"validation.energy");
    return research::energy_quality(std::get<runtime::WireObject>(wire.value));
}
std::vector<std::byte> gzip_stored(std::string_view source) {
    // RFC 1952 gzip containing RFC 1951 stored blocks. Reproducible timestamp,
    // no new compression library, and genuinely readable by gzip/Python.
    std::vector<std::byte> out;
    const auto byte=[&](unsigned x){out.push_back(static_cast<std::byte>(x&255U));};
    for(unsigned const x:{0x1fU,0x8bU,8U,0U,0U,0U,0U,0U,0U,255U})byte(x);
    std::size_t offset=0;
    do { // NOLINT(cppcoreguidelines-avoid-do-while) the terminator DEFLATE block must emit even for an empty source
        const auto count=std::min<std::size_t>(65535,source.size()-offset);
        byte(offset+count==source.size()?1U:0U);
        const auto n=static_cast<unsigned>(count);byte(n);byte(n>>8U);byte(n^65535U);byte((n^65535U)>>8U);
        for(std::size_t i=0;i<count;++i)byte(static_cast<unsigned char>(source[offset+i]));
        offset+=count;
    } while(offset<source.size());
    std::uint32_t crc=0xffffffffU;
    for(const char raw:source){const auto ch=static_cast<std::uint32_t>(static_cast<unsigned char>(raw));crc^=ch;for(int i=0;i<8;++i)crc=(crc>>1U)^(0xedb88320U&(0U-(crc&1U)));}
    crc^=0xffffffffU;
    const auto size=static_cast<std::uint32_t>(source.size()&0xffffffffULL);
    for(unsigned shift=0;shift<32;shift+=8)byte(crc>>shift);
    for(unsigned shift=0;shift<32;shift+=8)byte(size>>shift);
    return out;
}
} // namespace

std::size_t steps(double duration,double dt,double tolerance) {
    positive(duration,"duration");positive(dt,"dt");
    const double ratio=duration/dt;
    if(!std::isfinite(ratio)||ratio>=static_cast<double>(std::numeric_limits<std::int64_t>::max()))throw std::invalid_argument("rig step count exceeds integer range");
    // Python round: ties to even, independent of the process rounding mode.
    double n=std::floor(ratio);const double fraction=ratio-n;
    if(fraction>.5||(fraction==.5&&std::fmod(n,2.)!=0.))n+=1.;
    if(n<1.||std::abs(n*dt-duration)>tolerance)throw std::invalid_argument("rig duration must be an integer number of timesteps");
    return static_cast<std::size_t>(n);
}
void write_report(const std::filesystem::path& file,const Value& report) {
    write_report_text(file,io::canonical_json(report)+"\n");
}
void write_report_text(const std::filesystem::path& file,std::string_view json) {
    if(file.empty())throw std::invalid_argument("validation report path is empty");
    const auto parent=file.has_parent_path()?file.parent_path():std::filesystem::path(".");
    std::filesystem::create_directories(parent);
    const auto raw=std::as_bytes(std::span(json.data(),json.size()));
    auto payload=file.extension()==".gz"?gzip_stored(json):std::vector<std::byte>(raw.begin(),raw.end());
    std::error_code destination_error;const auto destination=std::filesystem::symlink_status(file,destination_error);
    if(!destination_error&&std::filesystem::exists(destination)){const auto opened=platform::open_regular(file);(void)opened;}
    static std::atomic<std::uint64_t> counter{0};
    std::filesystem::path staged;platform::FileHandle handle;
    for(unsigned attempt=0;attempt<128;++attempt) {
        staged=parent/(file.filename().string()+".writing-"+std::to_string(counter.fetch_add(1)));
        try {handle=platform::open_exclusive(staged);break;}
        catch(const std::system_error& error) {if(error.code()!=std::errc::file_exists)throw;}
    }
    if(!handle.valid())throw std::runtime_error("cannot create report staging file");
    try {
        platform::write_all(handle,payload);platform::sync(handle);handle.close();
        std::filesystem::rename(staged,file);platform::sync_directory(parent);
    } catch(...) {std::error_code ignored;std::filesystem::remove(staged,ignored);throw;}
}
std::string source_fingerprint(const std::filesystem::path& root) {
    if(!std::filesystem::is_directory(root))throw std::invalid_argument("source fingerprint requires a source directory");
    std::vector<std::filesystem::path> files;
    for(const auto& entry:std::filesystem::recursive_directory_iterator(root)) {
        if(!entry.is_regular_file())continue;const auto ext=entry.path().extension();
        if(ext==".cpp"||ext==".hpp"||ext==".h"||ext==".cmake"||entry.path().filename()=="CMakeLists.txt")files.push_back(entry.path());
    }
    std::ranges::sort(files,[&](const auto& a,const auto& b){return a.lexically_relative(root).generic_string()<b.lexically_relative(root).generic_string();});
    if(files.empty())throw std::invalid_argument("source fingerprint selected no native source files");
    io::Sha256 digest;
    const auto add=[&](std::string_view bytes){
        std::array<std::byte,8> length{};const auto n=static_cast<std::uint64_t>(bytes.size());
        for(unsigned i=0;i<8;++i)length[i]=static_cast<std::byte>((n>>(56U-8U*i))&255U);
        digest.update(length);digest.update(std::as_bytes(std::span(bytes.data(),bytes.size())));
    };
    for(const auto& file:files){add(file.lexically_relative(root).generic_string());add(read_text(file));}
    const auto bytes=digest.finish();std::string result;constexpr std::string_view hex="0123456789abcdef";
    for(auto const b:bytes){result.push_back(hex[b>>4U]);result.push_back(hex[b&15U]);}return result;
}
Value environment_contract(const std::filesystem::path& root) {
    const auto build=platform::build_identity();const auto loaded=engine::identity();
    const auto path=root/"native/dependencies.lock.json";
    const bool has_lock=std::filesystem::is_regular_file(path);
    const auto lock_sha=has_lock?file_hash(path):std::string{};
    bool engine_match=false;
    if(has_lock) {
        const auto lock=read_json(path);const auto& dependency=get(get(lock,"dependencies"),"mujoco");
        const auto* lane=find(get(dependency,"platforms"),build.platform);
        if(lane&&text(dependency,"version")==loaded.mujoco_version) {
            for(const auto& [name,digest]:io::as_object(get(*lane,"required_files"),"required files"))
                if(name.starts_with("lib/")&&io::as_string(digest,name)==loaded.library_sha256)engine_match=true;
        }
    }
    const bool lock_match=has_lock&&lock_sha==build.dependency_lock_sha256;
    return Value{Object{{"platform",value::text(build.platform)},{"compiler",value::text(build.compiler)},
        {"mujoco_version",value::text(loaded.mujoco_version)},{"mujoco_sha256",value::text(loaded.library_sha256)},
        {"numeric_provider",value::text(loaded.numeric_provider)},{"executable_sha256",value::text(build.executable_sha256)},
        {"dependency_lock_sha256",has_lock?value::text(lock_sha):Value{}},{"compiled_dependency_lock_sha256",value::text(build.dependency_lock_sha256)},
        {"declared_requirements_satisfied",Value{engine_match}},{"locked_versions_match",Value{lock_match}},
        {"locked_environment_verified",Value{engine_match&&lock_match}},
        {"scope",value::text("Native build lock identity and loaded MuJoCo image; runtime numerics identity remains separately recorded, not an empirical calibration claim.")}}};
}

IntervalCoverage::IntervalCoverage(std::int64_t first_interval,double start_time_s)
    :next_interval_(first_interval),previous_end_(start_time_s) {
    if(first_interval<0||!std::isfinite(start_time_s)||start_time_s<0.)
        throw std::invalid_argument("interval coverage requires a valid starting boundary");
}
void IntervalCoverage::observe(const runtime::PhysicalSampleData& sample) {
    // Same continuity tolerance as runtime::PeriodBuffer; this guard verifies
    // publication coverage without changing physical timestamps or weights.
    if(sample.interval_id!=next_interval_||!std::isfinite(sample.time_s)||sample.time_s<0.||
       !std::isfinite(sample.end_time_s)||sample.end_time_s<=sample.time_s||
       std::abs(sample.time_s-previous_end_)>1e-10)
        throw std::invalid_argument("validation sample coverage has a gap, duplicate, or invalid interval");
    if(next_interval_==std::numeric_limits<std::int64_t>::max())
        throw std::invalid_argument("validation interval ID overflow");
    ++next_interval_;previous_end_=sample.end_time_s;
}
void IntervalCoverage::finish(std::int64_t final_step,double end_time_s) const {
    if(final_step!=next_interval_||!std::isfinite(end_time_s)||
       std::abs(previous_end_-end_time_s)>1e-10)
        throw std::invalid_argument("validation samples do not cover every committed physics interval");
}
bool ValidPrefix::observe(const runtime::PhysicalSampleData& sample) {
    const double dt=sample.end_time_s-sample.time_s;
    if(!std::isfinite(dt)||dt<=0.)throw std::invalid_argument("audit needs a positive finite interval");
    ++seen;const auto channels=research::wire_value(runtime::Wire(sample.channels));
    const auto& status=get(channels,"model_status");std::vector<std::string> reasons;
    for(const auto& [name,bad,unknown]:std::array{
        std::tuple{"model_valid","model_violation","model_status_unavailable"},
        std::tuple{"numerically_valid","numerical_quality","numerical_not_evaluated"}}) {
        const auto check=optional(status,name);if(!true_value(check))reasons.emplace_back(false_value(check)?bad:unknown);
    }
    if(flag(channels,"rider_effort_budget_exceeded"))reasons.emplace_back("rider_effort_budget");
    if(!reasons.empty()&&io::is_null(first_bad))first_bad=Value{Object{{"time_s",value::real(sample.time_s)},
        {"reasons",value::strings(reasons)},{"model_event",optional(status,"first_model_violation")}}};
    if(!io::is_null(first_bad))return false;
    const double front=nonnegative(real(get(get(channels,"tires"),"front"),"normal_load_n"),"observed normal load");
    const double rear=nonnegative(real(get(get(channels,"tires"),"rear"),"normal_load_n"),"observed normal load");
    ++accepted;valid_until_s=sample.end_time_s;front_normal_impulse_ns+=front*dt;rear_normal_impulse_ns+=rear*dt;return true;
}
double trace_integral(std::span<const double> time,std::span<const double> values) {
    if(time.size()!=values.size()||time.size()<2)throw std::invalid_argument("trace needs paired scalar samples");
    std::vector<double> products;
    for(std::size_t i=0;i<time.size();++i) {
        scalar(time[i],"trace time");scalar(values[i],"trace value");
        if(i){if(time[i]<=time[i-1])throw std::invalid_argument("trace must have finite strictly increasing times");products.push_back(.5*(values[i]+values[i-1])*(time[i]-time[i-1]));}
    }
    return numeric::numpy_sum(products);
}
double interval_integral(const Value& rows,std::string_view key) {
    double result=0.;std::optional<double> previous;
    for(const auto& row:io::as_array(rows,"rows")) {
        const double start=real(row,"time_s"),end=real(row,"end_time_s"),x=real(row,key);
        if(end<=start)throw std::invalid_argument("invalid force interval");
        if(previous&&std::abs(start-*previous)>1e-9)throw std::invalid_argument("gap or overlap in force history");
        result+=x*(end-start);previous=end;
    }
    if(!std::isfinite(result))throw std::invalid_argument("interval integral overflow");return result;
}
double relative_change(double a,double b,double floor) {scalar(a,"value");scalar(b,"reference");positive(floor,"comparison scale");return std::abs(a-b)/std::max(std::abs(b),floor);}
Value rider_work_ledger(const Value& work) {
    numeric::PythonSum active;
    for(const auto& [name,item]:io::as_object(work,"component work")){const auto x=io::as_double(item,name);if(name.starts_with("act_rider_"))active.add(x);}
    const auto passive=real(work,"rider_passive_damping",0.);
    return Value{Object{{"active_joint_work_j",value::real(active.total())},{"passive_joint_work_j",value::real(passive)},
        {"net_joint_work_j",value::real(active.total()+passive)},{"motor_mechanical_work_j",value::real(real(work,"mid_drive",0.))}}};
}

// climbing_evidence.py:7-111: unknown status and interrupted histories never become valid.
Value support_evidence(const Value& input,std::optional<double> required) {
    if(required)positive(*required,"required duration");const auto& rows=io::as_array(input,"rows");
    Object out{{"interval_count",value::integer(static_cast<std::int64_t>(rows.size()))},{"complete",Value{false}},
        {"first_both_unloaded_s",{}},{"max_both_unloaded_s",value::real(0.)},{"max_front_gap_m",value::real(0.)},{"max_rear_gap_m",value::real(0.)},
        {"first_invalid_s",{}},{"invalid_reason",Value{Array{}}},{"first_unknown_s",{}},{"model_valid",{}},{"numerically_valid",{}},
        {"valid_for_learning",Value{false}},{"max_energy_residual_ratio",{}},{"support_observed",Value{!rows.empty()}},
        {"all_invalid_reasons",Value{Array{}}},{"invalid_reason_counts",empty_object()},
        {"valid_prefix_duration_s",value::real(0.)},{"valid_prefix_max_gap_m",value::real(0.)}};
    std::optional<double> previous,loss_start;double loss_duration=0.;
    bool model_known=!rows.empty(),numerical_known=!rows.empty(),model_valid=true,numerical_valid=true;
    bool prefix=!rows.empty()&&std::abs(real(rows.front(),"time_s"))<=1e-9;
    auto const read_out=[&](std::string_view name){return io::as_double(*io::find_member(out,name),name);};
    std::map<std::string,std::int64_t> counts;
    for(const auto& row:rows) {
        const double start=real(row,"time_s"),end=real(row,"end_time_s");
        if(start<0.||end<=start)throw std::invalid_argument("support evidence requires finite ordered intervals");
        if(previous&&std::abs(start-*previous)>1e-9)throw std::invalid_argument("support evidence requires contiguous intervals");previous=end;
        bool available=false,unknown=false,support_unknown=false;double gap_max=0.;
        const auto rider=optional(row,"rider",empty_object());
        for(const auto side:{"front","rear"}) {
            const auto pedal=optional(rider,std::string(side)+"_pedal",empty_object());
            const auto gap=optional(pedal,"gap_m"),load=optional(pedal,"normal_load_n");
            if(io::is_null(gap)||io::is_null(load)||!find(pedal,"in_platform")){value::set(out,"support_observed",Value{false});unknown=true;support_unknown=true;continue;}
            const double g=io::as_double(gap,"pedal gap"),f=nonnegative(io::as_double(load,"pedal load"),"pedal load");
            const auto key=std::string("max_")+side+"_gap_m";value::set(out,key,value::real(std::max(read_out(key),g)));gap_max=std::max(gap_max,g);
            available=available||(flag(pedal,"in_platform")&&f>1.);
        }
        if(available||support_unknown){loss_start.reset();loss_duration=0.;}
        else {
            if(!loss_start)loss_start=start;loss_duration+=end-start;
            value::set(out,"max_both_unloaded_s",value::real(std::max(read_out("max_both_unloaded_s"),loss_duration)));
            if(loss_duration>=.20-1e-12&&io::is_null(*io::find_member(out,"first_both_unloaded_s")))value::set(out,"first_both_unloaded_s",value::real(*loss_start));
        }
        std::vector<std::string> reasons;const auto status=optional(row,"model_status",empty_object());const auto valid=optional(status,"model_valid");
        if(false_value(valid)){model_valid=false;reasons.emplace_back("model_scope");}else if(!true_value(valid)){model_known=false;unknown=true;}
        const auto energy=optional(row,"energy",empty_object());
        if(!find(energy,"residual_j")||!find(energy,"energy_scale_j")){numerical_known=false;unknown=true;}
        else {
            try {
                const auto q=quality(energy);const auto* previous_peak=io::find_member(out,"max_energy_residual_ratio");
                const double peak=io::is_null(*previous_peak)?0.:io::as_double(*previous_peak,"energy residual");
                value::set(out,"max_energy_residual_ratio",value::real(std::max(peak,q.residual_ratio)));
                if(!q.acceptable){numerical_valid=false;reasons.emplace_back("energy_quality");}
            } catch(const std::exception&) {numerical_valid=false;reasons.emplace_back("invalid_energy");}
        }
        if(false_value(optional(status,"numerically_valid"))){numerical_valid=false;reasons.emplace_back("latched_numerical_failure");}
        if(!reasons.empty()&&io::is_null(*io::find_member(out,"first_invalid_s"))){value::set(out,"first_invalid_s",value::real(start));value::set(out,"invalid_reason",value::strings(reasons));}
        for(const auto& reason:reasons)++counts[reason];
        if(unknown||!reasons.empty())prefix=false;
        if(prefix){value::set(out,"valid_prefix_duration_s",value::real(read_out("valid_prefix_duration_s")+(end-start)));value::set(out,"valid_prefix_max_gap_m",value::real(std::max(read_out("valid_prefix_max_gap_m"),gap_max)));}
        if(unknown&&io::is_null(*io::find_member(out,"first_unknown_s")))value::set(out,"first_unknown_s",value::real(start));
    }
    value::set(out,"model_valid",!model_valid?Value{false}:model_known?Value{true}:Value{});
    value::set(out,"numerically_valid",!numerical_valid?Value{false}:numerical_known?Value{true}:Value{});
    Object count_values;std::vector<std::string> names;for(const auto& [name,count]:counts){names.push_back(name);count_values.emplace_back(name,value::integer(count));}
    value::set(out,"all_invalid_reasons",value::strings(names));value::set(out,"invalid_reason_counts",Value{std::move(count_values)});
    value::set(out,"valid_for_learning",Value{model_valid&&model_known&&numerical_valid&&numerical_known&&true_value(*io::find_member(out,"support_observed"))});
    value::set(out,"complete",Value{!rows.empty()&&required&&std::abs(real(rows.front(),"time_s"))<=1e-9&&std::abs(real(rows.back(),"end_time_s")-*required)<=1e-8});
    return Value{std::move(out)};
}
Value coasting_evidence(const Value& input) {
    bool entered=false,resumed=false;double loss=0.,maximum_loss=0.,maximum_gap=0.,stopped_gap=0.,stopped_duration=0.,rotation=0.,work=0.;Value first_loss;
    for(const auto& row:io::as_array(input,"rows")) {
        const double interval=positive(real(row,"end_time_s")-real(row,"time_s"),"coasting diagnostic interval");const auto& drive=get(row,"drive");
        const bool coasting=text(drive,"rider_mode")=="coasting";entered=entered||coasting;
        bool available=false;double gap=0.;
        for(const auto side:{"front","rear"}){const auto& p=get(get(row,"rider"),std::string(side)+"_pedal");available=available||(flag(p,"in_platform")&&real(p,"normal_load_n")>1.);gap=std::max(gap,real(p,"gap_m"));}
        if(coasting){maximum_gap=std::max(maximum_gap,gap);if(real(get(get(row,"support"),"crank_tracking"),"target_rate_rad_s")==0.){stopped_gap=std::max(stopped_gap,gap);stopped_duration+=interval;}}
        loss=coasting&&!available?loss+interval:0.;maximum_loss=std::max(maximum_loss,loss);
        if(loss>=.20&&io::is_null(first_loss))first_loss=value::real(real(row,"end_time_s")-loss);
        if(entered&&text(drive,"rider_mode")=="pedaling"){resumed=true;const double rate=real(drive,"crank_rad_s");rotation+=rate*interval;work+=std::max(0.,real(drive,"human_sensor_nm")*rate)*interval;}
    }
    return Value{Object{{"coast_entered",Value{entered}},{"pedaling_resumed",Value{resumed}},{"max_both_unloaded_coast_s",value::real(maximum_loss)},
        {"max_coasting_gap_m",value::real(maximum_gap)},{"max_stopped_coasting_gap_m",value::real(stopped_gap)},{"stopped_coast_duration_s",value::real(stopped_duration)},
        {"resume_crank_turns",value::real(rotation/(2.*std::numbers::pi))},{"resume_positive_work_j",value::real(work)},{"first_support_loss",std::move(first_loss)}}};
}
Value resume_evidence(const Value& input,double end) {
    if(!std::isfinite(end)||end<=4.)throw std::invalid_argument("resume observation must end after restart");
    std::vector<const Value*> rows;for(const auto& row:io::as_array(input,"rows"))if(real(row,"end_time_s")>4.&&real(row,"time_s")<end)rows.push_back(&row);
    Object result{{"window_s",pair(4.,end)},{"interval_count",value::integer(static_cast<std::int64_t>(rows.size()))},
        {"positive_crank_work_j",value::real(0.)},{"crank_rotation_rad",value::real(0.)},{"diagnosis",value::text("mixed_or_unresolved")}};
    if(rows.empty())return Value{std::move(result)};
    std::vector<double> durations,speeds,torques,requests,phases,positive_work,unavailable,saturated,gated,abs_speed;
    double correction=0.,last_raw=0.;
    for(std::size_t i=0;i<rows.size();++i) {
        const auto& row=*rows[i];const auto& drive=get(row,"drive");
        const double dt=positive(std::min(end,real(row,"end_time_s"))-std::max(4.,real(row,"time_s")),"diagnostic interval");durations.push_back(dt);
        const double speed=real(drive,"crank_rad_s",real(drive,"cadence_rpm",0.)*2.*std::numbers::pi/60.);speeds.push_back(speed);abs_speed.push_back(std::abs(speed));
        const double torque=real(drive,"human_sensor_nm",0.);torques.push_back(torque);
        const auto control=optional(row,"control",empty_object());const auto command=optional(control,"human_torque_nm");
        const double request=!io::is_null(command)&&io::as_double(command,"human torque")!=0.?io::as_double(command,"human torque"):real(drive,"human_command_nm",0.);requests.push_back(request);
        const double raw=real(drive,"crank_phase_rad");
        if(i){const double delta=raw-last_raw;double wrapped=std::fmod(delta+std::numbers::pi,2.*std::numbers::pi);if(wrapped<0.)wrapped+=2.*std::numbers::pi;wrapped-=std::numbers::pi;if(wrapped==-std::numbers::pi&&delta>0.)wrapped=std::numbers::pi;if(std::abs(delta)>=std::numbers::pi)correction+=wrapped-delta;}
        phases.push_back(raw+correction);last_raw=raw;positive_work.push_back(std::max(torque*speed,0.)*dt);
        bool available=false;const auto rider=optional(row,"rider",empty_object());
        for(const auto side:{"front","rear"}){const auto pedal=optional(rider,std::string(side)+"_pedal",empty_object());available=available||(flag(pedal,"in_platform")&&real(pedal,"normal_load_n",0.)>1.);}
        unavailable.push_back(request>0.&&!available?1.:0.);bool sat=false;
        for(const auto& [name,term]:io::as_object(optional(row,"joint_terms",empty_object()),"joint terms")){(void)name;sat=sat||flag(term,"saturated");}
        saturated.push_back(sat?1.:0.);gated.push_back(torque>0.&&text(drive,"motor_control_source")=="assist"&&flag(drive,"assist_demand_gated")?1.:0.);
    }
    const double duration=numeric::numpy_sum(durations),rotation=phases.back()-phases.front()+speeds.back()*durations.back(),work=numeric::numpy_sum(positive_work);
    const double no_support=dot(durations,unavailable)/duration,sat=dot(durations,saturated)/duration,gate=dot(durations,gated)/duration;
    const double peak=*std::ranges::max_element(requests),mean_speed=dot(abs_speed,durations)/duration;
    for(const auto& [key,x]:std::array{std::pair{"support_unavailable_fraction",no_support},std::pair{"actuator_saturated_fraction",sat},std::pair{"assist_gated_fraction",gate},
        std::pair{"window_duration_s",duration},std::pair{"positive_crank_work_j",work},std::pair{"crank_rotation_rad",rotation},std::pair{"requested_effort_peak_nm",peak},
        std::pair{"delivered_human_mean_nm",dot(torques,durations)/duration},std::pair{"mean_abs_crank_speed_rad_s",mean_speed}})value::set(result,key,value::real(x));
    if(duration<end-4.-.01)value::set(result,"incomplete_window",Value{true});
    else if(work>0.&&rotation>=2.*std::numbers::pi)value::set(result,"diagnosis",value::text("resumed"));
    else if(peak<=0.)value::set(result,"diagnosis",value::text("no_effort_request"));
    else {
        std::vector<std::string> causes;if(no_support>.5)causes.emplace_back("support_unavailable");if(sat>.5)causes.emplace_back("actuator_saturated");if(gate>.5)causes.emplace_back("assist_gated");
        if(causes.empty()&&peak>0.&&mean_speed<.2&&*std::ranges::max_element(torques)>0.)causes.emplace_back("mechanically_stalled");
        value::set(result,"candidate_causes",value::strings(causes));if(causes.size()==1)value::set(result,"diagnosis",value::text(causes.front()));
    }
    return Value{std::move(result)};
}
Value timing_metrics(std::span<const double> step_ns,double dt) {
    positive(dt,"timed physics interval");if(step_ns.empty())throw std::invalid_argument("timing requires at least one completed step");
    std::vector<double> sorted(step_ns.begin(),step_ns.end());for(double const x:sorted)positive(x,"step nanoseconds");std::ranges::sort(sorted);
    const auto percentile=[&](double q){const double index=static_cast<double>(sorted.size()-1)*q;const auto low=static_cast<std::size_t>(index),high=std::min(low+1,sorted.size()-1);const double fraction=index-static_cast<double>(low);const double difference=sorted[high]-sorted[low];return (fraction<.5?sorted[low]+difference*fraction:sorted[high]-difference*(1.-fraction))/1e6;};
    return Value{Object{{"step_count",value::integer(static_cast<std::int64_t>(step_ns.size()))},{"p50_step_ms",value::real(percentile(.5))},
        {"p95_step_ms",value::real(percentile(.95))},{"p99_step_ms",value::real(percentile(.99))},{"maximum_step_ms",value::real(sorted.back()/1e6)},
        {"real_time_factor",value::real(static_cast<double>(step_ns.size())*dt/(numeric::numpy_sum(step_ns)/1e9))}}};
}
Value persistent_events(const Value& input,double persistence,bool normal_only) {
    positive(persistence,"event persistence");Array events;std::optional<std::string> accepted,pending;double start=0.;
    for(const auto& row:io::as_array(input,"rows")) {
        const std::string state=normal_only?(real(row,"normal_load_n")>10.?"loaded":"unloaded"):text(row,"contact_state");
        if(!pending||state!=*pending){pending=state;start=real(row,"time_s");}
        if((!accepted||state!=*accepted)&&real(row,"end_time_s")-start+1e-12>=persistence) {
            Object event{{"state",value::text(state)},{"time_s",value::real(start)}};if(!normal_only)event.emplace_back("confirmed_at_s",get(row,"end_time_s"));
            events.push_back(Value{std::move(event)});accepted=state;
        }
    }return Value{std::move(events)};
}
Value contact_comparison(const Value& a,const Value& b) {
    Object checks,errors;for(const auto key:{"normal_impulse_ns","vertical_impulse_ns"}){const double e=relative_change(real(a,key),real(b,key),.1);checks.emplace_back(key,Value{e<=.02});errors.emplace_back(key,value::real(e));}
    const auto& x=io::as_array(get(a,"contact_events"),"events");const auto& y=io::as_array(get(b,"contact_events"),"events");bool matched=x.size()==y.size();double event=0.;
    if(matched)for(std::size_t i=0;i<x.size();++i){matched=matched&&text(x[i],"state")==text(y[i],"state");event=std::max(event,std::abs(real(x[i],"time_s")-real(y[i],"time_s")));}
    checks.emplace_back("events",Value{matched&&event<=.005});checks.emplace_back("energy",Value{std::max(real(a,"energy_residual_ratio"),real(b,"energy_residual_ratio"))<=.05});
    checks.emplace_back("no_native_contact",Value{real(a,"native_wheel_contact_count")==0.&&real(b,"native_wheel_contact_count")==0.});
    const bool passed=std::ranges::all_of(checks,[](const auto& entry){return true_value(entry.second);});
    return Value{Object{{"passed",Value{passed}},{"criteria",Value{std::move(checks)}},{"relative_errors",Value{std::move(errors)}},{"event_error_s",matched?value::real(event):Value{}}}};
}
Value compare_metrics(const Value& a,const Value& b) {
    Object criteria;for(const auto key:{"front_normal_impulse_ns","rear_normal_impulse_ns","front_vertical_impulse_ns","rear_vertical_impulse_ns"})criteria.emplace_back(key,criterion(relative_change(real(a,key),real(b,key),.1),.02));
    for(const auto side:{"front","rear"})for(const auto metric:{"mean_load_n","min_load_n"}){const auto key=std::string(side)+"_"+metric;criteria.emplace_back(key,criterion(relative_change(real(a,key),real(b,key),10.),.02));}
    for(const auto key:{"mean_fork_travel_m","max_fork_travel_m","mean_shock_stroke_m","max_shock_stroke_m"})criteria.emplace_back(key,criterion(std::abs(real(a,key)-real(b,key)),.001));
    criteria.emplace_back("peak_pitch_rate_rad_s",criterion(relative_change(real(a,"peak_pitch_rate_rad_s"),real(b,"peak_pitch_rate_rad_s"),.05),.05));
    const auto& x=io::as_array(get(a,"contact_events"),"events");const auto& y=io::as_array(get(b,"contact_events"),"events");bool matched=x.size()==y.size();double error=0.;Array actual,reference;
    for(const auto& entry:x)actual.push_back(get(entry,"state"));for(const auto& entry:y)reference.push_back(get(entry,"state"));
    if(matched)for(std::size_t i=0;i<x.size();++i){matched=matched&&text(x[i],"state")==text(y[i],"state");error=std::max(error,std::abs(real(x[i],"time_s")-real(y[i],"time_s")));}
    criteria.emplace_back("event_sequence",Value{Object{{"passed",Value{matched}},{"actual",Value{std::move(actual)}},{"reference",Value{std::move(reference)}}}});
    criteria.emplace_back("contact_event_time_s",matched?criterion(error,.005):Value{Object{{"passed",Value{false}},{"actual",{}},{"maximum",value::real(.005)},{"reason",value::text("categorical mismatch")}}});
    criteria.emplace_back("full_duration",Value{Object{{"passed",Value{flag(a,"complete")&&flag(b,"complete")&&std::abs(real(a,"duration_s")-real(b,"duration_s"))<1e-9}}}});
    criteria.emplace_back("model_scope",Value{Object{{"passed",Value{flag(a,"model_valid")&&flag(b,"model_valid")}},{"agreement",Value{io::canonical_json(optional(a,"model_valid"))==io::canonical_json(optional(b,"model_valid"))}}}});
    criteria.emplace_back("energy_quality",Value{Object{{"passed",Value{flag(a,"numerically_valid")&&flag(b,"numerically_valid")}},{"coarse_max_residual_ratio",optional(a,"max_energy_residual_ratio")},{"reference_max_residual_ratio",optional(b,"max_energy_residual_ratio")}}});
    const auto initial=optional(a,"initial_state_sha256");
    criteria.emplace_back("identical_initial_state",Value{Object{{"passed",Value{!io::is_null(initial)&&io::canonical_json(initial)==io::canonical_json(optional(b,"initial_state_sha256"))}}}});
    std::vector<std::string> failed;for(const auto& [name,c]:criteria)if(!flag(c,"passed"))failed.push_back(name);
    const bool passed=all_checks(criteria);return Value{Object{{"passed",Value{passed}},{"criteria",Value{std::move(criteria)}},{"failed_criteria",value::strings(failed)}}};
}
Value metrics_from_rows(const Value& input,double expected,std::string_view initial) {
    const auto& rows=io::as_array(input,"rows");if(rows.empty())throw std::invalid_argument("no completed physics intervals");
    Array ones,motor,human;double peak_rate=0.,max_pitch=-std::numeric_limits<double>::infinity(),residual=0.;bool valid=true,numeric_valid=true;
    for(const auto& row:rows){auto one=io::as_object(row,"row");value::set(one,"one",value::real(1.));ones.push_back(Value{std::move(one)});auto m=io::as_object(row,"row"),h=m; // NOLINT(bugprone-use-after-move) m is reinitialized each iteration; the copy precedes the move
        value::set(m,"power",value::real(real(row,"motor_torque_nm")*real(row,"drive_shaft_rad_s",real(row,"crank_rad_s"))));motor.push_back(Value{std::move(m)});
        value::set(h,"power",value::real(real(row,"human_torque_nm")*real(row,"crank_rad_s")));human.push_back(Value{std::move(h)});
        peak_rate=std::max(peak_rate,std::abs(real(row,"pitch_rate_rad_s")));max_pitch=std::max(max_pitch,real(row,"pitch_rad"));residual=std::max(residual,real(row,"energy_residual_ratio"));
        valid=valid&&flag(get(row,"model_status"),"model_valid");numeric_valid=numeric_valid&&flag(get(row,"model_status"),"numerically_valid")&&real(row,"energy_residual_ratio")<=.05;
    }
    const double duration=interval_integral(Value{std::move(ones)},"one");Object result{{"duration_s",value::real(duration)},{"complete",Value{std::abs(duration-expected)<1e-8}},
        {"initial_state_sha256",value::text(initial)},{"contact_events",persistent_events(input)},{"peak_pitch_rate_rad_s",value::real(peak_rate)},
        {"model_valid",Value{valid}},{"numerically_valid",Value{numeric_valid}},{"max_energy_residual_ratio",value::real(residual)},
        {"joint_positive_work_j",value::real(interval_integral(input,"joint_positive_power_w"))},{"motor_shaft_work_j",value::real(interval_integral(Value{std::move(motor)},"power"))},
        {"human_crank_work_j",value::real(interval_integral(Value{std::move(human)},"power"))},{"max_pitch_rad",value::real(max_pitch)},
        {"com_dx_m",value::real(real(rows.back(),"com_x_m")-real(rows.front(),"com_x_m"))},{"com_dz_m",value::real(real(rows.back(),"com_z_m")-real(rows.front(),"com_z_m"))}};
    for(const auto side:{"front","rear"}) {
        const auto prefix=std::string(side);const auto load=prefix+"_load_n";const double impulse=interval_integral(input,load);double minimum=std::numeric_limits<double>::infinity();
        for(const auto& row:rows)minimum=std::min(minimum,real(row,load));
        result.emplace_back(prefix+"_normal_impulse_ns",value::real(impulse));result.emplace_back(prefix+"_vertical_impulse_ns",value::real(interval_integral(input,prefix+"_vertical_force_n")));
        result.emplace_back(prefix+"_mean_load_n",value::real(impulse/duration));result.emplace_back(prefix+"_min_load_n",value::real(minimum));
    }
    for(const auto key:{"fork_travel_m","shock_stroke_m"}){double maximum=-std::numeric_limits<double>::infinity();for(const auto& row:rows)maximum=std::max(maximum,real(row,key));result.emplace_back(std::string("mean_")+key,value::real(interval_integral(input,key)/duration));result.emplace_back(std::string("max_")+key,value::real(maximum));}
    return Value{std::move(result)};
}
Value resolution_variants(const Value& times,const Value& roads,const Value& stations,bool include_stations) {
    for(const auto& count:io::as_array(stations,"station counts"))
        (void)io::as_int64(count,"station count");
    const auto t=numbers(times,"time steps"),x=numbers(roads,"road steps"),n=numbers(stations,"station counts");
    const std::array axes{t,x,n};
    for(std::size_t i=0;i<axes.size();++i){const auto& a=axes[i];if(a.size()<3)throw std::invalid_argument("resolution axes require at least three distinct positive values");
        for(std::size_t j=0;j<a.size();++j){positive(a[j],"resolution value");if(i==2&&(a[j]<16.||std::trunc(a[j])!=a[j]))throw std::invalid_argument("station counts must be integers of at least sixteen");if(j&&((i!=2&&a[j]>=a[j-1])||(i==2&&a[j]<=a[j-1])))throw std::invalid_argument("time and road steps decrease; station counts increase");}}
    const double dt=t[t.size()-2],dx=x[x.size()-2],count=n[n.size()-2];Array variants;
    const auto add=[&](std::string_view axis,double d,double r,double s){variants.push_back(Value{Array{value::text(axis),value::real(d),value::real(r),value::integer(static_cast<std::int64_t>(s))}});};
    for(double const d:t)add("time",d,dx,count);for(double const r:x)add("road",dt,r,count);if(include_stations)for(double const s:n)add("stations",dt,dx,s);
    return Value{Object{
        {"anchor",Value{Array{value::real(dt),value::real(dx),value::integer(static_cast<std::int64_t>(count))}}},
        {"variants",Value{std::move(variants)}}
    }};
}
} // namespace bike::validation
