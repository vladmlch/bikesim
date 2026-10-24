#include "plan.hpp"
#include "../cli/arguments.hpp"
#include "../io/json.hpp"
#include "../runtime/setup_values.hpp"
#include <cmath>
#include <map>
#include <algorithm>

namespace bike::batch {
using namespace runtime::setupvalue;
io::Value tail_json(std::string_view text) {
    const auto start=text.rfind("\n{");
    if(start!=std::string_view::npos)text.remove_prefix(start+1);
    else if(!text.starts_with('{'))return {};
    try{return io::parse_recording_json(text);}catch(const std::invalid_argument&){return {};}
}
io::Value result_record(const Job& job,const Result& result) {
    Array argv;for(const auto& arg:job.requested_argv?*job.requested_argv:job.argv)argv.push_back(text(arg));
    Value record{Object{{"index",integer(static_cast<std::int64_t>(job.index))},{"name",text(job.id)},
        {"exit",integer(result.exit_code)},{"wall_s",real(std::round(result.wall_s*1000.)/1000.)},
        {"out",text(job.output.string())},{"params",job.params},{"argv",Value{std::move(argv)}}}};
    if (job.key) { set(record,"job_key",text(*job.key)); set(record,"identity",job.identity); }
    if(const auto* report=std::get_if<Object>(&result.summary.data)) {
        if(const auto* outcome=io::find_member(*report,"outcome"))set(record,"outcome",*outcome);
        if(const auto* metrics=io::find_member(*report,"metrics"))set(record,"metrics",*metrics);
    }
    if(!result.err.empty()&&result.exit_code!=0) {
        auto start=result.err.size();std::size_t characters=0;
        while(start>0&&characters<400) {
            --start;
            while(start>0&&(static_cast<unsigned char>(result.err[start])&0xc0U)==0x80U)--start;
            ++characters;
        }
        set(record,result.exit_code<0?"error":"stderr_tail",text(result.err.substr(start)));
    }
    return record;
}
io::Value summarize(std::span<const Job> jobs,std::span<const Result> results,double elapsed,
    const std::filesystem::path& path,bool interrupted) {
    if(jobs.size()!=results.size())throw std::invalid_argument("batch summary identity mismatch");
    const bool spatial = std::ranges::any_of(jobs,[](const Job& job){return job.key.has_value();});
    const auto usable = [](const Result& result) {
        if (result.exit_code != 0) return false;
        if (const auto* object=std::get_if<Object>(&result.summary.data)) {
            if (const auto* status=io::find_member(*object,"model_status"))
                if (const auto* valid=io::find_member(io::as_object(*status,"model_status"),"model_valid");valid&&!io::as_bool(*valid,"model_valid")) return false;
            if (const auto* outcome=io::find_member(*object,"outcome")) {
                const auto* reason=std::get_if<std::string>(&outcome->data);
                if (const auto* values=std::get_if<Object>(&outcome->data)) {
                    if (const auto* crashed=io::find_member(*values,"crashed");crashed&&io::as_bool(*crashed,"outcome.crashed")) return false;
                    if (const auto* field=io::find_member(*values,"reason")) reason=std::get_if<std::string>(&field->data);
                }
                if (reason && (reason->starts_with("crash") || reason->starts_with("invalid") ||
                    *reason=="fallen" || *reason=="model_invalid" || *reason=="capsize" ||
                    *reason=="off_corridor" || *reason=="balance_lost" || *reason=="terrain_invalid" ||
                    *reason=="terrain_outside" || *reason=="numerical_failure")) return false;
            }
        }
        return true;
    };
    std::map<int,std::int64_t> counts;double wall=0.,sim=0.;
    for(std::size_t i=0;i<jobs.size();++i) {
        ++counts[results[i].exit_code];
        if (spatial && !usable(results[i])) continue;
        wall+=std::round(results[i].wall_s*1000.)/1000.;
        const auto* duration=io::find_member(io::as_object(jobs[i].params,"job.params"),"duration");
        if(duration&&std::holds_alternative<std::string>(duration->data)) {
            cli::Arguments parsed;parsed.set("duration",io::as_string(*duration,"job.duration"));
            sim+=parsed.real("duration");
        } else sim+=duration?io::as_double(*duration,"job.duration"):3.;
    }
    Object codes;for(const auto& [code,count]:counts)codes.emplace_back(std::to_string(code),integer(count));
    Value summary{Object{{"jobs",integer(static_cast<std::int64_t>(results.size()))},{"exit_codes",Value{std::move(codes)}},
        {"elapsed_s",real(std::round(elapsed*100.)/100.)},{"worker_wall_s",real(std::round(wall*100.)/100.)},
        {"requested_sim_seconds",real(std::round(sim*1000.)/1000.)},
        {"throughput_x",elapsed>0.?real(std::round(wall/elapsed*100.)/100.):Value{}},
        {"results",text(path.string())},{"interrupted",Value{interrupted}}}};
    if (spatial) {
        std::vector<std::size_t> order; for(std::size_t i=0;i<jobs.size();++i)order.push_back(i);
        std::ranges::sort(order,{},[&](std::size_t i){return jobs[i].key.value_or(jobs[i].id);});
        Array canonical;
        for(const auto i:order) {
            Value row{Object{{"job_key",text(jobs[i].key.value_or(jobs[i].id))},
                {"exit",integer(results[i].exit_code)},{"included_in_speed_stats",Value{usable(results[i])}}}};
            if(const auto* result=std::get_if<Object>(&results[i].summary.data))
                if(const auto* outcome=io::find_member(*result,"outcome"))set(row,"outcome",*outcome);
            canonical.push_back(std::move(row));
        }
        set(summary,"spatial_jobs",Value{std::move(canonical)});
        set(summary,"speed_stat_scope",text("only successful valid jobs; elapsed wall timing remains scheduling dependent"));
    }
    return summary;
}
} // namespace bike::batch
