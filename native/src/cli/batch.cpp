#include "application.hpp"
#include "../batch/plan.hpp"
#include "../batch/log.hpp"
#include "../configuration/load.hpp"
#include "../io/json.hpp"
#include "../io/npy.hpp"
#include "../platform/files.hpp"
#include "../platform/signals.hpp"
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <limits>
#include <ranges>
#include <utility>

namespace bike::cli {
int batch_command(const Arguments& args) {
    using Clock=std::chrono::steady_clock;
    const auto bytes=io::read_numpy_bytes(args.text("spec"));
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast) byte-to-char aliasing is the designed idiom for raw file bytes
    const std::string source(reinterpret_cast<const char*>(bytes.data()),bytes.size());
    auto jobs=batch::expand_spec(batch::parse_spec_toml(source,args.text("spec")));
    // Validate every job with the exact research parser before any process or
    // output is created. Typed setup is resolved later in each isolated child.
    for(auto& job:jobs) {
        std::vector<std::string_view> views;views.reserve(job.argv.size());
for(const auto& arg:job.argv)views.push_back(arg);
        try {const auto parsed=parse_arguments("bike-research",views);if(!parsed.help){validate_research_arguments(parsed);batch::resolve_job_identity(job,parsed);}}
        catch(const std::exception& error){throw ArgumentError("job "+job.id+": "+error.what());}
    }
    if(args.flag("dry_run")) {
        for(const auto& job:jobs) {std::cout<<job.id<<": bike-research";for(const auto& arg:job.argv)std::cout<<' '<<arg;std::cout<<'\n';}
        std::cout<<jobs.size()<<" job(s)\n";return 0;
    }
    const auto count=args.flag("serial")?1:args.integer("jobs",batch::performance_cores());
    if(count<1||std::cmp_greater(count,std::numeric_limits<unsigned>::max()))throw ArgumentError("--jobs must be positive");
    const auto root=std::filesystem::absolute(args.text("out"));
    if(std::filesystem::exists(root)&&!std::filesystem::is_empty(root)&&!args.flag("overwrite"))throw ArgumentError("output exists; choose another --out or pass --overwrite: "+root.string());
    std::filesystem::create_directories(root);
    const auto results_path=root/"batch_results.jsonl";
    const bool spatial = std::ranges::any_of(jobs,[](const auto& job){return job.key.has_value();});
    // --overwrite explicitly permits replacement of the batch log, but never
    // adoption or truncation of an already published run directory.
    auto results_file=batch::open_log(results_path,args.flag("overwrite"));
    const auto binary=executable_path().parent_path()/"bike-research";
    for(auto& job:jobs) {
        job.requested_argv=job.argv;
        job.output=root/job.id;
        if(std::filesystem::exists(job.output)&&!std::filesystem::is_empty(job.output)&&!args.flag("overwrite"))throw ArgumentError("job output exists: "+job.output.string());
        std::filesystem::create_directories(job.output);
        job.argv.insert(job.argv.begin(),binary.string());job.argv.emplace_back("--out");job.argv.push_back(job.output.string());
    }
    platform::InterruptShield interrupt;
    const auto started=Clock::now();std::size_t done=0;
    batch::ExecuteOptions options;
    options.cancelled=[&]{return interrupt.deferred()!=0;};
    options.completed=[&](std::size_t index,const batch::Result& result) {
        const auto record=batch::result_record(jobs[index],result);
        if (!spatial) batch::append_log(results_file,io::recording_json(record)+"\n");
        // Raw worker streams are kept for diagnostics without changing cwd.
        for(const auto& [name,text]:{std::pair{"stdout.log",&result.out},std::pair{"stderr.log",&result.err}}) {
            batch::write_log(jobs[index].output/name,*text,args.flag("overwrite"));
        }
        std::cout<<'['<<++done<<'/'<<jobs.size()<<"] "<<result.id<<" exit="<<result.exit_code<<" wall="<<result.wall_s<<"s\n";
    };
    const auto results=batch::execute(jobs,static_cast<unsigned>(count),options);
    if (spatial) {
        std::vector<std::size_t> order; order.reserve(jobs.size()); for(std::size_t i=0;i<jobs.size();++i)order.push_back(i);
        std::ranges::sort(order,{},[&](std::size_t i){return jobs[i].key.value_or(jobs[i].id);});
        for(const auto i:order)batch::append_log(results_file,io::recording_json(batch::result_record(jobs[i],results[i]))+"\n");
    }
    const auto elapsed=std::chrono::duration<double>(Clock::now()-started).count();
    results_file.close();
    std::cout<<io::recording_json(batch::summarize(jobs,results,elapsed,results_path,interrupt.deferred()!=0))<<'\n';
    return interrupt.deferred()?130:std::ranges::all_of(results,[](const auto& r){return r.exit_code==0;})?0:1;
}
} // namespace bike::cli
