#include "plan.hpp"
#include "../platform/process.hpp"
#include "../runtime/setup_values.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <thread>
#if defined(__APPLE__)
#include <sys/sysctl.h>
#elif defined(__linux__)
#include <sched.h>
#endif

namespace bike::batch {
unsigned performance_cores() {
#if defined(__APPLE__)
    int count=0;std::size_t size=sizeof(count);
    if(::sysctlbyname("hw.perflevel0.physicalcpu",&count,&size,nullptr,0)==0&&count>0)return static_cast<unsigned>(count);
#elif defined(__linux__)
    cpu_set_t available;CPU_ZERO(&available);
    if(::sched_getaffinity(0,sizeof(available),&available)==0) {
        const int count=CPU_COUNT(&available);if(count>0)return static_cast<unsigned>(count);
    }
#endif
    return std::max(1U,std::thread::hardware_concurrency());
}
std::vector<Result> execute(std::span<const Job> jobs,unsigned workers,const ExecuteOptions& options) {
    using Clock=std::chrono::steady_clock;
    if(workers==0)throw std::invalid_argument("worker count must be positive");
    std::vector<Result> results(jobs.size());
    if(jobs.empty())return results;
    for(std::size_t i=0;i<jobs.size();++i)results[i].id=jobs[i].id;
    std::atomic<std::size_t> next{0};
    std::stop_source stop;
    std::stop_callback const relay(options.cancellation,[&]{stop.request_stop();});
    std::mutex mutex;
    std::condition_variable wake;
    // Allocate delivery storage before launching a child. Result delivery
    // itself never allocates on a worker and cannot terminate its thread.
    std::vector<std::size_t> completed(jobs.size());
    std::size_t published=0;
    std::exception_ptr fatal;
    std::vector<std::jthread> pool;
    const auto work=[&](const std::stop_token& thread_stop) {
        std::stop_callback const relay_thread(thread_stop,[&]{stop.request_stop();});
        try {
        for(;;) {
            const auto index=next.fetch_add(1,std::memory_order_relaxed);if(index>=jobs.size())return;
            const auto& job=jobs[index];auto& result=results[index];
            const auto started=Clock::now();
            try {
                if(stop.stop_requested()) {
                    result.exit_code=130;result.err="cancelled before worker launch";
                } else {
                    platform::ProcessOptions child_options;child_options.cancellation=stop.get_token();
                    auto child=platform::run(job.argv,options.cwd,child_options);
                    result.exit_code=child.exit_code;result.out=std::move(child.out);result.err=std::move(child.err);
                    result.summary=tail_json(result.out);
                }
            } catch(const std::exception& error) {
                result.exit_code=-1;result.err=error.what();
            } catch(...) { result.exit_code=-1;result.err="non-standard worker failure"; }
            result.wall_s=std::chrono::duration<double>(Clock::now()-started).count();
            // No reader observes the result before the queue's release/acquire.
            {std::scoped_lock<std::mutex> const lock(mutex);completed[published++]=index;}
            wake.notify_one();
        }
        } catch(...) {
            {std::scoped_lock<std::mutex> const lock(mutex);if(!fatal)fatal=std::current_exception();}
            stop.request_stop();wake.notify_one();
        }
    };
    try {
        for(std::size_t i=0;i<std::min<std::size_t>(workers,jobs.size());++i)pool.emplace_back(work);
        std::size_t received=0;
        while(received<jobs.size()) {
            if(options.cancelled&&options.cancelled())stop.request_stop();
            std::unique_lock<std::mutex> lock(mutex);
            wake.wait_for(lock,std::chrono::milliseconds(20),[&]{return published>received||fatal;});
            if(fatal)std::rethrow_exception(fatal);
            const auto available=published;lock.unlock();
            while(received<available) {
                const auto index=completed[received++];
                if(options.completed)options.completed(index,results[index]);
            }
        }
    } catch(...) {
        stop.request_stop();
        for(auto& thread:pool)if(thread.joinable())thread.join();
        throw;
    }
    for(auto& thread:pool)if(thread.joinable())thread.join();
    return results;
}
} // namespace bike::batch