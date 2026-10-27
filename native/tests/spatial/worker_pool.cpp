#include "support/migration_test.hpp"
#include "runtime/spatial_worker_pool.hpp"
#include "allocation_faults.hpp"
#include "platform/argv.hpp"

#include <array>
#include <atomic>
#include <barrier>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <mutex>
#include <new>
#include <numeric>
#include <span>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <vector>

namespace {
using runtime::spatial::WorkerPool;
using runtime::spatial::partition_ranges;
struct Cells { std::span<std::uint64_t> values; };
void increment(void* raw,std::size_t begin,std::size_t end) noexcept {
    auto& c=*static_cast<Cells*>(raw);
    for(std::size_t i=begin;i<end;++i)c.values[i]+=i+1U;
}
template<class F>void logic_error(F&& f,const char* why) {
    bool rejected=false;
    try { f(); } catch(const std::logic_error&) { rejected=true; }
    bt::require(rejected,why);
}
struct Latch {
    std::mutex mutex;
    std::condition_variable cv;
    bool entered=false,released=false;
};
void blocked(void* raw,std::size_t begin,std::size_t end) noexcept {
    (void)begin;(void)end;
    auto& latch=*static_cast<Latch*>(raw);
    std::unique_lock lock(latch.mutex);
    latch.entered=true;latch.cv.notify_all();
    latch.cv.wait(lock,[&]{return latch.released;});
}
void no_work(void* raw,std::size_t begin,std::size_t end) noexcept {(void)raw;(void)begin;(void)end;}
struct AllocationLane {std::size_t allocations=0,frees=0;};
// A span, not a raw pointer: lane slots are indexed by range begin, and a bare
// pointer subscript is rejected by -Wunsafe-buffer-usage under -Werror.
struct AllocationLanes {std::span<AllocationLane> slots;};
void reset_allocations(void*,std::size_t,std::size_t) noexcept {allocation_counter::reset();}
void capture_allocations(void* raw,std::size_t begin,std::size_t) noexcept {
    auto& lanes=*static_cast<AllocationLanes*>(raw);
    lanes.slots[begin]={allocation_counter::count(),allocation_counter::freed()};
}
void allocate_control(void*,std::size_t,std::size_t) noexcept {
    void* pointer=::operator new(1,std::nothrow);
    ::operator delete(pointer);
}
void warm_allocations(unsigned threads) {
    WorkerPool pool(threads);
    const auto ranges=partition_ranges(threads,threads);
    std::array<AllocationLane,8> lanes{};
    AllocationLanes capture{lanes};
    pool.run(ranges,no_work,nullptr);
    pool.run(ranges,reset_allocations,nullptr);
    for(unsigned generation=0;generation<64;++generation)pool.run(ranges,no_work,nullptr);
    pool.run(ranges,capture_allocations,&capture);
    for(unsigned i=0;i<threads;++i)
        bt::require(lanes[i].allocations==0&&lanes[i].frees==0,
            "warmed pool dispatch/completion allocates or frees on neither caller nor worker threads");
    pool.run(ranges,reset_allocations,nullptr);
    pool.run(ranges,allocate_control,nullptr);
    pool.run(ranges,capture_allocations,&capture);
    for(unsigned i=0;i<threads;++i)
        bt::require(lanes[i].allocations==1&&lanes[i].frees==1,
            "per-thread allocation instrumentation detects the explicit negative control on every pool lane");
}
void overlap() {
    WorkerPool pool(2);const auto ranges=partition_ranges(2,2);Latch latch;
    // An incorrect stop() could join the deliberately blocked callbacks.
    // Bound that negative control as well: the correct rejection never waits
    // for this deadline and the releaser is notified during normal teardown.
    std::jthread releaser([&] {
        std::unique_lock lock(latch.mutex);
        if(!latch.cv.wait_for(lock,std::chrono::seconds(5),[&]{return latch.released;})) {
            latch.released=true;latch.cv.notify_all();
        }
    });
    std::exception_ptr failure;
    std::thread owner([&]{try {pool.run(ranges,blocked,&latch);} catch(...) {failure=std::current_exception();}});
    {
        std::unique_lock lock(latch.mutex);
        latch.cv.wait(lock,[&]{return latch.entered;});
    }
    // Always release/join before an assertion can throw, including the bad
    // implementation control where either concurrent operation is accepted.
    bool run_rejected=false,stop_rejected=false;
    try {pool.run(ranges,no_work,nullptr);} catch(const std::logic_error&) {run_rejected=true;}
    try {pool.stop();} catch(const std::logic_error&) {stop_rejected=true;}
    {std::lock_guard lock(latch.mutex);latch.released=true;}
    latch.cv.notify_all();owner.join();
    if(failure)std::rethrow_exception(failure);
    bt::require(run_rejected && stop_rejected,"overlapping run and stop reject before generation publication");
    std::array<std::uint64_t,2> data{};Cells cells{data};pool.run(ranges,increment,&cells);
    bt::require(data[0]==1 && data[1]==2,"rejected overlap leaves next generation usable");
}
struct LogicalRace {
    std::barrier<> barrier{4};
    std::atomic<std::uint64_t> shared{0};
};
void lost_update(void* raw,std::size_t begin,std::size_t end) noexcept {
    (void)begin;(void)end;auto& c=*static_cast<LogicalRace*>(raw);
    const auto before=c.shared.load(std::memory_order_relaxed);
    c.barrier.arrive_and_wait();c.shared.store(before+1U,std::memory_order_relaxed);
}
struct UnsafeRace {
    std::barrier<> barrier{4};
    std::uint64_t shared=0;
};
void unsafe_update(void* raw,std::size_t begin,std::size_t end) noexcept {
    (void)begin;(void)end;auto& c=*static_cast<UnsafeRace*>(raw);
    const auto before=c.shared;
    c.barrier.arrive_and_wait();c.shared=before+1U;
}
void race_control() {
    // Deliberately racy, test-only negative control. Never run this branch in
    // the positive CTest. V09: TSAN_OPTIONS=halt_on_error=1:exitcode=66
    // bike_spatial_worker_pool --tsan-race-control must report a data race.
    WorkerPool pool(4);UnsafeRace control;
    pool.run(partition_ranges(4,4),unsafe_update,&control);
    std::cerr<<"negative race-control final cell="<<control.shared<<'\n';
    bt::require(control.shared==4,"intentional shared-cell race must not qualify");
}
void cases() {
    WorkerPool serial(1);
    bt::require(serial.worker_count()==0,"N=1 creates no background workers");
    bt::invalid([]{WorkerPool invalid(0);});
    serial.stop();serial.stop();
    const auto empty_serial=partition_ranges(0,1);
    logic_error([&]{serial.run(empty_serial,increment,nullptr);},"stopped pool rejects run");
    for(const unsigned threads:{1U,2U,4U,8U}) {
        warm_allocations(threads);
        WorkerPool pool(threads);std::array<std::uint64_t,17> values{};Cells cells{values};
        const auto ranges=partition_ranges(values.size(),threads);
        for(unsigned generation=0;generation<1000;++generation)pool.run(ranges,increment,&cells);
        for(std::size_t i=0;i<values.size();++i)
            bt::require(values[i]==1000U*(i+1U),"1000 generations neither lose nor duplicate element work");
        bt::require(std::accumulate(values.begin(),values.end(),std::uint64_t{0})==153000U,
            "owner's canonical integer sum equals serial");
        std::vector<std::uint64_t> empty;Cells none{empty};
        pool.run(partition_ranges(0,threads),increment,&none);
        bt::invalid([&]{pool.run(ranges,nullptr,&cells);});
        bt::invalid([&]{pool.run({},increment,&cells);});
        auto invalid=ranges;invalid[0].begin=invalid[0].end+1;
        bt::invalid([&]{pool.run(invalid,increment,&cells);});
        pool.stop();pool.stop();
    }
    overlap();
    WorkerPool pool(4);LogicalRace shared;
    pool.run(partition_ranges(4,4),lost_update,&shared);
    bt::require(shared.shared.load()==1,"wrong shared-slot ownership is detected without invoking UB in the positive suite");
}
}
int main(int argc,char** argv) {
    // The shared wrapper keeps raw argv subscripting out of the test under
    // -Wunsafe-buffer-usage. "--tsan-race-control" is the argument contract
    // named by BIKE_CAP_TSAN_VIOLATION_CONTROL in BikeCompilerPolicy.cmake.
    const auto arguments=bike::platform::main_arguments(argc,argv);
    if(arguments.size()==2 && std::string_view(arguments[1])=="--tsan-race-control")return bt::run(race_control);
    if(arguments.size()!=1)return 2;
    return bt::run(cases);
}
