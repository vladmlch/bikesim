// Native transport/publication contracts for F145, included by T42.
#pragma once
#include "runtime/async_close.hpp"
#include "research/evidence.hpp"
#include "io/json.hpp"
#include <bit>

namespace async_close_contract {
inline runtime::AccountingConfig config(bool asynchronous) {
    runtime::AccountingConfig result;
    result.timestep_s=.001; result.period_steps=2; result.measurement_decimation=2;
    result.retention=runtime::RetentionMode::full; result.max_qpos=1; result.max_dofs=1;
    result.async_telemetry=asynchronous;
    return result;
}
inline runtime::AccountingState state(bool strict=false) {
    runtime::AccountingState result;
    result.energy.initial_energy_j=100.;result.energy.energy_scale_j=100.;
    result.energy.initial_battery_j=0.;result.energy.mechanical_energy_j=100.;
    result.monitor.strict=strict;
    return result;
}
inline runtime::RawStep raw(std::int64_t id) {
    runtime::RawStep result;
    result.interval_id=id;result.time_s=static_cast<double>(id)*.001;
    result.end_time_s=static_cast<double>(id+1)*.001;
    result.qpos={-0.};result.qvel={1.};result.qfrc_passive={0.};
    result.components={{"chain",{2.}}};result.electrical_power_w=0.;
    result.full=id%2==0;
    if(result.full){result.mechanical_energy_j=100.;result.elastic_energy_j=runtime::WireObject{};}
    result.constraint_snapshot={result.time_s,result.end_time_s,result.qvel,{}};
    const runtime::WireObject tire{{"unloaded_radius_m",.35},{"normal_load_n",0.},
        {"penetration_m",0.},{"patches",runtime::WireArray{}}};
    result.channels={{"tires",runtime::WireObject{{"front",tire},{"rear",tire}}},
        {"suspension",runtime::WireObject{{"linkage_closure_max_m",0.}}}};
    result.diagnostics={{"rider",runtime::WireObject{}},{"rider_welds",runtime::WireObject{}},
        {"endpoint_mass",runtime::Wire{}},{"rider_intent",runtime::Wire{}},{"inclination_rad",0.},
        {"rider_allocation",runtime::WireObject{}},{"rider_ik_saturation",runtime::WireObject{}},
        {"rider_support_targets",runtime::WireObject{}}};
    return result;
}
inline void compare(const runtime::PeriodAccounting& a,const runtime::PeriodAccounting& b) {
    const auto lhs=a.recorded_samples(),rhs=b.recorded_samples();
    bt::require(lhs.size()==rhs.size(),"async drain publishes the complete physical extent");
    for(std::size_t i=0;i<lhs.size();++i){
        bt::require(bike::io::recording_json(bike::research::wire_value(runtime::Wire{lhs[i]->as_wire()}))==
            bike::io::recording_json(bike::research::wire_value(runtime::Wire{rhs[i]->as_wire()})),
            "sync and worker apply the same period evaluator and preserve field values/order");
        bt::require(std::bit_cast<std::uint64_t>(lhs[i]->qpos[0])==std::bit_cast<std::uint64_t>(rhs[i]->qpos[0]),
            "worker transport preserves signed zero bits");
    }
}
inline void run() {
    runtime::PeriodAccounting sync(config(false),state(),1,nullptr,nullptr);
    runtime::PeriodAccounting asynchronous(config(true),state(),1,nullptr,nullptr);
    for(std::int64_t id=0;id<5;++id){
        asynchronous.collect(false);sync.push(raw(id));asynchronous.push(raw(id));
        if(sync.full()){sync.close_period();asynchronous.close_period();}
    }
    sync.flush();asynchronous.flush();compare(sync,asynchronous);
    const auto sealed=asynchronous.seal_episode();
    asynchronous.acknowledge_samples(asynchronous.prepare_samples());
    bt::require(sealed.samples.size()==5&&asynchronous.pending_samples().empty(),
        "sealing survives pending acknowledgement and worker close");
    runtime::PeriodAccounting detached(config(true),state(),1,nullptr,nullptr);
    detached.push(raw(0));detached.push(raw(1));detached.close_period();detached.detach_async();
    bt::require(!detached.asynchronous()&&detached.recorded_samples().size()==2,
        "detach replays every unacknowledged original exactly once");
    runtime::PeriodAccounting strict(config(true),state(true),7,nullptr,nullptr);
    auto invalid=raw(0);invalid.attachment_errors={"foot:unobservable"};
    strict.push(std::move(invalid));strict.push(raw(1));strict.close_period();
    bt::require(strict.pending_samples().empty(),"worker completion is invisible before main-thread collection");
    bool rejected=false;try{strict.flush();}catch(const runtime::InvalidReferenceRun&){rejected=true;}
    bt::require(rejected&&strict.recorded_samples().size()==2&&strict.state().monitor.first_failure->time_s==.001,
        "strict worker failure publishes the complete period with original first-failure time");
    strict.flush();bt::require(strict.seal_episode().generation==7,"strict delivery is acknowledged before retry");
    auto originals=std::make_shared<const std::vector<runtime::RawStep>>(std::vector<runtime::RawStep>{raw(0),raw(1)});
    runtime::AsyncCloseRunner failed(config(false),state(),1,{},[](std::uint64_t){throw std::runtime_error("injected");});
    failed.submit(originals);bool failure=false;
    try{(void)failed.peek(true);}catch(const runtime::AsyncPublicationError& error){failure=std::string(error.what()).find("injected")!=std::string::npos;}
    bt::require(failure&&failed.pending()==1,"worker errors propagate without losing unacknowledged raws");
    const auto replay=failed.detach_pending();
    bt::require(replay.size()==1&&replay[0]==originals,"detached evidence retains original ownership");
    failed.close();failed.close();
    runtime::AsyncCloseRunner stopped(config(false),state(),1,{});stopped.submit(originals);stopped.close();
    const auto stopped_raws=stopped.detach_pending();
    bt::require(stopped_raws.size()==1,"close joins before freeing the worker and keeps pending evidence detachable");
}
} // namespace async_close_contract
