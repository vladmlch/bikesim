#include "support/migration_test.hpp"
#include "research/policy.hpp"
#include "research/policy_session.hpp"
#include "research/evidence.hpp"
#include "runtime/research_io.hpp"
#include "support/startup_cases.hpp"
#include "io/json.hpp"
#include "platform/argv.hpp"
#include <array>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>

namespace {
template<class F>void rejected(F&& call){bool failed=false;try{call();}catch(const bike::research::PolicyFailure&){failed=true;}bt::require(failed,"ABI2 boundary captures invalid descriptor/demand/exception");}
void* create(const char*,BikeErrorV1*){return nullptr;}int reset(void*,const BikeObservationV2*,BikeErrorV1*){return 0;}
int act(void*,const BikeObservationV2*,BikeDemandV1*,BikeErrorV1*){return 0;}void destroy(void*){}
std::string wire_bytes(const runtime::Wire& value){return bike::io::canonical_json(bike::research::wire_value(value));}
void pacing(const std::string& plugin_path) {
    auto request=std::make_unique<bike::research::ResearchRequest>();
    request->setup=std::move(*spatial_tests::make_startup_request(
        bike::spatial::RootKind::free6,bike::spatial::TyreTier::fast,false,2.));
    request->capture_mode=runtime::ResearchCaptureMode::full;
    request->experiment.duration_s=.05;request->experiment.control_period_s=.01;
    request->experiment.actuator_delay_s=.02;request->experiment.seed_decimal="0";
    request->sensors_v2.sample_period_s=.005;request->sensors_v2.latency_s=.01;
    request->sensors_v2.maximum_read_period_s=.01;
    auto prepared=bike::research::prepare_environment(std::move(request));
    auto copy=std::make_unique<runtime::ResearchBootstrap>(prepared->bootstrap);
    auto first=runtime::make_research(std::move(prepared->bootstrap));
    auto second=runtime::make_research(std::move(copy));
    bike::research::PolicySession direct(std::move(first),bike::research::load_policy("plugin:"+plugin_path,bike::research::PolicyAbi::v2));
    bike::research::PolicySession paced(std::move(second),bike::research::load_policy("plugin:"+plugin_path,bike::research::PolicyAbi::v2));
    while(!direct.done()) {
        bt::require(!paced.done(),"paced session shares the same outcome boundary");
        const auto unpaced=direct.advance();paced.begin_advance();
        std::optional<runtime::ResearchTransition> sliced;
        while(!sliced) {
            const auto before=spatial_tests::physics_state_bytes(paced.environment().snapshot());
            (void)paced.environment().render_frame();(void)paced.environment().render_frame();
            bt::require(before==spatial_tests::physics_state_bytes(paced.environment().snapshot()),
                "repeated spatial presentation reads do not integrate or consume causal state");
            sliced=paced.advance_pending(60.,paced.environment().snapshot().step+1);
        }
        bt::require(wire_bytes(unpaced.as_wire())==wire_bytes(sliced->as_wire()),
            "typed ABI2 control and observation do not depend on external pacing yields");
    }
    bt::require(paced.done(),"unpaced and paced sessions terminate at the same accepted interval");
    const auto& a=direct.environment();const auto& b=paced.environment();
    bt::require(wire_bytes(a.commands_requested())==wire_bytes(b.commands_requested())&&
        wire_bytes(a.commands_applied())==wire_bytes(b.commands_applied())&&
        wire_bytes(a.observations())==wire_bytes(b.observations())&&wire_bytes(a.transitions())==wire_bytes(b.transitions()),
        "the complete command/applied/observation histories remain byte-identical");
    bt::require(spatial_tests::physics_state_bytes(a.snapshot())==spatial_tests::physics_state_bytes(b.snapshot()),
        "physical and material endpoint states match after paced policy dispatch");
    const auto original_generation=b.snapshot().generation;
    paced.reset("1");
    // Direct construction elides the large sealed value into cold storage.
    const std::unique_ptr<runtime::OwnedResearchEpisode> reset{new runtime::OwnedResearchEpisode(paced.environment().evidence_snapshot())};
    bt::require(reset->final_snapshot.generation>original_generation&&reset->final_snapshot.step==0&&
        reset->configuration.seed_decimal=="1"&&!bike::io::is_null(reset->checkpoint_initial),
        "reseed stages a fresh initial checkpoint and reset generation");
    using namespace runtime::setupvalue;
    bt::require(bike::io::as_string(field(field(field(reset->checkpoint_initial,"research"),"clock"),"seed_decimal"),"seed")=="1",
        "initial checkpoint records the new sensor seed rather than the previous episode");
}
}
int main(int argc,char** argv){return bt::run([&]{
    // The shared entry-point wrapper keeps raw argv subscripting out of the test body.
    const auto arguments=bike::platform::main_arguments(argc,argv);
    bt::require(arguments.size()==2,"policy_v2 test requires its plugin path");
    const std::string plugin_path(arguments[1]);
    using namespace bike::research;
    static_assert(std::is_standard_layout_v<BikeObservationV2> && std::is_trivially_copyable_v<BikeObservationV2>);
    static_assert(std::is_same_v<decltype(BikeObservationV2::values),double[11]>);
    BikeObservationV2 observation{};observation.struct_size=sizeof(observation);observation.sensor_schema_version=2;
    bt::require(std::size(observation.values)==11&&BIKE_SENSOR_VALUE_COUNT==9U,"ABI2 retains eleven channels without changing ABI1");
    const BikePolicyV2 good{.struct_size=sizeof(BikePolicyV2),.abi_version=2,.capabilities=0,.create=create,.reset=reset,.act=act,.destroy=destroy};
    validate_policy_abi_v2(&good);auto bad=good;bad.struct_size=0;rejected([&]{validate_policy_abi_v2(&bad);});
    bad=good;bad.abi_version=1;rejected([&]{validate_policy_abi_v2(&bad);});bad=good;bad.act=nullptr;rejected([&]{validate_policy_abi_v2(&bad);});
    runtime::spatial::SensorObservationV2 measured;measured.values[6]=2.;measured.values[7]=3.;measured.values[9]=5.;
    const auto abi=policy_observation_to_abi_v2(measured);bt::require(abi.valid_mask==0x7ffU&&abi.values[7]==3.,"host preserves V2 values/mask");
    auto builtin=load_policy("builtin:passthrough",PolicyAbi::v2);builtin->reset("0");bt::require(builtin->act_v2(measured,18.).motor_torque_nm==18.,"builtin receives V2 without fabricated IMU channels");
    auto plugin=load_policy("plugin:"+plugin_path,PolicyAbi::v2);
    plugin->reset("0");bt::require(plugin->act_v2(measured,0.).motor_torque_nm==25.,"real ABI2 plugin dispatch");
    auto nan=load_policy("plugin:"+plugin_path,bike::io::parse_json("{\"mode\":\"nan\"}"),PolicyAbi::v2);
    nan->reset("0");rejected([&]{(void)nan->act_v2(measured,0.);});nan->close();
    auto throwing=load_policy("plugin:"+plugin_path,bike::io::parse_json("{\"mode\":\"throw\"}"),PolicyAbi::v2);
    throwing->reset("0");rejected([&]{(void)throwing->act_v2(measured,0.);});throwing->close();
    measured.values[0]=std::numeric_limits<double>::quiet_NaN();bt::invalid([&]{(void)plugin->act_v2(measured,0.);});
    plugin->close();
    pacing(plugin_path);
});}
