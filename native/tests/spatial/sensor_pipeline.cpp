#include "support/migration_test.hpp"
#include "support/spatial_fixture.hpp"
#include "support/temporary_directory.hpp"
#include "fixture_paths.hpp"
#include "runtime/spatial_sensor_pipeline.hpp"
#include "runtime/research_io.hpp"
#include "runtime/value_readers.hpp"
#include "research/environment.hpp"
#include "research/evidence.hpp"
#include "io/json.hpp"
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>

namespace {
runtime::spatial::SensorObservationV2 raw(double time){runtime::spatial::SensorObservationV2 out;out.time_s=out.source_time_s=time;
    for(std::size_t i=0;i<out.values.size();++i)out.values[i]=static_cast<double>(i+1);return out;}
runtime::spatial::NoiseTapeV2 zeros(std::size_t n){runtime::spatial::NoiseTapeV2 tape;tape.noise.resize(n);tape.dropout_uniform.assign(n,.5);return tape;}
std::string state(const runtime::spatial::SensorPipelineV2& pipeline){return bike::io::canonical_json(bike::research::wire_value(pipeline.state_wire()));}
void request_latency_alias() {
    const bt::TemporaryDirectory temporary("spatial-sensor-delay");
    const auto source=std::filesystem::path(BIKE_SPATIAL_SOURCE_ROOT);
    const auto quoted=[](const std::filesystem::path& path){return bike::io::canonical_json(bike::io::Value{path.string()});};
    const auto request_path=temporary.path()/"request.toml";
    for(const bool has_alias:{false,true}) {
        {
            std::ofstream file(request_path,std::ios::binary|std::ios::trunc);
            file << "request_schema = 1\nphysics_file = " << quoted(source/"examples/spatial/free_fast.toml")
                 << "\ntrack_file = " << quoted(source/"tests/fixtures/migration/tracks/flat.toml")
                 << "\nrider_variant = \"lumped\"\n";
            if(has_alias)file << "sensor_delay_s = 0.0\n";
            file << "[sensors]\nlatency_s = 0.07\n[sensors_v2]\nlatency_s = 0.03\n";
            file.close();bt::require(static_cast<bool>(file),"sensor request fixture is complete");
        }
        const std::unique_ptr<bike::research::ResearchRequest> request{
            new bike::research::ResearchRequest(bike::research::read_research_request(request_path))};
        bt::require(request->setup.model.physics && request->setup.model.physics->spatial &&
            request->setup.model.physics->spatial->mode==bike::spatial::Mode::spatial3d,
            "sensor alias control selects an actual spatial research request");
        bt::require(request->sensors.delivery.latency_s==(has_alias?0.:.07) &&
            request->sensors_v2.latency_s==(has_alias?0.:.03),
            "request latency overrides both nested schemas while absence preserves each nested value");
        runtime::spatial::SensorPipelineV2 pipeline(request->sensors_v2,zeros(1));pipeline.reset(raw(0.));
        bt::require(pipeline.read(0.).valid==has_alias,"requested zero delay reaches the spatial delivery pipeline");
    }
}
}
int main(){return bt::run([]{
    using namespace runtime::spatial;
    request_latency_alias();
    SensorConfigV2 config;config.latency_s=.02;config.sample_period_s=.01;config.maximum_read_period_s=.02;config.maximum_age_s=.04;
    auto tape=zeros(20);tape.validate();bt::require(tape.noise.front().size()==11,"V2 has eleven measured channels");
    auto bad=tape;bad.dropout_uniform.clear();bt::invalid([&]{bad.validate();});
    SensorPipelineV2 pipeline(config,tape);pipeline.reset(raw(0.));
    bt::require(pipeline.state().cursor==1&&pipeline.state().samples_attempted==1,"reset consumes precisely one attempt");
    bt::require(!pipeline.read(0.).valid,"startup waits for configured latency");
    pipeline.push(raw(.01));const auto delayed=pipeline.read(.02);
    bt::require(delayed.valid&&delayed.source_time_s==0.&&delayed.values==raw(0.).values,"delivery selects the eligible source");
    pipeline.push(raw(.02));const auto captured=pipeline.state();SensorPipelineV2 resumed(config,tape);resumed.import_state(captured);
    bt::require(state(resumed)==state(pipeline),"nonempty FIFO imports exactly");
    for(int tick=3;tick<10;++tick){const double time=static_cast<double>(tick)*.01;pipeline.push(raw(time));resumed.push(raw(time));
        bt::require(pipeline.read(time).values==resumed.read(time).values&&state(pipeline)==state(resumed),"delayed continuation is byte-identical");}
    const auto cursor=pipeline.state().cursor;(void)pipeline.state();(void)pipeline.state_wire();bt::require(pipeline.state().cursor==cursor,"snapshot never consumes RNG");
    const auto stale=pipeline.read(.2);bt::require(!stale.valid&&stale.valid_mask==0,"stale sample is explicitly invalid");
    auto corrupt=resumed.state();corrupt.cursor=999;const auto unchanged=state(resumed);bt::invalid([&]{resumed.import_state(corrupt);});bt::require(state(resumed)==unchanged,"bad import retains complete prior state");
    corrupt=resumed.state();if(corrupt.queue.size()>1)corrupt.queue.back().source_time_s=corrupt.queue.front().source_time_s;
    bt::invalid([&]{resumed.import_state(corrupt);});
    SensorConfigV2 disabled;disabled.imu_enabled=false;disabled.latency_s=0.;SensorPipelineV2 no_imu(disabled,zeros(2));no_imu.reset(raw(0.));const auto masked=no_imu.read(0.);
    bt::require(masked.valid&&masked.valid_mask==0x7c0U&&masked.values[0]==0.&&masked.values[5]==0.&&masked.values[6]==7.,"disabled IMU masks six axes and retains encoders");
    SensorConfigV2 dropout;dropout.dropout_probability=1.;dropout.latency_s=0.;SensorPipelineV2 dropped(dropout,zeros(1));dropped.reset(raw(0.));
    bt::require(!dropped.read(0.).valid&&dropped.state().cursor==1&&dropped.state().samples_dropped==1,"dropout consumes one deterministic cursor");
    const auto dropped_before=state(dropped);
    auto inconsistent_drop=dropped.state();inconsistent_drop.samples_dropped=0;
    bt::invalid([&]{dropped.import_state(inconsistent_drop);});
    bt::require(state(dropped)==dropped_before,"drop counter corruption does not change the consumed tape prefix");
    inconsistent_drop=dropped.state();inconsistent_drop.queue.front().valid=true;inconsistent_drop.queue.front().valid_mask=0x7ffU;
    bt::invalid([&]{dropped.import_state(inconsistent_drop);});
    bt::require(state(dropped)==dropped_before,"checkpoint cannot turn a dropped attempt into valid measurement");
    inconsistent_drop=dropped.state();inconsistent_drop.startup->valid=true;inconsistent_drop.startup->valid_mask=0x7ffU;
    bt::invalid([&]{dropped.import_state(inconsistent_drop);});
    bt::require(state(dropped)==dropped_before,"checkpoint cannot revive a dropped startup sentinel");
    dropped.import_state(dropped.state());
    bt::require(state(dropped)==dropped_before,"authentic all-drop snapshot remains restorable");
    bool exhausted=false;try{dropped.push(raw(.01));}catch(const std::runtime_error&){exhausted=true;}bt::require(exhausted,"tape exhaustion invalidates run");
    SensorConfigV2 mixed;mixed.latency_s=0.;mixed.dropout_probability=.5;
    auto attempts=zeros(2);attempts.dropout_uniform={.75,.25};SensorPipelineV2 intermittent(mixed,attempts);intermittent.reset(raw(0.));
    bt::require(intermittent.read(0.).valid,"non-dropped control is delivered");intermittent.push(raw(.01));
    const auto missing=intermittent.read(.01);
    bt::require(!missing.valid&&missing.valid_mask==0&&missing.source_time_s==.01&&intermittent.state().cursor==2,
        "dropout consumes its own attempt and never reuses an earlier valid delivery");
    const auto suffix=intermittent.state();intermittent.import_state(suffix);
    bt::require(intermittent.state().queue.size()==1&&!intermittent.read(.01).valid,
        "retained suffix is checked at its original tape index after an earlier delivery is discarded");
    SensorPipelineV2 unavailable(mixed,zeros(1));auto unavailable_raw=raw(0.);unavailable_raw.valid=false;unavailable_raw.valid_mask=0;
    unavailable.reset(unavailable_raw);unavailable.import_state(unavailable.state());
    bt::require(!unavailable.read(0.).valid&&unavailable.state().samples_dropped==0,
        "nondropped input may remain invalid for an independent acquisition reason");
    SensorConfigV2 bounded;bounded.latency_s=0.;bounded.sample_period_s=.01;bounded.maximum_read_period_s=.02;
    SensorPipelineV2 unread(bounded,zeros(8));unread.reset(raw(0.));
    for(int i=1;i<5;++i)unread.push(raw(static_cast<double>(i)*.01));
    const auto before_overflow=state(unread);bool overflow=false;
    try{unread.push(raw(.05));}catch(const std::runtime_error&){overflow=true;}
    bt::require(overflow&&state(unread)==before_overflow,"ring exhaustion rejects before consuming another attempt");
    bt::invalid([&]{unread.push(raw(.04));});
    auto invalid_reset=raw(0.);invalid_reset.values[0]=std::numeric_limits<double>::quiet_NaN();
    bt::invalid([&]{unread.reset(invalid_reset);});bt::require(state(unread)==before_overflow,"invalid reset preserves prior queue and cursor");
    auto disabled_state=no_imu.state();disabled_state.queue.front().values[0]=1.;
    bt::invalid([&]{no_imu.import_state(disabled_state);});
    auto wire=raw(0.).as_wire();wire.emplace_back("unexpected",0);bt::invalid([&]{(void)SensorObservationV2::from_wire(wire);});
    auto invalid=raw(0.);invalid.valid_mask=0x800U;bt::invalid([&]{invalid.validate();});invalid=raw(0.);invalid.source_time_s=1.;bt::invalid([&]{invalid.validate();});
    auto wrong_width=raw(0.).as_wire();runtime::sample_wire::set(wrong_width,"angular_rate_sensor_rad_s",runtime::WireArray{0.,0.,0.,0.});
    bt::invalid([&]{(void)SensorObservationV2::from_wire(wrong_width);});
    invalid=raw(0.);invalid.valid=false;bt::invalid([&]{invalid.validate();});
    const auto before=bike::research::make_sensor_tape(bike::research::SensorConfig{},"0",4);
    const auto v2=bike::research::make_sensor_tape_v2(config,"0",4);bt::require(v2.noise.size()==4&&v2.noise.front().size()==11,"V2 seeded tape width");
    const auto after=bike::research::make_sensor_tape(bike::research::SensorConfig{},"0",4);
    bt::require(before.noise==after.noise&&before.dropout_uniform==after.dropout_uniform,"V2 domain-separated draws preserve V1 streams");
    const auto fixture=spatial_tests::json(spatial_tests::fixture("sensor/v1_seed0_expected.json"));
    const auto& expected=bike::io::as_object(fixture,"fixture");const auto* original=bike::io::find_member(expected,"expected");bt::require(original!=nullptr,"existing seed0 fixture available");
    const auto legacy=runtime::SensorObservation::from_wire(runtime::sample_wire::object(runtime::vread::wire_from_value(*original,"legacy")));
    bt::require(bike::io::canonical_json(bike::research::wire_value(legacy.as_wire()))==bike::io::canonical_json(*original),"V1 schema bytes preserve existing historical seed0 values");
});}
