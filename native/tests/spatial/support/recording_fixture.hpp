#pragma once
#include "startup_cases.hpp"
#include "recording/capture.hpp"
#include "recording/publish.hpp"
#include "research/environment.hpp"
#include "research/evidence.hpp"
#include "runtime/research_io.hpp"
#include "model/export.hpp"
#include "engine_call.hpp"
namespace spatial_tests {
struct RecordedResearch {
    std::unique_ptr<runtime::NativeResearchRuntime> environment;
    std::unique_ptr<bike::recording::RunCapture> capture;
};
inline RecordedResearch recording_fixture() {
    auto request=std::make_unique<bike::research::ResearchRequest>();
    request->setup=std::move(*make_startup_request(bike::spatial::RootKind::free6,bike::spatial::TyreTier::fast,false,2.));
    request->capture_mode=runtime::ResearchCaptureMode::full;
    request->experiment.duration_s=.1;request->experiment.control_period_s=.01;
    request->experiment.actuator_delay_s=.02;request->experiment.seed_decimal="0";
    request->sensors_v2.latency_s=.02;request->sensors_v2.sample_period_s=.005;
    auto prepared=bike::research::prepare_environment(std::move(request));
    auto metadata=prepared->metadata;
    bike::model::ModelOwner model{engine::load_model_image(prepared->bootstrap.ride.model.mjb)};
    if(!model)throw std::runtime_error("spatial recording fixture model failed");
    auto catalog=bike::recording::build_catalog(*model,bike::recording::CatalogVersion::spatial2);
    auto environment=runtime::make_research(std::move(prepared->bootstrap));
    runtime::RideControl control;control.human_torque_nm=5.;
    for(int i=0;i<10;++i) {
        environment->begin_control(control,0.,0.);
        auto result=environment->advance_control(std::nullopt);
        if(!result)throw std::runtime_error("unbounded spatial fixture control did not complete");
        environment->acknowledge_control();
        if(result->terminated||result->truncated)break;
    }
    auto episode=environment->seal_episode();
    runtime::setupvalue::set(episode.run_metadata,"identity_checks",bike::research::identity_receipt(episode.configuration.metadata));
    auto summary=bike::research::summarize(episode);
    auto capture=std::make_unique<bike::recording::RunCapture>(bike::recording::make_run_metadata("native","research",metadata),true);
    capture->add_episode(bike::recording::make_research_evidence(0,std::move(episode),std::move(catalog),std::move(summary)));
    return {.environment=std::move(environment),.capture=std::move(capture)};
}
}
