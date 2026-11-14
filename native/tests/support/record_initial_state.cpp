#include "record_initial_state.hpp"
#include "fixture_test.hpp"

#include "diag.hpp"

#include <algorithm>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>

#include "configuration/load.hpp"
#include "initialization/setup_owner.hpp"
#include "io/json.hpp"
#include "io/value.hpp"
#include "platform/provenance.hpp"
#include "research/environment.hpp"

namespace {
namespace io = bike::io;
using bike::initialization::SetupRequest;

// The strict setup-request schema read_setup_request enforces
// (initialization/request.cpp). A request whose top-level keys fit this set
// takes the setup loader; any other top-level member names a research
// request and the document goes through research::read_research_request.
constexpr std::string_view setup_keys[] = {
    "request_schema", "physics_file", "track_file", "model_mode",
    "rider_variant", "speed_kph", "seed_decimal", "use_equilibrium_cache",
    "initial_state_file", "overrides"};

bool setup_request(const io::Value& document) {
    const auto& members = io::as_object(document, "request");
    return std::ranges::all_of(members, [](const auto& member) {
        return std::ranges::find(setup_keys, member.first) !=
               std::end(setup_keys);
    });
}

// Write-side twin of the loaders' initial_state_file resolution
// (initialization/request.cpp, research/configuration.cpp): the document
// inside a sibling directory keyed by the running platform supersedes the
// shared spelling when it exists, so the write lands where the consumer
// will look first.
std::filesystem::path target_document(const io::Value& document,
                                      const std::filesystem::path& request) {
    const auto& object = io::as_object(document, "request");
    const auto* member = io::find_member(object, "initial_state_file");
    if (member == nullptr)
        throw std::invalid_argument(request.string() +
            ": the request declares no initial_state_file to regenerate");
    const auto& name = io::as_string(*member, "initial_state_file");
    if (name.empty() || name.find('\0') != std::string::npos)
        throw std::invalid_argument(request.string() +
            ": expected a nonempty initial_state_file path");
    const auto base = std::filesystem::absolute(request).parent_path();
    const std::filesystem::path file(name);
    auto target = std::filesystem::weakly_canonical(
        file.is_absolute() ? file : base / file);
    std::error_code error;
    if (const auto keyed = target.parent_path() / bike::platform::key() /
            target.filename();
        std::filesystem::is_regular_file(keyed, error))
        target = keyed;
    return target;
}

// The setup mutations prepare_environment applies before its settle
// (research/environment.cpp prepare_environment_into): a code-built request
// and a file-loaded one reach the identical t=0 through them. Mutations
// prepare_environment performs after the settle — sensors, programs,
// bootstrap — belong to the ride, not the document.
SetupRequest research_setup(bike::research::ResearchRequest request) {
    bike::research::sample_rider(request);
    request.setup.seed_decimal =
        bike::research::validate_seed(request.experiment.seed_decimal);
    request.setup.research_startup = true;
    request.setup.model.field =
        bike::research::research_field(request.setup.track,
                                       request.road_resolution_m);
    request.setup.runtime.strict = request.experiment.stop_on_model_violation;
    request.setup.runtime.async_telemetry = request.experiment.async_telemetry;
    request.setup.runtime.measurement_decimation =
        request.experiment.record_decimation;
    request.setup.runtime.record_retention =
        request.capture_mode == ::runtime::ResearchCaptureMode::full
            ? ::runtime::RetentionMode::full
            : request.capture_mode == ::runtime::ResearchCaptureMode::legacy
                ? ::runtime::RetentionMode::legacy
                : ::runtime::RetentionMode::off;
    request.setup.initial_state.reset();
    if (request.setup.model.physics)
        request.setup.model.physics->equilibrium_state_file.reset();
    return std::move(request.setup);
}

// Fresh start: every saved-state input dropped, the same settle the
// consuming path performs (app::prepare, research::prepare_environment ->
// make_settled_research_owner), then the sole writer that still exists.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
std::string capture(SetupRequest request) {
    const auto options = request.equilibrium;
    request.initial_state.reset();
    if (request.model.physics)
        request.model.physics->equilibrium_state_file.reset();
    auto owner = bike::initialization::make_setup(std::move(request));
    (void)bike::initialization::settle(*owner, options);
    return owner->capture_initial_state().payload_json();
}
} // namespace

void bt::record_initial_state(const std::filesystem::path& request) {
    if (!recording()) return;
    const auto document =
        bike::configuration::detail::parse_toml_file(request);
    const auto target = target_document(document, request);
    const auto payload = setup_request(document)
        ? capture(bike::initialization::read_setup_request(
              request, /*skip_initial_state=*/true))
        : capture(research_setup(
              bike::research::read_research_request(request)));
    write_text(target, payload);
}

void bt::record_initial_state(const bike::research::ResearchRequest& request,
                              const std::filesystem::path& document) {
    if (!recording()) return;
    write_text(document, capture(research_setup(request)));
}
NATIVE_DIAG_POP
