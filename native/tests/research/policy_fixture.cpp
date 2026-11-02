// Deliberately nonconforming plugins exercise the host error boundary. These
// callbacks must never be installed as production policies.
#include <bike/policy_v1.h>
#include <algorithm>
#include <cstdint>
#include <limits>
#include <ranges>
#include <stdexcept>
#include <string_view>

#define BIKE_FIXTURE_EXPORT __attribute__((visibility("default")))

#if defined(BIKE_POLICY_FIXTURE_MISSING_SYMBOL)
extern "C" BIKE_FIXTURE_EXPORT int bike_policy_fixture_without_api(void);
extern "C" BIKE_FIXTURE_EXPORT int bike_policy_fixture_without_api(void) { return 1; }
#else
namespace {
// NOLINTNEXTLINE(performance-enum-size) enum width matches the reference ABI layout
enum class Case : std::uint8_t { good, create_fail, reset_throw, act_throw, bad_number, bad_size, bad_error, destroy_throw };
struct State { Case scenario; };
int reject(BikeErrorV1* error, std::string_view text) {
    error->written = std::min(error->capacity, text.size());
    auto const dest = std::views::counted(error->data, static_cast<std::ptrdiff_t>(error->written));
    std::ranges::copy(text.substr(0, error->written), dest.begin());
    return 17;
}
int create(const char* data, std::size_t count, void** out, BikeErrorV1* error) {
    const std::string_view document(data, count);
    Case scenario = Case::good;
    if (document == R"({"case":"create_fail"})") scenario = Case::create_fail;
    else if (document == R"({"case":"reset_throw"})") scenario = Case::reset_throw;
    else if (document == R"({"case":"act_throw"})") scenario = Case::act_throw;
    else if (document == R"({"case":"bad_number"})") scenario = Case::bad_number;
    else if (document == R"({"case":"bad_size"})") scenario = Case::bad_size;
    else if (document == R"({"case":"bad_error"})") scenario = Case::bad_error;
    else if (document == R"({"case":"destroy_throw"})") scenario = Case::destroy_throw;
    else if (document != R"({"case":"good"})") return reject(error, "unknown fixture case");
    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory) boundary code holds a non-owning handle by design
    *out = new State{scenario};
    // Allocation already belongs to the host instance guard even on failure.
    return scenario == Case::create_fail ? reject(error, "injected create failure") : 0;
}
int reset(void* state, const char*, std::size_t, BikeErrorV1*) {
    if (static_cast<State*>(state)->scenario == Case::reset_throw) throw std::runtime_error("injected reset throw");
    return 0;
}
int act(void* state, const BikeObservationV1*, const BikeDemandV1*, BikeControlV1* output, BikeErrorV1* error) {
    const auto scenario = static_cast<State*>(state)->scenario;
    if (scenario == Case::act_throw) throw std::runtime_error("injected act throw");
    output->present_mask = BIKE_CONTROL_MOTOR_TORQUE;
    output->motor_torque_nm = scenario == Case::bad_number ? std::numeric_limits<double>::quiet_NaN() : 0.;
    if (scenario == Case::bad_size) output->struct_size = 0;
    if (scenario == Case::bad_error) error->written = error->capacity + 1;
    return 0;
}
void destroy(void* pointer) {
    const auto scenario = static_cast<State*>(pointer)->scenario;
    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory) boundary code holds a non-owning handle by design
    delete static_cast<State*>(pointer);
    if (scenario == Case::destroy_throw) throw std::runtime_error("injected destroy throw");
}
const BikePolicyV1 descriptor{.abi_version=BIKE_POLICY_ABI_VERSION, .struct_size=sizeof(BikePolicyV1), .required_capabilities=0, .create=create, .reset=reset, .act=act, .destroy=destroy,
    .sensor_schema_version=BIKE_SENSOR_SCHEMA_VERSION};
}
extern "C" BIKE_FIXTURE_EXPORT const BikePolicyV1* bike_policy_v1(void) { return &descriptor; }
#endif
