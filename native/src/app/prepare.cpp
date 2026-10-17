#include "prepare.hpp"
#include <memory>

namespace bike::app {
namespace {
[[gnu::noinline]] std::unique_ptr<initialization::SetupOwner> make_settled_owner(SetupRequest& request) {
    const auto options = request.equilibrium;
    auto setup = initialization::make_setup(std::move(request));
    (void)initialization::settle(*setup, options);
    return setup;
}
[[gnu::noinline]] void seal_owner_into(initialization::SetupOwner& setup, PreparedSimulation& out) {
    out.bootstrap = setup.seal_bootstrap();
    out.metadata = setup.metadata();
}
[[gnu::noinline]] void prepare_into(SetupRequest& request, PreparedSimulation& out) {
    const auto origin = request.origin;
    try {
        auto setup = make_settled_owner(request);
        seal_owner_into(*setup, out);
    } catch (const std::invalid_argument& error) {
        if (origin.empty()) throw;
        throw std::invalid_argument(origin + ": " + error.what());
    }
}
[[gnu::noinline]] std::unique_ptr<SetupRequest> load_request_ptr(const std::filesystem::path& request_toml) {
    return std::make_unique<SetupRequest>(initialization::read_setup_request(request_toml));
}
} // namespace

PreparedSimulation prepare(SetupRequest request) {
    PreparedSimulation out;
    prepare_into(request, out);
    return out;
}
PreparedSimulation prepare(const std::filesystem::path& request_toml) {
    auto req = load_request_ptr(request_toml);
    PreparedSimulation out;
    prepare_into(*req, out);
    return out;
}
} // namespace bike::app
