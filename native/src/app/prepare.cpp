#include "prepare.hpp"

namespace bike::app {
PreparedSimulation prepare(SetupRequest request) {
    const auto origin = request.origin;
    try {
        const auto options = request.equilibrium;
        auto setup = initialization::make_setup(std::move(request));
        (void)initialization::settle(*setup, options);
        auto bootstrap = setup->seal_bootstrap();
        return {std::move(bootstrap), setup->metadata()};
    } catch (const std::invalid_argument& error) {
        if (origin.empty()) throw;
        throw std::invalid_argument(origin + ": " + error.what());
    }
}
PreparedSimulation prepare(const std::filesystem::path& request_toml) {
    return prepare(initialization::read_setup_request(request_toml));
}
} // namespace bike::app
