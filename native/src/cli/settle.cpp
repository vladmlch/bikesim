#include "application.hpp"
#include "../platform/provenance.hpp"
#include "../platform/signals.hpp"
#include <cmath>
#include <iomanip>
#include <iostream>

namespace bike::cli {
int settle_command(const Arguments& args) {
    const auto steps = args.integer("max_steps", 400000);
    const double tolerance = args.real("tolerance", .05);
    if (steps <= 0 || !std::isfinite(tolerance) || tolerance <= 0.)
        throw ArgumentError("--max-steps and --tolerance must be positive");
    const auto output = std::filesystem::path(args.text("out"));
    if (std::filesystem::is_directory(output)) throw ArgumentError("--out must be a file path, not a directory");
    auto resolved = resolve_ride(args, true);
    resolved.setup.equilibrium.max_steps = steps;
    resolved.setup.equilibrium.tolerance = tolerance;
    const auto options = resolved.setup.equilibrium;
    const auto track = resolved.setup.track.name;
    std::cout << "[bike-settle] " << track << ": settling (budget " << steps << " steps, tolerance "
              << tolerance << ") -> " << output << '\n' << std::flush;
    const auto start = platform::monotonic_seconds();
    std::unique_ptr<initialization::SetupOwner> setup;
    initialization::EquilibriumResult equilibrium;
    platform::InterruptShield interrupt;
    try {
        setup = initialization::make_setup(std::move(resolved.setup));
        equilibrium = initialization::settle(*setup, options);
    } catch (const std::exception& error) {
        std::cerr << "[bike-settle] settle failed: " << error.what() << '\n'; return interrupt.deferred() ? 130 : 3;
    }
    if (interrupt.deferred()) return 130;
    const auto wall = platform::monotonic_seconds() - start;
    try { initialization::write_state_file(output, setup->capture_state_file()); }
    catch (const std::exception& error) {
        std::cerr << "[bike-settle] could not write " << output << ": " << error.what() << '\n'; return 4;
    }
    std::cout << "[bike-settle] " << track << ": " << (equilibrium.cache_hit ? "auto-cache hit" : "freshly settled")
              << ", steps=" << equilibrium.steps << " residual=" << std::setprecision(6) << equilibrium.residual_qacc
              << " wall=" << std::fixed << std::setprecision(1) << wall << "s\n"
              << "[bike-settle] fork " << equilibrium.fork_travel_mm << " mm, shock " << equilibrium.shock_stroke_mm
              << " mm, rear " << equilibrium.rear_travel_mm << " mm\n[" << "bike-settle] wrote " << output << '\n';
    return interrupt.deferred() ? 130 : 0;
}
} // namespace bike::cli
