#include "application.hpp"
#include "../platform/provenance.hpp"
#include "../platform/signals.hpp"
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <string>

namespace bike::cli {
int settle_command(const Arguments& args) {
    // The pose fingerprint binds the settle contract (max_steps, tolerance),
    // so the defaults here must be the ones a ride settles with: no explicit
    // budget (the solver's timestep-derived default) and tolerance 0.05.
    // An explicit --max-steps is a different contract, and a ride will not
    // accept the resulting pose through equilibrium_state_file.
    const std::optional<std::int64_t> steps =
        args.has("max_steps") ? std::optional<std::int64_t>(args.integer("max_steps")) : std::nullopt;
    const double tolerance = args.real("tolerance", .05);
    if ((steps && *steps <= 0) || !std::isfinite(tolerance) || tolerance <= 0.)
        throw ArgumentError("--max-steps and --tolerance must be positive");
    const auto output = std::filesystem::path(args.text("out"));
    if (std::filesystem::is_directory(output)) throw ArgumentError("--out must be a file path, not a directory");
    // ResolvedRide is several KB; heap storage keeps this frame bounded.
    const auto resolved = resolve_ride(args, true);
    resolved->setup.equilibrium.max_steps = steps;
    resolved->setup.equilibrium.tolerance = tolerance;
    const auto options = resolved->setup.equilibrium;
    const auto track = resolved->setup.track.name;
    std::cout << "[bike-settle] " << track << ": settling (budget "
              << (steps ? std::to_string(*steps) + " steps" : std::string("default")) << ", tolerance "
              << tolerance << ") -> " << output << '\n' << std::flush;
    if (steps || tolerance != initialization::EquilibriumOptions{}.tolerance)
        std::cout << "[bike-settle] note: a non-default --max-steps/--tolerance is bound into the pose "
                     "fingerprint; bike-ride loads only poses settled with the defaults\n" << std::flush;
    const auto start = platform::monotonic_seconds();
    std::unique_ptr<initialization::SetupOwner> setup;
    initialization::EquilibriumResult equilibrium;
    platform::InterruptShield const interrupt;
    try {
        setup = initialization::make_setup(std::move(resolved->setup));
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
