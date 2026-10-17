#include "application.hpp"
#include "recording.hpp"
#include "../platform/provenance.hpp"
#include <iostream>

namespace bike::cli {
std::filesystem::path executable_path() { return platform::executable_path(); }
int dispatch(std::string_view executable, std::span<const std::string_view> input) {
    const auto command = std::filesystem::path(executable).filename().string();
    // The standalone analyst main never links this simulation dispatcher.
    if (command == "bike-recording") return recording_command(input);
    try {
        const auto args = parse_arguments(command, input);
        if (args.help) { std::cout << help_text(command); return 0; }
        if (command == "bike-ride") return ride_command(args);
        if (command == "bike-research") return research_command(args);
        if (command == "bike-settle") return settle_command(args);
        if (command == "bike-export") return export_command(args);
        if (command == "bike-sim") return simulation_command(args);
        if (command == "bike-playground") return playground_command();
        if (command == "bike-replay") return replay_command(args);
        if (command == "bike-batch") return batch_command(args);
        throw ArgumentError("unknown executable: " + command);
    } catch (const ArgumentError& error) {
        std::cerr << command << ": error: " << error.what() << '\n'; return 2;
    } catch (const runtime::InvalidReferenceRun& error) {
        std::cerr << command << ": model invalid: " << error.what() << '\n'; return 2;
    } catch (const std::invalid_argument& error) {
        std::cerr << command << ": error: " << error.what() << '\n'; return 2;
    } catch (const std::exception& error) {
        std::cerr << command << ": " << error.what() << '\n'; return 1;
    }
}
} // namespace bike::cli
