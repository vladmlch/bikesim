#include "../runtime/realtime_runner.hpp"
#include "../platform/argv.hpp"
#include <iostream>
#include <vector>
int main(int argc, char** argv) {
    try {
        const auto owned = bike::platform::main_arguments(argc, argv);
        std::vector<std::string_view> arguments;
        for (std::size_t i = 1; i < owned.size(); ++i) arguments.emplace_back(owned[i]);
        return runtime::measurement::measure_realtime_command(arguments, std::cout, std::cerr);
    } catch (const std::exception& error) { std::cerr << "MEASUREMENT ERROR: " << error.what() << '\n'; return 2; }
}
