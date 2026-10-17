#include "application.hpp"
#include "../platform/argv.hpp"
#include <iostream>
#include <vector>

int main(int argc, char** argv) {
    try {
        const auto supplied=bike::platform::main_arguments(argc,argv);
        std::vector<std::string_view> arguments;
        arguments.reserve(argc > 1 ? static_cast<std::size_t>(argc - 1) : 0);
        for (std::size_t i = 1; i < supplied.size(); ++i) arguments.emplace_back(supplied[i]);
        return bike::cli::dispatch(supplied.empty() ? "bike-sim" : supplied.front(), arguments);
    } catch (const std::exception& error) {
        std::cerr << "native CLI: " << error.what() << '\n'; return 1;
    }
}
