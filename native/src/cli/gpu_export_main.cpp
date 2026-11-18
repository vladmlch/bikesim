#include "cli/gpu_export.hpp"
#include "platform/argv.hpp"

#include <iostream>

// NOLINTNEXTLINE(bugprone-exception-escape) run_gpu_export catches std::exception internally
int main(int argc, char **argv) {
    const auto args = bike::platform::main_arguments(argc, argv);
    return bike::cli::run_gpu_export(
        static_cast<int>(args.size()), args.data(), std::cout, std::cerr);
}
