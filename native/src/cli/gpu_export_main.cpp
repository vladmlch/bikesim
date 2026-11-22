#include "cli/gpu_export.hpp"
#include "platform/argv.hpp"

#include <exception>
#include <iostream>

int main(int argc, char **argv) {
    try {
        const auto args = bike::platform::main_arguments(argc, argv);
        return bike::cli::run_gpu_export(
            static_cast<int>(args.size()), args.data(), std::cout, std::cerr);
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    } catch (...) {
        return 1;
    }
}
