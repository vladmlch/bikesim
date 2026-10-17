#include "../batch/plan.hpp"
#include "../platform/argv.hpp"
#include "../platform/provenance.hpp"
#include <iostream>

int main(int argc, char** argv) {
    try {
        const auto supplied=bike::platform::main_arguments(argc,argv);
        std::vector<std::string_view> input;
        for (std::size_t i = 1; i < supplied.size(); ++i) input.emplace_back(supplied[i]);
        return bike::batch::research_sweep(input, bike::platform::executable_path().parent_path() / "bike-research");
    } catch (const std::exception& error) {
        std::cerr << "bike-research-sweep: " << error.what() << '\n'; return 2;
    }
}
