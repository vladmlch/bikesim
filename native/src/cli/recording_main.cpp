#include "recording.hpp"
#include "../platform/argv.hpp"
#include <vector>
int main(int argc, char** argv) {
    std::vector<std::string_view> arguments;
    const auto raw=bike::platform::main_arguments(argc,argv);
    for (std::size_t i = 1; i < raw.size(); ++i) arguments.emplace_back(raw[i]);
    return bike::cli::recording_command(arguments);
}
