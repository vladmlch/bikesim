// Native subprocess test helper. It accepts only bounded typed operations.
#include "../platform/files.hpp"
#include "../platform/argv.hpp"
#include <charconv>
#include <chrono>
#include <iostream>
#include <memory>
#include <string_view>
#include <thread>

namespace {
int integer(std::string_view value, int maximum) {
    int result = 0; const auto end=std::to_address(value.cend());
    const auto parsed = std::from_chars(value.data(), end, result);
    if (parsed.ec != std::errc{} || parsed.ptr != end || result < 0 || result > maximum)
        throw std::invalid_argument("bounded nonnegative integer required");
    return result;
}
}
int main(int argc, char** argv) {
    try {
        const auto supplied=bike::platform::main_arguments(argc,argv);
        if (argc < 3) throw std::invalid_argument("usage: bike-test-child exit CODE | sleep MILLISECONDS | write PATH TEXT | echo TEXT");
        const std::string_view mode = supplied[1];
        if (mode == "exit" && argc == 3) return integer(supplied[2], 255);
        if (mode == "sleep" && argc == 3) { std::this_thread::sleep_for(std::chrono::milliseconds(integer(supplied[2], 60000))); return 0; }
        if (mode == "echo" && argc == 3) { std::cout << supplied[2] << '\n'; return 0; }
        if (mode == "write" && argc == 4) {
            const std::string_view text = supplied[3];
            if (text.size() > 1048576) throw std::invalid_argument("test write exceeds 1 MiB");
            bike::platform::write_exclusive(supplied[2], std::as_bytes(std::span{text.data(), text.size()})); return 0;
        }
        throw std::invalid_argument("unknown child operation or wrong argument count");
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 2; }
}
