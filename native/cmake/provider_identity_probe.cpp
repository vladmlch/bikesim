// Configure-time probe of the loaded numerics providers (BikeProviderIdentity.cmake).
//
// try_run compiles this file together with runtime/provider_images.cpp, the TU
// the runtime identity compiles, and links the selected backend. It runs the
// result with an output file and the loader names the qualified build requires.
// On success the file receives one line per loaded provider image: the path, a
// tab, and the LC_UUID or "-". A required image that is not loaded is an error:
// nothing is written, the discovery message goes to stderr and the exit is 1.
//
// The probe deliberately has no logic of its own: the discovery, and therefore
// the reported identity, is whatever the runtime would report in this process.
#include "runtime/provider_images.hpp"

#include <exception>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "usage: provider_identity_probe <output> [required-image...]\n";
        return 2;
    }
    const std::string output_path = argv[1];
    const std::vector<std::string> required(argv + 2, argv + argc);
    try {
        std::string text;
        for (const runtime::ProviderImage &image :
             runtime::loaded_provider_images(required)) {
            text += image.path;
            text += '\t';
            text += image.macho_uuid.value_or("-");
            text += '\n';
        }
        std::ofstream output(output_path, std::ios::binary);
        output << text;
        output.close();
        if (!output) {
            std::cerr << "cannot write the provider identity to " << output_path << '\n';
            return 1;
        }
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
