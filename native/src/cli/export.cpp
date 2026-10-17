#include "application.hpp"
#include "../model/export.hpp"
#include <iostream>

namespace bike::cli {
int export_command(const Arguments& args) {
    const auto output = std::filesystem::path(args.text("output_dir", "output/models"));
    std::filesystem::create_directories(output);
    model::BuildRequest request;
    request.solver.emplace(request.bike);
    if (!args.flag("xml_only")) model::export_geometry_json(request, output / "coordinates.json");
    if (!args.flag("json_only")) model::export_playground_models(request, output);
    return 0;
}
} // namespace bike::cli
