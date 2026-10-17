#include "application.hpp"
#include "../model/export.hpp"
#include "../viz/plot.hpp"
#include <iostream>

namespace bike::cli {
int simulation_command(const Arguments& args) {
    model::BuildRequest request; request.solver.emplace(request.bike);
    const auto& solver = *request.solver;
    const auto plots = std::filesystem::path(args.text("output_plots_dir", "output/plots"));
    const auto models = std::filesystem::path(args.text("output_models_dir", "output/models"));
    std::filesystem::create_directories(plots); std::filesystem::create_directories(models);
    const bool all = args.flag("all");
    const bool defaults = !(args.flag("print_table") || args.flag("plot") || args.flag("plot_damper") ||
        args.flag("export_json") || args.flag("export_mujoco") || args.flag("playground") || all);
    if (all || defaults || args.flag("print_table"))
        std::cout << viz::geometry_table(request.bike, solver) << '\n' << viz::kinematics_table(solver)
                  << '\n' << viz::suspension_table(request.bike) << '\n';
    if (all || defaults || args.flag("plot")) {
        viz::write_artifact(viz::geometry_plot(request.bike, solver), plots / "linkage_geometry.png");
        viz::write_artifact(viz::leverage_plot(solver), plots / "leverage_ratio.png");
        viz::write_artifact(viz::compressed_comparison(request.bike, solver), plots / "suspension_compressed_comparison.png");
    }
    if (all || defaults || args.flag("plot") || args.flag("plot_damper"))
        viz::write_artifact(viz::damper_dyno(), plots / "damper_dyno_curves.png");
    if (all || args.flag("export_json")) model::export_geometry_json(request, models / "coordinates.json");
    if (all || args.flag("export_mujoco")) model::export_playground_models(request, models);
    return args.flag("playground") ? playground_command() : 0;
}
} // namespace bike::cli
