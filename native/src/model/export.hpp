#pragma once
#include "builder.hpp"
#include "../io/value.hpp"

namespace bike::model {
// Complete source dataclass records for setup/cache provenance. These are
// distinct from the human-facing hardpoint/geometry JSON export schema.
[[nodiscard]] io::Value bike_specs_value(const geometry::BikeSpecs& specs);
[[nodiscard]] io::Value mass_specs_value(const BikeMassSpecs& specs);
[[nodiscard]] io::Value geometry_value(const BuildRequest& request);
void export_mjcf(const BuildRequest& request,const std::filesystem::path& file);
void export_geometry_json(const BuildRequest& request,const std::filesystem::path& file);
void export_playground_models(const std::filesystem::path& directory);
// Configurable twin preserves source export_playground_models arguments:
// geometry/solver, include_rider and tyre_model are used; ride uses the
// default seated rider and standard is always bike-only.
void export_playground_models(const BuildRequest& request,const std::filesystem::path& directory);

struct ExportOptions {
    std::filesystem::path xml_file = "bike_model.xml";
    std::filesystem::path json_file = "coordinates.json";
    bool xml_only = false;
    bool json_only = false;
};
// The two selection flags compose the existing separate export seams.
void export_model(const BuildRequest& request,const ExportOptions& options = {});
} // namespace bike::model
