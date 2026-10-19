#include "contracts.hpp"
#include "value_util.hpp"
#include "../io/hash.hpp"
#include "../io/json.hpp"
#include "../platform/files.hpp"
#include <algorithm>

namespace bike::recording {
Asset asset(std::string name, std::string media_type, std::vector<std::byte> content) {
    Asset out{.name=std::move(name), .media_type=std::move(media_type), .sha256=io::sha256(content), .content=std::move(content)}; out.validate(); return out;
}
Asset json_asset(std::string name, const io::Value& value) {
    const auto json = io::recording_json(value) + '\n';
    const auto bytes = std::as_bytes(std::span(json.data(), json.size()));
    return asset(std::move(name), "application/json", {bytes.begin(), bytes.end()});
}
std::vector<Asset> replace_recipe(std::vector<Asset> assets, const io::Value& recipe) {
    std::erase_if(assets, [](const auto& value) { return value.name == "recipe.json"; });
    assets.push_back(json_asset("recipe.json", recipe)); return assets;
}
io::Value input_identity_at_finalization(const io::Value& metadata) {
    io::Value::Object result;
    const auto* resolved = io::find_member(io::as_object(metadata, "recording metadata"), "resolved_config");
    if (!resolved) return io::Value{std::move(result)}; // legacy owned bootstrap has no declared input paths
    const auto& config = io::as_object(*resolved, "resolved_config");
    const auto* physics = io::find_member(config, "physics");
    if (!physics) return io::Value{std::move(result)};
    const auto* articulated = io::find_member(io::as_object(*physics, "resolved_config.physics"), "articulated");
    if (!articulated) return io::Value{std::move(result)};
    const auto& profiles = io::as_object(*articulated, "resolved_config.physics.articulated");
    for (std::string_view key : {"joint_envelope", "joint_strength"}) {
        const auto* path = io::find_member(profiles, std::string(key) + "_path");
        if (!path || io::is_null(*path)) continue;
        const auto* expected = io::find_member(config, std::string(key) + "_sha256");
        io::Value::Object status{{"setup_sha256", expected ? *expected : io::Value{}}};
        try {
            const auto current = platform::file_sha256(io::as_string(*path, key));
            const bool same = expected && !io::is_null(*expected) && io::as_string(*expected, key) == current;
            status.emplace_back("status", value::text(same ? "unchanged" : "changed"));
            status.emplace_back("current_sha256", value::text(current));
        } catch (const std::exception& error) {
            status.emplace_back("status", value::text("unavailable"));
            status.emplace_back("error", value::text(error.what()));
        }
        result.emplace_back(key, io::Value{std::move(status)});
    }
    return io::Value{std::move(result)};
}
} // namespace bike::recording
