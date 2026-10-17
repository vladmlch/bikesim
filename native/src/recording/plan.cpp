#include "encoding.hpp"
#include "../runtime/encode.hpp"
#include "value_util.hpp"
#include <algorithm>
#include <array>
#include <set>

namespace bike::recording {
namespace {
io::Value leaf_plan(const FieldSpec& f, const std::vector<std::string>& absolute) {
    io::Value shape;
    if (f.shape && !f.shape->empty()) { io::Value::Array a; for (auto n : *f.shape) a.push_back(value::integer(n)); shape.data = std::move(a); }
    io::Value::Object out{{"name", value::text(f.key)}, {"dtype", value::text(f.dtype)}, {"path", value::strings(f.path)},
        {"optional", io::Value{f.optional}}, {"shape", std::move(shape)}, {"except_keys", value::strings(f.except_keys)},
        {"convert", absolute == std::vector<std::string>{"model_status", "numerically_valid"} ? value::text("verdict") : io::Value{}}};
    if (!f.children.empty()) {
        io::Value::Array children;
        for (const auto& c : f.children) { auto path = absolute; path.insert(path.end(), c.path.begin(), c.path.end()); children.push_back(leaf_plan(c, path)); }
        out.emplace_back("children", io::Value{std::move(children)});
    }
    return io::Value{std::move(out)};
}
} // namespace
runtime::EncodePlan compile_plan(const Catalog& catalog, std::string_view name) {
    const auto& spec = catalog.table(name);
    io::Value::Object plan{{"table", value::text(spec.name)}, {"kind", value::text(spec.kind)}};
    if (spec.kind == "channels") {
        io::Value::Array columns, masks; std::set<std::string> seen;
        auto add = [&](std::span<const std::string> path) {
            for (auto op : {"present", "valid"}) {
                auto mask = "__" + value::join(path) + "_" + op;
                if (seen.insert(mask).second) masks.push_back(io::Value{io::Value::Object{{"name", value::text(mask)}, {"path", value::strings(path)}, {"op", value::text(op)}}});
            }
        };
        for (const auto& f : spec.fields) {
            columns.push_back(leaf_plan(f, f.path));
            for (std::size_t depth = 1; depth < f.path.size(); ++depth) add(std::span<const std::string>(f.path).first(depth));
        }
        auto roots = spec.claims; std::sort(roots.begin(), roots.end());
        for (const auto& root : roots) { const std::array<std::string, 1> path{root}; add(path); }
        plan.emplace_back("roots", value::strings(roots)); plan.emplace_back("all_roots", value::strings(catalog.channel_roots()));
        plan.emplace_back("columns", io::Value{std::move(columns)}); plan.emplace_back("masks", io::Value{std::move(masks)});
    }
    return runtime::parse_encode_plan(io::Value{std::move(plan)});
}
} // namespace bike::recording
