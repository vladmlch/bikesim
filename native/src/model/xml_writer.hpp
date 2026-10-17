#pragma once
#include <initializer_list>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace bike::model::xml {
using Attributes = std::vector<std::pair<std::string, std::string>>;
// Stable child addresses are necessary for the component mass registry and
// parent maps while siblings are appended. Elements uniquely own descendants.
struct Element {
    std::string tag;
    Attributes attributes;
    std::vector<std::unique_ptr<Element>> children;
    explicit Element(std::string tag_, Attributes attributes_ = {});
    Element& add(std::string tag_, Attributes attributes_ = {});
    void set(std::string name, std::string value);
    [[nodiscard]] std::string get(std::string_view name,
                                  std::string_view fallback = {}) const;
    [[nodiscard]] Element* child(std::string_view tag_) const;
    [[nodiscard]] Element* find(std::string_view tag_, std::string_view name) const;
    [[nodiscard]] std::vector<Element*> direct(std::string_view tag_) const;
    [[nodiscard]] std::vector<Element*> descendants(std::string_view tag_) const;
    [[nodiscard]] std::unique_ptr<Element> remove(Element* child_);
};
[[nodiscard]] std::string escape_attribute(std::string_view value);
[[nodiscard]] std::string document(const Element& root);
} // namespace bike::model::xml
