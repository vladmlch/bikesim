#include "builder_internal.hpp"
#include <charconv>
#include <memory>
#include <stdexcept>
#include <utility>

namespace bike::model::xml {
Element::Element(std::string tag_,Attributes attributes_)
    :tag(std::move(tag_)),attributes(std::move(attributes_)) {}
Element& Element::add(std::string tag_,Attributes attributes_) {
    children.push_back(std::make_unique<Element>(std::move(tag_),std::move(attributes_)));
    return *children.back();
}
void Element::set(std::string name,std::string value) {
    for(auto& [key,old]:attributes) if(key==name) { old=std::move(value); return; }
    attributes.emplace_back(std::move(name),std::move(value));
}
std::string Element::get(std::string_view name,std::string_view fallback) const {
    for(const auto& [key,value]:attributes) if(key==name) return value;
    return std::string(fallback);
}
Element* Element::child(std::string_view tag_) const {
    for(const auto& value:children) if(value->tag==tag_) return value.get();
    return nullptr;
}
Element* Element::find(std::string_view tag_,std::string_view name) const {
    for(const auto& value:children) {
        if(value->tag==tag_ && value->get("name")==name) return value.get();
        if(auto* result=value->find(tag_,name)) return result;
    }
    return nullptr;
}
std::vector<Element*> Element::direct(std::string_view tag_) const {
    std::vector<Element*> values;
    for(const auto& value:children) if(value->tag==tag_) values.push_back(value.get());
    return values;
}
std::vector<Element*> Element::descendants(std::string_view tag_) const {
    std::vector<Element*> values;
    for(const auto& value:children) {
        if(value->tag==tag_) values.push_back(value.get());
        auto nested=value->descendants(tag_); values.insert(values.end(),nested.begin(),nested.end());
    }
    return values;
}
std::unique_ptr<Element> Element::remove(Element* child_) {
    for(auto it=children.begin();it!=children.end();++it) if(it->get()==child_) {
        auto result=std::move(*it); children.erase(it); return result;
    }
    throw std::invalid_argument("XML element is not a direct child");
}
std::string escape_attribute(std::string_view value) {
    std::string result;
    for(char raw:value) {
        const auto ch=static_cast<unsigned char>(raw);
        switch(ch) {
        case '&':result+="&amp;";break; case '<':result+="&lt;";break;
        case '>':result+="&gt;";break; case '"':result+="&quot;";break;
        case '\t':result+="&#9;";break;case '\n':result+="&#10;";break;case '\r':result+="&#13;";break;
        default:if(ch<0x20) throw std::invalid_argument("MJCF attribute contains an XML control character");
            result.push_back(static_cast<char>(ch));
        }
    }
    return result;
}
namespace {
void emit(const Element& element,std::string& result,std::size_t depth) {
    result.append(depth*2,' ');result+='<';result+=element.tag;
    for(const auto& [key,value]:element.attributes) {
        result+=' ';result+=key;result+="=\"";result+=escape_attribute(value);result+='"';
    }
    if(element.children.empty()) { result+="/>\n";return; }
    result+=">\n";
    for(const auto& child:element.children) emit(*child,result,depth+1);
    result.append(depth*2,' ');result+="</";result+=element.tag;result+=">\n";
}
}
std::string document(const Element& root) {
    std::string result="<?xml version=\"1.0\" encoding=\"utf-8\"?>\n";emit(root,result,0);return result;
}
} // namespace bike::model::xml

namespace bike::model::detail {
std::string vec(const Vec3& value,int precision) { return f(value[0],precision)+" "+f(value[1],precision)+" "+f(value[2],precision); }
std::string vec17(const Vec3& value) { return g(value[0])+" "+g(value[1])+" "+g(value[2]); }
std::string fromto(const Vec3& a,const Vec3& b,int precision) { return vec(a,precision)+" "+vec(b,precision); }
Element& geom(Element& parent,std::string name,std::string type,Attributes attributes) {
    Attributes all{{"name",std::move(name)},{"type",std::move(type)}};
    all.insert(all.end(),attributes.begin(),attributes.end());return parent.add("geom",std::move(all));
}
Element& joint(Element& parent,std::string name,std::string type,Attributes attributes) {
    Attributes all{{"name",std::move(name)},{"type",std::move(type)},{"pos","0 0 0"}};
    all.insert(all.end(),attributes.begin(),attributes.end());return parent.add("joint",std::move(all));
}
Element& site(Element& parent,std::string name,std::string pos,std::string size,std::string rgba) {
    Attributes all{{"name",std::move(name)},{"pos",std::move(pos)},{"size",std::move(size)}};
    if(!rgba.empty()) all.emplace_back("rgba",std::move(rgba));return parent.add("site",std::move(all));
}
Element& marker(Element& parent,std::string name,const Vec3& pos,std::string size,std::string material) {
    return geom(parent,std::move(name),"sphere",{{"pos",vec(pos)},{"size",std::move(size)},
        {"material",std::move(material)},{"contype","0"},{"conaffinity","0"},{"mass","0"}});
}
Element& body(Element& parent,std::string name,const Vec3& pos) { return parent.add("body",{{"name",std::move(name)},{"pos",vec(pos)}}); }
Element& required(Element& root,std::string_view tag,std::string_view name) {
    auto* value=root.find(tag,name);if(!value) throw std::invalid_argument("physical topology needs '"+std::string(name)+"'");return *value;
}
Element& block(Element& root,std::string_view tag) { if(auto* value=root.child(tag)) return *value;return root.add(std::string(tag)); }
Element& inertial(Element& owner,double mass,const Vec3& com,const Mat3& tensor) {
    XmlBodyElement carrier;add_body_inertial(carrier,mass,com,tensor);
    const auto& a=*carrier.inertial;
    return owner.add("inertial",{{"mass",a.mass},{"pos",a.pos},{"fullinertia",a.fullinertia}});
}
void set_inertia(Element& owner,double mass,const Vec3& com,const Mat3& tensor) {
    for(auto* value:owner.direct("inertial")) (void)owner.remove(value);
    inertial(owner,mass,com,tensor);
    for(auto* value:owner.direct("geom")) value->set("mass","0");
}
namespace {
double mass_of(const Element& element) {
    const auto token=element.get("mass","0");double value=0.;
    const auto* end=std::to_address(token.end());
    const auto r=std::from_chars(token.data(),end,value);
    if(r.ec!=std::errc{} || r.ptr!=end || !std::isfinite(value))
        throw std::invalid_argument("invalid mass attribute in component registry");
    return value;
}
}
void register_component(MassRegistry* registry,std::string name,std::vector<Element*> parts) {
    if(!registry) return;
    std::vector<Element*> positive;
    for(auto* part:parts) if(mass_of(*part)>0.) positive.push_back(part);
    for(const auto& entry:*registry) if(entry.first==name) throw std::invalid_argument("component '"+name+"' was registered twice");
    if(positive.empty()) throw std::invalid_argument("component '"+name+"' has no mass-bearing parts");
    registry->emplace_back(std::move(name),std::move(positive));
}
void assign_budgets(Element& frame,MassRegistry& registry,const BikeMassSpecs& masses) {
    const auto budgets=masses.component_masses();
    std::set<std::string> expected,actual;
    for(const auto& [name,mass]:budgets) { (void)mass;expected.insert(name); }
    std::set<Element*> all,registered_geoms,registered_inertials;
    for(const auto& [name,parts]:registry) {
        actual.insert(name);
        for(auto* part:parts) {
            if(!all.insert(part).second) throw std::invalid_argument("a mass-bearing part belongs to multiple components");
            if(part->tag=="geom") registered_geoms.insert(part);else if(part->tag=="inertial") registered_inertials.insert(part);
        }
    }
    if(expected!=actual) throw std::invalid_argument("mass registry mismatch");
    std::set<Element*> geoms,inertials;
    for(auto* item:frame.descendants("geom")) if(mass_of(*item)>0. && !item->get("name").starts_with("geom_rider_")) geoms.insert(item);
    if(geoms!=registered_geoms) throw std::invalid_argument("mass registry does not cover every bike geom exactly once");
    for(auto* item:frame.descendants("body")) if(item->get("name")=="front_wheel" || item->get("name")=="rear_wheel")
        for(auto* value:item->direct("inertial")) inertials.insert(value);
    if(inertials!=registered_inertials) throw std::invalid_argument("wheel inertial registry mismatch");
    for(auto& [name,parts]:registry) {
        const auto budget=std::find_if(budgets.begin(),budgets.end(),[&](const auto& item){return item.first==name;})->second;
        if(name=="front_wheel" || name=="rear_wheel") {
            if(parts.size()!=1 || parts[0]->tag!="inertial" || mass_of(*parts[0])!=budget)
                throw std::invalid_argument("wheel component '"+name+"' must own one budgeted inertial");
        } else {
            std::vector<XmlGeomElement> weights;weights.reserve(parts.size());
            for(auto* part:parts) {
                if(part->tag!="geom") throw std::invalid_argument("component '"+name+"' must own geoms");
                weights.push_back({part->get("mass","0")});
            }
            assign_component_mass(weights,budget);
            for(std::size_t i=0;i<parts.size();++i) parts[i]->set("mass",*weights[i].mass);
        }
    }
}
} // namespace bike::model::detail
