// W26: the brake codec owns hinge-relative latch state. Ground anchors live in the tyre.
#include "brake_state.hpp"
#include "../runtime/setup_values.hpp"
#include "../runtime/value_readers.hpp"

#include <cmath>
#include <string>
#include <utility>

namespace bike::spatial {
namespace {
using namespace runtime::setupvalue;
vread::Dict object(const Value& value, std::string path) {
    const auto& fields = vread::mapping(vread::Node{&value}, path);
    for (std::size_t i=0;i<fields.size();++i)
        for (std::size_t j=0;j<i;++j)
            if(fields[i].first==fields[j].first)
                vread::invalid(path+"."+fields[i].first,"duplicate key");
    return {.value=&fields,.path=std::move(path)};
}
double number(const vread::Dict& d, std::string_view key) {
    return vread::finite_real(vread::field(d,key),d.child(key));
}
std::int64_t count(const vread::Dict& d,std::string_view key,std::int64_t minimum=0) {
    const auto n=vread::integer(vread::field(d,key),d.child(key));
    if(n<minimum)vread::invalid(d.child(key),"integer below accepted range");
    return n;
}
std::uint8_t mask(const vread::Dict& d) {
    const auto n=count(d,"valid_mask");
    if(n>3)vread::invalid(d.child("valid_mask"),"only front/rear validity bits are defined");
    return static_cast<std::uint8_t>(n);
}
void schema(const vread::Dict& d) {
    if(count(d,"schema")!=1)vread::invalid(d.child("schema"),"unsupported state schema");
}
void nonnegative(double n,const std::string& path) {
    if(n<0.)vread::invalid(path,"expected nonnegative value");
}
Value brake_value(const BrakeState& s) {
    Array wheels;
    for (const auto& w : s.wheel)
        wheels.emplace_back(Object{{"engaged", Value{w.engaged}},
            {"latch_angle_rad", real(w.latch_angle_rad)},
            {"unwrapped_angle_rad", real(w.unwrapped_angle_rad)},
            {"stored_energy_j", real(w.stored_energy_j)},
            {"previous_demand", real(w.previous_demand)}});
    return Value{Object{{"schema", integer(s.schema)}, {"wheel", Value{std::move(wheels)}},
        {"interval_index", integer(s.interval_index)}, {"valid_mask", integer(s.valid_mask)}}};
}
} // namespace

BrakeState canonical_released_brake() noexcept { return {}; }
BrakeState decode_brake(const Value& value) {
    const auto d = object(value, "brake");
    vread::exact(d, vread::keys("schema", "wheel", "interval_index", "valid_mask"));
    schema(d);
    BrakeState s;
    const auto& wheels = vread::sequence(vread::field(d, "wheel"), d.child("wheel"));
    if (wheels.size() != 2) vread::invalid(d.child("wheel"), "expected front/rear pair");
    for (std::size_t i = 0; i < wheels.size(); ++i) {
        const auto w = object(wheels[i], d.child("wheel") + "[" + std::to_string(i) + "]");
        vread::exact(w, vread::keys("engaged", "latch_angle_rad", "unwrapped_angle_rad",
            "stored_energy_j", "previous_demand"));
        auto& target = s.wheel[i];
        target.engaged = vread::boolean(vread::field(w, "engaged"), w.child("engaged"));
        target.latch_angle_rad = number(w, "latch_angle_rad");
        target.unwrapped_angle_rad = number(w, "unwrapped_angle_rad");
        target.stored_energy_j = number(w, "stored_energy_j");
        nonnegative(target.stored_energy_j, w.child("stored_energy_j"));
        if (!target.engaged && target.stored_energy_j != 0.)
            vread::invalid(w.child("stored_energy_j"), "released brake cannot store bite energy");
        target.previous_demand = number(w, "previous_demand");
        if (target.previous_demand < 0. || target.previous_demand > 1.)
            vread::invalid(w.child("previous_demand"), "expected demand in [0,1]");
    }
    s.interval_index = count(d, "interval_index", -1);
    s.valid_mask = mask(d);
    return s;
}
Value encode_brake(const BrakeState& s) { auto v=brake_value(s); (void)decode_brake(v); return v; }
} // namespace bike::spatial
