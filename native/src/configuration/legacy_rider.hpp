// configuration/legacy_rider.hpp — reading documents written before the
// seated-only fields were removed from RiderSpecs.
//
// The seated rider variant (removed 2026-11-04) was the only reader of
// ten RiderSpecs fields: the legs mode, the saddle/pedal/bar load shares,
// the three body resonances, the damping ratio and the saddle interface
// spring/damper. No remaining variant reads them (pinned before removal
// by test_mjcf seated_parameters_are_inert). Recordings and initial-state
// contracts written earlier still carry them in the resolved rider
// section. Readers accept a section whose removed members hold the
// shipped defaults and reject any other value: only documents that a
// shipped configuration could write are promised to replay.
#pragma once

#include "fields.hpp"
#include "../io/value.hpp"

#include <algorithm>
#include <array>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <variant>

namespace bike::configuration::legacy {

inline constexpr std::string_view kRemovedRiderLegsKey = "legs";
inline constexpr std::string_view kRemovedRiderLegsDefault = "rigid";
inline constexpr std::array<std::pair<std::string_view, double>, 9> kRemovedRiderReals{{
    {"saddle_share", .55},
    {"pedal_share", .33},
    {"bar_share", .12},
    {"torso_resonance_hz", 7.},
    {"leg_resonance_hz", 5.},
    {"arm_resonance_hz", 4.},
    {"damping_ratio", .40},
    {"saddle_interface_k_n_m", 101'000.},
    {"saddle_interface_c_ns_m", 2'762.}}};

// The first removed member of a recorded rider section that does not hold
// its shipped default (or has the wrong JSON kind); nullopt when every
// removed member present is inert. Absent members are inert: documents
// written after the removal carry none.
[[nodiscard]] inline std::optional<std::string_view>
non_default_rider_field(const io::Value& rider) {
    const auto& object = io::as_object(rider, "rider");
    if (const auto* legs = io::find_member(object, kRemovedRiderLegsKey)) {
        const auto* text = std::get_if<std::string>(&legs->data);
        if (text == nullptr || *text != kRemovedRiderLegsDefault)
            return kRemovedRiderLegsKey;
    }
    for (const auto& [name, value] : kRemovedRiderReals) {
        const auto* member = io::find_member(object, name);
        if (member == nullptr) continue;
        const auto* number = std::get_if<io::Number>(&member->data);
        if (number == nullptr) return name;
        bool ok = false;
        const double parsed = io::detail::float_from_lexeme(number->lexeme, ok);
        if (!ok || parsed != value) return name;
    }
    return std::nullopt;
}

// Erases every removed member, in place. Callers check
// non_default_rider_field first; erasing never inspects values.
inline void erase_removed_rider_fields(io::Value& rider) {
    auto* object = std::get_if<io::Value::Object>(&rider.data);
    if (object == nullptr)
        throw std::invalid_argument("legacy rider section is not an object");
    std::erase_if(*object, [](const auto& member) {
        return member.first == kRemovedRiderLegsKey ||
               std::ranges::any_of(kRemovedRiderReals, [&](const auto& removed) {
                   return member.first == removed.first;
               });
    });
}

// The current rider section as pre-removal configs serialized it: the ten
// removed members re-added at their shipped defaults with the record
// writer's repr_double lexemes. Contracts hash key-sorted JSON
// (io::canonical_json), so member position does not matter.
[[nodiscard]] inline io::Value with_removed_rider_defaults(io::Value rider) {
    auto* object = std::get_if<io::Value::Object>(&rider.data);
    if (object == nullptr)
        throw std::invalid_argument("legacy rider section is not an object");
    if (io::find_member(*object, kRemovedRiderLegsKey) != nullptr)
        throw std::invalid_argument("legacy rider section already carries legs");
    object->emplace_back(std::string(kRemovedRiderLegsKey),
                         io::Value{std::string(kRemovedRiderLegsDefault)});
    for (const auto& [name, value] : kRemovedRiderReals)
        object->emplace_back(std::string(name),
                             io::Value{io::Number{detail::repr_double(value)}});
    return rider;
}

} // namespace bike::configuration::legacy
