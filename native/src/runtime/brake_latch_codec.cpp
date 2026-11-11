// runtime/brake_latch_codec.cpp — see the header for the contract.
#include "brake_latch_codec.hpp"

#include <string>

#include "setup_values.hpp"
#include "value_readers.hpp"

namespace runtime {

bike::io::Value encode_brake_latch(const StaticBrake::State &state) {
    bike::io::Value::Object out;
    out.emplace_back("front", state.latch_front
                                  ? setupvalue::real(*state.latch_front)
                                  : bike::io::Value{});
    out.emplace_back("rear", state.latch_rear
                                 ? setupvalue::real(*state.latch_rear)
                                 : bike::io::Value{});
    return bike::io::Value{std::move(out)};
}

StaticBrake::State decode_brake_latch(const bike::io::Value &value,
                                      std::string_view path) {
    const vread::Dict section{
        .value = &vread::mapping(vread::Node{&value}, path),
        .path = std::string(path)};
    vread::exact(section, vread::keys("front", "rear"));
    return {.latch_front =
                vread::optional_real(vread::field(section, "front"),
                                     section.child("front")),
            .latch_rear =
                vread::optional_real(vread::field(section, "rear"),
                                     section.child("rear"))};
}

} // namespace runtime
