// steer_profile.cpp — the cold io face of rider/steer_balance.cpp: wire
// read/write plus the canonical-profile digest. The T56 hot-kernel clause
// bars io/* include edges under native/src/rider/, so every io:: consumer
// lives here while the per-step physics stays in the kernel TU.

#include "../rider/steer_balance.hpp"
#include "../rider/steer_balance_detail.hpp"

#include "../io/hash.hpp"
#include "../io/json.hpp"
#include "../runtime/setup_values.hpp"
#include "../runtime/value_readers.hpp"

#include <cmath>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>

namespace bike::rider {
namespace {
using namespace runtime::setupvalue;

Value profile_value(const SteerProfile& p, bool identity) {
    Array gains;
    for (const auto& row : p.gains) gains.push_back(reals(row));
    return Value{Object{{"schema", integer(p.schema)},
        {"speed_nodes_mps", reals(p.speed_nodes_mps)}, {"gains", Value{std::move(gains)}},
        {"reference_torque_nm", real(p.reference_torque_nm)}, {"cap_nm", real(p.cap_nm)},
        {"delay_s", real(p.delay_s)}, {"sha256_hex", identity ? text(p.sha256_hex) : Value{}},
        {"provenance_status", text(p.provenance_status)}}};
}
} // namespace

namespace detail {
void validate_steer_profile(const SteerProfile& p) {
    if (p.schema != 1) throw std::invalid_argument("steer.profile.schema: expected 1");
    if (p.speed_nodes_mps.empty() || p.gains.size() != p.speed_nodes_mps.size())
        throw std::invalid_argument("steer.profile.gains: expected one six-component row per speed node");
    for (std::size_t i = 0; i < p.speed_nodes_mps.size(); ++i) {
        if (!std::isfinite(p.speed_nodes_mps[i]) || p.speed_nodes_mps[i] <= 0. ||
            (i != 0 && p.speed_nodes_mps[i] <= p.speed_nodes_mps[i - 1]))
            throw std::invalid_argument("steer.profile.speed_nodes_mps: expected increasing positive finite nodes");
        for (const double k : p.gains[i]) if (!std::isfinite(k))
            throw std::invalid_argument("steer.profile.gains: nonfinite coefficient");
    }
    if (!std::isfinite(p.reference_torque_nm) || !std::isfinite(p.cap_nm) || p.cap_nm <= 0. ||
        !std::isfinite(p.delay_s) || p.delay_s < 0.)
        throw std::invalid_argument("steer.profile: invalid torque cap, reference, or reaction delay");
    if (p.provenance_status != "provisional" && p.provenance_status != "calibrated-on-fixture" &&
        p.provenance_status != "measured" && p.provenance_status != "synthetic")
        throw std::invalid_argument("steer.profile.provenance_status: unsupported status");
}
std::string steer_profile_fingerprint(const SteerProfile& p) {
    const auto bytes = io::canonical_json(profile_value(p, false));
    return io::sha256(std::as_bytes(std::span{bytes}));
}
} // namespace detail

SteerProfile read_steer_profile(const io::Value& value) {
    const auto& obj = vread::mapping(vread::Node{&value}, "steer.profile");
    const vread::Dict d{.value=&obj, .path="steer.profile"};
    // Parsed JSON already rejects duplicates, but the public Value boundary
    // also accepts documents assembled by native callers. Do not silently
    // fingerprint only the first of two conflicting safety-critical fields.
    for (std::size_t i = 0; i < obj.size(); ++i)
        for (std::size_t j = 0; j < i; ++j)
            if (obj[i].first == obj[j].first)
                vread::invalid(d.child(obj[i].first), "duplicate key");
    vread::exact(d, vread::keys("schema", "speed_nodes_mps", "gains", "reference_torque_nm",
        "cap_nm", "delay_s", "sha256_hex", "provenance_status"));
    if (vread::integer(vread::field(d, "schema"), d.child("schema")) != 1)
        vread::invalid(d.child("schema"), "expected 1");
    SteerProfile p;
    p.speed_nodes_mps = vread::vector(vread::field(d, "speed_nodes_mps"), d.child("speed_nodes_mps"));
    const auto& rows = vread::sequence(vread::field(d, "gains"), d.child("gains"));
    for (std::size_t i=0; i<rows.size(); ++i)
        p.gains.push_back(vread::fixed<6>(vread::Node{&rows[i]}, d.child("gains")+"["+std::to_string(i)+"]"));
    p.reference_torque_nm = vread::finite_real(vread::field(d, "reference_torque_nm"), d.child("reference_torque_nm"));
    p.cap_nm = vread::finite_real(vread::field(d, "cap_nm"), d.child("cap_nm"));
    p.delay_s = vread::finite_real(vread::field(d, "delay_s"), d.child("delay_s"));
    p.provenance_status = vread::string(vread::field(d, "provenance_status"), d.child("provenance_status"));
    detail::validate_steer_profile(p);
    p.sha256_hex = detail::steer_profile_fingerprint(p);
    const auto* supplied = vread::field(d, "sha256_hex").value;
    if (!io::is_null(*supplied) && vread::string(vread::Node{supplied}, d.child("sha256_hex")) != p.sha256_hex)
        vread::invalid(d.child("sha256_hex"), "differs from canonical profile payload");
    return p;
}
io::Value write_steer_profile(const SteerProfile& p) {
    detail::validate_steer_profile(p);
    if (p.sha256_hex != detail::steer_profile_fingerprint(p))
        throw std::invalid_argument("steer.profile.sha256_hex: differs from canonical profile payload");
    return profile_value(p, true);
}
} // namespace bike::rider
