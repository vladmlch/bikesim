// test_t09.cpp — owned ride runtime + bootstrap boundary regressions
// (task T09): runtime::read_bootstrap / compose_bootstrap and
// NativeRideRuntime exercised in the Python-free graph. This target's
// compile context carries no Python/nanobind include, so compiling
// runtime/runtime.hpp + runtime/bootstrap_io.hpp IS the include-closure
// check — binding headers are deliberately absent.
//
// The oracle halves:
//   * bootstrap/* — capture_bootstrap directories: config.json,
//     state.json (externalized $array/$tuple/$nonfinite/$bytes tags),
//     model.mjb, meta.json pins and integration.f64 bitwise goldens.
//   * bootstrap/welded/steps_control_* — advance_python x4 under
//     RideControl(motor=25Nm, human=10Nm): the post-advance integration
//     vector was produced by production numba kernels, so it compares by
//     the assert_tree_close tolerance (1e-9); native-vs-native reset
//     replay compares bitwise (the same binary, the same state).
//   * bootstrap/faults/* — single-mutation documents whose expected.json
//     pins the exact "path: reason" diagnostic the nanobind decoder
//     produced ("invalid"), plus one recorded unexpected-acceptance.
#include "support/migration_test.hpp"
#include "runtime/runtime.hpp"
#include "runtime/bootstrap_io.hpp"
#include "diag.hpp"
#include "io/hash.hpp"
#include "io/json.hpp"
#include "io/value.hpp"

#include <bit>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iterator>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <variant>
#include <vector>

namespace {

namespace bio = bike::io;
namespace fs = std::filesystem;

// ---- document/field plumbing ----------------------------------------------

[[nodiscard]] bio::Value parse_doc(const fs::path &path) {
    return bio::parse_json(bt::text(path));
}

[[nodiscard]] const bio::Value &member(const bio::Value &value,
                                       std::string_view key) {
    const auto *const object = std::get_if<bio::Value::Object>(&value.data);
    if (object == nullptr)
        throw std::invalid_argument("document member access on a non-object");
    const bio::Value *const found = bio::find_member(*object, key);
    if (found == nullptr)
        throw std::invalid_argument("document member missing: " +
                                    std::string(key));
    return *found;
}

[[nodiscard]] std::int64_t as_int(const bio::Value &value) {
    const auto *const number = std::get_if<bio::Number>(&value.data);
    if (number == nullptr ||
        !bio::is_integer_lexeme(number->lexeme))
        throw std::invalid_argument("expected an integer document member");
    std::int64_t out = 0;
    const char *const last = std::to_address(number->lexeme.cend());
    const auto parsed =
        std::from_chars(number->lexeme.data(), last, out, 10);
    if (parsed.ec != std::errc() || parsed.ptr != last)
        throw std::invalid_argument("integer document member out of range");
    return out;
}

[[nodiscard]] const std::string &as_string(const bio::Value &value) {
    const auto *const text = std::get_if<std::string>(&value.data);
    if (text == nullptr)
        throw std::invalid_argument("expected a string document member");
    return *text;
}

[[nodiscard]] runtime::ModelImage model_image(const fs::path &directory) {
    const std::string bytes = bt::text(directory / "model.mjb");
    runtime::ModelImage out;
    out.mjb.reserve(bytes.size());
    for (const char unit : bytes)
        out.mjb.push_back(static_cast<std::byte>(unit));
    out.sha256 = bio::sha256(out.mjb);
    return out;
}

// assert_tree_close's leaf rule (atol=rtol=1e-9) — the welded advance
// oracle ran production numba kernels, which may sit ~1 ulp off the
// pure-Python arithmetic the native port is bitwise-locked to.
void require_close(std::span<const double> actual,
                   std::span<const double> expected, std::string_view what) {
    bt::require(actual.size() == expected.size(),
                "integration vector width differs from the oracle");
    for (std::size_t i = 0; i < actual.size(); ++i) {
        const double a = actual[i], e = expected[i];
        const bool close =
            std::isfinite(a) && std::isfinite(e) &&
            std::abs(a - e) <= 1e-9 + 1e-9 * std::abs(e);
        bt::require(close, what);
        if (!close) return;
    }
}

void require_bitwise(std::span<const double> actual,
                     std::span<const double> expected, std::string_view what) {
    bt::require(actual.size() == expected.size(), what);
    for (std::size_t i = 0; i < actual.size() && i < expected.size(); ++i)
        bt::require(
            std::bit_cast<std::uint64_t>(actual[i]) ==
                std::bit_cast<std::uint64_t>(expected[i]),
            what);
}

// ---- the staged decode ------------------------------------------------------

void checks_read_bootstrap(std::string_view name) {
    const fs::path directory = bt::fixture(
        (std::string("bootstrap/") + std::string(name)));
    // Box-held like the T08 snapshots — RuntimeBootstrap stages every
    // owned document member.
    const auto boot = std::make_unique<runtime::RuntimeBootstrap>(
        runtime::read_bootstrap(directory));
    const bio::Value meta = parse_doc(directory / "meta.json");

    // Model image + pins.
    bt::require(boot->model.sha256 ==
                    as_string(member(meta, "model_sha256")),
                "model image digest matches meta.json");
    bt::require(!boot->model.mjb.empty(), "model image bytes are owned");
    const bio::Value &dims = member(meta, "model_dims");
    const runtime::ModelDims &m = boot->state.model_dims;
    bt::require(m.nq == as_int(member(dims, "nq")) &&
                    m.nv == as_int(member(dims, "nv")) &&
                    m.nu == as_int(member(dims, "nu")) &&
                    m.na == as_int(member(dims, "na")) &&
                    m.nbody == as_int(member(dims, "nbody")) &&
                    m.njnt == as_int(member(dims, "njnt")) &&
                    m.nsensordata == as_int(member(dims, "nsensordata")),
                "state.model_dims decode matches meta.json");
    bt::require(
        boot->state.integration_state.size() ==
            static_cast<std::size_t>(
                as_int(member(meta, "integration_width"))),
        "integration vector width matches meta.json");
    bt::require(boot->state.step == 0 && boot->state.generation >= 1,
                "runtime scalars decode");
    // The decoded vector must be bitwise-identical to the f64 payload —
    // the $array tag resolution ran the LE element decode. The payload is
    // the ref the wire carried: integration.f64 separately pins the native
    // snapshot (the T25 golden) and may differ from the Python capture —
    // native_start_welded's leg-ctrl slots are zero by design there.
    const bio::Value state_doc = parse_doc(directory / "state.json");
    const std::string &array_ref =
        as_string(member(member(state_doc, "integration_state"), "$array"));
    require_bitwise(boot->state.integration_state,
                    bt::read_f64(bt::fixture(array_ref)),
                    "integration vector resolves bitwise");
    // The staged documents arrive owned — the welded captures all carry
    // rider_contacts as a staged Value (decoded later, against the live
    // writer topology, inside the runtime ctor).
    bt::require(!bio::is_null(boot->state.rider_contacts),
                "rider_contacts stages as an owned document");
    bt::require(boot->config.control_period_steps == 4 &&
                    boot->config.timestep_s == 0.00125 &&
                    boot->config.control_period_s == 0.005 &&
                    !boot->config.strict,
                "config envelope decodes the welded pins");
}

// ---- runtime construction + lifecycle ---------------------------------------

// By-value RuntimeBootstrap/RuntimeSnapshot returns materialize on this
// frame even boxed afterwards — the budget waiver mirrors the decode
// frames in runtime.cpp.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void checks_construct_and_advance() {
    const fs::path directory = bt::fixture("bootstrap/welded");
    // The runtime owns the model/data/writer ensemble — box-held like the
    // T08 snapshots, comfortably past the 8 KiB frame budget.
    const auto rt = std::make_unique<runtime::NativeRideRuntime>(
        runtime::read_bootstrap(directory));

    const std::vector<double> t0 =
        bt::read_f64(directory / "integration.f64");
    const auto at_zero =
        std::make_unique<const runtime::RuntimeSnapshot>(rt->snapshot());
    bt::require(at_zero->step == 0 && at_zero->generation == 1 &&
                    at_zero->time_s == 0. && !at_zero->outcome.has_value(),
                "t=0 snapshot reports the captured boundary");
    require_bitwise(at_zero->integration_state, t0,
                    "t=0 snapshot integration vector is bitwise-exact");

    // advance_python x4 under RideControl(motor=25Nm, human=10Nm).
    runtime::RideControl command;
    command.motor_torque_nm = 25.0;
    command.human_torque_nm = 10.0;
    const runtime::AdvanceResult result =
        rt->advance(4, command, 0., 0., std::nullopt);
    bt::require(result.step == 4 && result.reason == "target" &&
                    !result.outcome.has_value(),
                "advance commits to the target step");
    const std::vector<double> oracle =
        bt::read_f64(directory / "steps_control_integration.f64");
    const auto after =
        std::make_unique<const runtime::RuntimeSnapshot>(rt->snapshot());
    bt::require(after->step == 4 && after->time_s == 0.005 &&
                    after->generation == 1,
                "post-advance snapshot reports the committed boundary");
    require_close(after->integration_state, oracle,
                  "post-advance integration vector matches the oracle");
    // First-advance bytes are kept for the deterministic-replay check.
    const std::vector<double> first_run = after->integration_state;

    // reset() replays the stored bootstrap verbatim.
    rt->reset();
    const auto reset_snap =
        std::make_unique<const runtime::RuntimeSnapshot>(rt->snapshot());
    bt::require(reset_snap->step == 0 && reset_snap->generation == 2 &&
                    reset_snap->time_s == 0. &&
                    !reset_snap->outcome.has_value(),
                "reset restores the captured t=0 boundary");
    require_bitwise(reset_snap->integration_state, t0,
                    "reset restores the integration vector bitwise");
    const runtime::AdvanceResult replay =
        rt->advance(4, command, 0., 0., std::nullopt);
    bt::require(replay.step == 4 && replay.reason == "target",
                "re-advance after reset commits identically");
    require_bitwise(rt->snapshot().integration_state, first_run,
                    "reset replay reproduces the advance bitwise");

    // The captured reset bootstraps are valid inputs themselves: each
    // carries the post-reset generation intact.
    for (const auto &[name, generation]:
         {std::pair{"bootstrap/welded/reset/1", 2},
          std::pair{"bootstrap/welded/reset/2", 3}}) {
        const fs::path dir = bt::fixture(name);
        const auto captured = std::make_unique<runtime::NativeRideRuntime>(
            runtime::read_bootstrap(dir));
        const auto snap =
            std::make_unique<const runtime::RuntimeSnapshot>(
                captured->snapshot());
        bt::require(snap->step == 0 && snap->generation == generation,
                    "reset capture carries its generation");
        require_bitwise(snap->integration_state,
                        bt::read_f64(dir / "integration.f64"),
                        "reset capture restores its vector bitwise");
        captured->close();
        bt::require(captured->closed(), "close() seals the runtime");
    }
}
NATIVE_DIAG_POP

// ---- rejection contract ------------------------------------------------------

// Composes one bootstrap from the fault directory's documents where
// present and the welded_pinned documents elsewhere; the model artifact
// is always the welded capture's (the digest fault lives in state.json,
// not the bytes).
[[nodiscard]] runtime::RuntimeBootstrap
compose_fault(const fs::path &fault_dir, const fs::path &good_dir) {
    const auto doc_for = [&](const char *file) {
        const fs::path fault_doc = fault_dir / file;
        if (fs::exists(fault_doc))
            return runtime::resolve_bootstrap_document(
                parse_doc(fault_doc), fault_dir);
        return runtime::resolve_bootstrap_document(
            parse_doc(good_dir / file), good_dir);
    };
    // The config envelope decode is the caller's position-1 half of the
    // shared composition order — same split read_bootstrap() runs.
    runtime::RuntimeConfig config =
        runtime::decode_runtime_config(doc_for("config.json"));
    return runtime::compose_bootstrap(std::move(config),
                                      model_image(good_dir),
                                      doc_for("state.json"));
}

// Same by-value bootstrap temporaries — budget waived like above.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void checks_fault_matrix() {
    const fs::path good_dir = bt::fixture("bootstrap/welded_pinned");
    for (const char *name :
         {"control_period", "digest", "dims", "missing_key", "nonfinite",
          "schema", "unknown_key", "width"}) {
        const fs::path dir = bt::fixture(
            std::string("bootstrap/faults/") + name);
        const bio::Value expected = parse_doc(dir / "expected.json");
        const std::string &message =
            as_string(member(member(expected, "expected"), "message"));
        try {
            const auto rejected = std::make_unique<runtime::RuntimeBootstrap>(
                compose_fault(dir, good_dir));
            static_cast<void>(rejected);
            bt::require(false, "fault bootstrap must reject");
        } catch (const std::invalid_argument &failure) {
            bt::require(message == failure.what(),
                        "fault diagnostic must match the captured oracle");
        }
    }
    // state.drive.front_teeth removed: the oracle recorded acceptance.
    const fs::path tolerated =
        bt::fixture("bootstrap/faults/malformed_drive");
    const auto accepted = std::make_unique<runtime::RuntimeBootstrap>(
        compose_fault(tolerated, good_dir));
    bt::require(accepted->state.step == 0 && !accepted->model.mjb.empty(),
                "the unexpected-acceptance mutation composes");
}
NATIVE_DIAG_POP

} // namespace

namespace {

// One bt::run per check function — a single lambda's frame aggregates the
// inlined check bodies and trips the 8 KiB frame guard on the release
// build (same shape as the sibling migration mains).
void checks_welded() { checks_read_bootstrap("welded_pinned"); }
void checks_native_start() {
    checks_read_bootstrap("native_start_welded");
}

} // namespace

int main() {
    try {
        int failures = 0;
        failures += bt::run(checks_welded);
        failures += bt::run(checks_native_start);
        failures += bt::run(checks_construct_and_advance);
        failures += bt::run(checks_fault_matrix);
        return failures == 0 ? 0 : 1;
    } catch (...) {
        return 1;
    }
}
