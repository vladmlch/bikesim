// test_t07.cpp — typed writer-config boundary regressions (task T07):
// nativecfg::WriterConfig + nativecfg::decode over the owned
// bike::io::Value tree, and the typed Stepper(mjb, WriterConfig)
// construction path — exercised in the Python-free graph (this target's
// compile context carries no Python/nanobind include, so compiling the
// public headers IS the include-closure check). Failures must be
// std::invalid_argument naming the public path; construction must leave
// no partially published state — the alloc-fault sweep enumerates every
// throwing position.
#include "support/fixture_test.hpp"
#include "writer_config.hpp"
#include "stepper.hpp"
#include "io/json.hpp"
#include "allocation_faults.hpp"
#include "diag.hpp"

#include <functional>
#include <limits>
#include <new>
#include <optional>
#include <string>
#include <string_view>

namespace {

const std::string &model_path() {
    static const std::string path =
        bt::fixture("models/minimal.mjb").string();
    return path;
}

bike::io::Value parse(std::string_view text) {
    return bike::io::parse_json(text);
}

// One complete suspension wire section (physical values are inert for
// decode — the section/key/type layer is what T07 owns). `fork_legacy`
// controls the fork_damper field: std::nullopt omits it (a schema-2
// missing-key probe or a schema-1 document), a value emits the field;
// `hbo_start` drives the schema-1 resolved-metadata branch.
std::string suspension_doc(int schema, std::optional<bool> fork_legacy,
                           double hbo_start,
                           std::string_view mode = "physical",
                           bool shock_legacy = false,
                           bool coil_legacy = false) {
    std::string doc = R"({"schema":)" + std::to_string(schema) + R"(,"suspension":{
        "physics_mode":")" + std::string(mode) + R"(",
        "joints":{"fork":"fork_joint","shock":"shock_joint"},
        "air_spring":{"stanchion_inner_diam_mm":32.0,"total_travel_mm":160.0,
            "pos_chamber_length_mm":40.0,"neg_chamber_length_mm":160.0,
            "token_volume_cm3":10.0,"max_tokens":4,"gamma":1.4,
            "atm_pressure_pa":101325.0,"num_tokens":1,"gauge_pressure_psi":80.0},
        "fork_damper":{"max_hsc":8,"max_lsc":8,"max_reb":8,"hsc_clicks":4,
            "lsc_clicks":4,"rebound_clicks":4,"c_lsc_min":10.0,"c_lsc_max":100.0,
            "c_hsc_min":50.0,"c_hsc_max":200.0,"c_reb_min":20.0,"c_reb_max":150.0,
            "v_knee_comp":0.1,"v_knee_reb":0.2,"total_travel_mm":160.0,
            "hbo_start_mm":)" + std::to_string(hbo_start) + R"(,"c_hbo_base":300.0)";
    if (fork_legacy)
        doc += std::string(R"(,"legacy_behavior":)") +
               (*fork_legacy ? "true" : "false");
    doc += std::string(R"(},
        "shock_damper":{"max_hsc":8,"max_lsc":8,"max_reb":8,"hsc_clicks":4,
            "lsc_clicks":4,"rebound_clicks":4,"c_lsc_min":10.0,"c_lsc_max":100.0,
            "c_hsc_min":50.0,"c_hsc_max":200.0,"c_reb_min":20.0,"c_reb_max":150.0,
            "v_knee_comp":0.1,"v_knee_reb":0.2,"total_stroke_mm":60.0,
            "max_hbo":4,"hbo_clicks":2,"lockout_firm":false,"legacy_behavior":)") +
           (shock_legacy ? "true" : "false") +
           std::string(R"(,
            "hbo_start_mm":40.0,"c_hbo_min":200.0,"c_hbo_max":800.0,
            "lockout_preload_n":10.0,"lockout_stiffness":50000.0},
        "coil":{"rate_n_m":80000.0,"preload_mm":2.0,"stroke_mm":60.0,
            "bumper_length_mm":8.0,"bumper_peak_n":2000.0,"legacy_behavior":)") +
           (coil_legacy ? "true" : "false") +
           R"(},
        "end_stops":{"stiffness_n_m":100000.0,"damping_n_s_m":50.0}}})";
    return doc;
}

// Test bodies hold WriterConfig temporaries + capture lambdas in one frame
// — the same cold-path exemption the binding glue frames carry.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void checks_decode() {
    // The plan's regression: a schema-only config is a valid empty writer
    // set; an unknown top-level member rejects with std::invalid_argument.
    {
        const nativecfg::WriterConfig cfg =
            nativecfg::decode(parse(R"({"schema":2})"));
        bt::require(!cfg.drive && !cfg.cruise && !cfg.suspension &&
                        !cfg.brake && !cfg.resistance && !cfg.tire &&
                        !cfg.rider_contacts,
                    "schema-only config must decode to no writers");
    }
    bt::invalid([] {
        (void) nativecfg::decode(parse(R"({"schema":2,"unexpected":0})"));
    });
    // Empty object — the same "no writers" outcome the nb adapter's
    // empty-dict gate produces at the binding edge.
    {
        const nativecfg::WriterConfig cfg = nativecfg::decode(parse("{}"));
        bt::require(!cfg.drive && !cfg.cruise && !cfg.suspension &&
                        !cfg.brake && !cfg.resistance && !cfg.tire &&
                        !cfg.rider_contacts,
                    "empty object must decode to no writers");
    }
    // Root kind + unknown section/key rejections and the strict scalar
    // mapping: a float-spelled schema is not an integer; a bool member is
    // not a real.
    bt::invalid([] { (void) nativecfg::decode(parse(R"([1,2])")); });
    bt::invalid([] { (void) nativecfg::decode(parse(R"("config")")); });
    bt::invalid([] { (void) nativecfg::decode(parse(R"({"schema":3})")); });
    bt::invalid([] { (void) nativecfg::decode(parse(R"({"schema":2.0})")); });
    bt::invalid([] { (void) nativecfg::decode(parse(R"({"schema":true})")); });
    bt::invalid([] { (void) nativecfg::decode(parse(R"({"schema":1.9e308})")); });
    bt::invalid([] {
        (void) nativecfg::decode(parse(R"({"schema":2,"bogus":{}})"));
    });
    bt::invalid([] {
        (void) nativecfg::decode(
            parse(R"({"schema":2,"cruise":{"target_speed_kmh":20.0}})"));
    });
    bt::invalid([] {
        (void) nativecfg::decode(parse(
            R"({"schema":2,"rider_contacts":{"arm_reach_m":true}})"));
    });
    // Ordered-sequence gate: a mapping is not a sequence, a sequence of
    // the wrong width rejects.
    bt::invalid([] {
        (void) nativecfg::decode(parse(
            R"({"schema":2,"resistance":{"crr":0.004,"rolling_taper_rad_s":1.0,
                "rho_kg_m3":1.2,"cda_m2":0.3,"wind_world_mps":{},
                "point_body_m":[0,0,0],"bodies":{"frame":"f","front_wheel":"w","rear_wheel":"w"}}})"));
    });
    bt::invalid([] {
        (void) nativecfg::decode(parse(
            R"({"schema":2,"resistance":{"crr":0.004,"rolling_taper_rad_s":1.0,
                "rho_kg_m3":1.2,"cda_m2":0.3,"wind_world_mps":[0,0],
                "point_body_m":[0,0,0],"bodies":{"frame":"f","front_wheel":"w","rear_wheel":"w"}}})"));
    });

    // Schema-1 resolved-fork metadata still migrates without
    // reconstructing defaults (hbo_start_mm != 160 clears the legacy
    // branch), but the legacy damper curve itself was removed with
    // legacy mode (2026-11-04): both its schema-1 spelling
    // (hbo_start_mm == 160) and the schema-2 "legacy_behavior":true are
    // ingress rejections now, never silently modernized.
    bt::invalid([] {
        (void) nativecfg::decode(
            parse(suspension_doc(1, /*fork_legacy=*/std::nullopt, 160.0)));
    });
    bt::invalid([] {
        (void) nativecfg::decode(
            parse(suspension_doc(2, /*fork_legacy=*/true, 160.0)));
    });
    {
        const auto nonlegacy = nativecfg::decode(
            parse(suspension_doc(1, /*fork_legacy=*/std::nullopt, 161.0)));
        bt::require(nonlegacy.suspension &&
                        !nonlegacy.suspension->fork_damper.legacy_behavior,
                    "hbo_start_mm != 160 must clear the legacy branch");
    }
    // The schema-2 key is still rejected under schema 1, and schema 2
    // without the resolved key is a missing-required rejection.
    bt::invalid([] {
        (void) nativecfg::decode(
            parse(suspension_doc(1, /*fork_legacy=*/true, 160.0)));
    });
    bt::invalid([] {
        (void) nativecfg::decode(
            parse(suspension_doc(2, /*fork_legacy=*/std::nullopt, 160.0)));
    });
}

// The legacy damper curve was removed with legacy mode (2026-11-04): a wire
// config that asks for it is rejected at ingress, never silently modernized.
void legacy_damper_curve_is_rejected() {
    // fork_damper, shock_damper and coil each reject their own explicit
    // "legacy_behavior":true — the key stays required on the wire so
    // physical configs keep their exact bytes.
    bt::invalid([] {
        (void) nativecfg::decode(
            parse(suspension_doc(2, /*fork_legacy=*/true, 130.0)));
    });
    bt::invalid([] {
        (void) nativecfg::decode(
            parse(suspension_doc(2, /*fork_legacy=*/false, 130.0,
                                 "physical", /*shock_legacy=*/true)));
    });
    bt::invalid([] {
        (void) nativecfg::decode(
            parse(suspension_doc(2, /*fork_legacy=*/false, 130.0,
                                 "physical", /*shock_legacy=*/false,
                                 /*coil_legacy=*/true)));
    });
    // The schema-1 spelling of the same request — fork hbo_start_mm == 160 —
    // resolves to the legacy branch and rejects identically.
    bt::invalid([] {
        (void) nativecfg::decode(
            parse(suspension_doc(1, /*fork_legacy=*/std::nullopt, 160.0)));
    });
    // physics_mode "legacy" is not an accepted mode any more.
    bt::invalid([] {
        (void) nativecfg::decode(
            parse(suspension_doc(2, /*fork_legacy=*/false, 130.0, "legacy")));
    });
}

void checks_construction() {
    // Bare-schema config constructs a working Stepper with no writers.
    {
        const nativecfg::WriterConfig cfg =
            nativecfg::decode(parse(R"({"schema":2})"));
        Stepper stepper(model_path(), cfg);
        bt::require(!stepper.qpos().empty(),
                    "typed construction must load the model");
        bt::require(!stepper.has_tire() && !stepper.has_drive() &&
                        !stepper.has_suspension() && !stepper.has_cruise() &&
                        !stepper.has_resistance() &&
                        !stepper.has_rider_contacts(),
                    "empty writer config must install no writers");
        stepper.forward();
        bt::require(!stepper.qpos().empty(),
                    "forward() must run on the empty-config Stepper");
    }
    // An all-absent WriterConfig is the same empty-writers contract.
    {
        const Stepper stepper(model_path(), nativecfg::WriterConfig{});
        bt::require(!stepper.qpos().empty(),
                    "default WriterConfig must construct");
    }
    // An invalid decode leaves no half-constructed Stepper — the throw
    // happens before the constructor runs.
    bt::invalid([] {
        const Stepper stepper(
            model_path(),
            nativecfg::decode(parse(R"({"schema":2,"bogus":{}})")));
        (void) stepper;
    });
    // A section that decodes but fails the writer's model-side validation
    // (the suspension joints do not exist on minimal.mjb) must fail the
    // typed constructor atomically — std::invalid_argument, and no
    // partially published Stepper exists.
    {
        const nativecfg::WriterConfig cfg =
            nativecfg::decode(
                parse(suspension_doc(2, /*fork_legacy=*/false, 160.0)));
        bt::invalid([&] { const Stepper stepper(model_path(), cfg); });
    }
}

void checks_allocation_failures() {
    // decode() — pure function: enumerate every throwing position; each
    // must throw bad_alloc, and an unarmed retry must produce the
    // identical typed result. The measurement scope closes before the
    // enumeration loop so require() diagnostics stay unarmed.
    const std::string doc =
        suspension_doc(2, /*fork_legacy=*/false, 160.0);
    const bike::io::Value value = parse(doc);
    std::size_t decode_positions = 0;
    bool control_legacy = false;
    {
        const allocation_faults::Guard guard;
        allocation_faults::arm(std::numeric_limits<std::size_t>::max());
        const nativecfg::WriterConfig control = nativecfg::decode(value);
        decode_positions = allocation_faults::allocated();
        bt::require(control.suspension.has_value(),
                    "armed control decode must succeed");
        control_legacy = control.suspension &&
                         control.suspension->fork_damper.legacy_behavior;
    }
    bt::require(decode_positions > 0,
                "the decode sweep must enumerate real allocations");
    for (std::size_t fail_at = 0; fail_at < decode_positions; ++fail_at) {
        bool threw = false;
        {
            const allocation_faults::Guard inner;
            allocation_faults::arm(fail_at);
            try {
                const nativecfg::WriterConfig attempt =
                    nativecfg::decode(value);
                bt::require(
                    attempt.suspension &&
                        attempt.suspension->fork_damper.legacy_behavior ==
                            control_legacy,
                    "a survived decode position must match the control");
            } catch (const std::bad_alloc &) {
                threw = true;
            }
        }
        bt::require(threw,
                    "an armed decode allocation position must throw");
    }

    // Stepper(path, WriterConfig) — minimal.mjb admits no writer sections,
    // so the bare ctor path is swept: every throwing position must fail
    // construction and the unarmed retry must produce a working owner.
    std::size_t ctor_positions = 0;
    {
        const allocation_faults::Guard guard;
        allocation_faults::arm(std::numeric_limits<std::size_t>::max());
        const Stepper control(model_path(), nativecfg::WriterConfig{});
        ctor_positions = allocation_faults::allocated();
        bt::require(!control.qpos().empty(),
                    "armed control construction must succeed");
    }
    for (std::size_t fail_at = 0; fail_at < ctor_positions; ++fail_at) {
        bool threw = false;
        {
            const allocation_faults::Guard inner;
            allocation_faults::arm(fail_at);
            try {
                const Stepper attempt(model_path(),
                                      nativecfg::WriterConfig{});
                bt::require(!attempt.qpos().empty(),
                            "a survived construction position must "
                            "produce a working owner");
            } catch (const std::bad_alloc &) {
                threw = true;
            }
        }
        bt::require(threw,
                    "an armed construction position must throw");
    }
    // The unarmed retry after the sweep sees the successful state.
    const Stepper retry(model_path(), nativecfg::WriterConfig{});
    bt::require(!retry.qpos().empty(),
                "construction after the sweep must produce a working owner");
}
NATIVE_DIAG_POP

} // namespace

int main() {
    try {
        int failures = 0;
        failures += bt::run(checks_decode);
        failures += bt::run(legacy_damper_curve_is_rejected);
        failures += bt::run(checks_construction);
        failures += bt::run(checks_allocation_failures);
        return failures == 0 ? 0 : 1;
    } catch (...) {
        return 1;
    }
}
