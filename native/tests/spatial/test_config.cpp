#include "configuration/load.hpp"
#include "configuration/model_config.hpp"
#include "io/json.hpp"
#include "spatial/config.hpp"
#include "support/fixture_test.hpp"

#include <filesystem>
#include <string>

int main() {
    return bt::run([] {
        using namespace bike::spatial;
        using bike::io::Value;

        constexpr double dt = 0.00125;

        // 1. Planar table without "spatial" returns nullopt and keeps 2D schema/TOML/JSON untouched.
        const Value::Object empty_root{};
        bt::require(!parse_spatial_config(empty_root, dt).has_value(),
                    "absent spatial -> planar nullopt");

        const std::string schema_json = bike::io::canonical_json(bike::configuration::schema());
        bt::require(schema_json.find("spatial") == std::string::npos,
                    "planar SimulationPhysicsConfig schema contains zero spatial keys");

        const std::filesystem::path repo_root =
            bt::fixture("config").parent_path().parent_path().parent_path();
        const auto planar_cfg =
            bike::configuration::load_physics(repo_root / "examples/research/viewer_physics_welded.toml");
        const Value planar_val = bike::configuration::to_value(planar_cfg);
        const std::string planar_toml = bike::configuration::write_physics_toml(planar_cfg);
        bt::require(bike::io::find_member(bike::io::as_object(planar_val, "planar"), "spatial") ==
                            nullptr &&
                        planar_toml.find("[spatial") == std::string::npos,
                    "planar SimulationPhysicsConfig serialization contains zero spatial sections");

        // 2. Valid moving-start SpatialConfig roundtrips cleanly through canonical JSON.
        const Value valid_doc = bike::io::parse_json(R"({
            "spatial": {
                "tire_backend": "compliant_3d",
                "tire_preset": "2p8_1p2bar",
                "start_mode": "moving",
                "corridor_half_width_m": 1.5,
                "capsize_roll_rad": 1.0471975511965976,
                "steer": {
                    "range_deg": 45.0,
                    "control_period_s": 0.005,
                    "reaction_delay_s": 0.100,
                    "max_torque_nm": 35.0
                }
            }
        })");
        const auto parsed =
            parse_spatial_config(bike::io::as_object(valid_doc, "root"), dt);
        bt::require(parsed.has_value(), "valid spatial config parsed");
        bt::require(parsed->tire_backend == SpatialTireBackend::compliant_3d, "backend decoded");
        bt::require(parsed->tire_preset == SpatialTirePreset::mtb_2p8_1p2bar, "2.8 preset decoded");
        bt::require(require_exact_ticks(parsed->steer.control_period_s, dt, 1,
                                        "steer.control_period_s") == 4,
                    "5 ms is exact 4 ticks of 1.25 ms");
        bt::require(require_exact_ticks(parsed->steer.reaction_delay_s,
                                        parsed->steer.control_period_s, 0,
                                        "steer.reaction_delay_s") == 20,
                    "100 ms is exact 20 control ticks");

        const Value serialized = spatial_config_to_value(*parsed);
        const std::string serialized_json = bike::io::canonical_json(serialized);
        const Value reparsed_doc =
            bike::io::parse_json("{\"spatial\":" + serialized_json + "}");
        const auto reparsed =
            parse_spatial_config(bike::io::as_object(reparsed_doc, "root"), dt);
        bt::require(reparsed.has_value(), "serialized spatial config reparses");
        bt::require(serialized_json == bike::io::canonical_json(spatial_config_to_value(*reparsed)),
                    "canonical JSON roundtrip is bitwise deterministic");

        // 3. Valid standstill mode with supercritical start_support (roll >= 1500 N m/rad).
        const Value standstill_doc = bike::io::parse_json(R"({
            "spatial": {
                "tire_backend": "distributed_3d",
                "tire_preset": "2p8_1p2bar",
                "start_mode": "standstill",
                "corridor_half_width_m": 1.5,
                "capsize_roll_rad": 1.0,
                "steer": {
                    "range_deg": 45.0,
                    "control_period_s": 0.005,
                    "reaction_delay_s": 0.100,
                    "max_torque_nm": 35.0
                },
                "start_support": {
                    "release_speed_mps": 1.5,
                    "lateral_stiffness_n_per_m": 2500.0,
                    "lateral_damping_ns_per_m": 450.0,
                    "roll_stiffness_nm_per_rad": 2400.0,
                    "roll_damping_nms_per_rad": 420.0
                }
            }
        })");
        const auto parsed_standstill =
            parse_spatial_config(bike::io::as_object(standstill_doc, "root"), dt);
        bt::require(parsed_standstill.has_value() &&
                        parsed_standstill->start_mode == SpatialStartMode::standstill &&
                        parsed_standstill->start_support.has_value(),
                    "valid standstill config parsed");

        // 4. Decision 12: non-multiple delay fails closed (no silent std::llround!).
        const Value bad_delay_doc = bike::io::parse_json(R"({
            "spatial": {
                "tire_backend": "compliant_3d",
                "tire_preset": "2p8_1p2bar",
                "start_mode": "moving",
                "corridor_half_width_m": 1.5,
                "capsize_roll_rad": 1.0,
                "steer": {
                    "range_deg": 45.0,
                    "control_period_s": 0.005,
                    "reaction_delay_s": 0.103,
                    "max_torque_nm": 35.0
                }
            }
        })");
        bt::invalid([&] {
            (void)parse_spatial_config(bike::io::as_object(bad_delay_doc, "root"), dt);
        });

        // 5. Standstill without start_support, sub-critical roll stiffness (< 1500 N m/rad),
        //    and banned passive steer damping key all fail closed!
        const Value missing_support_doc = bike::io::parse_json(R"({
            "spatial": {
                "tire_backend": "compliant_3d",
                "tire_preset": "2p8_1p2bar",
                "start_mode": "standstill",
                "corridor_half_width_m": 1.5,
                "capsize_roll_rad": 1.0,
                "steer": {
                    "range_deg": 45.0,
                    "control_period_s": 0.005,
                    "reaction_delay_s": 0.100,
                    "max_torque_nm": 35.0
                }
            }
        })");
        bt::invalid([&] {
            (void)parse_spatial_config(bike::io::as_object(missing_support_doc, "root"), dt);
        });

        const Value subcritical_hold_doc = bike::io::parse_json(R"({
            "spatial": {
                "tire_backend": "compliant_3d",
                "tire_preset": "2p8_1p2bar",
                "start_mode": "standstill",
                "corridor_half_width_m": 1.5,
                "capsize_roll_rad": 1.0,
                "steer": {
                    "range_deg": 45.0,
                    "control_period_s": 0.005,
                    "reaction_delay_s": 0.100,
                    "max_torque_nm": 35.0
                },
                "start_support": {
                    "release_speed_mps": 1.5,
                    "lateral_stiffness_n_per_m": 2000.0,
                    "lateral_damping_ns_per_m": 300.0,
                    "roll_stiffness_nm_per_rad": 400.0,
                    "roll_damping_nms_per_rad": 80.0
                }
            }
        })");
        bt::invalid([&] {
            (void)parse_spatial_config(bike::io::as_object(subcritical_hold_doc, "root"), dt);
        });

        const Value banned_damping_doc = bike::io::parse_json(R"({
            "spatial": {
                "tire_backend": "compliant_3d",
                "tire_preset": "2p8_1p2bar",
                "start_mode": "moving",
                "corridor_half_width_m": 1.5,
                "capsize_roll_rad": 1.0,
                "damping_nms_rad": 0.5,
                "steer": {
                    "range_deg": 45.0,
                    "control_period_s": 0.005,
                    "reaction_delay_s": 0.100,
                    "max_torque_nm": 35.0
                }
            }
        })");
        bt::invalid([&] {
            (void)parse_spatial_config(bike::io::as_object(banned_damping_doc, "root"), dt);
        });
    });
}
