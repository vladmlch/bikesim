// test_rut_memory.cpp — direct C++ contracts for biketyre::RutMemory
// (tyre/rut_memory.cpp), the Stage B 1D deformable granular layer:
// finite-depth pressure law, plastic return mapping, multipass rut
// hardening, basal-slip traction ceiling, sparse state I/O and the
// layer_thickness_m validation band. Standalone like test_station.cpp.
#include "../src/tyre/rut_memory.hpp"

#include "../src/config_types.hpp"
#include "../src/config_validation.hpp"
#include "../src/pneumatic/geometry.hpp"
#include "../src/pneumatic/profile.hpp"

#include <array>
#include <cmath>
#include <exception>
#include <iostream>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {
    using TestFunction = void (*)();

    struct TestCase {
        std::string_view name;
        TestFunction run;
    };

    void require(bool condition, std::string_view message) {
        if (!condition)
            throw std::runtime_error(std::string(message));
    }

    template <class F>
    void require_throws(const F &f, std::string_view message) {
        try {
            f();
        } catch (const std::exception &) {
            return;
        }
        require(false, message);
    }

    // A flat 40 m road on a 5 mm grid — 8001 nodes.
    biketyre::RoadProfile make_road() {
        std::vector<double> z(8001, 0.0);
        return {0.0, 0.005, std::move(z)};
    }

    nativecfg::SurfaceSpec spec(std::string_view name) {
        nativecfg::SurfaceSpec s;
        s.name = std::string(name);
        s.mu_peak = 0.8;
        s.mu_slide = 0.6;
        s.slip_stiffness_per_load = 12.0;
        s.stribeck_speed_mps = 4.5;
        return s;
    }

    // Uniform hardpack default + one loose section [10, 20) with a 5 cm
    // deformable layer.
    nativecfg::SurfaceMap make_map() {
        nativecfg::SurfaceMap map;
        map.surface = spec("hardpack");
        nativecfg::SurfaceSection sec;
        sec.start_m = 10.0;
        sec.end_m = 20.0;
        sec.surface = spec("loose");
        sec.layer_thickness_m = 0.05;
        map.sections.push_back(sec);
        return map;
    }

    // The stock 29x2.4 front tyre (physical_config.hpp kFrontTyre) so
    // ray_elastic_force_n resolves a realistic element law.
    nativecfg::PneumaticTyreSpec make_tyre() {
        nativecfg::PneumaticTyreSpec t;
        t.name = "Schwalbe Magic Mary 29x2.4";
        t.outer_radius_mm = 372.0;
        t.rim_radius_mm = 320.0;
        t.section_height_mm = 57.0;
        t.pressure_bar = 1.5;
        t.crr_reference = 0.011;
        t.casing_width_mm = 60.0;
        t.compressed_casing_mm = 6.0;
        t.area_factor = 0.4435;
        t.carcass_stiffness_n_mm2 = 0.1194;
        t.contact_length_factor = 0.8589;
        t.rate_stiffening = 0.2484;
        t.rate_relaxation_s = 0.2;
        t.loss_factor = 0.0662;
        t.hysteresis_rate_eps_mps = 0.01;
        t.relaxation_length_mm = 60.0;
        t.rim_stiffness_n_mm2 = 28.0;
        t.tread_loss_crr = 0.00586;
        return t;
    }

    // One wheel pass worth of element rows: n rays, every contacting
    // ray parked at road_x, carrying f_n of normal force.
    struct Pass {
        std::vector<double> r, delta, rx, rz;
        biketyre::RayHitBuffers hits() {
            return {.r_m = r,
                    .delta_m = delta,
                    .road_x_m = rx,
                    .road_z_m = rz};
        }
        std::vector<double> force;
    };

    constexpr std::size_t kRays = 36;

    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (x, force, deflection) mirror the pass fixture's physical inputs
    Pass pass_at(double x_m, double f_n, double delta_m = 0.004) {
        Pass p;
        p.r.assign(kRays, 0.34);
        p.delta.assign(kRays, 0.0);
        p.rx.assign(kRays,
                    std::numeric_limits<double>::quiet_NaN());
        p.rz.assign(kRays, 0.0);
        p.force.assign(kRays, 0.0);
        for (std::size_t i = 16; i < 24; ++i) {
            p.delta[i] = delta_m;
            p.rx[i] = x_m;
            p.force[i] = f_n;
        }
        return p;
    }

    void layer_field_resolves() {
        const biketyre::RoadProfile road = make_road();
        const biketyre::RutMemory rut(road, make_map());
        require(rut.active(), "deformable section must activate");
        const auto h = rut.h_s_m();
        require(h[2000] > 0.049 && h[2000] < 0.051,
                "in-section node must resolve the authored thickness");
        require(h[1000] == 0.0 && h[6000] == 0.0,
                "off-layer nodes must stay rigid");
    }

    void virgin_deposit_and_hardening() {
        const biketyre::RoadProfile road = make_road();
        biketyre::RutMemory rut(road, make_map());
        const nativecfg::PneumaticTyreSpec tyre = make_tyre();
        const biketyre::RayRing ring(36, 1.0);
        Pass p = pass_at(12.0, 40.0);
        const auto hits = p.hits();
        rut.begin_step();
        rut.commit_pass(ring, tyre, p.force, hits.road_x_m);
        rut.finish_step();
        const std::size_t g =
                static_cast<std::size_t>(12.0 / 0.005);
        require(rut.z_max_m()[g] > 0.0, "deposit must sink the node");
        require(rut.z_p_m()[g] > 0.0,
                "deposit must leave a plastic rut");
        require(rut.soil_elastic_energy_j() > 0.0,
                "loaded soil must hold elastic energy");
        require(rut.step_plastic_loss_j() >= 0.0,
                "plastic loss must be nonnegative");
        // Multipass hardening: the same force on the preconsolidated
        // node must not deepen the plastic rut again.
        const double zp_first = rut.z_p_m()[g];
        rut.begin_step();
        rut.commit_pass(ring, tyre, p.force, hits.road_x_m);
        rut.finish_step();
        require(rut.z_p_m()[g] == zp_first,
                "same-load repeat must ride the compacted rut");
        // A heavier pass exceeds the hardened yield surface and
        // plastically deepens.
        Pass heavy = pass_at(12.0, 80.0);
        rut.begin_step();
        rut.commit_pass(ring, tyre, heavy.force, hits.road_x_m);
        rut.finish_step();
        require(rut.z_p_m()[g] > zp_first,
                "overload pass must deepen the plastic rut");
    }

    void elastic_release_counts_loss() {
        const biketyre::RoadProfile road = make_road();
        biketyre::RutMemory rut(road, make_map());
        const nativecfg::PneumaticTyreSpec tyre = make_tyre();
        const biketyre::RayRing ring(36, 1.0);
        Pass p = pass_at(12.0, 40.0);
        const auto hits = p.hits();
        rut.begin_step();
        rut.commit_pass(ring, tyre, p.force, hits.road_x_m);
        rut.finish_step();
        const double energy_loaded = rut.soil_elastic_energy_j();
        require(energy_loaded > 0.0, "loaded pass stores soil energy");
        // A step with no coverage releases the elastic depth — the
        // drop lands in the step loss, never negative.
        std::vector<double> zeros(kRays, 0.0);
        rut.begin_step();
        rut.commit_pass(ring, tyre, zeros, hits.road_x_m);
        rut.finish_step();
        require(rut.soil_elastic_energy_j() == 0.0,
                "uncovered release must drain elastic energy");
        require(rut.step_plastic_loss_j() >= energy_loaded - 1e-15,
                "released elastic depth must surface as loss");
        const std::size_t g =
                static_cast<std::size_t>(12.0 / 0.005);
        require(rut.z_e_m()[g] == 0.0 && rut.z_p_m()[g] > 0.0,
                "release drains z_e while the rut persists");
    }

    void probe_solve_is_pure() {
        const biketyre::RoadProfile road = make_road();
        biketyre::RutMemory rut(road, make_map());
        const nativecfg::PneumaticTyreSpec tyre = make_tyre();
        const biketyre::RayRing ring(36, 1.0);
        Pass p = pass_at(12.0, 40.0);
        const auto hits = p.hits();
        rut.begin_step();
        rut.commit_pass(ring, tyre, p.force, hits.road_x_m);
        rut.finish_step();
        const std::vector<double> zm_before(rut.z_max_m().begin(),
                                            rut.z_max_m().end());
        const std::vector<double> zp_before(rut.z_p_m().begin(),
                                            rut.z_p_m().end());
        const std::vector<double> ze_before(rut.z_e_m().begin(),
                                            rut.z_e_m().end());
        const double e_before = rut.soil_elastic_energy_j();
        // The probe path: const solve, no commit — semantic state must
        // stay bitwise.
        auto probe = p.hits();
        rut.solve_deflections(
                ring, tyre,
                tyre.outer_radius_mm * 0.001 * ring.dtheta_rad(),
                probe);
        require(rut.soil_elastic_energy_j() == e_before,
                "probe must not touch the energy accumulator");
        require(std::equal(zm_before.begin(), zm_before.end(),
                           rut.z_max_m().begin()) &&
                std::equal(zp_before.begin(), zp_before.end(),
                           rut.z_p_m().begin()) &&
                std::equal(ze_before.begin(), ze_before.end(),
                           rut.z_e_m().begin()),
                "probe must leave every committed field bitwise");
        // And the solve must actually soften the element deflection
        // on deformable ground.
        bool softened = false;
        for (std::size_t i = 16; i < 24; ++i)
            softened |= probe.delta_m[i] < 0.004;
        require(softened,
                "deformable ground must reduce element deflection");
    }

    void multipass_front_rear() {
        const biketyre::RoadProfile road = make_road();
        biketyre::RutMemory rut(road, make_map());
        const nativecfg::PneumaticTyreSpec tyre = make_tyre();
        const biketyre::RayRing ring(36, 1.0);
        Pass front = pass_at(12.0, 45.0);
        Pass rear = pass_at(12.0, 30.0);
        const auto hits = front.hits();
        rut.begin_step();
        rut.commit_pass(ring, tyre, front.force, hits.road_x_m);
        const std::size_t g =
                static_cast<std::size_t>(12.0 / 0.005);
        const double zp_after_front = rut.z_p_m()[g];
        rut.commit_pass(ring, tyre, rear.force, hits.road_x_m);
        rut.finish_step();
        require(rut.z_p_m()[g] == zp_after_front,
                "lighter rear must ride the front's rut untouched");
    }

    void basal_slip_cap() {
        const biketyre::RoadProfile road = make_road();
        const biketyre::RutMemory rut(road, make_map());
        const double cap_on =
                rut.traction_mu_cap(12.0, 0.002, 300.0);
        require(std::isfinite(cap_on) && cap_on > 0.0,
                "on-layer traction cap must be finite");
        const double cap_off =
                rut.traction_mu_cap(5.0, 0.002, 300.0);
        require(!std::isfinite(cap_off),
                "off-layer traction must be uncapped");
    }

    void snapshot_restore_roundtrip() {
        const biketyre::RoadProfile road = make_road();
        biketyre::RutMemory rut(road, make_map());
        const nativecfg::PneumaticTyreSpec tyre = make_tyre();
        const biketyre::RayRing ring(36, 1.0);
        Pass p = pass_at(12.0, 40.0);
        const auto hits = p.hits();
        rut.begin_step();
        rut.commit_pass(ring, tyre, p.force, hits.road_x_m);
        rut.finish_step();
        const biketyre::RutStateDiff diff = rut.snapshot();
        require(!diff.spans.empty(), "loaded rut must export spans");
        // Fresh memory restored from the diff must reproduce every
        // committed field.
        biketyre::RutMemory twin(road, make_map());
        twin.restore(diff, 0.5 * tyre.casing_width_mm * 0.001);
        require(std::equal(rut.z_max_m().begin(), rut.z_max_m().end(),
                           twin.z_max_m().begin()) &&
                std::equal(rut.z_p_m().begin(), rut.z_p_m().end(),
                           twin.z_p_m().begin()) &&
                std::equal(rut.z_e_m().begin(), rut.z_e_m().end(),
                           twin.z_e_m().begin()),
                "restore must reproduce the committed fields");
        require(twin.soil_elastic_energy_j() > 0.0,
                "restored elastic depth must re-derive its energy");
        rut.reset();
        require(rut.soil_elastic_energy_j() == 0.0 &&
                rut.step_plastic_loss_j() == 0.0 &&
                rut.z_p_m()[static_cast<std::size_t>(12.0 / 0.005)] ==
                        0.0,
                "reset must clear state and accumulators");
    }

    void validation_band() {
        // Native wire gate: nonzero thickness only on loose/scree,
        // only inside [0.012, 0.150].
        nativecfg::SurfaceSpec s = spec("loose");
        s.layer_thickness_m = 0.05;
        nativecfg::validate(s, "config.tire.pneumatic.surface");
        s.layer_thickness_m = 0.011;
        require_throws([&] { nativecfg::validate(s); },
                       "thickness below 12 mm must reject");
        s.layer_thickness_m = 0.151;
        require_throws([&] { nativecfg::validate(s); },
                       "thickness above 150 mm must reject");
        s.layer_thickness_m = 0.05;
        s.name = "hardpack";
        require_throws([&] { nativecfg::validate(s); },
                       "thickness on a rigid surface must reject");
    }

    // NOLINTNEXTLINE(bugprone-throwing-static-initialization,cert-err58-cpp) the registry holds only literal string_view/function-pointer pairs
    const std::array cases{
            TestCase{.name = "layer_field_resolves",
                     .run = layer_field_resolves},
            TestCase{.name = "virgin_deposit_and_hardening",
                     .run = virgin_deposit_and_hardening},
            TestCase{.name = "elastic_release_counts_loss",
                     .run = elastic_release_counts_loss},
            TestCase{.name = "probe_solve_is_pure",
                     .run = probe_solve_is_pure},
            TestCase{.name = "multipass_front_rear",
                     .run = multipass_front_rear},
            TestCase{.name = "basal_slip_cap", .run = basal_slip_cap},
            TestCase{.name = "snapshot_restore_roundtrip",
                     .run = snapshot_restore_roundtrip},
            TestCase{.name = "validation_band",
                     .run = validation_band},
    };
} // namespace

int main(int argc, char **argv) {
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunsafe-buffer-usage-in-container"
#endif
    const std::span<char *> arguments(argv, static_cast<std::size_t>(argc));
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
    if (arguments.size() == 2 &&
        std::string_view(arguments[1]) == "--list") {
        for (const TestCase &test_case: cases)
            std::cout << test_case.name << '\n';
        return 0;
    }
    if (arguments.size() > 2) {
        std::cerr << "usage: native_rut_memory_tests [case-name|--list]\n";
        return 2;
    }
    const std::string_view requested =
            arguments.size() == 1 ? "all" : std::string_view(arguments[1]);
    bool found = requested == "all";
    int status = 0;
    for (const TestCase &test_case: cases) {
        if (requested == "all" || requested == test_case.name) {
            found = true;
            try {
                test_case.run();
                std::cout << "PASS " << test_case.name << '\n';
            } catch (const std::exception &error) {
                status = 1;
                std::cerr << "FAIL " << test_case.name << ": "
                          << error.what() << '\n';
            }
        }
    }
    if (!found) {
        std::cerr << "unknown case: " << requested << '\n';
        return 2;
    }
    return status;
}
