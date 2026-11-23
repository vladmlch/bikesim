#include "allocation_faults.hpp"
#include "configuration/load.hpp"
#include "model/builder.hpp"
#include "model/rider_geometry.hpp"
#include "model_access.hpp"
#include "pneumatic/carcass.hpp"
#include "pneumatic/model.hpp"
#include "pneumatic_tire_fixture.hpp"
#include "reference_tyres.hpp"
#include "research/environment.hpp"
#include "research/evidence.hpp"
#include "runtime/runtime.hpp"
#include "runtime/samples.hpp"
#include "stepper.hpp"
#include "support/fixture_test.hpp"
#include "support/golden.hpp"
#include "terrain/trackfile.hpp"
#include "tyre/rut_memory.hpp"
#include "validation/runtime_access.hpp"
#include "writers/distributed_tire.hpp"
#include "writers/pneumatic_tire.hpp"
#include "writers/tire_common.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <format>
#include <limits>
#include <memory>
#include <vector>

namespace {

[[gnu::noinline]] void test_air_chord_and_ray_elastic_quadrature() {
    const nativecfg::PneumaticTyreSpec spec = bt::tyres::front_tyre();
    const double rho = 0.5 * spec.casing_width_mm * 0.001;
    const double area_pressure = spec.area_factor * spec.pressure_pa();
    const double carcass_si = spec.carcass_stiffness_n_mm2 * 1e6;
    const double ds = 0.008;
    for (const double d : {0.0, 1e-18, 1e-12, 1e-6, 0.003, 0.010, rho, rho + 0.006}) {
        const double w = biketyre::casing_chord_integral_m2(d, rho);
        bt::require(std::isfinite(w) && w >= 0.0, "chord integral finite and nonnegative");
        const double cap = std::min(d, rho);
        constexpr int count = 100000;
        double quad_w = 0.0;
        for (int k = 0; k < count; ++k) {
            const double t = (k + 0.5) / count;
            const double u = cap * t * t;
            quad_w += 2.0 * std::sqrt(std::max(0.0, 2.0 * rho * u - u * u))
                    * (2.0 * cap * t / count);
        }
        if (d > rho) quad_w += 2.0 * rho * (d - rho);
        const double quad_e = (area_pressure * quad_w + 0.5 * carcass_si * d * d) * ds;
        bt::require(std::abs(biketyre::ray_elastic_energy_j(d, spec, ds) - quad_e) < 1e-11,
                    "ray energy matches transformed quadrature");
    }
    const double eps = 1e-8;
    const double slope_l = (biketyre::casing_chord_integral_m2(rho, rho) -
                            biketyre::casing_chord_integral_m2(rho - eps, rho)) / eps;
    const double slope_r = (biketyre::casing_chord_integral_m2(rho + eps, rho) -
                            biketyre::casing_chord_integral_m2(rho, rho)) / eps;
    bt::require(std::abs(slope_l - 2.0 * rho) < 1e-6 &&
                std::abs(slope_r - 2.0 * rho) < 1e-10, "chord energy is C1 at cap");
}

[[gnu::noinline]] void test_maxwell_zero_carcass_stiffness_cap_inverse() {
    auto spec = bt::tyres::front_tyre();
    spec.carcass_stiffness_n_mm2 = 0.0;
    spec.rate_stiffening = 0.35;
    const double rho = 0.5 * spec.casing_width_mm * 0.001;
    const double ds = 0.008;
    const double saturated_force = biketyre::ray_elastic_force_n(rho, spec, ds);
    for (const double d : {rho, rho + 0.006}) {
        const double value = biketyre::ray_maxwell_energy_j(d, saturated_force, 0.0, spec, ds);
        bt::require(std::isfinite(value) && value >= 0.0,
                    "zero carcass stiffness saturation inverse is finite and nonnegative");
        bt::require(value < 1e-14, "saturated equilibrium has no Maxwell energy");
    }
    for (const double stiffness : {1e-8, 1e-5, 1e-3, 0.01}) {
        spec.carcass_stiffness_n_mm2 = stiffness;
        for (const double fraction : {0.9, 0.99, 0.999}) {
            const double d = fraction * rho;
            const double force = biketyre::ray_elastic_force_n(d, spec, ds);
            const double energy = biketyre::ray_maxwell_energy_j(d, force, 0.0, spec, ds);
            bt::require(energy < 1e-14, "inverse yields zero energy at equilibrium");
        }
    }
}

[[gnu::noinline]] void test_maxwell_hold_and_closed_cycle_passivity() {
    auto spec = bt::tyres::front_tyre();
    spec.rate_stiffening = 0.35;
    spec.rate_relaxation_s = 0.015;
    spec.loss_factor = 0.0;
    constexpr int n_rays = 41;
    const biketyre::RayRing ring(n_rays, 1.0);
    biketyre::CarcassState state(n_rays);
    const double ds = spec.outer_radius_mm * 0.001 * ring.dtheta_rad();
    const double dt = 0.0002;
    biketyre::RayHits hits{.r_m = std::vector<double>(ring.n()), .delta_m = std::vector<double>(ring.n()),
                           .road_x_m = std::vector<double>(ring.n()), .road_z_m = std::vector<double>(ring.n())};
    auto &delta = hits.delta_m;
    (void)biketyre::evaluate_carcass(ring, hits, spec, state, dt, 0.0, 0.0);
    delta[0] = 0.012;
    (void)biketyre::evaluate_carcass(ring, hits, spec, state, dt, 0.0, 0.0);
    const double pre = biketyre::ray_maxwell_energy_j(
            delta[0], state.elastic_prev_n()[0], state.maxwell_force_n()[0], spec, ds);
    const auto hold = biketyre::evaluate_carcass(ring, hits, spec, state, dt, 0.0, 0.0);
    const double post = biketyre::ray_maxwell_energy_j(
            delta[0], state.elastic_prev_n()[0], state.maxwell_force_n()[0], spec, ds);
    bt::require(pre > post && post >= 0.0, "Maxwell energy decreases in hold");
    bt::require(std::abs((pre - post) - hold.dissipated_power_w * dt) < 1e-14,
                "production Maxwell dissipation matches independently computed energy decrease");

    state = biketyre::CarcassState(n_rays);
    std::ranges::fill(delta, 0.0);
    (void)biketyre::evaluate_carcass(ring, hits, spec, state, dt, 0.0, 0.0);
    double work = 0.0;
    double loss = 0.0;
    const auto step = [&](double next) {
        const double previous = delta[0];
        delta[0] = next;
        const auto result = biketyre::evaluate_carcass(ring, hits, spec, state, dt, 0.0, 0.0);
        work += result.total_element_force_n[0] * (next - previous);
        loss += result.dissipated_power_w * dt;
    };
    for (int k = 1; k <= 2000; ++k) step(0.012 * k / 2000.0);
    for (int k = 0; k < 40; ++k) step(0.012);
    for (int k = 1999; k >= 0; --k) step(0.012 * k / 2000.0);
    for (int k = 0; k < 1600; ++k) {
        const double old_energy = biketyre::ray_maxwell_energy_j(
                0.0, state.elastic_prev_n()[0], state.maxwell_force_n()[0], spec, ds);
        state.advance_unloaded(0.0, dt, ring.dtheta_rad(), spec.rate_relaxation_s);
        const double new_energy = biketyre::ray_maxwell_energy_j(
                0.0, state.elastic_prev_n()[0], state.maxwell_force_n()[0], spec, ds);
        loss += std::max(0.0, old_energy - new_energy);
    }
    const double final_energy = biketyre::ray_maxwell_energy_j(
            0.0, state.elastic_prev_n()[0], state.maxwell_force_n()[0], spec, ds);
    bt::require(final_energy < 1e-12, "airborne Maxwell energy decays");
    bt::require(work >= 0.0 && std::abs(work - loss) < 1e-3, "closed cycle is passive");
}

[[gnu::noinline]] void test_single_rim_hysteresis_and_stored_energy_bitwise() {
    auto spec = bt::tyres::front_tyre();
    spec.rate_stiffening = 0.0;
    constexpr int n_rays = 41;
    const biketyre::RayRing ring(n_rays, 1.0);
    biketyre::CarcassState state(n_rays);
    constexpr double dt = 0.0005;
    biketyre::RayHits hits{.r_m = std::vector<double>(ring.n()), .delta_m = std::vector<double>(ring.n()),
                           .road_x_m = std::vector<double>(ring.n()), .road_z_m = std::vector<double>(ring.n())};
    (void)biketyre::evaluate_carcass(ring, hits, spec, state, dt, 0.0, 0.0);
    hits.delta_m[0] = spec.rim_strike_deflection_mm() * 0.001 + 0.004;
    const auto result = biketyre::evaluate_carcass(ring, hits, spec, state, dt, 0.0, 0.0);
    bt::require(std::abs(result.total_element_force_n[0] - result.elastic_force_n[0] -
                         result.hysteresis_force_n[0] - result.rim_force_n[0]) < 1e-12,
                "rim hysteresis counted once");
    bt::pneumatic::Rig rig("detailed", 0.00025, 0.006);
    rig.spin("front", 4.0);
    for (int step = 0; step < 5; ++step) {
        (void)rig.advance();
        bt::require(rig.tire->stored_energy(rig.engine.data.get()) == rig.tire->elastic_energy_j(),
                    "loaded stored energy bitwise");
        rig.engine.data->time += rig.dt();
    }
    model_access::mutable_buffer(rig.engine.data->qpos, rig.engine.model->nq)[1] = 0.25;
    ::engine::forward(rig.engine.model.get(), rig.engine.data.get());
    (void)rig.advance();
    bt::require(rig.tire->stored_energy(rig.engine.data.get()) == rig.tire->elastic_energy_j(),
                "airborne stored energy bitwise");
}

[[nodiscard]] nativecfg::SurfaceSpec surface(std::string_view name) {
    const auto &s = bike::terrain::get_surface(name);
    return {.name = s.name, .mu_peak = s.mu_peak, .mu_slide = s.mu_slide,
            .slip_stiffness_per_load = s.slip_stiffness_per_load,
            .stribeck_speed_mps = s.stribeck_speed_mps,
            .crr_delta = s.crr_delta, .load_sensitivity = s.load_sensitivity};
}

void bitwise_equal(std::span<const double> a, std::span<const double> b) {
    bt::require(a.size() == b.size(), "bitwise arrays have the same size");
    for (std::size_t i = 0; i < a.size(); ++i)
        bt::require(std::bit_cast<std::uint64_t>(a[i]) == std::bit_cast<std::uint64_t>(b[i]),
                    "binary64 state unchanged");
}

void same_patch_output(const biketyre::TyrePatchOutput &a, const biketyre::TyrePatchOutput &b) {
    bt::require(a.ray_start == b.ray_start && a.ray_stop == b.ray_stop &&
                a.surface_name == b.surface_name && a.fully_sliding == b.fully_sliding,
                "patch output metadata unchanged");
    bitwise_equal(a.normal_force_world_n, b.normal_force_world_n);
    bitwise_equal(a.tangent_world, b.tangent_world);
    bitwise_equal(a.force_world_n, b.force_world_n);
    bitwise_equal(a.centroid_world_m, b.centroid_world_m);
    bitwise_equal(std::array{a.normal_load_n, a.support_n, a.tangential_force_n,
                             a.mean_deflection_m, a.contact_length_m, a.slip_ratio,
                             a.transient_slip_ratio, a.sliding_speed_mps, a.slip_speed_mps,
                             a.friction_coefficient, a.dissipated_power_w, a.shear_energy_j, a.rolling_crr},
                  std::array{b.normal_load_n, b.support_n, b.tangential_force_n,
                             b.mean_deflection_m, b.contact_length_m, b.slip_ratio,
                             b.transient_slip_ratio, b.sliding_speed_mps, b.slip_speed_mps,
                             b.friction_coefficient, b.dissipated_power_w, b.shear_energy_j, b.rolling_crr});
}

void same_model_state(const biketyre::PneumaticTyreModel &a, const biketyre::PneumaticTyreModel &b) {
    const auto &ac = a.carcass_state();
    const auto &bc = b.carcass_state();
    bt::require(ac.initialized() == bc.initialized() && ac.rim_strike_active() == bc.rim_strike_active(),
                "carcass flags unchanged");
    bitwise_equal(ac.delta_prev_m(), bc.delta_prev_m());
    bitwise_equal(ac.elastic_prev_n(), bc.elastic_prev_n());
    bitwise_equal(ac.maxwell_force_n(), bc.maxwell_force_n());
    bitwise_equal(std::array{ac.rim_event_x_m(), ac.rim_event_speed_mps(), ac.rim_event_peak_load_n(),
                             ac.rim_event_peak_force_n(), ac.rim_event_energy_j()},
                  std::array{bc.rim_event_x_m(), bc.rim_event_speed_mps(), bc.rim_event_peak_load_n(),
                             bc.rim_event_peak_force_n(), bc.rim_event_energy_j()});
    const auto ap = a.patch_states(), bp = b.patch_states();
    bt::require(ap.size() == bp.size(), "patch state count unchanged");
    for (std::size_t i = 0; i < ap.size(); ++i) {
        bt::require(ap[i].start == bp[i].start && ap[i].stop == bp[i].stop &&
                    ap[i].has_detailed == bp[i].has_detailed, "patch state metadata unchanged");
        bitwise_equal(std::array{ap[i].kappa_prime, ap[i].detailed_kappa_prime},
                      std::array{bp[i].kappa_prime, bp[i].detailed_kappa_prime});
        bitwise_equal(ap[i].bristle_deflection_m, bp[i].bristle_deflection_m);
    }
    const auto &ao = a.last_outputs(), &bo = b.last_outputs();
    bt::require(ao.tyre_name == bo.tyre_name && ao.fully_sliding == bo.fully_sliding &&
                ao.rim_strike_active == bo.rim_strike_active && ao.coverage_event == bo.coverage_event &&
                ao.airborne == bo.airborne && ao.patch_count == bo.patch_count &&
                ao.rim_event.has_value() == bo.rim_event.has_value() &&
                ao.patch_slots.size() == bo.patch_slots.size(), "wheel output metadata unchanged");
    bitwise_equal(ao.hub_position_world_m, bo.hub_position_world_m);
    bitwise_equal(ao.hub_velocity_world_mps, bo.hub_velocity_world_mps);
    bitwise_equal(ao.force_world_n, bo.force_world_n);
    bitwise_equal(std::array{ao.pressure_bar, ao.omega_forward_radps, ao.normal_load_n, ao.support_n,
                             ao.mean_deflection_m, ao.contact_length_m, ao.slip_ratio, ao.transient_slip_ratio,
                             ao.dissipated_power_w, ao.carcass_dissipated_power_w, ao.rolling_crr},
                  std::array{bo.pressure_bar, bo.omega_forward_radps, bo.normal_load_n, bo.support_n,
                             bo.mean_deflection_m, bo.contact_length_m, bo.slip_ratio, bo.transient_slip_ratio,
                             bo.dissipated_power_w, bo.carcass_dissipated_power_w, bo.rolling_crr});
    if (ao.rim_event && bo.rim_event) {
        const auto &x = *ao.rim_event, &y = *bo.rim_event;
        bitwise_equal(std::array{x.x_m, x.speed_mps, x.peak_load_n, x.peak_rim_force_n, x.absorbed_energy_j},
                      std::array{y.x_m, y.speed_mps, y.peak_load_n, y.peak_rim_force_n, y.absorbed_energy_j});
    }
    for (std::size_t i = 0; i < ao.patch_slots.size(); ++i)
        same_patch_output(ao.patch_slots[i], bo.patch_slots[i]);
}

[[gnu::noinline]] void test_airborne_zero_dt_and_rejection_atomicity() {
    auto spec = bt::tyres::front_tyre();
    spec.rate_stiffening = 0.35;
    const nativecfg::SurfaceMap map{.surface = surface("hardpack"), .sections = {}};
    const biketyre::RoadProfile road(-1.0, 0.005, std::vector<double>(1801, 0.0));
    const std::array<double, 3> velocity{2.0, 0.0, 0.0};
    const std::array<double, 3> air{0.0, 0.0, spec.outer_radius_mm * 0.001 + 0.5};
    const std::array<double, 3> loaded{0.0, 0.0, spec.outer_radius_mm * 0.001 - 0.008};
    biketyre::PneumaticTyreModel model(spec, 128, 75.0, true), saved(spec, 128, 75.0, true);
    (void)model.evaluate(loaded, velocity, 7.0, road, map, 0.00025);
    bt::require(model.patch_states().size() == 1, "atomicity fixture has a patch");
    auto &carcass = model.carcass_state();
    std::ranges::fill(carcass.delta_prev_m(), 0.0);
    std::ranges::fill(carcass.elastic_prev_n(), 0.0);
    std::ranges::fill(carcass.maxwell_force_n(), -1e160);
    saved.assign(model);
    bool overflow = false;
    try {
        (void)model.evaluate(air, velocity, std::numeric_limits<double>::max(), road, map, 2.0);
    } catch (const std::overflow_error &) {
        overflow = true;
    }
    bt::require(overflow, "extreme airborne energy is rejected");
    same_model_state(model, saved);

    std::ranges::fill(carcass.maxwell_force_n(), -1.0);
    saved.assign(model);
    for (const double dt : {std::numeric_limits<double>::quiet_NaN(), 0.0, -1.0,
                            std::numeric_limits<double>::infinity()}) {
        bool rejected = false;
        try {
            (void)model.evaluate(air, velocity, 9.0, road, map, dt);
        } catch (const std::invalid_argument &) {
            rejected = true;
        }
        bt::require(rejected, "initialized airborne invalid dt rejected as invalid_argument");
        same_model_state(model, saved);
    }
    const double ds = spec.outer_radius_mm * 0.001 * model.ring().dtheta_rad();
    double before = 0.0;
    for (const double force : saved.carcass_state().maxwell_force_n())
        before += biketyre::ray_maxwell_energy_j(0.0, 0.0, force, spec, ds);
    biketyre::CarcassState direct(128);
    direct.assign(saved.carcass_state());
    direct.advance_unloaded(9.0, 0.00025, model.ring().dtheta_rad(), spec.rate_relaxation_s);
    double after = 0.0;
    for (const double force : direct.maxwell_force_n())
        after += biketyre::ray_maxwell_energy_j(0.0, 0.0, force, spec, ds);
    const auto &air_result = model.evaluate(air, velocity, 9.0, road, map, 0.00025);
    bitwise_equal(model.carcass_state().maxwell_force_n(), direct.maxwell_force_n());
    bitwise_equal(std::array{air_result.carcass_dissipated_power_w, air_result.dissipated_power_w},
                  std::array{std::max(0.0, before - after) / 0.00025, std::max(0.0, before - after) / 0.00025});
    (void)saved.evaluate(air, velocity, 9.0, road, map, 0.00025);
    same_model_state(model, saved);
    biketyre::PneumaticTyreModel fresh(spec, 128, 75.0, true);
    const auto &silent = fresh.evaluate(air, velocity, 9.0, road, map, 0.0);
    bt::require(silent.airborne && silent.dissipated_power_w == 0.0 &&
                silent.carcass_dissipated_power_w == 0.0, "fresh airborne zero dt is lossless");
}

[[gnu::noinline]] void test_zero_contact_length_shear_energy() {
    auto spec = bt::tyres::front_tyre();
    spec.contact_length_factor = 0.0;
    const nativecfg::SurfaceMap map{
            .surface = surface("hardpack"),
            .sections = {{.start_m = 0.0, .end_m = 5.0, .surface = surface("scree"), .blend_m = 0.0}}};
    const biketyre::RoadProfile road(-1.0, 0.005, std::vector<double>(1801, 0.0));
    for (const double x : {1.0, 5.0}) {
        biketyre::PneumaticTyreModel model(spec, 128, 75.0, true);
        const auto hits = biketyre::intersect(model.ring(), x, spec.outer_radius_mm * 0.001 - 0.008,
                                              spec.outer_radius_mm * 0.001, road);
        const auto &out = model.evaluate(std::array{x, 0.0, spec.outer_radius_mm * 0.001 - 0.008},
                                         std::array{2.0, 0.0, 0.0}, 7.0, road, map, 0.00025);
        bt::require(out.patch_count == 1 && out.patches()[0].contact_length_m == 0.0 &&
                    out.patches()[0].shear_energy_j == 0.0 &&
                    out.patches()[0].tangential_force_n == 0.0, "zero contact length resets shear energy");
        const auto &patch = out.patches()[0];
        const auto xs = std::span<const double>(hits.road_x_m).subspan(
                static_cast<std::size_t>(patch.ray_start),
                static_cast<std::size_t>(patch.ray_stop - patch.ray_start));
        const auto [lo, hi] = std::ranges::minmax_element(xs);
        bt::require((tirecommon::uniform_surface_for_span(map, *lo, *hi) != nullptr) == (x == 1.0),
                    "zero contact length covers uniform and mixed brush paths");
    }
}

// Non-golden counterpart of test_native_pneumatic_model's validation block.
[[gnu::noinline]] void test_model_validation_without_golden() {
    const auto spec = bt::tyres::front_tyre();
    const nativecfg::SurfaceMap map{.surface = surface("hardpack"), .sections = {}};
    const biketyre::RoadProfile road(-1.0, 0.005, std::vector<double>(1801, 0.0));
    const std::array<double, 3> hub{0.0, 0.0, spec.outer_radius_mm * 0.001 - 0.008}, vel{2.0, 0.0, 0.0};
    const double nan = std::numeric_limits<double>::quiet_NaN(), inf = std::numeric_limits<double>::infinity();
    const std::array<double, 4> hub4{};
    const std::array<double, 2> vel2{};
    const std::array<double, 3> nan_hub{0.0, nan, hub[2]}, inf_hub{0.0, 0.0, inf},
            nan_vel{nan, 0.0, 0.0}, inf_vel{0.0, 0.0, -inf};
    struct BadInput {
        std::span<const double> h, v;
        double omega, dt;
    };
    const std::array bad{
            BadInput{.h = hub4, .v = vel, .omega = 9.0, .dt = 0.002},
            BadInput{.h = hub, .v = vel2, .omega = 9.0, .dt = 0.002},
            BadInput{.h = nan_hub, .v = vel, .omega = 9.0, .dt = 0.002},
            BadInput{.h = inf_hub, .v = vel, .omega = 9.0, .dt = 0.002},
            BadInput{.h = hub, .v = nan_vel, .omega = 9.0, .dt = 0.002},
            BadInput{.h = hub, .v = inf_vel, .omega = 9.0, .dt = 0.002},
            BadInput{.h = hub, .v = vel, .omega = nan, .dt = 0.002},
            BadInput{.h = hub, .v = vel, .omega = inf, .dt = 0.002},
            BadInput{.h = hub, .v = vel, .omega = 9.0, .dt = 0.0},
            BadInput{.h = hub, .v = vel, .omega = 9.0, .dt = -0.002},
            BadInput{.h = hub, .v = vel, .omega = 9.0, .dt = nan},
            BadInput{.h = hub, .v = vel, .omega = 9.0, .dt = inf}};
    for (const bool detailed : {false, true}) {
        biketyre::PneumaticTyreModel model(spec, 128, 75.0, detailed), saved(spec, 128, 75.0, detailed);
        (void)model.evaluate(hub, vel, 9.0, road, map, 0.002);
        saved.assign(model);
        for (const auto &row : bad) {
            bool rejected = false;
            try {
                (void)model.evaluate(row.h, row.v, row.omega, road, map, row.dt);
            } catch (const std::invalid_argument &) {
                rejected = true;
            }
            bt::require(rejected, "loaded invalid model input rejected");
            same_model_state(model, saved);
        }
        const std::array<double, 1> nan1{nan};
        bt::pneumatic::rejects<std::invalid_argument>(
                [&] { (void)model.evaluate(hub4, nan1, nan, road, map, 0.002); }, "3-vectors");
        bt::pneumatic::rejects<std::invalid_argument>(
                [&] { (void)model.evaluate(inf_hub, vel, nan, road, map, 0.002); }, "finite");
        biketyre::CarcassState narrow(127);
        bt::pneumatic::rejects<std::invalid_argument>([&] { model.carcass_state().assign(narrow); }, "mismatch");
        biketyre::PatchBrushState wide;
        wide.bristle_deflection_m.resize(129);
        bt::pneumatic::rejects<std::invalid_argument>(
                [&] { model.restore_patch_states(std::array{wide}); }, "exceeds");
        same_model_state(model, saved);
        (void)model.evaluate(hub, vel, 9.0, road, map, 0.002);
        (void)saved.evaluate(hub, vel, 9.0, road, map, 0.002);
        same_model_state(model, saved);
    }
}

[[gnu::noinline]] void test_negative_centroid_mixed_surface_name() {
    const auto spec = bt::tyres::front_tyre();
    auto first = surface("scree");
    first.name = "first_section_long_surface_name";
    const nativecfg::SurfaceMap map{
            .surface = surface("hardpack"),
            .sections = {{.start_m = 0.0, .end_m = 0.025, .surface = first, .blend_m = 0.04}}};
    const biketyre::RoadProfile road(-1.0, 0.001, std::vector<double>(9001, 0.0));
    biketyre::PneumaticTyreModel model(spec, 256, 75.0, true);
    model.reserve_patch_names(first.name.size());
    const auto &out = model.evaluate(std::array{-0.02, 0.0, spec.outer_radius_mm * 0.001 - 0.008},
                                     std::array{2.0, 0.0, 0.0}, 7.0, road, map, 0.00025);
    bt::require(out.patch_count == 1, "mixed surface has one patch");
    const auto &patch = out.patches()[0];
    const auto hits = biketyre::intersect(model.ring(), -0.02, spec.outer_radius_mm * 0.001 - 0.008,
                                          spec.outer_radius_mm * 0.001, road);
    const auto xs = std::span<const double>(hits.road_x_m).subspan(
            static_cast<std::size_t>(patch.ray_start), static_cast<std::size_t>(patch.ray_stop - patch.ray_start));
    const auto [lo, hi] = std::ranges::minmax_element(xs);
    bt::require(*lo < 0.0 && *hi > 0.025 && patch.centroid_world_m[0] < 0.0 &&
                tirecommon::uniform_surface_for_span(map, *lo, *hi) == nullptr,
                "mixed patch straddles x zero and section transition with negative centroid");
    bt::require(patch.surface_name == first.name &&
                patch.surface_name == tirecommon::eval_surface(map, patch.centroid_world_m[0]).name,
                "negative mixed centroid uses canonical first-section name");
}

[[gnu::noinline]] void test_uniform_and_heterogeneous_brush_equivalence() {
    auto spec = surface("hardpack");
    spec.load_sensitivity = 0.2;
    const std::array<double, 5> loads{0.1, 2.3, 10.17, 36.4, 8.67};
    const std::array samples{tirecommon::sample_from_spec(spec), tirecommon::sample_from_spec(spec),
                             tirecommon::sample_from_spec(spec), tirecommon::sample_from_spec(spec),
                             tirecommon::sample_from_spec(spec)};
    for (const double wheel_load : {149.0, 700.0}) {
        auto adjusted = spec;
        const double factor = tirecommon::load_sensitivity_factor(wheel_load, spec.load_sensitivity);
        adjusted.mu_peak *= factor;
        adjusted.mu_slide *= factor;
        biketyre::DiscretisedBrush uniform(5), mixed(5);
        for (int i = 0; i < 100; ++i) {
            const auto a = uniform.step(loads, 0.05, 2.0, -0.15, 0.06, adjusted, 0.00025);
            const auto b = mixed.step_heterogeneous(loads, 0.05, 2.0, -0.15, 0.06,
                                                    samples, 149.0, wheel_load, 0.00025);
            bt::require(a.force_n == b.force_n, "uniform and heterogeneous brush force is bitwise equal");
            bt::require(a.fully_sliding == b.fully_sliding && a.friction_coefficient == b.friction_coefficient,
                        "uniform and heterogeneous brush output is bitwise equal");
            bt::pneumatic::numeric_equal(uniform.bristle_deflection_m(), mixed.bristle_deflection_m());
            bt::require(uniform.kappa_prime() == mixed.kappa_prime(), "brush relaxation is bitwise equal");
        }
        const std::array<double, 5> zeros{};
        for (const bool zero_length : {false, true}) {
            const auto &input = zero_length ? loads : zeros;
            const double half_length = zero_length ? 0.0 : 0.05;
            const auto a = uniform.step(input, half_length, 2.0, -0.15, 0.06, adjusted, 0.00025);
            const auto b = mixed.step_heterogeneous(input, half_length, 2.0, -0.15, 0.06,
                                                    samples, 149.0, wheel_load, 0.00025);
            bt::require(a.force_n == b.force_n && a.fully_sliding == b.fully_sliding &&
                        a.friction_coefficient == b.friction_coefficient,
                        "zero-load and zero-length brush paths preserve uniform output");
            bt::pneumatic::numeric_equal(uniform.bristle_deflection_m(), mixed.bristle_deflection_m());
        }
    }
}

[[gnu::noinline]] void test_brush_shear_energy_matches_element_stiffness() {
    const auto spec = bt::tyres::front_tyre();
    const nativecfg::SurfaceMap map{
            .surface = surface("hardpack"),
            .sections = {{.start_m = 5.0, .end_m = 8.0, .surface = surface("scree"), .blend_m = 0.0}}};
    const biketyre::RoadProfile road(-1.0, 0.005, std::vector<double>(1801, 0.0));
    for (const double x : {4.8, 5.0}) {
        biketyre::PneumaticTyreModel model(spec, 256, 75.0, true);
        const std::array<double, 3> hub{x, 0.0, spec.outer_radius_mm * 0.001 - 0.008};
        const std::array<double, 3> velocity{2.0, 0.0, 0.0};
        const double omega = 7.0, dt = 0.00025;
        const auto hits = biketyre::intersect(model.ring(), hub[0], hub[2],
                                              spec.outer_radius_mm * 0.001, road);
        biketyre::CarcassState carcass(256);
        const auto loads = biketyre::evaluate_carcass(model.ring(), hits, spec, carcass, dt, omega, 2.0);
        const auto &out = model.evaluate(hub, velocity, omega, road, map, dt);
        bt::require(out.patch_count == 1 && model.patch_states().size() == 1, "one detailed patch");
        const auto &patch = out.patches()[0];
        const auto &state = model.patch_states()[0];
        const auto first = static_cast<std::size_t>(patch.ray_start);
        const auto count = static_cast<std::size_t>(patch.ray_stop - patch.ray_start);
        const auto xs = std::span<const double>(hits.road_x_m).subspan(first, count);
        const auto [xmin, xmax] = std::ranges::minmax_element(xs);
        const bool uniform = tirecommon::uniform_surface_for_span(map, *xmin, *xmax) != nullptr;
        bt::require(uniform == (x == 4.8), "test exercises uniform and mixed surface paths");
        double total = 0.0;
        for (std::size_t i = 0; i < count; ++i) total += loads.total_element_force_n[first + i];
        const double half_length = patch.contact_length_m * 0.5;
        const double dx = 2.0 * half_length / static_cast<double>(count);
        double expected = 0.0;
        for (std::size_t i = 0; i < count; ++i) {
            const auto sample = tirecommon::eval_surface(map, xs[i]);
            const double stiffness = sample.slip_stiffness_per_load * total /
                                     (2.0 * half_length * half_length) * dx;
            const double u = state.bristle_deflection_m[i];
            expected += 0.5 * stiffness * u * u;
        }
        bt::require(expected > 0.0 && std::abs(patch.shear_energy_j - expected) < 1e-12,
                    "shear energy equals sum of one-half k u squared");
    }
}

[[gnu::noinline]] void test_pneumatic_surface_name_storage_survives_reset() {
    bt::pneumatic::Rig rig("detailed", 0.00025, 0.006);
    auto cfg = bt::pneumatic::configuration("detailed");
    if (!cfg.pneumatic.has_value() || !cfg.pneumatic->surface.has_value())
        throw std::logic_error("fixture needs a configured pneumatic surface");
    cfg.pneumatic->surface->name = "custom_surface_name_longer_than_sso";
    rig.tire = std::make_unique<PneumaticTireWriter>(rig.engine.model.get(), rig.engine.data.get(), cfg);
    const auto name = cfg.pneumatic->surface->name;
    const char *front = rig.tire->diagnostic_storage()[0].surface.data();
    const char *rear = rig.tire->diagnostic_storage()[1].surface.data();
    bt::require(rig.tire->diagnostic_storage()[0].surface.capacity() >= name.size() &&
                rig.tire->diagnostic_storage()[1].surface.capacity() >= name.size(),
                "long surface buffers reserved");
    std::vector<double> out(static_cast<std::size_t>(rig.engine.model->nv));
    const auto check = [&] {
        bt::require(rig.tire->try_compute_into(rig.engine.data.get(), rig.dt(), out) == CoreStatus::ok,
                    "long surface try_compute_into succeeds");
        bt::require(rig.tire->diagnostic_storage()[0].surface == name &&
                    rig.tire->diagnostic_storage()[1].surface == name &&
                    rig.tire->diagnostic_storage()[0].surface.data() == front &&
                    rig.tire->diagnostic_storage()[1].surface.data() == rear,
                    "loaded and airborne diagnostic strings reuse their buffers");
    };
    check();
    rig.tire->reset();
    bt::require(rig.tire->diagnostic_storage()[0].surface.data() == front &&
                rig.tire->diagnostic_storage()[1].surface.data() == rear,
                "reset preserves diagnostic string buffers");
    rig.engine.data->time += rig.dt();
    check();
}

[[gnu::noinline]] void test_pneumatic_crr_overflow_atomicity() {
    bt::pneumatic::Rig rig("fast", 0.0005, 0.006);
    auto cfg = bt::pneumatic::configuration("fast");
    if (!cfg.pneumatic.has_value() || !cfg.pneumatic->surface.has_value())
        throw std::logic_error("fixture needs a configured pneumatic surface");
    cfg.pneumatic->surface->crr_delta = std::numeric_limits<double>::max();
    rig.tire = std::make_unique<PneumaticTireWriter>(rig.engine.model.get(), rig.engine.data.get(), cfg);
    const auto qpos = model_access::mutable_buffer(rig.engine.data->qpos, rig.engine.model->nq);
    qpos[1] = 1.0;
    rig.spin("front", 4.0);
    (void)rig.advance();
    const auto state = rig.tire->state();
    const auto clock = bt::pneumatic::Access::clock(*rig.tire);
    const double energy = rig.tire->elastic_energy_j();
    const double loss = rig.tire->brush_loss_step_j();
    const auto snapshot_time = rig.tire->snapshot_storage()[0].time_s;
    qpos[1] = 0.0;
    rig.engine.data->time += rig.dt();
    ::engine::forward(rig.engine.model.get(), rig.engine.data.get());
    std::vector<double> out(static_cast<std::size_t>(rig.engine.model->nv), 0.0);
    bt::require(rig.tire->try_compute_into(rig.engine.data.get(), rig.dt(), out) != CoreStatus::ok,
                "finite extreme crr overflow is rejected");
    bt::pneumatic::numeric_equal(rig.tire->state(), state, true);
    bt::require(bt::pneumatic::Access::clock(*rig.tire) == clock &&
                rig.tire->elastic_energy_j() == energy && rig.tire->brush_loss_step_j() == loss &&
                rig.tire->snapshot_storage()[0].time_s == snapshot_time,
                "overflow leaves clock, energy, loss and snapshot unchanged");
    bt::require(rig.tire->try_compute_into(rig.engine.data.get(), rig.dt(), out) != CoreStatus::ok,
                "overflow retry remains at the same uncommitted timestamp");
}

[[gnu::noinline]] void test_pneumatic_world_rolling_moment_no_caliper_reaction() {
    constexpr double dt = 0.0005;
    const std::string xml = R"(<mujoco><compiler angle="radian"/>
      <option timestep="0.0005" gravity="0 0 0" integrator="Euler"/>
      <worldbody>
        <body name="frame" pos="0 0 0.366">
          <joint name="root_x" type="slide" axis="1 0 0"/>
          <joint name="root_z" type="slide" axis="0 0 1"/>
          <joint name="root_pitch" type="hinge" axis="0 1 0"/>
          <inertial mass="10.0" pos="0 0 0" diaginertia="1.0 1.2 1.0"/>
          <body name="front_wheel" pos="0 0 0">
            <joint name="front_wheel_spin" type="hinge" axis="0 1 0"/>
            <inertial mass="2.4" pos="0 0 0" diaginertia="0.15 0.30 0.15"/>
            <geom name="geom_front_contact" type="sphere" size="0.372" contype="0" conaffinity="0" mass="0"/>
          </body>
        </body>
        <body name="rear_wheel" pos="0.8 0 5.0">
          <joint name="rear_wheel_spin" type="hinge" axis="0 1 0"/>
          <inertial mass="2.4" pos="0 0 0" diaginertia="0.15 0.30 0.15"/>
          <geom name="geom_rear_contact" type="sphere" size="0.352" contype="0" conaffinity="0" mass="0"/>
        </body>
      </worldbody></mujoco>)";
    bt::ModelFixture eng(xml);
    ::engine::forward(eng.model.get(), eng.data.get());
    auto cfg = bt::pneumatic::configuration("fast");
    cfg.surface_mode = "track";
    if (!cfg.pneumatic.has_value()) throw std::logic_error("fixture needs pneumatic configuration");
    cfg.pneumatic->surface.reset();
    cfg.surface_map = nativecfg::SurfaceMap{.surface = surface("scree"), .sections = {}};
    auto writer_scree = std::make_unique<PneumaticTireWriter>(eng.model.get(), eng.data.get(), cfg);
    cfg.surface_map->surface.crr_delta = 0.0;
    auto writer_nocrr = std::make_unique<PneumaticTireWriter>(eng.model.get(), eng.data.get(), cfg);
    const auto velocity = model_access::mutable_buffer(eng.data->qvel, eng.model->nv);
    velocity[0] = 3.0;
    velocity[3] = 3.0 / 0.366;
    ::engine::forward(eng.model.get(), eng.data.get());
    const auto scree_force = writer_scree->qfrc(eng.data.get(), dt);
    const auto nocrr_force = writer_nocrr->qfrc(eng.data.get(), dt);
    const auto &snap = writer_scree->snapshot_storage()[0];
    bt::require(!snap.patches.empty(), "front wheel loaded");
    const auto nv = static_cast<std::size_t>(eng.model->nv);
    std::vector<double> expected(nv, 0.0);
    std::vector<mjtNum> jp(3 * nv, 0.0), jr(3 * nv, 0.0);
    const int body = bike::model::required_id(*eng.model, mjOBJ_BODY, "front_wheel");
    for (const TirePatch &patch : snap.patches) {
        bt::require(patch.couple_world_nm[1] < 0.0, "world couple opposes spin");
        mj_jac(eng.model.get(), eng.data.get(), jp.data(), jr.data(), patch.point_m.data(), body);
        const auto force = bt::pneumatic::snapshot_force(patch);
        for (std::size_t i = 0; i < nv; ++i)
            for (std::size_t k = 0; k < 3; ++k)
                expected[i] += jp[k * nv + i] * force[k] + jr[k * nv + i] * patch.couple_world_nm[k];
    }
    for (std::size_t i = 0; i < nv; ++i)
        bt::require(std::abs(scree_force[i] - expected[i]) <= 1e-12, "world patch force and couple projection");
    const auto applied = model_access::mutable_buffer(eng.data->qfrc_applied, eng.model->nv);
    std::ranges::copy(scree_force, applied.begin());
    ::engine::forward(eng.model.get(), eng.data.get());
    const double qacc_scree = model_access::readonly_buffer(eng.data->qacc, eng.model->nv)[2];
    std::ranges::copy(nocrr_force, applied.begin());
    ::engine::forward(eng.model.get(), eng.data.get());
    const double qacc_nocrr = model_access::readonly_buffer(eng.data->qacc, eng.model->nv)[2];
    bt::require(std::abs(qacc_scree - qacc_nocrr) <= 1e-12, "no frame caliper reaction");
    velocity[0] = -0.5;
    velocity[3] = 5.0;
    eng.data->time += dt;
    ::engine::forward(eng.model.get(), eng.data.get());
    (void)writer_scree->qfrc(eng.data.get(), dt);
    (void)writer_nocrr->qfrc(eng.data.get(), dt);
    bt::require(writer_scree->brush_loss_step_j() > writer_nocrr->brush_loss_step_j(),
                "extra rolling loss remains passive during rollback");
}

[[gnu::noinline]] void test_pneumatic_boundary_crossing_smoothness_and_stored_energy() {
    for (const double blend : {0.0, 0.4}) {
        bt::pneumatic::Rig rig("detailed", 0.00025, 0.008);
        auto cfg = bt::pneumatic::configuration("detailed");
        cfg.surface_mode = "track";
        if (!cfg.pneumatic.has_value()) throw std::logic_error("fixture needs pneumatic configuration");
        cfg.pneumatic->surface.reset();
        const auto x_grid = bike::geometry::detail::np_linspace(-1.0, 8.0, 1801);
        cfg.pneumatic->profile = {
                .x0_m = x_grid.front(), .dx_m = x_grid[1] - x_grid[0],
                .z_m = std::vector<double>(x_grid.size(), 0.0)};
        cfg.surface_map = nativecfg::SurfaceMap{
                .surface = surface("hardpack"),
                .sections = {{.start_m = 5.0, .end_m = 8.0, .surface = surface("scree"), .blend_m = blend}}};
        rig.tire = std::make_unique<PneumaticTireWriter>(
                rig.engine.model.get(), rig.engine.data.get(), cfg);
        const auto qpos = model_access::mutable_buffer(rig.engine.data->qpos, rig.engine.model->nq);
        const auto qvel = model_access::mutable_buffer(rig.engine.data->qvel, rig.engine.model->nv);
        qpos[0] = 4.55;
        qvel[0] = 2.0;
        qvel[2] = 2.0 / (bt::pneumatic::front_radius() - 0.008) + 1.2;
        ::engine::forward(rig.engine.model.get(), rig.engine.data.get());
        for (int step = 0; step < 2000; ++step) {
            (void)rig.advance();
            rig.engine.data->time += rig.dt();
        }
        std::vector<double> trace;
        bool before = false, after = false;
        for (int step = 0; step <= 3600; ++step) {
            (void)rig.advance();
            bt::require(rig.tire->stored_energy(rig.engine.data.get()) == rig.tire->elastic_energy_j(),
                        "detailed stored energy bitwise at boundary");
            double fx = 0.0;
            for (const TirePatch &patch : rig.tire->snapshot_storage()[0].patches) fx += patch.tangent_force_n;
            if (qpos[0] < 4.75) before = true;
            if (qpos[0] > 5.25) after = true;
            trace.push_back(fx);
            qpos[0] += qvel[0] * rig.dt();
            rig.engine.data->time += rig.dt();
            ::engine::forward(rig.engine.model.get(), rig.engine.data.get());
        }
        bt::require(before && after, "samples on both sides of entire footprint");
        const double drop = trace.front() - trace.back();
        bt::require(drop > 5.0, "tangential force drops across boundary");
        double max_jump = 0.0;
        for (std::size_t i = 1; i < trace.size(); ++i) {
            max_jump = std::max(max_jump, std::abs(trace[i] - trace[i - 1]));
        }
        // Individual discrete rays can ripple; inspect non-overlapping
        // 5 cm averages as well as strict endpoint and jump bounds.
        double max_window_rise = 0.0;
        constexpr std::size_t window = 100;
        double previous_mean = 0.0;
        for (std::size_t start = 0; start + window <= trace.size(); start += window) {
            double sum = 0.0;
            for (std::size_t i = start; i < start + window; ++i) sum += trace[i];
            const double mean = sum / static_cast<double>(window);
            if (start != 0) max_window_rise = std::max(max_window_rise, mean - previous_mean);
            previous_mean = mean;
        }
        const auto [low, high] = std::ranges::minmax_element(trace);
        bt::require(max_jump < 0.05 * drop, "footprint smooths boundary jump");
        bt::require(*low >= trace.back() - 0.1 && *high <= trace.front() + 0.1,
                    "boundary force remains within endpoint bounds");
        bt::require(max_window_rise <= 1e-6, "window-averaged force transitions monotonically");
    }
}

[[gnu::noinline]] void test_pneumatic_60m_mixed_surface_ride_energy_gate() {
    namespace research = bike::research;
    namespace sw = runtime::sample_wire;
    const auto root = bt::oracle::root().parent_path().parent_path().parent_path();
    auto owner = std::make_unique<research::ResearchRequest>();
    auto &request = *owner;
    request.setup.model.physics = bike::configuration::load_physics(
            root / "examples/research/viewer_physics_pneumatic.toml");
    request.setup.model.mode = "ride";
    request.setup.model.crank_joint = true;
    request.setup.model.rider = bike::model::rider_from_args("articulated_planar");
    request.setup.track = bike::terrain::track_from_dict(bike::terrain::parse_track_toml(
            "name = \"stage_a_mixed_60m\"\nlength_m = 120.0\nsurface = \"hardpack\"\n"
            "grade_profile = { knots = [[0.0, 0.02], [120.0, 0.02]] }\n"
            "[[surface_sections]]\nstart_m = 15.0\nend_m = 35.0\nsurface = \"scree\"\nblend_m = 0.4\n"
            "[[surface_sections]]\nstart_m = 42.0\nend_m = 58.0\nsurface = \"wet\"\nblend_m = 0.4\n"));
    request.experiment.duration_s = 22.0;
    request.experiment.record_decimation = 1;
    request.experiment.stop_on_model_violation = true;
    request.experiment.control_period_s = request.setup.model.physics->research.control_period_s;
    request.experiment.actuator_delay_s = request.setup.model.physics->research.actuator_delay_s;
    request.sensors.delivery.latency_s = request.setup.model.physics->research.sensor_latency_s;
    request.sensors.delivery.maximum_age_s = request.setup.model.physics->research.sensor_maximum_age_s;
    request.sensors.sample_period_s = 0.005;
    auto prepared = research::prepare_environment(std::move(owner));
    const auto period = prepared->bootstrap.config.control_steps;
    const auto max_steps = prepared->bootstrap.config.max_steps;
    auto rt = std::make_unique<runtime::NativeRideRuntime>(std::move(prepared->bootstrap.ride));
    while (!rt->snapshot().outcome.has_value() && rt->snapshot().step < max_steps) {
        const auto result = rt->advance(rt->snapshot().step + period, runtime::RideControl{}, 0., 0., std::nullopt);
        if (!rt->recorded_intervals().empty() && rt->recorded_intervals().back()->qpos[0] >= 62.0) break;
        if (result.reason == "outcome") break;
    }
    const auto snapshot = rt->snapshot();
    bt::require(!snapshot.first_failure.has_value(), "ride has no first failure");
    const auto rows = rt->recorded_intervals();
    bt::require(!rows.empty() && rows.back()->qpos[0] >= 60.0, "normal rider travels 60 m");
    for (const auto &row : rows) {
        bt::require(research::model_violations(*row).empty(), "no model violations");
        const auto &tires = sw::object(sw::required(row->channels, "tires"));
        for (const char *side : {"front", "rear"}) {
            const auto &wheel = sw::object(sw::required(tires, side));
            bt::require(sw::number(sw::required(wheel, "dissipated_power_w")) >= 0.0, "nonnegative power");
            bt::require(sw::number(sw::required(wheel, "brush_loss_j")) >= 0.0, "nonnegative brush loss");
        }
    }
    const auto &energy = sw::object(sw::required(rows.back()->channels, "energy"));
    const auto &elastic = sw::object(sw::required(energy, "elastic_energy_j"));
    bt::require(sw::find(elastic, "soil") == nullptr,
                "rigid ride omits soil from elastic_energy_j");
    bt::require(sw::number(sw::required(energy, "ground_plastic_loss_step_j")) == 0.0,
                "rigid ride reports zero ground_plastic_loss_step_j");
    const auto eq = research::energy_quality(energy, 0.05);
    bt::require(eq.acceptable && eq.residual_ratio <= 0.05,
                "60 m mixed-surface pneumatic ride satisfies energy_quality <= 0.05");
}

// ---- Stage C: Roots obstacle carrying wet_wood micro spans (Spec §3) -------

// The dispatch-pinned roots geometry: four 0.45 m roots over a 4 m
// section starting at x = 12 m, each wrapped in a wet_wood micro span
// with a 2 cm Hermite collar, on a flat hardpack track. The track comes
// through the same TOML pipeline the production setup consumes.
[[nodiscard]] bike::terrain::TrackSpec stage_c_roots_track() {
    return bike::terrain::track_from_dict(bike::terrain::parse_track_toml(
            "name = \"stage_c_roots\"\nlength_m = 40.0\nsurface = \"hardpack\"\n"
            "grade_profile = { knots = [[0.0, 0.0], [40.0, 0.0]] }\n"
            "[[obstacles]]\ntype = \"roots\"\nstart_m = 12.0\nn_bumps = 4\n"
            "height_m = 0.025\nwidth_m = 0.45\nsection_length_m = 4.0\n"
            "jitter = 0.1\nseed = 7\n"
            "surface = \"wet_wood\"\nsurface_blend_m = 0.02\n"));
}

[[nodiscard]] nativecfg::SurfaceSpec wet_wood_spec() {
    const bike::terrain::SurfaceSpec &s =
            bike::terrain::get_obstacle_surface("wet_wood");
    return {.name = s.name, .mu_peak = s.mu_peak, .mu_slide = s.mu_slide,
            .slip_stiffness_per_load = s.slip_stiffness_per_load,
            .stribeck_speed_mps = s.stribeck_speed_mps,
            .crr_delta = s.crr_delta, .load_sensitivity = s.load_sensitivity};
}

// Two wet_wood lenses inside the flat stands' [-2, 2] road window:
// every crossing is hardpack -> wet_wood -> hardpack with 2 cm collars.
// `sorted` controls the authored wire order — SurfaceMap validation
// accepts either, and the writers sort at construction.
[[nodiscard]] nativecfg::SurfaceMap stage_c_writer_map(bool sorted = true) {
    const nativecfg::SurfaceSpec wood = wet_wood_spec();
    nativecfg::SurfaceMap map{.surface = surface("hardpack"), .sections = {}};
    map.micro_spans = {
            {.start_m = 0.50, .end_m = 0.70, .blend_m = 0.02, .surface = wood},
            {.start_m = 0.95, .end_m = 1.15, .blend_m = 0.02,
             .surface = wood}};
    if (!sorted) std::ranges::reverse(map.micro_spans);
    return map;
}

struct StageCDataDeleter {
    void operator()(mjData *data) const noexcept {
        if (data != nullptr) mj_deleteData(data);
    }
};

struct StageCStand {
    bike::model::ModelOwner model;
    std::unique_ptr<mjData, StageCDataDeleter> data;
    explicit StageCStand(std::string_view xml)
        : model(bike::model::compile_mjcf(xml)),
          data(engine::make_data(model.get())) {
        engine::forward(model.get(), data.get());
    }
};

[[nodiscard]] std::unique_ptr<StageCStand> stage_c_flat_stand(double dt) {
    const std::string xml =
            R"(<mujoco><compiler angle="radian"/><option timestep=")" +
            std::format("{:.17g}", dt) +
            R"(" gravity="0 0 0" integrator="Euler"/>
      <asset><hfield name="road" nrow="2" ncol="801" size="2 1 0.01 0.2"/></asset>
      <worldbody>
        <geom name="terrain" type="hfield" hfield="road" pos="0 0 0" contype="0" conaffinity="0"/>
        <body name="front_wheel" pos="0 0 0.37">
          <joint name="root_x" type="slide" axis="1 0 0"/>
          <joint name="root_z" type="slide" axis="0 0 1"/>
          <joint name="front_wheel_spin" type="hinge" axis="0 1 0"/>
          <inertial mass="2.4" pos="0 0 0" diaginertia="0.15 0.30 0.15"/>
          <geom name="geom_front_contact" type="cylinder" size="0.37 0.025"
                euler="1.5707963267948966 0 0" contype="0" conaffinity="0" mass="0"/>
        </body>
        <body name="rear_wheel" pos="0 0 2.0">
          <geom name="geom_rear_contact" type="sphere" size="0.37" contype="0" conaffinity="0"/>
        </body>
      </worldbody>
    </mujoco>)";
    return std::make_unique<StageCStand>(xml);
}

// Same single-wheel flat-road stand, but the hfield geom is shifted so its
// local [-4, 4] window lands on world x in [10, 18] — covering the Roots
// track's authored wet_wood micro spans (~12.3..15.3 m) at their true
// coordinates, so the SurfaceMap needs no translation. contype/conaffinity
// stay 0: the tire writer raycasts the field itself.
[[nodiscard]] std::unique_ptr<StageCStand> stage_c_roots_stand(double dt) {
    const std::string xml =
            R"(<mujoco><compiler angle="radian"/><option timestep=")" +
            std::format("{:.17g}", dt) +
            R"(" gravity="0 0 0" integrator="Euler"/>
      <asset><hfield name="road" nrow="2" ncol="1601" size="4 1 0.01 0.2"/></asset>
      <worldbody>
        <geom name="terrain" type="hfield" hfield="road" pos="14 0 0" contype="0" conaffinity="0"/>
        <body name="front_wheel" pos="0 0 0.37">
          <joint name="root_x" type="slide" axis="1 0 0"/>
          <joint name="root_z" type="slide" axis="0 0 1"/>
          <joint name="front_wheel_spin" type="hinge" axis="0 1 0"/>
          <inertial mass="2.4" pos="0 0 0" diaginertia="0.15 0.30 0.15"/>
          <geom name="geom_front_contact" type="cylinder" size="0.37 0.025"
                euler="1.5707963267948966 0 0" contype="0" conaffinity="0" mass="0"/>
        </body>
        <body name="rear_wheel" pos="14 0 2.0">
          <geom name="geom_rear_contact" type="sphere" size="0.37" contype="0" conaffinity="0"/>
        </body>
      </worldbody>
    </mujoco>)";
    return std::make_unique<StageCStand>(xml);
}

[[nodiscard]] nativecfg::TireConfig
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) helper signature keeps cfg-field order
stage_c_distributed_config(nativecfg::SurfaceMap surface_map, int station_count,
                           double tire_mu) {
    const nativecfg::HingeDensity density{
        .knots_m = {0.0, 0.004, 0.012},
        .stiffness_n_m = {300000.0, 0.0, 0.0},
        .damping_ns_m = 0.0,
        .loss_factor = 0.05,
        .hysteresis_rate_eps_mps = 0.01,
        .provenance = "synthetic density; not fitted or measured",
    };
    const nativecfg::DistributedMaterial mat{
        .density = density,
        .fitting_dataset_id = "none-synthetic",
        .valid_load_range_n = {0.0, 2000.0},
        .valid_deflection_range_m = {0.0, 0.04},
        .calibration_status = "experimental_unvalidated",
    };
    nativecfg::TireConfig cfg;
    cfg.backend = "distributed_2d_reference";
    cfg.surface_mode = "track";
    cfg.front = {.material = std::nullopt, .tangent_k_n_m = 20000.0,
                 .mu = tire_mu, .relaxation_length_m = 0.2};
    cfg.rear = cfg.front;
    cfg.significant_delta_m = 0.0001;
    cfg.significance_fraction = 0.05;
    cfg.distinct_normal_deg = 20.0;
    cfg.surface_map = std::move(surface_map);
    cfg.distributed = nativecfg::DistributedConfig{
        .station_count = station_count,
        .density = density,
        .front = mat,
        .rear = mat,
    };
    return cfg;
}

[[nodiscard]] double stage_c_solve_depth(const StageCStand &stand,
                                         DistributedTireWriter &tire,
                                         // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) signature keeps probe order
                                         double dt, double target_load_n,
                                         double wheel_x_m = 0.0) {
    const auto *m = stand.model.get();
    auto *d = stand.data.get();
    const auto qpos = model_access::mutable_buffer(d->qpos, m->nq, "qpos");
    const auto qvel = model_access::mutable_buffer(d->qvel, m->nv, "qvel");
    std::ranges::fill(qpos, 0.0);
    std::ranges::fill(qvel, 0.0);
    // The probe must sit over the stand's road window — the roots stand's
    // hfield covers world x in [10, 18], not the origin.
    qpos[0] = wheel_x_m;
    double lo = 1e-7, hi = 0.06;
    for (int iter = 0; iter < 60; ++iter) {
        const double mid = 0.5 * (lo + hi);
        qpos[1] = -mid;
        engine::forward(m, d);
        const double fz = tire.probe_qfrc(d, dt)[1];
        if (fz < target_load_n) lo = mid; else hi = mid;
    }
    qpos[1] = 0.0;
    engine::forward(m, d);
    return 0.5 * (lo + hi);
}

// Recorded-interval friction trace, one per wheel: the minimum loaded
// friction_coefficient sampled while the wheel axis crossed each wet_wood
// span and the maximum sampled in the hardpack gaps between roots. Bins
// follow the recorded wheel_axis_m; only loaded, non-airborne rows count,
// so a skipped contact never masquerades as a friction dip.
struct StageCWireFriction {
    std::array<double, 4> wood_min{1e9, 1e9, 1e9, 1e9};
    std::array<double, 3> gap_max{};
    std::array<int, 4> wood_samples{};
    std::array<int, 3> gap_samples{};

    void observe(const runtime::WireObject &wheel,
                 const std::vector<bike::terrain::Obstacle::MicroSpan> &spans) {
        namespace sw = runtime::sample_wire;
        if (sw::boolean(sw::required(wheel, "airborne"))) return;
        if (sw::number(sw::required(wheel, "normal_load_n")) <= 1.0) return;
        const auto &axis = sw::array(sw::required(wheel, "wheel_axis_m"));
        const double x = sw::number(axis[0]);
        const std::string &surface_name =
                sw::string(sw::required(wheel, "surface"));
        const double mu =
                sw::number(sw::required(wheel, "friction_coefficient"));
        for (std::size_t i = 0; i < spans.size(); ++i) {
            const auto &span = spans[i];
            if (x >= span.start_m + 0.5 * span.blend_m &&
                x < span.end_m - 0.5 * span.blend_m &&
                surface_name == "wet_wood") {
                wood_min[i] = std::min(wood_min[i], mu);
                ++wood_samples[i];
            }
            if (i + 1 < spans.size() &&
                x >= span.end_m + 0.5 * span.blend_m &&
                x < spans[i + 1].start_m - 0.5 * spans[i + 1].blend_m &&
                surface_name == "hardpack") {
                gap_max[i] = std::max(gap_max[i], mu);
                ++gap_samples[i];
            }
        }
    }
    void check() const {
        for (const int samples : gap_samples)
            bt::require(samples > 0,
                        "loaded contact sampled in every dirt gap");
        for (std::size_t i = 0; i < wood_samples.size(); ++i) {
            bt::require(wood_samples[i] > 0,
                        "loaded contact sampled on every root");
            if (i > 0)
                bt::require(wood_min[i] < gap_max[i - 1],
                            "friction drops from preceding gap onto root");
            if (i < gap_samples.size())
                bt::require(wood_min[i] < gap_max[i],
                            "friction recovers from root into next gap");
        }
    }
};

[[gnu::noinline]] void test_roots_wet_wood_micro_span_ride_energy_gate() {
    namespace research = bike::research;
    namespace sw = runtime::sample_wire;
    const auto root =
            bt::oracle::root().parent_path().parent_path().parent_path();
    const bike::terrain::TrackSpec track = stage_c_roots_track();
    bt::require(track.obstacles.size() == 1,
                "roots track carries the roots obstacle");
    const auto spans = track.obstacles.front().micro_surface_spans();
    bt::require(spans.size() == 4,
                "roots obstacle generates four micro spans");

    // The production Stage A harness — prepare_environment +
    // NativeRideRuntime, whose PhysicalStep trapezoidal qvel_mid
    // accounting feeds research::energy_quality — at both pneumatic
    // tiers. The detailed tier needs the 0.25 ms step; the 1.25 ms
    // physics control period then stays five whole steps (dispatch:
    // timestep_s = 0.00025, period_s = 0.00125).
    for (const std::string_view tier_name : {"fast", "detailed"}) {
        auto owner = std::make_unique<research::ResearchRequest>();
        auto &request = *owner;
        request.setup.model.physics = bike::configuration::load_physics(
                root / "examples/research/viewer_physics_pneumatic.toml");
        if (tier_name == "detailed") {
            request.setup.model.physics->timestep_s = 0.00025;
            request.setup.model.physics->control_period_s = 0.00125;
            request.setup.model.physics->tires.pneumatic.tier = "detailed";
        }
        request.setup.model.mode = "ride";
        request.setup.model.crank_joint = true;
        request.setup.model.rider =
                bike::model::rider_from_args("articulated_planar");
        request.setup.track = track;
        request.experiment.duration_s = 18.0;
        request.experiment.record_decimation = 1;
        request.experiment.stop_on_model_violation = true;
        request.experiment.control_period_s =
                request.setup.model.physics->research.control_period_s;
        request.experiment.actuator_delay_s =
                request.setup.model.physics->research.actuator_delay_s;
        request.sensors.delivery.latency_s =
                request.setup.model.physics->research.sensor_latency_s;
        request.sensors.delivery.maximum_age_s =
                request.setup.model.physics->research.sensor_maximum_age_s;
        request.sensors.sample_period_s = 0.005;
        auto prepared = research::prepare_environment(std::move(owner));
        const auto period = prepared->bootstrap.config.control_steps;
        const auto max_steps = prepared->bootstrap.config.max_steps;
        auto rt = std::make_unique<runtime::NativeRideRuntime>(
                std::move(prepared->bootstrap.ride));
        // The rear hub trails the frame origin by about half a metre:
        // stopping at x = 19 m carries both wheels past the last root,
        // whose micro span ends near 15.4 m.
        while (!rt->snapshot().outcome.has_value() &&
               rt->snapshot().step < max_steps) {
            const auto result =
                    rt->advance(rt->snapshot().step + period,
                                runtime::RideControl{}, 0., 0., std::nullopt);
            if (!rt->recorded_intervals().empty() &&
                rt->recorded_intervals().back()->qpos[0] >= 19.0)
                break;
            if (result.reason == "outcome") break;
        }
        const auto snapshot = rt->snapshot();
        bt::require(!snapshot.first_failure.has_value(),
                    "roots ride has no first failure");
        const auto rows = rt->recorded_intervals();
        bt::require(!rows.empty() && rows.back()->qpos[0] >= 17.0,
                    "normal rider carries both wheels past the roots");
        std::array<StageCWireFriction, 2> friction;
        for (const auto &row : rows) {
            bt::require(research::model_violations(*row).empty(),
                        "no model violations on the roots ride");
            const auto &tires =
                    sw::object(sw::required(row->channels, "tires"));
            for (std::size_t s = 0; s < 2; ++s) {
                const auto &wheel = sw::object(sw::required(
                        tires, s == 0 ? "front" : "rear"));
                bt::require(sw::number(sw::required(
                                    wheel, "dissipated_power_w")) >= 0.0,
                            "nonnegative tire power on roots");
                bt::require(sw::number(sw::required(
                                    wheel, "brush_loss_j")) >= 0.0,
                            "nonnegative brush loss on roots");
                friction[s].observe(wheel, spans);
            }
        }
        for (const auto &trace : friction) trace.check();
        const auto &energy =
                sw::object(sw::required(rows.back()->channels, "energy"));
        const auto eq = research::energy_quality(energy, 0.05);
        bt::require(eq.acceptable && eq.residual_ratio <= 0.05,
                    "roots micro-span ride satisfies energy_quality <= 0.05");
    }
}

// The committed try_compute_into warm path over a micro-span crossing
// performs zero heap allocations for both backends; the surface-name
// buffers are reserved to >= 16 chars at construction (Spec §3.3).
[[gnu::noinline]] void test_micro_span_writer_zero_allocation() {
    const nativecfg::SurfaceMap map = stage_c_writer_map();
    for (const std::string_view tier_name : {"fast", "detailed"}) {
        constexpr double dt = 0.00005;
        bt::pneumatic::Rig rig(tier_name, dt, 0.008);
        auto cfg = bt::pneumatic::configuration(tier_name);
        cfg.surface_mode = "track";
        if (!cfg.pneumatic.has_value())
            throw std::logic_error("fixture needs pneumatic configuration");
        cfg.pneumatic->surface.reset();
        const auto x_grid = bike::geometry::detail::np_linspace(-1.0, 4.0, 1001);
        cfg.pneumatic->profile = {
                .x0_m = x_grid.front(), .dx_m = x_grid[1] - x_grid[0],
                .z_m = std::vector<double>(x_grid.size(), 0.0)};
        cfg.surface_map = map;
        rig.tire = std::make_unique<PneumaticTireWriter>(
                rig.engine.model.get(), rig.engine.data.get(), cfg);
        bt::require(rig.tire->diagnostic_storage()[0].surface.capacity() >=
                            std::size_t{16} &&
                        rig.tire->diagnostic_storage()[1].surface.capacity() >=
                            std::size_t{16},
                    "micro-span track reserves >= 16 surface-name capacity");

        const auto *m = rig.engine.model.get();
        auto *d = rig.engine.data.get();
        const auto qpos = model_access::mutable_buffer(d->qpos, m->nq, "qpos");
        const auto qvel = model_access::mutable_buffer(d->qvel, m->nv, "qvel");
        const auto applied = model_access::mutable_buffer(
                d->qfrc_applied, m->nv, "qfrc_applied");
        qpos[0] = -0.2;
        qvel[0] = 2.0;
        qvel[1] = 0.0;
        qvel[2] = 2.0 / (bt::pneumatic::front_radius() - 0.002);
        qvel[3] = 0.0;
        engine::forward(m, d);
        std::vector<double> out(static_cast<std::size_t>(m->nv), 0.0);
        // Settle carcass and brush on the hardpack approach before the
        // measured window opens.
        for (int step = 0; step < 4000 && qpos[0] < 0.2; ++step) {
            bt::require(rig.tire->try_compute_into(d, dt, out) ==
                                CoreStatus::ok,
                        "warmup try_compute_into succeeds");
            std::ranges::copy(out, applied.begin());
            engine::step(m, d);
            engine::forward(m, d);
        }
        bt::require(qpos[0] <= 0.2, "warmup stays on the hardpack approach");
        bool saw_wood = false, saw_dirt_after = false;
        {
            allocation_counter::reset();
            for (int step = 0; step < 30000 && qpos[0] < 1.4; ++step) {
                bt::require(rig.tire->try_compute_into(d, dt, out) ==
                                    CoreStatus::ok,
                            "measured try_compute_into succeeds");
                // Ruling 2: pneumatic stored/elastic energies are the same
                // fold — bitwise equality, asserted while the allocation
                // counter is armed (stored_energy binds stack RayHitBuffers
                // to preallocated scratch, so the zero-allocation gate
                // still holds).
                bt::require(rig.tire->stored_energy(d) ==
                                    rig.tire->elastic_energy_j(),
                            "pneumatic stored_energy bitwise equals "
                            "elastic_energy_j after compute");
                const std::string &name =
                        rig.tire->diagnostic_storage()[0].surface;
                if (name == "wet_wood") saw_wood = true;
                if (saw_wood && qpos[0] > 1.2 && name == "hardpack")
                    saw_dirt_after = true;
                std::ranges::copy(out, applied.begin());
                engine::step(m, d);
                engine::forward(m, d);
                bt::require(rig.tire->brush_loss_step_j() >= 0.0 &&
                                rig.tire->radial_dissipation_power_w() >= 0.0,
                            "nonnegative dissipation across micro spans");
            }
            bt::require(allocation_counter::count() == std::size_t{0},
                        "try_compute_into warm path allocates nothing over "
                        "micro spans");
            bt::require(qpos[0] >= 1.4,
                        "pneumatic wheel rolls across all micro spans");
        }
        bt::require(saw_wood && saw_dirt_after,
                    "wheel crossed wet_wood inside the measured window");
    }

    // DistributedTireWriter leg: the same micro-span map under an applied
    // 480 N load on the flat stand.
    {
        constexpr double dt = 0.0001;
        constexpr int stations = 64;
        constexpr double load = 480.0;
        auto ws = stage_c_flat_stand(dt);
        const auto *m = ws->model.get();
        auto *d = ws->data.get();
        auto tire = std::make_unique<DistributedTireWriter>(
                m, d, stage_c_distributed_config(map, stations, 2.0));
        bt::require(tire->diagnostic_storage()[0].surface.capacity() >=
                            std::size_t{16} &&
                        tire->diagnostic_storage()[1].surface.capacity() >=
                            std::size_t{16},
                    "distributed micro-span reserves >= 16 surface-name "
                    "capacity on both wheels");
        const double depth = stage_c_solve_depth(*ws, *tire, dt, load);
        const auto qpos = model_access::mutable_buffer(d->qpos, m->nq, "qpos");
        const auto qvel = model_access::mutable_buffer(d->qvel, m->nv, "qvel");
        const auto applied = model_access::mutable_buffer(
                d->qfrc_applied, m->nv, "qfrc_applied");
        qpos[0] = 0.05;
        qpos[1] = -depth;
        qpos[2] = 0.0;
        qvel[0] = 2.0;
        qvel[1] = 0.0;
        qvel[2] = 2.0 / (0.37 - depth);
        engine::forward(m, d);
        std::vector<double> out(static_cast<std::size_t>(m->nv), 0.0);
        bool saw_wood = false, saw_dirt_after = false;
        {
            allocation_counter::reset();
            for (int step = 0; step < 20000 && qpos[0] < 1.4; ++step) {
                bt::require(tire->try_compute_into(d, dt, out) ==
                                    CoreStatus::ok,
                            "distributed try_compute_into succeeds");
                // stored_energy keeps its oracle-pinned side-major
                // radial-then-shear fold (distributed_tire_forces.py
                // :117-129); elastic_energy_j folds radial+shear per side
                // inside accumulate (:204-214). Both orders are pinned by
                // the oracle, so multi-station states differ by binary64
                // ULPs — the parity bound is 1e-12, never a relaxed
                // shared order.
                bt::require(std::abs(tire->stored_energy(d) -
                                     tire->elastic_energy_j()) <= 1e-12,
                            "distributed stored-energy parity within 1e-12");
                const std::string &name =
                        tire->diagnostic_storage()[0].surface;
                if (name == "wet_wood") saw_wood = true;
                if (saw_wood && qpos[0] > 1.2 && name == "hardpack")
                    saw_dirt_after = true;
                std::ranges::copy(out, applied.begin());
                applied[1] -= load;
                engine::step(m, d);
                engine::forward(m, d);
                bt::require(tire->brush_loss_step_j() >= 0.0 &&
                                tire->radial_dissipation_power_w() >= 0.0,
                            "nonnegative distributed dissipation");
            }
            bt::require(allocation_counter::count() == std::size_t{0},
                        "distributed try_compute_into allocates nothing "
                        "over micro spans");
            bt::require(qpos[0] >= 1.4,
                        "distributed wheel rolls across all micro spans");
        }
        bt::require(saw_wood && saw_dirt_after,
                    "distributed wheel crossed wet_wood inside the window");
    }
}

// Unsorted-wire regression (ruling 3): SurfaceMap validation accepts a
// micro_spans list in any order; both writers sort at construction, so
// an unsorted wire must behave bitwise-identically to pre-sorted spans.
// The config factories confine each TireConfig to the ctor call — two
// live copies of the wide config struct would exceed the test frame.
[[nodiscard]] nativecfg::TireConfig stage_c_pneumatic_cfg(bool sorted) {
    auto cfg = bt::pneumatic::configuration("fast");
    cfg.surface_mode = "track";
    if (!cfg.pneumatic.has_value())
        throw std::logic_error("fixture needs pneumatic configuration");
    cfg.pneumatic->surface.reset();
    cfg.surface_map = stage_c_writer_map(sorted);
    return cfg;
}

[[gnu::noinline]] void test_micro_span_unsorted_wire_equivalent() {
    {
        constexpr double dt = 0.0001;
        const bt::pneumatic::Rig rig("fast", dt, 0.008);
        const auto sorted = std::make_unique<PneumaticTireWriter>(
                rig.engine.model.get(), rig.engine.data.get(),
                stage_c_pneumatic_cfg(true));
        const auto unsorted = std::make_unique<PneumaticTireWriter>(
                rig.engine.model.get(), rig.engine.data.get(),
                stage_c_pneumatic_cfg(false));
        const auto *m = rig.engine.model.get();
        auto *d = rig.engine.data.get();
        const auto qpos = model_access::mutable_buffer(d->qpos, m->nq, "qpos");
        const auto qvel = model_access::mutable_buffer(d->qvel, m->nv, "qvel");
        qvel[0] = 2.0;
        qvel[2] = 2.0 / (bt::pneumatic::front_radius() - 0.002);
        std::vector<double> a(static_cast<std::size_t>(m->nv), 0.0),
                b(static_cast<std::size_t>(m->nv), 0.0);
        // Approach, both collars, span interiors, gap interior and the
        // hardpack beyond — every eval_surface branch.
        for (const double x : {0.30, 0.49, 0.51, 0.60, 0.69, 0.71, 0.80,
                               0.94, 1.05, 1.16, 1.30}) {
            qpos[0] = x;
            d->time += dt;
            engine::forward(m, d);
            bt::require(sorted->try_compute_into(d, dt, a) == CoreStatus::ok,
                        "sorted writer computes");
            bt::require(unsorted->try_compute_into(d, dt, b) == CoreStatus::ok,
                        "unsorted writer computes");
            bitwise_equal(a, b);
            bt::require(sorted->stored_energy(d) ==
                            unsorted->stored_energy(d) &&
                            sorted->elastic_energy_j() ==
                                    unsorted->elastic_energy_j(),
                        "unsorted wire energies bitwise identical");
            for (std::size_t s = 0; s < 2; ++s) {
                const TireDiagnostics &da = sorted->diagnostic_storage()[s];
                const TireDiagnostics &db = unsorted->diagnostic_storage()[s];
                bt::require(da.surface == db.surface &&
                                da.friction_coefficient ==
                                        db.friction_coefficient &&
                                da.normal_load_n == db.normal_load_n &&
                                da.shear_energy_j == db.shear_energy_j &&
                                da.radial_energy_j == db.radial_energy_j,
                            "unsorted wire diagnostics bitwise identical");
            }
        }
    }

    {
        constexpr double dt = 0.0001;
        constexpr double load = 480.0;
        auto ws = stage_c_flat_stand(dt);
        const auto *m = ws->model.get();
        auto *d = ws->data.get();
        auto sorted = std::make_unique<DistributedTireWriter>(
                m, d, stage_c_distributed_config(
                             stage_c_writer_map(true), 64, 2.0));
        const double depth = stage_c_solve_depth(*ws, *sorted, dt, load);
        auto unsorted = std::make_unique<DistributedTireWriter>(
                m, d, stage_c_distributed_config(
                             stage_c_writer_map(false), 64, 2.0));
        const auto qpos = model_access::mutable_buffer(d->qpos, m->nq, "qpos");
        const auto qvel = model_access::mutable_buffer(d->qvel, m->nv, "qvel");
        qpos[1] = -depth;
        qvel[0] = 2.0;
        qvel[2] = 2.0 / (0.37 - depth);
        std::vector<double> a(static_cast<std::size_t>(m->nv), 0.0),
                b(static_cast<std::size_t>(m->nv), 0.0);
        for (const double x : {0.30, 0.60, 0.80, 1.05, 1.30}) {
            qpos[0] = x;
            d->time += dt;
            engine::forward(m, d);
            bt::require(sorted->try_compute_into(d, dt, a) == CoreStatus::ok,
                        "sorted distributed writer computes");
            bt::require(unsorted->try_compute_into(d, dt, b) ==
                                CoreStatus::ok,
                        "unsorted distributed writer computes");
            bitwise_equal(a, b);
            bt::require(sorted->elastic_energy_j() ==
                            unsorted->elastic_energy_j(),
                        "unsorted distributed wire energy bitwise identical");
            for (std::size_t s = 0; s < 2; ++s) {
                const TireDiagnostics &da = sorted->diagnostic_storage()[s];
                const TireDiagnostics &db = unsorted->diagnostic_storage()[s];
                bt::require(da.surface == db.surface &&
                                da.friction_coefficient ==
                                        db.friction_coefficient &&
                                da.normal_load_n == db.normal_load_n &&
                                da.shear_energy_j == db.shear_energy_j &&
                                da.radial_energy_j == db.radial_energy_j,
                            "unsorted distributed diagnostics identical");
            }
        }
    }
}

// Stage C Ruling 2: DistributedTireWriter diagnostics over the Roots
// track's own wet_wood micro spans at their authored coordinates. The
// distributed runtime wire intentionally omits
// friction_coefficient/surface/airborne
// (wire_distributed_tire_diagnostics), so the per-root friction dip and
// hardpack-gap recovery are read from diagnostic_storage() directly. The
// stand's hfield window covers world x in [10, 18], so the SurfaceMap
// carries the spans verbatim — no translation, identical eval_surface
// classification to the ride.
[[gnu::noinline]] void test_distributed_roots_wet_wood_diagnostics() {
    const auto track = stage_c_roots_track();
    const auto authored = track.obstacles.front().micro_surface_spans();
    bt::require(authored.size() == 4,
                "roots track authors four wet_wood micro spans");
    for (const auto &s : authored)
        bt::require(s.surface == "wet_wood" && s.blend_m == 0.02,
                    "authored root spans carry wet_wood and 2 cm collars");
    nativecfg::SurfaceMap map{.surface = surface("hardpack"), .sections = {}};
    for (const auto &s : authored)
        map.micro_spans.push_back(
                {.start_m = s.start_m,
                 .end_m = s.end_m,
                 .blend_m = s.blend_m,
                 .surface = wet_wood_spec()});

    constexpr double dt = 0.0001;
    constexpr int stations = 64;
    constexpr double load = 480.0;
    auto ws = stage_c_roots_stand(dt);
    const auto *m = ws->model.get();
    auto *d = ws->data.get();
    auto tire = std::make_unique<DistributedTireWriter>(
            m, d, stage_c_distributed_config(map, stations, 2.0));
    bt::require(tire->diagnostic_storage()[0].surface.capacity() >=
                        std::size_t{16} &&
                    tire->diagnostic_storage()[1].surface.capacity() >=
                        std::size_t{16},
                "distributed reserves >= 16 surface-name capacity on both "
                "wheels");
    // Probe on the hardpack approach ahead of the first root span.
    const double depth = stage_c_solve_depth(
            *ws, *tire, dt, load, map.micro_spans.front().start_m - 0.7);
    const auto qpos = model_access::mutable_buffer(d->qpos, m->nq, "qpos");
    const auto qvel = model_access::mutable_buffer(d->qvel, m->nv, "qvel");
    const auto applied = model_access::mutable_buffer(
            d->qfrc_applied, m->nv, "qfrc_applied");
    qpos[0] = map.micro_spans.front().start_m - 0.45;
    qpos[1] = -depth;
    qpos[2] = 0.0;
    // Rolling resistance bleeds ~0.6 m/s^2 on this rig: 2 m/s stalls
    // before the last root, so launch fast enough to cross all four.
    qvel[0] = 4.0;
    qvel[1] = 0.0;
    qvel[2] = 4.0 / (0.37 - depth);
    engine::forward(m, d);
    std::vector<double> out(static_cast<std::size_t>(m->nv), 0.0);
    // Per-root minimum and per-gap maximum loaded friction on the rolling
    // wheel — the distributed analogue of StageCWireFriction.
    constexpr std::size_t roots = 4;
    std::array<double, roots> wood_min{};
    std::array<double, roots - 1> gap_max{};
    std::array<int, roots> wood_n{};
    std::array<int, roots - 1> gap_n{};
    wood_min.fill(1e9);
    const double stop_x = map.micro_spans.back().end_m + 0.45;
    for (int step = 0; step < 60000 && qpos[0] < stop_x; ++step) {
        bt::require(tire->try_compute_into(d, dt, out) == CoreStatus::ok,
                    "distributed roots try_compute_into succeeds");
        // The distributed folds differ by binary64 ULPs by design —
        // keep the oracle-pinned 1e-12 parity bound, never bitwise.
        bt::require(std::abs(tire->stored_energy(d) -
                             tire->elastic_energy_j()) <= 1e-12,
                    "distributed stored-energy parity within 1e-12");
        bt::require(tire->brush_loss_step_j() >= 0.0,
                    "nonnegative distributed brush loss on roots");
        const TireDiagnostics &diag = tire->diagnostic_storage()[0];
        if (diag.normal_load_n > 1.0) {
            const double x = qpos[0];
            for (std::size_t i = 0; i < roots; ++i) {
                const auto &span = map.micro_spans[i];
                const double half = 0.5 * span.blend_m;
                if (x >= span.start_m + half && x < span.end_m - half &&
                    diag.surface == "wet_wood") {
                    wood_min[i] =
                            std::min(wood_min[i], diag.friction_coefficient);
                    ++wood_n[i];
                }
                if (i + 1 < roots) {
                    const auto &next = map.micro_spans[i + 1];
                    const double nhalf = 0.5 * next.blend_m;
                    if (x >= span.end_m + half &&
                        x < next.start_m - nhalf &&
                        diag.surface == "hardpack") {
                        gap_max[i] = std::max(gap_max[i],
                                              diag.friction_coefficient);
                        ++gap_n[i];
                    }
                }
            }
        }
        std::ranges::copy(out, applied.begin());
        applied[1] -= load;
        engine::step(m, d);
        engine::forward(m, d);
    }
    bt::require(qpos[0] >= stop_x,
                "distributed wheel crosses all four root spans");
    for (std::size_t i = 0; i < roots - 1; ++i)
        bt::require(gap_n[i] > 0,
                    "loaded contact sampled in every dirt gap");
    for (std::size_t i = 0; i < roots; ++i) {
        bt::require(wood_n[i] > 0,
                    "loaded contact sampled on every root");
        if (i > 0)
            bt::require(wood_min[i] < gap_max[i - 1],
                        "friction drops from preceding gap onto root");
        if (i < roots - 1)
            bt::require(wood_min[i] < gap_max[i],
                        "friction recovers from root into next gap");
    }
}

// Stage C Ruling 2 ride leg: viewer_physics_distributed.toml on the
// Roots wet_wood track through the same prepare_environment +
// NativeRideRuntime harness the pneumatic gate uses. The distributed
// wire omits friction_coefficient/surface, so this leg asserts energy
// quality, model-violation freedom and nonnegative brush loss; the
// per-root friction dip lives in the diagnostics test above.
[[gnu::noinline]] void test_distributed_roots_wet_wood_ride_energy_gate() {
    namespace research = bike::research;
    namespace sw = runtime::sample_wire;
    const auto root =
            bt::oracle::root().parent_path().parent_path().parent_path();
    const bike::terrain::TrackSpec track = stage_c_roots_track();
    auto owner = std::make_unique<research::ResearchRequest>();
    auto &request = *owner;
    request.setup.model.physics = bike::configuration::load_physics(
            root / "examples/research/viewer_physics_distributed.toml");
    request.setup.model.mode = "ride";
    request.setup.model.crank_joint = true;
    request.setup.model.rider =
            bike::model::rider_from_args("articulated_planar");
    request.setup.track = track;
    request.experiment.duration_s = 18.0;
    request.experiment.record_decimation = 1;
    request.experiment.stop_on_model_violation = true;
    request.experiment.control_period_s =
            request.setup.model.physics->research.control_period_s;
    request.experiment.actuator_delay_s =
            request.setup.model.physics->research.actuator_delay_s;
    request.sensors.delivery.latency_s =
            request.setup.model.physics->research.sensor_latency_s;
    request.sensors.delivery.maximum_age_s =
            request.setup.model.physics->research.sensor_maximum_age_s;
    request.sensors.sample_period_s = 0.005;
    auto prepared = research::prepare_environment(std::move(owner));
    const auto period = prepared->bootstrap.config.control_steps;
    const auto max_steps = prepared->bootstrap.config.max_steps;
    auto rt = std::make_unique<runtime::NativeRideRuntime>(
            std::move(prepared->bootstrap.ride));
    while (!rt->snapshot().outcome.has_value() &&
           rt->snapshot().step < max_steps) {
        const auto result =
                rt->advance(rt->snapshot().step + period,
                            runtime::RideControl{}, 0., 0., std::nullopt);
        if (!rt->recorded_intervals().empty() &&
            rt->recorded_intervals().back()->qpos[0] >= 19.0)
            break;
        if (result.reason == "outcome") break;
    }
    const auto snapshot = rt->snapshot();
    bt::require(!snapshot.first_failure.has_value(),
                "distributed roots ride has no first failure");
    const auto rows = rt->recorded_intervals();
    bt::require(!rows.empty() && rows.back()->qpos[0] >= 17.0,
                "rider carries both wheels past the roots");
    for (const auto &row : rows) {
        bt::require(research::model_violations(*row).empty(),
                    "no model violations on the distributed roots ride");
        const auto &tires =
                sw::object(sw::required(row->channels, "tires"));
        for (std::size_t s = 0; s < 2; ++s) {
            const auto &wheel = sw::object(sw::required(
                    tires, s == 0 ? "front" : "rear"));
            bt::require(sw::number(sw::required(
                                    wheel, "brush_loss_j")) >= 0.0,
                            "nonnegative brush loss on distributed roots");
        }
    }
    const auto &energy =
            sw::object(sw::required(rows.back()->channels, "energy"));
    const auto eq = research::energy_quality(energy, 0.05);
    bt::require(eq.acceptable && eq.residual_ratio <= 0.05,
                "distributed roots micro-span ride satisfies "
                "energy_quality <= 0.05");
}

// ---- Stage B: deformable granular layer (layer_thickness_m) -------------
//
// RutMemory is the world-fixed 1D soil grid the pneumatic writer engages
// when a loose/scree section carries a positive layer depth. These legs
// pin the writer contract — profile-sized serialized capacity, the
// front-then-rear multipass commit, probe purity, the bedrock sinkage
// bound and the allocation-free warm path — then the full ride gate.

// Two-wheel pneumatic stand on a flat scree layer: the front wheel
// slides on x/z joints, the rear wheel is world-fixed at x = 0.8 — both
// footprints land inside the [0.5, 4.0] scree section so every committed
// pass deposits into the same 1D rut grid. The profile is deliberately
// longer than the retired 8192-slot wire cap so the serialized rut block
// proves it is sized from the profile grid, not a fixed constant.
[[nodiscard]] nativecfg::TireConfig
stage_b_deformable_config(std::string_view tier) {
    auto cfg = bt::pneumatic::configuration(tier);
    cfg.surface_mode = "track";
    if (!cfg.pneumatic.has_value())
        throw std::logic_error("fixture needs pneumatic configuration");
    cfg.pneumatic->surface.reset();
    const auto x_grid =
            bike::geometry::detail::np_linspace(-1.0, 60.0, 12201);
    cfg.pneumatic->profile = {
            .x0_m = x_grid.front(), .dx_m = x_grid[1] - x_grid[0],
            .z_m = std::vector<double>(x_grid.size(), 0.0)};
    cfg.surface_map = nativecfg::SurfaceMap{
            .surface = surface("hardpack"),
            .sections = {{.start_m = 0.5, .end_m = 4.0,
                          .surface = surface("scree"), .blend_m = 0.0,
                          .layer_thickness_m = 0.05}}};
    return cfg;
}

void same_rut_state(const biketyre::RutStateDiff &a,
                    const biketyre::RutStateDiff &b) {
    bt::require(a.spans == b.spans, "rut state spans unchanged");
    bitwise_equal(a.z_max_m, b.z_max_m);
    bitwise_equal(a.z_p_m, b.z_p_m);
    bitwise_equal(a.z_e_m, b.z_e_m);
}

// Drop every node record whose grid index lies in [lo, hi) — used to
// erase the rear wheel's rut footprint from a snapshot while leaving the
// front wheel's bitwise identical, so a step-loss difference isolates
// the rear wheel's virgin-vs-compacted soil response.
[[nodiscard]] biketyre::RutStateDiff
rut_state_without_nodes(const biketyre::RutStateDiff &src,
                        std::uint32_t lo, std::uint32_t hi) {
    biketyre::RutStateDiff out;
    std::size_t at = 0;
    for (const auto &[b, e] : src.spans) {
        std::uint32_t run_b = 0;
        bool open = false;
        for (std::uint32_t g = b; g < e; ++g, ++at) {
            if (g >= lo && g < hi) {
                if (open) {
                    out.spans.emplace_back(run_b, g);
                    open = false;
                }
                continue;
            }
            if (!open) {
                run_b = g;
                open = true;
            }
            out.z_max_m.push_back(src.z_max_m[at]);
            out.z_p_m.push_back(src.z_p_m[at]);
            out.z_e_m.push_back(src.z_e_m[at]);
        }
        if (open) out.spans.emplace_back(run_b, e);
    }
    return out;
}

[[gnu::noinline]] void test_stage_b_deformable_layer_writer() {
    namespace pneu = bt::pneumatic;
    constexpr double dt = 0.0005;
    // 8 mm of geometric penetration on both wheels gives a comparable
    // static load; the front z slide then equalizes the loads exactly.
    // The rear body gains a vertical slide through rear_extra so both
    // hubs can follow the yielded rut — a fixed rear hub bridges a rut
    // deeper than its approach and unloads entirely.
    pneu::Rig rig("fast", dt, 0.008, pneu::rear_radius() - 0.008,
                  "<joint name=\"rear_root_z\" type=\"slide\" "
                  "axis=\"0 0 1\"/>");
    const auto cfg = stage_b_deformable_config("fast");
    rig.tire = std::make_unique<PneumaticTireWriter>(
            rig.engine.model.get(), rig.engine.data.get(), cfg);
    bt::require(rig.tire->has_deformable_layer() &&
                    rig.tire->rut_memory() != nullptr,
                "positive scree layer depth engages rut memory");
    const biketyre::RutMemory &rut = *rig.tire->rut_memory();
    const std::size_t nodes = rut.node_count();
    bt::require(nodes == 12201,
                "rut capacity is the profile grid node count");

    const auto *m = rig.engine.model.get();
    auto *d = rig.engine.data.get();
    const auto qpos = model_access::mutable_buffer(d->qpos, m->nq, "qpos");
    const auto qvel = model_access::mutable_buffer(d->qvel, m->nv, "qvel");
    std::vector<double> out(static_cast<std::size_t>(m->nv), 0.0);
    std::vector<double> probe_out(static_cast<std::size_t>(m->nv), 0.0);
    qpos[0] = 0.55;
    qpos[1] = 0.0;
    qvel[0] = 2.0;
    qvel[1] = 0.0;
    qvel[2] = 2.0 / pneu::front_radius();
    qvel[3] = 2.0 / pneu::rear_radius();
    ::engine::forward(m, d);
    const auto step_once = [&] {
        bt::require(rig.tire->try_compute_into(d, dt, out) == CoreStatus::ok,
                    "deformable warm compute succeeds");
        d->time += dt;
    };
    const auto normal_loads = [&] {
        const auto &diag = rig.tire->diagnostic_storage();
        return std::pair{diag[0].normal_load_n, diag[1].normal_load_n};
    };

    // ---- Serialized capacity: the rut block spans the whole grid ------
    {
        const auto &names = rig.tire->state_names();
        const std::size_t n_rays =
                static_cast<std::size_t>(cfg.pneumatic->n_rays); // NOLINT(bugprone-unchecked-optional-access) stage_b_deformable_config always engages pneumatic
        const std::size_t max_patches = (n_rays + 1) / 2;
        const std::size_t side_cols = 4 * n_rays + 8 + 3 * max_patches;
        const std::size_t rut_base = 2 * side_cols;
        bt::require(names.size() == rut_base + 1 + 4 * nodes,
                    "state names cover every profile-grid rut slot");
        bt::require(names[rut_base] == "rut.count" &&
                        names.back() == "rut.node.12200.z_e_m",
                    "rut block runs rut.count through the last grid node");
        const auto virgin_row = rig.tire->state();
        bt::require(virgin_row.size() == names.size() &&
                        virgin_row[rut_base] == 0.0,
                    "virgin rut serializes count 0 over the full row");
        // The decisive capacity proof: a canonical row dirtying EVERY
        // grid node (12201 > the retired 8192 cap) must decode on a
        // whole-track deformable map — the slot bound is the grid, not a
        // fixed constant.
        {
            auto whole_cfg = cfg;
            auto scree_base = surface("scree");
            scree_base.layer_thickness_m = 0.05;
            whole_cfg.surface_map = nativecfg::SurfaceMap{
                    .surface = scree_base, .sections = {}};
            const auto whole_writer =
                    std::make_unique<PneumaticTireWriter>(m, d, whole_cfg);
            const auto &whole_names = whole_writer->state_names();
            auto full = whole_writer->state();
            full[rut_base] = static_cast<double>(nodes);
            for (std::size_t k = 0; k < nodes; ++k) {
                full[rut_base + 1 + 4 * k + 0] = static_cast<double>(k);
                full[rut_base + 1 + 4 * k + 1] = 0.001;
                full[rut_base + 1 + 4 * k + 2] = 0.0;
                full[rut_base + 1 + 4 * k + 3] = 0.0;
            }
            whole_writer->set_state(whole_names, full);
            bt::require(whole_writer->rut_state().z_max_m.size() == nodes,
                        "full-grid rut row restores past the retired cap");
        }
        // Negative C2 wire gates: set_state must reject dirtying a rigid
        // node, zm > h_s, and ze > zm - zp.
        {
            auto bad_rigid = virgin_row;
            bad_rigid[rut_base] = 1.0;
            bad_rigid[rut_base + 1] = 0.0; // x = -1.0 m (rigid hardpack)
            bad_rigid[rut_base + 2] = 0.001;
            bad_rigid[rut_base + 3] = 0.0;
            bad_rigid[rut_base + 4] = 0.0;
            bt::pneumatic::rejects<std::runtime_error>(
                    [&] { rig.tire->set_state(names, bad_rigid); },
                    "invalid");
            auto bad_depth = virgin_row;
            bad_depth[rut_base] = 1.0;
            bad_depth[rut_base + 1] = 600.0; // x = 2.0 m (scree, h_s = 0.05)
            bad_depth[rut_base + 2] = 0.06;
            bad_depth[rut_base + 3] = 0.01;
            bad_depth[rut_base + 4] = 0.01;
            bt::pneumatic::rejects<std::runtime_error>(
                    [&] { rig.tire->set_state(names, bad_depth); },
                    "invalid");
            auto bad_ze = virgin_row;
            bad_ze[rut_base] = 1.0;
            bad_ze[rut_base + 1] = 600.0;
            bad_ze[rut_base + 2] = 0.02;
            bad_ze[rut_base + 3] = 0.015;
            bad_ze[rut_base + 4] = 0.01;
            bt::pneumatic::rejects<std::runtime_error>(
                    [&] { rig.tire->set_state(names, bad_ze); },
                    "invalid");
        }
        rig.tire->restore_rut_state(biketyre::RutStateDiff{});
    }

    // ---- Front wheel compacts virgin scree; rear follows 1.23 m behind
    // Roll the front wheel from x = 0.55 through the rear wheel's line
    // at x = 0.8 to its parked position — the 1.23 m follow distance the
    // multipass measurement uses.
    for (int i = 0; i < 1600 && qpos[0] < 2.03; ++i) {
        step_once();
        qpos[0] = std::min(2.03, qpos[0] + 0.001);
        ::engine::forward(m, d);
    }
    bt::require(qpos[0] >= 2.03, "front wheel reached the follow position");
    for (int i = 0; i < 300; ++i) {
        step_once();
    }
    // The yielded scree left ruts far deeper than the 8 mm design
    // approach: the coupled solve caps the equilibrium depression at
    // the geometric approach, so both tyres went effectively airborne.
    // Drop each hub by its footprint's committed depression — the
    // depressed surface then sits ~8 mm under the undeformed-road
    // approach again and the design penetration is restored. The
    // rear slide added through rear_extra makes the rear hub movable;
    // qpos[4] is its z coordinate (joint order: spin then slide).
    {
        // Interpolated committed surface depression (z_p + z_e) at the
        // hub's own x — the drop that re-centres the design approach on
        // the compacted surface. A footprint max would over-drop where
        // the rut is nonuniform.
        const auto rut_drop = [&](double hub_x) {
            const double u = (hub_x - rut.x0_m()) / rut.dx_m();
            if (u < 0.0 || u >= static_cast<double>(nodes - 1)) return 0.0;
            const std::size_t g = static_cast<std::size_t>(std::floor(u));
            const double a = u - static_cast<double>(g);
            const double h = std::max(rut.h_s_m()[g],
                                      rut.h_s_m()[g + 1]);
            if (!(h > 0.0)) return 0.0;
            const double s0 = rut.z_p_m()[g] + rut.z_e_m()[g];
            const double s1 = rut.z_p_m()[g + 1] + rut.z_e_m()[g + 1];
            return (1.0 - a) * s0 + a * s1;
        };
        const double drop_f = rut_drop(qpos[0]);
        const double drop_r = rut_drop(0.8);
        qpos[1] -= drop_f;
        qpos[4] -= drop_r;
        ::engine::forward(m, d);
        for (int i = 0; i < 300; ++i) step_once();
    }

    // The front wheel's pass left a plastic rut: z_p > 0 on nodes only
    // it covered (x in [1.0, 1.9] — past the rear wheel's reach).
    {
        bool plastic_rut = false;
        for (std::size_t g = 0; g < nodes; ++g) {
            const double x =
                    rut.x0_m() + static_cast<double>(g) * rut.dx_m();
            if (x >= 1.0 && x <= 1.9 && rut.z_p_m()[g] > 0.0 &&
                rut.z_max_m()[g] > 0.0)
                plastic_rut = true;
        }
        bt::require(plastic_rut,
                    "front pass leaves z_p > 0 on virgin scree");
    }

    // Equal normal load: the rear slide stays trimmed at its rut-follow
    // pose, so a Newton iteration on the front z slide alone matches
    // the front wheel's load to the rear wheel's.
    double equalized_rear_load = 0.0;
    {
        const auto loads_a = normal_loads();
        const double wfa = loads_a.first;
        qpos[1] += 0.002;
        ::engine::forward(m, d);
        for (int i = 0; i < 100; ++i) step_once();
        const auto loads_b = normal_loads();
        const double stiffness = (loads_b.first - wfa) / 0.002; // dW/dz < 0
        double z = qpos[1];
        for (int it = 0; it < 6 && std::abs(stiffness) > 1e-9; ++it) {
            const auto [wf, wr] = normal_loads();
            if (std::abs(wf - wr) <= std::max(0.5, 0.005 * wr)) break;
            z += (wr - wf) / stiffness;
            qpos[1] = z;
            ::engine::forward(m, d);
            for (int i = 0; i < 120; ++i) step_once();
        }
        const auto [wf_end, wr_end] = normal_loads();
        equalized_rear_load = wr_end;
        bt::require(std::abs(wf_end - wr_end) <=
                            std::max(1.0, 0.02 * wr_end),
                    "front and rear wheels carry equal normal load");
        bt::require(wr_end > 20.0, "wheels carry a real soil load");
        bt::require(rig.tire->soil_elastic_energy_j() > 0.0,
                    "loaded deformable layer stores elastic soil energy");
    }

    // ---- Multipass: rear on the front wheel's compacted rut -----------
    // Park the front wheel past the scree section on rigid hardpack:
    // its own rut commits identically in both variants but its
    // chassis-coupled transient would otherwise leak into the plastic
    // ledger, so the windows below price the rear wheel alone.
    qpos[0] = 5.0;
    ::engine::forward(m, d);
    for (int i = 0; i < 400; ++i) step_once();
    // `followed` is the rut the pass left; `virgin_at_rear` removes only
    // the rear footprint's nodes, so the front wheel's soil state — and
    // therefore its commit — is bitwise identical in both restores.
    const biketyre::RutStateDiff followed = rig.tire->rut_state();
    const std::uint32_t rear_lo = static_cast<std::uint32_t>(
            (0.65 - rut.x0_m()) / rut.dx_m());
    const std::uint32_t rear_hi = static_cast<std::uint32_t>(
            (0.95 - rut.x0_m()) / rut.dx_m());
    const biketyre::RutStateDiff virgin_at_rear =
            rut_state_without_nodes(followed, rear_lo, rear_hi);
    bt::require(!virgin_at_rear.z_max_m.empty() &&
                    virgin_at_rear.z_max_m.size() <
                            followed.z_max_m.size(),
                "rear footprint nodes removed from the virgin variant");
    // For each variant the rear hub (qpos[4]) is trimmed until the rear
    // load sits at the equalized target, then a fixed window measures
    // the plastic work the pass commits. The virgin variant's softer
    // nodes keep yielding — strictly more plastic work at the same
    // normal load; the compacted rut mostly reloads elastically.
    const auto multipass = [&](const biketyre::RutStateDiff &diff,
                               double &plastic_j) {
        rig.tire->restore_rut_state(diff);
        // Price the whole pass — equalization settling plus the steady
        // window. A static wheel reaches its rut equilibrium during the
        // load trim, so a post-equalization window alone reads ~0 even
        // on virgin soil; the compaction difference the multipass rule
        // is about is committed across the full run at equal load.
        const double p0 =
                rig.tire->rut_memory()->total_plastic_loss_j();
        double z = qpos[4];
        for (int it = 0; it < 14; ++it) {
            for (int i = 0; i < 80; ++i) step_once();
            const double load = normal_loads().second;
            if (std::abs(load - equalized_rear_load) <=
                    0.02 * equalized_rear_load)
                break;
            // qpos[4] is hub height (+ up): raising it unloads the wheel.
            // Positive feedback requires z += (load - target)/stiffness.
            z += (load - equalized_rear_load) / 60000.0;
            qpos[4] = z;
            ::engine::forward(m, d);
        }
        // A steady window closes the pass so the plastic total reflects
        // the settled rut, not a mid-transient frame.
        for (int i = 0; i < 120; ++i) step_once();
        plastic_j =
                rig.tire->rut_memory()->total_plastic_loss_j() - p0;
    };
    double loss_followed = 0.0, loss_virgin = 0.0;
    multipass(followed, loss_followed);
    const auto loads_f = normal_loads();
    multipass(virgin_at_rear, loss_virgin);
    const auto loads_v = normal_loads();
    bt::require(loss_virgin > 0.0,
                "virgin scree commits plastic soil work");
    bt::require(loss_followed < loss_virgin,
                "rear wheel on the compacted rut dissipates strictly "
                "less plastic work than on virgin soil");
    bt::require(std::abs(loads_f.second - loads_v.second) <=
                            0.05 * std::max(loads_f.second, loads_v.second) &&
                        std::abs(loads_f.first - loads_v.first) <=
                            0.05 * std::max(loads_f.first, loads_v.first),
                "multipass comparison runs at equal normal load");

    // ---- Bedrock bound, monotone force, 1800 N ramp, zero allocs -----
    rig.tire->restore_rut_state(biketyre::RutStateDiff{});
    qpos[0] = 1.5; // front away from the rear line
    qpos[1] = 0.0;
    qpos[4] = 0.0; // rear slide back to the design penetration
    ::engine::forward(m, d);
    for (int i = 0; i < 200; ++i) step_once();

    // The authored scree law is C1 and monotone hardening toward bedrock:
    // the analytic slope tracks the central finite difference and the
    // barrier term stiffens the virgin branch well before h_s.
    {
        const std::size_t g_law = 600; // x = 2.0 m — mid scree section
        const double hs = rut.h_s_m()[g_law];
        const double b_m =
                0.5 * cfg.pneumatic->front.casing_width_mm * 0.001; // NOLINT(bugprone-unchecked-optional-access) stage_b_deformable_config always engages pneumatic
        bt::require(hs == 0.05,
                    "scree node carries the authored layer depth");
        const double eps = 1e-6;
        double prev_p = rut.pressure_virgin(g_law, 0.0, b_m);
        bt::require(prev_p == 0.0, "regularized virgin pressure vanishes at z = 0");
        const double slope0 = rut.pressure_virgin_slope(g_law, 0.0, b_m);
        bt::require(std::isfinite(slope0) && slope0 > 0.0,
                    "regularized virgin slope is finite and positive at z = 0");
        for (int k = 1; k <= 40; ++k) {
            const double z = 0.95 * hs * k / 40.0;
            const double p = rut.pressure_virgin(g_law, z, b_m);
            const double slope_fd =
                    (rut.pressure_virgin(g_law, z + eps, b_m) -
                     rut.pressure_virgin(g_law, z - eps, b_m)) /
                    (2.0 * eps);
            const double slope_an =
                    rut.pressure_virgin_slope(g_law, z, b_m);
            bt::require(p > prev_p,
                        "virgin pressure law is monotone in z");
            bt::require(std::abs(slope_fd - slope_an) <=
                                1e-3 * std::max(1.0, std::abs(slope_an)),
                        "virgin pressure law is C1 — analytic slope "
                        "matches the finite difference");
            prev_p = p;
        }
        bt::require(rut.pressure_virgin_slope(g_law, 0.9 * hs, b_m) >
                        rut.pressure_virgin_slope(g_law, 0.3 * hs, b_m),
                    "bedrock barrier hardens the virgin pressure slope");
    }

    // Ramp the front penetration until the normal load reaches 1800 N.
    // Every committed node must keep z_max strictly below h_s, the
    // normal force must stay monotone, and the whole warm path must
    // perform zero heap allocations.
    double prev_load = -1.0;
    bool reached_1800 = false;
    allocation_counter::reset();
    for (int level = 0; level < 30 && !reached_1800; ++level) {
        qpos[1] -= 0.002; // 2 mm deeper each level
        ::engine::forward(m, d);
        for (int i = 0; i < 500; ++i) {
            step_once();
            bt::require(rig.tire->stored_energy(d) ==
                            rig.tire->elastic_energy_j(),
                        "stored energy bitwise equals the elastic field");
            bt::require(rig.tire->soil_elastic_energy_j() >= 0.0 &&
                            rig.tire->ground_plastic_loss_step_j() >= 0.0 &&
                            rig.tire->brush_loss_step_j() >= 0.0,
                        "nonnegative soil and tyre dissipation");
        }
        const auto ramp_loads = normal_loads();
        const double wf = ramp_loads.first;
        bt::require(wf >= prev_load - 0.5,
                    "normal force is monotone under increasing penetration");
        prev_load = wf;
        for (std::size_t g = 0; g < nodes; ++g) {
            const double hs_g = rut.h_s_m()[g];
            if (hs_g > 0.0)
                bt::require(rut.z_max_m()[g] < hs_g,
                            "sinkage stays strictly below the layer");
        }
        if (wf >= 1800.0) reached_1800 = true;
        if (qpos[1] < -0.06) break; // 68 mm penetration — past rim strike
    }
    bt::require(reached_1800, "load ramp reaches the 1800 N bound");
    bt::require(rig.tire->soil_elastic_energy_j() > 0.0,
                "loaded deformable layer stores elastic soil energy");
    bt::require(allocation_counter::count() == std::size_t{0},
                "deformable warm path performs zero heap allocations");

    // ---- Probe purity: advance=false mutates zero rut state ----------
    {
        const auto rut_before = rig.tire->rut_state();
        const auto state_before = rig.tire->state();
        const double elastic_before = rig.tire->elastic_energy_j();
        const double brush_before = rig.tire->brush_loss_step_j();
        const double soil_before = rig.tire->soil_elastic_energy_j();
        allocation_counter::reset();
        rig.tire->probe_compute_into(d, dt, probe_out);
        bt::require(allocation_counter::count() == std::size_t{0},
                    "deformable probe_compute_into performs zero heap allocations");
        same_rut_state(rut_before, rig.tire->rut_state());
        bitwise_equal(state_before, rig.tire->state());
        bt::require(rig.tire->elastic_energy_j() == elastic_before &&
                        rig.tire->brush_loss_step_j() == brush_before &&
                        rig.tire->soil_elastic_energy_j() == soil_before,
                    "probe leaves every committed energy field bitwise");
    }
    // I4: detailed tier probe_compute_into is also allocation-free.
    {
        const auto det_cfg = stage_b_deformable_config("detailed");
        const auto det_writer =
                std::make_unique<PneumaticTireWriter>(m, d, det_cfg);
        allocation_counter::reset();
        det_writer->probe_compute_into(d, 0.00025, probe_out);
        bt::require(allocation_counter::count() == std::size_t{0},
                    "detailed deformable probe_compute_into performs zero heap allocations");
    }
    // C1 + I1: steep scree slope where traction_mu_cap == 0.0 must not
    // produce NaN at kappa_prime == 0.0 in lumped_brush or detailed brush,
    // and heterogeneous detailed patches must honor traction_mu_cap.
    {
        const auto spec = bt::tyres::front_tyre();
        const std::size_t n_pts = 401;
        std::vector<double> z_steep(n_pts, 0.0);
        for (std::size_t i = 0; i < n_pts; ++i)
            z_steep[i] = 0.65 * (static_cast<double>(i) * 0.005); // grade 65% > mu_basal 0.42
        const biketyre::RoadProfile steep_road(0.0, 0.005, std::move(z_steep));
        const nativecfg::SurfaceMap steep_map{
                .surface = surface("hardpack"),
                .sections = {{.start_m = 0.5, .end_m = 1.5,
                              .surface = surface("scree"), .blend_m = 0.2,
                              .layer_thickness_m = 0.15}}};
        const biketyre::RutMemory steep_rut(steep_road, steep_map);
        const biketyre::SoilContact soil{.state = &steep_rut, .commit = nullptr};
        const double x_hub = 1.0;
        const double z_road = 0.65 * x_hub;
        // Light touch on steep scree with zero slip (kappa_prime == 0.0):
        // traction_mu_cap == 0.0.
        const std::array<double, 3> hub{
                x_hub, 0.0, z_road + spec.outer_radius_mm * 0.001 - 0.0008};
        const std::array<double, 3> vel{0.0, 0.0, 0.0};
        for (const bool detailed : {false, true}) {
            biketyre::PneumaticTyreModel model(spec, 64, 75.0, detailed);
            const auto &wheel_out = model.evaluate(
                    hub, vel, 0.0, steep_road, steep_map, 0.0005, &soil);
            bt::require(std::isfinite(wheel_out.force_world_n[0]) &&
                            std::isfinite(wheel_out.force_world_n[2]),
                        "mu_cap == 0 with kappa_prime == 0 stays finite");
        }
    }
    rig.tire->clear_settled_loss();
    bt::require(rig.tire->brush_loss_step_j() == 0.0 &&
                    rig.tire->ground_plastic_loss_step_j() == 0.0,
                "clear_settled_loss clears both tire brush and soil plastic step losses");
}

[[gnu::noinline]] void test_stage_b_deformable_layer_ride_energy_gate() {
    namespace research = bike::research;
    namespace sw = runtime::sample_wire;
    const auto root =
            bt::oracle::root().parent_path().parent_path().parent_path();
    auto owner = std::make_unique<research::ResearchRequest>();
    auto &request = *owner;
    request.setup.model.physics = bike::configuration::load_physics(
            root / "examples/research/viewer_physics_pneumatic.toml");
    request.setup.model.mode = "ride";
    request.setup.model.crank_joint = true;
    request.setup.model.rider =
            bike::model::rider_from_args("articulated_planar");
    request.setup.track = bike::terrain::track_from_dict(
            bike::terrain::parse_track_toml(
                    "name = \"stage_b_deformable\"\nlength_m = 120.0\n"
                    "surface = \"asphalt\"\n"
                    "grade_profile = { knots = [[0.0, 0.01], [120.0, 0.01]] }\n"
                    "[[surface_sections]]\nstart_m = 20.0\nend_m = 60.0\n"
                    "surface = \"scree\"\nblend_m = 0.4\n"
                    "layer_thickness_m = 0.05\n"
                    "[[surface_sections]]\nstart_m = 70.0\nend_m = 115.0\n"
                    "surface = \"hardpack\"\nblend_m = 0.4\n"));
    request.experiment.duration_s = 45.0;
    request.experiment.record_decimation = 1;
    request.experiment.stop_on_model_violation = true;
    request.experiment.control_period_s =
            request.setup.model.physics->research.control_period_s;
    request.experiment.actuator_delay_s =
            request.setup.model.physics->research.actuator_delay_s;
    request.sensors.delivery.latency_s =
            request.setup.model.physics->research.sensor_latency_s;
    request.sensors.delivery.maximum_age_s =
            request.setup.model.physics->research.sensor_maximum_age_s;
    request.sensors.sample_period_s = 0.005;
    auto prepared = research::prepare_environment(std::move(owner));
    const auto period = prepared->bootstrap.config.control_steps;
    const auto max_steps = prepared->bootstrap.config.max_steps;
    auto rt = std::make_unique<runtime::NativeRideRuntime>(
            std::move(prepared->bootstrap.ride));
    while (!rt->snapshot().outcome.has_value() &&
           rt->snapshot().step < max_steps) {
        const auto result =
                rt->advance(rt->snapshot().step + period,
                            runtime::RideControl{}, 0., 0., std::nullopt);
        if (!rt->recorded_intervals().empty() &&
            rt->recorded_intervals().back()->qpos[0] >= 74.0)
            break;
        if (result.reason == "outcome") break;
    }
    const auto snapshot = rt->snapshot();
    bt::require(!snapshot.first_failure.has_value(),
                "deformable ride has no first failure");
    const auto rows = rt->recorded_intervals();
    bt::require(!rows.empty() && rows.back()->qpos[0] >= 72.0,
                "rider carries both wheels across scree into hardpack");
    std::size_t energy_rows = 0;
    std::size_t bad_eq_rows = 0;
    double max_residual_ratio = 0.0;
    double max_soil_elastic_j = 0.0;
    double sum_ground_plastic_j = 0.0;
    bool saw_soil_elastic_positive = false;
    bool saw_ground_plastic_positive = false;
    for (const auto &row : rows) {
        bt::require(research::model_violations(*row).empty(),
                    "no model violations on the deformable ride");
        const auto &tires =
                sw::object(sw::required(row->channels, "tires"));
        for (const char *side : {"front", "rear"}) {
            const auto &wheel =
                    sw::object(sw::required(tires, side));
            bt::require(sw::number(sw::required(
                                    wheel, "dissipated_power_w")) >= 0.0,
                            "nonnegative tyre dissipation on deformable "
                            "soil");
            bt::require(sw::number(sw::required(
                                    wheel, "brush_loss_j")) >= 0.0,
                            "nonnegative brush loss on deformable soil");
        }
        // Strict energy accounting on every measured step: the residual
        // ratio gate the other ride legs use, applied per row, plus the
        // separated soil elastic and ground plastic loss channels.
        const auto *energy_w = sw::find(row->channels, "energy");
        bt::require(energy_w != nullptr,
                    "every measured step carries the energy channel");
        const auto &energy_obj = sw::object(*energy_w);
        const auto &elastic_obj =
                sw::object(sw::required(energy_obj, "elastic_energy_j"));
        const double soil_elastic =
                sw::number(sw::required(elastic_obj, "soil"));
        const double ground_plastic = sw::number(
                sw::required(energy_obj, "ground_plastic_loss_step_j"));
        bt::require(soil_elastic >= 0.0 && ground_plastic >= 0.0,
                    "soil elastic energy and ground plastic loss are "
                    "nonnegative");
        saw_soil_elastic_positive |= soil_elastic > 0.0;
        saw_ground_plastic_positive |= ground_plastic > 0.0;
        max_soil_elastic_j = std::max(max_soil_elastic_j, soil_elastic);
        sum_ground_plastic_j += ground_plastic;
        const auto eq = research::energy_quality(energy_obj, 0.05);
        max_residual_ratio = std::max(max_residual_ratio, eq.residual_ratio);
        bad_eq_rows += !(eq.acceptable && eq.residual_ratio <= 0.05);
        ++energy_rows;
    }
    bt::require(saw_soil_elastic_positive,
                "deformable ride records positive elastic_energy_j.soil");
    bt::require(saw_ground_plastic_positive,
                "deformable ride records positive ground_plastic_loss_step_j "
                "on virgin compaction");
    bt::require(energy_rows == rows.size(),
                "energy quality checked on every recorded step");
    bt::require(bad_eq_rows == 0,
                std::format("strict energy accounting failed: bad_eq={}, "
                            "max_ratio={:.6f}, max_soil_e={:.6f} J, "
                            "sum_plastic={:.6f} J",
                            bad_eq_rows, max_residual_ratio,
                            max_soil_elastic_j, sum_ground_plastic_j));

    // Rut engagement proof plus probe purity at the PhysicalStep level:
    // the canonical tire state carries the rut block, the ride left a
    // nonzero dirty-node count, and a non-advancing probe leaves the
    // whole tire state — rut included — bitwise unchanged.
    std::vector<std::string> tire_names;
    std::vector<double> state_before;
    bike::validation::RuntimeRigAccess::inspect(
            *rt, [&](const Stepper &stepper,
                     const runtime::PhysicalStep &) {
                tire_names = stepper.tire_state_names();
                state_before = stepper.tire_state();
            });
    const auto rut_col = std::ranges::find(tire_names, "rut.count");
    bt::require(rut_col != tire_names.end(),
                "deformable ride engages the rut state block");
    const std::size_t rut_idx =
            static_cast<std::size_t>(rut_col - tire_names.begin());
    bt::require(state_before[rut_idx] > 0.0,
                "the ride deposited a nonzero rut node count");
    bt::require(tire_names.back().starts_with("rut.node."),
                "rut block is sized from the ride profile grid");
    (void)rt->probe_step_inputs(runtime::RideControl{}, 0., 0.);
    std::vector<double> state_after;
    bike::validation::RuntimeRigAccess::inspect(
            *rt, [&](const Stepper &stepper,
                     const runtime::PhysicalStep &) {
                state_after = stepper.tire_state();
            });
    bitwise_equal(state_before, state_after);
}

} // namespace

// bt::run catches exceptions from the test body and reports failure.
int main() {
    // cppcheck-suppress throwInEntryPoint
    return bt::run([] {
        test_airborne_zero_dt_and_rejection_atomicity();
        test_zero_contact_length_shear_energy();
        test_negative_centroid_mixed_surface_name();
        test_model_validation_without_golden();
        test_air_chord_and_ray_elastic_quadrature();
        test_maxwell_zero_carcass_stiffness_cap_inverse();
        test_maxwell_hold_and_closed_cycle_passivity();
        test_single_rim_hysteresis_and_stored_energy_bitwise();
        test_uniform_and_heterogeneous_brush_equivalence();
        test_brush_shear_energy_matches_element_stiffness();
        test_pneumatic_surface_name_storage_survives_reset();
        test_pneumatic_crr_overflow_atomicity();
        test_roots_wet_wood_micro_span_ride_energy_gate();
        test_micro_span_writer_zero_allocation();
        test_micro_span_unsorted_wire_equivalent();
        test_distributed_roots_wet_wood_diagnostics();
        test_distributed_roots_wet_wood_ride_energy_gate();
        test_stage_b_deformable_layer_writer();
        test_pneumatic_world_rolling_moment_no_caliper_reaction();
        test_pneumatic_boundary_crossing_smoothness_and_stored_energy();
        test_pneumatic_60m_mixed_surface_ride_energy_gate();
        test_stage_b_deformable_layer_ride_energy_gate();
    });
}
