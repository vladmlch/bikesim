#include "configuration/load.hpp"
#include "model/rider_geometry.hpp"
#include "pneumatic/carcass.hpp"
#include "pneumatic/model.hpp"
#include "pneumatic_tire_fixture.hpp"
#include "reference_tyres.hpp"
#include "research/environment.hpp"
#include "research/evidence.hpp"
#include "runtime/runtime.hpp"
#include "runtime/samples.hpp"
#include "support/fixture_test.hpp"
#include "support/golden.hpp"
#include "terrain/trackfile.hpp"
#include "writers/pneumatic_tire.hpp"
#include "writers/tire_common.hpp"

#include <algorithm>
#include <array>
#include <cmath>
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
    const auto eq = research::energy_quality(energy, 0.05);
    bt::require(eq.acceptable && eq.residual_ratio <= 0.05,
                "60 m mixed-surface pneumatic ride satisfies energy_quality <= 0.05");
}

} // namespace

// bt::run catches exceptions from the test body and reports failure.
int main() {
    // cppcheck-suppress throwInEntryPoint
    return bt::run([] {
        test_air_chord_and_ray_elastic_quadrature();
        test_maxwell_zero_carcass_stiffness_cap_inverse();
        test_maxwell_hold_and_closed_cycle_passivity();
        test_single_rim_hysteresis_and_stored_energy_bitwise();
        test_uniform_and_heterogeneous_brush_equivalence();
        test_brush_shear_energy_matches_element_stiffness();
        test_pneumatic_surface_name_storage_survives_reset();
        test_pneumatic_crr_overflow_atomicity();
        test_pneumatic_world_rolling_moment_no_caliper_reaction();
        test_pneumatic_boundary_crossing_smoothness_and_stored_energy();
        test_pneumatic_60m_mixed_surface_ride_energy_gate();
    });
}
