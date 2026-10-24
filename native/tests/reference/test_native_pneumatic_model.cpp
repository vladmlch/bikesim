// test_native_pneumatic_model.cpp — C++ port of
// tests/reference/test_native_pneumatic_model.py (batch 3.12b, T??).
//
// Replays PneumaticTyre.evaluate end to end — outputs, carcass and patch
// state — plus set_pressure/reset and the validation order against the
// frozen two-platform goldens. Every comparison replays the production
// biketyre::PneumaticTyreModel; the capture-side `_golden.record` labels
// and order are documented on each consumer.
#include "pneumatic_reference_support.hpp"
#include "pneumatic_tire_fixture.hpp"

#include <set>

#include "pneumatic/model.hpp"
#include "terrain/types.hpp"

namespace {

using namespace pneumatic_reference;
constexpr std::string_view kFile = "test_native_pneumatic_model.py";

// The oracle's tier table plus the two test-only tiers.
struct TierCfg {
    std::int64_t n_rays;
    double half_angle_deg;
    double timestep_s;
    bool discretised;
};
constexpr TierCfg kFast{64, 75.0, 0.0005, false};
constexpr TierCfg kDetail{256, 75.0, 0.00025, true};
constexpr TierCfg kDetail64{64, 75.0, 0.0005, true};
constexpr TierCfg kFast256{256, 75.0, 0.00025, false};

// ---------------------------------------------------------------------
// Entry consumers — the capture order inside _assert_outputs_bitwise /
// _assert_state_bitwise, verbatim.
// ---------------------------------------------------------------------

// _record_string(...) — a |u1 bytes entry.
inline void text(bt::oracle::Node &golden, std::string_view actual,
                 std::string_view label) {
    const std::array<std::size_t, 1> shape{actual.size()};
    golden.bytes(std::as_bytes(std::span(actual)), "|u1", shape, label);
}

// _golden.record(np.asarray(..., np.int64)) — a little-endian i8 entry.
inline void ints(bt::oracle::Node &golden,
                 std::span<const std::int64_t> actual,
                 std::span<const std::size_t> shape, std::string_view label) {
    std::vector<std::byte> bytes(actual.size() * 8);
    for (std::size_t i = 0; i < actual.size(); ++i) {
        const auto bits = std::bit_cast<std::uint64_t>(actual[i]);
        for (unsigned shift = 0; shift < 64; shift += 8)
            bytes[i * 8 + shift / 8] =
                    static_cast<std::byte>((bits >> shift) & 0xffu);
    }
    golden.bytes(bytes, "<i8", shape, label);
}

inline void count(bt::oracle::Node &golden, std::int64_t actual,
                  std::string_view label) {
    const std::array value{actual};
    ints(golden, value, {}, label);
}

// The five rim-event floats the oracle's RimStrikeEvent carries —
// model.py _RIM_EVENT_KEYS order.
void rim_event(bt::oracle::Node &golden,
               const std::optional<biketyre::RimStrikeEvent> &ev,
               const std::string &msg) {
    boolean(golden, ev.has_value(), msg + " rim_event");
    if (!ev) return;
    scalar(golden, ev->x_m, msg + " rim_event.x_m");
    scalar(golden, ev->speed_mps, msg + " rim_event.speed_mps");
    scalar(golden, ev->peak_load_n, msg + " rim_event.peak_load_n");
    scalar(golden, ev->peak_rim_force_n, msg + " rim_event.peak_rim_force_n");
    scalar(golden, ev->absorbed_energy_j,
           msg + " rim_event.absorbed_energy_j");
}

// _assert_outputs_bitwise(got, want, msg).
void outputs(bt::oracle::Node &golden, const biketyre::WheelOutputs &got,
             const std::string &msg) {
    text(golden, got.tyre_name, msg + " tyre_name");
    vector(golden, got.hub_position_world_m, msg + " hub_position_world_m");
    vector(golden, got.hub_velocity_world_mps,
           msg + " hub_velocity_world_mps");
    vector(golden, got.force_world_n, msg + " force_world_n");
    scalar(golden, got.pressure_bar, msg + " pressure_bar");
    scalar(golden, got.omega_forward_radps, msg + " omega_forward_radps");
    scalar(golden, got.normal_load_n, msg + " normal_load_n");
    scalar(golden, got.support_n, msg + " support_n");
    scalar(golden, got.mean_deflection_m, msg + " mean_deflection_m");
    scalar(golden, got.contact_length_m, msg + " contact_length_m");
    scalar(golden, got.slip_ratio, msg + " slip_ratio");
    scalar(golden, got.transient_slip_ratio,
           msg + " transient_slip_ratio");
    scalar(golden, got.dissipated_power_w, msg + " dissipated_power_w");
    boolean(golden, got.fully_sliding, msg + " fully_sliding");
    boolean(golden, got.rim_strike_active, msg + " rim_strike_active");
    boolean(golden, got.coverage_event, msg + " coverage_event");
    boolean(golden, got.airborne, msg + " airborne");
    rim_event(golden, got.rim_event, msg);
    count(golden, static_cast<std::int64_t>(got.patch_count),
          msg + " patches");
    const auto patches = got.patches();
    for (std::size_t i = 0; i < patches.size(); ++i) {
        const auto &p = patches[i];
        const std::string pmsg =
                msg + " patches[" + std::to_string(i) + "]";
        const std::array rays{static_cast<std::int64_t>(p.ray_start),
                              static_cast<std::int64_t>(p.ray_stop)};
        ints(golden, rays, std::array<std::size_t, 1>{2},
                 pmsg + " rays");
        text(golden, p.surface_name, pmsg + " surface_name");
        vector(golden, p.normal_force_world_n,
               pmsg + " normal_force_world_n");
        vector(golden, p.tangent_world, pmsg + " tangent_world");
        vector(golden, p.force_world_n, pmsg + " force_world_n");
        vector(golden, p.centroid_world_m, pmsg + " centroid_world_m");
        scalar(golden, p.normal_load_n, pmsg + " normal_load_n");
        scalar(golden, p.support_n, pmsg + " support_n");
        scalar(golden, p.tangential_force_n, pmsg + " tangential_force_n");
        scalar(golden, p.mean_deflection_m, pmsg + " mean_deflection_m");
        scalar(golden, p.contact_length_m, pmsg + " contact_length_m");
        scalar(golden, p.slip_ratio, pmsg + " slip_ratio");
        scalar(golden, p.transient_slip_ratio,
               pmsg + " transient_slip_ratio");
        scalar(golden, p.sliding_speed_mps, pmsg + " sliding_speed_mps");
        scalar(golden, p.slip_speed_mps, pmsg + " slip_speed_mps");
        scalar(golden, p.friction_coefficient,
               pmsg + " friction_coefficient");
        scalar(golden, p.dissipated_power_w, pmsg + " dissipated_power_w");
        boolean(golden, p.fully_sliding, pmsg + " fully_sliding");
    }
}

// _assert_carcass_state_bitwise + _assert_state_bitwise: rings, the two
// flags, the five rim floats, then the ordered patch slots.
void state(bt::oracle::Node &golden, const biketyre::PneumaticTyreModel &model,
           const std::string &msg) {
    const auto &carcass = model.carcass_state();
    vector(golden, carcass.delta_prev_m(), msg + " carcass.delta_prev_m");
    vector(golden, carcass.elastic_prev_n(), msg + " carcass.elastic_prev_n");
    vector(golden, carcass.maxwell_force_n(), msg + " carcass.maxwell_force_n");
    boolean(golden, carcass.initialized(), msg + " carcass.initialized");
    boolean(golden, carcass.rim_strike_active(),
            msg + " carcass.rim_strike_active");
    scalar(golden, carcass.rim_event_x_m(), msg + " carcass.rim_event_x_m");
    scalar(golden, carcass.rim_event_speed_mps(),
           msg + " carcass.rim_event_speed_mps");
    scalar(golden, carcass.rim_event_peak_load_n(),
           msg + " carcass.rim_event_peak_load_n");
    scalar(golden, carcass.rim_event_peak_force_n(),
           msg + " carcass.rim_event_peak_force_n");
    scalar(golden, carcass.rim_event_energy_j(),
           msg + " carcass.rim_event_energy_j");
    const auto patches = model.patch_states();
    count(golden, static_cast<std::int64_t>(patches.size()),
          msg + " patch_states");
    for (std::size_t i = 0; i < patches.size(); ++i) {
        const auto &p = patches[i];
        const std::string smsg =
                msg + " patch_states[" + std::to_string(i) + "]";
        const std::array key{static_cast<std::int64_t>(p.start),
                             static_cast<std::int64_t>(p.stop)};
        ints(golden, key, std::array<std::size_t, 1>{2},
                 smsg + " key");
        scalar(golden, p.kappa_prime, smsg + " kappa_prime");
        boolean(golden, p.has_detailed, smsg + " detailed");
        if (p.has_detailed) {
            vector(golden, p.bristle_deflection_m,
                   smsg + " detailed.bristle_deflection_m");
            scalar(golden, p.detailed_kappa_prime,
                   smsg + " detailed.kappa_prime");
        }
    }
}

// ---------------------------------------------------------------------
// Shared roads / surface maps / drivers — the oracle file's helpers.
// ---------------------------------------------------------------------

[[nodiscard]] biketyre::RoadProfile flat_road() {
    return {0.0, 0.01, std::vector<double>(1201, 0.0)};
}

[[nodiscard]] biketyre::RoadProfile bump_road() {
    const auto x = arange_scaled(1201, 0.01);
    std::vector<double> z(1201);
    for (std::size_t i = 0; i < 1201; ++i) {
        const double bump = (x[i] - 5.05) / 0.16;
        const double dip = (x[i] - 4.72) / 0.10;
        const double tail = (x[i] - 5.55) / 0.09;
        z[i] = 0.028 * std::exp(-(bump * bump)) -
               0.018 * std::exp(-(dip * dip)) +
               0.012 * std::exp(-(tail * tail));
    }
    return {0.0, 0.01, std::move(z)};
}

[[nodiscard]] biketyre::RoadProfile twin_bump_road() {
    const auto x = arange_scaled(1201, 0.01);
    std::vector<double> z(1201);
    for (std::size_t i = 0; i < 1201; ++i) {
        const double left = (x[i] - 4.90) / 0.12;
        const double right = (x[i] - 5.20) / 0.12;
        const double trough = (x[i] - 5.05) / 0.05;
        z[i] = 0.030 * std::exp(-(left * left)) +
               0.030 * std::exp(-(right * right)) -
               0.020 * std::exp(-(trough * trough));
    }
    return {0.0, 0.01, std::move(z)};
}

[[nodiscard]] biketyre::RoadProfile spike_road() {
    const auto x = arange_scaled(1201, 0.01);
    std::vector<double> z(1201);
    for (std::size_t i = 0; i < 1201; ++i) {
        const double spike = (x[i] - 5.30) / 0.05;
        z[i] = 0.45 * std::exp(-(spike * spike));
    }
    return {0.0, 0.01, std::move(z)};
}

[[nodiscard]] nativecfg::SurfaceMap uniform_map(std::string_view name) {
    return {.surface = surface(name), .sections = {}};
}

// _zoned_map — hardpack base with loose [4.6,5.1) and scree [5.1,5.6).
[[nodiscard]] nativecfg::SurfaceMap zoned_map() {
    return {.surface = surface("hardpack"),
            .sections = {{4.6, 5.1, surface("loose")},
                         {5.1, 5.6, surface("scree")}}};
}

// Heap-owning factory: the model's scratch banks contain spans into its
// own storage, so the type is used in place, never moved.
[[nodiscard]] std::unique_ptr<biketyre::PneumaticTyreModel>
make_model(const nativecfg::PneumaticTyreSpec &tyre, const TierCfg &tier) {
    return std::make_unique<biketyre::PneumaticTyreModel>(
            tyre, tier.n_rays, tier.half_angle_deg, tier.discretised);
}

// One (hub, velocity, omega, dt[, pressure]) row of the oracle's `rows`.
struct Row {
    std::array<double, 3> hub, velocity;
    double omega, dt;
    std::optional<double> pressure;
};

[[nodiscard]] std::vector<Row>
rows(std::span<const double> xs, std::span<const double> zs,
     std::span<const double> omegas, std::span<const double> dts = {},
     double speed = 3.0) {
    std::vector<Row> out;
    const std::vector<double> default_dts(
            xs.size() ? xs.size() : 0, 0.002);
    const auto dt = dts.empty() ? std::span(default_dts) : dts;
    for (std::size_t i = 0; i < xs.size(); ++i)
        out.push_back({{xs[i], 0.0, zs[i]}, {speed, 0.0, 0.0},
                       omegas[i], dt[i], std::nullopt});
    return out;
}

// _run_sequence — the same multi-step sequence against the persistent
// model: optional set_pressure, evaluate, then the outputs + state
// consumers in capture order.
void run_sequence(bt::oracle::Node &golden,
                  const nativecfg::PneumaticTyreSpec &tyre,
                  const TierCfg &tier, const biketyre::RoadProfile &road,
                  const nativecfg::SurfaceMap &smap,
                  const std::vector<Row> &table,
                  const std::function<void(const biketyre::WheelOutputs &,
                                           const biketyre::PneumaticTyreModel &,
                                           const std::string &)> &check = {}) {
    auto model = make_model(tyre, tier);
    for (std::size_t i = 0; i < table.size(); ++i) {
        const auto &row = table[i];
        const std::string tag = "step " + std::to_string(i);
        if (row.pressure) model->set_pressure(*row.pressure);
        const auto &out = model->evaluate(row.hub, row.velocity, row.omega,
                                          road, smap, row.dt);
        outputs(golden, out, tag);
        state(golden, *model, tag);
        if (check) check(out, *model, tag);
    }
}

// An injected _PatchBrushState — the C++ twin of _injected_state.
[[nodiscard]] biketyre::PatchBrushState
injected(int start, int stop, double kappa_prime = 0.0,
         std::vector<double> bristle = {}, double detailed_kappa = 0.0) {
    biketyre::PatchBrushState state;
    state.start = start;
    state.stop = stop;
    state.kappa_prime = kappa_prime;
    if (!bristle.empty()) {
        state.has_detailed = true;
        state.bristle_deflection_m = std::move(bristle);
        state.detailed_kappa_prime = detailed_kappa;
    }
    return state;
}

// ---------------------------------------------------------------------
// test_model_progressive_load_unload_bitwise[tier{0,1}-tyre{0,1}]
// ---------------------------------------------------------------------
void test_model_progressive_load_unload_bitwise(std::size_t tier_index,
                                              std::size_t tyre_index) {
    auto golden = node(kFile, "test_model_progressive_load_unload_bitwise",
                       "tier" + std::to_string(tier_index) + "-tyre" +
                               std::to_string(tyre_index));
    const TierCfg tier = tier_index == 0 ? kFast : kDetail64;
    const auto tyre = tyre_index == 0 ? bike::legacy::front_tyre()
                                      : bike::legacy::rear_tyre();
    const auto road = bump_road();
    const double radius = tyre.outer_radius_mm * 0.001;
    std::vector<double> xs(16), zs(16), omegas;
    for (std::size_t k = 0; k < 16; ++k) {
        xs[k] = 4.6 + 0.03 * static_cast<double>(k);
        zs[k] = radius + 0.10 -
                std::min(static_cast<double>(k) * 0.016, 0.140);
    }
    for (const double dz : {-0.030, -0.028, -0.020, 0.02, 0.09})
        zs.push_back(radius + dz);
    // Python: xs += [xs[-1] + 0.03 * k for k in range(1, 6)] — each tail
    // value is xs[15] + 0.03*k, NOT a chained xs.back() + 0.03.
    const double xs_base = xs.back();
    for (std::size_t k = 1; k <= 5; ++k)
        xs.push_back(xs_base + 0.03 * static_cast<double>(k));
    omegas.resize(xs.size());
    for (std::size_t k = 0; k < xs.size(); ++k)
        omegas[k] = k % 5 < 3 ? 9.5 : -6.0;
    run_sequence(golden, tyre, tier, road, uniform_map("asphalt"),
                 ::rows(xs, zs, omegas));
    golden.done();
}

// test_model_authored_tiers_bitwise[tier{0,1}] — FAST256 then DETAIL.
void test_model_authored_tiers_bitwise(std::size_t tier_index) {
    auto golden = node(kFile, "test_model_authored_tiers_bitwise",
                       "tier" + std::to_string(tier_index));
    const TierCfg tier = tier_index == 0 ? kFast256 : kDetail;
    const auto tyre = bike::legacy::front_tyre();
    const auto road = bump_road();
    const double radius = tyre.outer_radius_mm * 0.001;
    std::vector<double> xs(10), zs(10), omegas(10, 8.0), dts(10);
    for (std::size_t k = 0; k < 10; ++k) {
        xs[k] = 4.8 + 0.02 * static_cast<double>(k);
        zs[k] = radius - 0.018 - 0.002 * static_cast<double>(k % 4);
        dts[k] = tier.timestep_s;
    }
    run_sequence(golden, tyre, tier, road, uniform_map("asphalt"),
                 ::rows(xs, zs, omegas, dts));
    golden.done();
}

// test_model_airborne_sequence_bitwise[tier{0,1}]
void test_model_airborne_sequence_bitwise(std::size_t tier_index) {
    auto golden = node(kFile, "test_model_airborne_sequence_bitwise",
                       "tier" + std::to_string(tier_index));
    const TierCfg tier = tier_index == 0 ? kFast : kDetail64;
    const auto tyre = bike::legacy::front_tyre();
    const double radius = tyre.outer_radius_mm * 0.001;
    const double hi = radius + 0.40, lo = radius - 0.018;
    const std::vector<double> xs(8, 5.0),
            zs{hi, hi, lo, lo, hi, hi, hi, lo}, omegas(8, 7.0);
    int liftoff = 0;
    run_sequence(golden, tyre, tier, flat_road(), uniform_map("asphalt"),
                 ::rows(xs, zs, omegas, {}, 4.0),
                 [&](const biketyre::WheelOutputs &out,
                     const biketyre::PneumaticTyreModel &model,
                     const std::string &tag) {
                     if (tag == "step 4" || tag == "step 5" ||
                         tag == "step 6") {
                         bt::require(out.airborne && out.patches().empty(),
                                     tag + " airborne fast path");
                         bt::require(model.patch_states().empty(),
                                     tag + " patch states cleared");
                         ++liftoff;
                     }
                 });
    bt::require(liftoff == 3, "airborne checks ran");
    golden.done();
}

// test_model_mixed_velocity_components_bitwise[tier{0,1}]
void test_model_mixed_velocity_components_bitwise(std::size_t tier_index) {
    auto golden = node(kFile, "test_model_mixed_velocity_components_bitwise",
                       "tier" + std::to_string(tier_index));
    const TierCfg tier = tier_index == 0 ? kFast : kDetail64;
    const auto tyre = bike::legacy::front_tyre();
    const double radius = tyre.outer_radius_mm * 0.001;
    const double lo = radius - 0.025;
    const std::vector<Row> table{
            {{5.0, 0.0, lo}, {3.0, 0.4, 0.0}, 9.0, 0.002, {}},
            {{5.0, 0.1, lo}, {2.0, -0.3, 0.8}, 9.0, 0.002, {}},
            {{5.0, 0.0, lo}, {1.5, 0.0, -1.2}, 9.0, 0.002, {}},
            {{5.0, 0.0, lo}, {0.0, 0.0, 2.0}, 9.0, 0.002, {}},
            {{5.0, 0.0, lo}, {0.0, -0.7, 0.0}, -4.0, 0.002, {}},
    };
    run_sequence(golden, tyre, tier, bump_road(), uniform_map("asphalt"),
                 table);
    golden.done();
}

// test_model_patch_migration_drift_bitwise[tier{0,1}]
void test_model_patch_migration_drift_bitwise(std::size_t tier_index) {
    auto golden = node(kFile, "test_model_patch_migration_drift_bitwise",
                       "tier" + std::to_string(tier_index));
    const TierCfg tier = tier_index == 0 ? kFast : kDetail64;
    const auto tyre = bike::legacy::front_tyre();
    const double radius = tyre.outer_radius_mm * 0.001;
    std::vector<double> xs(4, 4.80), zs(4, radius - 0.030),
            omegas(30, 10.0);
    for (std::size_t k = 1; k <= 26; ++k) {
        xs.push_back(4.80 + 0.006 * static_cast<double>(k));
        zs.push_back(radius - 0.030 -
                     0.001 * static_cast<double>(k % 3));
    }
    std::vector<std::set<std::pair<int, int>>> keys;
    run_sequence(golden, tyre, tier, bump_road(), uniform_map("asphalt"),
                 ::rows(xs, zs, omegas),
                 [&](const biketyre::WheelOutputs &,
                     const biketyre::PneumaticTyreModel &model,
                     const std::string &) {
                     keys.push_back(bt::pneumatic::patch_keys(model));
                 });
    std::size_t shifts = 0, same = 0;
    for (std::size_t i = 1; i < keys.size(); ++i)
        (keys[i] == keys[i - 1] ? same : shifts) += 1;
    bt::require(shifts >= 2, "patch never drifted");
    bt::require(same >= 1, "patch never held");
    golden.done();
}

// test_model_patch_split_and_merge_bitwise[tier{0,1}]
void test_model_patch_split_and_merge_bitwise(std::size_t tier_index) {
    auto golden = node(kFile, "test_model_patch_split_and_merge_bitwise",
                       "tier" + std::to_string(tier_index));
    const TierCfg tier = tier_index == 0 ? kFast : kDetail64;
    const auto tyre = bike::legacy::front_tyre();
    const double radius = tyre.outer_radius_mm * 0.001;
    std::vector<double> xs(34), zs(34), omegas(34, 8.5);
    for (std::size_t k = 0; k < 34; ++k) {
        xs[k] = 4.55 + 0.02 * static_cast<double>(k);
        zs[k] = radius - 0.006 - 0.001 * static_cast<double>(k % 4);
    }
    std::vector<std::size_t> counts;
    run_sequence(golden, tyre, tier, twin_bump_road(),
                 uniform_map("asphalt"), ::rows(xs, zs, omegas, {}, 1.0),
                 [&](const biketyre::WheelOutputs &out,
                     const biketyre::PneumaticTyreModel &,
                     const std::string &) {
                     counts.push_back(out.patch_count);
                 });
    bt::require(std::ranges::count(counts, 2) > 0, "never two patches");
    bt::require(std::ranges::count(counts, 1) > 0, "never one patch");
    golden.done();
}

// The shared probe: one loaded evaluate on a fresh model returns the
// live patch key (s0, e0).
[[nodiscard]] std::pair<int, int>
probe_key(const nativecfg::PneumaticTyreSpec &tyre, const TierCfg &tier,
          const biketyre::RoadProfile &road,
          const nativecfg::SurfaceMap &smap,
          const std::array<double, 3> &hub,
          const std::array<double, 3> &vel) {
    auto probe = make_model(tyre, tier);
    (void)probe->evaluate(hub, vel, 9.0, road, smap, 0.002);
    const auto states = probe->patch_states();
    bt::require(states.size() == 1, "probe produced one patch");
    return {states[0].start, states[0].stop};
}

// test_model_patch_no_overlap_drops_stale[tier{0,1}]
void test_model_patch_no_overlap_drops_stale(std::size_t tier_index) {
    auto golden = node(kFile, "test_model_patch_no_overlap_drops_stale",
                       "tier" + std::to_string(tier_index));
    const TierCfg tier = tier_index == 0 ? kFast : kDetail64;
    const auto tyre = bike::legacy::front_tyre();
    const auto road = flat_road();
    const auto smap = uniform_map("asphalt");
    const double radius = tyre.outer_radius_mm * 0.001;
    const std::array<double, 3> hub{5.0, 0.0, radius - 0.025},
            vel{3.0, 0.0, 0.0};

    const auto [s0, e0] = probe_key(tyre, tier, road, smap, hub, vel);
    const int stale_s = std::max(0, s0 - 8), stale_e = std::max(1, s0 - 5);
    bt::require(stale_e <= s0, "stale key disjoint");

    auto oracle2 = make_model(tyre, tier);
    oracle2->restore_patch_states(std::array{injected(
            stale_s, stale_e, 0.7,
            tier.discretised ? bike::geometry::detail::np_linspace(
                    0.01, 0.02, static_cast<std::size_t>(stale_e - stale_s))
                             : std::vector<double>{},
            0.7)});
    const auto &out = oracle2->evaluate(hub, vel, 9.0, road, smap, 0.002);
    outputs(golden, out, "stale-drop");
    state(golden, *oracle2, "stale-drop");
    const auto live = bt::pneumatic::patch_keys(*oracle2);
    bt::require(live == std::set<std::pair<int, int>>{{s0, e0}},
                "stale key dropped");
    golden.done();
}

// test_model_patch_migration_injected_candidates — three drive() calls.
void test_model_patch_migration_injected_candidates() {
    auto golden = node(kFile,
                       "test_model_patch_migration_injected_candidates");
    const TierCfg tier = kDetail64;
    const auto tyre = bike::legacy::front_tyre();
    const auto road = flat_road();
    const auto smap = uniform_map("asphalt");
    const double radius = tyre.outer_radius_mm * 0.001;
    const std::array<double, 3> hub{5.0, 0.0, radius - 0.025},
            vel{3.0, 0.0, 0.0};

    const auto [s0, e0] = probe_key(tyre, tier, road, smap, hub, vel);
    const std::size_t w = static_cast<std::size_t>(e0 - s0);
    bt::require(w >= 8, "patch width covers the crafted candidates");

    const auto drive = [&](std::vector<biketyre::PatchBrushState> states,
                           const std::string &tag) {
        auto model = make_model(tyre, tier);
        model->restore_patch_states(states);
        const auto &out = model->evaluate(hub, vel, 9.0, road, smap,
                                          0.002);
        outputs(golden, out, tag);
        state(golden, *model, tag);
        return bt::pneumatic::patch_keys(*model);
    };

    // tie-break — equal three-ray overlaps, the larger (start, stop) wins.
    auto keys = drive(
            {injected(s0 - 2, s0 + 3, 0.5,
                      bike::geometry::detail::np_linspace(0.01, 0.02, 5),
                      0.5),
             injected(e0 - 3, e0 + 2, 0.75,
                      bike::geometry::detail::np_linspace(-0.02, -0.01, 5),
                      0.75)},
            "tie-break");
    bt::require(keys == std::set<std::pair<int, int>>{{s0, e0}},
                "tie-break kept the live key");

    // exact-key reuse beats the larger-overlap alternative.
    drive({injected(s0, e0, 0.25,
                    bike::geometry::detail::np_linspace(0.03, 0.04, w),
                    0.25),
           injected(s0 - 4, e0 + 4, 0.9,
                    bike::geometry::detail::np_linspace(0.05, 0.06, w + 8),
                    0.9)},
          "exact-key");

    // dict last-write-wins on a repeated (start, stop) — the restored
    // patch_states list carries the same key twice.
    auto dup = make_model(tyre, tier);
    dup->restore_patch_states(std::vector<biketyre::PatchBrushState>{
            injected(s0, e0, 0.125), injected(s0, e0, -0.5)});
    const auto &out = dup->evaluate(hub, vel, 9.0, road, smap, 0.002);
    outputs(golden, out, "duplicate-key");
    state(golden, *dup, "duplicate-key");
    golden.done();
}

// test_model_migration_bristle_broadcast_semantics
void test_model_migration_bristle_broadcast_semantics() {
    auto golden = node(
            kFile, "test_model_migration_bristle_broadcast_semantics");
    const TierCfg tier = kDetail64;
    const auto tyre = bike::legacy::front_tyre();
    const auto road = flat_road();
    const auto smap = uniform_map("asphalt");
    const double radius = tyre.outer_radius_mm * 0.001;
    const std::array<double, 3> hub{5.0, 0.0, radius - 0.025},
            vel{3.0, 0.0, 0.0};

    const auto [s0, e0] = probe_key(tyre, tier, road, smap, hub, vel);
    const std::size_t w = static_cast<std::size_t>(e0 - s0);
    bt::require(w >= 8, "patch width covers the broadcast windows");

    const auto drive = [&](std::vector<double> bristle) {
        auto model = make_model(tyre, tier);
        model->restore_patch_states(std::array{injected(
                s0 - 2, e0 - 2, 0.4, std::move(bristle), 0.4)});
        return model;
    };

    // len-3 row → the clamped one-element RHS broadcasts across w-2 slots.
    {
        auto model = drive({0.01, -0.02, 0.03});
        const auto &out = model->evaluate(hub, vel, 9.0, road, smap,
                                          0.002);
        outputs(golden, out, "broadcast-fill");
        state(golden, *model, "broadcast-fill");
    }

    // len-4 row → two clamped elements cannot fill w-2 slots: the
    // oracle's ValueError; the failed migration leaves the pool intact.
    {
        auto model = drive({0.01, -0.02, 0.03, 0.04});
        bool rejected = false;
        try {
            (void)model->evaluate(hub, vel, 9.0, road, smap, 0.002);
        } catch (const std::invalid_argument &) { rejected = true; }
        bt::require(rejected, "broadcast rejection raised");
        bt::require(
                bt::pneumatic::patch_keys(*model) ==
                        std::set<std::pair<int, int>>{{s0 - 2, e0 - 2}},
                "rejected migration left patch pool untouched");
    }
    golden.done();
}

// test_model_migration_from_detailed_none
void test_model_migration_from_detailed_none() {
    auto golden = node(kFile, "test_model_migration_from_detailed_none");
    const TierCfg tier = kDetail64;
    const auto tyre = bike::legacy::front_tyre();
    const auto road = flat_road();
    const auto smap = uniform_map("asphalt");
    const double radius = tyre.outer_radius_mm * 0.001;
    const std::array<double, 3> hub{5.0, 0.0, radius - 0.025},
            vel{3.0, 0.0, 0.0};

    const auto [s0, e0] = probe_key(tyre, tier, road, smap, hub, vel);
    auto model = make_model(tyre, tier);
    model->restore_patch_states(
            std::array{injected(s0 - 1, e0 - 1, 0.33)});
    const auto &out = model->evaluate(hub, vel, 9.0, road, smap, 0.002);
    outputs(golden, out, "detailed-none");
    state(golden, *model, "detailed-none");
    bt::require(bt::pneumatic::patch_at(*model, s0, e0).has_detailed,
                "migrated patch gained a detailed brush");
    golden.done();
}

// test_model_rim_strike_bitwise[tier{0,1}]
void test_model_rim_strike_bitwise(std::size_t tier_index) {
    auto golden = node(kFile, "test_model_rim_strike_bitwise",
                       "tier" + std::to_string(tier_index));
    const TierCfg tier = tier_index == 0 ? kFast : kDetail64;
    const auto tyre = bike::legacy::front_tyre();
    const double radius = tyre.outer_radius_mm * 0.001;
    const double rim_defl = tyre.rim_strike_deflection_mm() * 0.001;
    const double cz_below = radius - rim_defl - 0.004;
    const double cz_deep = radius - rim_defl - 0.012;
    const std::vector<double> xs(6, 5.0),
            zs{0.36, cz_below, cz_deep, cz_below, 0.34, 0.36},
            omegas{-7.0, -7.5, -8.0, -8.0, -8.0, -7.0};
    std::vector<std::optional<biketyre::RimStrikeEvent>> events;
    run_sequence(golden, tyre, tier, flat_road(), uniform_map("asphalt"),
                 ::rows(xs, zs, omegas, {}, -3.2),
                 [&](const biketyre::WheelOutputs &out,
                     const biketyre::PneumaticTyreModel &,
                     const std::string &) {
                     events.push_back(out.rim_event);
                 });
    std::size_t done = 0;
    double peak = 0.0;
    for (const auto &ev : events)
        if (ev) {
            ++done;
            peak = ev->peak_load_n;
        }
    bt::require(done == 1 && peak > 0.0, "one completed rim event");
    golden.done();
}

// test_model_rolling_loss_signs_bitwise[tier{0,1}]
void test_model_rolling_loss_signs_bitwise(std::size_t tier_index) {
    auto golden = node(kFile, "test_model_rolling_loss_signs_bitwise",
                       "tier" + std::to_string(tier_index));
    const TierCfg tier = tier_index == 0 ? kFast : kDetail64;
    const auto tyre = bike::legacy::front_tyre();
    const double radius = tyre.outer_radius_mm * 0.001;
    const double lo = radius - 0.025;
    const std::vector<Row> table{
            {{5.0, 0.0, lo}, {3.0, 0.0, 0.0}, 8.0, 0.002, {}},
            {{5.0, 0.0, lo}, {-2.0, 0.0, 0.0}, -7.0, 0.002, {}},
            {{5.0, 0.0, lo}, {0.0, 0.0, 0.0}, 8.0, 0.002, {}},
            {{5.0, 0.0, lo}, {0.0, 0.0, 0.0}, -8.0, 0.002, {}},
            {{5.0, 0.0, lo}, {0.0, 0.0, 0.0}, 0.0, 0.002, {}},
            {{5.0, 0.0, lo}, {3.0, 0.0, 0.0}, 0.4, 0.002, {}},
            {{5.0, 0.0, lo}, {3.0, 0.0, 0.0}, 1.0, 0.002, {}},
            {{5.0, 0.0, lo}, {3.0, 0.0, 0.0}, 1.5, 0.002, {}},
    };
    std::vector<double> losses, loads;
    run_sequence(golden, tyre, tier, flat_road(), uniform_map("asphalt"),
                 table,
                 [&](const biketyre::WheelOutputs &out,
                     const biketyre::PneumaticTyreModel &model,
                     const std::string &tag) {
                     const std::size_t k =
                             static_cast<std::size_t>(
                                     std::stoul(tag.substr(5)));
                     const auto &p = out.patches().front();
                     const auto &vel = table[k].velocity;
                     const double v_x =
                             vel[0] * p.tangent_world[0] +
                             vel[1] * p.tangent_world[1] +
                             vel[2] * p.tangent_world[2];
                     loads.push_back(p.normal_load_n);
                     losses.push_back(biketyre::tread_loss_force(
                             biketyre::tread_loss_crr_at_pressure(
                                     model.tyre()),
                             p.normal_load_n, table[k].omega, v_x));
                     scalar(golden, v_x, tag + " loss.v_x");
                     scalar(golden, losses.back(), tag + " loss.force");
                 });
    bt::require(losses[0] * losses[1] < 0.0, "signs oppose");
    bt::require(std::signbit(losses[2]) && !std::signbit(losses[3]),
                "spin-sign fallback");
    // The oracle asserts `== 0.0`; the recorded loss.force entry pins
    // the sign bit too (a -0.0 payload would mismatch the golden).
    bt::require(losses[4] == 0.0, "zero row returns no force");
    bt::require(std::abs(losses[5]) / loads[5] <
                        std::abs(losses[6]) / loads[6],
                "taper is linear below 1 rad/s");
    bt::require(std::abs(std::abs(losses[6]) / loads[6] -
                         std::abs(losses[7]) / loads[7]) <=
                        1e-12 * std::abs(losses[7]) / loads[7],
                "taper saturates");
    golden.done();
}

// test_model_tread_loss_disabled_bitwise[tier{0,1}]
void test_model_tread_loss_disabled_bitwise(std::size_t tier_index) {
    auto golden = node(kFile, "test_model_tread_loss_disabled_bitwise",
                       "tier" + std::to_string(tier_index));
    const TierCfg tier = tier_index == 0 ? kFast : kDetail64;
    auto tyre = bike::legacy::front_tyre();
    tyre.tread_loss_crr = 0.0;
    const double radius = tyre.outer_radius_mm * 0.001;
    const std::vector<double> xs{5.0, 5.0, 5.0},
            zs(3, radius - 0.025), omegas(3, 9.0);
    run_sequence(golden, tyre, tier, flat_road(), uniform_map("asphalt"),
                 ::rows(xs, zs, omegas));
    golden.done();
}

// test_model_pressure_power_and_set_pressure_bitwise[tier{0,1}]
void test_model_pressure_power_and_set_pressure_bitwise(
        std::size_t tier_index) {
    auto golden = node(
            kFile, "test_model_pressure_power_and_set_pressure_bitwise",
            "tier" + std::to_string(tier_index));
    const TierCfg tier = tier_index == 0 ? kFast : kDetail64;
    const auto tyre = bike::legacy::front_tyre();
    const double radius = tyre.outer_radius_mm * 0.001;
    const double lo = radius - 0.025;
    const std::vector<Row> table{
            {{5.0, 0.0, lo}, {3.0, 0.0, 0.0}, 9.0, 0.002, 2.4},
            {{5.0, 0.0, lo}, {3.0, 0.0, 0.0}, 9.0, 0.002, std::nullopt},
            {{5.0, 0.0, lo}, {3.0, 0.0, 0.0}, 9.0, 0.002, 0.2},
            {{5.0, 0.0, lo}, {3.0, 0.0, 0.0}, 9.0, 0.002, 9.9},
            {{5.0, 0.0, lo}, {3.0, 0.0, 0.0}, 9.0, 0.002,
             std::numeric_limits<double>::quiet_NaN()},
    };
    run_sequence(golden, tyre, tier, flat_road(), uniform_map("asphalt"),
                 table);
    golden.done();
}

// test_model_set_pressure_clamp_bitwise — the labels carry repr(p).
// The oracle's debug set_pressure is with_pressure + a fresh tyre spec
// read; clamp_pressure_bar is the production clamp it routes through.
void test_model_set_pressure_clamp_bitwise() {
    auto golden = node(kFile, "test_model_set_pressure_clamp_bitwise");
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    // The oracle's tuple mixes float literals and np.nextafter values:
    // the last two labels carry repr(np.float64) — 'np.float64(...)'.
    const std::array pressures{
            0.0, 0.4, 0.8, 1.1, 1.5, 2.9, 3.0, 4.5, inf, -inf, nan};
    for (const double p : pressures)
        scalar(golden, biketyre::clamp_pressure_bar(p),
               "clamp " + repr_binary64(p));
    const std::array np_pressures{
            std::nextafter(0.8, 0.0), std::nextafter(3.0, 100.0)};
    for (const double p : np_pressures)
        scalar(golden, biketyre::clamp_pressure_bar(p),
               "clamp " + np_f64(p));
    golden.done();
}

// test_model_surface_sections_bitwise[tier{0,1}]
void test_model_surface_sections_bitwise(std::size_t tier_index) {
    auto golden = node(kFile, "test_model_surface_sections_bitwise",
                       "tier" + std::to_string(tier_index));
    const TierCfg tier = tier_index == 0 ? kFast : kDetail64;
    const auto tyre = bike::legacy::front_tyre();
    const double radius = tyre.outer_radius_mm * 0.001;
    std::vector<double> xs(30), zs(30, radius - 0.025), omegas(30, 9.0);
    for (std::size_t k = 0; k < 30; ++k) xs[k] = 4.3 + 0.05 * static_cast<double>(k);
    std::set<std::string> names;
    run_sequence(golden, tyre, tier, flat_road(), zoned_map(),
                 ::rows(xs, zs, omegas, {}, 2.5),
                 [&](const biketyre::WheelOutputs &out,
                     const biketyre::PneumaticTyreModel &,
                     const std::string &) {
                     for (const auto &p : out.patches())
                         names.insert(p.surface_name);
                 });
    bt::require(names.count("loose") && names.count("scree") &&
                        names.count("hardpack"),
                "all three zones engaged");
    golden.done();
}

// test_model_reset_bitwise[tier{0,1}]
void test_model_reset_bitwise(std::size_t tier_index) {
    auto golden = node(kFile, "test_model_reset_bitwise",
                       "tier" + std::to_string(tier_index));
    const TierCfg tier = tier_index == 0 ? kFast : kDetail64;
    const auto tyre = bike::legacy::front_tyre();
    const double radius = tyre.outer_radius_mm * 0.001;
    const std::vector<double> xs{4.9, 4.95, 5.0, 5.05},
            zs(4, radius - 0.030), omegas(4, 8.0);
    auto model = make_model(tyre, tier);
    const auto road = bump_road();
    const auto smap = uniform_map("asphalt");
    for (std::size_t i = 0; i < 4; ++i) {
        const std::string tag = "step " + std::to_string(i);
        const auto &out = model->evaluate(
                std::array{xs[i], 0.0, zs[i]},
                std::array{2.5, 0.0, 0.0}, omegas[i], road, smap, 0.002);
        outputs(golden, out, tag);
        state(golden, *model, tag);
    }
    model->reset();
    outputs(golden, model->last_outputs(), "reset");
    state(golden, *model, "reset");
    bt::require(model->patch_states().empty(), "reset cleared patches");
    golden.done();
}

// test_model_coverage_event_bitwise[tier{0,1}]
void test_model_coverage_event_bitwise(std::size_t tier_index) {
    auto golden = node(kFile, "test_model_coverage_event_bitwise",
                       "tier" + std::to_string(tier_index));
    const TierCfg tier = tier_index == 0 ? kFast : kDetail64;
    const auto tyre = bike::legacy::front_tyre();
    const double radius = tyre.outer_radius_mm * 0.001;
    const double lo = radius - 0.02;
    const std::vector<double> xs{5.02, 5.02, 5.02},
            zs{lo, radius + 0.06, radius + 0.5}, omegas(3, 8.0);
    int cov = 0;
    run_sequence(golden, tyre, tier, spike_road(), uniform_map("asphalt"),
                 ::rows(xs, zs, omegas, {}, 6.0),
                 [&](const biketyre::WheelOutputs &out,
                     const biketyre::PneumaticTyreModel &,
                     const std::string &) {
                     cov += out.coverage_event;
                 });
    bt::require(cov > 0, "coverage_event raised");
    golden.done();
}

// test_model_validation_bitwise_atomicity[tier{0,1}] — the rejection
// order, state preservation on both sides, and the carcass dt gate.
// The C++ twin of the oracle's swap-rollback: after each rejection the
// persistent model's committed state must stay byte-identical to the
// oracle's, which the `state` consumer asserts against the capture.
void test_model_validation_bitwise_atomicity(std::size_t tier_index) {
    auto golden = node(kFile, "test_model_validation_bitwise_atomicity",
                       "tier" + std::to_string(tier_index));
    const TierCfg tier = tier_index == 0 ? kFast : kDetail64;
    const auto tyre = bike::legacy::front_tyre();
    const auto road = flat_road();
    const auto smap = uniform_map("asphalt");
    const double radius = tyre.outer_radius_mm * 0.001;
    const std::array<double, 3> hub{5.0, 0.0, radius - 0.025},
            vel{3.0, 0.0, 0.0};
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    // Establish history on both sides (the oracle's loaded step).
    auto model = make_model(tyre, tier);
    (void)model->evaluate(hub, vel, 9.0, road, smap, 0.002);

    // Each rejection replays on the persistent model, then the
    // committed-state consumer checks failure atomicity against the
    // recorded oracle snapshot.
    const auto expect_reject = [&](std::span<const double> h,
                                   std::span<const double> v, double o,
                                   double dt, const char *tag) {
        rejects<std::invalid_argument>(
                [&] { (void)model->evaluate(h, v, o, road, smap, dt); });
        state(golden, *model, std::string(tag) + " atomicity");
    };

    const std::array<double, 4> hub4{0.0, 0.0, 0.0, 0.0};
    const std::array<double, 2> vel2{0.0, 0.0};
    const std::array<double, 3> nan_hub{5.0, nan, hub[2]},
            inf_hub{5.0, 0.0, inf}, nan_vel{nan, 0.0, 0.0},
            inf_vel{0.0, 0.0, -inf};
    const struct {
        const char *tag;
        std::span<const double> h, v;
        double o, dt;
    } bad[]{
            {"shape", hub4, vel, 9.0, 0.002},
            {"shape-vel", hub, vel2, 9.0, 0.002},
            {"nan-hub", nan_hub, vel, 9.0, 0.002},
            {"inf-hub", inf_hub, vel, 9.0, 0.002},
            {"nan-vel", hub, nan_vel, 9.0, 0.002},
            {"inf-vel", hub, inf_vel, 9.0, 0.002},
            {"nan-omega", hub, vel, nan, 0.002},
            {"inf-omega", hub, vel, inf, 0.002},
            // carcass-level checks that fire on the loaded path.
            {"zero-dt", hub, vel, 9.0, 0.0},
            {"neg-dt", hub, vel, 9.0, -0.002},
            {"nan-dt", hub, vel, 9.0, nan},
            {"inf-dt", hub, vel, 9.0, inf},
    };
    for (const auto &row : bad)
        expect_reject(row.h, row.v, row.o, row.dt, row.tag);

    // Ordering: the shape check precedes finiteness and spin.
    const std::array<double, 1> nan1{nan};
    rejects<std::invalid_argument>(
            [&] {
                (void)model->evaluate(hub4, nan1, nan, road, smap,
                                      0.002);
            },
            "3-vectors");
    rejects<std::invalid_argument>(
            [&] {
                (void)model->evaluate(std::array{inf, 0.0, 0.0}, vel,
                                      nan, road, smap, 0.002);
            },
            "finite");

    // State-intake rejection: the oracle swaps in a carcass ring one
    // ray too narrow; the C++ intake twin is assign() — fixed rings
    // never resize, the ring mismatch throws the same class.
    {
        biketyre::CarcassState narrow(tier.n_rays - 1);
        rejects<std::invalid_argument>(
                [&] { model->carcass_state().assign(narrow); });
    }
    {
        const auto wide = injected(
                10, 20, 0.1,
                std::vector<double>(
                        static_cast<std::size_t>(tier.n_rays + 1), 0.0));
        rejects<std::invalid_argument>(
                [&] { model->restore_patch_states(std::array{wide}); });
    }

    // The airborne fast path skips the carcass dt gate entirely on an
    // uninitialized state (advance_unloaded's early return) — but the
    // same call on an initialized state rejects it.
    const std::array<double, 3> air_hub{5.0, 0.0, radius + 0.5};
    auto fresh = make_model(tyre, tier);
    const auto &silent =
            fresh->evaluate(air_hub, vel, 9.0, road, smap, 0.0);
    outputs(golden, silent, "silent dt");
    (void)model->evaluate(air_hub, vel, 9.0, road, smap, 0.002);
    rejects<std::invalid_argument>(
            [&] { (void)model->evaluate(air_hub, vel, 9.0, road, smap,
                                        0.0); });

    // Post-failure valid call still matches bitwise.
    const auto &out = model->evaluate(hub, vel, 9.0, road, smap, 0.002);
    outputs(golden, out, "post-failure");
    state(golden, *model, "post-failure");
    golden.done();
}

} // namespace

// cppcheck-suppress throwInEntryPoint
int main() {
    return bt::run([] {
        for (std::size_t tier = 0; tier < 2; ++tier) {
            test_model_progressive_load_unload_bitwise(tier, 0);
            test_model_progressive_load_unload_bitwise(tier, 1);
            test_model_airborne_sequence_bitwise(tier);
            test_model_mixed_velocity_components_bitwise(tier);
            test_model_patch_migration_drift_bitwise(tier);
            test_model_patch_split_and_merge_bitwise(tier);
            test_model_patch_no_overlap_drops_stale(tier);
            test_model_rim_strike_bitwise(tier);
            test_model_rolling_loss_signs_bitwise(tier);
            test_model_tread_loss_disabled_bitwise(tier);
            test_model_pressure_power_and_set_pressure_bitwise(tier);
            test_model_surface_sections_bitwise(tier);
            test_model_reset_bitwise(tier);
            test_model_coverage_event_bitwise(tier);
            test_model_validation_bitwise_atomicity(tier);
        }
        for (std::size_t tier = 0; tier < 2; ++tier)
            test_model_authored_tiers_bitwise(tier);
        test_model_patch_migration_injected_candidates();
        test_model_migration_bristle_broadcast_semantics();
        test_model_migration_from_detailed_none();
        test_model_set_pressure_clamp_bitwise();
    });
}
