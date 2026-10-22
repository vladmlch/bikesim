#pragma once

// Source fixture shared by the pneumatic force/property and frozen-oracle
// ports. All physics runs through the existing production writer/model.
#include "support/model_fixture.hpp"
#include "cblas_abi.hpp"
#include "geometry/specs.hpp"
#include "legacy/tyre.hpp"
#include "model/mapping.hpp"
#include "writers/pneumatic_tire.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <format>
#include <memory>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace bike::testing {
struct PneumaticWriterAccess {
    [[nodiscard]] static const biketyre::PneumaticTyreModel &wheel(
            const PneumaticTireWriter &writer, std::size_t side) {
        bt::require(side < writer.tyres_.size(), "valid pneumatic test side");
        return *writer.tyres_[side];
    }
    [[nodiscard]] static const nativecfg::TireConfig &config(const PneumaticTireWriter &writer) {
        return writer.cfg_;
    }
    [[nodiscard]] static std::optional<double> clock(const PneumaticTireWriter &writer) {
        return writer.last_time_s_;
    }
    static void rear_count_control(PneumaticTireWriter &writer, bool malformed) {
        const auto width = static_cast<std::int64_t>(writer.tyres_[1]->n_rays()) + (malformed ? 1 : 0);
        // Allocate the complete n+1 state in BOTH live and backup banks.
        // assign() therefore remains in bounds. The ring keeps n rays, so
        // evaluate_carcass_into rejects its count mismatch before touching
        // any ray or output buffer, after the front evaluation completed.
        // Merely changing n_rays without resizing storage would be unsafe.
        writer.tyres_[1]->carcass_state() = biketyre::CarcassState(width);
        writer.backup_[1]->carcass_state() = biketyre::CarcassState(width);
        if (!malformed) {
            writer.tyres_[1]->reset();
            writer.backup_[1]->reset();
        }
    }
};
} // namespace bike::testing

namespace bt::pneumatic {
using Access = bike::testing::PneumaticWriterAccess;
inline constexpr std::array<std::string_view, 2> sides{"front", "rear"};
[[nodiscard]] inline double front_radius() { return bike::legacy::front_tyre().outer_radius_mm / 1000.; }
[[nodiscard]] inline double rear_radius() { return bike::legacy::rear_tyre().outer_radius_mm / 1000.; }

[[nodiscard]] inline nativecfg::TireConfig configuration(std::string_view tier_name = "fast") {
    const auto tier = bike::legacy::tier_spec(tier_name);
    const auto &surface = bike::terrain::get_surface("asphalt");
    const auto x = bike::geometry::detail::np_linspace(-2., 3., 1001);
    nativecfg::TireConfig out;
    out.backend = "pneumatic";
    out.surface_mode = "configured";
    // Unconsumed TireParameters still satisfy the real ingress contract.
    out.front.tangent_k_n_m = out.rear.tangent_k_n_m = 1.;
    out.front.mu = out.rear.mu = .8;
    out.front.relaxation_length_m = out.rear.relaxation_length_m = .06;
    out.significant_delta_m = .0001;
    out.significance_fraction = .05;
    out.distinct_normal_deg = 20.;
    out.pneumatic.emplace();
    auto &p = *out.pneumatic;
    p.tier = tier.name; p.n_rays = tier.n_rays; p.half_angle_deg = tier.half_angle_deg;
    p.discretised_brush = tier.discretised_brush; p.timestep_s = tier.timestep_s;
    p.front = bike::legacy::front_tyre(); p.rear = bike::legacy::rear_tyre();
    p.surface = nativecfg::SurfaceSpec{.name=surface.name, .mu_peak=surface.mu_peak, .mu_slide=surface.mu_slide,
        .slip_stiffness_per_load=surface.slip_stiffness_per_load, .stribeck_speed_mps=surface.stribeck_speed_mps};
    // RoadProfile.from_samples uses x[1]-x[0], not the rounded literal .005.
    p.profile = {.x0_m=x.front(), .dx_m=x[1] - x[0], .z_m=std::vector<double>(x.size(), 0.)};
    return out;
}

[[nodiscard]] inline std::string wheel_xml(double dt, double front_drop, double rear_z,
                                          std::string_view rear_extra) {
    return std::format(R"(<mujoco><compiler angle="radian"/>
      <option timestep="{}" gravity="0 0 -9.81" integrator="Euler"/>
      <worldbody><body name="front_wheel" pos="0 0 {}">
        <joint name="front_root_x" type="slide" axis="1 0 0"/>
        <joint name="front_root_z" type="slide" axis="0 0 1"/>
        <joint name="front_wheel_spin" type="hinge" axis="0 1 0"/>
        <inertial mass="6" pos="0 0 0" diaginertia=".3 .5 .3"/>
        <geom name="geom_front_contact" type="sphere" size="{}" contype="0" conaffinity="0" mass="0"/>
      </body><body name="rear_wheel" pos="0.8 0 {}">
        <joint name="rear_wheel_spin" type="hinge" axis="0 1 0"/>
        <inertial mass="6" pos="0 0 0" diaginertia=".3 .5 .3"/>
        <geom name="geom_rear_contact" type="sphere" size="{}" contype="0" conaffinity="0" mass="0"/>
        {}</body></worldbody></mujoco>)", dt, front_radius() - front_drop,
        front_radius(), rear_z, rear_radius(), rear_extra);
}

struct Rig {
    ModelFixture engine;
    std::unique_ptr<PneumaticTireWriter> tire;
    explicit Rig(std::string_view tier = "fast", double dt = .0005,
                 double front_drop = .004, double rear_z = 5., std::string_view rear_extra = {})
        : engine(wheel_xml(dt, front_drop, rear_z, rear_extra)) {
        ::engine::forward(engine.model.get(), engine.data.get());
        tire = std::make_unique<PneumaticTireWriter>(engine.model.get(), engine.data.get(), configuration(tier));
    }
    [[nodiscard]] double dt() const { return engine.model->opt.timestep; }
    [[nodiscard]] std::vector<double> advance() { return tire->qfrc(engine.data.get(), dt()); }
    void spin(std::string_view side, double speed) {
        const auto id = engine.dof((std::string(side) + "_wheel_spin").c_str());
        model_access::mutable_buffer(engine.data->qvel, engine.model->nv)[static_cast<std::size_t>(id)] = speed;
        ::engine::forward(engine.model.get(), engine.data.get());
    }
};

template<class Exception = std::invalid_argument, class F>
void rejects(F &&operation, std::string_view message) {
    bool rejected = false;
    try { std::forward<F>(operation)(); }
    catch (const Exception &error) {
        rejected = true;
        require(std::string_view(error.what()).find(message) != std::string_view::npos,
            std::string("rejection identifies ") + std::string(message) + ": " + error.what());
    }
    require(rejected, "invalid pneumatic operation rejected");
}
inline void numeric_equal(std::span<const double> actual, std::span<const double> expected, bool equal_nan = false) {
    require(actual.size() == expected.size(), "equal numeric array shape");
    for (std::size_t i = 0; i < actual.size(); ++i)
        require(actual[i] == expected[i] || (equal_nan && std::isnan(actual[i]) && std::isnan(expected[i])),
            "equal numeric cell " + std::to_string(i));
}
[[nodiscard]] inline std::size_t column(const std::vector<std::string> &names, std::string_view name) {
    const auto found = std::ranges::find(names, name);
    require(found != names.end(), "state column exists: " + std::string(name));
    return static_cast<std::size_t>(found - names.begin());
}
[[nodiscard]] inline std::set<std::pair<int, int>> patch_keys(const biketyre::PneumaticTyreModel &wheel) {
    std::set<std::pair<int, int>> out;
    for (const auto &patch : wheel.patch_states()) out.emplace(patch.start, patch.stop);
    return out;
}
[[nodiscard]] inline const biketyre::PatchBrushState &patch_at(const biketyre::PneumaticTyreModel &wheel,
                                                             int start, int stop) {
    for (const auto &patch : wheel.patch_states())
        if (patch.start == start && patch.stop == stop) return patch;
    throw std::runtime_error("rebuilt tyre patch is missing");
}
[[nodiscard]] inline std::array<double, 3> snapshot_force(const TirePatch &patch) {
    // ContactPatch.world_force_n property, used only for its J^T F identity.
    const std::array<double, 3> tangent{patch.normal[2], 0., -patch.normal[0]};
    std::array<double, 3> out{};
    for (std::size_t i = 0; i < out.size(); ++i)
        out[i] = patch.normal_load_n * patch.normal[i] + patch.tangent_force_n * tangent[i];
    return out;
}
inline void add_jtf(const mjModel &model, const mjData &data, std::string_view side,
                    const std::array<double, 3> &point, const std::array<double, 3> &force, // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API.
                    std::span<double> expected) {
    const int body = bike::model::required_id(model, mjOBJ_BODY, std::string(side) + "_wheel");
    const auto jacobian = bike::model::point_jacobian(model, data, body, point);
    const auto nv = static_cast<int>(model.nv);
    require(expected.size() == static_cast<std::size_t>(nv), "J^T F target width");
    std::vector<double> product(expected.size(), 0.);
    blas::dgemv(blas::Order::row_major, blas::Transpose::yes, 3, nv, 1.,
        jacobian.translation.data(), nv, force.data(), 1, 0., product.data(), 1);
    for (std::size_t i = 0; i < expected.size(); ++i) expected[i] += product[i];
}
inline void unloaded_state(const biketyre::PneumaticTyreModel &wheel) {
    require(!wheel.carcass_state().initialized(), "uninitialized carcass");
    require(std::ranges::all_of(wheel.carcass_state().delta_prev_m(), [](double x) { return x == 0.; }), "zero carcass history");
    require(wheel.patch_states().empty(), "empty patch history");
    require(wheel.last_outputs().patches().empty(), "empty last output patches");
}
} // namespace bt::pneumatic
