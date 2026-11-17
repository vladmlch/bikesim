#include "configuration/load.hpp"
#include "io/hash.hpp"
#include "model/builder.hpp"
#include "spatial/config.hpp"
#include "support/fixture_test.hpp"
#include "terrain/extrusion3d.hpp"
#include "terrain/heightfield3d.hpp"
#include "terrain/profile.hpp"
#include "terrain/source3d.hpp"

#include <mujoco/mujoco.h>

#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <numbers>
#include <ranges>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {
namespace mdl = bike::model;
namespace cfg = bike::configuration;
namespace trn = bike::terrain;
namespace sp = bike::spatial;

std::string sha256_hex(std::string_view bytes) {
    return bike::io::sha256(std::as_bytes(std::span{bytes}));
}

int find_id(const mjModel& model, mjtObj kind, const char* name) {
    return mj_name2id(&model, kind, name);
}

int require_id(const mjModel& model, mjtObj kind, const char* name) {
    const int id = find_id(model, kind, name);
    bt::require(id >= 0, std::string("required object missing: ") + name);
    return id;
}

sp::SpatialConfig make_spatial_config(sp::SpatialTireBackend backend, double range_deg = 45.) {
    sp::SpatialConfig out{};
    out.tire_backend = backend;
    out.tire_preset = sp::SpatialTirePreset::mtb_2p8_1p2bar;
    out.start_mode = sp::SpatialStartMode::moving;
    out.corridor_half_width_m = 1.5;
    out.capsize_roll_rad = 1.0;
    out.steer.range_deg = range_deg;
    out.steer.control_period_s = 0.005;
    out.steer.reaction_delay_s = 0.020;
    out.steer.max_torque_nm = 35.0;
    return out;
}

void test_planar_zero_byte_shield() {
    // 1. rider: none (standard export) — pinned 08a8955b bike-export SHA-256.
    {
        mdl::BuildRequest req;
        req.solver.emplace(req.bike);
        req.mode = "standard";
        const std::string xml = mdl::generate_mjcf(req);
        bt::require(
            sha256_hex(xml) ==
                "d9e52357c4a237b8cbd35d0865dbdfb84f2a6ba3ca40e1809fb3df7248e3cfbf",
            "planar rider:none standard XML matches 08a8955b byte-for-byte");
        bt::require(xml.find("root_free") == std::string::npos,
                    "planar XML must not contain root_free");
    }

    // 2. rider: none (ride mode, include_rider = false) — no root_free, planar joints preserved.
    {
        mdl::BuildRequest req;
        req.mode = "ride";
        req.include_rider = false;
        const std::string xml = mdl::generate_mjcf(req);
        bt::require(xml.find("root_x") != std::string::npos &&
                        xml.find("root_z") != std::string::npos &&
                        xml.find("root_pitch") != std::string::npos &&
                        xml.find("root_free") == std::string::npos,
                    "planar ride rider:none preserves root_x/root_z/root_pitch");
    }

    // 3. rider: lumped (ride export) — pinned 08a8955b bike-export SHA-256.
    {
        mdl::BuildRequest req;
        req.solver.emplace(req.bike);
        req.mode = "ride";
        req.rider = cfg::RiderSpecs{};
        const std::string xml = mdl::generate_mjcf(req);
        bt::require(
            sha256_hex(xml) ==
                "6dea27b0eb2c618f30764b40b6418781de645ec97094e4588d83c5aefd2cba47",
            "planar rider:lumped ride XML matches 08a8955b byte-for-byte");
    }

    // 4. rider: articulated_planar — byte-for-byte identical to tests/fixtures/model/welded.xml.
    {
        auto req = std::make_unique<mdl::BuildRequest>();
        req->mode = "ride";
        req->rider = cfg::RiderSpecs{};
        req->rider->variant = "articulated_planar";
        req->physics = cfg::load_physics(bt::fixture("config/viewer_physics_welded.toml"));
        req->field.ncol = 26001;
        req->field.radius_x_m = 65.;
        req->field.elevation_m = 30.1;
        const std::string xml = mdl::generate_mjcf(*req);
        const std::string expected = bt::text(bt::fixture("model/welded.xml"));
        bt::require(xml == expected,
                    "planar rider:articulated_planar XML matches welded.xml byte-for-byte");
    }
}

void test_spatial_topology_and_compilation() {
    for (const auto backend :
         {sp::SpatialTireBackend::compliant_3d, sp::SpatialTireBackend::distributed_3d}) {
        const sp::SpatialConfig spatial_cfg = make_spatial_config(backend, 45.0);
        auto req = std::make_unique<mdl::BuildRequest>();
        req->mode = "ride";
        req->include_rider = false;
        req->physics = cfg::default_physics();
        req->physics->drive_mode = "coast";
        req->spatial = spatial_cfg;

        const std::string xml = mdl::generate_mjcf(*req);
        const auto model = mdl::compile_mjcf(xml);

        // nq == nv + 1 due to freejoint quaternion (4 pos vs 3 vel).
        bt::require(model->nq == model->nv + 1, "spatial model has nq == nv + 1");

        // root_free present as mjJNT_FREE; planar root joints absent.
        const int root_free = require_id(*model, mjOBJ_JOINT, "root_free");
        const auto jnt_types = std::views::counted(model->jnt_type, model->njnt);
        bt::require(jnt_types[static_cast<std::size_t>(root_free)] == mjJNT_FREE,
                    "root_free is mjJNT_FREE");
        bt::require(find_id(*model, mjOBJ_JOINT, "root_x") == -1 &&
                        find_id(*model, mjOBJ_JOINT, "root_z") == -1 &&
                        find_id(*model, mjOBJ_JOINT, "root_pitch") == -1,
                    "planar root joints omitted in spatial mode");

        // steer_joint preserved with exact headtube axis, degree-converted limits, zero stiffness/damping.
        const int steer_id = require_id(*model, mjOBJ_JOINT, "steer_joint");
        const auto steer_idx = static_cast<std::size_t>(steer_id);
        bt::require(jnt_types[steer_idx] == mjJNT_HINGE, "steer_joint is mjJNT_HINGE");

        const auto jnt_limited = std::views::counted(model->jnt_limited, model->njnt);
        const auto jnt_range = std::views::counted(model->jnt_range, model->njnt * 2);
        const auto jnt_axis = std::views::counted(model->jnt_axis, model->njnt * 3);
        bt::require(jnt_limited[steer_idx] == 1, "steer_joint is limited");
        const double expected_rad = 45.0 * std::numbers::pi / 180.0;
        bt::require(std::abs(jnt_range[2U * steer_idx + 0U] + expected_rad) < 1e-6 &&
                        std::abs(jnt_range[2U * steer_idx + 1U] - expected_rad) < 1e-6,
                    "steer_joint range matches +-45 deg");

        const auto fixed = bike::geometry::get_fixed_frame_points(req->bike);
        const bike::geometry::Vec3 rel{
            fixed.P11[0] - fixed.P_HT_bot[0],
            fixed.P11[1] - fixed.P_HT_bot[1],
            fixed.P11[2] - fixed.P_HT_bot[2],
        };
        const double norm = std::sqrt(rel[0] * rel[0] + rel[1] * rel[1] + rel[2] * rel[2]);
        bt::require(std::abs(jnt_axis[3U * steer_idx + 0U] - rel[0] / norm) < 1e-5 &&
                        std::abs(jnt_axis[3U * steer_idx + 1U] - rel[1] / norm) < 1e-5 &&
                        std::abs(jnt_axis[3U * steer_idx + 2U] - rel[2] / norm) < 1e-5,
                    "steer_joint axis matches headtube vector");

        // Zero damping and zero stiffness on ALL joints/DOFs (Decision 26 / Design §1.2).
        const auto dof_damping = std::views::counted(model->dof_damping, model->nv);
        for (int dof = 0; dof < model->nv; ++dof) {
            bt::require(dof_damping[static_cast<std::size_t>(dof)] == 0.0,
                        "all DOFs have zero damping in spatial mode");
        }
        const auto jnt_stiffness = std::views::counted(model->jnt_stiffness, model->njnt);
        for (int jnt = 0; jnt < model->njnt; ++jnt) {
            bt::require(jnt_stiffness[static_cast<std::size_t>(jnt)] == 0.0,
                        "all joints have zero stiffness in spatial mode");
        }

        // Wheel contacts disabled for both compliant_3d and distributed_3d.
        const auto geom_contype = std::views::counted(model->geom_contype, model->ngeom);
        const auto geom_conaffinity = std::views::counted(model->geom_conaffinity, model->ngeom);
        for (const char* name : {"geom_front_contact", "geom_rear_contact"}) {
            const auto gid = static_cast<std::size_t>(require_id(*model, mjOBJ_GEOM, name));
            bt::require(geom_contype[gid] == 0 && geom_conaffinity[gid] == 0,
                        "spatial custom tire disables native wheel contacts");
        }

        // Steer sensors preserved in spatial mode.
        const int s_angle = require_id(*model, mjOBJ_SENSOR, "sensor_steer_angle");
        const int s_vel = require_id(*model, mjOBJ_SENSOR, "sensor_steer_velocity");
        const auto sensor_type = std::views::counted(model->sensor_type, model->nsensor);
        const auto sensor_objid = std::views::counted(model->sensor_objid, model->nsensor);
        bt::require(sensor_type[static_cast<std::size_t>(s_angle)] == mjSENS_JOINTPOS &&
                        sensor_objid[static_cast<std::size_t>(s_angle)] == steer_id,
                    "sensor_steer_angle tracks steer_joint");
        bt::require(sensor_type[static_cast<std::size_t>(s_vel)] == mjSENS_JOINTVEL &&
                        sensor_objid[static_cast<std::size_t>(s_vel)] == steer_id,
                    "sensor_steer_velocity tracks steer_joint");

        // catch_plane sized to cover spatial track X/Y corridor.
        const auto catch_id = static_cast<std::size_t>(require_id(*model, mjOBJ_GEOM, "catch_plane"));
        const auto geom_pos = std::views::counted(model->geom_pos, model->ngeom * 3);
        const auto geom_size = std::views::counted(model->geom_size, model->ngeom * 3);
        bt::require(std::abs(geom_pos[3U * catch_id + 0U] - req->field.geom_x_m()) < 1e-5,
                    "catch_plane centered on spatial track X");
        bt::require(geom_size[3U * catch_id + 0U] >= req->field.radius_x_m + 10.0 - 1e-6 &&
                        geom_size[3U * catch_id + 1U] >=
                            spatial_cfg.corridor_half_width_m + 10.0 - 1e-6,
                    "catch_plane covers full track X and Y corridor");
    }

    // Invalid steer range_deg fails closed.
    {
        auto req = std::make_unique<mdl::BuildRequest>();
        req->mode = "ride";
        req->spatial = make_spatial_config(sp::SpatialTireBackend::compliant_3d, 0.0);
        bt::invalid([&] { (void)mdl::generate_mjcf(*req); });
    }
}

void test_heightfield_3d_transpose_and_bounds() {
    // Asymmetric Grid3d (nx = 17, ny = 9) with z = 0.2 * x - 0.1 * y.
    const trn::Grid3dSpec gs{
        .nx = 17,
        .ny = 9,
        .x0_m = 0.,
        .y0_m = -0.8,
        .dx_m = 0.5,
        .dy_m = 0.2,
    };
    std::vector<double> elevations(gs.nx * gs.ny);
    for (std::size_t ix = 0; ix < gs.nx; ++ix) {
        const double x = gs.x0_m + static_cast<double>(ix) * gs.dx_m;
        for (std::size_t iy = 0; iy < gs.ny; ++iy) {
            const double y = gs.y0_m + static_cast<double>(iy) * gs.dy_m;
            elevations[ix * gs.ny + iy] = 0.2 * x - 0.1 * y;
        }
    }
    const std::vector<rider::SurfaceLabel> surfaces(
        (gs.nx - 1U) * (gs.ny - 1U), rider::SurfaceLabel::hardpack);
    const trn::Grid3d grid(gs, std::move(elevations), surfaces);

    auto req = std::make_unique<mdl::BuildRequest>();
    req->mode = "ride";
    req->include_rider = false;
    req->physics = cfg::default_physics();
    req->physics->drive_mode = "coast";
    req->spatial = make_spatial_config(sp::SpatialTireBackend::compliant_3d);

    const double ground_z_m = bike::geometry::compute_ground_z(req->bike) / 1000.;
    const trn::HeightField3dBuild hf = trn::build_heightfield_3d(grid, ground_z_m);
    bt::require(hf.spec.nrow == 9 && hf.spec.ncol == 17,
                "HeightFieldSpec sets nrow=ny and ncol=nx");
    bt::require(std::abs(hf.spec.radius_x_m - 4.0) < 1e-12 &&
                    std::abs(hf.spec.radius_y_m - 0.8) < 1e-12,
                "HeightFieldSpec sets half-extents from grid bounds");
    bt::require(hf.catch_plane_z_m < grid.bounds().z_min_m,
                "catch_plane_z_m is strictly below grid bounds.z_min_m");

    req->field = hf.spec;
    const auto model = mdl::compile_mjcf(mdl::generate_mjcf(*req));
    trn::populate_heightfield_3d(*model, grid, hf);

    const int hfield_id = require_id(*model, mjOBJ_HFIELD, "road");
    const auto hfield_adr = std::views::counted(model->hfield_adr, model->nhfield);
    const auto hfield_data = std::views::counted(model->hfield_data, model->nhfielddata);
    const auto offset = static_cast<std::size_t>(hfield_adr[static_cast<std::size_t>(hfield_id)]);
    const auto road = hfield_data.subspan(offset, gs.nx * gs.ny);

    for (std::size_t iy = 0; iy < gs.ny; ++iy) {
        for (std::size_t ix = 0; ix < gs.nx; ++ix) {
            const auto expected = static_cast<float>(
                (grid.node_z(ix, iy) + hf.spec.datum_z_m) / hf.spec.elevation_m);
            const float actual = road[iy * gs.nx + ix];
            bt::require(std::bit_cast<std::uint32_t>(actual) ==
                            std::bit_cast<std::uint32_t>(expected),
                        "hfield_data[iy * ncol + ix] matches Grid3d[ix * ny + iy] bitwise");
        }
    }

    // Verify asymmetry is preserved in MuJoCo storage (high-x/low-y vs low-x/high-y).
    bt::require(road[0U * gs.nx + (gs.nx - 1U)] > road[(gs.ny - 1U) * gs.nx + 0U],
                "transposed hfield preserves asymmetric slope z = 0.2*x - 0.1*y");

    // Also exercise Terrain3dQueryView + ExtrudedTrack3d sampling path.
    const trn::TrackSpec flat_track = trn::flat(10.0);
    const trn::ExtrudedTrack3d ext(flat_track, 1.0);
    const trn::Terrain3dSource src(ext);
    const trn::Grid3dSpec ext_gs =
        trn::make_grid3d_spec(src.view().bounds(), trn::DEFAULT_RESOLUTION_M, 0.25);
    const trn::HeightField3dBuild ext_hf =
        trn::build_heightfield_3d(src.view(), ext_gs, ground_z_m);
    bt::require(std::cmp_equal(ext_hf.spec.nrow, ext_gs.ny) &&
                    std::cmp_equal(ext_hf.spec.ncol, ext_gs.nx) &&
                    ext_hf.catch_plane_z_m < ext_hf.bounds.z_min_m,
                "Terrain3dQueryView sampling builds valid HeightField3dBuild");

    // Fail-closed checks: mismatched grid dimensions and catch_plane >= z_min.
    trn::HeightField3dBuild bad_catch = hf;
    bad_catch.catch_plane_z_m = grid.bounds().z_min_m;
    bt::invalid([&] { trn::populate_heightfield_3d(*model, grid, bad_catch); });

    trn::HeightField3dBuild bad_dims = hf;
    bad_dims.spec.nrow += 1;
    bt::invalid([&] { trn::populate_heightfield_3d(*model, grid, bad_dims); });
}

} // namespace

int main() {
    return bt::run([] {
        test_planar_zero_byte_shield();
        test_spatial_topology_and_compilation();
        test_heightfield_3d_transpose_and_bounds();
    });
}
