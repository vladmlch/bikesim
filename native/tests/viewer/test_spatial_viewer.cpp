#include "diag.hpp"
#include "engine_call.hpp"
#include "initialization/capture.hpp"
#include "model/builder.hpp"
#include "runtime/runtime.hpp"
#include "spatial/frames.hpp"
#include "support/fixture_test.hpp"
#include "viewer/hud.hpp"
#include "viewer/render_owner.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <string>
#include <vector>

namespace {

namespace view = bike::viewer;

constexpr double kDegToRad = std::numbers::pi / 180.0;
constexpr double kRadToDeg = 180.0 / std::numbers::pi;

constexpr std::string_view kSpatialScene = R"XML(<mujoco>
  <compiler autolimits="true"/>
  <option timestep="0.00125"/>
  <worldbody>
    <body name="frame" pos="0 0 0">
      <freejoint name="root_free"/>
      <geom name="frame_geom" type="sphere" size="0.1" mass="1"/>
    </body>
  </worldbody>
</mujoco>)XML";

constexpr std::string_view kPlanarScene = R"XML(<mujoco>
  <compiler autolimits="true"/>
  <option timestep="0.00125"/>
  <worldbody>
    <body name="frame" pos="0 0 0.5">
      <joint name="root_x" type="slide" axis="1 0 0"/>
      <geom name="frame_geom" type="sphere" size="0.1" mass="1"/>
    </body>
  </worldbody>
</mujoco>)XML";

void test_camera_spatial_tracking_and_unwrapping() {
    // 1. Planar shield stays untouched.
    mjvCamera planar_cam{};
    view::Camera planar_tracking(0.08);
    planar_tracking.update(planar_cam, 4.0, 1.0);
    bt::require(planar_cam.azimuth == 90.0 && planar_cam.elevation == -5.0 &&
                    planar_cam.distance == 2.5 &&
                    planar_cam.lookat[0] == 4.15 && planar_cam.lookat[1] == 0.0 &&
                    planar_cam.lookat[2] == 1.25,
                "planar Camera::update preset remains untouched");

    // 2. Spatial 3D XYZ tracking + heading-aligned offset + Z smoothing.
    mjvCamera cam{};
    view::Camera tracking(0.25);
    tracking.set_mode(view::CameraMode::chase);
    const double yaw0 = 3.10;
    tracking.update_spatial(cam, 10.0, -3.5, 1.2, yaw0);
    bt::require(std::abs(cam.lookat[0] - (10.0 + 0.20 * std::cos(yaw0))) < 1e-12 &&
                    std::abs(cam.lookat[1] - (-3.5 + 0.20 * std::sin(yaw0))) < 1e-12 &&
                    std::abs(cam.lookat[2] - 1.50) < 1e-12,
                "spatial camera tracks 3D XYZ position");
    bt::require(std::abs(cam.azimuth - (135.0 + yaw0 * kRadToDeg)) < 1e-12 &&
                    cam.elevation == -15.0 && cam.distance == 3.2,
                "spatial chase camera initializes azimuth/elevation/distance");

    // 3. Cross +pi -> -pi continuously without a 360 deg snap.
    double prev_az = cam.azimuth;
    double prev_unwrapped = tracking.unwrapped_yaw_rad();
    for (int step = 1; step <= 80; ++step) {
        const double unwrapped_true = yaw0 + 0.08 * static_cast<double>(step);
        const double wrapped = std::atan2(std::sin(unwrapped_true), std::cos(unwrapped_true));
        tracking.update_spatial(cam, 10.0 + 0.1 * static_cast<double>(step), -3.5 + 0.05 * static_cast<double>(step),
                                1.2 + 0.4, wrapped);
        const double delta_unwrapped = tracking.unwrapped_yaw_rad() - prev_unwrapped;
        const double delta_az = cam.azimuth - prev_az;
        bt::require(std::abs(delta_unwrapped - 0.08) < 1e-12,
                    "unwrapped heading increments continuously across +-pi");
        bt::require(delta_az > 0.0 && delta_az < 10.0,
                    "chase azimuth advances smoothly without 360 deg jump across +-pi");
        prev_unwrapped = tracking.unwrapped_yaw_rad();
        prev_az = cam.azimuth;
    }
    bt::require(tracking.unwrapped_yaw_rad() > 2.0 * std::numbers::pi,
                "unwrapped heading exceeds 2*pi after crossing +pi and full turn");
    bt::require(cam.lookat[2] > 1.50 && cam.lookat[2] < 1.90,
                "vertical lookat smooths smoothly toward new Z");

    // 4. Reverse turn across -pi using (hx, hy) overload.
    tracking.reset_preset();
    const double neg_yaw0 = -3.10;
    tracking.update_spatial(cam, 0.0, 0.0, 0.0, {std::cos(neg_yaw0), std::sin(neg_yaw0)});
    const double neg_yaw1 = 3.08;
    tracking.update_spatial(cam, 1.0, 2.0, 0.5, {std::cos(neg_yaw1), std::sin(neg_yaw1)});
    const double expected_delta =
        std::atan2(std::sin(neg_yaw1 - neg_yaw0), std::cos(neg_yaw1 - neg_yaw0));
    bt::require(expected_delta < 0.0 &&
                    std::abs(tracking.unwrapped_yaw_rad() - (neg_yaw0 + expected_delta)) < 1e-12,
                "vector heading overload unwraps negative turns across -pi");

    // 5. Mode cycling keeps unwrapped yaw continuous while snapping chase/side preset.
    tracking.cycle_mode();
    tracking.update_spatial(cam, 1.0, 2.0, 0.5, {std::cos(neg_yaw1), std::sin(neg_yaw1)});
    bt::require(std::abs(tracking.unwrapped_yaw_rad() - (neg_yaw0 + expected_delta)) < 1e-12,
                "cycling camera mode preserves continuous unwrapped yaw");

    // 6. Smooth chase azimuth/elevation/distance convergence.
    view::Camera smooth_cam(0.5);
    smooth_cam.set_mode(view::CameraMode::chase);
    smooth_cam.update_spatial(cam, 0.0, 0.0, 0.0, 0.0);
    cam.elevation = -35.0;
    cam.distance = 6.0;
    smooth_cam.update_spatial(cam, 0.0, 0.0, 0.0, 0.4);
    bt::require(std::abs(cam.elevation - (-25.0)) < 1e-12 &&
                    std::abs(cam.distance - 4.6) < 1e-12,
                "update_spatial smooths chase elevation and distance");

    bt::invalid([&] { tracking.update_spatial(cam, std::numeric_limits<double>::infinity(), 0.0, 0.0, 0.0); });
    bt::invalid([&] {
        tracking.update_spatial(cam, 0.0, 0.0, 0.0,
                                std::array<double, 2>{std::numeric_limits<double>::quiet_NaN(), 1.0});
    });
}

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_render_owner_spatial_set_state_and_camera() {
    auto spatial_model = bike::model::compile_mjcf(kSpatialScene);
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(engine::make_data(spatial_model.get()), mj_deleteData);
    const auto image = bike::initialization::capture_model_image(*spatial_model);
    view::RenderOwner renderer(image);
    renderer.set_camera(view::CameraMode::chase);

    // Step across +pi -> -pi with nonzero roll, pitch, and 3D XYZ.
    for (int i = 0; i < 6; ++i) {
        const double yaw_true = 3.05 + 0.05 * static_cast<double>(i);
        const auto q = bike::spatial::quat_from_euler_zyx(0.12, -0.05, yaw_true);
        const double x = 5.0 + 0.5 * static_cast<double>(i);
        const double y = -2.0 + 0.3 * static_cast<double>(i);
        const double z = 0.8 + 0.05 * static_cast<double>(i);
        const auto qpos = model_access::mutable_buffer(data->qpos, spatial_model->nq);
        qpos[0] = x;
        qpos[1] = y;
        qpos[2] = z;
        qpos[3] = q[0];
        qpos[4] = q[1];
        qpos[5] = q[2];
        qpos[6] = q[3];
        data->time = 0.01 * static_cast<double>(i);

        runtime::PresentationState presentation;
        presentation.position_m = x;
        presentation.z_m = z;
        const auto frame = bike::viewer::capture_frame(
            *spatial_model, *data, 0, i, image.sha256, presentation);
        renderer.apply(frame);
        bt::require(renderer.integration_state() == frame.state.integration_state,
                    "RenderOwner::apply preserves unit quaternion qpos via mj_setState without SLERP distortion");
        const auto cam = renderer.camera();
        const auto unwrapped = renderer.spatial_unwrapped_yaw_rad();
        bt::require(unwrapped.has_value() && std::abs(unwrapped.value_or(0.0) - yaw_true) < 1e-9,
                    "RenderOwner::apply unwraps frame yaw across +-pi");
        bt::require(std::abs(cam.lookat[0] - (x + 0.20 * std::cos(yaw_true))) < 1e-6 &&
                        std::abs(cam.lookat[1] - (y + 0.20 * std::sin(yaw_true))) < 1e-6,
                    "RenderOwner::apply tracks 3D horizontal lookat");
    }

    // Planar RenderOwner stays on planar path.
    const auto planar_model = bike::model::compile_mjcf(kPlanarScene);
    const std::unique_ptr<mjData, decltype(&mj_deleteData)> pdata(engine::make_data(planar_model.get()), mj_deleteData);
    const auto pimage = bike::initialization::capture_model_image(*planar_model);
    view::RenderOwner prenderer(pimage);
    runtime::PresentationState ppresentation;
    ppresentation.position_m = 2.0;
    ppresentation.z_m = 0.5;
    const auto pframe = bike::viewer::capture_frame(
        *planar_model, *pdata, 0, 0, pimage.sha256, ppresentation);
    prenderer.apply(pframe);
    bt::require(!prenderer.spatial_unwrapped_yaw_rad().has_value() &&
                    prenderer.camera().azimuth == 90.0 &&
                    prenderer.camera().lookat[0] == 2.15,
                "planar RenderOwner keeps planar camera update untouched");
}
NATIVE_DIAG_POP

struct HudFrameSpec {
    bool spatial = false;
    std::optional<std::string> launch_state;
    double roll_deg = 0.0;
    double yaw_deg = 0.0;
    double steer_deg = 0.0;
};

runtime::RenderFrame make_hud_frame(const HudFrameSpec& spec) {
    runtime::RenderFrame frame;
    auto sample = std::make_shared<runtime::PhysicalSampleData>();
    sample->time_s = 1.25;
    sample->end_time_s = 1.25125;
    if (spec.spatial) {
        sample->qpos.assign(15U, 0.0);
        sample->qvel.assign(14U, 0.0); // nq > nv signals root_free spatial state
        const auto q = bike::spatial::quat_from_euler_zyx(
            spec.roll_deg * kDegToRad, 0.0, spec.yaw_deg * kDegToRad);
        sample->qpos[0] = 3.0;
        sample->qpos[1] = 1.5;
        sample->qpos[2] = 0.6;
        sample->qpos[3] = q[0];
        sample->qpos[4] = q[1];
        sample->qpos[5] = q[2];
        sample->qpos[6] = q[3];
        sample->qpos[10] = spec.steer_deg * kDegToRad; // steer_joint
        // World X velocity is 0 (riding along +Y at yaw=90 deg), while forward speed is 5.0 m/s.
        sample->qvel[0] = 0.0;
        sample->qvel[1] = 5.0;
        frame.presentation.speed_mps = 5.0;
        frame.presentation.position_m = 3.0;
        if (spec.launch_state == "active") {
            sample->forces.emplace_back("launch_support", std::vector<double>(14U, 0.0));
        }
    } else {
        sample->qpos.assign(14U, 0.0);
        sample->qvel.assign(14U, 0.0);
        sample->qvel[0] = 4.0;
        frame.presentation.speed_mps = 4.0;
        frame.presentation.position_m = 3.0;
    }

    runtime::WireObject energy{{"measured", true}};
    if (spec.launch_state == "active" || spec.launch_state == "released") {
        energy.emplace_back("launch_support_work_j", 1.5);
    }

    sample->channels = {
        {"tires", runtime::WireObject{{"front", runtime::WireObject{{"normal_load_n", 400.0}}},
                                      {"rear", runtime::WireObject{{"normal_load_n", 500.0}}}}},
        {"drive", runtime::WireObject{}},
        {"rider", runtime::WireObject{
            {"grip_left", runtime::WireObject{{"force_on_bike_n", runtime::WireArray{0.0, 25.0, 0.0}}}},
            {"grip_right", runtime::WireObject{{"force_on_bike_n", runtime::WireArray{0.0, -15.0, 0.0}}}}}},
        {"energy", std::move(energy)}};

    frame.state.time_s = sample->time_s;
    frame.state.latest_sample = std::move(sample);
    frame.state.view = {{"drive_mode", runtime::Wire("articulated_effort")}};
    frame.presentation.root_x_dofadr = 0;
    return frame;
}

std::string join_columns(const std::vector<view::HudColumn>& columns) {
    std::string out;
    for (const auto& column : columns) out += column.text;
    return out;
}

void test_hud_3d_formatting_and_planar_shield() {
    const view::InputState input;

    // 1. Planar shield: no roll/yaw/steer/lsup columns, planar 2D bar magnitude ignores Y.
    const auto planar_frame = make_hud_frame({
        .spatial = false,
        .launch_state = std::nullopt,
        .roll_deg = 0.0,
        .yaw_deg = 0.0,
        .steer_deg = 0.0,
    });
    const auto planar_cols = view::physical_columns(planar_frame, 1.0);
    const auto planar_text = join_columns(planar_cols);
    bt::require(planar_text.find("roll=") == std::string::npos &&
                    planar_text.find("yaw=") == std::string::npos &&
                    planar_text.find("str=") == std::string::npos &&
                    planar_text.find("lsup=") == std::string::npos,
                "planar HUD omits 3D columns");
    bt::require(planar_text.find("L→  0N R→  0N") != std::string::npos,
                "planar HUD keeps 2D (x,z) grip force magnitude untouched");
    const auto planar_overlay = view::overlay_text(planar_frame, input, 1.0, view::Mode::physical);
    bt::require(planar_overlay.find("Roll ") == std::string::npos,
                "planar overlay omits 3D line");

    // 2. Spatial HUD: roll, unwrapped yaw (>180 deg), steer, launch_support states, 3D speed & bar force.
    const auto active_frame = make_hud_frame({
        .spatial = true,
        .launch_state = "active",
        .roll_deg = 12.5,
        .yaw_deg = 90.0,
        .steer_deg = -7.2,
    });
    const view::SpatialHudView unwrapped_view{.unwrapped_yaw_rad = 195.0 * kDegToRad};
    const auto cols_active = view::physical_columns(active_frame, 1.0, unwrapped_view);
    const auto text_active = join_columns(cols_active);
    const auto legend_active = view::hud_legend(cols_active);
    bt::require(legend_active.find("roll") != std::string::npos &&
                    legend_active.find("yaw") != std::string::npos &&
                    legend_active.find("steer") != std::string::npos &&
                    legend_active.find("lsup") != std::string::npos,
                "3D HUD legend includes roll, yaw, steer, and lsup");
    bt::require(text_active.find("v=+18.00km/h") != std::string::npos,
                "3D HUD displays horizontal forward speed rather than raw world qvel[0]");
    bt::require(text_active.find("L→ 25N R→ 15N") != std::string::npos,
                "3D HUD includes lateral Y grip force in bar magnitude");
    bt::require(text_active.find("roll=+12.5deg") != std::string::npos &&
                    text_active.find("yaw=+195.0deg") != std::string::npos &&
                    text_active.find("str= -7.2deg") != std::string::npos &&
                    text_active.find("lsup=active") != std::string::npos,
                "3D HUD prints roll, unwrapped yaw, steer, and active launch_support");

    const auto released_frame = make_hud_frame({
        .spatial = true,
        .launch_state = "released",
        .roll_deg = -3.0,
        .yaw_deg = -45.0,
        .steer_deg = 4.5,
    });
    const auto text_released = join_columns(view::physical_columns(released_frame, 1.0));
    bt::require(text_released.find("lsup=released") != std::string::npos &&
                    text_released.find("yaw= -45.0deg") != std::string::npos,
                "3D HUD displays released launch_support and wrapped yaw fallback");

    const auto off_frame = make_hud_frame({
        .spatial = true,
        .launch_state = "off",
        .roll_deg = 0.0,
        .yaw_deg = 0.0,
        .steer_deg = 0.0,
    });
    const auto text_off = join_columns(view::physical_columns(off_frame, 1.0));
    bt::require(text_off.find("lsup=off") != std::string::npos,
                "3D HUD displays off launch_support for moving starts");

    const auto inspection_line = view::hud_line(
        active_frame, input, 1.0, view::Mode::inspection, unwrapped_view);
    bt::require(inspection_line.find("Roll:+12.5deg") != std::string::npos &&
                    inspection_line.find("Yaw:+195.0deg") != std::string::npos &&
                    inspection_line.find("Steer:-7.2deg") != std::string::npos &&
                    inspection_line.find("LSup:active") != std::string::npos,
                "inspection/ride hud_line formats 3D metrics");

    const auto overlay = view::overlay_text(
        active_frame, input, 1.0, view::Mode::physical, unwrapped_view);
    bt::require(overlay.find("Roll +12.5 deg") != std::string::npos &&
                    overlay.find("Yaw +195.0 deg") != std::string::npos &&
                    overlay.find("Steer -7.2 deg") != std::string::npos &&
                    overlay.find("LaunchSupport active") != std::string::npos,
                "overlay_text includes 3D metrics when spatial is active");
}

} // namespace

int main() {
    // cppcheck-suppress throwInEntryPoint // test failures propagate as exceptions into bt::run's FAIL catch
    return bt::run([] {
        test_camera_spatial_tracking_and_unwrapping();
        test_render_owner_spatial_set_state_and_camera();
        test_hud_3d_formatting_and_planar_shield();
    });
}
