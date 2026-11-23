// T48 owned render-replica contracts. Written for later execution; no test or
// engine command was run while implementing this migration wave.
#include "diag.hpp"
#include "support/fixture_test.hpp"
#include "viewer/render_owner.hpp"
#include "viewer/session.hpp"
#include "initialization/capture.hpp"
#include "model/builder.hpp"
#include "model_access.hpp"
#include "engine_call.hpp"
#include <cmath>
#include <limits>
#include <memory>
#include <set>
#include <thread>

namespace {
constexpr std::string_view scene = R"XML(<mujoco>
  <compiler autolimits="true"/>
  <option timestep="0.00125"/>
  <worldbody>
    <body name="frame" pos="0 0 0.5">
      <joint name="root_x" type="slide" axis="1 0 0" frictionloss="1"/>
      <geom name="frame_geom" type="sphere" size="0.1" mass="1"/>
      <site name="frame_site" pos="0.05 0 0"/>
    </body>
    <site name="ground_site" pos="0 0 0"/>
    <geom name="marker_test" type="sphere" size="0.01" pos="2 0 0"/>
  </worldbody>
  <tendon><fixed name="spring" stiffness="10" damping="2"
      springlength="0.1" range="-1 1" limited="true">
    <joint joint="root_x" coef="1"/>
  </fixed></tendon>
</mujoco>)XML";

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void replica_contracts() {
    auto model = bike::model::compile_mjcf(scene);
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(engine::make_data(model.get()), mj_deleteData);
    const auto image = bike::initialization::capture_model_image(*model);
    runtime::PresentationState presentation;
    presentation.position_m = 0.; presentation.z_m = .5;
    auto frame = bike::viewer::capture_frame(*model, *data, 0, 0, image.sha256, presentation);
    const auto captured = frame.state.integration_state;
    const auto fields = frame.presentation.model_fields;
    bt::require(fields[0].size() == 1 && fields[1].size() == 6 && fields[2].size() == 1 &&
        fields[3].size() == 1 && fields[4].size() == 2 && fields[5].size() == 2,
        "render snapshot owns every mutable presentation array at its engine extent");

    bike::viewer::RenderOwner renderer(image);
    bt::require(renderer.has_markers(), "livery discovers compiled marker geoms by their source prefix");
    renderer.apply(frame);
    bt::require(renderer.integration_state() == captured && renderer.model_fields() == fields,
        "render replica accepts an owned integration state and all six model fields");
    model_access::mutable_buffer(model->dof_frictionloss, model->nv)[0] = 17.;
    model_access::mutable_buffer(data->qpos, model->nq)[0] = .75;
    data->time = 2.;
    bt::require(frame.state.integration_state == captured && frame.presentation.model_fields == fields &&
        renderer.integration_state() == captured && renderer.model_fields() == fields,
        "later owner mutations cannot change a captured frame or independent renderer");
    data.reset(); model.reset();
    renderer.apply(frame);
    bt::require(renderer.model_sha256() == image.sha256, "renderer owns its model after simulation destruction");

    auto changed = frame;
    changed.state.step = 1; changed.state.time_s = .00125;
    changed.state.integration_state[0] = changed.state.time_s;
    changed.presentation.model_fields[0][0] = 3.;
    changed.presentation.model_fields[1][0] = .125;
    changed.presentation.model_fields[2][0] = 21.;
    changed.presentation.model_fields[3][0] = 4.;
    changed.presentation.model_fields[4] = {.2, .3};
    changed.presentation.model_fields[5] = {-.5, .5};
    renderer.apply(changed);
    bt::require(renderer.model_fields() == changed.presentation.model_fields,
        "replica updates friction, sites, tendon spring, damping, length and range");
    bt::invalid([&] { renderer.apply(frame); });
    auto wrong = changed; wrong.model_sha256[0] = wrong.model_sha256[0] == '0' ? '1' : '0';
    bt::invalid([&] { renderer.apply(wrong); });
    wrong = changed; wrong.presentation.model_fields[1].pop_back();
    bt::invalid([&] { renderer.apply(wrong); });
    wrong = changed; wrong.state.integration_state.pop_back();
    bt::invalid([&] { renderer.apply(wrong); });
    wrong = changed; wrong.state.time_s = .5;
    bt::invalid([&] { renderer.apply(wrong); });
    wrong = changed; wrong.presentation.z_m = std::numeric_limits<double>::quiet_NaN();
    bt::invalid([&] { renderer.apply(wrong); });
    bt::require(renderer.integration_state() == changed.state.integration_state &&
        renderer.model_fields() == changed.presentation.model_fields,
        "validation errors leave the last accepted render replica intact");

    frame.state.generation = 1;
    renderer.apply(frame);
    bt::require(renderer.integration_state() == captured, "new generation permits reset to step zero");

    bike::viewer::Decorations decorations;
    decorations.ghost_qpos = std::vector<double>(static_cast<std::size_t>(frame.state.integration_state.size() > 1 ? 1 : 0), .25);
    decorations.arrows = {{.from = {0., 0., 0.}, .to = {0., 0., .5}, .rgba = {1.f, .4f, .1f, 1.f}}};
    decorations.surface_bands = {{.from = {0., .22, .012}, .to = {.2, .22, .012}, .radius_m = .018, .rgba = {.24f, .26f, .30f, .92f}}};
    decorations.overlay = "GPU vs CPU  max|Δq| 0.000e+00 (root_x)  run max 0.000e+00";
    auto bad_dec = decorations;
    renderer.set_decorations(std::move(decorations));
    bad_dec.ghost_qpos = std::vector<double>{0., 1.};
    bt::invalid([&] { renderer.set_decorations(bad_dec); });
    bad_dec.ghost_qpos = std::vector<double>{std::numeric_limits<double>::quiet_NaN()};
    bt::invalid([&] { renderer.set_decorations(bad_dec); });
    bad_dec.ghost_qpos = std::vector<double>{.25};
    bad_dec.arrows.front().to[2] = std::numeric_limits<double>::infinity();
    bt::invalid([&] { renderer.set_decorations(bad_dec); });
    bad_dec.arrows.front().to[2] = .5;
    bad_dec.surface_bands.front().from[0] = std::numeric_limits<double>::quiet_NaN();
    bt::invalid([&] { renderer.set_decorations(bad_dec); });
    bad_dec.surface_bands.front().from[0] = 0.;
    bad_dec.surface_bands.front().radius_m = 0.;
    bt::invalid([&] { renderer.set_decorations(bad_dec); });
    bad_dec.surface_bands.front().radius_m = std::numeric_limits<double>::infinity();
    bt::invalid([&] { renderer.set_decorations(bad_dec); });
    bad_dec.surface_bands.front().radius_m = .018;
    bad_dec.surface_bands.front().rgba[1] = std::numeric_limits<float>::quiet_NaN();
    bt::invalid([&] { renderer.set_decorations(bad_dec); });
    renderer.set_decorations({});

    // Stage D: surface_bands_near builds distinct RGBA bands across macro and
    // Stage C micro-surfaces within the sliding window [x - back, x + ahead].
    {
        runtime::TrackSurfaceRibbon ribbon;
        ribbon.road_x_m = {0., 40.};
        ribbon.road_z_m = {0., 4.};
        ribbon.labels = rider::SurfaceLabels(
            rider::SurfaceLabel::asphalt,
            {{.start_m = 4., .end_m = 6., .label = rider::SurfaceLabel::hardpack},
             {.start_m = 6., .end_m = 8., .label = rider::SurfaceLabel::wet},
             {.start_m = 8., .end_m = 10., .label = rider::SurfaceLabel::loose},
             {.start_m = 10., .end_m = 12., .label = rider::SurfaceLabel::scree}});
        ribbon.micro_spans = {
            {.start_m = 12.2, .end_m = 12.6, .surface = "dry_wood"},
            {.start_m = 13.0, .end_m = 13.4, .surface = "wet_wood"},
            {.start_m = 14.0, .end_m = 14.4, .surface = "polished_rock"},
            {.start_m = 15.0, .end_m = 15.4, .surface = "wet_rock"},
            {.start_m = 16.0, .end_m = 16.4, .surface = "mud_lens"}};
        const auto bands = bike::viewer::surface_bands_near(ribbon, 10., 8., 18.);
        bt::require(!bands.empty(), "surface_bands_near generates bands inside the window");
        std::set<std::array<float, 4>> colors;
        for (const auto &band : bands) {
            bt::require(band.from[0] >= 2. - 1e-9 && band.to[0] <= 28. + 1e-9,
                        "bands stay inside [center - back, center + ahead]");
            bt::require(std::isfinite(band.radius_m) && band.radius_m > 0.,
                        "band radius is finite and positive");
            colors.insert(band.rgba);
        }
        bt::require(colors.size() == 10,
                    "all 5 macro surfaces and 5 Stage C micro-surfaces have distinct RGBA colors");
        bike::viewer::Decorations ribbon_dec;
        ribbon_dec.surface_bands = bands;
        renderer.set_decorations(std::move(ribbon_dec));
        bt::require(renderer.model_sha256() == image.sha256,
                    "surface bands never alter compiled model identity");
        renderer.set_decorations({});
    }

    renderer.set_markers(true); renderer.set_markers(false);
    renderer.set_camera(bike::viewer::CameraMode::chase); renderer.reset_camera();
    renderer.draw(0, 0); // minimized/no-context path must remain display independent
    bool wrong_thread_rejected = false;
    std::thread other([&] {
        try { (void)renderer.model_fields(); }
        catch (const std::runtime_error&) { wrong_thread_rejected = true; }
    });
    other.join();
    bt::require(wrong_thread_rejected, "render access is confined to its owning event thread");
    renderer.close(); renderer.close();
    bt::require(renderer.closed(), "render cleanup is idempotent without graphics initialization");
    auto invalid_image = image; invalid_image.sha256 = std::string(64, '0');
    bt::invalid([&] { bike::viewer::RenderOwner const rejected(invalid_image); });
}
NATIVE_DIAG_POP
}
int main() { return bt::run(replica_contracts); }
