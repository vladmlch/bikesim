#include "render_owner.hpp"
#include "render_fields.hpp"
#include "../engine_call.hpp"
#include "../io/hash.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <span>
#include <stdexcept>
#include <thread>

namespace bike::viewer {
namespace {
struct ModelDelete { void operator()(mjModel* value) const noexcept { if (value) mj_deleteModel(value); } };
struct DataDelete { void operator()(mjData* value) const noexcept { if (value) mj_deleteData(value); } };
constexpr std::array<std::string_view, 6> field_names{"dof_frictionloss", "site_pos", "tendon_stiffness",
    "tendon_damping", "tendon_lengthspring", "tendon_range"};
bool finite(std::span<const double> values) {
    return std::ranges::all_of(values, [](double v) { return std::isfinite(v); });
}
std::array<double, 4> normalized(std::array<double, 4> value) {
    const double norm = std::hypot(std::hypot(value[0], value[1]), std::hypot(value[2], value[3]));
    if (!std::isfinite(norm) || norm <= 1e-15) throw std::invalid_argument("render frame: invalid spatial quaternion");
    for (double& v : value) v /= norm;
    return value;
}
std::array<double, 4> blend_quaternion(std::array<double, 4> from, std::array<double, 4> to) {
    from = normalized(from); to = normalized(to);
    double dot = 0.; for (std::size_t i = 0; i < from.size(); ++i) dot += from[i] * to[i];
    if (dot < 0.) { for (double& v : to) v = -v; dot = -dot; }
    dot = std::clamp(dot, 0., 1.);
    constexpr double alpha = .2;
    double a = 1. - alpha, b = alpha;
    if (dot < .9995) {
        const double theta = std::acos(dot), denominator = std::sin(theta);
        a = std::sin((1. - alpha) * theta) / denominator;
        b = std::sin(alpha * theta) / denominator;
    }
    for (std::size_t i = 0; i < from.size(); ++i) from[i] = a * from[i] + b * to[i];
    return normalized(from);
}
}
struct RenderOwner::Impl {
    std::unique_ptr<mjModel, ModelDelete> model;
    std::unique_ptr<mjData, DataDelete> data;
    std::thread::id thread = std::this_thread::get_id();
    std::string digest, overlay;
    std::optional<runtime::RenderFrame> frame;
    Camera tracking;
    mjvCamera camera{};
    mjvOption option{};
    mjvScene scene{};
    mjrContext context{};
    std::vector<std::pair<int, std::array<double, 3>>> marker_sizes;
    bool graphics = false, poisoned = false, is_closed = false;
    int free_root_qpos = -1;
    void require_live() const {
        if (thread != std::this_thread::get_id()) throw std::runtime_error("render owner must stay on its event thread");
        if (is_closed) throw std::runtime_error("render owner is closed");
        if (poisoned) throw engine::EngineFailure("render owner is poisoned after engine failure");
    }
    std::array<std::span<mjtNum>, 6> fields() {
        return detail::mutable_model_fields(*model);
    }
};
RenderOwner::RenderOwner(const runtime::ModelImage& image) : impl_(std::make_unique<Impl>()) {
    if (image.mjb.empty() || image.sha256 != io::sha256(image.mjb))
        throw std::invalid_argument("render model: missing image or SHA-256 mismatch");
    auto& p = *impl_;
    p.digest = image.sha256;
    p.model.reset(engine::load_model_image(image.mjb));
    p.data.reset(engine::make_data(p.model.get()));
    const int root = mj_name2id(p.model.get(), mjOBJ_JOINT, "root_free");
    if (root >= 0) {
        if (p.model->jnt_type[root] != mjJNT_FREE) throw std::invalid_argument("render root_free is not a free joint");
        p.free_root_qpos = p.model->jnt_qposadr[root];
    }
    mjv_defaultCamera(&p.camera);
    mjv_defaultOption(&p.option);
    mjv_defaultScene(&p.scene);
    mjr_defaultContext(&p.context);
    p.camera.type = mjCAMERA_FREE;
    if (p.model->ngeom > std::numeric_limits<int>::max()) throw std::invalid_argument("render geom IDs exceed the MuJoCo int range");
    for (int id = 0; id < p.model->ngeom; ++id) {
        const char* name = mj_id2name(p.model.get(), mjOBJ_GEOM, id);
        if (name && std::string_view(name).starts_with("marker_")) {
            const auto offset = 3U * static_cast<std::size_t>(id);
            const auto sizes = model_access::readonly_buffer(p.model->geom_size, detail::render_extent(p.model->ngeom, 3));
            p.marker_sizes.push_back({id, {sizes[offset], sizes[offset + 1], sizes[offset + 2]}});
        }
    }
    set_markers(false);
}
RenderOwner::~RenderOwner() { close(); }
void RenderOwner::apply(const runtime::RenderFrame& frame) {
    auto& p = *impl_;
    p.require_live();
    const auto& state = frame.state;
    if (frame.model_sha256 != p.digest) throw std::invalid_argument("render frame: wrong compiled model identity");
    if (state.generation < 0 || state.step < 0 || (p.frame &&
        (state.generation < p.frame->state.generation ||
        (state.generation == p.frame->state.generation && state.step < p.frame->state.step))))
        throw std::invalid_argument("render frame moved backwards without a generation change");
    const auto width = engine::state_size(p.model.get(), mjSTATE_INTEGRATION);
    if (state.integration_state.size() != static_cast<std::size_t>(width) || !finite(state.integration_state))
        throw std::invalid_argument("render frame: incompatible or nonfinite integration state");
    if (state.integration_state.empty() || !std::isfinite(state.time_s) || state.integration_state.front() != state.time_s)
        throw std::invalid_argument("render frame time does not match integration state");
    if (!std::isfinite(frame.presentation.position_m) || !std::isfinite(frame.presentation.z_m))
        throw std::invalid_argument("render frame: nonfinite camera endpoint");
    if (const auto& pose = frame.presentation.spatial) {
        if (pose->generation != static_cast<std::uint64_t>(state.generation) ||
            !finite(pose->position_world_m) || !finite(pose->quat_wxyz) || !finite(pose->lateral_slip_mps) ||
            !std::isfinite(pose->steer_angle_rad) || !std::isfinite(pose->steer_torque_nm) ||
            (pose->heading_rad && !std::isfinite(*pose->heading_rad)) ||
            (pose->roll_rad && !std::isfinite(*pose->roll_rad)) ||
            pose->attitude_valid != pose->heading_rad.has_value() ||
            pose->attitude_valid != pose->roll_rad.has_value())
            throw std::invalid_argument("render frame: invalid spatial presentation");
        (void)normalized(pose->quat_wxyz);
    }
    auto fields = p.fields();
    for (std::size_t i = 0; i < fields.size(); ++i) {
        if (fields[i].size() != frame.presentation.model_fields[i].size() || !finite(frame.presentation.model_fields[i]))
            throw std::invalid_argument("render frame: incompatible model field " + std::string(field_names[i]));
    }
    // Complete the throwing allocations before modifying the render replica.
    auto staged = frame;
    const bool generation_changed = !p.frame || p.frame->state.generation != state.generation;
    if (auto& pose = staged.presentation.spatial) {
        pose->quat_wxyz = normalized(pose->quat_wxyz);
        if (!generation_changed && p.frame->presentation.spatial) {
            const auto& previous = *p.frame->presentation.spatial;
            for (std::size_t axis = 0; axis < 3; ++axis)
                pose->position_world_m[axis] = .8 * previous.position_world_m[axis] +
                    .2 * pose->position_world_m[axis];
            pose->quat_wxyz = blend_quaternion(previous.quat_wxyz, pose->quat_wxyz);
        }
        if(!finite(pose->position_world_m))throw std::invalid_argument("render frame: interpolated spatial pose is nonfinite");
    }
    for (std::size_t i = 0; i < fields.size(); ++i)
        std::copy(staged.presentation.model_fields[i].begin(), staged.presentation.model_fields[i].end(), fields[i].begin());
    struct Refresh { mjModel* m; mjData* d; const double* state; int root;
        const runtime::SpatialPresentation* pose; } refresh{.m=p.model.get(), .d=p.data.get(),
        .state=staged.state.integration_state.data(), .root=p.free_root_qpos,
        .pose=staged.presentation.spatial ? &*staged.presentation.spatial : nullptr};
    engine::ErrorBuffer error;
    // Deliberately kinematic only. No mj_forward, mj_step, sensor acquisition,
    // force writers or policy evaluation can be reached from this operation.
    const bool ok = engine::invoke([](void* opaque) noexcept {
        auto const* r = static_cast<Refresh*>(opaque);
        mj_setState(r->m, r->d, r->state, mjSTATE_INTEGRATION);
        if (r->root >= 0 && r->pose) {
            for (std::size_t i = 0; i < 3; ++i) r->d->qpos[r->root + static_cast<int>(i)] = r->pose->position_world_m[i];
            for (std::size_t i = 0; i < 4; ++i) r->d->qpos[r->root + 3 + static_cast<int>(i)] = r->pose->quat_wxyz[i];
        }
        mj_kinematics(r->m, r->d);
        mj_comPos(r->m, r->d);
        mj_camlight(r->m, r->d);
        mj_tendon(r->m, r->d);
    }, &refresh, error);
    if (!ok) { p.poisoned = true; engine::throw_failure(error); }
    if (generation_changed) p.tracking.reset_preset();
    if (const auto& pose = staged.presentation.spatial)
        p.tracking.update_spatial(p.camera, pose->position_world_m, pose->attitude_valid ? pose->heading_rad : std::nullopt);
    else p.tracking.update(p.camera, staged.presentation.position_m, staged.presentation.z_m);
    p.frame = std::move(staged);
}
void RenderOwner::draw(int width, int height) {
    auto const& p = *impl_; p.require_live();
    if (width < 0 || height < 0) throw std::invalid_argument("negative framebuffer dimension");
    if (width == 0 || height == 0) return; // minimized window, no division by zero
#if defined(BIKE_BUILD_GUI) && BIKE_BUILD_GUI
    if (!p.frame) throw std::runtime_error("render owner has no frame");
    struct Draw { Impl* p; int width, height; } draw{&p, width, height};
    engine::ErrorBuffer error;
    const bool ok = engine::invoke([](void* opaque) noexcept {
        auto* draw_state = static_cast<Draw*>(opaque);
        auto* owner = draw_state->p;
        if (!owner->graphics) {
            // Mark first: cleanup must also free a partially initialized scene.
            owner->graphics = true;
            mjv_makeScene(owner->model.get(), &owner->scene, 20000);
            mjr_makeContext(owner->model.get(), &owner->context, mjFONTSCALE_150);
        }
        mjv_updateScene(owner->model.get(), owner->data.get(), &owner->option, nullptr,
            &owner->camera, mjCAT_ALL, &owner->scene);
        const mjrRect viewport{0, 0, draw_state->width, draw_state->height};
        mjr_render(viewport, &owner->scene, &owner->context);
        if (!owner->overlay.empty()) mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport,
            owner->overlay.c_str(), nullptr, &owner->context);
    }, &draw, error);
    if (!ok) { p.poisoned = true; engine::throw_failure(error); }
#else
    throw std::runtime_error("viewer unavailable: configure BIKE_BUILD_GUI=ON");
#endif
}
void RenderOwner::set_camera(CameraMode mode) { impl_->require_live(); impl_->tracking.set_mode(mode); }
void RenderOwner::cycle_camera() { impl_->require_live(); impl_->tracking.cycle_mode(); }
void RenderOwner::reset_camera() { impl_->require_live(); impl_->tracking.reset_preset(); }
void RenderOwner::move_camera(int action, double dx, double dy) {
    auto& p = *impl_; p.require_live();
    if (!std::isfinite(dx) || !std::isfinite(dy) || action < mjMOUSE_ROTATE_V || action > mjMOUSE_ZOOM)
        throw std::invalid_argument("invalid camera movement");
    if (p.graphics) mjv_moveCamera(p.model.get(), action, dx, dy, &p.camera);
}
void RenderOwner::set_markers(bool visible) {
    auto& p = *impl_; p.require_live();
    auto const sizes = model_access::mutable_buffer(p.model->geom_size, detail::render_extent(p.model->ngeom, 3));
    auto const colors = model_access::mutable_buffer(p.model->geom_rgba, detail::render_extent(p.model->ngeom, 4));
    for (const auto& [id, dimensions] : p.marker_sizes) {
        const auto index = static_cast<std::size_t>(id);
        colors[4 * index + 3] = visible ? 1.f : 0.f;
        for (std::size_t j = 0; j < 3; ++j) sizes[3 * index + j] = visible ? dimensions[j] : .0001;
    }
}
void RenderOwner::set_overlay(std::string text) { impl_->require_live(); impl_->overlay = std::move(text); }
void RenderOwner::close() noexcept {
    auto& p = *impl_;
    if (p.is_closed) return;
    if (p.thread != std::this_thread::get_id()) std::terminate();
    if (p.graphics) { mjr_freeContext(&p.context); mjv_freeScene(&p.scene); p.graphics = false; }
    p.data.reset(); p.model.reset(); p.frame.reset(); p.is_closed = true;
}
bool RenderOwner::closed() const noexcept { return impl_->is_closed; }
bool RenderOwner::has_markers() const noexcept { return !impl_->marker_sizes.empty(); }
std::string_view RenderOwner::model_sha256() const noexcept { return impl_->digest; }
std::vector<double> RenderOwner::integration_state() const {
    impl_->require_live();
    if (!impl_->frame) return {};
    return impl_->frame->state.integration_state;
}
std::array<std::vector<double>, 6> RenderOwner::model_fields() const {
    impl_->require_live();
    std::array<std::vector<double>, 6> out;
    const auto fields = impl_->fields();
    for (std::size_t i = 0; i < fields.size(); ++i) out[i].assign(fields[i].begin(), fields[i].end());
    return out;
}
} // namespace bike::viewer
