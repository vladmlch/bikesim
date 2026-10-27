#include "render_owner.hpp"
#include "render_fields.hpp"
#include "../engine_call.hpp"
#include "../io/hash.hpp"
#include "../spatial/frames.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <limits>
#include <optional>
#include <span>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <utility>

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
    // Accepted-frame identity and the smoothed display pose are all the next
    // apply() needs. Keeping them instead of a whole RenderFrame copy means a
    // steady-state apply allocates nothing: the integration vector reuses its
    // reserved capacity and SpatialPresentation owns no heap storage.
    bool has_frame = false;
    int frame_generation = 0;
    std::int64_t frame_step = 0;
    std::vector<double> integration;
    std::optional<runtime::SpatialPresentation> display_pose;
    Camera tracking;
    mjvCamera camera{};
    mjvOption option{};
    mjvScene scene{};
    mjrContext context{};
    std::vector<std::pair<int, std::array<double, 3>>> marker_sizes;
    bool graphics = false, poisoned = false, is_closed = false;
    int free_root_qpos = -1;
    int rider_root_qpos = -1;
    std::size_t qpos_state_offset = 0;
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
    const auto joint_types = model_access::readonly_buffer(p.model->jnt_type, p.model->njnt, "render jnt_type");
    const auto joint_qpos = model_access::readonly_buffer(p.model->jnt_qposadr, p.model->njnt, "render jnt_qposadr");
    const int root = mj_name2id(p.model.get(), mjOBJ_JOINT, "root_free");
    if (root >= 0) {
        if (joint_types[static_cast<std::size_t>(root)] != mjJNT_FREE)
            throw std::invalid_argument("render root_free is not a free joint");
        p.free_root_qpos = joint_qpos[static_cast<std::size_t>(root)];
    }
    const int rider = mj_name2id(p.model.get(), mjOBJ_JOINT, "rider_root_free");
    if (rider >= 0) {
        if (joint_types[static_cast<std::size_t>(rider)] != mjJNT_FREE)
            throw std::invalid_argument("render rider_root_free is not a free joint");
        p.rider_root_qpos = joint_qpos[static_cast<std::size_t>(rider)];
    }
    // mjSTATE_INTEGRATION stores TIME before QPOS. Ask the engine for the
    // width of that prefix instead of assuming a generalized coordinate ID.
    p.qpos_state_offset=static_cast<std::size_t>(engine::state_size(p.model.get(),mjSTATE_TIME));
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
    if (state.generation < 0 || state.step < 0 || (p.has_frame &&
        (state.generation < p.frame_generation ||
        (state.generation == p.frame_generation && state.step < p.frame_step))))
        throw std::invalid_argument("render frame moved backwards without a generation change");
    const auto width = engine::state_size(p.model.get(), mjSTATE_INTEGRATION);
    if (state.integration_state.size() != static_cast<std::size_t>(width) || !finite(state.integration_state))
        throw std::invalid_argument("render frame: incompatible or nonfinite integration state");
    if (state.integration_state.empty() || !std::isfinite(state.time_s) || state.integration_state.front() != state.time_s)
        throw std::invalid_argument("render frame time does not match integration state");
    if (!std::isfinite(frame.presentation.position_m) || !std::isfinite(frame.presentation.z_m))
        throw std::invalid_argument("render frame: nonfinite camera endpoint");
    if (const auto& pose = frame.presentation.spatial) {
        if (std::cmp_not_equal(pose->generation, state.generation) ||
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
    // Complete every throwing step (and the only allocation, on the first
    // frame) before modifying the render replica. The caller's frame is never
    // copied: only the spatial pose is staged, by value.
    p.integration.reserve(state.integration_state.size());
    const bool generation_changed = !p.has_frame || p.frame_generation != state.generation;
    std::optional<runtime::SpatialPresentation> staged_pose = frame.presentation.spatial;
    if (auto& pose = staged_pose) {
        pose->quat_wxyz = normalized(pose->quat_wxyz);
        if (!generation_changed && p.display_pose) {
            const auto& previous = *p.display_pose;
            for (std::size_t axis = 0; axis < 3; ++axis)
                pose->position_world_m[axis] = .8 * previous.position_world_m[axis] +
                    .2 * pose->position_world_m[axis];
            pose->quat_wxyz = blend_quaternion(previous.quat_wxyz, pose->quat_wxyz);
        }
        if(!finite(pose->position_world_m))throw std::invalid_argument("render frame: interpolated spatial pose is nonfinite");
    }
    std::optional<std::array<double,7>> rider_pose;
    if(p.free_root_qpos>=0&&p.rider_root_qpos>=0&&staged_pose) {
        const auto& source=*frame.presentation.spatial;
        const auto& target=*staged_pose;
        const auto address=p.qpos_state_offset+static_cast<std::size_t>(p.rider_root_qpos);
        std::array<double,3> relative{};
        std::array<double,4> rider_quat{};
        for(std::size_t i=0;i<3;++i)relative[i]=state.integration_state.at(address+i)-source.position_world_m[i];
        for(std::size_t i=0;i<4;++i)rider_quat[i]=state.integration_state.at(address+3+i);
        const auto rotation=normalized(spatial::quat_multiply(target.quat_wxyz,
            spatial::quat_conjugate(normalized(source.quat_wxyz))));
        const auto offset=spatial::rotate_body_to_world(rotation,relative);
        const auto quaternion=normalized(spatial::quat_multiply(rotation,normalized(rider_quat)));
        rider_pose.emplace();
        for(std::size_t i=0;i<3;++i)(*rider_pose)[i]=target.position_world_m[i]+offset[i];
        for(std::size_t i=0;i<4;++i)(*rider_pose)[3+i]=quaternion[i];
        if(!finite(*rider_pose))throw std::invalid_argument("render frame: interpolated rider pose is nonfinite");
    }
    for (std::size_t i = 0; i < fields.size(); ++i)
        std::copy(frame.presentation.model_fields[i].begin(), frame.presentation.model_fields[i].end(), fields[i].begin());
    // The qpos view is formed here, where a bad extent can still throw; the
    // noexcept engine frame below only indexes a checked span.
    const auto qpos = model_access::mutable_buffer(p.data->qpos, p.model->nq, "render qpos");
    struct Refresh { mjModel* m; mjData* d; std::span<mjtNum> qpos; const double* state; int root;
        const runtime::SpatialPresentation* pose; int rider; const std::array<double,7>* rider_pose;
    } refresh{.m=p.model.get(), .d=p.data.get(), .qpos=qpos,
        .state=state.integration_state.data(), .root=p.free_root_qpos,
        .pose=staged_pose ? &*staged_pose : nullptr,
        .rider=p.rider_root_qpos,.rider_pose=rider_pose?&*rider_pose:nullptr};
    engine::ErrorBuffer error;
    // Deliberately kinematic only. No mj_forward, mj_step, sensor acquisition,
    // force writers or policy evaluation can be reached from this operation.
    const bool ok = engine::invoke([](void* opaque) noexcept {
        auto const* r = static_cast<Refresh*>(opaque);
        mj_setState(r->m, r->d, r->state, mjSTATE_INTEGRATION);
        if (r->root >= 0 && r->pose) {
            const auto root = static_cast<std::size_t>(r->root);
            for (std::size_t i = 0; i < 3; ++i) r->qpos[root + i] = r->pose->position_world_m[i];
            for (std::size_t i = 0; i < 4; ++i) r->qpos[root + 3 + i] = r->pose->quat_wxyz[i];
        }
        // The articulated pelvis is an independent world free joint. Apply
        // the bike's display transform to it too, preserving the accepted
        // bike-relative rider pose and all local articulated coordinates.
        if(r->rider>=0&&r->rider_pose) {
            const auto pelvis=static_cast<std::size_t>(r->rider);
            for(std::size_t i=0;i<7;++i)r->qpos[pelvis+i]=(*r->rider_pose)[i];
        }
        mj_kinematics(r->m, r->d);
        mj_comPos(r->m, r->d);
        mj_camlight(r->m, r->d);
        mj_tendon(r->m, r->d);
    }, &refresh, error);
    if (!ok) { p.poisoned = true; engine::throw_failure(error); }
    if (generation_changed) p.tracking.reset_preset();
    if (staged_pose)
        p.tracking.update_spatial(p.camera, staged_pose->position_world_m,
            staged_pose->attitude_valid ? staged_pose->heading_rad : std::nullopt);
    else p.tracking.update(p.camera, frame.presentation.position_m, frame.presentation.z_m);
    // Capacity was reserved above, so these cannot throw or allocate.
    p.has_frame = true;
    p.frame_generation = state.generation;
    p.frame_step = state.step;
    p.integration.assign(state.integration_state.begin(), state.integration_state.end());
    p.display_pose = staged_pose;
}
void RenderOwner::draw(int width, int height) {
    impl_->require_live();
    if (width < 0 || height < 0) throw std::invalid_argument("negative framebuffer dimension");
    if (width == 0 || height == 0) return; // minimized window, no division by zero
#if defined(BIKE_BUILD_GUI) && BIKE_BUILD_GUI
    auto& p = *impl_;
    if (!p.has_frame) throw std::runtime_error("render owner has no frame");
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
    p.data.reset(); p.model.reset(); p.has_frame = false; p.display_pose.reset();
    std::vector<double>().swap(p.integration);
    p.is_closed = true;
}
bool RenderOwner::closed() const noexcept { return impl_->is_closed; }
bool RenderOwner::has_markers() const noexcept { return !impl_->marker_sizes.empty(); }
std::string_view RenderOwner::model_sha256() const noexcept { return impl_->digest; }
std::vector<double> RenderOwner::integration_state() const {
    impl_->require_live();
    if (!impl_->has_frame) return {};
    return impl_->integration;
}
std::vector<double> RenderOwner::rendered_qpos() const {
    impl_->require_live();
    if(!impl_->has_frame)return {};
    const auto qpos=model_access::readonly_buffer(impl_->data->qpos,impl_->model->nq);
    return {qpos.begin(),qpos.end()};
}
std::array<std::vector<double>, 6> RenderOwner::model_fields() const {
    impl_->require_live();
    std::array<std::vector<double>, 6> out;
    const auto fields = impl_->fields();
    for (std::size_t i = 0; i < fields.size(); ++i) out[i].assign(fields[i].begin(), fields[i].end());
    return out;
}
} // namespace bike::viewer
