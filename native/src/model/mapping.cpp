#include "mapping.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <limits>
#include <ranges>
#include <stdexcept>

#include "../cblas_abi.hpp"
#include "../engine_call.hpp"

namespace bike::model {
namespace {

void finite(const WorldVector& vector, std::string_view what) {
    if (!std::ranges::all_of(vector, [](double x) { return std::isfinite(x); }))
        throw std::invalid_argument(std::string(what));
}

void physical_body(const mjModel& model, int body, std::string_view what) {
    if (body <= 0 || body >= model.nbody)
        throw std::invalid_argument(std::string(what));
}

std::size_t jacobian_size(const mjModel& model) {
    if (model.nv < 0 || std::cmp_greater(model.nv,
            std::numeric_limits<int>::max()))
        throw std::invalid_argument("point Jacobian model has invalid nv");
    return 3 * static_cast<std::size_t>(model.nv);
}

void jacobian(const mjModel& model, const mjData& data, int body,
              const WorldVector& point, double* jp, double* jr) {
    struct Call {
        const mjModel* model;
        const mjData* data;
        int body;
        const double* point;
        double* jp;
        double* jr;
    } call{.model=&model, .data=&data, .body=body, .point=point.data(), .jp=jp, .jr=jr};
    engine::ErrorBuffer error;
    if (!engine::invoke([](void* raw) noexcept {
            auto const& c = *static_cast<Call*>(raw);
            mj_jac(c.model, c.data, c.jp, c.jr, c.point, c.body);
        }, &call, error)) engine::throw_failure(error);
}

WorldVector multiply(const mjModel& model, const mjData& data,
                     const std::vector<double>& matrix) {
    WorldVector velocity{};
    if (model.nv == 0) return velocity;
    // NumPy's (3,nv) @ (nv,) goes through the same pinned CBLAS provider.
    blas::cblas_dgemv(static_cast<int>(blas::Order::row_major),
        static_cast<int>(blas::Transpose::no), 3, static_cast<int>(model.nv),
        1., matrix.data(), static_cast<int>(model.nv), data.qvel, 1,
        0., velocity.data(), 1);
    return velocity;
}

} // namespace

int required_id(const mjModel& model, mjtObj kind, std::string_view name) {
    if (name.find('\0') != std::string_view::npos)
        throw std::invalid_argument("model object name contains a NUL byte");
    const std::string owned(name);
    const int result = mj_name2id(&model, kind, owned.c_str());
    if (result < 0) throw std::invalid_argument("model has no '" + owned + "'");
    return result;
}

std::string joint_name_at_qpos(const mjModel& model, int address) {
    const auto addresses = std::views::counted(model.jnt_qposadr, model.njnt);
    int found = -1;
    for (int joint = 0; joint < model.njnt; ++joint) {
        if (addresses[static_cast<std::size_t>(joint)] != address) continue;
        if (found >= 0) throw std::invalid_argument(
            "expected exactly one joint at qposadr " + std::to_string(address));
        found = joint;
    }
    if (found < 0) throw std::invalid_argument(
        "expected exactly one joint at qposadr " + std::to_string(address));
    const char* name = mj_id2name(&model, mjOBJ_JOINT, found);
    if (name == nullptr) throw std::invalid_argument(
        "joint at qposadr " + std::to_string(address) + " has no name");
    return name;
}

std::pair<int, int> joint_address(const mjModel& model, std::string_view name) {
    const auto joint = static_cast<std::size_t>(required_id(model, mjOBJ_JOINT, name));
    return {std::views::counted(model.jnt_qposadr, model.njnt)[joint],
            std::views::counted(model.jnt_dofadr, model.njnt)[joint]};
}

PointJacobian point_jacobian(const mjModel& model, const mjData& data,
                             int body, const WorldVector& point) {
    finite(point, "invalid shape or non-finite world point");
    physical_body(model, body, "point Jacobian requires a physical body");
    PointJacobian out{.translation=std::vector<double>(jacobian_size(model)),
                      .rotation=std::vector<double>(jacobian_size(model))};
    jacobian(model, data, body, point, out.translation.data(), out.rotation.data());
    return out;
}

void point_jacobian_into(const mjModel& model, const mjData& data, int body,
                         const WorldVector& point, std::span<double> translation) {
    finite(point, "invalid world point");
    physical_body(model, body, "point Jacobian requires a physical body");
    if (translation.size() != jacobian_size(model))
        throw std::invalid_argument("point Jacobian output must have shape (3,nv)");
    jacobian(model, data, body, point, translation.data(), nullptr);
}

void relative_point_jacobian(const mjModel& model, const mjData& data,
    int body_a, int body_b, const WorldVector& point,
    std::span<double> jacobian_a, std::span<double> jacobian_b) {
    finite(point, "invalid world point");
    physical_body(model, body_a, "relative Jacobian requires distinct physical bodies");
    physical_body(model, body_b, "relative Jacobian requires distinct physical bodies");
    if (body_a == body_b)
        throw std::invalid_argument("relative Jacobian requires distinct physical bodies");
    if (jacobian_a.size() != jacobian_size(model) || jacobian_b.size() != jacobian_size(model))
        throw std::invalid_argument("relative Jacobian outputs must have shape (3,nv)");
    // Aliasing would silently turn J_a-J_b into zero. Reject overlapping
    // buffers before either destination changes (pointer total order is defined).
    const std::less<> before;
    if (!jacobian_a.empty() &&
        before(jacobian_a.data(), std::to_address(jacobian_b.end())) &&
        before(jacobian_b.data(), std::to_address(jacobian_a.end())))
        throw std::invalid_argument("relative Jacobian buffers must not overlap");
    jacobian(model, data, body_a, point, jacobian_a.data(), nullptr);
    jacobian(model, data, body_b, point, jacobian_b.data(), nullptr);
    for (std::size_t i = 0; i < jacobian_a.size(); ++i) jacobian_a[i] -= jacobian_b[i];
}

WorldVector point_velocity(const mjModel& model, const mjData& data,
                           int body, const WorldVector& point) {
    return multiply(model, data, point_jacobian(model, data, body, point).translation);
}

WorldVector body_angular_velocity(const mjModel& model, const mjData& data,
                                  int body) {
    physical_body(model, body, "point Jacobian requires a physical body");
    const auto position = std::views::counted(data.xpos, model.nbody * 3)
        .subspan(static_cast<std::size_t>(body) * 3, 3);
    return multiply(model, data, point_jacobian(model, data, body,
        {position[0], position[1], position[2]}).rotation);
}

std::vector<double> map_wrench(const mjModel& model, mjData& data, int body,
    const WorldVector& point, const WorldVector& force,
    std::optional<WorldVector> torque) {
    finite(point, "invalid shape or non-finite world point");
    finite(force, "invalid shape or non-finite world force");
    const WorldVector moment = torque.value_or(WorldVector{});
    finite(moment, "invalid shape or non-finite world torque");
    physical_body(model, body, "wrench must act on a physical body");
    std::vector<double> out(static_cast<std::size_t>(model.nv));
    struct Call {
        const mjModel* model; mjData* data; int body;
        const double* point; const double* force; const double* torque; double* out;
    } call{.model=&model, .data=&data, .body=body, .point=point.data(), .force=force.data(), .torque=moment.data(), .out=out.data()};
    engine::ErrorBuffer error;
    if (!engine::invoke([](void* raw) noexcept {
            auto const& c = *static_cast<Call*>(raw);
            mj_applyFT(c.model, c.data, c.force, c.torque, c.point, c.body, c.out);
        }, &call, error)) engine::throw_failure(error);
    return out;
}

} // namespace bike::model
