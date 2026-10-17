// T26/T30 — contacts.py native_reference wheel rows, preserving source
// row/reduction order and the engine's geom2 wrench convention.
#include "control.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <span>
#include <stdexcept>
#include <utility>

#include "../cblas_abi.hpp"
#include "../engine_call.hpp"
#include "../model_access.hpp"
#include "../numeric_sum.hpp"
#include "../validation.hpp"

namespace runtime {
namespace {
using Vec3 = std::array<double, 3>;

double dot(const Vec3 &a, const Vec3 &b) {
    return blas::ddot(3, a.data(), 1, b.data(), 1);
}

double effective_radius(const TireSnapshot &snapshot) {
    if (snapshot.patches.empty()) return 0.;
    const bool loaded = std::ranges::any_of(snapshot.patches,
        [](const TirePatch &patch) { return patch.normal_load_n > 0.; });
    numeric::PythonSum terms, weights;
    for (const auto &patch : snapshot.patches) {
        const double weight = loaded ? patch.normal_load_n : 1.;
        Vec3 delta{};
        for (std::size_t j = 0; j < delta.size(); ++j)
            delta[j] = patch.point_m[j] - snapshot.wheel_axis_m[j];
        terms.add(weight * -dot(delta, patch.normal));
        weights.add(weight);
    }
    return std::max(0., terms.total() / weights.total());
}

double slip(const TireSnapshot &snapshot) {
    if (snapshot.patches.empty()) return 0.;
    const bool loaded = std::ranges::any_of(snapshot.patches,
        [](const TirePatch &patch) { return patch.normal_load_n > 0.; });
    numeric::PythonSum numerator, denominator;
    for (const auto &patch : snapshot.patches) {
        const double weight = loaded ? patch.normal_load_n : 1.;
        numerator.add(patch.slip_mps * weight);
        denominator.add(weight);
    }
    return numerator.total() / denominator.total();
}

bool road_loaded(const TireSnapshot &snapshot) {
    numeric::PythonSum total;
    for (const auto &patch : snapshot.patches)
        if (patch.working_surface) total.add(patch.normal_load_n);
    return total.total() > kContactLoadThresholdN;
}

void contact_force(const mjModel* model,const mjData* data,int row,std::array<double,6>& result){
    struct Call{const mjModel* model;const mjData* data;int row;double* result;} call{model,data,row,result.data()};
    engine::ErrorBuffer error;
    if(!engine::invoke([](void* raw)noexcept{auto& c=*static_cast<Call*>(raw);mj_contactForce(c.model,c.data,c.row,c.result);},&call,error))
        engine::throw_failure(error);
}

void point_jacobian(const mjModel* model,const mjData* data,const Vec3& point,int body,std::span<double> output){
    model_access::require_id(body,model->nbody,"native contact body");
    struct Call{const mjModel* model;const mjData* data;const double* point;int body;double* output;} call{model,data,point.data(),body,output.data()};
    engine::ErrorBuffer error;
    if(!engine::invoke([](void* raw)noexcept{auto& c=*static_cast<Call*>(raw);mj_jac(c.model,c.data,c.output,nullptr,c.point,c.body);},&call,error))
        engine::throw_failure(error);
}
} // namespace

NativeContactResult RuntimeContactQuery::query(
    const mjModel *model, const mjData *data, std::optional<double> time_s,
    std::span<const double> qvel) {
    if (model != model_ || data == nullptr)
        throw std::invalid_argument("contact query model/data owner mismatch");
    if (model->nv <= 0 || model->nv > std::numeric_limits<int>::max() ||
        model->ngeom < 0 || data->ncon < 0 || data->ncon > std::numeric_limits<int>::max())
        throw std::invalid_argument("contact Jacobian width is outside CBLAS range");
    const auto nv = static_cast<std::size_t>(model->nv);
    const int width = static_cast<int>(model->nv);
    if (qvel.empty()) qvel = model_access::readonly_buffer(data->qvel, model->nv);
    if (qvel.size() != nv || !std::ranges::all_of(qvel, [](double v) { return std::isfinite(v); }))
        throw std::invalid_argument("contact qvel must be a finite model-width vector");
    const double sample_time = time_s.value_or(data->time);
    if (!std::isfinite(sample_time) || !std::isfinite(model->opt.timestep) || model->opt.timestep <= 0.)
        throw std::invalid_argument("contact sample time/timestep is invalid");
    const double interval = std::nearbyint(sample_time / model->opt.timestep);
    // Both endpoints are checked before conversion: double(INT64_MAX)
    // rounds up to 2^63 and cannot itself be converted to int64_t.
    if (!std::isfinite(interval) || interval < -0x1p63 || interval >= 0x1p63)
        throw std::invalid_argument("contact interval is outside int64 range");
    NativeContactResult out;
    const std::array<int, 3> tracked_ids{front_id, rear_id, handlebar_id};
    const auto geom_count = model_access::checked_product(
        static_cast<std::size_t>(model->ngeom), 3,
        static_cast<std::size_t>(std::numeric_limits<mjtSize>::max()));
    const auto geom_positions = model_access::readonly_buffer(data->geom_xpos,
        static_cast<mjtSize>(geom_count));
    const auto geom_bodies = model_access::readonly_buffer(model->geom_bodyid, model->ngeom);
    for (std::size_t side = 0; side < out.snapshots.size(); ++side) {
        auto &snapshot = out.snapshots[side];
        snapshot.time_s = sample_time;
        snapshot.interval_id = static_cast<std::int64_t>(interval);
        snapshot.backend = "native_reference";
        const auto offset = static_cast<std::size_t>(tracked_ids[side]) * 3;
        std::ranges::copy(geom_positions.subspan(offset, 3), snapshot.wheel_axis_m.begin());
    }
    std::array<double, 3> magnitudes{}, verticals{};
    std::vector<double> jacobian(nv * 3, 0.);
    const auto rows = model_access::readonly_buffer(data->contact, data->ncon);
    const int terrain_id = model_access::resolve_id(model, mjOBJ_GEOM, "terrain");
    for (std::size_t row = 0; row < rows.size(); ++row) {
        const auto &contact = rows[row];
        const bool first_terrain = std::ranges::find(terrain_ids, contact.geom1) != terrain_ids.end();
        const bool second_terrain = std::ranges::find(terrain_ids, contact.geom2) != terrain_ids.end();
        const int tracked = first_terrain ? contact.geom2 : second_terrain ? contact.geom1 : -1;
        const auto found = std::ranges::find(tracked_ids, tracked);
        if (found == tracked_ids.end()) continue;
        const auto index = static_cast<std::size_t>(found - tracked_ids.begin());
        const double sign = first_terrain ? 1. : -1.;
        std::array<double, 6> wrench{};
        contact_force(model,data,static_cast<int>(row),wrench);
        if (!std::ranges::all_of(wrench, [](double v) { return std::isfinite(v); }))
            throw std::invalid_argument("native contact contains a nonfinite wrench");
        const auto frame = std::span{contact.frame};
        Vec3 normal{}, force{}, couple{};
        for (std::size_t j = 0; j < 3; ++j) normal[j] = sign * frame[j];
        blas::dgemv(blas::Order::row_major, blas::Transpose::yes, 3, 3, 1.,
                    frame.data(), 3, wrench.data(), 1, 0., force.data(), 1);
        blas::dgemv(blas::Order::row_major, blas::Transpose::yes, 3, 3, 1.,
                    frame.data(), 3, std::span{wrench}.subspan(3).data(), 1, 0., couple.data(), 1);
        for (std::size_t j = 0; j < 3; ++j) { force[j] *= sign; couple[j] *= sign; }
        magnitudes[index] += std::abs(wrench[0]);
        verticals[index] += wrench[0] * normal[2];
        if (index == 2) continue;
        normal[1] = 0.;
        const double length = std::sqrt(dot(normal, normal));
        if (!std::isfinite(length) || length <= 0.)
            throw std::invalid_argument("native contact has no planar normal");
        for (double &component : normal) component /= length;
        const Vec3 tangent{normal[2], 0., -normal[0]};
        Vec3 point{}, velocity{};
        std::ranges::copy(std::span{contact.pos}, point.begin());
        validation::derived_array(point, "native contact point");
        validation::derived_array(force, "native contact world force");
        validation::derived_array(couple, "native contact world couple");
        point_jacobian(model,data,point,geom_bodies[static_cast<std::size_t>(tracked)],jacobian);
        blas::dgemv(blas::Order::row_major, blas::Transpose::no, 3, width, 1.,
                    jacobian.data(), width, qvel.data(), 1, 0., velocity.data(), 1);
        TirePatch patch{.point_m = point, .normal = normal,
            .normal_load_n = std::max(0., wrench[0]),
            .tangent_force_n = dot(force, tangent), .slip_mps = dot(velocity, tangent),
            .working_surface = (first_terrain ? contact.geom1 : contact.geom2) == terrain_id,
            .native_world_force_n = force, .couple_world_nm = couple};
        validation::derived(patch.slip_mps, "native contact slip");
        validation::derived(patch.tangent_force_n, "native contact tangent force");
        out.snapshots[index].patches.push_back(std::move(patch));
    }
    for (auto &snapshot : out.snapshots) {
        snapshot.geometric_contact = !snapshot.patches.empty();
        snapshot.effective_radius_m = effective_radius(snapshot);
        validation::derived(snapshot.effective_radius_m, "native contact effective radius");
    }
    validation::derived_array(magnitudes, "native contact summed magnitude");
    validation::derived_array(verticals, "native contact summed support");
    // Validate both clocks on local copies before advancing any owner.
    auto next_front_load = front_load, next_rear_load = rear_load;
    auto next_front_filter = front_controller, next_rear_filter = rear_controller;
    out.contacts.front_load_n = next_front_load.update(magnitudes[0]);
    out.contacts.rear_load_n = next_rear_load.update(magnitudes[1]);
    out.contacts.front_support_n = std::max(0., verticals[0]);
    out.contacts.rear_support_n = std::max(0., verticals[1]);
    out.contacts.handlebar_load_n = magnitudes[2];
    out.contacts.front_slip_mps = slip(out.snapshots[0]);
    out.contacts.rear_slip_mps = slip(out.snapshots[1]);
    validation::derived(out.contacts.front_slip_mps,"native contact aggregate front slip");
    validation::derived(out.contacts.rear_slip_mps,"native contact aggregate rear slip");
    out.contacts.front_controller_grounded = next_front_filter.update(road_loaded(out.snapshots[0]), sample_time);
    out.contacts.rear_controller_grounded = next_rear_filter.update(road_loaded(out.snapshots[1]), sample_time);
    front_load = next_front_load; rear_load = next_rear_load;
    front_controller = next_front_filter; rear_controller = next_rear_filter;
    return out;
}
} // namespace runtime
