#include "support_geometry.hpp"

#include "../diag.hpp"
#include "../model_access.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace rider {
    UnreachableSoleTarget::~UnreachableSoleTarget() = default;

    namespace {
        Vec3 column(const Mat3 &rotation, std::size_t axis) {
            return {rotation[axis], rotation[3 + axis], rotation[6 + axis]};
        }

        // BoxFace/SoleGoal-sized returns are the designed interface; the flag's
        // useful half (by-value parameters) stays enabled.
        NATIVE_DIAG_PUSH
        NATIVE_DIAG_IGNORE("-Wlarge-by-value-copy")
        BoxFace upper_face_unchecked(const Vec3 &origin, const Mat3 &rotation, const Vec3 &half) {
            const double d0 = std::abs(rotation[6]) > 1e-14
                                  ? half[0] / std::abs(rotation[6])
                                  : std::numeric_limits<double>::infinity();
            const double d2 = std::abs(rotation[8]) > 1e-14
                                  ? half[2] / std::abs(rotation[8])
                                  : std::numeric_limits<double>::infinity();
            const std::size_t axis = d2 < d0 ? 2 : 0;
            const double distance = axis == 2 ? d2 : d0;
            const double sign = rotation[6 + axis] >= 0. ? 1. : -1.;
            const Vec3 normal = multiply(column(rotation, axis), sign);
            return {
                .point_m = add(origin, {0., 0., distance}), .normal = normal, .tangent = {normal[2], 0., -normal[0]}
            };
        }

        void validate_pad(double length, double radius, double compression) {
            validate_scalar(length, "pad half length");
            validate_scalar(radius, "pad radius");
            validate_scalar(compression, "compression");
            if (length < 0. || radius <= 0.) throw std::invalid_argument("invalid sole pad dimensions");
        }
    } // namespace
    void validate_box(const Vec3 &origin, const Mat3 &r, const Vec3 &half) {
        validate_vector(origin, "box origin");
        validate_vector(half, "box half size");
        for (double const x: r) validate_scalar(x, "box orientation");
        for (double const x: half) if (x <= 0.) throw std::invalid_argument("box dimensions must be positive");
        for (std::size_t i = 0; i < 3; ++i) {
            for (std::size_t j = 0; j < 3; ++j) {
                if (std::abs(dot(column(r, i), column(r, j)) - (i == j ? 1. : 0.)) > 1e-9)
                    throw std::invalid_argument("support box needs a planar proper rotation");
            }
            if (std::abs(r[3 * i + 1] - (i == 1 ? 1. : 0.)) > 1e-9)
                throw std::invalid_argument("support box needs a planar proper rotation");
        }
        const double determinant = r[0] * (r[4] * r[8] - r[5] * r[7]) - r[1] * (r[3] * r[8] - r[5] * r[6]) + r[2] * (
                                       r[3] * r[7] - r[4] * r[6]);
        if (std::abs(determinant - 1.) > 1e-9) throw
                std::invalid_argument("support box needs a planar proper rotation");
    }

    BoxPadContact box_pad_contact(const Vec3 &center, double radius, const Vec3 &origin, const Mat3 &r,
                                  const Vec3 &half) {
        validate_vector(center, "pad center");
        validate_scalar(radius, "pad radius");
        if (radius <= 0.) throw std::invalid_argument("invalid pad radius");
        validate_box(origin, r, half);
        return box_pad_contact_unchecked(center, radius, origin, r, half);
    }

    BoxPadContact box_pad_contact_unchecked(const Vec3 &center, double radius, const Vec3 &origin, const Mat3 &r,
                                            const Vec3 &half) {
        const Vec3 local = matvec(r, subtract(center, origin), true);
        double closest_x = std::min(std::max(local[0], -half[0]), half[0]);
        double closest_z = std::min(std::max(local[2], -half[2]), half[2]);
        const double delta_x = local[0] - closest_x, delta_z = local[2] - closest_z;
        // CPython hypot rounding is required here: see the recorded 1-ULP ruling.
        const double distance = hypot2(delta_x, delta_z);
        const double nan = std::numeric_limits<double>::quiet_NaN();
        double normal_x = nan, normal_z = nan, signed_distance = nan;
        if (distance > 1e-14) {
            normal_x = delta_x / distance;
            normal_z = delta_z / distance;
            signed_distance = distance;
        } else {
            const double clearance_x = half[0] - std::abs(local[0]), clearance_z = half[2] - std::abs(local[2]);
            const std::size_t axis = clearance_x <= clearance_z ? 0 : 2;
            const double coordinate = axis == 0 ? local[0] : local[2];
            double sign = coordinate > 0. ? 1. : coordinate < 0. ? -1. : 0.;
            if (sign == 0.) sign = r[6 + axis] >= 0. ? 1. : -1.;
            normal_x = axis == 0 ? sign : 0.;
            normal_z = axis == 0 ? 0. : sign;
            if (axis == 0) closest_x = sign * half[0];
            else closest_z = sign * half[2];
            signed_distance = -(axis == 0 ? clearance_x : clearance_z);
        }
        const Vec3 point = add(origin, matvec(r, {closest_x, local[1], closest_z}));
        const Vec3 normal = matvec(r, {normal_x, 0., normal_z});
        const bool width = std::abs(local[1]) <= half[1] + 1e-9;
        const bool footprint = width && std::abs(local[0]) <= half[0] + radius;
        return {
            .point_m = point, .normal = normal, .gap_m = signed_distance - radius, .within_width = width,
            .within_footprint = footprint
        };
    }

    BoxFace upper_box_face(const Vec3 &origin, const Mat3 &rotation, const Vec3 &half) {
        validate_box(origin, rotation, half);
        return upper_face_unchecked(origin, rotation, half);
    }

    NATIVE_DIAG_POP
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional params mirror support geometry API
    double sole_target_height(const Vec3 &origin, const Mat3 &r, const Vec3 &half, double sole_x, double length,
                              double pad_radius, double compression) {
        validate_box(origin, r, half);
        validate_pad(length, pad_radius, compression);
        validate_scalar(sole_x, "sole x");
        Vec3 face_normal = column(r, 2);
        if (face_normal[2] < 0.) face_normal = multiply(face_normal, -1.);
        if (face_normal[2] > 1e-8) {
            const double spread = std::abs(face_normal[0]) * length;
            const double effective = compression <= 0. || spread <= compression
                                         ? compression
                                         : 2. * compression - spread;
            const double height = origin[2] + (half[2] + pad_radius - effective - face_normal[0] * (sole_x - origin[0]))
                                  / face_normal[2] - pad_radius;
            bool valid = true;
            for (double const offset: {-length, length}) {
                const Vec3 center{sole_x + offset, origin[1], height + pad_radius};
                const Vec3 local = matvec(r, subtract(center, origin), true);
                const double direction = dot(face_normal, column(r, 2));
                const double sign = direction > 0. ? 1. : direction < 0. ? -1. : 0.;
                if (std::abs(local[0]) > half[0] || std::abs(local[1]) > half[1] || local[2] * sign < 0.) {
                    valid = false;
                    break;
                }
            }
            if (valid) return height;
        }
        const double radius = pad_radius - std::min(compression, 0.);
        using Vec2 = std::array<double, 2>;
        const Vec2 along{r[0] * half[0], r[6] * half[0]}, across{r[2] * half[2], r[8] * half[2]};
        std::array<Vec2, 4> corners{};
        constexpr std::array<Vec2, 4> signs{{{-1., -1.}, {-1., 1.}, {1., 1.}, {1., -1.}}};
        for (std::size_t i = 0; i < 4; ++i)
            for (std::size_t j = 0; j < 2; ++j)
                corners[i][j] = signs[i][0] * along[j] + signs[i][1] * across[j];
        const std::array<Vec2, 4> normals{{{-r[0], -r[6]}, {r[2], r[8]}, {r[0], r[6]}, {-r[2], -r[8]}}};
        // At most 16 corner/edge candidates, in the Python append order.
        std::array<double, 16> heights{};
        std::size_t count = 0;
        for (double const offset: {-length, length}) {
            const double query_x = sole_x + offset - origin[0];
            for (const Vec2 &corner: corners) {
                const double horizontal = query_x - corner[0];
                if (std::abs(horizontal) <= radius + 1e-12)
                    heights.at(count++) =
                            corner[1] + std::sqrt(std::max(0., radius * radius - horizontal * horizontal));
            }
            for (std::size_t i = 0; i < 4; ++i) {
                const Vec2 start{corners[i][0] + radius * normals[i][0], corners[i][1] + radius * normals[i][1]};
                const Vec2 end{
                    corners[(i + 1) % 4][0] + radius * normals[i][0], corners[(i + 1) % 4][1] + radius * normals[i][1]
                };
                const double span = end[0] - start[0];
                if (std::abs(span) > 1e-12) {
                    const double fraction = (query_x - start[0]) / span;
                    if (-1e-12 <= fraction && fraction <= 1. + 1e-12)
                        heights.at(count++) = start[1] + fraction * (end[1] - start[1]);
                }
            }
        }
        if (count == 0) throw UnreachableSoleTarget("sole target is outside the finite pedal footprint");
        const double upper_height = origin[2] + *std::ranges::max_element(std::span(heights).first(count)) - pad_radius;
        if (compression <= 0.) return upper_height;
        const auto penetration = [&](double height) {
            double total = 0., derivative = 0.;
            for (double const offset: {-length, length}) {
                const Vec3 center{sole_x + offset, origin[1], height + pad_radius};
                const auto contact = box_pad_contact_unchecked(center, pad_radius, origin, r, half);
                if (contact.within_footprint && contact.gap_m < 0.) {
                    if (center[2] < origin[2] && contact.normal[2] < -.5)
                        throw UnreachableSoleTarget("requested sole compression crosses the pedal surface");
                    total -= contact.gap_m;
                    derivative += contact.normal[2];
                }
            }
            return std::pair{total, derivative};
        };
        const double requested = 2. * compression;
        double upper = upper_height, lower = upper - std::max(requested, .001);
        bool bracketed = false;
        for (int attempt = 0; attempt < 32; ++attempt) {
            const auto [value, derivative] = penetration(lower);
            (void) derivative;
            if (value >= requested) {
                bracketed = true;
                break;
            }
            lower -= std::max(requested, .001);
        }
        if (!bracketed) throw UnreachableSoleTarget("requested pedal support load is not reachable");
        double height = (lower + upper) / 2.;
        for (int attempt = 0; attempt < 32; ++attempt) {
            const auto [value, derivative] = penetration(height);
            const double difference = value - requested;
            if (std::abs(difference) <= 1e-12) return height;
            if (difference > 0.) lower = height;
            else upper = height;
            const double candidate = derivative > 1e-12 ? height + difference / derivative : upper;
            height = lower < candidate && candidate < upper ? candidate : (lower + upper) / 2.;
        }
        throw std::runtime_error("finite pedal support target did not converge");
    }

    SoleGoal project_sole_goal(const Vec3 &origin, const Mat3 &r, const Vec3 &half, double length, double radius,
                               double compression, double shear) {
        validate_box(origin, r, half);
        validate_pad(length, radius, compression);
        validate_scalar(shear, "shear");
        const auto face = upper_face_unchecked(origin, r, half);
        const auto evaluate = [&](double requested_compression, double requested_shear) {
            Vec3 goal = add(add(face.point_m, multiply(face.normal, radius - requested_compression)),
                            multiply(face.tangent, requested_shear));
            goal[2] = sole_target_height(origin, r, half, goal[0], length, radius, requested_compression);
            return goal;
        };
        double applied_compression = compression, applied_shear = shear;
        std::vector<std::string> reasons;
        Vec3 goal{};
        try { goal = evaluate(applied_compression, applied_shear); } catch (const UnreachableSoleTarget &) {
            const double baseline = std::min(compression, 0.);
            bool reachable = false;
            for (int attempt = 0; attempt < 18; ++attempt) {
                try {
                    goal = evaluate(baseline, applied_shear);
                    reachable = true;
                    break;
                } catch (const UnreachableSoleTarget &) { applied_shear *= .5; }
            }
            if (!reachable) {
                applied_shear = 0.;
                goal = evaluate(baseline, applied_shear);
            }
            if (applied_shear != shear) reasons.emplace_back("finite_shear_footprint");
            applied_compression = baseline;
            if (compression > 0.) {
                double lower = 0., upper = compression;
                for (int attempt = 0; attempt < 18; ++attempt) {
                    const double candidate = (lower + upper) / 2.;
                    try {
                        const Vec3 tested = evaluate(candidate, applied_shear);
                        lower = candidate;
                        goal = tested;
                    } catch (const UnreachableSoleTarget &) { upper = candidate; }
                }
                applied_compression = lower;
                reasons.emplace_back("finite_compression_capacity");
            }
        }
        const bool saturated = !reasons.empty();
        return {
            .goal = goal,
            .diagnostic = {
                .requested_compression_m = compression, .applied_compression_m = applied_compression,
                .requested_shear_m = shear, .applied_shear_m = applied_shear, .saturated = saturated,
                .limiting_reasons = std::move(reasons)
            }
        };
    }

    void validate_planar_support_model(const mjModel *model, const mjData *data, std::span<const int> geoms) {
        const auto body_of = model_access::readonly_buffer(model->geom_bodyid, model->ngeom);
        const auto parents = model_access::readonly_buffer(model->body_parentid, model->nbody);
        std::set<int> ancestors;
        for (int const geom: geoms) {
            model_access::require_id(geom, model->ngeom,
                                     "invalid rider support geom");
            int body = body_of[static_cast<std::size_t>(geom)];
            model_access::require_id(body, model->nbody, "rider support body");
            // Ordered-parent walk: MuJoCo's kinematic tree stores
            // 0 <= parent < body for every non-root body, so a valid chain
            // strictly decreases and cannot cycle — a violated invariant or
            // a run past nbody steps is an invalid/cyclic model, not a loop.
            for (mjtSize steps = 0; body != 0; ++steps) {
                if (steps >= model->nbody)
                    throw std::invalid_argument(
                        "invalid or cyclic rider support parent chain");
                const int parent = parents[static_cast<std::size_t>(body)];
                if (parent < 0 || parent >= body)
                    throw std::invalid_argument(
                        "invalid or cyclic rider support parent chain");
                ancestors.insert(body);
                body = parent;
            }
        }
        const auto joint_bodies = model_access::readonly_buffer(model->jnt_bodyid, model->njnt);
        const auto joint_types = model_access::readonly_buffer(model->jnt_type, model->njnt);
        const auto axes = model_access::readonly_buffer(data->xaxis, 3 * model->njnt);
        for (std::size_t joint = 0; joint < joint_bodies.size(); ++joint) {
            if (!ancestors.contains(joint_bodies[joint])) continue;
            const auto type = joint_types[joint];
            if (type != mjJNT_HINGE && type != mjJNT_SLIDE)
                throw std::invalid_argument("rider supports require scalar planar joint topology");
            // Nonfinite axis entries are rejected before the axis-layout
            // tolerance comparisons — NaN would otherwise escape |x| > eps.
            for (std::size_t axis = 0; axis < 3; ++axis)
                if (!std::isfinite(axes[3 * joint + axis]))
                    throw std::invalid_argument(
                        "rider supports require finite joint axes");
            for (std::size_t axis = 0; axis < 3; ++axis) {
                if ((type == mjJNT_HINGE && std::abs(std::abs(axes[3 * joint + axis]) - (axis == 1 ? 1. : 0.)) > 1e-9)
                    || (type == mjJNT_SLIDE && axis == 1 && std::abs(axes[3 * joint + axis]) > 1e-9))
                    throw std::invalid_argument("rider supports require scalar planar joint topology");
            }
        }
        const auto geom_types = model_access::readonly_buffer(model->geom_type, model->ngeom);
        const auto positions = model_access::readonly_buffer(data->geom_xpos, 3 * model->ngeom);
        const auto rotations = model_access::readonly_buffer(data->geom_xmat, 9 * model->ngeom);
        const auto sizes = model_access::readonly_buffer(model->geom_size, 3 * model->ngeom);
        for (int const geom: geoms) {
            const auto id = static_cast<std::size_t>(geom);
            if (geom_types[id] != mjGEOM_BOX) throw std::invalid_argument("rider support geometry must be a box");
            Vec3 origin{}, half{};
            Mat3 r{};
            std::ranges::copy(positions.subspan(3 * id, 3), origin.begin());
            std::ranges::copy(sizes.subspan(3 * id, 3), half.begin());
            std::ranges::copy(rotations.subspan(9 * id, 9), r.begin());
            validate_box(origin, r, half);
        }
    }
} // namespace rider
