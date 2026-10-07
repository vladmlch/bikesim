#pragma once
#include "../model_access.hpp"
#include "policy_config.hpp"
#include <array>
#include <mujoco/mujoco.h>
#include <optional>
#include <span>
#include <type_traits>
#include <vector>

namespace drivetrain {
    using Vec2 = std::array<double, 2>;

    struct ChainGeometry {
        double length{}, psi{};
    };

    ChainGeometry chain_geometry(Vec2 front, Vec2 rear, double rf, double rr,
                                 Vec2 up = {0., 1.},
                                 std::optional<double> reference = std::nullopt);

    Vec2 chain_center_gradient(Vec2 front, Vec2 rear, double rf, double rr,
                               Vec2 up = {0., 1.},
                               std::optional<double> reference = std::nullopt);

    std::pair<double, double> chain_tension(double extension, double rate, double k,
                                            double c);

    double dot(std::span<const double> a, std::span<const double> b);

    double unwrap(double value, double reference);

    int resolve(const mjModel *m, mjtObj kind, const char *name);

    // Thin alias retained for TUs that adopted it before model_access.hpp
    // existed — it delegates to the checked helpers, so negative extents
    // and null positive-extent buffers throw std::invalid_argument before
    // any span is formed. New code calls model_access helpers directly.
    template<class T>
    auto buffer(T *p, mjtSize count) {
        if constexpr (std::is_const_v<T>) {
            return model_access::readonly_buffer(p, count);
        } else {
            return model_access::mutable_buffer(p, count);
        }
    }

    struct GeometryWorkspace {
        explicit GeometryWorkspace(mjtSize nv);

        std::vector<double> jp_f, jr_f, jp_r, jr_r, jacobian, difference, coordinates;

        double angle(const mjModel *m, const mjData *d, int body, bool accumulated);

        double evaluate(const mjModel *m, mjData *d, const GearingConfig &gear, int front,
                        int rear, int frame, std::optional<Vec2> angles = std::nullopt,
                        std::optional<double> psi = std::nullopt, bool accumulated = true);

        double theta_f{}, theta_r{}, psi{};
    };
} // namespace drivetrain
