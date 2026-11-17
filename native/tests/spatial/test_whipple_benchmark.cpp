#include "model_access.hpp"
#include "spatial/benchmark_whipple.hpp"
#include "support/fixture_test.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <limits>
#include <string>

namespace {

using namespace bike::spatial::whipple;

// Computes the minimum over all 24 permutations of max_i |actual[p[i]] - expected[i]|.
[[nodiscard]] double max_eigenvalue_bijection_error(const Eigenvalues4& actual,
                                                    const Eigenvalues4& expected) {
    std::array<std::size_t, 4> perm{0, 1, 2, 3};
    double best = std::numeric_limits<double>::infinity();
    for (bool more = true; more; more = std::ranges::next_permutation(perm).found) {
        double worst = 0.0;
        for (std::size_t i = 0; i < 4; ++i) {
            worst = std::max(worst, std::abs(actual[perm[i]] - expected[i]));
        }
        best = std::min(best, worst);
    }
    return best;
}

struct SpeedEigenReference {
    double speed_mps = 0.0;
    Eigenvalues4 eigenvalues{};
};

[[gnu::noinline]] void test_analytical_matrices_and_eigenvalues() {
    const WhippleParameters params{};
    const auto mats = WhippleCanonicalMatrices::from_parameters(params);

    // Canonical §2.3 benchmark matrices (14-decimal-place reference).
    constexpr Mat2 kRefM{
        80.81722000000000, 2.31941332208709,
        2.31941332208709, 0.29784188199686,
    };
    constexpr Mat2 kRefC1{
        0.00000000000000, 33.86641391492494,
        -0.85035641456978, 1.68540397397560,
    };
    constexpr Mat2 kRefK0{
        -80.95000000000000, -2.59951685249872,
        -2.59951685249872, -0.80329488458618,
    };
    constexpr Mat2 kRefK2{
        0.00000000000000, 76.59734589573222,
        0.00000000000000, 2.65431523794604,
    };

    constexpr double kMatrixTol = 1.0e-12;
    for (std::size_t i = 0; i < 4; ++i) {
        bt::require(std::abs(mats.M[i] - kRefM[i]) <= kMatrixTol,
                    "Appendix A M entry matches §2.3 within 1e-12");
        bt::require(std::abs(mats.C1[i] - kRefC1[i]) <= kMatrixTol,
                    "Appendix A C1 entry matches §2.3 within 1e-12");
        bt::require(std::abs(mats.K0[i] - kRefK0[i]) <= kMatrixTol,
                    "Appendix A K0 entry matches §2.3 within 1e-12");
        bt::require(std::abs(mats.K2[i] - kRefK2[i]) <= kMatrixTol,
                    "Appendix A K2 entry matches §2.3 within 1e-12");
    }

    // Verify parametric dependence (Appendix A equations respond to parameter variations).
    WhippleParameters perturbed = params;
    perturbed.m_b_kg += 10.0;
    const auto perturbed_mats = WhippleCanonicalMatrices::from_parameters(perturbed);
    bt::require(std::abs((perturbed_mats.M[0] - mats.M[0]) - 10.0 * params.z_b_m * params.z_b_m) <= 1e-12,
                "Appendix A M_pp responds analytically to m_b");

    // Exact 12-decimal-place reference eigenvalues from §2.5 across all 9 speeds.
    const std::array<SpeedEigenReference, 9> kRefTable{{
        {.speed_mps = 0.0, .eigenvalues = {{
            {5.530943717654, 0.0},
            {3.131643247907, 0.0},
            {-3.131643247907, 0.0},
            {-5.530943717654, 0.0},
        }}},
        {.speed_mps = 0.5, .eigenvalues = {{
            {4.548188534314, 0.0},
            {3.314256338640, 0.0},
            {-3.117658374163, 0.0},
            {-6.339980487410, 0.0},
        }}},
        {.speed_mps = 2.0, .eigenvalues = {{
            {2.682345175127, 1.680662965907},
            {2.682345175127, -1.680662965907},
            {-3.071586456415, 0.0},
            {-8.673879848317, 0.0},
        }}},
        {.speed_mps = 3.0, .eigenvalues = {{
            {1.706756056640, 2.315824473843},
            {1.706756056640, -2.315824473843},
            {-2.633661372537, 0.0},
            {-10.351014672459, 0.0},
        }}},
        {.speed_mps = 4.292, .eigenvalues = {{
            {0.000513063350, 3.434519607551},
            {0.000513063350, -3.434519607551},
            {-0.985456944757, 0.0},
            {-12.708714380251, 0.0},
        }}},
        {.speed_mps = 5.0, .eigenvalues = {{
            {-0.322866429004, 0.0},
            {-0.775341882196, 4.464867713788},
            {-0.775341882196, -4.464867713788},
            {-14.078389692798, 0.0},
        }}},
        {.speed_mps = 6.0, .eigenvalues = {{
            {-0.004066900770, 0.0},
            {-1.526444865841, 5.876730605987},
            {-1.526444865841, -5.876730605987},
            {-16.085371230980, 0.0},
        }}},
        {.speed_mps = 6.024, .eigenvalues = {{
            {-0.000043365909, 0.0},
            {-1.542216825434, 5.909272114903},
            {-1.542216825434, -5.909272114903},
            {-16.134420158111, 0.0},
        }}},
        {.speed_mps = 8.0, .eigenvalues = {{
            {0.143278797657, 0.0},
            {-2.693486835811, 8.460379713969},
            {-2.693486835811, -8.460379713969},
            {-20.279408943946, 0.0},
        }}},
    }};

    constexpr double kEigTol = 1.0e-10;
    for (const auto& entry : kRefTable) {
        const Eigenvalues4 actual = mats.reference_eigenvalues(entry.speed_mps);
        const double err = max_eigenvalue_bijection_error(actual, entry.eigenvalues);
        bt::require(err <= kEigTol,
                    "reference eigenvalues match §2.5 within 1e-10 s^-1 at speed " +
                        std::to_string(entry.speed_mps));
    }
}

[[gnu::noinline]] void test_mjcf_structural_invariants(const mjModel* model,
                                                       std::string_view xml) {
    bt::require(model != nullptr, "compiled Whipple mjModel is non-null");
    if (model == nullptr) {
        return;
    }
    bt::require(model->nbody == 5, "nbody == 5 (world + 4 bicycle bodies)");
    bt::require(model->njnt == 4, "njnt == 4");
    bt::require(model->nq == 10, "nq == 10");
    bt::require(model->nv == 9, "nv == 9");

    // Verify Z-up product-of-inertia sign flips in the MJCF XML.
    bt::require(xml.find("fullinertia=\"9.2 11 2.8 0 -2.4 0\"") != std::string_view::npos,
                "frame B carries IBxz_MJ = -2.4");
    bt::require(xml.find("fullinertia=\"0.05892 0.06 0.00708 0 0.00756 0\"") !=
                    std::string_view::npos,
                "steer H carries IHxz_MJ = +0.00756");

    const auto dof_damping =
        model_access::readonly_buffer(model->dof_damping, model->nv, "mjModel.dof_damping");
    const auto dof_armature =
        model_access::readonly_buffer(model->dof_armature, model->nv, "mjModel.dof_armature");
    const auto jnt_stiffness =
        model_access::readonly_buffer(model->jnt_stiffness, model->njnt, "mjModel.jnt_stiffness");
    const auto geom_contype =
        model_access::readonly_buffer(model->geom_contype, model->ngeom, "mjModel.geom_contype");
    const auto geom_conaffinity =
        model_access::readonly_buffer(model->geom_conaffinity, model->ngeom,
                                      "mjModel.geom_conaffinity");

    for (const double val : dof_damping) {
        bt::require(val == 0.0, "dof_damping == 0");
    }
    for (const double val : dof_armature) {
        bt::require(val == 0.0, "dof_armature == 0");
    }
    for (const double val : jnt_stiffness) {
        bt::require(val == 0.0, "jnt_stiffness == 0");
    }
    for (const int val : geom_contype) {
        bt::require(val == 0, "geom_contype == 0");
    }
    for (const int val : geom_conaffinity) {
        bt::require(val == 0, "geom_conaffinity == 0");
    }

    bt::require(std::abs(model->opt.gravity[0]) == 0.0 &&
                    std::abs(model->opt.gravity[1]) == 0.0 &&
                    std::abs(model->opt.gravity[2] - (-9.81)) <= 1e-15,
                "Z-up gravity is (0, 0, -9.81)");
}

[[gnu::noinline]] void test_mujoco_kkt_eigenvalues(const mjModel* model) {
    const auto mats = WhippleCanonicalMatrices::from_parameters({});
    constexpr std::array<double, 6> kSpeeds{0.5, 3.0, 4.292, 5.0, 6.024, 8.0};

    constexpr double kFixedPointTol = 1.0e-11;
    constexpr double kManifoldTol = 1.0e-14;
    constexpr double kKktEigTol = 1.0e-6;

    for (const double speed : kSpeeds) {
        const auto res = mujoco_kkt_linearize(model, speed);

        double f0_inf = 0.0;
        for (const double val : res.fixed_point_residual) {
            f0_inf = std::max(f0_inf, std::abs(val));
        }
        bt::require(f0_inf <= kFixedPointTol,
                    "upright fixed-point residual <= 1e-11 at speed " + std::to_string(speed));
        bt::require(res.max_manifold_gap_m <= kManifoldTol,
                    "holonomic wheel gap <= 1e-14 m at speed " + std::to_string(speed));

        const Eigenvalues4 ref_eigs = mats.reference_eigenvalues(speed);
        const double err = max_eigenvalue_bijection_error(res.eigenvalues, ref_eigs);
        bt::require(err <= kKktEigTol,
                    "Method A KKT eigenvalues match Meijaard <= 1e-6 s^-1 at speed " +
                        std::to_string(speed));

        if (speed == 5.0) {
            for (const auto& lam : res.eigenvalues) {
                bt::require(lam.real() < 0.0, "v=5.0 m/s is strictly self-stable");
            }
        }
    }
}

[[gnu::noinline]] void test_critical_speeds(const mjModel* model) {
    constexpr double kRefVw = 4.29238253634110;
    constexpr double kRefVc = 6.02426201538836;
    constexpr double kCritTol = 1.0e-5;

    const double vw = find_weave_critical_speed_mps(model);
    const double vc = find_capsize_critical_speed_mps(model);

    bt::require(std::abs(vw - kRefVw) <= kCritTol,
                "weave critical speed matches 4.29238253634110 within 1e-5 m/s");
    bt::require(std::abs(vc - kRefVc) <= kCritTol,
                "capsize critical speed matches 6.02426201538836 within 1e-5 m/s");
}

[[gnu::noinline]] void test_invalid_inputs(const mjModel* model) {
    bt::invalid([] {
        WhippleParameters bad{};
        bad.w_m = 0.0;
        (void)WhippleCanonicalMatrices::from_parameters(bad);
    });
    bt::invalid([] {
        WhippleParameters bad{};
        bad.g_mps2 = -9.81;
        (void)generate_whipple_mjcf(bad);
    });
    bt::invalid([] { (void)compile_whipple_mjcf(""); });
    bt::invalid([] { (void)mujoco_kkt_eigenvalues(nullptr, 5.0); });
    bt::invalid([model] {
        (void)find_weave_critical_speed_mps(model, {.low_mps = 5.0, .high_mps = 6.0});
    });
    bt::invalid([model] {
        (void)find_capsize_critical_speed_mps(model, {.low_mps = 3.0, .high_mps = 5.0});
    });
}

} // namespace

int main() {
    return bt::run([] {
        test_analytical_matrices_and_eigenvalues();
        const std::string xml = generate_whipple_mjcf({});
        const ModelOwner model = compile_whipple_mjcf(xml);
        test_mjcf_structural_invariants(model.get(), xml);
        test_mujoco_kkt_eigenvalues(model.get());
        test_critical_speeds(model.get());
        test_invalid_inputs(model.get());
    });
}
